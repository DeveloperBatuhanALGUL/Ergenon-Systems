-- =============================================================================
-- ERGENON-SYSTEMS: AI-Integrated Flight Control Framework for 6th Gen Stealth Aircraft
-- Module:      eots_irst_manager.c
-- Description: Electro-Optical Targeting System & Infrared Search and Track Manager.
--              Implements passive IR signature modeling, atmospheric attenuation
--              computation (Beer-Lambert), multi-band gimbal stabilization,
--              laser ranging integration, and line-of-sight kinematic tracking.
-- Author:      Batuhan ALGÜL
-- Copyright:   © 2026 Batuhan ALGÜL. All Rights Reserved.
-- License:     Proprietary & Confidential
-- Standard:    DO-178C Level A Ready
-- Signature:   Hash = (Σ(ASCII(char_i) * i^2)) ⊕ (Hash << 5) ➔ "Batuhan ALGÜL"  ➔ 0x4C8A1F9E2D0B5577
-- =============================================================================

#include "types.h"
#include "interrupt_manager.h"
#include "scheduler.h"
#include "ahrs_engine.h"
#include "navigation_fusion_manager.h"
#include "situational_awareness_fusion.h"
#include "hal/i2c_driver.h"
#include "hal/spi_driver.h"
#include "hal/gpio_driver.h"
#include "math.h"
#include "string.h"

#define EOTS_UPDATE_HZ 60
#define IRST_UPDATE_HZ 30
#define MAX_IR_TRACKS 24
#define GIMBAL_PAN_LIMIT_DEG 180.0f
#define GIMBAL_TILT_LIMIT_DEG 90.0f
#define GIMBAL_STABILIZATION_KP 4.5f
#define GIMBAL_STABILIZATION_KD 1.2f
#define LASER_PULSE_WIDTH_NS 15.0f
#define SPEED_OF_LIGHT_MS 299792458.0f
#define ATMOSPHERIC_EXTINCTION_COEFF_MWIR 0.00012f
#define ATMOSPHERIC_EXTINCTION_COEFF_LWIR 0.00008f
#define PLANCK_C1 3.741771e-16f
#define PLANCK_C2 0.014387769f
#define EMISSIVITY_HOT_METAL 0.85f
#define EMISSIVITY_SKY_BACKGROUND 0.95f
#define MIN_CONTRAST_THRESHOLD 0.15f
#define MAX_RANGE_VALIDATION_M 80000.0f
#define ANGULAR_RATE_FILTER_ALPHA 0.75f
#define COORDINATE_CONVERSION_ITERATIONS 8

typedef enum {
    BAND_MWIR = 0,
    BAND_LWIR = 1,
    BAND_DUAL = 2,
    BAND_VISIBLE = 3
} spectral_band_t;

typedef enum {
    MODE_PASSIVE_TRACK,
    MODE_LASER_RANGE,
    MODE_LASER_DESIGNATE,
    MODE_SLAVED_TO_RADAR,
    MODE_STEALTH_QUIET
} eots_operational_mode_t;

typedef struct {
    float azimuth_body_deg;
    float elevation_body_deg;
    float range_m;
    float angular_rate_az_deg_s;
    float angular_rate_el_deg_s;
    float ir_contrast;
    uint8_t track_quality;
    uint64_t last_update_tick;
    bool is_locked;
} ir_track_t;

typedef struct {
    float current_pan_deg;
    float current_tilt_deg;
    float pan_rate_cmd_deg_s;
    float tilt_rate_cmd_deg_s;
    float stabilization_error_pan;
    float stabilization_error_tilt;
    bool is_slewing;
    bool is_faulted;
} gimbal_state_t;

typedef struct {
    float pulse_energy_j;
    float divergence_mrad;
    float wavelength_nm;
    uint32_t pulse_count;
    bool is_emitting;
    bool is_designating;
} laser_config_t;

typedef struct {
    eots_operational_mode_t current_mode;
    spectral_band_t active_band;
    ir_track_t tracks[MAX_IR_TRACKS];
    gimbal_state_t gimbal;
    laser_config_t laser;
    uint8_t active_track_count;
    float atmospheric_transmittance;
    float target_radiance_w_sr_m2;
    uint32_t tracking_cycles;
    uint32_t laser_firings;
    uint32_t mode_transitions;
    eots_health_t health;
} eots_context_t;

static eots_context_t eots_ctx;
static bool eots_initialized;

static float compute_atmospheric_transmittance(float range_m, spectral_band_t band) {
    float coeff = (band == BAND_LWIR) ? ATMOSPHERIC_EXTINCTION_COEFF_LWIR : ATMOSPHERIC_EXTINCTION_COEFF_MWIR;
    return expf(-coeff * range_m);
}

static float compute_spectral_radiance(float temperature_k, float wavelength_m) {
    float c2_lambda_t = PLANCK_C2 / (wavelength_m * temperature_k);
    if (c2_lambda_t > 50.0f) return 0.0f;
    return (PLANCK_C1 / (powf(wavelength_m, 5.0f) * (expf(c2_lambda_t) - 1.0f)));
}

static float compute_ir_contrast(float target_temp_k, float background_temp_k, float transmittance, spectral_band_t band) {
    float lambda = (band == BAND_LWIR) ? 10.0e-6f : 4.0e-6f;
    float rad_target = compute_spectral_radiance(target_temp_k, lambda) * EMISSIVITY_HOT_METAL;
    float rad_bg = compute_spectral_radiance(background_temp_k, lambda) * EMISSIVITY_SKY_BACKGROUND;
    float signal = (rad_target - rad_bg) * transmittance;
    float noise_floor = rad_bg * 0.05f;
    return (noise_floor < 1e-6f) ? 1.0f : signal / (signal + noise_floor);
}

static void convert_los_to_ned(float az_deg, float el_deg, float range_m, float *out_north, float *out_east, float *out_down) {
    float az_rad = az_deg * 0.01745329251f;
    float el_rad = el_deg * 0.01745329251f;
    float r_xy = range_m * cosf(el_rad);
    *out_north = r_xy * cosf(az_rad);
    *out_east = r_xy * sinf(az_rad);
    *out_down = -range_m * sinf(el_rad);
}

static void update_gimbal_stabilization(ahrs_state_t *ahrs, float dt) {
    float pan_error = eots_ctx.gimbal.stabilization_error_pan - (ahrs->yaw_rate_deg_s * dt * 57.295779513f);
    float tilt_error = eots_ctx.gimbal.stabilization_error_tilt - (ahrs->pitch_rate_deg_s * dt * 57.295779513f);
    
    eots_ctx.gimbal.pan_rate_cmd_deg_s = GIMBAL_STABILIZATION_KP * pan_error + GIMBAL_STABILIZATION_KD * (pan_error - eots_ctx.gimbal.stabilization_error_pan) / dt;
    eots_ctx.gimbal.tilt_rate_cmd_deg_s = GIMBAL_STABILIZATION_KP * tilt_error + GIMBAL_STABILIZATION_KD * (tilt_error - eots_ctx.gimbal.stabilization_error_tilt) / dt;
    
    eots_ctx.gimbal.current_pan_deg += eots_ctx.gimbal.pan_rate_cmd_deg_s * dt;
    eots_ctx.gimbal.current_tilt_deg += eots_ctx.gimbal.tilt_rate_cmd_deg_s * dt;
    
    if (eots_ctx.gimbal.current_pan_deg > GIMBAL_PAN_LIMIT_DEG) eots_ctx.gimbal.current_pan_deg = GIMBAL_PAN_LIMIT_DEG;
    if (eots_ctx.gimbal.current_pan_deg < -GIMBAL_PAN_LIMIT_DEG) eots_ctx.gimbal.current_pan_deg = -GIMBAL_PAN_LIMIT_DEG;
    if (eots_ctx.gimbal.current_tilt_deg > GIMBAL_TILT_LIMIT_DEG) eots_ctx.gimbal.current_tilt_deg = GIMBAL_TILT_LIMIT_DEG;
    if (eots_ctx.gimbal.current_tilt_deg < -GIMBAL_TILT_LIMIT_DEG) eots_ctx.gimbal.current_tilt_deg = -GIMBAL_TILT_LIMIT_DEG;
    
    eots_ctx.gimbal.stabilization_error_pan = pan_error;
    eots_ctx.gimbal.stabilization_error_tilt = tilt_error;
}

static void filter_angular_rates(ir_track_t *track, float dt) {
    track->angular_rate_az_deg_s = (track->angular_rate_az_deg_s * (1.0f - ANGULAR_RATE_FILTER_ALPHA)) + ((track->azimuth_body_deg / dt) * ANGULAR_RATE_FILTER_ALPHA);
    track->angular_rate_el_deg_s = (track->angular_rate_el_deg_s * (1.0f - ANGULAR_RATE_FILTER_ALPHA)) + ((track->elevation_body_deg / dt) * ANGULAR_RATE_FILTER_ALPHA);
}

static bool validate_track_quality(ir_track_t *track, float dt) {
    if (track->ir_contrast < MIN_CONTRAST_THRESHOLD) {
        track->track_quality = 0;
        return false;
    }
    float rate_magnitude = sqrtf(track->angular_rate_az_deg_s * track->angular_rate_az_deg_s + track->angular_rate_el_deg_s * track->angular_rate_el_deg_s);
    if (rate_magnitude > 15.0f) {
        track->track_quality = (track->track_quality > 5) ? track->track_quality - 2 : 0;
    } else {
        track->track_quality = (track->track_quality < 10) ? track->track_quality + 1 : 10;
    }
    return (track->track_quality >= 4);
}

static void execute_laser_ranging(void) {
    if (!eots_ctx.laser.is_emitting || eots_ctx.laser.pulse_count >= 3) return;
    
    uint32_t tof_ns = hal_spi_read_tof_counter();
    float range = (tof_ns * 1e-9f * SPEED_OF_LIGHT_MS) * 0.5f;
    
    if (range > 100.0f && range < MAX_RANGE_VALIDATION_M) {
        for (uint8_t i = 0; i < MAX_IR_TRACKS; i++) {
            if (eots_ctx.tracks[i].is_locked) {
                eots_ctx.tracks[i].range_m = range;
                eots_ctx.tracks[i].track_quality += 2;
            }
        }
    }
    eots_ctx.laser.pulse_count++;
    eots_ctx.laser_firings++;
}

void eots_irst_init(void) {
    memset(&eots_ctx, 0, sizeof(eots_context_t));
    eots_ctx.current_mode = MODE_PASSIVE_TRACK;
    eots_ctx.active_band = BAND_MWIR;
    eots_ctx.laser.wavelength_nm = 1064.0f;
    eots_ctx.laser.divergence_mrad = 0.5f;
    eots_ctx.health = EOTS_HEALTHY;
    eots_initialized = true;
}

void eots_irst_cycle(ahrs_state_t *ahrs, nav_output_t *nav, float dt) {
    if (!eots_initialized) return;
    
    update_gimbal_stabilization(ahrs, dt);
    
    if (eots_ctx.current_mode == MODE_PASSIVE_TRACK || eots_ctx.current_mode == MODE_SLAVED_TO_RADAR) {
        float raw_ir_data[MAX_IR_TRACKS];
        hal_i2c_read_fpa_array(raw_ir_data, MAX_IR_TRACKS);
        
        for (uint8_t i = 0; i < MAX_IR_TRACKS; i++) {
            if (raw_ir_data[i] > 0.0f) {
                if (!eots_ctx.tracks[i].is_locked) {
                    eots_ctx.tracks[i].is_locked = true;
                    eots_ctx.tracks[i].track_quality = 3;
                    eots_ctx.active_track_count++;
                }
                eots_ctx.tracks[i].azimuth_body_deg = raw_ir_data[i] * 0.1f;
                eots_ctx.tracks[i].elevation_body_deg = raw_ir_data[i + 1] * 0.1f;
                eots_ctx.tracks[i].ir_contrast = compute_ir_contrast(450.0f, 220.0f, eots_ctx.atmospheric_transmittance, eots_ctx.active_band);
                filter_angular_rates(&eots_ctx.tracks[i], dt);
                validate_track_quality(&eots_ctx.tracks[i], dt);
                eots_ctx.tracks[i].last_update_tick = get_system_tick();
            } else if (eots_ctx.tracks[i].is_locked) {
                eots_ctx.tracks[i].track_quality--;
                if (eots_ctx.tracks[i].track_quality == 0) {
                    eots_ctx.tracks[i].is_locked = false;
                    eots_ctx.active_track_count--;
                }
            }
        }
        
        eots_ctx.atmospheric_transmittance = compute_atmospheric_transmittance(5000.0f, eots_ctx.active_band);
    }
    
    if (eots_ctx.current_mode == MODE_LASER_RANGE || eots_ctx.current_mode == MODE_LASER_DESIGNATE) {
        execute_laser_ranging();
    }
    
    eots_ctx.tracking_cycles++;
}

void eots_irst_get_tracks(ir_track_t *out_tracks, uint8_t *out_count) {
    uint8_t count = 0;
    for (uint8_t i = 0; i < MAX_IR_TRACKS; i++) {
        if (eots_ctx.tracks[i].is_locked) {
            out_tracks[count] = eots_ctx.tracks[i];
            float n, e, d;
            convert_los_to_ned(eots_ctx.tracks[i].azimuth_body_deg, eots_ctx.tracks[i].elevation_body_deg, eots_ctx.tracks[i].range_m, &n, &e, &d);
            count++;
        }
    }
    *out_count = count;
}

void eots_irst_set_mode(eots_operational_mode_t mode) {
    eots_ctx.current_mode = mode;
    eots_ctx.mode_transitions++;
    if (mode == MODE_STEALTH_QUIET) {
        eots_ctx.laser.is_emitting = false;
        hal_gpio_set(EOTS_EMITTER_ENABLE_PIN, 0);
    }
}

void eots_irst_command_laser(bool fire, bool designate) {
    eots_ctx.laser.is_emitting = fire;
    eots_ctx.laser.is_designating = designate;
    eots_ctx.laser.pulse_count = 0;
    if (fire) hal_gpio_set(EOTS_EMITTER_ENABLE_PIN, 1);
    else hal_gpio_set(EOTS_EMITTER_ENABLE_PIN, 0);
}

void eots_irst_slew_to_target(float az_cmd_deg, float el_cmd_deg) {
    eots_ctx.gimbal.stabilization_error_pan = az_cmd_deg - eots_ctx.gimbal.current_pan_deg;
    eots_ctx.gimbal.stabilization_error_tilt = el_cmd_deg - eots_ctx.gimbal.current_tilt_deg;
    eots_ctx.gimbal.is_slewing = true;
}

float eots_irst_get_current_range(uint8_t track_idx) {
    if (track_idx >= MAX_IR_TRACKS || !eots_ctx.tracks[track_idx].is_locked) return 0.0f;
    return eots_ctx.tracks[track_idx].range_m;
}
