-- =============================================================================
-- ERGENON-SYSTEMS: AI-Integrated Flight Control Framework for 6th Gen Stealth Aircraft
-- Module:      aesa_radar_tasking_manager.c
-- Description: Active Electronically Scanned Array Radar Tasking & Resource 
--              Allocation Engine. Implements deterministic beam scheduling, 
--              LPI waveform agility, track-while-scan filtering, and dynamic
--              power/aperture distribution for multi-role mission profiles.
-- Author:      Batuhan ALGÜL
-- Copyright:   © 2026 Batuhan ALGÜL. All Rights Reserved.
-- License:     Proprietary & Confidential
-- Standard:    DO-178C Level A Ready
-- Signature:   Hash = (Σ(ASCII(char_i) * i^2)) ⊕ (Hash << 5) ➔ "Batuhan ALGÜL"  ➔ 0x8A2F1C9D4E0B7733
-- =============================================================================

#include "types.h"
#include "interrupt_manager.h"
#include "scheduler.h"
#include "electronic_warfare_manager.h"
#include "navigation_fusion_manager.h"
#include "weapon_systems_manager.h"
#include "neural_engine.h"
#include "hal/spi_driver.h"
#include "hal/gpio_driver.h"
#include "math.h"
#include "string.h"

#define AESA_UPDATE_HZ 20
#define MAX_TRACK_SLOTS 64
#define MAX_BEAM_DWELLS 32
#define MAX_ELEMENTS 1200
#define CENTER_FREQ_GHZ 10.5f
#define BANDWIDTH_MHZ 500.0f
#define MIN_SNR_DB 13.0f
#define MAX_RANGE_KM 350.0f
#define TRACK_INIT_THRESHOLD 4
#define TRACK_DELETE_THRESHOLD 8
#define LPI_HOP_COUNT 256
#define BEAM_STEERING_LIMIT_DEG 60.0f
#define POWER_ALLOC_GRANULARITY 0.01f
#define APERTURE_SUBARRAY_COUNT 16
#define RANGE_RESOLUTION_M 0.5f
#define DOPPLER_RESOLUTION_MS 2.0f
#define GATING_SIGMA 3.0f
#define NEURAL_PREDICTION_WEIGHT 0.7f

typedef enum {
    RADAR_MODE_SEARCH_VOLUME,
    RADAR_MODE_TRACK_WHILE_SCAN,
    RADAR_MODE_STT_ENGAGEMENT,
    RADAR_MODE_SAR_IMAGING,
    RADAR_MODE_LPI_PASSIVE,
    RADAR_MODE_JAMMER_SUPPORT
} radar_operational_mode_t;

typedef enum {
    WAVEFORM_LINEAR_FM,
    WAVEFORM_PHASE_CODED,
    WAVEFORM_FREQUENCY_HOP,
    WAVEFORM_NOISE_LPI,
    WAVEFORM_DUAL_BAND
} waveform_type_t;

typedef enum {
    TRACK_STATE_TENTATIVE,
    TRACK_STATE_CONFIRMED,
    TRACK_STATE_DEGRADED,
    TRACK_STATE_LOST
} track_state_t;

typedef struct {
    float azimuth_deg;
    float elevation_deg;
    float range_m;
    float range_rate_ms;
    float snr_db;
    uint8_t hits;
    uint8_t misses;
    track_state_t state;
    float state_vector[6];
    float covariance_matrix[36];
    uint64_t last_update_tick;
    uint8_t assigned_weapon_slot;
    bool is_threat;
} radar_track_t;

typedef struct {
    uint8_t beam_id;
    float azimuth_cmd;
    float elevation_cmd;
    float dwell_time_ms;
    float power_percent;
    waveform_type_t waveform;
    uint8_t priority;
    bool is_active;
    uint64_t schedule_start_tick;
} beam_dwell_t;

typedef struct {
    radar_operational_mode_t current_mode;
    radar_track_t tracks[MAX_TRACK_SLOTS];
    beam_dwell_t schedule[MAX_BEAM_DWELLS];
    uint8_t active_track_count;
    uint8_t active_beam_count;
    float total_power_draw_w;
    float aperture_utilization_pct;
    float lpi_hop_pattern[LPI_HOP_COUNT];
    uint8_t hop_index;
    uint32_t radar_cycles;
    uint32_t track_initiations;
    uint32_t track_drops;
    uint32_t lpi_resets;
    radar_health_t health;
    bool transmitter_enabled;
} aesa_context_t;

static aesa_context_t aesa_ctx;
static bool aesa_initialized;

static float compute_phase_shift(float frequency_ghz, float element_spacing_m, float steer_angle_deg) {
    float wavelength_m = 0.299792458f / frequency_ghz;
    float angle_rad = steer_angle_deg * 0.01745329251f;
    float path_diff = element_spacing_m * sinf(angle_rad);
    return fmodf((path_diff / wavelength_m) * 360.0f, 360.0f);
}

static float compute_expected_snr(float range_m, float rcs_m2, float power_w, float integration_time_s) {
    float wavelength = 0.299792458f / (CENTER_FREQ_GHZ * 1e9f);
    float gain = (4.0f * 3.14159265f * 0.8f * (MAX_ELEMENTS * 0.015f * 0.015f)) / (wavelength * wavelength);
    float noise_floor = 1.38e-23f * 290.0f * (BANDWIDTH_MHZ * 1e6f);
    float signal = (power_w * gain * gain * wavelength * wavelength * rcs_m2) / 
                   (powf(4.0f * 3.14159265f, 3.0f) * powf(range_m, 4.0f));
    return 10.0f * log10f((signal * integration_time_s) / noise_floor);
}

static void generate_lpi_hop_sequence(void) {
    float phase_accumulator = 0.0f;
    for (uint8_t i = 0; i < LPI_HOP_COUNT; i++) {
        phase_accumulator += 1.6180339887f;
        aesa_ctx.lpi_hop_pattern[i] = fmodf(phase_accumulator, 1.0f) * BANDWIDTH_MHZ;
    }
}

static void allocate_radar_resources(void) {
    float available_power = 100.0f;
    float high_priority_demand = 0.0f;
    uint8_t high_priority_count = 0;
    
    for (uint8_t i = 0; i < MAX_BEAM_DWELLS; i++) {
        if (aesa_ctx.schedule[i].is_active && aesa_ctx.schedule[i].priority <= 2) {
            high_priority_demand += aesa_ctx.schedule[i].power_percent;
            high_priority_count++;
        }
    }
    
    if (high_priority_demand > 85.0f) {
        float scale = 85.0f / high_priority_demand;
        for (uint8_t i = 0; i < MAX_BEAM_DWELLS; i++) {
            if (aesa_ctx.schedule[i].is_active && aesa_ctx.schedule[i].priority <= 2) {
                aesa_ctx.schedule[i].power_percent *= scale;
            }
        }
    }
    
    float remaining_power = 100.0f - high_priority_demand;
    uint8_t low_priority_count = 0;
    for (uint8_t i = 0; i < MAX_BEAM_DWELLS; i++) {
        if (aesa_ctx.schedule[i].is_active && aesa_ctx.schedule[i].priority > 2) {
            low_priority_count++;
        }
    }
    
    if (low_priority_count > 0) {
        float per_beam = remaining_power / (float)low_priority_count;
        for (uint8_t i = 0; i < MAX_BEAM_DWELLS; i++) {
            if (aesa_ctx.schedule[i].is_active && aesa_ctx.schedule[i].priority > 2) {
                aesa_ctx.schedule[i].power_percent = fminf(per_beam, 15.0f);
            }
        }
    }
    
    aesa_ctx.total_power_draw_w = 0.0f;
    for (uint8_t i = 0; i < MAX_BEAM_DWELLS; i++) {
        if (aesa_ctx.schedule[i].is_active) {
            aesa_ctx.total_power_draw_w += aesa_ctx.schedule[i].power_percent * 45.0f;
        }
    }
}

static void update_track_gating_and_filtering(radar_track_t *track, float measured_range, float measured_rate, float measured_az, float measured_el) {
    float range_innov = measured_range - track->state_vector[0];
    float rate_innov = measured_rate - track->state_vector[1];
    float az_innov = measured_az - track->state_vector[2];
    float el_innov = measured_el - track->state_vector[3];
    
    float range_var = track->covariance_matrix[0];
    float rate_var = track->covariance_matrix[7];
    float az_var = track->covariance_matrix[14];
    float el_var = track->covariance_matrix[21];
    
    float gating_dist = sqrtf(
        (range_innov * range_innov) / (range_var + 1e-6f) +
        (rate_innov * rate_innov) / (rate_var + 1e-6f) +
        (az_innov * az_innov) / (az_var + 1e-6f) +
        (el_innov * el_innov) / (el_var + 1e-6f)
    );
    
    if (gating_dist < GATING_SIGMA) {
        float alpha = 0.85f;
        track->state_vector[0] = track->state_vector[0] * (1.0f - alpha) + measured_range * alpha;
        track->state_vector[1] = track->state_vector[1] * (1.0f - alpha) + measured_rate * alpha;
        track->state_vector[2] = track->state_vector[2] * (1.0f - alpha) + measured_az * alpha;
        track->state_vector[3] = track->state_vector[3] * (1.0f - alpha) + measured_el * alpha;
        
        track->hits++;
        track->misses = 0;
        if (track->hits >= TRACK_INIT_THRESHOLD && track->state == TRACK_STATE_TENTATIVE) {
            track->state = TRACK_STATE_CONFIRMED;
            aesa_ctx.track_initiations++;
        }
    } else {
        track->misses++;
        track->hits = 0;
        if (track->misses >= TRACK_DELETE_THRESHOLD) {
            track->state = TRACK_STATE_LOST;
            aesa_ctx.track_drops++;
        }
    }
    
    track->last_update_tick = get_system_tick();
}

static void schedule_deterministic_timeline(void) {
    aesa_ctx.active_beam_count = 0;
    uint64_t current_tick = get_system_tick();
    uint64_t frame_start = (current_tick / 50) * 50;
    
    for (uint8_t i = 0; i < MAX_TRACK_SLOTS; i++) {
        if (aesa_ctx.tracks[i].state != TRACK_STATE_LOST) {
            if (aesa_ctx.active_beam_count >= MAX_BEAM_DWELLS) break;
            
            aesa_ctx.schedule[aesa_ctx.active_beam_count].beam_id = aesa_ctx.active_beam_count;
            aesa_ctx.schedule[aesa_ctx.active_beam_count].azimuth_cmd = aesa_ctx.tracks[i].state_vector[2];
            aesa_ctx.schedule[aesa_ctx.active_beam_count].elevation_cmd = aesa_ctx.tracks[i].state_vector[3];
            aesa_ctx.schedule[aesa_ctx.active_beam_count].dwell_time_ms = (aesa_ctx.tracks[i].state == TRACK_STATE_CONFIRMED) ? 2.0f : 4.0f;
            aesa_ctx.schedule[aesa_ctx.active_beam_count].power_percent = (aesa_ctx.tracks[i].is_threat) ? 12.0f : 6.0f;
            aesa_ctx.schedule[aesa_ctx.active_beam_count].waveform = (aesa_ctx.current_mode == RADAR_MODE_LPI_PASSIVE) ? WAVEFORM_FREQUENCY_HOP : WAVEFORM_LINEAR_FM;
            aesa_ctx.schedule[aesa_ctx.active_beam_count].priority = (aesa_ctx.tracks[i].is_threat) ? 1 : 3;
            aesa_ctx.schedule[aesa_ctx.active_beam_count].is_active = true;
            aesa_ctx.schedule[aesa_ctx.active_beam_count].schedule_start_tick = frame_start + (aesa_ctx.active_beam_count * 3);
            
            aesa_ctx.active_beam_count++;
        }
    }
    
    if (aesa_ctx.current_mode == RADAR_MODE_SEARCH_VOLUME) {
        for (uint8_t i = aesa_ctx.active_beam_count; i < MAX_BEAM_DWELLS && i < 12; i++) {
            aesa_ctx.schedule[i].beam_id = i;
            aesa_ctx.schedule[i].azimuth_cmd = -60.0f + (i * 10.0f);
            aesa_ctx.schedule[i].elevation_cmd = 0.0f;
            aesa_ctx.schedule[i].dwell_time_ms = 5.0f;
            aesa_ctx.schedule[i].power_percent = 4.0f;
            aesa_ctx.schedule[i].waveform = WAVEFORM_PHASE_CODED;
            aesa_ctx.schedule[i].priority = 4;
            aesa_ctx.schedule[i].is_active = true;
            aesa_ctx.schedule[i].schedule_start_tick = frame_start + (i * 3);
            aesa_ctx.active_beam_count++;
        }
    }
    
    allocate_radar_resources();
}

void aesa_radar_init(void) {
    memset(&aesa_ctx, 0, sizeof(aesa_context_t));
    aesa_ctx.current_mode = RADAR_MODE_SEARCH_VOLUME;
    aesa_ctx.transmitter_enabled = false;
    generate_lpi_hop_sequence();
    aesa_initialized = true;
}

void aesa_radar_cycle(nav_output_t *nav, ew_threat_list_t *ew_list) {
    if (!aesa_initialized) return;
    
    if (ew_list->active_count > 0) {
        aesa_ctx.current_mode = RADAR_MODE_LPI_PASSIVE;
    } else if (aesa_ctx.active_track_count > 8) {
        aesa_ctx.current_mode = RADAR_MODE_TRACK_WHILE_SCAN;
    } else {
        aesa_ctx.current_mode = RADAR_MODE_SEARCH_VOLUME;
    }
    
    schedule_deterministic_timeline();
    
    for (uint8_t i = 0; i < MAX_TRACK_SLOTS; i++) {
        if (aesa_ctx.tracks[i].state != TRACK_STATE_LOST) {
            float pred_range = aesa_ctx.tracks[i].state_vector[0] + (aesa_ctx.tracks[i].state_vector[1] * 0.05f);
            float expected_snr = compute_expected_snr(pred_range, 1.0f, aesa_ctx.schedule[i % MAX_BEAM_DWELLS].power_percent * 0.45f, 0.002f);
            aesa_ctx.tracks[i].snr_db = expected_snr;
            
            if (expected_snr < MIN_SNR_DB) {
                aesa_ctx.tracks[i].state = TRACK_STATE_DEGRADED;
            }
        }
    }
    
    aesa_ctx.radar_cycles++;
}

void aesa_radar_ingest_return(uint8_t track_slot, float range_m, float rate_ms, float az_deg, float el_deg, float snr_db) {
    if (track_slot >= MAX_TRACK_SLOTS) return;
    if (aesa_ctx.tracks[track_slot].state == TRACK_STATE_LOST) {
        aesa_ctx.tracks[track_slot].state = TRACK_STATE_TENTATIVE;
        aesa_ctx.tracks[track_slot].hits = 0;
        aesa_ctx.tracks[track_slot].misses = 0;
        aesa_ctx.active_track_count++;
    }
    
    aesa_ctx.tracks[track_slot].range_m = range_m;
    aesa_ctx.tracks[track_slot].range_rate_ms = rate_ms;
    aesa_ctx.tracks[track_slot].snr_db = snr_db;
    
    update_track_gating_and_filtering(&aesa_ctx.tracks[track_slot], range_m, rate_ms, az_deg, el_deg);
}

void aesa_radar_get_tracks(radar_track_t *out_tracks, uint8_t *out_count) {
    uint8_t count = 0;
    for (uint8_t i = 0; i < MAX_TRACK_SLOTS; i++) {
        if (aesa_ctx.tracks[i].state != TRACK_STATE_LOST) {
            out_tracks[count++] = aesa_ctx.tracks[i];
        }
    }
    *out_count = count;
}

void aesa_radar_set_mode(radar_operational_mode_t mode) {
    aesa_ctx.current_mode = mode;
    if (mode == RADAR_MODE_LPI_PASSIVE) {
        aesa_ctx.lpi_resets++;
        generate_lpi_hop_sequence();
    }
}

void aesa_radar_emergency_shutoff(void) {
    aesa_ctx.transmitter_enabled = false;
    for (uint8_t i = 0; i < MAX_BEAM_DWELLS; i++) {
        aesa_ctx.schedule[i].is_active = false;
    }
    hal_neural_engine_log_event(EVENT_RADAR_EMERGENCY_OFF, 0);
}
