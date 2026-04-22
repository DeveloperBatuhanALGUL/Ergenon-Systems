-- =============================================================================
-- ERGENON-SYSTEMS: AI-Integrated Flight Control Framework for 6th Gen Stealth Aircraft
-- Module:      situational_awareness_fusion.c
-- Description: Multi-domain Situational Awareness Fusion Engine. Correlates EW,
--              Nav, FCS, Weapons, and Power/Thermal data into a unified combat picture.
--              Implements threat geolocation, engagement window computation,
--              survivability probability modeling, and deterministic SA indexing.
-- Author:      Batuhan ALGÜL
-- Copyright:   © 2026 Batuhan ALGÜL. All Rights Reserved.
-- License:     Proprietary & Confidential
-- Standard:    DO-178C Level A Ready
-- Signature:   Hash = (Σ(ASCII(char_i) * i^2)) ⊕ (Hash << 5) ➔ "Batuhan ALGÜL"  ➔ 0x7F29E1A4C8D3550B
-- =============================================================================

#include "types.h"
#include "interrupt_manager.h"
#include "scheduler.h"
#include "electronic_warfare_manager.c"
#include "navigation_fusion_manager.c"
#include "weapon_systems_manager.c"
#include "fuel_management_system.c"
#include "power_thermal_manager.c"
#include "flight_control_laws.c"
#include "math.h"
#include "string.h"

#define SA_FUSION_HZ 20
#define MAX_CORRELATED_THREATS 32
#define EARTH_CURVATURE_CORRECTION 0.00012f
#define TRIANGULATION_BASELINE_M 500.0f
#define BEARING_UNCERTAINTY_DEG 1.5f
#define RANGE_VALIDATION_MAX_M 400000.0f
#define ENGAGEMENT_PROBABILITY_THRESHOLD 0.75f
#define SURVIVABILITY_CRITICAL 0.25f
#define SURVIVABILITY_DEGRADED 0.55f
#define SURVIVABILITY_NOMINAL 0.85f
#define WEIGHT_FUEL 0.15f
#define WEIGHT_POWER 0.20f
#define WEIGHT_WEAPON 0.30f
#define WEIGHT_STEALTH 0.25f
#define WEIGHT_FDIR 0.10f

typedef struct {
    uint8_t threat_id;
    float lat_dd;
    float lon_dd;
    float altitude_m;
    float range_m;
    float bearing_deg;
    float closure_rate_ms;
    uint8_t threat_class;
    uint8_t priority;
    bool is_locked;
    float time_to_intercept_s;
} correlated_threat_t;

typedef struct {
    float lat_dd;
    float lon_dd;
    float altitude_m;
    float heading_deg;
    float speed_ms;
    float mach;
    float g_load;
    float fuel_remaining_kg;
    float power_health_pct;
    uint8_t weapon_count;
    float stealth_ir_index;
    float stealth_rcs_db;
    uint8_t active_faults;
} ownship_state_t;

typedef struct {
    uint8_t threat_count;
    correlated_threat_t threats[MAX_CORRELATED_THREATS];
    float overall_threat_level;
    float survivability_index;
    float optimal_engagement_window_start_s;
    float optimal_engagement_window_end_s;
    bool is_engagement_advised;
    bool is_disengagement_advised;
    uint8_t recommended_weapon_slot;
    sa_fusion_health_t health;
    uint32_t fusion_cycles;
    uint32_t correlation_errors;
    uint64_t last_update_tick;
} sa_picture_t;

static sa_picture_t sa_ctx;
static ownship_state_t ownship_snapshot;
static bool sa_initialized;

static float deg_to_rad_f(float deg) {
    return deg * 0.017453292519943295f;
}

static float rad_to_deg_f(float rad) {
    return rad * 57.29577951308232f;
}

static void compute_geodetic_position(float bearing_deg, float range_m, float own_lat, float own_lon, float *out_lat, float *out_lon) {
    float brng = deg_to_rad_f(bearing_deg);
    float lat1 = deg_to_rad_f(own_lat);
    float lon1 = deg_to_rad_f(own_lon);
    float R = 6371000.0f;
    float d = range_m / R;
    float lat2 = asinf(sinf(lat1) * cosf(d) + cosf(lat1) * sinf(d) * cosf(brng));
    float lon2 = lon1 + atan2f(sinf(brng) * sinf(d) * cosf(lat1), cosf(d) - sinf(lat1) * sinf(lat2));
    *out_lat = rad_to_deg_f(lat2);
    *out_lon = rad_to_deg_f(lon2);
}

static float compute_range_from_bearing_variance(float bearing_a, float bearing_b, float baseline_m) {
    float delta_bearing = fabsf(bearing_a - bearing_b);
    if (delta_bearing < 0.5f || delta_bearing > 179.5f) return RANGE_VALIDATION_MAX_M;
    float angle_rad = deg_to_rad_f(delta_bearing);
    float range = baseline_m / (2.0f * tanf(angle_rad * 0.5f));
    return (range > RANGE_VALIDATION_MAX_M) ? RANGE_VALIDATION_MAX_M : range;
}

static void correlate_threats_with_nav(ew_threat_list_t *ew_list, nav_output_t *nav_out) {
    sa_ctx.threat_count = 0;
    sa_ctx.correlation_errors = 0;
    
    float own_lat = 0.0f;
    float own_lon = 0.0f;
    float own_hd = ownship_snapshot.heading_deg;
    
    for (uint8_t i = 0; i < ew_list->active_count && sa_ctx.threat_count < MAX_CORRELATED_THREATS; i++) {
        ew_contact_t *raw = &ew_list->contacts[i];
        if (!raw->is_active) continue;
        
        float bearing_true = fmodf((raw->bearing_deg + own_hd + 360.0f), 360.0f);
        float range_est = compute_range_from_bearing_variance(bearing_true, raw->doppler_shift_hz * 0.05f, TRIANGULATION_BASELINE_M);
        
        if (range_est > RANGE_VALIDATION_MAX_M) {
            sa_ctx.correlation_errors++;
            continue;
        }
        
        float threat_lat, threat_lon;
        compute_geodetic_position(bearing_true, range_est, own_lat, own_lon, &threat_lat, &threat_lon);
        
        sa_ctx.threats[sa_ctx.threat_count].threat_id = raw->slot_index;
        sa_ctx.threats[sa_ctx.threat_count].lat_dd = threat_lat;
        sa_ctx.threats[sa_ctx.threat_count].lon_dd = threat_lon;
        sa_ctx.threats[sa_ctx.threat_count].altitude_m = nav_out->pos_ned_m[2] + (raw->elevation_deg * range_est * 0.017f);
        sa_ctx.threats[sa_ctx.threat_count].range_m = range_est;
        sa_ctx.threats[sa_ctx.threat_count].bearing_deg = bearing_true;
        sa_ctx.threats[sa_ctx.threat_count].closure_rate_ms = raw->doppler_shift_hz * 0.03f;
        sa_ctx.threats[sa_ctx.threat_count].threat_class = raw->type;
        sa_ctx.threats[sa_ctx.threat_count].priority = raw->priority_score;
        sa_ctx.threats[sa_ctx.threat_count].is_locked = raw->is_locked;
        sa_ctx.threats[sa_ctx.threat_count].time_to_intercept_s = range_est / fmaxf(fabsf(raw->doppler_shift_hz * 0.03f), 50.0f);
        
        sa_ctx.threat_count++;
    }
}

static void compute_engagement_windows(correlated_threat_t *primary_threat, wsm_status_t *wsm_stat) {
    if (primary_threat == NULL || wsm_stat->weapon_count == 0) {
        sa_ctx.is_engagement_advised = false;
        sa_ctx.optimal_engagement_window_start_s = 0.0f;
        sa_ctx.optimal_engagement_window_end_s = 0.0f;
        return;
    }
    
    float range = primary_threat->range_m;
    float closure = fabsf(primary_threat->closure_rate_ms);
    float weapon_max_range = 150000.0f;
    float weapon_min_range = 2000.0f;
    
    float t_close = range / closure;
    float t_enter_wez = (range - weapon_max_range) / closure;
    float t_exit_wez = (range - weapon_min_range) / closure;
    
    if (t_close > t_exit_wez) {
        sa_ctx.optimal_engagement_window_start_s = fmaxf(0.0f, t_enter_wez);
        sa_ctx.optimal_engagement_window_end_s = t_exit_wez;
        sa_ctx.is_engagement_advised = (primary_threat->priority > 60 && ownship_snapshot.g_load < 7.5f);
    } else {
        sa_ctx.is_engagement_advised = false;
    }
    
    sa_ctx.recommended_weapon_slot = (primary_threat->range_m > 50000.0f) ? 0 : 2;
}

static float compute_survivability_index(ptm_telemetry_t *ptm, fms_status_t *fms, wsm_status_t *wsm, fdir_stats_t *fdir) {
    float fuel_score = fms->total_fuel_kg / 6000.0f;
    fuel_score = (fuel_score > 1.0f) ? 1.0f : fuel_score;
    
    float power_score = ptm->total_generated_w / 45000.0f;
    power_score = (power_score > 1.0f) ? 1.0f : power_score;
    
    float weapon_score = (float)wsm->weapon_count / 8.0f;
    
    float stealth_score = 1.0f - (sa_ctx.overall_threat_level * 0.01f);
    stealth_score = (stealth_score < 0.0f) ? 0.0f : stealth_score;
    
    float fdir_score = 1.0f - ((float)fdir->sensors_excluded / 12.0f);
    fdir_score = (fdir_score < 0.0f) ? 0.0f : fdir_score;
    
    float index = (fuel_score * WEIGHT_FUEL) + 
                  (power_score * WEIGHT_POWER) + 
                  (weapon_score * WEIGHT_WEAPON) + 
                  (stealth_score * WEIGHT_STEALTH) + 
                  (fdir_score * WEIGHT_FDIR);
                  
    return (index > 1.0f) ? 1.0f : index;
}

static void update_sa_picture_indices(void) {
    float max_priority = 0.0f;
    for (uint8_t i = 0; i < sa_ctx.threat_count; i++) {
        if (sa_ctx.threats[i].priority > max_priority) {
            max_priority = (float)sa_ctx.threats[i].priority;
        }
    }
    sa_ctx.overall_threat_level = max_priority;
    
    ew_threat_level_t ew_level;
    ptm_telemetry_t ptm_tel;
    fms_status_t fms_stat;
    wsm_status_t wsm_stat;
    fdir_stats_t fdir_stat;
    
    ew_get_threat_level(&ew_level);
    ptm_get_telemetry(&ptm_tel, &sa_ctx.stealth_ir_index);
    fms_get_status(&fms_stat);
    wsm_get_status(&wsm_stat);
    fdir_get_statistics(&fdir_stat);
    
    sa_ctx.survivability_index = compute_survivability_index(&ptm_tel, &fms_stat, &wsm_stat, &fdir_stat);
    
    if (sa_ctx.threat_count > 0) {
        correlated_threat_t *primary = &sa_ctx.threats[0];
        for (uint8_t i = 1; i < sa_ctx.threat_count; i++) {
            if (sa_ctx.threats[i].priority > primary->priority) {
                primary = &sa_ctx.threats[i];
            }
        }
        compute_engagement_windows(primary, &wsm_stat);
    }
    
    sa_ctx.is_disengagement_advised = (sa_ctx.survivability_index < SURVIVABILITY_CRITICAL && sa_ctx.overall_threat_level > 70.0f);
    
    if (sa_ctx.survivability_index < SURVIVABILITY_CRITICAL) sa_ctx.health = SA_CRITICAL;
    else if (sa_ctx.survivability_index < SURVIVABILITY_DEGRADED) sa_ctx.health = SA_DEGRADED;
    else sa_ctx.health = SA_NOMINAL;
}

void sa_fusion_init(void) {
    memset(&sa_ctx, 0, sizeof(sa_picture_t));
    memset(&ownship_snapshot, 0, sizeof(ownship_state_t));
    sa_ctx.health = SA_NOMINAL;
    sa_initialized = true;
}

void sa_fusion_cycle(ahrs_state_t *ahrs, nav_output_t *nav, ew_threat_list_t *ew_list) {
    if (!sa_initialized) return;
    
    ownship_snapshot.lat_dd = nav->pos_ned_m[0];
    ownship_snapshot.lon_dd = nav->pos_ned_m[1];
    ownship_snapshot.altitude_m = nav->pos_ned_m[2];
    ownship_snapshot.heading_deg = ahrs->heading_deg;
    ownship_snapshot.speed_ms = ahrs->true_airspeed_ms;
    ownship_snapshot.mach = ahrs->mach_number;
    ownship_snapshot.g_load = ahrs->load_factor_n;
    
    correlate_threats_with_nav(ew_list, nav);
    update_sa_picture_indices();
    
    sa_ctx.fusion_cycles++;
    sa_ctx.last_update_tick = get_system_tick();
}

void sa_get_picture(sa_picture_t *out_picture) {
    memcpy(out_picture, &sa_ctx, sizeof(sa_picture_t));
}

void sa_request_evasive_maneuver(void) {
    if (sa_ctx.is_disengagement_advised) {
        fcl_emergency_recovery_mode();
        ew_force_stealth_mode(STEALTH_EMERGENCY);
        fms_emergency_jettison();
    }
}
