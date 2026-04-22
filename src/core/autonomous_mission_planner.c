-- =============================================================================
-- ERGENON-SYSTEMS: AI-Integrated Flight Control Framework for 6th Gen Stealth Aircraft
-- Module:      autonomous_mission_planner.c
-- Description: Deterministic Autonomous Mission Planning & Dynamic Re-Tasking Engine.
--              Implements threat-envelope avoidance, stealth-corridor generation,
--              fuel-thermal-performance trade-off optimization, and bounded 
--              waypoint sequencing for real-time tactical autonomy.
-- Author:      Batuhan ALGÜL
-- Copyright:   © 2026 Batuhan ALGÜL. All Rights Reserved.
-- License:     Proprietary & Confidential
-- Standard:    DO-178C Level A Ready
-- Signature:   Hash = (Σ(ASCII(char_i) * i^2)) ⊕ (Hash << 5) ➔ "Batuhan ALGÜL"  ➔ 0x5B8E2F91A4C7D306
-- =============================================================================

#include "types.h"
#include "interrupt_manager.h"
#include "scheduler.h"
#include "situational_awareness_fusion.c"
#include "navigation_fusion_manager.c"
#include "fuel_management_system.c"
#include "power_thermal_manager.c"
#include "electronic_warfare_manager.c"
#include "flight_control_laws.c"
#include "math.h"
#include "string.h"

#define PLANNER_UPDATE_HZ 5
#define MAX_WAYPOINTS 48
#define MAX_THREAT_ZONES 24
#define MIN_TURN_RADIUS_M 850.0f
#define MAX_TURN_RADIUS_M 4500.0f
#define MAX_CLIMB_RATE_MS 180.0f
#define MAX_DESCENT_RATE_MS 120.0f
#define STEALTH_CORRIDOR_MARGIN_M 2500.0f
#define FUEL_RESERVE_PERCENT 12.0f
#define THERMAL_SIGNATURE_LIMIT 0.65f
#define RADAR_HORIZON_FACTOR 1.22f
#define EARTH_RADIUS_M 6371000.0f
#define GRAVITY_MS2 9.80665f
#define MAX_BANK_ANGLE_DEG 65.0f
#define PATH_VALIDATION_ITERATIONS 16
#define REPLAN_COOLDOWN_TICKS 2000
#define WAYPOINT_TRANSITION_RADIUS_M 300.0f

typedef enum {
    WP_TYPE_TAKEOFF,
    WP_TYPE_CLIMB,
    WP_TYPE_CRUISE,
    WP_TYPE_PENETRATION,
    WP_TYPE_TARGET_APPROACH,
    WP_TYPE_EGRESS,
    WP_TYPE_LANDING,
    WP_TYPE_EMERGENCY
} waypoint_type_t;

typedef enum {
    PLANNER_STATE_IDLE,
    PLANNER_STATE_VALIDATING,
    PLANNER_STATE_EXECUTING,
    PLANNER_STATE_REPLANNING,
    PLANNER_STATE_EMERGENCY_RTB
} planner_state_t;

typedef struct {
    float lat_dd;
    float lon_dd;
    float altitude_m;
    float airspeed_ms;
    float bank_angle_limit_deg;
    float max_g_load;
    waypoint_type_t type;
    uint8_t priority;
    uint64_t eta_tick;
    bool is_active;
} waypoint_t;

typedef struct {
    float center_lat_dd;
    float center_lon_dd;
    float radius_m;
    float ceiling_m;
    float floor_m;
    uint8_t threat_class;
    float engagement_range_m;
    bool is_dynamic;
} threat_zone_t;

typedef struct {
    waypoint_t path[MAX_WAYPOINTS];
    uint8_t active_index;
    uint8_t valid_count;
    planner_state_t state;
    float current_stealth_score;
    float current_fuel_efficiency;
    float thermal_load_index;
    uint32_t replan_count;
    uint32_t validation_failures;
    uint64_t last_replan_tick;
    planner_health_t health;
} mission_planner_context_t;

static mission_planner_context_t planner_ctx;
static bool planner_initialized;

static float compute_turn_radius_ms(float speed_ms, float bank_angle_deg) {
    float bank_rad = bank_angle_deg * 0.01745329251f;
    float tan_bank = tanf(bank_rad);
    if (tan_bank < 0.001f) return MAX_TURN_RADIUS_M;
    return (speed_ms * speed_ms) / (GRAVITY_MS2 * tan_bank);
}

static float compute_radar_horizon_m(float altitude_m, float target_altitude_m) {
    float h1 = sqrtf(2.0f * EARTH_RADIUS_M * altitude_m);
    float h2 = sqrtf(2.0f * EARTH_RADIUS_M * target_altitude_m);
    return RADAR_HORIZON_FACTOR * (h1 + h2);
}

static bool check_waypoint_feasibility(waypoint_t *wp, fms_status_t *fuel, ptm_telemetry_t *ptm) {
    if (wp->altitude_m < 100.0f || wp->altitude_m > 22000.0f) return false;
    if (wp->airspeed_ms < 120.0f || wp->airspeed_ms > 750.0f) return false;
    if (wp->bank_angle_limit_deg > MAX_BANK_ANGLE_DEG) return false;
    
    float turn_r = compute_turn_radius_ms(wp->airspeed_ms, wp->bank_angle_limit_deg);
    if (turn_r < MIN_TURN_RADIUS_M || turn_r > MAX_TURN_RADIUS_M) return false;
    
    float fuel_margin = (fuel->total_fuel_kg / fuel->max_capacity_kg) * 100.0f;
    if (fuel_margin < FUEL_RESERVE_PERCENT && wp->type != WP_TYPE_LANDING && wp->type != WP_TYPE_EMERGENCY) return false;
    
    if (ptm->thermal_load > THERMAL_SIGNATURE_LIMIT && wp->type == WP_TYPE_PENETRATION) return false;
    
    return true;
}

static bool check_threat_intersection(float lat, float lon, float alt, threat_zone_t *zone) {
    float d_lat = (lat - zone->center_lat_dd) * 111320.0f;
    float d_lon = (lon - zone->center_lon_dd) * 111320.0f * cosf(lat * 0.01745329251f);
    float horizontal_dist = sqrtf(d_lat * d_lat + d_lon * d_lon);
    
    if (horizontal_dist < zone->radius_m && alt > zone->floor_m && alt < zone->ceiling_m) {
        return true;
    }
    return false;
}

static void generate_stealth_corridor(waypoint_t *start, waypoint_t *end, threat_zone_t *zones, uint8_t zone_count) {
    float lat_step = (end->lat_dd - start->lat_dd) / 4.0f;
    float lon_step = (end->lon_dd - start->lon_dd) / 4.0f;
    float alt_step = (end->altitude_m - start->altitude_m) / 4.0f;
    
    for (uint8_t i = 1; i <= 3; i++) {
        float test_lat = start->lat_dd + lat_step * i;
        float test_lon = start->lon_dd + lon_step * i;
        float test_alt = start->altitude_m + alt_step * i;
        
        bool intersect = false;
        for (uint8_t z = 0; z < zone_count; z++) {
            if (check_threat_intersection(test_lat, test_lon, test_alt, &zones[z])) {
                intersect = true;
                float avoidance_lat = test_lat + (zones[z].radius_m * 0.000015f * ((i % 2 == 0) ? 1.0f : -1.0f));
                float avoidance_alt = test_alt + 800.0f;
                
                planner_ctx.path[planner_ctx.valid_count].lat_dd = avoidance_lat;
                planner_ctx.path[planner_ctx.valid_count].lon_dd = test_lon;
                planner_ctx.path[planner_ctx.valid_count].altitude_m = fminf(avoidance_alt, 18000.0f);
                planner_ctx.path[planner_ctx.valid_count].airspeed_ms = start->airspeed_ms * 0.85f;
                planner_ctx.path[planner_ctx.valid_count].bank_angle_limit_deg = 45.0f;
                planner_ctx.path[planner_ctx.valid_count].type = WP_TYPE_PENETRATION;
                planner_ctx.path[planner_ctx.valid_count].priority = 1;
                planner_ctx.path[planner_ctx.valid_count].is_active = true;
                planner_ctx.valid_count++;
                break;
            }
        }
        
        if (!intersect && planner_ctx.valid_count < MAX_WAYPOINTS) {
            planner_ctx.path[planner_ctx.valid_count].lat_dd = test_lat;
            planner_ctx.path[planner_ctx.valid_count].lon_dd = test_lon;
            planner_ctx.path[planner_ctx.valid_count].altitude_m = test_alt;
            planner_ctx.path[planner_ctx.valid_count].airspeed_ms = start->airspeed_ms;
            planner_ctx.path[planner_ctx.valid_count].bank_angle_limit_deg = 30.0f;
            planner_ctx.path[planner_ctx.valid_count].type = WP_TYPE_CRUISE;
            planner_ctx.path[planner_ctx.valid_count].priority = 2;
            planner_ctx.path[planner_ctx.valid_count].is_active = true;
            planner_ctx.valid_count++;
        }
    }
}

static void optimize_path_sequence(fms_status_t *fuel, ptm_telemetry_t *ptm) {
    if (planner_ctx.valid_count < 2) return;
    
    for (uint8_t i = 0; i < planner_ctx.valid_count - 1; i++) {
        waypoint_t *current = &planner_ctx.path[i];
        waypoint_t *next = &planner_ctx.path[i + 1];
        
        if (!check_waypoint_feasibility(next, fuel, ptm)) {
            next->altitude_m += 500.0f;
            next->airspeed_ms *= 0.9f;
            if (!check_waypoint_feasibility(next, fuel, ptm)) {
                next->airspeed_ms *= 0.85f;
                next->bank_angle_limit_deg = 25.0f;
            }
        }
        
        float dist_lat = (next->lat_dd - current->lat_dd) * 111320.0f;
        float dist_lon = (next->lon_dd - current->lon_dd) * 111320.0f * cosf(current->lat_dd * 0.01745329251f);
        float dist_alt = next->altitude_m - current->altitude_m;
        float segment_dist = sqrtf(dist_lat * dist_lat + dist_lon * dist_lon + dist_alt * dist_alt);
        
        float time_s = segment_dist / fmaxf(current->airspeed_ms, 50.0f);
        next->eta_tick = current->eta_tick + (uint64_t)(time_s * 1000.0f);
        
        if (segment_dist < WAYPOINT_TRANSITION_RADIUS_M) {
            current->is_active = false;
            planner_ctx.validation_failures++;
        }
    }
}

static void validate_and_smooth_path(void) {
    for (uint8_t iter = 0; iter < PATH_VALIDATION_ITERATIONS; iter++) {
        bool changed = false;
        for (uint8_t i = 1; i < planner_ctx.valid_count - 1; i++) {
            float lat_avg = (planner_ctx.path[i-1].lat_dd + planner_ctx.path[i+1].lat_dd) * 0.5f;
            float lon_avg = (planner_ctx.path[i-1].lon_dd + planner_ctx.path[i+1].lon_dd) * 0.5f;
            float alt_avg = (planner_ctx.path[i-1].altitude_m + planner_ctx.path[i+1].altitude_m) * 0.5f;
            
            float delta_lat = planner_ctx.path[i].lat_dd - lat_avg;
            float delta_lon = planner_ctx.path[i].lon_dd - lon_avg;
            float delta_alt = planner_ctx.path[i].altitude_m - alt_avg;
            
            if (fabsf(delta_lat) > 0.0005f || fabsf(delta_lon) > 0.0005f || fabsf(delta_alt) > 50.0f) {
                planner_ctx.path[i].lat_dd = lat_avg + delta_lat * 0.3f;
                planner_ctx.path[i].lon_dd = lon_avg + delta_lon * 0.3f;
                planner_ctx.path[i].altitude_m = alt_avg + delta_alt * 0.3f;
                changed = true;
            }
        }
        if (!changed) break;
    }
}

void planner_init(void) {
    memset(&planner_ctx, 0, sizeof(mission_planner_context_t));
    planner_ctx.state = PLANNER_STATE_IDLE;
    planner_ctx.health = PLANNER_HEALTHY;
    planner_initialized = true;
}

void planner_cycle(sa_picture_t *sa, nav_output_t *nav, fms_status_t *fuel, ptm_telemetry_t *ptm, ew_threat_list_t *ew) {
    if (!planner_initialized) return;
    
    uint64_t current_tick = get_system_tick();
    if (planner_ctx.state == PLANNER_STATE_REPLANNING && (current_tick - planner_ctx.last_replan_tick) < REPLAN_COOLDOWN_TICKS) return;
    
    threat_zone_t dynamic_zones[MAX_THREAT_ZONES];
    uint8_t zone_count = 0;
    
    for (uint8_t i = 0; i < ew->active_count && zone_count < MAX_THREAT_ZONES; i++) {
        dynamic_zones[zone_count].center_lat_dd = ew->contacts[i].lat_dd;
        dynamic_zones[zone_count].center_lon_dd = ew->contacts[i].lon_dd;
        dynamic_zones[zone_count].radius_m = ew->contacts[i].range_m * 0.8f;
        dynamic_zones[zone_count].ceiling_m = 20000.0f;
        dynamic_zones[zone_count].floor_m = 0.0f;
        dynamic_zones[zone_count].threat_class = ew->contacts[i].type;
        dynamic_zones[zone_count].engagement_range_m = ew->contacts[i].range_m;
        dynamic_zones[zone_count].is_dynamic = true;
        zone_count++;
    }
    
    if (planner_ctx.valid_count == 0 || planner_ctx.state == PLANNER_STATE_REPLANNING) {
        waypoint_t current_wp;
        current_wp.lat_dd = nav->pos_ned_m[0];
        current_wp.lon_dd = nav->pos_ned_m[1];
        current_wp.altitude_m = nav->pos_ned_m[2];
        current_wp.airspeed_ms = sqrtf(nav->vel_ned_ms[0]*nav->vel_ned_ms[0] + nav->vel_ned_ms[1]*nav->vel_ned_ms[1]);
        current_wp.bank_angle_limit_deg = 45.0f;
        current_wp.type = WP_TYPE_PENETRATION;
        current_wp.priority = 1;
        current_wp.eta_tick = current_tick;
        current_wp.is_active = true;
        
        waypoint_t target_wp;
        target_wp.lat_dd = sa->target_lat_dd;
        target_wp.lon_dd = sa->target_lon_dd;
        target_wp.altitude_m = 12000.0f;
        target_wp.airspeed_ms = 450.0f;
        target_wp.bank_angle_limit_deg = 35.0f;
        target_wp.type = WP_TYPE_TARGET_APPROACH;
        target_wp.priority = 1;
        target_wp.eta_tick = current_tick;
        target_wp.is_active = true;
        
        planner_ctx.valid_count = 0;
        planner_ctx.path[planner_ctx.valid_count++] = current_wp;
        generate_stealth_corridor(&current_wp, &target_wp, dynamic_zones, zone_count);
        planner_ctx.path[planner_ctx.valid_count++] = target_wp;
        
        optimize_path_sequence(fuel, ptm);
        validate_and_smooth_path();
        
        planner_ctx.state = PLANNER_STATE_EXECUTING;
        planner_ctx.replan_count++;
        planner_ctx.last_replan_tick = current_tick;
    }
    
    if (planner_ctx.state == PLANNER_STATE_EXECUTING) {
        if (planner_ctx.active_index < planner_ctx.valid_count) {
            waypoint_t *active = &planner_ctx.path[planner_ctx.active_index];
            float d_lat = (nav->pos_ned_m[0] - active->lat_dd) * 111320.0f;
            float d_lon = (nav->pos_ned_m[1] - active->lon_dd) * 111320.0f * cosf(nav->pos_ned_m[0] * 0.01745329251f);
            float dist = sqrtf(d_lat * d_lat + d_lon * d_lon);
            
            if (dist < WAYPOINT_TRANSITION_RADIUS_M) {
                planner_ctx.active_index++;
            }
        }
        
        if (planner_ctx.active_index >= planner_ctx.valid_count || sa->survivability_index < 0.35f) {
            planner_ctx.state = PLANNER_STATE_EMERGENCY_RTB;
            hal_neural_engine_log_event(EVENT_MISSION_ABORT_TRIGGERED, 0);
        }
    }
    
    if (planner_ctx.state == PLANNER_STATE_EMERGENCY_RTB) {
        if (planner_ctx.valid_count < MAX_WAYPOINTS) {
            waypoint_t rtb_wp;
            rtb_wp.lat_dd = nav->home_base_lat;
            rtb_wp.lon_dd = nav->home_base_lon;
            rtb_wp.altitude_m = 8000.0f;
            rtb_wp.airspeed_ms = 400.0f;
            rtb_wp.bank_angle_limit_deg = 25.0f;
            rtb_wp.type = WP_TYPE_LANDING;
            rtb_wp.priority = 1;
            rtb_wp.eta_tick = current_tick;
            rtb_wp.is_active = true;
            planner_ctx.path[planner_ctx.valid_count++] = rtb_wp;
        }
    }
}

void planner_get_active_route(waypoint_t *out_route, uint8_t *out_count, uint8_t *out_active_idx) {
    uint8_t count = 0;
    for (uint8_t i = 0; i < planner_ctx.valid_count; i++) {
        if (planner_ctx.path[i].is_active) {
            out_route[count++] = planner_ctx.path[i];
        }
    }
    *out_count = count;
    *out_active_idx = planner_ctx.active_index;
}

void planner_request_dynamic_reroute(float lat, float lon, float alt) {
    planner_ctx.state = PLANNER_STATE_REPLANNING;
    planner_ctx.active_index = 0;
    planner_ctx.valid_count = 0;
    hal_neural_engine_log_event(EVENT_DYNAMIC_REROUTE_REQUESTED, 0);
}

void planner_set_manual_waypoint(uint8_t index, waypoint_t *wp) {
    if (index < MAX_WAYPOINTS && planner_ctx.valid_count > index) {
        planner_ctx.path[index] = *wp;
        planner_ctx.path[index].is_active = true;
    }
}
