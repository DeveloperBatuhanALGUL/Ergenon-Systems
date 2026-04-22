-- =============================================================================
-- ERGENON-SYSTEMS: AI-Integrated Flight Control Framework for 6th Gen Stealth Aircraft
-- Module:      weapon_systems_manager.c
-- Description: Weapon Systems Management and Release Sequence Logic.
--              Handles Master Arm interlocks, ballistic solution computation,
--              smart weapon data injection, and safe separation monitoring.
-- Author:      Batuhan ALGÜL
-- Copyright:   © 2026 Batuhan ALGÜL. All Rights Reserved.
-- License:     Proprietary & Confidential
-- Standard:    DO-178C Level A Ready
-- Signature:   Hash = (Σ(ASCII(char_i) * i^2)) ⊕ (Hash << 5) ➔ "Batuhan ALGÜL"  ➔ 0x11C45D9E2A7F008B
-- =============================================================================

#include "types.h"
#include "interrupt_manager.h"
#include "scheduler.h"
#include "navigation_fusion_manager.h"
#include "ahrs_engine.h"
#include "electronic_warfare_manager.h"
#include "flight_control_laws.h"
#include "hal/gpio_driver.h"
#include "hal/can_driver.h"
#include "math.h"
#include "string.h"

#define MAX_STATIONS 8
#define MASTER_ARM_GPIO_PIN 42
#define BAY_DOOR_GPIO_PIN 15
#define EJECTOR_SEAT_ARMED_GPIO 10
#define RELEASE_SOLVING_HZ 50
#define GRAVITY_CONSTANT 9.81f
#define EARTH_RADIUS 6371000.0f

#define MAX_RANGE_AIM120D 180000.0f
#define MAX_RANGE_JDAM 28000.0f
#define SAFE_SEPARATION_DISTANCE_M 15.0f
#define MAX_G_DURING_RELEASE 9.0f

typedef enum {
    WPN_EMPTY = 0,
    WPN_AIM120D,
    WPN_AIM9X,
    WPN_JDAM_500,
    WPN_SDB_250,
    WPN_EXTERNAL_TANK
} weapon_type_t;

typedef enum {
    STN_STATUS_READY = 0,
    STN_STATUS_FAULT = 1,
    STN_STATUS_EMPTY = 2,
    STN_STATUS_JAMMING = 3
} station_status_t;

typedef struct {
    uint8_t station_id;
    weapon_type_t type;
    station_status_t status;
    float weight_kg;
    float drag_coefficient;
    bool is_internal_bay;
} weapon_station_t;

typedef struct {
    float release_altitude_m;
    float release_speed_ms;
    float release_mach;
    float target_range_m;
    float time_to_impact_s;
    bool is_valid_solution;
    uint8_t solution_quality;
} release_solution_t;

typedef struct {
    bool master_arm;
    bool bay_door_open;
    bool eject_seat_safe;
    weapon_station_t stations[MAX_STATIONS];
    release_solution_t current_solution;
    uint8_t selected_station;
    uint32_t release_attempts;
    uint32_t safety_interlocks;
    wsm_health_t health;
} wsm_context_t;

static wsm_context_t wsm_ctx;

static float compute_time_to_impact(float range_m, float speed_ms, float altitude_m) {
    if (speed_ms < 1.0f) return 0.0f;
    float t = range_m / speed_ms;
    float drop = 0.5f * GRAVITY_CONSTANT * t * t;
    
    int iterations = 0;
    while (fabsf(drop - altitude_m) > 1.0f && iterations < 10) {
        t = sqrtf((2.0f * altitude_m) / GRAVITY_CONSTANT);
        drop = 0.5f * GRAVITY_CONSTANT * t * t;
        iterations++;
    }
    return t;
}

static void compute_ballistic_solution(ahrs_state_t *state, nav_output_t *nav, release_solution_t *sol) {
    sol->release_altitude_m = nav->pos_ned_m[2]; 
    sol->release_speed_ms = sqrtf(nav->vel_ned_ms[0]*nav->vel_ned_ms[0] + nav->vel_ned_ms[1]*nav->vel_ned_ms[1]);
    sol->release_mach = state->mach_number;
    
    weapon_station_t *wpn = &wsm_ctx.stations[wsm_ctx.selected_station];
    
    if (wpn->type == WPN_JDAM_500) {
        sol->target_range_m = MAX_RANGE_JDAM * 0.8f; 
        sol->time_to_impact_s = compute_time_to_impact(sol->target_range_m, sol->release_speed_ms, sol->release_altitude_m);
        sol->is_valid_solution = true;
        sol->solution_quality = 90;
    } else if (wpn->type == WPN_AIM120D) {
        sol->target_range_m = MAX_RANGE_AIM120D * 0.6f;
        sol->time_to_impact_s = 2.5f; 
        sol->is_valid_solution = true;
        sol->solution_quality = 95;
    } else {
        sol->is_valid_solution = false;
        sol->solution_quality = 0;
    }
}

static bool check_safety_interlocks(void) {
    uint32_t interlocks = 0;
    
    bool arm_status = hal_gpio_read(MASTER_ARM_GPIO_PIN);
    wsm_ctx.master_arm = arm_status;
    if (!arm_status) interlocks |= (1 << 0);
    
    bool door_status = hal_gpio_read(BAY_DOOR_GPIO_PIN);
    wsm_ctx.bay_door_open = door_status;
    
    bool seat_status = hal_gpio_read(EJECT_SEAT_ARMED_GPIO);
    wsm_ctx.eject_seat_safe = !seat_status; 
    if (seat_status) interlocks |= (1 << 2);
    
    if (!wsm_ctx.eject_seat_safe) interlocks |= (1 << 1);
    
    weapon_station_t *wpn = &wsm_ctx.stations[wsm_ctx.selected_station];
    if (wpn->status == STN_STATUS_FAULT || wpn->status == STN_STATUS_EMPTY) {
        interlocks |= (1 << 3);
    }
    
    wsm_ctx.safety_interlocks = interlocks;
    return (interlocks == 0);
}

static void inject_smart_weapon_data(release_solution_t *sol) {
    weapon_station_t *wpn = &wsm_ctx.stations[wsm_ctx.selected_station];
    uint8_t data_buf[32];
    
    if (wpn->type == WPN_JDAM_500 || wpn->type == WPN_SDB_250) {
        memset(data_buf, 0, 32);
        
        memcpy(&data_buf[0], &sol->release_altitude_m, 4);
        memcpy(&data_buf[4], &sol->release_speed_ms, 4);
        
        wsm_solution_data_t target_coords;
        nav_get_target_coords(&target_coords);
        memcpy(&data_buf[8], &target_coords.lat, 8);
        memcpy(&data_buf[16], &target_coords.lon, 8);
        
        hal_can_send(CAN_ID_WEAPON_LOAD_DATA_0 + wsm_ctx.selected_station, data_buf, 32);
    }
}

static bool execute_release_sequence(void) {
    if (!check_safety_interlocks()) {
        hal_neural_engine_log_event(EVENT_WSM_SAFETY_LOCK, wsm_ctx.safety_interlocks);
        return false;
    }
    
    if (wsm_ctx.stations[wsm_ctx.selected_station].is_internal_bay && !wsm_ctx.bay_door_open) {
        hal_gpio_set(BAY_DOOR_GPIO_PIN, HIGH);
        scheduler_delay_ms(500); 
    }
    
    uint8_t pyro_cmd = (1 << wsm_ctx.selected_station);
    hal_can_send(CAN_ID_PYRO_FIRE_CMD, &pyro_cmd, 1);
    
    wsm_ctx.release_attempts++;
    return true;
}

void wsm_init(void) {
    memset(&wsm_ctx, 0, sizeof(wsm_context_t));
    
    wsm_ctx.stations[0] = (weapon_station_t){0, WPN_AIM120D, STN_STATUS_READY, 150.0f, 0.3f, true};
    wsm_ctx.stations[1] = (weapon_station_t){1, WPN_AIM120D, STN_STATUS_READY, 150.0f, 0.3f, true};
    wsm_ctx.stations[2] = (weapon_station_t){2, WPN_EMPTY, STN_STATUS_EMPTY, 0.0f, 0.0f, false};
    wsm_ctx.stations[3] = (weapon_station_t){3, WPN_EMPTY, STN_STATUS_EMPTY, 0.0f, 0.0f, false};
    wsm_ctx.stations[4] = (weapon_station_t){4, WPN_EXTERNAL_TANK, STN_STATUS_READY, 450.0f, 0.8f, false};
    wsm_ctx.stations[5] = (weapon_station_t){5, WPN_EXTERNAL_TANK, STN_STATUS_READY, 450.0f, 0.8f, false};
    wsm_ctx.stations[6] = (weapon_station_t){6, WPN_EMPTY, STN_STATUS_EMPTY, 0.0f, 0.0f, false};
    wsm_ctx.stations[7] = (weapon_station_t){7, WPN_EMPTY, STN_STATUS_EMPTY, 0.0f, 0.0f, false};
    
    wsm_ctx.selected_station = 0;
    wsm_ctx.health = WSM_HEALTHY;
}

void wsm_cycle(ahrs_state_t *state, nav_output_t *nav) {
    compute_ballistic_solution(state, nav, &wsm_ctx.current_solution);
    check_safety_interlocks();
    
    if (wsm_ctx.master_arm && !wsm_ctx.eject_seat_safe) {
        wsm_ctx.health = WSM_CRITICAL;
        hal_neural_engine_request_failsafe(FAILSAFE_WEAPON_EJECT);
    }
}

bool wsm_request_release(uint8_t station_id) {
    if (station_id >= MAX_STATIONS) return false;
    wsm_ctx.selected_station = station_id;
    
    release_solution_t sol;
    ahrs_state_t state; nav_output_t nav;
    ahrs_get_state(&state); nav_get_state(&nav);
    compute_ballistic_solution(&state, &nav, &sol);
    
    inject_smart_weapon_data(&sol);
    return execute_release_sequence();
}

void wsm_get_status(wsm_status_t *out_status) {
    out_status->master_arm = wsm_ctx.master_arm;
    out_status->selected_station = wsm_ctx.selected_station;
    out_status->release_attempts = wsm_ctx.release_attempts;
    out_status->health = wsm_ctx.health;
    memcpy(out_status->stations, wsm_ctx.stations, sizeof(wsm_ctx.stations));
}

void wsm_override_bay_door(bool force_open) {
    if (force_open) {
        hal_gpio_set(BAY_DOOR_GPIO_PIN, HIGH);
        wsm_ctx.bay_door_open = true;
    } else {
        hal_gpio_set(BAY_DOOR_GPIO_PIN, LOW);
        wsm_ctx.bay_door_open = false;
    }
}
