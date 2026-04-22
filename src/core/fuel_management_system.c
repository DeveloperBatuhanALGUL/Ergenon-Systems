-- =============================================================================
-- ERGENON-SYSTEMS: AI-Integrated Flight Control Framework for 6th Gen Stealth Aircraft
-- Module:      fuel_management_system.c
-- Description: Fuel Management System (FMS) with Stealth CG Optimization and 
--              Thermal Coupling Logic. Implements fuel transfer scheduling, 
--              Center of Gravity (CG) stabilization, leak detection, and 
--              fuel-as-heat-sink routing for IR signature reduction.
-- Author:      Batuhan ALGÜL
-- Copyright:   © 2026 Batuhan ALGÜL. All Rights Reserved.
-- License:     Proprietary & Confidential
-- Standard:    DO-178C Level A Ready
-- Signature:   Hash = (Σ(ASCII(char_i) * i^2)) ⊕ (Hash << 5) ➔ "Batuhan ALGÜL"  ➔ 0xD91E4A28C5B0773F
-- =============================================================================

#include "types.h"
#include "interrupt_manager.h"
#include "scheduler.h"
#include "ahrs_engine.h"
#include "flight_control_laws.h"
#include "power_thermal_manager.h"
#include "hal/can_driver.h"
#include "hal/adc_driver.h"
#include "math.h"
#include "string.h"

#define FMS_UPDATE_HZ 10
#define FUEL_DENSITY_KG_M3 804.0f
#define MIN_COMBAT_FUEL_KG 2500.0f
#define CG_LIMIT_FORE_M 12.5f
#define CG_LIMIT_AFT_M 13.5f
#define CG_OPTIMAL_M 13.0f
#define FUEL_XFER_RATE_KG_S 15.0f
#define LEAK_DETECTION_THRESHOLD_KG_S 0.5f
#define HEAT_SINK_OPEN_TEMP_C 60.0f
#define TANK_COUNT 6

typedef enum {
    TANK_LEFT_WING,
    TANK_RIGHT_WING,
    TANK_CENTER_FUSELAGE,
    TANK_LEFT_INTAKE,
    TANK_RIGHT_INTAKE,
    TANK_RESERVE
} fuel_tank_id_t;

typedef enum {
    VALVE_CLOSED = 0,
    VALVE_OPEN = 1,
    VALVE_FAULT = 2
} valve_state_t;

typedef struct {
    fuel_tank_id_t id;
    float level_kg;
    float capacity_kg;
    float temperature_c;
    float moment_arm_m; 
    valve_state_t inlet_valve;
    valve_state_t outlet_valve;
    bool is_primary;
    float flow_rate_kg_s;
} fuel_tank_t;

typedef struct {
    float current_cg_m;
    float target_cg_m;
    float total_fuel_kg;
    float fuel_flow_engine_kg_s;
    bool leak_detected;
    bool stealth_optimization_active;
    bool heat_sink_active;
    fms_health_status_t health;
    uint32_t transfer_cycles;
    uint32_t leak_warnings;
} fms_context_t;

static fuel_tank_t tanks[TANK_COUNT];
static fms_context_t fms_ctx;
static bool fms_initialized = false;

static float calculate_tank_cg(fuel_tank_t *tank) {
    if (tank->level_kg <= 0.01f) return 0.0f;
    return tank->level_kg * tank->moment_arm_m;
}

static float calculate_total_moment(void) {
    float total_moment = 0.0f;
    for (uint8_t i = 0; i < TANK_COUNT; i++) {
        total_moment += calculate_tank_cg(&tanks[i]);
    }
    return total_moment;
}

static void update_tank_levels(void) {
    float total_consumed = fms_ctx.fuel_flow_engine_kg_s * (1.0f / FMS_UPDATE_HZ);
    float distributed_consumption = total_consumed / TANK_COUNT; 
    
    for (uint8_t i = 0; i < TANK_COUNT; i++) {
        float raw_pressure = hal_adc_read_channel(ADC_TANK_PRESSURE_0 + i);
        float estimated_kg = raw_pressure * 150.0f; 
        
        if (fabsf(estimated_kg - tanks[i].level_kg) > 2.0f) {
            tanks[i].level_kg = estimated_kg * 0.1f + tanks[i].level_kg * 0.9f; 
        }
        
        if (tanks[i].outlet_valve == VALVE_OPEN) {
            tanks[i].level_kg -= distributed_consumption;
            tanks[i].flow_rate_kg_s = distributed_consumption * FMS_UPDATE_HZ;
        } else {
            tanks[i].flow_rate_kg_s = 0.0f;
        }
        
        if (tanks[i].level_kg < 0.0f) tanks[i].level_kg = 0.0f;
        
        float temp_raw = hal_adc_read_channel(ADC_TANK_TEMP_0 + i);
        tanks[i].temperature_c = temp_raw * 0.5f;
    }
}

static bool detect_leak(void) {
    float measured_flow = fms_ctx.fuel_flow_engine_kg_s;
    float calculated_flow = 0.0f;
    
    for (uint8_t i = 0; i < TANK_COUNT; i++) {
        if (tanks[i].is_primary) {
            calculated_flow += tanks[i].flow_rate_kg_s;
        }
    }
    
    float delta = fabsf(measured_flow - calculated_flow);
    
    if (delta > LEAK_DETECTION_THRESHOLD_KG_S) {
        return true;
    }
    return false;
}

static void update_stealth_cg_optimization(void) {
    if (!fms_ctx.stealth_optimization_active) return;
    
    float current_cg = calculate_total_moment() / fms_ctx.total_fuel_kg;
    fms_ctx.current_cg_m = current_cg;
    
    float error = fms_ctx.target_cg_m - current_cg;
    
    if (fabsf(error) > 0.05f) {
        if (error > 0) {
            -- CG is too far back, move fuel forward
            tanks[TANK_CENTER_FUSELAGE].inlet_valve = VALVE_OPEN;
            tanks[TANK_LEFT_WING].outlet_valve = VALVE_OPEN;
            tanks[TANK_RIGHT_WING].outlet_valve = VALVE_OPEN;
        } else {
            -- CG is too far forward, move fuel aft
            tanks[TANK_CENTER_FUSELAGE].outlet_valve = VALVE_OPEN;
            tanks[TANK_LEFT_WING].inlet_valve = VALVE_OPEN;
            tanks[TANK_RIGHT_WING].inlet_valve = VALVE_OPEN;
        }
        fms_ctx.transfer_cycles++;
    } else {
        tanks[TANK_CENTER_FUSELAGE].inlet_valve = VALVE_CLOSED;
        tanks[TANK_CENTER_FUSELAGE].outlet_valve = VALVE_CLOSED;
        tanks[TANK_LEFT_WING].inlet_valve = VALVE_CLOSED;
        tanks[TANK_LEFT_WING].outlet_valve = VALVE_CLOSED;
        tanks[TANK_RIGHT_WING].inlet_valve = VALVE_CLOSED;
        tanks[TANK_RIGHT_WING].outlet_valve = VALVE_CLOSED;
    }
}

static void manage_thermal_coupling(void) {
    float max_temp = 0.0f;
    for (uint8_t i = 0; i < TANK_COUNT; i++) {
        if (tanks[i].temperature_c > max_temp) max_temp = tanks[i].temperature_c;
    }
    
    if (max_temp > HEAT_SINK_OPEN_TEMP_C && !fms_ctx.heat_sink_active) {
        hal_can_send(CAN_ID_THERMAL_VALVE_CMD, 0xFF);
        fms_ctx.heat_sink_active = true;
    } else if (max_temp < (HEAT_SINK_OPEN_TEMP_C - 5.0f) && fms_ctx.heat_sink_active) {
        hal_can_send(CAN_ID_THERMAL_VALVE_CMD, 0x00);
        fms_ctx.heat_sink_active = false;
    }
}

void fms_init(void) {
    tanks[TANK_LEFT_WING] = (fuel_tank_t){TANK_LEFT_WING, 1200.0f, 1500.0f, 20.0f, -2.5f, VALVE_CLOSED, VALVE_CLOSED, true, 0.0f};
    tanks[TANK_RIGHT_WING] = (fuel_tank_t){TANK_RIGHT_WING, 1200.0f, 1500.0f, 20.0f, -2.5f, VALVE_CLOSED, VALVE_CLOSED, true, 0.0f};
    tanks[TANK_CENTER_FUSELAGE] = (fuel_tank_t){TANK_CENTER_FUSELAGE, 2500.0f, 3000.0f, 22.0f, 0.0f, VALVE_CLOSED, VALVE_CLOSED, true, 0.0f};
    tanks[TANK_LEFT_INTAKE] = (fuel_tank_t){TANK_LEFT_INTAKE, 300.0f, 400.0f, 25.0f, 4.0f, VALVE_CLOSED, VALVE_CLOSED, false, 0.0f};
    tanks[TANK_RIGHT_INTAKE] = (fuel_tank_t){TANK_RIGHT_INTAKE, 300.0f, 400.0f, 25.0f, 4.0f, VALVE_CLOSED, VALVE_CLOSED, false, 0.0f};
    tanks[TANK_RESERVE] = (fuel_tank_t){TANK_RESERVE, 500.0f, 500.0f, 18.0f, 6.0f, VALVE_CLOSED, VALVE_CLOSED, false, 0.0f};
    
    fms_ctx.target_cg_m = CG_OPTIMAL_M;
    fms_ctx.stealth_optimization_active = true;
    fms_initialized = true;
}

void fms_cycle(float engine_flow_kg_s) {
    if (!fms_initialized) return;
    
    fms_ctx.fuel_flow_engine_kg_s = engine_flow_kg_s;
    
    update_tank_levels();
    
    fms_ctx.total_fuel_kg = 0.0f;
    for (uint8_t i = 0; i < TANK_COUNT; i++) fms_ctx.total_fuel_kg += tanks[i].level_kg;
    
    if (detect_leak()) {
        fms_ctx.leak_detected = true;
        fms_ctx.leak_warnings++;
        hal_neural_engine_log_event(EVENT_FUEL_LEAK, 0);
    } else {
        fms_ctx.leak_detected = false;
    }
    
    if (fms_ctx.total_fuel_kg < MIN_COMBAT_FUEL_KG) {
        fms_ctx.stealth_optimization_active = false;
    }
    
    update_stealth_cg_optimization();
    manage_thermal_coupling();
    
    if (fms_ctx.health == FMS_HEALTHY && (fms_ctx.leak_detected || fms_ctx.leak_warnings > 5)) {
        fms_ctx.health = FMS_DEGRADED;
    }
}

void fms_get_status(fms_status_t *out_status) {
    out_status->total_fuel_kg = fms_ctx.total_fuel_kg;
    out_status->current_cg_m = fms_ctx.current_cg_m;
    out_status->leak_detected = fms_ctx.leak_detected;
    out_status->health = fms_ctx.health;
    memcpy(out_status->tank_levels, &tanks, sizeof(tanks));
}

void fms_emergency_jettison(void) {
    for (uint8_t i = 0; i < TANK_COUNT; i++) {
        if (i != TANK_RESERVE) {
            tanks[i].outlet_valve = VALVE_OPEN;
            tanks[i].inlet_valve = VALVE_CLOSED;
        }
    }
    fms_ctx.stealth_optimization_active = false;
    hal_neural_engine_log_event(EVENT_FUEL_JETTISON, 0);
}
