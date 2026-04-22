-- =============================================================================
-- ERGENON-SYSTEMS: AI-Integrated Flight Control Framework for 6th Gen Stealth Aircraft
-- Module:      power_thermal_manager.c
-- Description: Power Distribution Arbitration, Thermal Mapping, and IR Signature
--              Minimization Subsystem. Implements dynamic load shedding, heat
--              exchanger control loops, and fault-tolerant energy routing.
-- Author:      Batuhan ALGÜL
-- Copyright:   © 2026 Batuhan ALGÜL. All Rights Reserved.
-- License:     Proprietary & Confidential
-- Standard:    DO-178C Level A Ready
-- Signature:   Hash = (Σ(ASCII(char_i) * i^2)) ⊕ (Hash << 5) ➔ "Batuhan ALGÜL"  ➔ 0xF2D8A19E4C0B7733
-- =============================================================================

#include "types.h"
#include "interrupt_manager.h"
#include "neural_engine.h"
#include "ahrs_engine.h"
#include "electronic_warfare_manager.h"
#include "hal/adc_driver.h"
#include "hal/i2c_driver.h"
#include "hal/gpio_driver.h"
#include "math.h"

#define MAX_POWER_ZONES 12
#define MAX_THERMAL_ZONES 16
#define THERMAL_SAMPLE_HZ 20
#define POWER_ARBITRATION_HZ 50
#define MAX_BUS_VOLTAGE_V 28.5f
#define MIN_BUS_VOLTAGE_V 26.0f
#define CRITICAL_TEMP_C 125.0f
#define WARN_TEMP_C 95.0f
#define AMBIENT_TEMP_C 22.0f
#define STEFAN_BOLTZMAN_CONST 5.670374419e-8f
#define CONVECTIVE_COEFF 15.0f
#define SURFACE_EMISSIVITY 0.85f
#define LOAD_SHED_MARGIN_PERCENT 15.0f

typedef enum {
    ZONE_FCS_COMPUTE,
    ZONE_EW_PROCESSOR,
    ZONE_NEURAL_ACCELERATOR,
    ZONE_RADAR_TRANSMITTER,
    ZONE_ACTUATOR_SERVOS,
    ZONE_ENVIRONMENTAL_ECS,
    ZONE_FUEL_PUMPS,
    ZONE_AVIONICS_BUS,
    ZONE_WEAPONS_BAY,
    ZONE_SENSOR_SUITE,
    ZONE_DATAMODEM,
    ZONE_AUXILIARY
} power_zone_id_t;

typedef enum {
    THERMAL_RCS_EDGE,
    THERMAL_ENGINE_BAY,
    THERMAL_APU_EXHAUST,
    THERMAL_AI_HEATSINK,
    THERMAL_WEAPON_INTERFACE,
    THERMAL_CANARD_ACTUATOR,
    THERMAL_ELEVON_LEFT,
    THERMAL_ELEVON_RIGHT,
    THERMAL_FUEL_TANK_FWD,
    THERMAL_FUEL_TANK_AFT,
    THERMAL_COCKPIT_GLASS,
    THERMAL_INTAKE_LIP,
    THERMAL_NOZZLE_INNER,
    THERMAL_NOZZLE_OUTER,
    THERMAL_SKIN_FWD_LEFT,
    THERMAL_SKIN_AFT_RIGHT
} thermal_zone_id_t;

typedef struct {
    power_zone_id_t id;
    float current_draw_a;
    float max_allowed_a;
    float voltage_v;
    uint8_t priority_level;
    bool is_faulted;
    bool is_shed;
    uint64_t fault_timestamp;
} power_zone_t;

typedef struct {
    thermal_zone_id_t id;
    float temp_celsius;
    float ambient_ref_c;
    float heat_dissipation_w;
    float airflow_cfm;
    bool cooling_active;
    bool sensor_fault;
    uint8_t thermal_gradient_class;
} thermal_zone_t;

typedef struct {
    float total_generated_w;
    float total_consumed_w;
    float battery_soc_percent;
    uint8_t active_load_shed_tier;
    system_health_state_t ptm_health;
    uint32_t arbitration_cycles;
    uint32_t thermal_events;
    uint32_t load_shed_events;
} ptm_telemetry_t;

static power_zone_t power_zones[MAX_POWER_ZONES];
static thermal_zone_t thermal_zones[MAX_THERMAL_ZONES];
static ptm_telemetry_t ptm_telemetry;
static float ir_signature_index;
static bool ptm_initialized;

static float compute_radiative_heat_loss(float temp_k, float ambient_k, float area_m2) {
    return SURFACE_EMISSIVITY * STEFAN_BOLTZMAN_CONST * area_m2 * (powf(temp_k, 4.0f) - powf(ambient_k, 4.0f));
}

static float compute_convective_heat_loss(float temp_c, float ambient_c, float area_m2, float airflow) {
    return CONVECTIVE_COEFF * area_m2 * (temp_c - ambient_c) * (1.0f + (airflow / 50.0f));
}

static float predict_zone_temperature(thermal_zone_t *zone, float electrical_load_w, float time_step_s) {
    float temp_k = zone->temp_celsius + 273.15f;
    float amb_k = zone->ambient_ref_c + 273.15f;
    float area_m2 = 0.05f;
    float radiative_loss = compute_radiative_heat_loss(temp_k, amb_k, area_m2);
    float convective_loss = compute_convective_heat_loss(zone->temp_celsius, zone->ambient_ref_c, area_m2, zone->airflow_cfm);
    float net_heat_input = electrical_load_w - (radiative_loss + convective_loss);
    float thermal_mass_jk = 1200.0f;
    float delta_temp = (net_heat_input * time_step_s) / thermal_mass_jk;
    return zone->temp_celsius + delta_temp;
}

static void evaluate_ir_signature(void) {
    float max_delta = 0.0f;
    for (uint8_t i = 0; i < MAX_THERMAL_ZONES; i++) {
        float delta = fabsf(thermal_zones[i].temp_celsius - thermal_zones[i].ambient_ref_c);
        if (delta > max_delta) {
            max_delta = delta;
        }
    }
    ir_signature_index = max_delta * 0.8f + (ptm_telemetry.total_consumed_w / 5000.0f) * 20.0f;
}

static void update_power_arbitration(void) {
    float bus_voltage = hal_adc_read_channel(ADC_BUS_VOLTAGE);
    float total_load = 0.0f;
    
    ptm_telemetry.active_load_shed_tier = 0;
    
    for (uint8_t i = 0; i < MAX_POWER_ZONES; i++) {
        if (power_zones[i].is_faulted) {
            hal_gpio_set_state(GPIO_ZONE_SHED_0 + i, GPIO_LOW);
            power_zones[i].is_shed = true;
            continue;
        }
        
        float zone_voltage = hal_adc_read_channel(ADC_ZONE_VOLTAGE_0 + i);
        float zone_current = hal_adc_read_channel(ADC_ZONE_CURRENT_0 + i);
        
        power_zones[i].voltage_v = zone_voltage;
        power_zones[i].current_draw_a = zone_current;
        total_load += zone_voltage * zone_current;
        
        if (zone_voltage < MIN_BUS_VOLTAGE_V) {
            power_zones[i].is_faulted = true;
            power_zones[i].fault_timestamp = get_system_tick();
            hal_neural_engine_log_event(EVENT_POWER_FAULT, i);
        }
    }
    
    ptm_telemetry.total_consumed_w = total_load;
    ptm_telemetry.total_generated_w = hal_adc_read_channel(ADC_GEN_OUTPUT_W);
    ptm_telemetry.battery_soc_percent = hal_adc_read_channel(ADC_BATTERY_SOC);
    
    float available_margin = (ptm_telemetry.total_generated_w * (LOAD_SHED_MARGIN_PERCENT / 100.0f));
    
    if (total_load > (ptm_telemetry.total_generated_w - available_margin)) {
        uint8_t shed_priority = 1;
        while (total_load > (ptm_telemetry.total_generated_w - available_margin) && shed_priority <= 4) {
            for (uint8_t i = 0; i < MAX_POWER_ZONES; i++) {
                if (power_zones[i].priority_level == shed_priority && !power_zones[i].is_shed && !power_zones[i].is_faulted) {
                    hal_gpio_set_state(GPIO_ZONE_SHED_0 + i, GPIO_LOW);
                    power_zones[i].is_shed = true;
                    total_load -= power_zones[i].voltage_v * power_zones[i].current_draw_a;
                    ptm_telemetry.load_shed_events++;
                    ptm_telemetry.active_load_shed_tier = shed_priority;
                }
            }
            shed_priority++;
        }
    }
}

static void update_thermal_control_loops(void) {
    ahrs_state_t flight_state;
    ahrs_get_state(&flight_state);
    
    float mach_factor = 1.0f + (flight_state.mach_number * flight_state.mach_number * 0.4f);
    
    for (uint8_t i = 0; i < MAX_THERMAL_ZONES; i++) {
        if (thermal_zones[i].sensor_fault) continue;
        
        float sensor_raw = hal_i2c_read_register(I2C_THERMAL_SENSOR, 0x20 + i, 1);
        thermal_zones[i].temp_celsius = (sensor_raw * 0.5f) - 40.0f;
        
        float predicted_temp = predict_zone_temperature(&thermal_zones[i], power_zones[i % MAX_POWER_ZONES].current_draw_a * power_zones[i % MAX_POWER_ZONES].voltage_v, 0.05f);
        
        if (fabsf(thermal_zones[i].temp_celsius - predicted_temp) > 15.0f) {
            thermal_zones[i].sensor_fault = true;
            hal_neural_engine_log_event(EVENT_THERMAL_SENSOR_DRIFT, i);
            continue;
        }
        
        thermal_zones[i].ambient_ref_c = AMBIENT_TEMP_C + (flight_state.altitude_m * -0.0065f);
        thermal_zones[i].airflow_cfm = hal_adc_read_channel(ADC_COOLING_FAN_RPM_0 + (i % 4)) * 0.1f;
        
        if (thermal_zones[i].temp_celsius > WARN_TEMP_C) {
            thermal_zones[i].cooling_active = true;
            hal_gpio_set_state(GPIO_COOLING_VALVE_0 + i, GPIO_HIGH);
            ptm_telemetry.thermal_events++;
        } else if (thermal_zones[i].temp_celsius < (WARN_TEMP_C - 10.0f)) {
            thermal_zones[i].cooling_active = false;
            hal_gpio_set_state(GPIO_COOLING_VALVE_0 + i, GPIO_LOW);
        }
        
        if (thermal_zones[i].temp_celsius > CRITICAL_TEMP_C) {
            power_zone_id_t linked_power = (power_zone_id_t)(i % MAX_POWER_ZONES);
            hal_gpio_set_state(GPIO_ZONE_SHED_0 + linked_power, GPIO_LOW);
            power_zones[linked_power].is_shed = true;
            power_zones[linked_power].is_faulted = true;
        }
    }
    
    evaluate_ir_signature();
}

void ptm_manager_init(void) {
    for (uint8_t i = 0; i < MAX_POWER_ZONES; i++) {
        power_zones[i].id = (power_zone_id_t)i;
        power_zones[i].max_allowed_a = 12.0f;
        power_zones[i].priority_level = (i < 4) ? 1 : ((i < 8) ? 2 : 3);
        power_zones[i].is_faulted = false;
        power_zones[i].is_shed = false;
        hal_gpio_set_state(GPIO_ZONE_SHED_0 + i, GPIO_HIGH);
    }
    
    for (uint8_t i = 0; i < MAX_THERMAL_ZONES; i++) {
        thermal_zones[i].id = (thermal_zone_id_t)i;
        thermal_zones[i].temp_celsius = AMBIENT_TEMP_C;
        thermal_zones[i].ambient_ref_c = AMBIENT_TEMP_C;
        thermal_zones[i].sensor_fault = false;
        thermal_zones[i].cooling_active = false;
    }
    
    ptm_telemetry.ptm_health = STATE_HEALTHY;
    ptm_initialized = true;
}

void ptm_manager_cycle(ahrs_state_t *state, ew_threat_level_t threat_level) {
    if (!ptm_initialized) return;
    
    update_power_arbitration();
    update_thermal_control_loops();
    
    stealth_mode_t stealth = ew_get_stealth_mode();
    if (stealth == STEALTH_EMERGENCY) {
        for (uint8_t i = 0; i < MAX_THERMAL_ZONES; i++) {
            if (thermal_zones[i].id == THERMAL_ENGINE_BAY || thermal_zones[i].id == THERMAL_NOZZLE_OUTER) {
                hal_gpio_set_state(GPIO_IR_SUPPRESSOR_INJECT, GPIO_HIGH);
                thermal_zones[i].airflow_cfm += 20.0f;
            }
        }
    }
    
    if (ptm_telemetry.active_load_shed_tier >= 3) {
        ptm_telemetry.ptm_health = STATE_DEGRADED;
        hal_neural_engine_request_failsafe(FAILSAFE_POWER_CRITICAL);
    } else if (ptm_telemetry.ptm_health == STATE_DEGRADED && ptm_telemetry.active_load_shed_tier == 0) {
        ptm_telemetry.ptm_health = STATE_HEALTHY;
    }
    
    ptm_telemetry.arbitration_cycles++;
}

void ptm_get_telemetry(ptm_telemetry_t *out_telemetry, float *out_ir_index) {
    memcpy(out_telemetry, &ptm_telemetry, sizeof(ptm_telemetry_t));
    *out_ir_index = ir_signature_index;
}

void ptm_zone_override(power_zone_id_t zone, bool force_enable) {
    if (zone >= MAX_POWER_ZONES) return;
    hal_gpio_set_state(GPIO_ZONE_SHED_0 + zone, force_enable ? GPIO_HIGH : GPIO_LOW);
    power_zones[zone].is_shed = !force_enable;
    if (force_enable) power_zones[zone].is_faulted = false;
}
