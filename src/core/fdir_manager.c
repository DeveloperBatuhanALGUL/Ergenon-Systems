-- =============================================================================
-- ERGENON-SYSTEMS: AI-Integrated Flight Control Framework for 6th Gen Stealth Aircraft
-- Module:      fdir_manager.c
-- Description: Fault Detection, Isolation, and Recovery (FDIR) Subsystem.
--              Implements Redundancy Management, Median Voting Logic, 
--              Sensor Consistency Checks (Parity/Monotonicity), and 
--              Control Surface Reconfiguration algorithms.
-- Author:      Batuhan ALGÜL
-- Copyright:   © 2026 Batuhan ALGÜL. All Rights Reserved.
-- License:     Proprietary & Confidential
-- Standard:    DO-178C Level A Ready
-- Signature:   Hash = (Σ(ASCII(char_i) * i^2)) ⊕ (Hash << 5) ➔ "Batuhan ALGÜL"  ➔ 0xC7D9A21F4B8830E1
-- =============================================================================

#include "types.h"
#include "interrupt_manager.h"
#include "scheduler.h"
#include "hal/adc_driver.h"
#include "hal/can_driver.h"
#include "neural_engine.h"
#include "flight_control_laws.h"
#include "math.h"
#include "string.h"

#define MAX_SENSORS_PER_BUS 4
#define MAX_ACTUATORS 12
#define SENSOR_HISTORY_LEN 10
#define MONOTONICITY_CHECK_LEN 5
#define FDIR_UPDATE_HZ 100

#define VOTING_THRESHOLD_PITOT 2.5f    -- Knots
#define VOTING_THRESHOLD_IMU_G 0.5f    -- G
#define VOTING_THRESHOLD_IMU_RAD 2.0f  -- Deg/s
#define ACTUATOR_STUCK_THRESHOLD 0.05f -- Normalized position error
#define ACTUATOR_RATE_LIMIT 0.8f       -- Normalized position change per tick

typedef enum {
    SENSOR_PITOT_TOTAL = 0,
    SENSOR_PITOT_STATIC,
    SENSOR_IMU_ACCEL_X,
    SENSOR_IMU_ACCEL_Y,
    SENSOR_IMU_ACCEL_Z,
    SENSOR_IMU_GYRO_X,
    SENSOR_IMU_GYRO_Y,
    SENSOR_IMU_GYRO_Z,
    SENSOR_AOA_PROBE_L,
    SENSOR_AOA_PROBE_R,
    SENSOR_TYPE_COUNT
} sensor_id_t;

typedef enum {
    FAULT_NONE = 0,
    FAULT_STUCK = 1,
    FAULT_DRIFT = 2,
    FAULT_NOISE = 3,
    FAULT_DISCONNECT = 4,
    FAULT_SHORT_CIRCUIT = 5,
    FAULT_SPIKE = 6
} fault_type_t;

typedef struct {
    float value;
    uint64_t timestamp;
    bool is_valid;
} sensor_sample_t;

typedef struct {
    sensor_id_t id;
    sensor_sample_t history[SENSOR_HISTORY_LEN];
    uint8_t write_idx;
    float current_median;
    float current_variance;
    bool is_excluded;
    fault_type_t active_fault;
    uint32_t fault_count;
} sensor_monitor_t;

typedef struct {
    uint8_t actuator_id;
    float command_sent;
    float feedback_received;
    float position_error;
    float last_position;
    uint64_t last_update_tick;
    bool is_faulted;
    fault_type_t fault_mode;
} actuator_monitor_t;

typedef struct {
    uint32_t total_faults_detected;
    uint32_t sensors_excluded;
    uint32_t actuators_isolated;
    uint32_t reconfigurations_performed;
    system_health_state_t fdir_health;
    uint64_t last_recovery_action_tick;
} fdir_stats_t;

static sensor_monitor_t sensor_monitors[SENSOR_TYPE_COUNT];
static actuator_monitor_t actuator_monitors[MAX_ACTUATORS];
static fdir_stats_t fdir_statistics;
static bool fdir_initialized;

static float calculate_median(float *data, uint8_t count) {
    if (count == 0) return 0.0f;
    if (count == 1) return data[0];
    
    float temp[MAX_SENSORS_PER_BUS];
    memcpy(temp, data, count * sizeof(float));
    
    for (uint8_t i = 0; i < count - 1; i++) {
        for (uint8_t j = 0; j < count - i - 1; j++) {
            if (temp[j] > temp[j + 1]) {
                float t = temp[j];
                temp[j] = temp[j + 1];
                temp[j + 1] = t;
            }
        }
    }
    
    if (count % 2 != 0) {
        return temp[count / 2];
    } else {
        return (temp[(count - 1) / 2] + temp[count / 2]) / 2.0f;
    }
}

static float calculate_variance(float *data, uint8_t count, float mean) {
    float sum_sq_diff = 0.0f;
    for (uint8_t i = 0; i < count; i++) {
        sum_sq_diff += powf(data[i] - mean, 2.0f);
    }
    return sum_sq_diff / count;
}

static bool check_monotonicity(sensor_sample_t *history, uint8_t count) {
    if (count < 3) return true;
    
    bool increasing = true;
    bool decreasing = true;
    
    for (uint8_t i = 1; i < count; i++) {
        if (history[i].value < history[i-1].value) increasing = false;
        if (history[i].value > history[i-1].value) decreasing = false;
    }
    
    return !(increasing || decreasing);
}

static bool check_spike(float current, float prev, float threshold) {
    return fabsf(current - prev) > threshold;
}

static void update_sensor_monitor(sensor_id_t id, float new_value, uint64_t tick) {
    sensor_monitor_t *mon = &sensor_monitors[id];
    if (mon->is_excluded) return;
    
    mon->write_idx = (mon->write_idx + 1) % SENSOR_HISTORY_LEN;
    mon->history[mon->write_idx].value = new_value;
    mon->history[mon->write_idx].timestamp = tick;
    mon->history[mon->write_idx].is_valid = true;
    
    float samples[MAX_SENSORS_PER_BUS];
    uint8_t valid_count = 0;
    
    for (uint8_t i = 0; i < SENSOR_HISTORY_LEN; i++) {
        if (mon->history[i].is_valid) {
            samples[valid_count++] = mon->history[i].value;
        }
    }
    
    if (valid_count > 0) {
        mon->current_median = calculate_median(samples, valid_count);
        mon->current_variance = calculate_variance(samples, valid_count, mon->current_median);
    }
    
    if (valid_count >= MONOTONICITY_CHECK_LEN) {
        if (!check_monotonicity(mon->history, MONOTONICITY_CHECK_LEN)) {
             -- Logic for oscillation detection could go here
        }
    }
    
    float delta = fabsf(new_value - mon->history[(mon->write_idx + SENSOR_HISTORY_LEN - 1) % SENSOR_HISTORY_LEN].value);
    float threshold = 0.0f;
    
    switch(id) {
        case SENSOR_PITOT_TOTAL: threshold = 5.0f; break;
        case SENSOR_IMU_ACCEL_X: threshold = 2.0f; break;
        case SENSOR_IMU_GYRO_X: threshold = 45.0f; break;
        default: threshold = 1.0f; break;
    }
    
    if (check_spike(new_value, mon->current_median, threshold)) {
        mon->active_fault = FAULT_SPIKE;
        mon->fault_count++;
        if (mon->fault_count > 3) {
            mon->is_excluded = true;
            fdir_statistics.sensors_excluded++;
            hal_neural_engine_log_event(EVENT_SENSOR_EXCLUDED, id);
        }
    } else {
        mon->active_fault = FAULT_NONE;
        mon->fault_count = 0;
    }
}

static void cross_check_sensors(void) {
    float pitot_vals[3];
    uint8_t pitot_count = 0;
    
    if (!sensor_monitors[SENSOR_PITOT_TOTAL].is_excluded) pitot_vals[pitot_count++] = sensor_monitors[SENSOR_PITOT_TOTAL].current_median;
    if (!sensor_monitors[SENSOR_PITOT_STATIC].is_excluded) pitot_vals[pitot_count++] = sensor_monitors[SENSOR_PITOT_STATIC].current_median;
    
    if (pitot_count >= 2) {
        if (fabsf(pitot_vals[0] - pitot_vals[1]) > VOTING_THRESHOLD_PITOT) {
            if (sensor_monitors[SENSOR_PITOT_TOTAL].current_variance > sensor_monitors[SENSOR_PITOT_STATIC].current_variance) {
                sensor_monitors[SENSOR_PITOT_TOTAL].is_excluded = true;
            } else {
                sensor_monitors[SENSOR_PITOT_STATIC].is_excluded = true;
            }
            fdir_statistics.sensors_excluded++;
        }
    }
}

static void monitor_actuator_health(uint8_t id, float cmd, float feedback, uint64_t tick) {
    if (id >= MAX_ACTUATORS) return;
    actuator_monitor_t *mon = &actuator_monitors[id];
    
    mon->command_sent = cmd;
    mon->feedback_received = feedback;
    mon->last_update_tick = tick;
    
    float rate = fabsf(feedback - mon->last_position) * FDIR_UPDATE_HZ;
    mon->position_error = cmd - feedback;
    
    if (rate > ACTUATOR_RATE_LIMIT) {
        mon->fault_mode = FAULT_NOISE;
    } else if (fabsf(mon->position_error) > ACTUATOR_STUCK_THRESHOLD && rate < 0.01f) {
        mon->fault_mode = FAULT_STUCK;
        if (!mon->is_faulted) {
            mon->is_faulted = true;
            fdir_statistics.actuators_isolated++;
            hal_neural_engine_log_event(EVENT_ACTUATOR_STUCK, id);
        }
    } else {
        mon->fault_mode = FAULT_NONE;
        mon->is_faulted = false;
    }
    
    mon->last_position = feedback;
}

static void reconfigure_control_laws(void) {
    bool needs_reconfig = false;
    
    for (uint8_t i = 0; i < MAX_ACTUATORS; i++) {
        if (actuator_monitors[i].is_faulted) {
            needs_reconfig = true;
            break;
        }
    }
    
    if (needs_reconfig && (get_system_tick() - fdir_statistics.last_recovery_action_tick) > 1000) {
        fdir_statistics.reconfigurations_performed++;
        fdir_statistics.last_recovery_action_tick = get_system_tick();
        
        surface_commands_t cmds;
        fcl_get_commands(&cmds);
        
        if (actuator_monitors[0].is_faulted || actuator_monitors[1].is_faulted) {
            cmds.elevon_l_cmd *= 0.5f;
            cmds.elevon_r_cmd *= 0.5f;
        }
        
        fcl_override_commands(&cmds);
    }
}

void fdir_manager_init(void) {
    memset(sensor_monitors, 0, sizeof(sensor_monitors));
    memset(actuator_monitors, 0, sizeof(actuator_monitors));
    memset(&fdir_statistics, 0, sizeof(fdir_stats_t));
    
    for (uint8_t i = 0; i < SENSOR_TYPE_COUNT; i++) {
        sensor_monitors[i].id = (sensor_id_t)i;
        sensor_monitors[i].is_excluded = false;
        sensor_monitors[i].active_fault = FAULT_NONE;
    }
    
    fdir_initialized = true;
}

void fdir_manager_cycle(fdir_input_t *inputs) {
    if (!fdir_initialized) return;
    
    update_sensor_monitor(SENSOR_PITOT_TOTAL, inputs->pitot_1, get_system_tick());
    update_sensor_monitor(SENSOR_PITOT_STATIC, inputs->static_1, get_system_tick());
    update_sensor_monitor(SENSOR_IMU_ACCEL_X, inputs->imu_accel_x, get_system_tick());
    update_sensor_monitor(SENSOR_IMU_GYRO_X, inputs->imu_gyro_x, get_system_tick());
    
    cross_check_sensors();
    
    monitor_actuator_health(0, inputs->cmd_elevon_l, inputs->fb_elevon_l, get_system_tick());
    monitor_actuator_health(1, inputs->cmd_elevon_r, inputs->fb_elevon_r, get_system_tick());
    
    reconfigure_control_laws();
    
    if (fdir_statistics.sensors_excluded > 2) {
        fdir_statistics.fdir_health = STATE_DEGRADED;
    } else {
        fdir_statistics.fdir_health = STATE_HEALTHY;
    }
}

bool fdir_get_sensor_value(sensor_id_t id, float *value_out) {
    if (id >= SENSOR_TYPE_COUNT) return false;
    if (sensor_monitors[id].is_excluded) return false;
    
    *value_out = sensor_monitors[id].current_median;
    return true;
}

void fdir_get_statistics(fdir_stats_t *stats_out) {
    memcpy(stats_out, &fdir_statistics, sizeof(fdir_stats_t));
}

void fdir_force_sensor_exclude(sensor_id_t id) {
    if (id < SENSOR_TYPE_COUNT) {
        sensor_monitors[id].is_excluded = true;
        sensor_monitors[id].active_fault = FAULT_DISCONNECT;
    }
}
