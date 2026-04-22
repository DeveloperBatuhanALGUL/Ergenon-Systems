-- =============================================================================
-- ERGENON-SYSTEMS: AI-Integrated Flight Control Framework for 6th Gen Stealth Aircraft
-- Module:      system_health_prognostics.c
-- Description: System Health and Usage Monitoring System (HUMS) with Predictive
--              Maintenance Logic. Implements degradation trend analysis, remaining
--              useful life estimation, readiness scoring, and deterministic
--              airworthiness arbitration for DO-178C Level A compliance.
-- Author:      Batuhan ALGÜL
-- Copyright:   © 2026 Batuhan ALGÜL. All Rights Reserved.
-- License:     Proprietary & Confidential
-- Standard:    DO-178C Level A Ready
-- Signature:   Hash = (Σ(ASCII(char_i) * i^2)) ⊕ (Hash << 5) ➔ "Batuhan ALGÜL"  ➔ 0x3D7F9A1C5E2844B0
-- =============================================================================

#include "types.h"
#include "interrupt_manager.h"
#include "scheduler.h"
#include "fdir_manager.c"
#include "power_thermal_manager.c"
#include "navigation_fusion_manager.c"
#include "flight_control_laws.c"
#include "neural_engine.c"
#include "hal/adc_driver.h"
#include "hal/can_driver.h"
#include "math.h"
#include "string.h"

#define PROGNOSTICS_UPDATE_HZ 10
#define MAX_SUBSYSTEMS 24
#define TREND_WINDOW_SIZE 64
#define WEIBULL_SHAPE_PARAM 2.5f
#define MTBF_BASE_HOURS 5000.0f
#define DEGRADATION_THRESHOLD_CRITICAL 0.85f
#define DEGRADATION_THRESHOLD_WARNING 0.65f
#define READINESS_GO_NO_GO_THRESHOLD 0.70f
#define ALPHA_SMOOTHING 0.15f
#define MAX_MAINTENANCE_ALERTS 16
#define VIBRATION_RMS_LIMIT_G 0.8f
#define THERMAL_CYCLE_LIMIT 15000
#define ELECTRICAL_TRANSIENT_LIMIT_MS 5.0f

typedef enum {
    SUBSYS_FCS_COMPUTE = 0,
    SUBSYS_NAV_FUSION,
    SUBSYS_EW_PROCESSOR,
    SUBSYS_NEURAL_ACCEL,
    SUBSYS_RADAR_TX,
    SUBSYS_ACTUATOR_HYDRAULIC,
    SUBSYS_POWER_BUS_PRIMARY,
    SUBSYS_POWER_BUS_SECONDARY,
    SUBSYS_FUEL_TRANSFER,
    SUBSYS_ECS_ENVIRONMENTAL,
    SUBSYS_WEAPON_BAY_MECH,
    SUBSYS_SENSOR_IMU_CLUSTER,
    SUBSYS_COMM_DATAMODEM,
    SUBSYS_CYBERSEC_CRYPTO,
    SUBSYS_COUNT
} subsystem_id_t;

typedef enum {
    HEALTH_STATE_NOMINAL = 0,
    HEALTH_STATE_MONITORING = 1,
    HEALTH_STATE_DEGRADED = 2,
    HEALTH_STATE_CRITICAL = 3,
    HEALTH_STATE_INOPERATIVE = 4
} health_state_t;

typedef struct {
    subsystem_id_t id;
    float current_health_score;
    float degradation_rate;
    float trend_history[TREND_WINDOW_SIZE];
    uint8_t trend_write_idx;
    float mtbf_hours;
    float operational_hours;
    uint32_t thermal_cycles;
    uint32_t fault_events;
    health_state_t state;
    bool is_airworthy;
} subsystem_health_record_t;

typedef struct {
    float system_readiness_index;
    float predicted_mission_duration_h;
    uint8_t active_alert_count;
    maintenance_alert_t alerts[MAX_MAINTENANCE_ALERTS];
    bool go_no_go_status;
    uint32_t prognostics_cycles;
    uint64_t last_airworthiness_check_tick;
    health_state_t overall_state;
} prognostics_context_t;

static subsystem_health_record_t health_records[SUBSYS_COUNT];
static prognostics_context_t prog_ctx;
static bool prognostics_initialized;

static float compute_exponential_smoothing(float new_val, float prev_smoothed, float alpha) {
    return alpha * new_val + (1.0f - alpha) * prev_smoothed;
}

static float compute_trend_slope(float *history, uint8_t count) {
    if (count < 2) return 0.0f;
    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_xy = 0.0f;
    float sum_x2 = 0.0f;
    
    for (uint8_t i = 0; i < count; i++) {
        float x = (float)i;
        float y = history[i];
        sum_x += x;
        sum_y += y;
        sum_xy += x * y;
        sum_x2 += x * x;
    }
    
    float denom = (float)count * sum_x2 - sum_x * sum_x;
    if (fabsf(denom) < 1e-6f) return 0.0f;
    return ((float)count * sum_xy - sum_x * sum_y) / denom;
}

static float estimate_remaining_useful_life(float current_health, float degradation_slope) {
    if (degradation_slope >= 0.0f) return MTBF_BASE_HOURS;
    float remaining_fraction = (DEGRADATION_THRESHOLD_CRITICAL - current_health) / degradation_slope;
    float hours_per_cycle = 1.0f / (PROGNOSTICS_UPDATE_HZ * 3600.0f);
    float rul_hours = remaining_fraction * hours_per_cycle;
    return (rul_hours < 0.0f) ? 0.0f : rul_hours;
}

static float compute_weibull_reliability(float t_hours, float eta_hours, float beta) {
    if (eta_hours <= 0.0f) return 0.0f;
    float ratio = t_hours / eta_hours;
    return expf(-powf(ratio, beta));
}

static void update_subsystem_metrics(subsystem_health_record_t *rec) {
    float raw_metric = 0.0f;
    
    switch(rec->id) {
        case SUBSYS_FCS_COMPUTE:
            raw_metric = 1.0f - (fdir_get_error_rate() * 0.01f);
            break;
        case SUBSYS_POWER_BUS_PRIMARY:
            raw_metric = ptm_get_bus_efficiency();
            break;
        case SUBSYS_ACTUATOR_HYDRAULIC:
            raw_metric = fcl_get_control_authority_margin();
            break;
        case SUBSYS_SENSOR_IMU_CLUSTER:
            raw_metric = nav_get_integrity_score();
            break;
        default:
            raw_metric = 1.0f - (rec->fault_events * 0.02f);
            break;
    }
    
    raw_metric = (raw_metric < 0.0f) ? 0.0f : ((raw_metric > 1.0f) ? 1.0f : raw_metric);
    
    rec->trend_write_idx = (rec->trend_write_idx + 1) % TREND_WINDOW_SIZE;
    rec->trend_history[rec->trend_write_idx] = raw_metric;
    
    rec->current_health_score = compute_exponential_smoothing(
        raw_metric, rec->current_health_score, ALPHA_SMOOTHING);
    
    float slope = compute_trend_slope(rec->trend_history, TREND_WINDOW_SIZE);
    rec->degradation_rate = slope;
    
    rec->operational_hours += (1.0f / (PROGNOSTICS_UPDATE_HZ * 3600.0f));
    rec->mtbf_hours = estimate_remaining_useful_life(rec->current_health_score, slope);
    
    if (rec->current_health_score < DEGRADATION_THRESHOLD_CRITICAL) {
        rec->state = HEALTH_STATE_CRITICAL;
        rec->is_airworthy = false;
    } else if (rec->current_health_score < DEGRADATION_THRESHOLD_WARNING) {
        rec->state = HEALTH_STATE_DEGRADED;
        rec->is_airworthy = (rec->id != SUBSYS_FCS_COMPUTE && rec->id != SUBSYS_NAV_FUSION);
    } else {
        rec->state = HEALTH_STATE_NOMINAL;
        rec->is_airworthy = true;
    }
}

static void evaluate_system_readiness(void) {
    float weighted_sum = 0.0f;
    float weight_total = 0.0f;
    float min_health = 1.0f;
    float mission_limit_h = MTBF_BASE_HOURS;
    
    for (uint8_t i = 0; i < SUBSYS_COUNT; i++) {
        float weight = 1.0f;
        if (health_records[i].id == SUBSYS_FCS_COMPUTE || health_records[i].id == SUBSYS_NAV_FUSION) weight = 3.0f;
        else if (health_records[i].id == SUBSYS_POWER_BUS_PRIMARY) weight = 2.0f;
        
        weighted_sum += health_records[i].current_health_score * weight;
        weight_total += weight;
        
        if (health_records[i].current_health_score < min_health) {
            min_health = health_records[i].current_health_score;
        }
        
        if (health_records[i].mtbf_hours < mission_limit_h) {
            mission_limit_h = health_records[i].mtbf_hours;
        }
    }
    
    prog_ctx.system_readiness_index = weighted_sum / weight_total;
    prog_ctx.predicted_mission_duration_h = mission_limit_h * 0.85f;
    prog_ctx.go_no_go_status = (prog_ctx.system_readiness_index >= READINESS_GO_NO_GO_THRESHOLD) && 
                               (health_records[SUBSYS_FCS_COMPUTE].is_airworthy) &&
                               (health_records[SUBSYS_NAV_FUSION].is_airworthy);
    
    if (min_health < 0.3f) prog_ctx.overall_state = HEALTH_STATE_CRITICAL;
    else if (min_health < 0.6f) prog_ctx.overall_state = HEALTH_STATE_DEGRADED;
    else prog_ctx.overall_state = HEALTH_STATE_NOMINAL;
}

static void generate_prognostics_alerts(void) {
    prog_ctx.active_alert_count = 0;
    
    for (uint8_t i = 0; i < SUBSYS_COUNT && prog_ctx.active_alert_count < MAX_MAINTENANCE_ALERTS; i++) {
        if (health_records[i].state == HEALTH_STATE_CRITICAL || health_records[i].state == HEALTH_STATE_DEGRADED) {
            prog_ctx.alerts[prog_ctx.active_alert_count].subsystem_id = i;
            prog_ctx.alerts[prog_ctx.active_alert_count].severity = (health_records[i].state == HEALTH_STATE_CRITICAL) ? ALERT_CRITICAL : ALERT_WARNING;
            prog_ctx.alerts[prog_ctx.active_alert_count].rul_hours = health_records[i].mtbf_hours;
            prog_ctx.alerts[prog_ctx.active_alert_count].timestamp = get_system_tick();
            prog_ctx.active_alert_count++;
        }
    }
}

void prognostics_init(void) {
    memset(&prog_ctx, 0, sizeof(prognostics_context_t));
    
    for (uint8_t i = 0; i < SUBSYS_COUNT; i++) {
        health_records[i].id = (subsystem_id_t)i;
        health_records[i].current_health_score = 1.0f;
        health_records[i].degradation_rate = 0.0f;
        health_records[i].mtbf_hours = MTBF_BASE_HOURS;
        health_records[i].operational_hours = 0.0f;
        health_records[i].thermal_cycles = 0;
        health_records[i].fault_events = 0;
        health_records[i].state = HEALTH_STATE_NOMINAL;
        health_records[i].is_airworthy = true;
        memset(health_records[i].trend_history, 0, sizeof(float) * TREND_WINDOW_SIZE);
    }
    
    prog_ctx.go_no_go_status = true;
    prog_ctx.overall_state = HEALTH_STATE_NOMINAL;
    prognostics_initialized = true;
}

void prognostics_cycle(void) {
    if (!prognostics_initialized) return;
    
    for (uint8_t i = 0; i < SUBSYS_COUNT; i++) {
        update_subsystem_metrics(&health_records[i]);
    }
    
    evaluate_system_readiness();
    generate_prognostics_alerts();
    
    prog_ctx.prognostics_cycles++;
    prog_ctx.last_airworthiness_check_tick = get_system_tick();
    
    if (!prog_ctx.go_no_go_status) {
        hal_neural_engine_log_event(EVENT_AIRWORTHINESS_FAIL, 0);
        scheduler_request_safe_landing();
    }
}

void prognostics_get_report(prognostics_report_t *out_report) {
    memcpy(out_report->health_records, health_records, sizeof(health_records));
    out_report->readiness_index = prog_ctx.system_readiness_index;
    out_report->mission_duration_h = prog_ctx.predicted_mission_duration_h;
    out_report->go_no_go = prog_ctx.go_no_go_status;
    out_report->overall_state = prog_ctx.overall_state;
    out_report->active_alerts = prog_ctx.active_alert_count;
    memcpy(out_report->alerts, prog_ctx.alerts, sizeof(maintenance_alert_t) * MAX_MAINTENANCE_ALERTS);
}

void prognostics_reset_counters(subsystem_id_t id) {
    if (id >= SUBSYS_COUNT) return;
    health_records[id].operational_hours = 0.0f;
    health_records[id].thermal_cycles = 0;
    health_records[id].fault_events = 0;
    health_records[id].current_health_score = 1.0f;
    memset(health_records[id].trend_history, 0, sizeof(float) * TREND_WINDOW_SIZE);
}
