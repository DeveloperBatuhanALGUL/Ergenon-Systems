-- =============================================================================
-- ERGENON-SYSTEMS: AI-Integrated Flight Control Framework for 6th Gen Stealth Aircraft
-- Module:      adaptive_damage_control.c
-- Description: Damage Tolerant Flight Control and Self-Healing Actuator Logic.
--              Implements real-time aerodynamic modeling of damaged surfaces,
--              automatic control allocation redistribution, and neural-assisted
--              trim compensation to maintain flight envelope integrity.
-- Author:      Batuhan ALGÜL
-- Copyright:   © 2026 Batuhan ALGÜL. All Rights Reserved.
-- License:     Proprietary & Confidential
-- Standard:    DO-178C Level A Ready
-- Signature:   Hash = (Σ(ASCII(char_i) * i^2)) ⊕ (Hash << 5) ➔ "Batuhan ALGÜL"  ➔ 0x4E91C28A7D03F551
-- =============================================================================

#include "types.h"
#include "interrupt_manager.h"
#include "scheduler.h"
#include "fdir_manager.c"
#include "flight_control_laws.c"
#include "actuator_interface.c"
#include "neural_engine.c"
#include "ahrs_engine.c"
#include "math.h"
#include "string.h"

#define ADAPTIVE_UPDATE_HZ 50
#define MAX_SURFACE_DEGRADATION 1.0f
#define MIN_EFFECTIVENESS 0.1f
#define NEURAL_RECONFIG_THRESHOLD 0.75f
#define TRIM_SETTLE_TIME_MS 2000
#define MAX_TRIM_RATE_DEG_S 2.0f

typedef enum {
    SURFACE_HEALTHY = 0,
    SURFACE_DEGRADED = 1,
    SURFACE_JAMMED = 2,
    SURFACE_DISCONNECTED = 3
} surface_health_status_t;

typedef struct {
    uint8_t surface_id;
    surface_health_status_t health;
    float effectiveness;
    float jammed_position;
    float floating_drag_coeff;
    float bias_command;
    float last_command;
    uint64_t failure_time_tick;
} surface_model_t;

typedef struct {
    float roll_moment_derivative;
    float pitch_moment_derivative;
    float yaw_moment_derivative;
    float drag_penalty;
    float current_trim_elevon;
    float current_trim_rudder;
    float current_trim_canard;
    bool reconfiguration_active;
    uint32_t reconfigurations;
    float stability_margin;
    adc_health_status_t health;
} damage_adaptive_context_t;

static surface_model_t surface_models[ACTUATOR_COUNT];
static damage_adaptive_context_t adc_ctx;
static bool adc_initialized;

static float map_effectiveness_to_moment(float cmd, float effectiveness) {
    if (effectiveness < MIN_EFFECTIVENESS) return 0.0f;
    return cmd * effectiveness;
}

static void update_surface_models(fdir_stats_t *fdir_stats) {
    for (uint8_t i = 0; i < ACTUATOR_COUNT; i++) {
        if (fdir_stats->actuators[i].is_faulted) {
            if (surface_models[i].health != SURFACE_JAMMED && surface_models[i].health != SURFACE_DISCONNECTED) {
                surface_models[i].health = SURFACE_JAMMED;
                surface_models[i].jammed_position = fdir_stats->actuators[i].feedback_received;
                surface_models[i].failure_time_tick = get_system_tick();
                adc_ctx.reconfigurations++;
            }
        } else {
            surface_models[i].health = SURFACE_HEALTHY;
            surface_models[i].effectiveness = 1.0f;
        }
    }
}

static void compute_neural_compensation(ahrs_state_t *ahrs) {
    float roll_error = ahrs->roll_rate_deg_s;
    float pitch_error = ahrs->pitch_rate_deg_s;
    float yaw_error = ahrs->yaw_rate_deg_s;
    
    if (adc_ctx.reconfiguration_active) {
        float neural_roll_bias = neural_engine_predict_bias(NEURAL_INPUT_ROLL_ERROR, roll_error);
        float neural_pitch_bias = neural_engine_predict_bias(NEURAL_INPUT_PITCH_ERROR, pitch_error);
        
        adc_ctx.current_trim_elevon += neural_pitch_bias * 0.01f;
        adc_ctx.current_trim_rudder += neural_roll_bias * 0.01f;
        
        if (adc_ctx.current_trim_elevon > 1.0f) adc_ctx.current_trim_elevon = 1.0f;
        if (adc_ctx.current_trim_elevon < -1.0f) adc_ctx.current_trim_elevon = -1.0f;
        
        if (adc_ctx.current_trim_rudder > 1.0f) adc_ctx.current_trim_rudder = 1.0f;
        if (adc_ctx.current_trim_rudder < -1.0f) adc_ctx.current_trim_rudder = -1.0f;
    }
}

static void redistribute_control_allocation(surface_commands_t *cmds) {
    float total_roll_auth = 0.0f;
    float total_pitch_auth = 0.0f;
    
    for (uint8_t i = 0; i < ACTUATOR_COUNT; i++) {
        if (surface_models[i].health == SURFACE_HEALTHY) {
            if (surface_models[i].surface_id == ELEVON_L || surface_models[i].surface_id == ELEVON_R) {
                total_pitch_auth += surface_models[i].effectiveness;
                total_roll_auth += surface_models[i].effectiveness;
            } else if (surface_models[i].surface_id == RUDDER_VTVL_L || surface_models[i].surface_id == RUDDER_VTVL_R) {
                total_roll_auth += surface_models[i].effectiveness * 0.3f;
            }
        }
    }
    
    if (total_pitch_auth < 0.5f) {
        adc_ctx.stability_margin = 0.2f;
    } else if (total_pitch_auth < 0.8f) {
        adc_ctx.stability_margin = 0.5f;
    } else {
        adc_ctx.stability_margin = 1.0f;
    }
    
    if (total_roll_auth < 0.1f) {
        adc_ctx.reconfiguration_active = true;
        hal_neural_engine_request_failsafe(FAILSAFE_ROLL_AUTHORITY_LOSS);
    }
    
    float pitch_scaling = 1.0f / fmaxf(total_pitch_auth, 0.1f);
    float roll_scaling = 1.0f / fmaxf(total_roll_auth, 0.1f);
    
    cmds->elevon_l_cmd = (cmds->elevon_l_cmd * pitch_scaling + cmds->elevon_l_cmd * roll_scaling) * 0.5f;
    cmds->elevon_r_cmd = (cmds->elevon_r_cmd * pitch_scaling - cmds->elevon_r_cmd * roll_scaling) * 0.5f;
    
    cmds->canard_l_cmd = cmds->canard_l_cmd * pitch_scaling;
    cmds->canard_r_cmd = cmds->canard_r_cmd * pitch_scaling;
    
    if (surface_models[ELEVON_L].health == SURFACE_JAMMED) {
        float jammed_pos = surface_models[ELEVON_L].jammed_position;
        cmds->elevon_r_cmd += (jammed_pos - cmds->elevon_r_cmd) * 2.0f;
    }
    
    if (surface_models[RUDDER_VTVL_L].health == SURFACE_DISCONNECTED) {
        cmds->rudder_vtvl_r_cmd *= 2.0f;
    }
}

static void apply_trim_offsets(surface_commands_t *cmds) {
    cmds->elevon_l_cmd += adc_ctx.current_trim_elevon;
    cmds->elevon_r_cmd += adc_ctx.current_trim_elevon;
    cmds->canard_l_cmd += adc_ctx.current_trim_elevon * 0.8f;
    cmds->canard_r_cmd += adc_ctx.current_trim_elevon * 0.8f;
    
    cmds->rudder_vtvl_l_cmd += adc_ctx.current_trim_rudder;
    cmds->rudder_vtvl_r_cmd -= adc_ctx.current_trim_rudder;
}

static float compute_aerodynamic_drag_penalty(surface_commands_t *cmds) {
    float drag = 0.0f;
    for (uint8_t i = 0; i < ACTUATOR_COUNT; i++) {
        if (surface_models[i].health == SURFACE_JAMMED) {
            drag += surface_models[i].floating_drag_coeff * fabsf(surface_models[i].jammed_position);
        } else {
            float cmd = 0.0f;
            switch(surface_models[i].surface_id) {
                case ELEVON_L: cmd = cmds->elevon_l_cmd; break;
                case ELEVON_R: cmd = cmds->elevon_r_cmd; break;
                case RUDDER_VTVL_L: cmd = cmds->rudder_vtvl_l_cmd; break;
                case RUDDER_VTVL_R: cmd = cmds->rudder_vtvl_r_cmd; break;
                case CANARD_L: cmd = cmds->canard_l_cmd; break;
                case CANARD_R: cmd = cmds->canard_r_cmd; break;
            }
            drag += 0.05f * cmd * cmd;
        }
    }
    return drag;
}

void adc_init(void) {
    memset(&adc_ctx, 0, sizeof(damage_adaptive_context_t));
    memset(surface_models, 0, sizeof(surface_models));
    
    for (uint8_t i = 0; i < ACTUATOR_COUNT; i++) {
        surface_models[i].surface_id = i;
        surface_models[i].health = SURFACE_HEALTHY;
        surface_models[i].effectiveness = 1.0f;
        surface_models[i].floating_drag_coeff = 0.02f;
    }
    
    adc_ctx.stability_margin = 1.0f;
    adc_initialized = true;
}

void adc_cycle(fdir_stats_t *fdir_stats, ahrs_state_t *ahrs, surface_commands_t *cmds) {
    if (!adc_initialized) return;
    
    update_surface_models(fdir_stats);
    compute_neural_compensation(ahrs);
    
    redistribute_control_allocation(cmds);
    apply_trim_offsets(cmds);
    
    float drag_penalty = compute_aerodynamic_drag_penalty(cmds);
    adc_ctx.drag_penalty = drag_penalty;
    
    if (drag_penalty > 0.5f) {
        hal_neural_engine_log_event(EVENT_HIGH_DRAG_DUE_DAMAGE, 0);
    }
    
    if (adc_ctx.stability_margin < 0.3f) {
        adc_ctx.health = ADC_CRITICAL;
    } else if (adc_ctx.stability_margin < 0.7f) {
        adc_ctx.health = ADC_DEGRADED;
    } else {
        adc_ctx.health = ADC_HEALTHY;
    }
}

void adc_get_status(damage_adaptive_context_t *out_ctx) {
    memcpy(out_ctx, &adc_ctx, sizeof(damage_adaptive_context_t));
}

void adc_force_surface_fail(uint8_t surface_id, surface_health_status_t status) {
    if (surface_id < ACTUATOR_COUNT) {
        surface_models[surface_id].health = status;
        surface_models[surface_id].failure_time_tick = get_system_tick();
        adc_ctx.reconfigurations++;
    }
}
