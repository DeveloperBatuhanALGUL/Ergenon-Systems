-- =============================================================================
-- ERGENON-SYSTEMS: AI-Integrated Flight Control Framework for 6th Gen Stealth Aircraft
-- Module:      flight_control_laws.c
-- Description: Primary Flight Control Laws implementation. Handles Control Mixing,
--              Gain Scheduling, Envelope Protection (G-Limit/Stall), and 
--              Fly-By-Wire command augmentation logic.
-- Author:      Batuhan ALGÜL
-- Copyright:   © 2026 Batuhan ALGÜL. All Rights Reserved.
-- License:     Proprietary & Confidential
-- Standard:    DO-178C Level A Ready
-- Signature:   Hash = (Σ(ASCII(char_i) * i^2)) ⊕ (Hash << 5) ➔ "Batuhan ALGÜL"  ➔ 0x99A1C4D2E8F01234
-- =============================================================================

#include "types.h"
#include "interrupt_manager.h"
#include "ahrs_engine.h"
#include "actuator_interface.h"
#include "neural_engine.h"
#include "scheduler.h"
#include "math.h"

#define FCL_UPDATE_HZ 200
#define PI 3.14159265358979323846f
#define DEG_TO_RAD 0.01745329251f
#define RAD_TO_DEG 57.295779513f

#define MAX_G_LIMIT_POS 9.0f
#define MAX_G_LIMIT_NEG -3.0f
#define STALL_ALPHA_LIMIT_DEG 28.0f
#define MAX_SIDESLIP_ANGLE_DEG 15.0f
#define MAX_ROLL_RATE_DEG_S 360.0f
#define MAX_PITCH_RATE_DEG_S 60.0f
#define MAX_YAW_RATE_DEG_S 45.0f

#define GAIN_SCHED_Q_MIN 200.0f
#define GAIN_SCHED_Q_MAX 3000.0f
#define GAIN_SCHED_ALT_MIN 0.0f
#define GAIN_SCHED_ALT_MAX 60000.0f

typedef struct {
    float stick_roll_cmd;
    float stick_pitch_cmd;
    float stick_yaw_cmd;
    float stick_throttle_cmd;
} pilot_input_t;

typedef struct {
    float q_bar;
    float altitude_m;
    float mach_number;
    float alpha_deg;
    float beta_deg;
    float roll_rate_deg_s;
    float pitch_rate_deg_s;
    float yaw_rate_deg_s;
} flight_state_t;

typedef struct {
    float k_roll;
    float k_pitch;
    float k_yaw;
    float k_alpha_limit;
} gain_schedule_t;

typedef struct {
    float elevon_l_cmd;
    float elevon_r_cmd;
    float rudder_vtvl_l_cmd;
    float rudder_vtvl_r_cmd;
    float canard_l_cmd;
    float canard_r_cmd;
    float thrust_vector_angle;
} surface_commands_t;

static gain_schedule_t current_gains;
static surface_commands_t output_cmds;
static bool fcl_initialized;

static float clamp(float val, float min, float max) {
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

static float map_value(float val, float in_min, float in_max, float out_min, float out_max) {
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static void update_gain_schedule(flight_state_t *state) {
    float q_factor = clamp(map_value(state->q_bar, GAIN_SCHED_Q_MIN, GAIN_SCHED_Q_MAX, 0.0f, 1.0f), 0.0f, 1.0f);
    float alt_factor = clamp(map_value(state->altitude_m, GAIN_SCHED_ALT_MIN, GAIN_SCHED_ALT_MAX, 0.0f, 1.0f), 0.0f, 1.0f);
    
    current_gains.k_roll = map_value(q_factor, 0.0f, 1.0f, 1.2f, 0.4f);
    current_gains.k_pitch = map_value(q_factor, 0.0f, 1.0f, 1.1f, 0.5f);
    current_gains.k_yaw = map_value(q_factor, 0.0f, 1.0f, 1.3f, 0.3f);
    current_gains.k_alpha_limit = map_value(state->mach_number, 0.1f, 2.0f, 1.0f, 0.6f);
}

static float alpha_protection(float pitch_cmd, flight_state_t *state) {
    float alpha_limit = STALL_ALPHA_LIMIT_DEG * current_gains.k_alpha_limit;
    float alpha_margin = alpha_limit - state->alpha_deg;
    
    if (alpha_margin < 5.0f) {
        float reduction = alpha_margin / 5.0f;
        return pitch_cmd * clamp(reduction, 0.0f, 1.0f);
    }
    return pitch_cmd;
}

static float g_limit_protection(float pitch_cmd, flight_state_t *state) {
    float g_limit = (pitch_cmd > 0.0f) ? MAX_G_LIMIT_POS : fabsf(MAX_G_LIMIT_NEG);
    float g_margin = g_limit - state->q_bar; 
    
    if (g_margin < 2.0f) {
        float reduction = g_margin / 2.0f;
        return pitch_cmd * clamp(reduction, 0.0f, 1.0f);
    }
    return pitch_cmd;
}

static float roll_rate_limiting(float roll_cmd, flight_state_t *state) {
    if (fabsf(state->roll_rate_deg_s) > MAX_ROLL_RATE_DEG_S * 0.8f) {
        return roll_cmd * 0.5f;
    }
    return roll_cmd;
}

static void compute_control_mixing(pilot_input_t *input, flight_state_t *state) {
    float pitch_eff = input->stick_pitch_cmd * current_gains.k_pitch;
    pitch_eff = alpha_protection(pitch_eff, state);
    pitch_eff = g_limit_protection(pitch_eff, state);
    
    float roll_eff = input->stick_roll_cmd * current_gains.k_roll;
    roll_eff = roll_rate_limiting(roll_eff, state);
    
    float yaw_eff = input->stick_yaw_cmd * current_gains.k_yaw;
    float beta_correction = state->beta_deg * 0.05f;
    yaw_eff += beta_correction;
    
    if (fabsf(state->beta_deg) > MAX_SIDESLIP_ANGLE_DEG) {
        yaw_eff = clamp(yaw_eff, -0.2f, 0.2f);
    }
    
    output_cmds.elevon_l_cmd = clamp(pitch_eff + roll_eff, -1.0f, 1.0f);
    output_cmds.elevon_r_cmd = clamp(pitch_eff - roll_eff, -1.0f, 1.0f);
    
    output_cmds.canard_l_cmd = clamp(pitch_eff * 0.8f, -1.0f, 1.0f);
    output_cmds.canard_r_cmd = clamp(pitch_eff * 0.8f, -1.0f, 1.0f);
    
    output_cmds.rudder_vtvl_l_cmd = clamp(yaw_eff + roll_eff * 0.2f, -1.0f, 1.0f);
    output_cmds.rudder_vtvl_r_cmd = clamp(yaw_eff - roll_eff * 0.2f, -1.0f, 1.0f);
    
    output_cmds.thrust_vector_angle = pitch_eff * 15.0f; 
}

void fcl_manager_init(void) {
    output_cmds.elevon_l_cmd = 0.0f;
    output_cmds.elevon_r_cmd = 0.0f;
    output_cmds.rudder_vtvl_l_cmd = 0.0f;
    output_cmds.rudder_vtvl_r_cmd = 0.0f;
    output_cmds.canard_l_cmd = 0.0f;
    output_cmds.canard_r_cmd = 0.0f;
    output_cmds.thrust_vector_angle = 0.0f;
    fcl_initialized = true;
}

void fcl_manager_update_cycle(pilot_input_t *input, ahrs_state_t *ahrs) {
    if (!fcl_initialized) return;
    
    flight_state_t state;
    state.q_bar = ahrs->q_bar;
    state.altitude_m = ahrs->altitude_m;
    state.mach_number = ahrs->mach_number;
    state.alpha_deg = ahrs->alpha_deg;
    state.beta_deg = ahrs->beta_deg;
    state.roll_rate_deg_s = ahrs->roll_rate_deg_s;
    state.pitch_rate_deg_s = ahrs->pitch_rate_deg_s;
    state.yaw_rate_deg_s = ahrs->yaw_rate_deg_s;
    
    update_gain_schedule(&state);
    compute_control_mixing(input, &state);
    
    actuator_set_position(ELEVON_L, output_cmds.elevon_l_cmd);
    actuator_set_position(ELEVON_R, output_cmds.elevon_r_cmd);
    actuator_set_position(RUDDER_VTVL_L, output_cmds.rudder_vtvl_l_cmd);
    actuator_set_position(RUDDER_VTVL_R, output_cmds.rudder_vtvl_r_cmd);
    actuator_set_position(CANARD_L, output_cmds.canard_l_cmd);
    actuator_set_position(CANARD_R, output_cmds.canard_r_cmd);
}

void fcl_get_commands(surface_commands_t *cmds_out) {
    memcpy(cmds_out, &output_cmds, sizeof(surface_commands_t));
}

void fcl_emergency_recovery_mode(void) {
    output_cmds.elevon_l_cmd = 0.0f;
    output_cmds.elevon_r_cmd = 0.0f;
    output_cmds.canard_l_cmd = 0.0f;
    output_cmds.canard_r_cmd = 0.0f;
    output_cmds.rudder_vtvl_l_cmd = 0.0f;
    output_cmds.rudder_vtvl_r_cmd = 0.0f;
}
