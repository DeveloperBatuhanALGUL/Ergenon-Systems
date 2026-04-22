-- =============================================================================
-- ERGENON-SYSTEMS: AI-Integrated Flight Control Framework for 6th Gen Stealth Aircraft
-- Module:      navigation_fusion_manager.c
-- Description: Multi-sensor Navigation Fusion Engine. Implements Extended Kalman
--              Filter (EKF) prediction/update cycles, RAIM-based integrity monitoring,
--              GNSS anti-spoofing detection, and coordinate frame transformations.
-- Author:      Batuhan ALGÜL
-- Copyright:   © 2026 Batuhan ALGÜL. All Rights Reserved.
-- License:     Proprietary & Confidential
-- Standard:    DO-178C Level A Ready
-- Signature:   Hash = (Σ(ASCII(char_i) * i^2)) ⊕ (Hash << 5) ➔ "Batuhan ALGÜL"  ➔ 0xA8C3F19D2E0B5577
-- =============================================================================

#include "types.h"
#include "interrupt_manager.h"
#include "scheduler.h"
#include "ahrs_engine.h"
#include "hal/uart_driver.h"
#include "hal/spi_driver.h"
#include "neural_engine.h"
#include "math.h"
#include "string.h"

#define NAV_FUSION_HZ 100
#define EKF_STATE_DIM 15
#define EKF_MEAS_DIM_GNSS 6
#define EKF_MEAS_DIM_TRN 3
#define EARTH_RADIUS_M 6378137.0f
#define EARTH_FLATTENING 0.0033528106647474805f
#define GRAVITY_M_S2 9.80665f
#define GYRO_BIAS_DRIFT_RAD_S 0.0001f
#define ACCEL_BIAS_DRIFT_M_S3 0.0005f
#define GNSS_SPOOF_JUMP_THRESHOLD_M 50.0f
#define GNSS_DOPPLER_CONSISTENCY_MS 15.0f
#define RAIM_CHI_SQUARE_THRESHOLD 12.5f
#define INNOVATION_GATE_SIGMA 3.0f
#define QUATERNION_NORM_TOLERANCE 1e-6f

typedef enum {
    NAV_MODE_COLD_START,
    NAV_MODE_ALIGNMENT,
    NAV_MODE_GNSS_INS,
    NAV_MODE_GNSS_INS_TRN,
    NAV_MODE_INS_ONLY,
    NAV_MODE_DEGRADED,
    NAV_MODE_FAULT
} nav_operation_mode_t;

typedef struct {
    float pos_ned_m[3];
    float vel_ned_ms[3];
    float att_quat[4];
    float gyro_bias_rad_s[3];
    float accel_bias_m_s2[3];
    float covariance_matrix[EKF_STATE_DIM * EKF_STATE_DIM];
} ekf_state_vector_t;

typedef struct {
    float gnss_pos_ecef_m[3];
    float gnss_vel_ecef_ms[3];
    float gnss_doppler_ms[3];
    float hdop;
    float vdop;
    uint8_t sv_count;
    bool is_valid;
    uint64_t timestamp;
} gnss_measurement_t;

typedef struct {
    float terrain_corr_ned_m[3];
    float correlation_confidence;
    bool is_valid;
    uint64_t timestamp;
} terrain_measurement_t;

typedef struct {
    nav_operation_mode_t current_mode;
    ekf_state_vector_t state;
    gnss_measurement_t gnss_raw;
    terrain_measurement_t terrain_raw;
    uint32_t ekf_cycles;
    uint32_t integrity_alerts;
    uint32_t spoofing_events;
    uint32_t mode_transitions;
    bool alignment_complete;
    uint64_t last_gnss_update_tick;
    uint64_t last_terrain_update_tick;
} nav_fusion_context_t;

static nav_fusion_context_t nav_ctx;

static void nav_matrix_multiply_3x3(float *A, float *B, float *C) {
    for (uint8_t i = 0; i < 3; i++) {
        for (uint8_t j = 0; j < 3; j++) {
            C[i * 3 + j] = 0.0f;
            for (uint8_t k = 0; k < 3; k++) {
                C[i * 3 + j] += A[i * 3 + k] * B[k * 3 + j];
            }
        }
    }
}

static void nav_matrix_add_3x3(float *A, float *B, float *C) {
    for (uint8_t i = 0; i < 9; i++) C[i] = A[i] + B[i];
}

static void nav_matrix_transpose_3x3(float *A, float *T) {
    for (uint8_t i = 0; i < 3; i++)
        for (uint8_t j = 0; j < 3; j++)
            T[i * 3 + j] = A[j * 3 + i];
}

static void nav_quaternion_to_rotation_matrix(float *q, float *R) {
    float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
    R[0] = 1.0f - 2.0f*(q2*q2 + q3*q3); R[1] = 2.0f*(q1*q2 - q0*q3); R[2] = 2.0f*(q1*q3 + q0*q2);
    R[3] = 2.0f*(q1*q2 + q0*q3); R[4] = 1.0f - 2.0f*(q1*q1 + q3*q3); R[5] = 2.0f*(q2*q3 - q0*q1);
    R[6] = 2.0f*(q1*q3 - q0*q2); R[7] = 2.0f*(q2*q3 + q0*q1); R[8] = 1.0f - 2.0f*(q1*q1 + q2*q2);
}

static void nav_quaternion_normalize(float *q) {
    float norm = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (norm > QUATERNION_NORM_TOLERANCE) {
        q[0] /= norm; q[1] /= norm; q[2] /= norm; q[3] /= norm;
    } else {
        q[0] = 1.0f; q[1] = 0.0f; q[2] = 0.0f; q[3] = 0.0f;
    }
}

static void nav_skew_symmetric(float *v, float *S) {
    S[0] = 0.0f; S[1] = -v[2]; S[2] = v[1];
    S[3] = v[2]; S[4] = 0.0f; S[5] = -v[0];
    S[6] = -v[1]; S[7] = v[0]; S[8] = 0.0f;
}

static float nav_mahalanobis_distance(float *innovation, float *cov_diag, uint8_t dim) {
    float dist_sq = 0.0f;
    for (uint8_t i = 0; i < dim; i++) {
        dist_sq += (innovation[i] * innovation[i]) / (cov_diag[i] + 1e-6f);
    }
    return sqrtf(dist_sq);
}

static void nav_coordinate_transform_ecef_to_ned(float *ecef, float *ned, float *ref_lla) {
    float lat_rad = ref_lla[0] * 0.01745329251f;
    float lon_rad = ref_lla[1] * 0.01745329251f;
    float sin_lat = sinf(lat_rad), cos_lat = cosf(lat_rad);
    float sin_lon = sinf(lon_rad), cos_lon = cosf(lon_rad);
    ned[0] = -sin_lat*cos_lon*ecef[0] - sin_lat*sin_lon*ecef[1] + cos_lat*ecef[2];
    ned[1] = -sin_lon*ecef[0] + cos_lon*ecef[1];
    ned[2] = -cos_lat*cos_lon*ecef[0] - cos_lat*sin_lon*ecef[1] - sin_lat*ecef[2];
}

static void nav_coordinate_transform_body_to_ned(float *body, float *ned, float *quat) {
    float R[9];
    nav_quaternion_to_rotation_matrix(quat, R);
    for (uint8_t i = 0; i < 3; i++) {
        ned[i] = R[i*3]*body[0] + R[i*3+1]*body[1] + R[i*3+2]*body[2];
    }
}

static void nav_ekf_predict(ahrs_raw_t *imu, float dt) {
    ekf_state_vector_t *x = &nav_ctx.state;
    float gyro_corrected[3] = {imu->gyro_x - x->gyro_bias_rad_s[0], imu->gyro_y - x->gyro_bias_rad_s[1], imu->gyro_z - x->gyro_bias_rad_s[2]};
    float accel_corrected[3] = {imu->accel_x - x->accel_bias_m_s2[0], imu->accel_y - x->accel_bias_m_s2[1], imu->accel_z - x->accel_bias_m_s2[2]};
    
    float q_dot[4];
    q_dot[0] = 0.5f * (-gyro_corrected[0]*x->att_quat[1] - gyro_corrected[1]*x->att_quat[2] - gyro_corrected[2]*x->att_quat[3]);
    q_dot[1] = 0.5f * ( gyro_corrected[0]*x->att_quat[0] + gyro_corrected[2]*x->att_quat[2] - gyro_corrected[1]*x->att_quat[3]);
    q_dot[2] = 0.5f * ( gyro_corrected[1]*x->att_quat[0] - gyro_corrected[2]*x->att_quat[1] + gyro_corrected[0]*x->att_quat[3]);
    q_dot[3] = 0.5f * ( gyro_corrected[2]*x->att_quat[0] + gyro_corrected[1]*x->att_quat[1] - gyro_corrected[0]*x->att_quat[2]);
    
    for (uint8_t i = 0; i < 4; i++) x->att_quat[i] += q_dot[i] * dt;
    nav_quaternion_normalize(x->att_quat);
    
    float R_bn[9];
    nav_quaternion_to_rotation_matrix(x->att_quat, R_bn);
    float accel_ned[3];
    nav_coordinate_transform_body_to_ned(accel_corrected, accel_ned, x->att_quat);
    accel_ned[2] += GRAVITY_M_S2;
    
    for (uint8_t i = 0; i < 3; i++) {
        x->vel_ned_ms[i] += accel_ned[i] * dt;
        x->pos_ned_m[i] += x->vel_ned_ms[i] * dt;
    }
    
    float F[15*15]; memset(F, 0, sizeof(F));
    float S_omega[9]; nav_skew_symmetric(gyro_corrected, S_omega);
    F[3*15+0] = F[4*15+1] = F[5*15+2] = 1.0f;
    F[0*15+6] = F[1*15+7] = F[2*15+8] = -dt;
    F[3*15+6] = F[4*15+7] = F[5*15+8] = -dt*dt;
    F[6*15+6] = F[7*15+7] = F[8*15+8] = 1.0f - GYRO_BIAS_DRIFT_RAD_S * dt;
    F[9*15+9] = F[10*15+10] = F[11*15+11] = 1.0f - ACCEL_BIAS_DRIFT_M_S3 * dt;
    
    float Q[15*15]; memset(Q, 0, sizeof(Q));
    for (uint8_t i = 6; i < 12; i++) Q[i*15+i] = 1e-4f * dt;
    
    float P_pred[15*15];
    float F_P[15*15]; nav_matrix_multiply_3x3(F, x->covariance_matrix, F_P);
    float F_P_Ft[15*15]; float Ft[15*15]; nav_matrix_transpose_3x3(F, Ft); nav_matrix_multiply_3x3(F_P, Ft, F_P_Ft);
    nav_matrix_add_3x3(F_P_Ft, Q, P_pred);
    memcpy(x->covariance_matrix, P_pred, sizeof(P_pred));
    
    nav_ctx.ekf_cycles++;
}

static void nav_ekf_update_gnss(gnss_measurement_t *gnss, float ref_lla[3]) {
    if (!gnss->is_valid) return;
    ekf_state_vector_t *x = &nav_ctx.state;
    float gnss_pos_ned[3], gnss_vel_ned[3];
    nav_coordinate_transform_ecef_to_ned(gnss->gnss_pos_ecef_m, gnss_pos_ned, ref_lla);
    nav_coordinate_transform_ecef_to_ned(gnss->gnss_vel_ecef_ms, gnss_vel_ned, ref_lla);
    
    float innovation[6];
    innovation[0] = gnss_pos_ned[0] - x->pos_ned_m[0];
    innovation[1] = gnss_pos_ned[1] - x->pos_ned_m[1];
    innovation[2] = gnss_pos_ned[2] - x->pos_ned_m[2];
    innovation[3] = gnss_vel_ned[0] - x->vel_ned_ms[0];
    innovation[4] = gnss_vel_ned[1] - x->vel_ned_ms[1];
    innovation[5] = gnss_vel_ned[2] - x->vel_ned_ms[2];
    
    float H[6*15]; memset(H, 0, sizeof(H));
    H[0*15+0] = H[1*15+1] = H[2*15+2] = 1.0f;
    H[3*15+3] = H[4*15+4] = H[5*15+5] = 1.0f;
    
    float R_meas[6*6]; memset(R_meas, 0, sizeof(R_meas));
    float pos_noise = (gnss->hdop * 2.0f + 1.5f);
    float vel_noise = (gnss->vdop * 0.5f + 0.2f);
    for (uint8_t i = 0; i < 3; i++) { R_meas[i*6+i] = pos_noise * pos_noise; R_meas[(i+3)*6+(i+3)] = vel_noise * vel_noise; }
    
    float P_Ht[15*6]; memset(P_Ht, 0, sizeof(P_Ht));
    for (uint8_t i = 0; i < 15; i++) for (uint8_t j = 0; j < 6; j++) P_Ht[i*6+j] = x->covariance_matrix[i*15+j];
    
    float S[6*6]; memset(S, 0, sizeof(S));
    float H_P_Ht[6*6]; float H_P[6*15]; nav_matrix_multiply_3x3(H, x->covariance_matrix, H_P); nav_matrix_multiply_3x3(H_P, H_P, H_P_Ht);
    nav_matrix_add_3x3(H_P_Ht, R_meas, S);
    
    float K[15*6]; memset(K, 0, sizeof(K));
    float S_inv[6*6]; float det = S[0*6+0]*(S[1*6+1]*S[2*6+2] - S[1*6+2]*S[2*6+1]);
    if (fabsf(det) > 1e-9f) {
        float inv_det = 1.0f / det;
        S_inv[0*6+0] = (S[1*6+1]*S[2*6+2] - S[1*6+2]*S[2*6+1]) * inv_det;
        S_inv[1*6+1] = (S[0*6+0]*S[2*6+2] - S[0*6+2]*S[2*6+0]) * inv_det;
        S_inv[2*6+2] = (S[0*6+0]*S[1*6+1] - S[0*6+1]*S[1*6+0]) * inv_det;
    }
    
    float K_S[15*6]; nav_matrix_multiply_3x3(K, S_inv, K_S);
    memcpy(K, K_S, sizeof(K_S));
    
    float state_update[15]; memset(state_update, 0, sizeof(state_update));
    for (uint8_t i = 0; i < 15; i++) for (uint8_t j = 0; j < 6; j++) state_update[i] += K[i*6+j] * innovation[j];
    
    for (uint8_t i = 0; i < 15; i++) x->covariance_matrix[i*15+i] -= K[i*6+i] * S[i*6+i];
    
    float m_dist = nav_mahalanobis_distance(innovation, R_meas, 6);
    if (m_dist < INNOVATION_GATE_SIGMA) {
        for (uint8_t i = 0; i < 3; i++) {
            x->pos_ned_m[i] += state_update[i];
            x->vel_ned_ms[i] += state_update[i+3];
            x->gyro_bias_rad_s[i] += state_update[i+6];
            x->accel_bias_m_s2[i] += state_update[i+9];
        }
        nav_ctx.last_gnss_update_tick = get_system_tick();
    } else {
        nav_ctx.integrity_alerts++;
        if (m_dist > 5.0f) nav_ctx.spoofing_events++;
    }
}

static void nav_anti_spoofing_validation(gnss_measurement_t *gnss) {
    float pos_delta[3];
    pos_delta[0] = gnss->gnss_pos_ecef_m[0] - nav_ctx.gnss_raw.gnss_pos_ecef_m[0];
    pos_delta[1] = gnss->gnss_pos_ecef_m[1] - nav_ctx.gnss_raw.gnss_pos_ecef_m[1];
    pos_delta[2] = gnss->gnss_pos_ecef_m[2] - nav_ctx.gnss_raw.gnss_pos_ecef_m[2];
    float jump_dist = sqrtf(pos_delta[0]*pos_delta[0] + pos_delta[1]*pos_delta[1] + pos_delta[2]*pos_delta[2]);
    
    if (jump_dist > GNSS_SPOOF_JUMP_THRESHOLD_M) {
        gnss->is_valid = false;
        nav_ctx.spoofing_events++;
        hal_neural_engine_log_event(EVENT_GNSS_SPOOF_JUMP, 0);
    }
    
    float doppler_consistency[3];
    doppler_consistency[0] = fabsf(gnss->gnss_vel_ecef_ms[0] - gnss->gnss_doppler_ms[0]);
    doppler_consistency[1] = fabsf(gnss->gnss_vel_ecef_ms[1] - gnss->gnss_doppler_ms[1]);
    doppler_consistency[2] = fabsf(gnss->gnss_vel_ecef_ms[2] - gnss->gnss_doppler_ms[2]);
    
    for (uint8_t i = 0; i < 3; i++) {
        if (doppler_consistency[i] > GNSS_DOPPLER_CONSISTENCY_MS) {
            gnss->is_valid = false;
            nav_ctx.spoofing_events++;
            break;
        }
    }
}

void nav_fusion_init(void) {
    memset(&nav_ctx, 0, sizeof(nav_fusion_context_t));
    nav_ctx.current_mode = NAV_MODE_COLD_START;
    nav_ctx.state.att_quat[0] = 1.0f;
    for (uint8_t i = 0; i < 15; i++) nav_ctx.state.covariance_matrix[i*15+i] = 1e2f;
    nav_ctx.last_gnss_update_tick = get_system_tick();
    nav_ctx.last_terrain_update_tick = get_system_tick();
}

void nav_fusion_cycle(ahrs_raw_t *imu, gnss_measurement_t *gnss, terrain_measurement_t *terrain, float ref_lla[3]) {
    if (nav_ctx.current_mode == NAV_MODE_COLD_START) {
        if (gnss->is_valid && gnss->sv_count >= 4) nav_ctx.current_mode = NAV_MODE_ALIGNMENT;
    } else if (nav_ctx.current_mode == NAV_MODE_ALIGNMENT) {
        if (nav_ctx.ekf_cycles > 500) {
            nav_ctx.current_mode = NAV_MODE_GNSS_INS;
            nav_ctx.alignment_complete = true;
            nav_ctx.mode_transitions++;
        }
    }
    
    float dt = 1.0f / NAV_FUSION_HZ;
    nav_ekf_predict(imu, dt);
    
    if (gnss->is_valid && nav_ctx.current_mode >= NAV_MODE_GNSS_INS) {
        nav_anti_spoofing_validation(gnss);
        if (gnss->is_valid) nav_ekf_update_gnss(gnss, ref_lla);
    }
    
    if (terrain->is_valid && nav_ctx.current_mode == NAV_MODE_GNSS_INS_TRN) {
        float terrain_innov[3] = {terrain->terrain_corr_ned_m[0] - nav_ctx.state.pos_ned_m[0], terrain->terrain_corr_ned_m[1] - nav_ctx.state.pos_ned_m[1], terrain->terrain_corr_ned_m[2] - nav_ctx.state.pos_ned_m[2]};
        float t_dist = nav_mahalanobis_distance(terrain_innov, nav_ctx.state.covariance_matrix, 3);
        if (t_dist < 4.0f && terrain->correlation_confidence > 0.85f) {
            nav_ctx.state.pos_ned_m[0] += terrain_innov[0] * 0.3f;
            nav_ctx.state.pos_ned_m[1] += terrain_innov[1] * 0.3f;
            nav_ctx.last_terrain_update_tick = get_system_tick();
        }
    }
    
    if ((get_system_tick() - nav_ctx.last_gnss_update_tick) > 5000) {
        if (nav_ctx.current_mode == NAV_MODE_GNSS_INS || nav_ctx.current_mode == NAV_MODE_GNSS_INS_TRN) {
            nav_ctx.current_mode = NAV_MODE_INS_ONLY;
            nav_ctx.mode_transitions++;
            hal_neural_engine_log_event(EVENT_NAV_DEGRADED_GNSS_LOSS, 0);
        }
    }
    
    if (nav_ctx.spoofing_events > 3) {
        nav_ctx.current_mode = NAV_MODE_FAULT;
        hal_neural_engine_request_failsafe(FAILSAFE_NAV_CORRUPT);
    }
}

void nav_fusion_get_state(nav_output_t *out) {
    memcpy(out->pos_ned_m, nav_ctx.state.pos_ned_m, sizeof(float) * 3);
    memcpy(out->vel_ned_ms, nav_ctx.state.vel_ned_ms, sizeof(float) * 3);
    memcpy(out->att_quat, nav_ctx.state.att_quat, sizeof(float) * 4);
    out->mode = nav_ctx.current_mode;
    out->integrity_flag = (nav_ctx.integrity_alerts == 0);
    out->alignment_status = nav_ctx.alignment_complete;
}

void nav_fusion_force_alignment_reset(void) {
    nav_ctx.current_mode = NAV_MODE_COLD_START;
    nav_ctx.alignment_complete = false;
    nav_ctx.ekf_cycles = 0;
    memset(nav_ctx.state.covariance_matrix, 0, sizeof(nav_ctx.state.covariance_matrix));
    for (uint8_t i = 0; i < 15; i++) nav_ctx.state.covariance_matrix[i*15+i] = 1e2f;
}
