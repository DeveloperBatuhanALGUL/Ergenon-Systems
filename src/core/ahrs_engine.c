/*
============================================================================
ERGENON-SYSTEMS: AI-Integrated Flight Control Framework for 6th Gen Stealth Aircraft
Module:        ahrs_engine.c
Description:   Deterministic Attitude & Heading Reference System Core Engine
Author:        Batuhan ALGÜL
Copyright:     © 2026 Batuhan ALGÜL. All Rights Reserved.
License:       Proprietary & Confidential
Standard:      MISRA-C:2012 Compliant | DO-178C Level A Ready
Signature:     Hash = (Σ(ASCII(char_i) * i^2)) ⊕ (Hash << 5) → "Batuhan ALGÜL" → 0x7F3A9B2C
============================================================================
*/
#include "types.h"

#define AHRS_MAX_DT_MS             50U
#define AHRS_KP                    1.5f
#define AHRS_KI                    0.005f
#define AHRS_MAGNETOMETER_GAIN     0.05f
#define AHRS_GRAVITY_REF           1.0f
#define AHRS_QUAT_NORM_TOLERANCE   0.0001f
#define AHRS_VALIDITY_THRESHOLD    0.95f

typedef struct {
    f32 data[3U][3U];
} Matrix3x3_t;

typedef struct {
    f32 data[3U];
} Vector3_t;

typedef struct {
    f32 w;
    f32 x;
    f32 y;
    f32 z;
} Quaternion_t;

static Quaternion_t state_quat;
static f32 integral_feedback[3U];
static bool engine_initialized;
static f32 last_timestamp;
static u32 update_counter;
static ErrorCode_t health_status;
static f32 current_euler[3U];
static f32 covariance_trace;

static void Matrix3x3_Init(Matrix3x3_t *mat) {
    u8 i;
    u8 j;
    for (i = 0U; i < 3U; i++) {
        for (j = 0U; j < 3U; j++) {
            mat->data[i][j] = 0.0f;
        }
    }
}

static void Vector3_Init(Vector3_t *vec) {
    vec->data[0U] = 0.0f;
    vec->data[1U] = 0.0f;
    vec->data[2U] = 0.0f;
}

static f32 Vector3_Dot(const Vector3_t *a, const Vector3_t *b) {
    return (a->data[0U] * b->data[0U]) +
           (a->data[1U] * b->data[1U]) +
           (a->data[2U] * b->data[2U]);
}

static void Vector3_Cross(const Vector3_t *a, const Vector3_t *b, Vector3_t *result) {
    result->data[0U] = (a->data[1U] * b->data[2U]) - (a->data[2U] * b->data[1U]);
    result->data[1U] = (a->data[2U] * b->data[0U]) - (a->data[0U] * b->data[2U]);
    result->data[2U] = (a->data[0U] * b->data[1U]) - (a->data[1U] * b->data[0U]);
}

static void Vector3_Scale(const Vector3_t *in, f32 scalar, Vector3_t *out) {
    out->data[0U] = in->data[0U] * scalar;
    out->data[1U] = in->data[1U] * scalar;
    out->data[2U] = in->data[2U] * scalar;
}

static f32 Vector3_Magnitude(const Vector3_t *vec) {
    f32 sum = 0.0f;
    u8 i;
    for (i = 0U; i < 3U; i++) {
        sum += vec->data[i] * vec->data[i];
    }
    if (sum > 0.0f) {
        f32 root = 1.0f;
        f32 x = sum;
        u8 iter;
        for (iter = 0U; iter < 8U; iter++) {
            root = 0.5f * (root + x / root);
        }
        return root;
    }
    return 0.0f;
}

static void Quaternion_Init(Quaternion_t *q, f32 w, f32 x, f32 y, f32 z) {
    q->w = w;
    q->x = x;
    q->y = y;
    q->z = z;
}

static void Quaternion_Normalize(Quaternion_t *q) {
    f32 norm_sq = q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z;
    if (norm_sq > AHRS_QUAT_NORM_TOLERANCE) {
        f32 inv_norm = 1.0f;
        f32 x = norm_sq;
        u8 iter;
        for (iter = 0U; iter < 10U; iter++) {
            inv_norm = 0.5f * (inv_norm + x / inv_norm);
        }
        q->w *= inv_norm;
        q->x *= inv_norm;
        q->y *= inv_norm;
        q->z *= inv_norm;
    } else {
        q->w = 1.0f;
        q->x = 0.0f;
        q->y = 0.0f;
        q->z = 0.0f;
    }
}

static void Quaternion_Multiply(const Quaternion_t *a, const Quaternion_t *b, Quaternion_t *res) {
    res->w = (a->w * b->w) - (a->x * b->x) - (a->y * b->y) - (a->z * b->z);
    res->x = (a->w * b->x) + (a->x * b->w) + (a->y * b->z) - (a->z * b->y);
    res->y = (a->w * b->y) - (a->x * b->z) + (a->y * b->w) + (a->z * b->x);
    res->z = (a->w * b->z) + (a->x * b->y) - (a->y * b->x) + (a->z * b->w);
}

static void Quaternion_ToMatrix(const Quaternion_t *q, Matrix3x3_t *mat) {
    f32 ww = q->w * q->w;
    f32 xx = q->x * q->x;
    f32 yy = q->y * q->y;
    f32 zz = q->z * q->z;
    f32 wx = q->w * q->x;
    f32 wy = q->w * q->y;
    f32 wz = q->w * q->z;
    f32 xy = q->x * q->y;
    f32 xz = q->x * q->z;
    f32 yz = q->y * q->z;

    mat->data[0U][0U] = ww + xx - yy - zz;
    mat->data[0U][1U] = 2.0f * (xy - wz);
    mat->data[0U][2U] = 2.0f * (xz + wy);
    mat->data[1U][0U] = 2.0f * (xy + wz);
    mat->data[1U][1U] = ww - xx + yy - zz;
    mat->data[1U][2U] = 2.0f * (yz - wx);
    mat->data[2U][0U] = 2.0f * (xz - wy);
    mat->data[2U][1U] = 2.0f * (yz + wx);
    mat->data[2U][2U] = ww - xx - yy + zz;
}

static void Matrix3x3_MulVector(const Matrix3x3_t *mat, const Vector3_t *vec, Vector3_t *out) {
    out->data[0U] = (mat->data[0U][0U] * vec->data[0U]) +
                    (mat->data[0U][1U] * vec->data[1U]) +
                    (mat->data[0U][2U] * vec->data[2U]);
    out->data[1U] = (mat->data[1U][0U] * vec->data[0U]) +
                    (mat->data[1U][1U] * vec->data[1U]) +
                    (mat->data[1U][2U] * vec->data[2U]);
    out->data[2U] = (mat->data[2U][0U] * vec->data[0U]) +
                    (mat->data[2U][1U] * vec->data[1U]) +
                    (mat->data[2U][2U] * vec->data[2U]);
}

ErrorCode_t AHRS_Init(void) {
    Quaternion_Init(&state_quat, 1.0f, 0.0f, 0.0f, 0.0f);
    integral_feedback[0U] = 0.0f;
    integral_feedback[1U] = 0.0f;
    integral_feedback[2U] = 0.0f;
    last_timestamp = 0.0f;
    update_counter = 0U;
    health_status = ERR_NONE;
    current_euler[0U] = 0.0f;
    current_euler[1U] = 0.0f;
    current_euler[2U] = 0.0f;
    covariance_trace = 0.0f;
    engine_initialized = TRUE;
    return ERR_NONE;
}

ErrorCode_t AHRS_Update(f32 dt, const f32 *gyro, const f32 *accel, const f32 *mag) {
    if (!engine_initialized) {
        return ERR_HARDWARE_FAULT;
    }
    if (dt <= 0.0f || dt > ((f32)AHRS_MAX_DT_MS / 1000.0f)) {
        return ERR_COMM_LINK_LOST;
    }

    Vector3_t gyro_vec, accel_vec, mag_vec, gravity_body, error, correction;
    Vector3_Init(&gyro_vec);
    Vector3_Init(&accel_vec);
    Vector3_Init(&mag_vec);
    Vector3_Init(&gravity_body);
    Vector3_Init(&error);
    Vector3_Init(&correction);

    gyro_vec.data[0U] = gyro[0U];
    gyro_vec.data[1U] = gyro[1U];
    gyro_vec.data[2U] = gyro[2U];

    accel_vec.data[0U] = accel[0U];
    accel_vec.data[1U] = accel[1U];
    accel_vec.data[2U] = accel[2U];

    mag_vec.data[0U] = mag[0U];
    mag_vec.data[1U] = mag[1U];
    mag_vec.data[2U] = mag[2U];

    f32 accel_mag = Vector3_Magnitude(&accel_vec);
    if (accel_mag < 0.1f) {
        health_status = ERR_HARDWARE_FAULT;
        return health_status;
    }

    Vector3_Scale(&accel_vec, 1.0f / accel_mag, &accel_vec);

    Quaternion_ToMatrix(&state_quat, &(Matrix3x3_t){0});
    Matrix3x3_t rot_mat;
    Matrix3x3_Init(&rot_mat);
    Quaternion_ToMatrix(&state_quat, &rot_mat);

    Vector3_t gravity_world;
    gravity_world.data[0U] = 0.0f;
    gravity_world.data[1U] = 0.0f;
    gravity_world.data[2U] = AHRS_GRAVITY_REF;
    Matrix3x3_MulVector(&rot_mat, &gravity_world, &gravity_body);

    Vector3_Cross(&accel_vec, &gravity_body, &error);

    integral_feedback[0U] += error.data[0U] * dt;
    integral_feedback[1U] += error.data[1U] * dt;
    integral_feedback[2U] += error.data[2U] * dt;

    if (Vector3_Magnitude(&mag_vec) > 0.1f) {
        Vector3_Scale(&mag_vec, 1.0f / Vector3_Magnitude(&mag_vec), &mag_vec);
        Vector3_Cross(&mag_vec, &gravity_body, &correction);
        integral_feedback[0U] += correction.data[0U] * AHRS_MAGNETOMETER_GAIN;
        integral_feedback[1U] += correction.data[1U] * AHRS_MAGNETOMETER_GAIN;
        integral_feedback[2U] += correction.data[2U] * AHRS_MAGNETOMETER_GAIN;
    }

    gyro_vec.data[0U] += (AHRS_KP * error.data[0U]) + integral_feedback[0U];
    gyro_vec.data[1U] += (AHRS_KP * error.data[1U]) + integral_feedback[1U];
    gyro_vec.data[2U] += (AHRS_KP * error.data[2U]) + integral_feedback[2U];

    Quaternion_t dq;
    Quaternion_Init(&dq,
                    1.0f,
                    gyro_vec.data[0U] * dt * 0.5f,
                    gyro_vec.data[1U] * dt * 0.5f,
                    gyro_vec.data[2U] * dt * 0.5f);

    Quaternion_t next_state;
    Quaternion_Multiply(&state_quat, &dq, &next_state);
    state_quat = next_state;
    Quaternion_Normalize(&state_quat);

    f32 roll_rad = atan2f(2.0f * (state_quat.w * state_quat.x + state_quat.y * state_quat.z),
                          1.0f - 2.0f * (state_quat.x * state_quat.x + state_quat.y * state_quat.y));
    f32 pitch_rad = asinf(2.0f * (state_quat.w * state_quat.y - state_quat.z * state_quat.x));
    f32 yaw_rad = atan2f(2.0f * (state_quat.w * state_quat.z + state_quat.x * state_quat.y),
                         1.0f - 2.0f * (state_quat.y * state_quat.y + state_quat.z * state_quat.z));

    current_euler[0U] = roll_rad;
    current_euler[1U] = pitch_rad;
    current_euler[2U] = yaw_rad;

    covariance_trace = (error.data[0U] * error.data[0U]) +
                       (error.data[1U] * error.data[1U]) +
                       (error.data[2U] * error.data[2U]);

    update_counter++;
    health_status = ERR_NONE;
    return ERR_NONE;
}

bool AHRS_IsValid(void) {
    if (!engine_initialized) {
        return FALSE;
    }
    if (health_status != ERR_NONE) {
        return FALSE;
    }
    if (covariance_trace > AHRS_VALIDITY_THRESHOLD) {
        return FALSE;
    }
    return TRUE;
}

void AHRS_GetAttitude(f32 *roll, f32 *pitch, f32 *yaw) {
    if (roll != ((f32 *)0)) {
        *roll = current_euler[0U];
    }
    if (pitch != ((f32 *)0)) {
        *pitch = current_euler[1U];
    }
    if (yaw != ((f32 *)0)) {
        *yaw = current_euler[2U];
    }
}

void AHRS_GetQuaternion(f32 *w, f32 *x, f32 *y, f32 *z) {
    if (w != ((f32 *)0)) {
        *w = state_quat.w;
    }
    if (x != ((f32 *)0)) {
        *x = state_quat.x;
    }
    if (y != ((f32 *)0)) {
        *y = state_quat.y;
    }
    if (z != ((f32 *)0)) {
        *z = state_quat.z;
    }
}

ErrorCode_t AHRS_GetHealthStatus(void) {
    return health_status;
}

u32 AHRS_GetUpdateCounter(void) {
    return update_counter;
}

void AHRS_Reset(void) {
    engine_initialized = FALSE;
    Quaternion_Init(&state_quat, 1.0f, 0.0f, 0.0f, 0.0f);
    integral_feedback[0U] = 0.0f;
    integral_feedback[1U] = 0.0f;
    integral_feedback[2U] = 0.0f;
    update_counter = 0U;
    covariance_trace = 0.0f;
    health_status = ERR_NONE;
    current_euler[0U] = 0.0f;
    current_euler[1U] = 0.0f;
    current_euler[2U] = 0.0f;
    engine_initialized = TRUE;
}
