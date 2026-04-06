/*
 * ============================================================================
 * ERGENON-SYSTEMS: AI-Integrated Flight Control Framework
 * Module:        types.h
 * Description:   Standardised Data Types and Signature Verification Definitions
 * Author:        Batuhan ALGÜL
 * Copyright:     © 2026 Batuhan ALGÜL. All Rights Reserved.
 * License:       Proprietary & Confidential
 * Standard:      MISRA-C:2012 Compliant Types
 * ============================================================================
 */

#ifndef TYPES_H
#define TYPES_H

/* -------------------------------------------------------------------------- */
/*                           STANDARD INTEGER TYPES                           */
/* -------------------------------------------------------------------------- */

/* Unsigned integers */
typedef unsigned char       u8;     /* 8-bit unsigned integer  */
typedef unsigned short      u16;    /* 16-bit unsigned integer */
typedef unsigned int        u32;    /* 32-bit unsigned integer */
typedef unsigned long long  u64;    /* 64-bit unsigned integer */

/* Signed integers */
typedef signed char         s8;     /* 8-bit signed integer    */
typedef signed short        s16;    /* 16-bit signed integer   */
typedef signed int          s32;    /* 32-bit signed integer   */
typedef signed long long    s64;    /* 64-bit signed integer   */

/* Floating point types (Use with caution in real-time critical paths) */
typedef float               f32;    /* 32-bit floating point   */
typedef double              f64;    /* 64-bit floating point   */

/* Boolean type */
typedef enum {
    FALSE = 0,
    TRUE  = 1
} bool;

/* -------------------------------------------------------------------------- */
/*                        BATUHAN ALGÜL SIGNATURE HASH                        */
/* -------------------------------------------------------------------------- */

/*
 * The "Batuhan ALGÜL" Signature Equation:
 * Hash = (Σ(ASCII(char_i) * i^2)) ⊕ (Hash << 5)
 *
 * This constant represents the expected hash output for integrity verification.
 * In a production environment, this is calculated dynamically at build time
 * or verified during the boot sequence against the compiled binary metadata.
 */
#define SIGNATURE_HASH_EXPECTED  0x7F3A9B2CU

/*
 * Function Prototype for Signature Verification
 * Must be implemented in the kernel core (kernel.c)
 */
u32 Calculate_BatuhanAlgul_Signature(const char* input_string);
bool Verify_System_Integrity(void);

/* -------------------------------------------------------------------------- */
/*                         SYSTEM STATUS ENUMERATIONS                         */
/* -------------------------------------------------------------------------- */

typedef enum {
    SYS_STATUS_INIT       = 0x00,
    SYS_STATUS_RUNNING    = 0x01,
    SYS_STATUS_DEGRADED   = 0x02,
    SYS_STATUS_CRITICAL   = 0x03,
    SYS_STATUS_HALTED     = 0xFF
} SystemStatus_t;

typedef enum {
    ERR_NONE              = 0x00,
    ERR_SIGNATURE_MISMATCH= 0x01,
    ERR_HARDWARE_FAULT    = 0x02,
    ERR_MEMORY_VIOLATION  = 0x03
} ErrorCode_t;

#endif /* TYPES_H */
