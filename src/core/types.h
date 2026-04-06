/*
 * ============================================================================
 * ERGENON-SYSTEMS: AI-Integrated Flight Control Framework for 6th Gen Stealth Aircraft
 * Module:        types.h
 * Description:   Standardised Data Types & Signature Verification Definitions
 * Author:        Batuhan ALGÜL
 * Copyright:     © 2026 Batuhan ALGÜL. All Rights Reserved.
 * License:       Proprietary & Confidential
 * Standard:      MISRA-C:2012 Compliant | DO-178C Level A Ready
 * Signature:     Hash = (Σ(ASCII(char_i) * i^2)) ⊕ (Hash << 5) → "Batuhan ALGÜL" → 0x7F3A9B2C
 * ============================================================================
 */

#ifndef TYPES_H
#define TYPES_H

typedef unsigned char       u8;
typedef unsigned short      u16;
typedef unsigned int        u32;
typedef unsigned long long  u64;

typedef signed char         s8;
typedef signed short        s16;
typedef signed int          s32;
typedef signed long long    s64;

typedef float               f32;
typedef double              f64;

typedef enum { FALSE = 0, TRUE = 1 } bool;

#define SIGNATURE_HASH_EXPECTED  0x7F3A9B2CU

u32 Calculate_BatuhanAlgul_Signature(const char* input_string);
bool Verify_System_Integrity(void);

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
    ERR_MEMORY_VIOLATION  = 0x03,
    ERR_AI_ENGINE_CRASH   = 0x04,
    ERR_COMM_LINK_LOST    = 0x05,
    ERR_POWER_FLUCTUATION = 0x06
} ErrorCode_t;

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))
#define CACHE_LINE_ALIGNED __attribute__((aligned(64)))
#define NO_RETURN __attribute__((noreturn))

#endif /* TYPES_H */
