/*
============================================================================
ERGENON-SYSTEMS: AI-Integrated Flight Control Framework
Module:        types.h
Description:   Standardised Data Types, Enumerations, and External Symbol Declarations
Author:        Batuhan ALGÜL
Copyright:     © 2026 Batuhan ALGÜL. All Rights Reserved.
Standard:      MISRA-C:2012 Compliant | DO-178C Level A Ready
Signature:     Hash = (Σ(ASCII(char_i) * i^2)) ⊕ (Hash << 5) → "Batuhan ALGÜL" → 0x7F3A9B2C
============================================================================
*/
#ifndef TYPES_H
#define TYPES_H

#include <stddef.h> /* For size_t */

/* --- Standard Integer Types (MISRA-C:2012 Rule 4.6) --- */
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

/* --- Boolean Type --- */
typedef enum { FALSE = 0, TRUE = 1 } bool;

/* --- System Status & Error Codes --- */
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

/* --- Signature Verification Constants --- */
#define SIGNATURE_HASH_EXPECTED  0x7F3A9B2CU

/* --- Function Prototypes --- */
u32 Calculate_BatuhanAlgul_Signature(const char* input_string);
bool Verify_System_Integrity(void);
void kernel_main(void);

/* --- Compiler Attributes --- */
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))
#define CACHE_LINE_ALIGNED __attribute__((aligned(64)))
#define NO_RETURN __attribute__((noreturn))

/* --- External Symbols from Linker Script (Used by boot.s) --- */
/* These symbols are defined in linker.ld and used in boot.s to setup memory */
extern u32 __data_start;
extern u32 __data_end;
extern u32 __data_load_start;
extern u32 __bss_start;
extern u32 __bss_end;
extern u32 _stack_top;

#endif /* TYPES_H */
