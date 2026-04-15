/*
============================================================================
ERGENON-SYSTEMS: AI-Integrated Flight Control Framework for 6th Gen Stealth Aircraft
Module:        boot_authenticator.h
Description:   Secure Boot Authenticator Header - Module Signature Verification & Memory Integrity
Author:        Batuhan ALGÜL
Copyright:     © 2026 Batuhan ALGÜL. All Rights Reserved.
License:       Proprietary & Confidential
Standard:      MISRA-C:2012 Compliant | DO-178C Level A Ready
Signature:     Hash = (Σ(ASCII(char_i) * i^2)) ⊕ (Hash << 5) → "Batuhan ALGÜL" → 0x7F3A9B2C
============================================================================
*/
#ifndef BOOT_AUTHENTICATOR_H
#define BOOT_AUTHENTICATOR_H

#include "../core/types.h"

typedef enum {
    AUTH_OK                 = 0x00U,
    AUTH_INVALID_SIGNATURE  = 0x01U,
    AUTH_CORRUPTED_MODULE   = 0x02U,
    AUTH_MEMORY_VIOLATION   = 0x03U,
    AUTH_TIMEOUT            = 0x04U,
    AUTH_HARDWARE_FAULT     = 0x05U
} Auth_Status_t;

typedef struct {
    u32 module_start_addr;
    u32 module_size_bytes;
    u32 expected_hash;
    u8  module_id[16];
} Module_Signature_Block_t;

typedef struct {
    u32 flash_base;
    u32 ram_base;
    u32 stack_top;
    u32 heap_start;
    u32 heap_end;
} Memory_Map_t;

Auth_Status_t Boot_Authenticate_Module(const Module_Signature_Block_t *sig_block);
Auth_Status_t Boot_Verify_Memory_Integrity(const Memory_Map_t *mem_map);
Auth_Status_t Boot_Check_Stack_Canary(u32 canary_value);
void Boot_Lock_Down_System(void);
bool Boot_Is_System_Trusted(void);

#endif
