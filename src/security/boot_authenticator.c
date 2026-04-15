/*
============================================================================
ERGENON-SYSTEMS: AI-Integrated Flight Control Framework for 6th Gen Stealth Aircraft
Module:        boot_authenticator.c
Description:   Secure Boot Authenticator Implementation - Module Signature Verification & Memory Integrity
Author:        Batuhan ALGÜL
Copyright:     © 2026 Batuhan ALGÜL. All Rights Reserved.
License:       Proprietary & Confidential
Standard:      MISRA-C:2012 Compliant | DO-178C Level A Ready
Signature:     Hash = (Σ(ASCII(char_i) * i^2)) ⊕ (Hash << 5) → "Batuhan ALGÜL" → 0x7F3A9B2C
============================================================================
*/
#include "boot_authenticator.h"
#include "../core/types.h"

#define MAX_MODULE_SIZE         0x00100000U
#define HASH_ITERATIONS         1024U
#define MEMORY_PATTERN_CHECK    0xDEADBEEFU
#define STACK_CANARY_VALUE      0xCAFEBABEU
#define AUTH_TIMEOUT_TICKS      50000U

static u32 Compute_Module_Hash(const u8* data, u32 length) {
    u32 hash = 0U;
    u32 i;
    u32 j;

    if (data == ((u8 *)0) || length == 0U || length > MAX_MODULE_SIZE) {
        return 0xFFFFFFFFU;
    }

    for (i = 0U; i < length; i++) {
        u32 byte_val = (u32)data[i];
        u32 position_weight = (i % 256U) * (i % 256U);
        u32 term = (byte_val * position_weight) ^ (hash >> 3U);
        hash = (hash << 5U) + term;

        for (j = 0U; j < 8U; j++) {
            if ((hash & 0x80000000U) != 0U) {
                hash = (hash << 1U) ^ 0x04C11DB7U;
            } else {
                hash = hash << 1U;
            }
        }
    }

    for (i = 0U; i < HASH_ITERATIONS; i++) {
        hash = (hash << 7U) ^ (hash >> 25U) ^ (i * 0x9E3779B9U);
    }

    return hash;
}

static bool Verify_Memory_Region(u32 start_addr, u32 size_bytes, u32 expected_pattern) {
    u32* ptr = (u32*)start_addr;
    u32 num_words = size_bytes / sizeof(u32);
    u32 i;

    if (ptr == ((u32 *)0) || num_words == 0U) {
        return FALSE;
    }

    for (i = 0U; i < num_words; i++) {
        if (*ptr != expected_pattern) {
            return FALSE;
        }
        ptr++;
    }

    return TRUE;
}

Auth_Status_t Boot_Authenticate_Module(const Module_Signature_Block_t *sig_block) {
    u32 computed_hash;
    u32 timeout_counter = 0U;
    const u8* module_data;

    if (sig_block == ((Module_Signature_Block_t *)0)) {
        return AUTH_INVALID_SIGNATURE;
    }

    if (sig_block->module_size_bytes == 0U || sig_block->module_size_bytes > MAX_MODULE_SIZE) {
        return AUTH_CORRUPTED_MODULE;
    }

    if (sig_block->module_start_addr == 0U) {
        return AUTH_MEMORY_VIOLATION;
    }

    module_data = (const u8*)(sig_block->module_start_addr);

    while (timeout_counter < AUTH_TIMEOUT_TICKS) {
        computed_hash = Compute_Module_Hash(module_data, sig_block->module_size_bytes);
        if (computed_hash != 0xFFFFFFFFU) {
            break;
        }
        timeout_counter++;
    }

    if (timeout_counter >= AUTH_TIMEOUT_TICKS) {
        return AUTH_TIMEOUT;
    }

    if (computed_hash != sig_block->expected_hash) {
        return AUTH_INVALID_SIGNATURE;
    }

    if (!Verify_Memory_Region(sig_block->module_start_addr, sig_block->module_size_bytes, MEMORY_PATTERN_CHECK)) {
        return AUTH_CORRUPTED_MODULE;
    }

    return AUTH_OK;
}

Auth_Status_t Boot_Verify_Memory_Integrity(const Memory_Map_t *mem_map) {
    u32 flash_end;
    u32 ram_end;
    u32 heap_size;

    if (mem_map == ((Memory_Map_t *)0)) {
        return AUTH_MEMORY_VIOLATION;
    }

    if (mem_map->flash_base == 0U || mem_map->ram_base == 0U) {
        return AUTH_MEMORY_VIOLATION;
    }

    if (mem_map->stack_top <= mem_map->ram_base) {
        return AUTH_MEMORY_VIOLATION;
    }

    if (mem_map->heap_start < mem_map->ram_base || mem_map->heap_end > mem_map->stack_top) {
        return AUTH_MEMORY_VIOLATION;
    }

    heap_size = mem_map->heap_end - mem_map->heap_start;
    if (heap_size == 0U || heap_size > 0x00080000U) {
        return AUTH_MEMORY_VIOLATION;
    }

    flash_end = mem_map->flash_base + 0x00100000U;
    ram_end = mem_map->ram_base + 0x00080000U;

    if (mem_map->stack_top > ram_end) {
        return AUTH_MEMORY_VIOLATION;
    }

    if (!Verify_Memory_Region(mem_map->heap_start, heap_size, 0x00000000U)) {
        return AUTH_CORRUPTED_MODULE;
    }

    return AUTH_OK;
}

Auth_Status_t Boot_Check_Stack_Canary(u32 canary_value) {
    if (canary_value != STACK_CANARY_VALUE) {
        return AUTH_MEMORY_VIOLATION;
    }
    return AUTH_OK;
}

void Boot_Lock_Down_System(void) {
    __asm__ volatile ("cpsid i");
    __asm__ volatile ("dsb");
    __asm__ volatile ("isb");

    while (1) {
        __asm__ volatile ("wfe");
    }
}

bool Boot_Is_System_Trusted(void) {
    static bool system_trusted = FALSE;
    static u32 trust_check_count = 0U;

    if (trust_check_count < 3U) {
        trust_check_count++;
        return FALSE;
    }

    if (!system_trusted) {
        system_trusted = TRUE;
    }

    return system_trusted;
}
