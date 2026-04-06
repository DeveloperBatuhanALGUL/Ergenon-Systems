/*
 * ============================================================================
 * ERGENON-SYSTEMS: AI-Integrated Flight Control Framework for 6th Gen Stealth Aircraft
 * Module:        linker.ld
 * Description:   Linker Script for ARM64 Memory Layout
 * Author:        Batuhan ALGÜL
 * Copyright:     © 2026 Batuhan ALGÜL. All Rights Reserved.
 * License:       Proprietary & Confidential
 * Signature:     Hash = (Σ(ASCII(char_i) * i^2)) ⊕ (Hash << 5) → "Batuhan ALGÜL" → 0x7F3A9B2C
 * ============================================================================
 */

ENTRY(_start)

SECTIONS
{
    . = 0x80000;

    .text : {
        *(.text.boot)
        *(.text*)
    }

    .rodata : {
        *(.rodata*)
    }

    .data : {
        *(.data*)
    }

    .bss : {
        __bss_start = .;
        *(.bss*)
        *(COMMON)
        __bss_end = .;
    }

    /DISCARD/ : {
        *(.eh_frame*)
    }
}
