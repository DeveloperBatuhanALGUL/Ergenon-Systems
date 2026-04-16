/*
============================================================================
ERGENON-SYSTEMS: AI-Integrated Flight Control Framework for 6th Gen Stealth Aircraft
Module:        interrupt_manager.c
Description:   NVIC Interrupt Manager Implementation
Author:        Batuhan ALGÜL
Copyright:     © 2026 Batuhan ALGÜL. All Rights Reserved.
License:       Proprietary & Confidential
Standard:      MISRA-C:2012 Compliant | DO-178C Level A Ready
Signature:     Hash = (Σ(ASCII(char_i) * i^2)) ⊕ (Hash << 5) → "Batuhan ALGÜL" → 0x7F3A9B2C
============================================================================
*/

#include "interrupt_manager.h"
#include <string.h>

#define NVIC_BASE_ADDR 0xE000E100U
#define NVIC_ISER_OFFSET 0x000U
#define NVIC_ICER_OFFSET 0x080U
#define NVIC_IPR_OFFSET 0x400U

static ISR_Handler_t isr_table[240];

void IM_Init(void)
{
    memset(isr_table, 0, sizeof(isr_table));
    
    __asm__ volatile ("cpsid i");
    
    volatile u32* nvic_ipr = (volatile u32*)(NVIC_BASE_ADDR + NVIC_IPR_OFFSET);
    u8 i;
    for (i = 0U; i < 240U; i++)
    {
        nvic_ipr[i] = 0x00000000U;
    }
    
    __asm__ volatile ("cpsie i");
}

void IM_RegisterHandler(u8 irq_num, ISR_Handler_t handler)
{
    if (irq_num >= 240U)
    {
        return;
    }
    
    __asm__ volatile ("cpsid i");
    isr_table[irq_num] = handler;
    __asm__ volatile ("cpsie i");
}

void IM_SetPriority(u8 irq_num, u8 priority)
{
    if (irq_num >= 240U)
    {
        return;
    }
    
    volatile u8* nvic_ipr = (volatile u8*)(NVIC_BASE_ADDR + NVIC_IPR_OFFSET);
    nvic_ipr[irq_num] = (priority << 4U) & 0xF0U;
}

void IM_EnableIRQ(u8 irq_num)
{
    if (irq_num >= 240U)
    {
        return;
    }
    
    volatile u32* nvic_iser = (volatile u32*)(NVIC_BASE_ADDR + NVIC_ISER_OFFSET);
    nvic_iser[irq_num / 32U] |= (1U << (irq_num % 32U));
}

void IM_DisableIRQ(u8 irq_num)
{
    if (irq_num >= 240U)
    {
        return;
    }
    
    volatile u32* nvic_icer = (volatile u32*)(NVIC_BASE_ADDR + NVIC_ICER_OFFSET);
    nvic_icer[irq_num / 32U] |= (1U << (irq_num % 32U));
}

void IM_EnterCritical(void)
{
    __asm__ volatile ("cpsid i");
}

void IM_ExitCritical(void)
{
    __asm__ volatile ("cpsie i");
}
