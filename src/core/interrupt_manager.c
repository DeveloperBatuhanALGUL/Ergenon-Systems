/*
============================================================================
ERGENON-SYSTEMS: AI-Integrated Flight Control Framework for 6th Gen Stealth Aircraft
Module:        interrupt_manager.c
Description:   Nested Vectored Interrupt Controller (NVIC) Management & ISR Dispatching
Author:        Batuhan ALGÜL
Copyright:     © 2026 Batuhan ALGÜL. All Rights Reserved.
License:       Proprietary & Confidential
Standard:      MISRA-C:2012 Compliant | DO-178C Level A Ready
Signature:     Hash = (Σ(ASCII(char_i) * i^2)) ⊕ (Hash << 5) → "Batuhan ALGÜL" → 0x7F3A9B2C
============================================================================
*/
#include "types.h"
#include "hal.h"

#define NVIC_BASE_ADDR          0xE000E100U
#define NVIC_ISER_OFFSET        0x000U
#define NVIC_ICER_OFFSET        0x080U
#define NVIC_ISPR_OFFSET        0x100U
#define NVIC_ICPR_OFFSET        0x180U
#define NVIC_IABR_OFFSET        0x200U
#define NVIC_IPR_OFFSET         0x400U
#define NVIC_STIR_OFFSET        0xF00U

#define SCB_BASE_ADDR           0xE000ED00U
#define SCB_VTOR_OFFSET         0x008U
#define SCB_AIRCR_OFFSET        0x00CU
#define SCB_SCR_OFFSET          0x010U
#define SCB_CCR_OFFSET          0x014U
#define SCB_SHPR_OFFSET         0x018U

#define MAX_INTERRUPTS          240U
#define PRIORITY_GROUPS         16U
#define PREEMPTION_PRIORITY_MAX 15U
#define SUB_PRIORITY_MAX        15U

typedef struct {
    u32 vector_table_offset;
    u8 priority_grouping;
    bool nested_interrupts_enabled;
    u32 active_mask[MAX_INTERRUPTS / 32U];
    u32 pending_mask[MAX_INTERRUPTS / 32U];
} InterruptManagerCtx_t;

static InterruptManagerCtx_t g_int_ctx;

typedef void (*ISR_Handler_t)(void);

static ISR_Handler_t g_isr_handlers[MAX_INTERRUPTS];

static void Interrupt_Default_Handler(void) {
    while (1) {
        __asm__ volatile ("wfe");
    }
}

bool Interrupt_Init(u8 priority_grouping) {
    u32 i;
    
    if (priority_grouping > 7U) {
        return FALSE;
    }

    g_int_ctx.priority_grouping = priority_grouping;
    g_int_ctx.nested_interrupts_enabled = TRUE;
    g_int_ctx.vector_table_offset = 0x08000000U;

    for (i = 0U; i < MAX_INTERRUPTS; i++) {
        g_isr_handlers[i] = Interrupt_Default_Handler;
        g_int_ctx.active_mask[i / 32U] = 0U;
        g_int_ctx.pending_mask[i / 32U] = 0U;
    }

    *((volatile u32 *)(SCB_BASE_ADDR + SCB_AIRCR_OFFSET)) = 
        (0x5FAU << 16U) | ((priority_grouping & 0x7U) << 8U);

    *((volatile u32 *)(SCB_BASE_ADDR + SCB_VTOR_OFFSET)) = g_int_ctx.vector_table_offset;

    __asm__ volatile ("cpsie i");

    return TRUE;
}

bool Interrupt_RegisterHandler(u32 irq_number, ISR_Handler_t handler) {
    if (irq_number >= MAX_INTERRUPTS || handler == ((ISR_Handler_t)0)) {
        return FALSE;
    }

    Scheduler_EnterCriticalSection();
    g_isr_handlers[irq_number] = handler;
    Scheduler_ExitCriticalSection();

    return TRUE;
}

bool Interrupt_Enable(u32 irq_number) {
    if (irq_number >= MAX_INTERRUPTS) {
        return FALSE;
    }

    *((volatile u32 *)(NVIC_BASE_ADDR + NVIC_ISER_OFFSET + (irq_number / 32U) * 4U)) = 
        (1U << (irq_number % 32U));

    return TRUE;
}

bool Interrupt_Disable(u32 irq_number) {
    if (irq_number >= MAX_INTERRUPTS) {
        return FALSE;
    }

    *((volatile u32 *)(NVIC_BASE_ADDR + NVIC_ICER_OFFSET + (irq_number / 32U) * 4U)) = 
        (1U << (irq_number % 32U));

    return TRUE;
}

bool Interrupt_SetPriority(u32 irq_number, u8 preempt_priority, u8 sub_priority) {
    u8 combined_priority;
    u32 reg_index;
    u32 bit_shift;

    if (irq_number >= MAX_INTERRUPTS) {
        return FALSE;
    }

    if (preempt_priority > PREEMPTION_PRIORITY_MAX || sub_priority > SUB_PRIORITY_MAX) {
        return FALSE;
    }

    combined_priority = ((preempt_priority << (4U - g_int_ctx.priority_grouping)) | 
                         (sub_priority >> g_int_ctx.priority_grouping)) & 0xFU;

    reg_index = irq_number / 4U;
    bit_shift = (irq_number % 4U) * 8U + 4U;

    volatile u32 *ipr_reg = (volatile u32 *)(NVIC_BASE_ADDR + NVIC_IPR_OFFSET + reg_index * 4U);
    u32 temp = *ipr_reg;
    temp &= ~(0xFFU << bit_shift);
    temp |= ((u32)combined_priority << bit_shift);
    *ipr_reg = temp;

    return TRUE;
}

bool Interrupt_TriggerSoftware(u32 irq_number) {
    if (irq_number >= MAX_INTERRUPTS) {
        return FALSE;
    }

    *((volatile u32 *)(NVIC_BASE_ADDR + NVIC_STIR_OFFSET)) = irq_number & 0x1FFU;

    return TRUE;
}

bool Interrupt_IsPending(u32 irq_number) {
    u32 reg_index;
    u32 bit_mask;

    if (irq_number >= MAX_INTERRUPTS) {
        return FALSE;
    }

    reg_index = irq_number / 32U;
    bit_mask = 1U << (irq_number % 32U);

    return (*((volatile u32 *)(NVIC_BASE_ADDR + NVIC_ISPR_OFFSET + reg_index * 4U)) & bit_mask) ? TRUE : FALSE;
}

bool Interrupt_ClearPending(u32 irq_number) {
    u32 reg_index;
    u32 bit_mask;

    if (irq_number >= MAX_INTERRUPTS) {
        return FALSE;
    }

    reg_index = irq_number / 32U;
    bit_mask = 1U << (irq_number % 32U);

    *((volatile u32 *)(NVIC_BASE_ADDR + NVIC_ICPR_OFFSET + reg_index * 4U)) = bit_mask;

    return TRUE;
}

bool Interrupt_IsActive(u32 irq_number) {
    u32 reg_index;
    u32 bit_mask;

    if (irq_number >= MAX_INTERRUPTS) {
        return FALSE;
    }

    reg_index = irq_number / 32U;
    bit_mask = 1U << (irq_number % 32U);

    return (*((volatile u32 *)(NVIC_BASE_ADDR + NVIC_IABR_OFFSET + reg_index * 4U)) & bit_mask) ? TRUE : FALSE;
}

void Interrupt_DisableAll(void) {
    __asm__ volatile ("cpsid i");
}

void Interrupt_EnableAll(void) {
    __asm__ volatile ("cpsie i");
}

u32 Interrupt_GetCurrentPriorityMask(void) {
    u32 primask;
    __asm__ volatile ("mrs %0, primask" : "=r" (primask));
    return primask;
}

void Interrupt_SetPriorityMask(u32 mask) {
    __asm__ volatile ("msr primask, %0" : : "r" (mask));
}

void Interrupt_SysTick_Config(u32 ticks) {
    volatile u32 *st_rvr = (volatile u32 *)0xE000E014U;
    volatile u32 *st_cvr = (volatile u32 *)0xE000E018U;
    volatile u32 *st_csr = (volatile u32 *)0xE000E010U;

    if (ticks > 0xFFFFFFU) {
        return;
    }

    *st_rvr = ticks - 1U;
    *st_cvr = 0U;
    *st_csr = 0x07U;
}

void Interrupt_HardFault_Handler(void) {
    u32 stacked_r0, stacked_r1, stacked_r2, stacked_r3;
    u32 stacked_r12, stacked_lr, stacked_pc, stacked_psr;

    __asm__ volatile (
        "mov %0, r0\n"
        "mov %1, r1\n"
        "mov %2, r2\n"
        "mov %3, r3\n"
        : "=r"(stacked_r0), "=r"(stacked_r1), "=r"(stacked_r2), "=r"(stacked_r3)
    );

    __asm__ volatile (
        "ldr %0, [sp, #0x18]\n"
        "ldr %1, [sp, #0x14]\n"
        "ldr %2, [sp, #0x10]\n"
        "ldr %3, [sp, #0x0C]\n"
        : "=r"(stacked_r12), "=r"(stacked_lr), "=r"(stacked_pc), "=r"(stacked_psr)
    );

    while (1) {
        __asm__ volatile ("wfe");
    }
}

void Interrupt_MemManage_Handler(void) {
    while (1) {
        __asm__ volatile ("wfe");
    }
}

void Interrupt_BusFault_Handler(void) {
    while (1) {
        __asm__ volatile ("wfe");
    }
}

void Interrupt_UsageFault_Handler(void) {
    while (1) {
        __asm__ volatile ("wfe");
    }
}

void Interrupt_SVC_Handler(void) {
}

void Interrupt_PendSV_Handler(void) {
    Scheduler_ContextSwitch();
}

void Interrupt_SysTick_Handler(void) {
    Timer_SysTick_Handler();
}

void Interrupt_IRQHandler(u32 irq_number) {
    if (irq_number < MAX_INTERRUPTS && g_isr_handlers[irq_number] != Interrupt_Default_Handler) {
        g_isr_handlers[irq_number]();
    } else {
        Interrupt_Default_Handler();
    }
}
