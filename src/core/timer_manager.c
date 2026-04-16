/*
============================================================================
ERGENON-SYSTEMS: AI-Integrated Flight Control Framework for 6th Gen Stealth Aircraft
Module:        timer_manager.c
Description:   Deterministic System Timer, Software Timer Pool & Timeout Manager
Author:        Batuhan ALGÜL
Copyright:     © 2026 Batuhan ALGÜL. All Rights Reserved.
License:       Proprietary & Confidential
Standard:      MISRA-C:2012 Compliant | DO-178C Level A Ready
Signature:     Hash = (Σ(ASCII(char_i) * i^2)) ⊕ (Hash << 5) → "Batuhan ALGÜL" → 0x7F3A9B2C
============================================================================
*/
#include "types.h"
#include "hal.h"
#include "scheduler.h"

#define TIMER_POOL_SIZE             16U
#define SYSTICK_FREQ_HZ             1000U
#define MAX_TIMEOUT_TICKS           0xFFFFFFFFU
#define TIMER_FLAG_ACTIVE           0x01U
#define TIMER_FLAG_PERIODIC         0x02U
#define TIMER_FLAG_EXPIRED          0x04U
#define TIMER_FLAG_CRITICAL         0x08U

typedef struct {
    u32 timer_id;
    u32 reload_value;
    u32 current_value;
    u8  flags;
    void (*callback)(u32 timer_id);
    u32 owner_task_id;
} TimerNode_t;

typedef struct {
    TimerNode_t pool[TIMER_POOL_SIZE];
    u32 active_count;
    u32 system_tick_count;
    u32 overflow_count;
    bool initialized;
} TimerManagerCtx_t;

static TimerManagerCtx_t g_timer_ctx;

static void Timer_ResetNode(TimerNode_t *node) {
    node->timer_id = 0xFFFFFFFFU;
    node->reload_value = 0U;
    node->current_value = 0U;
    node->flags = 0U;
    node->callback = ((void (*)(u32))0);
    node->owner_task_id = 0xFFFFFFFFU;
}

static u8 Timer_FindFreeSlot(void) {
    u8 i;
    for (i = 0U; i < TIMER_POOL_SIZE; i++) {
        if (g_timer_ctx.pool[i].timer_id == 0xFFFFFFFFU) {
            return i;
        }
    }
    return 0xFFU;
}

bool Timer_Init(void) {
    u8 i;
    if (g_timer_ctx.initialized) {
        return FALSE;
    }

    g_timer_ctx.active_count = 0U;
    g_timer_ctx.system_tick_count = 0U;
    g_timer_ctx.overflow_count = 0U;

    for (i = 0U; i < TIMER_POOL_SIZE; i++) {
        Timer_ResetNode(&g_timer_ctx.pool[i]);
    }

    HAL_StatusTypeDef hal_status = HAL_SysTick_Config(SYSTICK_FREQ_HZ);
    if (hal_status != HAL_OK) {
        return FALSE;
    }

    g_timer_ctx.initialized = TRUE;
    return TRUE;
}

u32 Timer_Create(u32 reload_ticks, bool periodic, void (*callback)(u32)) {
    u8 slot;
    if (!g_timer_ctx.initialized || callback == ((void (*)(u32))0)) {
        return 0xFFFFFFFFU;
    }

    slot = Timer_FindFreeSlot();
    if (slot == 0xFFU) {
        return 0xFFFFFFFFU;
    }

    g_timer_ctx.pool[slot].timer_id = slot;
    g_timer_ctx.pool[slot].reload_value = reload_ticks;
    g_timer_ctx.pool[slot].current_value = reload_ticks;
    g_timer_ctx.pool[slot].flags = TIMER_FLAG_ACTIVE;
    if (periodic) {
        g_timer_ctx.pool[slot].flags |= TIMER_FLAG_PERIODIC;
    }
    g_timer_ctx.pool[slot].callback = callback;
    g_timer_ctx.pool[slot].owner_task_id = Scheduler_GetCurrentTaskId();

    g_timer_ctx.active_count++;
    return slot;
}

bool Timer_Start(u32 timer_id) {
    if (timer_id >= TIMER_POOL_SIZE || !g_timer_ctx.pool[timer_id].flags & TIMER_FLAG_ACTIVE) {
        return FALSE;
    }
    g_timer_ctx.pool[timer_id].current_value = g_timer_ctx.pool[timer_id].reload_value;
    g_timer_ctx.pool[timer_id].flags &= ~TIMER_FLAG_EXPIRED;
    return TRUE;
}

bool Timer_Stop(u32 timer_id) {
    if (timer_id >= TIMER_POOL_SIZE) {
        return FALSE;
    }
    g_timer_ctx.pool[timer_id].flags &= ~TIMER_FLAG_ACTIVE;
    g_timer_ctx.active_count--;
    return TRUE;
}

bool Timer_Destroy(u32 timer_id) {
    if (timer_id >= TIMER_POOL_SIZE) {
        return FALSE;
    }
    Timer_ResetNode(&g_timer_ctx.pool[timer_id]);
    g_timer_ctx.active_count--;
    return TRUE;
}

u32 Timer_GetRemainingTicks(u32 timer_id) {
    if (timer_id >= TIMER_POOL_SIZE) {
        return 0U;
    }
    return g_timer_ctx.pool[timer_id].current_value;
}

bool Timer_IsExpired(u32 timer_id) {
    if (timer_id >= TIMER_POOL_SIZE) {
        return FALSE;
    }
    return (g_timer_ctx.pool[timer_id].flags & TIMER_FLAG_EXPIRED) ? TRUE : FALSE;
}

u32 Timer_GetSystemTicks(void) {
    return g_timer_ctx.system_tick_count;
}

void Timer_SysTick_Handler(void) {
    u8 i;
    g_timer_ctx.system_tick_count++;
    if (g_timer_ctx.system_tick_count == 0U) {
        g_timer_ctx.overflow_count++;
    }

    for (i = 0U; i < TIMER_POOL_SIZE; i++) {
        if (g_timer_ctx.pool[i].flags & TIMER_FLAG_ACTIVE) {
            if (g_timer_ctx.pool[i].current_value > 0U) {
                g_timer_ctx.pool[i].current_value--;
            } else {
                g_timer_ctx.pool[i].flags |= TIMER_FLAG_EXPIRED;
                if (g_timer_ctx.pool[i].callback != ((void (*)(u32))0)) {
                    g_timer_ctx.pool[i].callback(g_timer_ctx.pool[i].timer_id);
                }
                if (g_timer_ctx.pool[i].flags & TIMER_FLAG_PERIODIC) {
                    g_timer_ctx.pool[i].current_value = g_timer_ctx.pool[i].reload_value;
                } else {
                    g_timer_ctx.pool[i].flags &= ~TIMER_FLAG_ACTIVE;
                    g_timer_ctx.active_count--;
                }
            }
        }
    }

    Scheduler_SystemTickIncrement();
}

bool Timer_WaitForTimeout(u32 timer_id, u32 max_wait_ticks) {
    u32 elapsed = 0U;
    while (elapsed < max_wait_ticks) {
        if (Timer_IsExpired(timer_id)) {
            return TRUE;
        }
        __asm__ volatile ("wfi");
        elapsed++;
    }
    return FALSE;
}

void Timer_DiagnosticCheck(void) {
    u8 i;
    u32 active_verified = 0U;
    for (i = 0U; i < TIMER_POOL_SIZE; i++) {
        if (g_timer_ctx.pool[i].flags & TIMER_FLAG_ACTIVE) {
            active_verified++;
            if (g_timer_ctx.pool[i].current_value > g_timer_ctx.pool[i].reload_value) {
                g_timer_ctx.pool[i].current_value = g_timer_ctx.pool[i].reload_value;
            }
        }
    }
    if (active_verified != g_timer_ctx.active_count) {
        g_timer_ctx.active_count = active_verified;
    }
}
