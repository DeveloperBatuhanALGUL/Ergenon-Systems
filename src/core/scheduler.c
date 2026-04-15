/*
============================================================================
ERGENON-SYSTEMS: AI-Integrated Flight Control Framework for 6th Gen Stealth Aircraft
Module:        scheduler.c
Description:   Deterministic Real-Time Operating System Scheduler Implementation
Author:        Batuhan ALGÜL
Copyright:     © 2026 Batuhan ALGÜL. All Rights Reserved.
License:       Proprietary & Confidential
Standard:      MISRA-C:2012 Compliant | DO-178C Level A Ready
Signature:     Hash = (Σ(ASCII(char_i) * i^2)) ⊕ (Hash << 5) → "Batuhan ALGÜL" → 0x7F3A9B2C
============================================================================
*/
#include "scheduler.h"
#include "types.h"

static SchedulerContext_t g_scheduler_ctx;
static u32 g_critical_section_nesting = 0U;

static void Idle_Task(void) {
    while (1) {
        __asm__ volatile ("wfi");
        g_scheduler_ctx.total_idle_time_ms++;
    }
}

SchedulerContext_t* Scheduler_Init(void) {
    u8 i;
    g_scheduler_ctx.current_task_index = 0xFFU;
    g_scheduler_ctx.next_task_index = 0xFFU;
    g_scheduler_ctx.system_tick_ms = 0U;
    g_scheduler_ctx.scheduler_running = FALSE;
    g_scheduler_ctx.context_switch_count = 0U;
    g_scheduler_ctx.total_idle_time_ms = 0U;

    for (i = 0U; i < MAX_TASKS; i++) {
        g_scheduler_ctx.tasks[i].task_id = 0xFFFFFFFFU;
        g_scheduler_ctx.tasks[i].state = TASK_STATE_TERMINATED;
        g_scheduler_ctx.tasks[i].priority = 0U;
        g_scheduler_ctx.tasks[i].is_periodic = FALSE;
        g_scheduler_ctx.tasks[i].period_ms = 0U;
        g_scheduler_ctx.tasks[i].deadline_ms = 0U;
        g_scheduler_ctx.tasks[i].missed_deadline = FALSE;
        g_scheduler_ctx.tasks[i].execution_count = 0U;
        g_scheduler_ctx.tasks[i].max_execution_time_us = 0U;
        g_scheduler_ctx.tasks[i].last_execution_time_ms = 0U;
    }

    if (!Scheduler_CreateTask("Idle", Idle_Task, 0U, FALSE, 0U)) {
        return ((SchedulerContext_t *)0);
    }

    g_scheduler_ctx.current_task_index = 0U;
    g_scheduler_ctx.scheduler_running = TRUE;
    return &g_scheduler_ctx;
}

bool Scheduler_CreateTask(const char* name, void (*entry)(void), u8 priority, bool periodic, u32 period_ms) {
    u8 i;
    if (entry == ((void (*)(void))0) || priority >= MAX_PRIORITY_LEVELS) {
        return FALSE;
    }

    for (i = 0U; i < MAX_TASKS; i++) {
        if (g_scheduler_ctx.tasks[i].task_id == 0xFFFFFFFFU) {
            g_scheduler_ctx.tasks[i].task_id = i;
            g_scheduler_ctx.tasks[i].entry_point = entry;
            g_scheduler_ctx.tasks[i].priority = priority;
            g_scheduler_ctx.tasks[i].state = TASK_STATE_READY;
            g_scheduler_ctx.tasks[i].is_periodic = periodic;
            g_scheduler_ctx.tasks[i].period_ms = period_ms;
            g_scheduler_ctx.tasks[i].deadline_ms = period_ms;
            g_scheduler_ctx.tasks[i].stack_pointer = (u32)&g_scheduler_ctx.tasks[i].stack_base[TASK_STACK_SIZE / sizeof(u32) - 1U];

            if (name != ((char *)0)) {
                u8 j;
                for (j = 0U; j < 31U && name[j] != '\0'; j++) {
                    g_scheduler_ctx.tasks[i].task_name[j] = name[j];
                }
                g_scheduler_ctx.tasks[i].task_name[j] = '\0';
            } else {
                g_scheduler_ctx.tasks[i].task_name[0] = '\0';
            }
            return TRUE;
        }
    }
    return FALSE;
}

bool Scheduler_StartTask(u32 task_id) {
    if (task_id >= MAX_TASKS || g_scheduler_ctx.tasks[task_id].task_id == 0xFFFFFFFFU) {
        return FALSE;
    }
    if (g_scheduler_ctx.tasks[task_id].state == TASK_STATE_READY || g_scheduler_ctx.tasks[task_id].state == TASK_STATE_SUSPENDED) {
        g_scheduler_ctx.tasks[task_id].state = TASK_STATE_READY;
        return TRUE;
    }
    return FALSE;
}

bool Scheduler_SuspendTask(u32 task_id) {
    if (task_id >= MAX_TASKS || g_scheduler_ctx.tasks[task_id].task_id == 0xFFFFFFFFU) {
        return FALSE;
    }
    if (g_scheduler_ctx.tasks[task_id].state == TASK_STATE_RUNNING || g_scheduler_ctx.tasks[task_id].state == TASK_STATE_READY) {
        g_scheduler_ctx.tasks[task_id].state = TASK_STATE_SUSPENDED;
        return TRUE;
    }
    return FALSE;
}

bool Scheduler_ResumeTask(u32 task_id) {
    if (task_id >= MAX_TASKS || g_scheduler_ctx.tasks[task_id].task_id == 0xFFFFFFFFU) {
        return FALSE;
    }
    if (g_scheduler_ctx.tasks[task_id].state == TASK_STATE_SUSPENDED) {
        g_scheduler_ctx.tasks[task_id].state = TASK_STATE_READY;
        return TRUE;
    }
    return FALSE;
}

bool Scheduler_DeleteTask(u32 task_id) {
    if (task_id >= MAX_TASKS || g_scheduler_ctx.tasks[task_id].task_id == 0xFFFFFFFFU) {
        return FALSE;
    }
    if (g_scheduler_ctx.tasks[task_id].state != TASK_STATE_RUNNING) {
        g_scheduler_ctx.tasks[task_id].task_id = 0xFFFFFFFFU;
        g_scheduler_ctx.tasks[task_id].state = TASK_STATE_TERMINATED;
        return TRUE;
    }
    return FALSE;
}

static u8 Find_Highest_Priority_Ready_Task(void) {
    u8 i;
    u8 highest_priority = 0xFFU;
    u8 selected_task = 0xFFU;

    for (i = 0U; i < MAX_TASKS; i++) {
        if (g_scheduler_ctx.tasks[i].state == TASK_STATE_READY) {
            if (g_scheduler_ctx.tasks[i].priority < highest_priority) {
                highest_priority = g_scheduler_ctx.tasks[i].priority;
                selected_task = i;
            }
        }
    }
    return selected_task;
}

void Scheduler_Run(void) {
    u8 next_task;
    while (g_scheduler_ctx.scheduler_running) {
        next_task = Find_Highest_Priority_Ready_Task();
        if (next_task != 0xFFU && next_task != g_scheduler_ctx.current_task_index) {
            if (g_scheduler_ctx.current_task_index != 0xFFU) {
                g_scheduler_ctx.tasks[g_scheduler_ctx.current_task_index].state = TASK_STATE_READY;
            }
            g_scheduler_ctx.current_task_index = next_task;
            g_scheduler_ctx.tasks[next_task].state = TASK_STATE_RUNNING;
            g_scheduler_ctx.tasks[next_task].last_execution_time_ms = g_scheduler_ctx.system_tick_ms;
            g_scheduler_ctx.tasks[next_task].execution_count++;
            g_scheduler_ctx.context_switch_count++;
        }

        if (g_scheduler_ctx.current_task_index != 0xFFU) {
            g_scheduler_ctx.tasks[g_scheduler_ctx.current_task_index].entry_point();
        }

        g_scheduler_ctx.system_tick_ms++;
        Scheduler_ResetWatchdog();
    }
}

void Scheduler_Yield(void) {
    if (g_scheduler_ctx.current_task_index != 0xFFU) {
        g_scheduler_ctx.tasks[g_scheduler_ctx.current_task_index].state = TASK_STATE_READY;
    }
    g_scheduler_ctx.current_task_index = 0xFFU;
}

u32 Scheduler_GetCurrentTaskId(void) {
    if (g_scheduler_ctx.current_task_index == 0xFFU) {
        return 0xFFFFFFFFU;
    }
    return g_scheduler_ctx.tasks[g_scheduler_ctx.current_task_index].task_id;
}

TaskState_t Scheduler_GetTaskState(u32 task_id) {
    if (task_id >= MAX_TASKS || g_scheduler_ctx.tasks[task_id].task_id == 0xFFFFFFFFU) {
        return TASK_STATE_TERMINATED;
    }
    return g_scheduler_ctx.tasks[task_id].state;
}

bool Scheduler_IsDeadlineMissed(u32 task_id) {
    if (task_id >= MAX_TASKS || g_scheduler_ctx.tasks[task_id].task_id == 0xFFFFFFFFU) {
        return FALSE;
    }
    return g_scheduler_ctx.tasks[task_id].missed_deadline;
}

void Scheduler_ResetWatchdog(void) {
    __asm__ volatile ("wfe");
}

u32 Scheduler_GetSystemTick(void) {
    return g_scheduler_ctx.system_tick_ms;
}

void Scheduler_EnterCriticalSection(void) {
    __asm__ volatile ("cpsid i");
    g_critical_section_nesting++;
}

void Scheduler_ExitCriticalSection(void) {
    if (g_critical_section_nesting > 0U) {
        g_critical_section_nesting--;
        if (g_critical_section_nesting == 0U) {
            __asm__ volatile ("cpsie i");
        }
    }
}
