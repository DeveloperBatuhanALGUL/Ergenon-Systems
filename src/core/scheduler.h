/*
============================================================================
ERGENON-SYSTEMS: AI-Integrated Flight Control Framework for 6th Gen Stealth Aircraft
Module:        scheduler.h
Description:   Deterministic Real-Time Operating System Scheduler Header
Author:        Batuhan ALGÜL
Copyright:     © 2026 Batuhan ALGÜL. All Rights Reserved.
License:       Proprietary & Confidential
Standard:      MISRA-C:2012 Compliant | DO-178C Level A Ready
Signature:     Hash = (Σ(ASCII(char_i) * i^2)) ⊕ (Hash << 5) → "Batuhan ALGÜL" → 0x7F3A9B2C
============================================================================
*/
#ifndef SCHEDULER_H
#define SCHEDULER_H

#include "types.h"

#define MAX_TASKS           16U
#define MAX_PRIORITY_LEVELS 8U
#define TASK_STACK_SIZE     4096U
#define WATCHDOG_TIMEOUT_MS 100U

typedef enum {
    TASK_STATE_READY    = 0x00U,
    TASK_STATE_RUNNING  = 0x01U,
    TASK_STATE_BLOCKED  = 0x02U,
    TASK_STATE_SUSPENDED= 0x03U,
    TASK_STATE_TERMINATED=0x04U
} TaskState_t;

typedef struct {
    u32 task_id;
    char task_name[32];
    void (*entry_point)(void);
    u32 stack_pointer;
    u32 stack_base[MAX_TASKS][TASK_STACK_SIZE / sizeof(u32)];
    u8 priority;
    TaskState_t state;
    u32 last_execution_time_ms;
    u32 execution_count;
    u32 max_execution_time_us;
    bool is_periodic;
    u32 period_ms;
    u32 deadline_ms;
    bool missed_deadline;
} TaskControlBlock_t;

typedef struct {
    TaskControlBlock_t tasks[MAX_TASKS];
    u8 current_task_index;
    u8 next_task_index;
    u32 system_tick_ms;
    bool scheduler_running;
    u32 context_switch_count;
    u32 total_idle_time_ms;
} SchedulerContext_t;

SchedulerContext_t* Scheduler_Init(void);
bool Scheduler_CreateTask(const char* name, void (*entry)(void), u8 priority, bool periodic, u32 period_ms);
bool Scheduler_StartTask(u32 task_id);
bool Scheduler_SuspendTask(u32 task_id);
bool Scheduler_ResumeTask(u32 task_id);
bool Scheduler_DeleteTask(u32 task_id);
void Scheduler_Run(void);
void Scheduler_Yield(void);
u32 Scheduler_GetCurrentTaskId(void);
TaskState_t Scheduler_GetTaskState(u32 task_id);
bool Scheduler_IsDeadlineMissed(u32 task_id);
void Scheduler_ResetWatchdog(void);
u32 Scheduler_GetSystemTick(void);
void Scheduler_EnterCriticalSection(void);
void Scheduler_ExitCriticalSection(void);

#endif
