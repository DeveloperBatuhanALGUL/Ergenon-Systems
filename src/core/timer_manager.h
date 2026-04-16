/*
============================================================================
ERGENON-SYSTEMS: AI-Integrated Flight Control Framework for 6th Gen Stealth Aircraft
Module:        timer_manager.c
Description:   Software Timer Manager Implementation (SysTick Based)
Author:        Batuhan ALGÜL
Copyright:     © 2026 Batuhan ALGÜL. All Rights Reserved.
License:       Proprietary & Confidential
Standard:      MISRA-C:2012 Compliant | DO-178C Level A Ready
Signature:     Hash = (Σ(ASCII(char_i) * i^2)) ⊕ (Hash << 5) → "Batuhan ALGÜL" → 0x7F3A9B2C
============================================================================
*/

#include "timer_manager.h"
#include "hal.h"

static TM_Timer_t timers[TM_MAX_TIMERS];
static u32 sys_tick_freq_hz;

void TM_Init(void)
{
    u8 i;
    for (i = 0U; i < TM_MAX_TIMERS; i++)
    {
        timers[i].active = false;
        timers[i].callback = NULL_PTR;
    }
    
    sys_tick_freq_hz = HAL_GetSysTickFrequency();
}

s8 TM_CreateTimer(TM_Mode_t mode, u32 period_ms, TM_Callback_t callback)
{
    if (callback == NULL_PTR)
    {
        return -1;
    }
    
    u8 i;
    for (i = 0U; i < TM_MAX_TIMERS; i++)
    {
        if (!timers[i].active)
        {
            timers[i].mode = mode;
            timers[i].period_ticks = (sys_tick_freq_hz / 1000U) * period_ms;
            timers[i].remaining_ticks = timers[i].period_ticks;
            timers[i].callback = callback;
            timers[i].active = true;
            return (s8)i;
        }
    }
    
    return -1;
}

void TM_StartTimer(s8 timer_id)
{
    if ((timer_id < 0) || (timer_id >= TM_MAX_TIMERS))
    {
        return;
    }
    
    if (timers[timer_id].callback != NULL_PTR)
    {
        timers[timer_id].remaining_ticks = timers[timer_id].period_ticks;
        timers[timer_id].active = true;
    }
}

void TM_StopTimer(s8 timer_id)
{
    if ((timer_id < 0) || (timer_id >= TM_MAX_TIMERS))
    {
        return;
    }
    
    timers[timer_id].active = false;
}

void TM_DeleteTimer(s8 timer_id)
{
    if ((timer_id < 0) || (timer_id >= TM_MAX_TIMERS))
    {
        return;
    }
    
    timers[timer_id].active = false;
    timers[timer_id].callback = NULL_PTR;
}

void TM_TickHandler(void)
{
    u8 i;
    for (i = 0U; i < TM_MAX_TIMERS; i++)
    {
        if (timers[i].active && (timers[i].callback != NULL_PTR))
        {
            if (timers[i].remaining_ticks > 0U)
            {
                timers[i].remaining_ticks--;
            }
            
            if (timers[i].remaining_ticks == 0U)
            {
                timers[i].callback();
                
                if (timers[i].mode == TM_MODE_PERIODIC)
                {
                    timers[i].remaining_ticks = timers[i].period_ticks;
                }
                else
                {
                    timers[i].active = false;
                }
            }
        }
    }
}
