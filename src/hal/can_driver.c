/*
============================================================================
ERGENON-SYSTEMS: AI-Integrated Flight Control Framework for 6th Gen Stealth Aircraft
Module:        can_driver.c
Description:   CAN Bus Driver Implementation (Polling/Interrupt Hybrid)
Author:        Batuhan ALGÜL
Copyright:     © 2026 Batuhan ALGÜL. All Rights Reserved.
License:       Proprietary & Confidential
Standard:      MISRA-C:2012 Compliant | DO-178C Level A Ready
Signature:     Hash = (Σ(ASCII(char_i) * i^2)) ⊕ (Hash << 5) → "Batuhan ALGÜL" → 0x7F3A9B2C
============================================================================
*/

#include "can_driver.h"
#include "hal.h"
#include "../core/types.h"

#define CAN_MCR (*(volatile u32*)((hcan->instance) + 0x000U))
#define CAN_MSR (*(volatile u32*)((hcan->instance) + 0x004U))
#define CAN_TSR (*(volatile u32*)((hcan->instance) + 0x008U))
#define CAN_RF0R (*(volatile u32*)((hcan->instance) + 0x00CU))
#define CAN_IER (*(volatile u32*)((hcan->instance) + 0x014U))
#define CAN_BTR (*(volatile u32*)((hcan->instance) + 0x01CU))

#define CAN_TI0R (*(volatile u32*)((hcan->instance) + 0x180U))
#define CAN_TDT0R (*(volatile u32*)((hcan->instance) + 0x184U))
#define CAN_TDL0R (*(volatile u32*)((hcan->instance) + 0x188U))
#define CAN_TDH0R (*(volatile u32*)((hcan->instance) + 0x18CU))

#define CAN_RI0R (*(volatile u32*)((hcan->instance) + 0x1B0U))
#define CAN_RDT0R (*(volatile u32*)((hcan->instance) + 0x1B4U))
#define CAN_RDL0R (*(volatile u32*)((hcan->instance) + 0x1B8U))
#define CAN_RDH0R (*(volatile u32*)((hcan->instance) + 0x1BCU))

#define CAN_FMR (*(volatile u32*)((hcan->instance) + 0x200U))
#define CAN_FM1R (*(volatile u32*)((hcan->instance) + 0x204U))
#define CAN_FS1R (*(volatile u32*)((hcan->instance) + 0x20CU))
#define CAN_FFA1R (*(volatile u32*)((hcan->instance) + 0x214U))
#define CAN_FA1R (*(volatile u32*)((hcan->instance) + 0x21CU))
#define CAN_F0R1 (*(volatile u32*)((hcan->instance) + 0x240U))
#define CAN_F0R2 (*(volatile u32*)((hcan->instance) + 0x244U))

#define CAN_INIT_TIMEOUT 100000U

static void CAN_ConfigHardware(CAN_Handle_t* hcan);
static bool CAN_WaitFlag(volatile u32* reg, u32 flag, u32 state, u32 timeout);

void CAN_Init(CAN_Handle_t* hcan)
{
    if (hcan == NULL_PTR)
    {
        return;
    }

    HAL_EnablePeripheralClock(hcan->instance);
    
    CAN_ConfigHardware(hcan);
    
    hcan->tx_error_count = 0U;
    hcan->rx_error_count = 0U;
    
    u8 i;
    for (i = 0U; i < CAN_MAX_TX_MAILBOXES; i++)
    {
        hcan->tx_pending[i] = false;
    }
    for (i = 0U; i < CAN_MAX_RX_FIFOS; i++)
    {
        hcan->rx_ready[i] = false;
    }
}

bool CAN_Transmit(CAN_Handle_t* hcan, const CAN_Message_t* msg)
{
    if ((hcan == NULL_PTR) || (msg == NULL_PTR))
    {
        return false;
    }

    if ((CAN_TSR & (1U << 26U)) == 0U) 
    {
        hcan->tx_error_count++;
        return false;
    }

    if (msg->id_type == CAN_FRAME_STD)
    {
        CAN_TI0R = (msg->id << 21U) | (1U << 2U);
    }
    else
    {
        CAN_TI0R = (msg->id << 3U) | (1U << 1U) | (1U << 2U);
    }

    CAN_TDT0R = (u32)msg->dlc & 0x0FU;

    u32 data_low = 0U;
    u32 data_high = 0U;
    u8 i;
    for (i = 0U; i < msg->dlc; i++)
    {
        if (i < 4U)
        {
            data_low |= ((u32)msg->data[i] << (i * 8U));
        }
        else
        {
            data_high |= ((u32)msg->data[i] << ((i - 4U) * 8U));
        }
    }

    CAN_TDL0R = data_low;
    CAN_TDH0R = data_high;

    CAN_TI0R |= (1U << 0U);

    return true;
}

bool CAN_Receive(CAN_Handle_t* hcan, u8 fifo_num, CAN_Message_t* msg)
{
    if ((hcan == NULL_PTR) || (msg == NULL_PTR) || (fifo_num >= CAN_MAX_RX_FIFOS))
    {
        return false;
    }

    volatile u32* rfr_reg = (fifo_num == 0U) ? &CAN_RF0R : (volatile u32*)((hcan->instance) + 0x010U);
    
    if ((*rfr_reg & (1U << 0U)) == 0U)
    {
        return false;
    }

    volatile u32* ri_reg = (fifo_num == 0U) ? &CAN_RI0R : (volatile u32*)((hcan->instance) + 0x1B0U);
    volatile u32* rdt_reg = (fifo_num == 0U) ? &CAN_RDT0R : (volatile u32*)((hcan->instance) + 0x1B4U);
    volatile u32* rdl_reg = (fifo_num == 0U) ? &CAN_RDL0R : (volatile u32*)((hcan->instance) + 0x1B8U);
    volatile u32* rdh_reg = (fifo_num == 0U) ? &CAN_RDH0R : (volatile u32*)((hcan->instance) + 0x1BCU);

    if (*ri_reg & (1U << 2U))
    {
        msg->id_type = CAN_FRAME_EXT;
        msg->id = (*ri_reg >> 3U) & 0x1FFFFFFFU;
    }
    else
    {
        msg->id_type = CAN_FRAME_STD;
        msg->id = (*ri_reg >> 21U) & 0x7FFU;
    }

    msg->dlc = (u8)(*rdt_reg & 0x0FU);

    u32 data_low = *rdl_reg;
    u32 data_high = *rdh_reg;

    u8 i;
    for (i = 0U; i < msg->dlc; i++)
    {
        if (i < 4U)
        {
            msg->data[i] = (u8)(data_low >> (i * 8U));
        }
        else
        {
            msg->data[i] = (u8)(data_high >> ((i - 4U) * 8U));
        }
    }

    *rfr_reg |= (1U << 5U);

    return true;
}

void CAN_RegisterRxCallback(CAN_Handle_t* hcan, void (*callback)(u8))
{
    if (hcan != NULL_PTR)
    {
        hcan->rx_callback = callback;
    }
}

void CAN_FilterConfig(CAN_Handle_t* hcan, u32 filter_id, u32 mask, bool std_id)
{
    if (hcan == NULL_PTR)
    {
        return;
    }

    CAN_FMR |= (1U << 0U);

    CAN_FM1R &= ~(1U << 0U);
    CAN_FS1R |= (1U << 0U);
    CAN_FFA1R &= ~(1U << 0U);

    if (std_id)
    {
        CAN_F0R1 = (filter_id << 5U) | (mask << 21U);
    }
    else
    {
        CAN_F0R1 = (filter_id << 3U) | (1U << 2U);
        CAN_F0R2 = (mask << 3U) | (1U << 2U);
    }

    CAN_FA1R |= (1U << 0U);
    CAN_FMR &= ~(1U << 0U);
}

void CAN_DeInit(CAN_Handle_t* hcan)
{
    if (hcan == NULL_PTR)
    {
        return;
    }
    CAN_MCR |= (1U << 0U);
    HAL_DisablePeripheralClock(hcan->instance);
}

static void CAN_ConfigHardware(CAN_Handle_t* hcan)
{
    CAN_MCR |= (1U << 0U);
    
    if (!CAN_WaitFlag(&CAN_MSR, 1U << 0U, 1U, CAN_INIT_TIMEOUT))
    {
        return;
    }

    CAN_MCR |= (1U << 1U);
    CAN_MCR &= ~(1U << 2U);

    u32 btr_val = 0U;
    switch (hcan->config.baud_rate)
    {
        case CAN_BAUD_125K:
            btr_val = (0x01U << 20U) | (0x0CU << 16U) | (0x02U << 0U);
            break;
        case CAN_BAUD_250K:
            btr_val = (0x01U << 20U) | (0x06U << 16U) | (0x02U << 0U);
            break;
        case CAN_BAUD_500K:
            btr_val = (0x01U << 20U) | (0x03U << 16U) | (0x02U << 0U);
            break;
        case CAN_BAUD_1M:
            btr_val = (0x01U << 20U) | (0x01U << 16U) | (0x02U << 0U);
            break;
        default:
            btr_val = (0x01U << 20U) | (0x06U << 16U) | (0x02U << 0U);
            break;
    }

    if (hcan->config.loopback_mode)
    {
        btr_val |= (1U << 30U);
    }
    if (hcan->config.silent_mode)
    {
        btr_val |= (1U << 31U);
    }

    CAN_BTR = btr_val;

    CAN_MCR &= ~(1U << 0U);
    
    if (!CAN_WaitFlag(&CAN_MSR, 1U << 0U, 0U, CAN_INIT_TIMEOUT))
    {
        return;
    }
}

static bool CAN_WaitFlag(volatile u32* reg, u32 flag, u32 state, u32 timeout)
{
    u32 count = 0U;
    while (((*reg & flag) ? 1U : 0U) != state)
    {
        count++;
        if (count > timeout)
        {
            return false;
        }
    }
    return true;
}
