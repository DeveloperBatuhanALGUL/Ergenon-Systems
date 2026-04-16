/*
============================================================================
ERGENON-SYSTEMS: AI-Integrated Flight Control Framework for 6th Gen Stealth Aircraft
Module:        adc_driver.c
Description:   Analog-to-Digital Converter Driver Implementation
Author:        Batuhan ALGÜL
Copyright:     © 2026 Batuhan ALGÜL. All Rights Reserved.
License:       Proprietary & Confidential
Standard:      MISRA-C:2012 Compliant | DO-178C Level A Ready
Signature:     Hash = (Σ(ASCII(char_i) * i^2)) ⊕ (Hash << 5) → "Batuhan ALGÜL" → 0x7F3A9B2C
============================================================================
*/

#include "adc_driver.h"
#include "hal.h"

#define ADC_CR2 (*(volatile u32*)((hadc->instance) + 0x04U))
#define ADC_SQR1 (*(volatile u32*)((hadc->instance) + 0x2CU))
#define ADC_SQR3 (*(volatile u32*)((hadc->instance) + 0x34U))
#define ADC_DR (*(volatile u32*)((hadc->instance) + 0x4CU))
#define ADC_SR (*(volatile u32*)((hadc->instance) + 0x00U))

#define ADC_TIMEOUT 10000U

static void ADC_ConfigChannels(ADC_Handle_t* hadc);

void ADC_Init(ADC_Handle_t* hadc)
{
    if (hadc == NULL_PTR)
    {
        return;
    }

    HAL_EnablePeripheralClock(hadch->instance);

    volatile u32* cr1_reg = (volatile u32*)((hadc->instance) + 0x00U);
    *cr1_reg &= ~(0xFU << 24U); 
    *cr1_reg |= (hadc->resolution & 0xFU) << 24U;

    ADC_ConfigChannels(hadc);

    volatile u32* cr2_reg = (volatile u32*)((hadc->instance) + 0x04U);
    if (hadc->continuous_mode)
    {
        *cr2_reg |= (1U << 1U);
    }
    else
    {
        *cr2_reg &= ~(1U << 1U);
    }

    hadc->status = ADC_STATUS_OK;
}

ADC_Status_t ADC_Start(ADC_Handle_t* hadc)
{
    if (hadc == NULL_PTR)
    {
        return ADC_STATUS_ERROR;
    }

    if (hadc->status == ADC_STATUS_BUSY)
    {
        return ADC_STATUS_BUSY;
    }

    ADC_CR2 |= (1U << 0U);

    u32 timeout = ADC_TIMEOUT;
    while ((ADC_SR & (1U << 1U)) == 0U)
    {
        if (timeout-- == 0U)
        {
            hadc->status = ADC_STATUS_TIMEOUT;
            return ADC_STATUS_TIMEOUT;
        }
    }

    hadc->status = ADC_STATUS_BUSY;
    return ADC_STATUS_OK;
}

ADC_Status_t ADC_Stop(ADC_Handle_t* hadc)
{
    if (hadc == NULL_PTR)
    {
        return ADC_STATUS_ERROR;
    }

    ADC_CR2 &= ~(1U << 0U);
    hadc->status = ADC_STATUS_OK;
    return ADC_STATUS_OK;
}

u16 ADC_GetValue(const ADC_Handle_t* hadc, u8 channel_index)
{
    if ((hadc == NULL_PTR) || (channel_index >= hadc->num_channels))
    {
        return 0U;
    }

    return hadc->data_buffer[channel_index];
}

void ADC_DeInit(ADC_Handle_t* hadc)
{
    if (hadc == NULL_PTR)
    {
        return;
    }
    ADC_Stop(hadc);
    HAL_DisablePeripheralClock(hadc->instance);
}

static void ADC_ConfigChannels(ADC_Handle_t* hadc)
{
    u8 i;
    u32 sqr3_val = 0U;
    
    for (i = 0U; i < hadc->num_channels; i++)
    {
        if (i < 6U)
        {
            u8 shift = (i * 5U);
            sqr3_val &= ~(0x1FU << shift);
            sqr3_val |= ((u32)hadc->channels[i].channel_number << shift);
        }
    }
    
    ADC_SQR3 = sqr3_val;
    
    volatile u32* smpr2_reg = (volatile u32*)((hadc->instance) + 0x14U);
    volatile u32* smpr1_reg = (volatile u32*)((hadc->instance) + 0x10U);
    
    for (i = 0U; i < hadc->num_channels; i++)
    {
        u8 chan = hadc->channels[i].channel_number;
        u8 samp_time = hadc->channels[i].sampling_time;
        
        if (chan < 10U)
        {
            u8 shift = (chan * 3U);
            *smpr2_reg &= ~(0x07U << shift);
            *smpr2_reg |= ((u32)samp_time << shift);
        }
        else
        {
            u8 shift = ((chan - 10U) * 3U);
            *smpr1_reg &= ~(0x07U << shift);
            *smpr1_reg |= ((u32)samp_time << shift);
        }
    }
}
