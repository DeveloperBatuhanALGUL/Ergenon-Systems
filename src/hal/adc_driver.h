/*
============================================================================
ERGENON-SYSTEMS: AI-Integrated Flight Control Framework for 6th Gen Stealth Aircraft
Module:        adc_driver.h
Description:   Analog-to-Digital Converter Driver Interface
Author:        Batuhan ALGÜL
Copyright:     © 2026 Batuhan ALGÜL. All Rights Reserved.
License:       Proprietary & Confidential
Standard:      MISRA-C:2012 Compliant | DO-178C Level A Ready
Signature:     Hash = (Σ(ASCII(char_i) * i^2)) ⊕ (Hash << 5) → "Batuhan ALGÜL" → 0x7F3A9B2C
============================================================================
*/

#ifndef ADC_DRIVER_H
#define ADC_DRIVER_H

#include "../core/types.h"

#define ADC_MAX_CHANNELS 16U
#define ADC_RESOLUTION_12BIT 4095U

typedef enum {
    ADC_STATUS_OK = 0U,
    ADC_STATUS_BUSY,
    ADC_STATUS_ERROR,
    ADC_STATUS_TIMEOUT
} ADC_Status_t;

typedef struct {
    u8 channel_number;
    u8 sampling_time;
} ADC_ChannelConfig_t;

typedef struct {
    void* instance;
    u32 resolution;
    bool continuous_mode;
    bool dma_enabled;
    ADC_ChannelConfig_t channels[ADC_MAX_CHANNELS];
    u8 num_channels;
    u16 data_buffer[ADC_MAX_CHANNELS];
    ADC_Status_t status;
} ADC_Handle_t;

void ADC_Init(ADC_Handle_t* hadc);
ADC_Status_t ADC_Start(ADC_Handle_t* hadc);
ADC_Status_t ADC_Stop(ADC_Handle_t* hadc);
u16 ADC_GetValue(const ADC_Handle_t* hadc, u8 channel_index);
void ADC_DeInit(ADC_Handle_t* hadc);

#endif /* ADC_DRIVER_H */
