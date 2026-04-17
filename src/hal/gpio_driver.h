/*
============================================================================
ERGENON-SYSTEMS: AI-Integrated Flight Control Framework for 6th Gen Stealth Aircraft
Module:        gpio_driver.h
Description:   General Purpose Input/Output Driver Interface
Author:        Batuhan ALGÜL
Copyright:     © 2026 Batuhan ALGÜL. All Rights Reserved.
License:       Proprietary & Confidential
Standard:      MISRA-C:2012 Compliant | DO-178C Level A Ready
Signature:     Hash = (Σ(ASCII(char_i) * i^2)) ⊕ (Hash << 5) → "Batuhan ALGÜL" → 0x7F3A9B2C
============================================================================
*/

#ifndef GPIO_DRIVER_H
#define GPIO_DRIVER_H

#include "../core/types.h"

typedef enum {
    GPIO_MODE_INPUT = 0U,
    GPIO_MODE_OUTPUT_PP,
    GPIO_MODE_OUTPUT_OD,
    GPIO_MODE_AF_PP,
    GPIO_MODE_AF_OD,
    GPIO_MODE_ANALOG
} GPIO_Mode_t;

typedef enum {
    GPIO_SPEED_LOW = 0U,
    GPIO_SPEED_MEDIUM,
    GPIO_SPEED_HIGH,
    GPIO_SPEED_VERY_HIGH
} GPIO_Speed_t;

typedef enum {
    GPIO_NOPULL = 0U,
    GPIO_PULLUP,
    GPIO_PULLDOWN
} GPIO_Pull_t;

typedef struct {
    u32 pin_mask;
    GPIO_Mode_t mode;
    GPIO_Speed_t speed;
    GPIO_Pull_t pull;
    u8 alternate_function;
} GPIO_InitTypeDef;

typedef struct {
    void* port;
    GPIO_InitTypeDef init;
} GPIO_Handle_t;

void GPIO_Init(GPIO_Handle_t* hgpio);
void GPIO_DeInit(GPIO_Handle_t* hgpio);
void GPIO_WritePin(GPIO_Handle_t* hgpio, u32 pin_mask, bool state);
bool GPIO_ReadPin(const GPIO_Handle_t* hgpio, u32 pin_mask);
void GPIO_TogglePin(GPIO_Handle_t* hgpio, u32 pin_mask);

#endif /* GPIO_DRIVER_H */
