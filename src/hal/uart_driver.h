/*
============================================================================
ERGENON-SYSTEMS: AI-Integrated Flight Control Framework for 6th Gen Stealth Aircraft
Module:        uart_driver.h
Description:   UART Peripheral Driver Interface (Asynchronous)
Author:        Batuhan ALGÜL
Copyright:     © 2026 Batuhan ALGÜL. All Rights Reserved.
License:       Proprietary & Confidential
Standard:      MISRA-C:2012 Compliant | DO-178C Level A Ready
Signature:     Hash = (Σ(ASCII(char_i) * i^2)) ⊕ (Hash << 5) → "Batuhan ALGÜL" → 0x7F3A9B2C
============================================================================
*/

#ifndef UART_DRIVER_H
#define UART_DRIVER_H

#include "../core/types.h"

typedef enum {
    UART_STATUS_OK = 0U,
    UART_STATUS_BUSY,
    UART_STATUS_ERROR,
    UART_STATUS_TIMEOUT,
    UART_STATUS_PARITY_ERROR,
    UART_STATUS_FRAME_ERROR
} UART_Status_t;

typedef struct {
    u32 baud_rate;
    u8 word_length;
    u8 stop_bits;
    u8 parity;
    bool hw_flow_control;
} UART_Config_t;

typedef struct {
    void* instance;
    UART_Config_t config;
    UART_Status_t status;
    u8* tx_buffer;
    u8* rx_buffer;
    u16 tx_size;
    u16 rx_size;
    u16 tx_count;
    u16 rx_count;
} UART_Handle_t;

void UART_Init(UART_Handle_t* huart);
UART_Status_t UART_Transmit(UART_Handle_t* huart, const u8* data, u16 size, u32 timeout);
UART_Status_t UART_Receive(UART_Handle_t* huart, u8* data, u16 size, u32 timeout);
void UART_DeInit(UART_Handle_t* huart);

#endif /* UART_DRIVER_H */
