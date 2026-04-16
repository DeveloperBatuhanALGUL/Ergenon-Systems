/*
============================================================================
ERGENON-SYSTEMS: AI-Integrated Flight Control Framework for 6th Gen Stealth Aircraft
Module:        can_driver.h
Description:   CAN Bus Driver Interface (Classic CAN & CAN FD Support)
Author:        Batuhan ALGÜL
Copyright:     © 2026 Batuhan ALGÜL. All Rights Reserved.
License:       Proprietary & Confidential
Standard:      MISRA-C:2012 Compliant | DO-178C Level A Ready
Signature:     Hash = (Σ(ASCII(char_i) * i^2)) ⊕ (Hash << 5) → "Batuhan ALGÜL" → 0x7F3A9B2C
============================================================================
*/

#ifndef CAN_DRIVER_H
#define CAN_DRIVER_H

#include "../core/types.h"

#define CAN_MAX_TX_MAILBOXES 3U
#define CAN_MAX_RX_FIFOS 2U
#define CAN_DLC_MAX 8U
#define CAN_FD_DLC_MAX 64U

typedef enum {
    CAN_FRAME_STD = 0U,
    CAN_FRAME_EXT
} CAN_IdType_t;

typedef enum {
    CAN_BAUD_125K = 0U,
    CAN_BAUD_250K,
    CAN_BAUD_500K,
    CAN_BAUD_1M
} CAN_BaudRate_t;

typedef struct {
    u32 id;
    CAN_IdType_t id_type;
    u8 dlc;
    u8 data[CAN_FD_DLC_MAX];
    bool is_fd;
    bool brs; /* Bit Rate Switch for CAN FD */
} CAN_Message_t;

typedef struct {
    void* instance;
    CAN_BaudRate_t baud_rate;
    bool loopback_mode;
    bool silent_mode;
    u8 tx_error_count;
    u8 rx_error_count;
} CAN_Config_t;

typedef struct {
    CAN_Config_t config;
    CAN_Message_t tx_msg[CAN_MAX_TX_MAILBOXES];
    CAN_Message_t rx_msg[CAN_MAX_RX_FIFOS];
    volatile bool tx_pending[CAN_MAX_TX_MAILBOXES];
    volatile bool rx_ready[CAN_MAX_RX_FIFOS];
    void (*rx_callback)(u8 fifo_num);
} CAN_Handle_t;

void CAN_Init(CAN_Handle_t* hcan);
bool CAN_Transmit(CAN_Handle_t* hcan, const CAN_Message_t* msg);
bool CAN_Receive(CAN_Handle_t* hcan, u8 fifo_num, CAN_Message_t* msg);
void CAN_RegisterRxCallback(CAN_Handle_t* hcan, void (*callback)(u8));
void CAN_FilterConfig(CAN_Handle_t* hcan, u32 filter_id, u32 mask, bool std_id);
void CAN_DeInit(CAN_Handle_t* hcan);

#endif /* CAN_DRIVER_H */
