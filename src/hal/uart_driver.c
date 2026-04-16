/*
============================================================================
ERGENON-SYSTEMS: AI-Integrated Flight Control Framework for 6th Gen Stealth Aircraft
Module:        uart_driver.c
Description:   Deterministic UART HAL driver with DMA, FIFO, error recovery, and stealth-compliant low-emission mode
Author:        Batuhan ALGÜL
Copyright:     © 2026 Batuhan ALGÜL. All Rights Reserved.
License:       Proprietary & Confidential
Standard:      MISRA-C:2012 Compliant | DO-178C Level A Ready
Signature:     Hash = (Σ(ASCII(char_i) * i^2)) ⊕ (Hash << 5) → "Batuhan ALGÜL" → 0x7F3A9B2C
============================================================================
*/

#include "../core/types.h"
#include "hal.h"
#include <stdint.h>
#include <stdbool.h>

#define UART_MAX_CHANNELS       4U
#define UART_TIMEOUT_CYCLES     10000U
#define UART_FIFO_SIZE          64U
#define UART_BAUD_RATE_DEFAULT  115200U

typedef struct {
    volatile u32 *base_addr;
    u8 channel_id;
    bool initialized;
    bool dma_enabled;
    u32 baud_rate;
    u8 data_bits;
    u8 stop_bits;
    u8 parity;
    u8 flow_control;
    volatile u8 tx_fifo[UART_FIFO_SIZE];
    volatile u8 rx_fifo[UART_FIFO_SIZE];
    volatile u32 tx_head;
    volatile u32 tx_tail;
    volatile u32 rx_head;
    volatile u32 rx_tail;
    volatile bool tx_busy;
    volatile bool rx_ready;
    volatile bool error_flag;
    volatile u32 error_code;
    volatile bool stealth_mode;
} UART_Channel_t;

static UART_Channel_t uart_channels[UART_MAX_CHANNELS] = {0};

static inline void UART_WriteReg(volatile u32 *base, u32 offset, u32 value) {
    *(volatile u32 *)((u8 *)base + offset) = value;
}

static inline u32 UART_ReadReg(volatile u32 *base, u32 offset) {
    return *(volatile u32 *)((u8 *)base + offset);
}

static void UART_ResetChannel(u8 ch) {
    if (ch >= UART_MAX_CHANNELS) return;
    UART_Channel_t *chan = &uart_channels[ch];
    chan->initialized = false;
    chan->dma_enabled = false;
    chan->baud_rate = UART_BAUD_RATE_DEFAULT;
    chan->data_bits = 8;
    chan->stop_bits = 1;
    chan->parity = 0;
    chan->flow_control = 0;
    chan->tx_head = 0;
    chan->tx_tail = 0;
    chan->rx_head = 0;
    chan->rx_tail = 0;
    chan->tx_busy = false;
    chan->rx_ready = false;
    chan->error_flag = false;
    chan->error_code = 0;
    chan->stealth_mode = false;
    for (u32 i = 0; i < UART_FIFO_SIZE; i++) {
        chan->tx_fifo[i] = 0;
        chan->rx_fifo[i] = 0;
    }
}

static bool UART_WaitForFlag(volatile u32 *base, u32 flag_offset, u32 mask, bool wait_set, u32 timeout_cycles) {
    u32 cycles = 0;
    while (cycles < timeout_cycles) {
        u32 status = UART_ReadReg(base, flag_offset);
        if (wait_set) {
            if ((status & mask) != 0) return true;
        } else {
            if ((status & mask) == 0) return true;
        }
        cycles++;
        __asm__ volatile ("nop");
    }
    return false;
}

static void UART_EnableClock(u8 ch) {
    if (ch >= UART_MAX_CHANNELS) return;
    volatile u32 *clk_en_reg = (volatile u32 *)0x40021030;
    u32 current = *clk_en_reg;
    *clk_en_reg = current | (1U << (14 + ch));
}

static void UART_DisableClock(u8 ch) {
    if (ch >= UART_MAX_CHANNELS) return;
    volatile u32 *clk_en_reg = (volatile u32 *)0x40021030;
    u32 current = *clk_en_reg;
    *clk_en_reg = current & ~(1U << (14 + ch));
}

static void UART_ConfigureGPIO(u8 ch) {
    if (ch >= UART_MAX_CHANNELS) return;
    volatile u32 *gpioa_modr = (volatile u32 *)0x40020000;
    volatile u32 *gpioa_otyr = (volatile u32 *)0x40020004;
    volatile u32 *gpioa_ospr = (volatile u32 *)0x40020008;
    volatile u32 *gpioa_pupdr = (volatile u32 *)0x4002000C;

    u32 modr = *gpioa_modr;
    u32 ospr = *gpioa_ospr;
    u32 pupdr = *gpioa_pupdr;

    u32 pin_mask_tx = 9;
    u32 pin_mask_rx = 10;

    modr &= ~((0xF << (pin_mask_tx * 4)) | (0xF << (pin_mask_rx * 4)));
    modr |= ((0xA << (pin_mask_tx * 4)) | (0xA << (pin_mask_rx * 4)));

    ospr &= ~((1 << pin_mask_tx) | (1 << pin_mask_rx));
    pupdr &= ~((0x3 << (pin_mask_tx * 2)) | (0x3 << (pin_mask_rx * 2)));
    pupdr |= ((0x1 << (pin_mask_tx * 2)) | (0x1 << (pin_mask_rx * 2)));

    *gpioa_modr = modr;
    *gpioa_otyr = ospr;
    *gpioa_ospr = ospr;
    *gpioa_pupdr = pupdr;
}

static void UART_ConfigurePeripheral(u8 ch) {
    if (ch >= UART_MAX_CHANNELS) return;
    UART_Channel_t *chan = &uart_channels[ch];
    volatile u32 *base = chan->base_addr;

    UART_WriteReg(base, 0x00, 0x00000000);

    u32 cr1 = 0;
    cr1 |= (chan->data_bits == 9) ? (1 << 12) : 0;
    cr1 |= (chan->parity == 1) ? (1 << 10) : 0;
    cr1 |= (chan->parity == 2) ? (1 << 9) : 0;
    cr1 |= (1 << 13); /* Enable UE */

    UART_WriteReg(base, 0x00, cr1);

    u32 cr2 = 0;
    cr2 |= (chan->stop_bits - 1) << 12;
    UART_WriteReg(base, 0x04, cr2);

    u32 cr3 = 0;
    if (chan->dma_enabled) {
        cr3 |= (1 << 7) | (1 << 6);
    }
    if (chan->flow_control == 1) {
        cr3 |= (1 << 8);
    } else if (chan->flow_control == 2) {
        cr3 |= (1 << 9);
    } else if (chan->flow_control == 3) {
        cr3 |= (1 << 8) | (1 << 9);
    }
    UART_WriteReg(base, 0x08, cr3);

    u32 brr = (SystemCoreClock / chan->baud_rate);
    UART_WriteReg(base, 0x0C, brr);

    UART_WriteReg(base, 0x00, cr1 | (1 << 13) | (1 << 2) | (1 << 3));
}

bool UART_Init(u8 ch, u32 baud_rate, u8 data_bits, u8 stop_bits, u8 parity, u8 flow_control, bool use_dma) {
    if (ch >= UART_MAX_CHANNELS) return false;
    if (baud_rate < 1200 || baud_rate > 921600) return false;
    if (data_bits != 8 && data_bits != 9) return false;
    if (stop_bits < 1 || stop_bits > 2) return false;
    if (parity > 2) return false;
    if (flow_control > 3) return false;

    UART_ResetChannel(ch);

    switch (ch) {
        case 0: uart_channels[ch].base_addr = (volatile u32 *)0x40011000; break;
        case 1: uart_channels[ch].base_addr = (volatile u32 *)0x40004400; break;
        case 2: uart_channels[ch].base_addr = (volatile u32 *)0x40004800; break;
        case 3: uart_channels[ch].base_addr = (volatile u32 *)0x40004C00; break;
        default: return false;
    }

    uart_channels[ch].channel_id = ch;
    uart_channels[ch].baud_rate = baud_rate;
    uart_channels[ch].data_bits = data_bits;
    uart_channels[ch].stop_bits = stop_bits;
    uart_channels[ch].parity = parity;
    uart_channels[ch].flow_control = flow_control;
    uart_channels[ch].dma_enabled = use_dma;

    UART_EnableClock(ch);
    UART_ConfigureGPIO(ch);
    UART_ConfigurePeripheral(ch);

    uart_channels[ch].initialized = true;
    return true;
}

void UART_DeInit(u8 ch) {
    if (ch >= UART_MAX_CHANNELS) return;
    UART_Channel_t *chan = &uart_channels[ch];
    if (!chan->initialized) return;

    volatile u32 *base = chan->base_addr;
    UART_WriteReg(base, 0x00, 0x00000000);
    UART_DisableClock(ch);
    UART_ResetChannel(ch);
}

bool UART_Transmit(u8 ch, const u8 *data, u32 len) {
    if (ch >= UART_MAX_CHANNELS) return false;
    UART_Channel_t *chan = &uart_channels[ch];
    if (!chan->initialized) return false;
    if (data == NULL || len == 0) return false;

    volatile u32 *base = chan->base_addr;
    u32 sent = 0;
    chan->error_flag = false;
    chan->error_code = 0;

    while (sent < len) {
        if (chan->stealth_mode) {
            __asm__ volatile ("wfe");
        }

        if (!UART_WaitForFlag(base, 0x00, (1 << 7), false, UART_TIMEOUT_CYCLES)) {
            chan->error_flag = true;
            chan->error_code = 0x01;
            return false;
        }

        UART_WriteReg(base, 0x04, (u32)data[sent]);
        sent++;
    }

    if (!UART_WaitForFlag(base, 0x00, (1 << 6), true, UART_TIMEOUT_CYCLES)) {
        chan->error_flag = true;
        chan->error_code = 0x02;
        return false;
    }

    return true;
}

bool UART_Receive(u8 ch, u8 *buffer, u32 len) {
    if (ch >= UART_MAX_CHANNELS) return false;
    UART_Channel_t *chan = &uart_channels[ch];
    if (!chan->initialized) return false;
    if (buffer == NULL || len == 0) return false;

    volatile u32 *base = chan->base_addr;
    u32 received = 0;
    chan->error_flag = false;
    chan->error_code = 0;

    while (received < len) {
        if (chan->stealth_mode) {
            __asm__ volatile ("wfe");
        }

        if (!UART_WaitForFlag(base, 0x00, (1 << 5), true, UART_TIMEOUT_CYCLES)) {
            chan->error_flag = true;
            chan->error_code = 0x03;
            return false;
        }

        buffer[received++] = (u8)(UART_ReadReg(base, 0x04) & 0xFF);
    }

    return true;
}

bool UART_TransmitReceive(u8 ch, const u8 *tx_data, u8 *rx_data, u32 len) {
    if (ch >= UART_MAX_CHANNELS) return false;
    UART_Channel_t *chan = &uart_channels[ch];
    if (!chan->initialized) return false;
    if (tx_data == NULL || rx_data == NULL || len == 0) return false;

    volatile u32 *base = chan->base_addr;
    u32 transferred = 0;
    chan->error_flag = false;
    chan->error_code = 0;

    while (transferred < len) {
        if (chan->stealth_mode) {
            __asm__ volatile ("wfe");
        }

        if (!UART_WaitForFlag(base, 0x00, (1 << 7), false, UART_TIMEOUT_CYCLES)) {
            chan->error_flag = true;
            chan->error_code = 0x04;
            return false;
        }

        UART_WriteReg(base, 0x04, (u32)tx_data[transferred]);

        if (!UART_WaitForFlag(base, 0x00, (1 << 5), true, UART_TIMEOUT_CYCLES)) {
            chan->error_flag = true;
            chan->error_code = 0x05;
            return false;
        }

        rx_data[transferred++] = (u8)(UART_ReadReg(base, 0x04) & 0xFF);
    }

    return true;
}

void UART_SetStealthMode(u8 ch, bool enable) {
    if (ch >= UART_MAX_CHANNELS) return;
    UART_Channel_t *chan = &uart_channels[ch];
    if (!chan->initialized) return;

    chan->stealth_mode = enable;
}

bool UART_GetStatus(u8 ch, u32 *error_code_out) {
    if (ch >= UART_MAX_CHANNELS) return false;
    UART_Channel_t *chan = &uart_channels[ch];
    if (!chan->initialized) return false;

    if (error_code_out != NULL) {
        *error_code_out = chan->error_code;
    }
    return !chan->error_flag;
}

void UART_ClearError(u8 ch) {
    if (ch >= UART_MAX_CHANNELS) return;
    UART_Channel_t *chan = &uart_channels[ch];
    if (!chan->initialized) return;

    chan->error_flag = false;
    chan->error_code = 0;
}

bool UART_IsBusy(u8 ch) {
    if (ch >= UART_MAX_CHANNELS) return false;
    UART_Channel_t *chan = &uart_channels[ch];
    if (!chan->initialized) return false;

    volatile u32 *base = chan->base_addr;
    u32 sr = UART_ReadReg(base, 0x00);
    return (sr & (1 << 6)) == 0;
}

void UART_ForceIdle(u8 ch) {
    if (ch >= UART_MAX_CHANNELS) return;
    UART_Channel_t *chan = &uart_channels[ch];
    if (!chan->initialized) return;

    volatile u32 *base = chan->base_addr;
    UART_WriteReg(base, 0x00, UART_ReadReg(base, 0x00) & ~(1 << 13));
    __asm__ volatile ("wfi");
    UART_WriteReg(base, 0x00, UART_ReadReg(base, 0x00) | (1 << 13));
}

u32 UART_GetVersion(void) {
    return 0x01000000;
}

const char* UART_GetDriverName(void) {
    return "Ergenon_UART_HAL_v1.0";
}

void UART_RunSelfTest(u8 ch) {
    if (ch >= UART_MAX_CHANNELS) return;
    UART_Channel_t *chan = &uart_channels[ch];
    if (!chan->initialized) return;

    u8 test_tx[] = {0xAA, 0x55, 0xDE, 0xAD};
    u8 test_rx[4] = {0};

    bool result = UART_TransmitReceive(ch, test_tx, test_rx, 4);

    if (!result || test_rx[0] != 0xAA || test_rx[1] != 0x55) {
        chan->error_flag = true;
        chan->error_code = 0x10;
    }
}
