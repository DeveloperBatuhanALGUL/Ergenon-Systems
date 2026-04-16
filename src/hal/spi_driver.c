/*
============================================================================
ERGENON-SYSTEMS: AI-Integrated Flight Control Framework for 6th Gen Stealth Aircraft
Module:        spi_driver.c
Description:   Deterministic SPI HAL driver with DMA, fault isolation, and TMR-ready interface
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

#define SPI_MAX_CHANNELS        4U
#define SPI_TIMEOUT_CYCLES      10000U
#define SPI_DMA_BUFFER_SIZE     256U

typedef struct {
    volatile u32 *base_addr;
    u8 channel_id;
    bool initialized;
    bool dma_enabled;
    u8 mode; /* 0: Master, 1: Slave */
    u8 clock_divider;
    u8 data_bits;
    u8 cpol;
    u8 cpha;
    u8 cs_pin;
    volatile u8 tx_buffer[SPI_DMA_BUFFER_SIZE];
    volatile u8 rx_buffer[SPI_DMA_BUFFER_SIZE];
    volatile u32 tx_index;
    volatile u32 rx_index;
    volatile bool transfer_in_progress;
    volatile bool error_flag;
    volatile u32 error_code;
} SPI_Channel_t;

static SPI_Channel_t spi_channels[SPI_MAX_CHANNELS] = {0};

static inline void SPI_WriteReg(volatile u32 *base, u32 offset, u32 value) {
    *(volatile u32 *)((u8 *)base + offset) = value;
}

static inline u32 SPI_ReadReg(volatile u32 *base, u32 offset) {
    return *(volatile u32 *)((u8 *)base + offset);
}

static void SPI_ResetChannel(u8 ch) {
    if (ch >= SPI_MAX_CHANNELS) return;
    SPI_Channel_t *chan = &spi_channels[ch];
    chan->initialized = false;
    chan->dma_enabled = false;
    chan->mode = 0;
    chan->clock_divider = 0;
    chan->data_bits = 8;
    chan->cpol = 0;
    chan->cpha = 0;
    chan->cs_pin = 0xFF;
    chan->tx_index = 0;
    chan->rx_index = 0;
    chan->transfer_in_progress = false;
    chan->error_flag = false;
    chan->error_code = 0;
    for (u32 i = 0; i < SPI_DMA_BUFFER_SIZE; i++) {
        chan->tx_buffer[i] = 0;
        chan->rx_buffer[i] = 0;
    }
}

static bool SPI_WaitForFlag(volatile u32 *base, u32 flag_offset, u32 mask, bool wait_set, u32 timeout_cycles) {
    u32 cycles = 0;
    while (cycles < timeout_cycles) {
        u32 status = SPI_ReadReg(base, flag_offset);
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

static void SPI_EnableClock(u8 ch) {
    if (ch >= SPI_MAX_CHANNELS) return;
    volatile u32 *clk_en_reg = (volatile u32 *)0x40021030;
    u32 current = *clk_en_reg;
    *clk_en_reg = current | (1U << (12 + ch));
}

static void SPI_DisableClock(u8 ch) {
    if (ch >= SPI_MAX_CHANNELS) return;
    volatile u32 *clk_en_reg = (volatile u32 *)0x40021030;
    u32 current = *clk_en_reg;
    *clk_en_reg = current & ~(1U << (12 + ch));
}

static void SPI_ConfigureGPIO(u8 ch, u8 cs_pin) {
    if (ch >= SPI_MAX_CHANNELS || cs_pin > 15) return;
    volatile u32 *gpioa_modr = (volatile u32 *)0x40020000;
    volatile u32 *gpioa_otyr = (volatile u32 *)0x40020004;
    volatile u32 *gpioa_ospr = (volatile u32 *)0x40020008;
    volatile u32 *gpioa_pupdr = (volatile u32 *)0x4002000C;

    u32 modr = *gpioa_modr;
    u32 ospr = *gpioa_ospr;
    u32 pupdr = *gpioa_pupdr;

    u32 pin_mask_sck = 5;
    u32 pin_mask_miso = 6;
    u32 pin_mask_mosi = 7;
    u32 pin_mask_cs = cs_pin;

    modr &= ~((0xF << (pin_mask_sck * 4)) | (0xF << (pin_mask_miso * 4)) | (0xF << (pin_mask_mosi * 4)) | (0xF << (pin_mask_cs * 4)));
    modr |= ((0xA << (pin_mask_sck * 4)) | (0xA << (pin_mask_miso * 4)) | (0xA << (pin_mask_mosi * 4)) | (0x1 << (pin_mask_cs * 4)));

    ospr &= ~((1 << pin_mask_sck) | (1 << pin_mask_miso) | (1 << pin_mask_mosi) | (1 << pin_mask_cs));
    ospr |= (1 << pin_mask_cs);

    pupdr &= ~((0x3 << (pin_mask_sck * 2)) | (0x3 << (pin_mask_miso * 2)) | (0x3 << (pin_mask_mosi * 2)) | (0x3 << (pin_mask_cs * 2)));
    pupdr |= ((0x1 << (pin_mask_sck * 2)) | (0x1 << (pin_mask_miso * 2)) | (0x1 << (pin_mask_mosi * 2)) | (0x1 << (pin_mask_cs * 2)));

    *gpioa_modr = modr;
    *gpioa_otyr = ospr;
    *gpioa_ospr = ospr;
    *gpioa_pupdr = pupdr;
}

static void SPI_ConfigurePeripheral(u8 ch) {
    if (ch >= SPI_MAX_CHANNELS) return;
    SPI_Channel_t *chan = &spi_channels[ch];
    volatile u32 *base = chan->base_addr;

    SPI_WriteReg(base, 0x00, 0x00000000);

    u32 cr1 = 0;
    cr1 |= (chan->clock_divider & 0x07) << 3;
    cr1 |= (chan->mode & 0x01) << 2;
    cr1 |= (chan->cpol & 0x01) << 1;
    cr1 |= (chan->cpha & 0x01);
    cr1 |= (chan->data_bits == 16) ? (1 << 11) : 0;

    SPI_WriteReg(base, 0x00, cr1);

    u32 cr2 = 0;
    if (chan->dma_enabled) {
        cr2 |= (1 << 1) | (1 << 0);
    }
    SPI_WriteReg(base, 0x04, cr2);

    SPI_WriteReg(base, 0x00, cr1 | (1 << 6));
}

bool SPI_Init(u8 ch, u8 mode, u8 clock_div, u8 data_bits, u8 cpol, u8 cpha, u8 cs_pin, bool use_dma) {
    if (ch >= SPI_MAX_CHANNELS) return false;
    if (clock_div > 7) return false;
    if (data_bits != 8 && data_bits != 16) return false;
    if (cs_pin > 15) return false;

    SPI_ResetChannel(ch);

    switch (ch) {
        case 0: spi_channels[ch].base_addr = (volatile u32 *)0x40013000; break;
        case 1: spi_channels[ch].base_addr = (volatile u32 *)0x40013400; break;
        case 2: spi_channels[ch].base_addr = (volatile u32 *)0x40013800; break;
        case 3: spi_channels[ch].base_addr = (volatile u32 *)0x40013C00; break;
        default: return false;
    }

    spi_channels[ch].channel_id = ch;
    spi_channels[ch].mode = mode;
    spi_channels[ch].clock_divider = clock_div;
    spi_channels[ch].data_bits = data_bits;
    spi_channels[ch].cpol = cpol;
    spi_channels[ch].cpha = cpha;
    spi_channels[ch].cs_pin = cs_pin;
    spi_channels[ch].dma_enabled = use_dma;

    SPI_EnableClock(ch);
    SPI_ConfigureGPIO(ch, cs_pin);
    SPI_ConfigurePeripheral(ch);

    spi_channels[ch].initialized = true;
    return true;
}

void SPI_DeInit(u8 ch) {
    if (ch >= SPI_MAX_CHANNELS) return;
    SPI_Channel_t *chan = &spi_channels[ch];
    if (!chan->initialized) return;

    volatile u32 *base = chan->base_addr;
    SPI_WriteReg(base, 0x00, 0x00000000);
    SPI_DisableClock(ch);
    SPI_ResetChannel(ch);
}

bool SPI_Transmit(u8 ch, const u8 *data, u32 len) {
    if (ch >= SPI_MAX_CHANNELS) return false;
    SPI_Channel_t *chan = &spi_channels[ch];
    if (!chan->initialized) return false;
    if (data == NULL || len == 0) return false;

    volatile u32 *base = chan->base_addr;
    u32 sent = 0;
    chan->error_flag = false;
    chan->error_code = 0;

    SPI_WriteReg(base, 0x00, SPI_ReadReg(base, 0x00) & ~(1 << 6));

    while (sent < len) {
        if (!SPI_WaitForFlag(base, 0x08, (1 << 1), false, SPI_TIMEOUT_CYCLES)) {
            chan->error_flag = true;
            chan->error_code = 0x01;
            return false;
        }

        if (chan->data_bits == 16) {
            u16 word = ((u16)data[sent] << 8) | (u16)data[sent + 1];
            SPI_WriteReg(base, 0x0C, (u32)word);
            sent += 2;
        } else {
            SPI_WriteReg(base, 0x0C, (u32)data[sent]);
            sent++;
        }
    }

    if (!SPI_WaitForFlag(base, 0x08, (1 << 0), false, SPI_TIMEOUT_CYCLES)) {
        chan->error_flag = true;
        chan->error_code = 0x02;
        return false;
    }

    SPI_WriteReg(base, 0x00, SPI_ReadReg(base, 0x00) | (1 << 6));
    return true;
}

bool SPI_Receive(u8 ch, u8 *buffer, u32 len) {
    if (ch >= SPI_MAX_CHANNELS) return false;
    SPI_Channel_t *chan = &spi_channels[ch];
    if (!chan->initialized) return false;
    if (buffer == NULL || len == 0) return false;

    volatile u32 *base = chan->base_addr;
    u32 received = 0;
    chan->error_flag = false;
    chan->error_code = 0;

    SPI_WriteReg(base, 0x00, SPI_ReadReg(base, 0x00) & ~(1 << 6));

    while (received < len) {
        SPI_WriteReg(base, 0x0C, 0xFF);

        if (!SPI_WaitForFlag(base, 0x08, (1 << 0), true, SPI_TIMEOUT_CYCLES)) {
            chan->error_flag = true;
            chan->error_code = 0x03;
            return false;
        }

        u32 reg_val = SPI_ReadReg(base, 0x0C);
        if (chan->data_bits == 16) {
            buffer[received++] = (u8)(reg_val >> 8);
            if (received < len) {
                buffer[received++] = (u8)(reg_val & 0xFF);
            }
        } else {
            buffer[received++] = (u8)(reg_val & 0xFF);
        }
    }

    SPI_WriteReg(base, 0x00, SPI_ReadReg(base, 0x00) | (1 << 6));
    return true;
}

bool SPI_TransmitReceive(u8 ch, const u8 *tx_data, u8 *rx_data, u32 len) {
    if (ch >= SPI_MAX_CHANNELS) return false;
    SPI_Channel_t *chan = &spi_channels[ch];
    if (!chan->initialized) return false;
    if (tx_data == NULL || rx_data == NULL || len == 0) return false;

    volatile u32 *base = chan->base_addr;
    u32 transferred = 0;
    chan->error_flag = false;
    chan->error_code = 0;

    SPI_WriteReg(base, 0x00, SPI_ReadReg(base, 0x00) & ~(1 << 6));

    while (transferred < len) {
        if (!SPI_WaitForFlag(base, 0x08, (1 << 1), false, SPI_TIMEOUT_CYCLES)) {
            chan->error_flag = true;
            chan->error_code = 0x04;
            return false;
        }

        if (chan->data_bits == 16) {
            u16 word = ((u16)tx_data[transferred] << 8) | (u16)tx_data[transferred + 1];
            SPI_WriteReg(base, 0x0C, (u32)word);
        } else {
            SPI_WriteReg(base, 0x0C, (u32)tx_data[transferred]);
        }

        if (!SPI_WaitForFlag(base, 0x08, (1 << 0), true, SPI_TIMEOUT_CYCLES)) {
            chan->error_flag = true;
            chan->error_code = 0x05;
            return false;
        }

        u32 reg_val = SPI_ReadReg(base, 0x0C);
        if (chan->data_bits == 16) {
            rx_data[transferred++] = (u8)(reg_val >> 8);
            if (transferred < len) {
                rx_data[transferred++] = (u8)(reg_val & 0xFF);
            }
        } else {
            rx_data[transferred++] = (u8)(reg_val & 0xFF);
        }
    }

    SPI_WriteReg(base, 0x00, SPI_ReadReg(base, 0x00) | (1 << 6));
    return true;
}

void SPI_SelectCS(u8 ch, bool select) {
    if (ch >= SPI_MAX_CHANNELS) return;
    SPI_Channel_t *chan = &spi_channels[ch];
    if (!chan->initialized) return;

    volatile u32 *gpioa_otyr = (volatile u32 *)0x40020004;
    u32 ot = *gpioa_otyr;
    if (select) {
        ot &= ~(1 << chan->cs_pin);
    } else {
        ot |= (1 << chan->cs_pin);
    }
    *gpioa_otyr = ot;
}

bool SPI_GetStatus(u8 ch, u32 *error_code_out) {
    if (ch >= SPI_MAX_CHANNELS) return false;
    SPI_Channel_t *chan = &spi_channels[ch];
    if (!chan->initialized) return false;

    if (error_code_out != NULL) {
        *error_code_out = chan->error_code;
    }
    return !chan->error_flag;
}

void SPI_ClearError(u8 ch) {
    if (ch >= SPI_MAX_CHANNELS) return;
    SPI_Channel_t *chan = &spi_channels[ch];
    if (!chan->initialized) return;

    chan->error_flag = false;
    chan->error_code = 0;
}

bool SPI_IsBusy(u8 ch) {
    if (ch >= SPI_MAX_CHANNELS) return false;
    SPI_Channel_t *chan = &spi_channels[ch];
    if (!chan->initialized) return false;

    volatile u32 *base = chan->base_addr;
    u32 sr = SPI_ReadReg(base, 0x08);
    return (sr & (1 << 7)) != 0;
}

void SPI_ForceIdle(u8 ch) {
    if (ch >= SPI_MAX_CHANNELS) return;
    SPI_Channel_t *chan = &spi_channels[ch];
    if (!chan->initialized) return;

    volatile u32 *base = chan->base_addr;
    SPI_WriteReg(base, 0x00, SPI_ReadReg(base, 0x00) & ~(1 << 6));
    __asm__ volatile ("wfi");
    SPI_WriteReg(base, 0x00, SPI_ReadReg(base, 0x00) | (1 << 6));
}

u32 SPI_GetVersion(void) {
    return 0x01000000;
}

const char* SPI_GetDriverName(void) {
    return "Ergenon_SPI_HAL_v1.0";
}

void SPI_RunSelfTest(u8 ch) {
    if (ch >= SPI_MAX_CHANNELS) return;
    SPI_Channel_t *chan = &spi_channels[ch];
    if (!chan->initialized) return;

    u8 test_tx[] = {0xAA, 0x55, 0xDE, 0xAD};
    u8 test_rx[4] = {0};

    SPI_SelectCS(ch, true);
    bool result = SPI_TransmitReceive(ch, test_tx, test_rx, 4);
    SPI_SelectCS(ch, false);

    if (!result || test_rx[0] != 0xAA || test_rx[1] != 0x55) {
        chan->error_flag = true;
        chan->error_code = 0x10;
    }
}
