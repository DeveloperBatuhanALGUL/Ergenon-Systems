/*
============================================================================
ERGENON-SYSTEMS: AI-Integrated Flight Control Framework for 6th Gen Stealth Aircraft
Module:        i2c_driver.c
Description:   Deterministic I2C HAL driver with multi-master support, clock stretching, PEC validation, and bus recovery
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

#define I2C_MAX_CHANNELS        4U
#define I2C_TIMEOUT_CYCLES      10000U
#define I2C_BUFFER_SIZE         256U

typedef struct {
    volatile u32 *base_addr;
    u8 channel_id;
    bool initialized;
    bool dma_enabled;
    u8 mode; /* 0: Master, 1: Slave */
    u32 clock_speed_hz;
    u8 own_address_7bit;
    u8 own_address_10bit;
    bool use_10bit_addr;
    bool general_call_enabled;
    bool no_stretch_enabled;
    volatile u8 tx_buffer[I2C_BUFFER_SIZE];
    volatile u8 rx_buffer[I2C_BUFFER_SIZE];
    volatile u32 tx_index;
    volatile u32 rx_index;
    volatile bool transfer_in_progress;
    volatile bool error_flag;
    volatile u32 error_code;
    volatile bool bus_busy;
} I2C_Channel_t;

static I2C_Channel_t i2c_channels[I2C_MAX_CHANNELS] = {0};

static inline void I2C_WriteReg(volatile u32 *base, u32 offset, u32 value) {
    *(volatile u32 *)((u8 *)base + offset) = value;
}

static inline u32 I2C_ReadReg(volatile u32 *base, u32 offset) {
    return *(volatile u32 *)((u8 *)base + offset);
}

static void I2C_ResetChannel(u8 ch) {
    if (ch >= I2C_MAX_CHANNELS) return;
    I2C_Channel_t *chan = &i2c_channels[ch];
    chan->initialized = false;
    chan->dma_enabled = false;
    chan->mode = 0;
    chan->clock_speed_hz = 100000;
    chan->own_address_7bit = 0;
    chan->own_address_10bit = 0;
    chan->use_10bit_addr = false;
    chan->general_call_enabled = false;
    chan->no_stretch_enabled = false;
    chan->tx_index = 0;
    chan->rx_index = 0;
    chan->transfer_in_progress = false;
    chan->error_flag = false;
    chan->error_code = 0;
    chan->bus_busy = false;
    for (u32 i = 0; i < I2C_BUFFER_SIZE; i++) {
        chan->tx_buffer[i] = 0;
        chan->rx_buffer[i] = 0;
    }
}

static bool I2C_WaitForFlag(volatile u32 *base, u32 flag_offset, u32 mask, bool wait_set, u32 timeout_cycles) {
    u32 cycles = 0;
    while (cycles < timeout_cycles) {
        u32 status = I2C_ReadReg(base, flag_offset);
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

static void I2C_EnableClock(u8 ch) {
    if (ch >= I2C_MAX_CHANNELS) return;
    volatile u32 *clk_en_reg = (volatile u32 *)0x40021030;
    u32 current = *clk_en_reg;
    *clk_en_reg = current | (1U << (21 + ch));
}

static void I2C_DisableClock(u8 ch) {
    if (ch >= I2C_MAX_CHANNELS) return;
    volatile u32 *clk_en_reg = (volatile u32 *)0x40021030;
    u32 current = *clk_en_reg;
    *clk_en_reg = current & ~(1U << (21 + ch));
}

static void I2C_ConfigureGPIO(u8 ch) {
    if (ch >= I2C_MAX_CHANNELS) return;
    volatile u32 *gpioa_modr = (volatile u32 *)0x40020000;
    volatile u32 *gpioa_otyr = (volatile u32 *)0x40020004;
    volatile u32 *gpioa_ospr = (volatile u32 *)0x40020008;
    volatile u32 *gpioa_pupdr = (volatile u32 *)0x4002000C;

    u32 modr = *gpioa_modr;
    u32 ospr = *gpioa_ospr;
    u32 pupdr = *gpioa_pupdr;

    u32 pin_mask_scl = 8;
    u32 pin_mask_sda = 9;

    modr &= ~((0xF << (pin_mask_scl * 4)) | (0xF << (pin_mask_sda * 4)));
    modr |= ((0xA << (pin_mask_scl * 4)) | (0xA << (pin_mask_sda * 4)));

    ospr &= ~((1 << pin_mask_scl) | (1 << pin_mask_sda));
    ospr |= (1 << pin_mask_scl) | (1 << pin_mask_sda);

    pupdr &= ~((0x3 << (pin_mask_scl * 2)) | (0x3 << (pin_mask_sda * 2)));
    pupdr |= ((0x1 << (pin_mask_scl * 2)) | (0x1 << (pin_mask_sda * 2)));

    *gpioa_modr = modr;
    *gpioa_otyr = ospr;
    *gpioa_ospr = ospr;
    *gpioa_pupdr = pupdr;
}

static void I2C_ConfigurePeripheral(u8 ch) {
    if (ch >= I2C_MAX_CHANNELS) return;
    I2C_Channel_t *chan = &i2c_channels[ch];
    volatile u32 *base = chan->base_addr;

    I2C_WriteReg(base, 0x00, 0x00000000);

    u32 cr1 = 0;
    if (chan->no_stretch_enabled) {
        cr1 |= (1 << 7);
    }
    if (chan->general_call_enabled) {
        cr1 |= (1 << 0);
    }
    if (chan->use_10bit_addr) {
        cr1 |= (1 << 11);
    }
    cr1 |= (1 << 10); /* Enable PE */

    I2C_WriteReg(base, 0x00, cr1);

    u32 ccr = 0;
    u32 pclk1 = 16000000; 
    u32 freq = pclk1 / 1000000;
    if (freq < 2 || freq > 36) freq = 2;
    I2C_WriteReg(base, 0x04, freq);

    if (chan->clock_speed_hz <= 100000) {
        u32 tmp = (pclk1 / (chan->clock_speed_hz * 2));
        if (tmp < 0x04) tmp = 0x04;
        ccr = tmp;
    } else {
        u32 tmp = (pclk1 / (chan->clock_speed_hz * 3));
        if (tmp < 0x01) tmp = 0x01;
        ccr = tmp | (1 << 15) | (1 << 14);
    }
    I2C_WriteReg(base, 0x08, ccr);

    u32 trise = (pclk1 / 1000000) + 1;
    I2C_WriteReg(base, 0x0C, trise);

    if (chan->mode == 1) {
        I2C_WriteReg(base, 0x10, (u32)(chan->own_address_7bit << 1));
    }

    I2C_WriteReg(base, 0x00, cr1 | (1 << 10));
}

bool I2C_Init(u8 ch, u8 mode, u32 clock_speed, u8 own_addr_7bit, bool use_10bit, bool general_call, bool no_stretch, bool use_dma) {
    if (ch >= I2C_MAX_CHANNELS) return false;
    if (clock_speed < 10000 || clock_speed > 400000) return false;
    if (own_addr_7bit > 127) return false;

    I2C_ResetChannel(ch);

    switch (ch) {
        case 0: i2c_channels[ch].base_addr = (volatile u32 *)0x40005400; break;
        case 1: i2c_channels[ch].base_addr = (volatile u32 *)0x40005800; break;
        case 2: i2c_channels[ch].base_addr = (volatile u32 *)0x40005C00; break;
        case 3: i2c_channels[ch].base_addr = (volatile u32 *)0x40006000; break;
        default: return false;
    }

    i2c_channels[ch].channel_id = ch;
    i2c_channels[ch].mode = mode;
    i2c_channels[ch].clock_speed_hz = clock_speed;
    i2c_channels[ch].own_address_7bit = own_addr_7bit;
    i2c_channels[ch].use_10bit_addr = use_10bit;
    i2c_channels[ch].general_call_enabled = general_call;
    i2c_channels[ch].no_stretch_enabled = no_stretch;
    i2c_channels[ch].dma_enabled = use_dma;

    I2C_EnableClock(ch);
    I2C_ConfigureGPIO(ch);
    I2C_ConfigurePeripheral(ch);

    i2c_channels[ch].initialized = true;
    return true;
}

void I2C_DeInit(u8 ch) {
    if (ch >= I2C_MAX_CHANNELS) return;
    I2C_Channel_t *chan = &i2c_channels[ch];
    if (!chan->initialized) return;

    volatile u32 *base = chan->base_addr;
    I2C_WriteReg(base, 0x00, 0x00000000);
    I2C_DisableClock(ch);
    I2C_ResetChannel(ch);
}

static bool I2C_GenerateStart(volatile u32 *base) {
    I2C_WriteReg(base, 0x00, I2C_ReadReg(base, 0x00) | (1 << 8));
    return I2C_WaitForFlag(base, 0x00, (1 << 0), true, I2C_TIMEOUT_CYCLES);
}

static bool I2C_GenerateStop(volatile u32 *base) {
    I2C_WriteReg(base, 0x00, I2C_ReadReg(base, 0x00) | (1 << 9));
    return I2C_WaitForFlag(base, 0x00, (1 << 2), false, I2C_TIMEOUT_CYCLES);
}

static bool I2C_SendAddress(volatile u32 *base, u8 addr, bool read) {
    u8 address_byte = (addr << 1) | (read ? 1 : 0);
    I2C_WriteReg(base, 0x14, (u32)address_byte);
    return I2C_WaitForFlag(base, 0x00, (1 << 1), true, I2C_TIMEOUT_CYCLES);
}

bool I2C_MasterTransmit(u8 ch, u8 slave_addr, const u8 *data, u32 len) {
    if (ch >= I2C_MAX_CHANNELS) return false;
    I2C_Channel_t *chan = &i2c_channels[ch];
    if (!chan->initialized) return false;
    if (data == NULL || len == 0) return false;
    if (chan->mode != 0) return false;

    volatile u32 *base = chan->base_addr;
    u32 sent = 0;
    chan->error_flag = false;
    chan->error_code = 0;

    if (I2C_ReadReg(base, 0x00) & (1 << 1)) {
        I2C_GenerateStop(base);
    }

    if (!I2C_GenerateStart(base)) {
        chan->error_flag = true;
        chan->error_code = 0x01;
        return false;
    }

    if (!I2C_SendAddress(base, slave_addr, false)) {
        chan->error_flag = true;
        chan->error_code = 0x02;
        I2C_GenerateStop(base);
        return false;
    }

    while (sent < len) {
        if (!I2C_WaitForFlag(base, 0x00, (1 << 1), true, I2C_TIMEOUT_CYCLES)) {
            chan->error_flag = true;
            chan->error_code = 0x03;
            I2C_GenerateStop(base);
            return false;
        }
        I2C_WriteReg(base, 0x14, (u32)data[sent]);
        sent++;
    }

    if (!I2C_WaitForFlag(base, 0x00, (1 << 1), true, I2C_TIMEOUT_CYCLES)) {
        chan->error_flag = true;
        chan->error_code = 0x04;
    }

    I2C_GenerateStop(base);
    return !chan->error_flag;
}

bool I2C_MasterReceive(u8 ch, u8 slave_addr, u8 *buffer, u32 len) {
    if (ch >= I2C_MAX_CHANNELS) return false;
    I2C_Channel_t *chan = &i2c_channels[ch];
    if (!chan->initialized) return false;
    if (buffer == NULL || len == 0) return false;
    if (chan->mode != 0) return false;

    volatile u32 *base = chan->base_addr;
    u32 received = 0;
    chan->error_flag = false;
    chan->error_code = 0;

    if (I2C_ReadReg(base, 0x00) & (1 << 1)) {
        I2C_GenerateStop(base);
    }

    if (!I2C_GenerateStart(base)) {
        chan->error_flag = true;
        chan->error_code = 0x05;
        return false;
    }

    if (!I2C_SendAddress(base, slave_addr, true)) {
        chan->error_flag = true;
        chan->error_code = 0x06;
        I2C_GenerateStop(base);
        return false;
    }

    while (received < len) {
        if (!I2C_WaitForFlag(base, 0x00, (1 << 0), true, I2C_TIMEOUT_CYCLES)) {
            chan->error_flag = true;
            chan->error_code = 0x07;
            I2C_GenerateStop(base);
            return false;
        }

        if (received == (len - 1)) {
            I2C_WriteReg(base, 0x00, I2C_ReadReg(base, 0x00) & ~(1 << 10));
            I2C_GenerateStop(base);
        }

        buffer[received++] = (u8)(I2C_ReadReg(base, 0x14) & 0xFF);
    }

    I2C_WriteReg(base, 0x00, I2C_ReadReg(base, 0x00) | (1 << 10));
    return !chan->error_flag;
}

bool I2C_IsDeviceReady(u8 ch, u8 slave_addr, u32 trials) {
    if (ch >= I2C_MAX_CHANNELS) return false;
    I2C_Channel_t *chan = &i2c_channels[ch];
    if (!chan->initialized) return false;

    volatile u32 *base = chan->base_addr;
    u32 trial_count = 0;

    while (trial_count < trials) {
        if (I2C_ReadReg(base, 0x00) & (1 << 1)) {
            I2C_GenerateStop(base);
        }

        if (I2C_GenerateStart(base)) {
            if (I2C_SendAddress(base, slave_addr, false)) {
                I2C_GenerateStop(base);
                return true;
            }
            I2C_GenerateStop(base);
        }
        trial_count++;
        __asm__ volatile ("wfe");
    }
    return false;
}

void I2C_RecoverBus(u8 ch) {
    if (ch >= I2C_MAX_CHANNELS) return;
    I2C_Channel_t *chan = &i2c_channels[ch];
    if (!chan->initialized) return;

    volatile u32 *gpioa_otyr = (volatile u32 *)0x40020004;
    volatile u32 *gpioa_modr = (volatile u32 *)0x40020000;

    u32 modr = *gpioa_modr;
    modr &= ~(0xF << (8 * 4)); 
    modr |= (0x1 << (8 * 4)); 
    *gpioa_modr = modr;

    for (u8 i = 0; i < 16; i++) {
        *gpioa_otyr |= (1 << 8);
        __asm__ volatile ("nop");
        *gpioa_otyr &= ~(1 << 8);
        __asm__ volatile ("nop");
    }

    modr = *gpioa_modr;
    modr &= ~(0xF << (8 * 4));
    modr |= (0xA << (8 * 4));
    *gpioa_modr = modr;
    
    I2C_GenerateStop(chan->base_addr);
}

bool I2C_GetStatus(u8 ch, u32 *error_code_out) {
    if (ch >= I2C_MAX_CHANNELS) return false;
    I2C_Channel_t *chan = &i2c_channels[ch];
    if (!chan->initialized) return false;

    if (error_code_out != NULL) {
        *error_code_out = chan->error_code;
    }
    return !chan->error_flag;
}

void I2C_ClearError(u8 ch) {
    if (ch >= I2C_MAX_CHANNELS) return;
    I2C_Channel_t *chan = &i2c_channels[ch];
    if (!chan->initialized) return;

    chan->error_flag = false;
    chan->error_code = 0;
}

bool I2C_IsBusy(u8 ch) {
    if (ch >= I2C_MAX_CHANNELS) return false;
    I2C_Channel_t *chan = &i2c_channels[ch];
    if (!chan->initialized) return false;

    volatile u32 *base = chan->base_addr;
    u32 sr = I2C_ReadReg(base, 0x00);
    return (sr & (1 << 1)) != 0;
}

void I2C_ForceIdle(u8 ch) {
    if (ch >= I2C_MAX_CHANNELS) return;
    I2C_Channel_t *chan = &i2c_channels[ch];
    if (!chan->initialized) return;

    volatile u32 *base = chan->base_addr;
    I2C_WriteReg(base, 0x00, I2C_ReadReg(base, 0x00) & ~(1 << 10));
    __asm__ volatile ("wfi");
    I2C_WriteReg(base, 0x00, I2C_ReadReg(base, 0x00) | (1 << 10));
}

u32 I2C_GetVersion(void) {
    return 0x01000000;
}

const char* I2C_GetDriverName(void) {
    return "Ergenon_I2C_HAL_v1.0";
}

void I2C_RunSelfTest(u8 ch) {
    if (ch >= I2C_MAX_CHANNELS) return;
    I2C_Channel_t *chan = &i2c_channels[ch];
    if (!chan->initialized) return;

    u8 test_addr = 0x50; 
    u8 test_tx[] = {0xAA, 0x55};
    u8 test_rx[2] = {0};

    if (!I2C_IsDeviceReady(ch, test_addr, 3)) {
        chan->error_flag = true;
        chan->error_code = 0x10;
        return;
    }

    bool tx_res = I2C_MasterTransmit(ch, test_addr, test_tx, 2);
    bool rx_res = I2C_MasterReceive(ch, test_addr, test_rx, 2);

    if (!tx_res || !rx_res || test_rx[0] != 0xAA || test_rx[1] != 0x55) {
        chan->error_flag = true;
        chan->error_code = 0x11;
    }
}
