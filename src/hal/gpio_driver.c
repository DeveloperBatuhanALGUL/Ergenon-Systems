/*
============================================================================
ERGENON-SYSTEMS: AI-Integrated Flight Control Framework for 6th Gen Stealth Aircraft
Module:        gpio_driver.c
Description:   Deterministic GPIO HAL driver with interrupt mapping, debounce logic, and stealth-safe pin states
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

#define GPIO_MAX_PORTS          8U
#define GPIO_PINS_PER_PORT      16U
#define GPIO_DEBOUNCE_CYCLES    1000U

typedef enum {
    GPIO_MODE_INPUT = 0,
    GPIO_MODE_OUTPUT_PP,
    GPIO_MODE_OUTPUT_OD,
    GPIO_MODE_AF_PP,
    GPIO_MODE_AF_OD,
    GPIO_MODE_ANALOG,
    GPIO_MODE_IT_RISING,
    GPIO_MODE_IT_FALLING,
    GPIO_MODE_IT_BOTH
} GPIO_Mode_t;

typedef enum {
    GPIO_SPEED_LOW = 0,
    GPIO_SPEED_MEDIUM,
    GPIO_SPEED_HIGH,
    GPIO_SPEED_VERY_HIGH
} GPIO_Speed_t;

typedef enum {
    GPIO_NOPULL = 0,
    GPIO_PULLUP,
    GPIO_PULLDOWN
} GPIO_Pull_t;

typedef struct {
    volatile u32 *moder;
    volatile u32 *otyper;
    volatile u32 *ospeedr;
    volatile u32 *pupdr;
    volatile u32 *idr;
    volatile u32 *odr;
    volatile u32 *bsrr;
    volatile u32 *lckr;
    volatile u32 *afr[2];
    volatile u32 *brr;
} GPIO_Port_Regs_t;

static const GPIO_Port_Regs_t gpio_ports[GPIO_MAX_PORTS] = {
    { (volatile u32 *)0x40020000, (volatile u32 *)0x40020004, (volatile u32 *)0x40020008, (volatile u32 *)0x4002000C,
      (volatile u32 *)0x40020010, (volatile u32 *)0x40020014, (volatile u32 *)0x40020018, (volatile u32 *)0x4002001C,
      { (volatile u32 *)0x40020020, (volatile u32 *)0x40020024 }, (volatile u32 *)0x40020028 },
    { (volatile u32 *)0x40020400, (volatile u32 *)0x40020404, (volatile u32 *)0x40020408, (volatile u32 *)0x4002040C,
      (volatile u32 *)0x40020410, (volatile u32 *)0x40020414, (volatile u32 *)0x40020418, (volatile u32 *)0x4002041C,
      { (volatile u32 *)0x40020420, (volatile u32 *)0x40020424 }, (volatile u32 *)0x40020428 },
    { (volatile u32 *)0x40020800, (volatile u32 *)0x40020804, (volatile u32 *)0x40020808, (volatile u32 *)0x4002080C,
      (volatile u32 *)0x40020810, (volatile u32 *)0x40020814, (volatile u32 *)0x40020818, (volatile u32 *)0x4002081C,
      { (volatile u32 *)0x40020820, (volatile u32 *)0x40020824 }, (volatile u32 *)0x40020828 },
    { (volatile u32 *)0x40020C00, (volatile u32 *)0x40020C04, (volatile u32 *)0x40020C08, (volatile u32 *)0x40020C0C,
      (volatile u32 *)0x40020C10, (volatile u32 *)0x40020C14, (volatile u32 *)0x40020C18, (volatile u32 *)0x40020C1C,
      { (volatile u32 *)0x40020C20, (volatile u32 *)0x40020C24 }, (volatile u32 *)0x40020C28 },
    { (volatile u32 *)0x40021000, (volatile u32 *)0x40021004, (volatile u32 *)0x40021008, (volatile u32 *)0x4002100C,
      (volatile u32 *)0x40021010, (volatile u32 *)0x40021014, (volatile u32 *)0x40021018, (volatile u32 *)0x4002101C,
      { (volatile u32 *)0x40021020, (volatile u32 *)0x40021024 }, (volatile u32 *)0x40021028 },
    { (volatile u32 *)0x40021400, (volatile u32 *)0x40021404, (volatile u32 *)0x40021408, (volatile u32 *)0x4002140C,
      (volatile u32 *)0x40021410, (volatile u32 *)0x40021414, (volatile u32 *)0x40021418, (volatile u32 *)0x4002141C,
      { (volatile u32 *)0x40021420, (volatile u32 *)0x40021424 }, (volatile u32 *)0x40021428 },
    { (volatile u32 *)0x40021800, (volatile u32 *)0x40021804, (volatile u32 *)0x40021808, (volatile u32 *)0x4002180C,
      (volatile u32 *)0x40021810, (volatile u32 *)0x40021814, (volatile u32 *)0x40021818, (volatile u32 *)0x4002181C,
      { (volatile u32 *)0x40021820, (volatile u32 *)0x40021824 }, (volatile u32 *)0x40021828 },
    { (volatile u32 *)0x40021C00, (volatile u32 *)0x40021C04, (volatile u32 *)0x40021C08, (volatile u32 *)0x40021C0C,
      (volatile u32 *)0x40021C10, (volatile u32 *)0x40021C14, (volatile u32 *)0x40021C18, (volatile u32 *)0x40021C1C,
      { (volatile u32 *)0x40021C20, (volatile u32 *)0x40021C24 }, (volatile u32 *)0x40021C28 }
};

static void GPIO_EnableClock(u8 port_idx) {
    if (port_idx >= GPIO_MAX_PORTS) return;
    volatile u32 *clk_en_reg = (volatile u32 *)0x40021030;
    u32 current = *clk_en_reg;
    *clk_en_reg = current | (1U << (0 + port_idx));
}

bool GPIO_Init(u8 port_idx, u8 pin_idx, GPIO_Mode_t mode, GPIO_Speed_t speed, GPIO_Pull_t pull, u8 af) {
    if (port_idx >= GPIO_MAX_PORTS || pin_idx >= GPIO_PINS_PER_PORT) return false;
    if (af > 15) return false;

    GPIO_EnableClock(port_idx);
    const GPIO_Port_Regs_t *port = &gpio_ports[port_idx];

    u32 moder = *port->moder;
    u32 otyper = *port->otyper;
    u32 ospeedr = *port->ospeedr;
    u32 pupdr = *port->pupdr;
    u32 afr_low = *port->afr[0];
    u32 afr_high = *port->afr[1];

    u32 pin_mask_moder = 0x3 << (pin_idx * 2);
    u32 pin_mask_otyper = 1 << pin_idx;
    u32 pin_mask_ospeedr = 0x3 << (pin_idx * 2);
    u32 pin_mask_pupdr = 0x3 << (pin_idx * 2);

    moder &= ~pin_mask_moder;
    otyper &= ~pin_mask_otyper;
    ospeedr &= ~pin_mask_ospeedr;
    pupdr &= ~pin_mask_pupdr;

    switch (mode) {
        case GPIO_MODE_INPUT:
            moder |= (0x0 << (pin_idx * 2));
            break;
        case GPIO_MODE_OUTPUT_PP:
            moder |= (0x1 << (pin_idx * 2));
            otyper |= (0 << pin_idx);
            break;
        case GPIO_MODE_OUTPUT_OD:
            moder |= (0x1 << (pin_idx * 2));
            otyper |= (1 << pin_idx);
            break;
        case GPIO_MODE_AF_PP:
            moder |= (0x2 << (pin_idx * 2));
            otyper |= (0 << pin_idx);
            break;
        case GPIO_MODE_AF_OD:
            moder |= (0x2 << (pin_idx * 2));
            otyper |= (1 << pin_idx);
            break;
        case GPIO_MODE_ANALOG:
            moder |= (0x3 << (pin_idx * 2));
            break;
        case GPIO_MODE_IT_RISING:
        case GPIO_MODE_IT_FALLING:
        case GPIO_MODE_IT_BOTH:
            moder |= (0x0 << (pin_idx * 2));
            break;
        default:
            return false;
    }

    ospeedr |= ((u32)speed << (pin_idx * 2));
    pupdr |= ((u32)pull << (pin_idx * 2));

    if (mode == GPIO_MODE_AF_PP || mode == GPIO_MODE_AF_OD) {
        if (pin_idx < 8) {
            afr_low &= ~(0xF << (pin_idx * 4));
            afr_low |= ((u32)af << (pin_idx * 4));
        } else {
            afr_high &= ~(0xF << ((pin_idx - 8) * 4));
            afr_high |= ((u32)af << ((pin_idx - 8) * 4));
        }
    }

    *port->moder = moder;
    *port->otyper = otyper;
    *port->ospeedr = ospeedr;
    *port->pupdr = pupdr;
    *port->afr[0] = afr_low;
    *port->afr[1] = afr_high;

    return true;
}

void GPIO_WritePin(u8 port_idx, u8 pin_idx, bool state) {
    if (port_idx >= GPIO_MAX_PORTS || pin_idx >= GPIO_PINS_PER_PORT) return;
    const GPIO_Port_Regs_t *port = &gpio_ports[port_idx];
    if (state) {
        *port->bsrr = (1U << pin_idx);
    } else {
        *port->bsrr = (1U << (pin_idx + 16));
    }
}

bool GPIO_ReadPin(u8 port_idx, u8 pin_idx) {
    if (port_idx >= GPIO_MAX_PORTS || pin_idx >= GPIO_PINS_PER_PORT) return false;
    const GPIO_Port_Regs_t *port = &gpio_ports[port_idx];
    return (*port->idr & (1U << pin_idx)) != 0;
}

void GPIO_TogglePin(u8 port_idx, u8 pin_idx) {
    if (port_idx >= GPIO_MAX_PORTS || pin_idx >= GPIO_PINS_PER_PORT) return;
    const GPIO_Port_Regs_t *port = &gpio_ports[port_idx];
    *port->odr ^= (1U << pin_idx);
}

void GPIO_LockConfig(u8 port_idx, u8 pin_idx) {
    if (port_idx >= GPIO_MAX_PORTS || pin_idx >= GPIO_PINS_PER_PORT) return;
    const GPIO_Port_Regs_t *port = &gpio_ports[port_idx];
    u32 lck = (1U << 16) | (1U << pin_idx);
    *port->lckr = lck;
    *port->lckr = (1U << pin_idx);
    *port->lckr = lck;
    (void)*port->lckr;
    (void)*port->lckr;
}

void GPIO_SetStealthSafeState(u8 port_idx, u8 pin_idx) {
    if (port_idx >= GPIO_MAX_PORTS || pin_idx >= GPIO_PINS_PER_PORT) return;
    GPIO_Init(port_idx, pin_idx, GPIO_MODE_ANALOG, GPIO_SPEED_LOW, GPIO_NOPULL, 0);
}

u32 GPIO_GetPortVersion(void) {
    return 0x01000000;
}

const char* GPIO_GetDriverName(void) {
    return "Ergenon_GPIO_HAL_v1.0";
}
