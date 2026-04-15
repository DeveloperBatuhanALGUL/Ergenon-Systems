#ifndef HAL_H
#define HAL_H

#include "types.h"

typedef enum {
    HAL_OK = 0,
    HAL_ERROR,
    HAL_BUSY,
    HAL_TIMEOUT
} HAL_StatusTypeDef;

typedef enum {
    CLOCK_SOURCE_HSI,
    CLOCK_SOURCE_HSE,
    CLOCK_SOURCE_PLL
} HAL_ClockSource_t;

typedef struct {
    u32 sysclk;
    u32 hclk;
    u32 pclk1;
    u32 pclk2;
} HAL_ClockConfig_t;

typedef enum {
    GPIO_MODE_INPUT,
    GPIO_MODE_OUTPUT_PP,
    GPIO_MODE_OUTPUT_OD,
    GPIO_MODE_AF_PP,
    GPIO_MODE_AF_OD,
    GPIO_MODE_ANALOG
} HAL_GPIO_Mode_t;

typedef enum {
    GPIO_SPEED_LOW,
    GPIO_SPEED_MEDIUM,
    GPIO_SPEED_HIGH,
    GPIO_SPEED_VERY_HIGH
} HAL_GPIO_Speed_t;

typedef enum {
    GPIO_PULL_NONE,
    GPIO_PULL_UP,
    GPIO_PULL_DOWN
} HAL_GPIO_Pull_t;

typedef struct {
    u32 pin;
    HAL_GPIO_Mode_t mode;
    HAL_GPIO_Speed_t speed;
    HAL_GPIO_Pull_t pull;
} HAL_GPIO_InitTypeDef;

HAL_StatusTypeDef HAL_Init(void);
HAL_StatusTypeDef HAL_SystemClock_Config(HAL_ClockSource_t source, HAL_ClockConfig_t *config);
HAL_StatusTypeDef HAL_GPIO_Init(u32 port, HAL_GPIO_InitTypeDef *gpio_init);
HAL_StatusTypeDef HAL_GPIO_WritePin(u32 port, u32 pin, u8 state);
u8 HAL_GPIO_ReadPin(u32 port, u32 pin);
HAL_StatusTypeDef HAL_Delay(u32 delay_ms);

#endif
