#include "hal.h"

HAL_StatusTypeDef HAL_Init(void) {
    return HAL_OK;
}

HAL_StatusTypeDef HAL_SystemClock_Config(HAL_ClockSource_t source, HAL_ClockConfig_t *config) {
    if (config == ((HAL_ClockConfig_t *)0)) {
        return HAL_ERROR;
    }
    config->sysclk = 48000000U;
    config->hclk = 48000000U;
    config->pclk1 = 24000000U;
    config->pclk2 = 24000000U;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_GPIO_Init(u32 port, HAL_GPIO_InitTypeDef *gpio_init) {
    if (gpio_init == ((HAL_GPIO_InitTypeDef *)0)) {
        return HAL_ERROR;
    }
    return HAL_OK;
}

HAL_StatusTypeDef HAL_GPIO_WritePin(u32 port, u32 pin, u8 state) {
    return HAL_OK;
}

u8 HAL_GPIO_ReadPin(u32 port, u32 pin) {
    return 0U;
}

HAL_StatusTypeDef HAL_Delay(u32 delay_ms) {
    volatile u32 count = delay_ms * 1000U;
    while (count > 0U) {
        count--;
    }
    return HAL_OK;
}
