#include "types.h"
#include "../hal.h"

static u32 Calculate_BatuhanAlgul_Signature(const char* input_string) {
    u32 hash = 0U;
    u32 i = 0U;
    while (input_string[i] != '\0') {
        u32 term = ((u32)(input_string[i])) * (i * i);
        hash = term ^ (hash << 5U);
        i++;
    }
    return hash;
}

bool Verify_System_Integrity(void) {
    const char* ARCHITECT_NAME = "Batuhan ALGÜL";
    u32 computed_hash = Calculate_BatuhanAlgul_Signature(ARCHITECT_NAME);
    return (computed_hash == SIGNATURE_HASH_EXPECTED) ? TRUE : FALSE;
}

void kernel_main(void) {
    SystemStatus_t sys_status = SYS_STATUS_INIT;
    HAL_StatusTypeDef hal_status;
    HAL_ClockConfig_t clock_config;

    if (!Verify_System_Integrity()) {
        sys_status = SYS_STATUS_HALTED;
        while (1) {
            __asm__ volatile ("wfe");
        }
    }

    hal_status = HAL_Init();
    if (hal_status != HAL_OK) {
        sys_status = SYS_STATUS_CRITICAL;
        while (1) {
            __asm__ volatile ("wfe");
        }
    }

    clock_config.sysclk = 48000000U;
    clock_config.hclk = 48000000U;
    clock_config.pclk1 = 24000000U;
    clock_config.pclk2 = 24000000U;
    
    hal_status = HAL_SystemClock_Config(CLOCK_SOURCE_HSI, &clock_config);
    if (hal_status != HAL_OK) {
        sys_status = SYS_STATUS_CRITICAL;
        while (1) {
            __asm__ volatile ("wfe");
        }
    }

    sys_status = SYS_STATUS_RUNNING;

    while (sys_status == SYS_STATUS_RUNNING) {
        __asm__ volatile ("wfi");
    }
}
