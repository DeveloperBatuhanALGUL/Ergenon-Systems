#include "mpu.h"

#define MPU_BASE_ADDR       0xE000ED90U
#define MPU_TYPE_REG        (*(volatile u32 *)(MPU_BASE_ADDR + 0x00U))
#define MPU_CTRL_REG        (*(volatile u32 *)(MPU_BASE_ADDR + 0x04U))
#define MPU_RNR_REG         (*(volatile u32 *)(MPU_BASE_ADDR + 0x08U))
#define MPU_RBAR_REG        (*(volatile u32 *)(MPU_BASE_ADDR + 0x0CU))
#define MPU_RASR_REG        (*(volatile u32 *)(MPU_BASE_ADDR + 0x10U))

#define MPU_CTRL_ENABLE     (1U << 0U)
#define MPU_CTRL_PRIVDEFENA (1U << 2U)
#define MPU_RBAR_VALID      (1U << 4U)
#define MPU_RASR_ENABLE     (1U << 0U)
#define MPU_RASR_XN         (1U << 28U)
#define MPU_RASR_AP_SHIFT   24U
#define MPU_RASR_TEX_SHIFT  19U
#define MPU_RASR_S_SHIFT    18U
#define MPU_RASR_C_SHIFT    17U
#define MPU_RASR_B_SHIFT    16U
#define MPU_RASR_SIZE_SHIFT 1U
#define MPU_RASR_SUBREG_SHIFT 8U

static MPU_Status_t Validate_Config(const MPU_RegionConfig_t *config) {
    if (config == ((MPU_RegionConfig_t *)0)) {
        return MPU_ERROR_INVALID_REGION;
    }
    if ((config->base_address & 0x1FU) != 0U) {
        return MPU_ERROR_ALIGNMENT;
    }
    if ((config->size < MPU_SIZE_32B) || (config->size > MPU_SIZE_4G)) {
        return MPU_ERROR_SIZE_MISMATCH;
    }
    return MPU_OK;
}

MPU_Status_t MPU_Init(void) {
    MPU_CTRL_REG = 0U;
    u8 i;
    for (i = 0U; i < MPU_REGION_COUNT; i++) {
        MPU_RNR_REG = i;
        MPU_RASR_REG = 0U;
    }
    MPU_CTRL_REG = MPU_CTRL_ENABLE | MPU_CTRL_PRIVDEFENA;
    __asm__ volatile ("dsb");
    __asm__ volatile ("isb");
    return MPU_OK;
}

MPU_Status_t MPU_EnableRegion(MPU_RegionId_t region_id, const MPU_RegionConfig_t *config) {
    MPU_Status_t status = Validate_Config(config);
    if (status != MPU_OK) {
        return status;
    }
    if (region_id >= MPU_REGION_COUNT) {
        return MPU_ERROR_INVALID_REGION;
    }

    u32 rbar_val = (config->base_address & 0xFFFFFFF0U) | MPU_RBAR_VALID | region_id;
    u32 rasr_val = 0U;

    rasr_val |= (config->attr_index & 0xFU) << MPU_RASR_TEX_SHIFT;
    if ((config->attr_index & 0x8U) != 0U) {
        rasr_val |= (1U << MPU_RASR_C_SHIFT);
    }
    if ((config->attr_index & 0x4U) != 0U) {
        rasr_val |= (1U << MPU_RASR_B_SHIFT);
    }

    rasr_val |= (config->access_perm & 0x7U) << MPU_RASR_AP_SHIFT;
    rasr_val |= (config->size & 0x1FU) << MPU_RASR_SIZE_SHIFT;

    if (config->subregion_disable) {
        rasr_val |= ((u32)(config->subregion_mask) & 0xFFU) << MPU_RASR_SUBREG_SHIFT;
    }

    rasr_val |= MPU_RASR_ENABLE;

    MPU_RNR_REG = region_id;
    MPU_RBAR_REG = rbar_val;
    MPU_RASR_REG = rasr_val;

    __asm__ volatile ("dsb");
    __asm__ volatile ("isb");

    return MPU_OK;
}

MPU_Status_t MPU_DisableRegion(MPU_RegionId_t region_id) {
    if (region_id >= MPU_REGION_COUNT) {
        return MPU_ERROR_INVALID_REGION;
    }
    MPU_RNR_REG = region_id;
    MPU_RASR_REG = 0U;
    __asm__ volatile ("dsb");
    __asm__ volatile ("isb");
    return MPU_OK;
}

MPU_Status_t MPU_GetRegionConfig(MPU_RegionId_t region_id, MPU_RegionConfig_t *config) {
    if (region_id >= MPU_REGION_COUNT) {
        return MPU_ERROR_INVALID_REGION;
    }
    if (config == ((MPU_RegionConfig_t *)0)) {
        return MPU_ERROR_INVALID_REGION;
    }

    MPU_RNR_REG = region_id;
    u32 rbar = MPU_RBAR_REG;
    u32 rasr = MPU_RASR_REG;

    config->base_address = rbar & 0xFFFFFFF0U;
    config->size = (MPU_RegionSize_t)((rasr >> MPU_RASR_SIZE_SHIFT) & 0x1FU);
    config->attr_index = (MPU_AttributeIndex_t)((rasr >> MPU_RASR_TEX_SHIFT) & 0xFU);
    config->access_perm = (MPU_AccessPermission_t)((rasr >> MPU_RASR_AP_SHIFT) & 0x7U);
    config->enable = (rasr & MPU_RASR_ENABLE) ? TRUE : FALSE;
    config->subregion_disable = ((rasr >> MPU_RASR_SUBREG_SHIFT) & 0xFFU) != 0U;
    config->subregion_mask = (u8)((rasr >> MPU_RASR_SUBREG_SHIFT) & 0xFFU);

    return MPU_OK;
}

bool MPU_IsEnabled(void) {
    return (MPU_CTRL_REG & MPU_CTRL_ENABLE) ? TRUE : FALSE;
}

void MPU_MemoryBarrier(void) {
    __asm__ volatile ("dsb");
    __asm__ volatile ("isb");
}
