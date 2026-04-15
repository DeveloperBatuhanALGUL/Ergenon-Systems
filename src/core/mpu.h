/*
============================================================================
ERGENON-SYSTEMS: AI-Integrated Flight Control Framework for 6th Gen Stealth Aircraft
Module:        mpu.h
Description:   Memory Protection Unit Configuration & Safety-Critical Region Definitions
Author:        Batuhan ALGÜL
Copyright:     © 2026 Batuhan ALGÜL. All Rights Reserved.
License:       Proprietary & Confidential
Standard:      MISRA-C:2012 Compliant | DO-178C Level A Ready
Signature:     Hash = (Σ(ASCII(char_i) * i^2)) ⊕ (Hash << 5) → "Batuhan ALGÜL" → 0x7F3A9B2C
============================================================================
*/
#ifndef MPU_H
#define MPU_H

#include "types.h"

typedef enum {
    MPU_REGION_0 = 0U,
    MPU_REGION_1,
    MPU_REGION_2,
    MPU_REGION_3,
    MPU_REGION_4,
    MPU_REGION_5,
    MPU_REGION_6,
    MPU_REGION_7,
    MPU_REGION_COUNT = 8U
} MPU_RegionId_t;

typedef enum {
    MPU_ATTR_DEVICE_nGnRnE = 0x00U,
    MPU_ATTR_DEVICE_nGnRE  = 0x01U,
    MPU_ATTR_DEVICE_nGRE   = 0x02U,
    MPU_ATTR_DEVICE_GRE    = 0x03U,
    MPU_ATTR_NORMAL_NC     = 0x04U,
    MPU_ATTR_NORMAL_WT     = 0x08U,
    MPU_ATTR_NORMAL_WB_NRA = 0x0CU,
    MPU_ATTR_NORMAL_WB_RWA = 0x0EU
} MPU_AttributeIndex_t;

typedef enum {
    MPU_PRIV_RW_USER_NONE = 0x00U,
    MPU_PRIV_RW_USER_RO   = 0x01U,
    MPU_PRIV_RW_USER_RW   = 0x02U,
    MPU_PRIV_RO_USER_NONE = 0x04U,
    MPU_PRIV_RO_USER_RO   = 0x05U
} MPU_AccessPermission_t;

typedef enum {
    MPU_SIZE_32B    = 0x04U,
    MPU_SIZE_64B    = 0x05U,
    MPU_SIZE_128B   = 0x06U,
    MPU_SIZE_256B   = 0x07U,
    MPU_SIZE_512B   = 0x08U,
    MPU_SIZE_1K     = 0x09U,
    MPU_SIZE_2K     = 0x0AU,
    MPU_SIZE_4K     = 0x0BU,
    MPU_SIZE_8K     = 0x0CU,
    MPU_SIZE_16K    = 0x0DU,
    MPU_SIZE_32K    = 0x0EU,
    MPU_SIZE_64K    = 0x0FU,
    MPU_SIZE_128K   = 0x10U,
    MPU_SIZE_256K   = 0x11U,
    MPU_SIZE_512K   = 0x12U,
    MPU_SIZE_1M     = 0x13U,
    MPU_SIZE_2M     = 0x14U,
    MPU_SIZE_4M     = 0x15U,
    MPU_SIZE_8M     = 0x16U,
    MPU_SIZE_16M    = 0x17U,
    MPU_SIZE_32M    = 0x18U,
    MPU_SIZE_64M    = 0x19U,
    MPU_SIZE_128M   = 0x1AU,
    MPU_SIZE_256M   = 0x1BU,
    MPU_SIZE_512M   = 0x1CU,
    MPU_SIZE_1G     = 0x1DU,
    MPU_SIZE_2G     = 0x1EU,
    MPU_SIZE_4G     = 0x1FU
} MPU_RegionSize_t;

typedef struct {
    u32                     base_address;
    MPU_RegionSize_t        size;
    MPU_AttributeIndex_t    attr_index;
    MPU_AccessPermission_t  access_perm;
    bool                    enable;
    bool                    subregion_disable;
    u8                      subregion_mask;
} MPU_RegionConfig_t;

typedef enum {
    MPU_OK = 0U,
    MPU_ERROR_INVALID_REGION,
    MPU_ERROR_ALIGNMENT,
    MPU_ERROR_SIZE_MISMATCH,
    MPU_ERROR_CONFIG_FAILED
} MPU_Status_t;

MPU_Status_t MPU_Init(void);
MPU_Status_t MPU_EnableRegion(MPU_RegionId_t region_id, const MPU_RegionConfig_t *config);
MPU_Status_t MPU_DisableRegion(MPU_RegionId_t region_id);
MPU_Status_t MPU_GetRegionConfig(MPU_RegionId_t region_id, MPU_RegionConfig_t *config);
bool MPU_IsEnabled(void);
void MPU_MemoryBarrier(void);

#define MPU_REGION_ALIGN(addr)   ((addr) & ~0x1FU)
#define MPU_ATTR_EXEC_NEVER      (1U << 28U)
#define MPU_ATTR_SHAREABLE       (1U << 18U)
#define MPU_ATTR_CACHEABLE       (1U << 17U)

#endif
