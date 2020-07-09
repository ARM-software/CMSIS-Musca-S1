/*
 * Copyright (c) 2018-2020 Arm Limited. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef __FLASH_LAYOUT_H__
#define __FLASH_LAYOUT_H__

#include "mem_layout.h"

/* Size of a Secure and of a Non-secure image */
#define FLASH_S_PARTITION_SIZE          (REGION_IMAGE_1_S_SIZE)
#define FLASH_NS_PARTITION_SIZE         (REGION_IMAGE_1_NS_SIZE)
#define FLASH_MAX_PARTITION_SIZE        ((FLASH_S_PARTITION_SIZE >   \
                                          FLASH_NS_PARTITION_SIZE) ? \
                                         FLASH_S_PARTITION_SIZE :    \
                                         FLASH_NS_PARTITION_SIZE)

#ifdef BL2
#if (defined(REGION_IMAGE_1_S_SIZE) && defined(REGION_IMAGE_2_S_SIZE) && \
     (REGION_IMAGE_1_S_SIZE != REGION_IMAGE_2_S_SIZE))
#error "Secure image size must be the same in primary and secondary slot!"
#endif
#if (defined(REGION_IMAGE_1_NS_SIZE) && defined(REGION_IMAGE_2_NS_SIZE) && \
     (REGION_IMAGE_1_NS_SIZE != REGION_IMAGE_2_NS_SIZE))
#error "Non-secure image size must be the same in primary and secondary slot!"
#endif
#endif /* BL2 */

/* Sector size of the flash hardware */
#define FLASH_AREA_IMAGE_SECTOR_SIZE    (0x1000)   /* 4 KB */

/* Flash layout info for BL2 bootloader */
#define FLASH_BASE_ADDRESS              (REGION_CODE_BL2_START)

/* Offset and size definitions of the flash partitions that are handled by the
 * bootloader. The image swapping is done between IMAGE_PRIMARY and
 * IMAGE_SECONDARY, SCRATCH is used as a temporary storage during image
 * swapping.
 */
#define FLASH_AREA_BL2_OFFSET      (REGION_IMAGE_BL2_START - REGION_FLASH_S_START)
#define FLASH_AREA_BL2_SIZE        (REGION_IMAGE_BL2_SIZE)

#if !defined(MCUBOOT_IMAGE_NUMBER) || (MCUBOOT_IMAGE_NUMBER == 1)
/* Secure + Non-secure image primary slot */
#define FLASH_AREA_0_ID            (1)
#define FLASH_AREA_0_OFFSET        (REGION_IMAGE_1_S_START - REGION_FLASH_S_START)
#define FLASH_AREA_0_SIZE          (FLASH_S_PARTITION_SIZE + \
                                    FLASH_NS_PARTITION_SIZE)
/* Secure + Non-secure secondary slot */
#define FLASH_AREA_2_ID            (2)
#define FLASH_AREA_2_OFFSET        (REGION_IMAGE_2_S_START - REGION_FLASH_S_START)
#define FLASH_AREA_2_SIZE          (FLASH_S_PARTITION_SIZE + \
                                    FLASH_NS_PARTITION_SIZE)
/* Scratch area */
#define FLASH_AREA_SCRATCH_ID      (3)
#define FLASH_AREA_SCRATCH_OFFSET  (REGION_SCRATCH_START - REGION_FLASH_S_START)
#define FLASH_AREA_SCRATCH_SIZE    (FLASH_S_PARTITION_SIZE + \
                                    FLASH_NS_PARTITION_SIZE)
/* The maximum number of status entries supported by the bootloader. */
#define MCUBOOT_STATUS_MAX_ENTRIES ((FLASH_S_PARTITION_SIZE + \
                                     FLASH_NS_PARTITION_SIZE) / \
                                    FLASH_AREA_SCRATCH_SIZE)
/* Maximum number of image sectors supported by the bootloader. */
#define MCUBOOT_MAX_IMG_SECTORS    ((FLASH_S_PARTITION_SIZE + \
                                     FLASH_NS_PARTITION_SIZE) / \
                                    FLASH_AREA_IMAGE_SECTOR_SIZE)
#elif (MCUBOOT_IMAGE_NUMBER == 2)
/* Secure image primary slot */
#define FLASH_AREA_0_ID            (1)
#define FLASH_AREA_0_OFFSET        (REGION_IMAGE_1_S_START - REGION_FLASH_S_START)
#define FLASH_AREA_0_SIZE          (FLASH_S_PARTITION_SIZE)
/* Non-secure image primary slot */
#define FLASH_AREA_1_ID            (2)
#define FLASH_AREA_1_OFFSET        (REGION_IMAGE_1_NS_START - REGION_FLASH_NS_START)
#define FLASH_AREA_1_SIZE          (FLASH_NS_PARTITION_SIZE)
/* Secure image secondary slot */
#define FLASH_AREA_2_ID            (3)
#define FLASH_AREA_2_OFFSET        (REGION_IMAGE_2_S_START - REGION_FLASH_S_START)
#define FLASH_AREA_2_SIZE          (FLASH_S_PARTITION_SIZE)
/* Non-secure image secondary slot */
#define FLASH_AREA_3_ID            (4)
#define FLASH_AREA_3_OFFSET        (REGION_IMAGE_2_NS_START - REGION_FLASH_NS_START)
#define FLASH_AREA_3_SIZE          (FLASH_NS_PARTITION_SIZE)
/* Scratch area */
#define FLASH_AREA_SCRATCH_ID      (5)
#define FLASH_AREA_SCRATCH_OFFSET  (REGION_SCRATCH_START - REGION_FLASH_S_START)
#define FLASH_AREA_SCRATCH_SIZE    (FLASH_MAX_PARTITION_SIZE)
/* The maximum number of status entries supported by the bootloader. */
#define MCUBOOT_STATUS_MAX_ENTRIES (FLASH_MAX_PARTITION_SIZE / \
                                    FLASH_AREA_SCRATCH_SIZE)
/* Maximum number of image sectors supported by the bootloader. */
#define MCUBOOT_MAX_IMG_SECTORS    (FLASH_MAX_PARTITION_SIZE / \
                                    FLASH_AREA_IMAGE_SECTOR_SIZE)
#else /* MCUBOOT_IMAGE_NUMBER > 2 */
#error "Only MCUBOOT_IMAGE_NUMBER 1 and 2 are supported!"
#endif /* MCUBOOT_IMAGE_NUMBER */

/* Secure Storage (SST) Service definitions */
#define FLASH_SST_AREA_OFFSET           (REGION_SST_START - REGION_FLASH_S_START)
#define FLASH_SST_AREA_SIZE             (REGION_SST_SIZE)

/* Internal Trusted Storage (ITS) Service definitions */
#define FLASH_ITS_AREA_OFFSET           (REGION_ITS_START - REGION_FLASH_S_START)
#define FLASH_ITS_AREA_SIZE             (REGION_ITS_SIZE)

/* NV Counters definitions */
#define FLASH_NV_COUNTERS_AREA_OFFSET   (REGION_NV_COUNTERS_START - REGION_FLASH_S_START)
#define FLASH_NV_COUNTERS_AREA_SIZE     (FLASH_AREA_IMAGE_SECTOR_SIZE)
#if (defined(REGION_NV_COUNTERS_SIZE) && \
     (REGION_NV_COUNTERS_SIZE < FLASH_NV_COUNTERS_AREA_SIZE))
#error "NV Counters region size is not big enough!"
#endif

/* Offset and size definition in flash area used by assemble.py */
#define SECURE_IMAGE_OFFSET             (0x0)
#define SECURE_IMAGE_MAX_SIZE           FLASH_S_PARTITION_SIZE

#define NON_SECURE_IMAGE_OFFSET         (SECURE_IMAGE_OFFSET + \
                                         SECURE_IMAGE_MAX_SIZE)
#define NON_SECURE_IMAGE_MAX_SIZE       FLASH_NS_PARTITION_SIZE

/* Secure Storage (SST) Service definitions */
#define SST_FLASH_AREA_ADDR     FLASH_SST_AREA_OFFSET
/* Dedicated flash area for SST */
#define SST_FLASH_AREA_SIZE     FLASH_SST_AREA_SIZE
#define SST_SECTOR_SIZE         FLASH_AREA_IMAGE_SECTOR_SIZE
/* Number of SST_SECTOR_SIZE per block */
#define SST_SECTORS_PER_BLOCK   (0x1)
/* Specifies the smallest flash programmable unit in bytes */
#define SST_FLASH_PROGRAM_UNIT  (0x8)
/* The maximum asset size to be stored in the SST area */
#define SST_MAX_ASSET_SIZE      (2048)
/* The maximum number of assets to be stored in the SST area */
#define SST_NUM_ASSETS          (10)

/* Internal Trusted Storage (ITS) Service definitions */
#define ITS_FLASH_AREA_ADDR     FLASH_ITS_AREA_OFFSET
/* Dedicated flash area for ITS */
#define ITS_FLASH_AREA_SIZE     FLASH_ITS_AREA_SIZE
#define ITS_SECTOR_SIZE         FLASH_AREA_IMAGE_SECTOR_SIZE
/* Number of ITS_SECTOR_SIZE per block */
#define ITS_SECTORS_PER_BLOCK   (0x1)
/* Specifies the smallest flash programmable unit in bytes */
#define ITS_FLASH_PROGRAM_UNIT  (0x8)
/* The maximum asset size to be stored in the ITS area */
#define ITS_MAX_ASSET_SIZE      (512)
/* The maximum number of assets to be stored in the ITS area */
#define ITS_NUM_ASSETS          (10)

/* NV Counters definitions */
#define TFM_NV_COUNTERS_AREA_ADDR    FLASH_NV_COUNTERS_AREA_OFFSET
#define TFM_NV_COUNTERS_AREA_SIZE    (0x18) /* 24 Bytes */
#define TFM_NV_COUNTERS_SECTOR_ADDR  FLASH_NV_COUNTERS_AREA_OFFSET
#define TFM_NV_COUNTERS_SECTOR_SIZE  FLASH_AREA_IMAGE_SECTOR_SIZE

/* Shared data area between bootloader and runtime firmware.
 * Shared data area is allocated at the beginning of the RAM, it is overlapping
 * with TF-M Secure code's MSP stack
 */
#define BOOT_TFM_SHARED_DATA_BASE REGION_DATA_BOOT_START
#define BOOT_TFM_SHARED_DATA_SIZE (0x400)
#if (defined(REGION_DATA_BOOT_SIZE) && \
     (REGION_DATA_BOOT_SIZE != BOOT_TFM_SHARED_DATA_SIZE))
#error "Boot data size must be 0x400!"
#endif

#endif /* __FLASH_LAYOUT_H__ */
