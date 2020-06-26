/*
 * Copyright (c) 2017-2020 Arm Limited. All rights reserved.
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

#ifndef __REGION_DEFS_H__
#define __REGION_DEFS_H__

#include "mem_layout.h"
#include "flash_layout.h"

#define BL2_HEAP_SIZE           (REGION_HEAP_BL2_SIZE)
#define BL2_MSP_STACK_SIZE      (REGION_STACK_BL2_SIZE)

#define S_HEAP_SIZE             (REGION_HEAP_S_SIZE)
#define S_MSP_STACK_SIZE_INIT   (S_MSP_STACK_SIZE - BOOT_TFM_SHARED_DATA_SIZE)
#define S_MSP_STACK_SIZE        (REGION_STACK_MSP_S_SIZE)
#define S_PSP_STACK_SIZE        (REGION_STACK_PSP_S_SIZE)

#define NS_HEAP_SIZE            (REGION_HEAP_NS_SIZE)
#define NS_MSP_STACK_SIZE       (REGION_STACK_MSP_NS_SIZE)
#define NS_PSP_STACK_SIZE       (REGION_STACK_PSP_NS_SIZE)

/* This size of buffer is big enough to store an attestation
 * token produced by initial attestation service
 */
#define PSA_INITIAL_ATTEST_TOKEN_MAX_SIZE   (0x250)

#ifndef LINK_TO_SECONDARY_PARTITION
#define IMAGE_SECONDARY_START   (REGION_IMAGE_2_S_START)
#define IMAGE_SECONDARY_SIZE    (REGION_IMAGE_2_S_SIZE + REGION_IMAGE_2_NS_SIZE)
#else
#define IMAGE_SECONDARY_START   (REGION_IMAGE_1_S_START)
#define IMAGE_SECONDARY_SIZE    (REGION_IMAGE_1_S_SIZE + REGION_IMAGE_1_NS_SIZE)
#endif /* !LINK_TO_SECONDARY_PARTITION */

#ifdef BL2
#define IMAGE_NS_START          (REGION_HEADER_NS_START)
#define IMAGE_NS_SIZE           (REGION_HEADER_NS_SIZE + \
                                 REGION_CODE_NS_SIZE + \
                                 REGION_TRAILER_NS_SIZE)
#else
#define IMAGE_NS_START          (REGION_CODE_NS_START)
#define IMAGE_NS_SIZE           (REGION_CODE_NS_SIZE)
#endif /* BL2 */

/* Boot partition structure if MCUBoot is used:
 * Bootloader header
 * Image area
 * Trailer
 */
#ifdef BL2
#define BL2_HEADER_SIZE      (0x400)       /* 1 KB */
#define BL2_TRAILER_SIZE     (0x400)       /* 1 KB */
#if (defined(REGION_HEADER_S_SIZE) && \
     (REGION_HEADER_S_SIZE != BL2_HEADER_SIZE))
#error "Secure image header size must be 1 KB!"
#endif
#if (defined(REGION_HEADER_NS_SIZE) && \
     (REGION_HEADER_NS_SIZE != BL2_HEADER_SIZE))
#error "Non-secure image header size must be 1 KB!"
#endif
#else
/* No header if no bootloader, but keep image code size the same */
#define BL2_HEADER_SIZE      (0x0)
#define BL2_TRAILER_SIZE     (0x800)
#endif /* BL2 */

/* Secure regions */
#define S_CODE_START    (REGION_CODE_S_START)
#define S_CODE_SIZE     (REGION_CODE_S_SIZE)
#define S_CODE_LIMIT    (S_CODE_START + S_CODE_SIZE - 1)

#define S_DATA_START    (REGION_DATA_S_START)
#define S_DATA_SIZE     (REGION_DATA_S_SIZE)
#define S_DATA_LIMIT    (S_DATA_START + S_DATA_SIZE - 1)

/* CMSE Veneers region */
#define CMSE_VENEER_REGION_START  (REGION_VENEERS_START)
#define CMSE_VENEER_REGION_SIZE   (0x340)
#if (defined(REGION_VENEERS_SIZE) && \
     (REGION_VENEERS_SIZE != CMSE_VENEER_REGION_SIZE))
#error "Veneer region size must be 0x340!"
#endif

/* Non-secure regions */
#define NS_CODE_START   (REGION_CODE_NS_START)
#define NS_CODE_SIZE    (REGION_CODE_NS_SIZE)
#define NS_CODE_LIMIT   (NS_CODE_START + NS_CODE_SIZE - 1)

#define NS_DATA_START   (REGION_DATA_NS_START)
#define NS_DATA_SIZE    (REGION_DATA_NS_SIZE)
#define NS_DATA_LIMIT   (NS_DATA_START + NS_DATA_SIZE - 1)

/* NS partition information is used for MPC and SAU configuration */
#define NS_PARTITION_START (IMAGE_NS_START)
#define NS_PARTITION_SIZE  (IMAGE_NS_SIZE)

/* Secondary partition for new images in case of firmware upgrade */
#define SECONDARY_PARTITION_START (IMAGE_SECONDARY_START)
#define SECONDARY_PARTITION_SIZE  (IMAGE_SECONDARY_SIZE)

#ifdef BL2
/* Bootloader regions */
#define BL2_CODE_START    (REGION_CODE_BL2_START)
#define BL2_CODE_SIZE     (REGION_CODE_BL2_SIZE)
#define BL2_CODE_LIMIT    (BL2_CODE_START + BL2_CODE_SIZE - 1)

#define BL2_DATA_START    (REGION_DATA_BL2_START)
#define BL2_DATA_SIZE     (REGION_DATA_BL2_SIZE)
#define BL2_DATA_LIMIT    (BL2_DATA_START + BL2_DATA_SIZE - 1)
#endif /* BL2 */

/* Shared data area between bootloader and runtime firmware.
 * Shared data area is allocated at the beginning of the RAM, it is overlapping
 * with TF-M Secure code's MSP stack
 */
#define BOOT_TFM_SHARED_DATA_BASE REGION_DATA_BOOT_START
#define BOOT_TFM_SHARED_DATA_SIZE (0x400)
#define BOOT_TFM_SHARED_DATA_LIMIT (BOOT_TFM_SHARED_DATA_BASE + \
                                    BOOT_TFM_SHARED_DATA_SIZE - 1)
#if (defined(REGION_DATA_BOOT_SIZE) && \
     (REGION_DATA_BOOT_SIZE != BOOT_TFM_SHARED_DATA_SIZE))
#error "Boot data size must be 0x400!"
#endif

#endif /* __REGION_DEFS_H__ */
