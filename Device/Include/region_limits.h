/*
 * Copyright (c) 2017 ARM Limited
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

#ifndef REGION_LIMITS_H
#define REGION_LIMITS_H

/* Internal data SRAM */
#define TOTAL_RAM_SIZE    (0x00020000) /* 128 kB */
#define S_RAM_ALIAS_BASE  (0x30000000)
#define NS_RAM_ALIAS_BASE (0x20000000)

/* QSPI flash area */
#define TOTAL_QSPI_FLASH_SIZE    (0x00040000) /* Only 256kB of flash can be used */
#define S_QSPI_FLASH_ALIAS_BASE  (0x10200000)
#define NS_QSPI_FLASH_ALIAS_BASE (0x00200000)

/* Code SRAM area */
#define TOTAL_CODE_SRAM_SIZE     (0x00200000)
#define S_CODE_SRAM_ALIAS_BASE   (0x10000000)
#define NS_CODE_SRAM_ALIAS_BASE  (0x00000000)

#define S_CODE_SRAM_EXEC_BASE    (S_CODE_SRAM_ALIAS_BASE)
#define S_CODE_SRAM_EXEC_LIMIT   (S_CODE_SRAM_EXEC_BASE + (TOTAL_CODE_SRAM_SIZE/2))
#define NS_CODE_SRAM_EXEC_BASE   (NS_CODE_SRAM_ALIAS_BASE + (TOTAL_CODE_SRAM_SIZE/2))
#define NS_CODE_SRAM_EXEC_LIMIT  (NS_CODE_SRAM_EXEC_BASE + (TOTAL_CODE_SRAM_SIZE/2))

/* Code MRAM area */
#define TOTAL_CODE_MRAM_SIZE     (0x00200000)
#define S_CODE_MRAM_ALIAS_BASE   (0x1A000000)
#define NS_CODE_MRAM_ALIAS_BASE  (0x0A000000)

#define S_CODE_MRAM_EXEC_BASE    (S_CODE_MRAM_ALIAS_BASE)
#define S_CODE_MRAM_EXEC_LIMIT   (S_CODE_MRAM_EXEC_BASE + (TOTAL_CODE_MRAM_SIZE/2))
#define NS_CODE_MRAM_EXEC_BASE   (NS_CODE_MRAM_ALIAS_BASE + (TOTAL_CODE_MRAM_SIZE/2))
#define NS_CODE_MRAM_EXEC_LIMIT  (NS_CODE_MRAM_EXEC_BASE + (TOTAL_CODE_MRAM_SIZE/2))

/* Code memory */
#if defined __USE_FLASH
  #define TOTAL_ROM_SIZE           TOTAL_QSPI_FLASH_SIZE
  #define S_ROM_ALIAS_BASE         S_QSPI_FLASH_ALIAS_BASE
  #define NS_ROM_ALIAS_BASE        NS_QSPI_FLASH_ALIAS_BASE
#elif defined __USE_MRAM
  #define TOTAL_ROM_SIZE           TOTAL_CODE_MRAM_SIZE
  #define S_ROM_ALIAS_BASE         S_CODE_MRAM_ALIAS_BASE
  #define NS_ROM_ALIAS_BASE        NS_CODE_MRAM_ALIAS_BASE
#else
  #define TOTAL_ROM_SIZE           TOTAL_CODE_SRAM_SIZE
  #define S_ROM_ALIAS_BASE         S_CODE_SRAM_ALIAS_BASE
  #define NS_ROM_ALIAS_BASE        NS_CODE_SRAM_ALIAS_BASE
#endif


#endif /*REGION_LIMITS_H*/
