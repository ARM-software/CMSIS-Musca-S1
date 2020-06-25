/* -----------------------------------------------------------------------------
 * Copyright (c) 2019-2020 ARM Ltd.
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from
 * the use of this software. Permission is granted to anyone to use this
 * software for any purpose, including commercial applications, and to alter
 * it and redistribute it freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software in
 *    a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 *
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 *
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * $Date:        17. June 2020
 * $Revision:    V1.2.0
 *
 * Project:      RTE Device Configuration for Musca-S1 device
 * -------------------------------------------------------------------------- */

//-------- <<< Use Configuration Wizard in Context Menu >>> --------------------

#ifndef __RTE_DEVICE_H
#define __RTE_DEVICE_H

// <e> USART0 (Universal synchronous asynchronous receiver transmitter) [Driver_USART0]
// <i> Configuration settings for Driver_USART0 in component ::CMSIS Driver:USART
#define RTE_USART0                      0

//   <o> USART0_RX Pin <0=>Not Used <1=>PA0
#define RTE_USART0_RX_ID                0
#if    (RTE_USART0_RX_ID == 0)
#define RTE_USART0_RX                   0
#elif  (RTE_USART0_RX_ID == 1)
#define RTE_USART0_RX                   1
#else
#error "Invalid USART0_RX Pin Configuration!"
#endif

//   <o> USART0_TX Pin <0=>Not Used <1=>PA1
#define RTE_USART0_TX_ID                0
#if    (RTE_USART0_TX_ID == 0)
#define RTE_USART0_TX                   0
#elif  (RTE_USART0_TX_ID == 1)
#define RTE_USART0_TX                   1
#else
#error "Invalid USART0_TX Pin Configuration!"
#endif

//   <o> USART0_CK Pin <0=>Not Used <1=>PA9
#define RTE_USART0_CK_ID                0
#if    (RTE_USART0_CK_ID == 0)
#define RTE_USART0_CK                   0
#elif  (RTE_USART0_CK_ID == 1)
#define RTE_USART0_CK                   1
#else
#error "Invalid USART0_CK Pin Configuration!"
#endif

//   <o> USART0_CTS Pin <0=>Not Used <1=>PA5
#define RTE_USART0_CTS_ID               0
#if    (RTE_USART0_CTS_ID == 0)
#define RTE_USART0_CTS                  0
#elif  (RTE_USART0_CTS_ID == 1)
#define RTE_USART0_CTS                  1
#else
#error "Invalid USART0_CTS Pin Configuration!"
#endif

//   <o> USART0_RTS Pin <0=>Not Used <1=>PA6
#define RTE_USART0_RTS_ID               0
#if    (RTE_USART0_RTS_ID == 0)
#define RTE_USART0_RTS                  0
#elif  (RTE_USART0_RTS_ID == 1)
#define RTE_USART0_RTS                  1
#else
#error "Invalid USART0_RTS Pin Configuration!"
#endif

// </e>


// <e> USART1 (Universal synchronous asynchronous receiver transmitter) [Driver_USART1]
// <i> Configuration settings for Driver_USART1 in component ::CMSIS Driver:USART
#define RTE_USART1                      1

//   <o> USART1_RX Pin <0=>Not Used <1=>PA16
#define RTE_USART1_RX_ID                1
#if    (RTE_USART1_RX_ID == 0)
#define RTE_USART1_RX                   0
#elif  (RTE_USART1_RX_ID == 1)
#define RTE_USART1_RX                   1
#else
#error "Invalid USART1_RX Pin Configuration!"
#endif

//   <o> USART1_TX Pin <0=>Not Used <1=>PA17
#define RTE_USART1_TX_ID                1
#if    (RTE_USART1_TX_ID == 0)
#define RTE_USART1_TX                   0
#elif  (RTE_USART1_TX_ID == 1)
#define RTE_USART1_TX                   1
#else
#error "Invalid USART1_TX Pin Configuration!"
#endif

//   <o> USART1_CTS Pin <0=>Not Used <1=>PA7
#define RTE_USART1_CTS_ID               0
#if    (RTE_USART1_CTS_ID == 0)
#define RTE_USART1_CTS                  0
#elif  (RTE_USART1_CTS_ID == 1)
#define RTE_USART1_CTS                  1
#else
#error "Invalid USART1_CTS Pin Configuration!"
#endif

//   <o> USART1_RTS Pin <0=>Not Used <1=>PA8
#define RTE_USART1_RTS_ID               0
#if    (RTE_USART1_RTS_ID == 0)
#define RTE_USART1_RTS                  0
#elif  (RTE_USART1_RTS_ID == 1)
#define RTE_USART1_RTS                  1
#else
#error "Invalid USART1_RTS Pin Configuration!"
#endif

// </e>


// <e> SPI0 (Serial Peripheral Interface 0) [Driver_SPI0]
// <i> Configuration settings for Driver_SPI0 in component ::CMSIS Driver:SPI
#define RTE_SPI0                        0

//   <o> SPI0_MISO Pin <0=>Not Used <1=>PA12
#define RTE_SPI0_MISO_ID                0
#if    (RTE_SPI0_MISO_ID == 0)
#define RTE_SPI0_MISO                   0
#elif  (RTE_SPI0_MISO_ID == 1)
#define RTE_SPI0_MISO                   1
#else
#error "Invalid SPI0_MISO Pin Configuration!"
#endif

//   <o> SPI0_MOSI Pin <0=>Not Used <1=>PA11
#define RTE_SPI0_MOSI_ID                0
#if    (RTE_SPI0_MOSI_ID == 0)
#define RTE_SPI0_MOSI                   0
#elif  (RTE_SPI0_MOSI_ID == 1)
#define RTE_SPI0_MOSI                   1
#else
#error "Invalid SPI0_MOSI Pin Configuration!"
#endif

//   <o> SPI0_SCK Pin <0=>Not Used <1=>PA13
#define RTE_SPI0_SCK_ID                 0
#if    (RTE_SPI0_SCK_ID == 0)
#define RTE_SPI0_SCK                    0
#elif  (RTE_SPI0_SCK_ID == 1)
#define RTE_SPI0_SCK                    1
#else
#error "Invalid SPI0_SCK Pin Configuration!"
#endif

//   <o> SPI0_NSS Pin <0=>Not Used <1=>PA10
#define RTE_SPI0_NSS_ID                 0
#if    (RTE_SPI0_NSS_ID == 0)
#define RTE_SPI0_NSS                    0
#elif  (RTE_SPI0_NSS_ID == 1)
#define RTE_SPI0_NSS                    1
#else
#error "Invalid SPI0_NSS Pin Configuration!"
#endif

// </e>


// <e> I2C0 (Inter-integrated Circuit Interface 0) [Driver_I2C0]
// <i> Configuration settings for Driver_I2C0 in component ::CMSIS Driver:I2C
#define RTE_I2C0                        0

//   <o> I2C0_SDA Pin <0=>Not Used <1=>PA14
#define RTE_I2C0_SDA_ID                 0
#if    (RTE_I2C0_SDA_ID == 0)
#define RTE_I2C0_SDA                    0
#elif  (RTE_I2C0_SDA_ID == 1)
#define RTE_I2C0_SDA                    1
#else
#error "Invalid I2C0_SDA Pin Configuration!"
#endif

//   <o> I2C0_SCL Pin <0=>Not Used <1=>PA15
#define RTE_I2C0_SCL_ID                 0
#if    (RTE_I2C0_SCL_ID == 0)
#define RTE_I2C0_SCL                    0
#elif  (RTE_I2C0_SCL_ID == 1)
#define RTE_I2C0_SCL                    1
#else
#error "Invalid I2C0_SCL Pin Configuration!"
#endif

// </e>


// <e> I2C1 (Inter-integrated Circuit Interface 1) [Driver_I2C1]
// <i> Configuration settings for Driver_I2C1 in component ::CMSIS Driver:I2C
#define RTE_I2C1                        0

//   <o> I2C1_SDA Pin <0=>Not Used <1=>PA18
#define RTE_I2C1_SDA_ID                 0
#if    (RTE_I2C1_SDA_ID == 0)
#define RTE_I2C1_SDA                    0
#elif  (RTE_I2C1_SDA_ID == 1)
#define RTE_I2C1_SDA                    1
#else
#error "Invalid I2C1_SDA Pin Configuration!"
#endif

//   <o> I2C1_SCL Pin <0=>Not Used <1=>PA19
#define RTE_I2C1_SCL_ID                 0
#if    (RTE_I2C1_SCL_ID == 0)
#define RTE_I2C1_SCL                    0
#elif  (RTE_I2C1_SCL_ID == 1)
#define RTE_I2C1_SCL                    1
#else
#error "Invalid I2C0_SCL Pin Configuration!"
#endif

// </e>

// <e> Flash eMRAM (Flash Interface 1) [Driver_FLASH0]
// <i> Configuration settings for Driver_FLASH0 in component ::CMSIS Driver:Flash
#define RTE_FLASH0                      0

//   <o> Flash security mode
//       <0=> Non-secure  <1=> Secure
#define RTE_FLASH0_SECURE               1

//   <o> Offset for Flash partition in eMRAM (Bytes) <0x0-0x200000:8>
#define RTE_FLASH0_BASE_OFFSET             0x00000000

//   <o> Size of Flash partition (Bytes) <0x400-0x200000:8>
#define RTE_FLASH0_SIZE                    0x00200000

//   <o> Size of Flash sectors (Bytes) <0x8-0x200000:8>
#define RTE_FLASH0_SECTOR_SIZE             0x00001000

//   <o> Size of Flash pages (Bytes) <0x8-0x200000:8>
#define RTE_FLASH0_PAGE_SIZE               0x00001000

//   <o> Minimum data size for programming (Bytes) <0x1-0x200000>
#define RTE_FLASH0_PROGRAM_UNIT            0x1


// </e>

#endif  /* __RTE_DEVICE_H */
