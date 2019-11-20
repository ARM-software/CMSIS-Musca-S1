/* -----------------------------------------------------------------------------
 * Copyright (c) 2019 ARM Ltd.
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
 *
 * $Date:        19. November 2019
 * $Revision:    V1.0.0
 *
 * Project:      SPI Driver Definitions for Musca-S1 device
 * -------------------------------------------------------------------------- */

#ifndef SPI_MUSCA_S1_H_
#define SPI_MUSCA_S1_H_

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "Driver_SPI.h"

#include "RTE_Components.h"
#include  CMSIS_device_header
#include "RTE_Device.h"

#if (defined(RTE_Drivers_SPI) \
     && (RTE_SPI0 == 0))
  #error "SPI not configured in RTE_Device.h!"
#endif

// SPI Flags
#define SPI_FLAG_INITIALIZED        (1U)
#define SPI_FLAG_POWERED            (1U << 1)
#define SPI_FLAG_CONFIGURED         (1U << 2)
#define SPI_FLAG_TX_ENABLED         (1U << 3)
#define SPI_FLAG_RX_ENABLED         (1U << 4)
#define SPI_FLAG_SS_SWCTRL          (1U << 5)      // Software controlled SSEL

// SPI pin
typedef const struct _SPI_PIN {
        uint16_t                pin;               // Pin
        uint16_t                af;                // Alternate function
} SPI_PIN;

// SPI Input/Output Configuration
typedef const struct _SPI_IO {
        SPI_PIN                *mosi;              // Pointer to MOSI pin configuration
        SPI_PIN                *miso;              // Pointer to MISO pin configuration
        SPI_PIN                *sck;               // Pointer to SCK pin configuration
        SPI_PIN                *nss;               // Pointer to NSS pin configuration
} SPI_IO;

// SPI status
typedef struct _SPI_STATUS {
        uint8_t                 busy;              // Transmitter/Receiver busy flag
        uint8_t                 data_lost;         // Data lost: Receive overflow / Transmit underflow (cleared on start of transfer operation)
        uint8_t                 mode_fault;        // Mode fault detected; optional (cleared on start of transfer operation)
  const uint8_t                 padding[1];
} SPI_STATUS;

// SPI Information (Run-Time)
typedef struct _SPI_INFO {
        ARM_SPI_SignalEvent_t   cb_event;          // Event Callback
        SPI_STATUS              status;            // Status flags
        uint32_t                flags;             // Current SPI state
        uint32_t                mode;              // Current SPI mode
} SPI_INFO;

// SPI Transfer Information (Run-Time)
typedef struct _SPI_TRANSFER_INFO {
        uint32_t                num;               // Total number of transfers
        uint8_t                *rx_buf;            // Pointer to in data buffer
  const uint8_t                *tx_buf;            // Pointer to out data buffer
        uint32_t                rx_cnt;            // Number of data received
        uint32_t                tx_cnt;            // Number of data sent
        uint16_t                def_val;           // Default transfer value
  const uint8_t                 padding[2];
} SPI_TRANSFER_INFO;

// SPI Resources Definition
typedef const struct {
        ARM_SPI_CAPABILITIES    capabilities;      // Capabilities
        SPI_IO                  io;                // SPI Input/Output pins
        SPI_TypeDef            *reg;               // SPI peripheral pointer
        SPI_INFO               *info;              // Run-Time Information
        SPI_TRANSFER_INFO      *xfer;              // SPI transfer information
        IRQn_Type               irq_num;           // SPI IRQ Number
        uint8_t                 reset_bit;         // reset bit for RESET_CTRL register
  const uint8_t                 padding[2];
} SPI_RESOURCES;

#endif /* SPI_MUSCA_S1_H_ */
