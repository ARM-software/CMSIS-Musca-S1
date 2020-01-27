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
 * Project:      I2C Driver Definitions for Musca-S1 device
 * -------------------------------------------------------------------------- */

#ifndef I2C_MUSCA_S1_H_
#define I2C_MUSCA_S1_H_

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "Driver_I2C.h"

#include "RTE_Components.h"
#include  CMSIS_device_header
#include "RTE_Device.h"

#if (defined(RTE_Drivers_I2C) \
     && (RTE_I2C0 == 0)       \
     && (RTE_I2C1 == 0))
  #error "I2C not configured in RTE_Device.h!"
#endif

// I2C Flags
#define I2C_FLAG_INITIALIZED        (1U)
#define I2C_FLAG_POWERED            (1U << 1)
#define I2C_FLAG_CONFIGURED         (1U << 2)

#define I2C_DIR_TX                  (0U)           // Direction: transmitter
#define I2C_DIR_RX                  (1U)           // Direction: receiver

#define XFER_CTRL_XPENDING          (1U)           // Transfer pending
#define XFER_CTRL_TXACTIVE          (1U << 1)      // Transfer send active
#define XFER_CTRL_RXACTIVE          (1U << 2)      // Transfer receive active

// I2C pin
typedef const struct _I2C_PIN {
        uint16_t                pin;               // Pin
        uint16_t                af;                // Alternate function
} I2C_PIN;

// I2C Input/Output Configuration
typedef const struct _I2C_IO {
        I2C_PIN                *sda;               // Pointer to SDA pin configuration
        I2C_PIN                *scl;               // Pointer to SCL pin configuration
} I2C_IO;

// I2C Information (Run-Time)
typedef struct _I2C_INFO {
        ARM_I2C_SignalEvent_t   cb_event;          // Event Callback
        ARM_I2C_STATUS          status;            // Status flags
        uint16_t                flags;             // Current I2C state
        uint16_t                mode;              // Current I2C mode
} I2C_INFO;

// I2C Transfer Information (Run-Time)
typedef struct _I2C_TRANSFER_INFO {
        uint32_t                num;               // Number of data to transfer
        uint32_t                cnt;               // Data transfer counter
        uint32_t                tsr_cur;           // current transfer size register value
  const uint8_t                *tx_data;           // Pointer to transmit data
        uint8_t                *rx_data;           // Pointer to receive data
        uint16_t                addr;              // Device address
        uint8_t                 ctrl;              // Transfer control flags
  const uint8_t                 padding[1];
} I2C_TRANSFER_INFO;


// I2C Resources Definition
typedef const struct {
        ARM_I2C_CAPABILITIES    capabilities;      // Capabilities
        I2C_IO                  io;                // SPI Input/Output pins
        I2C_TypeDef            *reg;               // I2C peripheral register interface
        I2C_INFO               *info;              // Run-Time information
        I2C_TRANSFER_INFO      *xfer;              // I2C transfer information
        IRQn_Type               irq_num;           // I2C IRQ Number
        uint8_t                 reset_bit;         // reset bit for RESET_CTRL register
  const uint8_t                 padding[2];
} I2C_RESOURCES;

#endif /* I2C_MUSCA_S1_H_ */
