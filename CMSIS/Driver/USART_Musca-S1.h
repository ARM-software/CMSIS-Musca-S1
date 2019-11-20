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
 * Project:      USART Driver Definitions for Musca-S1 device
 * -------------------------------------------------------------------------- */

#ifndef USART_MUSCA_S1_H_
#define USART_MUSCA_S1_H_

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "Driver_USART.h"

#include "RTE_Components.h"
#include  CMSIS_device_header
#include "RTE_Device.h"

#if (defined(RTE_Drivers_USART) \
     && (RTE_USART0 == 0)       \
     && (RTE_USART1 == 0))
  #error "USART not configured in RTE_Device.h!"
#endif

// USART Flags
#define USART_FLAG_INITIALIZED      (1U)
#define USART_FLAG_POWERED          (1U << 1)
#define USART_FLAG_CONFIGURED       (1U << 2)
#define USART_FLAG_TX_ENABLED       (1U << 3)
#define USART_FLAG_RX_ENABLED       (1U << 4)

// USART synchronous xfer modes
#define USART_SYNC_MODE_TX          (1U)
#define USART_SYNC_MODE_RX          (2U)
#define USART_SYNC_MODE_TX_RX       (USART_SYNC_MODE_TX | \
                                     USART_SYNC_MODE_RX)

// USART pin
typedef const struct _USART_PIN {
        uint16_t                pin;               // Pin
        uint16_t                af;                // Alternate function
} USART_PIN;

// USART Input/Output Configuration
typedef const struct _USART_IO {
        USART_PIN              *rx;                // RX  Pin identifier
        USART_PIN              *tx;                // TX  Pin identifier
        USART_PIN              *sclk;              // SCLK Pin identifier
        USART_PIN              *rts;               // RTS Pin identifier
        USART_PIN              *cts;               // CTS Pin identifier
} USART_IO;

// USART Transfer Information (Run-Time)
typedef struct _USART_TRANSFER_INFO {
        uint32_t                rx_num;            // Total number of receive data
        uint32_t                tx_num;            // Total number of transmit data
        uint8_t                *rx_buf;            // Pointer to in data buffer
  const uint8_t                *tx_buf;            // Pointer to out data buffer
        uint32_t                rx_cnt;            // Number of data received
        uint32_t                tx_cnt;            // Number of data sent
        uint8_t                 break_flag;        // Transmit break flag
        uint8_t                 send_active;       // Send active flag
  const uint8_t                 padding[2];
} USART_TRANSFER_INFO;

// USART status
typedef struct _USART_STATUS {
        uint8_t                 tx_busy;           // Transmitter busy flag
        uint8_t                 rx_busy;           // Receiver busy flag
        uint8_t                 tx_underflow;      // Transmit data underflow detected (cleared on start of next send operation)
        uint8_t                 rx_overflow;       // Receive data overflow detected (cleared on start of next receive operation)
        uint8_t                 rx_break;          // Break detected on receive (cleared on start of next receive operation)
        uint8_t                 rx_framing_error;  // Framing error detected on receive (cleared on start of next receive operation)
        uint8_t                 rx_parity_error;   // Parity error detected on receive (cleared on start of next receive operation)
  const uint8_t                 padding[1];
} USART_STATUS;

// USART Information (Run-time)
typedef struct _USART_INFO {
        ARM_USART_SignalEvent_t cb_event;          // Event Callback
        USART_STATUS            status;            // Status flags
        uint32_t                flags;             // Current USART flags
        uint32_t                mode;              // Current USART mode
        uint32_t                flow_control;      // Flow control
} USART_INFO;

// USART Resources Definitions
typedef const struct {
        ARM_USART_CAPABILITIES  capabilities;      // Capabilities
        USART_IO                io;                // USART Input/Output pins
        UART_TypeDef           *reg;               // USART peripheral pointer
        USART_INFO             *info;              // Run-Time Information
        USART_TRANSFER_INFO    *xfer;              // USART transfer information
        IRQn_Type               irq_num;           // USART IRQ Number
  const uint8_t                 padding0[3];
        uint32_t                rx_timeout_val;    // Receive timeout value
        uint8_t                 reset_bit;         // reset bit for RESET_CTRL register
  const uint8_t                 padding1[3];
} USART_RESOURCES;

#endif /* USART_MUSCA_S1_H_ */
