/*
 * Copyright (c) 2013-2020 Arm Limited. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "Driver_USART.h"

#include "RTE_Components.h"
#include  CMSIS_device_header
#include "RTE_Device.h"

#if ((RTE_USART0 == 1) && (RTE_USART1 == 0))
  #define RTE_USART 0
  #define USART UART0
  #define UART_IRQn UART0_IRQn
#elif ((RTE_USART0 == 0) && (RTE_USART1 == 1))
  #define RTE_USART 1
  #define USART UART1
  #define UART_IRQn UART1_IRQn
#else
  #error "USART not configured in RTE_Device.h!"
#endif

#define SCC_REGISTER   SCC

#define ARM_USART_DRV_VERSION    ARM_DRIVER_VERSION_MAJOR_MINOR(1, 0)  /* driver version */

/* Driver Version */
static const ARM_DRIVER_VERSION DriverVersion = { 
    ARM_USART_API_VERSION,
    ARM_USART_DRV_VERSION
};

/* Driver Capabilities */
static const ARM_USART_CAPABILITIES DriverCapabilities = {
    1, /* supports UART (Asynchronous) mode */
    0, /* supports Synchronous Master mode */
    0, /* supports Synchronous Slave mode */
    0, /* supports UART Single-wire mode */
    0, /* supports UART IrDA mode */
    0, /* supports UART Smart Card mode */
    0, /* Smart Card Clock generator available */
    0, /* RTS Flow Control available */
    0, /* CTS Flow Control available */
    0, /* Transmit completed event: \ref ARM_USART_EVENT_TX_COMPLETE */
    0, /* Signal receive character timeout event: \ref ARM_USART_EVENT_RX_TIMEOUT */
    0, /* RTS Line: 0=not available, 1=available */
    0, /* CTS Line: 0=not available, 1=available */
    0, /* DTR Line: 0=not available, 1=available */
    0, /* DSR Line: 0=not available, 1=available */
    0, /* DCD Line: 0=not available, 1=available */
    0, /* RI Line: 0=not available, 1=available */
    0, /* Signal CTS change event: \ref ARM_USART_EVENT_CTS */
    0, /* Signal DSR change event: \ref ARM_USART_EVENT_DSR */
    0, /* Signal DCD change event: \ref ARM_USART_EVENT_DCD */
    0, /* Signal RI change event: \ref ARM_USART_EVENT_RI */
    0  /* Reserved (must be zero) */
};

//
//   Functions
//

/**
  \fn          void USART_PinInit (uint16_t pin, uint16_t af)
  \brief       Initialize USART pin
*/
static void USART_PinInit (uint16_t pin, uint16_t af)
{
      SCC_REGISTER->IOMUX_MAIN_INSEL   &= ~(1U << pin);
      SCC_REGISTER->IOMUX_MAIN_OUTSEL  &= ~(1U << pin);
      SCC_REGISTER->IOMUX_MAIN_OENSEL  &= ~(1U << pin);
      SCC_REGISTER->IOMUX_ALTF1_INSEL  &= ~(1U << pin);
      SCC_REGISTER->IOMUX_ALTF1_OUTSEL &= ~(1U << pin);
      SCC_REGISTER->IOMUX_ALTF1_OENSEL &= ~(1U << pin);
      SCC_REGISTER->IOMUX_ALTF2_INSEL  &= ~(1U << pin);
      SCC_REGISTER->IOMUX_ALTF2_OUTSEL &= ~(1U << pin);
      SCC_REGISTER->IOMUX_ALTF2_OENSEL &= ~(1U << pin);
      SCC_REGISTER->IOMUX_ALTF3_INSEL  &= ~(1U << pin);
      SCC_REGISTER->IOMUX_ALTF3_OUTSEL &= ~(1U << pin);
      SCC_REGISTER->IOMUX_ALTF3_OENSEL &= ~(1U << pin);

  switch (af) {
    case 1:   /* Alternate Function 1 */
      SCC_REGISTER->IOMUX_ALTF1_INSEL  |=  (1U << pin);
      SCC_REGISTER->IOMUX_ALTF1_OUTSEL |=  (1U << pin);
      SCC_REGISTER->IOMUX_ALTF1_OENSEL |=  (1U << pin);
      break;
    case 2:   /* Alternate Function 2 */
      SCC_REGISTER->IOMUX_ALTF2_INSEL  |=  (1U << pin);
      SCC_REGISTER->IOMUX_ALTF2_OUTSEL |=  (1U << pin);
      SCC_REGISTER->IOMUX_ALTF2_OENSEL |=  (1U << pin);
      break;
    case 3:   /* Alternate Function 3 */
      SCC_REGISTER->IOMUX_ALTF3_INSEL  |=  (1U << pin);
      SCC_REGISTER->IOMUX_ALTF3_OUTSEL |=  (1U << pin);
      SCC_REGISTER->IOMUX_ALTF3_OENSEL |=  (1U << pin);
      break;
    default:  /* Primary Function */
      SCC_REGISTER->IOMUX_MAIN_INSEL   |=  (1U << pin);
      SCC_REGISTER->IOMUX_MAIN_OUTSEL  |=  (1U << pin);
      SCC_REGISTER->IOMUX_MAIN_OENSEL  |=  (1U << pin);
      break;
  }
}

static int32_t USART_SetBaudrate (uint32_t clk, uint32_t baudrate) {
#define UART_SAMPLING_FACTOR      (16U)
#define UART_FBRD_WIDTH           ( 6U)

  /* Avoiding float calculations, bauddiv is left shifted by 6 */
  uint64_t bauddiv = (((uint64_t)clk) << UART_FBRD_WIDTH) / (16U * baudrate);

  /* Valid bauddiv value
   * uart_clk (min) >= 16 x baud_rate (max)
   * uart_clk (max) <= 16 x 65535 x baud_rate (min)
   */
  if((bauddiv < (    1U << UART_FBRD_WIDTH)) ||
     (bauddiv > (65535U << UART_FBRD_WIDTH))   ) {
    return -1;
  }

  USART->IBRD = (uint32_t)(bauddiv >> UART_FBRD_WIDTH);
  USART->FBRD = (uint32_t)(bauddiv & ((1U << UART_FBRD_WIDTH) - 1U));

  __DMB();

  /* In order to internally update the contents of uartibrd or uartfbrd,
     a uartlcr_h write must always be performed at the end
     ARM DDI 0183F, Pg 3-13 */
  USART->LCR_H = USART->LCR_H;

  return 0;
}
                                     
static ARM_DRIVER_VERSION ARM_USART_GetVersion(void)
{
  return DriverVersion;
}

static ARM_USART_CAPABILITIES ARM_USART_GetCapabilities(void)
{
  return DriverCapabilities;
}

static int32_t ARM_USART_Initialize(ARM_USART_SignalEvent_t cb_event)
{
  #if (RTE_USART == 0)
  #if defined (RTE_USART0_RX) && (RTE_USART0_RX != 0U)
    USART_PinInit(0U, 1U);
  #endif
  #if defined (RTE_USART0_TX) && (RTE_USART0_TX != 0U)
    USART_PinInit(1U, 1U);
  #endif
  #elif (RTE_USART == 1)
  #if defined (RTE_USART1_RX) && (RTE_USART1_RX != 0U)
    USART_PinInit(16U, 0U);
  #endif
  #if defined (RTE_USART1_TX) && (RTE_USART1_TX != 0U)
    USART_PinInit(17U, 0U);
  #endif
  #endif
  
  return ARM_DRIVER_OK;
}

static int32_t ARM_USART_Uninitialize(void)
{
  return ARM_DRIVER_OK;
}

static int32_t ARM_USART_PowerControl(ARM_POWER_STATE state)
{
    switch (state)
    {
    case ARM_POWER_OFF:
        break;

    case ARM_POWER_LOW:
        break;

    case ARM_POWER_FULL:
        break;
    }
    return ARM_DRIVER_OK;
}

static int32_t ARM_USART_Send(const void *data, uint32_t num)
{
  const uint8_t* buf = (const uint8_t*)data;
  
  if ((USART->FR & UART_FR_TXFE_Msk) == 0U) {
    // Transmit FIFO not empty, there are bytes to be sent out
    return ARM_DRIVER_ERROR_BUSY;
  }
  
  const uint32_t irq_state = NVIC_GetEnableIRQ(UART_IRQn);
  NVIC_DisableIRQ(UART_IRQn);
  
  for(uint32_t i = 0U; i<num; i++) {
    USART->DR = buf[i];
    while (USART->FR & UART_FR_TXFF_Msk);
  }
  
  while (USART->FR & UART_FR_TXFE_Msk);
  
  if (irq_state) {
    NVIC_EnableIRQ(UART_IRQn);
  }

  return ARM_DRIVER_OK;
}

static int32_t ARM_USART_Receive(void *data, uint32_t num)
{
  return ARM_DRIVER_ERROR_UNSUPPORTED;
}

static int32_t ARM_USART_Transfer(const void *data_out, void *data_in, uint32_t num)
{
  return ARM_DRIVER_ERROR_UNSUPPORTED;
}

static uint32_t ARM_USART_GetTxCount(void)
{
  return 0U;
}

static uint32_t ARM_USART_GetRxCount(void)
{
  return 0U;
}

static int32_t ARM_USART_Control(uint32_t control, uint32_t arg)
{
  const uint32_t mode = ARM_USART_MODE_ASYNCHRONOUS;

  // Reset local variables
  uint32_t uart_cr    = 0U;
  uint32_t uart_lcr_h = 0U;
  uint32_t uart_imsc  = 0U;

  if (control & (1U << 4)) {
    // USART Miscellaneous Operations
    switch (control & ARM_USART_CONTROL_Msk) {
      case ARM_USART_SET_DEFAULT_TX_VALUE:       // Set default Transmit value (Synchronous Receive only); arg = value
        return ARM_DRIVER_ERROR_UNSUPPORTED;

      case ARM_USART_SET_IRDA_PULSE:             // Set IrDA Pulse in ns; arg: 0=3/16 of bit period
        return ARM_DRIVER_ERROR_UNSUPPORTED;

      case ARM_USART_SET_SMART_CARD_GUARD_TIME:  // Set Smart Card Guard Time; arg = number of bit periods
        return ARM_DRIVER_ERROR_UNSUPPORTED;

      case ARM_USART_SET_SMART_CARD_CLOCK:       // Set Smart Card Clock in Hz; arg: 0=Clock not generated
        return ARM_DRIVER_ERROR_UNSUPPORTED;

      case ARM_USART_CONTROL_SMART_CARD_NACK:    // Smart Card NACK generation; arg: 0=disabled, 1=enabled
        return ARM_DRIVER_ERROR_UNSUPPORTED;

      case ARM_USART_CONTROL_TX:                 // Transmitter; arg: 0=disabled, 1=enabled
        if (arg != 0U) {
          // Set transmit FIFO level to 1/8 full (4 bytes)
          USART->IFLS = USART->IFLS & ~(0x07U);

          // Enable USART transmitter
          USART->CR |= UART_CR_TXE_Msk;
          return ARM_DRIVER_OK;
        }
        return ARM_DRIVER_ERROR_UNSUPPORTED;

      case ARM_USART_CONTROL_RX:                 // Receiver; arg: 0=disabled, 1=enabled
        return ARM_DRIVER_ERROR_UNSUPPORTED;

      case ARM_USART_CONTROL_BREAK:              // Continuous Break transmission; arg: 0=disabled, 1=enabled
        return ARM_DRIVER_ERROR_UNSUPPORTED;

      case ARM_USART_ABORT_SEND:                 // Abort ARM_USART_Send
        return ARM_DRIVER_ERROR_UNSUPPORTED;
      
      case ARM_USART_ABORT_RECEIVE:              // Abort ARM_USART_Receive
        return ARM_DRIVER_ERROR_UNSUPPORTED;

      case ARM_USART_ABORT_TRANSFER:             // Abort ARM_USART_Transfer
        // Transfer not implemented
        return ARM_DRIVER_ERROR_UNSUPPORTED;

      default: 
        return ARM_DRIVER_ERROR;
    }
  } else {
    // USART Mode
    if ((USART->FR & UART_FR_TXFE_Msk) == 0U) {
      // Transmit FIFO not empty, there are bytes to be sent out
      return ARM_DRIVER_ERROR_BUSY;
    }

    if ((control & ARM_USART_CONTROL_Msk) != ARM_USART_MODE_ASYNCHRONOUS) {
      return ARM_DRIVER_ERROR_UNSUPPORTED;
    }

    // Data Bits
    switch (control & ARM_USART_DATA_BITS_Msk) {
      case ARM_USART_DATA_BITS_5:
        break;
      case ARM_USART_DATA_BITS_6:
          uart_lcr_h |= (1U << UART_LCR_H_WLEN_Pos);
        break;
      case ARM_USART_DATA_BITS_7:
          uart_lcr_h |= (2U << UART_LCR_H_WLEN_Pos);
        break;
      case ARM_USART_DATA_BITS_8:
          uart_lcr_h |= (3U << UART_LCR_H_WLEN_Pos);
        break;
      default:  
        return ARM_USART_ERROR_DATA_BITS;
    }

    // Parity Bit
    switch (control & ARM_USART_PARITY_Msk) {
      case ARM_USART_PARITY_NONE:
        break;
      case ARM_USART_PARITY_ODD:
        uart_lcr_h |=  UART_LCR_H_PEN_Msk;
        break;
      case ARM_USART_PARITY_EVEN:
        uart_lcr_h |= (UART_LCR_H_EPS_Msk | UART_LCR_H_PEN_Msk);
        break;
      default: 
        return ARM_USART_ERROR_PARITY;
    }

    // Stop Bit
    switch (control & ARM_USART_STOP_BITS_Msk) {
      case ARM_USART_STOP_BITS_1:
        break;
      case ARM_USART_STOP_BITS_2:
        uart_lcr_h |= UART_LCR_H_STP2_Msk;
        break;
      default: 
        return ARM_USART_ERROR_STOP_BITS;
    }

    if ((control & ARM_USART_FLOW_CONTROL_Msk) != ARM_USART_FLOW_CONTROL_NONE) {
      return ARM_USART_ERROR_FLOW_CONTROL;
    }

    // Configuration is valid, apply settings

    // Setup baud rate dividers
    USART_SetBaudrate(SystemCoreClock, arg);

    // Configure USART registers
    USART->IMSC  = uart_imsc;
    USART->LCR_H = uart_lcr_h | UART_LCR_H_FEN_Msk /* Enable FIFO */;
    USART->CR    = uart_cr;

    // Enable UART
    USART->CR |= UART_CR_UARTEN_Msk;
  }

  return ARM_DRIVER_OK;
}

static ARM_USART_STATUS ARM_USART_GetStatus(void)
{
  static const ARM_USART_STATUS status;
  return status;
}

static int32_t ARM_USART_SetModemControl(ARM_USART_MODEM_CONTROL control)
{
  return ARM_DRIVER_OK;
}

static ARM_USART_MODEM_STATUS ARM_USART_GetModemStatus(void)
{
  static const ARM_USART_MODEM_STATUS status;
  return status;
}

// End USART Interface

extern \
ARM_DRIVER_USART ARM_Driver_USART_(0);
ARM_DRIVER_USART ARM_Driver_USART_(0) = {
    ARM_USART_GetVersion,
    ARM_USART_GetCapabilities,
    ARM_USART_Initialize,
    ARM_USART_Uninitialize,
    ARM_USART_PowerControl,
    ARM_USART_Send,
    ARM_USART_Receive,
    ARM_USART_Transfer,
    ARM_USART_GetTxCount,
    ARM_USART_GetRxCount,
    ARM_USART_Control,
    ARM_USART_GetStatus,
    ARM_USART_SetModemControl,
    ARM_USART_GetModemStatus
};
