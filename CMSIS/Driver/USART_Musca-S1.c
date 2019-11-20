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
 * Driver:       Driver_USART0, Driver_USART1
 *
 * Configured:   via RTE_Device.h configuration file
 * Project:      USART Driver for Musca-S1 device
 * --------------------------------------------------------------------------
 * Use the following configuration settings in the middleware component
 * to connect to this driver.
 *
 *   Configuration Setting                   Value   UART Interface
 *   ---------------------                   -----   --------------
 *   Connect to hardware via Driver_USART# = 0       use UART0
 *   Connect to hardware via Driver_USART# = 1       use UART1
 * --------------------------------------------------------------------------
 * Note(s): possible defines select the used communication interface:
 *            __USE_SECURE  - use register in secure address space
 *                          - use register in non-secure address space (default)
 *----------------------------------------------------------------------------*/

/* History:
 *  Version 1.0.0
 *    Initial release
 */

#include "USART_Musca-S1.h"

#define UNUSED(x) (void)(x)           /* macro to get rid of 'unused parameter' warning */

#define ARM_USART_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1,0)

// Driver Version
static const ARM_DRIVER_VERSION usart_driver_version = { ARM_USART_API_VERSION, ARM_USART_DRV_VERSION };

// Receiver Timeout values
#ifndef USART0_RX_TIMEOUT_VAL
  #define USART0_RX_TIMEOUT_VAL     30U
#endif
#ifndef USART1_RX_TIMEOUT_VAL
  #define USART1_RX_TIMEOUT_VAL     30U
#endif

#if defined __USE_SECURE
  #define SCC_REGISTER   SECURE_SCC
  #define GPIO_REGISTER  SECURE_GPIO0
#else
  #define SCC_REGISTER   SCC
  #define GPIO_REGISTER  GPIO0
#endif


// USART0
#if (defined (RTE_USART0) && (RTE_USART0 != 0))

#if defined __USE_SECURE
  #define UART0_REGISTER SECURE_UART0
#else
  #define UART0_REGISTER UART0
#endif

// USART0 Run-Time Information
static USART_INFO USART0_Info = {
  .cb_event = 0U,
  .status   = {
    .tx_busy          = 0U,
    .rx_busy          = 0U,
    .tx_underflow     = 0U,
    .rx_overflow      = 0U,
    .rx_break         = 0U,
    .rx_framing_error = 0U,
    .rx_parity_error  = 0U,
    .padding     = {0U}
  },
  .flags        = 0U,
  .mode         = 0U,
  .flow_control = 0U
};

static USART_TRANSFER_INFO USART0_TransferInfo = {
  .rx_num      = 0U,
  .tx_num      = 0U,
  .rx_buf      = 0U,
  .tx_buf      = 0U,
  .rx_cnt      = 0U,
  .tx_cnt      = 0U,
  .break_flag  = 0U,
  .send_active = 0U,
  .padding     = {0U, 0U}
};

#if defined (RTE_USART0_RX) && (RTE_USART0_RX != 0U)
static USART_PIN USART0_pin_rx = {
  .pin = 0U,      // PA0
  .af  = 1U       // Alternate Function 1
};
#endif

#if defined (RTE_USART0_TX) && (RTE_USART0_TX != 0U)
static USART_PIN USART0_pin_tx = {
  .pin =  1U,     // PA1
  .af  =  1U      // Alternate Function 1
};
#endif

#if defined (RTE_USART0_CK) && (RTE_USART0_CK != 0U)
static USART_PIN USART0_pin_ck = {
  .pin =  9U,     // PA9
  .af  =  2U      // Alternate Function 2
};
#endif

#if defined (RTE_USART0_CTS) && (RTE_USART0_CTS != 0U)
static USART_PIN USART0_pin_cts = {
  .pin =  5U,     // PA5
  .af  =  2U      // Alternate Function 2
};
#endif

#if defined (RTE_USART0_RTS) && (RTE_USART0_RTS != 0U)
static USART_PIN USART0_pin_rts = {
  .pin =  6U,     // PA6
  .af  =  2U      // Alternate Function 2
};
#endif

// USART0 Resources
static const USART_RESOURCES USART0_Resources = {
  {      // Capabilities
    1U,  // supports UART (Asynchronous) mode
    0U,  // supports Synchronous Master mode
    0U,  // supports Synchronous Slave mode
    0U,  // supports UART Single-wire mode
    0U,  // supports UART IrDA mode
    0U,  // supports UART Smart Card mode
    0U,  // Smart Card Clock generator
#if defined (RTE_USART0_RTS) && (RTE_USART0_RTS != 0U)
    1U,  // RTS Flow Control available
#else
    0U,  // RTS Flow Control available
#endif
#if defined (RTE_USART0_CTS) && (RTE_USART0_CTS != 0U)
    1U,  // CTS Flow Control available
#else
    0U,  // CTS Flow Control available
#endif
    0U,  // Transmit completed event: \ref ARM_USART_EVENT_TX_COMPLETE
    0U,  // Signal receive character timeout event: \ref ARM_USART_EVENT_RX_TIMEOUT
#if defined (RTE_USART0_RTS) && (RTE_USART0_RTS != 0U)
    1U,  // RTS Line: 0=not available, 1=available
#else
    0U,  // RTS Line: 0=not available, 1=available
#endif
#if defined (RTE_USART0_CTS) && (RTE_USART0_CTS != 0U)
    1U,  // CTS Line: 0=not available, 1=available
#else
    0U,  // CTS Line: 0=not available, 1=available
#endif
    0U,  // DTR Line: 0=not available, 1=available
    0U,  // DSR Line: 0=not available, 1=available
    0U,  // DCD Line: 0=not available, 1=available
    0U,  // RI Line: 0=not available, 1=available
#if defined (RTE_USART0_CTS) && (RTE_USART0_CTS != 0U)
    1U,  // Signal CTS change event: \ref ARM_USART_EVENT_CTS
#else
    0U,  // Signal CTS change event: \ref ARM_USART_EVENT_CTS
#endif
    0U,  // Signal DSR change event: \ref ARM_USART_EVENT_DSR
    0U,  // Signal DCD change event: \ref ARM_USART_EVENT_DCD
    0U,  // Signal RI change event: \ref ARM_USART_EVENT_RI
    0U   // Reserved (must be zero)
  },
  .io = {
#if defined (RTE_USART0_RX) && (RTE_USART0_RX != 0U)
    .rx  = &USART0_pin_rx,
#else
    NULL,
#endif
#if defined (RTE_USART0_TX) && (RTE_USART0_TX != 0U)
    .tx  = &USART0_pin_tx,
#else
    NULL,
#endif
#if defined (RTE_USART0_CK) && (RTE_USART0_CK != 0U)
    .ck  = &USART0_pin_ck,
#else
    NULL,
#endif
#if defined (RTE_USART0_CTS) && (RTE_USART0_CTS != 0U)
    .cts = &USART0_pin_cts,
#else
    NULL,
#endif
#if defined (RTE_USART0_RTS) && (RTE_USART0_RTS != 0U)
    .rts = &USART0_pin_rts
#else
    NULL,
#endif
  },
  .reg            =  UART0_REGISTER,
  .info           = &USART0_Info,
  .xfer           = &USART0_TransferInfo,
  .irq_num        =  UART0_IRQn,
  .padding0       =  {0U, 0U, 0U},
  .rx_timeout_val = USART0_RX_TIMEOUT_VAL,
  .reset_bit      =  7U,
  .padding1       =  {0U, 0U, 0U}
};

#endif

// USART1
#if (defined (RTE_USART1) && (RTE_USART1 != 0))

#if defined __USE_SECURE
  #define UART1_REGISTER SECURE_UART1
#else
  #define UART1_REGISTER UART1
#endif

// USART1 Run-Time Information
static USART_INFO USART1_Info = {
  .cb_event = 0U,
  .status   = {
    .tx_busy          = 0U,
    .rx_busy          = 0U,
    .tx_underflow     = 0U,
    .rx_overflow      = 0U,
    .rx_break         = 0U,
    .rx_framing_error = 0U,
    .rx_parity_error  = 0U,
    .padding     = {0U}
  },
  .flags        = 0U,
  .mode         = 0U,
  .flow_control = 0U
};

static USART_TRANSFER_INFO USART1_TransferInfo = {
  .rx_num      = 0U,
  .tx_num      = 0U,
  .rx_buf      = 0U,
  .tx_buf      = 0U,
  .rx_cnt      = 0U,
  .tx_cnt      = 0U,
  .break_flag  = 0U,
  .send_active = 0U,
  .padding     = {0U, 0U}
};

#if defined (RTE_USART1_RX) && (RTE_USART1_RX != 0U)
static USART_PIN USART1_pin_rx = {
  .pin = 16U,     // PA16
  .af  =  0U      // Primary Function
};
#endif

#if defined (RTE_USART1_TX) && (RTE_USART1_TX != 0U)
static USART_PIN USART1_pin_tx = {
  .pin = 17U,     // PA17
  .af  =  0U      // Primary Function
};
#endif

#if defined (RTE_USART1_CTS) && (RTE_USART1_CTS != 0U)
static USART_PIN USART1_pin_cts = {
  .pin =  7U,     // PA7
  .af  =  2U      // Alternate Function 2
};
#endif

#if defined (RTE_USART1_RTS) && (RTE_USART1_RTS != 0U)
static USART_PIN USART1_pin_rts = {
  .pin =  8U,     // PA8 */
  .af  =  2U      // Alternate Function 2
};
#endif

// USART1 Resources
static const USART_RESOURCES USART1_Resources = {
  {     // Capabilities
    1U,  // supports UART (Asynchronous) mode
    0U,  // supports Synchronous Master mode
    0U,  // supports Synchronous Slave mode
    0U,  // supports UART Single-wire mode
    0U,  // supports UART IrDA mode
    0U,  // supports UART Smart Card mode
    0U,  // Smart Card Clock generator
#if defined (RTE_USART1_RTS) && (RTE_USART1_RTS != 0U)
    1U,  // RTS Flow Control available
#else
    0U,  // RTS Flow Control available
#endif
#if defined (RTE_USART1_CTS) && (RTE_USART1_CTS != 0U)
    1U,  // CTS Flow Control available
#else
    0U,  // CTS Flow Control available
#endif
    0U,  // Transmit completed event: \ref ARM_USART_EVENT_TX_COMPLETE
    0U,  // Signal receive character timeout event: \ref ARM_USART_EVENT_RX_TIMEOUT
#if defined (RTE_USART1_RTS) && (RTE_USART1_RTS != 0U)
    1U,  // RTS Line: 0=not available, 1=available
#else
    0U,  // RTS Line: 0=not available, 1=available
#endif
#if defined (RTE_USART1_CTS) && (RTE_USART1_CTS != 0U)
    1U,  // CTS Line: 0=not available, 1=available
#else
    0U,  // CTS Line: 0=not available, 1=available
#endif
    0U,  // DTR Line: 0=not available, 1=available
    0U,  // DSR Line: 0=not available, 1=available
    0U,  // DCD Line: 0=not available, 1=available
    0U,  // RI Line: 0=not available, 1=available
#if defined (RTE_USART1_CTS) && (RTE_USART1_CTS != 0U)
    1U,  // Signal CTS change event: \ref ARM_USART_EVENT_CTS
#else
    0U,  // Signal CTS change event: \ref ARM_USART_EVENT_CTS
#endif
    0U,  // Signal DSR change event: \ref ARM_USART_EVENT_DSR
    0U,  // Signal DCD change event: \ref ARM_USART_EVENT_DCD
    0U,  // Signal RI change event: \ref ARM_USART_EVENT_RI
    0U   // Reserved (must be zero)
  },
  .io = {
#if defined (RTE_USART1_RX) && (RTE_USART1_RX != 0U)
    .rx  = &USART1_pin_rx,
#else
    NULL,
#endif
#if defined (RTE_USART1_TX) && (RTE_USART1_TX != 0U)
    .tx  = &USART1_pin_tx,
#else
    NULL,
#endif
#if defined (RTE_USART1_CK) && (RTE_USART0_CK != 0U)
    .ck  = &USART1_pin_ck,
#else
    NULL,
#endif
#if defined (RTE_USART1_CTS) && (RTE_USART1_CTS != 0U)
    .cts = &USART1_pin_cts,
#else
    NULL,
#endif
#if defined (RTE_USART1_RTS) && (RTE_USART1_RTS != 0U)
    .rts = &USART1_pin_rts
#else
    NULL,
#endif
  },
  .reg            =  UART1_REGISTER,
  .info           = &USART1_Info,
  .xfer           = &USART1_TransferInfo,
  .irq_num        =  UART1_IRQn,
  .padding0       =  {0U, 0U, 0U},
  .rx_timeout_val = USART1_RX_TIMEOUT_VAL,
  .reset_bit      =  8U,
  .padding1       =  {0U, 0U, 0U}
};
#endif


/**
  \fn          int USART_SetBaudrate (const USART_RESOURCES *usart, uint32_t clk, uint32_t baudrate)
  \brief       Configure baudrate
  \param[in]   usart     Pointer to USART resources
  \param[in]   clk       USART clock
  \param[in]   baudrate  Baudrate
  \return      execution status
                -  0  = OK
                - -1  = Failed
*/
static int32_t USART_SetBaudrate (const USART_RESOURCES *usart,
                                     uint32_t         clk,
                                     uint32_t         baudrate) {

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

    usart->reg->IBRD = (uint32_t)(bauddiv >> UART_FBRD_WIDTH);
    usart->reg->FBRD = (uint32_t)(bauddiv & ((1U << UART_FBRD_WIDTH) - 1U));

    __DMB();

    /* In order to internally update the contents of uartibrd or uartfbrd,
       a uartlcr_h write must always be performed at the end
       ARM DDI 0183F, Pg 3-13 */
    usart->reg->LCR_H = usart->reg->LCR_H;

    return 0;
}

/**
  \fn          void USART_PinInit (uint16_t pin, uint16_t af)
  \brief       Initialize USART pin
*/
static void USART_PinInit (uint16_t pin, uint16_t af) {

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

/**
  \fn          void USART_PinDeInit (uint16_t pin)
  \brief       DeInitialize USART pin
*/
static void USART_PinDeInit (uint16_t pin) {

  SCC_REGISTER->IOMUX_MAIN_INSEL   |=  (1U << pin);
  SCC_REGISTER->IOMUX_MAIN_OUTSEL  |=  (1U << pin);
  SCC_REGISTER->IOMUX_MAIN_OENSEL  |=  (1U << pin);
}

// USART Driver functions

/**
  \fn          ARM_DRIVER_VERSION USARTx_GetVersion (void)
  \brief       Get driver version.
  \return      \ref ARM_DRIVER_VERSION
*/
static ARM_DRIVER_VERSION USARTx_GetVersion (void) {
  return usart_driver_version;
}

/**
  \fn          ARM_USART_CAPABILITIES USART_GetCapabilities (const USART_RESOURCES *usart)
  \brief       Get driver capabilities
  \param[in]   usart     Pointer to USART resources
  \return      \ref ARM_USART_CAPABILITIES
*/
static ARM_USART_CAPABILITIES USART_GetCapabilities (const USART_RESOURCES *usart) {
  return usart->capabilities;
}

/**
  \fn          int32_t USART_Initialize (const USART_RESOURCES         *usart,
                                               ARM_USART_SignalEvent_t  cb_event)
  \brief       Initialize USART Interface.
  \param[in]   usart     Pointer to USART resources
  \param[in]   cb_event  Pointer to \ref ARM_USART_SignalEvent
  \return      \ref execution_status
*/
static int32_t USART_Initialize (const USART_RESOURCES         *usart,
                                       ARM_USART_SignalEvent_t  cb_event) {

  if (usart->info->flags & USART_FLAG_INITIALIZED) {
    // Driver is already initialized
    return ARM_DRIVER_OK;
  }

  // Initialize Run-time Resources
  usart->info->cb_event = cb_event;

  // Clear Status flags
  usart->info->status.tx_busy          = 0U;
  usart->info->status.rx_busy          = 0U;
  usart->info->status.tx_underflow     = 0U;
  usart->info->status.rx_overflow      = 0U;
  usart->info->status.rx_break         = 0U;
  usart->info->status.rx_framing_error = 0U;
  usart->info->status.rx_parity_error  = 0U;

  usart->info->mode        = 0U;

  // Clear transfer information
  memset((void *)usart->xfer, 0, sizeof(USART_TRANSFER_INFO));

  // configure RX pin
  if (usart->io.rx) {
    USART_PinInit (usart->io.rx->pin, usart->io.rx->af);
  }

  // configure TX pin
  if (usart->io.tx) {
    USART_PinInit (usart->io.tx->pin, usart->io.tx->af);
  }

  // configure SCLK pin
  if (usart->io.sclk) {
    USART_PinInit (usart->io.sclk->pin, usart->io.sclk->af);
  }

  // configure CTS pin
  if (usart->io.cts) {
    USART_PinInit (usart->io.cts->pin, usart->io.cts->af);
  }

  // configure RTS pin
  if (usart->io.rts) {
    USART_PinInit (usart->io.rts->pin, usart->io.rts->af);
  }

  usart->info->flags = USART_FLAG_INITIALIZED;

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USART_Uninitialize (const USART_RESOURCES *usart)
  \brief       De-initialize USART Interface.
  \param[in]   usart     Pointer to USART resources
  \return      \ref execution_status
*/
static int32_t USART_Uninitialize (const USART_RESOURCES *usart) {

  // unConfigure RX pin
  if (usart->io.rx) {
    USART_PinDeInit (usart->io.rx->pin);
  }

  // unConfigure TX pin
  if (usart->io.tx) {
    USART_PinDeInit (usart->io.tx->pin);
  }

  // unConfigure CLK pin
  if (usart->io.sclk) {
    USART_PinDeInit (usart->io.sclk->pin);
  }

  // unConfigure CTS pin
  if (usart->io.cts) {
    USART_PinDeInit (usart->io.cts->pin);
  }

  // unConfigure RTS pin
  if (usart->io.rts) {
    USART_PinDeInit (usart->io.rts->pin);
  }

  // reset status flags
  usart->info->flags = 0U;

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USART_PowerControl (const USART_RESOURCES *usart,
                                                 ARM_POWER_STATE  state)
  \brief       Control USART Interface Power.
  \param[in]   usart  Pointer to USART resources
  \param[in]   state  Power state
  \return      \ref execution_status
*/
static int32_t USART_PowerControl (const USART_RESOURCES *usart,
                                         ARM_POWER_STATE  state) {

  if ((state != ARM_POWER_OFF)  &&
      (state != ARM_POWER_FULL) &&
      (state != ARM_POWER_LOW)) {
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  switch (state) {
    case ARM_POWER_OFF:
      // clear and disable USART IRQ
      NVIC_DisableIRQ(usart->irq_num);
      NVIC_ClearPendingIRQ(usart->irq_num);

      // Disable USART
      usart->reg->CR = 0U;

      // hold UART in Reset
      SCC_REGISTER->RESET_CTRL &= ~(1U << usart->reset_bit);

      // Clear Status flags
      usart->info->status.tx_busy          = 0U;
      usart->info->status.rx_busy          = 0U;
      usart->info->status.tx_underflow     = 0U;
      usart->info->status.rx_overflow      = 0U;
      usart->info->status.rx_break         = 0U;
      usart->info->status.rx_framing_error = 0U;
      usart->info->status.rx_parity_error  = 0U;

      usart->xfer->send_active             = 0U;

      // Clear powered flag
      usart->info->flags &= ~USART_FLAG_POWERED;
      break;

    case ARM_POWER_LOW:
      return ARM_DRIVER_ERROR_UNSUPPORTED;

    case ARM_POWER_FULL:
      if ((usart->info->flags & USART_FLAG_INITIALIZED) == 0U) {
        return ARM_DRIVER_ERROR;
      }
      if ((usart->info->flags & USART_FLAG_POWERED)     != 0U) {
        return ARM_DRIVER_OK;
      }

      // clear Status flags
      usart->info->status.tx_busy          = 0U;
      usart->info->status.rx_busy          = 0U;
      usart->info->status.tx_underflow     = 0U;
      usart->info->status.rx_overflow      = 0U;
      usart->info->status.rx_break         = 0U;
      usart->info->status.rx_framing_error = 0U;
      usart->info->status.rx_parity_error  = 0U;

      usart->xfer->send_active             = 0U;
      usart->xfer->break_flag              = 0U;

      usart->info->mode                    = 0U;
      usart->info->flow_control            = 0U;

      // set flag initialized
      usart->info->flags = USART_FLAG_POWERED | USART_FLAG_INITIALIZED;

      // clear and enable USART IRQ
      NVIC_ClearPendingIRQ(usart->irq_num);
      NVIC_EnableIRQ(usart->irq_num);

      // release UART from reset
      SCC_REGISTER->RESET_CTRL |= (1U << usart->reset_bit);
      while ((SCC_REGISTER->RESET_CTRL & (1U << usart->reset_bit)) == 0U);

      // disable USART
      usart->reg->CR = 0U;
      break;
  }

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USART_Send (const USART_RESOURCES *usart,
                                   const void            *data,
                                         uint32_t         num)
  \brief       Start sending data to USART transmitter.
  \param[in]   usart Pointer to USART resources
  \param[in]   data  Pointer to buffer with data to send to USART transmitter
  \param[in]   num   Number of data items to send
  \return      \ref execution_status
*/
static int32_t USART_Send (const USART_RESOURCES *usart,
                           const void            *data,
                                 uint32_t         num) {

  if ((data == NULL) || (num == 0U)) {
    // Invalid parameters
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  if ((usart->info->flags & USART_FLAG_CONFIGURED) == 0U) {
    // USART is not configured (mode not selected)
    return ARM_DRIVER_ERROR;
  }

  if (usart->xfer->send_active != 0U) {
    // Send is not completed yet
    return ARM_DRIVER_ERROR_BUSY;
  }

  // set Send Active flag
  usart->xfer->send_active = 1U;

  // save transmit buffer info
  usart->xfer->tx_buf = (const uint8_t *)data;
  usart->xfer->tx_num = num;
  usart->xfer->tx_cnt = 0U;

  // enable TX interrupt
  usart->reg->IMSC |= UART_IMSC_TXIM_Msk;

  // start sending
  usart->reg->DR = usart->xfer->tx_buf[usart->xfer->tx_cnt];

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USART_Receive (const USART_RESOURCES *usart,
                                            void            *data,
                                            uint32_t         num)
  \brief       Start receiving data from USART receiver.
  \param[in]   usart Pointer to USART resources
  \param[out]  data  Pointer to buffer for data to receive from USART receiver
  \param[in]   num   Number of data items to receive
  \return      \ref execution_status
*/
static int32_t USART_Receive (const USART_RESOURCES *usart,
                                    void            *data,
                                    uint32_t         num) {

  if ((data == NULL) || (num == 0U)) {
    // invalid parameters
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  if ((usart->info->flags & USART_FLAG_CONFIGURED) == 0U) {
    // USART is not configured (mode not selected)
    return ARM_DRIVER_ERROR;
  }

  // check if receiver is busy
  if (usart->info->status.rx_busy == 1U) {
    return ARM_DRIVER_ERROR_BUSY;
  }

  // disable RXNE Interrupt
  usart->reg->IMSC &= ~UART_IMSC_RXIM_Msk;

  // save number of data to be received
  usart->xfer->rx_num = num;

  // clear RX status
  usart->info->status.rx_break          = 0U;
  usart->info->status.rx_framing_error  = 0U;
  usart->info->status.rx_overflow       = 0U;
  usart->info->status.rx_parity_error   = 0U;

  // save receive buffer info
  usart->xfer->rx_buf = (uint8_t *)data;
  usart->xfer->rx_cnt =  0U;

  // set RX Busy flag
  usart->info->status.rx_busy = 1U;

  // enable RXIM interrupt
  usart->reg->IMSC |= UART_IMSC_RXIM_Msk;

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t USART_Transfer (const USART_RESOURCES *usart,
                                       const void            *data_out,
                                             void            *data_in,
                                             uint32_t         num)
  \brief       Start sending/receiving data to/from USART transmitter/receiver.
  \param[in]   usart     Pointer to USART resources
  \param[in]   data_out  Pointer to buffer with data to send to USART transmitter
  \param[out]  data_in   Pointer to buffer for data to receive from USART receiver
  \param[in]   num       Number of data items to transfer
  \return      \ref execution_status
*/
static int32_t USART_Transfer (const USART_RESOURCES *usart,
                               const void            *data_out,
                                     void            *data_in,
                                     uint32_t         num) {
  UNUSED(data_out);
  UNUSED(data_in);
  UNUSED(num);
  UNUSED(usart);

  // Synchronous mode not supported
  return ARM_DRIVER_ERROR_UNSUPPORTED;
}

/**
  \fn          uint32_t USART_GetTxCount (const USART_RESOURCES *usart)
  \brief       Get transmitted data count.
  \param[in]   usart     Pointer to USART resources
  \return      number of data items transmitted
*/
static uint32_t USART_GetTxCount (const USART_RESOURCES *usart) {

  return usart->xfer->tx_cnt;
}

/**
  \fn          uint32_t USART_GetRxCount (const USART_RESOURCES *usart)
  \brief       Get received data count.
  \param[in]   usart     Pointer to USART resources
  \return      number of data items received
*/
static uint32_t USART_GetRxCount (const USART_RESOURCES *usart) {

  return usart->xfer->rx_cnt;
}

/**
  \fn          int32_t USART_Control (const USART_RESOURCES *usart,
                                            uint32_t         control,
                                            uint32_t         arg)
  \brief       Control USART Interface.
  \param[in]   usart    Pointer to USART resources
  \param[in]   control  Operation
  \param[in]   arg      Argument of operation (optional)
  \return      common \ref execution_status and driver specific \ref usart_execution_status
*/
static int32_t USART_Control (const USART_RESOURCES *usart,
                                    uint32_t         control,
                                    uint32_t         arg) {
  uint32_t mode;
  uint32_t uart_cr, uart_lcr_h, uart_imsc;

  // Reset local variables
  uart_cr = 0U;
  uart_lcr_h = 0U;
  uart_imsc = 0U;

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
        // Check if TX pin available
        if (usart->io.tx == NULL) { return ARM_DRIVER_ERROR; }

        if (arg) {
          // select USART TX pin function
//        USART_PinInit (usart->io.tx->pin, usart->io.tx->af);

          // enable USART transmitter
          usart->reg->CR |= UART_CR_TXE_Msk;

          usart->info->flags |= USART_FLAG_TX_ENABLED;
        } else {
          // disable USART transmitter
          usart->reg->CR &= ~UART_CR_TXE_Msk;

          // select GPIO pin function
//        USART_PinDeInit (usart->io.tx->pin);

          usart->info->flags &= ~USART_FLAG_TX_ENABLED;
        }
        return ARM_DRIVER_OK;

      case ARM_USART_CONTROL_RX:                 // Receiver; arg: 0=disabled, 1=enabled
        // Check if RX line available
        if (usart->io.rx == NULL) { return ARM_DRIVER_ERROR; }

        if (arg) {
          // select USART RX pin function
//        USART_PinInit (usart->io.rx->pin, usart->io.rx->af);

          // enable USART error interrupts
          usart->reg->IMSC |=  (UART_IMSC_OEIM_Msk |      /* Overrun error */
                                UART_IMSC_BEIM_Msk |      /* Break error */
                                UART_IMSC_PEIM_Msk |      /* Parity error */
                                UART_IMSC_FEIM_Msk  );    /* Framing error */

          // enable USART receiver
          usart->reg->CR |=  UART_CR_RXE_Msk;

          usart->info->flags |=  USART_FLAG_RX_ENABLED;
        } else {
          // disable USART receiver
          usart->reg->CR &= ~UART_CR_RXE_Msk;

          // disable USART error interrupts
          usart->reg->IMSC &= ~(UART_IMSC_OEIM_Msk |      /* Overrun error */
                                UART_IMSC_BEIM_Msk |      /* Break error */
                                UART_IMSC_PEIM_Msk |      /* Parity error */
                                UART_IMSC_FEIM_Msk  );    /* Framing error */

          // select GPIO pin function
//        USART_PinDeInit (usart->io.rx->pin);

          usart->info->flags &= ~USART_FLAG_RX_ENABLED;
        }
        return ARM_DRIVER_OK;

      case ARM_USART_CONTROL_BREAK:              // Continuous Break transmission; arg: 0=disabled, 1=enabled
        if (arg) {
          if (usart->xfer->send_active != 0U) { return ARM_DRIVER_ERROR_BUSY; }

          // set Send Active and Break flag
          usart->xfer->send_active = 1U;
          usart->xfer->break_flag  = 1U;

          // send break and enable TX interrupt
          usart->reg->IMSC  |= UART_IMSC_TXIM_Msk;
          usart->reg->LCR_H |= UART_LCR_H_BRK_Msk;
        } else {
            if (usart->xfer->break_flag) {
              // remove break and disable TX interrupt
              usart->reg->IMSC  &= ~UART_IMSC_TXIM_Msk;
              usart->reg->LCR_H &= ~UART_LCR_H_BRK_Msk;

              // clear Break and Send Active flag
              usart->xfer->break_flag  = 0U;
              usart->xfer->send_active = 0U;
            }
        }
        return ARM_DRIVER_OK;

      case ARM_USART_ABORT_SEND:                 // Abort ARM_USART_Send
        // disable TX interrupt
        usart->reg->IMSC &= ~UART_IMSC_TXIM_Msk;
  
        // clear Break and Send Active flag
        usart->xfer->break_flag  = 0U;
        usart->xfer->send_active = 0U;
        return ARM_DRIVER_OK;

      case ARM_USART_ABORT_RECEIVE:              // Abort ARM_USART_Receive
        // disable RX interrupt
        usart->reg->IMSC &= ~UART_IMSC_RXIM_Msk;
  
        // clear RX Busy status
        usart->info->status.rx_busy = 0U;
        return ARM_DRIVER_OK;

      case ARM_USART_ABORT_TRANSFER:             // Abort ARM_USART_Transfer
        // disable TX and RX interrupt
        usart->reg->IMSC &= ~(UART_IMSC_TXIM_Msk | UART_IMSC_RXIM_Msk);
  
        // clear RX Busy status and Send Active flag
        usart->info->status.rx_busy = 0U;
        usart->xfer->send_active    = 0U;
        return ARM_DRIVER_OK;

      default: 
        return ARM_DRIVER_ERROR;
    }
  } else {
    // USART Mode

    // Check if busy
    if ((usart->info->status.rx_busy != 0U) ||
        (usart->xfer->send_active    != 0U)   ) {
      return ARM_DRIVER_ERROR_BUSY;
    }

    if (((usart->reg->FR & UART_FR_TXFF_Msk) != 0U) &&
        ((usart->reg->FR & UART_FR_RXFF_Msk) != 0U)   ) {
      return ARM_DRIVER_ERROR_BUSY;
    }

    switch (control & ARM_USART_CONTROL_Msk) {
      case ARM_USART_MODE_ASYNCHRONOUS:          // UART (Asynchronous); arg = Baudrate
        mode = ARM_USART_MODE_ASYNCHRONOUS;
        break;

      case ARM_USART_MODE_SYNCHRONOUS_MASTER:    // Synchronous Master (generates clock signal); arg = Baudrate
        return ARM_DRIVER_ERROR_UNSUPPORTED;

      case ARM_USART_MODE_SYNCHRONOUS_SLAVE:     // Synchronous Slave (external clock signal)
        return ARM_DRIVER_ERROR_UNSUPPORTED;

      case ARM_USART_MODE_SINGLE_WIRE:           // UART Single-wire (half-duplex); arg = Baudrate
        return ARM_DRIVER_ERROR_UNSUPPORTED;

      case ARM_USART_MODE_IRDA:                  // UART IrDA; arg = Baudrate
        return ARM_DRIVER_ERROR_UNSUPPORTED;

      case ARM_USART_MODE_SMART_CARD:            // UART Smart Card; arg = Baudrate
        return ARM_DRIVER_ERROR_UNSUPPORTED;

      default:
        return ARM_DRIVER_ERROR;
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
      default:  return ARM_USART_ERROR_DATA_BITS;
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
      default: return ARM_USART_ERROR_PARITY;
    }

    // Stop Bit
    switch (control & ARM_USART_STOP_BITS_Msk) {
      case ARM_USART_STOP_BITS_1:
        break;
      case ARM_USART_STOP_BITS_2:
        uart_lcr_h |= UART_LCR_H_STP2_Msk;
        break;
      default: return ARM_USART_ERROR_STOP_BITS;
    }

    // Flow Control
    switch (control & ARM_USART_FLOW_CONTROL_Msk) {
      case ARM_USART_FLOW_CONTROL_NONE:
        break;
      case ARM_USART_FLOW_CONTROL_RTS:
        // Check if RTS pin is available
        if (usart->io.rts == NULL) { return ARM_USART_ERROR_FLOW_CONTROL; }

        // select USART RTS pin function
//      USART_PinInit (usart->io.rts->pin, usart->io.rts->af);

        // enable RTS function
        uart_cr |= UART_CR_RTSEn_Msk;
        break;
      case ARM_USART_FLOW_CONTROL_CTS:
        // Check if CTS pin is available
        if (usart->io.cts == NULL) { return ARM_USART_ERROR_FLOW_CONTROL; }

        // select USART CTS pin function
//      USART_PinInit (usart->io.cts->pin, usart->io.cts->af);

        // enable CTS function and CTS interrupt
        uart_cr |= UART_CR_CTSEn_Msk;
        uart_imsc |= UART_IMSC_CTSMIM_Msk;
        break;
      case ARM_USART_FLOW_CONTROL_RTS_CTS:
        // Check if RTS pin and CTS pin is available
        if (usart->io.rts == NULL) { return ARM_USART_ERROR_FLOW_CONTROL; }
        if (usart->io.cts == NULL) { return ARM_USART_ERROR_FLOW_CONTROL; }

        // enable RTS, CTS function and CTS interrupt
        uart_cr |= (UART_CR_RTSEn_Msk | UART_CR_CTSEn_Msk);
        uart_imsc |= UART_IMSC_CTSMIM_Msk;
        break;
      default: return ARM_USART_ERROR_FLOW_CONTROL;
    }

    // Clock Polarity  (only synchronous mode)

    // Clock Phase     (only synchronous mode)

    // Baudrate
    USART_SetBaudrate(usart, SystemCoreClock, arg);

    // configure USART registers
    usart->reg->IMSC  = uart_imsc;
    usart->reg->LCR_H = uart_lcr_h;
    usart->reg->CR    = uart_cr;

    // USART Enable
    usart->reg->CR |= UART_CR_UARTEN_Msk;

    // Configuration is OK - Mode is valid
    usart->info->mode = mode;

    // Set configured flag
    usart->info->flags |= USART_FLAG_CONFIGURED;
  }

  return ARM_DRIVER_OK;
}

/**
  \fn          ARM_USART_STATUS USART_GetStatus (const USART_RESOURCES *usart)
  \brief       Get USART status.
  \param[in]   usart     Pointer to USART resources
  \return      USART status \ref ARM_USART_STATUS
*/
static ARM_USART_STATUS USART_GetStatus (const USART_RESOURCES *usart) {
  ARM_USART_STATUS status;

  if (usart->xfer->send_active != 0U) {
    status.tx_busy        = 1U;
  } else {
    status.tx_busy        = ((usart->reg->FR & UART_FR_BUSY_Msk) ? (1U) : (0U));
  }
  status.rx_busy          = usart->info->status.rx_busy;
  status.tx_underflow     = usart->info->status.tx_underflow;
  status.rx_overflow      = usart->info->status.rx_overflow;
  status.rx_break         = usart->info->status.rx_break;
  status.rx_framing_error = usart->info->status.rx_framing_error;
  status.rx_parity_error  = usart->info->status.rx_parity_error;

  return status;
}

/**
  \fn          int32_t USART_SetModemControl (const USART_RESOURCES         *usart,
                                                    ARM_USART_MODEM_CONTROL  control)
  \brief       Set USART Modem Control line state.
  \param[in]   usart     Pointer to USART resources
  \param[in]   control   \ref ARM_USART_MODEM_CONTROL
  \return      \ref execution_status
*/
static int32_t USART_SetModemControl (const USART_RESOURCES         *usart,
                                            ARM_USART_MODEM_CONTROL  control) {

  if ((control != ARM_USART_RTS_CLEAR) &&
      (control != ARM_USART_RTS_SET)   &&
      (control != ARM_USART_DTR_CLEAR) &&
      (control != ARM_USART_DTR_SET)) {
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  switch (control) {
    case ARM_USART_RTS_CLEAR:
      if ((usart->info->flow_control == ARM_USART_FLOW_CONTROL_NONE) ||
          (usart->info->flow_control == ARM_USART_FLOW_CONTROL_CTS)) {
        if (usart->io.rts) {
          GPIO_REGISTER->DATAOUT |=  (1U << usart->io.rts->pin);
        }
      } else {
        // Hardware RTS
        return ARM_DRIVER_ERROR;
      }
      break;
    case ARM_USART_RTS_SET:
      if ((usart->info->flow_control == ARM_USART_FLOW_CONTROL_NONE) ||
          (usart->info->flow_control == ARM_USART_FLOW_CONTROL_CTS)) {
        if (usart->io.rts) {
          GPIO_REGISTER->DATAOUT &= ~(1U << usart->io.rts->pin);
        }
      } else {
        // Hardware RTS
        return ARM_DRIVER_ERROR;
      }
      break;
    case ARM_USART_DTR_CLEAR:
    case ARM_USART_DTR_SET:
      return ARM_DRIVER_ERROR_UNSUPPORTED;
  }

  return ARM_DRIVER_OK;
}

/**
  \fn          ARM_USART_MODEM_STATUS USART_GetModemStatus (const USART_RESOURCES *usart)
  \brief       Get USART Modem Status lines state.
  \param[in]   usart     Pointer to USART resources
  \return      modem status \ref ARM_USART_MODEM_STATUS
*/
static ARM_USART_MODEM_STATUS USART_GetModemStatus (const USART_RESOURCES *usart) {
  ARM_USART_MODEM_STATUS modem_status;

  modem_status.cts = 0U;
  if ((usart->info->flow_control == ARM_USART_FLOW_CONTROL_NONE) ||
      (usart->info->flow_control == ARM_USART_FLOW_CONTROL_RTS)) {
    if (usart->io.cts) {
      if ((GPIO_REGISTER->DATA & (1U << usart->io.cts->pin)) != 0U) {
        modem_status.cts = 1U;
      }
    }
  }
  modem_status.dsr = 0U;
  modem_status.ri  = 0U;
  modem_status.dcd = 0U;

  return modem_status;
}

/**
  \fn          void USART_IRQHandler (UART_RESOURCES *usart)
  \brief       USART global Interrupt handler.
  \param[in]   usart     Pointer to USART resources
*/
static void USART_IRQHandler (USART_RESOURCES *usart) {
  uint32_t uart_mis, uart_ris, uart_dr;
  uint32_t event;

  while (usart->reg->MIS != 0) {
    uart_mis = usart->reg->MIS;
    uart_ris = usart->reg->RIS;

    // Reset local variables
    event = 0U;

    // check for errors
    if ((uart_mis & (UART_IMSC_OEIM_Msk |
                     UART_IMSC_BEIM_Msk |
                     UART_IMSC_PEIM_Msk |
                     UART_IMSC_FEIM_Msk  )) != 0U) {
       // Overrun error interrupt
      if (uart_mis & UART_IMSC_OEIM_Msk ) {
        usart->info->status.rx_overflow      = _FLD2VAL(UART_IMSC_OEIM, uart_mis);
        event |= ARM_USART_EVENT_RX_OVERFLOW;
      }

      // Break error interrupt
      if (uart_mis & UART_IMSC_BEIM_Msk ) {
        usart->info->status.rx_overflow      = _FLD2VAL(UART_IMSC_BEIM, uart_mis);
        event |= ARM_USART_EVENT_RX_BREAK;
      }

      // Parity error interrupt
      if (uart_mis & UART_IMSC_PEIM_Msk ) {
        usart->info->status.rx_parity_error  = _FLD2VAL(UART_IMSC_PEIM, uart_mis);
        event |= ARM_USART_EVENT_RX_PARITY_ERROR;
      }

      // Framing error interrupt
      if (uart_mis & UART_IMSC_FEIM_Msk ) {
        usart->info->status.rx_framing_error = _FLD2VAL(UART_IMSC_FEIM, uart_mis);
        event |= ARM_USART_EVENT_RX_FRAMING_ERROR;
      }
    }

      // Receive interrupt
    if (uart_mis & UART_IMSC_RXIM_Msk ) {
      uart_dr = usart->reg->DR;

      if (usart->xfer->rx_buf != NULL) {
        usart->xfer->rx_buf[usart->xfer->rx_cnt++] = (uint8_t)(uart_dr & UART_DR_DATA_Msk);
      }

      if (usart->xfer->rx_cnt >= usart->xfer->rx_num) {
        // RX is completed
        usart->xfer->rx_num = 0U;
        usart->info->status.rx_busy = 0U;

        // RX interrupt disable
        usart->reg->IMSC &= ~UART_IMSC_RXIM_Msk;

        // Event
        event |= ARM_USART_EVENT_RECEIVE_COMPLETE;
      }
      usart->reg->ICR = UART_IMSC_RXIM_Msk;
    }

    // Transmit interrupt
    if (uart_mis & UART_IMSC_TXIM_Msk ) {
      if (usart->xfer->tx_num > 0) {

        usart->xfer->tx_cnt++;

        if (usart->xfer->tx_cnt < usart->xfer->tx_num) {
          // Send next byte
          usart->reg->DR = usart->xfer->tx_buf[usart->xfer->tx_cnt];
        } else {
          // TX is completed
          usart->xfer->tx_num = 0U;
          usart->info->status.tx_busy = 0U;

          // Set Send active flag
          usart->xfer->send_active = 0U;

          // TX interrupt disable
          usart->reg->IMSC &= ~UART_IMSC_TXIM_Msk;

          // Event
          event |= ARM_USART_EVENT_SEND_COMPLETE;
        }
      }
      usart->reg->ICR = UART_IMSC_TXIM_Msk;
    }

    // Send Event
    if ((event && usart->info->cb_event) != 0U) {
      usart->info->cb_event (event);
    }

    // clear interrupt
    usart->reg->ICR = uart_mis;
  }
}

#if (defined (RTE_USART0) && (RTE_USART0 == 1))
// USART0 Driver Wrapper functions
       void                    UART0_IRQHandler       (void);
static ARM_USART_CAPABILITIES  USART0_GetCapabilities (void)                                                { return USART_GetCapabilities (&USART0_Resources);                         }
static int32_t                 USART0_Initialize      (ARM_USART_SignalEvent_t cb_event)                    { return USART_Initialize      (&USART0_Resources, cb_event);               }
static int32_t                 USART0_Uninitialize    (void)                                                { return USART_Uninitialize    (&USART0_Resources);                         }
static int32_t                 USART0_PowerControl    (ARM_POWER_STATE state)                               { return USART_PowerControl    (&USART0_Resources, state);                  }
static int32_t                 USART0_Send            (const void *data, uint32_t num)                      { return USART_Send            (&USART0_Resources, data, num);              }
static int32_t                 USART0_Receive         (void *data, uint32_t num)                            { return USART_Receive         (&USART0_Resources, data, num);              }
static int32_t                 USART0_Transfer        (const void *data_out, void *data_in, uint32_t num)   { return USART_Transfer        (&USART0_Resources, data_out, data_in, num); }
static uint32_t                USART0_GetTxCount      (void)                                                { return USART_GetTxCount      (&USART0_Resources);                         }
static uint32_t                USART0_GetRxCount      (void)                                                { return USART_GetRxCount      (&USART0_Resources);                         }
static int32_t                 USART0_Control         (uint32_t control, uint32_t arg)                      { return USART_Control         (&USART0_Resources, control, arg);           }
static ARM_USART_STATUS        USART0_GetStatus       (void)                                                { return USART_GetStatus       (&USART0_Resources);                         }
static int32_t                 USART0_SetModemControl (ARM_USART_MODEM_CONTROL control)                     { return USART_SetModemControl (&USART0_Resources, control);                }
static ARM_USART_MODEM_STATUS  USART0_GetModemStatus  (void)                                                { return USART_GetModemStatus  (&USART0_Resources);                         }
       void                    UART0_IRQHandler       (void)                                                {        USART_IRQHandler      (&USART0_Resources);                         }

// USART0 Driver Control Block
extern ARM_DRIVER_USART Driver_USART0;
       ARM_DRIVER_USART Driver_USART0 = {
    USARTx_GetVersion,
    USART0_GetCapabilities,
    USART0_Initialize,
    USART0_Uninitialize,
    USART0_PowerControl,
    USART0_Send,
    USART0_Receive,
    USART0_Transfer,
    USART0_GetTxCount,
    USART0_GetRxCount,
    USART0_Control,
    USART0_GetStatus,
    USART0_SetModemControl,
    USART0_GetModemStatus
};
#endif

#if (defined (RTE_USART1) && (RTE_USART1 == 1))
// USART1 Driver Wrapper functions
       void                    UART1_IRQHandler       (void);
static ARM_USART_CAPABILITIES  USART1_GetCapabilities (void)                                                { return USART_GetCapabilities (&USART1_Resources);                         }
static int32_t                 USART1_Initialize      (ARM_USART_SignalEvent_t cb_event)                    { return USART_Initialize      (&USART1_Resources, cb_event);               }
static int32_t                 USART1_Uninitialize    (void)                                                { return USART_Uninitialize    (&USART1_Resources);                         }
static int32_t                 USART1_PowerControl    (ARM_POWER_STATE state)                               { return USART_PowerControl    (&USART1_Resources, state);                  }
static int32_t                 USART1_Send            (const void *data, uint32_t num)                      { return USART_Send            (&USART1_Resources, data, num);              }
static int32_t                 USART1_Receive         (void *data, uint32_t num)                            { return USART_Receive         (&USART1_Resources, data, num);              }
static int32_t                 USART1_Transfer        (const void *data_out, void *data_in, uint32_t num)   { return USART_Transfer        (&USART1_Resources, data_out, data_in, num); }
static uint32_t                USART1_GetTxCount      (void)                                                { return USART_GetTxCount      (&USART1_Resources);                         }
static uint32_t                USART1_GetRxCount      (void)                                                { return USART_GetRxCount      (&USART1_Resources);                         }
static int32_t                 USART1_Control         (uint32_t control, uint32_t arg)                      { return USART_Control         (&USART1_Resources, control, arg);           }
static ARM_USART_STATUS        USART1_GetStatus       (void)                                                { return USART_GetStatus       (&USART1_Resources);                         }
static int32_t                 USART1_SetModemControl (ARM_USART_MODEM_CONTROL control)                     { return USART_SetModemControl (&USART1_Resources, control);                }
static ARM_USART_MODEM_STATUS  USART1_GetModemStatus  (void)                                                { return USART_GetModemStatus  (&USART1_Resources);                         }
       void                    UART1_IRQHandler       (void)                                                {        USART_IRQHandler      (&USART1_Resources);                         }

// USART1 Driver Control Block
extern ARM_DRIVER_USART Driver_USART1;
       ARM_DRIVER_USART Driver_USART1 = {
    USARTx_GetVersion,
    USART1_GetCapabilities,
    USART1_Initialize,
    USART1_Uninitialize,
    USART1_PowerControl,
    USART1_Send,
    USART1_Receive,
    USART1_Transfer,
    USART1_GetTxCount,
    USART1_GetRxCount,
    USART1_Control,
    USART1_GetStatus,
    USART1_SetModemControl,
    USART1_GetModemStatus
};
#endif
