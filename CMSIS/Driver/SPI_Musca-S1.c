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
 * Driver:       Driver_SPI0
 *
 * Configured:   via RTE_Device.h configuration file
 * Project:      SPI Driver for Musca-S1 device
 * --------------------------------------------------------------------------
 * Use the following configuration settings in the middleware component
 * to connect to this driver.
 *
 *   Configuration Setting                   Value   SPI Interface
 *   ---------------------                   -----   --------------
 *   Connect to hardware via Driver_SPI# =   0       use SPI0
 * --------------------------------------------------------------------------
 * Note(s): possible defines select the used communication interface:
 *            __USE_SECURE  - use register in secure address space
 *                          - use register in non-secure address space (default)
 *----------------------------------------------------------------------------*/

/* History:
 *  Version 1.0.0
 *    Initial release
 */

#include "SPI_Musca-S1.h"

#define UNUSED(x) (void)(x)           /* macro to get rid of 'unused parameter' warning */

#define ARM_SPI_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1,0)

// Driver Version
static const ARM_DRIVER_VERSION spi_driver_version = {ARM_SPI_API_VERSION, ARM_SPI_DRV_VERSION};

#if defined __USE_SECURE
  #define SCC_REGISTER   SECURE_SCC
  #define GPIO_REGISTER  SECURE_GPIO0
#else
  #define SCC_REGISTER   SCC
  #define GPIO_REGISTER  GPIO0
#endif


// SPI0
#if (defined (RTE_SPI0) && (RTE_SPI0 != 0))

#if defined __USE_SECURE
  #define SPI0_REGISTER  SECURE_SPI0
#else
  #define SPI0_REGISTER  SPI0
#endif

// SPI0 Run-Time Information
static SPI_INFO SPI0_Info = {
  .cb_event = 0U,
  .status   = {
    .busy       = 0U,
    .data_lost  = 0U,
    .mode_fault = 0U,
    .padding    = {0U}
  },
  .flags    = 0U,
  .mode     = 0U
};

static SPI_TRANSFER_INFO SPI0_TransferInfo = {
  .num         = 0U,
  .rx_buf      = 0U,
  .tx_buf      = 0U,
  .rx_cnt      = 0U,
  .tx_cnt      = 0U,
};

#if defined (RTE_SPI0_MOSI) && (RTE_SPI0_MOSI != 0U)
static SPI_PIN SPI0_pin_mosi = {
  .pin = 11U,     // PA11
  .af  =  1U      // Alternate Function 1
};
#endif

#if defined (RTE_SPI0_MISO) && (RTE_SPI0_MISO != 0U)
static SPI_PIN SPI0_pin_miso = {
  .pin = 12U,     // PA12
  .af  =  1U      // Alternate Function 1
};
#endif

#if defined (RTE_SPI0_SCK) && (RTE_SPI0_SCK != 0U)
static SPI_PIN SPI0_pin_sck = {
  .pin = 13U,     // PA13
  .af  =  1U      // Alternate Function 1
};
#endif

#if defined (RTE_SPI0_NSS) && (RTE_SPI0_NSS != 0U)
static SPI_PIN SPI0_pin_nss = {
  .pin = 10U,     // PA10
  .af  =  1U      // Alternate Function 1
};
#endif

// SPI0 Resources
static const SPI_RESOURCES SPI0_Resources = {
  {      // Capabilities
    1U,  // Simplex Mode (Master and Slave)
    0U,  // TI Synchronous Serial Interface
    0U,  // Microwire Interface
    1U,  // Signal Mode Fault event: \ref ARM_SPI_EVENT_MODE_FAULT
    0U   // Reserved (must be zero)
  },
  .io = {
#if defined (RTE_SPI0_MOSI) && (RTE_SPI0_MOSI != 0U)
    .mosi  = &SPI0_pin_mosi,
#else
    NULL,
#endif
#if defined (RTE_SPI0_MISO) && (RTE_SPI0_MISO != 0U)
    .miso  = &SPI0_pin_miso,
#else
    NULL,
#endif
#if defined (RTE_SPI0_SCK) && (RTE_SPI0_SCK != 0U)
    .sck  = &SPI0_pin_sck,
#else
    NULL,
#endif
#if defined (RTE_SPI0_NSS) && (RTE_SPI0_NSS != 0U)
    .nss = &SPI0_pin_nss,
#else
    NULL,
#endif
  },
  .reg            =  SPI0_REGISTER,
  .info           = &SPI0_Info,
  .xfer           = &SPI0_TransferInfo,
  .irq_num        =  SPI_IRQn,
  .reset_bit      =  5U,
  .padding        =  {0U, 0U}
};
#endif


/**
  \fn          void SPI_PinInit (uint16_t pin, uint16_t af)
  \brief       Initialize SPI pin
*/
static void SPI_PinInit (uint16_t pin, uint16_t af) {

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
  \fn          void SPI_PinDeInit (uint16_t pin)
  \brief       DeInitialize SPI pin
*/
static void SPI_PinDeInit (uint16_t pin) {

  SCC_REGISTER->IOMUX_MAIN_INSEL   |=  (1U << pin);
  SCC_REGISTER->IOMUX_MAIN_OUTSEL  |=  (1U << pin);
  SCC_REGISTER->IOMUX_MAIN_OENSEL  |=  (1U << pin);
}

// SPI Driver functions

/**
  \fn          ARM_DRIVER_VERSION SPIx_GetVersion (void)
  \brief       Get driver version.
  \return      \ref ARM_DRIVER_VERSION
*/
static ARM_DRIVER_VERSION SPIx_GetVersion (void) {
  return spi_driver_version;
}

/**
  \fn          ARM_SPI_CAPABILITIES SPI_GetCapabilities (void)
  \brief       Get driver capabilities
  \param[in]   spi       Pointer to SPI resources
  \return      \ref ARM_SPI_CAPABILITIES
*/
static ARM_SPI_CAPABILITIES SPI_GetCapabilities (const SPI_RESOURCES *spi) {
  return spi->capabilities;
}

/**
  \fn          int32_t SPI_Initialize (const SPI_RESOURCES         *spi,
                                             ARM_SPI_SignalEvent_t  cb_event)
  \brief       Initialize SPI Interface.
  \param[in]   spi       Pointer to SPI resources
  \param[in]   cb_event  Pointer to \ref ARM_SPI_SignalEvent
  \return      \ref execution_status
*/
static int32_t SPI_Initialize (const SPI_RESOURCES         *spi,
                                     ARM_SPI_SignalEvent_t  cb_event) {

  if (spi->info->flags & SPI_FLAG_INITIALIZED) {
    // Driver is already initialized
    return ARM_DRIVER_OK;
  }

  // Initialize Run-time Resources
  spi->info->cb_event = cb_event;

  // Clear Status flags
  spi->info->status.busy       = 0U;
  spi->info->status.data_lost  = 0U;
  spi->info->status.mode_fault = 0U;

  spi->info->mode        = 0U;

  // Clear transfer information
  memset((void *)spi->xfer, 0, sizeof(SPI_TRANSFER_INFO));

  // configure MOSI pin
  if (spi->io.mosi) {
    SPI_PinInit (spi->io.mosi->pin, spi->io.mosi->af);
  }

  // configure MISO pin
  if (spi->io.miso) {
    SPI_PinInit (spi->io.miso->pin, spi->io.miso->af);
  }

  // configure SCK pin
  if (spi->io.sck) {
    SPI_PinInit (spi->io.sck->pin, spi->io.sck->af);
  }

  // configure NSS pin
  if (spi->io.nss) {
    SPI_PinInit (spi->io.nss->pin, spi->io.nss->af);
  }

  spi->info->flags = SPI_FLAG_INITIALIZED;

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t SPI_Uninitialize (const SPI_RESOURCES *spi)
  \brief       De-initialize SPI Interface.
  \param[in]   spi       Pointer to SPI resources
  \return      \ref execution_status
*/
static int32_t SPI_Uninitialize (const SPI_RESOURCES *spi) {

  // unConfigure MOSI pin
  if (spi->io.mosi) {
    SPI_PinDeInit (spi->io.mosi->pin);
  }

  // unConfigure MISO pin
  if (spi->io.miso) {
    SPI_PinDeInit (spi->io.miso->pin);
  }

  // unConfigure SCK pin
  if (spi->io.sck) {
    SPI_PinDeInit (spi->io.sck->pin);
  }

  // unConfigure NSS pin
  if (spi->io.nss) {
    SPI_PinDeInit (spi->io.nss->pin);
  }

  // reset status flags
  spi->info->flags = 0U;

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t SPI_PowerControl (const SPI_RESOURCES   *spi,
                                               ARM_POWER_STATE  state)
  \brief       Control SPI Interface Power.
  \param[in]   spi       Pointer to SPI resources
  \param[in]   state     Power state
  \return      \ref execution_status
*/
static int32_t SPI_PowerControl (const SPI_RESOURCES   *spi,
                                       ARM_POWER_STATE  state) {

  if ((state != ARM_POWER_OFF)  &&
      (state != ARM_POWER_FULL) &&
      (state != ARM_POWER_LOW)) {
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  switch (state) {
    case ARM_POWER_OFF:
      // clear and disable SPI IRQ
      NVIC_DisableIRQ(spi->irq_num);
      NVIC_ClearPendingIRQ(spi->irq_num);

      // reset SPI configuration
      spi->reg->CR  = 0U;

      // Disableable SPI
      spi->reg->ENR = 0U;

      // hold SPI in Reset
      SCC_REGISTER->RESET_CTRL &= ~(1U << spi->reset_bit);

      // Clear Status flags
      spi->info->status.busy       = 0U;
      spi->info->status.data_lost  = 0U;
      spi->info->status.mode_fault = 0U;

      // Clear powered flag
      spi->info->flags &= ~SPI_FLAG_POWERED;
      break;

    case ARM_POWER_LOW:
      return ARM_DRIVER_ERROR_UNSUPPORTED;

    case ARM_POWER_FULL:
      if ((spi->info->flags & SPI_FLAG_INITIALIZED) == 0U) {
        return ARM_DRIVER_ERROR;
      }
      if ((spi->info->flags & SPI_FLAG_POWERED)     != 0U) {
        return ARM_DRIVER_OK;
      }

      // clear Status flags
      spi->info->status.busy       = 0U;
      spi->info->status.data_lost  = 0U;
      spi->info->status.mode_fault = 0U;

      spi->xfer->def_val           = 0U;

      // set flag initialized
      spi->info->flags = SPI_FLAG_POWERED | SPI_FLAG_INITIALIZED;

      // clear and enable SPI IRQ
      NVIC_ClearPendingIRQ(spi->irq_num);
      NVIC_EnableIRQ(spi->irq_num);

      // release SPI from reset
      SCC_REGISTER->RESET_CTRL |= (1U << spi->reset_bit);
      while ((SCC_REGISTER->RESET_CTRL & (1U << spi->reset_bit)) == 0U);

      // reset SPI configuration
      spi->reg->CR = 0U;
      break;
  }

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t SPI_Send (const SPI_RESOURCES *spi,
                                 const void          *data,
                                       uint32_t       num)
  \brief       Start sending data to SPI transmitter.
  \param[in]   spi       Pointer to SPI resources
  \param[in]   data      Pointer to buffer with data to send to SPI transmitter
  \param[in]   num       Number of data items to send
  \return      \ref execution_status
*/
static int32_t SPI_Send (const SPI_RESOURCES *spi,
                         const void          *data,
                               uint32_t       num) {

  if ((data == NULL) || (num == 0U))                          { return ARM_DRIVER_ERROR_PARAMETER; }
  if ((spi->info->flags & SPI_FLAG_CONFIGURED) == 0U)         { return ARM_DRIVER_ERROR; }
  if ( spi->info->status.busy)                                { return ARM_DRIVER_ERROR_BUSY; }

  // Check if transmit pin available
  if ((((spi->io.mosi != NULL) && ((spi->info->mode & ARM_SPI_CONTROL_Msk) == ARM_SPI_MODE_MASTER)) ||
       ((spi->io.miso != NULL) && ((spi->info->mode & ARM_SPI_CONTROL_Msk) == ARM_SPI_MODE_SLAVE ))) == 0U) {
    return ARM_DRIVER_ERROR;
  }

  // Update SPI statuses
  spi->info->status.busy       = 1U;
  spi->info->status.data_lost  = 0U;
  spi->info->status.mode_fault = 0U;

  // Save transfer info
  spi->xfer->rx_buf = NULL;
  spi->xfer->tx_buf = data;
  spi->xfer->num    = num;
  spi->xfer->rx_cnt = 0U;
  spi->xfer->tx_cnt = 0U;

  // Enable RX FIFO Not Empty, TX FIFO Not Full interrupt
  spi->reg->IER = (SPI_IR_RNE_Msk |
                   SPI_IR_TNF_Msk  );

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t SPI_Receive (const SPI_RESOURCES *spi,
                                          void          *data,
                                          uint32_t       num)
  \brief       Start receiving data from SPI receiver.
  \param[in]   spi   Pointer to SPI resources
  \param[out]  data  Pointer to buffer for data to receive from SPI receiver
  \param[in]   num   Number of data items to receive
  \return      \ref execution_status
*/
static int32_t SPI_Receive (const SPI_RESOURCES *spi,
                                  void          *data,
                                  uint32_t       num) {

  if ((data == NULL) || (num == 0U))                          { return ARM_DRIVER_ERROR_PARAMETER; }
  if ((spi->info->flags & SPI_FLAG_CONFIGURED) == 0U)         { return ARM_DRIVER_ERROR; }
  if ( spi->info->status.busy)                                { return ARM_DRIVER_ERROR_BUSY; }

  // Update SPI statuses
  spi->info->status.busy       = 1U;
  spi->info->status.data_lost  = 0U;
  spi->info->status.mode_fault = 0U;

  // Save transfer info
  spi->xfer->rx_buf = (uint8_t *)data;
  spi->xfer->tx_buf = NULL;
  spi->xfer->num    = num;
  spi->xfer->rx_cnt = 0U;
  spi->xfer->tx_cnt = 0U;

  // Enable RX FIFO Not Empty, TX FIFO Not Full interrupt
  spi->reg->IER = (SPI_IR_RNE_Msk |
                   SPI_IR_TNF_Msk  );

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t SPI_Transfer (const SPI_RESOURCES *spi,
                                     const void          *data_out,
                                           void          *data_in,
                                           uint32_t       num)
  \brief       Start sending/receiving data to/from SPI transmitter/receiver.
  \param[in]   spi       Pointer to SPI resources
  \param[in]   data_out  Pointer to buffer with data to send to SPI transmitter
  \param[out]  data_in   Pointer to buffer for data to receive from SPI receiver
  \param[in]   num       Number of data items to transfer
  \return      \ref execution_status
*/
static int32_t SPI_Transfer (const SPI_RESOURCES *spi,
                             const void          *data_out,
                                   void          *data_in,
                                   uint32_t       num) {

  if ((data_out == NULL) || (data_in == NULL) || (num == 0U)) { return ARM_DRIVER_ERROR_PARAMETER; }
  if ((spi->info->flags & SPI_FLAG_CONFIGURED) == 0U)         { return ARM_DRIVER_ERROR; }
  if ( spi->info->status.busy)                                { return ARM_DRIVER_ERROR_BUSY; }

  // Update SPI statuses
  spi->info->status.busy       = 1U;
  spi->info->status.data_lost  = 0U;
  spi->info->status.mode_fault = 0U;

  // Save transfer info
  spi->xfer->rx_buf = (uint8_t *)((uint32_t)data_in);
  spi->xfer->tx_buf = data_out;
  spi->xfer->num    = num;
  spi->xfer->rx_cnt = 0U;
  spi->xfer->tx_cnt = 0U;

  // Enable RX FIFO Not Empty, TX FIFO Not Full interrupt
  spi->reg->IER = (SPI_IR_RNE_Msk |
                   SPI_IR_TNF_Msk  );

  return ARM_DRIVER_OK;
}

/**
  \fn          uint32_t SPI_GetDataCount (const SPI_RESOURCES *spi)
  \brief       Get transferred data count.
  \param[in]   spi  Pointer to SPI resources
  \return      number of data items transferred
*/
static uint32_t SPI_GetDataCount (const SPI_RESOURCES *spi) {
  return (spi->xfer->rx_cnt);
}

/**
  \fn          int32_t SPI_Control (const SPI_RESOURCES *spi,
                                          uint32_t       control,
                                          uint32_t       arg)
  \brief       Control SPI Interface.
  \param[in]   spi      Pointer to SPI resources
  \param[in]   control  operation
  \param[in]   arg      argument of operation (optional)
  \return      \ref execution_status
*/
static int32_t SPI_Control (const SPI_RESOURCES *spi, uint32_t control, uint32_t arg) {
  uint32_t mode, val, pclk;
  uint32_t spi_cr;

  // Reset local variables
    mode = 0U;
  spi_cr = 0U;

  if ((spi->info->flags & SPI_FLAG_POWERED) == 0U) { return ARM_DRIVER_ERROR; }

  if ((control & ARM_SPI_CONTROL_Msk) == ARM_SPI_ABORT_TRANSFER) {
    // Disable TX FIFO Not Full, RX FIFO Not Empty interrupt
    spi->reg->IDR = (SPI_IR_RNE_Msk |
                     SPI_IR_TNF_Msk  );

    /* Clear RX, TX FIFO */
    spi->reg->CR |= (SPI_CR_TXCLR_Msk |          // TX FIFO Clear
                     SPI_CR_RXCLR_Msk  );        // RX FIFO Clear

    // Clear Transfer info
    spi->xfer->num    = 0U;
    spi->xfer->rx_cnt = 0U;
    spi->xfer->tx_cnt = 0U;

    spi->info->status.busy = 0U;

    return ARM_DRIVER_OK;
  }

  // Check for busy flag
  if (spi->info->status.busy) { return ARM_DRIVER_ERROR_BUSY; }

  switch (control & ARM_SPI_CONTROL_Msk) {
    case ARM_SPI_MODE_INACTIVE:                 // SPI Inactive
      mode |= ARM_SPI_MODE_INACTIVE;
      break;

    case ARM_SPI_MODE_MASTER:                   // SPI Master (Output on MOSI, Input on MISO); arg = Bus Speed in bps
      mode |= ARM_SPI_MODE_MASTER;

      // Master enabled
      spi_cr |= SPI_CR_MSEL_Msk;
      break;

    case ARM_SPI_MODE_SLAVE:                     // SPI Slave  (Output on MISO, Input on MOSI)
    case ARM_SPI_MODE_MASTER_SIMPLEX:            // SPI Master (Output/Input on MOSI); arg = Bus Speed in bps
    case ARM_SPI_MODE_SLAVE_SIMPLEX:             // SPI Slave  (Output/Input on MISO)
      return ARM_SPI_ERROR_MODE;

    case ARM_SPI_SET_BUS_SPEED:                  // Set Bus Speed in bps; arg = value
      // Set SPI Bus Speed
      pclk = SystemCoreClock;
      for (val = 0U; val < 8U; val++) {
        if (arg >= (pclk >> (val + 1U))) break;
      }
      if ((val == 8U) || (arg < (pclk >> (val + 1U)))) {
        // Requested Bus Speed can not be configured
        return ARM_DRIVER_ERROR;
      }
      // Disable SPI, update prescaler and enable SPI
      spi->reg->ENR = ~SPI_ENR_SPIE_Msk;
      spi->reg->CR  =  (spi->reg->CR & ~(SPI_CR_MBRD_Msk)) | (val << SPI_CR_MBRD_Pos);
      spi->reg->ENR =  SPI_ENR_SPIE_Msk;

      return ARM_DRIVER_OK;

    case ARM_SPI_GET_BUS_SPEED:                  // Get Bus Speed in bps
      // Return current bus speed
      return (int32_t)(SystemCoreClock >> (_FLD2VAL(SPI_CR_MBRD, spi->reg->CR) + 1U));

    case ARM_SPI_SET_DEFAULT_TX_VALUE:           // Set default Transmit value; arg = value
      spi->xfer->def_val = (uint16_t)(arg & 0xFFFFU);
      return ARM_DRIVER_OK;

    case ARM_SPI_CONTROL_SS:                     // Control Slave Select; arg = 0:inactive, 1:active
      if (((spi->info->mode & ARM_SPI_CONTROL_Msk)        != ARM_SPI_MODE_MASTER)  ||
          ((spi->info->mode & ARM_SPI_SS_MASTER_MODE_Msk) != ARM_SPI_SS_MASTER_SW)) {
        return ARM_DRIVER_ERROR;
      }
      if (spi->io.nss == NULL) {
        return ARM_DRIVER_ERROR;
      }
      if (arg == ARM_SPI_SS_INACTIVE) {
        GPIO_REGISTER->DATAOUT |=  (1U << spi->io.nss->pin);
      } else {
        GPIO_REGISTER->DATAOUT &= ~(1U << spi->io.nss->pin);
      }
      return ARM_DRIVER_OK;

    default:
      return ARM_DRIVER_ERROR_UNSUPPORTED;
  }

  // Frame format:
  switch (control & ARM_SPI_FRAME_FORMAT_Msk) {
    case ARM_SPI_CPOL0_CPHA0:
      break;

    case ARM_SPI_CPOL0_CPHA1:
      spi_cr |=  SPI_CR_CPHA_Msk;
      break;

    case ARM_SPI_CPOL1_CPHA0:
      spi_cr |=  SPI_CR_CPOL_Msk;
      break;

    case ARM_SPI_CPOL1_CPHA1:
      spi_cr |= (SPI_CR_CPHA_Msk |
                 SPI_CR_CPOL_Msk  );
      break;

    case ARM_SPI_TI_SSI:
    case ARM_SPI_MICROWIRE:
      return ARM_SPI_ERROR_FRAME_FORMAT;

    default: return ARM_SPI_ERROR_FRAME_FORMAT;
  }

  // Data Bits
  val = (control & ARM_SPI_DATA_BITS_Msk) >> ARM_SPI_DATA_BITS_Pos;
    switch (val) {
    case  8U:
      spi_cr |= _VAL2FLD(SPI_CR_TWS, 0U);
      break;

//  case 16U:
//        spi_cr |= _VAL2FLD(SPI_CR_TWS, 1U);
//    break;
//
//  case 24U:
//        spi_cr |= _VAL2FLD(SPI_CR_TWS, 2U);
//    break;
//
//  case 32U:
//        spi_cr |= _VAL2FLD(SPI_CR_TWS, 3U);
//    break;

    default:
            return ARM_SPI_ERROR_DATA_BITS;
    }

  // Bit order
  if ((control & ARM_SPI_BIT_ORDER_Msk) == ARM_SPI_LSB_MSB) {
    return ARM_SPI_ERROR_BIT_ORDER;
  }

  // Slave select master modes
  if (mode == ARM_SPI_MODE_MASTER) {
    switch (control & ARM_SPI_SS_MASTER_MODE_Msk) {
      case ARM_SPI_SS_MASTER_UNUSED:             // SPI Slave Select when Master: Not used (default)
        if (spi->io.nss != NULL) {
          // Unconfigure NSS pin
          SPI_PinDeInit (spi->io.nss->pin);
        }

        mode |= ARM_SPI_SS_MASTER_UNUSED;
        break;

      case ARM_SPI_SS_MASTER_SW:                 // SPI Slave Select when Master: Software controlled
        if (spi->io.nss != NULL) {
          // Configure NSS pin as GPIO output
          SPI_PinDeInit (spi->io.nss->pin);
          GPIO_REGISTER->OUTENSET = (1U << spi->io.nss->pin); /* enable Output */
          

          mode |= ARM_SPI_SS_MASTER_SW;
        } else {
          // NSS pin is not available
          return ARM_SPI_ERROR_SS_MODE;
        }
        break;

      case ARM_SPI_SS_MASTER_HW_OUTPUT:          // SPI Slave Select when Master: Hardware controlled Output
        if (spi->io.nss) {
          // Configure NSS pin - SPI NSS alternative function
          SPI_PinInit (spi->io.nss->pin, spi->io.nss->af);

          // Configure Peripheral Chip Select Lines (master mode only)
//        spi_cr |= 0;

          mode |= ARM_SPI_SS_MASTER_HW_OUTPUT;
        } else {
          // NSS pin is not available
          return ARM_SPI_ERROR_SS_MODE;
        }
        break;

      case ARM_SPI_SS_MASTER_HW_INPUT:           // SPI Slave Select when Master: Hardware monitored Input
      default:
        return ARM_SPI_ERROR_SS_MODE;
    }
  }

  // Slave select slave modes
  if (mode ==  ARM_SPI_MODE_SLAVE) {
    switch (control & ARM_SPI_SS_SLAVE_MODE_Msk) {
      case ARM_SPI_SS_SLAVE_HW:                  // SPI Slave Select when Slave: Hardware monitored (default)
      case ARM_SPI_SS_SLAVE_SW:                  // SPI Slave Select when Slave: Software controlled
      default:
        return ARM_SPI_ERROR_SS_MODE;
    }
  }

  // Set SPI Bus Speed
  pclk = SystemCoreClock;
  for (val = 0U; val < 8U; val++) {
    if (arg >= (pclk >> (val + 1U))) break;
  }
  if ((val == 8U) || (arg < (pclk >> (val + 1U)))) {
    // Requested Bus Speed can not be configured
    return ARM_DRIVER_ERROR;
  }
  // Save prescaler value
  spi_cr |= _VAL2FLD(SPI_CR_MBRD, val);

  spi->info->mode = mode;

  // Configure registers
  spi->reg->CR  = spi_cr;
  spi->reg->IER = (SPI_IR_TUF_Msk |              // enable TX FIFO Underflow interrupt
                   SPI_IR_MF_Msk  |              // enable Mode Fail interrupt
                   SPI_IR_ROF_Msk  );            // enable RX FIFO Overflow interrupt


  if ((mode & ARM_SPI_CONTROL_Msk) == ARM_SPI_MODE_INACTIVE) {
    spi->info->flags &= ~((uint32_t)SPI_FLAG_CONFIGURED);
  } else {
    spi->info->flags |=  SPI_FLAG_CONFIGURED;
  }

  // Enable SPI
  spi->reg->ENR = SPI_ENR_SPIE_Msk;

  return ARM_DRIVER_OK;
}

/**
  \fn          ARM_SPI_STATUS SPI_GetStatus (const SPI_RESOURCES *spi)
  \brief       Get SPI status.
  \param[in]   spi       Pointer to SPI resources
  \return      SPI status \ref ARM_SPI_STATUS
*/
static ARM_SPI_STATUS SPI_GetStatus (const SPI_RESOURCES *spi) {
  ARM_SPI_STATUS status;

  status.busy       = spi->info->status.busy;
  status.data_lost  = spi->info->status.data_lost;
  status.mode_fault = spi->info->status.mode_fault;

  return status;
}

/**
  \fn          void SPIx_IRQHandler (SPI_RESOURCES *spi)
  \brief       SPI global Interrupt handler.
  \param[in]   spi       Pointer to SPI resources
*/
static void SPIx_IRQHandler (SPI_RESOURCES *spi) {
  uint32_t spi_isr;
  uint32_t spi_cr;
  uint32_t event;
  uint32_t data_32bit;
  uint16_t data_16bit;
  uint8_t  data_8bit;

  spi_isr = spi->reg->ISR;
  spi_cr  = spi->reg->CR;

  // Reset local variables
  event = 0U;

  // check for errors
  if ((spi_isr & SPI_IR_TUF_Msk) != 0U) {
    spi->info->status.data_lost = 1U;

    event |=ARM_SPI_EVENT_DATA_LOST;
  }

  if ((spi_isr & SPI_IR_MF_Msk) != 0U) {
    spi->info->status.mode_fault = 1U;

    event |= ARM_SPI_EVENT_MODE_FAULT;
  }

  if ((spi_isr & SPI_IR_ROF_Msk) != 0U) {
    // in case of Overrun only read the data
    switch (_FLD2VAL(SPI_CR_TWS, spi_cr)) {
      case 0U:   //  8 bit data size
        data_8bit = *(volatile uint8_t *)(&spi->reg->RDR);
        break;
    
      case 1U:   // 16 bit data size
        data_16bit = *(volatile uint16_t *)(&spi->reg->RDR);
        break;
    
      case 2U:   // 24 bit data size
      case 3U:   // 32 bit data size
        data_32bit = *(volatile uint32_t *)(&spi->reg->RDR);
        break;
    }

    spi->info->status.data_lost = 1U;

    event |=ARM_SPI_EVENT_DATA_LOST;
  }

  // Receive interrupt (RX FIFO not empty)
  if ((spi_isr & SPI_IR_RNE_Msk) != 0U) {
    // Receive Buffer Not Empty
    if (spi->xfer->rx_cnt < spi->xfer->num) {
      switch (_FLD2VAL(SPI_CR_TWS, spi_cr)) {
        case 0U:   //  8 bit data size
          data_8bit = *(volatile uint8_t *)(&spi->reg->RDR);
          if (spi->xfer->rx_buf != NULL) {
            *(spi->xfer->rx_buf++) = data_8bit;
          }
          break;
  
        case 1U:   // 16 bit data size
          data_16bit = *(volatile uint16_t *)(&spi->reg->RDR);
          if (spi->xfer->rx_buf != NULL) {
            *(spi->xfer->rx_buf++) = (uint8_t) data_16bit;
            *(spi->xfer->rx_buf++) = (uint8_t)(data_16bit >> 8U);
          }
          break;
  
        case 2U:   // 24 bit data size
        case 3U:   // 32 bit data size
          data_32bit = *(volatile uint32_t *)(&spi->reg->RDR);
          if (spi->xfer->rx_buf != NULL) {
            *(spi->xfer->rx_buf++) = (uint8_t) data_32bit;
            *(spi->xfer->rx_buf++) = (uint8_t)(data_32bit >>  8U);
            *(spi->xfer->rx_buf++) = (uint8_t)(data_32bit >> 16U);
            *(spi->xfer->rx_buf++) = (uint8_t)(data_32bit >> 24U);
          }
          break;
      }

      spi->xfer->rx_cnt++;

      if (spi->xfer->rx_cnt == spi->xfer->num) {
        // Disable RX FIFO Not Empty Interrupt
        spi->reg->IDR = SPI_IR_RNE_Msk;

        // Clear busy flag
        spi->info->status.busy = 0U;

        // Transfer completed
        event |= ARM_SPI_EVENT_TRANSFER_COMPLETE;
      }
    }
    else {
      // Unexpected transfer, data lost
      event |= ARM_SPI_EVENT_DATA_LOST;
    }
  }

  // Transmit interrupt (TX FIFO not full)
  if ((spi_isr & SPI_IR_TNF_Msk) != 0U) {
    if (spi->xfer->tx_cnt < spi->xfer->num) {
        switch (_FLD2VAL(SPI_CR_TWS, spi_cr)) {
          case 0U:   //  8 bit data size
            if (spi->xfer->tx_buf != NULL) {
              data_8bit = *(spi->xfer->tx_buf++);
            } else {
              data_8bit = (uint8_t)spi->xfer->def_val;
            }
            // Write data to data register
            *(volatile uint8_t *)(&spi->reg->TDR) = data_8bit;
            break;

          case 1U:   // 16 bit data size
            if (spi->xfer->tx_buf != NULL) {
              data_16bit  = (uint16_t)*(spi->xfer->tx_buf++);
              data_16bit |= (uint16_t)*(spi->xfer->tx_buf++) << 8U;
            } else {
              data_16bit  = (uint16_t)spi->xfer->def_val;
            }
            // Write data to data register
            *(volatile uint16_t *)(&spi->reg->TDR) = data_16bit;
            break;

          case 2U:   // 24 bit data size
          case 3U:   // 32 bit data size
            if (spi->xfer->tx_buf != NULL) {
              data_32bit  = (uint32_t)*(spi->xfer->tx_buf++);
              data_32bit |= (uint32_t)*(spi->xfer->tx_buf++) << 8U;
              data_32bit |= (uint32_t)*(spi->xfer->tx_buf++) << 16U;
              data_32bit |= (uint32_t)*(spi->xfer->tx_buf++) << 24U;
            } else {
              data_32bit  = (uint32_t)spi->xfer->def_val;
            }
            // Write data to data register
            *(volatile uint32_t *)(&spi->reg->TDR) = data_32bit;
            break;
        }

      spi->xfer->tx_cnt++;

      if (spi->xfer->tx_cnt == spi->xfer->num) {
        // All data sent, disable TX Buffer Not Full Interrupt
        spi->reg->IDR = SPI_IR_TNF_Msk;
      }
    } else {
      // Unexpected transfer, data lost
      event |= ARM_SPI_EVENT_DATA_LOST;
    }
  }


  // Send Event
  if ((event && spi->info->cb_event) != 0U) {
    spi->info->cb_event (event);
  }

}

#if (defined (RTE_SPI0) && (RTE_SPI0 == 1))
/* SPI0 Driver Wrapper functions */
       void                  SPI_IRQHandler      (void);
static ARM_SPI_CAPABILITIES  SPI0_GetCapabilities (void)                                              { return SPI_GetCapabilities (&SPI0_Resources);                          }
static int32_t               SPI0_Initialize      (ARM_SPI_SignalEvent_t pSignalEvent)                { return SPI_Initialize      (&SPI0_Resources, pSignalEvent);           }
static int32_t               SPI0_Uninitialize    (void)                                              { return SPI_Uninitialize    (&SPI0_Resources);                         }
static int32_t               SPI0_PowerControl    (ARM_POWER_STATE state)                             { return SPI_PowerControl    (&SPI0_Resources, state);                  }
static int32_t               SPI0_Send            (const void *data, uint32_t num)                    { return SPI_Send            (&SPI0_Resources, data, num);              }
static int32_t               SPI0_Receive         (void *data, uint32_t num)                          { return SPI_Receive         (&SPI0_Resources, data, num);              }
static int32_t               SPI0_Transfer        (const void *data_out, void *data_in, uint32_t num) { return SPI_Transfer        (&SPI0_Resources, data_out, data_in, num); }
static uint32_t              SPI0_GetDataCount    (void)                                              { return SPI_GetDataCount    (&SPI0_Resources);                         }
static int32_t               SPI0_Control         (uint32_t control, uint32_t arg)                    { return SPI_Control         (&SPI0_Resources, control, arg);           }
static ARM_SPI_STATUS        SPI0_GetStatus       (void)                                              { return SPI_GetStatus       (&SPI0_Resources);                         }
       void                  SPI_IRQHandler      (void)                                               {        SPIx_IRQHandler     (&SPI0_Resources);                         }

// SPI0 Driver Control Block */
extern ARM_DRIVER_SPI Driver_SPI0;
       ARM_DRIVER_SPI Driver_SPI0 = {
         SPIx_GetVersion,
         SPI0_GetCapabilities,
         SPI0_Initialize,
         SPI0_Uninitialize,
         SPI0_PowerControl,
         SPI0_Send,
         SPI0_Receive,
         SPI0_Transfer,
         SPI0_GetDataCount,
         SPI0_Control,
         SPI0_GetStatus
       };
#endif
