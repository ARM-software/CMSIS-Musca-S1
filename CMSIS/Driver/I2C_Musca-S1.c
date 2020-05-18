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
 * Driver:       Driver_I2C0, Driver_I2C1
 *
 * Configured:   via RTE_Device.h configuration file
 * Project:      I2C Driver for Musca-S1 device
 * --------------------------------------------------------------------------
 * Use the following configuration settings in the middleware component
 * to connect to this driver.
 *
 *   Configuration Setting                   Value   I2C Interface
 *   ---------------------                   -----   --------------
 *   Connect to hardware via Driver_I2C# =   0       use I2C0
 *   Connect to hardware via Driver_I2C# =   1       use I2C1
 * --------------------------------------------------------------------------
 * Note(s): possible defines select the used communication interface:
 *            __USE_SECURE  - use register in secure address space
 *                          - use register in non-secure address space (default)
 *----------------------------------------------------------------------------*/

/* History:
 *  Version 1.0.0
 *    Initial release
 */

#include "I2C_Musca-S1.h"

#define  I2C_IxR_ALL_Msk     (I2C_IxR_ARB_LOST_Msk | \
                              I2C_IxR_RX_UNF_Msk   | \
                              I2C_IxR_TX_OVF_Msk   | \
                              I2C_IxR_RX_OVF_Msk   | \
                              I2C_IxR_SLV_RDY_Msk  | \
                              I2C_IxR_TO_Msk       | \
                              I2C_IxR_NACK_Msk     | \
                              I2C_IxR_DATA_Msk     | \
                              I2C_IxR_COMP_Msk      )

#define I2C_IxR_ERR_Msk      (I2C_IxR_ARB_LOST_Msk | \
                              I2C_IxR_RX_UNF_Msk   | \
                              I2C_IxR_TX_OVF_Msk   | \
                              I2C_IxR_RX_OVF_Msk   | \
                              I2C_IxR_NACK_Msk      )

#define I2C_IxR_MASTER_Msk   (I2C_IxR_ARB_LOST_Msk | \
                              I2C_IxR_RX_UNF_Msk   | \
                              I2C_IxR_TX_OVF_Msk   | \
                              I2C_IxR_RX_OVF_Msk   | \
                              I2C_IxR_NACK_Msk     | \
                              I2C_IxR_DATA_Msk     | \
                              I2C_IxR_COMP_Msk      )

#define I2C_IxR_SLAVE_Msk    (I2C_IxR_RX_UNF_Msk   | \
                              I2C_IxR_TX_OVF_Msk   | \
                              I2C_IxR_RX_OVF_Msk   | \
                              I2C_IxR_TO_Msk       | \
                              I2C_IxR_NACK_Msk     | \
                              I2C_IxR_DATA_Msk     | \
                              I2C_IxR_COMP_Msk      )


#define I2C_FIFO_DEPTH		      8U                          /* I2C FIFO depth */
#define I2C_DATA_INTR_DEPTH  (I2C_FIFO_DEPTH - 2U)          /* FIFO depth at which the DATA interrupt occurs */
#define I2C_MAX_XFER_SIZE      15U                          /* maximum transfer size */
#define I2C_XFER_SIZE        (I2C_MAX_XFER_SIZE - 3U)       /* Transfer size in multiples of data interrupt depth */


#define UNUSED(x) (void)(x)           /* macro to get rid of 'unused parameter' warning */

#define ARM_I2C_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1,0)

// Driver Version
static const ARM_DRIVER_VERSION i2c_driver_version = {ARM_I2C_API_VERSION, ARM_I2C_DRV_VERSION};

#if defined __USE_SECURE
  #define SCC_REGISTER   SECURE_SCC
  #define GPIO_REGISTER  SECURE_GPIO0
#else
  #define SCC_REGISTER   SCC
  #define GPIO_REGISTER  GPIO0
#endif


// I2C0
#if (defined (RTE_I2C0) && (RTE_I2C0 != 0))

#if defined __USE_SECURE
  #define I2C0_REGISTER  SECURE_I2C0
#else
  #define I2C0_REGISTER  I2C0
#endif

// I2C0 Run-Time Information
static I2C_INFO I2C0_Info = {
  .cb_event = 0U,
  .status   = {
    .busy             = 0U,
    .mode             = 0U,
    .direction        = 0U,
    .general_call     = 0U,
    .arbitration_lost = 0U,
    .bus_error        = 0U,
    .reserved         = 0U
  },
  .flags    = 0U,
  .mode     = 0U
};

static I2C_TRANSFER_INFO I2C0_TransferInfo = {
  .num         = 0U,
  .cnt         = 0U,
  .tx_data     = 0U,
  .rx_data     = 0U,
  .addr        = 0U,
  .ctrl        = 0U
};

#if defined (RTE_I2C0_SDA) && (RTE_I2C0_SDA != 0U)
static I2C_PIN I2C0_pin_sda = {
  .pin = 14U,     // PA14
  .af  =  1U      // Alternate Function 1
};
#endif

#if defined (RTE_I2C0_SCL) && (RTE_I2C0_SCL != 0U)
static I2C_PIN I2C0_pin_scl = {
  .pin = 15U,     // PA15
  .af  =  1U      // Alternate Function 1
};
#endif

// I2C0 Resources
static const I2C_RESOURCES I2C0_Resources = {
  {      // Capabilities
    1U,  // Supports 10-bit addressing
    0U   // Reserved (must be zero)
  },
  .io = {
#if defined (RTE_I2C0_SDA) && (RTE_I2C0_SDA != 0U)
    .sda  = &I2C0_pin_sda,
#else
    NULL,
#endif
#if defined (RTE_I2C0_SCL) && (RTE_I2C0_SCL != 0U)
    .scl  = &I2C0_pin_scl,
#else
    NULL,
#endif
  },
  .reg            =  I2C0_REGISTER,
  .info           = &I2C0_Info,
  .xfer           = &I2C0_TransferInfo,
  .irq_num        =  I2C0_IRQn,
  .reset_bit      =  2U,
};
#endif


// I2C1
#if (defined (RTE_I2C1) && (RTE_I2C1 != 0))

#if defined __USE_SECURE
  #define I2C1_REGISTER  SECURE_I2C1
#else
  #define I2C1_REGISTER  I2C1
#endif

// I2C1 Run-Time Information
static       I2C_INFO I2C1_Info = {
  .cb_event = 0U,
  .status   = {
    .busy             = 0U,
    .mode             = 0U,
    .direction        = 0U,
    .general_call     = 0U,
    .arbitration_lost = 0U,
    .bus_error        = 0U,
    .reserved         = 0U
  },
  .flags    = 0U,
  .mode     = 0U
};

static I2C_TRANSFER_INFO I2C1_TransferInfo = {
  .num         = 0U,
  .cnt         = 0U,
  .data        = 0U,
  .addr        = 0U,
  .ctrl        = 0U
};

#if defined (RTE_I2C1_SDA) && (RTE_I2C1_SDA != 0U)
static I2C_PIN I2C1_pin_sda = {
  .pin = 18U,     // PA18
  .af  =  0U      // Primary Function
};
#endif

#if defined (RTE_I2C1_SCL) && (RTE_I2C1_SCL != 0U)
static I2C_PIN I2C1_pin_scl = {
  .pin = 19U,     // PA19
  .af  =  0U      // Primary Function
};
#endif

// I2C1 Resources
static const I2C_RESOURCES I2C1_Resources = {
  {      // Capabilities
    1U,  // Supports 10-bit addressing
    0U   // Reserved (must be zero)
  },
  .io = {
#if defined (RTE_I2C1_SDA) && (RTE_I2C1_SDA != 0U)
    .sda  = &I2C1_pin_sda,
#else
    NULL,
#endif
#if defined (RTE_I2C1_SCL) && (RTE_I2C1_SCL != 0U)
    .scl  = &I2C1_pin_scl,
#else
    NULL,
#endif
  },
  .reg            =  I2C1_REGISTER,
  .info           = &I2C1_Info,
  .xfer           = &I2C1_TransferInfo,
  .irq_num        =  I2C1_IRQn,
  .reset_bit      =  3U,
};
#endif


/**
  \fn          void I2C_PinInit (uint16_t pin, uint16_t af)
  \brief       Initialize I2C pin
*/
static void I2C_PinInit (uint16_t pin, uint16_t af) {

  if (pin < 32U) {
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
}

/**
  \fn          void I2C_PinDeInit (uint16_t pin)
  \brief       DeInitialize I2C pin
*/
static void I2C_PinDeInit (uint16_t pin) {

  if (pin < 32U) {
    SCC_REGISTER->IOMUX_MAIN_INSEL   |=  (1U << pin);
    SCC_REGISTER->IOMUX_MAIN_OUTSEL  |=  (1U << pin);
    SCC_REGISTER->IOMUX_MAIN_OENSEL  |=  (1U << pin);
  }
}

/**
  \fn          int I2C_SetClock (const I2C_RESOURCES *i2c, uint32_t pclk, uint32_t clock)
  \brief       Configure baudrate
  \param[in]   i2c       Pointer to I2C resources
  \param[in]   pclk      Peripheral clock
  \param[in]   fsclk     Clockrate
  \return      execution status
                -  0  = OK
                - -1  = Failed
*/
static int32_t I2C_SetClock(const I2C_RESOURCES *i2c,
                                  uint32_t       fpclk,
                                  uint32_t       fscl) {
#define I2C_DIV_A_Max      (64U)
#define I2C_DIV_B_Max      ( 4U)

  uint32_t i2c_cr;
  uint32_t div_a, div_b;

  /* Fscl = Fpclk / (22 x (divisor_a+1) x (divisor_b+1))
     The clock can not be faster than the Fpclk divide by 22. */

  // calculate (divisor_a) x (divisor_b)
  div_a = fpclk / (22 * fscl);

  // If the calculated value is negative or 0, the fscl input is out of range
  if (!div_a || (div_a > (I2C_DIV_A_Max * I2C_DIV_B_Max)))
    return -1;

  div_b = 1;
  while (div_a > I2C_DIV_A_Max) {
    div_b  += 1;
    div_a >>= 1;
  }

  i2c_cr = i2c->reg->CR;
  i2c_cr &= ~(I2C_CR_DIV_A_Msk |
              I2C_CR_DIV_B_Msk  );
  i2c_cr |= (((div_a-1) << I2C_CR_DIV_A_Pos) |
             ((div_b-1) << I2C_CR_DIV_B_Pos)  );
  i2c->reg->CR = i2c_cr;
  __DMB();

  return 0;
}

/**
  \fn          void I2C_Reset (const I2C_RESOURCES *i2c)
  \brief       Reset I2C peripheral
  \param[in]   i2c       Pointer to I2C resources
*/
static void I2C_Reset(const I2C_RESOURCES *i2c) {
  volatile uint32_t i2c_regval;

  // disable all interrupts
  i2c->reg->IDR = I2C_IxR_ALL_Msk;

  // clear hold, slave monitor and clear fifo
  i2c_regval = i2c->reg->CR;
  i2c_regval &= ~(I2C_CR_HOLD_Msk | I2C_CR_SLVMON_Msk);
  i2c_regval |=  I2C_CR_CLRFIFO_Msk;
  i2c->reg->CR = i2c_regval;

  // clear transfer size register
  i2c->reg->TSR = 0U;

  // clear interupt status register
  i2c_regval = i2c->reg->ISR;
  i2c->reg->ISR = i2c_regval;

  // clear status register
  i2c_regval = i2c->reg->SR;
//  i2c->reg->SR = i2c_regval;
}

// -- I2C Driver functions -------------------------------------------------

/**
  \fn          ARM_DRIVER_VERSION I2Cx_GetVersion (void)
  \brief       Get driver version.
  \return      \ref ARM_DRIVER_VERSION
*/
static ARM_DRIVER_VERSION I2Cx_GetVersion (void) {
  return i2c_driver_version;
}

/**
  \fn          ARM_I2C_CAPABILITIES I2C_GetCapabilities (void)
  \brief       Get driver capabilities
  \param[in]   i2c       Pointer to I2C resources
  \return      \ref ARM_I2C_CAPABILITIES
*/
static ARM_I2C_CAPABILITIES I2C_GetCapabilities (const I2C_RESOURCES *i2c) {
  return i2c->capabilities;
}

/**
  \fn          int32_t I2C_Initialize (const I2C_RESOURCES         *i2c,
                                             ARM_I2C_SignalEvent_t  cb_event)
  \brief       Initialize I2C Interface.
  \param[in]   i2c       Pointer to I2C resources
  \param[in]   cb_event  Pointer to \ref ARM_I2C_SignalEvent
  \return      \ref execution_status
*/
static int32_t I2C_Initialize (const I2C_RESOURCES         *i2c,
                                     ARM_I2C_SignalEvent_t  cb_event) {

  if (i2c->info->flags & I2C_FLAG_INITIALIZED) {
    // Driver is already initialized
    return ARM_DRIVER_OK;
  }

  // Initialize Run-time Resources
  i2c->info->cb_event = cb_event;

  // Clear Status flags
  i2c->info->status.busy             = 0U;
  i2c->info->status.mode             = 0U;
  i2c->info->status.direction        = 0U;
  i2c->info->status.general_call     = 0U;
  i2c->info->status.arbitration_lost = 0U;
  i2c->info->status.bus_error        = 0U;

  i2c->info->mode        = 0U;

  // Clear transfer information
  memset((void *)i2c->xfer, 0, sizeof(I2C_TRANSFER_INFO));

  // configure SDA pin
  if (i2c->io.sda) {
    I2C_PinInit (i2c->io.sda->pin, i2c->io.sda->af);
  }

  // configure SCL pin
  if (i2c->io.scl) {
    I2C_PinInit (i2c->io.scl->pin, i2c->io.scl->af);
  }

  i2c->info->flags = I2C_FLAG_INITIALIZED;

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t I2C_Uninitialize (I2C_RESOURCES *i2c)
  \brief       De-initialize I2C Interface.
  \param[in]   i2c       Pointer to I2C resources
  \return      \ref execution_status
*/
static int32_t I2C_Uninitialize (const I2C_RESOURCES *i2c) {

  // unConfigure SDA pin
  if (i2c->io.sda) {
    I2C_PinDeInit (i2c->io.sda->pin);
  }

  // unConfigure SCL pin
  if (i2c->io.scl) {
    I2C_PinDeInit (i2c->io.scl->pin);
  }

  // reset status flags
  i2c->info->flags = 0U;

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t I2C_PowerControl (const I2C_RESOURCES   *i2c,
                                               ARM_POWER_STATE  state)
  \brief       Control I2C Interface Power.
  \param[in]   i2c       Pointer to I2C resources
  \param[in]   state     Power state
  \return      \ref execution_status
*/
static int32_t I2C_PowerControl (const I2C_RESOURCES   *i2c,
                                       ARM_POWER_STATE  state) {
  volatile uint32_t i2c_regval;

  if ((state != ARM_POWER_OFF)  &&
      (state != ARM_POWER_FULL) &&
      (state != ARM_POWER_LOW)) {
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  switch (state) {
    case ARM_POWER_OFF:
      // clear and disable I2C IRQ
      NVIC_DisableIRQ(i2c->irq_num);
      NVIC_ClearPendingIRQ(i2c->irq_num);

      // Disableable I2C
      I2C_Reset(i2c);

      // hold I2C in Reset
      SCC_REGISTER->RESET_CTRL &= ~(1U << i2c->reset_bit);

      // Clear Status flags
      i2c->info->status.busy             = 0U;
      i2c->info->status.mode             = 0U;
      i2c->info->status.direction        = 0U;
      i2c->info->status.general_call     = 0U;
      i2c->info->status.arbitration_lost = 0U;
      i2c->info->status.bus_error        = 0U;

      // Clear powered flag
      i2c->info->flags &= ~I2C_FLAG_POWERED;
      break;

    case ARM_POWER_LOW:
      return ARM_DRIVER_ERROR_UNSUPPORTED;

    case ARM_POWER_FULL:
      if ((i2c->info->flags & I2C_FLAG_INITIALIZED) == 0U) {
        return ARM_DRIVER_ERROR;
      }
      if ((i2c->info->flags & I2C_FLAG_POWERED)     != 0U) {
        return ARM_DRIVER_OK;
      }

      // clear Status flags
      i2c->info->status.busy             = 0U;
      i2c->info->status.mode             = 0U;
      i2c->info->status.direction        = 0U;
      i2c->info->status.general_call     = 0U;
      i2c->info->status.arbitration_lost = 0U;
      i2c->info->status.bus_error        = 0U;

      // set flag initialized
      i2c->info->flags = I2C_FLAG_POWERED;

      // release I2C from reset
      SCC_REGISTER->RESET_CTRL |= (1U << i2c->reset_bit);
      while ((SCC_REGISTER->RESET_CTRL & (1U << i2c->reset_bit)) == 0U);

      // Disable all interrupts
      i2c->reg->IDR = I2C_IxR_ALL_Msk;

      // clear hold bit and fifos
      i2c->reg->CR = (i2c->reg->CR & ~(I2C_CR_HOLD_Msk | I2C_CR_SLVMON_Msk)) | I2C_CR_CLRFIFO_Msk;

      // set transfer size register to zero
      i2c->reg->TSR = 0U;

      // clear interrupt status register
      i2c_regval = i2c->reg->ISR;
      i2c->reg->ISR = i2c_regval;

      // clear status register
      i2c_regval = i2c->reg->SR;
//      i2c->reg->SR = i2c_regval;

      // clear and enable I2C IRQ
      NVIC_ClearPendingIRQ(i2c->irq_num);
      NVIC_EnableIRQ(i2c->irq_num);
      break;
  }

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t I2C_MasterTransmit (const I2C_RESOURCES *i2c,
                                                 uint32_t       addr,
                                                 const uint8_t *data,
                                                 uint32_t       num,
                                                 bool           xfer_pending)
  \brief       Start transmitting data as I2C Master.
  \param[in]   i2c           Pointer to I2C resources
  \param[in]   addr          Slave address (7-bit or 10-bit)
  \param[in]   data          Pointer to buffer with data to send to I2C Slave
  \param[in]   num           Number of data bytes to send
  \param[in]   xfer_pending  Transfer operation is pending - Stop condition will not be generated
  \return      \ref execution_status
*/
static int32_t I2C_MasterTransmit (const I2C_RESOURCES *i2c,
                                         uint32_t       addr,
                                         const uint8_t *data,
                                         uint32_t       num,
                                         bool           xfer_pending) {
  uint32_t i2c_cr = 0U;
  uint32_t regval;
  uint32_t fifo_sz;

  if ((data == NULL) || (num == 0U)) {
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  if ((addr & ~((uint32_t)ARM_I2C_ADDRESS_10BIT | (uint32_t)ARM_I2C_ADDRESS_GC)) > 0x3FFU) {
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  if (i2c->info->status.busy) {
    return (ARM_DRIVER_ERROR_BUSY);
  }

  i2c->info->status.busy             = 1U;
  i2c->info->status.mode             = 1U;      // master
  i2c->info->status.direction        = 0U;      // tx
  i2c->info->status.bus_error        = 0U;
  i2c->info->status.arbitration_lost = 0U;

  // Save transmit buffer info
  i2c->xfer->tx_data = data;
  i2c->xfer->num     = num;
  i2c->xfer->cnt     = 0U;
  i2c->xfer->addr    = (uint16_t)addr;
  i2c->xfer->ctrl    = XFER_CTRL_TXACTIVE;

  if (xfer_pending) {
    i2c->xfer->ctrl |= XFER_CTRL_XPENDING;
  }

  // configure acken
  i2c_cr |= I2C_CR_ACKEN_Msk;

  // configure interface mode and direction of transfer
  i2c_cr |= ((0U << I2C_CR_RW_Pos) |     // transmit
             (1U << I2C_CR_MS_Pos)  );   // master

  // clear the FIFO
  i2c_cr |= I2C_CR_CLRFIFO_Msk;

  // configure addressing mode
  i2c_cr |= ( (addr & ARM_I2C_ADDRESS_10BIT)
             ? (0U << I2C_CR_NEA_Pos)    // 10 bit address
             : (1U << I2C_CR_NEA_Pos) ); //  7 bit address

  // update control register
  i2c->reg->CR = (i2c->reg->CR & (I2C_CR_DIV_B_Msk | I2C_CR_DIV_A_Msk) ) | i2c_cr;

  // calculate free fifo space
  fifo_sz = I2C_FIFO_DEPTH - i2c->reg->TSR;

  // calculate number of bytes to write to the fifo
  fifo_sz = ((i2c->xfer->num > fifo_sz)) ? fifo_sz : i2c->xfer->num;

  // fill the fifo
  while (fifo_sz--) {
    i2c->reg->DR = *i2c->xfer->tx_data++;
    i2c->xfer->cnt++;
  }

  // configure hold bit
  if ((i2c->xfer->ctrl & XFER_CTRL_XPENDING) ||
      (i2c->xfer->num > i2c->xfer->cnt)        ) {
    i2c->reg->CR |= (1U << I2C_CR_HOLD_Pos);   // set Hold bit
  }

  // clear all interrupts
  regval = i2c->reg->ISR;
  i2c->reg->ISR = regval;

  // enable the master interrupts
  i2c->reg->IER = I2C_IxR_MASTER_Msk;

  // writing the slave address starts the transmission
  i2c->reg->AR = i2c->xfer->addr & I2C_AR_ADD_Msk;

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t I2C_MasterReceive (const I2C_RESOURCES *i2c,
                                                uint32_t       addr,
                                                uint8_t       *data,
                                                uint32_t       num,
                                                bool           xfer_pending)
  \brief       Start receiving data as I2C Master.
  \param[in]   i2c           Pointer to I2C resources
  \param[in]   addr          Slave address (7-bit or 10-bit)
  \param[out]  data          Pointer to buffer for data to receive from I2C Slave
  \param[in]   num           Number of data bytes to receive
  \param[in]   xfer_pending  Transfer operation is pending - Stop condition will not be generated
  \return      \ref execution_status
*/
static int32_t I2C_MasterReceive (const I2C_RESOURCES *i2c,
                                        uint32_t       addr,
                                        uint8_t       *data,
                                        uint32_t       num,
                                        bool           xfer_pending) {
  uint32_t i2c_cr = 0U;
  uint32_t regval;

  if ((data == NULL) || (num == 0U)) {
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  if (addr > 0x7F) {
    return ARM_DRIVER_ERROR_PARAMETER;
  }

  if (i2c->info->status.busy) {
    return (ARM_DRIVER_ERROR_BUSY);
  }

  i2c->info->status.busy             = 1U;
  i2c->info->status.mode             = 1U;      // master
  i2c->info->status.direction        = 1U;      // rx
  i2c->info->status.bus_error        = 0U;
  i2c->info->status.arbitration_lost = 0U;

  i2c->xfer->rx_data = data;
  i2c->xfer->num     = num;
  i2c->xfer->cnt     = 0U;
  i2c->xfer->addr    = (uint16_t)addr;
  i2c->xfer->ctrl    = XFER_CTRL_RXACTIVE;

  if (xfer_pending) {
    i2c->xfer->ctrl |= XFER_CTRL_XPENDING;
  }

  // configure acken
  i2c_cr |= I2C_CR_ACKEN_Msk;

  // configure interface mode and direction of transfer
  i2c_cr |= ((1U << I2C_CR_RW_Pos) |     // receive
             (1U << I2C_CR_MS_Pos)  );   // master

  // clear the FIFO
  i2c_cr |= I2C_CR_CLRFIFO_Msk;

  // configure addressing mode
  i2c_cr |= ( (addr & ARM_I2C_ADDRESS_10BIT)
             ? (0U << I2C_CR_NEA_Pos)    // 10 bit address
             : (1U << I2C_CR_NEA_Pos) ); //  7 bit address

  // update control register
  i2c->reg->CR = (i2c->reg->CR & (I2C_CR_DIV_B_Msk | I2C_CR_DIV_A_Msk) ) | i2c_cr;

  // configure bytes to receive
  i2c->xfer->tsr_cur = (i2c->xfer->num > I2C_XFER_SIZE) ? I2C_XFER_SIZE : i2c->xfer->num;
  i2c->reg->TSR = (i2c->xfer->num > I2C_XFER_SIZE) ? I2C_XFER_SIZE : i2c->xfer->num;

  // configure hold bit
  if ((i2c->xfer->ctrl & XFER_CTRL_XPENDING) ||
      (i2c->xfer->num > I2C_FIFO_DEPTH)        ) {
    i2c->reg->CR |= (1U << I2C_CR_HOLD_Pos);   // set Hold bit
  }

  // clear all interrupts
  regval = i2c->reg->ISR;
  i2c->reg->ISR = regval;

  // enable the master interrupts
  i2c->reg->IER = I2C_IxR_MASTER_Msk;

  // writing the slave address starts the transmission
  i2c->reg->AR = i2c->xfer->addr & I2C_AR_ADD_Msk;

  return ARM_DRIVER_OK;
}

/**
  \fn          int32_t I2C_SlaveTransmit (const I2C_RESOURCES *i2c,
                                          const uint8_t       *data,
                                                uint32_t       num)
  \brief       Start transmitting data as I2C Slave.
  \param[in]   i2c           Pointer to I2C resources
  \param[in]   data          Pointer to buffer with data to send to I2C Master
  \param[in]   num           Number of data bytes to send
  \return      \ref execution_status
*/
static int32_t I2C_SlaveTransmit (const I2C_RESOURCES *i2c,
                                  const uint8_t       *data,
                                        uint32_t       num) {

  UNUSED(data);
  UNUSED(num);
  UNUSED(i2c);

  return ARM_DRIVER_ERROR_UNSUPPORTED;
}

/**
  \fn          int32_t I2C_SlaveReceive (const I2C_RESOURCES *i2c,
                                               uint8_t       *data,
                                               uint32_t       num)
  \brief       Start receiving data as I2C Slave.
  \param[in]   i2c           Pointer to I2C resources
  \param[out]  data          Pointer to buffer for data to receive from I2C Master
  \param[in]   num           Number of data bytes to receive
  \return      \ref execution_status
*/
static int32_t I2C_SlaveReceive (const I2C_RESOURCES *i2c,
                                       uint8_t       *data,
                                       uint32_t       num) {

  UNUSED(data);
  UNUSED(num);
  UNUSED(i2c);

  return ARM_DRIVER_ERROR_UNSUPPORTED;
}

/**
  \fn          int32_t I2C_GetDataCount (void)
  \brief       Get transferred data count.
  \return      number of data bytes transferred; -1 when Slave is not addressed by Master
*/
static int32_t I2C_GetDataCount (const I2C_RESOURCES *i2c) {
  return ((int32_t)i2c->xfer->cnt);
}


/**
  \fn          int32_t I2C_Control (const I2C_RESOURCES *i2c,
                                          uint32_t       control,
                                          uint32_t       arg)
  \brief       Control I2C Interface.
  \param[in]   i2c      pointer to I2C resources
  \param[in]   control  operation
  \param[in]   arg      argument of operation (optional)
  \return      \ref execution_status
*/
static int32_t I2C_Control (const I2C_RESOURCES *i2c,
                                  uint32_t       control,
                                  uint32_t       arg) {

  if ((i2c->info->flags & I2C_FLAG_POWERED) == 0U) {
    // I2C not powered
    return ARM_DRIVER_ERROR;
  }

  switch (control) {
    case ARM_I2C_OWN_ADDRESS:
      if (arg) {
        i2c->reg->CR &= ~I2C_CR_MS_Msk;          // enable slave mode
        i2c->reg->AR  = (arg & I2C_AR_ADD_Msk);  // set slave address
      } else {
        i2c->reg->CR |=  I2C_CR_MS_Msk;          // disable slave mode
      }
      break;

    case ARM_I2C_BUS_SPEED:
      switch (arg) {
        case ARM_I2C_BUS_SPEED_STANDARD:         // Standard Speed (100kHz)
          I2C_SetClock(i2c,  SystemCoreClock, 100000U);
          break;
        case ARM_I2C_BUS_SPEED_FAST:             // Fast Speed (400kHz)
          I2C_SetClock(i2c,  SystemCoreClock, 400000U);
          break;
        case ARM_I2C_BUS_SPEED_FAST_PLUS:        // Fast+ Speed (1MHz)
          I2C_SetClock(i2c,  SystemCoreClock, 1000000U);
          break;
        case ARM_I2C_BUS_SPEED_HIGH:             // Fast+ Speed (3.4MHz)
        default:
          return ARM_DRIVER_ERROR_UNSUPPORTED;
      }
      break;

    case ARM_I2C_BUS_CLEAR:
      NVIC_DisableIRQ (i2c->irq_num);

      // Reset peripheral
      I2C_Reset(i2c);

      NVIC_ClearPendingIRQ (i2c->irq_num);
      NVIC_EnableIRQ (i2c->irq_num);
      break;

    case ARM_I2C_ABORT_TRANSFER:
      NVIC_DisableIRQ (i2c->irq_num);

    // Reset peripheral
      I2C_Reset(i2c);

      i2c->xfer->num  = 0U;
      i2c->xfer->cnt  = 0U;

      i2c->xfer->addr = 0U;
      i2c->xfer->ctrl = 0U;

      i2c->info->status.busy             = 0U;
      i2c->info->status.mode             = 0U;
      i2c->info->status.direction        = 0U;
      i2c->info->status.general_call     = 0U;
      i2c->info->status.arbitration_lost = 0U;
      i2c->info->status.bus_error        = 0U;

      SCC_REGISTER->RESET_CTRL &= ~(1U << i2c->reset_bit);
      SCC_REGISTER->RESET_CTRL |=  (1U << i2c->reset_bit);
      while ((SCC_REGISTER->RESET_CTRL & (1U << i2c->reset_bit)) == 0U);

      NVIC_ClearPendingIRQ (i2c->irq_num);
      NVIC_EnableIRQ (i2c->irq_num);
      break;

    default:
      return ARM_DRIVER_ERROR;
  }

  return ARM_DRIVER_OK;
}

/**
  \fn          ARM_I2C_STATUS I2C_GetStatus (I2C_RESOURCES *i2c)
  \brief       Get I2C status.
  \param[in]   i2c      pointer to I2C resources
  \return      I2C status \ref ARM_I2C_STATUS
*/
static ARM_I2C_STATUS I2C_GetStatus (const I2C_RESOURCES *i2c) {
  return i2c->info->status;
}

/**
  \fn          void I2C_MasterHandler (I2C_RESOURCES *i2c)
  \brief       I2C Master state event handler.
  \param[in]   i2c  Pointer to I2C resources
  \return      I2C event notification flags
*/
static uint32_t I2C_MasterHandler (I2C_RESOURCES *i2c) {
  uint32_t event  = 0U;
  uint32_t i2c_isr;
  uint32_t fifo_sz;
  uint32_t xfer_remain;
  uint8_t  i2c_dr;
  uint32_t tsr_upd;

  // read and clear interrupt status
  i2c_isr = i2c->reg->ISR;
  i2c->reg->ISR = i2c_isr;

  // handling nack and arbitration lost interrupt
  if (i2c_isr & I2C_IxR_ARB_LOST_Msk) {
    i2c->info->status.arbitration_lost = 1U;
    event |= ARM_I2C_EVENT_ARBITRATION_LOST;
  }

  if (i2c_isr & I2C_IxR_NACK_Msk) {
    // abort transmission
    i2c->info->status.busy = 0U;
    i2c->xfer->ctrl &= ~(XFER_CTRL_RXACTIVE | XFER_CTRL_TXACTIVE);

    event |=  ARM_I2C_EVENT_ADDRESS_NACK;
  }

  // handle direction master receive
  if (i2c->xfer->ctrl & XFER_CTRL_RXACTIVE){
    tsr_upd = 0U;

    if (i2c_isr & (I2C_IxR_COMP_Msk | I2C_IxR_DATA_Msk)) {
      // read the received bytes
      while (i2c->reg->SR & I2C_SR_RXDV_Msk) {
        i2c_dr = (uint8_t)i2c->reg->DR;
        *i2c->xfer->rx_data++ = i2c_dr;
        i2c->xfer->cnt++;

        i2c->xfer->tsr_cur--;
        if ((i2c->xfer->tsr_cur               == (I2C_FIFO_DEPTH + 1U)) &&
            ((i2c->xfer->num - i2c->xfer->cnt) > (I2C_FIFO_DEPTH + 1U))   ) {
              tsr_upd = 1U;
          break;
        }
      }

      // refill Transfer Size Register
      if (tsr_upd) {
        // wait until FIFO is full to quiesce the bus
        while (i2c->reg->TSR != 1U) __NOP();

        xfer_remain = i2c->xfer->num - i2c->xfer->cnt - I2C_FIFO_DEPTH;
        i2c->xfer->tsr_cur = ((xfer_remain > I2C_XFER_SIZE) ? I2C_XFER_SIZE : xfer_remain) + I2C_FIFO_DEPTH;
        i2c->reg->TSR      = ((xfer_remain > I2C_XFER_SIZE) ? I2C_XFER_SIZE : xfer_remain);
      }
    }

    if (i2c_isr & I2C_IxR_COMP_Msk) {

      i2c->info->status.busy = 0U;
      i2c->xfer->ctrl &= ~XFER_CTRL_RXACTIVE;

      // disable the master interrupts
      i2c->reg->IDR = I2C_IxR_MASTER_Msk;
    }

  } // end of XFER_CTRL_RXACTIVE

  // handle direction master transmit
  if (i2c->xfer->ctrl & XFER_CTRL_TXACTIVE) {

    if (i2c_isr & I2C_IxR_COMP_Msk) {
      if (i2c->xfer->cnt < i2c->xfer->num) {
        xfer_remain = i2c->xfer->num - i2c->xfer->cnt;

        // calculate free fifo space
        fifo_sz = I2C_FIFO_DEPTH - i2c->reg->TSR;

        // calculate number of bytes to write to the fifo
        fifo_sz = ((xfer_remain > fifo_sz)) ? fifo_sz : xfer_remain;

        // fill the fifo
        while (fifo_sz--) {
          i2c->reg->DR = *i2c->xfer->tx_data++;
          i2c->xfer->cnt++;
        }

        // configure hold bit
        if ((i2c->xfer->ctrl & XFER_CTRL_XPENDING) ||
            (i2c->xfer->num > i2c->xfer->cnt)        ) {
          i2c->reg->CR |=  (1U << I2C_CR_HOLD_Pos);   // set Hold bit
        } else {
          i2c->reg->CR &= ~(1U << I2C_CR_HOLD_Pos);   // clear Hold bit
        }

        // writing the slave address starts the transmission
        i2c->reg->AR = i2c->xfer->addr & I2C_AR_ADD_Msk;
      } else {
        // disable the master interrupts
        i2c->reg->IDR = I2C_IxR_MASTER_Msk;

        i2c->info->status.busy = 0U;
        i2c->xfer->ctrl &= ~XFER_CTRL_TXACTIVE;
      }
    }
  }

  return event;
}

/**
  \fn          void I2C_SlaveHandler (I2C_RESOURCES *i2c)
  \brief       I2C Slave state event handler.
  \param[in]   i2c      Pointer to I2C resources
  \return      I2C event notification flags
*/
static uint32_t I2C_SlaveHandler (I2C_RESOURCES *i2c) {
  uint32_t event  = 0U;
  uint32_t i2c_isr, i2c_sr;
  uint32_t i;

  // read and clear interrupt status
  i2c_isr = i2c->reg->ISR;
  i2c->reg->ISR = i2c_isr;

  // ignore masked interrupts
  i2c_isr &= ~i2c->reg->IMR;

  // determine transfer mode (send/receive)
  i2c_sr = i2c->reg->SR;

  // handle data send/receive
  if (i2c_sr & I2C_SR_RXRW_Msk) {
    // send data to master
    if (i2c_isr & I2C_IxR_DATA_Msk)
      i2c->reg->DR = i2c->xfer->tx_data[i2c->xfer->cnt++];

    if (i2c_isr & I2C_IxR_COMP_Msk) {
      event |= ARM_I2C_EVENT_TRANSFER_DONE;
    }
  } else {
    // receive data from master
    if (i2c_isr & I2C_IxR_DATA_Msk) {
      i = i2c->reg->TSR;      // read number of bytes to receive
      while (i--) {           // Read data
        i2c->xfer->rx_data[i2c->xfer->cnt++] = (uint8_t)i2c->reg->DR;
      }
    }

    if (i2c_isr & I2C_IxR_COMP_Msk) {
      i = i2c->reg->TSR;      // read number of bytes to receive
      while (i--) {           // Read data
        i2c->xfer->rx_data[i2c->xfer->cnt++] = (uint8_t)i2c->reg->DR;
      }

      event |= ARM_I2C_EVENT_TRANSFER_DONE;
    }
  }

  // master indicated xfer stop or fifo underflow/overflow
  if (i2c_isr & (I2C_IxR_NACK_Msk   | I2C_IxR_RX_OVF_Msk |
                 I2C_IxR_RX_UNF_Msk | I2C_IxR_TX_OVF_Msk) ) {
//  id->slave_state = CDNS_I2C_SLAVE_STATE_IDLE;
//  i2c_slave_event(id->slave, I2C_SLAVE_STOP, NULL);
    i2c->reg->CR |= I2C_CR_CLRFIFO_Msk;
  }

  return event;
}

/**
  \fn          void I2C_IRQHandler (I2C_RESOURCES *i2c)
  \brief       I2C global Interrupt handler.
  \param[in]   i2c       Pointer to I2C resources
*/
static void I2C_IRQHandler (I2C_RESOURCES *i2c) {
  uint32_t event;

  if (i2c->info->status.mode == 1U) {
    event = I2C_MasterHandler (i2c);
  }
  else {
    event = I2C_SlaveHandler (i2c);
  }

  // Send Event
  if ((event && i2c->info->cb_event) != 0U) {
    i2c->info->cb_event (event);
  }

}

#if (defined (RTE_I2C0) && (RTE_I2C0 == 1))
/* I2C0 Driver Wrapper functions */
       void                  I2C0_IRQHandler      (void);
static ARM_I2C_CAPABILITIES  I2C0_GetCapabilities (void)                                                                { return I2C_GetCapabilities (&I2C0_Resources);                                }
static int32_t               I2C0_Initialize      (ARM_I2C_SignalEvent_t cb_event)                                      { return I2C_Initialize      (&I2C0_Resources, cb_event);                      }
static int32_t               I2C0_Uninitialize    (void)                                                                { return I2C_Uninitialize    (&I2C0_Resources);                                }
static int32_t               I2C0_PowerControl    (ARM_POWER_STATE state)                                               { return I2C_PowerControl    (&I2C0_Resources, state);                         }
static int32_t               I2C0_MasterTransmit  (uint32_t addr, const uint8_t *data, uint32_t num, bool xfer_pending) { return I2C_MasterTransmit  (&I2C0_Resources, addr, data, num, xfer_pending); }
static int32_t               I2C0_MasterReceive   (uint32_t addr, uint8_t *data, uint32_t num, bool xfer_pending)       { return I2C_MasterReceive   (&I2C0_Resources, addr, data, num, xfer_pending); }
static int32_t               I2C0_SlaveTransmit   (const uint8_t *data, uint32_t num)                                   { return I2C_SlaveTransmit   (&I2C0_Resources, data, num);                     }
static int32_t               I2C0_SlaveReceive    (uint8_t *data, uint32_t num)                                         { return I2C_SlaveReceive    (&I2C0_Resources, data, num);                     }
static int32_t               I2C0_GetDataCount    (void)                                                                { return I2C_GetDataCount    (&I2C0_Resources);                                }
static int32_t               I2C0_Control         (uint32_t control, uint32_t arg)                                      { return I2C_Control         (&I2C0_Resources, control, arg);                  }
static ARM_I2C_STATUS        I2C0_GetStatus       (void)                                                                { return I2C_GetStatus       (&I2C0_Resources);                                }
       void                  I2C0_IRQHandler      (void)                                                                {        I2C_IRQHandler      (&I2C0_Resources);                                }

/* I2C0 Driver Control Block */
extern ARM_DRIVER_I2C Driver_I2C0;
       ARM_DRIVER_I2C Driver_I2C0 = {
         I2Cx_GetVersion,
         I2C0_GetCapabilities,
         I2C0_Initialize,
         I2C0_Uninitialize,
         I2C0_PowerControl,
         I2C0_MasterTransmit,
         I2C0_MasterReceive,
         I2C0_SlaveTransmit,
         I2C0_SlaveReceive,
         I2C0_GetDataCount,
         I2C0_Control,
         I2C0_GetStatus
       };
#endif

#if (defined (RTE_I2C1) && (RTE_I2C1 == 1))
/* I2C1 Driver Wrapper functions */
       void                  I2C1_IRQHandler      (void);
static ARM_I2C_CAPABILITIES  I2C1_GetCapabilities (void)                                                                { return I2C_GetCapabilities (&I2C1_Resources);                                }
static int32_t               I2C1_Initialize      (ARM_I2C_SignalEvent_t cb_event)                                      { return I2C_Initialize      (&I2C1_Resources, cb_event);                      }
static int32_t               I2C1_Uninitialize    (void)                                                                { return I2C_Uninitialize    (&I2C1_Resources);                                }
static int32_t               I2C1_PowerControl    (ARM_POWER_STATE state)                                               { return I2C_PowerControl    (&I2C1_Resources, state);                         }
static int32_t               I2C1_MasterTransmit  (uint32_t addr, const uint8_t *data, uint32_t num, bool xfer_pending) { return I2C_MasterTransmit  (&I2C1_Resources, addr, data, num, xfer_pending); }
static int32_t               I2C1_MasterReceive   (uint32_t addr, uint8_t *data, uint32_t num, bool xfer_pending)       { return I2C_MasterReceive   (&I2C1_Resources, addr, data, num, xfer_pending); }
static int32_t               I2C1_SlaveTransmit   (const uint8_t *data, uint32_t num)                                   { return I2C_SlaveTransmit   (&I2C1_Resources, data, num);                     }
static int32_t               I2C1_SlaveReceive    (uint8_t *data, uint32_t num)                                         { return I2C_SlaveReceive    (&I2C1_Resources, data, num);                     }
static int32_t               I2C1_GetDataCount    (void)                                                                { return I2C_GetDataCount    (&I2C1_Resources);                                }
static int32_t               I2C1_Control         (uint32_t control, uint32_t arg)                                      { return I2C_Control         (&I2C1_Resources, control, arg);                  }
static ARM_I2C_STATUS        I2C1_GetStatus       (void)                                                                { return I2C_GetStatus       (&I2C1_Resources);                                }
       void                  I2C1_IRQHandler      (void)                                                                {        I2C_IRQHandler      (&I2C1_Resources);                                }

/* I2C1 Driver Control Block */
extern ARM_DRIVER_I2C Driver_I2C1;
       ARM_DRIVER_I2C Driver_I2C1 = {
         I2Cx_GetVersion,
         I2C1_GetCapabilities,
         I2C1_Initialize,
         I2C1_Uninitialize,
         I2C1_PowerControl,
         I2C1_MasterTransmit,
         I2C1_MasterReceive,
         I2C1_SlaveTransmit,
         I2C1_SlaveReceive,
         I2C1_GetDataCount,
         I2C1_Control,
         I2C1_GetStatus
       };
#endif
