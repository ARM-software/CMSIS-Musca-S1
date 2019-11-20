/**************************************************************************//**
 * @file     Musca-S1.h
 * @brief    CMSIS Core Peripheral Access Layer Header File for
 *           Musca-S1 Device (configured for Musca_S1 without FPU)
 * @version  V1.0.0
 * @date     19. November 2019
 ******************************************************************************/
/* Copyright (c) 2019 ARM LIMITED

   All rights reserved.
   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
   - Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   - Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
   - Neither the name of ARM nor the names of its contributors may be used
     to endorse or promote products derived from this software without
     specific prior written permission.
   *
   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
   ---------------------------------------------------------------------------*/


#ifndef MUSCA_S1_H
#define MUSCA_S1_H

#ifdef __cplusplus
extern "C" {
#endif


/* -------------------------  Interrupt Number Definition  ------------------------ */

typedef enum IRQn
{
/* -------------------  Processor Exceptions Numbers  ----------------------------- */
  NonMaskableInt_IRQn           =  -14,     /*  2 Non Maskable Interrupt */
  HardFault_IRQn                =  -13,     /*  3 HardFault Interrupt */
  MemoryManagement_IRQn         =  -12,     /*  4 Memory Management Interrupt */
  BusFault_IRQn                 =  -11,     /*  5 Bus Fault Interrupt */
  UsageFault_IRQn               =  -10,     /*  6 Usage Fault Interrupt */
  SecureFault_IRQn              =   -9,     /*  7 Secure Fault Interrupt */
  SVCall_IRQn                   =   -5,     /* 11 SV Call Interrupt */
  DebugMonitor_IRQn             =   -4,     /* 12 Debug Monitor Interrupt */
  PendSV_IRQn                   =   -2,     /* 14 Pend SV Interrupt */
  SysTick_IRQn                  =   -1,     /* 15 System Tick Interrupt */

/* ----------------------  Core_IoT Specific Interrupt Numbers  ------------------- */
  NONSEC_WATCHDOG_RESET_IRQn    =    0,     /*    Non-Secure Watchdog Reset Interrupt */
  NONSEC_WATCHDOG_IRQn          =    1,     /*    Non-Secure Watchdog Interrupt */
  S32K_TIMER_IRQn               =    2,     /*    S32K Timer Interrupt */
  TIMER0_IRQn                   =    3,     /*    TIMER 0 Interrupt */
  TIMER1_IRQn                   =    4,     /*    TIMER 1 Interrupt */
  DUALTIMER_IRQn                =    5,     /*    Dual Timer Interrupt */
  MHU_0_IRQn                    =    6,     /*    Message Handling Unit 0 */
  MHU_1_IRQn                    =    7,     /*    Message Handling Unit 1 */
  MPC_IRQn                      =    9,     /*    MPC Combined (Secure) Interrupt */
  PPC_IRQn                      =   10,     /*    PPC Combined (Secure) Interrupt */
  MSC_IRQn                      =   11,     /*    MSC Combined (Secure) Interrput */
  BRIDGE_ERROR_IRQn             =   12,     /*    Bridge Error Combined (Secure) Interrupt */
  CPU_0_Cache_Invalid_IRQn      =   13,     /*    CPU 0 Instruction Cache Invaidation Interrupt */
  SYS_PPU_IRQn                  =   15,     /*    SYS_PPU Interrupt */

/* ----------------------  Expansion Specific Interrupt Numbers  ------------------ */
  GP_TIMER_IRQn                 =   33,     /*    General Purpose Timer Interrupt */
  I2C0_IRQn                     =   34,     /*    I2C 0 Interrupt */
  I2C1_IRQn                     =   35,     /*    I2C 1 Interrupt */
  I2S_IRQn                      =   36,     /*    I2S Interrupt */
  SPI_IRQn                      =   37,     /*    SPI Interrupt */
  QSPI_IRQn                     =   38,     /*    QSPI Interrupt */
  UART0RX_IRQn                  =   39,     /*    UART 0 RX FIFO Interrupt  active HIGH */
  UART0TX_IRQn                  =   40,     /*    UART 0 TX FIFO Interrupt  active HIGH */
  UART0_RxTimeout_IRQn          =   41,     /*    UART 0 RX Timeout Interrupt */
  UART0_ModemStatus_IRQn        =   42,     /*    UART 0 Modem Status Interrupt */
  UART0_Error_IRQn              =   43,     /*    UART 0 Error Interrupt */
  UART0_IRQn                    =   44,     /*    UART 0 Interrupt active HIGH */
  UART1RX_IRQn                  =   45,     /*    UART 1 RX FIFO Interrupt  active HIGH */
  UART1TX_IRQn                  =   46,     /*    UART 1 TX FIFO Interrupt  active HIGH */
  UART1_RxTimeout_IRQn          =   47,     /*    UART 1 RX Timeout Interrupt */
  UART1_ModemStatus_IRQn        =   48,     /*    UART 1 Modem Status Interrupt */
  UART1_Error_IRQn              =   49,     /*    UART 1 Error Interrupt */
  UART1_IRQn                    =   50,     /*    UART 1 Interrupt active HIGH */
  GPIO_0_IRQn                   =   51,     /*    All P0 I/O pins used as irq source */
  GPIO_1_IRQn                   =   52,     /*    There are 16 pins in total */
  GPIO_2_IRQn                   =   53,
  GPIO_3_IRQn                   =   54,
  GPIO_4_IRQn                   =   55,
  GPIO_5_IRQn                   =   56,
  GPIO_6_IRQn                   =   57,
  GPIO_7_IRQn                   =   58,
  GPIO_8_IRQn                   =   59,
  GPIO_9_IRQn                   =   60,
  GPIO_10_IRQn                  =   61,
  GPIO_11_IRQn                  =   62,
  GPIO_12_IRQn                  =   63,
  GPIO_13_IRQn                  =   64,
  GPIO_14_IRQn                  =   65,
  GPIO_15_IRQn                  =   66,
  GPIO_COMBINED_IRQn            =   67,      /*   GPIO 0 Combined Interrupt */
  PVT_IRQn                      =   68,      /*   PVT Sensor Interrupt */
  PWM0_IRQn                     =   70,      /*   PWM 0 Interrupt */
  RTC_IRQn                      =   71,      /*   RTC Interrupt */
  GP_TIMER0_IRQn                =   72,      /*   General Purpose Timer 0 Interrupt */
  GP_TIMER1_IRQn                =   73,      /*   General Purpose Timer 1 Interrupt */
  PWM1_IRQn                     =   74,      /*   PWM 1 Interrupt */
  PWM2_IRQn                     =   75,      /*   PWM 2 Interrupt */
  IOMUX_IRQn                    =   76       /*   IO Multiplexer Interrupt */
} IRQn_Type;


/* ================================================================================ */
/* ================      Processor and Core Peripheral Section     ================ */
/* ================================================================================ */

/* -------  Start of section using anonymous unions and disabling warnings  ------- */
#if   defined (__CC_ARM)
  #pragma push
  #pragma anon_unions
#elif defined (__ICCARM__)
  #pragma language=extended
#elif defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
  #pragma clang diagnostic push
  #pragma clang diagnostic ignored "-Wc11-extensions"
  #pragma clang diagnostic ignored "-Wreserved-id-macro"
#elif defined (__GNUC__)
  /* anonymous unions are enabled by default */
#elif defined (__TMS470__)
  /* anonymous unions are enabled by default */
#elif defined (__TASKING__)
  #pragma warning 586
#elif defined (__CSMC__)
  /* anonymous unions are enabled by default */
#else
  #warning Not supported compiler type
#endif


/* --------  Configuration of the Cortex-M Processor and Core Peripherals  ------- */
#define __CM33_REV            0x0001U       /* Core revision r0p1 */
#define __SAUREGION_PRESENT       1U        /* SAU regions are present */
#define __MPU_PRESENT             1U        /* MPU present */
#define __VTOR_PRESENT            1U        /* VTOR present */
#define __FPU_PRESENT             1U        /* FPU present */
#define __DSP_PRESENT             1U        /* DSP extension present */
#define __NVIC_PRIO_BITS          3U        /* Number of Bits used for Priority Levels */
#define __Vendor_SysTickConfig    0U        /* Set to 1 if different SysTick Config is used */

#include "core_cm33.h"                      /* Processor and core peripherals */
#include "system_Musca-S1.h"                /* System Header */


/* ================================================================================ */
/* ================       Device Specific Peripheral Section       ================ */
/* ================================================================================ */

/*------------- Universal Asynchronous Receiver Transmitter (UART) -----------*/
typedef struct /* see "PrimeCell UART (PL011) Technical Reference Manual r1p5" */
{
  __IOM  uint32_t  DR;                      /* Offset: 0x000 (R/W) Data Register */
  union {
  __IOM  uint32_t  RSR;                     /* Offset: 0x004 (R/W) Receive Status Register */
  __IOM  uint32_t  ECR;                     /* Offset: 0x004 (R/W) Error Clear Register */
    };
         uint32_t  RESERVED0[4];
  __IM   uint32_t  FR;                      /* Offset: 0x018 (R/ ) Flag Register */
         uint32_t  RESERVED1[1];
  __IOM  uint32_t  ILPR;                    /* Offset: 0x020 (R/W) IrDA Low-Power Counter Register */
  __IOM  uint32_t  IBRD;                    /* Offset: 0x024 (R/W) Integer Baud Rate Registe */
  __IOM  uint32_t  FBRD;                    /* Offset: 0x028 (R/W) Fractional Baud Rate Register */
  __IOM  uint32_t  LCR_H;                   /* Offset: 0x02C (R/W) Line Control Register */
  __IOM  uint32_t  CR;                      /* Offset: 0x030 (R/W) Control Register */
  __IOM  uint32_t  IFLS;                    /* Offset: 0x034 (R/W) Interrupt FIFO Level Select Register */
  __IOM  uint32_t  IMSC;                    /* Offset: 0x038 (R/W) Interrupt Mask Set/Clear Register */
  __IM   uint32_t  RIS;                     /* Offset: 0x03C (R/ ) Raw Interrupt Status Register */
  __IM   uint32_t  MIS;                     /* Offset: 0x040 (R/ ) Masked Interrupt Status Register */
  __OM   uint32_t  ICR;                     /* Offset: 0x044 ( /W) Interrupt Clear Register */
  __IOM  uint32_t  DMACR;                   /* Offset: 0x048 (R/W) DMA Control Register */
} UART_TypeDef;

/* UART DR Register Definitions */
#define UART_DR_OE_Pos                   11                                              /* UART DR.OE: Overrun error */
#define UART_DR_OE_Msk                   (1UL << UART_DR_OE_Pos)

#define UART_DR_BE_Pos                   10                                              /* UART DR.BE: Break error */
#define UART_DR_BE_Msk                   (1UL << UART_DR_BE_Pos)

#define UART_DR_PE_Pos                    9                                              /* UART DR.PE: Parity error */
#define UART_DR_PE_Msk                   (1UL << UART_DR_PE_Pos)

#define UART_DR_FE_Pos                    8                                              /* UART DR.FE: Framing error */
#define UART_DR_FE_Msk                   (1UL << UART_DR_FE_Pos)

#define UART_DR_DATA_Pos                  0                                              /* UART DR.DATA: Data character */
#define UART_DR_DATA_Msk                 (0xFFUL /*<< UART_DR_DATA_Pos*/)

/* UART FR Register Definitions */
#define UART_FR_RI_Pos                    8                                              /* UART FR.RI: Ring indicator */
#define UART_FR_RI_Msk                   (1UL << UART_FR_RI_Pos)

#define UART_FR_TXFE_Pos                  7                                              /* UART FR.TXFE: Transmit FIFO empty */
#define UART_FR_TXFE_Msk                 (1UL << UART_FR_TXFE_Pos)

#define UART_FR_RXFF_Pos                  6                                              /* UART FR.RXFF: Receive FIFO full */
#define UART_FR_RXFF_Msk                 (1UL << UART_FR_RXFF_Pos)

#define UART_FR_TXFF_Pos                  5                                              /* UART FR.TXFF: Transmit FIFO full */
#define UART_FR_TXFF_Msk                 (1UL << UART_FR_TXFF_Pos)

#define UART_FR_RXFE_Pos                  4                                              /* UART FR.RXFE: Receive FIFO empty */
#define UART_FR_RXFE_Msk                 (1UL << UART_FR_RXFE_Pos)

#define UART_FR_BUSY_Pos                  3                                              /* UART FR.BUSY: UART busy */
#define UART_FR_BUSY_Msk                 (1UL << UART_FR_BUSY_Pos)

#define UART_FR_DCD_Pos                   2                                              /* UART FR.DCD: Data carrier detect */
#define UART_FR_DCD_Msk                  (1UL << UART_FR_DCD_Pos)

#define UART_FR_DSR_Pos                   1                                              /* UART FR.DSR: Data set ready */
#define UART_FR_DSR_Msk                  (1UL << UART_FR_DSR_Pos)

#define UART_FR_CTS_Pos                   0                                              /* UART FR.CTS: Clear to send */
#define UART_FR_CTS_Msk                  (1UL /*<< UART_CR_CTS_Pos*/)

/* UART LCR_H Register Definitions */
#define UART_LCR_H_SPS_Pos                7                                              /* UART LCR_H.SPS: Stick parity select */
#define UART_LCR_H_SPS_Msk               (1UL << UART_LCR_H_SPS_Pos)

#define UART_LCR_H_WLEN_Pos               5                                              /* UART LCR_H.WLEN: Word length */
#define UART_LCR_H_WLEN_Msk              (3UL << UART_LCR_H_WLEN_Pos)

#define UART_LCR_H_FEN_Pos                4                                              /* UART LCR_H.FEN: Enable FIFOs */
#define UART_LCR_H_FEN_Msk               (1UL << UART_LCR_H_FEN_Pos)

#define UART_LCR_H_STP2_Pos               3                                              /* UART LCR_H.STP2: Two stop bits select */
#define UART_LCR_H_STP2_Msk              (1UL << UART_LCR_H_STP2_Pos)

#define UART_LCR_H_EPS_Pos                2                                              /* UART LCR_H.EPS: Even parity select */
#define UART_LCR_H_EPS_Msk               (1UL << UART_LCR_H_EPS_Pos)

#define UART_LCR_H_PEN_Pos                1                                              /* UART LCR_H.PEN: Parity enable */
#define UART_LCR_H_PEN_Msk               (1UL << UART_LCR_H_PEN_Pos)

#define UART_LCR_H_BRK_Pos                0                                              /* UART LCR_H.BRK: Send break */
#define UART_LCR_H_BRK_Msk               (1UL /*<< UART_LCR_H_BRK_Pos*/)

/* UART CR Register Definitions */
#define UART_CR_CTSEn_Pos                15                                              /* UART CR.CTSEn: CTS hardware flow control enable */
#define UART_CR_CTSEn_Msk                (1UL << UART_CR_CTSEn_Pos)

#define UART_CR_RTSEn_Pos                14                                              /* UART CR.RTSEn: RTS hardware flow control enable */
#define UART_CR_RTSEn_Msk                (1UL << UART_CR_RTSEn_Pos)

#define UART_CR_Out2_Pos                 13                                              /* UART CR.Out2: UART Out2 */
#define UART_CR_Out2_Msk                 (1UL << UART_CR_Out2_Pos)

#define UART_CR_Out1_Pos                 12                                              /* UART CR.Out1: UART Out1 */
#define UART_CR_Out1_Msk                 (1UL << UART_CR_Out1_Pos)

#define UART_CR_RTS_Pos                  11                                              /* UART CR.RTS: Request to send */
#define UART_CR_RTS_Msk                  (1UL << UART_CR_RTS_Pos)

#define UART_CR_DTR_Pos                  10                                              /* UART CR.DTR: Data transmit ready */
#define UART_CR_DTR_Msk                  (1UL << UART_CR_DTR_Pos)

#define UART_CR_RXE_Pos                   9                                              /* UART CR.RXE: Receive enable */
#define UART_CR_RXE_Msk                  (1UL << UART_CR_RXE_Pos)

#define UART_CR_TXE_Pos                   8                                              /* UART CR.TXE: Transmit enable */
#define UART_CR_TXE_Msk                  (1UL << UART_CR_TXE_Pos)

#define UART_CR_LBE_Pos                   7                                              /* UART CR.LBE: Loopback enable */
#define UART_CR_LBE_Msk                  (1UL << UART_CR_LBE_Pos)

#define UART_CR_SIRLP_Pos                 2                                              /* UART CR.SIRLP: SIR low-power IrDA mode */
#define UART_CR_SIRLP_Msk                (1UL << UART_CR_SIRLP_Pos)

#define UART_CR_SIREN_Pos                 1                                              /* UART CR.SIREN: SIR enable */
#define UART_CR_SIREN_Msk                (1UL << UART_CR_SIREN_Pos)

#define UART_CR_UARTEN_Pos                0                                              /* UART CR.UARTEN: UART enable */
#define UART_CR_UARTEN_Msk               (1UL /*<< UART_CR_UARTEN_Pos*/)

/* UART IMSC Register Definitions */
#define UART_IMSC_OEIM_Pos               10                                              /* UART IMSC.OEIM: Overrun error interrupt */
#define UART_IMSC_OEIM_Msk               (1UL << UART_IMSC_OEIM_Pos)

#define UART_IMSC_BEIM_Pos                9                                              /* UART IMSC.BEIM: Break error interrupt */
#define UART_IMSC_BEIM_Msk               (1UL << UART_IMSC_BEIM_Pos)

#define UART_IMSC_PEIM_Pos                8                                              /* UART IMSC.PEIM: Parity error interrupt */
#define UART_IMSC_PEIM_Msk               (1UL << UART_IMSC_PEIM_Pos)

#define UART_IMSC_FEIM_Pos                7                                              /* UART IMSC.FEIM: Framing error interrupt */
#define UART_IMSC_FEIM_Msk               (1UL << UART_IMSC_FEIM_Pos)

#define UART_IMSC_RTIM_Pos                6                                              /* UART IMSC.RTIM: Receive timeout interrupt */
#define UART_IMSC_RTIM_Msk               (1UL << UART_IMSC_RTIM_Pos)

#define UART_IMSC_TXIM_Pos                5                                              /* UART IMSC.TXIM: Transmit interrupt */
#define UART_IMSC_TXIM_Msk               (1UL << UART_IMSC_TXIM_Pos)

#define UART_IMSC_RXIM_Pos                4                                              /* UART IMSC.RXIM: Receive interrupt */
#define UART_IMSC_RXIM_Msk               (1UL << UART_IMSC_RXIM_Pos)

#define UART_IMSC_DSRMIM_Pos              3                                              /* UART IMSC.DSRMIM: nUARTDSR modem interrupt */
#define UART_IMSC_DSRMIM_Msk             (1UL << UART_IMSC_DSRMIM_Pos)

#define UART_IMSC_DCDMIM_Pos              2                                              /* UART IMSC.DCDMIM: nUARTDCD modem interrupt */
#define UART_IMSC_DCDMIM_Msk             (1UL << UART_IMSC_DCDMIM_Pos)                   /

#define UART_IMSC_CTSMIM_Pos              1                                              /* UART IMSC.CTSMIM: nUARTCTS modem interrupt */
#define UART_IMSC_CTSMIM_Msk             (1UL << UART_IMSC_CTSMIM_Pos)

#define UART_IMSC_RIMIM_Pos               0                                              /* UART IMSC.RIMIM: nUARTRI modem interrupt */
#define UART_IMSC_RIMIM_Msk              (1UL /*<< UART_IMSC_RIMIM_Pos*/)


/*------------- Serial Peripheral Interface (SPI) ----------------------------*/
typedef struct /* see "Musca-S1 Engineering Specification" */
{
  __IOM  uint32_t  CR;                      /* Offset: 0x000 (R/W) Main Configuration Register */
  __IM   uint32_t  ISR;                     /* Offset: 0x004 (R/ ) Interrupt Status Register */
  __OM   uint32_t  IER;                     /* Offset: 0x008 ( /W) Interrupt Enable Register */
  __OM   uint32_t  IDR;                     /* Offset: 0x00C ( /W) Interrupt Disable Registe */
  __IM   uint32_t  IMR;                     /* Offset: 0x010 (R/ ) Interrupt Mask Register */
  __IOM  uint32_t  ENR;                     /* Offset: 0x014 (R/W) SPI Enable/Disable Register */
  __IOM  uint32_t  DELAY;                   /* Offset: 0x018 (R/W) Delay Register */
  __OM   uint32_t  TDR;                     /* Offset: 0x01C ( /W) Transmit Data Register */
  __IM   uint32_t  RDR;                     /* Offset: 0x020 (R/ ) Receive Data Register */
  __IOM  uint32_t  SIC;                     /* Offset: 0x024 (R/W) Slave Idle Count Register */
  __IOM  uint32_t  TTH;                     /* Offset: 0x028 (R/W) TX FIFO Threshold Level Register */
  __IOM  uint32_t  RTH;                     /* Offset: 0x02C (R/W) RX FIFO Threshold Level Register */
} SPI_TypeDef;

/* SPI Main Configuration Register (CR) Definitions */
#define SPI_CR_TXCLR_Pos                 20                                              /* SPI CR.TXCLR: TX FIFO Clear */
#define SPI_CR_TXCLR_Msk                 (1UL << SPI_CR_TXCLR_Pos)

#define SPI_CR_RXCLR_Pos                 19                                              /* SPI CR.RXCLR: RX FIFO Clear */
#define SPI_CR_RXCLR_Msk                 (1UL << SPI_CR_RXCLR_Pos)

#define SPI_CR_TWS_Pos                    6                                              /* SPI CR.TWS: Transfer Word Size */
#define SPI_CR_TWS_Msk                   (3UL << SPI_CR_TWS_Pos)

#define SPI_CR_MBRD_Pos                   3                                              /* SPI CR.MBRD: Master Baud Rate Divisor */
#define SPI_CR_MBRD_Msk                  (7UL << SPI_CR_MBRD_Pos)

#define SPI_CR_CPHA_Pos                   2                                              /* SPI CR.CPHA: Clock Phase */
#define SPI_CR_CPHA_Msk                  (1UL << SPI_CR_CPHA_Pos)

#define SPI_CR_CPOL_Pos                   1                                              /* SPI CR.CPOL: Clock Polarity */
#define SPI_CR_CPOL_Msk                  (1UL << SPI_CR_CPOL_Pos)

#define SPI_CR_MSEL_Pos                   0                                              /* SPI CR.MSEL: Mode Select */
#define SPI_CR_MSEL_Msk                  (1UL /*<< SPI_CR_MSEL_Pos*/)

/* SPI Interrupt Register (ISR, IER, IDR, IMR) Definitions */
#define SPI_IR_TUF_Pos                    6                                              /* SPI ISR.TUF: TX FIFO Underflow */
#define SPI_IR_TUF_Msk                   (1UL << SPI_IR_TUF_Pos)

#define SPI_IR_RF_Pos                     5                                              /* SPI ISR.RF: RX FIFO Full */
#define SPI_IR_RF_Msk                    (1UL << SPI_IR_RF_Pos)

#define SPI_IR_RNE_Pos                    4                                              /* SPI ISR.RNE: RX FIFO Not Empty */
#define SPI_IR_RNE_Msk                   (1UL << SPI_IR_RNE_Pos)

#define SPI_IR_TF_Pos                     3                                              /* SPI ISR.TF: TX FIFO Full */
#define SPI_IR_TF_Msk                    (1UL << SPI_IR_TF_Pos)

#define SPI_IR_TNF_Pos                    2                                              /* SPI ISR.TNF: TX FIFO Not Full */
#define SPI_IR_TNF_Msk                   (1UL << SPI_IR_TNF_Pos)

#define SPI_IR_MF_Pos                     1                                              /* SPI ISR.MF: Mode Fail */
#define SPI_IR_MF_Msk                    (1UL << SPI_IR_MF_Pos)

#define SPI_IR_ROF_Pos                    0                                              /* SPI ISR.ROF: RX FIFO Overflow */
#define SPI_IR_ROF_Msk                   (1UL /*<< SPI_IR_ROF_Pos*/)

/* SPI SPI Enable/Disable Register (ENR) Definitions */
#define SPI_ENR_SPIE_Pos                  0                                              /* SPI ENR.SPIE: SPI Enable */
#define SPI_ENR_SPIE_Msk                 (1UL /*<< SPI_ENR_SPIE_Pos*/)


/*------------- Inter Integrated Circuit Bus (I2C) ---------------------------*/
typedef struct /* see "Musca-S1 Engineering Specification" */
{
  __IOM  uint32_t  CR;                      /* Offset: 0x000 (R/W) Control Register */
  __IM   uint32_t  SR;                      /* Offset: 0x004 (R/ ) Status Register */
  __IOM  uint32_t  AR;                      /* Offset: 0x008 (R/W) Address Register */
  __IOM  uint32_t  DR;                      /* Offset: 0x00C (R/W) Data Register */
  __IM   uint32_t  ISR;                     /* Offset: 0x010 (R/ ) Interrupt Status Register */
  __IOM  uint32_t  TSR;                     /* Offset: 0x014 (R/W) Transfer Size Register */
  __IOM  uint32_t  SMPR;                    /* Offset: 0x018 (R/W) Slave Monitor Pause Register */
  __IOM  uint32_t  TOR;                     /* Offset: 0x01C (R/W) Time Out Register */
  __IM   uint32_t  IMR;                     /* Offset: 0x020 (R/ ) Interrupt Mask Register */
  __OM   uint32_t  IER;                     /* Offset: 0x024 ( /W) Interrupt Enable Register */
  __OM   uint32_t  IDR;                     /* Offset: 0x028 ( /W) Interrupt Disable Register */
  __IOM  uint32_t  GFCR;                    /* Offset: 0x02C (R/W) Glitch Filter Control Register */
} I2C_TypeDef;

/* I2C Control Register (CR) Definitions */
#define I2C_CR_DIV_B_Pos                 14                                              /* I2C CR.DIV_B: Divisor B */
#define I2C_CR_DIV_B_Msk                 (3UL << I2C_CR_DIV_B_Pos)

#define I2C_CR_DIV_A_Pos                  8                                              /* I2C CR.DIV_A: Divisor A */
#define I2C_CR_DIV_A_Msk                 (0x3FUL << I2C_CR_DIV_A_Pos)

#define I2C_CR_CLRFIFO_Pos                6                                              /* I2C CR.CLRFIFO: Clear FIFO */
#define I2C_CR_CLRFIFO_Msk               (1UL << I2C_CR_CLRFIFO_Pos)

#define I2C_CR_SLVMON_Pos                 5                                              /* I2C CR.SLVMON: Slave Monitor mode */
#define I2C_CR_SLVMON_Msk                (1UL << I2C_CR_SLVMON_Pos)

#define I2C_CR_HOLD_Pos                   4                                              /* I2C CR.HOLD: Hold mode */
#define I2C_CR_HOLD_Msk                  (1UL << I2C_CR_HOLD_Pos)

#define I2C_CR_ACKEN_Pos                  3                                              /* I2C CR.ACKEN: Acknowledge Enable */
#define I2C_CR_ACKEN_Msk                 (1UL << I2C_CR_ACKEN_Pos)

#define I2C_CR_NEA_Pos                    2                                              /* I2C CR.NEA: Normal/Extended Address */
#define I2C_CR_NEA_Msk                   (1UL << I2C_CR_NEA_Pos)

#define I2C_CR_MS_Pos                     1                                              /* I2C CR.MS: Master/Slave */
#define I2C_CR_MS_Msk                    (1UL << I2C_CR_MS_Pos)

#define I2C_CR_RW_Pos                     0                                              /* I2C CR.RW: Read/Write */
#define I2C_CR_RW_Msk                    (1UL /*<< I2C_CR_RW_Pos*/)

/* I2C Status Register (CR) Definitions */
#define I2C_SR_BA_Pos                     8                                              /* I2C SR.BA: Bus Active */
#define I2C_SR_BA_Msk                    (1UL << I2C_SR_BA_Pos)

#define I2C_SR_RXOVF_Pos                  7                                              /* I2C SR.RXOVF: Receiver Overflow */
#define I2C_SR_RXOVF_Msk                 (1UL << I2C_SR_RXOVF_Pos)

#define I2C_SR_TXDV_Pos                   6                                              /* I2C SR.TXDV: Transmitter Data Valid */
#define I2C_SR_TXDV_Msk                  (1UL << I2C_SR_TXDV_Pos)

#define I2C_SR_RXDV_Pos                   5                                              /* I2C SR.RXDV: Receiver Data Valid */
#define I2C_SR_RXDV_Msk                  (1UL << I2C_SR_RXDV_Pos)

#define I2C_SR_RXRW_Pos                   3                                              /* I2C SR.RXRW: RX read/write flag */
#define I2C_SR_RXRW_Msk                  (1UL << I2C_SR_RXRW_Pos)

/* I2C Interrupt Register (ISR, IER, IDR, IMR) Definitions */
#define I2C_IR_ARB_LOST_Pos               9                                              /* I2C ISR.ARB_LOST: Arbitration Lost */
#define I2C_IR_ARB_LOST_Msk              (1UL << I2C_IR_ARB_LOST_Pos)

#define I2C_IR_RX_UNF_Pos                 7                                              /* I2C ISR.RX_UNF: FIFO Receive Underflow */
#define I2C_IR_RX_UNF_Msk                (1UL << I2C_IR_RX_UNF_Pos)

#define I2C_IR_TX_OVF_Pos                 6                                              /* I2C ISR.TUF: FIFO Transmit Overflow */
#define I2C_IR_TX_OVF_Msk                (1UL << I2C_IR_TX_OVF_Pos)

#define I2C_IR_RX_OVF_Pos                 5                                              /* I2C ISR.RX_OVF: FIFO Receive Overflow */
#define I2C_IR_RX_OVF_Msk                (1UL << I2C_IR_RX_OVF_Pos)

#define I2C_IR_SLV_RDY_Pos                4                                              /* I2C ISR.SLV_RDY: Monitored Slave Ready */
#define I2C_IR_SLV_RDY_Msk               (1UL << I2C_IR_SLV_RDY_Pos)

#define I2C_IR_TO_Pos                     3                                              /* I2C ISR.TO: Transfer Time Out */
#define I2C_IR_TO_Msk                    (1UL << I2C_IR_TO_Pos)

#define I2C_IR_NACK_Pos                   2                                              /* I2C ISR.NACK: Transfer Not Acknowledged */
#define I2C_IR_NACK_Msk                  (1UL << I2C_IR_NACK_Pos)

#define I2C_IR_DATA_Pos                   1                                              /* I2C ISR.DATA: More Data */
#define I2C_IR_DATA_Msk                  (1UL << I2C_IR_DATA_Pos)

#define I2C_IR_COMP_Pos                   0                                              /* I2C ISR.COMP: Transfer Complete */
#define I2C_IR_COMP_Msk                  (1UL /*<< I2C_IR_COMP_Pos*/)


/*------------- Inter-IC Sound (I2S) -----------------------------------------*/
typedef struct /* see "Musca-S1 Engineering Specification" */
{
  __IOM  uint32_t  CTRL;                    /* Offset: 0x000 (R/W) Transceiver Control Register */
  __IOM  uint32_t  INTR_STAT;               /* Offset: 0x004 (R/W) Interrupt Status Register */
  __IOM  uint32_t  SRR;                     /* Offset: 0x008 (R/W) Sample Rate and Resolution Control Register */
  __IOM  uint32_t  CID_CTRL;                /* Offset: 0x00C (R/W) Clock Strobes and Interrupt Masks Control Register */
  __IOM  uint32_t  TFIFO_STAT;              /* Offset: 0x010 (R/W) Transmit FIFO Level Status Register */
  __IOM  uint32_t  RFIFO_STAT;              /* Offset: 0x014 (R/W) Receive FIFO Level Status Register */
  __IOM  uint32_t  TFIFO_CTRL;              /* Offset: 0x018 (R/W) Transmit FIFO Thresholds Control Register */
  __IOM  uint32_t  RFIFO_CTRL;              /* Offset: 0x01C (R/W) Receive FIFO Rhresholds Control Register */
  __IOM  uint32_t  DEV_CONF;                /* Offset: 0x020 (R/W) Device Configuration Register */
  __IOM  uint32_t  POLL_STAT;               /* Offset: 0x024 (R/W) Polling Status Register */
} I2S_TypeDef;


/*----------------------------- Timer (TIMER) -------------------------------*/
typedef struct /* see "Cortex-M System Design Kit Technical Reference Manual r1p1" */
{
  __IOM  uint32_t  CTRL;                    /* Offset: 0x000 (R/W) Control Register */
  __IOM  uint32_t  VALUE;                   /* Offset: 0x004 (R/W) Current Value Register */
  __IOM  uint32_t  RELOAD;                  /* Offset: 0x008 (R/W) Reload Value Register */
  union {
  __IM   uint32_t  INTSTATUS;               /* Offset: 0x00C (R/ ) Interrupt Status Register */
  __OM   uint32_t  INTCLEAR;                /* Offset: 0x00C ( /W) Interrupt Clear Register */
    };
} TIMER_TypeDef;

/* TIMER CTRL Register Definitions */
#define TIMER_CTRL_INTEN_Pos              3                                              /* TIMER CTRL.INTEN: Interrupt enable */
#define TIMER_CTRL_INTEN_Msk             (1UL << TIMER_CTRL_INTEN_Pos)

#define TIMER_CTRL_SELEXTCLK_Pos          2                                              /* TIMER CTRL.SELEXTCLK: Select external input as clock */
#define TIMER_CTRL_SELEXTCLK_Msk         (1UL << TIMER_CTRL_SELEXTCLK_Pos)

#define TIMER_CTRL_SELEXTEN_Pos           1                                              /* TIMER CTRL.SELEXTEN: Select external input as enable */
#define TIMER_CTRL_SELEXTEN_Msk          (1UL << TIMER_CTRL_SELEXTEN_Pos)

#define TIMER_CTRL_EN_Pos                 0                                              /* TIMER CTRL.EN: Enable */
#define TIMER_CTRL_EN_Msk                (1UL /*<< TIMER_CTRL_EN_Pos*/)

/* TIMER VALUE Register Definitions */
#define TIMER_VALUE_Pos                   0                                              /* TIMER VALUE: Current value */
#define TIMER_VALUE_Msk                  (0xFFFFFFFFUL /*<< TIMER_VALUE_Pos*/)

/* TIMER RELOAD Register Definitions */
#define TIMER_RELOAD_Pos                  0                                              /* TIMER RELOAD: Reload value */
#define TIMER_RELOAD_Msk                 (0xFFFFFFFFUL /*<< TIMER_RELOAD_Pos*/)

/* TIMER INTSTATUS Register Definitions */
#define TIMER_INTSTATUS_INT_Pos           0                                              /* TIMER INTSTATUS.INT: Interrupt status */
#define TIMER_INTSTATUS_INT_Msk          (1UL /*<< TIMER_INTSTATUS_INT_Pos*/)

/* TIMER INTCLEAR Register Definitions */
#define TIMER_INTCLEAR_INT_Pos            0                                             /* TIMER INTCLEAR.INT: Interrupt clear */
#define TIMER_INTCLEAR_INT_Msk           (1UL /*<< TIMER_INTCLEAR_INT_Pos*/)


/*----------------------General-purpose Timer (GP_TIMER) -----------------------------*/
typedef struct /* see "?" */
{
  __IOM  uint32_t RESET;                    /* Offset: 0x000 (R/ ) Reset Control Register */
  __IOM  uint32_t INTM;                     /* Offset: 0x004 (R/W) Masked Interrupt Status Register */
  __IOM  uint32_t INTC;                     /* Offset: 0x008 (R/W) Interrupt Clear Register */
         uint32_t RESERVED0[1U];
  __IOM  uint32_t ALARM0;                   /* Offset: 0x010 (R/W) Alarm 0 Data Value Register */
  __IOM  uint32_t ALARM1;                   /* Offset: 0x014 (R/W) Alarm 1 Data Value Register */
  __IM   uint32_t INTR;                     /* Offset: 0x018 (R/ ) Raw Interrupt Status Register */
  __IM   uint32_t COUNTER;                  /* Offset: 0x01C (R/ ) Counter Data Value Register */
} GP_TIMER_TypeDef;


/*-------------------- General Purpose Input Output (GPIO) -------------------*/
typedef struct /* see "Cortex-M System Design Kit Technical Reference Manual r1p1" */
{
  __IOM  uint32_t  DATA;                    /* Offset: 0x000 (R/W) DATA Register */
  __IOM  uint32_t  DATAOUT;                 /* Offset: 0x004 (R/W) Data Output Latch Register */
         uint32_t  RESERVED0[2];
  __IOM  uint32_t  OUTENSET;                /* Offset: 0x010 (R/W) Output Enable Set Register */
  __IOM  uint32_t  OUTENCLR;                /* Offset: 0x014 (R/W) Output Enable Clear Register */
  __IOM  uint32_t  ALTFUNCSET;              /* Offset: 0x018 (R/W) Alternate Function Set Register */
  __IOM  uint32_t  ALTFUNCCLR;              /* Offset: 0x01C (R/W) Alternate Function Clear Register */
  __IOM  uint32_t  INTENSET;                /* Offset: 0x020 (R/W) Interrupt Enable Set Register */
  __IOM  uint32_t  INTENCLR;                /* Offset: 0x024 (R/W) Interrupt Enable Clear Register */
  __IOM  uint32_t  INTTYPESET;              /* Offset: 0x028 (R/W) Interrupt Type Set Register */
  __IOM  uint32_t  INTTYPECLR;              /* Offset: 0x02C (R/W) Interrupt Type Clear Register */
  __IOM  uint32_t  INTPOLSET;               /* Offset: 0x030 (R/W) Interrupt Polarity Set Register */
  __IOM  uint32_t  INTPOLCLR;               /* Offset: 0x034 (R/W) Interrupt Polarity Clear Register */
  union {
  __IM   uint32_t  INTSTATUS;               /* Offset: 0x038 (R/ ) Interrupt Status Register */
  __OM   uint32_t  INTCLEAR;                /* Offset: 0x038 ( /W) Interrupt Clear Register */
  };
         uint32_t RESERVED1[241];
  __IOM  uint32_t MASKLOWBYTE[256];         /* Offset: 0x400 (R/W) Lower byte masked Access Register */
  __IOM  uint32_t MASKHIGHBYTE[256];        /* Offset: 0x800 (R/W) Upper byte masked Access Register */
} GPIO_TypeDef;


/*------------- System Information (SYSINFO) ----------------------------*/
typedef struct /* see " ? " */
{
  __IM  uint32_t SYS_VERSION;               /* Offset: 0x000 (R/ ) System Version Register                 ( 0x2004_1743) */
  __IM  uint32_t SYS_CONFIG;                /* Offset: 0x004 (R/ ) System Hardware Configuration Register  ( 0x2230_1544) */
} SYSINFO_TypeDef;


/*------------- System Control (SYSCTRL) ----------------------------*/
typedef struct /* see " ? " */
{
  __IM  uint32_t SECDBGSTAT;          /* Offset: 0x000 (R/ ) Secure Debug Configuration Status Register */
  __OM  uint32_t SECDBGSET;           /* Offset: 0x004 ( /W) Secure Debug Configuration Set Register */
  __OM  uint32_t SECDBGCLR;           /* Offset: 0x008 ( /W) Secure Debug Configuration Clear Register */
  __IOM uint32_t SCSECCTRL;           /* Offset: 0x00C (R/W) System Control Security Control Register */
  __IOM uint32_t FCLK_DIV;            /* Offset: 0x010 (R/W) Fast Clock Divider Configuration Register */
  __IOM uint32_t SYSCLK_DIV;          /* Offset: 0x014 (R/W) System Clock Divider Configuration Register */
  __IOM uint32_t CLOCK_FORCE;         /* Offset: 0x018 (R/W) Clock Forces */
        uint32_t RESERVED0[57];
  __IOM uint32_t RESET_SYNDROME;      /* Offset: 0x100 (R/W) Reset syndrome */
  __IOM uint32_t RESET_MASK;          /* Offset: 0x104 (R/W) Reset MASK */
  __OM  uint32_t SWRESET;             /* Offset: 0x108 ( /W) Software Reset */
  __IOM uint32_t GRETREG;             /* Offset: 0x10C (R/W) General Purpose Retention Register */
  __IOM uint32_t INITSVTOR0;          /* Offset: 0x110 (R/W) Initial Secure Reset Vector Register For CPU 0 */
  __IOM uint32_t INITSVTOR1;          /* Offset: 0x114 (R/W) Initial Secure Reset Vector Register For CPU 1 */
  __IOM uint32_t CPUWAIT;             /* Offset: 0x118 (R/W) CPU Boot wait control after reset */
  __IOM uint32_t NMI_ENABLE;          /* Offset: 0x11C (R/W) NMI Enable Register */
  __IOM uint32_t WICCTRL;             /* Offset: 0x120 (R/W) CPU WIC Request and Acknowledgement */
  __IOM uint32_t EWCTRL;              /* Offset: 0x124 (R/W) External Wakeup Control */
        uint32_t RESERVED1[54];
  __IOM uint32_t PDCM_PD_SYS_SENSE;   /* Offset: 0x200 (R/W) Power Control Dependency Matrix PD_SYS Power Domain Sensitivity */
        uint32_t RESERVED2[2];
  __IOM uint32_t PDCM_PD_SRAM0_SENSE; /* Offset: 0x20C (R/W) Power Control Dependency Matrix PD_SRAM0 Power Domain Sensitivity */
  __IOM uint32_t PDCM_PD_SRAM1_SENSE; /* Offset: 0x210 (R/W) Power Control Dependency Matrix PD_SRAM1 Power Domain Sensitivity */
  __IOM uint32_t PDCM_PD_SRAM2_SENSE; /* Offset: 0x214 (R/W) Power Control Dependency Matrix PD_SRAM2 Power Domain Sensitivity */
  __IOM uint32_t PDCM_PD_SRAM3_SENSE; /* Offset: 0x218 (R/W) Power Control Dependency Matrix PD_SRAM3 Power Domain Sensitivity */
} SYSCTRL_TypeDef;


/*------------------- Serial Communication Controller (SCC) ------------*/
typedef struct /* see "Musca-S1 Engineering Specification" */
{
  __IOM uint32_t CLK_CTRL_SEL;                    /* Offset: 0x000 (R/W) */
  __IOM uint32_t CLK_PLL_PREDIV_CTRL;             /* Offset: 0x004 (R/W) */
  __IOM uint32_t CLK_BBGEN_DIV_CLK;               /* Offset: 0x008 (R/W) */
        uint32_t RESERVED0[1U];
  __IOM uint32_t CLK_POSTDIV_QSPI;                /* Offset: 0x010 (R/W) */
  __IOM uint32_t CLK_POSTDIV_RTC;                 /* Offset: 0x014 (R/W) */
        uint32_t RESERVED0_1[1U];
  __IOM uint32_t CLK_POSTDIV_TEST;                /* Offset: 0x01C (R/W) */
  __IOM uint32_t CTRL_BYPASS_DIV;                 /* Offset: 0x020 (R/W) */
  __IOM uint32_t PLL_CTRL_PLL0_CLK;               /* Offset: 0x024 (R/W) */
        uint32_t RESERVED0_2[2U];
  __IOM uint32_t CLK_CTRL_ENABLE;                 /* Offset: 0x030 (R/W) */
  __IM  uint32_t CLK_STATUS;                      /* Offset: 0x034 (R/ ) */
        uint32_t RESERVED0_3[2U];
  __IOM uint32_t RESET_CTRL;                      /* Offset: 0x040 (R/W) */
        uint32_t RESERVED0_4[1U];
  __IOM uint32_t DBG_CTRL;                        /* Offset: 0x048 (R/W) */
  __IOM uint32_t SRAM_CTRL;                       /* Offset: 0x04C (R/W) */
  __IOM uint32_t INTR_CTRL;                       /* Offset: 0x050 (R/W) */
        uint32_t RESERVED1[1U];
  __IOM uint32_t CPU0_VTOR;                       /* Offset: 0x058 (R/W) */
  __IOM uint32_t CPU0_VTOR_1;                     /* Offset: 0x05C (R/W) */
  __IOM uint32_t CPU1_VTOR;                       /* Offset: 0x060 (R/W) */
  __IOM uint32_t CPU1_VTOR_1;                     /* Offset: 0x064 (R/W) */
  __IOM uint32_t IOMUX_MAIN_INSEL;                /* Offset: 0x068 (R/W) */
        uint32_t RESERVED2[1U];
  __IOM uint32_t IOMUX_MAIN_OUTSEL;               /* Offset: 0x070 (R/W) */
        uint32_t RESERVED3[1U];
  __IOM uint32_t IOMUX_MAIN_OENSEL;               /* Offset: 0x078 (R/W) */
        uint32_t RESERVED4[1U];
  __IOM uint32_t IOMUX_MAIN_DEFAULT_IN;           /* Offset: 0x080 (R/W) */
        uint32_t RESERVED5[1U];
  __IOM uint32_t IOMUX_ALTF1_INSEL;               /* Offset: 0x088 (R/W) */
        uint32_t RESERVED6[1U];
  __IOM uint32_t IOMUX_ALTF1_OUTSEL;              /* Offset: 0x090 (R/W) */
        uint32_t RESERVED7[1U];
  __IOM uint32_t IOMUX_ALTF1_OENSEL;              /* Offset: 0x098 (R/W) */
        uint32_t RESERVED8[1U];
  __IOM uint32_t IOMUX_ALTF1_DEFAULT_IN;          /* Offset: 0x0A0 (R/W) */
        uint32_t RESERVED9[1U];
  __IOM uint32_t IOMUX_ALTF2_INSEL;               /* Offset: 0x0A8 (R/W) */
        uint32_t RESERVED10[1U];
  __IOM uint32_t IOMUX_ALTF2_OUTSEL;              /* Offset: 0x0B0 (R/W) */
        uint32_t RESERVED12[1U];
  __IOM uint32_t IOMUX_ALTF2_OENSEL;              /* Offset: 0x0B8 (R/W) */
        uint32_t RESERVED13[1U];
  __IOM uint32_t IOMUX_ALTF2_DEFAULT_IN;          /* Offset: 0x0C0 (R/W) */
        uint32_t RESERVED14[1U];
  __IOM uint32_t IOMUX_ALTF3_INSEL;               /* Offset: 0x0C8 (R/W) */
        uint32_t RESERVED15[1U];
  __IOM uint32_t IOMUX_ALTF3_OUTSEL;              /* Offset: 0x0D0 (R/W) */
        uint32_t RESERVED16[1U];
  __IOM uint32_t IOMUX_ALTF3_OENSEL;              /* Offset: 0x0D8 (R/W) */
        uint32_t RESERVED17[1U];
  __IOM uint32_t IOMUX_ALTF3_DEFAULT_IN;          /* Offset: 0x0E0 (R/W) */
        uint32_t RESERVED18[1U];
  __IOM uint32_t IOPAD_DS0;                       /* Offset: 0x0E8 (R/W) */
        uint32_t RESERVED19[1U];
  __IOM uint32_t IOPAD_DS1;                       /* Offset: 0x0F0 (R/W) */
        uint32_t RESERVED20[1U];
  __IOM uint32_t IOPAD_PE;                        /* Offset: 0x0F8 (R/W) */
        uint32_t RESERVED21[1U];
  __IOM uint32_t IOPAD_PS;                        /* Offset: 0x100 (R/W) */
        uint32_t RESERVED22[1U];
  __IOM uint32_t IOPAD_SR;                        /* Offset: 0x108 (R/W) */
        uint32_t RESERVED23[1U];
  __IOM uint32_t IOPAD_IS;                        /* Offset: 0x110 (R/W) */
        uint32_t RESERVED24[1U];
  __IOM uint32_t PVT_CTRL ;                       /* Offset: 0x118 (R/W) */
        uint32_t RESERVED25[5U];
  __IOM uint32_t SPARE0;                          /* Offset: 0x130 (R/W) */
  __IOM uint32_t SRAM_RW_MARGINE;                 /* Offset: 0x134 (R/W) */
  __IOM uint32_t STATIC_CONF_SIG0;                /* Offset: 0x138 (R/W) */
  __IOM uint32_t STATIC_CONF_SIG1;                /* Offset: 0x13C (R/W) */
  __IOM uint32_t REQ_SET;                         /* Offset: 0x140 (R/W) */
  __IOM uint32_t REQ_CLEAR;                       /* Offset: 0x144 (R/W) */
  __IOM uint32_t PCSM_CTRL_OVEERIDE;              /* Offset: 0x148 (R/W) */
  __IOM uint32_t PD_CPU0_ISO_OVEERIDE;            /* Offset: 0x14C (R/W) */
  __IOM uint32_t PD_CPU1_ISO_OVEERIDE;            /* Offset: 0x150 (R/W) */
  __IOM uint32_t SYS_SRAM_RW_ASSIST0;             /* Offset: 0x154 (R/W) */
  __IOM uint32_t SYS_SRAM_RW_ASSIST1;             /* Offset: 0x158 (R/W) */
        uint32_t RESERVED26[2U];
  __IOM uint32_t SYS_SRAM_RW_ASSIST4;             /* Offset: 0x164 (R/W) */
  __IOM uint32_t SYS_SRAM_RW_ASSIST5;             /* Offset: 0x168 (R/W) */
        uint32_t RESERVED27[6U];
  __IOM uint32_t REQ_EDGE_SEL;                    /* Offset: 0x184 (R/W) */
  __IOM uint32_t REQ_ENABLE;                      /* Offset: 0x188 (R/W) */
        uint32_t RESERVED28[3U];
  __IOM uint32_t SCC_MRAM_CTRL0;                  /* Offset: 0x198 (R/W) */
  __IOM uint32_t SCC_MRAM_CTRL1;                  /* Offset: 0x19C (R/W) */
  __IOM uint32_t SCC_MRAM_CTRL2;                  /* Offset: 0x1A0 (R/W) */
  __IOM uint32_t SCC_MRAM_CTRL3;                  /* Offset: 0x1A4 (R/W) */
  __IOM uint32_t SCC_MRAM_CTRL4;                  /* Offset: 0x1A8 (R/W) */
        uint32_t RESERVED29[1U];
  __IOM uint32_t SCC_MRAM_DIN0;                   /* Offset: 0x1B0 (R/W) */
  __IOM uint32_t SCC_MRAM_DIN1;                   /* Offset: 0x1B4 (R/W) */
  __IOM uint32_t SCC_MRAM_DIN2;                   /* Offset: 0x1B8 (R/W) */
        uint32_t RESERVED30[1U];
  __IM  uint32_t SCC_MRAM_DOUT0;                  /* Offset: 0x1C0 (R/ ) */
  __IM  uint32_t SCC_MRAM_DOUT1;                  /* Offset: 0x1C4 (R/ ) */
  __IM  uint32_t SCC_MRAM_DOUT2;                  /* Offset: 0x1C8 (R/ ) */
  __IM  uint32_t SCC_MRAM_STATUS;                 /* Offset: 0x1CC (R/ ) */
        uint32_t RESERVED31[4U];
  __IOM uint32_t SELECTION_CONTROL_REG;           /* Offset: 0x1E0 (R/W) */
        uint32_t RESERVED32[7U];
  __IOM uint32_t AZ_CTRL;                         /* Offset: 0x200 (R/W) */
  __IOM uint32_t CASTOR_OTP_CTRL;                 /* Offset: 0x204 (R/W) */
        uint32_t RESERVED33[6U];
  __IOM uint32_t SPARE_CTRL0;                     /* Offset: 0x222 (R/W) */
  __IOM uint32_t SPARE_CTRL1;                     /* Offset: 0x226 (R/W) */
        uint32_t RESERVED34[117U];
  __IM  uint32_t CHIP_ID;                         /* Offset: 0x400 (R/ ) */
  __IM  uint32_t IO_IN_STATUS;                    /* Offset: 0x404 (R/ ) */
} SCC_TypeDef;


/*------------------- Memory Protection Controller -----------------------------*/
typedef struct /* see "ARM CoreLink SSE-200 Subsystem Technical Reference Manual r1p0" */
{
  __IOM  uint32_t CTRL;                     /* Offset: 0x000 (R/W) Control Register */
         uint32_t RESERVED0[3];
  __IM   uint32_t BLK_MAX;                  /* Offset: 0x010 (R/ ) Block Maximum Register */
  __IM   uint32_t BLK_CFG;                  /* Offset: 0x014 (R/ ) Block Configuration Register */
  __IOM  uint32_t BLK_IDX;                  /* Offset: 0x018 (R/W) Block Index Register */
  __IOM  uint32_t BLK_LUT;                  /* Offset: 0x01C (R/W) Block Lookup Tabe Register */
  __IM   uint32_t INT_STAT;                 /* Offset: 0x020 (R/ ) Interrupt Status Register */
  __OM   uint32_t INT_CLEAR;                /* Offset: 0x024 ( /W) Interrupt Clear Register */
  __IOM  uint32_t INT_EN;                   /* Offset: 0x028 (R/W) Interrupt Enable Register */
  __IM   uint32_t INT_INFO1;                /* Offset: 0x02C (R/ ) Interrupt Info1 Register */
  __IM   uint32_t INT_INFO2;                /* Offset: 0x030 (R/ ) Interrupt Info2 Register */
  __OM   uint32_t INT_SET;                  /* Offset: 0x034 ( /W) Interrupt Set Register */
} MPC_TypeDef;


/*------------------- Secure Privilege Control Block -----------------------------*/
typedef struct /* see "ARM CoreLink SSE-200 Subsystem Technical Reference Manual r1p0" */
{
         uint32_t RESERVED0[4U];
  __IOM  uint32_t SECRESPCFG;               /* Offset: 0x010 (R/W) Security Violation Response Configuration Register */
  __IOM  uint32_t NSCCFG;                   /* Offset: 0x014 (R/W) Non Secure Callable Configuration for IDAU */
         uint32_t RESERVED1[1U];
  __IM   uint32_t SECMPCINTSTATUS;          /* Offset: 0x01C (R/ ) Secure MPC Interrupt Status */
  __IM   uint32_t SECPPCINTSTAT;            /* Offset: 0x020 (R/ ) Secure PPC Interrupt Status */
  __OM   uint32_t SECPPCINTCLR;             /* Offset: 0x024 ( /W) Secure PPC Interrupt Clear */
  __IOM  uint32_t SECPPCINTEN;              /* Offset: 0x028 (R/W) Secure PPC Interrupt Enable */
         uint32_t RESERVED2[1U];
  __IM   uint32_t SECMSCINTSTAT;            /* Offset: 0x030 (R/ ) Secure MSC Interrupt Status */
  __OM   uint32_t SECMSCINTCLR;             /* Offset: 0x034 ( /W) Secure MSC Interrupt Clear */
  __IOM  uint32_t SECMSCINTEN;              /* Offset: 0x038 (R/W) Secure MSC Interrupt Enable */
         uint32_t RESERVED3[1U];
  __IM   uint32_t BRGINTSTAT;               /* Offset: 0x040 (R/ ) Bridge Buffer Error Interrupt Status */
  __OM   uint32_t BRGINTCLR;                /* Offset: 0x044 ( /W) Bridge Buffer Error Interrupt Clear */
  __IOM  uint32_t BRGINTEN;                 /* Offset: 0x048 (R/W) Bridge Buffer Error Interrupt Enable */
         uint32_t RESERVED4[1U];
  __IOM  uint32_t AHBNSPPC[4U];             /* Offset: 0x050 (R/W) Non-Secure Access AHB slave Peripheral Protection Control */
  __IOM  uint32_t AHBNSPPCEXP[4U];          /* Offset: 0x060 (R/W) Expansion Non_Secure Access AHB slave Peripheral Protection Control */
  __IOM  uint32_t APBNSPPC[4U];             /* Offset: 0x070 (R/W) Non-Secure Access APB slave Peripheral Protection Control */
  __IOM  uint32_t APBNSPPCEXP[4U];          /* Offset: 0x080 (R/W) Expansion Non_Secure Access APB slave Peripheral Protection Control */
  __IM   uint32_t AHBSPPPC[4U];             /* Offset: 0x090 (R/ ) Secure Unprivileged Access AHB slave Peripheral Protection Control */
  __IOM  uint32_t AHBSPPPCEXP[4U];          /* Offset: 0x0A0 (R/W) Expansion Secure Unprivileged Access AHB slave Peripheral Protection Control */
  __IOM  uint32_t APBSPPPC[4U];             /* Offset: 0x0B0 (R/W) Secure Unprivileged Access APB slave Peripheral Protection Control */
  __IOM  uint32_t APBSPPPCEXP[4U];          /* Offset: 0x0C0 (R/W) Expansion Secure Unprivileged Access APB slave Peripheral Protection Control */
  __IM   uint32_t NSMSCEXP;                 /* Offset: 0x0D0 (R/ ) Expansion MSC Non-Secure Configuration */
} SPC_TypeDef;


/*------------------- Non-Secure Privilege Control Block -----------------------------*/
typedef struct /* see "ARM CoreLink SSE-200 Subsystem Technical Reference Manual r1p0" */
{
         uint32_t RESERVED0[36U];
  __IOM  uint32_t AHBNSPPPC[4U];            /* Offset: 0x090 (R/W) 0x0000_0000 Non-Secure Unprivileged Access AHB slave Peripheral Protection Control */
  __IOM  uint32_t AHBNSPPPCEXP[4U];         /* Offset: 0x0A0 (R/W) 0x0000_0000 Expansion 0 Non-Secure Unprivileged Access AHB slave Peripheral Protection Control */
  __IOM  uint32_t APBNSPPPC[4U];            /* Offset: 0x0B0 (R/W) 0x0000_0000 Non-Secure Unprivileged Access APB slave Peripheral Protection Control */
  __IOM  uint32_t APBNSPPPCEXP[4U];         /* Offset: 0x0C0 (R/W) 0x0000_0000 Expansion 0 Non-Secure Unprivileged Access APB slave Peripheral Protection Control */
} NSPC_TypeDef;


/*------------------------ Arm Power Policy Unit (PPU) -------------------------------*/
typedef struct /* see "Arm Power Policy Unit Version 1.1 Architecture Specification" */
{
  __IOM  uint32_t PWPR;                     /* Offset: 0x000 (R/W) Power Policy Register  */
  __IOM  uint32_t PMER;                     /* Offset: 0x004 (R/W) Power Mode Emulation Register */
  __IM   uint32_t PWSR;                     /* Offset: 0x008 (R/ ) Power Status Register */
         uint32_t RESERVED0[1U];
  __IM   uint32_t DISR;                     /* Offset: 0x010 (R/ ) Device Interface Input Current Status Register */
  __IM   uint32_t MISR;                     /* Offset: 0x014 (R/ ) Miscellaneous Input Current Status Register */
  __IM   uint32_t STSR;                     /* Offset: 0x018 (R/ ) Stored Status Register */
  __IOM  uint32_t UNLK;                     /* Offset: 0x01C (R/W) Unlock Register */
  __IOM  uint32_t PWCR;                     /* Offset: 0x020 (R/W) Power Configuration Register */
  __IOM  uint32_t PTCR;                     /* Offset: 0x024 (R/W) Power Mode Transition Configuration Register */
  __IOM  uint32_t IMR;                      /* Offset: 0x030 (R/W) Interrupt Mask Register */
  __IOM  uint32_t AIMR;                     /* Offset: 0x034 (R/W) Additional Interrupt Mask Register */
  __IOM  uint32_t ISR;                      /* Offset: 0x038 (R/W) Interrupt Status Register */
  __IOM  uint32_t AISR;                     /* Offset: 0x03C (R/W) Additional Interrupt Status Register */
  __IOM  uint32_t IESR;                     /* Offset: 0x040 (R/W) Input Edge Sensitivity Register */
  __IOM  uint32_t OPSR;                     /* Offset: 0x044 (R/W) Operating Mode Active Edge Sensitivity Register */
  __IOM  uint32_t FUNRR;                    /* Offset: 0x050 (R/W) Functional Retention RAM Configuration Register */
  __IOM  uint32_t FULRR;                    /* Offset: 0x054 (R/W) Full Retention RAM Configuration Register */
  __IOM  uint32_t MEMRR;                    /* Offset: 0x058 (R/W) Memory Retention RAM Configuration Register */
  __IOM  uint32_t EDTR0;                    /* Offset: 0x160 (R/W) Power Mode Entry Delay Register 0 */
  __IOM  uint32_t EDTR1;                    /* Offset: 0x164 (R/W) Power Mode Entry Delay Register 1 */
  __IOM  uint32_t DCDR0;                    /* Offset: 0x170 (R/W) Device Control Delay Configuration Register 0 */
  __IOM  uint32_t DCDR1;                    /* Offset: 0x174 (R/W) Device Control Delay Configuration Register 1 */
         uint32_t RESERVED1[910U];
  __IM   uint32_t IDR0;                     /* Offset: 0xFB0 (R/ ) PPU Identification Register 0 */
  __IM   uint32_t IDR1;                     /* Offset: 0xFB4 (R/ ) PPU Identification Register 1 */
  __IM   uint32_t IIDR;                     /* Offset: 0xFC8 (R/ ) Implementation Identification Register */
  __IM   uint32_t AIDR;                     /* Offset: 0xFCC (R/ ) Architecture Identification Register */
} PPU_TypeDef;




/* --------------------  End of section using anonymous unions  ------------------- */
#if   defined (__CC_ARM)
  #pragma pop
#elif defined (__ICCARM__)
  /* leave anonymous unions enabled */
#elif (__ARMCC_VERSION >= 6010050)
  #pragma clang diagnostic pop
#elif defined (__GNUC__)
  /* anonymous unions are enabled by default */
#elif defined (__TMS470__)
  /* anonymous unions are enabled by default */
#elif defined (__TASKING__)
  #pragma warning restore
#elif defined (__CSMC__)
  /* anonymous unions are enabled by default */
#else
  #warning Not supported compiler type
#endif




/* ================================================================================ */
/* ================              Peripheral memory map             ================ */
/* ================================================================================ */

/* --- peripherals (corelink SSE200) */

#define        SYSINFO_BASE      (0x40020000UL)
#define        S32K_TIMER_BASE   (0x4002F000UL)

/* --- peripherals */
#define        TIMER_BASE        (0x4010B000UL)
#define        SCC_BASE          (0x4010C800UL)
#define        UART0_BASE        (0x40101000UL)
#define        UART1_BASE        (0x40102000UL)
#define        SPI0_BASE         (0x40103000UL)
#define        I2C0_BASE         (0x40104000UL)
#define        I2C1_BASE         (0x40105000UL)
#define        I2S0_BASE         (0x40106000UL)
#define        GPIO0_BASE        (0x40110000UL)




/* ================================================================================ */
/* ================          Secure Peripheral memory map          ================ */
/* ================================================================================ */

/* --- peripherals (corelink SEE200) */

#define SECURE_SYSINFO_BASE      (0x50020000UL)
#define SECURE_SYSCTRL_BASE      (0x50021000UL)
#define SECURE_S32K_TIMER_BASE   (0x5002F000UL)

#define        SYS_PPU_BASE      (0x50022000UL)   /* PPU for the PD_SYS domain */
#define        CPU0CORE_PPU_BASE (0x50023000UL)   /* PPU for the PD_CPU0CORE domain */
#define        CPU0DBG_PPU_BASE  (0x50024000UL)   /* PPU for the PD_CPU0DBG domain */
#define        CPU1CORE_PPU_BASE (0x50025000UL)   /* PPU for the PD_CPU1CORE domain */
#define        CPU1DBG_PPU_BASE  (0x50026000UL)   /* PPU for the PD_CPU1DBG domain */
#define        CRYPTO_PPU_BASE   (0x50027000UL)   /* PPU for the PD_CRYPTO domain */
#define        DBG_PPU_BASE      (0x50029000UL)   /* PPU for the PD_DEBUG domain */
#define        RAM0_PPU_BASE     (0x5002A000UL)   /* PPU for the PD_SRAM0 domain */
#define        RAM1_PPU_BASE     (0x5002B000UL)   /* PPU for the PD_SRAM1 domain */
#define        RAM2_PPU_BASE     (0x5002C000UL)   /* PPU for the PD_SRAM2 domain */
#define        RAM3_PPU_BASE     (0x5002D000UL)   /* PPU for the PD_SRAM3 domain */

#define        MPCSSRAM1_BASE    (0x58007000UL)
#define        MPCSSRAM2_BASE    (0x58008000UL)
#define        MPCSSRAM3_BASE    (0x58009000UL)

/* AHB peripherals */
#define SECURE_TIMER_BASE        (0x5010B000UL)
#define SECURE_SCC_BASE          (0x5010C800UL)
#define SECURE_UART0_BASE        (0x50101000UL)
#define SECURE_UART1_BASE        (0x50102000UL)
#define SECURE_SPI0_BASE         (0x50103000UL)
#define SECURE_I2C0_BASE         (0x50104000UL)
#define SECURE_I2C1_BASE         (0x50105000UL)
#define SECURE_I2S0_BASE         (0x50106000UL)
#define SECURE_GPIO0_BASE        (0x50110000UL)



/* ================================================================================ */
/* ================             Peripheral declaration             ================ */
/* ================================================================================ */

#define        SYSINFO       ((SYSINFO_TypeDef    *)        SYSINFO_BASE      )
#define        S32K_TIMER    ((TIMER_TypeDef      *)        S32K_TIMER_BASE   )

#define        TIMER         ((GP_TIMER_TypeDef   *)        TIMER_BASE        )
#define        SCC           ((SCC_TypeDef        *)        SCC_BASE          )
#define        UART0         ((UART_TypeDef       *)        UART0_BASE        )
#define        UART1         ((UART_TypeDef       *)        UART1_BASE        )
#define        SPI0          ((SPI_TypeDef        *)        SPI0_BASE         )
#define        I2C0          ((I2C_TypeDef        *)        I2C0_BASE         )
#define        I2C1          ((I2C_TypeDef        *)        I2C1_BASE         )
#define        I2S0          ((I2S_TypeDef        *)        I2S0_BASE         )
#define        GPIO0         ((GPIO_TypeDef       *)        GPIO0_BASE        )

/* -------- */


#define SECURE_SYSINFO       ((SYSINFO_TypeDef    *) SECURE_SYSINFO_BASE      )
#define SECURE_SYSCTRL       ((SYSCTRL_TypeDef    *) SECURE_SYSCTRL_BASE      )
#define SECURE_S32K_TIMER    ((TIMER_TypeDef      *) SECURE_S32K_TIMER_BASE   )

#define        SYS_PPU       ((PPU_TypeDef        *)        SYS_PPU_BASE      )
#define        CPU0CORE_PPU  ((PPU_TypeDef        *)        CPU0CORE_PPU_BASE )
#define        CPU0DBG_PPU   ((PPU_TypeDef        *)        CPU0DBG_PPU_BASE  )
#define        CPU1CORE_PPU  ((PPU_TypeDef        *)        CPU1CORE_PPU_BASE )
#define        CPU1DBG_PPU   ((PPU_TypeDef        *)        CPU1DBG_PPU_BASE  )
#define        CRYPTO_PPU    ((PPU_TypeDef        *)        CRYPTO_PPU_BASE   )
#define        DBG_PPU       ((PPU_TypeDef        *)        DBG_PPU_BASE      )
#define        RAM0_PPU      ((PPU_TypeDef        *)        RAM0_PPU_BASE     )
#define        RAM1_PPU      ((PPU_TypeDef        *)        RAM1_PPU_BASE     )
#define        RAM2_PPU      ((PPU_TypeDef        *)        RAM2_PPU_BASE     )
#define        RAM3_PPU      ((PPU_TypeDef        *)        RAM3_PPU_BASE     )

#define        MPCSSRAM1     ((MPC_TypeDef        *)        MPCSSRAM1_BASE    )
#define        MPCSSRAM2     ((MPC_TypeDef        *)        MPCSSRAM2_BASE    )
#define        MPCSSRAM3     ((MPC_TypeDef        *)        MPCSSRAM3_BASE    )

#define SECURE_TIMER         ((GP_TIMER_TypeDef   *) SECURE_TIMER_BASE        )
#define SECURE_SCC           ((SCC_TypeDef        *) SECURE_SCC_BASE          )
#define SECURE_UART0         ((UART_TypeDef       *) SECURE_UART0_BASE        )
#define SECURE_UART1         ((UART_TypeDef       *) SECURE_UART1_BASE        )
#define SECURE_SPI0          ((SPI_TypeDef        *) SECURE_SPI0_BASE         )
#define SECURE_I2C0          ((I2C_TypeDef        *) SECURE_I2C0_BASE         )
#define SECURE_I2C1          ((I2C_TypeDef        *) SECURE_I2C1_BASE         )
#define SECURE_I2S0          ((I2S_TypeDef        *) SECURE_I2S0_BASE         )
#define SECURE_GPIO0         ((GPIO_TypeDef       *) SECURE_GPIO0_BASE        )

#ifdef __cplusplus
}
#endif

#endif  /* MUSCA_S1_H */
