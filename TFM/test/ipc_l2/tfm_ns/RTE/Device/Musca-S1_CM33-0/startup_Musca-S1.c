/**************************************************************************//**
 * @file     startup_Musca-S1.c
 * @brief    CMSIS Core Device Startup File for
 *           Musca-S1 Device
 * @version  V2.0.0
 * @date     22. May 2020
 ******************************************************************************/
/* Copyright (c) 2019-2020 ARM LIMITED

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

#include "Musca-S1.h"

/*----------------------------------------------------------------------------
  External References
 *----------------------------------------------------------------------------*/
extern uint32_t __INITIAL_SP;
extern uint32_t __STACK_LIMIT;

extern __NO_RETURN void __PROGRAM_START(void);

/*----------------------------------------------------------------------------
  Internal References
 *----------------------------------------------------------------------------*/
__NO_RETURN void Reset_Handler  (void);
            void Default_Handler(void);

/*----------------------------------------------------------------------------
  Exception / Interrupt Handler
 *----------------------------------------------------------------------------*/
/* Exceptions */
void NMI_Handler                     (void) __attribute__ ((weak, alias("Default_Handler")));
void HardFault_Handler               (void) __attribute__ ((weak));
void MemManage_Handler               (void) __attribute__ ((weak, alias("Default_Handler")));
void BusFault_Handler                (void) __attribute__ ((weak, alias("Default_Handler")));
void UsageFault_Handler              (void) __attribute__ ((weak, alias("Default_Handler")));
void SecureFault_Handler             (void) __attribute__ ((weak, alias("Default_Handler")));
void SVC_Handler                     (void) __attribute__ ((weak, alias("Default_Handler")));
void DebugMon_Handler                (void) __attribute__ ((weak, alias("Default_Handler")));
void PendSV_Handler                  (void) __attribute__ ((weak, alias("Default_Handler")));
void SysTick_Handler                 (void) __attribute__ ((weak, alias("Default_Handler")));

/* SSE-200 Interrupts */
void NS_WATCHDOG_RESET_IRQHandler    (void) __attribute__ ((weak, alias("Default_Handler")));
void NS_WATCHDOG_IRQHandler          (void) __attribute__ ((weak, alias("Default_Handler")));
void S32K_TIMER_IRQHandler           (void) __attribute__ ((weak, alias("Default_Handler")));
void TIMER0_IRQHandler               (void) __attribute__ ((weak, alias("Default_Handler")));
void TIMER1_IRQHandler               (void) __attribute__ ((weak, alias("Default_Handler")));
void DUALTIMER_IRQHandler            (void) __attribute__ ((weak, alias("Default_Handler")));
void MHU0_IRQHandler                 (void) __attribute__ ((weak, alias("Default_Handler")));
void MHU1_IRQHandler                 (void) __attribute__ ((weak, alias("Default_Handler")));
void CRYPTOCELL_IRQHandler           (void) __attribute__ ((weak, alias("Default_Handler")));
void MPC_IRQHandler                  (void) __attribute__ ((weak, alias("Default_Handler")));
void PPC_IRQHandler                  (void) __attribute__ ((weak, alias("Default_Handler")));
void S_MSC_COMBINED_IRQHandler       (void) __attribute__ ((weak, alias("Default_Handler")));
void S_BRIDGE_ERR_IRQHandler         (void) __attribute__ ((weak, alias("Default_Handler")));
void I_CACHE_INV_ERR_IRQHandler      (void) __attribute__ ((weak, alias("Default_Handler")));
void SYS_PPU_IRQHandler              (void) __attribute__ ((weak, alias("Default_Handler")));
void CPU0_PPU_IRQHandler             (void) __attribute__ ((weak, alias("Default_Handler")));
void CPU1_PPU_IRQHandler             (void) __attribute__ ((weak, alias("Default_Handler")));
void CPU0_DGB_PPU_IRQHandler         (void) __attribute__ ((weak, alias("Default_Handler")));
void CPU1_DGB_PPU_IRQHandler         (void) __attribute__ ((weak, alias("Default_Handler")));
void CRYPTOCELL_PPU_IRQHandler       (void) __attribute__ ((weak, alias("Default_Handler")));
void RAM0_PPU_IRQHandler             (void) __attribute__ ((weak, alias("Default_Handler")));
void RAM1_PPU_IRQHandler             (void) __attribute__ ((weak, alias("Default_Handler")));
void RAM2_PPU_IRQHandler             (void) __attribute__ ((weak, alias("Default_Handler")));
void RAM3_PPU_IRQHandler             (void) __attribute__ ((weak, alias("Default_Handler")));
void DEBUG_PPU_IRQHandler            (void) __attribute__ ((weak, alias("Default_Handler")));
void CPU0_CTI_IRQHandler             (void) __attribute__ ((weak, alias("Default_Handler")));
void CPU1_CTI_IRQHandler             (void) __attribute__ ((weak, alias("Default_Handler")));

/* Expansion Interrupts */
void GP_TIMER_IRQHandler             (void) __attribute__ ((weak, alias("Default_Handler")));
void I2C0_IRQHandler                 (void) __attribute__ ((weak, alias("Default_Handler")));
void I2C1_IRQHandler                 (void) __attribute__ ((weak, alias("Default_Handler")));
void I2S_IRQHandler                  (void) __attribute__ ((weak, alias("Default_Handler")));
void SPI_IRQHandler                  (void) __attribute__ ((weak, alias("Default_Handler")));
void QSPI_IRQHandler                 (void) __attribute__ ((weak, alias("Default_Handler")));
void UART0RX_IRQHandler              (void) __attribute__ ((weak, alias("Default_Handler")));
void UART0TX_IRQHandler              (void) __attribute__ ((weak, alias("Default_Handler")));
void UART0_RxTimeout_IRQHandler      (void) __attribute__ ((weak, alias("Default_Handler")));
void UART0_ModemStatus_IRQHandler    (void) __attribute__ ((weak, alias("Default_Handler")));
void UART0_Error_IRQHandler          (void) __attribute__ ((weak, alias("Default_Handler")));
void UART0_IRQHandler                (void) __attribute__ ((weak, alias("Default_Handler")));
void UART1RX_IRQHandler              (void) __attribute__ ((weak, alias("Default_Handler")));
void UART1TX_IRQHandler              (void) __attribute__ ((weak, alias("Default_Handler")));
void UART1_RxTimeout_IRQHandler      (void) __attribute__ ((weak, alias("Default_Handler")));
void UART1_ModemStatus_IRQHandler    (void) __attribute__ ((weak, alias("Default_Handler")));
void UART1_Error_IRQHandler          (void) __attribute__ ((weak, alias("Default_Handler")));
void UART1_IRQHandler                (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO_0_IRQHandler               (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO_1_IRQHandler               (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO_2_IRQHandler               (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO_3_IRQHandler               (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO_4_IRQHandler               (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO_5_IRQHandler               (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO_6_IRQHandler               (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO_7_IRQHandler               (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO_8_IRQHandler               (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO_9_IRQHandler               (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO_10_IRQHandler              (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO_11_IRQHandler              (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO_12_IRQHandler              (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO_13_IRQHandler              (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO_14_IRQHandler              (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO_15_IRQHandler              (void) __attribute__ ((weak, alias("Default_Handler")));
void GPIO_COMBINED_IRQHandler        (void) __attribute__ ((weak, alias("Default_Handler")));
void PVT_IRQHandler                  (void) __attribute__ ((weak, alias("Default_Handler")));
void PWM0_IRQHandler                 (void) __attribute__ ((weak, alias("Default_Handler")));
void RTC_IRQHandler                  (void) __attribute__ ((weak, alias("Default_Handler")));
void GP_TIMER0_IRQHandler            (void) __attribute__ ((weak, alias("Default_Handler")));
void GP_TIMER1_IRQHandler            (void) __attribute__ ((weak, alias("Default_Handler")));
void PWM1_IRQHandler                 (void) __attribute__ ((weak, alias("Default_Handler")));
void PWM2_IRQHandler                 (void) __attribute__ ((weak, alias("Default_Handler")));
void IOMUX_IRQHandler                (void) __attribute__ ((weak, alias("Default_Handler")));


/*----------------------------------------------------------------------------
  Exception / Interrupt Vector table
 *----------------------------------------------------------------------------*/
 
#if defined ( __GNUC__ )
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif

extern const VECTOR_TABLE_Type __VECTOR_TABLE[496];
       const VECTOR_TABLE_Type __VECTOR_TABLE[496] __VECTOR_TABLE_ATTRIBUTE = {
  (VECTOR_TABLE_Type)(&__INITIAL_SP),       /*     Initial Stack Pointer */
  Reset_Handler,                            /*     Reset Handler */
  NMI_Handler,                              /*     NMI Handler */
  HardFault_Handler,                        /*     Hard Fault Handler */
  MemManage_Handler,                        /*     MPU Fault Handler */
  BusFault_Handler,                         /*     Bus Fault Handler */
  UsageFault_Handler,                       /*     Usage Fault Handler */
  SecureFault_Handler,                      /*     Secure Fault Handler */
  0,                                        /*     Reserved */
  0,                                        /*     Reserved */
  0,                                        /*     Reserved */
  SVC_Handler,                              /*     SVCall Handler */
  DebugMon_Handler,                         /*     Debug Monitor Handler */
  0,                                        /*     Reserved */
  PendSV_Handler,                           /*     PendSV Handler */
  SysTick_Handler,                          /*     SysTick Handler */

  /* SSE-200 Interrupts */
  NS_WATCHDOG_RESET_IRQHandler,             /*  0: Non-Secure Watchdog Reset Request Interrupt */
  NS_WATCHDOG_IRQHandler,                   /*  1: Non-Secure Watchdog Interrupt */
  S32K_TIMER_IRQHandler,                    /*  2: S32K Timer Interrupt */
  TIMER0_IRQHandler,                        /*  3: CMSDK Timer 0 Interrupt */
  TIMER1_IRQHandler,                        /*  4: CMSDK Timer 1 Interrupt */
  DUALTIMER_IRQHandler,                     /*  5: CMSDK Dual Timer Interrupt */
  MHU0_IRQHandler,                          /*  6: Message Handling Unit 0 Interrupt */
  MHU1_IRQHandler,                          /*  7: Message Handling Unit 1 Interrupt */
  CRYPTOCELL_IRQHandler,                    /*  8: CryptoCell-312 Interrupt */
  MPC_IRQHandler,                           /*  9: Secure Combined MPC Interrupt */
  PPC_IRQHandler,                           /* 10: Secure Combined PPC Interrupt */
  S_MSC_COMBINED_IRQHandler,                /* 11: Secure Combined MSC Interrupt */
  S_BRIDGE_ERR_IRQHandler,                  /* 12: Secure Bridge Error Combined Interrupt */
  I_CACHE_INV_ERR_IRQHandler,               /* 13: Intsruction Cache Invalidation Interrupt */
  0,                                        /* 14: Reserved */
  SYS_PPU_IRQHandler,                       /* 15: System PPU Interrupt */
  CPU0_PPU_IRQHandler,                      /* 16: CPU0 PPU Interrupt */
  CPU1_PPU_IRQHandler,                      /* 17: CPU1 PPU Interrupt */
  CPU0_DGB_PPU_IRQHandler,                  /* 18: CPU0 Debug PPU Interrupt */
  CPU1_DGB_PPU_IRQHandler,                  /* 19: CPU1 Debug PPU Interrupt */
  CRYPTOCELL_PPU_IRQHandler,                /* 20: CryptoCell PPU Interrupt */
  0,                                        /* 21: Reserved */
  RAM0_PPU_IRQHandler,                      /* 22: RAM 0 PPU Interrupt */
  RAM1_PPU_IRQHandler,                      /* 23: RAM 1 PPU Interrupt */
  RAM2_PPU_IRQHandler,                      /* 24: RAM 2 PPU Interrupt */
  RAM3_PPU_IRQHandler,                      /* 25: RAM 3 PPU Interrupt */
  DEBUG_PPU_IRQHandler,                     /* 26: Debug PPU Interrupt */
  0,                                        /* 27: Reserved */
  CPU0_CTI_IRQHandler,                      /* 28: CPU0 CTI Interrupt */
  CPU1_CTI_IRQHandler,                      /* 29: CPU1 CTI Interrupt */
  0,                                        /* 30: Reserved */
  0,                                        /* 31: Reserved */

  /* Expansion Interrupts */
  0,                                        /* 32: Reserved */
  GP_TIMER_IRQHandler,                      /* 33: General Purpose Timer */
  I2C0_IRQHandler,                          /* 34: I2C0 */
  I2C1_IRQHandler,                          /* 35: I2C1 */
  I2S_IRQHandler,                           /* 36: I2S */
  SPI_IRQHandler,                           /* 37: SPI */
  QSPI_IRQHandler,                          /* 38: QSPI */
  UART0RX_IRQHandler,                       /* 39: UART0 receive FIFO interrupt */
  UART0TX_IRQHandler,                       /* 40: UART0 transmit FIFO interrupt */
  UART0_RxTimeout_IRQHandler,               /* 41: UART0 receive timeout interrupt */
  UART0_ModemStatus_IRQHandler,             /* 42: UART0 modem status interrupt */
  UART0_Error_IRQHandler,                   /* 43: UART0 error interrupt */
  UART0_IRQHandler,                         /* 44: UART0 interrupt */
  UART1RX_IRQHandler,                       /* 45: UART0 receive FIFO interrupt */
  UART1TX_IRQHandler,                       /* 46: UART0 transmit FIFO interrupt */
  UART1_RxTimeout_IRQHandler,               /* 47: UART0 receive timeout interrupt */
  UART1_ModemStatus_IRQHandler,             /* 48: UART0 modem status interrupt */
  UART1_Error_IRQHandler,                   /* 49: UART0 error interrupt */
  UART1_IRQHandler,                         /* 50: UART0 interrupt */
  GPIO_0_IRQHandler,                        /* 51: GPIO 0 interrupt */
  GPIO_1_IRQHandler,                        /* 52: GPIO 1 interrupt */
  GPIO_2_IRQHandler,                        /* 53: GPIO 2 interrupt */
  GPIO_3_IRQHandler,                        /* 54: GPIO 3 interrupt */
  GPIO_4_IRQHandler,                        /* 55: GPIO 4 interrupt */
  GPIO_5_IRQHandler,                        /* 56: GPIO 5 interrupt */
  GPIO_6_IRQHandler,                        /* 57: GPIO 6 interrupt */
  GPIO_7_IRQHandler,                        /* 58: GPIO 7 interrupt */
  GPIO_8_IRQHandler,                        /* 59: GPIO 8 interrupt */
  GPIO_9_IRQHandler,                        /* 60: GPIO 9 interrupt */
  GPIO_10_IRQHandler,                       /* 61: GPIO 10 interrupt */
  GPIO_11_IRQHandler,                       /* 62: GPIO 11 interrupt */
  GPIO_12_IRQHandler,                       /* 63: GPIO 12 interrupt */
  GPIO_13_IRQHandler,                       /* 64: GPIO 13 interrupt */
  GPIO_14_IRQHandler,                       /* 65: GPIO 14 interrupt */
  GPIO_15_IRQHandler,                       /* 66: GPIO 15 interrupt */
  GPIO_COMBINED_IRQHandler,                 /* 67: Combined interrupt */
  PVT_IRQHandler,                           /* 68: PVT sensor interrupt */
  0,                                        /* 69: Reserved */
  PWM0_IRQHandler,                          /* 70: PWM0 interrupt */
  RTC_IRQHandler,                           /* 71: RTC interrupt */
  GP_TIMER0_IRQHandler,                     /* 72: General Purpose Timer0 */
  GP_TIMER1_IRQHandler,                     /* 73: General Purpose Timer1 */
  PWM1_IRQHandler,                          /* 74: PWM1 interrupt */
  PWM2_IRQHandler,                          /* 75: PWM2 interrupt */
  IOMUX_IRQHandler                          /* 76: IOMUX interrupt */
};

#if defined ( __GNUC__ )
#pragma GCC diagnostic pop
#endif

/*----------------------------------------------------------------------------
  Reset Handler called on controller reset
 *----------------------------------------------------------------------------*/
__NO_RETURN void Reset_Handler(void)
{
  __set_MSPLIM((uint32_t)(&__STACK_LIMIT));

  SystemInit();                             /* CMSIS System Initialization */
  __PROGRAM_START();                        /* Enter PreMain (C library entry point) */
}


#if defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
  #pragma clang diagnostic push
  #pragma clang diagnostic ignored "-Wmissing-noreturn"
#endif

/*----------------------------------------------------------------------------
  Hard Fault Handler
 *----------------------------------------------------------------------------*/
void HardFault_Handler(void)
{
  while(1);
}

/*----------------------------------------------------------------------------
  Default Handler for Exceptions / Interrupts
 *----------------------------------------------------------------------------*/
void Default_Handler(void) {

  while(1);
}

#if defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
  #pragma clang diagnostic pop
#endif
