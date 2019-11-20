/**************************************************************************//**
 * @file     startup_Musca-S1.S
 * @brief    CMSIS Core Device Startup File for
 *           Musca-S1 Device
 * @version  V1.0.0
 * @date     02. May 2019
 ******************************************************************************/
/*
 * Copyright (c) 2019 Arm Limited. All rights reserved.
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

                .syntax  unified
                .arch    armv8-m.main

                .eabi_attribute Tag_ABI_align_preserved, 1


                .section RESET
                .align   2
                .globl   Image$$ARM_LIB_STACK$$ZI$$Limit    /* Linker symbol from scatter file */
                .globl   Image$$ARM_LIB_STACK$$ZI$$Base    /* Linker symbol from scatter file */

                .globl   __Vectors
                .globl   __Vectors_End
                .globl   __Vectors_Size
__Vectors:
                .long    Image$$ARM_LIB_STACK$$ZI$$Limit    /*     Top of Stack */
                .long    Reset_Handler                      /*     Reset Handler */
                .long    NMI_Handler                        /* -14 NMI Handler */
                .long    HardFault_Handler                  /* -13 Hard Fault Handler */
                .long    MemManage_Handler                  /* -12 MPU Fault Handler */
                .long    BusFault_Handler                   /* -11 Bus Fault Handler */
                .long    UsageFault_Handler                 /* -10 Usage Fault Handler */
                .long    SecureFault_Handler                /*  -9 Secure Fault Handler */
                .long    0                                  /*     Reserved */
                .long    0                                  /*     Reserved */
                .long    0                                  /*     Reserved */
                .long    SVC_Handler                        /*  -5 SVCall Handler */
                .long    DebugMon_Handler                   /*  -4 Debug Monitor Handler */
                .long    0                                  /*     Reserved */
                .long    PendSV_Handler                     /*  -2 PendSV Handler */
                .long    SysTick_Handler                    /*  -1 SysTick Handler */

                /* SSE-200 Interrupts */
                .long    NS_WATCHDOG_RESET_IRQHandler    /*  0: Non-Secure Watchdog Reset Request Interrupt */
                .long    NS_WATCHDOG_IRQHandler          /*  1: Non-Secure Watchdog Interrupt */
                .long    S32K_TIMER_IRQHandler           /*  2: S32K Timer Interrupt */
                .long    TIMER0_IRQHandler               /*  3: CMSDK Timer 0 Interrupt */
                .long    TIMER1_IRQHandler               /*  4: CMSDK Timer 1 Interrupt */
                .long    DUALTIMER_IRQHandler            /*  5: CMSDK Dual Timer Interrupt */
                .long    MHU0_IRQHandler                 /*  6: Message Handling Unit 0 Interrupt */
                .long    MHU1_IRQHandler                 /*  7: Message Handling Unit 1 Interrupt */
                .long    CRYPTOCELL_IRQHandler           /*  8: CryptoCell-312 Interrupt */
                .long    MPC_Handler                     /*  9: Secure Combined MPC Interrupt */
                .long    PPC_Handler                     /* 10: Secure Combined PPC Interrupt */
                .long    S_MSC_COMBINED_IRQHandler       /* 11: Secure Combined MSC Interrupt */
                .long    S_BRIDGE_ERR_IRQHandler         /* 12: Secure Bridge Error Combined Interrupt */
                .long    I_CACHE_INV_ERR_IRQHandler      /* 13: Intsruction Cache Invalidation Interrupt */
                .long    0                               /* 14: Reserved */
                .long    SYS_PPU_IRQHandler              /* 15: System PPU Interrupt */
                .long    CPU0_PPU_IRQHandler             /* 16: CPU0 PPU Interrupt */
                .long    CPU1_PPU_IRQHandler             /* 17: CPU1 PPU Interrupt */
                .long    CPU0_DGB_PPU_IRQHandler         /* 18: CPU0 Debug PPU Interrupt */
                .long    CPU1_DGB_PPU_IRQHandler         /* 19: CPU1 Debug PPU Interrupt */
                .long    CRYPTOCELL_PPU_IRQHandler       /* 20: CryptoCell PPU Interrupt */
                .long    0                               /* 21: Reserved */
                .long    RAM0_PPU_IRQHandler             /* 22: RAM 0 PPU Interrupt */
                .long    RAM1_PPU_IRQHandler             /* 23: RAM 1 PPU Interrupt */
                .long    RAM2_PPU_IRQHandler             /* 24: RAM 2 PPU Interrupt */
                .long    RAM3_PPU_IRQHandler             /* 25: RAM 3 PPU Interrupt */
                .long    DEBUG_PPU_IRQHandler            /* 26: Debug PPU Interrupt */
                .long    0                               /* 27: Reserved */
                .long    CPU0_CTI_IRQHandler             /* 28: CPU0 CTI Interrupt */
                .long    CPU1_CTI_IRQHandler             /* 29: CPU1 CTI Interrupt */
                .long    0                               /* 30: Reserved */
                .long    0                               /* 31: Reserved */

                /* Expansion Interrupts */
                .long    0                               /* 32: Reserved */
                .long    GpTimer_IRQHandler              /* 33: General Purpose Timer */
                .long    I2C0_IRQHandler                 /* 34: I2C0 */
                .long    I2C1_IRQHandler                 /* 35: I2C1 */
                .long    I2S_IRQHandler                  /* 36: I2S */
                .long    SPI_IRQHandler                  /* 37: SPI */
                .long    QSPI_IRQHandler                 /* 38: QSPI */
                .long    UART0RX_Handler                 /* 39: UART0 receive FIFO interrupt */
                .long    UART0TX_Handler                 /* 40: UART0 transmit FIFO interrupt */
                .long    UART0_RxTimeout_IRQHandler      /* 41: UART0 receive timeout interrupt */
                .long    UART0_ModemStatus_IRQHandler    /* 42: UART0 modem status interrupt */
                .long    UART0_Error_IRQHandler          /* 43: UART0 error interrupt */
                .long    UART0_IRQHandler                /* 44: UART0 interrupt */
                .long    UART1RX_Handler                 /* 45: UART0 receive FIFO interrupt */
                .long    UART1TX_Handler                 /* 46: UART0 transmit FIFO interrupt */
                .long    UART1_RxTimeout_IRQHandler      /* 47: UART0 receive timeout interrupt */
                .long    UART1_ModemStatus_IRQHandler    /* 48: UART0 modem status interrupt */
                .long    UART1_Error_IRQHandler          /* 49: UART0 error interrupt */
                .long    UART1_IRQHandler                /* 50: UART0 interrupt */
                .long    GPIO_0_IRQHandler               /* 51: GPIO 0 interrupt */
                .long    GPIO_1_IRQHandler               /* 52: GPIO 1 interrupt */
                .long    GPIO_2_IRQHandler               /* 53: GPIO 2 interrupt */
                .long    GPIO_3_IRQHandler               /* 54: GPIO 3 interrupt */
                .long    GPIO_4_IRQHandler               /* 55: GPIO 4 interrupt */
                .long    GPIO_5_IRQHandler               /* 56: GPIO 5 interrupt */
                .long    GPIO_6_IRQHandler               /* 57: GPIO 6 interrupt */
                .long    GPIO_7_IRQHandler               /* 58: GPIO 7 interrupt */
                .long    GPIO_8_IRQHandler               /* 59: GPIO 8 interrupt */
                .long    GPIO_9_IRQHandler               /* 60: GPIO 9 interrupt */
                .long    GPIO_10_IRQHandler              /* 61: GPIO 10 interrupt */
                .long    GPIO_11_IRQHandler              /* 62: GPIO 11 interrupt */
                .long    GPIO_12_IRQHandler              /* 63: GPIO 12 interrupt */
                .long    GPIO_13_IRQHandler              /* 64: GPIO 13 interrupt */
                .long    GPIO_14_IRQHandler              /* 65: GPIO 14 interrupt */
                .long    GPIO_15_IRQHandler              /* 66: GPIO 15 interrupt */
                .long    Combined_IRQHandler             /* 67: Combined interrupt */
                .long    PVT_IRQHandler                  /* 68: PVT sensor interrupt */
                .long    0                               /* 69: Reserved */
                .long    PWM_0_IRQHandler                /* 70: PWM0 interrupt */
                .long    RTC_IRQHandler                  /* 71: RTC interrupt */
                .long    GpTimer0_IRQHandler             /* 72: General Purpose Timer0 */
                .long    GpTimer1_IRQHandler             /* 73: General Purpose Timer1 */
                .long    PWM_1_IRQHandler                /* 74: PWM1 interrupt */
                .long    PWM_2_IRQHandler                /* 75: PWM2 interrupt */
                .long    IOMUX_IRQHandler                /* 76: IOMUX interrupt */
__Vectors_End:
                .equ     __Vectors_Size, __Vectors_End - __Vectors
                .size    __Vectors, . - __Vectors


                .thumb
                .section .text
                .align   2

                .thumb_func
                .type    Reset_Handler, %function
                .globl   Reset_Handler
                .fnstart
                .cantunwind
Reset_Handler:
                ldr      r0, =Image$$ARM_LIB_STACK$$ZI$$Base
                msr      msplim, r0

                bl       SystemInit
                bl       __main

                .fnend
                .size    Reset_Handler, . - Reset_Handler


                .thumb_func
                .type    Default_Handler, %function
                .weak    Default_Handler
                .fnstart
                .cantunwind
Default_Handler:
                b        .
                .fnend
                .size    Default_Handler, . - Default_Handler

/* Macro to define default exception/interrupt handlers.
 * Default handler are weak symbols with an endless loop.
 * They can be overwritten by real handlers.
 */
                .macro   Set_Default_Handler  Handler_Name
                .weak    \Handler_Name
                .set     \Handler_Name, Default_Handler
                .endm


/* Default exception/interrupt handler */

                Set_Default_Handler  NMI_Handler
                Set_Default_Handler  HardFault_Handler
                Set_Default_Handler  MemManage_Handler
                Set_Default_Handler  BusFault_Handler
                Set_Default_Handler  UsageFault_Handler
                Set_Default_Handler  SecureFault_Handler
                Set_Default_Handler  SVC_Handler
                Set_Default_Handler  DebugMon_Handler
                Set_Default_Handler  PendSV_Handler
                Set_Default_Handler  SysTick_Handler

                /* SSE-200 Interrupts */
                Set_Default_Handler  NS_WATCHDOG_RESET_IRQHandler
                Set_Default_Handler  NS_WATCHDOG_IRQHandler
                Set_Default_Handler  S32K_TIMER_IRQHandler
                Set_Default_Handler  TIMER0_IRQHandler
                Set_Default_Handler  TIMER1_IRQHandler
                Set_Default_Handler  DUALTIMER_IRQHandler
                Set_Default_Handler  MHU0_IRQHandler
                Set_Default_Handler  MHU1_IRQHandler
                Set_Default_Handler  CRYPTOCELL_IRQHandler
                Set_Default_Handler  MPC_Handler
                Set_Default_Handler  PPC_Handler
                Set_Default_Handler  S_MSC_COMBINED_IRQHandler
                Set_Default_Handler  S_BRIDGE_ERR_IRQHandler
                Set_Default_Handler  I_CACHE_INV_ERR_IRQHandler
                Set_Default_Handler  SYS_PPU_IRQHandler
                Set_Default_Handler  CPU0_PPU_IRQHandler
                Set_Default_Handler  CPU1_PPU_IRQHandler
                Set_Default_Handler  CPU0_DGB_PPU_IRQHandler
                Set_Default_Handler  CPU1_DGB_PPU_IRQHandler
                Set_Default_Handler  CRYPTOCELL_PPU_IRQHandler
                Set_Default_Handler  RAM0_PPU_IRQHandler
                Set_Default_Handler  RAM1_PPU_IRQHandler
                Set_Default_Handler  RAM2_PPU_IRQHandler
                Set_Default_Handler  RAM3_PPU_IRQHandler
                Set_Default_Handler  DEBUG_PPU_IRQHandler
                Set_Default_Handler  CPU0_CTI_IRQHandler
                Set_Default_Handler  CPU1_CTI_IRQHandler

                /* Expansion Interrupts */
                Set_Default_Handler  GpTimer_IRQHandler
                Set_Default_Handler  I2C0_IRQHandler
                Set_Default_Handler  I2C1_IRQHandler
                Set_Default_Handler  I2S_IRQHandler
                Set_Default_Handler  SPI_IRQHandler
                Set_Default_Handler  QSPI_IRQHandler
                Set_Default_Handler  UART0RX_Handler
                Set_Default_Handler  UART0TX_Handler
                Set_Default_Handler  UART0_RxTimeout_IRQHandler
                Set_Default_Handler  UART0_ModemStatus_IRQHandler
                Set_Default_Handler  UART0_Error_IRQHandler
                Set_Default_Handler  UART0_IRQHandler
                Set_Default_Handler  UART1RX_Handler
                Set_Default_Handler  UART1TX_Handler
                Set_Default_Handler  UART1_RxTimeout_IRQHandler
                Set_Default_Handler  UART1_ModemStatus_IRQHandler
                Set_Default_Handler  UART1_Error_IRQHandler
                Set_Default_Handler  UART1_IRQHandler
                Set_Default_Handler  GPIO_0_IRQHandler
                Set_Default_Handler  GPIO_1_IRQHandler
                Set_Default_Handler  GPIO_2_IRQHandler
                Set_Default_Handler  GPIO_3_IRQHandler
                Set_Default_Handler  GPIO_4_IRQHandler
                Set_Default_Handler  GPIO_5_IRQHandler
                Set_Default_Handler  GPIO_6_IRQHandler
                Set_Default_Handler  GPIO_7_IRQHandler
                Set_Default_Handler  GPIO_8_IRQHandler
                Set_Default_Handler  GPIO_9_IRQHandler
                Set_Default_Handler  GPIO_10_IRQHandler
                Set_Default_Handler  GPIO_11_IRQHandler
                Set_Default_Handler  GPIO_12_IRQHandler
                Set_Default_Handler  GPIO_13_IRQHandler
                Set_Default_Handler  GPIO_14_IRQHandler
                Set_Default_Handler  GPIO_15_IRQHandler
                Set_Default_Handler  Combined_IRQHandler
                Set_Default_Handler  PVT_IRQHandler
                Set_Default_Handler  PWM_0_IRQHandler
                Set_Default_Handler  RTC_IRQHandler
                Set_Default_Handler  GpTimer0_IRQHandler
                Set_Default_Handler  GpTimer1_IRQHandler
                Set_Default_Handler  PWM_1_IRQHandler
                Set_Default_Handler  PWM_2_IRQHandler
                Set_Default_Handler  IOMUX_IRQHandler

                .end
