/**************************************************************************//**
 * @file     tz_config.c
 * @brief    TrustZone Configuration for Musca-S1
 * @version  V1.0.0
 * @date     25. May 2020
 ******************************************************************************/
/* Copyright (c) 2020 ARM LIMITED

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

/*
//-------- <<< Use Configuration Wizard in Context Menu >>> -----------------
*/


/*
// <h>Memory Configuration
// <i>Each memory area can be split into a secure and a non-secure part, devided by
// <i>the non-secure base address. All memory below this adress is configured to
// <i>secure, all memory above to non-secure.
*/

/*
//     <o>Code SRAM Non-secure base address <0x00000000-0x001FFFFF:4096>
//     <i>Non-secure address range 0x00000000-0x001FFFFF
//     <i>Block size 4KB
*/
#define TZ_CSRAM_NS_BASE  0x00000000UL
#define TZ_CSRAM_BASE     0x00000000UL
#define TZ_CSRAM_LIMIT    0x001FFFFFUL
#define TZ_CSRAM_BSIZE    4096U
#define TZ_CSRAM_BCNT     16U

/*
//     <o>QSPI Flash Non-secure base address <0x00200000-0x021FFFFF:4096>
//     <i>Non-secure address range 0x00200000-0x021FFFFF
//     <i>Block size 4KB
*/
#define TZ_QSPI_NS_BASE   0x00200000UL
#define TZ_QSPI_BASE      0x00200000UL
#define TZ_QSPI_LIMIT     0x021FFFFFUL
#define TZ_QSPI_BSIZE     4096U
#define TZ_QSPI_BCNT      256U


/*
//     <o>eMRAM Non-secure base address <0x0A000000-0x0A1FFFFF:4096>
//     <i>Non-secure address range 0x0A000000-0x0A1FFFFF
//     <i>Block size 4KB
*/
#define TZ_MRAM_NS_BASE  0x0A120000UL
#define TZ_MRAM_BASE     0x0A000000UL
#define TZ_MRAM_LIMIT    0x0A1FFFFFUL
#define TZ_MRAM_BSIZE    4096U
#define TZ_MRAM_BCNT     16U

/*
//     <o>SRAM Non-secure base address <0x20000000-0x2007FFFF:256>
//     <i>Non-secure address range 0x20000000-0x2007FFFF
//     <i>Block size 256 Bytes
*/
#define TZ_SRAM_NS_BASE  0x20020000UL
#define TZ_SRAM_BASE_(N) (0x20000000UL+(N*0x20000UL))
#define TZ_SRAM_LIMIT    0x2007FFFFUL
#define TZ_SRAM_BSIZE    256U
#define TZ_SRAM_BCNT     16U


/*
//     <o>Veneer region base address <0x0-0xFFFFFFFF:32>
//     <i>The veneer region must match the secure code area.
*/
#define TZ_VENEER_BASE  0x1A08FCC0

/*
//     <o>Veneer region size <0x0-0x1000:32>
//     <i>The veneer region must match the secure code area.
*/
#define TZ_VENEER_SIZE  0x340


/*
// </h>
*/

/*
// <h>Peripheral Configuration
*/

/*
//     <o>S32K Watchdog
//       <0=>Secure (privileged)
//       <1=>Secure (unprivileged)
//       <2=>Non-secure (privileged)
//       <3=>Non-secure (unprivileged)
//     <i>Select 
*/
#define TZ_S32K_WDOG_CFG 3

/*
//     <o>S32K Timer
//       <0=>Secure (privileged)
//       <1=>Secure (unprivileged)
//       <2=>Non-secure (privileged)
//       <3=>Non-secure (unprivileged)
//     <i>Select 
*/
#define TZ_S32KTIMER_CFG 3

/*
//     <o>Message Handling Unit 0
//       <0=>Secure (privileged)
//       <1=>Secure (unprivileged)
//       <2=>Non-secure (privileged)
//       <3=>Non-secure (unprivileged)
//     <i>Select 
*/
#define TZ_MHU0_CFG 3

/*
//     <o>Message Handling Unit 1
//       <0=>Secure (privileged)
//       <1=>Secure (unprivileged)
//       <2=>Non-secure (privileged)
//       <3=>Non-secure (unprivileged)
//     <i>Select 
*/
#define TZ_MHU1_CFG 3

/*
//     <o>Dual Timer
//       <0=>Secure (privileged)
//       <1=>Secure (unprivileged)
//       <2=>Non-secure (privileged)
//       <3=>Non-secure (unprivileged)
//     <i>Select 
*/
#define TZ_DUALTIMER_CFG 3

/*
//     <o>Timer 0
//       <0=>Secure (privileged)
//       <1=>Secure (unprivileged)
//       <2=>Non-secure (privileged)
//       <3=>Non-secure (unprivileged)
//     <i>Select 
*/
#define TZ_TIMER0_CFG 3

/*
//     <o>Timer 1
//       <0=>Secure (privileged)
//       <1=>Secure (unprivileged)
//       <2=>Non-secure (privileged)
//       <3=>Non-secure (unprivileged)
//     <i>Select 
*/
#define TZ_TIMER1_CFG 3

/*
//     <o>General Purpose Timer
//       <0=>Secure (privileged)
//       <1=>Secure (unprivileged)
//       <2=>Non-secure (privileged)
//       <3=>Non-secure (unprivileged)
//     <i>Select 
*/
#define TZ_GPTIMER_CFG 3

/*
//     <o>General purpose I/O
//       <0=>Secure (privileged)
//       <1=>Secure (unprivileged)
//       <2=>Non-secure (privileged)
//       <3=>Non-secure (unprivileged)
//     <i>Select 
*/
#define TZ_GPIO_CFG 3

/*
//     <o>Puls width modulation 0
//       <0=>Secure (privileged)
//       <1=>Secure (unprivileged)
//       <2=>Non-secure (privileged)
//       <3=>Non-secure (unprivileged)
//     <i>Select 
*/
#define TZ_PWM0_CFG 3

/*
//     <o>Puls width modulation 1
//       <0=>Secure (privileged)
//       <1=>Secure (unprivileged)
//       <2=>Non-secure (privileged)
//       <3=>Non-secure (unprivileged)
//     <i>Select 
*/
#define TZ_PWM1_CFG 3

/*
//     <o>Puls width modulation 2
//       <0=>Secure (privileged)
//       <1=>Secure (unprivileged)
//       <2=>Non-secure (privileged)
//       <3=>Non-secure (unprivileged)
//     <i>Select 
*/
#define TZ_PWM2_CFG 3

/*
//     <o>Real Time Clock
//       <0=>Secure (privileged)
//       <1=>Secure (unprivileged)
//       <2=>Non-secure (privileged)
//       <3=>Non-secure (unprivileged)
//     <i>Select 
*/
#define TZ_RTC_CFG 3

/*
//     <o>Puls width modulation 2
//       <0=>Secure (privileged)
//       <1=>Secure (unprivileged)
//       <2=>Non-secure (privileged)
//       <3=>Non-secure (unprivileged)
//     <i>Select 
*/
#define TZ_PWM2_CFG 3

/*
//     <o>Inter-integrated Circuit 0
//       <0=>Secure (privileged)
//       <1=>Secure (unprivileged)
//       <2=>Non-secure (privileged)
//       <3=>Non-secure (unprivileged)
//     <i>Select 
*/
#define TZ_I2C0_CFG 3

/*
//     <o>Inter-integrated Circuit 1
//       <0=>Secure (privileged)
//       <1=>Secure (unprivileged)
//       <2=>Non-secure (privileged)
//       <3=>Non-secure (unprivileged)
//     <i>Select 
*/
#define TZ_I2C1_CFG 3

/*
//     <o>Inter-integrated Sound
//       <0=>Secure (privileged)
//       <1=>Secure (unprivileged)
//       <2=>Non-secure (privileged)
//       <3=>Non-secure (unprivileged)
//     <i>Select 
*/
#define TZ_I2S_CFG 3

/*
//     <o>Serial Peripheral Interface 0
//       <0=>Secure (privileged)
//       <1=>Secure (unprivileged)
//       <2=>Non-secure (privileged)
//       <3=>Non-secure (unprivileged)
//     <i>Select 
*/
#define TZ_SPI_CFG 3

/*
//     <o>Serial Configuration Controller
//       <0=>Secure (privileged)
//       <1=>Secure (unprivileged)
//       <2=>Non-secure (privileged)
//       <3=>Non-secure (unprivileged)
//     <i>Select 
*/
#define TZ_SCC_CFG 3

/*
//     <o>Universal Asynchronous Receiver Transmitter  0
//       <0=>Secure (privileged)
//       <1=>Secure (unprivileged)
//       <2=>Non-secure (privileged)
//       <3=>Non-secure (unprivileged)
//     <i>Select 
*/
#define TZ_UART0_CFG 3

/*
//     <o>Universal Asynchronous Receiver Transmitter  1
//       <0=>Secure (privileged)
//       <1=>Secure (unprivileged)
//       <2=>Non-secure (privileged)
//       <3=>Non-secure (unprivileged)
//     <i>Select 
*/
#define TZ_UART1_CFG 3

/*
// </h>
*/


/*
//------------------ <<< end of configuration section >>> -------------------
*/

#include "Musca-S1.h"

#define TZ_VENEER_LIMIT (TZ_VENEER_BASE + TZ_VENEER_SIZE - 1)

#define _PRIV(P) ((P) & 1U)
#define _NS(P)  (((P) & 2U) >> 1U)

#define NVIC_ITNS(P, B)    (_NS(P) << B)
#define SPCB_PRIV(P, B)    ((_PRIV(P) &~ _NS(P)) << B)
#define SPCB_NS(P, B)      (_NS(P) << B)
#define NSPCB_PRIV(P, B)   ((_PRIV(P) & _NS(P)) << B)

/**
* Setup Secure Attribution Unit
* using a simple memory layout, i.e. splitted into secure and non-secure parts
* by a single non-secure base address.
*
* Musca-S1 requires 6 regions:
* 1) Non-secure portion of Code SRAM
* 2) Non-secure portion of Flash
* 3) Non-secure portion of eMRAM
* 4) Non-secure portion of SRAM
* 5) Non-secure callable veneer region of secure code
* 6) Non-secure peripheral address space
*/
void TZ_Config_SAU(void)
{
  /* Disable SAU */
  SAU->CTRL = 0U;
  
  /* Configure SAU region 0 - Code SRAM */
  SAU->RNR = 0U;
  SAU->RBAR = TZ_CSRAM_NS_BASE;
  SAU->RLAR = (TZ_CSRAM_LIMIT & SAU_RLAR_LADDR_Msk) | 
               /* Region memory attribute index */
               ((0U << SAU_RLAR_NSC_Pos) & SAU_RLAR_NSC_Msk) |
               /* Enable region */
               ((1U << SAU_RLAR_ENABLE_Pos) & SAU_RLAR_ENABLE_Msk);  

  /* Configure SAU region 1 - QSPI Flash */
  SAU->RNR = 1U;
  SAU->RBAR = TZ_QSPI_NS_BASE;
  SAU->RLAR = (TZ_QSPI_LIMIT & SAU_RLAR_LADDR_Msk) | 
               /* Region memory attribute index */
               ((0U << SAU_RLAR_NSC_Pos) & SAU_RLAR_NSC_Msk) |
               /* Enable region */
               ((1U << SAU_RLAR_ENABLE_Pos) & SAU_RLAR_ENABLE_Msk);  

  /* Configure SAU region 2 - eMRAM */
  SAU->RNR = 2U;
  SAU->RBAR = TZ_MRAM_NS_BASE;
  SAU->RLAR = (TZ_MRAM_LIMIT & SAU_RLAR_LADDR_Msk) | 
               /* Region memory attribute index */
               ((0U << SAU_RLAR_NSC_Pos) & SAU_RLAR_NSC_Msk) |
               /* Enable region */
               ((1U << SAU_RLAR_ENABLE_Pos) & SAU_RLAR_ENABLE_Msk);  

  /* Configure SAU region 3 - SRAM */
  SAU->RNR = 3U;
  SAU->RBAR = TZ_SRAM_NS_BASE;
  SAU->RLAR = (TZ_SRAM_LIMIT & SAU_RLAR_LADDR_Msk) | 
               /* Region memory attribute index */
               ((0U << SAU_RLAR_NSC_Pos) & SAU_RLAR_NSC_Msk) |
               /* Enable region */
               ((1U << SAU_RLAR_ENABLE_Pos) & SAU_RLAR_ENABLE_Msk);  

  /* Configure SAU region 4 - Veneer */
  SAU->RNR = 4U;
  SAU->RBAR = (0x10000000 | TZ_VENEER_BASE);
  SAU->RLAR = ((0x10000000 | TZ_VENEER_LIMIT) & SAU_RLAR_LADDR_Msk) | 
               /* Region memory attribute index */
               ((1U << SAU_RLAR_NSC_Pos) & SAU_RLAR_NSC_Msk) |
               /* Enable region */
               ((1U << SAU_RLAR_ENABLE_Pos) & SAU_RLAR_ENABLE_Msk);  

  /* Configure SAU region 5 - non-secure pripherals */
  SAU->RNR = 5U;
  SAU->RBAR = 0x40000000U;
  SAU->RLAR = (0x4FFFFFFFU & SAU_RLAR_LADDR_Msk) | 
               /* Region memory attribute index */
               ((0U << SAU_RLAR_NSC_Pos) & SAU_RLAR_NSC_Msk) |
               /* Enable region */
               ((1U << SAU_RLAR_ENABLE_Pos) & SAU_RLAR_ENABLE_Msk);

  /* Force memory writes before continuing */
  __DSB();
  /* Flush and refill pipeline with updated permissions */
  __ISB();
    
  /* Configure SAU Control */
  SAU->CTRL = ((1U << SAU_CTRL_ENABLE_Pos) & SAU_CTRL_ENABLE_Msk) |         /* enable SAU */
              ((0U << SAU_CTRL_ALLNS_Pos)  & SAU_CTRL_ALLNS_Msk)   ;
}

/** Configure interrupt security assignments
* All interrupts of non-secure peripherals are mapped to non-secure state.
*/
void TZ_Config_NVIC(void)
{
  NVIC->ITNS[0] = 
    3U |                              /* NS Watchdog */
    NVIC_ITNS(TZ_S32KTIMER_CFG,  2U) |
    NVIC_ITNS(TZ_TIMER0_CFG,     3U) |
    NVIC_ITNS(TZ_TIMER1_CFG,     4U) |
    NVIC_ITNS(TZ_DUALTIMER_CFG,  5U) |
    NVIC_ITNS(TZ_MHU0_CFG,       6U) |
    NVIC_ITNS(TZ_MHU1_CFG,       7U);

  NVIC->ITNS[1] = 
    NVIC_ITNS(TZ_GPTIMER_CFG,    1U) |
    NVIC_ITNS(TZ_I2C0_CFG,       2U) |
    NVIC_ITNS(TZ_I2C1_CFG,       3U) |
    NVIC_ITNS(TZ_I2S_CFG,        4U) |
    NVIC_ITNS(TZ_SPI_CFG,        5U) |
    NVIC_ITNS(TZ_UART0_CFG,      7U) |
    NVIC_ITNS(TZ_UART0_CFG,      8U) |
    NVIC_ITNS(TZ_UART0_CFG,      9U) |
    NVIC_ITNS(TZ_UART0_CFG,     10U) |
    NVIC_ITNS(TZ_UART0_CFG,     11U) |
    NVIC_ITNS(TZ_UART0_CFG,     12U) |
    NVIC_ITNS(TZ_UART1_CFG,     13U) |
    NVIC_ITNS(TZ_UART1_CFG,     14U) |
    NVIC_ITNS(TZ_UART1_CFG,     15U) |
    NVIC_ITNS(TZ_UART1_CFG,     16U) |
    NVIC_ITNS(TZ_UART1_CFG,     17U) |
    NVIC_ITNS(TZ_UART1_CFG,     18U) |
    NVIC_ITNS(TZ_GPIO_CFG,      19U) |
    NVIC_ITNS(TZ_GPIO_CFG,      20U) |
    NVIC_ITNS(TZ_GPIO_CFG,      21U) |
    NVIC_ITNS(TZ_GPIO_CFG,      22U) |
    NVIC_ITNS(TZ_GPIO_CFG,      23U) |
    NVIC_ITNS(TZ_GPIO_CFG,      24U) |
    NVIC_ITNS(TZ_GPIO_CFG,      25U) |
    NVIC_ITNS(TZ_GPIO_CFG,      26U) |
    NVIC_ITNS(TZ_GPIO_CFG,      27U) |
    NVIC_ITNS(TZ_GPIO_CFG,      28U) |
    NVIC_ITNS(TZ_GPIO_CFG,      29U) |
    NVIC_ITNS(TZ_GPIO_CFG,      30U) |
    NVIC_ITNS(TZ_GPIO_CFG,      31U);

  NVIC->ITNS[2] = 
    NVIC_ITNS(TZ_GPIO_CFG,       0U) |
    NVIC_ITNS(TZ_GPIO_CFG,       1U) |
    NVIC_ITNS(TZ_GPIO_CFG,       2U) |
    NVIC_ITNS(TZ_GPIO_CFG,       3U) |
    NVIC_ITNS(TZ_PWM0_CFG,       6U) |
    NVIC_ITNS(TZ_RTC_CFG,        7U) |
    NVIC_ITNS(TZ_PWM1_CFG,      10U) |
    NVIC_ITNS(TZ_PWM2_CFG,      11U);
}

/** Helper functiopn to calculate MPC LUT entry.
* @param idx LUT index
* @param ba MPC's memory region base address 
* @param bs MPC's memory block size
* @param la Non-secure portion's base address 
* @return LUT entry for given table index.
*/
__STATIC_FORCEINLINE uint32_t lut_value(uint32_t idx, uint32_t ba, uint32_t bs, uint32_t la) {
  const uint32_t lower = ba+idx*bs*32U;
  const uint32_t upper = lower+bs*32U;
  if (la <= lower) {
    return 0xFFFFFFFFU;
  }
  if (la >= upper) {
    return 0x0U;
  }
  return 0xFFFFFFFFU << ((la - lower) / bs);
}

/** Helper function to initialize MPC LUT.
* @param mpc Pointer to MPC register set
* @param ba MPC's memory region base address 
* @param bs MPC's memory block size
* @param bc MPC's LUT size
* @param la Non-secure portion's base address 
*/
__STATIC_FORCEINLINE void load_mpc(MPC_TypeDef* mpc, uint32_t ba, uint32_t bs, uint32_t bc, uint32_t la) {
  for(uint32_t i=0U; i<bc; i++) {
    mpc->BLK_IDX = i;
  	mpc->BLK_LUT = lut_value(i, ba, bs, la);
  }  
}

/** Configure Memory Protection Controllers
* All MPCs are configured according to the secure/non-secure portions.
*/
void TZ_Config_MPC(void)
{
  /* MPC Code SRAM */
  load_mpc(MPCSCSRAM, TZ_CSRAM_BASE, TZ_CSRAM_BSIZE, TZ_CSRAM_BCNT, TZ_CSRAM_NS_BASE);

  /* MPC SQPI Flash */
  load_mpc(MPCSQSPI, TZ_QSPI_BASE, TZ_QSPI_BSIZE, TZ_QSPI_BCNT, TZ_QSPI_NS_BASE);

  /* MPC eMRAM*/
  load_mpc(MPCSEMRAM, TZ_MRAM_BASE, TZ_MRAM_BSIZE, TZ_MRAM_BCNT, TZ_MRAM_NS_BASE);

  /* MPC SRAM*/
  load_mpc(MPCSSRAM0, TZ_SRAM_BASE_(0), TZ_SRAM_BSIZE, TZ_SRAM_BCNT, TZ_SRAM_NS_BASE);
  load_mpc(MPCSSRAM1, TZ_SRAM_BASE_(1), TZ_SRAM_BSIZE, TZ_SRAM_BCNT, TZ_SRAM_NS_BASE);
  load_mpc(MPCSSRAM2, TZ_SRAM_BASE_(2), TZ_SRAM_BSIZE, TZ_SRAM_BCNT, TZ_SRAM_NS_BASE);
  load_mpc(MPCSSRAM3, TZ_SRAM_BASE_(3), TZ_SRAM_BSIZE, TZ_SRAM_BCNT, TZ_SRAM_NS_BASE);
}

/** Configure Peripheral Protection Controllers
* All PPCs are configured according to the peripheral security state allocation.
*/
void TZ_Config_PPC(void)
{
  SPCB->AHBSPPPCEXP[0] =
      SPCB_PRIV(TZ_GPIO_CFG,      1U);

  SPCB->AHBNSPPCEXP[0] =
      SPCB_NS(TZ_GPIO_CFG,        1U);

  NSPCB->AHBNSPPPCEXP[0] =
      NSPCB_PRIV(TZ_GPIO_CFG,     1U);
  
  SPCB->APBSPPPCEXP[1] = 
      SPCB_PRIV(TZ_PWM2_CFG,     13U) |
      SPCB_PRIV(TZ_PWM1_CFG,     12U) |
      SPCB_PRIV(TZ_SCC_CFG,      11U) |
      SPCB_PRIV(TZ_GPTIMER_CFG,  11U) |
      SPCB_PRIV(TZ_RTC_CFG,       7U) |
      SPCB_PRIV(TZ_PWM0_CFG,      6U) |
      SPCB_PRIV(TZ_I2S_CFG,       5U) |
      SPCB_PRIV(TZ_I2C1_CFG,      4U) |
      SPCB_PRIV(TZ_I2C0_CFG,      3U) |
      SPCB_PRIV(TZ_SPI_CFG,       2U) |
      SPCB_PRIV(TZ_UART1_CFG,     1U) |
      SPCB_PRIV(TZ_UART0_CFG,     0U);

  SPCB->APBNSPPCEXP[1] =
      SPCB_NS(TZ_PWM2_CFG,       13U) |
      SPCB_NS(TZ_PWM1_CFG,       12U) |
      SPCB_NS(TZ_SCC_CFG,        11U) |
      SPCB_NS(TZ_GPTIMER_CFG,    11U) |
      SPCB_NS(TZ_RTC_CFG,         7U) |
      SPCB_NS(TZ_PWM0_CFG,        6U) |
      SPCB_NS(TZ_I2S_CFG,         5U) |
      SPCB_NS(TZ_I2C1_CFG,        4U) |
      SPCB_NS(TZ_I2C0_CFG,        3U) |
      SPCB_NS(TZ_SPI_CFG,         2U) |
      SPCB_NS(TZ_UART1_CFG,       1U) |
      SPCB_NS(TZ_UART0_CFG,       0U);

  NSPCB->APBNSPPPCEXP[1] =
      NSPCB_PRIV(TZ_PWM2_CFG,    13U) |
      NSPCB_PRIV(TZ_PWM1_CFG,    12U) |
      NSPCB_PRIV(TZ_SCC_CFG,     11U) |
      NSPCB_PRIV(TZ_GPTIMER_CFG, 11U) |
      NSPCB_PRIV(TZ_RTC_CFG,      7U) |
      NSPCB_PRIV(TZ_PWM0_CFG,     6U) |
      NSPCB_PRIV(TZ_I2S_CFG,      5U) |
      NSPCB_PRIV(TZ_I2C1_CFG,     4U) |
      NSPCB_PRIV(TZ_I2C0_CFG,     3U) |
      NSPCB_PRIV(TZ_SPI_CFG,      2U) |
      NSPCB_PRIV(TZ_UART1_CFG,    1U) |
      NSPCB_PRIV(TZ_UART0_CFG,    0U);
}
