/*----------------------------------------------------------------------------
 * Name:    ulp_man.c
 * Purpose: Ultra Low Power Management
 *          Code and data placed in SRAM3 (see scatter file)
 *          SRAM3 is active during Ultra Low Power down.
 *----------------------------------------------------------------------------*/
/* Copyright (c) 2020 Arm Ltd

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
	 
#include "RTE_Components.h"             // Component selection
#include  CMSIS_device_header           // Device header

#include "ulp_man.h"
#include "ulp_wut.h"
/*----------------------------------------------------------------------------
  Support functions
 *---------------------------------------------------------------------------*/
static void apSleepUs(uint32_t val_us);
static void apSleep(uint32_t val);

static void apSleepUs(uint32_t val) {
  for (uint32_t _i = val; _i > 0; _i--){
    __asm volatile (""); 
  }
}

static void apSleep(uint32_t val) {
  apSleepUs(val * 1000UL);
}


/*============================================================================
  Ultra Low Power Handling
 *============================================================================*/
#define osPS_sz (64U)
static uint32_t osPSP_cur;              /* current PSP */
static uint32_t osPSPLIM_cur;           /* current PSPLIM */
static uint32_t osPS_tmp[osPS_sz];      /* new Process stack */

/*----------------------------------------------------------------------------
  Put into sleep mode for idle_ms
 *----------------------------------------------------------------------------*/
uint32_t Musca_ULP_Idle (uint32_t idle_ms)
{
    uint32_t actual_idle_ms; 
  
    /* PSP handling */
    osPSP_cur    = __get_PSP();                        /* save the current PSP */
    osPSPLIM_cur = __get_PSPLIM();
    __set_PSP((uint32_t)&osPS_tmp[osPS_sz]);           /* set PSP to ULP RAM */
    __set_PSPLIM((uint32_t)&osPS_tmp[0]);

    uint32_t timer_value = S32KCLOCK*idle_ms/1000; 
    S32K_TIMER_Start(timer_value);                     /* start the wake up timer */

    Musca_ULP_Entry();                                 /* Enter the low power state */

    SCB->SCR |= (SCB_SCR_SLEEPDEEP_Msk);               /* Setting the sleep deep bit */
    __DSB();
    __WFI();                                           /* Wait for S32K timer interrupt */

    __NOP();
     Musca_ULP_Exit();                                 /* Leave the low power state */

    __set_PSP(osPSP_cur);                              /* restore PSP */
    __set_PSPLIM(osPSPLIM_cur);

    actual_idle_ms = (1000*S32K_TIMER_Check())/S32KCLOCK;  /* check if S32K timer interrupt happened */
    return actual_idle_ms;
}


/*----------------------------------------------------------------------------
  Musca Low Power entry function
 *----------------------------------------------------------------------------*/
void Musca_ULP_Entry (void)
{
  /* MRAM power-off */
  #ifndef ULP_NO_MRAM_PWR_OFF
    SECURE_SCC->SCC_MRAM_CTRL1   =   0x3fffffffUL;       /* MRAM Stop in DirectAccess mode */
    SECURE_SCC->SCC_MRAM_CTRL0  |=  (   1UL << 31);      /* Activate DirectAccess (DA) mode */

    SECURE_SCC->SCC_MRAM_CTRL0  |=  (0x0aUL <<  8);      /* Power gate VDD_18 first */
    SECURE_SCC->SCC_MRAM_CTRL0  |=  (0x05UL <<  8);      /* Power gate VDD after 1us */
  #endif

  /* SRAM power-off */
  #ifndef ULP_NO_SRAM_PWR_OFF
    SECURE_SCC->SRAM_CTRL        =   0xffffffffUL;       /* Power down code SRAM cells */
  #endif                                                 /* We use cells 0..3 to hold code and data during Ultra Low Power down */

  /*  */
  #ifndef ULP_NO_PPU_OFF
    CRYPTO_PPU->PWPR = 0x00000000UL;                     /* Turn off crypto (static) */
    RAM0_PPU->PWPR   = 0x00000100UL;                     /* Turn off SRAM0 PPU dynamically */
    RAM1_PPU->PWPR   = 0x00000100UL;                     /* Turn off SRAM1 PPU dynamically */
    RAM2_PPU->PWPR   = 0x00000100UL;                     /* Turn off SRAM2 PPU dynamically */
//  RAM3_PPU->PWPR   = 0x00000102UL;                     /* Turn off SRAM3 PPU dynamically */ /* SRAM3 used as ULP RAM */
  #endif

  /*  */
  #ifndef ULP_NO_BBGEN
    SECURE_SCC->CLK_CTRL_SEL    |=  (   1UL <<  4);      /* BBGen Switch to Fast clk for CP boot */
    SECURE_SCC->SPARE_CTRL0      =   0x000000c7UL;       /* BBBGen Voltage VBBP and VBBN */
    SECURE_SCC->CLK_CTRL_ENABLE |=  (   1UL << 11);      /* Enable BBGen Clock */
    apSleep(10UL);                                       /* Wait for CP settle */
    SECURE_SCC->CLK_CTRL_SEL    &= ~(   1UL <<  4);      /* Switch back to Slow clock because fast clock will stop */
  #endif

  /* Switch main clock to clock selected for ULP mode */
  #ifndef ULP_NO_CLK_SWITCH_TO_X32K
    SECURE_SCC->CLK_CTRL_SEL      &= ~(1UL <<  0);       /* ctrl_sel_pre_mux_clk set to X32K */
    SECURE_SCC->CLK_CTRL_SEL      |=  (1UL <<  2);       /* ctrl_sel_main_mux_clk set to pre_mux_clk */
    while ((SECURE_SCC->CLK_STATUS & (1U << 0)) != (1U << 0)) __NOP();

    SECURE_SYSCTRL->SYSCLK_DIV     =   0UL;              /* FCLKDIV set to div by 1 (optional) */
    SECURE_SCC->PLL_CTRL_PLL0_CLK |= ((1UL << 31) |      /* Switch off LP_PLL */
                                      (1UL << 30)  );    /* Switch off INT_PLL */
  #endif

  /* turn OFF CPU1 */
  #ifndef ULP_NO_CPU1_OFF
    SECURE_SYSCTRL->CPUWAIT = 0x00000002U;               /* CPU1 Shall wait at Boot stage */
    CPU1CORE_PPU->PWPR      = 0x00000100U;               /* Turn off CPU1 PPU dynamically */
  #endif

}

/*----------------------------------------------------------------------------
  Musca Low Power exit function
 *----------------------------------------------------------------------------*/
void Musca_ULP_Exit (void)
{
  #ifndef ULP_NO_CLK_SWITCH_TO_X32K
    SECURE_SCC->PLL_CTRL_PLL0_CLK &= ~(1U << 31);        /* Power-ON and enable LP_PLL */
    while ((SECURE_SCC->CLK_STATUS & (1U << 1)) != (1U << 1)) __NOP();

    SECURE_SYSCTRL->SYSCLK_DIV = 3U;                     /* Div by 4 (set back) */

    SECURE_SCC->CLK_CTRL_SEL &= ~(1U << 12);             /* ctrl_pll_mux_clk_sel set to LP PLL */
    SECURE_SCC->CLK_CTRL_SEL &= ~(1U <<  2);             /* ctrl_sel_main_mux_clk set to pll0_clk */
    while ((SECURE_SCC->CLK_STATUS & (1U << 0)) != (1U << 0)) __NOP();
  #endif

  #ifndef ULP_NO_BBGEN
    SECURE_SCC->SPARE_CTRL0 = 0x00000000UL;              /* BBGen Power-OFF */
  #endif

  #ifndef ULP_NO_PPU_OFF
    CPU0CORE_PPU->PWPR = 0x00000108UL;                   /* Turn ON CORE PPU */
    RAM0_PPU->PWPR     = 0x00000108UL;                   /* Turn on SRAM1 PPU dynamically */
    RAM1_PPU->PWPR     = 0x00000108UL;                   /* Turn on SRAM2 PPU dynamically */
    RAM2_PPU->PWPR     = 0x00000108UL;                   /* Turn on SRAM3 PPU dynamically */
    RAM3_PPU->PWPR     = 0x00000108UL;                   /* Turn on SRAM0 PPU dynamically */
    SYS_PPU->PWPR      = 0x00000108UL;                   /* Turn on BASE PPU dynamically */
    CRYPTO_PPU->PWPR   = 0x00000108UL;                   /* Turn on Crypto PPU dynamically */
  #endif

  #ifndef ULP_NO_SRAM_PWR_OFF
    SECURE_SCC->SRAM_CTRL = 0x00000000UL;                /* Code SRAM Cells Power-ON  */
  #endif

  #ifndef ULP_NO_MRAM_PWR_OFF
    SECURE_SCC->SCC_MRAM_CTRL0 &= ~(0x5UL <<  8);        /* Power ON MRAM VDD after VDD Core */
    apSleep(5UL);                                        /* Wait for 5us */
    SECURE_SCC->SCC_MRAM_CTRL0 &= ~(0xaUL <<  8);        /* Power gate VDD_18 first 1us after stop mode */

    SECURE_SCC->SCC_MRAM_CTRL0 &= ~(  1UL << 31);        /* Disabling DA will make the FSM take MRAM through init phase */
  #endif
}
