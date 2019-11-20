/*----------------------------------------------------------------------------
 * Name:    Blinky.c
 * Purpose: LED Flasher
 * Note(s): 
 *----------------------------------------------------------------------------*/

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

#include <stdio.h>
#include <math.h>

#include "RTE_Components.h"             // Component selection
#include  CMSIS_device_header           // Device header

#include "pwr_man.h"


/*----------------------------------------------------------------------------
  Support functions
 *---------------------------------------------------------------------------*/
static void apSleepUs(uint32_t val_us);
static void apSleep(uint32_t val);

static void apSleepUs(uint32_t val) {
	for(uint32_t _i = val; _i>0; _i--);
}

static void apSleep(uint32_t val) {
	apSleepUs(val * 1000UL);
}

/*----------------------------------------------------------------------------
  Musca-S1 Low-Power Entry
 *---------------------------------------------------------------------------*/
#define S32K_CLOCK   (32768U)    /* 1 second */


static volatile uint32_t s32k_timer_irq;                 /* S32K Timer interrupt happened */
static volatile uint32_t s32k_timer_val;                 /* S32K Timer current value */

void Musca_ULP_Entry (void)
{

		/* Maram power-off */
	#ifndef ULP_NO_MRAM_PWR_OFF
		SECURE_SCC->SCC_MRAM_CTRL1 =  (0x3fUL<<24)  | (0xffffff); 	/* MRAM Stop in DirectAccess mode */
		SECURE_SCC->SCC_MRAM_CTRL0 |= 1UL<<31;											/* Activate DirectAccess (DA) mode*/

		SECURE_SCC->SCC_MRAM_CTRL0 |= (0xaUL<<8); 		/* Power gate VDD_18 first */
		SECURE_SCC->SCC_MRAM_CTRL0 |= (0x5UL<<8); 		/* Power gate VDD after 1us */
	#endif
		

	#ifndef ULP_NO_SRAM_PWR_OFF
		SECURE_SCC->SRAM_CTRL =  0xfffffff0UL; /* Power down SRAM cells except 4 cells = 256KByte for wakeup function */
	#endif  
		
	#ifndef ULP_NO_BBGEN	
		SECURE_SCC->CLK_CTRL_SEL |=  (1UL<<4) ;  /* BBGen Switch to Fast clk for CP boot */
		SECURE_SCC->SPARE_CTRL0 = 0xC7UL;        /* BBBGen Voltage VBBP and VBBN */
		SECURE_SCC->CLK_CTRL_ENABLE |= 1UL<<11;  /* Enable BBGen Clock */
		apSleep(10UL);   							   /* Wait for CP settle */
		SECURE_SCC->CLK_CTRL_SEL &=  ~(1UL<<4) ;  /* Switch back to Slow clock because fast clock will stop */
	#endif 
		
		/* Siwtch main clock to clock selected for ULP mode */
	#ifndef ULP_NO_CLK_SWITCH_TO_X32K
		SECURE_SCC->CLK_CTRL_SEL |=  (1UL<<2) ; /* Clock Pre_mux set X32K */
    SECURE_SCC->CLK_CTRL_SEL &= ~(1UL<<0) ; /* MAINCLK set to Selected Input Clock X32K */
		SECURE_SYSCTRL->SYSCLK_DIV = 0x00000000; /* FCLKDIV set to div by 1 (optional) */
		SystemCoreClock = X32K_CLOCK; 
		SECURE_SCC->PLL_CTRL_PLL0_CLK |= (1UL<<31);  /* Switch off LP_PLL */
		SECURE_SCC->PLL_CTRL_PLL0_CLK |= (1UL<<30);  /* Switch off INT_PLL */
	#endif 


		/* turn OFF CPU1 */
	#ifndef ULP_NO_CPU1_OFF
		SECURE_SYSCTRL->CPUWAIT = 0x00000002; // CPU1 Shall wait at Boot stage
		CPU1CORE_PPU->PWPR    = 0x00000100; // Turn off CPU1 PPU dynamically
	#endif


	#ifndef ULP_NO_PPU_OFF
		CRYPTO_PPU->PWPR   = 0x00000000; // turn off crypto (static)
		RAM0_PPU->PWPR = 0x00000100; // Turn off SRAM1 PPU dynamically
		RAM1_PPU->PWPR = 0x00000100; // Turn off SRAM2 PPU dynamically
		RAM2_PPU->PWPR = 0x00000100; // Turn off SRAM3 PPU dynamically
		RAM3_PPU->PWPR = 0x00000102; // SRAM0 should change MEM_RET instead because it has the stack and vtor
	#endif


  /* Last steps to ULP */
  s32k_timer_irq = 0U;
  s32k_timer_val = (S32K_CLOCK) / 2;

//  NVIC_EnableIRQ (S32K_TIMER_IRQn);  	 /* Set Wakeup source */
//  
//  SECURE_S32K_TIMER->RELOAD = s32k_timer_val;
//  SECURE_S32K_TIMER->VALUE  = s32k_timer_val;
//  SECURE_S32K_TIMER->CTRL   = ((1U << 3) |               /* enable S32K Timer interrupt */
//                               (1U << 0)  );             /* enable S32K Timer */
  
  // SCB->SCR |= (SCB_SCR_SLEEPDEEP_Msk);                   /* Setting the sleep deep bit */
	// __WFI();																							 /* CPU  Goes to deepsleep     */

  apSleepUs(500UL);
	Musca_ULP_Exit();
}


/*----------------------------------------------------------------------------
  Musca-S1 Low-Power Exit
 *---------------------------------------------------------------------------*/
void Musca_ULP_Exit (void)
{
	
  #ifndef ULP_NO_PPU_OFF
		CPU0CORE_PPU->PWPR   = 0x00000108;  /* turn ON CORE ppu  */
		RAM0_PPU->PWPR    = 0x00000108; /* Turn on SRAM1 PPU dynamically  */
		RAM1_PPU->PWPR    = 0x00000108; /* Turn on SRAM2 PPU dynamically  */
		RAM2_PPU->PWPR    = 0x00000108; /* Turn on SRAM3 PPU dynamically  */
		RAM3_PPU->PWPR    = 0x00000108; /* Turn on SRAM0 PPU dynamically  */
		SYS_PPU->PWPR     = 0x00000108; /* Turn on BASE PPU dynamically   */
		CRYPTO_PPU->PWPR  = 0x00000108; /* Turn on Crypto PPU dynamically */
  #endif

  #ifndef ULP_NO_CLK_SWITCH_TO_X32K
		SECURE_SCC->PLL_CTRL_PLL0_CLK &= ~(1UL<<31); /* Power-ON and Enable LP_PLL */
		apSleepUs(200UL);   /* Wait for PLL Lock */
		SECURE_SYSCTRL->SYSCLK_DIV = 0x00000003; /* Div by 4 (set back)    */
		SECURE_SCC->CLK_CTRL_SEL &= ~(1UL<<2) ;  /* MAINCLK set LP_PLL Out */
		SECURE_SCC->CLK_CTRL_SEL &= ~(1UL<<12); 
	
	  SystemCoreClock = SYS_CLOCK; 
  #endif
	
	#ifndef ULP_NO_BBGEN	
		SECURE_SCC->SPARE_CTRL0 = 0x0UL;  /* BBGen Power-OFF */
  #endif
	
	#ifndef ULP_NO_SRAM_PWR_OFF
		SECURE_SCC->SRAM_CTRL = 0x00000000UL; /* Code SRAM Cells Power-ON  */
  #endif
	
	#ifndef ULP_NO_MRAM_PWR_OFF
		SECURE_SCC->SCC_MRAM_CTRL0 &= ~(0x5UL<<8); /* Power ON MRAM VDD after VDDCOre */
		apSleep(5UL);   /* Wait for 5us */
		SECURE_SCC->SCC_MRAM_CTRL0 &= ~(0xaUL<<8); /* Power gate VDD_18 first 1us after stop mode */
		SECURE_SCC->SCC_MRAM_CTRL0 &= ~(1UL<<31); /* Disabling DA will make the FSM take MRAM thorugh init phase */
  #endif

	
  if (s32k_timer_irq == 0U) {

    SECURE_S32K_TIMER->INTCLEAR = 1U;                    /* clear S32K Timer interrupt */
    SECURE_S32K_TIMER->CTRL = 0U;                        /* disable S32K Timer interrupt */

    NVIC_DisableIRQ(S32K_TIMER_IRQn);
    NVIC_ClearPendingIRQ(S32K_TIMER_IRQn);

    s32k_timer_val = SECURE_S32K_TIMER->RELOAD - SECURE_S32K_TIMER->VALUE;
  }
	
}


/*----------------------------------------------------------------------------
  Musca-S1 Wakeup Event Handler
 *---------------------------------------------------------------------------*/
void S32K_TIMER_IRQHandler (void);
void S32K_TIMER_IRQHandler (void)
{
  s32k_timer_irq = 1U;
  s32k_timer_val = SECURE_S32K_TIMER->VALUE;

  SECURE_S32K_TIMER->INTCLEAR = 1U;                      /* clear S32K Timer interrupt */
  SECURE_S32K_TIMER->CTRL = 0U;                          /* disable S32K Timer interrupt */

  NVIC_DisableIRQ(S32K_TIMER_IRQn);
  NVIC_ClearPendingIRQ(S32K_TIMER_IRQn);
	
	Musca_ULP_Exit();
}
 

