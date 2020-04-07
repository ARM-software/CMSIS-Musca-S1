/*----------------------------------------------------------------------------
 * Name:    ulp_wut.c
 * Purpose: Ultra Low Power Wake Up Timer
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

#include "ulp_wut.h"

/*============================================================================
  Interrupt Vector Handling
 *============================================================================*/
typedef void( *pFunc )( void );

extern const pFunc __Vectors[];


/*----------------------------------------------------------------------------
  Interrupt Vector table
 *----------------------------------------------------------------------------*/
extern pFunc sram0_Vectors[76 + 16];       /* 92 */
       pFunc sram0_Vectors[76 + 16]  __attribute__ ((section ("RAM_VECTORS")));


/*----------------------------------------------------------------------------
  Copy Interrupt Vectors from ROM to RAM
 *----------------------------------------------------------------------------*/
static void sram0_copyVectors (void) {
  uint32_t i;

  for (i = 0; i < (76 + 16); i++) {
    sram0_Vectors[i] = __Vectors[i];
  }
}

/*----------------------------------------------------------------------------
  Configure VTOR
 *----------------------------------------------------------------------------*/
static void sram0_configVTOR(uint32_t sel) {

  if (sel == 1U) {
    SCB->VTOR = (uint32_t) sram0_Vectors;
  }
  else {
    SCB->VTOR = (uint32_t) __Vectors;
  }
}

/*============================================================================
  WakeUp timer handling
 *============================================================================*/

static uint32_t s32k_timer_irq;                 /* S32K Timer interrupt happened */
       uint32_t s32k_timer_val;                 /* S32K Timer current value */


/*----------------------------------------------------------------------------
  S32K Timer prepare
 *----------------------------------------------------------------------------*/
void S32K_TIMER_Prepare (void)
{
  sram0_copyVectors();
  sram0_configVTOR(1U);
}


/*----------------------------------------------------------------------------
  S32K Timer start
 *----------------------------------------------------------------------------*/
void S32K_TIMER_Start (uint32_t value)
{
  s32k_timer_irq = 0U;
  s32k_timer_val = value;

  NVIC_EnableIRQ (S32K_TIMER_IRQn);

  SECURE_S32K_TIMER->VALUE  = s32k_timer_val;
  SECURE_S32K_TIMER->RELOAD = s32k_timer_val;
  SECURE_S32K_TIMER->CTRL   = ((1U << 3) |               /* enable S32K Timer interrupt */
                               (1U << 0)  );             /* enable S32K Timer */
}

/*----------------------------------------------------------------------------
  S32K Timer ckeck
 *----------------------------------------------------------------------------*/
uint32_t S32K_TIMER_Check (void)
{
  if (s32k_timer_irq == 0U) {                            /* if S32K timer interrupt did not happen */
    SECURE_S32K_TIMER->INTCLEAR = 1U;                    /* clear S32K Timer interrupt */
    SECURE_S32K_TIMER->CTRL = 0U;                        /* disable S32K Timer interrupt */

    NVIC_DisableIRQ(S32K_TIMER_IRQn);
    NVIC_ClearPendingIRQ(S32K_TIMER_IRQn);

    s32k_timer_val = SECURE_S32K_TIMER->RELOAD - SECURE_S32K_TIMER->VALUE;	
  }
  return s32k_timer_val;
}


/*----------------------------------------------------------------------------
  S32K Timer interrup handler
 *----------------------------------------------------------------------------*/
void S32K_TIMER_IRQHandler (void);
void S32K_TIMER_IRQHandler (void)
{
  s32k_timer_irq = 1U;
  s32k_timer_val = SECURE_S32K_TIMER->RELOAD;

  SECURE_S32K_TIMER->INTCLEAR = 1U;                      /* clear S32K Timer interrupt */
  SECURE_S32K_TIMER->CTRL = 0U;                          /* disable S32K Timer interrupt */

  NVIC_DisableIRQ(S32K_TIMER_IRQn);
  NVIC_ClearPendingIRQ(S32K_TIMER_IRQn);

}
