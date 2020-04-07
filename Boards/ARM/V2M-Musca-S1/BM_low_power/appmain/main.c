/*----------------------------------------------------------------------------
 * Name:    main.c
 * Purpose: Bare-metal Low Power Blinky
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

/*
 * --------Included Headers--------
 */

#include "ulp_man.h"
#include "ulp_wut.h"
#include "Board_LED.h" 


#include "RTE_Components.h"             // Component selection
#include  CMSIS_device_header           // Device header

#ifdef RTE_Compiler_EventRecorder
#include "EventRecorder.h"
#endif


extern uint32_t SystemCoreClock;

/*----------------------------------------------------------------------------
Some active code on SRAM
 *---------------------------------------------------------------------------*/
static void apCount_sram(void) __attribute__((section("sram1")));
static void apCount_sram(void) {

  /* Some active wait. Use _asm to avoid compiler optimization*/
  for(uint32_t _i = 0UL; _i<1000000UL; _i++){
    __asm volatile (""); 
  }
}

/*----------------------------------------------------------------------------
  Some active code on MRAM
 *---------------------------------------------------------------------------*/
static void apCount_mram(void) __attribute__((section("mram1")));
static void apCount_mram (void) {
	
  /* Some active wait. Use _asm to avoid compiler optimization*/
  for(uint32_t _i = 0UL; _i<1000000UL; _i++){
    __asm volatile (""); 
  }
}

/*----------------------------------------------------------------------------
  CoreClock configuration
 *---------------------------------------------------------------------------*/
static void SystemCoreClockConfig(void) {

}

/*----------------------------------------------------------------------------
  main function
 *---------------------------------------------------------------------------*/
int main(void)
{
  SystemCoreClockConfig();  /* Configure the System Clock */
  SystemCoreClockUpdate();  /* Update System Core Clock info */
	
  LED_Initialize();
  S32K_TIMER_Prepare();   
	
#ifdef RTE_Compiler_EventRecorder
  EventRecorderInitialize (EventRecordAll, 1);
#endif 	
	
  while(1)
  {
    LED_SetOut(1);        /* Red LED */
    apCount_sram();       /* Count in SRAM */

    LED_SetOut(2);        /* Green LED */
    apCount_mram();	  /* Count in MRAM */
		
    LED_SetOut(3);	  /* Yellow LED */
    Musca_ULP_Idle(500);  /* Inactive ULP Mode */
  }
}
