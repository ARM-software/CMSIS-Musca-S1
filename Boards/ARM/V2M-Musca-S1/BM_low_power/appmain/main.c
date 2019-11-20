/*
 * Copyright:
 * ----------------------------------------------------------------
 * This confidential and proprietary software may be used only as
 * authorised by a licensing agreement from ARM Limited
 *   (C) COPYRIGHT 2017 ARM Limited
 *       ALL RIGHTS RESERVED
 * The entire notice above must be reproduced on all authorised
 * copies and copies may only be made to the extent permitted
 * by a licensing agreement from ARM Limited.
 * ----------------------------------------------------------------
 * File:     main.c
 * Release:  Version 2.0
 * ----------------------------------------------------------------
 *
 */

/*
 * --------Included Headers--------
 */

#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>


#include "pwr_man.h"
#include "Board_LED.h" 


#include "RTE_Components.h"             // Component selection
#include  CMSIS_device_header           // Device header

#ifdef RTE_Compiler_EventRecorder
#include "EventRecorder.h"
#endif


extern uint32_t SystemCoreClock;

static uint32_t  *p_sram_buf = (uint32_t *)0x1003E000;  /*Memory buffer in SRAM */ 
static uint32_t  *p_mram_buf = (uint32_t *)0x1A13E000;  /*Memory buffer in MRAM */ 


static uint32_t  buf[1024]; 

/*----------------------------------------------------------------------------
  Init Buf
 *---------------------------------------------------------------------------*/
static void init_buf(void);
static void init_buf(void){
	for(uint32_t _i = 0UL; _i<1024;_i++) buf[_i] = _i;
}

/*----------------------------------------------------------------------------
  Some active code on SRAM
 *---------------------------------------------------------------------------*/
static void apCount(void);
static void apCount(void) {
	uint32_t err=0;
	memcpy(p_sram_buf,buf, sizeof(buf));
	for(uint32_t _i = 0UL; _i<1024; _i++)if(*(p_sram_buf+_i) != buf[_i]) err++;

	for(uint32_t _i = 0UL; _i<500000UL; _i++);   /* Some active wait */
}

/*----------------------------------------------------------------------------
  Some active code on MRAM
 *---------------------------------------------------------------------------*/
static void apCount_mram(void) __attribute__((section("mram1")));
static void apCount_mram (void) {
	uint32_t err=0;
	memcpy(p_mram_buf,buf, sizeof(buf));
	for(uint32_t _i = 0UL; _i<1024; _i++) if(*(p_mram_buf+_i) != buf[_i]) err++;
	
	for(uint32_t _i = 0UL; _i<500000UL; _i++);  /* Some active wait */
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
  init_buf();
	
	  while(1)
    {			
      LED_SetOut(1);
			apCount();			      /*Count and compare in SRAM */
			LED_SetOut(2);    
 			apCount_mram();			  /*Count and compare in MRAM */
			LED_SetOut(3);
      Musca_ULP_Entry();   	/*Inactive ULP Mode */
		}
}
