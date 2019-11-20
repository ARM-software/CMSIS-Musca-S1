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
 * File:     pwr_man.c
 * Release:  Version 1.0
 * ----------------------------------------------------------------
 *
 */
 
 /*
 * --------Included Headers--------
 */
#ifndef _PWR_MAN_H_
#define _PWR_MAN_H_

#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <stdint.h>

/*----------------------------------------------------------------------------
  Power Management Defines
 *----------------------------------------------------------------------------*/
#define REFCLK_FREQ_HZ              32768UL
#define FASTCLK_FREQ_HZ          32000000UL
#define PLL_DEFAULT_FREQ_HZ     200000000UL

#define PLL_OFF                 0UL



/*----------------------------------------------------------------------------
 * Type defs
 *----------------------------------------------------------------------------
 */

 typedef enum {
	LP_PLL  = 0,
	REFCLK  = 1,
	FASTCLK = 2,
	INT_PLL = 3
	
} MAINCLK_Src_TypeDef;


typedef enum {
	CPU_OFF   = 0,
	CPU_ON    = 1,
	CPU_SLEEP = 2,
	CPU_WAIT  = 3
} CPU_Mode_TypeDef;
 
typedef struct {
	uint8_t		SRAM_RELOAD;
	uint32_t	SRAM_CELLS_ON;		// SRAM Cells to turn at Power up
	uint32_t	SRAM_CELLS_ULP;		// SRAM Cells to turn OFF in ULP Mode
	
	uint8_t		MRAM_FAST_ON;
	uint32_t	MRAM_CELLS_ON;		// MRAM Cells to keep ON at power up
	uint32_t	MRAM_CELLS_ULP;		// MRAM Cells to turn OFF in ULP Mode
	
	MAINCLK_Src_TypeDef	MAINCLK_ON;
	MAINCLK_Src_TypeDef	MAINCLK_ULP;
	uint32_t  PLL_FREQ_ON;
	uint32_t  PLL_FREQ_ULP;
	
	uint8_t	  BBGEN_ON;
	uint8_t	  BBGEN_ULP;	
		
	CPU_Mode_TypeDef	CPU0_ON;
	CPU_Mode_TypeDef	CPU0_ULP;	
	
  CPU_Mode_TypeDef	CPU1_ON;
	CPU_Mode_TypeDef	CPU1_ULP;

	void (*ulp_entry_callback)(void);
	void (*ulp_callback)(void *);	
	void (*ulp_exit_callback)(void);
	
} Power_Modes_TypeDef;


typedef void (*Ulp_func_ptr_Typedef)(Power_Modes_TypeDef *);


/*----------------------------------------------------------------------------
  Ultra-Low-Power Configuration
 *----------------------------------------------------------------------------*/

//#define ULP_NO_CLK_SWITCH_TO_X32K
//#define ULP_NO_MRAM_PWR_OFF
//#define ULP_NO_SRAM_PWR_OFF
//#define ULP_NO_BBGEN
//#define ULP_NO_CPU1_OFF
//#define ULP_NO_PPU_OFF

/*----------------------------------------------------------------------------
  API
 *----------------------------------------------------------------------------*/
extern  void Musca_ULP_Entry (void);
extern  void Musca_ULP_Exit (void);


#endif // _PWR_MAN_H_
