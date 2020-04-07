/*----------------------------------------------------------------------------
 * Name:    ulp_man.h
 * Purpose: Ultra Low Power Management
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
	 
#ifndef ULP_MAN_H_
#define ULP_MAN_H_
#include <stdint.h>

/*----------------------------------------------------------------------------
  Configuration
 *----------------------------------------------------------------------------*/
// #define ULP_NO_MRAM_PWR_OFF
// #define ULP_NO_SRAM_PWR_OFF
// #define ULP_NO_BBGEN
// #define ULP_NO_CLK_SWITCH_TO_X32K
// #define ULP_NO_CPU1_OFF
// #define ULP_NO_PPU_OFF


/*----------------------------------------------------------------------------
  API
 *----------------------------------------------------------------------------*/
extern void Musca_ULP_Entry (void);
extern void Musca_ULP_Exit  (void);
extern uint32_t Musca_ULP_Idle (uint32_t idle_ms);

#endif /* ULP_MAN_H_ */
