/*-----------------------------------------------------------------------------
 * Name:    LED_V2M-Musca-S1.c
 * Purpose: LED interface for ARM V2M-Musca-S1 evaluation board
 * Rev.:    1.0.0
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

#include "RTE_Components.h"             // Component selection
#include  CMSIS_device_header           // Device header
#include "Board_LED.h"                  // ::Board Support:LED

#define LED_NUM  (3)                    /* Number of available LEDs */

#define GPIO_PIN_RESET       ( 0U )
#define GPIO_PIN_SET         ( 1U )

typedef struct {
  GPIO_TypeDef *port;
  uint32_t      pin;
  uint32_t      active_state;
} PIN_CONFIG_t;

static const PIN_CONFIG_t PIN_CONFIG[LED_NUM] = {
    { SECURE_GPIO0, (1U << 2), GPIO_PIN_RESET },   // LED RED   (PA2)
    { SECURE_GPIO0, (1U << 3), GPIO_PIN_RESET },   // LED GREEN (PA3)
    { SECURE_GPIO0, (1U << 4), GPIO_PIN_RESET }    // LED BLUE  (PA4)
};



/**
  \fn          int32_t LED_Initialize (void)
  \brief       Initialize I/O interface for LEDs
  \returns
   - \b  0: function succeeded
   - \b -1: function failed
*/
int32_t LED_Initialize (void) {
  uint32_t num;

  SECURE_SCC->RESET_CTRL |= (1U << 9);   /* exit GPIO reset state */
  while ((SECURE_SCC->RESET_CTRL & (1U << 9)) == 0U);

  for (num = 0; num < LED_NUM; num++) {
    PIN_CONFIG[num].port->OUTENSET |=  PIN_CONFIG[num].pin;   // configure as output
    PIN_CONFIG[num].port->DATAOUT  |=  PIN_CONFIG[num].pin;   // LED off
  }

  return 0;
}

/**
  \fn          int32_t LED_Uninitialize (void)
  \brief       De-initialize I/O interface for LEDs
  \returns
   - \b  0: function succeeded
   - \b -1: function failed
*/
int32_t LED_Uninitialize (void) {

  return 0;
}

/**
  \fn          int32_t LED_On (uint32_t num)
  \brief       Turn on a single LED indicated by \em num
  \param[in]   num  LED number
  \returns
   - \b  0: function succeeded
   - \b -1: function failed
*/
int32_t LED_On (uint32_t num) {

  if (num >= LED_NUM) {
    return -1;
  }

  if (PIN_CONFIG[num].active_state == GPIO_PIN_RESET) {
    PIN_CONFIG[num].port->DATAOUT &= ~PIN_CONFIG[num].pin;
  } else {
    PIN_CONFIG[num].port->DATAOUT |=  PIN_CONFIG[num].pin;
  }

  return 0;
}

/**
  \fn          int32_t LED_Off (uint32_t num)
  \brief       Turn off a single LED indicated by \em num
  \param[in]   num  LED number
  \returns
   - \b  0: function succeeded
   - \b -1: function failed
*/
int32_t LED_Off (uint32_t num) {

  if (num >= LED_NUM) {
    return -1;
  }

  if (PIN_CONFIG[num].active_state == GPIO_PIN_RESET) {
    PIN_CONFIG[num].port->DATAOUT |=  PIN_CONFIG[num].pin;
  } else {
    PIN_CONFIG[num].port->DATAOUT &= ~PIN_CONFIG[num].pin;
  }

  return 0;
}

/**
  \fn          int32_t LED_SetOut (uint32_t val)
  \brief       Control all LEDs with the bit vector \em val
  \param[in]   val  each bit represents the status of one LED.
  \returns
   - \b  0: function succeeded
   - \b -1: function failed
*/
int32_t LED_SetOut(uint32_t val) {
  uint32_t n;

  for (n = 0; n < LED_NUM; n++) {
    if (val & (1U << n)) {
      LED_On (n);
    } else {
      LED_Off(n);
    }
  }

  return 0;
}

/**
  \fn          uint32_t LED_GetCount (void)
  \brief       Get number of available LEDs on evaluation hardware
  \return      Number of available LEDs
*/
uint32_t LED_GetCount (void) {

  return LED_NUM;
}
