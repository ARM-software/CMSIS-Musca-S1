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
#include "cmsis_os2.h"                  // ::CMSIS:RTOS2
#include "RTX_Config.h"                 // ARM::CMSIS:RTOS2:Keil RTX5

#include "RTE_Components.h"             // Component selection
#include  CMSIS_device_header           // Device header
#include "Board_LED.h"                  // ::Board Support:LED


#if defined RTE_CMSIS_RTOS2_RTX5
/* stack size must be multiple of 8 Bytes */
#define APP_MAIN_STK_SZ (1024U)
extern uint64_t app_main_stk[APP_MAIN_STK_SZ / 8];
       uint64_t app_main_stk[APP_MAIN_STK_SZ / 8];
extern const osThreadAttr_t app_main_attr;
       const osThreadAttr_t app_main_attr = {
  .stack_mem  = &app_main_stk[0],
  .stack_size = sizeof(app_main_stk)
};
#endif


static osThreadId_t tid_thrLED;                /* Thread id of thread: LED */
static osThreadId_t tid_thrCalc;               /* Thread id of thread: Calc */

/*----------------------------------------------------------------------------
  thrLED: blink LED
 *----------------------------------------------------------------------------*/
__NO_RETURN static void thrLED(void *argument)
{
  (void)argument;

  uint32_t ledNum = 0;

#if   defined QSPI
  ledNum = 0;
#elif defined MRAM
  ledNum = 1;
#else
  ledNum = 2;
#endif

  for (;;) {
    osThreadFlagsWait(0x0001, osFlagsWaitAny ,osWaitForever);
    LED_On(ledNum);
    osDelay(300U);
    LED_Off(ledNum);
  }

}

/*----------------------------------------------------------------------------
  thrCalc: check button state
 *----------------------------------------------------------------------------*/
__NO_RETURN static void thrCalc(void *argument)
{
  (void)argument;
  
  volatile double sum = 0;
  uint32_t x;

  for (;;) {
    osThreadFlagsWait(0x0001, osFlagsWaitAny ,osWaitForever);

    for (x = 0; x < 1000; x++) {
      sum += sin (4.0) * (x + 1);
    }
    sum = sum + x;
  }
}


/*----------------------------------------------------------------------------
 * Application main thread
 *---------------------------------------------------------------------------*/
__NO_RETURN void app_main (void *argument);
__NO_RETURN void app_main (void *argument)
{
  (void)argument;

  LED_Initialize ();

  tid_thrLED = osThreadNew (thrLED, NULL, NULL);         /* create LED thread */
  if (tid_thrLED == NULL) { /* add error handling */ }

  tid_thrCalc = osThreadNew (thrCalc, NULL, NULL);       /* create Calc thread */
  if (tid_thrCalc == NULL) { /* add error handling */ }

//  osThreadExit ();
  for (;;) {
    osThreadFlagsSet(tid_thrLED,  0x0001U);
    osThreadFlagsSet(tid_thrCalc, 0x0001U);
  
    osDelay(1000U);
  }
}


/*----------------------------------------------------------------------------
  Musca-S1 Low-Power
 *---------------------------------------------------------------------------*/
#define S32K_CLOCK   (32768U)    /* 1 second */

static volatile uint32_t osTick_sleep;                   /* OS ticks to sleep */
static volatile uint32_t s32k_timer_irq;                 /* S32K Timer interrupt happened */
static volatile uint32_t s32k_timer_val;                 /* S32K Timer current value */

static void Musca_LP_Entry (void)
{
  s32k_timer_irq = 0U;
  s32k_timer_val = (osTick_sleep * S32K_CLOCK) / OS_TICK_FREQ;

  NVIC_EnableIRQ (S32K_TIMER_IRQn);
  
  SECURE_S32K_TIMER->RELOAD = s32k_timer_val;
  SECURE_S32K_TIMER->VALUE  = s32k_timer_val;
  SECURE_S32K_TIMER->CTRL   = ((1U << 3) |               /* enable S32K Timer interrupt */
                               (1U << 0)  );             /* enable S32K Timer */
  
  SCB->SCR |= (SCB_SCR_SLEEPDEEP_Msk);                   /* Setting the sleep deep bit */
}

static void Musca_LP_Exit (void)
{
  if (s32k_timer_irq == 0U) {

    SECURE_S32K_TIMER->INTCLEAR = 1U;                    /* clear S32K Timer interrupt */
    SECURE_S32K_TIMER->CTRL = 0U;                        /* disable S32K Timer interrupt */

    NVIC_DisableIRQ(S32K_TIMER_IRQn);
    NVIC_ClearPendingIRQ(S32K_TIMER_IRQn);

    s32k_timer_val = SECURE_S32K_TIMER->RELOAD - SECURE_S32K_TIMER->VALUE;
  }
  
  osTick_sleep = (s32k_timer_val * OS_TICK_FREQ + (S32K_CLOCK- 1U)) / S32K_CLOCK;
}

void S32K_TIMER_IRQHandler (void);
void S32K_TIMER_IRQHandler (void)
{
  s32k_timer_irq = 1U;
  s32k_timer_val = SECURE_S32K_TIMER->VALUE;

  SECURE_S32K_TIMER->INTCLEAR = 1U;                      /* clear S32K Timer interrupt */
  SECURE_S32K_TIMER->CTRL = 0U;                          /* disable S32K Timer interrupt */

  NVIC_DisableIRQ(S32K_TIMER_IRQn);
  NVIC_ClearPendingIRQ(S32K_TIMER_IRQn);
}
 
            void osRtxIdleThread (void *argument);
__NO_RETURN void osRtxIdleThread (void *argument)
{
  (void)argument;
  
  for (;;) {
    osTick_sleep = osKernelSuspend();                    /* returns OS ticks to sleep */

    if (osTick_sleep > 0) {                              /* Is there some time to sleep? */

      Musca_LP_Entry();                                  /* Enter the low power state */

      __WFI();
                                                         /* Leave the low power state */
      Musca_LP_Exit();                                   /* osTick_sleep is recalculated */
    }

    /* Adjust the kernel ticks with the amount of ticks slept */
    osKernelResume (osTick_sleep);
  }
}
