/*----------------------------------------------------------------------------
 * Name:    ulp_wut.c
 * Purpose: Ultra Low Power Wake Up Timer
 *          Code and data placed in SRAM3 (see scatter file)
 *          SRAM3 is active during Ultra Low Power down.
 *----------------------------------------------------------------------------*/

#include "RTE_Components.h"             // Component selection
#include  CMSIS_device_header           // Device header

#include "cmsis_os2.h"                  // ::CMSIS:RTOS2
#include "RTX_Config.h"                 // ARM::CMSIS:RTOS2:Keil RTX5

#include "ulp_wut.h"
#include "ulp_osp.h"


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
void S32K_TIMER_Start (void)
{
  s32k_timer_irq = 0U;
  s32k_timer_val = (osTick_sleep * S32KCLOCK) / OS_TICK_FREQ;

  NVIC_EnableIRQ (S32K_TIMER_IRQn);

  SECURE_S32K_TIMER->VALUE  = s32k_timer_val;
  SECURE_S32K_TIMER->RELOAD = s32k_timer_val;
  SECURE_S32K_TIMER->CTRL   = ((1U << 3) |               /* enable S32K Timer interrupt */
                               (1U << 0)  );             /* enable S32K Timer */
}


/*----------------------------------------------------------------------------
  S32K Timer ckeck
 *----------------------------------------------------------------------------*/
void S32K_TIMER_Check (void)
{
  if (s32k_timer_irq == 0U) {                            /* if S32K timer interrupt did not happen */
    SECURE_S32K_TIMER->INTCLEAR = 1U;                    /* clear S32K Timer interrupt */
    SECURE_S32K_TIMER->CTRL = 0U;                        /* disable S32K Timer interrupt */

    NVIC_DisableIRQ(S32K_TIMER_IRQn);
    NVIC_ClearPendingIRQ(S32K_TIMER_IRQn);

    s32k_timer_val = SECURE_S32K_TIMER->RELOAD - SECURE_S32K_TIMER->VALUE;
  }

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
