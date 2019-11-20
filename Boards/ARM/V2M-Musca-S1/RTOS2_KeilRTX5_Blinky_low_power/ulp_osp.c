/*----------------------------------------------------------------------------
 * Name:    ulp_osp.c
 * Purpose: Ultra Low Power OS part
 *          Code and data placed in SRAM3 (see scatter file)
 *          SRAM3 is active during Ultra Low Power down.
 *----------------------------------------------------------------------------*/

#include "RTE_Components.h"             // Component selection
#include  CMSIS_device_header           // Device header

#include "cmsis_os2.h"                  // ::CMSIS:RTOS2
#include "RTX_Config.h"                 // ARM::CMSIS:RTOS2:Keil RTX5

#include "ulp_man.h"
#include "ulp_wut.h"


extern uint32_t osTick_sleep;           /* OS ticks to sleep */
       uint32_t osTick_sleep;

#define osPS_sz (64U)
static uint32_t osPSP_cur;              /* current PSP */
static uint32_t osPSPLIM_cur;           /* current PSPLIM */
static uint32_t osPS_tmp[osPS_sz];      /* new Process stack */

/*----------------------------------------------------------------------------
  CMSIS OS Idle thread
 *----------------------------------------------------------------------------*/
            void osRtxIdleThread (void *argument);
__NO_RETURN void osRtxIdleThread (void *argument)
{
  (void)argument;

  for (;;) {
    osTick_sleep = osKernelSuspend();                    /* returns OS ticks to sleep */

    if (osTick_sleep > 0) {                              /* Is there some time to sleep? */

      /* PSP handling */
      osPSP_cur    = __get_PSP();                        /* save the current PSP */
      osPSPLIM_cur = __get_PSPLIM();
      __set_PSP((uint32_t)&osPS_tmp[osPS_sz]);           /* set PSP to ULP RAM */
      __set_PSPLIM((uint32_t)&osPS_tmp[0]);

      S32K_TIMER_Start();                                /* start the wake up timer */

      Musca_ULP_Entry();                                 /* Enter the low power state */

      SCB->SCR |= (SCB_SCR_SLEEPDEEP_Msk);               /* Setting the sleep deep bit */
      __DSB();
      __WFI();                                           /* Wait for S32K timer interrupt */

     __NOP();
      Musca_ULP_Exit();                                  /* Leave the low power state */

      __set_PSP(osPSP_cur);                              /* restore PSP */
      __set_PSPLIM(osPSPLIM_cur);

      S32K_TIMER_Check();                                /* check if S32K timer interrupt happened */

      /* calculate how long we slept */
      osTick_sleep = (s32k_timer_val * OS_TICK_FREQ + (S32KCLOCK - 1U)) / S32KCLOCK;
    }

    /* Adjust the kernel ticks with the amount of ticks slept */
    osKernelResume (osTick_sleep);
  }

}
