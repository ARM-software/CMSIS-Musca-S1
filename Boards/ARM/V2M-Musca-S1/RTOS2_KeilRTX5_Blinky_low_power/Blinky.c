/*----------------------------------------------------------------------------
 * Name:    Blinky.c
 * Purpose: LED Flasher
 * Note(s):
 *----------------------------------------------------------------------------*/

#include <stdio.h>
#include <math.h>
#include "cmsis_os2.h"                  // ::CMSIS:RTOS2
#include "RTX_Config.h"                 // ARM::CMSIS:RTOS2:Keil RTX5

#include "RTE_Components.h"             // Component selection
#include  CMSIS_device_header           // Device header
#include "Board_LED.h"                  // ::Board Support:LED

#include "ulp_man.h"
#include "ulp_wut.h"


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
  thrCalc: some calculations
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
  S32K_TIMER_Prepare();                                  /* prepare the interrupt vector */

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
