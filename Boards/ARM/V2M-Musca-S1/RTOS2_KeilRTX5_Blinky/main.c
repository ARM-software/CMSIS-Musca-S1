
#include "cmsis_os2.h"                  // ::CMSIS:RTOS2

#include "RTE_Components.h"             // Component selection
#include  CMSIS_device_header           // Device header

#ifdef RTE_Compiler_EventRecorder
#include "EventRecorder.h"
#endif

extern uint32_t SystemCoreClock;

extern void                 app_main (void *arg);
#if defined RTE_CMSIS_RTOS2_RTX5
extern const osThreadAttr_t app_main_attr;
#endif

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
  SystemCoreClockConfig();                              /* Configure the System Clock */
  SystemCoreClockUpdate();                              /* Update System Core Clock info */

  osKernelInitialize();                                 /* Initialize CMSIS-RTOS */
																								        
#if   defined RTE_CMSIS_RTOS2_RTX5
  osThreadNew(app_main, NULL, &app_main_attr);          /* Create application main thread */
#elif defined RTE_CMSIS_RTOS2_FreeRTOS
  osThreadNew(app_main, NULL, NULL);                    /* create application main thread */
#endif
  if (osKernelGetState() == osKernelReady) {            
    osKernelStart();                                    /* Start thread execution */
  }
  for (;;);
}
