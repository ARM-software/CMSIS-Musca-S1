/*----------------------------------------------------------------------------
 * CMSIS-RTOS 'main' function template
 *---------------------------------------------------------------------------*/
 
#include "RTE_Components.h"
#include  CMSIS_device_header
#include "cmsis_os2.h"

#ifdef  RTE_TFM_TEST_FRAMEWORK_NS
#define main main0
#include "Driver_USART.h"
#include "tfm_test_config.h"
#include "tfm_config_rte.h"
#endif

/*----------------------------------------------------------------------------
 * Application main thread
 *---------------------------------------------------------------------------*/
__NO_RETURN static void app_main (void *argument) {
  (void)argument;
  // ...
  for (;;) {}
}
 
int main (void) {
 
  // System Initialization
  SystemCoreClockUpdate();
  // ...
 
#ifndef RTE_TFM_TEST_FRAMEWORK_NS
  osKernelInitialize();                 // Initialize CMSIS-RTOS
  osThreadNew(app_main, NULL, NULL);    // Create application main thread
  osKernelStart();                      // Start thread execution
  for (;;) {}
#else
  return 0;
#endif
}

#ifdef RTE_TFM_TEST_FRAMEWORK_NS
extern ARM_DRIVER_USART NS_DRIVER_STDIO;
int32_t tfm_ns_platform_init(void)
{
    int32_t ret;

    main();

    ret = NS_DRIVER_STDIO.Initialize(NULL);
    if (ret != ARM_DRIVER_OK) {
      return ARM_DRIVER_ERROR;
    }

    ret = NS_DRIVER_STDIO.PowerControl(ARM_POWER_FULL);
    if (ret != ARM_DRIVER_OK) {
      return ARM_DRIVER_ERROR;
    }

    ret = NS_DRIVER_STDIO.Control(ARM_USART_MODE_ASYNCHRONOUS,
                                  DEFAULT_UART_BAUDRATE);
    if (ret != ARM_DRIVER_OK) {
      return ARM_DRIVER_ERROR;
    }

    (void)NS_DRIVER_STDIO.Control(ARM_USART_CONTROL_TX, 1);

    return ARM_DRIVER_OK;
}
#endif
