/*----------------------------------------------------------------------------
 * Name:    ulp_wut.h
 * Purpose: Ultra Low Power Wake Up Timer
 *----------------------------------------------------------------------------*/

#ifndef ULP_WUT_H_
#define ULP_WUT_H_

#include <stdint.h>


#define S32KCLOCK    (32768U)                            /* 1 second */

extern uint32_t s32k_timer_val;                          /* S32K Timer current value */

/*----------------------------------------------------------------------------
  API
 *----------------------------------------------------------------------------*/
extern void S32K_TIMER_Prepare (void);
extern void S32K_TIMER_Start (void);
extern void S32K_TIMER_Check (void);


#endif /* ULP_WUT_H_ */
