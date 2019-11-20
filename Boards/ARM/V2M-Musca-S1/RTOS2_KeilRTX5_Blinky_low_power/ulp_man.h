/*----------------------------------------------------------------------------
 * Name:    ulp_man.h
 * Purpose: Ultra Low Power Management
 *----------------------------------------------------------------------------*/

#ifndef ULP_MAN_H_
#define ULP_MAN_H_

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


#endif /* ULP_MAN_H_ */
