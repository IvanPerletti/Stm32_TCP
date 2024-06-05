/******************************************************************************
 * @file        TIWdog.h
 * @brief       Implementation of IWDOG driver for STM32
 * @author      Perletti Rossi\n
 *  General Medical Merate - GMM.spa - Seriate - ITALY
 * @version     1.0
 * @date        January, 2016
 * @pre         Initialize and Enable the internal watchdog
 * @post        Nope
 * @warning     Improper use can crash your application. Please be carefull and 
 * double check the @ref IWDOG_LSI_FREQ 
 * @copyright   GMM.spa - All Rights Reserved
 * @ide         Keil uVision 5
 * @packs       STM32F4xx Keil packs version 2.2.0 or greater required
 * @stdperiph   STM32F4xx Standard peripheral drivers version 1.4.0 or greater 
 * required
 *
 ******************************************************************************/
#ifndef TIWDOG_H_
#define TIWDOG_H_

#ifdef __cplusplus
extern "C" {
#endif 
#include "stm32f4xx.h"
#include "stm32f4xx_iwdg.h"
#include "misc.h"
#ifdef __cplusplus
}
#endif 

#pragma once /* avoid redefinitions */

/**
 * @defgroup TIWdog DEFINES
 * @warning please be carefull and double check the @ref IWDOG_LSI_FREQ 
 * @{
 */

/** LSI FREQUENCY [Hz] 
 * In order to verify the LSI freq. a oscilloscope probe can 
 * be attached to LED3 pin. If LSI freq. was subposted correctly
 * the led will be toggled as follows:
 * 2000 [msec] off
 *  500 [msec] on
 *
 * please copy paste the following code in main()
 *
 * pitbull.init(500);
 * STM_EVAL_LEDOn(LED3);
 * pitbull.start();
 * while(1){}
 *
 */
#define IWDOG_LSI_FREQ 										100000

/** IWDOG PRESCALER */
#define IWDOG_PRESCALER										IWDG_Prescaler_128

/** IWdog OK. */
#define IWDOG_ERR_OK  										0x00000000

/** IWdog ERROR FAIL. */
#define IWDOG_ERR_FAIL  			    				    0xFFFFFFFF

/** IWdog resumed from IWDG reset. */
#define IWDOG_ERR_RES_FROM_RESET  				            0x000000AA

/** IWdog ARMED. */
#define IWDOG_ERR_ARMED 				  				    0x0000BB00

/** IWdog DISARMED. */
#define IWDOG_ERR_DISARMED 			  				        0x00CC0000

/** @} */



class TIWdog {
public:
  TIWdog();
  virtual 	~TIWdog();
  int32_t	init(uint32_t lReload_ms);
  int32_t	deInit();
  int32_t   start();
  int32_t   stop();
  void      reload();
  void      resetMeNow();
  int32_t   state();

private:    // variables
  /* IWDog status */
  int32_t   iwdog_status;
private:    // functions	
  void      config(uint32_t ms);
};

#endif /* TSTM32RTC */
