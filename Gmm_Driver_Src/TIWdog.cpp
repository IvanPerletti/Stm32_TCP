/******************************************************************************
 * @file        TIWdog.cpp
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

#include "TIWdog.h"

#ifdef __cplusplus
extern "C" {
#endif 
#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_iwdg.h"
#include "misc.h"
#ifdef __cplusplus
}
#endif 

/**
 * @brief Class Constructor
 */

TIWdog::TIWdog() {
  // TODO Auto-generated constructor stub
  iwdog_status = 0;
}

/**
 * @brief Class Destructor
 */
TIWdog::~TIWdog() {
  // TODO Auto-generated destructor stub
  deInit();
}

//-----------------------------------------------------------------------------
/**
 * @brief Main initialization of the Peripheral
 * @return @ref IWDOG_ERR_RES_FROM_RESET if the function was called after a
 * IWDG reset or @ref IWDOG_ERR_DISARMED.
 * @param[in] lReload_ms the watchdog timeout in msec
 * @warning please be carefull and double check the @ref IWDOG_LSI_FREQ 
 */
int32_t TIWdog::init(uint32_t lReload_ms)
{

  /* Check if the system has resumed from IWDG reset */
  if (RCC_GetFlagStatus(RCC_FLAG_IWDGRST) != RESET)
  {
    /* IWDGRST flag set */
    iwdog_status = IWDOG_ERR_RES_FROM_RESET;
    /* Clear reset flags */
    RCC_ClearFlag();
  }
  else
  {
    iwdog_status = IWDOG_ERR_DISARMED;
  }

  /* Enable the LSI oscillator */
  RCC_LSICmd(ENABLE);

  /* Wait till LSI is ready */
  while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET)
  {
  }

  /* Low Level IWDOG configuration  */
  config(lReload_ms);

  return iwdog_status;
}
//-----------------------------------------------------------------------------
/**
 * @brief Safe De-Initialization of the class: disable LSI OSC
 * @return flag for operation status SUCCESS or NOT
 */
int32_t TIWdog::deInit()
{ 
  /* Disable the LSI oscillator */
  RCC_LSICmd(DISABLE);

  iwdog_status = IWDOG_ERR_DISARMED;

  return IWDOG_ERR_OK;
}
//-----------------------------------------------------------------------------
/**
 * @brief Start watchdog 
 * @return flag for operation status SUCCESS or NOT
 */
int32_t TIWdog::start()
{
  /* Reload IWDG counter */
  IWDG_ReloadCounter();
  /* Enable IWDG (the LSI oscillator will be enabled by hardware) */
  IWDG_Enable();

  iwdog_status = IWDOG_ERR_ARMED;

  /* Return OK */
  return IWDOG_ERR_OK; 
}
//-----------------------------------------------------------------------------
/**
 * @brief Stop watchdog: disable LSI OSC. Equivalent to @ref TIWdog::deInit
 * @return flag for operation status SUCCESS or NOT
 */
int32_t TIWdog::stop()
{
  /* Disable the LSI oscillator */
  RCC_LSICmd(DISABLE);

  iwdog_status = IWDOG_ERR_DISARMED;

  /* Return OK */
  return IWDOG_ERR_OK; 
}
//-----------------------------------------------------------------------------
/**
 * @brief Funtion must be called before watchdog timeout.       
 * @return NULL     
 */
void TIWdog::reload()
{
  /* Reload IWDG counter */
  IWDG_ReloadCounter();
}
//-----------------------------------------------------------------------------
/**
 * @brief Funtion to get the driver state.       
 * @return IWDOG status flag    
 */
int32_t TIWdog::state()
{
  return iwdog_status;
}
//-----------------------------------------------------------------------------
/**
 * @brief Funtion that generate an Hardfault exception. It will generate an
 * IWDG reset.
 * @note the function can be called in any driver state.
 * @return NULL    
 */
void TIWdog::resetMeNow()
{
  /* As the following address is invalid (not mapped), a Hardfault exception
   will be generated with an infinite loop and when the IWDG counter reaches 0
   the IWDG reset occurs */
  *(__IO uint32_t *) 0x00040001 = 0xFF;
}
//-----------------------------------------------------------------------------
/**
 * @brief Low Level IWDOG configuration 
 * @param[in] ms 
 * @return NULL 
 */
void TIWdog::config(uint32_t ms) {
  int32_t wdog_pre;

  assert_param(IS_IWDG_PRESCALER(IWDOG_PRESCALER));

  switch (IWDOG_PRESCALER)
  {
  case IWDG_Prescaler_4:
    wdog_pre = 4;
    break;
  case IWDG_Prescaler_8:
    wdog_pre = 8;
    break;
  case IWDG_Prescaler_16:
    wdog_pre = 16;
    break;
  case IWDG_Prescaler_32:
    wdog_pre = 32;
    break;
  case IWDG_Prescaler_64:
    wdog_pre = 64;
    break;
  case IWDG_Prescaler_128:
    wdog_pre = 128;
    break;
  case IWDG_Prescaler_256:
    wdog_pre = 256;
    break;
  default:
    /* NEVER REACHED */
    break;
  }

  /* Enable write access to IWDG_PR and IWDG_RLR registers */
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);

  /* IWDG counter clock: LSI/32 */
  IWDG_SetPrescaler(IWDG_Prescaler_32);

  /* Set counter reload value to obtain 250ms IWDG TimeOut.
     IWDG counter clock Frequency = LsiFreq/32
     Counter Reload Value = = LsiFreq / ( wdog_pre * timeout_freq )                         
   */   
  ms = IWDOG_LSI_FREQ / (wdog_pre * (1000 / ms) );

  /* checking maximun */
  assert_param(IS_IWDG_RELOAD(ms));

  IWDG_SetReload(ms);

  /* Reload IWDG counter */
  IWDG_ReloadCounter();
}
