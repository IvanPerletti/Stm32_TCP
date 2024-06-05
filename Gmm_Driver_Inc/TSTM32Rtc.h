/******************************************************************************
 * @file 			TSTM32Rtc.h
 * @brief			Implementation of RTC driver for STM32
 * @author		Perletti Rossi\n
 *  General 	Medical Merate - GMM.spa - Seriate - ITALY
 * @version		1.0
 * @date			January, 2016
 * @pre				Initialize and Enable the Serial class for the communication
 * @post			Nope
 * @warning		Improper use can crash your application
 * @copyright GMM.spa - All Rights Reserved
 * @ide       Keil uVision 5
 * @packs     STM32F4xx Keil packs version 2.2.0 or greater required
 * @stdperiph STM32F4xx Standard peripheral drivers version 1.4.0 or greater required
 *
 ******************************************************************************/
#ifndef TSTM32RTC_H_
#define TSTM32RTC_H_

#ifdef __cplusplus
extern "C" {
#endif 
#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_rtc.h"
#include "stm32f4xx_pwr.h"
#include "misc.h"
#include "time.h"
#ifdef __cplusplus
}
#endif 

#pragma once /* avoid redefinitions */

/**
 * @defgroup TSTM32Rtc DEFINES
 *
 * @{
 */

/** Internal status registers for RTC. */
#define STM32RTC_STATUS_REG                  	RTC_BKP_DR19		

/** RTC initialised. */
#define STM32RTC_STATUS_INIT_OK              	0x1234 

/** RTC time OK. */
#define STM32RTC_STATUS_TIME_OK                 0x0021

/** RTC time OK. */
#define STM32RTC_STATUS_DATE_OK                 0x4300

/** RTC date and time OK. */
#define STM32RTC_STATUS_DATETIME_OK             0x4321

/** RTC status 0. */
#define	STM32RTC_STATUS_ZERO            		0x0000



/* RTC clock is: f_clk = RTCCLK(LSI or LSE) / ((RTC_SYNC_PREDIV + 1) * (RTC_ASYNC_PREDIV + 1)) */
/* Sync pre division for clock */
#ifndef RTC_SYNC_PREDIV
#define RTC_SYNC_PREDIV							0x3FF
#endif
/* Async pre division for clock */
#ifndef RTC_ASYNC_PREDIV
#define RTC_ASYNC_PREDIV						0x1F
#endif
/* NVIC global Priority set */
#ifndef RTC_PRIORITY
#define RTC_PRIORITY							0x04
#endif
/* Sub priority for wakeup trigger */
#ifndef RTC_WAKEUP_SUBPRIORITY
#define RTC_WAKEUP_SUBPRIORITY					0x00
#endif
/* Sub priority for alarm trigger */
#ifndef RTC_ALARM_SUBPRIORITY
#define RTC_ALARM_SUBPRIORITY					0x01
#endif

/** @} */

/**
 * @defgroup DS1338 Reference Year
 *
 * @{
 */

/** UNIX TIME. */
#define STM32RTC_UNIX_REF						1970

#define STM32RTC_1900_TO_UNIX_REF               70

#define STM32RTC_SECONDS_PER_DAY           	    86400L

#define STM32RTC_SECONDS_FROM_1970_TO_2000 	    946684800L

#define RTC_LEAP_YEAR(year)             ((((year) % 4 == 0) && ((year) % 100 != 0)) || ((year) % 400 == 0))
#define RTC_DAYS_IN_YEAR(x)             RTC_LEAP_YEAR(x) ? 366 : 365
#define RTC_OFFSET_YEAR                 1970
#define RTC_SECONDS_PER_DAY             86400
#define RTC_SECONDS_PER_HOUR            3600
#define RTC_SECONDS_PER_MINUTE          60
#define RTC_BCD2BIN(x)                  ((((x) >> 4) & 0x0F) * 10 + ((x) & 0x0F))
#define RTC_CHAR2NUM(x)                 ((x) - '0')
#define RTC_CHARISNUM(x)                ((x) >= '0' && (x) <= '9')

/** @} */


/** STM32RTC OK. */
#define STM32RTC_ERR_OK  						 0x00000000

/** STM32RTC ERROR FAIL. */
#define STM32RTC_ERR_FAIL  			    		 0xFFFFFFFF

/** @} */


/**
 * @brief  Select RTC clock source
 * @note   Internal clock is not accurate and should not be used in production
 */
typedef enum {
  STM32RTC_ClockSource_Internal = 0x00, /*!< Use internal clock source for RTC (LSI oscillator) */
  STM32RTC_ClockSource_External         /*!< Use external clock source for RTC (LSE oscillator) */
} enumSTM32RTCClockSource;

class TSTM32Rtc {
public:
  TSTM32Rtc();
  virtual 	~TSTM32Rtc();
  int32_t	init(enumSTM32RTCClockSource source);
  int32_t	deInit();
  int32_t   setTime(const struct tm* t);
  int32_t   getTime(struct tm* t);
  int32_t   setDate(const struct tm* t);
  int32_t   getDate(struct tm* t);
  int32_t   setDateTime(const struct tm* t);
  int32_t   getDateTime(struct tm* t);
  int32_t	store(uint8_t u8Data, uint8_t u8MemoAddr);
  int32_t	get(uint8_t* u8DataP, uint8_t u8MemoAddr);
  int32_t	storeMultiple(const uint8_t* u8DataP, uint8_t u8MemoAddr, uint8_t u8NByte);
  int32_t	getMultiple(uint8_t* u8DataP, uint8_t u8MemoAddr, uint8_t u8NByte);
  int32_t	scan(uint8_t* u8DataP);
private:    // variables
  /* Default RTC status */
  uint32_t  TSTM32Rtc_status;
  /* RTC declarations */
  RTC_TimeTypeDef RTC_TimeStruct;
  RTC_InitTypeDef RTC_InitStruct;
  RTC_DateTypeDef RTC_DateStruct;
  int32_t   reference_year;
  uint8_t   u8aRtcRAM[80];  // SRAM rtc back-up data
private:    // functions	
  uint8_t	bcd2bin(uint8_t u8Data);  // Convert Binary-coded decimal to binary
  uint8_t   bin2bcd(uint8_t u8Data);  // Convert binary to binary-coded decimal
  uint8_t   conv2d(uint8_t* p);       // Convert a char array to corresponding unsigned char
  void      config(enumSTM32RTCClockSource source);
};

#endif /* TSTM32RTC */
