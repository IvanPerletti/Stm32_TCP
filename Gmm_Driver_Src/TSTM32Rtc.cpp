/******************************************************************************
 * @file        TSTM32Rtc.cpp
 * @brief       Implementation of RTC driver for STM32
 * @author      Perletti Rossi\n
 *  General Medical Merate - GMM.spa - Seriate - ITALY
 * @version     1.0
 * @date        December, 2015
 * @pre         Initialize and Enable the Serial class for the communication
 * @post        Nope
 * @warning     Improper use can crash your application
 * @copyright   GMM.spa - All Rights Reserved
 * @ide         Keil uVision 5
 * @packs       STM32F4xx Keil packs version 2.2.0 or greater required
 * @stdperiph   STM32F4xx Standard peripheral drivers version 1.4.0 or greater required
 *
 ******************************************************************************/

#include "TSTM32Rtc.h"

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

/**
 * @brief Class Constructor. Default value are applied to clock_address
 * and reference_year.
 */
TSTM32Rtc::TSTM32Rtc() {

  TSTM32Rtc_status = STM32RTC_STATUS_ZERO;
  u8aRtcRAM = 0;
  reference_year = STM32RTC_1900_TO_UNIX_REF; // 1st Jan 1970
}

/**
 * @brief Class Destructor. It calls deInit().
 */
TSTM32Rtc::~TSTM32Rtc() {
  deInit();
}

//-----------------------------------------------------------------------------
/**
 * @brief Main initialization of the Peripheral and internal class RAM
 * @return the driver status.
 */
int32_t TSTM32Rtc::init(enumSTM32RTCClockSource source)
{
  uint32_t status;

  /* Enable PWR peripheral clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

  /* Allow access to BKP Domain */
  PWR_BackupAccessCmd(ENABLE);

  /* Get RTC status */
  status = RTC_ReadBackupRegister(STM32RTC_STATUS_REG);

  /* Start internal clock */
  config(source);

  /* Disable write protection */
  RTC_WriteProtectionCmd(DISABLE);

  /* Wait for RTC APB registers synchronization
   * (needed after start-up from Reset) */
  RTC_WaitForSynchro();

  /* Enable write protection */
  RTC_WriteProtectionCmd(ENABLE);

  /* Update state*/
  TSTM32Rtc_status = status;

  return TSTM32Rtc_status;
}
//-----------------------------------------------------------------------------
/**
 * @brief Safe De-Initialization of the class: all the states are assigned to
 * default values
 * @return the driver status.
 */
int32_t TSTM32Rtc::deInit()
{                    
  return TSTM32Rtc_status;
}
//-----------------------------------------------------------------------------
/**
 * @brief Time setting function
 * @param[in] struct tm* t only sec, min and hour are used  
 * @return flag for operation status SUCCESS or NOT
 */
int32_t     TSTM32Rtc::setTime(const struct tm* t)
{

  /* Fill time */
  RTC_TimeStruct.RTC_Hours = t->tm_hour;
  RTC_TimeStruct.RTC_Minutes = t->tm_min;
  RTC_TimeStruct.RTC_Seconds = t->tm_sec;

  /* Set the RTC time base to 1s and hours format to 24h */
  RTC_InitStruct.RTC_HourFormat = RTC_HourFormat_24;
  RTC_InitStruct.RTC_AsynchPrediv = RTC_ASYNC_PREDIV;
  RTC_InitStruct.RTC_SynchPrediv = RTC_SYNC_PREDIV;
  RTC_Init(&RTC_InitStruct);

  /* Set time */
  RTC_SetTime(RTC_Format_BIN, &RTC_TimeStruct);

  /* Allow access to BKP Domain */
  PWR_BackupAccessCmd(ENABLE);
  /* Write backup registers */
  RTC_WriteBackupRegister(STM32RTC_STATUS_REG, STM32RTC_STATUS_TIME_OK);
  /* Disable access to BKP Domain */
  PWR_BackupAccessCmd(DISABLE);


  TSTM32Rtc_status |= STM32RTC_STATUS_TIME_OK;

  /* Return OK */
  return STM32RTC_ERR_OK; 
}
//-----------------------------------------------------------------------------
/**
 * @brief Funtion to get the device time 
 * @param[out] struct tm* t only sec, min and hour are used @ref struct tm        
 * @return flag for operation status SUCCESS or NOT     
 */
int32_t     TSTM32Rtc::getTime(struct tm* t)
{

  RTC_GetTime(RTC_Format_BIN, &RTC_TimeStruct);

  t->tm_sec = RTC_TimeStruct.RTC_Seconds;
  t->tm_min = RTC_TimeStruct.RTC_Minutes;
  t->tm_hour = RTC_TimeStruct.RTC_Hours;

  return STM32RTC_ERR_OK;
}
//-----------------------------------------------------------------------------
/**
 * @brief Date setting function
 * @param[in] struct tm* t only wday, mday, mon and year are used @ref struct tm          
 * @return flag for operation status SUCCESS or NOT     
 */
int32_t     TSTM32Rtc::setDate(const struct tm* t)
{
  /* Fill date */
  RTC_DateStruct.RTC_Date = t->tm_mday;
  // month 1-12
  RTC_DateStruct.RTC_Month = t->tm_mon+1;
  RTC_DateStruct.RTC_Year = t->tm_year - reference_year;
  // week day from 1 to 7 (sunday to monday)
  RTC_DateStruct.RTC_WeekDay = t->tm_wday+1;

  /* Set the RTC time base to 1s and hours format to 24h */
  RTC_InitStruct.RTC_HourFormat = RTC_HourFormat_24;
  RTC_InitStruct.RTC_AsynchPrediv = RTC_ASYNC_PREDIV;
  RTC_InitStruct.RTC_SynchPrediv = RTC_SYNC_PREDIV;
  RTC_Init(&RTC_InitStruct);

  RTC_SetDate(RTC_Format_BIN, &RTC_DateStruct);

  /* Allow access to BKP Domain */
  PWR_BackupAccessCmd(ENABLE);
  /* Write backup registers */
  RTC_WriteBackupRegister(STM32RTC_STATUS_REG, STM32RTC_STATUS_DATE_OK);
  /* Disable access to BKP Domain */
  PWR_BackupAccessCmd(DISABLE);

  TSTM32Rtc_status |= STM32RTC_STATUS_DATE_OK;
  /* Return OK */
  return STM32RTC_ERR_OK;
}
//-----------------------------------------------------------------------------
/**
 * @brief Function to get the device date 
 * @param[out] struct tm* t only wday, mday, mon and year are used
 * @ref struct tm
 * @return                  SUCCESS or NOT      
 */
int32_t     TSTM32Rtc::getDate(struct tm* t)
{
  RTC_GetDate(RTC_Format_BIN, &RTC_DateStruct);

  t->tm_wday = RTC_DateStruct.RTC_WeekDay - 1;
  t->tm_mday = RTC_DateStruct.RTC_Date;
  t->tm_mon = RTC_DateStruct.RTC_Month - 1;
  t->tm_year = RTC_DateStruct.RTC_Year + reference_year;

  /* Return OK */
  return STM32RTC_ERR_OK;
}
//-----------------------------------------------------------------------------
/**
 * @brief Funtion to set the device time and date 
 * @param[in] struct tm* t  @ref struct tm
 * @return                  SUCCESS or NOT      
 */
int32_t   TSTM32Rtc::setDateTime(const struct tm* t)
{
  /* Fill time */
  RTC_TimeStruct.RTC_Hours = t->tm_hour;
  RTC_TimeStruct.RTC_Minutes = t->tm_min;
  RTC_TimeStruct.RTC_Seconds = t->tm_sec;

  /* Fill date */
  RTC_DateStruct.RTC_Date = t->tm_mday;
  // month 1-12
  RTC_DateStruct.RTC_Month = t->tm_mon+1;
  RTC_DateStruct.RTC_Year = t->tm_year - reference_year;
  // week day from 1 to 7 (sunday to monday)
  RTC_DateStruct.RTC_WeekDay = t->tm_wday+1;

  /* Set the RTC time base to 1s and hours format to 24h */
  RTC_InitStruct.RTC_HourFormat = RTC_HourFormat_24;
  RTC_InitStruct.RTC_AsynchPrediv = RTC_ASYNC_PREDIV;
  RTC_InitStruct.RTC_SynchPrediv = RTC_SYNC_PREDIV;
  RTC_Init(&RTC_InitStruct);

  RTC_SetTime(RTC_Format_BIN, &RTC_TimeStruct);

  RTC_SetDate(RTC_Format_BIN, &RTC_DateStruct);

  /* Allow access to BKP Domain */
  PWR_BackupAccessCmd(ENABLE);
  /* Write backup registers */
  RTC_WriteBackupRegister(STM32RTC_STATUS_REG, STM32RTC_STATUS_DATETIME_OK);
  /* Disable access to BKP Domain */
  PWR_BackupAccessCmd(DISABLE);

  TSTM32Rtc_status = STM32RTC_STATUS_DATETIME_OK;

  /* Return OK */
  return STM32RTC_ERR_OK;
}
//-----------------------------------------------------------------------------
/**
 * @brief Funtion to get the device time and date 
 * @param[in] struct tm* t  
 * @return                  SUCCESS or NOT      
 */
int32_t   TSTM32Rtc::getDateTime(struct tm* t)
{
  RTC_GetTime(RTC_Format_BIN, &RTC_TimeStruct);
  RTC_GetDate(RTC_Format_BIN, &RTC_DateStruct);

  t->tm_sec = RTC_TimeStruct.RTC_Seconds;
  t->tm_min = RTC_TimeStruct.RTC_Minutes;
  t->tm_hour = RTC_TimeStruct.RTC_Hours;

  t->tm_wday = RTC_DateStruct.RTC_WeekDay - 1;
  t->tm_mday = RTC_DateStruct.RTC_Date;
  t->tm_mon = RTC_DateStruct.RTC_Month - 1;
  t->tm_year = RTC_DateStruct.RTC_Year + reference_year;

  /* Return OK */
  return STM32RTC_ERR_OK;
}
//-----------------------------------------------------------------------------
/**
 * @brief Save in RTC 80 Bytes RAM a data
 * @note The last 4 bytes are used for saving the driver state.
 * @param[in] u8Data        a byte to save
 * @param[in] u8MemoAddr    a byte that is the storing address. Must be between
 * 0 to 75
 * @detail The backup registers are 32-bit registers used to store 80 bytes of
 * user application data when VDD power is not present. Backup registers are not
 * reset by a system, a power reset, or when the device wakes up from the
 * Standby mode (see Section 2.2.19: Low-power modes).
 * @return                  SUCCESS or NOT      
 */
int32_t     TSTM32Rtc::store(uint8_t u8Data, uint8_t u8MemoAddr)
{
  int32_t drx_reg, drx, drb;

  /* Allow access to BKP Domain */
  PWR_BackupAccessCmd(ENABLE);

  // calc RTC_BKP_DRx 0-12
  drx_reg = u8MemoAddr / 4;
  if (drx_reg > 18) return STM32RTC_ERR_FAIL; // invalid address

  // read RTC_BKP_DRx
  drx = RTC_ReadBackupRegister(drx_reg);

  // calc byte inside word 0-3
  drb = u8MemoAddr % 4;

  // replace with new byte
  drx &=  (0xFFFFFFFF & (((int32_t)u8Data) << drb));

  RTC_WriteBackupRegister(drx_reg, drx);

  /* Disable access to BKP Domain */
  PWR_BackupAccessCmd(DISABLE);

  /* Return OK */
  return STM32RTC_ERR_OK;
}
//-----------------------------------------------------------------------------
/**
 * @brief Direct read from RTC 80 Bytes RAM
 * @note The last 4 bytes are used for saving the driver state.
 * @param[out] u8DataP      pointer to uint8_t where the copy of the RAM byte 
 * will be stored
 * @param[in] u8MemoAddr    a byte that is the storing address. Must be between
 * 0 to 75
 * @detail The backup registers are 32-bit registers used to store 80 bytes of
 * user application data when VDD power is not present. Backup registers are not
 * reset by a system, a power reset, or when the device wakes up from the
 * Standby mode (see Section 2.2.19: Low-power modes).
 * @return                  SUCCESS or NOT              
 */
int32_t     TSTM32Rtc::get(uint8_t* u8DataP, uint8_t u8MemoAddr)
{
  /* Allow access to BKP Domain */
  PWR_BackupAccessCmd(ENABLE);

  int32_t drx_reg, drx, drb;

  // calc RTC_BKP_DRx 0-12
  drx_reg = u8MemoAddr / 4;
  if (drx_reg > 18) return STM32RTC_ERR_FAIL; // invalid address

  // read RTC_BKP_DRx
  drx = RTC_ReadBackupRegister(drx_reg);

  // calc byte inside word 0-3
  drb = u8MemoAddr % 4;

  // read correct byte
  *u8DataP = (uint8_t)(0x000000FF & (drx >> drb));

  /* Disable access to BKP Domain */
  PWR_BackupAccessCmd(DISABLE);

  /* Return OK */
  return STM32RTC_ERR_OK;
}
//-----------------------------------------------------------------------------
/**
 * @brief Save in RTC 80 Bytes RAM a multiple data
 * @note This function is not optimized.
 * @note The last 4 bytes are used for saving the driver state.
 * @param[in] u8DataP       pointer to a byte array to save
 * @param[in] u8MemoAddr    a byte that is the starting address. Must be between
 * 0 to 55
 * @param[in] u8NByte       a byte that is the number of byte to be written.
 * Must be less than 75-u8MemoAddr
 * @return                  SUCCESS or NOT          
 */
int32_t     TSTM32Rtc::storeMultiple(const uint8_t* u8DataP, uint8_t u8MemoAddr, uint8_t u8NByte)
{

  if ((u8MemoAddr + u8NByte) > 75)
  {
    // you're tring to store in a non valid location
    return STM32RTC_ERR_FAIL;
  }

  while(u8NByte--)
  {
    store(*(u8DataP++), u8MemoAddr++);
  }

  /* Return OK */
  return STM32RTC_ERR_OK;
}
//-----------------------------------------------------------------------------
/**
 * @brief Direct read from RTC 80 Bytes RAM.
 * @note This function is not optimized.
 * @note The last 4 bytes are used for saving the driver state.
 * @param[out] u8DataP      pointer to uint8_t where the copy of the RAM bytes 
 * will be stored
 * @param[in] u8MemoAddr    a byte that is the storing address. Must be between
 * 0 to 75
 * @param[in] u8NByte       a byte that is the number of byte to be readed. Must 
 * be less than 75-u8MemoAddr
 * @return                  SUCCESS or NOT              
 */
int32_t     TSTM32Rtc::getMultiple(uint8_t* u8DataP, uint8_t u8MemoAddr, uint8_t u8NByte)
{
  if ((u8MemoAddr + u8NByte) > 75)
  {
    // you're tring to store in a non valid location
    return STM32RTC_ERR_FAIL;
  }

  while(u8NByte--)
  {
    get( u8DataP++, u8MemoAddr++);
  }

  /* Return OK */
  return STM32RTC_ERR_OK;
}
//-----------------------------------------------------------------------------
/**
 * @brief Read all the RTC data and store them locally
 * @note The last 4 bytes are used for saving the driver state.
 * @param[out] u8DataP      pointer to uint8_t where the copy of the RAM bytes 
 * will be stored. If NULL @ref DS1338::u8aRtcRAM[80] will be used
 */
int32_t TSTM32Rtc::scan(uint8_t* u8DataP)
{

  int32_t *wp;
  int32_t i;

  if (u8DataP == NULL) u8DataP = &(u8aRtcRAM[0]);

  /* Allow access to BKP Domain */
  PWR_BackupAccessCmd(ENABLE);

  // read RTC_BKP_DRx
  wp = (int32_t*)u8DataP;

  for (i = 0; i < 20; i++)
  {
    *(wp+i) =  RTC_ReadBackupRegister(RTC_BKP_DR1);
  }

  /* Disable access to BKP Domain */
  PWR_BackupAccessCmd(DISABLE);

  /* Return OK */
  return STM32RTC_ERR_OK;
}
//-----------------------------------------------------------------------------
/** @defgroup TSTM32Rtc_private_functions */
/**
 * @ingroup TSTM32Rtc_private_functions
 * @brief Convert Binary-coded decimal to binary
 * @param[in] u8Data    bcd datum
 * @return[out]         binary converted value
 */
uint8_t TSTM32Rtc::bcd2bin(uint8_t u8Data)
{
  return (u8Data - 6 * (u8Data >> 4));
}
//-----------------------------------------------------------------------------
/**
 * @ingroup TSTM32Rtc_private_functions
 * @brief Convert binary to binary-coded decimal
 * @param[in] u8Data   binary datum
 * @return             bcd converted value
 */
uint8_t TSTM32Rtc::bin2bcd(uint8_t u8Data)
{
  return (u8Data + 6 * (u8Data / 10));
}
//-----------------------------------------------------------------------------
/**
 * @ingroup TSTM32Rtc_private_functions
 * @brief Convert a char array to corresponding unsigned char
 * @param[in] p       pointer to char array of two element
 * @return            the 2 digit binary converted value
 */
uint8_t TSTM32Rtc::conv2d(uint8_t* p)          
{
  unsigned char v = 0;
  if ('0' <= *p && *p <= '9') 
  {
    v = *p - '0';
  }
  return (10 * v + *++p - '0');
} 
//-----------------------------------------------------------------------------
/**
 * @brief Low Level RTC configuration
 * @ingroup TSTM32Rtc_private_functions
 * @param[in] source @ref enumSTM32RTCClockSource
 */
void TSTM32Rtc::config(enumSTM32RTCClockSource source) {
  if (source == STM32RTC_ClockSource_Internal) {
    /* Enable the LSI OSC */
    RCC_LSICmd(ENABLE);

    /* Wait till LSI is ready */
    while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET);

    /* Select the RTC Clock Source */
    RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);
  } else if (source == STM32RTC_ClockSource_External) {
    /* Enable the LSE OSC */
    RCC_LSEConfig(RCC_LSE_ON);

    /* Wait till LSE is ready */
    while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET);

    /* Select the RTC Clock Source */
    RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
  }

  /* Enable the RTC Clock */
  RCC_RTCCLKCmd(ENABLE);

  /* Disable write protection */
  RTC_WriteProtectionCmd(DISABLE);

  /* Wait for RTC APB registers synchronization (needed after start-up from Reset) */
  RTC_WaitForSynchro();

  /* Enable write protection */
  RTC_WriteProtectionCmd(ENABLE);

  /* Allow access to BKP Domain */
  PWR_BackupAccessCmd(ENABLE);
  /* Write status */
  RTC_WriteBackupRegister(STM32RTC_STATUS_REG, STM32RTC_STATUS_INIT_OK);

}
