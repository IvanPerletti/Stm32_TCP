
/******************************************************************************
 * @file 		Main.cpp
 * @brief		main source file for Main Controller project - STM32F4 chip
 * @details		This class is used to configure and use the Main project:
 * @author		Ronchi - Perletti \n
 *  General Medical Merate - GMM.spa - Seriate - ITALY
 * @version		1.0
 * @date		February 4th, 2014
 * @pre			Initialize and Enable the Serial class for the communication
 * @post		Nope
 * @bug			Not all memory is freed when deleting an object of this class.
 * @warning		Improper use can crash your application
 * @copyright 	GMM.spa - All Rights Reserved
 *
 ******************************************************************************/
#include "main.h"
//#define USE_FULL_ASSERT_a
extern "C" {
#include "watchDog_i1.h"
#include <stdio.h>
#include "stm32f4xx_it.h"
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
//#include "stm32f4_discovery.h"
}

#include "TAutoma_SerialTest.h"
#include "TAutoma_EthTest.h"


#if !defined( _FW_VER_MAJOR_ ) || !defined( _FW_VER_MINOR_ )
#error "_FW_VER_MAJOR_ / _FW_VER_MINOR_ NOT DEFINED in main.h"
#warning "USE_FULL_ASSERT , USE_STDPERIPH_DRIVER , STM32F4XX , HSE_VALUE=8000000"
#warning "USE_STM32F4_DISCOVERY , USE_USB_OTG_FS , STM32F40_41xxx ,  _USB_DEBUG_ , _SW_VER = 101"
#endif
#define _DBG_REALTIME // "Hard REal Time Certification

/* Private typedef -----------------------------------------------------------*/

void LED_WDOG_Init_Config(void);
void AutomatonsExecution(void);
#define		DBG_PB15_TGL		(GPIO_ToggleBits(GPIOB, GPIO_Pin_15))
#define		DBG_PB15_UP 		(GPIO_SetBits(GPIOB, GPIO_Pin_15))	// PIN DEBUG DBG CN1-2. WP1		PB15
#define		DBG_PB15_DWN		(GPIO_ResetBits(GPIOB, GPIO_Pin_15))// PIN DEBUG DBG CN1-2. WP1		PB15
#define		DBG_PB8_UP			(GPIO_SetBits(GPIOB, GPIO_Pin_8)) 	// PIN DEBUG DBG CN1-3. SCL1	PB08
#define		DBG_PB8_DWN			(GPIO_ResetBits(GPIOB, GPIO_Pin_8)) // PIN DEBUG DBG CN1-3. SCL1	PB08
#define		DBG_PB7_UP			(GPIO_SetBits(GPIOB, GPIO_Pin_7)) 	// PIN DEBUG DBG CN1-3. SCL1	PB07
#define		DBG_PB7_DWN			(GPIO_ResetBits(GPIOB, GPIO_Pin_7)) // PIN DEBUG DBG CN1-3. SCL1	PB07
//---------------------------------------------------------------------------

//void HardFault_Handler(void)
//{
//  __ASM volatile("BKPT #01"); // add bkPnt
//  while(1);
//}

long getFwVersion(void);
long monthBuild( const char * u8aDate);
void MainAutoma_PID_Init(void);
void MainAutoma_GEN_Init(void);
void MainAutoma_CALYPSO_Init(void);

void MainInizialization(void);

//------------------------------------------------------------------------------
#ifdef  USE_FULL_ASSERT
void initAssertGpio(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	/* GPIOA and GPIOB clocks enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	/* GPIOA Configuration: Channel 1 and 3 as alternate function push-pull */

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_11 | GPIO_Pin_12 ;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
}
//------------------------------------------------------------------------------
/**
 * @brief  Reports the name of the source file and the source line number
 *			where the assert_param error has occurred.
 *			Use it as "assert_param(statementToBeTrue)"
 * @param  file		pointer to the source file name
 * @param  line		assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
	char assertString[128];
	unsigned long ii=0, kk;
	long lCounter;
	initAssertGpio();
	LED_WDOG_Init_Config();
	while(ii<128)
		assertString[ii++]= 0;

	sprintf(assertString,"%s on line %ul\r\n", file, line) ;

	// printf(assertString); // before enable Targert>Debug-Setting>Trace
	// see http://www.keil.com/support/man/docs/ulink2/ulink2_trace_itm_viewer.htm
	GPIO_SetBits(GPIOE, GPIO_Pin_11); // verde OFF
	GPIO_SetBits(GPIOE, GPIO_Pin_12); // Rosso OFF

	lCounter = 0x00FADE00U;
	ii=0;
	while(1)
	{
		while(ii < lCounter)
		{
			if( (ii&0x0000FFFFU)==0 )
			{
				kk = (ii >> 16);
				if (kk < 128 && assertString[kk] != 0 )
					USART_SendData(UART4,(uint16_t)assertString[kk]);
			}
			ii+=2;
			if((ii&1023) == 0)
				GPIO_ToggleBits(GPIOA, GPIO_Pin_10); // Buzzer TOGGLE
		}
		GPIO_ToggleBits(GPIOE, GPIO_Pin_12); // Rosso TOGGLE
		ii = 0;
	}
}
#else
void initAssertGpio(void)
{
	(void)0;
}
#endif
//---------------------------------------------------------------------------
#define WATCHDOG_PWM 0

TChronoMeter timer_DigIO; // timer to trigger IO refresh
TChronoMeter timer_Automi; // timer to trigger Automi refresh
TChronoMeter timer_AEC_SPI;  // timer to trigger AEC SPI refresh
TChronoMeter timer_Synch; // timer to trigger dummy refresh
TChronoMeter timerGreenLed; // timer to trigger LEd green refresh
TChronoMeter timerPolling;
TChronoMeter timer_SerAutoma;
TChronoMeter timerTest2_1;

/**
 * Working for Pre-Amplifier AEC Claymont 1001
 * Dip-SW1_ 0000
 * Dip-SW2  1000
 * Dip-SW3	10011100
 * @return
 */

int main (void)
{
	float faDbgAutomaTime[11];
	bool bNeedUpdateOut;//					

#if WATCHDOG_PWM /** @todo TO REMOVE BEFORE RELEASE xxx! ! !!!! */
	InitWatchDog_PWM();
#endif
	LED_WDOG_Init_Config();
	GPIO_SetBits(GPIOE, GPIO_Pin_12); // Rosso OFF
#ifdef _USB_DEBUG_
	tDbg.start();
#endif
#ifdef _RS232_DEBUG_
	tDbgRs232.start();
#else
#warning "NEED RS232 DEFINE ?"
#endif
	timer_Synch.start();
	timer_DigIO.start();
	timer_SerAutoma.start();


	while(1)
	{
//		if (timerPolling.exceed(50.1f))
//		{
//			bridgingCAN.executeSM();
//			timerPolling.start();
//		}
		if (timer_SerAutoma.exceed(50.1f)) //2000
		{

			automaSerial.executeSM();
			automaEth.executeSM();
			timer_SerAutoma.start();
		}
	}
}

//---------------------------------------------------------------------------
/**
 * @brief Setup required to initialize and use the LEDs and WDOG.
 *  WDOG is not initialized as PWM but as GPIO_Mode_OUT
 * @pre		Nope
 * @post	Nope
 */
void LED_WDOG_Init_Config(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	//	WATCHDOG TOGGLE LED
	/* GPIOE Periph clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	/* Configure PE12, PE11 in output push-pull mode 
	PE11 	 LED GREEN LED
	PE12 	 LED RED LED
	PE14	 LED WDOG
	 */

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14;
#if WATCHDOG_PWM
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13; // remove WDOG Pin
#endif
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOE, &GPIO_InitStructure);



	//	DBG PINS see CN1 for EEPROM SLOT
	//	CN1-1. GND 		(Near the LEDs)
	//	CN1-2. WP1		PB15	
	//	CN1-3. SCL1		PB08
	//	CN1-4. SDA1		PB07
	//	CN1-5. 3.3V
	//	CN1-6. GND
	//	CN1-7. SE1		PB14
	//	CN1-8. GND
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_Init(GPIOB, &GPIO_InitStructure);	

	//	WATCHDOG TOGGLE LED HAS TO BE SET IN THREAD AUTOMI 10ms
}
//-------------------------------------------------------------------------
long monthBuild( const char * u8aDate)
{
	long lMonth = 0;
	switch ( u8aDate[2] )
	{
	case 'n' :
		lMonth = 1 + 5*(u8aDate[1]=='u') ; break;
	case 'b' : lMonth = 2  ; break;
	case 'r' :
		lMonth = 3 + (u8aDate[1]=='p') ; break;
	case 'y' : lMonth = 5  ; break;
	case 'l' : lMonth = 7  ; break;
	case 'g' : lMonth = 8  ; break;
	case 'p' : lMonth = 9  ; break;
	case 't' : lMonth = 10 ; break;
	case 'v' : lMonth = 11 ; break;
	case 'c' : lMonth = 12 ; break;
	default: lMonth = 0; break;
	}
	return (lMonth);
}
//-------------------------------------------------------------------------
bool isDigit(char u8aData )
{
	return (u8aData >= '0' && u8aData<= '9' );
}
//-------------------------------------------------------------------------
long getFwVersion(void)
{
	// Example of __DATE__ string: "Jul 27 2012"
	// Example of __DATE__ string: "Jul  7 2012"
	// Example of __TIME__ string: "21:06:19"
	char u8aDate[16] = MMM_DD_YYYY_DATE; // date captured from linker output
	long lMonth = 0; // month
	long lDay = 0; // day 
	long lFwRelease;// FW release number

	lMonth = monthBuild(u8aDate);
	if (lMonth>0){
		if (isDigit(u8aDate[4] )){
			lDay = 10 * (u8aDate[4]-'0');
		}
		if (isDigit(u8aDate[5]) ){
			lDay += u8aDate[5]-'0';
		}
	}
	lFwRelease =
			(_FW_VER_MAJOR_<<24)+
			(_FW_VER_MINOR_<<16)+
			(  lMonth<< 8)+
			(  lDay      );
	return (lFwRelease);
}

#if ( defined(USE_DLL) && defined (USE_HIRIS))
#error "Conflicting defines: USE_DLL or USE_HIRIS ?"
#elif ( defined(USE_DLL) && defined (USE_HIRIS))
#error "Missing define: USE_DLL or USE_HIRIS ?"
#endif
