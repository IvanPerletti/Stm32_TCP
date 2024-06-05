/******************************************************************************
 *	@name		TClock.cpp
 *  @brief		Timer Clock class
 *  @details	This class is used to configure and use an STM32F4 Timer.
 *  	By default it is working on the TIM2 clock
 *  @author		Ivan Perletti - General Medical Merate- GMM.spa - Seriate - ITALY
 *  @version	1.0
 *  @date		September 9th, 2014
 *  @pre		Have to configure an external TClock class to pass as intput
 *  @bug		Not all memory is freed when deleting an object of this class.
 *  @warning	Improper use can crash your application
 *  @copybrief	 GMM.spa - All Rights Reserved
 *	@remarks 	Please check and set up the system_stm32f4xx.c
 *			 	*		#define PLL_M      8
 *			 	*		#define PLL_N      336
 *				*
 *				*		//SYSCLK = (PLL_VCO / PLL_P)
 *				*		#define PLL_P      2
 *				*
 *				*		//USB OTG FS, SDIO and RNG Clock =  (PLL_VCO / PLLQ)
 *				*		#define PLL_Q      7
 *
 * 		in MakeFile define "HSE_VALUE=8000000" External oscillator frequency
 ******************************************************************************/

#include "TClock.h"

//--Private Definitions -------------------------------------------------------------------
/** bExtAllowOtherInit: Mask describing the Chrono Status :
 * - 0x00 no settings
 * - 0x01 initialization successful*/
bool bExtAllowOtherInit = true;

TClock tClock;
//--Class methods -------------------------------------------------------------------------

/**
 * @brief Constructor
 * @param timersNum number of timers to initialize
 */
TClock::TClock(void)
:	lChronoTick(0)
{
	if(bExtAllowOtherInit){
		char s8InitOk = init(bExtAllowOtherInit);
		assert_param( s8InitOk );
		bExtAllowOtherInit = false;
	}
}
// ----------------------------------------------------------------------------------------
/**
 * @brief 		Destructor
 */
TClock::~TClock() {
	lChronoTick = 0;
	stopReset();
	bExtAllowOtherInit = true;
}
// ----------------------------------------------------------------------------------------
/**
 * @brief 		NVIC TIM RCC_APB Initialization
 * @param 		variable to avoid double initialization
 * @remarks		thought "bExtAllowOtherInit" to be passed as input
 * @return		0x01 if init ok
 * 				0x00 if init didn't have success
 */
char TClock::init(bool bAllowOtherInit)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	uint16_t Period;

	/* GPIOE Periph clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	if(bAllowOtherInit)
	{
		// PB8 is the SDA1 for I2C pin for the Moog-EEPROM
		// it is the pin 4 on the strip-male connector
		/* Configure PB8 in output push-pull mode */
		GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
		/* Enable the TIM2 global Interrupt */
		NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);

		/* TIM2 clock enable */
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

		/* in system_stm32f4xx.c
		 *		#define PLL_M      8
		 *		#define PLL_N      336
		 *
		 *		//SYSCLK = (PLL_VCO / PLL_P)
		 *		#define PLL_P      2
		 *
		 *		//USB OTG FS, SDIO and RNG Clock =  (PLL_VCO / PLLQ)
		 *		#define PLL_Q      7
		 *
		 * in MakeFile define "HSE_VALUE=8000000" External oscillator frequency*/
		/* Time base configuration */
		
		TIM_TimeBaseStructure.TIM_Prescaler = ((168000000 / 1000000) / 2) - 1; // 8 MHz Clock down to 1 MHz (adjust per your external clock)
		Period = 100; // tics of 1MHz clock == 1Mhz / Period
		TIM_TimeBaseStructure.TIM_Period = Period - 1; // {x2} 1 MHz down to 1 KHz (1 ms)
		TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

		stopReset();// stopReset all the counters
		/* TIM IT enable */
		TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
		/* TIM2 enable counter */
		TIM_Cmd(TIM2, ENABLE);
		return(0x01);
	}
	return(0x00);
}
// ----------------------------------------------------------------------------------------
/**
 * @brief Start a wished ChronoTimer: you can use up to /chronoDimension/ different timers
 * @param chronoTimerNumber: number of the timer to initialized
 * @return error:
 			  - 0x00 no error
 	 	 	  - &=0x01 WARNING class not initialized: used an init_Chrono() call
 	 	 	  - &=0x02 WARNING timer already initialized, start Time overwritten
 	 	 	  - &=0x04 not valid timer Number
 */
void TClock::start(void)
{
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);/* TIM IT enable */
	TIM_Cmd(TIM2, ENABLE);/* TIM2 enable counter */
}
// ----------------------------------------------------------------------------------------
/**
 * Read the actual time-tick-counter
 * @return the time-Tick-Counter
 * @remarks it is a long long type variable
 */
unsigned long TClock::watch(void)
{
	return (lChronoTick);
}
// ----------------------------------------------------------------------------------------
/**
 * @brief Stop the Clock freezing the Interrupt routine and TIM# peripheral. As a
  consequence the /lChronoTick/ stays frozen
 */
void TClock::freeze(void)
{
	TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE); // freeze interrupt routine
	TIM_Cmd(TIM2, DISABLE); // disable timer
}
// ----------------------------------------------------------------------------------------
/**
 * @brief reset all the ChronoTimers
 */
void TClock::stopReset(void)
{
	lChronoTick = 0;
	TIM2->CNT = 0x1;
}
// ----------------------------------------------------------------------------------------
/**
 * @brief Decrease the Timing counter some milliseconds back
 * @param lMoveUpDelay
 */
void TClock::delay(long int lMoveUpDelay)
{
	if (lMoveUpDelay > 0)
		lChronoTick -= lMoveUpDelay;
}
// ----------------------------------------------------------------------------------------
/**
 * @brief 	main routine called in the C function "TIMx_IRQHandler"
 * @param ttClock the *itself* pointer [due to fights between C and CPP]
 */
void TClock::routineIRQ( TClock *ttClock)
{
	GPIO_ToggleBits(GPIOB, GPIO_Pin_7); // TOGGLE
	ttClock->lChronoTick++;
}
// ----------------------------------------------------------------------------------------
/**
 * @brief STM32F4 "C" default Interrupt Request Routine
 */
void TIM2_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		tClock.routineIRQ(&tClock);
	}
}
