/******************************************************************************
 *	@name		TClock.h
 *  @brief		Timer Clock class header
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

#ifndef TCLOCK_H
#define TCLOCK_H

extern "C" {
#include "stm32f4xx_tim.h"
#include "stm32f4xx_gpio.h"
#include "misc.h"
#include "stm32f4xx_rcc.h"
//#include "stm32f4_discovery.h"
void TIM2_IRQHandler(void);
}



class TClock {
private:
	/*! Protected [1ms] ChronoTick
			@remark Pay attention to the HSE value and the External Quartz frequency
			@remark kept upper in the stack for fast access reason*/
	unsigned long lChronoTick;

public:
	TClock();
	virtual ~TClock();
	void start(void);
	unsigned long watch(void);
	void freeze(void);
	void stopReset(void);
	void delay(long int lMoveUpDelay);
	void routineIRQ( TClock *ttClock);
private:
	char init(bool allowOtherInit);
};

extern TClock tClock;

#endif /* TCLOCK_H */
