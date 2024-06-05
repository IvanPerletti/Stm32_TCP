
/******************************************************************************
 *	@name		TChronoMeter.h
 *  @brief		Chronometer class header
 *  @details	This class is used to configure and use a chronometer class.
 *  	As default it accepts an STM32 1ms-period-timer
 *  @author		Ivan Perletti - General Medical Merate- GMM.spa - Seriate - ITALY
 *  @version	1.0
 *  @date		September 15th, 2014
 *  @pre		Have to configure an external TClock class to pass as intput
 *  @bug		Not all memory is freed when deleting an object of this class.
 *  @warning	Improper use can crash your application
 *  @copybrief	 GMM.spa - All Rights Reserved
 *
 ******************************************************************************/

#ifndef TCHRONOMETER_H
#define TCHRONOMETER_H

#include "TClock.h"
extern "C"
{

}
extern "C" {
#include "stm32f4xx_tim.h"
#include "misc.h"
#include "stm32f4xx_rcc.h"
//#include "stm32f4_discovery.h"
void TIM2_IRQHandler(void);
}
class TChronoMeter
{
private:
	TClock* universalClock;
	unsigned long u32Tick0;
	unsigned long u32Tick1;
	bool bPaused;
	float fTicPrescaler; /// prescaler for the TChronometer
public:
	TChronoMeter();
	virtual ~TChronoMeter();
	void start(void);
	void pause(void);
	void resume(void);
	float watch(void);
//	bool exceed(long lTimeThreshold);
	bool exceed(float lTimeThreshold);
	/**
	 * @brief It adds positive or negative Time Delay
	 * @param lDeltaTime 	amount of time to be shifted
	 */
	inline void shift(const float fDeltaTime){u32Tick0-=fDeltaTime*fTicPrescaler;};
	void stopReset(void);
	float now(void);
};

#endif // TCHRONOMETER_H
