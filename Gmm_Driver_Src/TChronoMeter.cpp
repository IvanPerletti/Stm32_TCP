/******************************************************************************
 *	@name		TChronoMeter.cpp
 *  @brief		Chronometer class
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

#include "TChronoMeter.h"

// max signed long  2147483647
// ----------------------------------------------------------------------------
extern TClock tClock;
/**
 * @brief Constructor - initialization as default
 * @pre 	tClock has to be already declared and implemented
 */
TChronoMeter::TChronoMeter()
: 	universalClock(&tClock),
  	u32Tick0(0),
  	u32Tick1(0),
  	bPaused(0),
  	fTicPrescaler(10)
{
	start();
}
// ----------------------------------------------------------------------------
/**
 * @brief 	Destructor - deInitialization
 */
TChronoMeter::~TChronoMeter()
{
	stopReset();
}
// ----------------------------------------------------------------------------
/**
 * @brief Resume and continues to count clocks
 */
void TChronoMeter::start(void)
{
	bPaused = false;
	u32Tick0 = universalClock->watch();
}
// ----------------------------------------------------------------------------
/**
 * @brief Stops counting clocks WITHOUT resetting counters
 */
void TChronoMeter::pause(void)
{
	if (bPaused == false)
		u32Tick1 = universalClock->watch();
	bPaused = true;
}
// ----------------------------------------------------------------------------
/**
 * @brief Resumes counting clocks WITHOUT resetting counters
 */
void TChronoMeter::resume(void)
{
	if (bPaused == true)
		u32Tick0 = universalClock->watch() - (u32Tick1 - u32Tick0);
	bPaused = false;
}
// ----------------------------------------------------------------------------
/**
 * @brief Stops counting clocks WITHOUT resetting counters
 */
float TChronoMeter::watch(void)
{
	float fDelta;
	if (bPaused == false)
		u32Tick1 = universalClock->watch();

	//	if (u32Tick1 < u32Tick0)
	//		delta = 0;
	//	else
	fDelta = u32Tick1 - u32Tick0;

	return(fDelta / fTicPrescaler);
}
// ----------------------------------------------------------------------------
/**
 * @brief Stops counting clocks WITHOUT resetting counters
 */
bool TChronoMeter::exceed(float fTimeThreshold)
{
	bool bVar = 0;
	float fDelta = watch(); // ms
	if (fDelta >= fTimeThreshold)
		bVar = 1;
	return(bVar);
}
// ----------------------------------------------------------------------------
/**
 * @brief Stops counting clocks and resetting counters
 */
void TChronoMeter::stopReset(void)
{
	u32Tick0 = universalClock->watch();
	u32Tick1 = u32Tick0;
	bPaused = false;
}
// ----------------------------------------------------------------------------
/**
 * @brief Estimate the actual time now TIME
 */
float TChronoMeter::now(void)
{
	//	DBG		static long long emulTime = 0;
	//	DBG		return(emulTime++);
	return( universalClock->watch()/fTicPrescaler );
}
