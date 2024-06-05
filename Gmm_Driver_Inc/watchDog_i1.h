/*
 * watchDog_i1.h
 *
 *  Created on: 25/feb/2014
 *		Author: perletti
 */

#ifndef WATCHDOG_I1_H_
#define WATCHDOG_I1_H_

#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"

void InitWatchDog_PWM(void);
void setWatchDogPeriodUp(unsigned int  milliPeriod);
void playWatchDog(void);
void killWatchDog(void);

#endif /* WATCHDOG_I1_H_ */
