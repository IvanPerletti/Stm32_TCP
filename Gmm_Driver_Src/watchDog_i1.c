/**
 *  \brief		Activates the Watch-Dog pin by producing 1kHz PWM with period 0.5
 *  \details	This class is used to configure PE14 PWM at 1kHz
 *  \author	Ivan Perletti - General Medical Merate- GMM.spa - Seriate
 *  \version	1.00
 *  \date		February 25th, 2014
 *  \pre		Simply include the header file "watchDog_i1.h"
 *  \warning	Improper use may crash your application
 *  \copyright GMM.spa - All Rights Reserved
 */
///----------------------------------------------------------------------------------------
#include "watchDog_i1.h"

#include "misc.h"
#include "stm32f4xx_rcc.h"
//#include "stm32f4_discovery.h"
//#include "stm32f4xx_gpio.h"

/**
 * @brief Initializes the 1KHz PWM on the PE14 to keep wathDog Alive
 * @param milliPeriod 		stay UP period [0:1000]
 */
void InitWatchDog_PWM(void)
{
	GPIO_InitTypeDef		GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef		TIM_OCInitStructure;

	// Timer1 1 CH: PE9	CH1
	//			2 CH: PE11	CH2
	//			3 CH: PE13	CH3
	//			4 CH: PE14	CH4
	// Reset Timer 1

	// TIM_DeInit(TIM1);

	//	GPIO_InitStructure.GPIO_Pin	= (GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_14);
	GPIO_InitStructure.GPIO_Pin	= GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP ;

	/* PWM1 Mode configuration: Channel3 */
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode 				= TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState  = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
	TIM_OCInitStructure.TIM_Pulse		= 500;
	TIM_OCInitStructure.TIM_OCPolarity 		= TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity  = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState  = TIM_OCIdleState_Reset;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

	// Clock enable
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	// GPIO Pins function as TIM1 pins
	//	GPIO_PinAFConfig(GPIOE, GPIO_PinSource9,  GPIO_AF_TIM1);
	//	GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1);
	//	GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_TIM1);
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	//			TIM_OC1Init(TIM1, &TIM_OCInitStructure); // PE9
	//			TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
	//
	//			TIM_OC2Init(TIM1, &TIM_OCInitStructure); // PE11 PWM
	//			TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
	//
	//			// Timer Stuff
	//			TIM_OC3Init(TIM1, &TIM_OCInitStructure); // PE 13
	//			TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);

	TIM_OC4Init(TIM1, &TIM_OCInitStructure); // PE 14
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);

	// ** required for timers 1 or 8 **
	// Enables or disables the TIM peripheral Main Outputs.
	TIM_CtrlPWMOutputs(TIM1, ENABLE);

	/* TIM1 enable counter */
	TIM_Cmd(TIM1, ENABLE);
	playWatchDog();
	TIM_TimeBaseStructure.TIM_Prescaler		= (168-1);//TIM_Prescaler * 8MhZ HSE = N_tick
	TIM_TimeBaseStructure.TIM_Period		= (5000-1);
	TIM_TimeBaseStructure.TIM_CounterMode	= TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
}
///----------------------------------------------------------------------------------------
void playWatchDog(void)
{
	TIM_SetCompare4(TIM1,500); // set brightness
}
///----------------------------------------------------------------------------------------
void killWatchDog(void)
{
	TIM_SetCompare4(TIM1,1); // set brightness
}
///----------------------------------------------------------------------------------------
/**
 * @brief Set the period of the PWM acting on the WatchDog pin.
 * @param milliPeriod 		period of PWM up [0:999];
 */
void setWatchDogPeriodUp(unsigned int  milliPeriod)
{
	if(milliPeriod>999)
		milliPeriod = 999;
	TIM_SetCompare4(TIM1,milliPeriod%1000); // set brightness
}
