/*
 * TSerial485.h
 *
 ******************************************************************************
 *
 *  \brief		Serial Port Class
 *  \details	This class is used to configure and use the Serial ports on
 *  STM32F4 board
 *  \author	Ivan Perletti - General Medical Merate- GMM.spa - Seriate
 *  \version	1.1a
 *  \date		January 15th, 2014
 *  \pre		Enable interrupt routine for each USART declared and used. Ex:
 * void USART3_IRQHandler(void) { usart3.Interrupt_Handler(&usart3); }
 * and add extern TSerial usart3; in the top .c file
 *  \bug		Not all memory is freed when deleting an object of this class.
 *  \warning	Improper use can crash your application
 *  \copyright GMM.spa - All Rights Reserved
 *
 ******************************************************************************/

#ifndef TSERIAL485_H_
#define TSERIAL485_H_

#define MAX_SERIAL485_NUM 5

#include "TSerial.h"
#include "TSerial485.h"
#include "IQ_Generic.h"

#include <stdio.h>

#define MAX_SERIAL485_CALLBACKS 5

//----Private Struct-----------------------------------------------------------------------
/**
 * @brief struct: here are listed all the USART parameters to work with
 * STM32F4 board
 * @note UART5 RX & TX pins belong to 2 different GPIOx family. The setup
 * procedure will behave differently if UART5 is called. For this
 * reason the GPIOx value will be neglected
 */
typedef struct  {
	int usartID;/*! USART name number among: 1,2,3,4,5,6*/
	GPIO_TypeDef * GPIOx;/*! like: GPIOD*/
	unsigned short int GPIO_Pin;/*! like:  GPIO_Pin_6 | GPIO_Pin_7*/
} usartRTSSetup;


//--- Class -------------------------------------------------------------------------------

class TSerial485 : public TSerial {
	int rtsNum; /*! RTS struct position */
public:

	TSerial485(unsigned char num);
	virtual ~TSerial485();

	static long getInstance(TSerial485* &pSerial485, int serial_id);
	static long IdUserSerial485X[2];
	static long s32CurrentQ;

	IQ_Generic<long, MAX_SERIAL485_NUM> qOpened;
	void closeInstance(long qID);
	void closeAllUsers(void);

	char openInstance(long qID);
	char isInstance_open(long qID);

	void Interrupt_Handler(TSerial485 * usartN);
	void rxIRQ(void);

private:
	void init_RS485_RTS(void);
	int SearchRtsSetup(int usartID);

};
//--- STM32F4 interrupt -------------------------------------------------------------------
extern "C" {
void USART2_IRQHandler(void);
void USART3_IRQHandler(void);
}

extern TSerial485 comCM22;
extern TSerial485 comCM23;

#endif /* TSERIAL485_H_ */
