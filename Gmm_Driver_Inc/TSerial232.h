/*
 * TSerial232.h
 *
 ******************************************************************************
 *
 *  \brief		Serial Port Class
 *  \details	This class is used to configure and use the Serial ports on
 *  STM32F4 board
 *  \author	Ivan Perletti - General Medical Merate- GMM.spa - Seriate - ITALY
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

#ifndef TSERIAL232_H_
#define TSERIAL232_H_

#include <stdio.h>
#include "IQ_Generic.h"

#include "TSerial.h"

#include "stm32f4xx.h"
#include "misc.h"			// I recommend you have a look at these in the ST firmware folder
#include <stm32f4xx_usart.h> // under Libraries/STM32F4xx_StdPeriph_Driver/inc and src
//#include "stm32f4_discovery.h"

#define MAX_SERIAL232_NUM 5
#define MAX_SERIAL232_CALLBACKS MAX_SERIAL232_NUM

#define WRITE_MULTIPLE_CHAR

class TSerial232 : public TSerial {
	int rtsNum; /*! RTS struct position */
public:

	TSerial232(unsigned char num);
	TSerial232(unsigned char num,
			baudRate baudPipp,
			stopBits stopNum ,
			parity parityBitNum,
			flowControl flowCtrlChoicel,
			dataBits dataBitNum);
	virtual ~TSerial232();

	static long getInstance(TSerial232* &pSerial232, int serial_id);
	static long IdUserSerial232X[3];
	static long s32CurrentQ;

	IQ_Generic<long, MAX_SERIAL232_NUM> qOpened;
	void closeInstance(long qID);
	void closeAllUsers(void);
	char openInstance(long qID);
	char isInstance_open(long qID);

	void Interrupt_Handler(TSerial232 * usartN);
	void rxIRQ(void);
};

// Singleton Serial232
extern TSerial232 comCM3; // digital PC
extern TSerial232 comCM24; // generator
extern TSerial232 comCM25; // aux
//--- STM32F4 interrupt -------------------------------------------------------------------
extern "C" {
void UART4_IRQHandler(void);
void UART5_IRQHandler(void);
void USART6_IRQHandler(void);
}
//-----------------------------------------------------------------------------------------
#endif /* TSERIAL232_H_ */
