/******************************************************************************
 *
 *  \brief		Serial Port RS485 Class
 *  \details	This class is used to configure and use the Serial ports RS 485 on
 *  STM32F4 board
 *  \author	Ivan Perletti - General Medical Merate- GMM.spa - Seriate - ITALY
 *  \version	1.1a
 *  \date		January 25th, 2014
 *  \pre		Enable interrupt routine for each USART declared and used.
 *  \bug		Not all memory is freed when deleting an object of this class.
 *  \warning	Improper use can crash your application
 *  \copyright GMM.spa - All Rights Reserved
 *
 ******************************************************************************/

#include "TSerial485.h"
#include <stdio.h>

extern "C"{
#include "misc.h"
#include "stm32f4xx_rcc.h"
//#include "stm32f4_discovery.h"
#include "stm32f4xx_gpio.h"
}

long TSerial485::IdUserSerial485X[2] = {0,0};

TSerial485 comCM22(3);
TSerial485 comCM23(2);

char interruptTrap  = 0; // xxx DEBUG TRAP

usartRTSSetup setupRTS[] =
{
		{2,	GPIOD,	GPIO_Pin_4}, // RS485 usart2 with PD4
		{3, GPIOD, 	GPIO_Pin_12},// RS485 usart3 with PD12
};
//----Class Methods------------------------------------------------------------------------
/**
 * @brief Constructor
 * @param num 	Serial number to be initialized as Serial RS485
 */
TSerial485::TSerial485(unsigned char num):  TSerial (num)
{
	rtsNum = SearchRtsSetup(num);
	if(rtsNum>=0)
	{
		init_RS485_RTS(); // REady TO Send Pn sinitialization
		setupCOM(Baud9600, OneStop, NoParity, NoFlowControl, Data8);
	}
}
//-----------------------------------------------------------------------------------------
/**
 * @brief Distructor
 */
TSerial485::~TSerial485() {
	GPIO_DeInit(GPIOD);
}

//-----------------------------------------------------------------------------------------
/**
 * @brief Create subscriber pointer to specific Serial485 port
 * and assign unique identifier for the same serial physical port
 * @param pSerial485		pointer to serial485
 * @param serial_id			number of UART/USART port
 * @return s32Result	    instance ID
 */
long TSerial485::getInstance(TSerial485* &pSerial485, int serial_id)
{
	TSerial485* pSerialInterface;
	switch(serial_id)
	{
	case(2):
						pSerialInterface = &comCM23;
	break;
	case(3):
						pSerialInterface = &comCM22;
	break;

	default:
		break;
	}

	long s32Result = -1;

	if (s32Result + 1 < MAX_SERIAL485_NUM)
	{
		pSerial485 = pSerialInterface;

		// consider only subscriber on Serial485 CM23 CM22
		s32Result = IdUserSerial485X[serial_id-2];
		IdUserSerial485X[serial_id-2]++;
	}
	else
	{
		pSerial485 = NULL;
	}

	return s32Result;
}

//------------------------------------------------------------------------------
/**@brief Close specific instance (last user physically close port)
 * @param qID	instance identifier
 */
void TSerial485::closeInstance(long qID)
{
	if (qOpened.indexOf(qID) >= 0)
		qOpened.remove(qOpened.indexOf(qID));

	if (qOpened.size() == 0)
	{
		close();
	}
}
//-----------------------------------------------------------------------------

void TSerial485::closeAllUsers(void)
{
	qOpened.reset();
}
//------------------------------------------------------------------------------
/**@brief Open specific instance
 * @param qID	instance identifier
 */
char TSerial485::openInstance(long qID)
{
	if (qID >= 0 && qID < MAX_SERIAL485_NUM)
		{
			if (qOpened.indexOf(qID) < 0)
			{
				qOpened.push(qID);
				close();
				open();
			}
		}
}
//------------------------------------------------------------------------------
/**@brief Check if specific instance is open
 * @param qID	instance identifier
 */
char TSerial485::isInstance_open(long qID)
{
	return (isOpen() && (qOpened.indexOf(qID) >= 0));
}
//-----------------------------------------------------------------------------------------
/**
 * @brief init GPIO to be used as RTS flag. This pin has to be HIGH for all the TX procedure
 */
void TSerial485::init_RS485_RTS(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);	//Enable GPIOD peripheral clock

	GPIO_InitStruct.GPIO_Pin =  setupRTS[rtsNum].GPIO_Pin;	//Select LED GPIO pins to configure
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;		//Set pins to output
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;	//Set GPIO clock speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;		//Set pin type to push/pull
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;		//Set pullup/pulldown resistors to be inactive
	GPIO_Init(setupRTS[rtsNum].GPIOx, &GPIO_InitStruct);				//Initialize GPIOD

	// GPIO_SetBits(setupRTS[rtsNum].GPIOx,setupRTS[rtsNum].GPIO_Pin);// #set PIN
	GPIO_ResetBits(setupRTS[rtsNum].GPIOx,setupRTS[rtsNum].GPIO_Pin); // #reset PIN
	USART_ITConfig(USARTPORT, USART_IT_ERR, ENABLE); 
}
//-----------------------------------------------------------------------------------------
/**
 * @brief search inside struct if some setup correspond to the wished USART
 * @param usartID 		number of the interested UART to be searched
 */
int TSerial485::SearchRtsSetup(int usartID)
{
	int ii;
	int structSize = sizeof(setupRTS)/sizeof(setupRTS[0]);
	for(ii=0;ii<structSize ;ii++ )
	{
		if(usartID == setupRTS[ii].usartID)
			return(ii);
	}
	return (-1);
}
//-----------------------------------------------------------------------------
/**
 * @brief Interrupt handler routine: this is a static function so it doesn't
 * have access to private class parameters. these can be viewed passing the
 * class address and then using class1->parameter
 * @note  A NEW RX CHAR CANNOT OVERWRITE A SLOT IF NOT ALREADY READ
 * @note In the USART#_IRQHandler() include the class declaration file and
 * void USART1_IRQHandler(void) {  usart#.Interrupt_Handler(&usart#); }
 * @note Critical situation: tail is just behind head: new RX char cannot be
 * written in CircBuff while ReadPort makes slot free moving head ptr forward
 * Interrupt Source:
  -----------------
		1. USART_IT_TXE : specifies the interrupt source for the Tx buffer empty
						interrupt.
		2. USART_IT_RXNE : specifies the interrupt source for the Rx buffer not
						empty interrupt.
		3. USART_IT_TC : specifies the interrupt source for the Transmit complete
						interrupt.
		4. USART_IT_IDLE : specifies the interrupt source for the Idle Line interrupt.
		5. USART_IT_CTS : specifies the interrupt source for the CTS interrupt.
		6. USART_IT_LBD : specifies the interrupt source for the LIN break detection
						interrupt.
		7. USART_IT_PE : specifies the interrupt source for the parity error interrupt.
		8. USART_IT_ERR :  specifies the interrupt source for the errors interrupt.

		@todo avoid infinite message sending or infinite message receiving
 */

void  TSerial485::Interrupt_Handler(TSerial485 * usartN)
{
	static unsigned char yy;

	//int iTimOut; // TimeOut Counter
	interruptTrap=0;


	//  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
	//	Check if interrupt was generated by SEND task					SEND TSK
	if (USART_GetITStatus(usartN->USARTPORT, USART_IT_TXE) != RESET)
	{
		interruptTrap=0;
		usartN->tx_head &= (RXBUFFERSIZE-1);
		usartN->tx_tail &= (RXBUFFERSIZE-1);

		if (usartN->tx_tail		!=		usartN->tx_head )
		{
			GPIO_SetBits(setupRTS[rtsNum].GPIOx,setupRTS[rtsNum].GPIO_Pin);
			USART_SendData(usartN->USARTPORT, (uint16_t)usartN->bCircArrTx[usartN->tx_head]);
			usartN->tx_head++;
		}
		else {// obs: usartN->tx_tail  ==	usartN->tx_head // no other characters to transmit
			USART_ITConfig(usartN->USARTPORT, USART_IT_TC, ENABLE);
			USART_ITConfig(usartN->USARTPORT, USART_IT_TXE, DISABLE);

		}
	}

	//  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
	// Check if interrupt was generated by							RECEIVE TSK
	if (USART_GetITStatus(usartN->USARTPORT, USART_IT_RXNE) != RESET)
	{
		interruptTrap=1;
		USART_ClearITPendingBit(usartN->USARTPORT, USART_IT_RXNE);
		usartN->rx_head &= (RXBUFFERSIZE-1);
		usartN->rx_tail &= (RXBUFFERSIZE-1);
		yy =  USART_ReceiveData(usartN->USARTPORT);
		if (    ((usartN->rx_tail+1)&(RXBUFFERSIZE-1)) 
				!= usartN->rx_head )
		{// here enters only if there is enough space in RxCircBuff
			usartN->bCircArrRx[usartN->rx_tail] = yy;
			usartN->rx_tail++;
			usartN->rx_tail &= (RXBUFFERSIZE-1);
		}
		rxIRQ();
	}
	//  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
	//																TX COMPLETE
	

	if (USART_GetITStatus(usartN->USARTPORT, USART_IT_TC) != RESET)
	{
		interruptTrap=2;
		USART_ClearITPendingBit(usartN->USARTPORT, USART_IT_TC);
		USART_ITConfig(usartN->USARTPORT, USART_IT_TXE, DISABLE);
		GPIO_ResetBits(setupRTS[rtsNum].GPIOx,setupRTS[rtsNum].GPIO_Pin);
	}

	//#define USART_IT_TXE                         ((uint16_t)0x0727)

	//#define USART_IT_RXNE                        ((uint16_t)0x0525)
	//#define USART_IT_TC                          ((uint16_t)0x0626)

	//#define USART_IT_PE                          ((uint16_t)0x0028)


	//#define USART_IT_LBD                         ((uint16_t)0x0846)
	//#define USART_IT_CTS                         ((uint16_t)0x096A)
	//#define USART_IT_ERR                         ((uint16_t)0x0060)
	//#define USART_IT_NE                          ((uint16_t)0x0260)
	//#define USART_IT_FE                          ((uint16_t)0x0160)		
	//  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
	//																USART_IT_IDLE ERROR
	if (USART_GetITStatus(usartN->USARTPORT, USART_IT_IDLE) != RESET)	{
		interruptTrap=3;
	}
	//  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
	//																USART_IT_ORE_RX ERROR
	if (USART_GetITStatus(usartN->USARTPORT, USART_IT_ORE_RX) != RESET)	{
		interruptTrap=3;
	}
	//  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
	//																USART_IT_ORE_ER ERROR
	if (USART_GetITStatus(usartN->USARTPORT, USART_IT_ORE_ER) != RESET)	{
		interruptTrap=4;
	}	
	//  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
	//																NOISE ERROR
	if (USART_GetITStatus(usartN->USARTPORT, USART_IT_NE) != RESET)	{
		interruptTrap=5;
	}
	//  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
	//																FRAME ERR
	if (USART_GetITStatus(usartN->USARTPORT, USART_IT_FE) != RESET)	{
		interruptTrap=6;
	}
	//  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
	//																PARITY ERR
	if (USART_GetITStatus(usartN->USARTPORT, USART_IT_PE) != RESET)	{
		interruptTrap=7;
	}	

//	while (interruptTrap>2 && interruptTrap<11 ) ONLY FOR DEBUG TRAP
//		interruptTrap++;
}//end	Interrupt_Handler
//-----------------------------------------------------------------------------
/**
 * @brief Read single byte received and trigger callback
 * function for all subscriber by passing payload
 * (see TSerial232::registerRxCallback function)
 */
void TSerial485::rxIRQ(void)
{
	char payload;
	read(&payload); //read single byte

	for (int32_t ii = 0; ii < MAX_SERIAL485_CALLBACKS; ii++)
	{
		if (callbacks[ii].instance != NULL && qOpened.has(ii))
		{
			trampoline(&payload, callbacks[ii]);
		}
	}
}
//-----------------------------------------------------------------------------------------
void USART2_IRQHandler(void)
{
	comCM23.Interrupt_Handler(&comCM23);
}
//-----------------------------------------------------------------------------------------
void USART3_IRQHandler(void)
{
	comCM22.Interrupt_Handler(&comCM22);
}
//-----------------------------------------------------------------------------------------
