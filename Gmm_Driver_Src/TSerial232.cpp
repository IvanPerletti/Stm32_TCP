/******************************************************************************
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
#include "TSerial232.h"

TSerial232 comCM3(6); // pid rad
TSerial232 comCM24(5); // generator
TSerial232 comCM25(4); // aux Serial

long TSerial232::IdUserSerial232X[3] = {0,0,0};

// If i subscribe two instances (users) of singleton comCM3 for example the first has the serial_id : 0 , the second has the serial_id 1
// If i subscribe two instances (users) of singleton comCM24 the first has the serial_id : 0 , the second has the serial_id 1
// The same for comCM25

//------------------------------------------------------------------------------
TSerial232::TSerial232(unsigned char num):TSerial (num),rtsNum(0)
{
	setupCOM(Baud19200, TwoStop, NoParity, NoFlowControl, Data8);
}
//------------------------------------------------------------------------------
TSerial232::TSerial232(unsigned char num,
		baudRate baudPipp=Baud19200,
		stopBits stopNum = OneStop,
		parity parityBitNum = NoParity,
		flowControl flowCtrlChoice = NoFlowControl,
		dataBits dataBitNum = Data8):TSerial (num)
{
	rtsNum = 0;
	setupCOM(baudPipp, stopNum, parityBitNum, flowCtrlChoice, dataBitNum);
}
//------------------------------------------------------------------------------
TSerial232::~TSerial232() {
}
//------------------------------------------------------------------------------
/**
 * @brief Create subscriber pointer to specific Serial232 port
 * and assign unique identifier (instance ID) for the same serial physical port
 * @param pSerial232		pointer to serial232
 * @param serial_id			number of UART/USART port
 * @return s32Result	    instance ID
 */
long TSerial232::getInstance(TSerial232* &pSerial232, int serial_id)
{
	TSerial232* pSerialInterface;
	switch(serial_id)
	{
	case(4):
						pSerialInterface = &comCM25;
	break;
	case(5):
						pSerialInterface = &comCM24;
	break;
	case(6):
						pSerialInterface = &comCM3;
	break;
	default:
		break;
	}

	long s32Result = -1;

	if (s32Result + 1 < MAX_SERIAL232_NUM)
	{
		pSerial232 = pSerialInterface;

		// consider only subscriber on Serial232 CM25 CM24 or CM3
		s32Result = IdUserSerial232X[serial_id-4];
		IdUserSerial232X[serial_id-4]++;
	}
	else
	{
		pSerial232 = NULL;
	}

	return s32Result;
}

//------------------------------------------------------------------------------
/**@brief Close specific instance (last user physically close port)
 * @param qID	instance identifier
 */
void TSerial232::closeInstance(long qID)
{
	if (qOpened.indexOf(qID) >= 0)
		qOpened.remove(qOpened.indexOf(qID));

	if (qOpened.size() == 0)
	{
		close();
	}
}
//-----------------------------------------------------------------------------
/**
 * @brief Reset the queue of all the opened users
 */

void TSerial232::closeAllUsers(void)
{
	qOpened.reset();
}
//------------------------------------------------------------------------------
/**@brief Open specific instance
 * @param qID	instance identifier
 */
char TSerial232::openInstance(long qID)
{
	if (qID >= 0 && qID < MAX_SERIAL232_NUM)
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
char TSerial232::isInstance_open(long qID)
{
	return (isOpen() && (qOpened.indexOf(qID) >= 0));
}
//------------------------------------------------------------------------------
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
  ------------------
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

 */

//OLD TSERIAL WORKING HANDLER
void  TSerial232::Interrupt_Handler(TSerial232 * usartN)
{
	static unsigned char yy;
	





	//char interruptTrap = 0;
	// int iTimOut; // TimeOut Counter
	//  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
	//	Check if interrupt was generated by SEND task					SEND TSK
	if (USART_GetITStatus(usartN->USARTPORT, USART_IT_TXE) != RESET)
	{
//#ifdef _USB_DEBUG_
//	tDbg.write((char*)"in interr tx");
//#endif
		usartN->tx_head &= (RXBUFFERSIZE-1);
		usartN->tx_tail &= (RXBUFFERSIZE-1);
		if (usartN->tx_tail		!=		usartN->tx_head )
		{
			USART_SendData(usartN->USARTPORT,(uint16_t) usartN->bCircArrTx[usartN->tx_head]);
			usartN->tx_head++;
		}
		else {// usartN->tx_tail  ==	usartN->tx_head // no other characters to transmit
			USART_ITConfig(usartN->USARTPORT, USART_IT_TC, ENABLE);
			USART_ITConfig(usartN->USARTPORT, USART_IT_TXE, DISABLE);
		}
	}


	//  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
	// Check if interrupt was generated by							RECEIVE TSK
	if (USART_GetITStatus(usartN->USARTPORT, USART_IT_RXNE) != RESET)
	{
//#ifdef _USB_DEBUG_
//	tDbg.write((char*)"in interr rx");
//#endif

		usartN->rx_head &= (RXBUFFERSIZE-1);
		usartN->rx_tail &= (RXBUFFERSIZE-1);
		yy =  USART_ReceiveData(usartN->USARTPORT);
		if (       ((usartN->rx_tail+1)&(RXBUFFERSIZE-1))
				!= usartN->rx_head/**/)
		{// here enters only if there is enough space in RxCircBuff
			usartN->bCircArrRx[usartN->rx_tail] = yy;
			usartN->rx_tail++;
			usartN->rx_tail &= (RXBUFFERSIZE-1);

//#ifdef _USB_DEBUG_
//	tDbg.write((char*)"rx_head ");
//	tDbg.write((char*)"%d  ", rx_head);
//	tDbg.write((char*)"rx_tail ");
//	tDbg.write((char*)"%d  ", rx_tail);
//#endif
			
		}
		rxIRQ();
	}

	if (USART_GetITStatus(usartN->USARTPORT, USART_IT_TC) != RESET)
	{
//#ifdef _USB_DEBUG_
//	tDbg.write((char*)"interrupt USART_IT_TC");
//#endif

		USART_ClearITPendingBit(usartN->USARTPORT, USART_IT_TC);
		USART_ITConfig(usartN->USARTPORT, USART_IT_TXE, DISABLE);
	}
	//  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
	//																USART_IT_ORE_RX ERROR
	if (USART_GetITStatus(usartN->USARTPORT, USART_IT_ORE_RX) != RESET)	{

#ifdef _USB_DEBUG_
	tDbg.write((char*)"IN USART_IT_ORE_RX    ");
#endif

		// interruptTrap=3;
		USART_ClearITPendingBit(usartN->USARTPORT, USART_IT_ORE_RX);
	}
	//  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
	//																USART_IT_ORE_ER ERROR
	if (USART_GetITStatus(usartN->USARTPORT, USART_IT_ORE_ER) != RESET)	{

#ifdef _USB_DEBUG_
	tDbg.write((char*)"IN USART_IT_ORE_RX    ");
#endif
	// interruptTrap=4;
		USART_ClearITPendingBit(usartN->USARTPORT, USART_IT_ORE_ER);
	}	
	//  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
	//																NOISE ERROR
	if (USART_GetITStatus(usartN->USARTPORT, USART_IT_NE) != RESET)	{

#ifdef _USB_DEBUG_
	tDbg.write((char*)"IN USART_IT_ORE_ER    ");
#endif
		// interruptTrap=5;
		USART_ClearITPendingBit(usartN->USARTPORT, USART_IT_NE);
	}
	//  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
	//																PARITY ERR
	if (USART_GetITStatus(usartN->USARTPORT, USART_IT_FE) != RESET)	{

#ifdef _USB_DEBUG_
	tDbg.write((char*)"IN USART_IT_FE    ");
#endif
		// interruptTrap=6;
		USART_ClearITPendingBit(usartN->USARTPORT, USART_IT_FE);
	}
	//  . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
	//																PARITY ERR
	if (USART_GetITStatus(usartN->USARTPORT, USART_IT_PE) != RESET)	{

#ifdef _USB_DEBUG_
	tDbg.write((char*)"IN USART_IT_PE    ");
#endif
		// interruptTrap=7;
		USART_ClearITPendingBit(usartN->USARTPORT, USART_IT_PE);
	}	
	/*            @arg USART_IT_IDLE: Idle line detection interrupt
	 *            @arg USART_IT_ORE_RX : OverRun Error interrupt if the RXNEIE bit is set
	 *            @arg USART_IT_ORE_ER : OverRun Error interrupt if the EIE bit is set
	 *            @arg USART_IT_NE:   Noise Error interrupt
	 *            @arg USART_IT_FE:   Framing Error interrupt
	 *            @arg USART_IT_PE:   Parity Error interrupt
	 */
	//	while (interruptTrap>2 && interruptTrap<11 ) ONLY FOR DEBUG TRAP
	//		interruptTrap++;
}//end	Interrupt_Handler

//-----------------------------------------------------------------------------
/**
 * @brief Read bytes received on serial port and trigger callback
 * function for all subscriber
 * (see TSerial232::registerRxCallback function)
 */
void TSerial232::rxIRQ(void)
{
	char payload;
	read(&payload); //physically read payload on serial port

	// copy payload for all users RX queue
	for (int32_t ii = 0; ii < MAX_SERIAL232_CALLBACKS; ii++)
	{
		if (callbacks[ii].instance != NULL && qOpened.has(ii))
		{
			trampoline(&payload, callbacks[ii]);
		}
	}
}

//-----------------------------------------------------------------------------
/**
 * @brief  This function handles USART1 global interrupt request.
 * @param  None
 * @retval None
 */
void UART4_IRQHandler(void)
{
	comCM25.Interrupt_Handler(&comCM25);
}
//------------------------------------------------------------------------------
void UART5_IRQHandler(void)
{
	comCM24.Interrupt_Handler(&comCM24);
}
//------------------------------------------------------------------------------
void USART6_IRQHandler(void)
{
	comCM3.Interrupt_Handler(&comCM3);
}
