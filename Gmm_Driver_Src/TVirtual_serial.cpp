/*
 * TVirtual_Serial.cpp
 *
 *  Created on: 16 feb 2024
 *      Author: diaferia
 */

#include <string.h>
#include "TVirtual_Serial.h"

//------------------------------------------------------------------------------
TVirtual_Serial::TVirtual_Serial(int serial_id):
serial_TypeID(serial_id)
{

}
//------------------------------------------------------------------------------
TVirtual_Serial::~TVirtual_Serial(void)
{

}

//------------------------------------------------------------------------------
/**
 *  @brief Register instance to specific serial type port
 *  @ and assign the unique id number to the user (unique for users of the same physical port)
 *  @look at IdUserSerial232X in TSerial232.cpp
 */
void TVirtual_Serial::subscribe(void)
{
	switch(serial_TypeID)
	{
	case(4):
	case(5):
	case(6):
	serial_InstID = TSerial232::getInstance(pSerial232, serial_TypeID);
	if(pSerial232)
	{
		pSerial232->registerRxCallback(this, &TVirtual_Serial::SerialRxIRQ);
	}
	break;
	case(2):
	case(3):
	serial_InstID = TSerial485::getInstance(pSerial485, serial_TypeID);
	if(pSerial485)
	{
		pSerial485->registerRxCallback(this, &TVirtual_Serial::SerialRxIRQ);
	}
	break;
	default:
		break;
	}
}

//------------------------------------------------------------------------------
/**@brief Check if virtual serial port is RS-232
 */
bool TVirtual_Serial::is232Com(void){
	bool isSerial232 = false;
	if(serial_TypeID>=4)
	{
		isSerial232 = true;
	}
	return isSerial232;
}

//------------------------------------------------------------------------------
/**
 * @brief Set virtual serial COM setup up with this parameters set
 * (one user can set physical parameters)
 * @param baudPipp			baud rate
 * @param stopNum			stop bits
 * @param parityBitNum		parity bit
 * @param flowCtrlChoice	flow control method
 * @param dataBitNum		data bit number
 */
void TVirtual_Serial::setupCOM(baudRate baudPipp,
		stopBits stopNum ,
		parity parityBitNum,
		flowControl flowCtrlChoice,
		dataBits dataBitNum)
{
	if(is232Com())
	{
		pSerial232->setupCOM(baudPipp, stopNum, parityBitNum, flowCtrlChoice, dataBitNum);
	}
	else
	{
		pSerial485->setupCOM(baudPipp, stopNum, parityBitNum, flowCtrlChoice, dataBitNum);
	}
}

//------------------------------------------------------------------------------
/**
 * @brief Close virtually specific instance of TSerial232 or TSerial485
 */
void TVirtual_Serial::close(void)
{
	if(is232Com())
	{
		pSerial232->closeInstance(serial_InstID);
	}
	else
	{
		pSerial485->closeInstance(serial_InstID);
	}
}

//------------------------------------------------------------------------------

void TVirtual_Serial::closeAllUsers(void)
{
	if(is232Com())
	{
		pSerial232->closeAllUsers();
	}
	else
	{
		pSerial485->closeAllUsers();
	}
}
//------------------------------------------------------------------------------

/**
 * @brief Open specific instance of TSerial232 or TSerial485
 */
char TVirtual_Serial::open(void)
{
	unsigned char error = 0;
	if(is232Com())
	{
		error = pSerial232->openInstance(serial_InstID);
	}
	else
	{
		error = pSerial485->openInstance(serial_InstID);
	}

	return error;
}
//------------------------------------------------------------------------------
/**
 * @brief Check if virtual serial port is open
 */
bool TVirtual_Serial::isOpen(void)
{
	bool isSerialOpen = false;
	if(is232Com())
	{
		isSerialOpen = pSerial232->isInstance_open(serial_InstID);
	}
	else
	{
		isSerialOpen = pSerial485->isInstance_open(serial_InstID);

	}
	return isSerialOpen;
}

//------------------------------------------------------------------------------
/**
 * @brief Read from user rx queue one byte
 * @param rxByte	pointer to single byte received
 */
int TVirtual_Serial::read(char *rxByte)
{
	int qq=0;
	char cOneByte = 0x00;
	if(isToRead())
	{ //  new data to read on USART
		while( isToRead())
		{
			rxQ.pop(cOneByte);// if interrupt happens here NO PROBLEM
//			tDbg.write("in virtual ser: %d\r\n", cOneByte);

			rxByte[qq] = cOneByte;
//			rx_head++;// if interrupt happens here will be %RXBUFFERSIZE
//			rx_head%=RXBUFFERSIZE;
			qq++;
//			qq%=RXBUFFERSIZE;
		}
	}
	return(qq);//number of char it reads
//
//	int is_read = 0;
//	is_read = rxQ.pop(rxByte);
//
//	return is_read;
}
//------------------------------------------------------------------------------
/**
 * @brief Check if rx queue contains some bytes to read
 */
bool TVirtual_Serial::isToRead(void)
{
	return rxQ.size()>0;
}

//------------------------------------------------------------------------------
/**
 * @brief Check if there is anything to read from rx queue
 * @return the number of character to be read
 */
int TVirtual_Serial::bytesAvailable(void)
{
	return rxQ.size();
}

//------------------------------------------------------------------------------
unsigned char TVirtual_Serial::write(const volatile char *msg,
		short int charNum2Send)
{
	//	char single_byte;
	//
	//	for(int ii=0; ii<charNum2Send; ii++)
	//	{
	//		single_byte = msg[ii];
	//		txQ.push(single_byte);
	//	}

	// Direct access on singleton
	if(is232Com())
	{
		pSerial232->write(msg,charNum2Send);
	}
	else
	{
		pSerial485->write(msg,charNum2Send);
	}
}
//------------------------------------------------------------------------------
void TVirtual_Serial::write(void)
{
	//Generate TX Interrupt
	if(is232Com())
		{
			pSerial232->releaseIT_TX();
		}
		else
		{
			pSerial485->releaseIT_TX();
		}
}
//------------------------------------------------------------------------------

int TVirtual_Serial::bytesToWrite(void)
{
	int n_bytes = 0;

	if(is232Com())
	{
		n_bytes = pSerial232->bytesToWrite();
	}
	else
	{
		n_bytes = pSerial485->bytesToWrite();
	}

	return n_bytes;
}

//------------------------------------------------------------------------------
/**
 * @brief Interrupt service function that push in user rx queue
 * single byte readed from physical serial port
 * @param payload 	byte readed
 */
void TVirtual_Serial::SerialRxIRQ(char *payload)
{
	rxQ.push(payload);
}
//------------------------------------------------------------------------------
void TVirtual_Serial::SerialTxIRQ(char *payload)
{
	//	char payload[64];
	//	for(int ii=0;)
	//	txQ.pop(&payload);
}
//--------------------------------------------------------------------------------

void TVirtual_Serial::resetBytesRX(void)
{
	rxQ.reset();
}
