/*
 * TVirtual_Eth.cpp
 *
 *  Created on: 16 feb 2024
 *      Author: diaferia
 */

#include <string.h>
#include "TVirtual_Eth.h"

//------------------------------------------------------------------------------
TVirtual_Eth::TVirtual_Eth()
{

}
//------------------------------------------------------------------------------
TVirtual_Eth::~TVirtual_Eth(void)
{

}

//------------------------------------------------------------------------------
/**
 *  @brief Register instance to specific serial type port
 *  @ and assign the unique id number to the user (unique for users of the same physical port)
 *  @look at IdUserSerial232X in TSerial232.cpp
 */
void TVirtual_Eth::subscribe(void)
{
	eth_InstID = TEthLAN8720::getInstance(pEthLAN8720);
	if(pEthLAN8720)
	{
		pEthLAN8720->registerRxCallback(this, &TVirtual_Eth::EthRx);
	}
}

void TVirtual_Eth::setupEth(ip_addr_t ip_local, ip_addr_t mask, ip_addr_t gw, ip_addr_t ip_server)
{
	if(pEthLAN8720)
	{
		pEthLAN8720->setupEth(ip_local, mask, gw, ip_server);
	}
}

//------------------------------------------------------------------------------
/**
 * @brief Close virtually specific instance of TSerial232 or TSerial485
 */
void TVirtual_Eth::close(void)
{
	pEthLAN8720->closeInstance(eth_InstID);
}

//------------------------------------------------------------------------------

void TVirtual_Eth::closeAllUsers(void)
{
	pEthLAN8720->closeAllUsers();
}

//------------------------------------------------------------------------------

/**
 * @brief Open specific instance of TSerial232 or TSerial485
 */
char TVirtual_Eth::open(void)
{
	return pEthLAN8720->openInstance(eth_InstID);
}

//------------------------------------------------------------------------------
/**
 * @brief Check if virtual serial port is open
 */
bool TVirtual_Eth::isOpen(void)
{
	return pEthLAN8720->isInstance_open(eth_InstID);
}

void TVirtual_Eth::poll(uint32_t localTime)
{
	pEthLAN8720->poll(localTime);
}

//------------------------------------------------------------------------------
/**
 * @brief Read from user rx queue one byte
 * @param rxByte	pointer to single byte received
 */
int TVirtual_Eth::read(char *rxByte)
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
bool TVirtual_Eth::isToRead(void)
{
	return rxQ.size()>0;
}

//------------------------------------------------------------------------------
/**
 * @brief Check if there is anything to read from rx queue
 * @return the number of character to be read
 */
int TVirtual_Eth::bytesAvailable(void)
{
	return rxQ.size();
}

//------------------------------------------------------------------------------
unsigned char TVirtual_Eth::write(const volatile char *msg,
		short int charNum2Send)
{
	pEthLAN8720->write(msg,charNum2Send);
	
	return 0x0;
}
//------------------------------------------------------------------------------
void TVirtual_Eth::write(void)
{
}
//------------------------------------------------------------------------------

int TVirtual_Eth::bytesToWrite(void)
{
	return pEthLAN8720->bytesToWrite();
}

//------------------------------------------------------------------------------
/**
 * @brief Interrupt service function that push in user rx queue
 * single byte readed from physical serial port
 * @param payload 	byte readed
 */
void TVirtual_Eth::EthRx(char *payload)
{
	rxQ.push(payload);
}

//--------------------------------------------------------------------------------

void TVirtual_Eth::resetBytesRX(void)
{
	rxQ.reset();
}
