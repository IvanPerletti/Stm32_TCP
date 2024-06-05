/*
 * TAutoma_SerialTest.cpp
 *
 *  Created on: 15 feb 2024
 *      Author: mdotti
 *
 *      Simple state machine:
 *      1) Create two TVirtual_Serial object (constructor argument is number of physical port in this case comCM25 (RS-232))
 *      2) Subscribe objects once and get unique instance pointer (getInstance fnc returns pointer to singleton comCM25)
 *      3) Read from instances RX buffers (allows more users to read the same data received from the physical serial port)
 *      4) Write to comCM25 physical bus by using two instances of the same type
 */
#include "TAutoma_SerialTest.h"
#include <stdio.h>
#include <string.h>

TAutomaSerial_Test automaSerial;

TAutomaSerial_Test::TAutomaSerial_Test(void):
								serial1(4), //serial_id 4: comCM25
								serial(4),
								bInit(true),
								nBytesAvail(0),
								nBytesAvail_1(0),
								bNeedToTransmit(0),
								bNeedToTransmit_1(0)
{
	pSerial232 = &serial;
	pSerial232_1 = &serial1;

	//timoutRX.start();
	state	= ST_INIT;
}
//---------------------------------------------------------------------------
void TAutomaSerial_Test::executeSM()
{
	switch(state)
	{
	case ST_RESET:
		stat_Reset();
		break;
	case ST_INIT:
		stat_Init();
		break;
	case ST_READ:
		stat_Read();
		break;
	case ST_TRANSM:
		stat_Transm();
		break;
	}
}
//---------------------------------------------------------------------------

void TAutomaSerial_Test::stat_Init()
{
	pSerial232->resetBytesRX();
	pSerial232_1->resetBytesRX();

	if(bInit) //entering only once
	{
		pSerial232->subscribe();
		pSerial232_1->subscribe();
		bInit=false;
	}

	bool err;
	bool err_1;

	if (pSerial232 != NULL)
	{
		if (pSerial232->isOpen())
		{
			pSerial232->close();
		}

		// is possible to do setupCOM only once because pSerial232 and pSerial232_1 are pointer to the same physical port (4: comCM25)
		pSerial232->setupCOM(Baud9600, OneStop, NoParity, NoFlowControl, Data8);
		err = pSerial232->open();
	}

	if (pSerial232_1 != NULL)
	{
		//pSerial232_1->setupCOM(Baud9600, OneStop, NoParity, NoFlowControl, Data8);

		if (pSerial232_1->isOpen())
		{
			pSerial232_1->close();
		}
		err_1 = pSerial232_1->open();
	}

	if (!err && !err_1)
	{
		setState(ST_READ);
	}
}
//---------------------------------------------------------------------------
void TAutomaSerial_Test::stat_Read()
{

	if( pSerial232->bytesAvailable())
	{
		nBytesAvail = pSerial232->read(msgRx);

		if(nBytesAvail)
		{
			bNeedToTransmit = 1;
			//timoutRX.start();
		}
	}

	//Always satisfied because pSerial232 and pSerial232_1 rxQ buffers has the same content
	if(pSerial232_1->bytesAvailable())
	{
		nBytesAvail_1 = pSerial232_1->read(msgRx_1);

		if(nBytesAvail_1)
		{
			bNeedToTransmit_1 = 1;
		}
	}

	setState(ST_TRANSM);

	//	if(timoutRX.exceed(5000)) //
	//	{
	//		setState(ST_INIT);
	//		timoutRX.start();
	//	}
}

//---------------------------------------------------------------------------
void TAutomaSerial_Test::stat_Transm()
{
	//Transmit what we have read in previous state
	if(bNeedToTransmit)
	{
		pSerial232->write(msgRx,nBytesAvail);
		bNeedToTransmit = 0;
	}
	if(bNeedToTransmit_1)
	{
		pSerial232_1->write(msgRx_1,nBytesAvail_1);
		bNeedToTransmit_1 = 0;
	}

	setState(ST_READ);
}

//---------------------------------------------------------------------------
void TAutomaSerial_Test::stat_Reset()
{
	//only one user can physically close port for all
	//(we have to decide what instance has more priority on the physical port)

	pSerial232->closeAllUsers();
	pSerial232->close(); // virtual close: if closeAllUsers is called before close fnc becomes physical close

//	pSerial232_1->closeAllUsers();
//	pSerial232_1->close();

	pSerial232->resetBytesRX();
	pSerial232_1->resetBytesRX();

	//timoutRX.start();
	setState(ST_INIT);
}

