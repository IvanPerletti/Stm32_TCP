#include "TAutoma_EthTest.h"
#include <stdio.h>
#include <string.h>

TAutomaEth_Test automaEth;

TAutomaEth_Test::TAutomaEth_Test(void):
								eth(),
								eth_1(),
								bInit(true),
								nBytesAvail(0),
								nBytesAvail_1(0),
								bNeedToTransmit(0),
								bNeedToTransmit_1(0)
{
	pEth = &eth;
	pEth_1 = &eth_1;

	//timoutRX.start();
	state	= ST_INIT;
}
//---------------------------------------------------------------------------
void TAutomaEth_Test::executeSM()
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

void TAutomaEth_Test::stat_Init()
{
	pEth->resetBytesRX();
	pEth_1->resetBytesRX();

	if(bInit) //entering only once
	{
		pEth->subscribe();
		pEth_1->subscribe();
		bInit=false;
	}

	bool err;
	bool err_1;

	if (pEth != NULL)
	{
		if (pEth->isOpen())
		{
			pEth->close();
		}

		// is possible to do setupCOM only once because pEth232 and pEth232_1 are pointer to the same physical port (4: comCM25)
		err = pEth->open();
	}

	if (pEth_1 != NULL)
	{
		if (pEth_1->isOpen())
		{
			pEth_1->close();
		}
		err_1 = pEth_1->open();
	}

	if (!err && !err_1)
	{
		setState(ST_READ);
	}
}
//---------------------------------------------------------------------------
void TAutomaEth_Test::stat_Read()
{

	if( pEth->bytesAvailable())
	{
		nBytesAvail = pEth->read(msgRx);

		if(nBytesAvail)
		{
			bNeedToTransmit = 1;
			//timoutRX.start();
		}
	}

	//Always satisfied because pEth232 and pEth232_1 rxQ buffers has the same content
	if(pEth_1->bytesAvailable())
	{
		nBytesAvail_1 = pEth_1->read(msgRx_1);

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
void TAutomaEth_Test::stat_Transm()
{
	//Transmit what we have read in previous state
	if(bNeedToTransmit)
	{
		pEth->write(msgRx,nBytesAvail);
		bNeedToTransmit = 0;
	}
	if(bNeedToTransmit_1)
	{
		pEth_1->write(msgRx_1,nBytesAvail_1);
		bNeedToTransmit_1 = 0;
	}

	setState(ST_READ);
}

//---------------------------------------------------------------------------
void TAutomaEth_Test::stat_Reset()
{
	//only one user can physically close port for all
	//(we have to decide what instance has more priority on the physical port)

	pEth->closeAllUsers();
	pEth->close(); // virtual close: if closeAllUsers is called before close fnc becomes physical close

	pEth->resetBytesRX();
	pEth_1->resetBytesRX();

	//timoutRX.start();
	setState(ST_INIT);
}

