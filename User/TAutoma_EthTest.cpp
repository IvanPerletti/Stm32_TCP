#include "TAutoma_EthTest.h"
#include <stdio.h>
#include <string.h>

 
/*Static IP ADDRESS: IP_ADDR0.IP_ADDR1.IP_ADDR2.IP_ADDR3 */
#define IP_ADDR0   192
#define IP_ADDR1   168
#define IP_ADDR2   1
#define IP_ADDR3   10
   
/*NETMASK*/
#define NETMASK_ADDR0   255
#define NETMASK_ADDR1   255
#define NETMASK_ADDR2   255
#define NETMASK_ADDR3   0

/*Gateway Address*/
#define GW_ADDR0   192
#define GW_ADDR1   168
#define GW_ADDR2   1
#define GW_ADDR3   254

#define SERVER_IP_ADDR0   192
#define SERVER_IP_ADDR1   168
#define SERVER_IP_ADDR2   1
#define SERVER_IP_ADDR3   105

#define DEST_PORT       7

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

	bool err = false;
	bool err_1 = false;

	if (pEth != NULL)
	{
		ip_addr_t ip_local;
		ip_addr_t netmask;
		ip_addr_t gw;
		ip_addr_t ip_server;

		if (pEth->isOpen())
		{
			pEth->close();
		}

		IP4_ADDR(&ip_local, IP_ADDR0, IP_ADDR1, IP_ADDR2, IP_ADDR3);
		IP4_ADDR(&netmask, NETMASK_ADDR0, NETMASK_ADDR1 , NETMASK_ADDR2, NETMASK_ADDR3);
		IP4_ADDR(&gw, GW_ADDR0, GW_ADDR1, GW_ADDR2, GW_ADDR3);
		IP4_ADDR(&ip_server, SERVER_IP_ADDR0, SERVER_IP_ADDR1, SERVER_IP_ADDR2, SERVER_IP_ADDR3);
		pEth->setupEth(ip_local, netmask, gw, ip_server);
		
		err = pEth->open();
	}

	if (pEth_1 != NULL)
	{
		err_1 = pEth_1->open();
	}

	if (!err && !err_1)
	{
		setState(ST_READ);
			//bNeedToTransmit = 1;
		  //setState(ST_TRANSM);
	}
}
//---------------------------------------------------------------------------
void TAutomaEth_Test::stat_Read()
{
	pEth->poll(tClock.watch());

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
	pEth->poll(tClock.watch());

	//Transmit what we have read in previous state
	if(bNeedToTransmit)
	{
		if (pEth->isOpen()) {
			pEth->write(msgRx, nBytesAvail);
			//pEth->write("test", 4);
			bNeedToTransmit = 0;
		}
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

