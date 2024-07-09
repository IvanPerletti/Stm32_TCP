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
#include <stdarg.h>

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

#define SERVER_PORT       2020


TAutomaSerial_Test automaSerial;

TAutomaSerial_Test::TAutomaSerial_Test(void):
								serial(4),
								serial1(4), //serial_id 4: comCM25
								eth(),
								bEthOpen(false),
								bInit(true),
								nBytesAvail(0),
								nBytesAvail_1(0)
{
	pSerial232 = &serial;
	//pSerial232_1 = &serial1;

	pEth = &eth;
	
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
	case ST_MENU:
		stat_Menu();
		break;
	}
}
//---------------------------------------------------------------------------

void TAutomaSerial_Test::stat_Init()
{
	if (pSerial232 != NULL)
		pSerial232->resetBytesRX();
	if (pSerial232_1 != NULL)
		pSerial232_1->resetBytesRX();

	if (pEth)
		pEth->resetBytesRX();


	if(bInit) //entering only once
	{
		if (pSerial232 != NULL)
			pSerial232->subscribe();
		if (pSerial232_1 != NULL)
			pSerial232_1->subscribe();
		if (pEth)
			pEth->subscribe();

		bInit=false;
	}

	bool err = false;
	bool err_1 = false;

	if (pSerial232 != NULL)
	{
		if (pSerial232->isOpen())
		{
			pSerial232->close();
		}

		// is possible to do setupCOM only once because pSerial232 and pSerial232_1 are pointer to the same physical port (4: comCM25)
		pSerial232->setupCOM(Baud115200, OneStop, NoParity, NoFlowControl, Data8);
		err = pSerial232->open();
	}


	if (!err && !err_1)
	{
		ClearScreen();
		PutStrXY(5, 5, "ETH Test");
		PutStrXY(0, 7, "1) Connect");
		PutStrXY(0, 8, "2) Disconnect");
		PutStrXY(0, 12, "Disconnected");
		setState(ST_MENU);
	}
}
//---------------------------------------------------------------------------
void TAutomaSerial_Test::stat_Menu()
{
	if (pEth)
	{
		pEth->poll(tClock.watch());
		
		if (bEthOpen != pEth->isOpen())
		{
			bEthOpen = pEth->isOpen();
			PutStrXY(0, 12, bEthOpen ? "Connected" : "Disconnected");
		}
		
		if( pEth->bytesAvailable())
		{
			nBytesAvail = pEth->read(msgRx);

			if(nBytesAvail)
			{
				pEth->write(msgRx, nBytesAvail);
			}
		}
	}

	if (pSerial232 != NULL)
	{
		if( pSerial232->bytesAvailable())
		{
			nBytesAvail = pSerial232->read(msgRx);

			if(nBytesAvail)
			{
				switch (msgRx[0])
				{
					case '1':
						if (pEth != NULL)
						{
							bool err = false;
							ip_addr_t ip_local;
							ip_addr_t netmask;
							ip_addr_t gw;
							ip_addr_t ip_server;

							PutStrXY(0, 12, "Connecting...");
							
							if (pEth->isOpen())
							{
								pEth->close();
							}

							IP4_ADDR(&ip_local, IP_ADDR0, IP_ADDR1, IP_ADDR2, IP_ADDR3);
							IP4_ADDR(&netmask, NETMASK_ADDR0, NETMASK_ADDR1 , NETMASK_ADDR2, NETMASK_ADDR3);
							IP4_ADDR(&gw, GW_ADDR0, GW_ADDR1, GW_ADDR2, GW_ADDR3);
							IP4_ADDR(&ip_server, SERVER_IP_ADDR0, SERVER_IP_ADDR1, SERVER_IP_ADDR2, SERVER_IP_ADDR3);
							pEth->setupEth(ip_local, netmask, gw, ip_server, SERVER_PORT);
							
							err = pEth->open();

							if (err)
								PutStrXY(0, 12, "Error opening connection");
						}
						break;
					case '2':
						if (pEth)
						{
							PutStrXY(0, 12, "Disconnecting...");

							pEth->close();
						}
						break;
					default:
						break;
				}
			}
		}
	}
}

//---------------------------------------------------------------------------
void TAutomaSerial_Test::stat_Reset()
{
	//only one user can physically close port for all
	//(we have to decide what instance has more priority on the physical port)

	if (pSerial232 != NULL)
	{
		pSerial232->closeAllUsers();
		pSerial232->close(); // virtual close: if closeAllUsers is called before close fnc becomes physical close
	}
//	pSerial232_1->closeAllUsers();
//	pSerial232_1->close();

	if (pSerial232_1 != NULL)
	{
		pSerial232->resetBytesRX();
		pSerial232_1->resetBytesRX();
	}
	
	//timoutRX.start();
	setState(ST_INIT);
}

uint8_t TAutomaSerial_Test::PutStr(char *str)
{
  return (pSerial232->write((const volatile char *)str, strlen(str)));
}

uint8_t TAutomaSerial_Test::PutStrY(int y, char *str)
{
  ClearLine(y);
  return (pSerial232->write((const volatile char *)str, strlen(str)));
}

uint8_t TAutomaSerial_Test::PutStrXY(int x, int y, char *str)
{
  GotoXY(x, y);
  PutStr("\033[K");
  GotoXY(x, y);
  return (pSerial232->write((const volatile char *)str, strlen(str)));
}

uint8_t TAutomaSerial_Test::PutLine(char *str)
{
  if (strlen(str))
	  pSerial232->write((const volatile char *)str, strlen(str));
  return (pSerial232->write((const volatile char *)"\r\n", 2));
}

void TAutomaSerial_Test::ClearScreen(void)
{
	PutStr("\033[2J");
}

void TAutomaSerial_Test::ClearDown(int y)
{
	GotoXY(1, y);
	PutStr("\033[J");
}

void TAutomaSerial_Test::ClearLine(int y)
{
	GotoXY(1, y);
	PutStr("\033[2K");
	GotoXY(1, y);
}

void TAutomaSerial_Test::ClearToEOL(int x, int y)
{
	GotoXY(x, y);
	PutStr("\033[0K");
	GotoXY(x, y);
}

void TAutomaSerial_Test::GotoXY(int x, int y)
{
    char str[20];

    sprintf(str, "\033[%d;%df", y+1, x+1);
    PutStr(str);
}
