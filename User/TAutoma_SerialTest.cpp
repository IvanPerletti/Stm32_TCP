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

#include "TDigitalPort.h"
#include "TAnalogPort_S76.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>


#include "tcpprotocol.h"

/*Static IP ADDRESS: IP_1_ADDR0.IP_1_ADDR1.IP_1_ADDR2.IP_1_ADDR3 */
#define IP_1_ADDR0   192
#define IP_1_ADDR1   168
#define IP_1_ADDR2   1
#define IP_1_ADDR3   100
   
/*Static IP ADDRESS: IP_2_ADDR0.IP_2_ADDR1.IP_2_ADDR2.IP_2_ADDR3 */
#define IP_2_ADDR0   192
#define IP_2_ADDR1   168
#define IP_2_ADDR2   2
#define IP_2_ADDR3   100
   
/*NETMASK*/
#define NETMASK_ADDR0   255
#define NETMASK_ADDR1   255
#define NETMASK_ADDR2   255
#define NETMASK_ADDR3   0

/*Gateway Address*/
#define GW_ADDR0   0
#define GW_ADDR1   0
#define GW_ADDR2   0
#define GW_ADDR3   0

#define SERVER_IP_1_ADDR0   192
#define SERVER_IP_1_ADDR1   168
#define SERVER_IP_1_ADDR2   1
#define SERVER_IP_1_ADDR3   101

#define SERVER_IP_2_ADDR0   192
#define SERVER_IP_2_ADDR1   168
#define SERVER_IP_2_ADDR2   2
#define SERVER_IP_2_ADDR3   101

#define SERVER_PORT_A       2020
#define SERVER_PORT_B       8443

#define MSG_TEST						"TEST Message"
//#define MSG_TEST						"00000000001111111111222222222233333333334444444444"
//#define N_MSG_TEST					100

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

	ICan::getInstance(pCan);
	
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


void TAutomaSerial_Test::parseCommand(char *message)
{
    if (tcpProtocolSlave.fromCommand(message))
    {
        if (tcpProtocolSlave.getTarget() == TcpProtocol::eTargetDin)
        {
            char *message = tcpProtocolSlave.toAnswer(digitalPort.check((enumDigitalIn)tcpProtocolSlave.getIdx()) == HIGH ? TcpProtocol::eStateOn : TcpProtocol::eStateOff);
#ifndef STM32F4XX
            ui->plainTextEditLog->appendPlainText(QTime::currentTime().toString("hh:mm:ss.zzz") + "    Message sent " + QString(message));
            tcpSlave->write(message);
#else
            pEth->write(message, (int)strlen(message));
#endif
        }
        else if (tcpProtocolSlave.getTarget() == TcpProtocol::eTargetDout)
        {
#ifndef STM32F4XX
            outputButtons[tcpProtocolSlave.getIdx()-1]->setIcon(tcpProtocolSlave.getState() == TcpProtocol::eStateOn ? QIcon(":/icons/on.png") : QIcon(":/icons/off.png"));
#endif
            if (tcpProtocolSlave.getState() == TcpProtocol::eStateOn)
                digitalPort.setNow((enumDigitalOut)tcpProtocolSlave.getIdx());
            else
                digitalPort.resetNow((enumDigitalOut)tcpProtocolSlave.getIdx());
            char *message = tcpProtocolSlave.toAnswer(digitalPort.check((enumDigitalOut)tcpProtocolSlave.getIdx()) == HIGH ? TcpProtocol::eStateOn : TcpProtocol::eStateOff);
#ifndef STM32F4XX
            ui->plainTextEditLog->appendPlainText(QTime::currentTime().toString("hh:mm:ss.zzz") + "    Message sent " + QString(message));
            tcpSlave->write(message);
#else
            pEth->write(message, (int)strlen(message));
#endif
        }
        else if (tcpProtocolSlave.getTarget() == TcpProtocol::eTargetAnalog)
        {
            char *message = tcpProtocolSlave.toAnswer(tAnalogPort.read((enumAnalogPort)(tcpProtocolSlave.getIdx()-1)));
#ifndef STM32F4XX
            ui->plainTextEditLog->appendPlainText(QTime::currentTime().toString("hh:mm:ss.zzz") + "    Message sent " + QString(message));
            tcpSlave->write(message);
#else
            pEth->write(message, (int)strlen(message));
#endif
        }
        else if (tcpProtocolSlave.getTarget() == TcpProtocol::eTargetCAN)
        {
            char *message = NULL;

            if (tcpProtocolSlave.getCommand() == TcpProtocol::eCmdOpen)
            {
                pCan->open(0);
                message = tcpProtocolSlave.toAnswer(TcpProtocol::eStateOpened);
            }
            else if (tcpProtocolSlave.getCommand() == TcpProtocol::eCmdClose)
            {
                pCan->close(0);
                message = tcpProtocolSlave.toAnswer(TcpProtocol::eStateClosed);
            }
            else if (tcpProtocolSlave.getCommand() == TcpProtocol::eCmdSet)
            {
                if (tcpProtocolSlave.getParam() == TcpProtocol::eParamSpeed)
                {
                    int speed = strtol(tcpProtocolSlave.getParams(0), NULL, 10);
                    switch (speed)
                    {
                    case 125:
                        pCan->setSpeed(ICan::CAN_BUS_SPEED_125);
                        break;
                    case 250:
                        pCan->setSpeed(ICan::CAN_BUS_SPEED_250);
                        break;
                    case 500:
                        pCan->setSpeed(ICan::CAN_BUS_SPEED_500);
                        break;
                    default:
                        ; // send error answer
                    }
                    message = tcpProtocolSlave.toAnswer(tcpProtocolSlave.getParam(), (int)pCan->getSpeed());
                }
                else if (tcpProtocolSlave.getParam() == TcpProtocol::eParamFilter)
                {
                    bool ret = false;
                    uint16_t filters[4];

                    for (int ind=0; ind<tcpProtocolSlave.getNParams(); ind++)
                        filters[ind] = (uint16_t)strtol(tcpProtocolSlave.getParams(ind), NULL, 10);
                    switch (tcpProtocolSlave.getNParams())
                    {
                    case 1:
                        ret = pCan->registerFilter(0, filters[0]);
                        break;
                    case 2:
                        ret = pCan->registerFilter(0, filters[0], filters[1]);
                        break;
                    case 3:
                        ret = pCan->registerFilter(0, filters[0], filters[1], filters[2]);
                        break;
                    case 4:
                        ret = pCan->registerFilter(0, filters[0], filters[1], filters[2], filters[3]);
                        break;
                    }
                    if (ret)
                        message = tcpProtocolSlave.toAnswer(tcpProtocolSlave.getParam(), tcpProtocolSlave.getNParams());
                    else
                        ; // send error answer
                }
            }
            else if (tcpProtocolSlave.getCommand() == TcpProtocol::eCmdSend)
            {
                if (tcpProtocolSlave.getParam() == TcpProtocol::eParamData)
                {
                    bool ret = false;
                    CanTxMsg canTxMsg;

                    if (tcpProtocolSlave.getNParams() > 1)
                    {
                        canTxMsg.StdId = (uint32_t)strtol(tcpProtocolSlave.getParams(0), NULL, 16);
                        canTxMsg.IDE = 0;
                        canTxMsg.DLC = tcpProtocolSlave.getNParams() - 1;
                        for (int ind=0; ind<tcpProtocolSlave.getNParams() - 1; ind++)
                            canTxMsg.Data[ind] = (uint16_t)strtol(tcpProtocolSlave.getParams(ind+1), NULL, 16);
                        pCan->write(&canTxMsg);
                        ret = true;
                    }
                    if (ret)
                        message = tcpProtocolSlave.toAnswer(tcpProtocolSlave.getParam(), tcpProtocolSlave.getNParams());
                    else
                        ; // send error answer
                }

            }
            if (message)
            {
#ifndef STM32F4XX
                ui->plainTextEditLog->appendPlainText(QTime::currentTime().toString("hh:mm:ss.zzz") + "    Message sent " + QString(message));
                tcpSlave->write(message);
#else
                pEth->write(message, (int)strlen(message));
#endif
            }
        }
    }
}


//void TAutomaSerial_Test::parseCommand(char *message)
//{
//    if (tcpProtocolSlave.fromCommand(message))
//    {
//        if (tcpProtocolSlave.getTarget() == TcpProtocol::eTargetDin)
//        {
//            char *message = tcpProtocolSlave.toAnswer(digitalPort.check((enumDigitalIn)tcpProtocolSlave.getIdx()) == HIGH ? TcpProtocol::eStateOn : TcpProtocol::eStateOff);
//#ifndef STM32F4XX
//            ui->plainTextEditLog->appendPlainText(QTime::currentTime().toString("hh:mm:ss.zzz") + "    Message sent " + QString(message));
//            tcpSlave->write(message);
//#else
//						pEth->write(message, (int)strlen(message));
//#endif
//        }
//        else if (tcpProtocolSlave.getTarget() == TcpProtocol::eTargetDout)
//        {
//#ifndef STM32F4XX
//            outputButtons[tcpProtocolSlave.getIdx()-1]->setIcon(tcpProtocolSlave.getState() == TcpProtocol::eStateOn ? QIcon(":/icons/on.png") : QIcon(":/icons/off.png"));
//#endif					
//            if (tcpProtocolSlave.getState() == TcpProtocol::eStateOn)
//                digitalPort.setNow((enumDigitalOut)tcpProtocolSlave.getIdx());
//            else
//                digitalPort.resetNow((enumDigitalOut)tcpProtocolSlave.getIdx());
//            char *message = tcpProtocolSlave.toAnswer(digitalPort.check((enumDigitalOut)tcpProtocolSlave.getIdx()) == HIGH ? TcpProtocol::eStateOn : TcpProtocol::eStateOff);
//#ifndef STM32F4XX
//            ui->plainTextEditLog->appendPlainText(QTime::currentTime().toString("hh:mm:ss.zzz") + "    Message sent " + QString(message));
//            tcpSlave->write(message);
//#else
//						pEth->write(message, (int)strlen(message));
//#endif
//        }
//        else if (tcpProtocolSlave.getTarget() == TcpProtocol::eTargetAnalog)
//        {
//            char *message = tcpProtocolSlave.toAnswer(tAnalogPort.read((enumAnalogPort)(tcpProtocolSlave.getIdx()-1)));
//#ifndef STM32F4XX
//            ui->plainTextEditLog->appendPlainText(QTime::currentTime().toString("hh:mm:ss.zzz") + "    Message sent " + QString(message));
//            tcpSlave->write(message);
//#else
//						pEth->write(message, (int)strlen(message));
//#endif
//        }
//    }
//}

void TAutomaSerial_Test::monitor(void)
{
	if (pEth)
	{
		pEth->poll(timoutRX.now());
		if( (nBytesAvail = pEth->bytesAvailable()) > 0)
		{
			int idx;
			char str[50];
			
			nBytesAvail = pEth->read(msgRx);
			sprintf(str, "\nT=%08d Rx %d bytes:", (int)timoutRX.now(), nBytesAvail);
			PutStr(str);
			for (idx=0; idx<nBytesAvail; idx++) {
				sprintf(str, " 0x%02X", msgRx[idx]);
				PutStr(str);
			}


			parseCommand(msgRx);

#if 0			
			//ui->plainTextEditLog->appendPlainText(QTime::currentTime().toString("hh:mm:ss.zzz") + "    Message received " + QString(data).chopped(1));
			if (tcpSlave.fromCommand(msgRx))
			{
					if (tcpSlave.getTarget() == TcpProtocol::eTargetDin)
					{
							char *message = tcpSlave.toAnswer(digitalPort.check((enumDigitalIn)tcpSlave.getIdx()) == HIGH ? TcpProtocol::eStateOn : TcpProtocol::eStateOff);
							//ui->plainTextEditLog->appendPlainText(QTime::currentTime().toString("hh:mm:ss.zzz") + "    Message sent " + QString(message));
							pEth->write(message, (int)strlen(message));
					}
					else if (tcpSlave.getTarget() == TcpProtocol::eTargetDout)
					{
							if (tcpSlave.getState() == TcpProtocol::eStateOn)
									digitalPort.setNow((enumDigitalOut)tcpSlave.getIdx());
							else
									digitalPort.resetNow((enumDigitalOut)tcpSlave.getIdx());
							char *message = tcpSlave.toAnswer(digitalPort.check((enumDigitalOut)tcpSlave.getIdx()) == HIGH ? TcpProtocol::eStateOn : TcpProtocol::eStateOff);
							//ui->plainTextEditLog->appendPlainText(QTime::currentTime().toString("hh:mm:ss.zzz") + "    Message sent " + QString(message));
							pEth->write(message, strlen(message));
					}
					else if (tcpSlave.getTarget() == TcpProtocol::eTargetAnalog)
					{
							char *message = tcpSlave.toAnswer(tAnalogPort.read((enumAnalogPort)(tcpSlave.getIdx())));
							//ui->plainTextEditLog->appendPlainText(QTime::currentTime().toString("hh:mm:ss.zzz") + "    Message sent " + QString(message));
							pEth->write(message, strlen(message));
					}
			}
#endif
			
//#ifdef	USE_GPIO_FOR_DEBUG
//			digitalPort.resetNow(DO_PC9);
//#endif
//			
//			if (++cntPacket < N_MSG_TEST) 
//			{
//#ifdef	USE_GPIO_FOR_DEBUG
//				digitalPort.setNow(DO_PC8);
//#endif
//				pEth->write(msgRx, nBytesAvail);
//			}
		}
	}
}
//---------------------------------------------------------------------------
void TAutomaSerial_Test::showMainMenu(void)
{
	char str[50];
	
	ClearScreen();
	PutStrXY(5, 5, "ETH Test");
	sprintf(str, "1) Connect to %d.%d.%d.%d:%d", SERVER_IP_1_ADDR0, SERVER_IP_1_ADDR1, SERVER_IP_1_ADDR2, SERVER_IP_1_ADDR3, SERVER_PORT_A);
	PutStrXY(0, 7, str);
	sprintf(str, "2) Connect to %d.%d.%d.%d:%d", SERVER_IP_2_ADDR0, SERVER_IP_2_ADDR1, SERVER_IP_2_ADDR2, SERVER_IP_2_ADDR3, SERVER_PORT_A);
	PutStrXY(0, 8, str);
	sprintf(str, "3) Connect to %d.%d.%d.%d:%d", SERVER_IP_1_ADDR0, SERVER_IP_1_ADDR1, SERVER_IP_1_ADDR2, SERVER_IP_1_ADDR3, SERVER_PORT_B);
	PutStrXY(0, 9, str);
	PutStrXY(0, 10,"4) Connect to void");
	PutStrXY(0, 11,"5) Send message <TEST Message>");
//	PutStrXY(0, 11,"5) Start test");
#ifdef	USE_GPIO_FOR_DEBUG
	PutStrXY(0, 12,"6) Reset OUT C8");
#endif
	PutStrXY(0, 14,"D) Disconnect");
		
	PutStrXY(0, 16, (pEth && pEth->isOpen()) ? "Connected" : "Disconnected");
}

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


	digitalPort.init();
	
	if (!err && !err_1)
	{
		showMainMenu();
		setState(ST_MENU);
	}
}
//---------------------------------------------------------------------------
void TAutomaSerial_Test::stat_Menu()
{
	if (pEth)
	{
		if (bEthOpen != pEth->isOpen())
		{
			bEthOpen = pEth->isOpen();
			PutStrXY(0, 16, bEthOpen ? "Connected" : "Disconnected");
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
					case '2':
					case '3':
						if (pEth != NULL)
						{
							bool err = false;
							ip_addr_t ip_local;
							ip_addr_t netmask;
							ip_addr_t gw;
							ip_addr_t ip_server;

							PutStrXY(0, 16, "Connecting...");
							
							if (pEth->isOpen())
							{
								pEth->close();
							}

							if (msgRx[0] == '1' || msgRx[0] == '3')  
							{
								IP4_ADDR(&ip_local, IP_1_ADDR0, IP_1_ADDR1, IP_1_ADDR2, IP_1_ADDR3);
								IP4_ADDR(&ip_server, SERVER_IP_1_ADDR0, SERVER_IP_1_ADDR1, SERVER_IP_1_ADDR2, SERVER_IP_1_ADDR3);
							} 
							else 
							{
								IP4_ADDR(&ip_local, IP_2_ADDR0, IP_2_ADDR1, IP_2_ADDR2, IP_2_ADDR3);
								IP4_ADDR(&ip_server, SERVER_IP_2_ADDR0, SERVER_IP_2_ADDR1, SERVER_IP_2_ADDR2, SERVER_IP_2_ADDR3);
							} 
							IP4_ADDR(&netmask, NETMASK_ADDR0, NETMASK_ADDR1 , NETMASK_ADDR2, NETMASK_ADDR3);
							IP4_ADDR(&gw, GW_ADDR0, GW_ADDR1, GW_ADDR2, GW_ADDR3);
							pEth->setupEth(ip_local, netmask, gw, ip_server, (msgRx[0] == '3') ? SERVER_PORT_B : SERVER_PORT_A);
							
							err = pEth->open();

							if (err)
								PutStrXY(0, 16, "Error opening connection");
						}
						break;
					case '4':
						if (pEth != NULL)
						{
							bool err = false;
							
							PutStrXY(0, 16, "Connecting...");
							
							if (pEth->isOpen())
							{
								pEth->close();
							}

							err = pEth->open();

							if (err)
								PutStrXY(0, 16, "Error opening connection");
						}
						break;
					case '5':
						cntPacket = 0;
#ifdef	USE_GPIO_FOR_DEBUG
						digitalPort.setNow(DO_PC8);
#endif
						pEth->write(MSG_TEST, strlen(MSG_TEST));
						break;
					case '6':
						PutStr("PC8 OFF\n");
						digitalPort.resetNow(DO_PC8);
						break;
					case '7':
						PutStr("PC8 ON\n");
						digitalPort.setNow(DO_PC8);
						break;
					case 'D':
						if (pEth)
						{
							PutStrXY(0, 16, "Disconnecting...");

							pEth->close();
						}
						break;
					default:
						showMainMenu();
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
