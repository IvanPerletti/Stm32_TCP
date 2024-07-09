/*
 * TAutoma_SerialTest.h
 *
 *  Created on: 15 feb 2024
 *      Author: mdotti
 */

#ifndef GMM_DRIVER_INC_TAUTOMA_SERIALTEST_H_
#define GMM_DRIVER_INC_TAUTOMA_SERIALTEST_H_

#include "TChronoMeter.h"
#include "IQ_Generic.h"
#include "TVirtual_Serial.h"
#include "TVirtual_Eth.h"

#include <TEthLAN8720.h>
#include <TSerial232.h>		// RS232 ...
#include <TSerial485.h>		// RS485 ...

class TAutomaSerial_Test
{
public:

	TAutomaSerial_Test(void);
	void executeSM(void);

private:
	typedef enum
	{
		ST_RESET,
		ST_INIT,
		ST_MENU,
	} structSM; // state machine

	structSM       state;

protected:
	// state machine
	void setState(structSM enNxtState){state = enNxtState;};
	void stat_Reset();
	void stat_Init();
	void stat_Menu();

	TVirtual_Serial serial;
	TVirtual_Serial serial1;

	TVirtual_Serial* pSerial232;
	TVirtual_Serial* pSerial232_1;

	
	TVirtual_Eth eth;
	TVirtual_Eth* pEth;
	bool bEthOpen;
	
	bool bInit;

	int nBytesAvail;
	int nBytesAvail_1;

	char msgRx[128];
	char msgRx_1[128];

	TChronoMeter timoutRX;

	uint8_t PutStr(char *str);
	uint8_t PutStrY(int y, char *str);
	uint8_t PutStrXY(int x, int y, char *str);
	uint8_t PutLine(char *str);
	void ClearScreen(void);
	void ClearDown(int y);
	void ClearLine(int y);
	void ClearToEOL(int x, int y);
	void GotoXY(int x, int y);
	
};
extern TAutomaSerial_Test automaSerial;

#endif /* GMM_DRIVER_INC_TAUTOMA_SERIALTEST_H_ */
