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
		ST_READ,
		ST_TRANSM,
	} structSM; // state machine

	structSM       state;

protected:
	// state machine
	void setState(structSM enNxtState){state = enNxtState;};
	void stat_Reset();
	void stat_Init();
	void stat_Read();
	void stat_Transm();

	TVirtual_Serial serial;
	TVirtual_Serial serial1;

	TVirtual_Serial* pSerial232;
	TVirtual_Serial* pSerial232_1;

	bool bInit;

	int nBytesAvail_1;
	int nBytesAvail;

	char msgRx[128];
	char msgRx_1[128];

	TChronoMeter timoutRX;

	bool bNeedToTransmit;
	bool bNeedToTransmit_1;
};
extern TAutomaSerial_Test automaSerial;

#endif /* GMM_DRIVER_INC_TAUTOMA_SERIALTEST_H_ */
