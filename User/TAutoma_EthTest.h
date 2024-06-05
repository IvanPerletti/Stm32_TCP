#ifndef GMM_DRIVER_INC_TAUTOMA_ETHTEST_H_
#define GMM_DRIVER_INC_TAUTOMA_ETHTEST_H_

#include "TChronoMeter.h"
#include "IQ_Generic.h"
#include "TVirtual_Eth.h"

#include <TEthLAN8720.h>

class TAutomaEth_Test
{
public:

	TAutomaEth_Test(void);
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

	TVirtual_Eth eth;
	TVirtual_Eth eth_1;

	TVirtual_Eth* pEth;
	TVirtual_Eth* pEth_1;

	bool bInit;

	int nBytesAvail_1;
	int nBytesAvail;

	char msgRx[128];
	char msgRx_1[128];

	TChronoMeter timoutRX;

	bool bNeedToTransmit;
	bool bNeedToTransmit_1;
};
extern TAutomaEth_Test automaEth;

#endif /* GMM_DRIVER_INC_TAUTOMA_ETHTEST_H_ */
