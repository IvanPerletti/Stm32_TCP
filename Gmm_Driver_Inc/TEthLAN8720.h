/*
 * TEthLAN8720.h
 *
 ******************************************************************************/

#ifndef TETHLAN8720_H_
#define TETHLAN8720_H_

#include <stdio.h>
#include "IQ_Generic.h"

#include "TEth.h"

#include "stm32f4xx.h"
#include "misc.h"
#include <stm32f4x7_eth.h> 
//#include "stm32f4_discovery.h"

#define MAX_ETHLAN8720_NUM 5
#define MAX_ETHLAN8720_CALLBACKS MAX_ETHLAN8720_NUM

#define WRITE_MULTIPLE_CHAR

class TEthLAN8720 : public TEth {
	
public:

	TEthLAN8720(void);
	virtual ~TEthLAN8720();

	static long getInstance(TEthLAN8720* &pEthLAN8720);
	static long IdUserEthLAN8720;
	static long s32CurrentQ;

	IQ_Generic<long, MAX_ETHLAN8720_NUM> qOpened;
	void closeInstance(long qID);
	void closeAllUsers(void);
	char openInstance(long qID);
	char isInstance_open(long qID);
};

// Singleton EthLAN8720
extern TEthLAN8720 ethLAN8720;

//-----------------------------------------------------------------------------------------
#endif /* TETHLAN8720_H_ */
