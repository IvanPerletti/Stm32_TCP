/*
 * TVirtual_Eth.h
 *
 *  Created on: 16 feb 2024
 *      Author: diaferia
 */

#include "TEthLAN8720.h"
#include "IQ_Generic.h"

#define RX_BYTE_NUM 64
#define TX_BYTE_NUM RX_BYTE_NUM

#ifndef GMM_DRIVER_INC_TCOM_ETH_H_
#define GMM_DRIVER_INC_TCOM_ETH_H_

class TVirtual_Eth {

public:

	TVirtual_Eth(void);
	virtual ~TVirtual_Eth(void);

	void subscribe(void);

	void setupEth(ip_addr_t ip_local, ip_addr_t mask, ip_addr_t gw, ip_addr_t ip_server);

	void close(void);
	void closeAllUsers(void);
	char open(void);
	bool isOpen(void);
	void poll(uint32_t localTime);

	int read(char *rxByte);
	bool isToRead(void);
	int bytesAvailable(void);

	unsigned char write(const volatile char *msg,
			short int charNum2Send);
	void write(void);
	int bytesToWrite(void);
	void resetBytesRX(void);
	void EthRx(char *payload);
	void EthTx(char *payload);

private:

	long eth_InstID;

	TEthLAN8720* pEthLAN8720;

	IQ_Generic<char,RX_BYTE_NUM> rxQ;
	IQ_Generic<char,TX_BYTE_NUM> txQ;
};

#endif /* GMM_DRIVER_INC_TCOM_ETH_H_ */
