/*
 * TVirtual_Eth.h
 *
 *  Created on: 16 feb 2024
 *      Author: diaferia
 */

#include "TEthLAN8720.h"
#include "IQ_Generic.h"

#define ETH_RX_BYTE_NUM	64
#define ETH_TX_BYTE_NUM ETH_RX_BYTE_NUM

#ifndef GMM_DRIVER_INC_TCOM_ETH_H_
#define GMM_DRIVER_INC_TCOM_ETH_H_

class TVirtual_Eth {

public:

	TVirtual_Eth(void);
	virtual ~TVirtual_Eth(void);

	bool subscribe(void);

	void setupEth(ip_addr_t local_ip, ip_addr_t mask, ip_addr_t gw, ip_addr_t server_ip, int server_port);

	void close(void);
	void closeAllUsers(void);
	char open(void);
	bool isOpen(void);
	void poll(uint32_t localTime);

	int read(char *rxByte);
	bool isToRead(void);
	int bytesAvailable(void);

	unsigned char write(const char *msg, unsigned short int charNum2Send);
	int bytesToWrite(void);
	void resetBytesRX(void);
	void rxCallback(char *payload);

private:

	long eth_InstID;

	TEthLAN8720* pEthLAN8720;

	IQ_Generic<char,ETH_RX_BYTE_NUM> rxQ;
};

#endif /* GMM_DRIVER_INC_TCOM_ETH_H_ */
