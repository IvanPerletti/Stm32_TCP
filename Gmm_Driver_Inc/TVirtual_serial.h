/*
 * TVirtual_Serial.h
 *
 *  Created on: 16 feb 2024
 *      Author: diaferia
 */

#include "TSerial232.h"
#include "TSerial485.h"
#include "IQ_Generic.h"

#define RX_BYTE_NUM 512
#define TX_BYTE_NUM RX_BYTE_NUM

#ifndef GMM_DRIVER_INC_TCOM_SERIAL_H_
#define GMM_DRIVER_INC_TCOM_SERIAL_H_

class TVirtual_Serial {

public:

	TVirtual_Serial(int serial_id);
	virtual ~TVirtual_Serial(void);

	void subscribe(void);
	bool is232Com(void);

	void setupCOM(baudRate baudPipp,
			stopBits stopNum ,
			parity parityBitNum,
			flowControl flowCtrlChoice,
			dataBits dataBitNum);
	void close(void);
	void closeAllUsers(void);
	char open(void);
	bool isOpen(void);

	int read(char *rxByte);
	bool isToRead(void);
	int bytesAvailable(void);

	unsigned char write(const volatile char *msg,
			short int charNum2Send);
	void write(void);
	int bytesToWrite(void);
	void resetBytesRX(void);
	void SerialRxIRQ(char *payload);
	void SerialTxIRQ(char *payload);

private:

	int serial_TypeID;
	long serial_InstID;

	TSerial232* pSerial232;
	TSerial485* pSerial485;

	IQ_Generic<char,RX_BYTE_NUM> rxQ;
	IQ_Generic<char,TX_BYTE_NUM> txQ;
};

#endif /* GMM_DRIVER_INC_TCOM_SERIAL_H_ */
