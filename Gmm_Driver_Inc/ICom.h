/******************************************************************************
 *
 *  \brief      Definition of the ICom interface
 *  \details    This class implements the ICom interface abstract class
 *  \author     Matteo De Silvestri - General Medical Merate
 *  GMM.spa - Seriate - Italy
 *  \version    1.0
 *  \date       June 3rd, 2019
 *  \warning    Improper use can crash your application
 *  \copyright  GMM.spa - All Rights Reserved
 *
 *****************************************************************************/

#ifndef I_COM_H
#define I_COM_H

#include <stdint.h>
#include "stMessage.h"

#define MAX_Q_NUM 5
#define MAX_ID_NUM 4 * 14 //limited by Can filters (14 filters, 4 IDs per filter)

typedef enum enComTypes {
	COM_CAN,
	COM_RS232,
	COM_RS485,
	COM_CAN_SERIAL,
	COM_STUB,
	COM_NUMEL
} enComTypes_t;


class ICom {

public:

	//constructor
	ICom(void) {}

	//pure virtual destructor
	virtual ~ICom(void) = 0;

	//pure virtual methods
	virtual void init() = 0;
	virtual void open(void) = 0;
	virtual void close() = 0;
	virtual int read(stAbstractMessage*, long qID = 0) = 0;

	virtual void write(const stAbstractMessage*) = 0;
	virtual bool isOpen(void) = 0;
	virtual bool isToRead(long qID = 0) = 0;
	virtual bool txPending(void){ return false; }
	virtual bool isInError(void){ return false; }
	virtual uint8_t getErrCntr(void){return 0;}

	//---------------------------------------------------------------------------
	/**
	 * @brief Creates connection to new user and assign it a new rx queue
	 * @param pCom			pointer to user ICom interface
	 * @return s32Result	user personal queue ID; -1 if all queues have been already taken
	 */
	long getInstance(ICom* &pCom) {
		long s32Result = -1;

		if (s32CurrentQ + 1 < MAX_Q_NUM) {
			pCom = this;
			s32Result = s32CurrentQ;
			s32CurrentQ++;
		}

		return s32Result;
	}
	virtual void peek(stMessage* msg){};
	virtual void Rx2TxMsg(stMessage* txMsg, stMessage rxMsg){};

	//virtual methods
	virtual void pauseIT_TX(void) {}
	virtual void releaseIT_TX(void) {}
	virtual void pauseIT_RX(void) {}
	virtual void releaseIT_RX(void) {}
	virtual void rxIRQ(void) {}
	virtual void txIRQ(void) {}
	virtual void registerID(uint16_t u16Id) {}

	enComTypes getComType(void) { return comType; }

protected:
	enComTypes_t comType;
	long s32CurrentQ;
	//consider adding non-common members that might be useful for only some child classes
};

inline ICom::~ICom() {}

#endif
