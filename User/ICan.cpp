#include "ICan.h"

// private defines -------------------------------------------------------------
#define CAN_CLK						RCC_APB1Periph_CAN1
#define CAN_RX_PIN					GPIO_Pin_0
#define CAN_TX_PIN					GPIO_Pin_1
#define CAN_GPIO_PORT				GPIOD
#define CAN_GPIO_CLK				RCC_AHB1Periph_GPIOD
#define CAN_AF_PORT					GPIO_AF_CAN1
#define CAN_RX_SOURCE				GPIO_PinSource0
#define CAN_TX_SOURCE				GPIO_PinSource1

CAN_Struct str_init = {CAN1, 3, 0, 4, 0};
ICan* pCanInterface;
long ICan::s32CurrentQ = 0;

/**
 * Constructor
 * @param str_init
 */
ICan::ICan(CAN_Struct str_init):
																															pCANx(str_init.pCAN),
																															u8RxPP(str_init.u8RxPP),
																															u8RxSP(str_init.u8RxSP),
																															u8TxPP(str_init.u8TxPP),
																															u8TxSP(str_init.u8TxSP),
																															bInitialized(false),
																															bRunning(false),
																															lErrCounter(0),
																															s32CurrentRxCB(0),
																															s32CurrentTxCB(0),
																															eCanSpeed(CAN_BUS_SPEED_500)


{
	init(MODE_NORMAL);
}
//----------------------------------------------------------------------------
/**
 * Destructor
 */
ICan::~ICan(void)
{
	//destructor
}
//----------------------------------------------------------------------------
/**
 * GPIO configuration
 */
void ICan::CAN_GPIO_Config(void)
{
	// CAN GPIOs configuration --------------------------------- //
	// Enable GPIO clock //
	RCC_AHB1PeriphClockCmd(CAN_GPIO_CLK, ENABLE);
	RCC_APB1PeriphClockCmd(CAN_CLK, ENABLE);

	// Connect CAN pins to AF9 //
	GPIO_PinAFConfig(CAN_GPIO_PORT, CAN_RX_SOURCE, CAN_AF_PORT);
	GPIO_PinAFConfig(CAN_GPIO_PORT, CAN_TX_SOURCE, CAN_AF_PORT);

	//GPIO_PinRemapConfig(GPIO_Remap1_CAN1, ENABLE);//Ó³Éä

	// Configure CAN RX and TX pins //
	GPIO_InitStructure.GPIO_Pin = CAN_RX_PIN | CAN_TX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_Init(CAN_GPIO_PORT, &GPIO_InitStructure);
}
//----------------------------------------------------------------------------
/**
 * NVIC CAN configuration
 */
void ICan::CAN_NVIC_Config(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = u8RxPP;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = u8RxSP;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = CAN1_TX_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = u8TxPP;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = u8TxSP;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
//----------------------------------------------------------------------------
/**
 * CAN mode configuration
 * @param CAN_Mode  to be passed
 */
void ICan::CAN_Mode_Config(enCanMode CAN_Mode)
{
	if (!IS_CAN_MODE(CAN_Mode))
		CAN_Mode = MODE_NORMAL;

	if (pCANx != 0)
		CAN_DeInit(pCANx);

	CAN_StructInit(&CAN_InitStructure);

	//CANµ¥ÔªÉèÖÃ 
	CAN_InitStructure.CAN_TTCM = DISABLE;			   //MCR-TTCM
	CAN_InitStructure.CAN_ABOM = DISABLE;			   //MCR-ABOM 
	CAN_InitStructure.CAN_AWUM = DISABLE;			   //MCR-AWUM
	CAN_InitStructure.CAN_NART = DISABLE;			   //MCR-NART
	CAN_InitStructure.CAN_RFLM = DISABLE;			   //MCR-RFLM
	CAN_InitStructure.CAN_TXFP = DISABLE;			   //MCR-TXFP
	CAN_InitStructure.CAN_Mode = CAN_Mode;

	//ÉèÖÃ²¨ÌØÂÊ  36/(1+5+2)/9 =500K
	CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
	CAN_InitStructure.CAN_BS1 = CAN_BS1_16tq;
	CAN_InitStructure.CAN_BS2 = CAN_BS2_4tq;

	setSpeed(eCanSpeed);

	if (pCANx != 0)
		CAN_Init(pCANx, &CAN_InitStructure);

}
//----------------------------------------------------------------------------
/**
 * @brief CAN filter configuration by Id Mask
 * @param u16Id01  mask Id to be passed
 * @param u16Id02  mask Id to be passed
 * @param u16Id03  mask Id to be passed
 * @param u16Id04  mask Id to be passed
 * @return true if filter has been configured
 */
bool ICan::filterByIdList( uint16_t u16Id01, uint16_t u16Id02, uint16_t u16Id03, uint16_t u16Id04 )
{
	bool bFilterConfigured = false;

	if ( u8FilterNumb >= 0 && u8FilterNumb <= 13 )
	{
		CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;
		CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
		CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdList;
		CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_16bit; //CAN_FilterScale_16bit;

		CAN_FilterInitStructure.CAN_FilterIdHigh      = ( u16Id01 << 5 );
		CAN_FilterInitStructure.CAN_FilterIdLow       = ( u16Id02 << 5 );
		CAN_FilterInitStructure.CAN_FilterMaskIdHigh  = ( u16Id03 << 5 );
		CAN_FilterInitStructure.CAN_FilterMaskIdLow   = ( u16Id04 << 5 );
		CAN_FilterInitStructure.CAN_FilterNumber = u8FilterNumb;
		CAN_FilterInit(&CAN_FilterInitStructure);
		u8FilterNumb++;
		bFilterConfigured = true;
	}
	return( bFilterConfigured );
}
//----------------------------------------------------------------------------
/**
 * @brief CAN filter configuration by Id Mask
 * @param u16IdAndMask  mask Id to be passed
 * @param u16IdComp  mask Id to be passed
 * @return true if filter has been configured
 * @remark the Mask applied to Can ID# follows the following rule
 *  - if (( ID# & u16IdAndMask ) == u16IdComp) then AddToFifo
 *  @note this C code can test all the messages that will pass rule above
	    #include <stdio.h>
		#define COMP 0x100   //  u16IdAnd
		#define MASK 0x700   //  u16IdComp
		int main(int argc, char **argv)
		{
		  int id;
		  for(id=0; id<0x800; id++)
			if ((id & MASK) == COMP) printf("%03X\n", id);
		  return(1);
		}
 */
bool ICan::filterByIdMask( uint16_t u16IdAndMask, uint16_t u16IdComp)
{
	bool bFilterConfigured = false;
	if ( u8FilterNumb <= 13 )
	{
		CAN_FilterInitStructure.CAN_FilterNumber = u8FilterNumb;
		CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0; // FIFO = 0
		CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask; // Filter mode = identifier mask based filtering
		CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
		CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
		/* Filter 0x.0x1FF */
		CAN_FilterInitStructure.CAN_FilterIdHigh =     ( u16IdComp << 5 ); // 11-bit ID in top bits
		CAN_FilterInitStructure.CAN_FilterIdLow = 0;
		CAN_FilterInitStructure.CAN_FilterMaskIdHigh = ( u16IdAndMask << 5 );
		CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0;
		CAN_FilterInit(&CAN_FilterInitStructure);
		u8FilterNumb++;
		bFilterConfigured = true;
	}
	return( bFilterConfigured );
}
//----------------------------------------------------------------------------
/**
 * @brief CAN filter configuration by Id Mask
 * @param u16CanFiltLow   lower range bound
 * @param u16CanFiltHigh  upper range bound
 * @return true if filter has been configured
 * @remark the Mask applied to Can ID# can enlarge Id filtered

 */
bool ICan::filterByRange( uint16_t u16CanFiltLow, uint16_t u16CanFiltHigh)
{
	bool bFilterConfigured = false;
	uint16_t u16IdBMasked;
	uint16_t u16IdComp, u16IdMask;


	for ( int ii=0; ii<16; ii++ )
	{
		u16IdMask    = 0xFFff << ii;
		u16IdComp    = u16IdMask & u16CanFiltLow ;
		u16IdBMasked = u16IdMask & u16CanFiltHigh;
		if( u16IdComp == u16IdBMasked )
		{
			bFilterConfigured = filterByIdMask(u16IdMask, u16IdComp);
			break;
		}
	}
	return( bFilterConfigured );

	/*
	//aternative, only works if extremes have the same number of bits
	bool bFilterConfigured = false;
	uint16_t u16IdComp;
	uint16_t u16IdMask = uint16_t(0xFFFF);
    uint16_t tmpHigh = u16CanFiltHigh;
    uint16_t tmpLow = u16CanFiltLow;
    int i = 0;

    while(tmpHigh != tmpLow && tmpLow != 0)
    {
        tmpHigh >>= 1;
        tmpLow >>= 1;
        u16IdMask <<= 1;
        i++;
    }
    u16IdComp = uint16_t(tmpHigh<<i);

    bFilterConfigured = filterByIdMask(u16IdMask, u16IdComp);

	return( bFilterConfigured );
	 */
}
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
/**
 * Configuration for CAN interrupt
 */
void ICan::CAN_IT_Config(void)
{
	if (pCANx != 0) 
	{
		CAN_ITConfig(pCANx, CAN_IT_FMP0, ENABLE);
		CAN_ITConfig(pCANx, CAN_IT_TME, ENABLE);
	}
}
//----------------------------------------------------------------------------
/**
 * Set Speet to CAN
 * @param speed
 */
void ICan::setSpeed(enCanSpeed speed)
{
	eCanSpeed = speed;
	switch (eCanSpeed)
	{
	case CAN_BUS_SPEED_125:
		CAN_InitStructure.CAN_Prescaler = 16;
		break;
	case CAN_BUS_SPEED_250:
		CAN_InitStructure.CAN_Prescaler = 8; //	8/42MHz = 0.190476190 us
		// 		CAN Baudrate => 250kbps (CAN clocked at 21 MHz) //
		//		CAN_SJW_1tq + CAN_BS1_16tq + CAN_BS2_4tq = 1+16+4 = 21tq => 21*0.19047 = 4us => 250kHz
		break;
	case CAN_BUS_SPEED_500:
	default:
		CAN_InitStructure.CAN_Prescaler = 4; //	4/42MHz = 0.095238095 us
		//		CAN Baudrate => 500kbps (CAN clocked at 42 MHz) //
		// 		CAN_SJW_1tq + CAN_BS1_16tq + CAN_BS2_4tq = 1+16+4 = 21tq => 21*0.09524 = 2us => 500kHz
	}
}
//----------------------------------------------------------------------------
/**
 * CAN port initialization
 * @param mode
 */
bool ICan::init(enCanMode mode)
{
	if (!bInitialized)
	{
		CAN_GPIO_Config();
		CAN_NVIC_Config();
		CAN_Mode_Config(mode);

		for (int i = 0; i < MAX_Q_NUM; i++)
			rxQ[i].reset();
		txQ.reset();

		bInitialized = true;
	}

	return(true);
}
//----------------------------------------------------------------------------
/**
 * Open CAN port and configure registered filters
 * @param qID	instance ID of the user wanting to open the port
 */
void ICan::open(long qID)
{
	if (qID >= 0 && qID < MAX_Q_NUM)
	{
		if (qOpened.indexOf(qID) < 0)
		{
			qOpened.push(qID);

#warning IL DRIVER VECCHIO MI CHIUDE TUTTA LA PORTA X OGNI UTENTE ... FORSE DEVO PRIMA METTERE NUOVI DRIVER E POI TESTARE CODICE

			closeComm(); //IL DRIVER VECCHIO MI CHIUDE TUTTA LA PORTA X OGNI UTENTE ... FORSE DEVO PRIMA METTERE NUOVI DRIVER E POI TESTARE CODICE
			initFilters();
			openComm();
		}
	}
}
//----------------------------------------------------------------------------
/**
 * try to set particular Set CAN mode
 * @param mode   to be set
 * @return true or false
 *
 * @remark CAn modes can be
 *  - CAN_Mode_Normal
 *  - CAN_Mode_LoopBack
 *  - CAN_Mode_Silent
 *  - CAN_Mode_Silent_LoopBack

 */
bool ICan::setMode(enCanMode mode)
{
	bool result = false;

	if (IS_CAN_MODE(mode))
	{
		CAN_Mode_Config(mode);
		CAN_IT_Config();
		result = true;
	}
	return result;
}
//----------------------------------------------------------------------------
/**
 * Build and deliver CAN message
 * @param msgID     ID for CAN message
 * @param pMsg      ptr to message payload
 * @param nByte     size of payload array
 */
void ICan::write(const unsigned int msgID,
		const char *pMsg,
		unsigned int nBytes)
{
	SetupTxMessage (msgID, pMsg, nBytes);



}
//----------------------------------------------------------------------------
/**
 * Write new message
 * @param msg  ptr to message structure to be delivered
 */
void ICan::write(CanTxMsg* msg, enTxMode txMode)
{
	txAppend.push(msg);
}
//----------------------------------------------------------------------------
/**
 * Execute queue buffering
 */
void ICan::execute(void)
{
	CanTxMsg* msg;
	if ( 
		txAppend.size() >0 &&
		txQ.size() == 0     )
	{
		pauseIT_TX();
		txAppend.pop(msg);
		txQ.push(msg);
		releaseIT_TX();
		transmitData();
	}
}
//----------------------------------------------------------------------------
/**
 * Read current message and copy to msg
 * @param msg  ptr to message structure to be filled
 * @return number of messages read
 */
long ICan::read(CanRxMsg* msg, long qID)
{
	long result = -1;

	if (qID <= s32CurrentQ && qID >= 0)
	{
		result = rxQ[qID].pop(msg);
	}

	return result;
}
//----------------------------------------------------------------------------
/**
 * @brief Check for new messages and feed them to rx queues
 */
void ICan::receiveData(void)
{
	CanRxMsg msg;
	CAN_Receive(pCANx, CAN_FIFO0, &msg);
	if (msg.RTR == CAN_RTR_Data)
	{
		for (int i = 0; i <= s32CurrentQ; i++)
			rxQ[i].push(&msg);
	}
	else if (lErrCounter < 5)
	{
		increaseErrorCounter();
	}
	else
	{
		this->closeComm();
	}
}
//----------------------------------------------------------------------------
/**
 * @brief Take messages from tx queue and send them
 */
void ICan::transmitData(void)
{
	if (txQ.size() > 0)
	{
		CanTxMsg msg;
		txQ.pop(&msg);
		if (CAN_Transmit(pCANx, &msg) == CAN_TxStatus_NoMailBox)
		{
			//reset port
			closeComm();
			initFilters();
			openComm();
		}else
		{
#ifdef DBG_ICAN
			//--- DEBUG -----
			tDbg.write("\r%%ICAN\tTX:%03x@",msg.StdId);
			for (int i=0; i<8; i++)
			{
				tDbg.write("%02x ",msg.Data[i]);
			}
#endif
		}
	}

	if (txQ.size() == 0)
	{
		pauseIT_TX();
	}
}
//----------------------------------------------------------------------------
/**
 * Interrupt service function RX
 */
void ICan::pauseIT_TX(void)
{
	CAN_ITConfig(pCANx, CAN_IT_TME, DISABLE);
}
//----------------------------------------------------------------------------
/**
 * Re activate  CAN interrupt TX
 */
void ICan::releaseIT_TX(void)
{
	CAN_ITConfig(pCANx, CAN_IT_TME, ENABLE);
	if( txQ.size() > 0 )
	{
		transmitData(); // triggers the first transmission
	}
}
//----------------------------------------------------------------------------
/**
 * Pause CAN interrupt RX
 */
void ICan::pauseIT_RX(void)
{
	CAN_ITConfig(pCANx, CAN_IT_FMP0, DISABLE);
}
//----------------------------------------------------------------------------
/**
 * Re activate  CAN interrupt RX
 */
void ICan::releaseIT_RX(void)
{
	CAN_ITConfig(pCANx, CAN_IT_FMP0, ENABLE);
}
//---------------------------------------------------------------------------
/**
 * @brief prepare the CAN message:
 * @param msgID		message identifier
 * @param pMsg		the message sequence (max 8 byte) !IT IS NOT A STRING!
 * @param nBytes		number of byte of the message !! (you can transmit 0)
 */
void ICan::SetupTxMessage (const unsigned int msgID,
		const char *pMsg,
		unsigned int nBytes)
{
	unsigned int i;
	CanTxMsg txDummy;

	if (nBytes > 8)
	{
		nBytes = 8;
	}

	txDummy.StdId = msgID;
	txDummy.ExtId = 0;
	txDummy.IDE = CAN_Id_Standard;
	txDummy.RTR = CAN_RTR_Data;
	txDummy.DLC = nBytes; // number of transmitted Byte

	//--- number of transmitted Byte
	for (i=0; i<nBytes; i++)
	{
		txDummy.Data[i] = pMsg[i];
	}
	for (i=nBytes; i<8; i++) 
	{
		txDummy.Data[i] = 0;
	}

	txAppend.push(&txDummy);
}
//---------------------------------------------------------------------------
/**
 * @brief Close CAN port instance
 * @param qID	instance ID of hte user wanting to close the port
 */
void ICan::close(long qID)
{
	if (qOpened.indexOf(qID) >= 0)
		qOpened.remove(qOpened.indexOf(qID));

	if (qOpened.size() == 0)
	{
		closeComm();
	}
}
//---------------------------------------------------------------------------
/**
 * @brief increase CAN errors counter
 */
void ICan::increaseErrorCounter(void)
{
	lErrCounter++;
}
//---------------------------------------------------------------------------
/**
 * @brief Physically open CAN port
 */
void ICan::openComm(void)
{
	CAN_IT_Config();
	bRunning = true;
}
//---------------------------------------------------------------------------
/**
 * @brief Physically close CAN port
 */
void ICan::closeComm(void)
{
	CAN_DeInit(pCANx);
	eCanSpeed = CAN_BUS_SPEED_500;
	u8FilterNumb = 0;
	lErrCounter = 0;
	for (int i = 0; i < MAX_Q_NUM; i++)
		rxQ[i].reset();
	txQ.reset();
	bInitialized = false;
	bRunning = false;
}
//---------------------------------------------------------------------------
/**
 * @brief Initialize CAN filters
 */
void ICan::initFilters(void)
{
	int cnt = 0;
	uint16_t aIDs[4] = {0, 0, 0, 0};
	IQ_Generic<uint16_t, MAX_FILTERS_NUM> qUniqeFilters;

	getUniqueFilters(qUniqeFilters);

	init(MODE_NORMAL);

	for (int i = 0; i < qUniqeFilters.size(); i++)
	{
		aIDs[cnt] = qUniqeFilters.at(i);
		cnt++;
		if (cnt == 4)	//register block of 4 filters
		{
			filterByIdList(aIDs[0], aIDs[1], aIDs[2], aIDs[3]);
			cnt = 0;
			aIDs[0] = 0;
			aIDs[1] = 0;
			aIDs[2] = 0;
			aIDs[3] = 0;
		}
	}

	if (cnt > 0)	//register remaining filters ( qUniqeFilters.size() % 4 )
		filterByIdList(aIDs[0], aIDs[1], aIDs[2], aIDs[3]);
}
//---------------------------------------------------------------------------
/**
 * @brief Builds list of all the registered filter IDs (without repetitions)
 * @param qDst			destination queue
 */
void ICan::getUniqueFilters(IQ_Generic<uint16_t, MAX_FILTERS_NUM>& qDst)
{
	for (int i = 0; i < qOpened.size(); i++)
	{
		for (int j = 0; j < qFilters[qOpened.at(i)].size(); j++)
		{
			if (qDst.indexOf(qFilters[qOpened.at(i)].at(j)) < 0) //add if unique
			{
				qDst.push(qFilters[qOpened.at(i)].at(j));
			}

			if (qDst.isFull()) //exit both cycles
			{
				j = qFilters[qOpened.at(i)].size();
				i = qOpened.size();
			}
		}
	}
}
//---------------------------------------------------------------------------
/**
 * @brief Creates connection to new interface and assign it a new rx queue
 * @param pCan			pointer to interface ICan port
 * @return s32Result	user personal queue ID; -1 if all queues have been already taken
 */
long ICan::getInstance(ICan* &pCan) {

	static ICan iCan(str_init);
	pCanInterface = &iCan;

	long s32Result = -1;

	if (s32CurrentQ + 1 < MAX_Q_NUM) {
		pCan = &iCan;
		s32Result = s32CurrentQ;
		s32CurrentQ++;
	}

	return s32Result;
}
//---------------------------------------------------------------------------
/**
 * @brief Add callback function to apRxCallbackList list
 * @param pFun		pointer to callback function
 * @return bResult	false if max number of callbacks has already been reached; true otherwise
 */
bool ICan::registerRxCallback(pmf_t pFun)
{
	bool bResult = false;
	if (pFun != 0)
	{
		if (s32CurrentRxCB + 1 < MAX_Q_NUM)
		{
			apRxCallbackList[s32CurrentRxCB] = pFun;
			s32CurrentRxCB++;
			bResult = true;
		}
	}

	return bResult;
}
//---------------------------------------------------------------------------
/**
 * @brief Add callback function to apTxCallbackList list
 * @param pFun		pointer to callback function (member functions not allowed)
 * @return bResult	false if max number of callbacks has already been reached; true otherwise
 */
bool ICan::registerTxCallback(pmf_t pFun)
{
	bool result = false;
	if (pFun != 0)
	{
		if (s32CurrentTxCB + 1 < MAX_Q_NUM)
		{
			apTxCallbackList[s32CurrentTxCB] = pFun;
			s32CurrentTxCB++;
			result = true;
		}
	}
	return result;
}
//---------------------------------------------------------------------------
/**
 * @brief Add single ID to filter list
 * @param qID		instance ID
 * @param u16ID		msg ID to be registered
 * @return bResult	true if ID actually registered
 */
bool ICan::registerFilter(long qID, uint16_t u16ID)
{
	bool bRegistered = false;

	if (qID >= 0 && qID < MAX_Q_NUM)
	{
		if ( !this->qFilters[qID].isFull() && filtersAvailable() > 0)
		{
			this->qFilters[qID].push(u16ID);
			bRegistered = true;
		}
	}

	return bRegistered;
}
//---------------------------------------------------------------------------
/**
 * @brief Add two IDs to filter list
 * @param qID		instance ID
 * @param u16IDn		msg IDs to be registered
 * @return bResult	true if IDs actually registered
 */
bool ICan::registerFilter(long qID, uint16_t u16ID0, uint16_t u16ID1)
{
	bool bResult = true;

	bResult = bResult && registerFilter(qID, u16ID0);
	bResult = bResult && registerFilter(qID, u16ID1);

	return bResult;
}
//---------------------------------------------------------------------------
/**
 * @brief Add three IDs to filter list
 * @param qID		instance ID
 * @param u16IDn		msg IDs to be registered
 * @return bResult	true if IDs actually registered
 */
bool ICan::registerFilter(long qID, uint16_t u16ID0, uint16_t u16ID1, uint16_t u16ID2)
{
	bool bResult = true;

	bResult = bResult && registerFilter(qID, u16ID0, u16ID1);
	bResult = bResult && registerFilter(qID, u16ID2);

	return bResult;
}
//---------------------------------------------------------------------------
/**
 * @brief Add four IDs to filter list
 * @param qID		instance ID
 * @param u16IDn		msg IDs to be registered
 * @return bResult	true if IDs actually registered
 */
bool ICan::registerFilter(long qID, uint16_t u16ID0, uint16_t u16ID1, uint16_t u16ID2, uint16_t u16ID3)
{
	bool bResult = true;

	bResult = bResult && registerFilter(qID, u16ID0, u16ID1);
	bResult = bResult && registerFilter(qID, u16ID2, u16ID3);

	return bResult;
}
//---------------------------------------------------------------------------
/**
 * @brief Compute number of filters which can still be registered
 * @return lResult	number of available filters
 */
long ICan::filtersAvailable(void)
{
	long lResult = MAX_FILTERS_NUM;
	IQ_Generic<uint16_t, MAX_FILTERS_NUM> qUniqeIDs;

	getUniqueFilters(qUniqeIDs);

	lResult -= qUniqeIDs.size();


	return lResult;
}//---------------------------------------------------------------------------
/**
 * @brief Rx IRQ Handler
 */
bool ICan::txPending(void)
{
	bool bTxPending = true;
	if ( 	CAN_TransmitStatus(pCANx,0) == CAN_TxStatus_Ok ||
			CAN_TransmitStatus(pCANx,1) == CAN_TxStatus_Ok ||
			CAN_TransmitStatus(pCANx,2) == CAN_TxStatus_Ok )
	{
		bTxPending = false;
	}
	return ( bTxPending );
}
//---------------------------------------------------------------------------
/**
 * @brief Rx IRQ Handler
 */
void ICan::rxIRQ(void)
{
	receiveData();
	for (int i = 0; i < s32CurrentRxCB; i++)
	{
		(*apRxCallbackList[i])();
	}
}
//---------------------------------------------------------------------------
/**
 * @brief Tx IRQ Handler
 */
void ICan::txIRQ(void)
{
	for (int i = 0; i < s32CurrentTxCB; i++)
	{
		(*apTxCallbackList[i])();
	}
	transmitData();
}

//---------------------------------------------------------------------------
/**
 * @brief Interrupt CAN RX on Port 0
 */
void CAN1_RX0_IRQHandler(void)
{//declared as extern in ICan.h
	//receive ISR
	if (pCanInterface != 0)
		pCanInterface->rxIRQ();
}
//---------------------------------------------------------------------------
/**
 * @brief Interrupt CAN TX on Port 0
 */
void CAN1_TX_IRQHandler(void)
{//declared as extern in ICan.h
	//Transmit ISR
	if (pCanInterface != 0)
		pCanInterface->txIRQ();
}
