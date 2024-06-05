/******************************************************************************
 *
 *  \brief      Standard message class for GMM Protocol
 *  \details    This class defines the abstract interface for standard protocol message.
 *              The template structure allows to define msg length as needed
 *  \author     Matteo De Silvestri - General Medical Merate
 *  GMM.spa - Seriate - Italy
 *  \version    1.0
 *  \date       June 3rd, 2019
 *  \pre        When instantiating need to specify max message length (stTemplateMessage<*maxDataLength*>)
 *  \warning    Improper use can crash your application
 *  \copyright  GMM.spa - All Rights Reserved
 *
 *****************************************************************************/

#ifndef IMESSAGE_H
#define IMESSAGE_H


#ifdef QT_CORE_LIB
#include <QDebug>
#endif

#include <string.h>
#include <stdint.h>

#define MAX_DATA_LEN 8
#define MIN_DATA_LEN 1
#define ID_LEN 2
#define ST_MSG_LEN 8
#define ST_CAN_SEIRAL_MSG_LEN 20

typedef struct {
    uint32_t data_H;
    uint32_t data_L;
} dataBlock_t;

class stAbstractMessage
{
public:
    stAbstractMessage() {}
    stAbstractMessage(const stAbstractMessage& msg)
    {
    	*this = msg;
    }
    virtual ~stAbstractMessage() = 0;

    //pure virtual member functions
    //getters
    virtual uint8_t maxPayload(void) = 0;
    virtual const uint32_t& msgID(void) const = 0;
    virtual const uint8_t* pMsg(int idx = 0) const = 0;
    virtual const uint8_t& nBytes(void) const = 0;
    //setters
    virtual void msgID(uint32_t) = 0;
	virtual void pMsg(const uint8_t* pMsg, int nBytes, int offset) = 0;
	virtual void pMsg(uint8_t byte, int offset) = 0;
    virtual void nBytes(uint8_t) = 0;

    //static member functions
    /**
     * @brief:  build a byte array from "size" bytes of src starting
     *          from "offset" byte of src
     **/
    template <typename T>
    static uint8_t toByteArray(uint8_t* dst, T src, unsigned int size = sizeof(T), unsigned int offset = 0) {
        uint8_t nBytes = 0;
        memset(dst, 0, size);
        unsigned int shift = size * 8;

        if (size <= sizeof(T) && size > 0) {    //if size bigger than 4Bytes, variable is too big to fit inside a long
                                                //consider trimming the variable instead of ignoring the request
            shift += shift < sizeof(T) * 8 ? offset * 8 : 0;
            for (unsigned int i = 0; i < size; i++) {
                shift -= 8;
                dst[i] = uint8_t( (src >> shift ) & 0xFF);
                nBytes++;
            }
        }
        return nBytes;
    }

    /**
     * @brief:  builds a T number from "size" elements of a byte array (src),
     *          shifting them to left of "offset" bytes (up to sizeof(T))
     **/
    template <typename T>
    static T fromByteArray(const uint8_t* src, unsigned int size = sizeof(T), unsigned int offset = 0) {
        T dst = 0;
        unsigned int shift = size * 8;

        if (size <= sizeof(T) && size > 0) {
            shift += shift < sizeof(T) * 8 ? offset * 8 : 0;
            for (unsigned int i = 0; i < size; i++) {
                shift -= 8;
                dst |= T(src[i] << shift);
            }
        }
        return dst;
    }
};

template <int maxDataLen = MAX_DATA_LEN>
class stTemplateMessage: public stAbstractMessage
{
public:

	//Constructor
    stTemplateMessage()
	{
	    m_msgID = 0;
	    m_nBytes = 0;
        memset(m_pMsg, 0, maxDataLen);
	}

    //Constructor
    stTemplateMessage(const stAbstractMessage& parent):
		stAbstractMessage(parent)
	{
	    m_msgID = 0;
	    m_nBytes = 0;
        memset(m_pMsg, 0, maxDataLen);
	}

	//Constructor
    stTemplateMessage(uint32_t id, uint8_t* msg, uint8_t bytes)
	{
	    m_msgID = id & 0x7FF; //CAN IDs are 11bits
        m_nBytes = bytes > maxDataLen ? maxDataLen : bytes;
        memset(m_pMsg, 0, maxDataLen); //be sure to initialize array
	    memcpy(m_pMsg, msg, m_nBytes);
	}

	//Destructor
    virtual ~stTemplateMessage()
	{

	}

//----Can message utility methods-----------------------------------------------------

    virtual uint8_t getCmd(void) {
    	uint8_t result = 0;
        if (maxDataLen > 0)
    		result = m_pMsg[0];
        return result;
    }

    virtual uint8_t getIdx(void) {
    	uint8_t result = 0;
        if (maxDataLen > 1)
    		result = m_pMsg[1];
        return result;
    }

    virtual uint16_t getSubIdx(void) {
    	uint16_t result = 0;
        if (maxDataLen > 3)
    		result = fromByteArray<uint16_t>(m_pMsg + 2);
		return result;
    }

    virtual uint32_t getCompIdx(void) {
    	uint32_t result = 0;
        if (maxDataLen > 3)
    		result = fromByteArray<uint32_t>(m_pMsg + 1, 3, 0);
        return result;
    }

    virtual uint32_t getData(uint8_t size = 4) {
    	uint32_t result = 0;
        if (maxDataLen > 7)
		result = fromByteArray<uint32_t>(m_pMsg + 4, size);
		return result;
    }

    virtual uint8_t setCmd(uint8_t u8Cmd) {
        uint8_t size = 0;

        if (maxDataLen > 0)
        {
			m_pMsg[0] = u8Cmd;
			size = 1;
    	}
        return size;
    }

    virtual uint8_t setIdx(uint8_t u8Idx) {
        uint8_t size = 0;

        if (maxDataLen > 1)
        {
					m_pMsg[1] = u8Idx;
					size = 1;
        }
        return size;
    }

    virtual uint8_t setSubIdx(uint8_t u8SubIdx_h, uint8_t u8SubIdx_l) {
        uint8_t size = 0;

        if (maxDataLen > 3)
        {
					m_pMsg[2] = u8SubIdx_h;
					m_pMsg[3] = u8SubIdx_l;
					size = 2;
        }
        return size;
    }

    virtual uint8_t setSubIdx(uint16_t u16SubIdx) {
        uint8_t size = 0;

        if (maxDataLen > 3)
        	size += toByteArray<uint16_t>(m_pMsg + 2, u16SubIdx);

        return size;
    }

    virtual uint8_t setData(uint8_t u8Data_hh, uint8_t u8Data_hl, uint8_t u8Data_lh, uint8_t u8Data_ll) {
        uint8_t size = 0;

        if (maxDataLen > 7)
        {
					m_pMsg[4] = u8Data_hh;
					m_pMsg[5] = u8Data_hl;
					m_pMsg[6] = u8Data_lh;
					m_pMsg[7] = u8Data_ll;
					size = 4;
        }
        return size;
    }

    virtual uint8_t setData(uint32_t u32Data, uint8_t dataSize = 4) {
        uint8_t size = 0;

        if (maxDataLen > 7)
        	size += toByteArray<uint32_t>(m_pMsg + 4, u32Data, dataSize);

        return size;
    }

    virtual uint8_t setDataBlock(dataBlock_t dataBlock) {
        uint8_t size = 0;

        if (maxDataLen > 7)
        {
			size += toByteArray<uint32_t>(m_pMsg, dataBlock.data_H);
			size += toByteArray<uint32_t>(m_pMsg + 4, dataBlock.data_L);
        }
        return size;
    }

    virtual dataBlock_t getDataBlock(void) {
        dataBlock_t dataBlock;
        if (maxDataLen > 3)
        {
			dataBlock.data_H = fromByteArray<uint32_t>(m_pMsg);
            if (maxDataLen > 7)
	        {
	        	dataBlock.data_L = fromByteArray<uint32_t>(m_pMsg + 4);
	        }
        }
        return dataBlock;
    }
//------------------------------------------------------------------------------------

    //getters
    virtual uint8_t maxPayload(void) { return (maxDataLen); }
    virtual const uint32_t& msgID(void) const { return m_msgID; }
    virtual const uint8_t* pMsg(int idx = 0) const { return idx < maxDataLen ? (&(m_pMsg[0]) + idx) : 0;}
    virtual const uint8_t& nBytes(void) const { return m_nBytes; }
	
    //setters
    virtual void msgID(uint32_t msgID) { m_msgID = msgID& 0x7FF; }//CAN IDs are 11bits
	virtual void pMsg(const uint8_t* pMsg, int nBytes, int offset) { memcpy(m_pMsg + offset, pMsg, nBytes); }
	virtual void pMsg(uint8_t byte, int offset) { m_pMsg[offset] = byte; }
	virtual void nBytes(uint8_t nBytes) { m_nBytes = nBytes; }

    bool operator == (const stTemplateMessage &lhs) const {
        int i = 0;
        bool result = (this->m_nBytes == lhs.nBytes()) && (this->m_msgID == lhs.msgID());
        while(i < this->m_nBytes && result == true) {
            result = this->m_pMsg[i] == *lhs.pMsg(i);
            i++;
        }
        return	result;
    }

    bool operator != (const stTemplateMessage &lhs) const {
        return !(*this == lhs);
    }

    uint32_t	m_msgID;
    uint8_t		m_pMsg[maxDataLen];
    uint8_t		m_nBytes;
};

inline stAbstractMessage::~stAbstractMessage() {}

typedef stTemplateMessage<MAX_DATA_LEN> stMessage;
typedef stTemplateMessage<ST_CAN_SEIRAL_MSG_LEN> stCanSerialMessage;

#endif // IMESSAGE_H
