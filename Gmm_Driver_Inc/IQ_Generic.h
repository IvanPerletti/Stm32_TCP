/******************************************************************************
 *
 *  \brief		Messages queue template class
 *  \details	This class is used to implement queues for arbitrary type of messages
 *  \author 	Matteo De Silvestri - General Medical Merate
 *  GMM.spa - Seriate - Italy
 *  \version	1.0
 *  \date		June 3rd, 2019
 *  \pre		When instantiating need to specify type (IQ_Generic<*type*>)
 *  \warning	Improper use can crash your application
 *  \copyright	GMM.spa - All Rights Reserved
 *
 *****************************************************************************/

#ifndef IQ_GENERIC_H
#define IQ_GENERIC_H

#ifndef QT_CORE_LIB
extern "C" {
#include "stm32f4xx.h"
}
#else
#include "stdbool.h"
#include "stdint.h"
#include "string.h"
#endif

#define Q_SIZE 20		//queue max size: when reached no more elements are enqued
#define Q_LIMIT (Q_SIZE + 1)

template<typename Q_Type, int maxSize = Q_SIZE>
class IQ_Generic
{

public:
	//-----------------------------------------------------------------------
	/**
	 * constructor
	 */
	IQ_Generic(void){
		reset();
	}
	//-----------------------------------------------------------------------
	/**
	 * destructor
	 */
	~IQ_Generic(void){}
	//-----------------------------------------------------------------------
	/**
	 * @brief append element to queue (if slots still available)
	 * @param msg	pointer to element to be added
	 */
	void push(Q_Type* msg)
	{
		const uint8_t u8Index = (u8BottomIdx + 1)%(maxSize);
		if ( u8Index != u8TopIdx)
		{
			queue[u8BottomIdx % maxSize] = *msg;
			u8BottomIdx = u8Index;
		}
	}
	//-----------------------------------------------------------------------
	/**
	 * @brief append element to queue (if slots still available)
	 * @param msg	to element to be added
	 */
	void push(const Q_Type& msg)
	{
		const uint8_t u8Index = (u8BottomIdx + 1)%(maxSize);
		if ( u8Index != u8TopIdx)
		{
			queue[u8BottomIdx % maxSize] = msg;
			u8BottomIdx = u8Index;
		}
	}
	//-----------------------------------------------------------------------
	/**
	 * @brief append block of elements to queue (until queue is full)
	 * @param src	pointer to source buffer
	 */
	void push(Q_Type* src, int num)
	{
		if (num >= this->freeSlots())
		{
			num = this->freeSlots() - 1;
		}
		const uint8_t u8Index = (u8BottomIdx + num)%(maxSize);
		if ( u8Index != u8TopIdx)
		{
			memcpy(&queue[u8BottomIdx % maxSize], src, num * sizeof(Q_Type));
			u8BottomIdx = u8Index;
		}



		long i = 0;

		while(i < num && !this->isFull())
			this->push((Q_Type*)(src + i++));
	}
	//-----------------------------------------------------------------------
	/**
	 * @brief append element to queue discarding oldest element if necessary
	 * @param msg	element to be added
	 */
	void pushForced(const Q_Type& msg)
	{
		const uint8_t u8Index = (u8BottomIdx + 1)%(maxSize);
		if ( u8Index == u8TopIdx)
		{
			this->pop();	//if queue is full, remove first element to make room for the new one
		}

		this->push(msg);
	}
	//-----------------------------------------------------------------------
	/**
	 * @brief append element to queue, if not already present
	 * @param msg	pointer to element to be added
	 */
	void pushIfNot(Q_Type* msg)
	{
		if (has(*msg) == false)
		{
			const uint8_t u8Index = (u8BottomIdx + 1)%(maxSize);
			if ( u8Index != u8TopIdx)
			{
				queue[u8BottomIdx % maxSize] = *msg;
				u8BottomIdx = u8Index;
			}
		}
	}
	//-----------------------------------------------------------------------
	/**
	 * @brief append element to queue, if not already present
	 * @param msg	element to be added
	 */
	void pushIfNot(const Q_Type& msg)
	{
		if (has(msg) == false)
		{
			const uint8_t u8Index = (u8BottomIdx + 1)%(maxSize);
			if ( u8Index != u8TopIdx)
			{
				queue[u8BottomIdx % maxSize] = msg;
				u8BottomIdx = u8Index;
			}
		}
	}

	//-----------------------------------------------------------------------
	/**
	 * @brief extract first element
	 * @param msg	pointer to destination buffer, if NULL element is simply discarded
	 * @return number of extracted elements
	 */
	long pop(Q_Type* msg = (Q_Type*)(0))
	{
		long result = 0;
		const uint8_t u8Index = (u8TopIdx + 1)%maxSize;
		if ( !isEmpty() )
		{
			if (msg != 0) {
				*msg = queue[u8TopIdx % maxSize];
				result = 1;
			}
			u8TopIdx = u8Index;
		}
		return result;
	}

	/**
	 * @brief extract first element
	 * @param msg	destination buffer
	 * @return number of extracted elements
	 */
	long pop(Q_Type& msg)
	{
		long result = 0;
		const uint8_t u8Index =  (u8TopIdx + 1)%maxSize;
		if ( !isEmpty() )
		{
			msg = queue[u8TopIdx % maxSize];
			result = 1;
			u8TopIdx = u8Index;
		}
		return result;
	}

	/**
	 * @brief extract block of elements (from the top)
	 * @param msg	destination buffer
	 * @param num	amount of elements to extract (if == -1, extract all elements)
	 * @return number of extracted elements
	 */
	long pop(Q_Type* msg, int num)
	{
		long result = 0;

		if (num < 0)
			num = this->size();

		while(result < num && !this->isEmpty())
			this->pop((Q_Type*)(msg + result++));

		return result;
	}
	//-----------------------------------------------------------------------
	/**
	 * @brief reset class stuff
	 */
	void reset(void) {
		u8TopIdx = 0;
		u8BottomIdx = 0;
	}
	//-----------------------------------------------------------------------
	/**
	 * @brief delete element at given position
	 * @return trye if operation succeeded, false otherwise
	 */
	bool remove(int idx) {
		bool result = false;
		if (this->size() > 0 && idx < this->size())
		{
			for (int i = idx; i < this->size() - 1; i++)
			{
				at(i) = at(i + 1);
			}
			u8BottomIdx = u8BottomIdx > 0 ? (u8BottomIdx - 1) : (maxSize + 1);
			result = true;
		}
		return result;
	}
	//-----------------------------------------------------------------------
	/**
	 * @brief insert value at arbitrary position
	 * @return true if operation succeeded, false otherwise (position not acceptable, or queue full)
	 */
	bool insert(Q_Type* msg, int idx) {
		bool result = false;
		const uint8_t u8Index = (u8BottomIdx + 1)%(maxSize + 1);
		if (this->size() > idx && u8Index != u8TopIdx)
		{
			this->push(at(this->size() - 1));
			for (int i = u8BottomIdx - 2; i >= idx; i--)
			{
				at(i + 1) = at(i);
			}
			at(idx) = *msg;
			result = true;
		}
		return result;
	}
	//-----------------------------------------------------------------------
	/**
	 * @brief test if queue is full
	 * @return true or false
	 */
	inline bool isFull(void)
	{
		const uint8_t u8Index = (u8BottomIdx + 1)%(maxSize + 1);
		return(u8Index == u8TopIdx);
	}
	//-----------------------------------------------------------------------
	/**
	 * @brief test if queue is Empty
	 * @return true or false
	 */
	inline bool isEmpty(void)
	{
		return(u8BottomIdx == u8TopIdx);
	}
	//-----------------------------------------------------------------------
	/**
	 * @brief check size of queue
	 * @return true or false
	 */
	inline int32_t size(void)
	{
		int32_t iDelta = (int32_t)u8BottomIdx - (int32_t)u8TopIdx;
		while( iDelta < 0 ){
			iDelta += maxSize;
		}
		return(iDelta);
	}
	//-----------------------------------------------------------------------
	/**
	 * @brief access value at given position
	 * @param lIdx	queue position to access
	 * @param bOk	optional, filled with true or false depending on operation success
	 * @return value at position lIdx
	 */

	Q_Type& at(const unsigned int ulIdx, bool * bOk = ((bool *) 0))
	{
		if ( bOk )
		{
			*bOk = ulIdx < this->size();
		}
		return queue[(u8TopIdx + ulIdx)%maxSize ];
	}
	//-----------------------------------------------------------------------
	/**
	 * @brief check if given element is already present in queue
	 * @param msg	element to check against
	 * @return true if present, false if not
	 */
	bool has(Q_Type msg) {
		bool result = false;
		if (this->size())
		{
			for (int i = 0; i < this->size() && result == false; i++)
			{
				if (at(i) == msg)
				{
					result = true;
				}
			}
		}
		return result;
	}
	//-----------------------------------------------------------------------
	//-----------------------------------------------------------------------
	/**
	 * @brief discard block of elements (starting from the top)
	 * @param num	amount of elements to discard
	 */
	void reject(int num)
	{
		if (num >= this->size())
		{
			this->reset();
		}
		else
		{
			u8TopIdx += num;
		}
	}
	//-----------------------------------------------------------------------
	/**
	 * @brief transfer block of data from queue to given destination
	 * @param dst		data destination buffer
	 * @param maxNum	maximum number of elements to be transfered
	 * @return number of transfered elements
	 */
	long flush(Q_Type* dst, int maxNum = -1)
	{
		//#warning "remove: duplicate of pop(Q_Type*, int)"
		int i = 0;
		if (maxNum < 0)
			maxNum = this->size();

		while(i < maxNum && !this->isEmpty())
			this->pop((Q_Type*)(dst + i++));

		return i;
	}
	//-----------------------------------------------------------------------
	/**
	 * @brief compute amount of available slots in queue
	 * @return number available slots
	 */
	int32_t freeSlots(void)
	{
		return int32_t(maxSize) - this->size();
	}
	//-----------------------------------------------------------------------
	/**
	 * @brief find position of first occurrence of given element
	 * @param data	element to be checked against
	 * @return position of given element, -1 if not present
	 */
	int32_t indexOf(Q_Type data)
	{
		int32_t s32Result = -1;
		uint8_t idx = 0;
		while(s32Result < 0 && idx < this->size())
		{
			if (this->at(idx) == data)
				s32Result = int32_t(idx);

			idx++;
		}

		return s32Result;
	}

protected:

	uint8_t u8TopIdx; /// top index ptr: it moved POPping queue
	uint8_t u8BottomIdx; /// bottom  index ptr: it moved PUSHing queue
	Q_Type queue[maxSize]; /// array with queued elements
};

#endif
