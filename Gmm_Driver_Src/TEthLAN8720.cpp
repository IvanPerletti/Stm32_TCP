#include "TEthLAN8720.h"

TEthLAN8720 ethLAN8720;

long TEthLAN8720::IdUserEthLAN8720 = 0;

//------------------------------------------------------------------------------
TEthLAN8720::TEthLAN8720(void):TEth ()
{
}
//------------------------------------------------------------------------------
TEthLAN8720::~TEthLAN8720() {
}
//------------------------------------------------------------------------------
/**
 * @brief Create subscriber pointer to specific Serial232 port
 * and assign unique identifier (instance ID) for the same serial physical port
 * @param pSerial232		pointer to serial232
 * @param serial_id			number of UART/USART port
 * @return s32Result	    instance ID
 */
long TEthLAN8720::getInstance(TEthLAN8720* &pEthLAN8720)
{
	long s32Result = -1;

	if ((IdUserEthLAN8720 + 1) < MAX_ETHLAN8720_NUM)
	{
		pEthLAN8720 = &ethLAN8720;

		s32Result = IdUserEthLAN8720;
		IdUserEthLAN8720++;
	}
	else
	{
		pEthLAN8720 = NULL;
	}

	return s32Result;
}

//------------------------------------------------------------------------------
/**@brief Close specific instance (last user physically close port)
 * @param qID	instance identifier
 */
void TEthLAN8720::closeInstance(long qID)
{
	if (qOpened.indexOf(qID) >= 0)
		qOpened.remove(qOpened.indexOf(qID));

	if (qOpened.size() == 0)
	{
		close();
	}
}
//-----------------------------------------------------------------------------
/**
 * @brief Reset the queue of all the opened users
 */

void TEthLAN8720::closeAllUsers(void)
{
	qOpened.reset();
}
//------------------------------------------------------------------------------
/**@brief Open specific instance
 * @param qID	instance identifier
 */
char TEthLAN8720::openInstance(long qID)
{
	if (qID >= 0 && qID < MAX_ETHLAN8720_NUM)
		{
			if (qOpened.indexOf(qID) < 0)
			{
				qOpened.push(qID);
				close();
				open();
			}
		}
}

//------------------------------------------------------------------------------
/**@brief Check if specific instance is open
 * @param qID	instance identifier
 */
char TEthLAN8720::isInstance_open(long qID)
{
	return (isOpen() && (qOpened.indexOf(qID) >= 0));
}
