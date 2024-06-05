/******************************************************************************
 *
 * @brief		Serial Port Class
 * @file 		TSerial.h
 * @details		This class is used to configure and use the Serial ports on
 * STM32F4 board
 * @author		Ronchi - Perletti \n
 *  General Medical Merate - GMM.spa - Seriate - ITALY
 * @version		1.0
 * @date		January 24th, 2014
 * @pre			Enable interrupt routine for each USART declared and used. Ex:
 * void USART3_IRQHandler(void) { usart3.Interrupt_Handler(&usart3); }
 * and add extern TSerial usart3; in the top .c file
 * @bug			Not all memory is freed when deleting an object of this class.
 * @warning		Improper use can crash your application
 * @copyright 	GMM.spa - All Rights Reserved
 *
 ******************************************************************************/


#ifndef TETH_H_
#define TETH_H_

#include <stdio.h>

#include "stm32f4xx.h"
#include "misc.h"			// I recommend you have a look at these in the ST firmware folder
#include "stm32f4x7_eth.h" // under Libraries/STM32F4xx_StdPeriph_Driver/inc and src
//#include "stm32f4_discovery.h"
#include "netif/etharp.h"

/* Private define ------------------------------------------------------------*/
#define RXBUFFERSIZE 128 // CHOOSE A POWER OF TWO NUMBER (speed for modulus)
#define MAX_ETH_CALLBACKS 10
#if RXBUFFERSIZE & (RXBUFFERSIZE-1) != 0
#define RXBUFFERSIZE 128 // CHOOSE A POWER OF TWO NUMBER (optimized modulus function)
#endif


//----Private Struct-----------------------------------------------------------

//--------------------------------------------------------------------------------
class TVirtual_Eth; // Forward declaration

// Struct to hold the instance and member function pointer
struct EthCallbackData
{
	void (TVirtual_Eth::*memberFunc)(char *payload);
	TVirtual_Eth* instance;
};

#define ETH_PHY_ADDRESS       ((uint16_t) 0x01) 

/* Ethernet Flags for EthStatus variable */   
#define ETH_INIT_FLAG           0x01 /* Ethernet Init Flag */
#define ETH_LINK_FLAG           0x10 /* Ethernet Link Flag */

/* MII and RMII mode selection, for STM324xG-EVAL Board(MB786) RevB ***********/
#define RMII_MODE  // User have to provide the 50 MHz clock by soldering a 50 MHz
                     // oscillator (ref SM7745HEV-50.0M or equivalent) on the U3
                     // footprint located under CN3 and also removing jumper on JP5. 
                     // This oscillator is not provided with the board. 
                     // For more details, please refer to STM3240G-EVAL evaluation
                     // board User manual (UM1461).

                                     
//#define MII_MODE

//----Classes--------------------------------------------------------
/**
 * @brief Serial port class: automatically configures the serial port by
 * declaring it. For ex to enable USART3 type :\n TEth usart3(3)
 * @note remember to de-comment interrupt routine for each USART enabled. Ex:
 * void USART3_IRQHandler(void) { usart3.Interrupt_Handler(&usart3); }
 */
class TEth {
public:
	TEth();
	virtual ~TEth();
	EthCallbackData callbacks[MAX_ETH_CALLBACKS];

protected:
	int rx_head; /*!< 1st  Not-Already *READ* char ptr in RxCircBuuffer*/
	int rx_tail; /*!< Last Not-Already *READ* char ptr in RxCircBuuffer*/

	int tx_head; /*!< 1st  Not-Already *SENT* char ptr in TxCircBuuffer*/
	int tx_tail; /*!< Last Not-Already *SENT* char ptr in TxCircBuuffer*/

	volatile char bCircArrRx[RXBUFFERSIZE];/*!< circular buffer for RX msg*/
	volatile char bCircArrTx[RXBUFFERSIZE];/*!< circular buffer for TX msg*/
	ETH_InitTypeDef ETH_InitStructure;
	__IO uint32_t  EthStatus;
	struct netif gnetif;
protected://#####################################################################

	unsigned char RCC_Configuration(void);

	unsigned char GPIO_Config(void);

	unsigned char	ETH_StructConfig(void);

	unsigned char NVIC_StructConfig(void);

	//	char bufferIsFull(void); /*! @todo to implement if needed*/

	unsigned char setup_HW(void);

public://#####################################################################

	void  cleanAllLocalVariables(void);

	virtual char open(void);

	char isOpen (void);

	void close (void);

	void ETH_puts(const volatile char *s);

	void cleanBuffer(volatile char *buffer, const unsigned short int numChar);

	void cleanBuffer(void);

	int read(volatile char *msgRx);

	int bytesAvailable(void);

	int bytesToWrite(void);

	unsigned char write(const volatile char *msg,
			short int numOfchar2Send);

	unsigned char append(const volatile char *msg,
			unsigned short int charNum2Send);

	//---------------------------------------------------------------------------------
	/**
	 * @brief registerRxCallback register call back
	 * @param instance: pointer to TVirtual_Serial instance ("this")
	 * @param memberFunc: TVirtual_Serial function pointer
	 */
	void registerRxCallback(TVirtual_Eth* instance, void (TVirtual_Eth::*memberFunc)(char *payload))
	{
		for (int32_t i = 0; i < MAX_ETH_CALLBACKS; ++i)
		{
			if (callbacks[i].instance == NULL)
			{
				callbacks[i].memberFunc = memberFunc;
				callbacks[i].instance = instance;
				break;
			}
		}
		// Handle the case where the array is full
	}

	//---------------------------------------------------------------------------------
	/**
	 * @brief trampoline method to be invoked by class on interrupt occurrence (see rxIRQ() inside TEth232::Interrupt_Handler)
	 * @param message payload
	 * @param cbData struct that holds instance and pointer to function
	 */
	static void trampoline(char* payload, EthCallbackData cbData)
	{
		if (cbData.instance!=NULL)
		{
			(cbData.instance->*cbData.memberFunc)(payload);
		}
	}
	
	friend void ETH_link_callback(struct netif *netif);

};

#endif /* TETH_H_ */