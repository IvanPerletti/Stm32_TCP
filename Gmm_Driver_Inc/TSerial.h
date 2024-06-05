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


#ifndef TSERIAL_H_
#define TSERIAL_H_

#include <stdio.h>

#include "stm32f4xx.h"
#include "misc.h"			// I recommend you have a look at these in the ST firmware folder
#include "stm32f4xx_usart.h" // under Libraries/STM32F4xx_StdPeriph_Driver/inc and src
//#include "stm32f4_discovery.h"

/* Private define ------------------------------------------------------------*/
#define RXBUFFERSIZE 128 // CHOOSE A POWER OF TWO NUMBER (speed for modulus)
#define MAX_SERIAL_CALLBACKS 10
#if RXBUFFERSIZE & (RXBUFFERSIZE-1) != 0
#define RXBUFFERSIZE 128 // CHOOSE A POWER OF TWO NUMBER (optimized modulus function)
#endif



//----Private Struct-----------------------------------------------------------
/**
 * @brief struct: here are listed all the USART parameters to work with
 * STM32F4 board
 * @note UART5 RX & TX pins belong to 2 different GPIOx family. The setup
 * procedure will behave differently if UART5 is called. For this
 * reason the GPIOx value will be neglected
 */
typedef struct  {
	int usartID;/*!< USART name number among: 1,2,3,4,5,6*/
	USART_TypeDef* USARTusart;/*! like:  USART1*/
	unsigned int RCC_APB_USART;/*! like: RCC_APB2Periph_USART1*/
	unsigned int RCC_AHB1_GPIO;/*! like: RCC_AHB1Periph_GPIOC*/
	unsigned short int GPIO_Pin;/*! like:  GPIO_Pin_6 | GPIO_Pin_7*/
	GPIO_TypeDef * GPIOx;/*! like: GPIOD*/
	unsigned char GPIO_PinSrcA;/*! like: GPIO_PinSource6*/
	unsigned char GPIO_PinSrcB;/*! like: GPIO_PinSource7*/
	unsigned char GPIO_AF_USARTx;/*! like:  GPIO_AF_USART1*/
	unsigned char IRQ_usart;/*! like: USART1_IRQn*/
} usartSetup;

enum baudRate	{
	Baud1200 	= 1200,
	Baud2400 	= 2400,
	Baud4800 	= 4800,
	Baud9600 	= 9600,
	Baud19200 	= 19200,
	Baud38400 	= 38400,
	Baud57600 	= 57600,
	Baud115200 	= 115200,
};

//--------------------------------------------------------------------------------
class TVirtual_Serial; // Forward declaration

// Struct to hold the instance and member function pointer
struct SerialCallbackData
{
	void (TVirtual_Serial::*memberFunc)(char *payload);
	TVirtual_Serial* instance;
};
//...............................................................................
//				QSerialPort::OneStop		1	1 stop bit.
//				QSerialPort::OneAndHalfStop	3	1.5 stop bits.
//				QSerialPort::TwoStop		2	2 stop bits.
//#define USART_StopBits_1						((uint16_t)0x0000)
//#define USART_StopBits_0_5					((uint16_t)0x1000)
//#define USART_StopBits_2						((uint16_t)0x2000)
//#define USART_StopBits_1_5					((uint16_t)0x3000)
enum stopBits {
	OneStop			= 0x0000,
	HalfStop		= 0x1000,
	TwoStop			= 0x2000,
	OneAndHalfStop	= 0x3000,
};

//...............................................................................
//Constant	Value	Description
//QSerialPort::NoParity	0	No parity.
//QSerialPort::EvenParity	2	Even parity.
//QSerialPort::OddParity	3	Odd parity.

//#define USART_Parity_No						((uint16_t)0x0000)
//#define USART_Parity_Even					((uint16_t)0x0400)
//#define USART_Parity_Odd						((uint16_t)0x0600)
enum parity {
	NoParity 	= 0x0000,
	EvenParity 	= 0x0400,
	OddParity	= 0x0600, // dispary
};

//...............................................................................
//#define USART_HardwareFlowControl_None		((uint16_t)0x0000)
//#define USART_HardwareFlowControl_RTS		((uint16_t)0x0100)
//#define USART_HardwareFlowControl_CTS		((uint16_t)0x0200)
//#define USART_HardwareFlowControl_RTS_CTS	((uint16_t)0x0300)
enum flowControl{
	NoFlowControl 		= 0x0000, // QSerialPort::NoFlowControl		0	No flow control.
	SoftwareControl_RTS = 0x0100, // QSerialPort::SoftwareControl	2	Software flow control (XON/XOFF).
	SoftwareControl_CTS = 0x0200, // QSerialPort::SoftwareControl	2	Software flow control (XON/XOFF).
	HardwareControl 	= 0x0300, // QSerialPort::HardwareControl	1	Hardware flow control (RTS/CTS).
};

//...............................................................................
//			QSerialPort::Data5	5	Five bits.
//			QSerialPort::Data6	6	Six bits.
//			QSerialPort::Data7	7	Seven bits
//			QSerialPort::Data8	8	Eight bits.
//			QSerialPort::UnknownDataBits	-1	Unknown number of bits.
//#define USART_WordLength_8b					((uint16_t)0x0000)
//#define USART_WordLength_9b					((uint16_t)0x1000)
enum dataBits{
	Data8 = 0x0000,
	Data9 = 0x1000,
};

//...............................................................................
#define USART_STRUCT_OK(USART_InitStructure)  ((USART_InitStructure.USART_BaudRate) && \
		(USART_InitStructure.USART_WordLength) && \
		(USART_InitStructure.USART_StopBits) && \
		(USART_InitStructure.USART_Parity) && \
		(USART_InitStructure.USART_HardwareFlowControl) && \
		(USART_InitStructure.USART_Mode &&))


//----Classes--------------------------------------------------------
/**
 * @brief Serial port class: automatically configures the serial port by
 * declaring it. For ex to enable USART3 type :\n TSerial usart3(3)
 * @note remember to de-comment interrupt routine for each USART enabled. Ex:
 * void USART3_IRQHandler(void) { usart3.Interrupt_Handler(&usart3); }
 */
class TSerial {
public:
	TSerial(int num);
	virtual ~TSerial();
	SerialCallbackData callbacks[MAX_SERIAL_CALLBACKS];


protected:
	unsigned char structPtr;/*!< Pointer to this class: needed in the Interrupt routine */
	int stpIndex; /*!< UART-struct-setup index; (-1) if not found */
	int usartID; /*!< USART name number among: 1,2,3,4,5,6*/
	USART_TypeDef* USARTPORT;/*!< TSerial USART address according to STM32F4 libraries*/

	int rx_head; /*!< 1st  Not-Already *READ* char ptr in RxCircBuuffer*/
	int rx_tail; /*!< Last Not-Already *READ* char ptr in RxCircBuuffer*/

	int tx_head; /*!< 1st  Not-Already *SENT* char ptr in TxCircBuuffer*/
	int tx_tail; /*!< Last Not-Already *SENT* char ptr in TxCircBuuffer*/

	volatile char bCircArrRx[RXBUFFERSIZE];/*!< circular buffer for RX msg*/
	volatile char bCircArrTx[RXBUFFERSIZE];/*!< circular buffer for TX msg*/
	USART_InitTypeDef USART_InitStructure;/*!< TSerial USART Init Struct*/
protected://#####################################################################

	void SearchUsartSetup(int numUART);

	unsigned char RCC_Configuration(void);

	unsigned char GPIO_Config(void);

	unsigned char USART_StructConfig(void);

	unsigned char NVIC_StructConfig(void);

	//	char bufferIsFull(void); /*! @todo to implement if needed*/

	unsigned char setup_HW(void);

public://#####################################################################

	void setupCOM(baudRate baudPipp, stopBits stopNum, parity parityBitNum,
			flowControl flowCtrlChoice, dataBits dataBitNum);

	void  cleanAllLocalVariables(void);

	int getComNum (void);

	virtual char open(void);

	char isOpen (void);

	void close (void);

	void USART_puts(USART_TypeDef* stpIndex, const volatile char *s);

	void cleanBuffer(volatile char *buffer, const unsigned short int numChar);

	void cleanBuffer(void);

	int read(volatile char *msgRx);

	int bytesAvailable(void);

	int bytesToWrite(void);

	unsigned char write(const volatile char *msg,
			short int numOfchar2Send);

	unsigned char append(const volatile char *msg,
			unsigned short int charNum2Send);

	void pauseIT_TX (void);

	void releaseIT_TX (void);

	//---------------------------------------------------------------------------------
	/**
	 * @brief registerRxCallback register call back
	 * @param instance: pointer to TVirtual_Serial instance ("this")
	 * @param memberFunc: TVirtual_Serial function pointer
	 */
	void registerRxCallback(TVirtual_Serial* instance, void (TVirtual_Serial::*memberFunc)(char *payload))
	{
		for (int32_t i = 0; i < MAX_SERIAL_CALLBACKS; ++i)
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
	 * @brief trampoline method to be invoked by class on interrupt occurrence (see rxIRQ() inside TSerial232::Interrupt_Handler)
	 * @param message payload
	 * @param cbData struct that holds instance and pointer to function
	 */
	static void trampoline(char* payload, SerialCallbackData cbData)
	{
		if (cbData.instance!=NULL)
		{
			(cbData.instance->*cbData.memberFunc)(payload);
		}
	}
};


// Extern Fun --------------------------------------------------------------
extern baudRate convert2Baud(int presumedBaud);
extern stopBits convert2StopBit(int presumedStpBit);
extern parity convert2Parity(int presumedParity);
extern flowControl convert2Flow(int presumedFlow);
extern dataBits convert2DataBit(int presumedDataBit);

extern "C" {
void USART1_IRQHandler(void);

void UART4_IRQHandler(void);

void EXTI0_IRQHandler(void);
}
#endif /* TSERIAL_H_ */
