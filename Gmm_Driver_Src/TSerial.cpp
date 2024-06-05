/******************************************************************************
 *
 *  \brief		Serial Port Class
 *  \details	This class is used to configure and use the Serial ports on
 *  STM32F4 board
 *  \author	Ivan Perletti - General Medical Merate- GMM.spa - Seriate - ITALY
 *  \version	1.1a
 *  \date		January 15th, 2014
 *  \pre		Enable interrupt routine for each USART declared and used. Ex:
 * void USART3_IRQHandler(void) { usart3.Interrupt_Handler(&usart3); }
 * and add extern TSerial usart3; in the top .c file
 *  \bug		Not all memory is freed when deleting an object of this class.
 *  \warning	Improper use can crash your application
 *  \copyright GMM.spa - All Rights Reserved
 *
 ******************************************************************************/

#include "TSerial.h"
extern "C"{
#include "stm32f4xx_rcc.h"
//#include "stm32f4_discovery.h"
#include "stm32f4xx_gpio.h"

}
//----Private Struct-----------------------------------------------------------

//  USART		TX		RX
// --------------------------
//  USART1		PB6		PB7
//  USART2		PA2		PA3
//  USART3		PD8		PD9
/**
 * @brief struct where are listed all the USART parameter to work with
 * STM32F4 board
 */
usartSetup UART_PinOutStruct[]=
{
		{1, USART1, RCC_APB2Periph_USART1, RCC_AHB1Periph_GPIOB, GPIO_Pin_6 | GPIO_Pin_7  , GPIOB, GPIO_PinSource6  , GPIO_PinSource7 , GPIO_AF_USART1, USART1_IRQn},
		{2, USART2, RCC_APB1Periph_USART2, RCC_AHB1Periph_GPIOD, GPIO_Pin_5 | GPIO_Pin_6  , GPIOD, GPIO_PinSource5  , GPIO_PinSource6 , GPIO_AF_USART2, USART2_IRQn},
		{3, USART3, RCC_APB1Periph_USART3, RCC_AHB1Periph_GPIOD, GPIO_Pin_8 | GPIO_Pin_9  , GPIOD, GPIO_PinSource8  , GPIO_PinSource9 , GPIO_AF_USART3, USART3_IRQn},
		{4, UART4 , RCC_APB1Periph_UART4 , RCC_AHB1Periph_GPIOC, GPIO_Pin_10 | GPIO_Pin_11, GPIOC, GPIO_PinSource10 , GPIO_PinSource11, GPIO_AF_UART4 , UART4_IRQn},
		{5, UART5 , RCC_APB1Periph_UART5 , RCC_AHB1Periph_GPIOD, GPIO_Pin_2 | GPIO_Pin_12 , GPIOD, GPIO_PinSource2  , GPIO_PinSource12, GPIO_AF_UART5 , UART5_IRQn},
		{6, USART6, RCC_APB2Periph_USART6, RCC_AHB1Periph_GPIOC, GPIO_Pin_6 | GPIO_Pin_7  , GPIOC, GPIO_PinSource6  , GPIO_PinSource7 , GPIO_AF_USART6, USART6_IRQn},
};

#define UART_COHERENCE(PERIPH)	(	\
		((PERIPH) == USART1)	|| 	\
		((PERIPH) == USART2) 	|| 	\
		((PERIPH) == USART3) 	|| 	\
		((PERIPH) == UART4)		|| 	\
		((PERIPH) == UART5)		|| 	\
		((PERIPH) == USART6)	)

//- --------------------------------------------------------------------------
baudRate convert2Baud(int presumedBaud)
{
	switch (presumedBaud) {
	case 1200:
	case 2400:
	case 4800:
	case 9600:
	case 19200:
	case 38400:
	case 57600:
	case 115200:
		return((baudRate)presumedBaud);
	default:
		assert_param(0); // not a valid baudrate
	}
	return(Baud1200);
}
//- --------------------------------------------------------------------------
/**
 * input has to be 10 times the  Stop Bit Num:
 * - 05 = Half
 * - 10 = OneStop
 * - 15 = OneHalf
 * - 20 = TwoStop
 * @param presumedStpBit
 * @return Stop bit Mask bit (STM32F4 rule)
 */
stopBits convert2StopBit(int presumedStpBit)
{
	stopBits ii;
	switch (presumedStpBit){
	case 10:
	case OneStop:
		ii=OneStop; break;
	case 5:
	case HalfStop:
		ii=HalfStop; break;
	case 2:
	case TwoStop:
		ii=TwoStop; break;
	case 15:
	case OneAndHalfStop:
		ii=OneAndHalfStop; break;
	default:
		assert_param(0); // not a valid stopBit
	}
	return(ii);
}
//- --------------------------------------------------------------------------
/**
 * @brief Convert integer to Parity mask bit (STM32F4 rule)
 * @param presumedParity presumed parity value [0 1 2]
 * 			- 0 no parity
 * 			- 1 odd parity
 * 			- 2 even parity
 * @return parity mask bit (STM32F4 rule)
 */
parity convert2Parity(int presumedParity)
{
	parity ii;
	switch (presumedParity)	{
	case NoParity:
		ii=NoParity; break;
	case 1:
		ii=OddParity; break; // dispari
	case 2:
	case EvenParity:
		ii=EvenParity; break; // pari
	default:
		assert_param(0); // not a valid stopBit
	}
	return(ii);
}
//- --------------------------------------------------------------------------
/**
 * @brief convert to Flow control mask bit (STM32F4 rule)
 * @param presumedFlow value to be tested:
		-	0	No flow control.
		-	1	Software flow control RTS (XON/XOFF).
		-	2	Software flow control CTS (XON/XOFF).
		-	3 Hardware flow control (RTS/CTS).
 * @return mask bit corresponding to the flow command
 */
flowControl convert2Flow(int presumedFlow)
{
	flowControl ii;
	switch (presumedFlow)	{
	case NoFlowControl:
		ii = NoFlowControl; break;
	case 1:
	case SoftwareControl_RTS:
		ii = SoftwareControl_RTS; break;
	case 2:
	case SoftwareControl_CTS:
		ii = SoftwareControl_CTS; break;
	case 3:
	case HardwareControl:
		ii = HardwareControl; break;
	default:
		assert_param(0); // not a valid value
	}
	return(ii);
}
//- --------------------------------------------------------------------------
/**
 * @brief convert to USART Data mask bit (STM32F4 rule)
 * @param presumedDataBit
 * @return mask bit corresponding to the Data Bit chosen
 */
dataBits convert2DataBit(int presumedDataBit)
{
	dataBits ii;
	switch (presumedDataBit)	{
	case 8:
	case Data8:
		ii = Data8; break;
	case 9:
	case Data9:
		ii = Data9; break;
	default:
		assert_param(0); // not a valid value
	}
	return(ii);
}
//----Class Methods------------------------------------------------------------------------
/**
 * @brief Class constructor: declare and *basic* initialization of the USART
 * @param num		number of the port to initialize. See usartSetup struct for details
 */
TSerial::TSerial(int num):structPtr(0),
		stpIndex(0),
		usartID(num),
		USARTPORT(0),
		rx_head(0),
		rx_tail(0),
		tx_head(0),
		tx_tail(0)
{
	cleanAllLocalVariables();
	SearchUsartSetup(num);
	/*!< @remark use this function below to initialize COM */
	//	setupCOM(Baud19200, OneStop, NoParity, NoFlowControl, Data8);
	USART_InitStructure.USART_BaudRate = 0;
	USART_InitStructure.USART_WordLength = 0;
	USART_InitStructure.USART_StopBits = 0;
	USART_InitStructure.USART_Parity = 0;
	USART_InitStructure.USART_HardwareFlowControl = 0;
	USART_InitStructure.USART_Mode = 0;
}
//-----------------------------------------------------------------------------------------
/**
 * @brief Class Destructor: reset *basic* initialization of the class
 */
TSerial::~TSerial() {
	usartID 	= 0;
	stpIndex 	= 0;
	usartID 	= 0;
	USARTPORT 	= 0;
	cleanAllLocalVariables();
	USART_InitStructure.USART_BaudRate = 0;
	USART_InitStructure.USART_WordLength = 0;
	USART_InitStructure.USART_StopBits = 0;
	USART_InitStructure.USART_Parity = 0;
	USART_InitStructure.USART_HardwareFlowControl = 0;
	USART_InitStructure.USART_Mode = 0;
}
//-----------------------------------------------------------------------------------------
//##################################################			PRIVATE FUNCTIONS
/**
 * @brief Prepare the COM setup up with this parameters set
 * @param baudPipp			baud rate
 * @param stopNum			stop bits
 * @param parityBitNum		parity bit
 * @param flowCtrlChoice	flow control method
 * @param dataBitNum		data bit number
 * @def (Baud9600, OneStop, NoParity, NoFlowControl, Data8);
 */
void TSerial::setupCOM(
		baudRate baudPipp=Baud9600,
		stopBits stopNum = OneStop,
		parity parityBitNum = NoParity,
		flowControl flowCtrlChoice = NoFlowControl,
		dataBits dataBitNum = Data8)
{
	USART_InitStructure.USART_BaudRate 		= baudPipp;
	USART_InitStructure.USART_WordLength 	= dataBitNum;
	USART_InitStructure.USART_StopBits 		= stopNum;
	USART_InitStructure.USART_Parity 		= parityBitNum;
	USART_InitStructure.USART_HardwareFlowControl = flowCtrlChoice;
	USART_InitStructure.USART_Mode 			= USART_Mode_Rx | USART_Mode_Tx;
}
//-----------------------------------------------------------------------------------------
/**
 @brief Clean all the local variables\n
  unsigned char structPtr,  pointer to this class: need in the Interrupt routine
  char stpIndex, Usart struct setup index: my differ from usartID; (-1) if not found
  char usartID,  name of the USART# / UART#
  USART_TypeDef* USARTPORT,		USART address according to STM32F4 libraries
 *TO-READ parameters*
  - int rx_head, 1st  Not-Already *READ* char ptr in RxCircBuffer
  - int rx_tail, Last Not-Already *READ* char ptr in RxCircBuffer
  - volatile char bCircArrRx[RXBUFFERSIZE],	< circular buffer for RX msg
 *TO-SEND parameters*
  - int tx_head, 1st  Not-Already *SENT* char ptr in TxCircBuffer
  - int tx_tail, Last Not-Already *SENT* char ptr in TxCircBuffer
  - volatile char bCircArrTx[RXBUFFERSIZE];
 */
void  TSerial::cleanAllLocalVariables(void)
{
	structPtr=0;
	stpIndex=0;
	//	USARTPORT=0;
	rx_tail=0;
	rx_head=0;
	tx_tail=0;
	tx_head=0;
	cleanBuffer(bCircArrRx,RXBUFFERSIZE);
	cleanBuffer(bCircArrTx,RXBUFFERSIZE);
}
//------------------------------------------------------------------------------
/**
 * @brief Get the current Serial COM port number
 * @return serial COM number
 */
int TSerial::getComNum (void)
{ 
return usartID; 
};
//------------------------------------------------------------------------------
/**
 * @brief Search inside the UART_PinOutStruct if some setup corresponds to the wished USART,
 * then assigns class-variable stpIndex, usartID, and USARTPORT
 * @param numUART		number of the interested UART
 * @todo replace while instead of 
 */
void TSerial::SearchUsartSetup(int numUART)
{
	int structSize = sizeof(UART_PinOutStruct) / sizeof(UART_PinOutStruct[0]);
	int ii;
	for(ii=0;ii<structSize;ii++)
	{
		if(numUART == UART_PinOutStruct[ii].usartID)
		{
			stpIndex = ii;
			usartID = UART_PinOutStruct[stpIndex].usartID;
			USARTPORT = UART_PinOutStruct[stpIndex].USARTusart;
		}
	}
}
//-----------------------------------------------------------------------------------------
/**
 * @brief System Clock Configuration
 * @param usartID		number of the usart {1,2,3,4,5,6}
 * @retval Error number - see USART_Init() for details
 * @remarks Only USART1 and USART6 are connected to APB2 the other USARTs are
 * connected to APB1
 */
unsigned char  TSerial::RCC_Configuration(void)
{
	if (UART_PinOutStruct[stpIndex].RCC_APB_USART && UART_PinOutStruct[stpIndex].RCC_AHB1_GPIO )
	{
		switch(usartID)	{
		case 1:
		case 6:
			// enable APB2 peripheral clock for USART1 and USART6	*/
			// only USART1 and USART6 are connected to APB2 the other USARTs are connected to APB1
			RCC_APB2PeriphClockCmd(UART_PinOutStruct[stpIndex].RCC_APB_USART, ENABLE);
			/* GPIO clock enable */
			RCC_AHB1PeriphClockCmd(UART_PinOutStruct[stpIndex].RCC_AHB1_GPIO, ENABLE);
			break;
		case 5:
			RCC_APB1PeriphClockCmd(UART_PinOutStruct[stpIndex].RCC_APB_USART, ENABLE);
			/* GPIO clock enable */
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
			break;
		default :/* USART clock enable */
			RCC_APB1PeriphClockCmd(UART_PinOutStruct[stpIndex].RCC_APB_USART, ENABLE);
			/* GPIO clock enable */
			RCC_AHB1PeriphClockCmd(UART_PinOutStruct[stpIndex].RCC_AHB1_GPIO, ENABLE);
			break;
		}
		return (0x00);
	}
	else
		return(0x02);// error: not disposable RCC config
}
//-----------------------------------------------------------------------------------------
/**
 * @brief GPIO Configuration: prepares the pins to be used as RX TX
 * @retval Error number - 0x00 no error, 0x04 not valid usartID
 * @note UART5 RX & TX pins belong to 2 different GPIOx family. In the
 * procedure an if-clause will behave differently if UART5 is called. For this
 * reason the GPIOx value will be neglected
 */
unsigned char TSerial::GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	/*-------------------------- GPIO Configuration ----------------------------*/
	//  GPIO_InitStructure.GPIO_Pin =  ????  follow swithc statement
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	if (	UART_COHERENCE(USARTPORT)				&&
			UART_PinOutStruct[stpIndex].GPIO_Pin		&&
			UART_PinOutStruct[stpIndex].GPIOx			&&
			UART_PinOutStruct[stpIndex].GPIO_PinSrcA	&&
			UART_PinOutStruct[stpIndex].GPIO_PinSrcB	&&
			UART_PinOutStruct[stpIndex].GPIO_AF_USARTx  )
	{
		if (usartID==5)
		{
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
			GPIO_Init(GPIOD, &GPIO_InitStructure);
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
			GPIO_Init(GPIOC, &GPIO_InitStructure);
			// now all the values are passed to the GPIO_Init() function which sets the GPIO registers
			GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_UART5);
			GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);
			//The RX and TX pins are now connected to their AF so that the USART can take over control of the pins
			return(0x00);
		}	else
		{
			GPIO_InitStructure.GPIO_Pin = UART_PinOutStruct[stpIndex].GPIO_Pin;
			GPIO_Init(UART_PinOutStruct[stpIndex].GPIOx, &GPIO_InitStructure);
			// now all the values are passed to the GPIO_Init() function which sets the GPIO registers
			GPIO_PinAFConfig(UART_PinOutStruct[stpIndex].GPIOx, UART_PinOutStruct[stpIndex].GPIO_PinSrcA, UART_PinOutStruct[stpIndex].GPIO_AF_USARTx);
			GPIO_PinAFConfig(UART_PinOutStruct[stpIndex].GPIOx, UART_PinOutStruct[stpIndex].GPIO_PinSrcB, UART_PinOutStruct[stpIndex].GPIO_AF_USARTx);
			//The RX and TX pins are now connected to their AF so that the USART can take over control of the pins
			return(0x00);
		}
	}
	return(0x04);// there is no such a USART
}
//-----------------------------------------------------------------------------------------
/**
 * @brief USART structure configuration:
 *	USART configured as follow:
 *	- BaudRate					<b>baudrate</b>
 *	- Word Length				8 Bits
 *	- Stop Bit					Two
 *	- Parity					No
 *	- Hardware flow control		disabled (RTS and CTS signals)
 *	- Receive and transmit		enabled
 * @retval Error number - 0x00 no error, 0x08 not valid usartID
 * @done maybe add some other input such as "Stop Bit" number
 */
unsigned char TSerial::USART_StructConfig(void)
{
	if( UART_COHERENCE(USARTPORT) )
	{
		assert_param(USART_InitStructure.USART_BaudRate);// assert USART_InitStructure has valid fields
		/*! @note do not use other ASSERT for USART_InitStructure:
		 * parameters can be 0x00 */

		// void setupCOM(baudRate baudPipp, stopBits stopNum, parity parityBitNum,
		//				flowControl flowCtrlChoice, dataBits dataBitNum);

		USART_Init(USARTPORT, &USART_InitStructure);
		return(0x00);
	}
	return(0x08);// there is no such a USART
}
//-----------------------------------------------------------------------------------------
/**
 * @brief Configure the Nested Vectored Interrupt Controller (NVIC)
 * @retval Error number - see USART_Init
 */
unsigned char  TSerial::NVIC_StructConfig(void)
{
	if( UART_COHERENCE(USARTPORT) )
	{
		NVIC_InitTypeDef NVIC_InitStructure;
		/* Configure the NVIC Preemption Priority Bits */
		//						NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
		/*Prepare the NVIC Struct*/
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		/*Specialized configuration for NVIC*/
		NVIC_InitStructure.NVIC_IRQChannel = UART_PinOutStruct[stpIndex].IRQ_usart;
		NVIC_Init(&NVIC_InitStructure);// Assign the NVIC struct
		return(0x00);
	}
	return(0x10);// there is no such a USART
}

//-----------------------------------------------------------------------------------------
//##################################################			PUBLIC FUNCTIONS
//-----------------------------------------------------------------------------------------
/**
 * @brief USART Full Initialization
 * @param baudrate		UART speed
 * @retval Error code bits:
			- &=0x01 - invalid number: there is no such a USART configuration;
			- &=0x02 - RCC System Clocks Configuration error
			- &=0x04 - GPIO Configuration error
			- &=0x08 - USART_Struct Configuration error
			- &=0x10 - Nested VEctor Interrupt Routine error
 */
unsigned char  TSerial::setup_HW(void)
{
	unsigned char error = 0x00;

	assert_param(usartID);
	SearchUsartSetup(usartID);

	if(	UART_COHERENCE(USARTPORT) ) // if I enter here I am sure setup is ok. USARTPORT != 0
	{
		assert_param(UART_COHERENCE(USARTPORT));

		USART_DeInit(USARTPORT);
		error|=RCC_Configuration();/*!< System Clocks Configuration */
		error|=GPIO_Config();/*!< GPIO Configuration */
		error|=USART_StructConfig();/*!< stpIndex configuration */
		error|=NVIC_StructConfig();/*!< Nested VEctor Interrupt Routine*/
		// if I enter successfully here I am sure setup is ok. USARTPORT != 0

		return (error);
	}
	return (0x01);
}

//-----------------------------------------------------------------------------------------
/**
 * @brief Open the chosed port
 * @return Error code (see TSerial::setup_HW(void) )
 * @pre USE TSerial::setupCOM() before calling open() function: otherwise default setup is loaded
 * @remark  do not use other ASSERT for USART_InitStructure: parameters can be 0x00
 */
char TSerial::open(void)
{
	unsigned char error = 0;
	if( UART_COHERENCE(USARTPORT) )
	{
		USARTPORT = UART_PinOutStruct[stpIndex].USARTusart;
		assert_param(UART_COHERENCE(USARTPORT));

		error = setup_HW();
		if(USART_InitStructure.USART_BaudRate == 0)
		{
			setupCOM();// default setup if no explicit parameters declaration
		}
		/*! @note do not use other ASSERT for USART_InitStructure:
		 * other parameters can be 0x00 */
		USART_ITConfig(USARTPORT, USART_IT_RXNE, ENABLE);
		//	USART_ITConfig(USARTPORT, USART_IT_ERR, ENABLE);
		//	USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
		USART_Cmd(USARTPORT, ENABLE);
	}
	return(error);
}
//-----------------------------------------------------------------------------------------
/**
 * @brief Check if the port is open
 * @return port status:
 				- 0x00 if port is closed
  				- 0x01 if port is open
 */
char TSerial::isOpen (void)
{
	if (UART_COHERENCE(USARTPORT))
	{
		assert_param( UART_COHERENCE(USARTPORT));
		return(USARTPORT->CR1);
	}
	return(0x00);
}
//-----------------------------------------------------------------------------------------
/**
 * @brief Close the port without de-initializing it. All the RX / TX memory will be erased
 */
void TSerial::close (void)
{
	if (UART_COHERENCE(USARTPORT))
	{
		assert_param(IS_USART_ALL_PERIPH(USARTPORT));
		USART_ITConfig(USARTPORT, USART_IT_RXNE, DISABLE);
		USART_Cmd(USARTPORT, DISABLE);
		cleanBuffer();
	}
}

//-----------------------------------------------------------------------------------------
/**
 * @brief Transmit a string of characters via the USART specified in stpIndex.
 * @param stpIndex	Can be any of the USART# ( USART1, USART2 etc. )
 * @param *s		is the string you want to send
 *
 * @note The string has to be passed to the function as a pointer because
 *		the compiler doesn't know the 'string' data type. In standard
 *		C a string is just an array of characters
 *
 * @note At the moment it takes a volatile char because the received_string variable
 * declared as volatile char --> otherwise the compiler will spit out warnings
 * */
void TSerial::USART_puts(USART_TypeDef* stpIndex, const volatile char *s)
{
	if( UART_COHERENCE(USARTPORT) )
	{
		USART_ITConfig(USARTPORT, USART_IT_TXE, DISABLE);
		while(*s){
			// wait until data register is empty
			while( !(USARTPORT->SR & 0x00000040) );
			USART_SendData(USARTPORT, (uint16_t)*s);
			s++;
		}
		USART_ITConfig(USARTPORT, USART_IT_TXE, ENABLE);
	}
}

//-----------------------------------------------------------------------------------------
/**
 * @brief Wipe the input buffer data replacing the first numChar elements with
 *  "-" character. No way to know the real dimension of the buffer: you risk to
 *  modify the memory next to the buffer if numChar is greater than the real
 *  buffer dimension
 * @param *buffer 		ptr to the array to wipe
 * @param numChar		num of char to replace [use sizeof(buffer) / sizeof(char]]
 */
void TSerial::cleanBuffer(volatile char *buffer, const unsigned short int numChar)
{
	unsigned int bb;
	for(bb=0; bb<numChar; bb++)
		buffer[bb]='-';
}
//-----------------------------------------------------------------------------------------
/**
 * @brief Wipe the private bCircArrRx data replacing elements with "*" char
 */
void TSerial::cleanBuffer(void)
{
	unsigned int bb;
	//	USART_ITConfig(USARTPORT, USART_IT_TXE, DISABLE);
	for(bb=0; bb<RXBUFFERSIZE; bb++){
		bCircArrRx[bb]='(';
		bCircArrTx[bb]='(';
	}
	tx_tail=0;
	tx_head=0;
	rx_tail=0;
	rx_head=0;
	//	USART_ITConfig(USARTPORT, USART_IT_TXE, ENABLE);
}
//-----------------------------------------------------------------------------------------
/**
 * @brief Read eventual Usart new message: new message is the right circular
 * sub-buffer between rx_head and rx_tail head ptrs
 * @param msgRx		buffer into which copy new received data read from circular buffer
 * @retval number of read char
 * - if > 0 is the number of char,
 * - (-1) error, RX Cable seems unplugged
 * @note While reading interrupt is allowed to modify RX_CircBuff
 * warranty for no critical in this time
 * @note Avoided to pause RX interrupt for incomplete message coming
 */
int TSerial::read(volatile char *msgRx)
{
	int qq=0;

//#ifdef _USB_DEBUG_
//	tDbg.write((char*)"READ rx_head ");
//	tDbg.write((char*)"%d  ", rx_head);
//	tDbg.write((char*)"READ rx_tail ");
//	tDbg.write((char*)"%d  ", rx_tail);
//#endif

	if(rx_head!=rx_tail)
	{ //  new data to read on USART
		while( rx_head!=rx_tail)
		{
			msgRx[qq]=bCircArrRx[rx_head];// if interrupt happens here NO PROBLEM
			rx_head++;// if interrupt happens here will be %RXBUFFERSIZE
			rx_head%=RXBUFFERSIZE;
			qq++;
			qq%=RXBUFFERSIZE;
		}
	}
	return(qq);//number of char it reads
}

//-----------------------------------------------------------------------------------------
/**
 * @brief Check if there is anything to read from Receiving buffer
 * @return the number of character to be read
 */
int TSerial::bytesAvailable(void)
{
	int notAlreadyReadChar = rx_tail - rx_head;
	if(notAlreadyReadChar<0)
		notAlreadyReadChar+=RXBUFFERSIZE;
	return(notAlreadyReadChar);
}
//-----------------------------------------------------------------------------------------
/**
 * @brief Check if there is anything to be transmitted on the serial port
 * @return number of characters to be read
 */
int TSerial::bytesToWrite(void)
{
	int notAlreadyWrittenChar = tx_tail - tx_head;
	if(notAlreadyWrittenChar<0)
		notAlreadyWrittenChar+=RXBUFFERSIZE;
	return(notAlreadyWrittenChar);
}
//-----------------------------------------------------------------------------------------
//									/**
//									 *  @brief  Writes on the serial port a msg until \0 or max length of msg
//									 *  it is a high priority write which pause current transmission
//									 *  @retval  error:
//									 *				&=0x00 no error
//									 *				&=0x01 some data not sent, stpIndex->SR data register NOT empty
//									 *				&=0x02 max MSG length reached
//									 * @bug Not tested in parallel with write()
//									 * */
//									unsigned char TSerial::writePort(const volatile char *msg) {
//
//										unsigned short int counter = 0x0001, ii = 0;
//										unsigned char error = 0x00;
//
//										if(USARTPORT)// if initialized, it is different from NULL == 0
//										{
//											while (msg[ii] && ii < RXBUFFERSIZE) {
//												// wait until data register is empty
//												while (!(USARTPORT->SR & 0x00000040) && counter)
//													counter++; // STOP if overflows since it equals 0x0000 -> error
//
//												if (counter) {  // counter not overflowing
//													USART_SendData(USARTPORT, msg[ii++]);
//													//		tx_tail++; tx_tail%=RXBUFFERSIZE;
//													error |= 0x00; // sure for char sent - no error
//												}
//												else {
//													error |= 0x01; // or operator: some char not sent
//													break; // non need to go over if some data not sent
//												}
//												counter = 0x01; // reset counter for the next loop
//											}
//											if (ii >= RXBUFFERSIZE)
//												error |= 0x02; // error: max MSG length reached
//										}
//										return (error);
//									}
//								//-----------------------------------------------------------------------------------------
//								/**
//								 * @brief Writes on the serial port a msg of fixed length
//								 * @param msg 		ptr to the message to send
//								 * @param charNum2Send 		number of char to send, it is a high priority function
//								 * which pause current transmission and leave no signs in bCircArrTx
//								 *  @retval  Error:
//								 *				&=0x00 no error
//								 *				&=0x01 some data not sent, stpIndex->SR data register NOT empty
//								 *				&=0x02 max MSG length reached, some data not sent
//								 *
//								 * */
//								unsigned char TSerial::writePort(const volatile char *msg,
//										unsigned short int charNum2Send)
//								{
//									unsigned short int counter = 0x0001, ii = 0;
//									unsigned char error = 0x00;
//									charNum2Send = (RXBUFFERSIZE<charNum2Send)?RXBUFFERSIZE:charNum2Send; // optimized min(a,b)
//									if(USARTPORT)// if initialized, it is different from NULL == 0
//									{
//										USART_ITConfig(USARTPORT, USART_IT_TXE, DISABLE);// pause TX interrupt @@@
//										while (ii < charNum2Send)
//										{
//											// wait until data register is empty
//											while (!(USARTPORT->SR & 0x00000040) && counter)
//												counter++; // STOP if overflows since it equals 0x0000 -> error
//
//											if (counter) {  // counter not overflowing
//												USART_SendData(USARTPORT, msg[ii++]);
//												//		tx_tail++; tx_tail%=RXBUFFERSIZE;
//												error |= 0x00; // sure for char sent - no error
//											}
//											else {
//												error |= 0x01; // or operator: some char not sent
//												break; // non need to go over if some data not sent
//											}
//											counter = 0x01; // reset counter for the next loop
//										}
//										USART_ITConfig(USARTPORT, USART_IT_TXE, ENABLE);// resume TX interrupt @@@
//										if (ii >= RXBUFFERSIZE)
//											error |= 0x02; // error: max MSG length reached
//									}
//									return (error);
//								}
//-----------------------------------------------------------------------------------------
/**
 * @param msg 		ptr to the message array you want to transmit
 * @param charNum2Send  ptr to the number of char to effectively transmit
 * @return error
				- |=0x01 - message completely sent
				- &=0x01 - Port Not correctly initialized
				- &=0x02 - Too much character to send: stay < RXBUFFERSIZE
 */
unsigned char TSerial::write(const volatile char *msg,
		short int charNum2Send)
{
	unsigned short int  qq=0;
	int delta;

	if(USARTPORT)// if initialized, it is different from NULL == 0
	{

		assert_param(IS_USART_ALL_PERIPH(USARTPORT));
		if (charNum2Send>0 && charNum2Send<=RXBUFFERSIZE)
		{
			USART_ITConfig(USARTPORT, USART_IT_TXE, DISABLE);
			//	here tx_tail and tx_head are sticked: bCircArrTx too !!!
			delta = (tx_head - tx_tail ) ;// Estimate how many free slots in the TX Queue
			if(delta<=0)
				delta+=RXBUFFERSIZE;
			if(delta>3 && delta>charNum2Send) {
				while (qq<charNum2Send)
				{
					bCircArrTx[tx_tail] = msg[qq];// overcome rx-buffer circularity
					tx_tail++;
					tx_tail%=RXBUFFERSIZE;
					qq++;
					// qq%=RXBUFFERSIZE;
				}
				USART_ITConfig(USARTPORT, USART_IT_TXE, ENABLE);
				return (0x00);// message completely sent
			}
			USART_ITConfig(USARTPORT, USART_IT_TXE, ENABLE);
		}
		return (0x02);// Too much character to send: stay < RXBUFFERSIZE
	}
	return (0x01);// Port Not correctly initialized
}

//-----------------------------------------------------------------------------------------
/**
 * @brief append a msg onto circual array. Instead of TSerial::write() this
 * function does not ENABLE Interrupt USART_IT_TXE
 * @param msg	ptr to the message array you want to transmit
 * @param charNum2Send	ptr to the number of char to effectively transmit
 * @return error
				- |=0x01 - message completely sent
				- &=0x01 - Port Not correctly initialized
				- &=0x02 - Too much character to send: stay < RXBUFFERSIZE
 */
unsigned char TSerial::append(const volatile char *msg,
		unsigned short int charNum2Send)
{
	unsigned short int  qq=0;
	int delta;

	if(USARTPORT)// if initialized, it is different from NULL == 0
	{
		assert_param(IS_USART_ALL_PERIPH(USARTPORT));
		if (charNum2Send<=RXBUFFERSIZE)
		{
			USART_ITConfig(USARTPORT, USART_IT_TXE, DISABLE);
			//	here tx_tail and tx_head are sticked: bCircArrTx too !!!
			delta = (tx_head - tx_tail ) ;// Estimate how many free slots in the TX Queue
			if(delta<=0)
				delta+=RXBUFFERSIZE;
			if(delta>3 && delta>charNum2Send) {
				while (qq<charNum2Send)
				{
					bCircArrTx[tx_tail] = msg[qq];// overcome rx-buffer circularity
					tx_tail++;
					tx_tail%=RXBUFFERSIZE;
					qq++;
					// qq%=RXBUFFERSIZE;
				}
				//				USART_ITConfig(USARTPORT, USART_IT_TXE, ENABLE); // need release TXE interrupt
				return (0x00);// message completely sent
			}
			//			USART_ITConfig(USARTPORT, USART_IT_TXE, ENABLE);
		}
		return (0x02);// Too much character to send: stay < RXBUFFERSIZE
	}
	return (0x01);// Port Not correctly initialized
}
//-----------------------------------------------------------------------------
/**
 * @brief Method to block all the Serial Interrupt routines.
 */
 void TSerial::pauseIT_TX (void)
{
	USART_ITConfig(USARTPORT, USART_IT_TXE, DISABLE);
}
//-----------------------------------------------------------------------------
/**
 * @brief Method to continue all the Serial Interrupt routines.
 */
 void TSerial::releaseIT_TX (void)
{
	USART_ITConfig(USARTPORT, USART_IT_TXE, ENABLE);
}

