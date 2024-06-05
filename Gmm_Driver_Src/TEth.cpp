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

#include "TEth.h"
extern "C"{
#include "stm32f4xx_rcc.h"
//#include "stm32f4_discovery.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4x7_eth.h"
#include "lwip/mem.h"
#include "lwip/memp.h"
#include "lwip/tcp.h"
#include "lwip/tcp_impl.h"
#include "lwip/udp.h"
#include "netif/etharp.h"
#include "ethernetif.h"
}
//----Private Struct-----------------------------------------------------------

//void ETH_link_callback(struct netif *netif)
//{
//  __IO uint32_t timeout = 0;
// uint32_t tmpreg;
// uint16_t RegValue;
//  struct ip_addr ipaddr;
//  struct ip_addr netmask;
//  struct ip_addr gw;
//  uint8_t iptab[4] = {0};
//  uint8_t iptxt[20];

//  if(netif_is_link_up(netif))
//  {
//    /* Restart the auto-negotiation */
//    if(ETH_InitStructure.ETH_AutoNegotiation != ETH_AutoNegotiation_Disable)
//    {
//      /* Reset Timeout counter */
//      timeout = 0;

//      /* Enable auto-negotiation */
//      ETH_WritePHYRegister(ETH_PHY_ADDRESS, PHY_BCR, PHY_AutoNegotiation);

//      /* Wait until the auto-negotiation will be completed */
//      do
//      {
//        timeout++;
//      } while (!(ETH_ReadPHYRegister(ETH_PHY_ADDRESS, PHY_BSR) & PHY_AutoNego_Complete) && (timeout < (uint32_t)PHY_READ_TO));  

//      /* Reset Timeout counter */
//      timeout = 0;

//      /* Read the result of the auto-negotiation */
//      RegValue = ETH_ReadPHYRegister(ETH_PHY_ADDRESS, PHY_SR);

//      /* Configure the MAC with the Duplex Mode fixed by the auto-negotiation process */
//			/* Mask out bits which are not for speed and link indication, bits 4:2 are used */
//			RegValue = (RegValue >> 2) & 0x07;

//			/* Switch statement */
//			switch (RegValue) {
//				case 1: /* Base 10, half-duplex */
//					ETH_InitStructure.ETH_Speed = ETH_Speed_10M;
//					ETH_InitStructure.ETH_Mode = ETH_Mode_HalfDuplex;
//					break;
//				case 2: /* Base 100, half-duplex */
//					ETH_InitStructure.ETH_Speed = ETH_Speed_100M;
//					ETH_InitStructure.ETH_Mode = ETH_Mode_HalfDuplex;
//					break;
//				case 5: /* Base 10, full-duplex */
//					ETH_InitStructure.ETH_Speed = ETH_Speed_10M;
//					ETH_InitStructure.ETH_Mode = ETH_Mode_FullDuplex;
//					break;
//				case 6: /* Base 100, full-duplex */
//					ETH_InitStructure.ETH_Speed = ETH_Speed_100M;
//					ETH_InitStructure.ETH_Mode = ETH_Mode_FullDuplex;
//					break;
//				default:
//					break;
//			}

//      /*------------------------ ETHERNET MACCR Re-Configuration --------------------*/
//      /* Get the ETHERNET MACCR value */  
//      tmpreg = ETH->MACCR;

//      /* Set the FES bit according to ETH_Speed value */ 
//      /* Set the DM bit according to ETH_Mode value */ 
//      tmpreg |= (uint32_t)(ETH_InitStructure.ETH_Speed | ETH_InitStructure.ETH_Mode);

//      /* Write to ETHERNET MACCR */
//      ETH->MACCR = (uint32_t)tmpreg;

//      _eth_delay_(ETH_REG_WRITE_DELAY);
//      tmpreg = ETH->MACCR;
//      ETH->MACCR = tmpreg;
//    }

//    /* Restart MAC interface */
//    ETH_Start();

//    IP4_ADDR(&ipaddr, IP_ADDR0, IP_ADDR1, IP_ADDR2, IP_ADDR3);
//    IP4_ADDR(&netmask, NETMASK_ADDR0, NETMASK_ADDR1 , NETMASK_ADDR2, NETMASK_ADDR3);
//    IP4_ADDR(&gw, GW_ADDR0, GW_ADDR1, GW_ADDR2, GW_ADDR3);

//    netif_set_addr(&gnetif, &ipaddr , &netmask, &gw);
//    
//    /* When the netif is fully configured this function must be called.*/
//    netif_set_up(&gnetif);    

//		EthLinkStatus = 0;
//  }
//  else
//  {
//    ETH_Stop();
//    /*  When the netif link is down this function must be called.*/
//    netif_set_down(&gnetif);
//  }
//}

//----Class Methods------------------------------------------------------------------------
/**
 * @brief Class constructor: declare and *basic* initialization of the USART
 * @param num		number of the port to initialize. See usartSetup struct for details
 */
TEth::TEth():
		rx_head(0),
		rx_tail(0),
		tx_head(0),
		tx_tail(0)
{
	cleanAllLocalVariables();
}
//-----------------------------------------------------------------------------------------
/**
 * @brief Class Destructor: reset *basic* initialization of the class
 */
TEth::~TEth() {
	cleanAllLocalVariables();
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
void  TEth::cleanAllLocalVariables(void)
{
	rx_tail=0;
	rx_head=0;
	tx_tail=0;
	tx_head=0;
	cleanBuffer(bCircArrRx,RXBUFFERSIZE);
	cleanBuffer(bCircArrTx,RXBUFFERSIZE);
}
//-----------------------------------------------------------------------------------------
/**
 * @brief System Clock Configuration
 * @param usartID		number of the usart {1,2,3,4,5,6}
 * @retval Error number - see USART_Init() for details
 * @remarks Only USART1 and USART6 are connected to APB2 the other USARTs are
 * connected to APB1
 */
unsigned char  TEth::RCC_Configuration(void)
{
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_ETH_MAC | RCC_AHB1Periph_ETH_MAC_Tx |
                        RCC_AHB1Periph_ETH_MAC_Rx, ENABLE);
  
  /* Enable GPIOs clocks */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB |
                         RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOI |
                         RCC_AHB1Periph_GPIOG | RCC_AHB1Periph_GPIOH |
                         RCC_AHB1Periph_GPIOF, ENABLE);

  /* Enable SYSCFG clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);  

	return (0x00);
}
//-----------------------------------------------------------------------------------------
/**
 * @brief GPIO Configuration: prepares the pins to be used as RX TX
 * @retval Error number - 0x00 no error, 0x04 not valid usartID
 * @note UART5 RX & TX pins belong to 2 different GPIOx family. In the
 * procedure an if-clause will behave differently if UART5 is called. For this
 * reason the GPIOx value will be neglected
 */
unsigned char TEth::GPIO_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Configure MCO (PA8) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;  
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* MII/RMII Media interface selection --------------------------------------*/
#ifdef MII_MODE /* Mode MII with STM324xx-EVAL  */
 #ifdef PHY_CLOCK_MCO


  /* Output HSE clock (25MHz) on MCO pin (PA8) to clock the PHY */
  RCC_MCO1Config(RCC_MCO1Source_HSE, RCC_MCO1Div_1);
 #endif /* PHY_CLOCK_MCO */

  SYSCFG_ETH_MediaInterfaceConfig(SYSCFG_ETH_MediaInterface_MII);
#elif defined RMII_MODE  /* Mode RMII with STM324xx-EVAL */

  SYSCFG_ETH_MediaInterfaceConfig(SYSCFG_ETH_MediaInterface_RMII);
#endif

/* Ethernet pins configuration ************************************************/
   /*
        ETH_MDIO -------------------------> PA2
        ETH_MDC --------------------------> PC1
        ETH_PPS_OUT ----------------------> PB5
        ETH_MII_CRS ----------------------> PH2
        ETH_MII_COL ----------------------> PH3
        ETH_MII_RX_ER --------------------> PI10
        ETH_MII_RXD2 ---------------------> PH6
        ETH_MII_RXD3 ---------------------> PH7
        ETH_MII_TX_CLK -------------------> PC3
        ETH_MII_TXD2 ---------------------> PC2
        ETH_MII_TXD3 ---------------------> PB8
        ETH_MII_RX_CLK/ETH_RMII_REF_CLK---> PA1
        ETH_MII_RX_DV/ETH_RMII_CRS_DV ----> PA7
        ETH_MII_RXD0/ETH_RMII_RXD0 -------> PC4
        ETH_MII_RXD1/ETH_RMII_RXD1 -------> PC5
        ETH_MII_TX_EN/ETH_RMII_TX_EN -----> PG11
        ETH_MII_TXD0/ETH_RMII_TXD0 -------> PG13
        ETH_MII_TXD1/ETH_RMII_TXD1 -------> PG14
                                                  */

  /* Configure PA1, PA2 and PA7 */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_7;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_ETH);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_ETH);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_ETH);

  /* Configure PB11, PB12 and PB13 */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_ETH);	
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_ETH);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_ETH);

  /* Configure PC1, PC4 and PC5 */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource1, GPIO_AF_ETH);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource4, GPIO_AF_ETH);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource5, GPIO_AF_ETH);
                                
	return(0x00);
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
unsigned char TEth::ETH_StructConfig(void)
{
  /* Software reset */
  ETH_SoftwareReset();

  /* Wait for software reset */
  while (ETH_GetSoftwareResetStatus() == SET);

  /* ETHERNET Configuration --------------------------------------------------*/
  /* Call ETH_StructInit if you don't like to configure all ETH_InitStructure parameter */
  ETH_StructInit(&ETH_InitStructure);

  /* Fill ETH_InitStructure parametrs */
  /*------------------------   MAC   -----------------------------------*/
  ETH_InitStructure.ETH_AutoNegotiation = ETH_AutoNegotiation_Enable;
//  ETH_InitStructure.ETH_AutoNegotiation = ETH_AutoNegotiation_Disable;
//  ETH_InitStructure.ETH_Speed = ETH_Speed_10M;
//  ETH_InitStructure.ETH_Mode = ETH_Mode_FullDuplex;

  ETH_InitStructure.ETH_LoopbackMode = ETH_LoopbackMode_Disable;
  ETH_InitStructure.ETH_RetryTransmission = ETH_RetryTransmission_Disable;
  ETH_InitStructure.ETH_AutomaticPadCRCStrip = ETH_AutomaticPadCRCStrip_Disable;
  ETH_InitStructure.ETH_ReceiveAll = ETH_ReceiveAll_Disable;
  ETH_InitStructure.ETH_BroadcastFramesReception = ETH_BroadcastFramesReception_Enable;
  ETH_InitStructure.ETH_PromiscuousMode = ETH_PromiscuousMode_Disable;
  ETH_InitStructure.ETH_MulticastFramesFilter = ETH_MulticastFramesFilter_Perfect;
  ETH_InitStructure.ETH_UnicastFramesFilter = ETH_UnicastFramesFilter_Perfect;
#ifdef CHECKSUM_BY_HARDWARE
  ETH_InitStructure.ETH_ChecksumOffload = ETH_ChecksumOffload_Enable;
#endif

  /*------------------------   DMA   -----------------------------------*/  
  
  /* When we use the Checksum offload feature, we need to enable the Store and Forward mode: 
  the store and forward guarantee that a whole frame is stored in the FIFO, so the MAC can insert/verify the checksum, 
  if the checksum is OK the DMA can handle the frame otherwise the frame is dropped */
  ETH_InitStructure.ETH_DropTCPIPChecksumErrorFrame = ETH_DropTCPIPChecksumErrorFrame_Enable; 
  ETH_InitStructure.ETH_ReceiveStoreForward = ETH_ReceiveStoreForward_Enable;
  ETH_InitStructure.ETH_TransmitStoreForward = ETH_TransmitStoreForward_Enable;

  ETH_InitStructure.ETH_ForwardErrorFrames = ETH_ForwardErrorFrames_Disable;
  ETH_InitStructure.ETH_ForwardUndersizedGoodFrames = ETH_ForwardUndersizedGoodFrames_Disable;
  ETH_InitStructure.ETH_SecondFrameOperate = ETH_SecondFrameOperate_Enable;
  ETH_InitStructure.ETH_AddressAlignedBeats = ETH_AddressAlignedBeats_Enable;
  ETH_InitStructure.ETH_FixedBurst = ETH_FixedBurst_Enable;
  ETH_InitStructure.ETH_RxDMABurstLength = ETH_RxDMABurstLength_32Beat;
  ETH_InitStructure.ETH_TxDMABurstLength = ETH_TxDMABurstLength_32Beat;
  ETH_InitStructure.ETH_DMAArbitration = ETH_DMAArbitration_RoundRobin_RxTx_2_1;

  /* Configure Ethernet */
  EthStatus = ETH_Init(&ETH_InitStructure, ETH_PHY_ADDRESS);

  /* Get Ethernet link status*/
  if(ETH_ReadPHYRegister(ETH_PHY_ADDRESS, PHY_BSR) & 0x4)
  {
    EthStatus |= ETH_LINK_FLAG;
  }

	return(0x00);
}
//-----------------------------------------------------------------------------------------
/**
 * @brief Configure the Nested Vectored Interrupt Controller (NVIC)
 * @retval Error number - see USART_Init
 */
unsigned char  TEth::NVIC_StructConfig(void)
{
	return(0x00);
}

//-----------------------------------------------------------------------------------------
//##################################################			PUBLIC FUNCTIONS
//-----------------------------------------------------------------------------------------
/**
 * @brief ETH Full Initialization
 * @retval Error code bits:
			- &=0x01 - invalid number: there is no such a USART configuration;
			- &=0x02 - RCC System Clocks Configuration error
			- &=0x04 - GPIO Configuration error
			- &=0x08 - ETH_Struct Configuration error
			- &=0x10 - Nested VEctor Interrupt Routine error
 */
unsigned char TEth::setup_HW(void)
{
	unsigned char error = 0x00;

  /* Reset ETHERNET on AHB Bus */
  ETH_DeInit();
	
	error|=RCC_Configuration();/*!< System Clocks Configuration */
	error|=GPIO_Config();/*!< GPIO Configuration */
	error|=ETH_StructConfig();/*!< stpIndex configuration */
	error|=NVIC_StructConfig();/*!< Nested VEctor Interrupt Routine*/

	return (error);
}

//-----------------------------------------------------------------------------------------
/**
 * @brief Open the chosed port
 * @return Error code (see TEth::setup_HW(void) )
 * @pre USE TEth::setupCOM() before calling open() function: otherwise default setup is loaded
 * @remark  do not use other ASSERT for USART_InitStructure: parameters can be 0x00
 */
char TEth::open(void)
{
	unsigned char error = 0;
  struct ip_addr ipaddr;
  struct ip_addr netmask;
  struct ip_addr gw;

	error = setup_HW();

  /* Initializes the dynamic memory heap defined by MEM_SIZE.*/
  mem_init();

  /* Initializes the memory pools defined by MEMP_NUM_x.*/
  memp_init();

  IP4_ADDR(&ipaddr, IP_ADDR0, IP_ADDR1, IP_ADDR2, IP_ADDR3);
  IP4_ADDR(&netmask, NETMASK_ADDR0, NETMASK_ADDR1 , NETMASK_ADDR2, NETMASK_ADDR3);
  IP4_ADDR(&gw, GW_ADDR0, GW_ADDR1, GW_ADDR2, GW_ADDR3);
  
  /* - netif_add(struct netif *netif, struct ip_addr *ipaddr,
  struct ip_addr *netmask, struct ip_addr *gw,
  void *state, err_t (* init)(struct netif *netif),
  err_t (* input)(struct pbuf *p, struct netif *netif))

  Adds your network interface to the netif_list. Allocate a struct
  netif and pass a pointer to this structure as the first argument.
  Give pointers to cleared ip_addr structures when using DHCP,
  or fill them with sane numbers otherwise. The state pointer may be NULL.

  The init function pointer must point to a initialization function for
  your ethernet netif interface. The following code illustrates it's use.*/
  netif_add(&gnetif, &ipaddr, &netmask, &gw, NULL, &ethernetif_init, &ethernet_input);

  /*  Registers the default network interface.*/
  netif_set_default(&gnetif);

  if (EthStatus == (ETH_INIT_FLAG | ETH_LINK_FLAG))
  { 
    /* Set Ethernet link flag */
    gnetif.flags |= NETIF_FLAG_LINK_UP;

    /* When the netif is fully configured this function must be called.*/
    netif_set_up(&gnetif);
  }
  else
  {
    /*  When the netif link is down this function must be called.*/
    netif_set_down(&gnetif);
  }
  
  /* Set the link callback function, this function is called on change of link status*/
  // !!!!! netif_set_link_callback(&gnetif, ETH_link_callback);

	return(error);
}
//-----------------------------------------------------------------------------------------
/**
 * @brief Check if the port is open
 * @return port status:
 				- 0x00 if port is closed
  				- 0x01 if port is open
 */
char TEth::isOpen (void)
{
	return(0x00);
}
//-----------------------------------------------------------------------------------------
/**
 * @brief Close the port without de-initializing it. All the RX / TX memory will be erased
 */
void TEth::close (void)
{

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
void TEth::ETH_puts(const volatile char *s)
{

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
void TEth::cleanBuffer(volatile char *buffer, const unsigned short int numChar)
{
	unsigned int bb;
	for(bb=0; bb<numChar; bb++)
		buffer[bb]='-';
}
//-----------------------------------------------------------------------------------------
/**
 * @brief Wipe the private bCircArrRx data replacing elements with "*" char
 */
void TEth::cleanBuffer(void)
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
int TEth::read(volatile char *msgRx)
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
int TEth::bytesAvailable(void)
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
int TEth::bytesToWrite(void)
{
	int notAlreadyWrittenChar = tx_tail - tx_head;
	if(notAlreadyWrittenChar<0)
		notAlreadyWrittenChar+=RXBUFFERSIZE;
	return(notAlreadyWrittenChar);
}
//-----------------------------------------------------------------------------------------
/**
 * @param msg 		ptr to the message array you want to transmit
 * @param charNum2Send  ptr to the number of char to effectively transmit
 * @return error
				- |=0x01 - message completely sent
				- &=0x01 - Port Not correctly initialized
				- &=0x02 - Too much character to send: stay < RXBUFFERSIZE
 */
unsigned char TEth::write(const volatile char *msg,
		short int charNum2Send)
{
	unsigned short int  qq=0;
	int delta;

	if (charNum2Send>0 && charNum2Send<=RXBUFFERSIZE)
	{
	}
	return (0x02);// Too much character to send: stay < RXBUFFERSIZE
}

//-----------------------------------------------------------------------------------------
/**
 * @brief append a msg onto circual array. Instead of TEth::write() this
 * function does not ENABLE Interrupt USART_IT_TXE
 * @param msg	ptr to the message array you want to transmit
 * @param charNum2Send	ptr to the number of char to effectively transmit
 * @return error
				- |=0x01 - message completely sent
				- &=0x01 - Port Not correctly initialized
				- &=0x02 - Too much character to send: stay < RXBUFFERSIZE
 */
unsigned char TEth::append(const volatile char *msg,
		unsigned short int charNum2Send)
{
	unsigned short int  qq=0;
	int delta;

	if (charNum2Send<=RXBUFFERSIZE)
	{
	}
	return (0x02);// Too much character to send: stay < RXBUFFERSIZE
}

