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
	#include <string.h>
	#include "stm32f4xx_rcc.h"
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

ETH_InitTypeDef TEth::ETH_InitStructure;
__IO uint32_t TEth::EthStatus;
__IO uint8_t TEth::EthLinkStatus;

struct netif TEth::gnetif;
ip_addr_t TEth::ip_local;
ip_addr_t TEth::netmask;
ip_addr_t TEth::gateway;
ip_addr_t TEth::ip_server;

struct client *TEth::esTx;
struct tcp_pcb *TEth::pcbTx;

extern "C" {
	void Delay(int ms)
	{
		ms = 0;
	}
}

err_t tcp_client_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
<<<<<<< HEAD
void tcp_client_err(void *arg, err_t err);
=======
>>>>>>> ea15693cf22a041b6b3e33cfc334eba75aeabe42
void tcp_client_connection_close(struct tcp_pcb *tpcb, struct client * es);
err_t tcp_client_poll(void *arg, struct tcp_pcb *tpcb);
err_t tcp_client_sent(void *arg, struct tcp_pcb *tpcb, u16_t len);
void tcp_client_send(struct tcp_pcb *tpcb, struct client * es);
err_t tcp_client_connected(void *arg, struct tcp_pcb *tpcb, err_t err);

void ETH_link_callback(struct netif *netif)
{
	__IO uint32_t timeout = 0;
	uint32_t tmpreg;
	uint16_t RegValue;

  if(netif_is_link_up(netif))
  {
    /* Restart the auto-negotiation */
    if(TEth::ETH_InitStructure.ETH_AutoNegotiation != ETH_AutoNegotiation_Disable)
    {
      /* Reset Timeout counter */
      timeout = 0;

      /* Enable auto-negotiation */
      ETH_WritePHYRegister(ETH_PHY_ADDRESS, PHY_BCR, PHY_AutoNegotiation);

      /* Wait until the auto-negotiation will be completed */
      do
      {
        timeout++;
      } while (!(ETH_ReadPHYRegister(ETH_PHY_ADDRESS, PHY_BSR) & PHY_AutoNego_Complete) && (timeout < (uint32_t)PHY_READ_TO));  

      /* Reset Timeout counter */
      timeout = 0;

      /* Read the result of the auto-negotiation */
      RegValue = ETH_ReadPHYRegister(ETH_PHY_ADDRESS, PHY_SR);

      /* Configure the MAC with the Duplex Mode fixed by the auto-negotiation process */
			/* Mask out bits which are not for speed and link indication, bits 4:2 are used */
			RegValue = (RegValue >> 2) & 0x07;

			/* Switch statement */
			switch (RegValue) {
				case 1: /* Base 10, half-duplex */
					TEth::ETH_InitStructure.ETH_Speed = ETH_Speed_10M;
					TEth::ETH_InitStructure.ETH_Mode = ETH_Mode_HalfDuplex;
					break;
				case 2: /* Base 100, half-duplex */
					TEth::ETH_InitStructure.ETH_Speed = ETH_Speed_100M;
					TEth::ETH_InitStructure.ETH_Mode = ETH_Mode_HalfDuplex;
					break;
				case 5: /* Base 10, full-duplex */
					TEth::ETH_InitStructure.ETH_Speed = ETH_Speed_10M;
					TEth::ETH_InitStructure.ETH_Mode = ETH_Mode_FullDuplex;
					break;
				case 6: /* Base 100, full-duplex */
					TEth::ETH_InitStructure.ETH_Speed = ETH_Speed_100M;
					TEth::ETH_InitStructure.ETH_Mode = ETH_Mode_FullDuplex;
					break;
				default:
					break;
			}

      /*------------------------ ETHERNET MACCR Re-Configuration --------------------*/
      /* Get the ETHERNET MACCR value */  
      tmpreg = ETH->MACCR;

      /* Set the FES bit according to ETH_Speed value */ 
      /* Set the DM bit according to ETH_Mode value */ 
      tmpreg |= (uint32_t)(TEth::ETH_InitStructure.ETH_Speed | TEth::ETH_InitStructure.ETH_Mode);

      /* Write to ETHERNET MACCR */
      ETH->MACCR = (uint32_t)tmpreg;

      //_eth_delay_(ETH_REG_WRITE_DELAY);
      tmpreg = ETH->MACCR;
      ETH->MACCR = tmpreg;
    }

    /* Restart MAC interface */
    ETH_Start();

    netif_set_addr(&TEth::gnetif, &TEth::ip_local , &TEth::netmask, &TEth::gateway);
    
    /* When the netif is fully configured this function must be called.*/
    netif_set_up(&TEth::gnetif);    

		TEth::EthLinkStatus = 0;
  }
  else
  {
    ETH_Stop();
    /*  When the netif link is down this function must be called.*/
    netif_set_down(&TEth::gnetif);
  }
}

/* Handle the incoming TCP Data */

void tcp_client_handle (struct tcp_pcb *tpcb, struct client *es)
{
	TEth::esTx = es;
	TEth::pcbTx = tpcb;
}

<<<<<<< HEAD
void tcp_client_err(void *arg, err_t err)
{
  struct client *es;

  if (err != ERR_OK)
	{
    LWIP_ASSERT("arg != NULL",arg != NULL);

    es = (struct client *)arg;
    es->state = ES_NOT_CONNECTED;
	}
}

=======
>>>>>>> ea15693cf22a041b6b3e33cfc334eba75aeabe42
/**
  * @brief tcp_receiv callback
  * @param arg: argument to be passed to receive callback 
  * @param tpcb: tcp connection control block 
  * @param err: receive error code 
  * @retval err_t: retuned error  
  */
err_t tcp_client_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{ 
  struct client *es;
  err_t ret_err;
  

  LWIP_ASSERT("arg != NULL",arg != NULL);
  
  es = (struct client *)arg;
  
  /* if we receive an empty tcp frame from server => close connection */
  if (p == NULL)
  {
    /* remote host closed connection */
    es->state = ES_CLOSING;
    if(es->p_tx == NULL)
    {
       /* we're done sending, close connection */
       tcp_client_connection_close(tpcb, es);
    }
    else
    {    
      /* send remaining data*/
      tcp_client_send(tpcb, es);
    }
    ret_err = ERR_OK;
  }   
  /* else : a non empty frame was received from echo server but for some reason err != ERR_OK */
  else if(err != ERR_OK)
  {
    /* free received pbuf*/
    if (p != NULL)
    {
      es->p_tx = NULL;
      pbuf_free(p);
    }

    ret_err = err;
  }
  else if(es->state == ES_CONNECTED)
  {       
    /* Acknowledge data reception */
    tcp_recved(tpcb, p->tot_len);  

    /* handle the received data */
    tcp_client_handle(tpcb, es);
    
    pbuf_free(p);

    ret_err = ERR_OK;
  }

  /* data received when connection already closed */
  else
  {
    /* Acknowledge data reception */
    tcp_recved(tpcb, p->tot_len);
    
    /* free pbuf and do nothing */
    pbuf_free(p);
    ret_err = ERR_OK;
  }
  return ret_err;
}

/**
  * @brief function used to send data
  * @param  tpcb: tcp control block
  * @param  es: pointer on structure of type client containing info on data 
  *             to be sent
  * @retval None 
  */
void tcp_client_send(struct tcp_pcb *tpcb, struct client * es)
{
  struct pbuf *ptr;
  err_t wr_err = ERR_OK;
 
  while ((wr_err == ERR_OK) &&
         (es->p_tx != NULL) && 
         (es->p_tx->len <= tcp_sndbuf(tpcb)))
  {
    
    /* get pointer on pbuf from es structure */
    ptr = es->p_tx;

    /* enqueue data for transmission */
    wr_err = tcp_write(tpcb, ptr->payload, ptr->len, 1);
    
    if (wr_err == ERR_OK)
    { 
      /* continue with next pbuf in chain (if any) */
      es->p_tx = ptr->next;
      
      if(es->p_tx != NULL)
      {
        /* increment reference count for es->p */
        pbuf_ref(es->p_tx);
      }
      
      /* free pbuf: will free pbufs up to es->p (because es->p has a reference count > 0) */
      pbuf_free(ptr);
   }
   else if(wr_err == ERR_MEM)
   {
      /* we are low on memory, try later, defer to poll */
     es->p_tx = ptr;
   }
   else
   {
     /* other problem ?? */
   }
  }
}

/**
  * @brief  This function implements the tcp_poll callback function
  * @param  arg: pointer on argument passed to callback
  * @param  tpcb: tcp connection control block
  * @retval err_t: error code
  */
err_t tcp_client_poll(void *arg, struct tcp_pcb *tpcb)
{
  err_t ret_err;
  struct client *es;

  es = (struct client*)arg;
  if (es != NULL)
  {
    if (es->p_tx != NULL)
    {
      /* there is a remaining pbuf (chain) , try to send data */
      tcp_client_send(tpcb, es);
    }
    else
    {
      /* no remaining pbuf (chain)  */
      if(es->state == ES_CLOSING)
      {
        /* close tcp connection */
        tcp_client_connection_close(tpcb, es);
      }
    }
    ret_err = ERR_OK;
  }
  else
  {
    /* nothing to be done */
    tcp_abort(tpcb);
    ret_err = ERR_ABRT;
  }
  return ret_err;
}

/**
  * @brief  This function implements the tcp_sent LwIP callback (called when ACK
  *         is received from remote host for sent data) 
  * @param  arg: pointer on argument passed to callback
  * @param  tcp_pcb: tcp connection control block
  * @param  len: length of data sent 
  * @retval err_t: returned error code
  */
err_t tcp_client_sent(void *arg, struct tcp_pcb *tpcb, u16_t len)
{
  struct client *es;

  LWIP_UNUSED_ARG(len);

  es = (struct client *)arg;
  
  if(es->p_tx != NULL)
  {
    /* still got pbufs to send */
    tcp_client_send(tpcb, es);
  }
  else
  {
    /* if no more data to send and client closed connection*/
    if(es->state == ES_CLOSING)
    	tcp_client_connection_close(tpcb, es);
  }

  return ERR_OK;
}

/**
  * @brief This function is used to close the tcp connection with server
  * @param tpcb: tcp connection control block
  * @param es: pointer on client structure
  * @retval None
  */
void tcp_client_connection_close(struct tcp_pcb *tpcb, struct client * es )
{
  /* remove callbacks */
  tcp_recv(tpcb, NULL);
  tcp_sent(tpcb, NULL);
  tcp_poll(tpcb, NULL,0);

<<<<<<< HEAD
	tcp_client_handle(NULL, NULL);
	
=======
>>>>>>> ea15693cf22a041b6b3e33cfc334eba75aeabe42
  if (es != NULL)
  {
    mem_free(es);
  }

  /* close tcp connection */
  tcp_close(tpcb);
  
}

/**
  * @brief Function called when TCP connection established
  * @param tpcb: pointer on the connection contol block
  * @param err: when connection correctly established err should be ERR_OK 
  * @retval err_t: returned error 
  */
err_t tcp_client_connected(void *arg, struct tcp_pcb *tpcb, err_t err)
{
	struct client *es = NULL;
	
  if (err == ERR_OK)   
  {
    /* allocate structure es to maintain tcp connection informations */
    es = (struct client *)mem_malloc(sizeof(struct client));
  
    if (es != NULL)
    {
      es->state = ES_CONNECTED;
      es->pcb = tpcb;
      es->retries = 0;
      es->p_tx = NULL;      
         
			/* pass newly allocated es structure as argument to tpcb */
			tcp_arg(tpcb, es);

<<<<<<< HEAD
=======
			/* initialize LwIP tcp_recv callback function */ 
			tcp_recv(tpcb, tcp_client_recv);

			/* initialize LwIP tcp_sent callback function */
			tcp_sent(tpcb, tcp_client_sent);

			/* initialize LwIP tcp_poll callback function */
			tcp_poll(tpcb, tcp_client_poll, 1);    

>>>>>>> ea15693cf22a041b6b3e33cfc334eba75aeabe42
	    /* handle the TCP data */
	    tcp_client_handle(tpcb, es);
			
      return ERR_OK;
    }
    else
    {
      /* close connection */
      tcp_client_connection_close(tpcb, es);
      
      /* return memory allocation error */
      return ERR_MEM;  
    }
  }
  else
  {
    /* close connection */
    tcp_client_connection_close(tpcb, es);
  }
  return err;
}
    

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
	
	tcp_client_handle(NULL, NULL);
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
unsigned char TEth::NVIC_StructConfig(void)
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
	
	error |= RCC_Configuration();/*!< System Clocks Configuration */
	error |= GPIO_Config();/*!< GPIO Configuration */
	error |= ETH_StructConfig();/*!< stpIndex configuration */
	error |= NVIC_StructConfig();/*!< Nested VEctor Interrupt Routine*/

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
	char error = 0;

	error = setup_HW();

	if (error == 0) {
		/* Initializes the dynamic memory heap defined by MEM_SIZE.*/
		mem_init();

		/* Initializes the memory pools defined by MEMP_NUM_x.*/
		memp_init();
		
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
		netif_add(&gnetif, &ip_local, &netmask, &gateway, NULL, &ethernetif_init, &ethernet_input);

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
		netif_set_link_callback(&gnetif, ETH_link_callback);
	
		/* create new tcp pcb */
		client_pcb = tcp_new();
		
<<<<<<< HEAD
		if (client_pcb != NULL) {
			/* connect to destination address/port */
			error = tcp_connect(client_pcb, &ip_server , DEST_PORT, tcp_client_connected);
			
			if (error == ERR_OK) {
				/* initialize LwIP tcp_recv callback function */ 
				tcp_recv(client_pcb, tcp_client_recv);

				/* initialize LwIP tcp_sent callback function */
				tcp_sent(client_pcb, tcp_client_sent);

				/* initialize LwIP tcp_poll callback function */
				tcp_poll(client_pcb, tcp_client_poll, 1);    

				/* initialize LwIP tcp_err callback function */
				tcp_err(client_pcb, tcp_client_err);
			}
		}
=======
		if (client_pcb != NULL)
			/* connect to destination address/port */
			error = tcp_connect(client_pcb, &ip_server , DEST_PORT, tcp_client_connected);
>>>>>>> ea15693cf22a041b6b3e33cfc334eba75aeabe42
		else
			error = ERR_MEM;
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
char TEth::isOpen (void)
{
	if (esTx)
		return (esTx->state == ES_CONNECTED);
	return (0x0);
}
//-----------------------------------------------------------------------------------------
/**
 * @brief Close the port without de-initializing it. All the RX / TX memory will be erased
 */
void TEth::close (void)
{

}

void TEth::poll(uint32_t localTime)
{
	/* check if any packet received */
	if (ETH_CheckFrameReceived())
	{ 
		/* process received ethernet packet */
		ethernetif_input(&gnetif);
	}
    /* handle periodic timers for LwIP */
#if LWIP_TCP
  /* TCP periodic process every 250 ms */
  if (localTime - TCPTimer >= TCP_TMR_INTERVAL)
  {
    TCPTimer =  localTime;
    tcp_tmr();
  }
#endif

  /* ARP periodic process every 5s */
  if ((localTime - ARPTimer) >= ARP_TMR_INTERVAL)
  {
    ARPTimer =  localTime;
    etharp_tmr();
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
void TEth::puts(const char *s)
{
<<<<<<< HEAD
	strncpy((char*)dataTx, (const char *)s, strlen(s));
	dataTx[sizeof(dataTx) - 1] = '\0';

	/* allocate pbuf */
	esTx->p_tx = pbuf_alloc(PBUF_TRANSPORT, strlen((char*)dataTx) , PBUF_POOL);
=======
	u8_t data[100];
	
	strncpy((char*)data, (const char *)s, sizeof(data) - 1);
	data[sizeof(data) - 1] = '\0';

	/* allocate pbuf */
	esTx->p_tx = pbuf_alloc(PBUF_TRANSPORT, strlen((char*)data) , PBUF_POOL);
>>>>>>> ea15693cf22a041b6b3e33cfc334eba75aeabe42
 
	if (esTx->p_tx)
	{       
		/* copy data to pbuf */
<<<<<<< HEAD
		pbuf_take(esTx->p_tx, (char*)dataTx, strlen((char*)dataTx));
=======
		pbuf_take(esTx->p_tx, (char*)data, strlen((char*)data));
>>>>>>> ea15693cf22a041b6b3e33cfc334eba75aeabe42

		/* send data */
		tcp_client_send(esTx->pcb, esTx);
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
unsigned char TEth::write(const char *msg, unsigned short int charNum2Send)
{
<<<<<<< HEAD
	strncpy((char*)dataTx, (const char *)msg, charNum2Send);
	dataTx[sizeof(dataTx) - 1] = '\0';
=======
	u8_t data[100];
	
	strncpy((char*)data, (const char *)msg, charNum2Send);
	data[sizeof(data) - 1] = '\0';
>>>>>>> ea15693cf22a041b6b3e33cfc334eba75aeabe42

	/* allocate pbuf */
	esTx->p_tx = pbuf_alloc(PBUF_TRANSPORT, charNum2Send , PBUF_POOL);
 
	if (esTx->p_tx)
	{       
		/* copy data to pbuf */
<<<<<<< HEAD
		pbuf_take(esTx->p_tx, (char*)dataTx, strlen((char*)dataTx));
=======
		pbuf_take(esTx->p_tx, (char*)data, strlen((char*)data));
>>>>>>> ea15693cf22a041b6b3e33cfc334eba75aeabe42

		/* send data */
		tcp_client_send(esTx->pcb, esTx);
	}	
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
unsigned char TEth::append(const char *msg,
		unsigned short int charNum2Send)
{
	unsigned short int  qq=0;
	int delta;

	if (charNum2Send<=RXBUFFERSIZE)
	{
	}
	return (0x02);// Too much character to send: stay < RXBUFFERSIZE
}

