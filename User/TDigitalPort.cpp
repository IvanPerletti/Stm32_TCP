/******************************************************************************************
 *
 *  \brief      DIGITAL PORT Class
 *  \details    This class is used to configure digital Port communication onto MainController
 *  by using the 4 shift register for Input and 5 shift register for output. Developed for
 *  STM32F4 chip
 *  \author     Ivan Perletti - General Medical Merate- GMM.spa - Seriate - Italy
 *  \version    1.0
 *  \date       June 27th, 2016
 *  \pre        SpiLineINPUT e SpiLinOUTPUT to be initialized 16B per Word
 *  \bug        Not all memory is freed when deleting an object of this class.
 *  \warning    Improper use can crash your application
 *  \copyright GMM.spa - All Rights Reserved
 *
 ******************************************************************************************/

#include "TDigitalPort.h"

TDigitalPort digitalPort;

#define SIZE_OF_VECT(VECT) (sizeof(VECT) / sizeof(VECT[0]))	

// LOCAL CONST-----------------------------------------------------------------
const GPIO_InitTypeDef_F4 karGPIO_InitTypeDef[] =
{
	#define X(NAME, PORT, PIN, PP) {GPIO_INPUT, PORT, PIN, RCC_AHB1Periph_##PORT, GPIO_Mode_IN, GPIO_Speed_2MHz, GPIO_OType_PP, PP},
		GPIO_INIT_INPUT
	#undef X
	#define X(NAME, PORT, PIN, SPEED, OTYPE, PP) {GPIO_OUTPUT, PORT, PIN, RCC_AHB1Periph_##PORT, GPIO_Mode_OUT, SPEED, OTYPE, PP},
		GPIO_INIT_OUTPUT
	#undef X
};

#if defined(GPIO_INIT_EXTI)
const EXTI_InitTypeDef_F4 karEXTI_InitTypeDef[] =
{
	#define X(NAME, PORT, EXTI_PIN, PP, EXTI_TRIGGER, EXTI_LINE, IRQn, PRIORITY, SUBPRIORITY) {EXTI_PortSource##PORT, EXTI_PIN, EXTI_LINE, EXTI_Mode_Interrupt, EXTI_TRIGGER, ENABLE},
		GPIO_INIT_EXTI
	#undef X
};
#endif

#if defined(GPIO_INIT_EXTI)
const NVIC_InitTypeDef karNVIC_InitTypeDef[] =
{	
	#define X(NAME, PORT, EXTI_PIN, PP, EXTI_TRIGGER, EXTI_LINE, IRQn, PRIORITY, SUBPRIORITY) {IRQn, PRIORITY, SUBPRIORITY, ENABLE},
		GPIO_INIT_EXTI
	#undef X		
};
#endif

// LOCAL DEFINE-----------------------------------------------------------------
#define BASE_GPIO_INPUT		0
#define BASE_GPIO_OUTPUT	DI_LAST_REAL
#if (BASE_GPIO_OUTPUT < BASE_GPIO_INPUT)
#error "GPIO_INIT_OUTPUT must be after GPIO_INIT_INPUT!"
#endif

//-----------------------------------------------------------------------------
/**
  * @brief  Initializes the EXTI peripheral according to the specified
  *         parameters in the EXTI_InitStruct.
  * @param  EXTI_InitStruct: pointer to a EXTI_InitTypeDef_F4 structure
  *         that contains the configuration information for the EXTI peripheral.
  * @retval None
  */
void EXTI_Init_F4( EXTI_InitTypeDef_F4* EXTI_InitStruct)
{
  uint32_t tmp = 0;

  /* Check the parameters */
  assert_param(IS_EXTI_MODE(EXTI_InitStruct->EXTI_Mode));
  assert_param(IS_EXTI_TRIGGER(EXTI_InitStruct->EXTI_Trigger));
  assert_param(IS_EXTI_LINE(EXTI_InitStruct->EXTI_Line));  
  assert_param(IS_FUNCTIONAL_STATE(EXTI_InitStruct->EXTI_LineCmd));

  tmp = (uint32_t)EXTI_BASE;
     
  if (EXTI_InitStruct->EXTI_LineCmd != DISABLE)
  {
    /* Clear EXTI line configuration */
    EXTI->IMR &= ~EXTI_InitStruct->EXTI_Line;
    EXTI->EMR &= ~EXTI_InitStruct->EXTI_Line;
    
    tmp += EXTI_InitStruct->EXTI_Mode;

    *(__IO uint32_t *) tmp |= EXTI_InitStruct->EXTI_Line;

    /* Clear Rising Falling edge configuration */
    EXTI->RTSR &= ~EXTI_InitStruct->EXTI_Line;
    EXTI->FTSR &= ~EXTI_InitStruct->EXTI_Line;
    
    /* Select the trigger for the selected external interrupts */
    if (EXTI_InitStruct->EXTI_Trigger == EXTI_Trigger_Rising_Falling)
    {
      /* Rising Falling edge */
      EXTI->RTSR |= EXTI_InitStruct->EXTI_Line;
      EXTI->FTSR |= EXTI_InitStruct->EXTI_Line;
    }
    else
    {
      tmp = (uint32_t)EXTI_BASE;
      tmp += EXTI_InitStruct->EXTI_Trigger;

      *(__IO uint32_t *) tmp |= EXTI_InitStruct->EXTI_Line;
    }
  }
  else
  {
    tmp += EXTI_InitStruct->EXTI_Mode;

    /* Disable the selected external lines */
    *(__IO uint32_t *) tmp &= ~EXTI_InitStruct->EXTI_Line;
  }
}
//-----------------------------------------------------------------------------
/**
  * @brief  Deinitializes the Alternate Functions (remap and EXTI configuration)
  *   registers to their default reset values.
  * @param  None
  * @retval None
  */
void SYSCFG_DeInit_F4(void)
{
	RCC->APB2RSTR |= RCC_APB2Periph_SYSCFG;
	RCC->APB2RSTR &= ~RCC_APB2Periph_SYSCFG;
}
//-----------------------------------------------------------------------------
/**
  * @brief  Selects the GPIO pin used as EXTI Line.
  * @param  EXTI_PortSourceGPIOx : selects the GPIO port to be used as source for
  *          EXTI lines where x can be (A..I).
  * @param  EXTI_PinSourcex: specifies the EXTI line to be configured.
  *           This parameter can be EXTI_PinSourcex where x can be (0..15, except
  *           for EXTI_PortSourceGPIOI x can be (0..11).
  * @retval None
  */
void SYSCFG_EXTILineConfig_F4(uint8_t EXTI_PortSourceGPIOx, uint8_t EXTI_PinSourcex)
{
  uint32_t tmp = 0x00;

  /* Check the parameters */
  assert_param(IS_EXTI_PORT_SOURCE(EXTI_PortSourceGPIOx));
  assert_param(IS_EXTI_PIN_SOURCE(EXTI_PinSourcex));

	RCC->APB2ENR |= RCC_APB2Periph_SYSCFG;
	
  tmp = ((uint32_t)0x0F) << (0x04 * (EXTI_PinSourcex & (uint8_t)0x03));
  SYSCFG->EXTICR[EXTI_PinSourcex >> 0x02] &= ~tmp;
  SYSCFG->EXTICR[EXTI_PinSourcex >> 0x02] |= (((uint32_t)EXTI_PortSourceGPIOx) << (0x04 * (EXTI_PinSourcex & (uint8_t)0x03)));
}
//------------------------------------------------------------------------------
/**
 * @brief Constructor
 * @param none
 */
TDigitalPort::TDigitalPort()
{
	bClassInitialize = false;
	resetAllOUT();
}
//------------------------------------------------------------------------------
/**
 * @brief Destructor
 * @param none
 */
TDigitalPort::~TDigitalPort()
{
}
// ----------------------------------------------------------------------------
/**
  * @brief  Deinitializes Digital Port
  * @param  None
  * @retval None
  */
void TDigitalPort::deInit(void)
{	
	uint32_t u32I, RCC_AHB1Periph_GPIOx;
	#if defined(GPIO_INIT_EXTI)
	NVIC_InitTypeDef NVIC_InitStructure;
	#endif
	
	for (u32I = 0; u32I < SIZE_OF_VECT(karGPIO_InitTypeDef); u32I++)
	{
		RCC_AHB1Periph_GPIOx = karGPIO_InitTypeDef[u32I].RCC_AHB1Periph_GPIOx;
		RCC->AHB1RSTR |= RCC_AHB1Periph_GPIOx;
    RCC->AHB1RSTR &= ~RCC_AHB1Periph_GPIOx;		
	}
	
	#if defined(GPIO_INIT_EXTI)
		for (u32I = 0; u32I < SIZE_OF_VECT(karNVIC_InitTypeDef); u32I++)
		{
			SYSCFG_DeInit_F4();
			EXTI_DeInit();
			NVIC_InitStructure.NVIC_IRQChannel = karNVIC_InitTypeDef[u32I].NVIC_IRQChannel;
			NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
			NVIC_Init(&NVIC_InitStructure);
		}
	#endif	
	
	bClassInitialize = false;	
}

// ----------------------------------------------------------------------------
/**
  * @brief  Initializes the Digital Port
  * @retval None
  */
void TDigitalPort::init(void)
{
	uint32_t u32I, u32PinPos, u32Pos, u32CurrentPin;
	GPIO_InitTypeDef_F4 *GPIO_InitStructure;
	#if defined(GPIO_INIT_EXTI)
	EXTI_InitTypeDef_F4 *EXTI_InitStructure;
	NVIC_InitTypeDef *NVIC_InitStructure;
	#endif
	
	GPIO_TypeDef* GPIOx;
	
	if (bClassInitialize == false)
	{
		//deInit();
		
		bClassInitialize = true;
		for (u32I = 0; u32I < SIZE_OF_VECT(karGPIO_InitTypeDef); u32I++)
		{
			/* Check the parameters */
			GPIO_InitStructure = (GPIO_InitTypeDef_F4 *)&karGPIO_InitTypeDef[u32I];	
			
			assert_param(IS_GPIO_TYPE(GPIO_InitStructure->GPIO_Type));
			assert_param(IS_GPIO_ALL_PERIPH(GPIO_InitStructure->GPIO_Port));
			assert_param(IS_GET_GPIO_PIN(GPIO_InitStructure->GPIO_Pin));
			assert_param(IS_GPIO_MODE(GPIO_InitStructure->GPIO_Mode));
			assert_param(IS_GPIO_PUPD(GPIO_InitStructure->GPIO_PuPd));
			
			GPIOx = GPIO_InitStructure->GPIO_Port;		
			
			RCC->AHB1ENR |= GPIO_InitStructure->RCC_AHB1Periph_GPIOx;
			
			for (u32PinPos = 0x00; u32PinPos < 0x10; u32PinPos++)
			{
				u32Pos = ((uint32_t)0x01) << u32PinPos;
				u32CurrentPin = GPIO_InitStructure->GPIO_Pin & u32Pos;
				
				if (u32CurrentPin == u32Pos)
				{								
					GPIOx->MODER  &= ~(GPIO_MODER_MODER0 << (u32PinPos * 2));
					GPIOx->MODER |= (((uint32_t)GPIO_InitStructure->GPIO_Mode) << (u32PinPos * 2));

					if ((GPIO_InitStructure->GPIO_Mode == GPIO_Mode_OUT) || (GPIO_InitStructure->GPIO_Mode == GPIO_Mode_AF))
					{
						// GPIO_INPUT, GPIO_OUTPUT
						
						// Check Speed mode parameters 
						assert_param(IS_GPIO_SPEED(GPIO_InitStructure->GPIO_Speed));

						// Speed mode configuration 
						GPIOx->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR0 << (u32PinPos * 2));
						GPIOx->OSPEEDR |= ((uint32_t)(GPIO_InitStructure->GPIO_Speed) << (u32PinPos * 2));

						// Check Output mode parameters 
						assert_param(IS_GPIO_OTYPE(GPIO_InitStructure->GPIO_OType));

						// Output mode configuration
						GPIOx->OTYPER  &= ~((GPIO_OTYPER_OT_0) << ((uint16_t)u32PinPos)) ;
						GPIOx->OTYPER |= (uint16_t)(((uint16_t)GPIO_InitStructure->GPIO_OType) << ((uint16_t)u32PinPos));
					}

					// Pull-up Pull down resistor configuration
					GPIOx->PUPDR &= ~(GPIO_PUPDR_PUPDR0 << ((uint16_t)u32PinPos * 2));
					GPIOx->PUPDR |= (((uint32_t)GPIO_InitStructure->GPIO_PuPd) << (u32PinPos * 2));								
				}
			}
		}
		
		#if defined(GPIO_INIT_EXTI)
		for (u32I = 0; u32I < SIZE_OF_VECT(karEXTI_InitTypeDef); u32I++)
		{
			EXTI_InitStructure = (EXTI_InitTypeDef_F4 *)&karEXTI_InitTypeDef[u32I];
			NVIC_InitStructure = (NVIC_InitTypeDef *)&karNVIC_InitTypeDef[u32I];
							
			SYSCFG_EXTILineConfig_F4(EXTI_InitStructure->EXTI_PortSource, EXTI_InitStructure->EXTI_PinSource);
			EXTI_Init_F4(EXTI_InitStructure);
			NVIC_Init(NVIC_InitStructure);
		}
		#endif 
	}
}

//------------------------------------------------------------------------------
/**
 * @brief <b> Suspensive code</b>: it reads the actual digital Input on the
 * shift register via SPI2
 */
void TDigitalPort::updateIN(void)
{
	uint32_t u32I;
	GPIO_InitTypeDef_F4 *GPIO_InitStructure;
	
	if (bClassInitialize != false)
	{
		for (u32I = 0; u32I < DI_LAST_REAL; u32I++)
		{
			GPIO_InitStructure = (GPIO_InitTypeDef_F4 *)&karGPIO_InitTypeDef[u32I + BASE_GPIO_INPUT];
			
			if ((GPIO_InitStructure->GPIO_Port->IDR & GPIO_InitStructure->GPIO_Pin) != 0)
			{
				pinstDigitalIn[u32I] = HIGH;
			}
			else
			{
				pinstDigitalIn[u32I] = LOW;
			}
		}
	}
}
//------------------------------------------------------------------------------
/**
 * @brief <b> Suspensive code</b>: it transfers the actual long long spiWord to
 *  the shift register via SPI2
 */
void TDigitalPort::updateOUT(void)
{
	uint32_t u32I;
	GPIO_InitTypeDef_F4 *GPIO_InitStructure;
	
	if (bClassInitialize != false)
	{
		for (u32I = 0; u32I < DO_NUMEL; u32I++)
		{
			GPIO_InitStructure = (GPIO_InitTypeDef_F4 *)&karGPIO_InitTypeDef[u32I + BASE_GPIO_OUTPUT];
			
			if (pinstDigitalOut[u32I] == HIGH)
			{
				GPIO_InitStructure->GPIO_Port->BSRRL = GPIO_InitStructure->GPIO_Pin;
			}
			else	// LOW
			{
				GPIO_InitStructure->GPIO_Port->BSRRH = (GPIO_InitStructure->GPIO_Pin);
			}
		}
	}
}

//------------------------------------------------------------------------------
/**
 * @brief <b> Suspensive code </b>: Immediately set the shiftRegister output
 *  to previously set value
 * @param 	outputPortNum
 */
void TDigitalPort::setNow(enumDigitalOut outputPortNum)
{
	GPIO_InitTypeDef_F4 *GPIO_InitStructure;
	
	if ((bClassInitialize != false) && (IS_VALID_OUTPUT(outputPortNum)))
	{	
		pinstDigitalOut[outputPortNum] = HIGH;
		GPIO_InitStructure = (GPIO_InitTypeDef_F4 *)&karGPIO_InitTypeDef[outputPortNum + BASE_GPIO_OUTPUT];
		
		GPIO_InitStructure->GPIO_Port->BSRRL = GPIO_InitStructure->GPIO_Pin; 
	}
}
//------------------------------------------------------------------------------
/**
 * @brief <b> Suspensive code </b>: Immediately set all the shiftRegister
 * output to ZERO
 * @param outputPortNum
 */
void TDigitalPort::resetNow(enumDigitalOut outputPortNum)
{
	GPIO_InitTypeDef_F4 *GPIO_InitStructure;
	
	if ((bClassInitialize != false) && (IS_VALID_OUTPUT(outputPortNum)))
	{
		pinstDigitalOut[outputPortNum] = LOW;
		GPIO_InitStructure = (GPIO_InitTypeDef_F4 *)&karGPIO_InitTypeDef[outputPortNum + BASE_GPIO_OUTPUT];
		
		GPIO_InitStructure->GPIO_Port->BSRRH = GPIO_InitStructure->GPIO_Pin; 
	}
}

//------------------------------------------------------------------------------
/**
 * @brief <b> Suspensive code </b>: Immediately toggle all the shiftRegister
 * output 
 * @param outputPortNum
 */
void TDigitalPort::ToggleNow(enumDigitalOut outputPortNum)
{	
	if ((bClassInitialize != false) && (IS_VALID_OUTPUT(outputPortNum)))
	{
		if (check(outputPortNum) == LOW)
		{
			setNow(outputPortNum);
		}		
		else
		{
			resetNow(outputPortNum);
		}
	}
}


//------------------------------------------------------------------------------
/**
 * @brief Reads the shift-register Input Port Number.
 * @remark Value is referred to last historical update (made by
 *  "TDigitalPort::updateIN()" * call)
 * @param inputPortNum		number of the input port to be read
 * @return Input Port Status
 * 						- 0x00 LOW
 * 						- 0x01 HIGH
 */
pinStatus TDigitalPort::check(enumDigitalIn inputPortNum)
{
	pinStatus result;
	
	result = LOW;
	if ((bClassInitialize != false) && (IS_VALID_INPUT(inputPortNum)))
	{
		result = pinstDigitalIn[inputPortNum];
	}
	
	return (result);
}
//------------------------------------------------------------------------------
/**
 * @brief Reads the shift-register Input Port Number.
 * @overload pinStatus TDigitalPort::check(enumDigitalOut inputPortNum)
 * @remark Value is referred to last historical update (made by
 *  "TDigitalPort::updateIN()" call)
 * @param outPortNum		number of the input port to be read
 * @return Input Port Status
 * 						- 0x00 LOW
 * 						- 0x01 HIGH
 */
pinStatus TDigitalPort::check(enumDigitalOut outPortNum)
{
	pinStatus result;
	
	result = LOW;
	if ((bClassInitialize != false) && (IS_VALID_OUTPUT(outPortNum)))
	{
		result = pinstDigitalOut[outPortNum];
	}
	
	return result;
}

//------------------------------------------------------------------------------
/**
 * @brief Prepare the output port to be set HIGH.
 * @remark The effectiveness will come after TDigitalPort::updateOUT(void) call
 * @param outputPortNum	 number of the output port to be set
 */
void TDigitalPort::set(enumDigitalOut outputPortNum)
{
	if ((bClassInitialize != false) && (IS_VALID_OUTPUT(outputPortNum)))
	{	
		pinstDigitalOut[outputPortNum] = HIGH;
	}
}

//------------------------------------------------------------------------------
/**
 * @brief Prepare the output port to be set LOW.
 * @remark The effectiveness will come after TDigitalPort::updateOUT(void) call
 * @param outputPortNum		number of the output port to be set
 */
void TDigitalPort::reset(enumDigitalOut outputPortNum)
{
	if ((bClassInitialize != false) && (IS_VALID_OUTPUT(outputPortNum)))
	{
		pinstDigitalOut[outputPortNum] = LOW;
	}
}

//------------------------------------------------------------------------------
/**
 * @brief Prepare the output port to be set LOW or HIGH.
 * @remark The effectiveness will come after TDigitalPort::updateOUT(void) call
 * @param outputPortNum		number of the output port to be set
 */
void TDigitalPort::assign(enumDigitalOut outputPortNum, pinStatus lowOrHigh)
{
	if ((bClassInitialize != false) && (IS_VALID_OUTPUT(outputPortNum)))
	{
		if (lowOrHigh == LOW)
		{
			reset(outputPortNum);
		}
		else //  (lowOrHigh == 'HIGH')
		{
			set(outputPortNum);
		}
	}
}

//------------------------------------------------------------------------------
/**
 * @brief Set the given fake input as HIGH
 * @param inputPortNum	 number of the fake input port to be set, needs to be a value between DI_LAST_REAL and DI_NUMEL
 */
void TDigitalPort::set(enumDigitalIn inputPortNum)
{
	if ((bClassInitialize != false) && (IS_FAKE_INPUT(inputPortNum)))
	{
		pinstDigitalIn[inputPortNum] = HIGH;
	}
}

//------------------------------------------------------------------------------
/**
 * @brief Set the given fake input as LOW
 * @param inputPortNum	 number of the fake input port to be reset, needs to be a value between DI_LAST_REAL and DI_NUMEL
 */
void TDigitalPort::reset(enumDigitalIn inputPortNum)
{
	if ((bClassInitialize != false) && (IS_FAKE_INPUT(inputPortNum)))
	{
		pinstDigitalIn[inputPortNum] = LOW;
	}
}

//------------------------------------------------------------------------------
/**
 * @brief Set the given fake input to given value (LOW or HIGH)
 * @param inputPortNum	 number of the fake input port to be set, needs to be a value between DI_LAST_REAL and DI_NUMEL
 */
void TDigitalPort::assign(enumDigitalIn inputPortNum, pinStatus lowOrHigh)
{
	if ((bClassInitialize != false) && (IS_FAKE_INPUT(inputPortNum)))
	{
		if (lowOrHigh == LOW)
		{
			reset(inputPortNum);
		}
		else //  (lowOrHigh == 'HIGH')
		{
			set(inputPortNum);
		}
	}
}

//------------------------------------------------------------------------------
/**
 * @brief <b>NON Suspensive code</b>: it wipes the output status message
 * @remark The effectiveness will come after
 * TDigitalPort::setNow(enumDigitalOut outputPortNum) call
 */
void TDigitalPort::resetAllOUT(void)
{
	uint32_t u32I;
	
	if (bClassInitialize != false)
	{
		for (u32I=0; u32I < DI_NUMEL; u32I++)	
		{
			pinstDigitalIn[u32I] = LOW;
		}
		for (u32I=0; u32I < DO_NUMEL; u32I++)	
		{
			pinstDigitalOut[u32I] = LOW;
		}
	}
}
