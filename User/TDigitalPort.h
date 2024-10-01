/******************************************************************************************
 *
 *  \brief              DIGITAL PORT Class
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

#ifndef TDIGITALPORT_H_
#define TDIGITALPORT_H_

/* Includes ------------------------------------------------------------------*/

#include "stm32f4xx_gpio.h"

/* XMACRO INPUT ---------------------------------------------------------------*/
#define GPIO_INIT_INPUT	\
X(DI_PC6       ,	GPIOC , GPIO_Pin_6 ,	GPIO_PuPd_DOWN)


#define GPIO_INIT_INPUT_F	\
X(DI_PREP)



/* XMACRO OUTPUT --------------------------------------------------------------*/
#define GPIO_INIT_OUTPUT	\
X(DO_PC8         	, GPIOC 	, GPIO_Pin_8  	,	GPIO_Speed_2MHz,	GPIO_OType_PP,	GPIO_PuPd_NOPULL)	\
X(DO_PC9	        , GPIOC 	, GPIO_Pin_9  	,	GPIO_Speed_2MHz,	GPIO_OType_PP,	GPIO_PuPd_NOPULL)

/* XMACRO EXTI --------------------------------------------------------------*/
#define GPIO_INIT_EXTI
#undef  GPIO_INIT_EXTI

/* Typedef -------------------------------------------------------------------*/
/** 
  * @brief  GPIO TYPE
  */   
typedef enum
{ 
  GPIO_INPUT   	 = 0x00, 
  GPIO_OUTPUT  	 = 0x01
}GPIOType;
#define IS_GPIO_TYPE(TYPE) (((TYPE) == GPIO_INPUT) || ((TYPE) == GPIO_OUTPUT))

/**
 * @brief Pin status for digital input and output:
 * 			- LOW value
 * 			- HIGH value
 */
typedef enum {LOW = 0, HIGH = 1, NOPE =-1 } pinStatus;



/** 
  * @brief   GPIO Init structure definition  
  */ 
typedef struct
{
	GPIOType	GPIO_Type;						/*!< Specifies the GPIO Type */
	
	GPIO_TypeDef*	GPIO_Port;				/*!< Specifies the GPIO port to be configured. */
	
  uint32_t GPIO_Pin;              /*!< Specifies the GPIO pins to be configured.
                                       This parameter can be any value of @ref GPIO_pins_define */
	
	uint32_t RCC_AHB1Periph_GPIOx;	/*!< Specifies the RCC_AHB1_Peripherals */

  GPIOMode_TypeDef GPIO_Mode;     /*!< Specifies the operating mode for the selected pins.
                                       This parameter can be a value of @ref GPIOMode_TypeDef */

  GPIOSpeed_TypeDef GPIO_Speed;   /*!< Specifies the speed for the selected pins.
                                       This parameter can be a value of @ref GPIOSpeed_TypeDef */

  GPIOOType_TypeDef GPIO_OType;   /*!< Specifies the operating output type for the selected pins.
                                       This parameter can be a value of @ref GPIOOType_TypeDef */

  GPIOPuPd_TypeDef GPIO_PuPd;     /*!< Specifies the operating Pull-up/Pull down for the selected pins.
                                       This parameter can be a value of @ref GPIOPuPd_TypeDef */		
} GPIO_InitTypeDef_F4;


/** 
  * @brief   GPIO Init structure definition  
  */ 
typedef struct
{	
	uint32_t 	EXTI_PortSource;			/*!< Specifies the EXTI Port */
	
	uint32_t	EXTI_PinSource;				/*!< Specifies the EXTI Pin */
	
  uint32_t EXTI_Line;               /*!< Specifies the EXTI lines to be enabled or disabled.
                                         This parameter can be any combination value of @ref EXTI_Lines */
   
  EXTIMode_TypeDef EXTI_Mode;       /*!< Specifies the mode for the EXTI lines.
                                         This parameter can be a value of @ref EXTIMode_TypeDef */

  EXTITrigger_TypeDef EXTI_Trigger; /*!< Specifies the trigger signal active edge for the EXTI lines.
                                         This parameter can be a value of @ref EXTITrigger_TypeDef */

  FunctionalState EXTI_LineCmd;     /*!< Specifies the new state of the selected EXTI lines.
                                         This parameter can be set either to ENABLE or DISABLE */					
}EXTI_InitTypeDef_F4;



/**
 * @brief Input pin enumeration mapping
 */
typedef enum {
	#define X(NAME, PORT, PIN, PP)	NAME,
		GPIO_INIT_INPUT
	#undef X
		DI_LAST_REAL,
#define X(NAME)	NAME##_FAKE,
	GPIO_INIT_INPUT_F
#undef X
	DI_NUMEL
} enumDigitalIn;
#define IS_VALID_INPUT(INPUT) (INPUT < DI_NUMEL)
#define IS_REAL_INPUT(INPUT) (INPUT < DI_LAST_REAL)
#define IS_FAKE_INPUT(INPUT) ( ( DI_LAST_REAL < (INPUT) ) && ( (INPUT) < DI_NUMEL ) )

/**
 * @brief Output pin enumeration mapping
 */
typedef enum {
	#define X(NAME, PORT, PIN, SPEED, OTYPE, PP)	NAME,	
		GPIO_INIT_OUTPUT
	#undef X
	DO_NUMEL
}enumDigitalOut;
#define IS_VALID_OUTPUT(OUTPUT) (OUTPUT < DO_NUMEL)

//-----------------------------------------------------------------------------

class TDigitalPort {

	public:
	TDigitalPort();
	~TDigitalPort();
	
	void init(void);
	void deInit(void);
	
	void updateIN(void);
	void updateOUT(void);
	
	void setNow(enumDigitalOut outputPortNum);
	void resetNow(enumDigitalOut outputPortNum);	
	void ToggleNow(enumDigitalOut outputPortNum);
	pinStatus check(enumDigitalIn inputPortNum);
	pinStatus check(enumDigitalOut inputPortNum);	
	void set(enumDigitalOut outputPortNum);
	void reset(enumDigitalOut outputPortNum);	
	void assign(enumDigitalOut outputPortNum, pinStatus lowOrHigh);
	void set(enumDigitalIn inputPortNum);
	void reset(enumDigitalIn inputPortNum);
	void assign(enumDigitalIn inputPortNum, pinStatus lowOrHigh);
	void resetAllOUT(void);

	private:
	bool bClassInitialize;
	pinStatus pinstDigitalIn[DI_NUMEL];
	pinStatus pinstDigitalOut[DO_NUMEL];	
};

extern TDigitalPort digitalPort;
#endif
