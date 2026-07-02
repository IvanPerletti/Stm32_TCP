
#include "TAnalogPort.h"

//Private Inclusions ----------------------------------------------------------
extern "C" {
#include "stm32f4xx_adc.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
}

TAnalogPort tAnalogPort; // class definition

//_^_^_^_^_^_^_^_^_^_^_^_^_^_^_^_^_^_^_^_^_^_^_^_^_^_^_^_^_^_^_^_^_^_^_^_^_^_^_
// Class methods
TAnalogPort::TAnalogPort(void)
: bIsOpen (0)
{
	for(int ii=0; ii< ANALOG_NUMEL; ii++)
		u16aAdcValue[ii]=0;
	kk=0;
	init();
}
//-----------------------------------------------------------------------------
TAnalogPort::~TAnalogPort(void)
{
	deInit();
	bIsOpen = 0;
}
//-----------------------------------------------------------------------------
void TAnalogPort::read ( unsigned short * pu16Arr )
{
	int ii;
	if ( pu16Arr != 0 ) 
	{
		for (ii=0; ii<ANALOG_NUMEL; ii++)	{
			pu16Arr[ii] = u16aAdcValue[ii];
		}
	}
}
//-----------------------------------------------------------------------------
unsigned short TAnalogPort::read ( enumAnalogPort portNum )
{
	short u16Position = portNum;
	assert_param(IS_ANALOG_PORT(portNum));
	return (u16aAdcValue[u16Position]);
	// CONVERSION EXAMPLE: u32data = (u32data++*3000)>>12;
}

//-----------------------------------------------------------------------------
void TAnalogPort::deInit(void)
{
	close();
	DMA_DeInit(DMA2_Stream2); // please refers to DM310020 -> "DMA2 request mapping"
	ADC_DeInit();
	//	don't type GPIO_DeInit: it will de init other Pins Functions'
}
//-----------------------------------------------------------------------------
void TAnalogPort::open(void)
{
	ADC_DMACmd(ADC2, ENABLE);	/* Enable ADC2 DMA */
	bIsOpen = true;

}
//-----------------------------------------------------------------------------
void TAnalogPort::close(void)
{
	ADC_DMACmd(ADC2, DISABLE);	/* Enable ADC2 DMA */
	bIsOpen = false;

}

bool TAnalogPort::isOpen(void)  {return (bIsOpen);}
//Private Functions -----------------------------------------------------------
/**
 * @brief Configuration set for Dynamic Memory Access, GPIO and ADC
 */
void TAnalogPort::init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	deInit();
	/* Enable ADC2, DMA2 and GPIO clocks ***************/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2 | RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);
	configDMA();

	/* Configure ADC2 Channel12 pin as analog input *****/
	GPIO_InitStructure.GPIO_Pin =
			GPIO_Pin_0 |
			GPIO_Pin_3 |
			GPIO_Pin_4 |
			GPIO_Pin_5 |
			GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin =	GPIO_Pin_0	|	GPIO_Pin_1;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	configADC();
	/* Enable DMA request after last transfer (Single-ADC mode) */
	ADC_DMARequestAfterLastTransferCmd(ADC2, ENABLE);
	open();

	/* Enable ADC2 */
	ADC_Cmd(ADC2, ENABLE);
	ADC_SoftwareStartConv(ADC2);
}
//-----------------------------------------------------------------------------
/**
 * @brief Configuration function for Direct Memory Access 2
 */
void TAnalogPort::configDMA(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	/* DMA2 Stream0 channel0 configuration ****/
	DMA_DeInit(DMA2_Stream2); // please refers to DM310020 -> "DMA2 request mapping"
	DMA_InitStructure.DMA_Channel = DMA_Channel_1;// please refers to DM310020 -> "DMA2 request mapping"
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) (uint32_t)&ADC2->DR;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t) & u16aAdcValue[0];
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = ANALOG_NUMEL;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; // orig dis
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; //orig dis
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream2, &DMA_InitStructure);
	DMA_Cmd(DMA2_Stream2, ENABLE);
}
//------------------------------------------------------------------------------
/**
 * @brief Configuration for GPIO to become Analog Input
 */
void TAnalogPort::configGpioForADC(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	/* Configure ADC2 Channel12 pin as analog input *****/
	GPIO_InitStructure.GPIO_Pin =
			GPIO_Pin_0 |
			GPIO_Pin_3 |
			GPIO_Pin_4 |
			GPIO_Pin_5 |
			GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure); // PA0, PA3, PA4, PA5, PA6

	GPIO_InitStructure.GPIO_Pin =	GPIO_Pin_0	|	GPIO_Pin_1;
	GPIO_Init(GPIOB, &GPIO_InitStructure);// PB0, PB1
}
//------------------------------------------------------------------------------
/**
 * @brief Configuration for ADC
 */
void TAnalogPort::configADC(void)
{
	ADC_InitTypeDef ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;

	/* ADC Common Init ****/
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_15Cycles;
	ADC_CommonInit(&ADC_CommonInitStructure);

	/* ADC2 Init ****/
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE; //orig disable
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = ANALOG_NUMEL;
	ADC_Init(ADC2, &ADC_InitStructure);

	/* ADC2 regular channel12 configuration ****/
	ADC_RegularChannelConfig(ADC2, ADC_Channel_0, 1, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_3, 2, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_4, 3, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_5, 4, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_6, 5, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_8, 6, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_9, 7, ADC_SampleTime_15Cycles);
}


