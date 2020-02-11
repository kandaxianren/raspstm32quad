#include    "stm32f10x.h"
#include    "stm32f10x_dma.h"
#include    "stm32f10x_adc.h"
__IO u16 ADC1result[4];
void DMAinit()
{
		DMA_InitTypeDef DMA_InitStructure;

	 
		DMA_InitStructure.DMA_PeripheralBaseAddr =((u32)0x40012400+0x4c);//要设置所读寄存器地址  外设

		DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&ADC1result;
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	  DMA_InitStructure.DMA_BufferSize = 4;
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
		DMA_InitStructure.DMA_PeripheralDataSize =DMA_PeripheralDataSize_HalfWord;
		DMA_InitStructure.DMA_MemoryDataSize =DMA_MemoryDataSize_HalfWord;
		DMA_InitStructure.DMA_Mode = DMA_Mode_Circular ;
		DMA_InitStructure.DMA_Priority = DMA_Priority_High ;
		DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
		DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	
	  DMA_Cmd(DMA1_Channel1, ENABLE);
}
void ADxinit()
{

	ADC_InitTypeDef ADC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
		
		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
	GPIO_Init(GPIOB, &GPIO_InitStructure);	
GPIO_WriteBit(GPIOB, GPIO_Pin_10|GPIO_Pin_11, Bit_SET);

		
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 4;
	ADC_Init(ADC1,&ADC_InitStructure);
		
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1,	ADC_SampleTime_55Cycles5);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2,	ADC_SampleTime_55Cycles5);	
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3,	ADC_SampleTime_55Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 4,	ADC_SampleTime_55Cycles5);

	RCC_ADCCLKConfig(RCC_PCLK2_Div6);	
	ADC_Cmd(ADC1, ENABLE);	
	ADC_DMACmd(ADC1, ENABLE);
		
	ADC_ResetCalibration(ADC1);
	while(ADC_GetResetCalibrationStatus(ADC1));
	ADC_StartCalibration(ADC1);
	while(ADC_GetCalibrationStatus(ADC1));
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

void keyscan()
{
  if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_10)==0)
	{}
	else{}
	if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11)==0)
  {}
	else{}
}