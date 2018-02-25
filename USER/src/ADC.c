/******************************************************************************

                  ��Ȩ���� (C), 1994-2016, ���ݾ���ŷ�϶�С�ҵ����޹�˾

 ******************************************************************************
  �� �� ��   : ADC.c
  �� �� ��   : V1.0
  ��    ��   : ���ͼ
  ��������   : 2016��3��15��
  ����޸�   :
  ��������   : ADC���ü�����
  �����б�   :
  �޸���ʷ   :
  1.��    ��   : 2016��3��15��
    ��    ��   : ���ͼ
    �޸�����   : �����ļ�

******************************************************************************/

/*----------------------------------------------*
 * ����ͷ�ļ�                                   *
 *----------------------------------------------*/
#include "stm32f10x.h"
/*----------------------------------------------*
 * �ⲿ����˵��                                 *
 *----------------------------------------------*/

/*----------------------------------------------*
 * �ⲿ����ԭ��˵��                             *
 *----------------------------------------------*/
void ADC_Conf(void);

/*----------------------------------------------*
 * �ڲ�����ԭ��˵��                             *
 *----------------------------------------------*/
uint16_t Get_Average(uint16_t * Data_Adr, uint8_t Data_Num);
uint16_t Get_DataDiffABS(uint16_t Num1,uint16_t Num2);


/*----------------------------------------------*
 * ȫ�ֱ���                                     *
 *----------------------------------------------*/
uint16_t Batt_Vol,Batt_Temp,Batt_ChargeCur,ZSMotor_Cur,DustMotor_Cur,LMotor_Cur,RMotor_Cur;
FunctionalState ADCScanEn,ADCDataValid[ADC_Channel_Num];

/*----------------------------------------------*
 * ģ�鼶����                                   *
 *----------------------------------------------*/
///ch0->PSD
///ch1->��ؼ�
///ch2->�ҵؼ�
///ch3->
///ch4->������
///ch5->�ŵ����
///ch6->��ص�ѹ
///ch7->
///ch14->
///ch15->
///ch8->
///ch9->


///ch10->��ɨ�������
///ch11->���ֵ������
///ch12->���ֵ������
///ch13->�����������

#define ADC_Check_Value_Num	6

///
uint16_t ADC_ConvertedValue[ADC_Channel_Num];
uint16_t ADC_ConvertedValue1[ADC_Channel_Num][ADC_Check_Value_Num];
uint16_t ADC_ConvertedValue_New[ADC_Channel_Num];
uint16_t ADC_ConvertedValue_Old[ADC_Channel_Num];
uint16_t ADC_ConvertedValue_Val[ADC_Channel_Num];

uint8_t ADC_Ready;

/*----------------------------------------------*
 * ��������                                     *
 *----------------------------------------------*/




/*****************************************************************************
 �� �� ��  : ADC_Conf
 ��������  : ADC��ʼ��
 �������  : void  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2016��3��15��
    ��    ��   : ���ͼ
    �޸�����   : �����ɺ���

*****************************************************************************/
void ADC_Conf(void)
{
	ADC_InitTypeDef ADC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	uint8_t i;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_AFIO, ENABLE);
	
	/* DMA channel1 configuration ----------------------------------------------*/
	DMA_DeInit(DMA1_Channel1);
	DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)ADC_ConvertedValue;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = ADC_Channel_Num;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);

	/* Enable DMA1 channel1 */
	DMA_Cmd(DMA1_Channel1, ENABLE);

	/* ADC1 configuration ------------------------------------------------------*/
	//ADC����������ɨ��ģʽ������ģʽ�������Ҷ��롢���ⲿ��������
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = ADC_Channel_Num;
	ADC_Init(ADC1, &ADC_InitStructure);

	/* ADC1 regular channel configuration */ 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 1, ADC_SampleTime_1Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_1Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 3, ADC_SampleTime_1Cycles5);


	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 4, ADC_SampleTime_1Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 5, ADC_SampleTime_1Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 6, ADC_SampleTime_1Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 7, ADC_SampleTime_1Cycles5);
	
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 8, ADC_SampleTime_1Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 9, ADC_SampleTime_1Cycles5);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 10, ADC_SampleTime_1Cycles5);


	/* Enable ADC1 DMA */
	ADC_DMACmd(ADC1, ENABLE);

	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);

	/* Enable ADC1 reset calibaration register */   
	ADC_ResetCalibration(ADC1);
	/* Check the end of ADC1 reset calibration register */
	while(ADC_GetResetCalibrationStatus(ADC1));

	/* Start ADC1 calibaration */
	ADC_StartCalibration(ADC1);
	/* Check the end of ADC1 calibration */
	while(ADC_GetCalibrationStatus(ADC1));

	/* Start ADC1 Software Conversion */ 
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);

	for(i = 0 ;i < ADC_Channel_Num ; i++)
	{
		ADCDataValid[i] = DISABLE;	
	}
	
	ADC_Ready = 0;
}


/*****************************************************************************
 �� �� ��  : ADC_Deal
 ��������  : ADC���ݴ���
 �������  : void  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 

 1.20msɨ��һ��
 
 �޸���ʷ      :
  1.��    ��   : 2016��4��21��
    ��    ��   : ���ͼ
    �޸�����   : �����ɺ���

*****************************************************************************/
void ADC_Deal(void)
{
	uint8_t i,j;
    uint16_t Temp_Data;
	
	if(!ADCScanEn)
		return;
	ADCScanEn = DISABLE;
	
	for(i = 0 ; i < ADC_Check_Value_Num ; i++)
	{	
		//�ȴ�ת�����
//		while(ADC_GetFlagStatus(ADC1 , ADC_FLAG_EOC) != SET);
		
		for(j = 0 ; j < ADC_Channel_Num ; j++)
		{
			ADC_ConvertedValue1[j][i] = ADC_ConvertedValue[j];	
		}
	}
	
	for(i = 0 ; i < ADC_Channel_Num ; i++)
	{
		ADC_ConvertedValue_New[i] = Get_Average(ADC_ConvertedValue1[i],ADC_Check_Value_Num);
        Temp_Data = Get_DataDiffABS(ADC_ConvertedValue_New[i] , ADC_ConvertedValue_Old[i]);
		if(Temp_Data <= ADC_DELT)
		{
			ADC_ConvertedValue_Val[i] = ADC_ConvertedValue_New[i];
			
			if(ADC_ConvertedValue_Val[i] != 0 && ADC_Ready == 0)
				ADC_Ready = 1;	
			
			if(ADC_Ready == 1)
				ADCDataValid[i] = ENABLE;
		}
		ADC_ConvertedValue_Old[i] = ADC_ConvertedValue_New[i];
	}
	
	
	//����ת��
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);

}


/*****************************************************************************
 �� �� ��  : Get_Average
 ��������  : ������ƽ��ֵ
 �������  : uint16_t * Data_Adr  --�����׵�ַ
             uint8_t Data_Num    ---�������ݸ��� 
 �������  : ƽ��ֵ
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2016��3��16��
    ��    ��   : ���ͼ
    �޸�����   : �����ɺ���

*****************************************************************************/
uint16_t Get_Average(uint16_t * Data_Adr, uint8_t Data_Num)
{
	uint32_t Data_Sum;
	uint16_t Data_Max,Data_Min,Data_Res;
	uint8_t i;
	
	Data_Max = 0x0000;
	Data_Min = 0xffff;
	Data_Sum = 0x0000;

	if(Data_Num <= 2)
	{
		return 0;
	}
	
	for(i = 0; i < Data_Num;i++)
	{
		if(Data_Adr[i] > Data_Max)
		{
			Data_Max = Data_Adr[i];
		}
		if(Data_Adr[i] < Data_Min)
		{
			Data_Min = Data_Adr[i];
		}
		Data_Sum += Data_Adr[i];
	}

	Data_Sum -= Data_Min;
	Data_Sum -= Data_Max;
	Data_Res  = Data_Sum / (Data_Num - 2);
	return Data_Res;

}

/*****************************************************************************
 �� �� ��  : Get_DataDiffABS
 ��������  : ��2������ľ���ֵ
 �������  : uint16_t Num1  
             uint16_t Num2  
 �������  : ��ľ���ֵ
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2016��3��16��
    ��    ��   : ���ͼ
    �޸�����   : �����ɺ���

*****************************************************************************/
uint16_t Get_DataDiffABS(uint16_t Num1 , uint16_t Num2)
{
	if(Num1 > Num2)
	{
		return (Num1 - Num2);
	}
	else
	{
		return (Num2 - Num1);
	}
}

