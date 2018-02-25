/******************************************************************************

                  版权所有 (C), 1994-2016, 杭州九阳欧南多小家电有限公司

 ******************************************************************************
  文 件 名   : ADC.c
  版 本 号   : V1.0
  作    者   : 田宏图
  生成日期   : 2016年3月15日
  最近修改   :
  功能描述   : ADC配置及操作
  函数列表   :
  修改历史   :
  1.日    期   : 2016年3月15日
    作    者   : 田宏图
    修改内容   : 创建文件

******************************************************************************/

/*----------------------------------------------*
 * 包含头文件                                   *
 *----------------------------------------------*/
#include "stm32f10x.h"
/*----------------------------------------------*
 * 外部变量说明                                 *
 *----------------------------------------------*/

/*----------------------------------------------*
 * 外部函数原型说明                             *
 *----------------------------------------------*/
void ADC_Conf(void);

/*----------------------------------------------*
 * 内部函数原型说明                             *
 *----------------------------------------------*/
uint16_t Get_Average(uint16_t * Data_Adr, uint8_t Data_Num);
uint16_t Get_DataDiffABS(uint16_t Num1,uint16_t Num2);


/*----------------------------------------------*
 * 全局变量                                     *
 *----------------------------------------------*/
uint16_t Batt_Vol,Batt_Temp,Batt_ChargeCur,ZSMotor_Cur,DustMotor_Cur,LMotor_Cur,RMotor_Cur;
FunctionalState ADCScanEn,ADCDataValid[ADC_Channel_Num];

/*----------------------------------------------*
 * 模块级变量                                   *
 *----------------------------------------------*/
///ch0->PSD
///ch1->左地检
///ch2->右地检
///ch3->
///ch4->充电电流
///ch5->放电电流
///ch6->电池电压
///ch7->
///ch14->
///ch15->
///ch8->
///ch9->


///ch10->中扫电机电流
///ch11->左轮电机电流
///ch12->右轮电机电流
///ch13->吸尘电机电流

#define ADC_Check_Value_Num	6

///
uint16_t ADC_ConvertedValue[ADC_Channel_Num];
uint16_t ADC_ConvertedValue1[ADC_Channel_Num][ADC_Check_Value_Num];
uint16_t ADC_ConvertedValue_New[ADC_Channel_Num];
uint16_t ADC_ConvertedValue_Old[ADC_Channel_Num];
uint16_t ADC_ConvertedValue_Val[ADC_Channel_Num];

uint8_t ADC_Ready;

/*----------------------------------------------*
 * 常量定义                                     *
 *----------------------------------------------*/




/*****************************************************************************
 函 数 名  : ADC_Conf
 功能描述  : ADC初始化
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2016年3月15日
    作    者   : 田宏图
    修改内容   : 新生成函数

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
	//ADC独立工作、扫描模式、单次模式、数据右对齐、无外部触发工作
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
 函 数 名  : ADC_Deal
 功能描述  : ADC数据处理
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 

 1.20ms扫描一次
 
 修改历史      :
  1.日    期   : 2016年4月21日
    作    者   : 田宏图
    修改内容   : 新生成函数

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
		//等待转换完成
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
	
	
	//开启转换
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);

}


/*****************************************************************************
 函 数 名  : Get_Average
 功能描述  : 数组求平均值
 输入参数  : uint16_t * Data_Adr  --数组首地址
             uint8_t Data_Num    ---数组数据个数 
 输出参数  : 平均值
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2016年3月16日
    作    者   : 田宏图
    修改内容   : 新生成函数

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
 函 数 名  : Get_DataDiffABS
 功能描述  : 求2个数差的绝对值
 输入参数  : uint16_t Num1  
             uint16_t Num2  
 输出参数  : 差的绝对值
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2016年3月16日
    作    者   : 田宏图
    修改内容   : 新生成函数

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

