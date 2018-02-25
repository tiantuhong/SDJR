/******************************************************************************

                  版权所有 (C), 1994-2015, 杭州九阳欧南多小家电有限公司

 ******************************************************************************
  文 件 名   : Init.c
  版 本 号   : V1.0
  作    者   : 田宏图
  生成日期   : 2016年3月11日
  最近修改   :
  功能描述   : 设备初始化及中断
  函数列表   :
              Device_Init
              EXIT_Init
              EXTI4_IRQHandler
              EXTI9_5_IRQHandler
              GPIO_Init
              TIM1_Init
              TIM3_Init
              TIM6_Init
  修改历史   :
  1.日    期   : 2016年3月11日
    作    者   : 田宏图
    修改内容   : 创建文件

******************************************************************************/

/*----------------------------------------------*
 * 包含头文件                                   *
 *----------------------------------------------*/
#include "stm32f10x.h"
#include "imu.h"
/*----------------------------------------------*
 * 外部变量说明                                 *
 *----------------------------------------------*/

/*----------------------------------------------*
 * 外部函数原型说明                             *
 *----------------------------------------------*/
void Device_Init( void );
/*----------------------------------------------*
 * 内部函数原型说明                             *
 *----------------------------------------------*/
void GPIO_Conf(void);
void EXIT_Conf(void);
void TIM4_Conf(void);
void TIM3_Conf(void);
void TIM6_Conf(void);
void TIM2_Conf(void);
void TIM1_Conf(void);
void TIM7_Conf(void);

/*----------------------------------------------*
 * 全局变量                                     *
 *----------------------------------------------*/
uint32_t SystemRunTime; //系统运行时间单位ms
uint8_t SystePwrMode; //系统电源模式 0-正常模式，1-stop模式
/*----------------------------------------------*
 * 模块级变量                                   *
 *----------------------------------------------*/



void GetLockCode(void)
{
    static u32 CpuID[3];
    //??CPU??ID
    CpuID[0]=*(vu32*)(0x1ffff7e8);
    CpuID[1]=*(vu32*)(0x1ffff7ec);
    CpuID[2]=*(vu32*)(0x1ffff7f0);
}

/*----------------------------------------------*
 * 常量定义                                     *
 *----------------------------------------------*/

/*----------------------------------------------*
 * 宏定义                                       *
 *----------------------------------------------*/






/*****************************************************************************
 函 数 名  : Device_Init
 功能描述  : 设备初始化
 输入参数  : void
 输出参数  : 无
 返 回 值  : void
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2016年3月11日
    作    者   : 田宏图
    修改内容   : 新生成函数

*****************************************************************************/
void Device_Init( void )
{
//    float ret;
//    ret = ANGLE2MILEAGE;
    SystemInit();//HSI /2 *16  = 8/2*16=64MHz
    GPIO_Conf();//I/O初始化
    
    _5V_OUT_ON;
    _3V3_OUT_ON;
    
    Gyro_Conf();
    
    LED1(1); //上电LED1亮一下
    
	EXIT_Conf();//外部中断初始化
	TIM4_Conf();
	TIM3_Conf();
	TIM6_Conf();
	TIM2_Conf();
    TIM1_Conf();
    TIM7_Conf();
    ADC_Conf();
	UART_Conf();
	Motor_Conf();
	RobitMode_Conf();
	Signal_Conf();
	Battery_Conf();
	Actuator_Conf();
	Display_Conf();
//	motion_init();
//    Voice_Init();
//    GetLockCode();
//    IMU_Init();
    
    HandshakeInit();
    
   

	SystemRunTime = 0;
    SystePwrMode = 0;

//	LED4(1);
	
}
/*****************************************************************************
 函 数 名  : GPIO_Conf
 功能描述  : GPIO初始化
 输入参数  : void
 输出参数  : 无
 返 回 值  : void
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2016年3月8日
    作    者   : 田宏图
    修改内容   : 新生成函数

*****************************************************************************/
void GPIO_Conf( void )
{
	GPIO_InitTypeDef GPIO_InitStructure;
 
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE |RCC_APB2Periph_AFIO, ENABLE); 						 

////////////////////GPIOA////////////////////////////////////////////////////////////////////
	//模拟输入		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6;   
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;                                        
	GPIO_Init(GPIOA, &GPIO_InitStructure);


	//推挽输出 (3/7/8/11/12 暂未使用，设置为推挽输出)
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_15;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
  	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/*
	*  USART1_TX -> PA9 , USART1_RX ->PA10
	*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;	         
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);		   

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;	        
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	
///////////////////GPIOA////////////////////////////////////////////////////////////////////


///////////////////GPIOB////////////////////////////////////////////////////////////////////

	//模拟输入		
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;   
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;                                        
//	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	//上拉输入
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_4 | GPIO_Pin_5;// | GPIO_Pin_0;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
  	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//下拉输入
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
//  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 
//  	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	//推挽输出
	GPIO_InitStructure.GPIO_Pin =   GPIO_Pin_3 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15 | GPIO_Pin_12;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
  	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//复用推挽输出TIM4
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 ;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
  	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//复用开漏输出I2C2
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
		
///////////////////GPIOB////////////////////////////////////////////////////////////////////


///////////////////GPIOC////////////////////////////////////////////////////////////////////

	//模拟输入		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; 
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	//上拉输入
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
//  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
//  	GPIO_Init(GPIOC, &GPIO_InitStructure);

	//推挽输出
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_4 /*| GPIO_Pin_5*/;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
  	GPIO_Init(GPIOC, &GPIO_InitStructure);	

	//复用推挽输出TIM3_CH3   TIM3_CH4
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_6 | GPIO_Pin_7;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
  	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	//TIM3_CH1 -> PC6 TIM3_CH2 -> PC7 TIM3_CH3 -> PC8 TIM3_CH4 -> PC9
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM3 , ENABLE);

///////////////////GPIOC////////////////////////////////////////////////////////////////////


///////////////////GPIOD////////////////////////////////////////////////////////////////////

	//浮空输入		
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
//	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	//推挽输出
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
  	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	
	//上拉输入
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_6;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
  	GPIO_Init(GPIOD, &GPIO_InitStructure);
    
    //复用推挽输出
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
  	GPIO_Init(GPIOD, &GPIO_InitStructure);
    
	//USART2-> PD5/PD6
	GPIO_PinRemapConfig(GPIO_Remap_USART2,ENABLE);

///////////////////GPIOD////////////////////////////////////////////////////////////////////


///////////////////GPIOE////////////////////////////////////////////////////////////////////


	//推挽输出
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_7 ;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
  	GPIO_Init(GPIOE, &GPIO_InitStructure);

	//上拉输入
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_3 | GPIO_Pin_8 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
  	GPIO_Init(GPIOE, &GPIO_InitStructure);
    
    //浮空输入		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
	GPIO_Init(GPIOE, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 ;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
  	GPIO_Init(GPIOE, &GPIO_InitStructure);
    GPIO_PinRemapConfig(GPIO_FullRemap_TIM1, ENABLE);
	
///////////////////GPIOE////////////////////////////////////////////////////////////////////
	
}

void GPIO_Conf_StopMode(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    //模拟输入		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;   
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;                                        
	GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    GPIO_Init(GPIOE, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
  	GPIO_Init(GPIOE, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
  	GPIO_Init(GPIOD, &GPIO_InitStructure);
}

/*****************************************************************************
 函 数 名  : EXIT_Conf
 功能描述  : 外部中断初始化: PB5/PC10/PE4  上拉输入 上升沿中断
 输入参数  : void
 输出参数  : 无
 返 回 值  : void
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2016年3月8日
    作    者   : 田宏图
    修改内容   : 新生成函数

*****************************************************************************/
void EXIT_Conf( void )
{
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); 

	// 加速度计中断
//    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource0);
    
    // 左轮测速中断
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource1);

	//陀螺仪中断
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource3);
    
    // 拖布位置中断
//    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource4);

	//右轮测速中断
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource5);

	//超声波测距中断
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource2);

	//红外信号接收中断
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource12); 
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource13);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource14);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource15);
	
//	EXTI_ClearITPendingBit(EXTI_Line0);
    EXTI_ClearITPendingBit(EXTI_Line1);
    EXTI_ClearITPendingBit(EXTI_Line2);
	EXTI_ClearITPendingBit(EXTI_Line3);
	EXTI_ClearITPendingBit(EXTI_Line5);
	EXTI_ClearITPendingBit(EXTI_Line12);
	EXTI_ClearITPendingBit(EXTI_Line13);
	EXTI_ClearITPendingBit(EXTI_Line14);
	EXTI_ClearITPendingBit(EXTI_Line15);


	//左右轮测速中断，双边沿中断
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;//EXTI_Trigger_Falling;//
	EXTI_InitStructure.EXTI_Line = EXTI_Line1 | EXTI_Line5;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	//陀螺仪中断，下降沿中断
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_Line = EXTI_Line3;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
    
    //加速度计中断，上升沿中断
//	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
//	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
//	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
//	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
//	EXTI_Init(&EXTI_InitStructure);

//	//超声波测距中断，双边沿中断
//	//红外信号接收双边沿中断
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_Line = EXTI_Line2 | EXTI_Line12 | EXTI_Line13 | EXTI_Line14 | EXTI_Line15;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
    
    
	//超声波中断优先级最高
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  													
	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;   
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;   
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;	      
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	//测速中断优先级次高
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  													
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;   //EXTI1_IRQn | 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;   
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;	      
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  													
	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;   //EXTI1_IRQn | 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;   
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;	      
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	//陀螺仪中断优先级次之
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  													
	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;   
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;   
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;	      
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
    
    //加速度计中断优先级设置
//    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  													
//	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;   
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;   
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;	      
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
	
//	//红外接收中断优先级最低
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  													
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;   
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;   
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;	      
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	
	NVIC_Init(&NVIC_InitStructure);
}

void EXIT_Conf_StopMode(void)
{
    EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
    
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource8);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource9);
    
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_Line = EXTI_Line1 | EXTI_Line2 | EXTI_Line3 | EXTI_Line4 | EXTI_Line5 | EXTI_Line6 | EXTI_Line7 | EXTI_Line13 | EXTI_Line10 | EXTI_Line11 | EXTI_Line12 | EXTI_Line14 | EXTI_Line15;
	EXTI_InitStructure.EXTI_LineCmd = DISABLE;
	EXTI_Init(&EXTI_InitStructure);
    
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_Line = EXTI_Line8;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
    
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_Line = EXTI_Line9;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
        
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);  													
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;   
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;   
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;	      
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}



////////////////////////////////////
//TIM1PWM 输出10K 4通道
//TIM3PWM输出10K 1通道 CH1
///////////////////////////////////

/*****************************************************************************
 函 数 名  : TIM4_Conf
 功能描述  : TIM4定时器初始化
 输入参数  : void
 输出参数  : 无
 返 回 值  : void
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2016年3月11日
    作    者   : 田宏图
    修改内容   : 新生成函数

*****************************************************************************/
void TIM4_Conf( void )
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef	TIM_OCInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4 , ENABLE);
	
	TIM_DeInit(TIM4);
	//设置TIM4   向上计数模式   14.28K频率
	TIM_TimeBaseStructure.TIM_Period = TIM4_Period;		 																					
	TIM_TimeBaseStructure.TIM_Prescaler = (TIM4_Prescaler - 1);				   
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 			
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 		
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	//TIM4 - CH1 / CH2 / CH3 /CH4  PWM1 Mode 1 Output 高电平有效
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
	TIM_OC1Init(TIM4 , &TIM_OCInitStructure);
	TIM_OC2Init(TIM4 , &TIM_OCInitStructure);
	TIM_OC3Init(TIM4 , &TIM_OCInitStructure);
	TIM_OC4Init(TIM4 , &TIM_OCInitStructure);
	
//	TIM_ITConfig(TIM4, TIM_IT_Update , ENABLE);
//	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  													
//	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;   
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;   
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;	      
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&NVIC_InitStructure);

	
	TIM_ClearFlag(TIM4, TIM_FLAG_Update);
	TIM_GenerateEvent(TIM4, TIM_EventSource_Update);

	TIM_ITConfig(TIM4, TIM_IT_Update , ENABLE);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  													
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;   
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;   
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;	      
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
	  
	TIM_Cmd(TIM4, ENABLE);
}


void TIM4_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM4 , TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM4 , TIM_IT_Update);
		
		if(WallCheckEn == ENABLE)
		{
//				WALLPOWERSW;
		}
		else
		{
//			WALLPOWEROFF;
		}
		
	}

}

/*****************************************************************************
 函 数 名  : TIM3_Init
 功能描述  : TIM3定时器初始化
 输入参数  : void
 输出参数  : 无
 返 回 值  : void
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2016年3月11日
    作    者   : 田宏图
    修改内容   : 新生成函数

*****************************************************************************/
void TIM3_Conf( void )
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef	TIM_OCInitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 , ENABLE);
	
	TIM_DeInit(TIM3);
	//设置TIM3  72M主时钟  向上计数模式   10K频率
	TIM_TimeBaseStructure.TIM_Period = TIM3_Period;		 																					
	TIM_TimeBaseStructure.TIM_Prescaler = (TIM3_Prescaler - 1);				   
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 			
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 		
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	//TIM3 - CH1  PWM1 Mode 1Output 高电平有效
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
    TIM_OC1Init(TIM3 , &TIM_OCInitStructure);
	TIM_OC2Init(TIM3 , &TIM_OCInitStructure);
	TIM_OC3Init(TIM3 , &TIM_OCInitStructure);
	TIM_OC4Init(TIM3 , &TIM_OCInitStructure);
	
	
	TIM_ClearFlag(TIM3, TIM_FLAG_Update);							    
	TIM_Cmd(TIM3, ENABLE);
}

/*****************************************************************************
 函 数 名  : TIM2_Conf
 功能描述  : TIM2初始化，用于左轮、右轮和前进轮测速
 				一个计数1us ,溢出时间65536us ,溢出中断使能
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2016年3月30日
    作    者   : 田宏图
    修改内容   : 新生成函数

*****************************************************************************/
void TIM2_Conf(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , ENABLE);
	
	TIM_DeInit(TIM2);
	//设置TIM2  1M主时钟  向上计数模式   65.535ms溢出
	TIM_TimeBaseStructure.TIM_Period = 0xffff;		 																					
	TIM_TimeBaseStructure.TIM_Prescaler = (TIM2_Prescaler - 1);				   
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 			
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 		
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);


	TIM_ClearFlag(TIM2, TIM_FLAG_Update);	
	TIM_ITConfig(TIM2, TIM_IT_Update , ENABLE);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  													
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;   
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;   
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;	      
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
	TIM_Cmd(TIM2, ENABLE);
}

/*****************************************************************************
 函 数 名  : TIM1_Conf
 功能描述  : TIM1初始化，用于左轮、右轮和前进轮测速
 				一个计数1us ,溢出时间65536us ,溢出中断使能
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2016年3月30日
    作    者   : 田宏图
    修改内容   : 新生成函数

*****************************************************************************/
void TIM1_Conf(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef	TIM_OCInitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 , ENABLE);
	
	TIM_DeInit(TIM1);
	//设置TIM2  1M主时钟  向上计数模式   65.535ms溢出
	TIM_TimeBaseStructure.TIM_Period = TIM3_Period;		 																					
	TIM_TimeBaseStructure.TIM_Prescaler = (TIM3_Prescaler - 1);			   
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 			
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 		
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
    
//	TIM_Cmd(TIM1, ENABLE);
	
	//TIM1 - CH3  PWM1 Mode 1Output 高电平有效
    //TIM1 - CH2  PWM1 Mode 1Output 高电平有效
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
//	TIM_OC3Init(TIM1 , &TIM_OCInitStructure);
//    TIM_OC2Init(TIM1 , &TIM_OCInitStructure);
    TIM_OC1Init(TIM1 , &TIM_OCInitStructure);
	
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
//	TIM_ClearFlag(TIM1, TIM_FLAG_Update);							    
	TIM_Cmd(TIM1, ENABLE);
}

/*****************************************************************************
 函 数 名  : TIM6_Init
 功能描述  : TIM6定时器初始化,1ms溢出中断
 输入参数  : void
 输出参数  : 无
 返 回 值  : void
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2016年3月11日
    作    者   : 田宏图
    修改内容   : 新生成函数

*****************************************************************************/
void TIM6_Conf( void )
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6 , ENABLE);
	
	TIM_DeInit(TIM6);
	//设置TIM1    向上计数模式   1ms溢出
	TIM_TimeBaseStructure.TIM_Period = TIM6_Period;		 																					
	TIM_TimeBaseStructure.TIM_Prescaler = (TIM6_Prescaler - 1);				   
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 			
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 		
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);

	TIM_ClearFlag(TIM6, TIM_FLAG_Update);	
	TIM_ITConfig(TIM6, TIM_IT_Update , ENABLE);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  													
	NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;   
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;   
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 5;	      
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
	TIM_Cmd(TIM6, ENABLE);
}

/*****************************************************************************
 函 数 名  : TIM7_Init
 功能描述  : TIM7定时器初始化,65.53ms溢出中断 用于超声波测距计时
 输入参数  : void
 输出参数  : 无
 返 回 值  : void
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2016年3月11日
    作    者   : 田宏图
    修改内容   : 新生成函数

*****************************************************************************/
void TIM7_Conf( void )
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7 , ENABLE);
	
	TIM_DeInit(TIM7);
	//设置TIM3  64M主时钟  向上计数模式   65.53ms溢出
	TIM_TimeBaseStructure.TIM_Period = 0xffff;		 																					
	TIM_TimeBaseStructure.TIM_Prescaler = (TIM7_Prescaler - 1);				   
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 			
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 		
	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);
    
	TIM_Cmd(TIM7, ENABLE);
}
/*****************************************************************************
 函 数 名  : TIM6_IRQHandler
 功能描述  : TIM6中断,1ms溢出中断
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2016年3月11日
    作    者   : 田宏图
    修改内容   : 新生成函数

*****************************************************************************/
void TIM6_IRQHandler(void)
{
	static uint8_t Cnt0 = 0 ,Cnt1 = 0,Cnt2 = 0,Cnt3 = 0,Cnt5 = 0, Cnt6 = 0;
	static uint16_t Cnt4 = 0;
	if(TIM_GetITStatus(TIM6 , TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM6 , TIM_IT_Update);
		
		
        
		if(++Cnt0 >= 50)
		{
			Cnt0 = 0;		
			MotorStartupEn = ENABLE;
			ActuatorEn = ENABLE;
			
			DisplayScanEn = ENABLE;
//			Uart2_SendEn = ENABLE;
            Vocie_DealEn = 1;
		}

		if(++Cnt3 >= UART_Send_Fq)
		{
			Cnt3 = 0;
			Uart_SendEn = ENABLE;
            
            Uart2_SendEn = ENABLE;
		}
		
		if(++Cnt1 >= 20)
		{
			Cnt1 = 0;
			KeyScanEn = ENABLE;
			
			
			BatteryChargeEn = ENABLE;
			PathPlanEn = ENABLE;
            
            BatteryCheckEn = ENABLE;
            
            CarpetCheckEn = 1;
		}

		if(++Cnt2 >= 5) 
		{
			Cnt2 = 0;				
			AccCtrlTimeEn = ENABLE;
			ADCScanEn = ENABLE;
		}
        
        if(++Cnt6 >= SpeedAdjTime)
        {
            Cnt6 = 0;
            AdjustSpeedEn = ENABLE;	
        }

		if(++Cnt4 >= 1000)
		{
			Cnt4 = 0;
			BatteryOffCnt = 0;
//			BatteryCheckEn = ENABLE;
            HandShakeEn = ENABLE;
            Battery_Charge_En = 1;
		}
		
		if(++Cnt5 >= 30)
		{
			Cnt5 = 0;
			Ultra_En = ENABLE;		
            vl53l0x_Mesure_En = 0;    
		}
		if(BatteryCheckEn)
		{
			BatteryOffCnt ++;
		}
        
        Icm_Init_Cnt++;
        

        MotorLinerModeEn = ENABLE;


		SystemRunTime ++;
		
//		MotorStopCtrl();
		MotorStopCtrlEn = ENABLE;
		
		UartTimerJudge();
		
//		RobitMode_Ctrl();
		RobitModeCtrlEn = ENABLE;
//		SignalCheck();
		SignalCheckEn = ENABLE;
        

		if(RightMotor.Status != SYS_STOP && RightMotor.Status != SYS_STOPING && RightMotor.Status != SYS_STARTUP)
		{
			RightMotor.Block_Cnt ++;
			if(RightMotor.Block_Cnt > MOTOR_BLOCK_TIME)
			{
				RightMotor.Status = SYS_BLOCKING;
			}
		}
		if(LeftMotor.Status != SYS_STOP && LeftMotor.Status != SYS_STOPING && LeftMotor.Status != SYS_STARTUP)
		{
			LeftMotor.Block_Cnt ++;
			if(LeftMotor.Block_Cnt > MOTOR_BLOCK_TIME)
			{
				LeftMotor.Status = SYS_BLOCKING;	
			}
			
		}
		
	}
}

void EXTI15_10_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line12) != RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line12);
		
		if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12) != RESET)
		{
			IndrCheck(3, 1);
		}
		else
		{
			IndrCheck(3, 0);
		}
	}
	
	if(EXTI_GetITStatus(EXTI_Line13) != RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line13);
		
        if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13) != RESET)
        {
            IndrCheck(2, 1);
        }
        else
        {
            IndrCheck(2, 0);
        }

	}
	
	if(EXTI_GetITStatus(EXTI_Line14) != RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line14);
		
//		if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_14) != RESET)
//		{
//			IndrCheck(0, 1);
//		}
//		else
//		{
//			IndrCheck(0, 0);
//		}
	}
	
	if(EXTI_GetITStatus(EXTI_Line15) != RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line15);

		if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_15) != RESET)
		{
			IndrCheck(1, 1);
		}
		else
		{
			IndrCheck(1, 0);
		}
		
	}
}

