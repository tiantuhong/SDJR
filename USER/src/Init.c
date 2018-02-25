/******************************************************************************

                  ��Ȩ���� (C), 1994-2015, ���ݾ���ŷ�϶�С�ҵ����޹�˾

 ******************************************************************************
  �� �� ��   : Init.c
  �� �� ��   : V1.0
  ��    ��   : ���ͼ
  ��������   : 2016��3��11��
  ����޸�   :
  ��������   : �豸��ʼ�����ж�
  �����б�   :
              Device_Init
              EXIT_Init
              EXTI4_IRQHandler
              EXTI9_5_IRQHandler
              GPIO_Init
              TIM1_Init
              TIM3_Init
              TIM6_Init
  �޸���ʷ   :
  1.��    ��   : 2016��3��11��
    ��    ��   : ���ͼ
    �޸�����   : �����ļ�

******************************************************************************/

/*----------------------------------------------*
 * ����ͷ�ļ�                                   *
 *----------------------------------------------*/
#include "stm32f10x.h"
#include "imu.h"
/*----------------------------------------------*
 * �ⲿ����˵��                                 *
 *----------------------------------------------*/

/*----------------------------------------------*
 * �ⲿ����ԭ��˵��                             *
 *----------------------------------------------*/
void Device_Init( void );
/*----------------------------------------------*
 * �ڲ�����ԭ��˵��                             *
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
 * ȫ�ֱ���                                     *
 *----------------------------------------------*/
uint32_t SystemRunTime; //ϵͳ����ʱ�䵥λms
uint8_t SystePwrMode; //ϵͳ��Դģʽ 0-����ģʽ��1-stopģʽ
/*----------------------------------------------*
 * ģ�鼶����                                   *
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
 * ��������                                     *
 *----------------------------------------------*/

/*----------------------------------------------*
 * �궨��                                       *
 *----------------------------------------------*/






/*****************************************************************************
 �� �� ��  : Device_Init
 ��������  : �豸��ʼ��
 �������  : void
 �������  : ��
 �� �� ֵ  : void
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2016��3��11��
    ��    ��   : ���ͼ
    �޸�����   : �����ɺ���

*****************************************************************************/
void Device_Init( void )
{
//    float ret;
//    ret = ANGLE2MILEAGE;
    SystemInit();//HSI /2 *16  = 8/2*16=64MHz
    GPIO_Conf();//I/O��ʼ��
    
    _5V_OUT_ON;
    _3V3_OUT_ON;
    
    Gyro_Conf();
    
    LED1(1); //�ϵ�LED1��һ��
    
	EXIT_Conf();//�ⲿ�жϳ�ʼ��
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
 �� �� ��  : GPIO_Conf
 ��������  : GPIO��ʼ��
 �������  : void
 �������  : ��
 �� �� ֵ  : void
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2016��3��8��
    ��    ��   : ���ͼ
    �޸�����   : �����ɺ���

*****************************************************************************/
void GPIO_Conf( void )
{
	GPIO_InitTypeDef GPIO_InitStructure;
 
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE |RCC_APB2Periph_AFIO, ENABLE); 						 

////////////////////GPIOA////////////////////////////////////////////////////////////////////
	//ģ������		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6;   
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;                                        
	GPIO_Init(GPIOA, &GPIO_InitStructure);


	//������� (3/7/8/11/12 ��δʹ�ã�����Ϊ�������)
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

	//ģ������		
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;   
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;                                        
//	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	//��������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_4 | GPIO_Pin_5;// | GPIO_Pin_0;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
  	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//��������
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
//  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; 
//  	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	//�������
	GPIO_InitStructure.GPIO_Pin =   GPIO_Pin_3 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15 | GPIO_Pin_12;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
  	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//�����������TIM4
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 ;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
  	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//���ÿ�©���I2C2
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
		
///////////////////GPIOB////////////////////////////////////////////////////////////////////


///////////////////GPIOC////////////////////////////////////////////////////////////////////

	//ģ������		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; 
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	//��������
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
//  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
//  	GPIO_Init(GPIOC, &GPIO_InitStructure);

	//�������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_4 /*| GPIO_Pin_5*/;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
  	GPIO_Init(GPIOC, &GPIO_InitStructure);	

	//�����������TIM3_CH3   TIM3_CH4
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_6 | GPIO_Pin_7;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
  	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	//TIM3_CH1 -> PC6 TIM3_CH2 -> PC7 TIM3_CH3 -> PC8 TIM3_CH4 -> PC9
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM3 , ENABLE);

///////////////////GPIOC////////////////////////////////////////////////////////////////////


///////////////////GPIOD////////////////////////////////////////////////////////////////////

	//��������		
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
//	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	//�������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
  	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	
	//��������
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_6;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
  	GPIO_Init(GPIOD, &GPIO_InitStructure);
    
    //�����������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
  	GPIO_Init(GPIOD, &GPIO_InitStructure);
    
	//USART2-> PD5/PD6
	GPIO_PinRemapConfig(GPIO_Remap_USART2,ENABLE);

///////////////////GPIOD////////////////////////////////////////////////////////////////////


///////////////////GPIOE////////////////////////////////////////////////////////////////////


	//�������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_7 ;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
  	GPIO_Init(GPIOE, &GPIO_InitStructure);

	//��������
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_3 | GPIO_Pin_8 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
  	GPIO_Init(GPIOE, &GPIO_InitStructure);
    
    //��������		
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
    
    //ģ������		
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
 �� �� ��  : EXIT_Conf
 ��������  : �ⲿ�жϳ�ʼ��: PB5/PC10/PE4  �������� �������ж�
 �������  : void
 �������  : ��
 �� �� ֵ  : void
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2016��3��8��
    ��    ��   : ���ͼ
    �޸�����   : �����ɺ���

*****************************************************************************/
void EXIT_Conf( void )
{
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); 

	// ���ٶȼ��ж�
//    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource0);
    
    // ���ֲ����ж�
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource1);

	//�������ж�
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE, GPIO_PinSource3);
    
    // �ϲ�λ���ж�
//    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource4);

	//���ֲ����ж�
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource5);

	//����������ж�
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource2);

	//�����źŽ����ж�
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


	//�����ֲ����жϣ�˫�����ж�
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;//EXTI_Trigger_Falling;//
	EXTI_InitStructure.EXTI_Line = EXTI_Line1 | EXTI_Line5;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	//�������жϣ��½����ж�
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_Line = EXTI_Line3;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
    
    //���ٶȼ��жϣ��������ж�
//	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
//	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
//	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
//	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
//	EXTI_Init(&EXTI_InitStructure);

//	//����������жϣ�˫�����ж�
//	//�����źŽ���˫�����ж�
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_Line = EXTI_Line2 | EXTI_Line12 | EXTI_Line13 | EXTI_Line14 | EXTI_Line15;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
    
    
	//�������ж����ȼ����
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  													
	NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;   
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;   
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;	      
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	//�����ж����ȼ��θ�
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

	//�������ж����ȼ���֮
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  													
	NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;   
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;   
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;	      
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
    
    //���ٶȼ��ж����ȼ�����
//    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  													
//	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;   
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;   
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;	      
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
	
//	//��������ж����ȼ����
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
//TIM1PWM ���10K 4ͨ��
//TIM3PWM���10K 1ͨ�� CH1
///////////////////////////////////

/*****************************************************************************
 �� �� ��  : TIM4_Conf
 ��������  : TIM4��ʱ����ʼ��
 �������  : void
 �������  : ��
 �� �� ֵ  : void
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2016��3��11��
    ��    ��   : ���ͼ
    �޸�����   : �����ɺ���

*****************************************************************************/
void TIM4_Conf( void )
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef	TIM_OCInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4 , ENABLE);
	
	TIM_DeInit(TIM4);
	//����TIM4   ���ϼ���ģʽ   14.28KƵ��
	TIM_TimeBaseStructure.TIM_Period = TIM4_Period;		 																					
	TIM_TimeBaseStructure.TIM_Prescaler = (TIM4_Prescaler - 1);				   
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 			
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 		
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	//TIM4 - CH1 / CH2 / CH3 /CH4  PWM1 Mode 1 Output �ߵ�ƽ��Ч
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
 �� �� ��  : TIM3_Init
 ��������  : TIM3��ʱ����ʼ��
 �������  : void
 �������  : ��
 �� �� ֵ  : void
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2016��3��11��
    ��    ��   : ���ͼ
    �޸�����   : �����ɺ���

*****************************************************************************/
void TIM3_Conf( void )
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef	TIM_OCInitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 , ENABLE);
	
	TIM_DeInit(TIM3);
	//����TIM3  72M��ʱ��  ���ϼ���ģʽ   10KƵ��
	TIM_TimeBaseStructure.TIM_Period = TIM3_Period;		 																					
	TIM_TimeBaseStructure.TIM_Prescaler = (TIM3_Prescaler - 1);				   
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 			
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 		
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	//TIM3 - CH1  PWM1 Mode 1Output �ߵ�ƽ��Ч
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
 �� �� ��  : TIM2_Conf
 ��������  : TIM2��ʼ�����������֡����ֺ�ǰ���ֲ���
 				һ������1us ,���ʱ��65536us ,����ж�ʹ��
 �������  : void  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2016��3��30��
    ��    ��   : ���ͼ
    �޸�����   : �����ɺ���

*****************************************************************************/
void TIM2_Conf(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , ENABLE);
	
	TIM_DeInit(TIM2);
	//����TIM2  1M��ʱ��  ���ϼ���ģʽ   65.535ms���
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
 �� �� ��  : TIM1_Conf
 ��������  : TIM1��ʼ�����������֡����ֺ�ǰ���ֲ���
 				һ������1us ,���ʱ��65536us ,����ж�ʹ��
 �������  : void  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2016��3��30��
    ��    ��   : ���ͼ
    �޸�����   : �����ɺ���

*****************************************************************************/
void TIM1_Conf(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef	TIM_OCInitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 , ENABLE);
	
	TIM_DeInit(TIM1);
	//����TIM2  1M��ʱ��  ���ϼ���ģʽ   65.535ms���
	TIM_TimeBaseStructure.TIM_Period = TIM3_Period;		 																					
	TIM_TimeBaseStructure.TIM_Prescaler = (TIM3_Prescaler - 1);			   
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 			
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 		
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
    
//	TIM_Cmd(TIM1, ENABLE);
	
	//TIM1 - CH3  PWM1 Mode 1Output �ߵ�ƽ��Ч
    //TIM1 - CH2  PWM1 Mode 1Output �ߵ�ƽ��Ч
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
 �� �� ��  : TIM6_Init
 ��������  : TIM6��ʱ����ʼ��,1ms����ж�
 �������  : void
 �������  : ��
 �� �� ֵ  : void
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2016��3��11��
    ��    ��   : ���ͼ
    �޸�����   : �����ɺ���

*****************************************************************************/
void TIM6_Conf( void )
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6 , ENABLE);
	
	TIM_DeInit(TIM6);
	//����TIM1    ���ϼ���ģʽ   1ms���
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
 �� �� ��  : TIM7_Init
 ��������  : TIM7��ʱ����ʼ��,65.53ms����ж� ���ڳ���������ʱ
 �������  : void
 �������  : ��
 �� �� ֵ  : void
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2016��3��11��
    ��    ��   : ���ͼ
    �޸�����   : �����ɺ���

*****************************************************************************/
void TIM7_Conf( void )
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7 , ENABLE);
	
	TIM_DeInit(TIM7);
	//����TIM3  64M��ʱ��  ���ϼ���ģʽ   65.53ms���
	TIM_TimeBaseStructure.TIM_Period = 0xffff;		 																					
	TIM_TimeBaseStructure.TIM_Prescaler = (TIM7_Prescaler - 1);				   
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; 			
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 		
	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);
    
	TIM_Cmd(TIM7, ENABLE);
}
/*****************************************************************************
 �� �� ��  : TIM6_IRQHandler
 ��������  : TIM6�ж�,1ms����ж�
 �������  : void  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2016��3��11��
    ��    ��   : ���ͼ
    �޸�����   : �����ɺ���

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

