#ifndef  _INIT_H
#define  _INIT_H

#define		MOTOR_DRIVER_Frq	14286
#define		TIM4_Prescaler		1    
#define		TIM4_Period			(uint16_t)(SYSCLK_FREQ_72MHz / (TIM4_Prescaler) / MOTOR_DRIVER_Frq)
#define		Charge_Frq			10000
#define		TIM3_Prescaler		TIM4_Prescaler
#define		TIM3_Period			(uint16_t)(SYSCLK_FREQ_72MHz / (TIM3_Prescaler) / Charge_Frq)

#if defined MCU_STM32
	#define TIM2_Prescaler		72
#elif defined MCU_GD32	
	#define TIM2_Prescaler		108	
#endif

#define 	TIM6_Prescaler		2
#define		TIM6_Period			(uint16_t)(SYSCLK_FREQ_72MHz / TIM6_Prescaler / 1000)

#define		TIM7_Prescaler		TIM2_Prescaler

#define _5V_OUT_ON              GPIO_WriteBit(GPIOD,GPIO_Pin_14,Bit_SET)
#define _5V_OUT_OFF             GPIO_WriteBit(GPIOD,GPIO_Pin_14,Bit_RESET)
#define _3V3_OUT_ON             GPIO_WriteBit(GPIOD,GPIO_Pin_13,Bit_SET)
#define _3V3_OUT_OFF            GPIO_WriteBit(GPIOD,GPIO_Pin_13,Bit_RESET)
//#define Test_On             GPIO_WriteBit(GPIOA, GPIO_Pin_7, Bit_SET)
//#define Test_Off            GPIO_WriteBit(GPIOA, GPIO_Pin_7, Bit_RESET)
//#define Test_Sw				if(GPIO_ReadOutputDataBit(GPIOA,GPIO_Pin_7)) Test_Off; else Test_On	

//#define Test2_On            GPIO_WriteBit(GPIOC, GPIO_Pin_4, Bit_SET)
//#define Test2_Off           GPIO_WriteBit(GPIOC, GPIO_Pin_4, Bit_RESET)
//#define Test2_Sw			if(GPIO_ReadOutputDataBit(GPIOC,GPIO_Pin_4)) Test2_Off; else Test2_On

//#define Test3_On            GPIO_WriteBit(GPIOC, GPIO_Pin_5, Bit_SET)
//#define Test3_Off           GPIO_WriteBit(GPIOC, GPIO_Pin_5, Bit_RESET)
//#define Test3_Sw			if(GPIO_ReadOutputDataBit(GPIOC,GPIO_Pin_5)) Test3_Off; else Test3_On

//#define Test4_On            GPIO_WriteBit(GPIOB, GPIO_Pin_0, Bit_SET)
//#define Test4_Off           GPIO_WriteBit(GPIOB, GPIO_Pin_0, Bit_RESET)
//#define Test4_Sw			if(GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_0)) Test4_Off; else Test4_On

extern uint32_t SystemRunTime; //系统运行时间单位ms
extern uint8_t SystePwrMode; //系统电源模式 0-正常模式，1-stop模式
extern void Device_Init( void );
extern void GPIO_Conf_StopMode(void);
extern void EXIT_Conf_StopMode( void );
#endif
