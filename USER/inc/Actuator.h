#ifndef _ACTUATOR_H
#define _ACTUATOR_H

#define	DCFMotorOn()		//GPIO_WriteBit(GPIOB,GPIO_Pin_2,Bit_SET)
#define	DCFMotorOff()		//GPIO_WriteBit(GPIOB,GPIO_Pin_2,Bit_RESET)

#define	XPMotorOn(_PWM_)	//TIM_SetCompare3(TIM1,(_PWM_))				//GPIO_WriteBit(GPIOB,GPIO_Pin_2,Bit_SET)
#define	XPMotorOff()		//TIM_SetCompare3(TIM1,0)					//GPIO_WriteBit(GPIOB,GPIO_Pin_2,Bit_RESET)
#define ZS_DUST_MOTOR_PWM_MAX	TIM3_Period


#define	ZSMotorOn(_PWM_)	TIM_SetCompare1(TIM3,_PWM_)
#define	ZSMotorOff()		TIM_SetCompare1(TIM3,0)

#define	DustMotorOn(_PWM_)	TIM_SetCompare2(TIM3,_PWM_)
#define	DustMotorOff()		TIM_SetCompare2(TIM3,0)
#define DustPowerOn()		GPIO_WriteBit(GPIOD,GPIO_Pin_15,Bit_SET)
#define DustPowerOff()      GPIO_WriteBit(GPIOD,GPIO_Pin_15,Bit_RESET)
#define DustSpeedMin        5800
#define DustPwm2Speed_K     10 >> 3


#define MOP_FRONT_ON(_PWM_) TIM_SetCompare3(TIM3,(_PWM_))        //GPIO_WriteBit(GPIOB,GPIO_Pin_1,Bit_SET)
#define MOP_FRONT_OFF()     TIM_SetCompare3(TIM3,0)        //GPIO_WriteBit(GPIOB,GPIO_Pin_1,Bit_RESET)
#define MOP_BACK_ON(_PWM_)  TIM_SetCompare4(TIM3,(_PWM_))        //GPIO_WriteBit(GPIOB,GPIO_Pin_2,Bit_SET)
#define MOP_BACK_OFF()      TIM_SetCompare4(TIM3,0)        //GPIO_WriteBit(GPIOB,GPIO_Pin_2,Bit_RESET)



#define MOP_B_RUN(_PWM_)            MOP_FRONT_ON((_PWM_)); MOP_BACK_OFF()    
#define MOP_F_RUN(_PWM_)            MOP_BACK_ON((_PWM_)); MOP_FRONT_OFF()
#define MOP_BRAKE()                 MOP_FRONT_ON(TIM3_Period);MOP_BACK_ON(TIM3_Period)
#define MOP_STOP()                  MOP_BACK_OFF();MOP_FRONT_OFF()

typedef struct
{
    FunctionalState MopEn;
    FunctionalState MopSwitchEn;
    uint8_t MopOffCnt;
    uint8_t MopLoopCnt;
    MOTOR_RUNDIR_TypeDef MopDir;
    uint16_t MopPwm;
    uint8_t LoopInit;
    uint8_t LoopStatus;
    uint8_t LoopOverflow;
    uint8_t LoopOverflowCnt;
    uint8_t Pos;
    uint8_t StartPos;
    uint8_t StopPos;
    uint8_t TurnDir;
    uint8_t Init;
}MopTypeDef;
extern FunctionalState SprayWaterEn,ActuatorEn,CleanEn,DustEn,RagChangeEn;
extern MopTypeDef MopPar;
extern uint16_t CleanPwmOn, DustPwmOn;
extern uint8_t fanLevelPre, sideBroomLevelPre, midBroomLevelPre;
extern uint16_t timeDiffer[4];

extern void Actuator_Conf(void);
extern void Actuator_Deal(void);
extern void RagChangeDeal(void);
extern void MopCtrl(void);
#endif
