#ifndef _ROBIT_RUN_H
#define _ROBIT_RUN_H

/*----------------------------------------------*
 * ºê¶¨Òå                                       *
 *----------------------------------------------*/


//½Ç¶È¿ØÖÆÄ£Ê½ÖÐ£¬Ò»¸öÑ­»·ÄÜ¹»Ôö¼ÓµÄ½Ç¶È£¬µ¥Î»0.01du
//Ä¿Ç°ÍÓÂÝÒÇ×î´ó²âÁ¿½ÇËÙ¶ÈÎª200¶È/s,¶ø»úÆ÷Êµ¼Ê×î´ó½ÇËÙ¶ÈÔÚ150¶È/s×óÓÒ
//Ä¿Ç°µÄÍÓÂÝÒÇ²É¼¯µÄÆµÂÊÎª100Hz,²ÉÑùÊ±¼äÎª10ms,µ¥´Î½Ç¶È×î´ó±ä»¯Á¿200
#define ANGLE_DELT  200

#if defined MCU_STM32
#define PWM_K	9  / 8
#elif defined MCU_GD32
#define PWM_K	27 / 16
#endif

#define		Motor_L_F_PWM(_Num_)	TIM_SetCompare1(TIM4,_Num_*PWM_K)
#define		Motor_L_B_PWM(_Num_)	TIM_SetCompare2(TIM4,_Num_*PWM_K)
#define		Motor_R_F_PWM(_Num_)	TIM_SetCompare3(TIM4,_Num_*PWM_K)
#define		Motor_R_B_PWM(_Num_)	TIM_SetCompare4(TIM4,_Num_*PWM_K)

//#define		Motor_L_F_DownOn()		GPIO_WriteBit(GPIOE,GPIO_Pin_0,Bit_RESET)
//#define		Motor_L_B_DownOn()		GPIO_WriteBit(GPIOE,GPIO_Pin_1,Bit_RESET)
//#define		Motor_R_F_DownOn()		GPIO_WriteBit(GPIOE,GPIO_Pin_3,Bit_RESET)
//#define		Motor_R_B_DownOn()		GPIO_WriteBit(GPIOE,GPIO_Pin_2,Bit_RESET)

//#define		Motor_L_F_DownOff()		GPIO_WriteBit(GPIOE,GPIO_Pin_0,Bit_SET)
//#define		Motor_L_B_DownOff()		GPIO_WriteBit(GPIOE,GPIO_Pin_1,Bit_SET)
//#define		Motor_R_F_DownOff()		GPIO_WriteBit(GPIOE,GPIO_Pin_3,Bit_SET)
//#define		Motor_R_B_DownOff()		GPIO_WriteBit(GPIOE,GPIO_Pin_2,Bit_SET)



#define		Motor_L_B_Run(_Pwm_)	Motor_L_B_PWM(0);Motor_L_F_PWM(_Pwm_);
#define		Motor_L_F_Run(_Pwm_)	Motor_L_F_PWM(0);Motor_L_B_PWM(_Pwm_);
#define		Motor_R_F_Run(_Pwm_)	Motor_R_B_PWM(0);Motor_R_F_PWM(_Pwm_);
#define		Motor_R_B_Run(_Pwm_)	Motor_R_F_PWM(0);Motor_R_B_PWM(_Pwm_);
#define		Motor_L_Brake()			Motor_L_B_PWM(TIM4_Period);Motor_L_F_PWM(TIM4_Period);
#define		Motor_R_Brake()			Motor_R_B_PWM(TIM4_Period);Motor_R_F_PWM(TIM4_Period);
#define		Motor_L_Stop()			Motor_L_B_PWM(0);Motor_L_F_PWM(0);
#define		Motor_R_Stop()			Motor_R_B_PWM(0);Motor_R_F_PWM(0);

#define		Motor_Circle_PlusNum	16
#define		Motor_K		60000000L / Motor_Circle_PlusNum
#define		Motor_InterTimerMin		150  //100

#define		Motor_Stop_Time		50      //µ¥Î»ms
#define		MOTOR_BLOCK_TIME	50		//µ¥Î»ms
#define 	PWM_STATRUP_VAL_STEP 50
#define 	PWM_STATRUP_VAL_MAX  4400
#define 	MOT_RPM_SPEED_MIN_2 650  
#define 	MOT_RPM_SPEED_MIN   700     
#define     MOT_RPM_SPEED_MAX_2 6500
#define 	MOT_RPM_SPEED_MAX   6000 
#define 	PWM_STATRUP_VAL     2800
#define 	PWM_VAL_MAX   		4480//TIM4_Period  //4480
#define 	PWM_VAL_MIN  	 	500
#define		PWM_VAL_MIN_Low		900
#define     ANGLERATE_MIN       1800

#define		Motor_Stop_Step		50

#define		Edage_Distance		300 //280  // 5cm

typedef enum{
  SYS_STOP,       //µç»úÒÑÍ£Ö¹ 
  SYS_STARTUP,    //µç»úÆô¶¯ÖÐ
  SYS_RUNNING,    //µç»úÕýÔÚÔËÐÐÖÐ
  SYS_STOPING,    //µç»úÕýÔÚÍ£Ö¹ÖÐ
  SYS_BLOCKING,   //µç»ú¶Â×ª×´Ì¬
  SYS_CHECK,      //µç»úÆô¶¯Ç°¼ì²é
  SYS_ERROR,	  //¹ÊÕÏ×´Ì¬
}SystemSta;

typedef enum 
{ 
	MOTOR_RUN_DIR_ZW    = (uint8_t)0x00,  /* µç»úÕý×ª */
	
    MOTOR_RUN_DIR_FW   = (uint8_t)0x01, /* µç»ú·´×ª */
    
} MOTOR_RUNDIR_TypeDef;


typedef struct
{
  uint16_t RunCycle;		

  uint16_t RunCircle;
  
  uint16_t CurSpeed;

  uint16_t TargSpeed;
  
  uint16_t BaseSpeed;
    
  int32_t Vacc;
    
  int32_t Wacc;
    
  int16_t  SpeedAdjAll;
  
  uint16_t Block_Cnt;
  
  uint16_t StopCnt;

  uint16_t PWMVal;

  FunctionalState RunEnable;

  FlagStatus RunFlag;
	
  FlagStatus SpeedRisedFlag;

  SystemSta  Status;

  MOTOR_RUNDIR_TypeDef Dir;
  
  MOTOR_RUNDIR_TypeDef TargDir;

} Motor_TypeDef;

typedef struct
{
  uint16_t PreCountValue;   //ÉÏÒ»´Î¼ÆÊýÖµ
  
  uint8_t OverflowCnt;     //¶¨Ê±Æ÷Òç³ö´ÎÊý

  uint32_t CircleTime;      //µç»úÔËÐÐÒ»È¦Ê±¼ä  µ¥Î»1us

  FunctionalState CalEn;	//¼ÆËãËÙ¶ÈÊ¹ÄÜ
  
  FunctionalState ValidForAnguar; //ËÙ¶ÈÓÐÐ§£¬ÓÃÓÚ¼ÆËã½ÇËÙ¶È

  FunctionalState Valid;    //ËÙ¶ÈÓÐÐ§£¬ÓÃÓÚËÙ¶ÈPIDµ÷½Ú

  uint8_t  PulseCnt;

} MotorSpeed_TypeDef;



extern Motor_TypeDef LeftMotor;
extern Motor_TypeDef RightMotor;
extern MotorSpeed_TypeDef LeftMotorSpeed,RightMotorSpeed;
extern FunctionalState AdjustSpeedEn,MotorLinerModeEn,MotorStopCtrlEn,RobitModeCtrlEn,AccCtrlTimeEn;
extern uint32_t TargMileage,LeftCurMileage,RightCurMileage, TargMileageForArc;
extern FunctionalState RobitRouteModeEn;
extern FunctionalState MotorStartupEn;
extern uint8_t RobitStatus;  				// 0--Í£Ö¹×´Ì¬ 1-- ¹¤×÷×´Ì¬
extern uint8_t RobitRunningMode;           //É¨µØ»ú¹¤×÷Ä£Ê½  0--¿ª»·¹¤×÷·½Ê½ 1--±Õ»·¹¤×÷·½Ê½
extern uint8_t RobitRunningClosedLoopMode; //É¨µØ»ú±Õ»·¹¤×÷·½Ê½  0--ËÙ¶È±Õ»··½Ê½ 1--ÐÐ³Ì±Õ»·¹¤×÷·½Ê½
extern uint8_t RobitRunbyRouteMode;		//É¨µØ»úÐÐ³Ì±Õ»··½Ê½  0--Ö±ÏßÐÐÊ»Ä¿±êÀï³ÌÊý1--ÖáÐÄ×ª¶¯Ä¿±ê½Ç¶È
extern int32_t  LeftMileageAll,RightMileageAll; //×óÓÒÂÖÀÛ¼ÆÀï³ÌÊý£¬Ç°½øÔö¼Ó£¬ºóÍË¼õÐ¡¡£
extern int32_t  LeftMileageAllPre,RightMileageAllPre;
extern FlagStatus LeftMotorSwDirFlag,RightMotorSwDirFlag;//É¨µØ»úËÙ¶È±Õ»·Ä£Ê½ÖÐ£¬·½ÏòÇÐ»»±êÖ¾Î»
extern uint16_t   LeftMotorSpeedBak,RightMotorSpeedBak;//É¨µØ»úËÙ¶È±Õ»·Ä£Ê½ÖÐ£¬·½ÏòÇÐ»»Ê±µÄËÙ¶È±¸·Ý
extern MOTOR_RUNDIR_TypeDef    LeftMotorDirBak,RightMotorDirBak;//É¨µØ»úËÙ¶È±Õ»·Ä£Ê½ÖÐ£¬·½ÏòÇÐ»»Ê±µÄ·½Ïò±¸·Ý
extern uint8_t SpeedAdjTime;   //ÂÖ×ÓËÙ¶ÈPIDµ÷½ÚÊ±¼ä
extern uint16_t LeftMotorErrorCnt,RightMotorErrorCnt;
extern int16_t Sys_Startangle,AnglerateTarg, AnglerateTargInit;
extern uint8_t  TargCirle;
extern uint16_t  TargAngle, RotationAngleCur;
extern uint16_t StepModeCnt;
extern uint8_t Robit_Dir;    //0---Ç°½ø  1---ºóÍ
extern int32_t RobotLineSpeed, RobotLineSpeedTarg, RobotAngleRate; //»úÆ÷ÈËÏßËÙ¶È(mm/s)ºÍ½ÇËÙ¶È(ºÁ»¡¶È/s)
extern FunctionalState VaccEn, WaccEn; //Ïß¼ÓËÙÊ¹ÄÜ±êÖ¾ £¬ ½Ç¼ÓËÙ¶ÈÊ¹ÄÜ±êÖ¾
extern uint8_t VaccInit, WaccInit;
extern int16_t AngularVelocityOdo;   
extern uint8_t CarpetStatus, CarpetCheckEn;
extern uint8_t Sta_Gyro, UpCnt, DownCnt;
extern uint8_t earthWaveCnt, earthAvgOk;
extern uint16_t earthAvg;
extern uint16_t earthWaveMax,earthWaveMin;

extern void Robit_RunLR(uint8_t LeftMotor_Speed, uint8_t LeftMotor_Dir, uint8_t RightMotor_Speed, uint8_t RightMotor_Dir);
extern void Robit_RunDir(uint8_t Speed, uint8_t Dir);
extern void Robit_Stop(void);
extern void MotorStopCtrl(void);
extern void CalSpeed(void);
extern void LeftMotorEnterRun(uint8_t LeftMotor_Dir);
extern void RightMotorEnterRun(uint8_t RightMotor_Dir);
extern void RightMotorStop(void);
extern void LeftMotorStop(void);
extern void RightMotorRunInit(void);
extern void MotorRunCtrl(void);
extern void Robit_RouteMoving(void);
extern void Motor_Conf(void);
extern void RobitMode_Conf(void);
extern void MotorStartupCtrl(void);
extern void MotorBlockCtrl(void);
extern void AccCtrl(void);
extern void RobitMode_Ctrl(void);
extern uint16_t CalAngleDif(int16_t Angle1, int16_t Angle2);
//extern int16_t DistanceCalu(void);
extern int16_t CalOdoAngularVelocity(int16_t leftSpeed, int16_t rightSpeed);
extern void CarpetCheck(void);

#endif

