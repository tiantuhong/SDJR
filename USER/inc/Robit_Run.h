#ifndef _ROBIT_RUN_H
#define _ROBIT_RUN_H

/*----------------------------------------------*
 * �궨��                                       *
 *----------------------------------------------*/


//�Ƕȿ���ģʽ�У�һ��ѭ���ܹ����ӵĽǶȣ���λ0.01du
//Ŀǰ���������������ٶ�Ϊ200��/s,������ʵ�������ٶ���150��/s����
//Ŀǰ�������ǲɼ���Ƶ��Ϊ100Hz,����ʱ��Ϊ10ms,���νǶ����仯��200
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

#define		Motor_Stop_Time		50      //��λms
#define		MOTOR_BLOCK_TIME	50		//��λms
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
  SYS_STOP,       //�����ֹͣ 
  SYS_STARTUP,    //���������
  SYS_RUNNING,    //�������������
  SYS_STOPING,    //�������ֹͣ��
  SYS_BLOCKING,   //�����ת״̬
  SYS_CHECK,      //�������ǰ���
  SYS_ERROR,	  //����״̬
}SystemSta;

typedef enum 
{ 
	MOTOR_RUN_DIR_ZW    = (uint8_t)0x00,  /* �����ת */
	
    MOTOR_RUN_DIR_FW   = (uint8_t)0x01, /* �����ת */
    
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
  uint16_t PreCountValue;   //��һ�μ���ֵ
  
  uint8_t OverflowCnt;     //��ʱ���������

  uint32_t CircleTime;      //�������һȦʱ��  ��λ1us

  FunctionalState CalEn;	//�����ٶ�ʹ��
  
  FunctionalState ValidForAnguar; //�ٶ���Ч�����ڼ�����ٶ�

  FunctionalState Valid;    //�ٶ���Ч�������ٶ�PID����

  uint8_t  PulseCnt;

} MotorSpeed_TypeDef;



extern Motor_TypeDef LeftMotor;
extern Motor_TypeDef RightMotor;
extern MotorSpeed_TypeDef LeftMotorSpeed,RightMotorSpeed;
extern FunctionalState AdjustSpeedEn,MotorLinerModeEn,MotorStopCtrlEn,RobitModeCtrlEn,AccCtrlTimeEn;
extern uint32_t TargMileage,LeftCurMileage,RightCurMileage, TargMileageForArc;
extern FunctionalState RobitRouteModeEn;
extern FunctionalState MotorStartupEn;
extern uint8_t RobitStatus;  				// 0--ֹͣ״̬ 1-- ����״̬
extern uint8_t RobitRunningMode;           //ɨ�ػ�����ģʽ  0--����������ʽ 1--�ջ�������ʽ
extern uint8_t RobitRunningClosedLoopMode; //ɨ�ػ��ջ�������ʽ  0--�ٶȱջ���ʽ 1--�г̱ջ�������ʽ
extern uint8_t RobitRunbyRouteMode;		//ɨ�ػ��г̱ջ���ʽ  0--ֱ����ʻĿ�������1--����ת��Ŀ��Ƕ�
extern int32_t  LeftMileageAll,RightMileageAll; //�������ۼ��������ǰ�����ӣ����˼�С��
extern int32_t  LeftMileageAllPre,RightMileageAllPre;
extern FlagStatus LeftMotorSwDirFlag,RightMotorSwDirFlag;//ɨ�ػ��ٶȱջ�ģʽ�У������л���־λ
extern uint16_t   LeftMotorSpeedBak,RightMotorSpeedBak;//ɨ�ػ��ٶȱջ�ģʽ�У������л�ʱ���ٶȱ���
extern MOTOR_RUNDIR_TypeDef    LeftMotorDirBak,RightMotorDirBak;//ɨ�ػ��ٶȱջ�ģʽ�У������л�ʱ�ķ��򱸷�
extern uint8_t SpeedAdjTime;   //�����ٶ�PID����ʱ��
extern uint16_t LeftMotorErrorCnt,RightMotorErrorCnt;
extern int16_t Sys_Startangle,AnglerateTarg, AnglerateTargInit;
extern uint8_t  TargCirle;
extern uint16_t  TargAngle, RotationAngleCur;
extern uint16_t StepModeCnt;
extern uint8_t Robit_Dir;    //0---ǰ��  1---���
extern int32_t RobotLineSpeed, RobotLineSpeedTarg, RobotAngleRate; //���������ٶ�(mm/s)�ͽ��ٶ�(������/s)
extern FunctionalState VaccEn, WaccEn; //�߼���ʹ�ܱ�־ �� �Ǽ��ٶ�ʹ�ܱ�־
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

