#ifndef _SIGNAL_H
#define _SIGNAL_H

#define EARTH_CHECK_NUM	2
//#define WALLPOWERON		GPIO_WriteBit(GPIOA,GPIO_Pin_12,Bit_RESET)
//#define WALLPOWEROFF	GPIO_WriteBit(GPIOA,GPIO_Pin_12,Bit_SET)
//#define EARTHPOWERON	GPIO_WriteBit(GPIOE,GPIO_Pin_5,Bit_RESET)
//#define EARTHPOWEROFF	GPIO_WriteBit(GPIOE,GPIO_Pin_5,Bit_SET)
//#define INFRASIGNALON	GPIO_WriteBit(GPIOE,GPIO_Pin_7,Bit_RESET)
//#define INFRASIGNALOFF	GPIO_WriteBit(GPIOE,GPIO_Pin_7,Bit_SET)

//#define WALLPOWERSW		if(GPIO_ReadOutputDataBit(GPIOA,GPIO_Pin_12))\
//							WALLPOWEROFF;\
//						else  \
//							WALLPOWERON

//#define SpeedLEDEn		GPIO_WriteBit(GPIOE,GPIO_Pin_6,Bit_RESET)
//#define SpeedLEDDis		GPIO_WriteBit(GPIOE,GPIO_Pin_6,Bit_SET)	

#define LWALLADCLIMIT	0x0D93
#define LMWALLADCLIMIT	0x0D93
#define MWALLADCLIMIT	0x0D93
#define RMWALLADCLIMIT	0x0D93
#define RWALLADCLIMIT	0x0D93
#define WALLCHECKDELT	0x0064

#define ERATHCHECKDELT2 0x012c  //灵敏度2 300
#define ERATHCHECKDELT	0x03e8  //灵敏度 1000
						
#define ERATHCHECKBASE	0x0C1C	//基准判定值 3100
#define BLACKERATHVALUE 0x0ED8  // 3800
#define ERATHCHECKWAVE  0x08    // 正常波动 -8~+8

#define ULTRAOBSDIS_MIN 200  //2cm
#define ULTRAOBSDIS_MAX 235  //2.5cm

#define ULTRAOBSDIS_FRONT   400

#define ULTRAOBSCHECK_FRONT 590


#define ULTRAFRONTTRIC_H	GPIO_WriteBit(GPIOE,GPIO_Pin_7,Bit_SET)
#define ULTRAFRONTTRIC_L	GPIO_WriteBit(GPIOE,GPIO_Pin_7,Bit_RESET)

#define ULTRALEFFRONTTRIC_H	//GPIO_WriteBit(GPIOE,GPIO_Pin_3,Bit_SET)
#define ULTRALEFFRONTTRIC_L //GPIO_WriteBit(GPIOE,GPIO_Pin_3,Bit_RESET)

#define ULTRALEFBEHTRIC_H   //GPIO_WriteBit(GPIOE,GPIO_Pin_4,Bit_SET)
#define ULTRALEFBEHTRIC_L   //GPIO_WriteBit(GPIOE,GPIO_Pin_4,Bit_RESET)


typedef struct
{
	uint32_t Sum;
	uint16_t Base;
	uint16_t Delt;
	uint8_t  Cnt;
	uint8_t  CheckCnt;
	uint8_t  OK;
	uint16_t Data_Pre;
}WallCheckFrame;

typedef struct
{
	uint16_t PreTime;
	uint8_t  RevByte;
	uint16_t RevData;
	uint16_t RevDataBak;
	FunctionalState RevDataVlid;
	FunctionalState FirstFlag;
	FunctionalState RevOK;
	uint8_t RevOKCnt;
	uint8_t  FreeTimeCnt;

} Indr_TypeDef;

extern uint8_t LeftBump,RightBump,MidBump, BumpBak;// 0 - 未碰撞 1--碰撞 
extern FunctionalState BumpSendEn,WheelLiftSendEn,WallSendEn,WallCheckEn,SignalCheckEn;
extern uint8_t WheelLift;//左右轮有一个被抬起，即认为被抬起
extern uint8_t WallSignal[5];//前方障碍物信息，0-无障碍物1-有障碍物
extern uint8_t RagChangeSingal, RagChangeSingalB;	
extern WallCheckFrame WallData[5],EarthData[EARTH_CHECK_NUM];
extern FunctionalState BumpFlag, BumpOverFlag, BumpOverForCloseWallFlag;
extern Indr_TypeDef Indr[4];

extern FunctionalState Ultra_En,Ultra_Check_En[3],Ultra_Rev_OK[3],Ultra_Rev_First[3],Ultra_Jump[3],Ultra_Rising[3],Ultra_Falling[3];
extern uint8_t Ultra_ID,Ultra_RevID,Ultra_RevLv[3];
extern uint16_t Ultra_RevTime[3],Ultra_RevTimeVlid[3];
extern uint8_t UltraObs, UltraDecelerationObs; 
extern uint16_t PSD_Distance, PSD_AD;;

extern void SignalCheck(void);
extern void Signal_Conf(void);
extern void Ultra_Deal(void);
extern void Ultra_Check(void);
extern void delay15us(void);
extern uint16_t UltraTime2Dis(uint16_t _UltraRevTime_);
extern void IndrCheck(uint8_t indrid, uint8_t swmode);
#endif
