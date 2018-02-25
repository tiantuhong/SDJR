#ifndef _BATTERY_H
#define _BATTERY_H

//#define Set_Charge_PWM(_Num_)		TIM_SetCompare1(TIM3,_Num_)
#define BATTERY_CHARGE_CURRENT		1800
#define BATTERY_CHARGE_CURRENT_MIN	0x30
#define BATTERY_CHARGE_CURRENT_FULL	500

#define BATTERY_AD2mV_K             (3300 * 11) >> 10


#define BATTERY_EN                  //GPIO_WriteBit(GPIOD,GPIO_Pin_10,Bit_SET)
#define BATTERY_DIS                 //GPIO_WriteBit(GPIOD,GPIO_Pin_10,Bit_RESET)
#define BATTERY_CHARGEING_STA       //GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_11)      //0=正在充电，1=未充电
#define BATTERY_COMPLET_STA         //GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_12)      //0=充电完成，1=充电未完成
#define BATTERY_ERROR_STA           //GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_13)      //0=充电故障，1=充电正常

extern uint8_t BatterySoc, BatterySocDis;	//电池剩余电量 最大100 最小0
extern uint8_t BatteryConnect;//0--未连接 1--连接
extern uint16_t BatteryVol2Soc[11];
extern FunctionalState SocketPower,JackPower,BatteryCheckEn,BatteryChargeEn;
extern uint8_t BatteryError;
extern uint8_t BatteryStatus;						//0-未充电 1-启动充电 2-稳定充电
extern uint16_t BatteryOffCnt;
extern uint8_t  BatteryFullCnt,BatterySocCnt;
extern uint16_t BatteryVolPre, BatteryVolByKalman, BatteryVolNoKalman;
extern void Battery_Conf(void);
extern void BatterySocCheck(void);
extern void BatteryCharge(void);
#endif
