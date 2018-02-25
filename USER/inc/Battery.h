#ifndef _BATTERY_H
#define _BATTERY_H

//#define Set_Charge_PWM(_Num_)		TIM_SetCompare1(TIM3,_Num_)
#define BATTERY_CHARGE_CURRENT		1800
#define BATTERY_CHARGE_CURRENT_MIN	0x30
#define BATTERY_CHARGE_CURRENT_FULL	500

#define BATTERY_AD2mV_K             (3300 * 11) >> 10


#define BATTERY_EN                  //GPIO_WriteBit(GPIOD,GPIO_Pin_10,Bit_SET)
#define BATTERY_DIS                 //GPIO_WriteBit(GPIOD,GPIO_Pin_10,Bit_RESET)
#define BATTERY_CHARGEING_STA       //GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_11)      //0=���ڳ�磬1=δ���
#define BATTERY_COMPLET_STA         //GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_12)      //0=�����ɣ�1=���δ���
#define BATTERY_ERROR_STA           //GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_13)      //0=�����ϣ�1=�������

extern uint8_t BatterySoc, BatterySocDis;	//���ʣ����� ���100 ��С0
extern uint8_t BatteryConnect;//0--δ���� 1--����
extern uint16_t BatteryVol2Soc[11];
extern FunctionalState SocketPower,JackPower,BatteryCheckEn,BatteryChargeEn;
extern uint8_t BatteryError;
extern uint8_t BatteryStatus;						//0-δ��� 1-������� 2-�ȶ����
extern uint16_t BatteryOffCnt;
extern uint8_t  BatteryFullCnt,BatterySocCnt;
extern uint16_t BatteryVolPre, BatteryVolByKalman, BatteryVolNoKalman;
extern void Battery_Conf(void);
extern void BatterySocCheck(void);
extern void BatteryCharge(void);
#endif
