/******************************************************************************

                  版权所有 (C), 1994-2016, 杭州九阳欧南多小家电有限公司

 ******************************************************************************
  文 件 名   : Battery.c
  版 本 号   : V1.0
  作    者   : 田宏图
  生成日期   : 2016年4月25日
  最近修改   :
  功能描述   : 电池功能模块
  函数列表   :
  修改历史   :
  1.日    期   : 2016年4月25日
    作    者   : 田宏图
    修改内容   : 创建文件

******************************************************************************/

/*----------------------------------------------*
 * 包含头文件                                   *
 *----------------------------------------------*/
#include "stm32f10x.h"
#include "kalman_filter.h"
/*----------------------------------------------*
 * 外部变量说明                                 *
 *----------------------------------------------*/

/*----------------------------------------------*
 * 外部函数原型说明                             *
 *----------------------------------------------*/

/*----------------------------------------------*
 * 内部函数原型说明                             *
 *----------------------------------------------*/
uint16_t BatteryVolCorrect(uint16_t _vol_);
/*----------------------------------------------*
 * 全局变量                                     *
 *----------------------------------------------*/
uint8_t BatterySoc, BatterySocDis;				//电池剩余电量 最大100 最小0
uint8_t BatteryStatus;						//0-未充电 1-正在充电 2-充满电
uint8_t BatteryChargeReadyStatus;			//电池充电准备状态 0，1，2
uint8_t BatteryConnect;						//0--未连接 1--连接
uint16_t BatteryChargePWMVal;				//充电控制PWM值
FunctionalState SocketPower,JackPower,BatteryCheckEn,BatteryChargeEn;
uint8_t BatteryError;
uint16_t BatteryOffCnt;
/*----------------------------------------------*
 * 模块级变量                                   *
 *----------------------------------------------*/
uint8_t ChargeCnt,ChargeStartCnt;
uint16_t BatteryChargeStartCur;
uint16_t BatteryVolPre,BatteryVolold;
uint8_t  BatteryFullCnt,BatterySocCnt;
uint16_t BatteryFullSmallCurCnt;
FunctionalState BatteryChargeFullEn,KalmanInit,BattertDisInit;

kalman1_state BatteryKalman;
uint16_t BatteryVolByKalman, BatteryVolNoKalman;
/*----------------------------------------------*
 * 常量定义                                     *
 *----------------------------------------------*/
uint16_t BatteryVol2Soc[11] = 
{
	// 100%/16.8V   90%/16.24  80%/15.92   70%/15.68  60%/15.48  50%/15.28  40%/15.16  30%/15.08  20% 14.96  10%/14.72  0%/12V
	     0x767,      0x728,     0x704,      0x6e9,     0x6d2,     0x6bc,      0x6b9,     0x6a5,     0x698,     0x67c,    0x54a
};

/*----------------------------------------------*
 * 宏定义                                       *
 *----------------------------------------------*/

/*****************************************************************************
 函 数 名  : Battery_Conf
 功能描述  : 电池参数初始化
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2016年4月25日
    作    者   : 田宏图
    修改内容   : 新生成函数

*****************************************************************************/
void Battery_Conf(void)
{
	BatterySoc = 0;	
    BatterySocDis = 0;
	SocketPower = DISABLE;
	JackPower = DISABLE;
	BatteryCheckEn = ENABLE;
	BatteryChargeEn = DISABLE;
	BatteryChargeFullEn = ENABLE;
	BatteryConnect = 1;
	BatteryChargePWMVal = 0;
	ChargeCnt = 0;
	ChargeStartCnt = 0;
	BatteryOffCnt = 0;
	BatteryFullCnt = 0;
	BatterySocCnt = 0;
	BatteryFullSmallCurCnt = 0;
	BatteryError = 0;
	BatteryChargeStartCur = 0;
	BatteryVolPre = 0;
    BATTERY_EN;
    
    KalmanInit = DISABLE;
    BattertDisInit = DISABLE;
    
}


/*****************************************************************************
 函 数 名  : BatterySocCheck
 功能描述  : 电池电量检测
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 

 1. 1s调用一次
 2.充电时，关闭充电检测开路电压
 
 修改历史      :
  1.日    期   : 2016年4月25日
    作    者   : 田宏图
    修改内容   : 新生成函数

*****************************************************************************/
void BatterySocCheck(void)
{
	uint8_t i;
    static uint16_t SocDisCnt = 0;
	
	if(!BatteryCheckEn)
		return;

	if(!ADCDataValid[2])
		return;
	
    if(!KalmanInit)
    {
        kalman1_init(&BatteryKalman, ADC_ConvertedValue_Val[2], 500, 0.1, 800);
        KalmanInit = ENABLE;
    }   
    
	if(BatteryStatus != 0)
	{
//		Set_Charge_PWM(0);
		
		//延时10ms
		if(BatteryOffCnt < 10)
			return;
		
        BatteryVolByKalman = kalman1_filter(&BatteryKalman, ADC_ConvertedValue_Val[2]);
        
        SocDisCnt++;
        if(SocDisCnt >= 250)
        { 
            SocDisCnt = 0;
            
            if(BatteryVolByKalman >= BatteryVolPre)
            {
                BatteryFullCnt = 0;

                if(BatteryVolByKalman < BatteryVolold + 10 )
                        BatteryVolPre = BatteryVolByKalman;
                
                BatteryVolold = BatteryVolByKalman;
            }
            else
            {								
                BatteryFullCnt ++;
            }
            
            if(0)//BATTERY_CHARGEING_STA == 1 && BATTERY_COMPLET_STA == 0)
            {
                BatterySoc = 100;
            }
            else
            {
                for(i = 0 ;i < 10;i++)
                {
                    
                    if(BatteryVolByKalman < BatteryVol2Soc[i] && BatteryVolByKalman >= BatteryVol2Soc[i+1])
                    {
                        BatterySoc = (9 - i) * 10;	
                        break;
                    }
                }
            }
            
            if(!BattertDisInit)  
            {
                BatterySocDis = BatterySoc;
                BattertDisInit = ENABLE;
            }  
            
            if(BatterySocDis < BatterySoc)
            {
                BatterySocDis++;
            }
        }
	}
	else
	{
        BatteryVolNoKalman = BatteryVolCorrect(ADC_ConvertedValue_Val[2]);
        BatteryVolByKalman = kalman1_filter(&BatteryKalman, BatteryVolNoKalman);
        
		BatteryVolPre = 0;
        
        SocDisCnt++;
        if(SocDisCnt >= 250)
        {          
            SocDisCnt = 0;
            
            if(BatteryVolByKalman >= BatteryVol2Soc[0])
            {
                BatterySoc = 100;	
            }
            else 
            {
                for(i = 0 ;i < 10;i++)
                {
                    
                    if(BatteryVolByKalman < BatteryVol2Soc[i] && BatteryVolByKalman >= BatteryVol2Soc[i+1])
                    {                       
                        BatterySoc = (9 - i) * 10 + (BatteryVolByKalman - BatteryVol2Soc[i+1]) * 10 / (BatteryVol2Soc[i] - BatteryVol2Soc[i+1]) ;	
                        break;
                    }
                }
            }
            
            if(!BattertDisInit)  
            {
                BatterySocDis = BatterySoc;
                BattertDisInit = ENABLE;
            } 
            
            if(BatterySocDis > BatterySoc)
            {
                BatterySocDis--;
            }
        }
	}
	ADCDataValid[2] = DISABLE;
	BatteryCheckEn = DISABLE;
}


uint16_t BatteryVolCorrect(uint16_t _vol_)
{
    if(RobitStatus)
    {
        if(CleanEn && DustEn && MopPar.MopEn)
        {
            return (_vol_ + 70);
        }
        else
        {
            return (_vol_ + 25);
        }
    }
    else
    {
        if(CleanEn && DustEn && MopPar.MopEn)
        {
            return (_vol_ + 64);
        }
        else
        {
            return (_vol_ + 19);
        }
    }
}

/*****************************************************************************
 函 数 名  : BatteryCharge
 功能描述  : 电池充电控制
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
1.20ms调用一次
 
 修改历史      :
  1.日    期   : 2016年4月25日
    作    者   : 田宏图
    修改内容   : 新生成函数

*****************************************************************************/
void BatteryCharge(void)
{
//	uint16_t Tmpcurrent;
	if(!BatteryChargeEn)
		return;
	BatteryChargeEn = DISABLE;
	
	//充电座是否连接
	if(SocketPower != ENABLE /*&& JackPower != ENABLE*/)
	{
		//关闭充电
//		Set_Charge_PWM(0);
		BatteryStatus = 0;

		BatteryChargeFullEn = ENABLE;
		
		return;
	}

	//电池是否连接
//	if(BatteryConnect == 0)
//	{
//		//关闭充电
////		Set_Charge_PWM(0);
//		BatteryStatus = 0;
//		return;	
//	}
	
	//电池电量是否充满
//	if(BatteryChargeFullEn == ENABLE)
//	{
//		if(BatterySoc == 100)
//		{
//			if(BatteryStatus != 0)
//			{
//				BatteryStatus = 3;
//			}
//			else
//			{
//				//关闭充电
////				Set_Charge_PWM(0);
//				BatteryStatus = 0;
//				return;	
//			}
//		}	
//	}
//	else
//	{
//		if(BatterySoc >= 90)
//		{
//			//关闭充电
////			Set_Charge_PWM(0);
//			BatteryStatus = 0;
//			return;	
//		}
//	}

	if(BatteryError)
		return;
	
	
	//是否正在检测电池电量(检测电池电量时会关闭充电，这时不做充电处理)
//	if(BatteryCheckEn)
//		return;
	
	//根据电池充电状态不同处理
//	switch(BatteryStatus)
//	{
//		case 0:
//			BatteryStatus = 1;
//			BatteryChargePWMVal = 5;
//			ChargeCnt = 5;
//			ChargeStartCnt = 0;
//			BatteryChargeReadyStatus = 0;
//			break;
//		case 1:
//			//启动充电状态，电流逐步增大至设定电流
//			if(++ChargeCnt <= 5)
//			{
////				Set_Charge_PWM(BatteryChargePWMVal);	
//			}
//			else
//			{
//				ChargeCnt = 0;
//				switch(BatteryChargeReadyStatus)
//				{
//					case 0:
//						//第1秒不检测，滤掉可能的上电冲击
//						if(++ChargeStartCnt <= 10)
//							return;
//						
//						if(ADC_ConvertedValue_Val[2] >= BATTERY_CHARGE_CURRENT_MIN && ADC_ConvertedValue_Val[2] < BATTERY_CHARGE_CURRENT)
//						{
//							BatteryChargeReadyStatus = 1;
//							BatteryChargeStartCur = ADC_ConvertedValue_Val[2];
//						}
//						else
//						{
//							BatteryError = 1;
//							//关闭充电
////							Set_Charge_PWM(0);
//							BatteryStatus = 0;
//							return;
//						}
//						break;
//					case 1:
//						if(BatteryChargePWMVal < 100)
//						{
//							BatteryChargePWMVal += 10;	
//						}
//						else
//						{
//							if(ADC_ConvertedValue_Val[2] > BATTERY_CHARGE_CURRENT_MIN && ADC_ConvertedValue_Val[2] < BATTERY_CHARGE_CURRENT)
//							{
//								BatteryChargeReadyStatus = 2;
//							}
//							else
//							{
//								BatteryError = 2;
//								
//								//关闭充电
//								Set_Charge_PWM(0);
//								BatteryStatus = 0;
//								return;
//							}
//						}
//						
//						Set_Charge_PWM(BatteryChargePWMVal);
//						
//						break;
//					case 2:
//					
////						Tmpcurrent = ( BATTERY_CHARGE_CURRENT * 7 ) >> 3;
//						
//						if(ADC_ConvertedValue_Val[2] < BATTERY_CHARGE_CURRENT)
//						{
//							if(BatteryChargePWMVal < (TIM3_Period - 2))
//								BatteryChargePWMVal += 2;
//							else
//							{
//								BatteryError = 3;
//								
//								//关闭充电
//								Set_Charge_PWM(0);
//								BatteryStatus = 0;
//								return;
//							}
//						}
//						else
//						{
//							BatteryStatus = 2;		
//						}
//						ChargeCnt = 5;
//						Set_Charge_PWM(BatteryChargePWMVal);
//						
//						break;
//					default:
//						break;
//				}

//			}			
//			break;
//		case 2:
//			//稳定充电状态，充电电流恒流控制(0.5A)
//			if(ADC_ConvertedValue_Val[2] > BATTERY_CHARGE_CURRENT)
//			{
//                if(BatteryChargePWMVal > 10)
//                    BatteryChargePWMVal -= 1;
//			}
//			else
//			{
//                if(BatteryChargePWMVal < (TIM3_Period - 1))
//                    BatteryChargePWMVal += 1;
//			}
//			Set_Charge_PWM(BatteryChargePWMVal);
//			
//			BatteryFullSmallCurCnt = 0;
//			break;
//		case 3:	
//			
//			//小电流充电  20分钟然后断开
//			if(ADC_ConvertedValue_Val[2] > BATTERY_CHARGE_CURRENT_FULL)
//			{
//                if(BatteryChargePWMVal > 10)
//                    BatteryChargePWMVal -= 1;
//			}
//			else
//			{
//                if(BatteryChargePWMVal < (TIM3_Period - 1))
//                    BatteryChargePWMVal += 1;
//			}
//			
//			Set_Charge_PWM(BatteryChargePWMVal);

//			if(++BatteryFullSmallCurCnt > 60000)
//			{
//				//关闭充电
//				Set_Charge_PWM(0);
//				BatteryStatus = 0;
//				BatteryChargeFullEn = DISABLE;
//			}
//			
//			break;
//		default:
//			//关闭充电
//			Set_Charge_PWM(0);
//			BatteryStatus = 0;
//			break;
//	}



    
}

