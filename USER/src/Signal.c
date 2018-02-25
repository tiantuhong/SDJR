/******************************************************************************

                  版权所有 (C), 1994-2015, 杭州九阳欧南多小家电有限公司

 ******************************************************************************
  文 件 名   : Signal.c
  版 本 号   : V1.0
  作    者   : 田宏图
  生成日期   : 2016年4月13日
  最近修改   :
  功能描述   : 扫地机传感器信号检测
  函数列表   :
  修改历史   :
  1.日    期   : 2016年4月13日
    作    者   : 田宏图
    修改内容   : 创建文件

******************************************************************************/

/*----------------------------------------------*
 * 包含头文件                                   *
 *----------------------------------------------*/
#include "stm32f10x.h"
/*----------------------------------------------*
 * 外部变量说明                                 *
 *----------------------------------------------*/
extern u8 new_flag_x;//????????
extern uint16_t K_x; //????
uint16_t PSD_AD;
/*----------------------------------------------*
 * 外部函数原型说明                             *
 *----------------------------------------------*/
uint16_t UltraTime2Dis(uint16_t _UltraRevTime_);

extern int32_t filter_1_x(int32_t NEW_DATA, int32_t OLD_DATA,float k,u8 flag);
/*----------------------------------------------*
 * 内部函数原型说明                             *
 *----------------------------------------------*/
uint16_t PSDVoltoDis(uint16_t ad_vol);
/*----------------------------------------------*
 * 全局变量                                     *
 *----------------------------------------------*/


uint8_t LeftBump,RightBump,MidBump, BumpBak;		// 0 - 未碰撞 1--碰撞 
uint8_t LeftWheelLift,RightWheelLift;	            // 0 - 轮子抬起 1--轮子着陆
uint8_t WheelLift;						            //左右轮有一个被抬起，即认为被抬起 
//uint8_t WallSignal[5];				            //前方障碍物信息，0-无障碍物1-有障碍物
uint8_t RagChangeSingal,RagChangeSingalB;				            //抹布更换位置信号，0-位置未检测到 1-位置检测到
FunctionalState BumpSendEn,WheelLiftSendEn,WallSendEn,WallCheckEn,SignalCheckEn;
WallCheckFrame WallData[5],EarthData[EARTH_CHECK_NUM];
Indr_TypeDef Indr[4];

FunctionalState BumpFlag, BumpOverFlag, BumpOverForCloseWallFlag;

FunctionalState Ultra_En,Ultra_Check_En[3],Ultra_Rev_OK[3],Ultra_Rev_First[3],Ultra_Jump[3],Ultra_Rising[3],Ultra_Falling[3];
uint8_t Ultra_ID,Ultra_RevID,Ultra_RevLv[3],Ultra_RevCnt[3] = {0,0,0};
uint16_t Ultra_RevTime[3],Ultra_RevTimeVlid[3],Ultra_RevTimeVlidBak[3];
uint32_t Ultra_RevTimeSum[3] = {0,0,0};
uint8_t UltraObs;   //超声波障碍物标志，< 2cm 有障碍物 >2.5没有障碍物
uint8_t UltraDecelerationObs; //超声波减速障碍物标志位， 10cm有障碍物需要减速，大于10cm不需要
uint16_t PSD_Distance;



/*----------------------------------------------*
 * 模块级变量                                   *
 *----------------------------------------------*/

/*----------------------------------------------*
 * 常量定义                                     *
 *----------------------------------------------*/

/*----------------------------------------------*
 * 宏定义                                       *
 *----------------------------------------------*/
//#define EARTH_CHECK_SIGNAL_IN
#define  LIGHTWEIGHT_OBSTACLE_CHECK_EN
#define  ULTRA_JUMP_VAL		300
#define  ULTRA_CHANGE_VAL	15

#define INDR_HEADBYTE_MAX	34000
#define INDR_HEADBYTE_MIN	30000

#define INDR_LONGBYTE_MAX	3000
#define INDR_LONGBYTE_MIN	2500

#define INDR_SHORTBYTE_MAX	800
#define INDR_SHORTBYTE_MIN	600

#define INDR_REV_FREE_TIME	10   //单位ms,红外接收帧之间的空闲时间
#define INDR_REV_LOST_TIME	100   //单位ms,红外接收丢失时间

#define PSD_K               81269   //PSD传感器系数，用于将PSD电压转换为距离 该系数与PSD的参数有关
#define PSD_K2              27      //PSD传感器系数，用于将PSD电压转换为距离 该系数与PSD距离机器边界的距离有关

#define OBS_DISTANCE        450     //轻质障碍物判定距离
#define ULTRAOBSCHECK_FRONT 590     //超声波障碍物检测，减速距离

void Signal_Conf(void)
{
	uint8_t i;
	LeftBump = 0;
	RightBump = 0;
	MidBump = 0;
	LeftWheelLift = 0;
	RightWheelLift = 0;
	WheelLift = 0;
	RagChangeSingal = 0;

	BumpSendEn = DISABLE;
	BumpOverFlag = ENABLE;
    BumpOverForCloseWallFlag = DISABLE;
	WheelLiftSendEn = DISABLE;
	WallSendEn = DISABLE;
	WallCheckEn = DISABLE;//ENABLE;
	BumpFlag = DISABLE;
    BumpBak = 0;
    UltraObs = 0;
    PSD_Distance = 0;
    
    GarbageStatus = STATUS_GARBAGE_BOX_PLUGIN;

	for(i = 0; i < 5; i++)
	{
		WallData[i].Base = 0x0000;
		WallData[i].Sum = 0;
		WallData[i].OK = 0;
		WallData[i].CheckCnt = 0;
		WallData[i].Cnt = 0;
		WallData[i].Delt = 0;
	}
	for(i = 0; i < EARTH_CHECK_NUM; i++)
	{
		EarthData[i].Base = 0xffff;
		EarthData[i].Sum = 0;
		EarthData[i].OK = 0;
		EarthData[i].CheckCnt = 0;
		EarthData[i].Cnt = 0;
		EarthData[i].Delt = 0;
		EarthData[i].Data_Pre = 0;
	}

//	WALLPOWERON;

//	EARTHPOWERON;
//	INFRASIGNALON;
//	SpeedLEDEn;
}

void SignalCheck(void)
{
    static uint16_t Ultra_Obs_Cnt = 0, Ultra_DecelerationObs_Cnt = 0;
	uint8_t TmpLeftBumpPre,TmpRightBumpPre,TmpWheelLiftPre,TmpWallSignal[5],i;
	
	if(!SignalCheckEn)
		return;
	SignalCheckEn = DISABLE;
	
	TmpRightBumpPre = RightBump;
	TmpLeftBumpPre = LeftBump;
	TmpWheelLiftPre = WheelLift;

	for(i = 0;i < 5;i++)
	{
		TmpWallSignal[i] = WallData[i].OK;
	}

/////碰撞检测	
	if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_0))
	{
		RightBump = 0;
	}
	else
	{
		//GPIO_WriteBit(GPIOD, GPIO_Pin_4, Bit_RESET);
		RightBump = 1;
	}

	if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_1))
	{
		LeftBump = 0;
	}
	else
	{
		LeftBump = 1;
	}
	

	if(RightBump != TmpRightBumpPre)
	{
//		BumpSendEn = ENABLE;
		
		if(RightBump == 1)
		{
			RightMotorStop();
			LeftMotorStop();
            
            //清楚左右轮换向启动标志 ，否则会导致误启动
            LeftMotorSwDirFlag = RESET;
			RightMotorSwDirFlag = RESET;
            
			BumpFlag = ENABLE;
			BumpOverFlag = DISABLE;
            BumpOverForCloseWallFlag = DISABLE;
            
            BumpBak |= 0x02;
		}
	}
	if(LeftBump != TmpLeftBumpPre)
	{
//		BumpSendEn = ENABLE;
		
		if(LeftBump == 1)
		{
			RightMotorStop();
			LeftMotorStop();
            
            
            //清楚左右轮换向启动标志 ，否则会导致误启动
            LeftMotorSwDirFlag = RESET;
			RightMotorSwDirFlag = RESET;
            
			BumpFlag = ENABLE;
			BumpOverFlag = DISABLE;
            BumpOverForCloseWallFlag = DISABLE;
            
            BumpBak |= 0x01;
		}
	}
///碰撞检测完成

    //RagChangeSingal 遮挡为0 未遮挡为1
	if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_2))
	{
		RagChangeSingal = 0;
	}
	else
	{
		RagChangeSingal = 1;
	}
	
    //RagChangeSingalB 遮挡为0 未遮挡为1
    if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_3))
    {
        RagChangeSingalB = 0;
    }
    else
    {
        RagChangeSingalB = 1;
    }
	
//轮子抬起检测	
	if(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_0))
	{
		RightWheelLift = 1;
	}
	else
	{
		//GPIO_WriteBit(GPIOD, GPIO_Pin_4, Bit_RESET);	
		RightWheelLift = 0;
	}

	if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_5))
	{
		LeftWheelLift = 1;
	}
	else
	{
		//GPIO_WriteBit(GPIOD, GPIO_Pin_4, Bit_RESET);	
		LeftWheelLift = 0;
	}


	if(RightWheelLift == 1 && LeftWheelLift == 1)
	{
		WheelLift = 1;
	}
	else
	{
		WheelLift = 0;
	}

	if(WheelLift != TmpWheelLiftPre)
	{
		WheelLiftSendEn = ENABLE;
		
	}
//轮子抬起检测完成

#if 0
//红外障碍物检测
	for(i = 0;i < 5;i++)
	{
		if(ADCDataValid[i+7] == ENABLE)
		{
			ADCDataValid[i+7] = DISABLE;
			WallData[i].Delt = 0;
			if(WallData[i].Base > ADC_ConvertedValue_Val[i+7])
			{
				WallData[i].Delt = WallData[i].Base - ADC_ConvertedValue_Val[i+7];
			}

			if(WallData[i].Delt > WALLCHECKDELT)
			{
				if(++WallData[i].CheckCnt > 7)
				{
					WallData[i].OK = 1;
					WallData[i].CheckCnt = 7;
				}
			}
			else
			{
				if(WallData[i].CheckCnt < 4)
				{
					WallData[i].CheckCnt = 4;	
				}
				if(--WallData[i].CheckCnt < 4)
				{
					WallData[i].OK = 0;
					WallData[i].CheckCnt = 4;
					
					if((WallData[i].Data_Pre <= ADC_ConvertedValue_Val[i+7] + 5) && (WallData[i].Data_Pre + 5 >= ADC_ConvertedValue_Val[i+7]))
					{
						WallData[i].Sum += ADC_ConvertedValue_Val[i+7];
						if(++WallData[i].Cnt > 63)
						{
							WallData[i].Cnt = 63;
							WallData[i].Base = (uint16_t)((WallData[i].Sum) >> 6);
							WallData[i].Sum -= WallData[i].Base;
						}
					}
				}
				WallData[i].Data_Pre = ADC_ConvertedValue_Val[i+7];
			}
		}
	}
#endif
    
    //PSD 距离计算
    if(ADCDataValid[7])
    {
        ADCDataValid[7] = DISABLE;
                
        PSD_AD = filter_1_x(ADC_ConvertedValue_Val[7], PSD_AD, K_x, new_flag_x);
        
        PSD_Distance = PSDVoltoDis(PSD_AD);
    }
//#if 0
//	if(ADCDataValid[7] == ENABLE)
//	{
//		ADCDataValid[7] = DISABLE;
//		
//		if(ADC_ConvertedValue_Val[7] < LWALLADCLIMIT)
//		{
//			WallSignal[0] = 1;
//		}
//		else
//		{
//			WallSignal[0] = 0;
//			
//		}
//		
//		ADCDataValid[8] = DISABLE;
//		if(ADC_ConvertedValue_Val[8] < LMWALLADCLIMIT)
//		{
//			WallSignal[1] = 1;
//			
//		}
//		else
//		{
//			WallSignal[1] = 0;
//			
//		}

//		ADCDataValid[9] = DISABLE;
//		if(ADC_ConvertedValue_Val[9] < MWALLADCLIMIT)
//		{
//			
//			WallSignal[2] = 1;
//			
//		}
//		else
//		{
//			
//			WallSignal[2] = 0;
//			
//		}

//		ADCDataValid[10] = DISABLE;
//		if(ADC_ConvertedValue_Val[10] < RMWALLADCLIMIT)
//		{
//			WallSignal[3] = 1;
//	        
//		}
//		else
//		{
//			WallSignal[3] = 0;
//	        
//		}

//		ADCDataValid[11] = DISABLE;
//		if(ADC_ConvertedValue_Val[11] < RWALLADCLIMIT)
//		{
//			WallSignal[4] = 1;
//		}
//		else
//		{
//			WallSignal[4] = 0;
//	//        GPIO_WriteBit(GPIOD, GPIO_Pin_4, Bit_RESET);
//		}

		for(i = 0 ; i < 5 ; i++ )
		{
			if(WallData[i].OK != TmpWallSignal[i])
			{
				WallSendEn = ENABLE;
			}
		}
//	}
//#endif

//红外障碍物检测完成
#ifdef  EARTH_CHECK_SIGNAL_IN	
////地检检测
	for(i = 0;i < EARTH_CHECK_NUM;i++)
	{
		if(ADCDataValid[i] == ENABLE)
		{
			ADCDataValid[i] = DISABLE;
			EarthData[i].Delt = 0;
			if(EarthData[i].Base < ADC_ConvertedValue_Val[i])
			{
				EarthData[i].Delt = ADC_ConvertedValue_Val[i] - EarthData[i].Base;
			}
            else
            {
                EarthData[i].Delt = 0;
            }

            //第一种
            //1.当前值相对于基准值的增量大于阈值  (当前值发生波动)
            //2.当前值大于设定的限值              (当前值的波动不是由于地板材质的突变造成的，该限值应大于黑色地面的测量值)
            //满足以上两个条件 增加悬空基数
           
            
            //第三种
            //基准值大于限值
            //               (EarthData[i].Delt > ERATHCHECKDELT2 && EarthData[i].Base > BLACKERATHVALUE) ||
			if((EarthData[i].Delt > ERATHCHECKDELT && ADC_ConvertedValue_Val[i] > BLACKERATHVALUE) ||\
               (EarthData[i].Delt > 0 && EarthData[i].Base > 3800 && EarthData[i].Base <= 4096))
			{
				if(++EarthData[i].CheckCnt > 7)
				{
					if(EarthData[i].OK == 0)
					{
						RightMotorStop();
						LeftMotorStop();
                        //清楚左右轮换向启动标志 ，否则会导致误启动
                        LeftMotorSwDirFlag = RESET;
                        RightMotorSwDirFlag = RESET;
					}
					EarthData[i].OK = 1;
					EarthData[i].CheckCnt = 7;				
				}
				
				////1.轮子着陆 2.当前值小于设定的基准值 3.该值变化不大 
				////以上3个条件满足 基准值可更新
				if(ADC_ConvertedValue_Val[i] < ERATHCHECKBASE /*&& WheelLift == 1*/)
				{
//					if((EarthData[i].Data_Pre <= ADC_ConvertedValue_Val[i] + ERATHCHECKWAVE) && (EarthData[i].Data_Pre + ERATHCHECKWAVE >= ADC_ConvertedValue_Val[i]))
//					{
						EarthData[i].Sum += ADC_ConvertedValue_Val[i];
						if(++EarthData[i].Cnt > 31)
						{
							EarthData[i].Cnt = 31;
							EarthData[i].Base = (uint16_t)((EarthData[i].Sum) >> 5);
							EarthData[i].Sum -= EarthData[i].Base;
						}
//					}
					EarthData[i].Data_Pre = ADC_ConvertedValue_Val[i];
				}
				
			}
			else
			{
				if(EarthData[i].CheckCnt < 4)
				{
					EarthData[i].CheckCnt = 4;	
				}
				if(--EarthData[i].CheckCnt < 4)
				{
					EarthData[i].OK = 0;
					EarthData[i].CheckCnt = 4;
					
					if((EarthData[i].Data_Pre <= ADC_ConvertedValue_Val[i] + ERATHCHECKWAVE) && (EarthData[i].Data_Pre + ERATHCHECKWAVE >= ADC_ConvertedValue_Val[i]))
					{
//                    if(ADC_ConvertedValue_Val[i] < ERATHCHECKBASE)
//                    {
						EarthData[i].Sum += ADC_ConvertedValue_Val[i];
						if(++EarthData[i].Cnt > 31)
						{
							EarthData[i].Cnt = 31;
							EarthData[i].Base = (uint16_t)((EarthData[i].Sum) >> 5);
							EarthData[i].Sum -= EarthData[i].Base;
						}
//                    }
					}
				}
				EarthData[i].Data_Pre = ADC_ConvertedValue_Val[i];
			}
		}
	}
////地检检测
#endif  
       
#ifdef LIGHTWEIGHT_OBSTACLE_CHECK_EN   
    //超声波检测轻质障碍物
    if(RobitStatus)
    {
        //1.超声波距离小于 设定值
        //2.持续时间大于2S
        if(Ultra_RevTimeVlid[0] <= OBS_DISTANCE)
        {
            if(++Ultra_Obs_Cnt >= 2000)
            {
                if(0 == UltraObs)
                {
                    RightMotorStop();
                    LeftMotorStop();
                    //清楚左右轮换向启动标志 ，否则会导致误启动
                    LeftMotorSwDirFlag = RESET;
                    RightMotorSwDirFlag = RESET;
                }
                UltraObs = 1;
            }
        }
        else
        {
            Ultra_Obs_Cnt = 0;
            UltraObs = 0;    
        }
        
        //1.超声波距离小于设定值
        //2.持续时间大于200ms,至少2次数据
        if(Ultra_RevTimeVlid[0] <= ULTRAOBSCHECK_FRONT)
        {
            if(++Ultra_DecelerationObs_Cnt >= 200)
            {
                UltraDecelerationObs = 1;
            }
        }
        else
        {
            Ultra_DecelerationObs_Cnt = 0;
            UltraDecelerationObs = 0;
        }
    }
    else
    {
        Ultra_Obs_Cnt = 0;
        Ultra_DecelerationObs_Cnt = 0;
    }
#endif   
//充电座连接检测
	if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_11))
	{
		SocketPower = ENABLE;
        BatteryStatus = 1;
	}
	else
	{
		SocketPower = DISABLE;
        BatteryStatus = 0;
	}
    
//    if(BATTERY_CHARGEING_STA)
//    {
//        BatteryStatus = 0;
//    }
//    else
//    {
//        if(BATTERY_COMPLET_STA)
//        {
//            BatteryStatus = 1;
//        }
//        else
//        {
//            BatteryStatus = 2;
//        }
//    }

//充电座检测完成

	//红外检测数据计数	
	Indr[0].FreeTimeCnt++;
	Indr[1].FreeTimeCnt++;
	Indr[2].FreeTimeCnt++;
	Indr[3].FreeTimeCnt++;
	if(Indr[0].FreeTimeCnt >= INDR_REV_FREE_TIME)
	{
		Indr[0].FirstFlag = DISABLE;

	}
	if(Indr[1].FreeTimeCnt >= INDR_REV_FREE_TIME)
	{
		Indr[1].FirstFlag = DISABLE;

	}
	if(Indr[2].FreeTimeCnt >= INDR_REV_FREE_TIME)
	{
		Indr[2].FirstFlag = DISABLE;

	}
	if(Indr[3].FreeTimeCnt >= INDR_REV_FREE_TIME)
	{
		Indr[3].FirstFlag = DISABLE;
	}
	Indr[1].RevOKCnt++;
	if(Indr[1].RevOKCnt >= INDR_REV_LOST_TIME)
	{
		Indr[1].RevOK = DISABLE;
	}
	if(Indr[1].RevDataVlid)
	{
//		if((Indr[1].RevDataBak >> 8) == 0xaa)
		if(Indr[1].RevDataBak  == 0xaa58)
		{
			Indr[1].RevOK = ENABLE;
			Indr[1].RevOKCnt = 0;
		}
		Indr[1].RevDataVlid = DISABLE;
	}
	
	Indr[2].RevOKCnt++;
	if(Indr[2].RevOKCnt >= INDR_REV_LOST_TIME)
	{
		Indr[2].RevOK = DISABLE;
	}
	if(Indr[2].RevDataVlid)
	{
//		if((Indr[1].RevDataBak >> 8) == 0xaa)
		if(Indr[2].RevDataBak  == 0xaa58)
		{
			Indr[2].RevOK = ENABLE;
			Indr[2].RevOKCnt = 0;
		}
		Indr[2].RevDataVlid = DISABLE;
	}
	
	Indr[3].RevOKCnt++;
	if(Indr[3].RevOKCnt >= INDR_REV_LOST_TIME)
	{
		Indr[3].RevOK = DISABLE;
	}
	if(Indr[3].RevDataVlid)
	{
//		if((Indr[1].RevDataBak >> 8) == 0xaa)
		if(Indr[3].RevDataBak  == 0xaa58)
		{
			Indr[3].RevOK = ENABLE;
			Indr[3].RevOKCnt = 0;
		}
		Indr[3].RevDataVlid = DISABLE;
	}



    //20170608 暂时屏蔽换拖布功能(该功能已用于旋转拖布)
//	RagChangeDeal();
    MopCtrl();
	UART_SendSignal();
	
}





void delay15us(void)
{
	uint8_t i;
	for(i = 0;i<125;i++)
	{
		Delay_nop();
	}
}

/*****************************************************************************
 函 数 名  : Ultra_Check
 功能描述  : 超声波测距处理
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 

 1. 3个超声波模块依次进行测距处理
 2. 测距时间间隔30ms
 3. 超声波飞行时间通过中断计算
 4. 此函数进行使能处理和测距距离计算
 
 修改历史      :
  1.日    期   : 2016年11月7日
    作    者   : tht
    修改内容   : 新生成函数

*****************************************************************************/
void Ultra_Check(void)
{
	static uint8_t UltraRisingCnt[3], UltraFallingCnt[3];
    static  uint8_t UltraJmupBase[3], UltraJumpCnt[3];//跳变前的超声波值（基准值），检测到的跳变数据的个数 
    uint16_t tmpdata;
	//30ms 进行一次测量
	if(Ultra_En != ENABLE)
		return;
	Ultra_En = DISABLE;

	if(Ultra_Rev_OK[Ultra_ID] == ENABLE)
	{
		Ultra_Rev_OK[Ultra_ID] = DISABLE;

		//数据滤波
//		Ultra_RevTime[Ultra_ID] = ;
        tmpdata = Ultra_RevTimeVlid[Ultra_ID];
        
		if(Ultra_RevCnt[Ultra_ID] < 4)
		{
			Ultra_RevCnt[Ultra_ID] ++;
			Ultra_RevTimeSum[Ultra_ID] += Ultra_RevTime[Ultra_ID];
			Ultra_RevTimeVlid[Ultra_ID] = Ultra_RevTime[Ultra_ID];
		}
		else
		{
			Ultra_RevTimeSum[Ultra_ID] += Ultra_RevTime[Ultra_ID];
			Ultra_RevTimeSum[Ultra_ID] -= Ultra_RevTimeVlid[Ultra_ID];
			Ultra_RevTimeVlid[Ultra_ID] = Ultra_RevTimeSum[Ultra_ID] >> 2;
		}
        

        
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////		
		// 当前值在 备份值 ± 15 之间，认为数据保持稳定
		// 当前值小于备份值-15，认为数据在减小
		// 当前值大于备份值+15，认为在增加
		//累计3次认为有效
		if(Ultra_RevTimeVlid[Ultra_ID] + ULTRA_CHANGE_VAL <= Ultra_RevTimeVlidBak[Ultra_ID])//&& Ultra_RevTimeVlid[Ultra_ID] <= Ultra_RevTimeVlidBak[Ultra_ID] + ULTRA_CHANGE_VAL)
		{
			Ultra_RevTimeVlidBak[Ultra_ID] = Ultra_RevTimeVlid[Ultra_ID];
			UltraFallingCnt[Ultra_ID]++;
			
			if(UltraFallingCnt[Ultra_ID] >= 3)
			{
				Ultra_Falling[Ultra_ID] = ENABLE;
				Ultra_Rising[Ultra_ID] = DISABLE;
			}
		}
		else if(Ultra_RevTimeVlid[Ultra_ID] >= Ultra_RevTimeVlidBak[Ultra_ID] + ULTRA_CHANGE_VAL)
		{
			Ultra_RevTimeVlidBak[Ultra_ID] = Ultra_RevTimeVlid[Ultra_ID];
			UltraRisingCnt[Ultra_ID]++;
			
			if(UltraRisingCnt[Ultra_ID] >= 3)
			{
				Ultra_Rising[Ultra_ID] = ENABLE;
				Ultra_Falling[Ultra_ID] = DISABLE;
			}
		}
		else
		{
			Ultra_Rising[Ultra_ID] = DISABLE;
			Ultra_Falling[Ultra_ID] = DISABLE;
			UltraFallingCnt[Ultra_ID] = 0;
			UltraRisingCnt[Ultra_ID] = 0;
		}
///////////////////////////////////////////////////////////////////////////////////////////////////////////
//		Ultra_RevTimeVlidBak[Ultra_ID] = Ultra_RevTimeVlid[Ultra_ID];
        
        //检测超声波数据跳变
        if(0 == UltraJumpCnt[Ultra_ID])
        {
            if(Ultra_RevTimeVlid[Ultra_ID] > tmpdata + ULTRA_JUMP_VAL || Ultra_RevTimeVlid[Ultra_ID] + ULTRA_JUMP_VAL < tmpdata)
            {
                UltraJmupBase[Ultra_ID] = tmpdata;
                UltraJumpCnt[Ultra_ID] ++;				
            }
            else
            {
                UltraJumpCnt[Ultra_ID] = 0;
                Ultra_Jump[Ultra_ID] = DISABLE;
            }
        }
        
        if(UltraJumpCnt[Ultra_ID] > 0 )
        {
            if((Ultra_RevTimeVlid[Ultra_ID] > UltraJmupBase[Ultra_ID] + ULTRA_JUMP_VAL || Ultra_RevTimeVlid[Ultra_ID] + ULTRA_JUMP_VAL < UltraJmupBase[Ultra_ID]))
            {
                UltraJumpCnt[Ultra_ID] ++;
                if(UltraJumpCnt[Ultra_ID] >= 3)
                {
                    Ultra_Jump[Ultra_ID] = ENABLE;
                    UltraJumpCnt[Ultra_ID] = 0;
                }
            }
            else
            {
                UltraJumpCnt[Ultra_ID] = 0;
                Ultra_Jump[Ultra_ID] = DISABLE;
            }
        }
	}

	Ultra_Check_En[0] = DISABLE;
	Ultra_Check_En[1] = DISABLE;
	Ultra_Check_En[2] = DISABLE;

	Ultra_ID ++;
	if(Ultra_ID >= 3)
	{
		Ultra_ID = 0;
	}

	switch(Ultra_ID)
	{
		case 0:
			ULTRAFRONTTRIC_H;
			delay15us();
			ULTRAFRONTTRIC_L;
			break;
		case 1:
			ULTRALEFFRONTTRIC_H;
			delay15us();
			ULTRALEFFRONTTRIC_L;
			break;
		case 2:
			ULTRALEFBEHTRIC_H;
			delay15us();
			ULTRALEFBEHTRIC_L;
			break;
	}

	Ultra_Check_En[Ultra_ID] = ENABLE;
	Ultra_Rev_OK[Ultra_ID]	= DISABLE;
	Ultra_Rev_First[Ultra_ID] = DISABLE;
	

	
}

/*****************************************************************************
 函 数 名  : Ultra_Deal
 功能描述  : 超声波数据处理，中断函数中调用
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2016年11月7日
    作    者   : tht
    修改内容   : 新生成函数

*****************************************************************************/
void Ultra_Deal(void)
{
//	uint16_t tmpdata;
//	static  uint8_t UltraJmupBase[3], UltraJumpCnt[3];//跳变前的超声波值（基准值），检测到的跳变数据的个数 
	//使能ID 和接收ID 是否一致
	if(Ultra_ID != Ultra_RevID)
		return;

	//是否使能
	if(Ultra_Check_En[Ultra_ID] != ENABLE)
		return;

	if(Ultra_Rev_OK[Ultra_ID] == ENABLE)
		return;
	
	if(Ultra_RevLv[Ultra_ID] == 1)
	{
		//上升沿
		if(Ultra_Rev_First[Ultra_ID] != ENABLE)
		{
			//开启定时器
			TIM_SetCounter(TIM7,0);
			TIM_Cmd(TIM7,ENABLE);
			Ultra_Rev_First[Ultra_ID] = ENABLE;
			Ultra_Rev_OK[Ultra_ID] = DISABLE;

		}
	}
	else
	{
		if(Ultra_Rev_First[Ultra_ID] == ENABLE)
		{
			//关闭定时器
			TIM_Cmd(TIM7,DISABLE);
			//读取定时值
//			tmpdata = Ultra_RevTime[Ultra_ID];
			Ultra_RevTime[Ultra_ID] = TIM_GetCounter(TIM7);
			
			Ultra_Rev_OK[Ultra_ID] = ENABLE;
			Ultra_Check_En[Ultra_ID] = DISABLE;
			
			
			//检测超声波数据跳变
//			if(0 == UltraJumpCnt[Ultra_ID])
//			{
//				if(Ultra_RevTime[Ultra_ID] > tmpdata + ULTRA_JUMP_VAL || Ultra_RevTime[Ultra_ID] + ULTRA_JUMP_VAL < tmpdata)
//				{
//					UltraJmupBase[Ultra_ID] = tmpdata;
//					UltraJumpCnt[Ultra_ID] ++;				
//				}
//				else
//				{
//					UltraJumpCnt[Ultra_ID] = 0;
//                    Ultra_Jump[Ultra_ID] = DISABLE;
//				}
//			}
//			
//			if(UltraJumpCnt[Ultra_ID] > 0 )
//			{
//                if((Ultra_RevTime[Ultra_ID] > UltraJmupBase[Ultra_ID] + ULTRA_JUMP_VAL || Ultra_RevTime[Ultra_ID] + ULTRA_JUMP_VAL < UltraJmupBase[Ultra_ID]))
//				{
//                    UltraJumpCnt[Ultra_ID] ++;
//                    if(UltraJumpCnt[Ultra_ID] >= 3)
//                    {
//                        Ultra_Jump[Ultra_ID] = ENABLE;
//                        UltraJumpCnt[Ultra_ID] = 0;
//                    }
//                }
//                else
//                {
//                    UltraJumpCnt[Ultra_ID] = 0;
//                    Ultra_Jump[Ultra_ID] = DISABLE;
//                }
//			}
		}
	}
	
}

uint16_t UltraTime2Dis(uint16_t _UltraRevTime_)
{
	uint16_t tmpdata;
	tmpdata = _UltraRevTime_;
	tmpdata *= 17;
	tmpdata /= 100;
	
	return tmpdata;
}


void IndrCheck(uint8_t indrid, uint8_t swmode)
{
	uint8_t tmpid, tmpmode;
	uint16_t tmptime,tmpdelt;
	
	tmpid = indrid;
	tmpmode = swmode;
	
	if(indrid >= 4)
	{
		return;
	}
		
	tmptime = TIM_GetCounter(TIM1);
//    test1 = tmptime;
//
	if(DISABLE == Indr[tmpid].FirstFlag)
	{
		if(0 == tmpmode)
		{
			Indr[tmpid].FirstFlag = ENABLE;
			Indr[tmpid].PreTime = tmptime;
			Indr[tmpid].FreeTimeCnt = 0;
			Indr[tmpid].RevByte = 0;
//            Test2_Sw;            
		}

	}
	else
	{       
        
        
		if(Indr[tmpid].PreTime < tmptime)
		{
			tmpdelt = tmptime - Indr[tmpid].PreTime;
		}
		else
		{
			tmpdelt = tmptime + (0xffff - Indr[tmpid].PreTime);
		}
//        test2 = tmpdelt;
		if(tmpdelt <= INDR_LONGBYTE_MAX && tmpdelt >= INDR_LONGBYTE_MIN)
		{
			Indr[tmpid].RevData |= 0x01;
			Indr[tmpid].RevData <<= 1;
			Indr[tmpid].RevByte++;
			Indr[tmpid].PreTime = tmptime;
			Indr[tmpid].FreeTimeCnt = 0;
			
			if(Indr[tmpid].RevByte >= 15)
			{
				Indr[tmpid].RevDataBak = Indr[tmpid].RevData;
				Indr[tmpid].RevDataVlid = ENABLE;
			}
            
		}
		else if(tmpdelt <= INDR_SHORTBYTE_MAX && tmpdelt >= INDR_SHORTBYTE_MIN)
		{
			Indr[tmpid].RevData &= ~0x01;
			Indr[tmpid].RevData <<= 1;
			Indr[tmpid].RevByte++;
			Indr[tmpid].PreTime = tmptime;
			Indr[tmpid].FreeTimeCnt = 0;
			
			if(Indr[tmpid].RevByte >= 15)
			{
				Indr[tmpid].RevDataBak = Indr[tmpid].RevData;
				Indr[tmpid].RevDataVlid = ENABLE;
			}
            
		}
		else
		{
			Indr[tmpid].FreeTimeCnt = 0;
            
		}
	}
}

uint16_t PSDVoltoDis(uint16_t ad_vol)
{
    uint32_t psd_k;
    
    psd_k = PSD_K << 3;
    if(ad_vol < 3009)
    {
        psd_k = psd_k / ad_vol - (PSD_K2 << 3);
    }
    else
    {
        psd_k = 0;
    }
    
    return (uint16_t)psd_k;
    
}


//超声波中断处理函数
void EXTI2_IRQHandler(void)
{
	if ( EXTI_GetITStatus(EXTI_Line2) != RESET )
	{
		EXTI_ClearITPendingBit(EXTI_Line2);
		
        
        
		Ultra_RevID = 0;

		if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_2) == SET)
		{
			Ultra_RevLv[0] = 1;	
		}
		else
		{
			Ultra_RevLv[0] = 0;	
		}
		
		Ultra_Deal();
	}
}
