/******************************************************************************

                  版权所有 (C), 1994-2015, 杭州九阳欧南多小家电有限公司

 ******************************************************************************
  文 件 名   : Actuator.c
  版 本 号   : V1.0
  作    者   : 田宏图
  生成日期   : 2016年4月28日
  最近修改   :
  功能描述   : 边扫、中扫、吸尘电机控制
  函数列表   :
  修改历史   :
  1.日    期   : 2016年4月28日
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

/*----------------------------------------------*
 * 外部函数原型说明                             *
 *----------------------------------------------*/

/*----------------------------------------------*
 * 内部函数原型说明                             *
 *----------------------------------------------*/

/*----------------------------------------------*
 * 全局变量                                     *
 *----------------------------------------------*/
FunctionalState SprayWaterEn,ActuatorEn,CleanEn,DustEn,RagChangeEn;
uint8_t SparyWaterInit,SparyWaterOnCnt;
uint8_t CleanInit = 0,DustInit = 0;
uint16_t SparyWaterOffCnt,CleanPwm,CleanPwmStep,DustPwm,DustPwmStep;
uint16_t CleanPwmOn, DustPwmOn;
uint8_t fanLevelPre, sideBroomLevelPre, midBroomLevelPre;
/*----------------------------------------------*
 * 模块级变量                                   *
 *----------------------------------------------*/
MopTypeDef MopPar;
/*----------------------------------------------*
 * 常量定义                                     *
 *----------------------------------------------*/

/*----------------------------------------------*
 * 宏定义                                       *
 *----------------------------------------------*/
#define MOPSTOPTIME     50      //拖地间隔时间 单位ms

void Actuator_Conf(void)
{
	SprayWaterEn = DISABLE;
	RagChangeEn = DISABLE;
	SparyWaterInit = 0;
    DustPowerOff();
    
    //拖地参数设置
    MopPar.MopDir = MOTOR_RUN_DIR_ZW;
    MopPar.MopEn = DISABLE;
    MopPar.MopLoopCnt = 0;
    MopPar.MopOffCnt = 0;//MOPSTOPTIME;
    MopPar.MopPwm = 7200;//PWM_VAL_MAX ;
    MopPar.MopSwitchEn = DISABLE;
    MopPar.LoopInit = 0;
    MopPar.LoopStatus = 1;
    MopPar.LoopOverflow = 0;
    MopPar.LoopOverflowCnt = 0;
    MopPar.Init = 0;
    
    CleanPwmOn = TIM3_Period / 3;//2;
    DustPwmOn = 10;//TIM3_Period / 2;
    
    fanLevelPre = 0;
    sideBroomLevelPre = 0;
    midBroomLevelPre = 0;
}

//函数50ms调用一次
void Actuator_Deal(void)
{
	
	if(!ActuatorEn)
		return;
	ActuatorEn = DISABLE;
	
	//清扫电机软启动
	if(CleanEn)
	{
		if(0 == CleanInit)
		{
			CleanInit = 1;
			CleanPwm = 2000;
			CleanPwmStep = 100;
		}
		
		if(CleanPwm < (CleanPwmOn) - CleanPwmStep)
		{
			CleanPwm += CleanPwmStep;
		}
        else
        {
            CleanPwm = CleanPwmOn;
        }
		ZSMotorOn(CleanPwm);
	}
	else
	{
		ZSMotorOff();
		CleanInit = 0;
	}
	
	//吸尘电机软启动
	if(DustEn)
	{
		DustPowerOn();
        DustMotorOn(DustPwmOn);
	}
	else
	{
		DustMotorOff();
		DustPowerOff();
		DustInit = 0;
	}
	
	

}


void RagChangeDeal(void)
{
	static uint8_t RagChangeOnInit = 0, RagChangeOffInit = 0;
	static uint16_t RagChangeCnt = 0;
	
	//抹布更换处理
	//1.检测到微动开关信号，电机停止，抹布可更换
	//2.从微动开关闭合状态，电机运行0.5s,抹布安转成功
	//3.标志位RagChangeEn ENABLE=抹布需要更换  DISABLE=抹布需要安装
	if(ENABLE == RagChangeEn)
	{
		if(0 == RagChangeOffInit)
		{
			if(RagChangeSingal)
			{
				XPMotorOff();
				RagChangeOffInit = 1;
			}
			else
			{
				XPMotorOn(TIM3_Period / 3);
			}
		}
		else
		{
			XPMotorOff();
		}
		RagChangeOnInit = 0;
	}
	else
	{
		if(0 == RagChangeOnInit)
		{
			if(RagChangeSingal)
			{
				XPMotorOff();
				RagChangeOnInit = 1;
				RagChangeCnt = 280;
			}
			else
			{
				XPMotorOn(TIM3_Period / 3);
			}
		}
		else
		{
			if(RagChangeCnt > 0)
			{
				RagChangeCnt--;
				XPMotorOn(TIM3_Period / 3);
			}
			else
			{
				XPMotorOff();
			}
		}
		RagChangeOffInit = 0;
		
	}
}

uint16_t timeDiffer[4];

//1ms 执行一次该函数
void MopCtrl(void)
{
    static uint8_t RagChangeSingalBak = 0;//, MopTurnDir, MopPos = 0;
    
    //RagChangeSingal = 1 信号被遮挡
    //RagChangeSingal = 0 信号未被遮挡
    
    
    //初始化位置
    if(!MopPar.Init)
    {
        static uint16_t highLvTime, lowLvTime, timeCnt = 0, circleTurn = 0;//, timeDiffer[4];
        static uint8_t MopPosCheckStart = 0;
//        MopPar.Init = 1;
//        RagChangeSingalBak = RagChangeSingal;
//        if(RagChangeSingal)
//        {
//            MopPar.Pos = 1;
//            MopPar.StartPos = MopPar.Pos - 1;
//            MopPar.StopPos = MopPar.Pos + 1;
//        }
//        else
//        {
//            MopPar.Pos = 0;
//            MopPar.StartPos = MopPar.Pos;
//            MopPar.StopPos = MopPar.Pos + 2;
//        }
        MOP_B_RUN(MopPar.MopPwm);
        timeCnt++;
        if(RagChangeSingalBak != RagChangeSingal)
        {
            //上升沿
            if(0 == RagChangeSingal)
            {
                //第一次进入
                if(0 == MopPosCheckStart)
                {
                    MopPosCheckStart = 1;
                    timeCnt = 0;
                    highLvTime = 0;
                    lowLvTime = 0;
                    circleTurn = 0;
                }
                else
                {
                    lowLvTime = timeCnt;
                    timeDiffer[circleTurn] = Get_DataDiffABS(lowLvTime, highLvTime);
                    timeCnt = 0;
                    highLvTime = 0;
                    
                    if(timeDiffer[circleTurn] > 250)
                    {
                        MopPar.Pos = 2;
                        MopPar.StartPos = 0;
                        MopPar.StopPos = 2;
                        MopPar.Init = 1;
                        MOP_BRAKE();
                    }
                    
                    circleTurn++;
 
                    //已经旋转了一圈
                    if(circleTurn >= 4)
                    {
                        uint16_t max = 0;
                        uint8_t i, max_i;
                        for(i = 0; i < 4; i++)
                        {
                            if(max < timeDiffer[i])
                            {
                                max = timeDiffer[i];
                                max_i = i;
                            }
                        }
                        

                        MopPar.Pos = (8 - 2 * max_i) & 0x07;
                       
                        MopPar.StartPos = 0;
                        MopPar.StopPos = 2;
                        MopPar.Init = 1;
                        MOP_BRAKE();
                    }
                }
            }
            //下降沿
            else if(MopPosCheckStart)
            {
                highLvTime = timeCnt;
                timeCnt = 0;
            }
        }
        
        RagChangeSingalBak = RagChangeSingal;
        return;
    }
    
    //位置计算
    if(RagChangeSingalBak != RagChangeSingal)
    {
        //上升沿
        //MopTurnDir 1--顺时针转  0--逆时针转
        if(RagChangeSingal)
        {
            if(RagChangeSingalB)
            {
                MopPar.TurnDir = 1;
            }
            else
            {
                MopPar.TurnDir = 0;
            }
        }
        //下降沿
        else
        {
            if(RagChangeSingalB)
            {
                MopPar.TurnDir = 0;
            }
            else
            {
                MopPar.TurnDir = 1;
            }
        }
        
        //信号发生变化是更新拖布位置
        // MopTurnDir = 1 顺时针 + ； MopTurnDir = 0 逆时针-
        if(MopPar.TurnDir)
        {
            MopPar.Pos++;
        }
        else
        {
            MopPar.Pos--;
        }
        //范围0~7 防止溢出(+/-溢出)
        MopPar.Pos &= 0x07;
        
        RagChangeSingalBak = RagChangeSingal;
        LED4_SW;
    }
    
    //停止处理，
    if(!MopPar.MopEn)
    {
        //每次拖布停止的位置都停止在齿位置
        
        //间隙位置
        if(MopPar.Pos & 0x01)
        {
            if(0 == MopPar.LoopOverflow)
            {
                if(MopPar.MopDir == MOTOR_RUN_DIR_ZW)
                {
                    MOP_B_RUN(MopPar.MopPwm);
                }
                else
                {
                    MOP_F_RUN(MopPar.MopPwm);
                }
            }   
            MopPar.MopOffCnt = 0;
        }
        //齿位置
        else
        {
            if(MopPar.MopOffCnt < MOPSTOPTIME)
                MopPar.MopOffCnt++;
            
            if(MopPar.MopOffCnt < MOPSTOPTIME)
            {
                MOP_BRAKE();
            }
            else
            {
                MOP_STOP();
            }
            
            MopPar.LoopStatus = 1;
            MopPar.LoopOverflow = 1;
            MopPar.LoopOverflowCnt = 0;
            RagChangeSingalBak = RagChangeSingal;
        }
        return;
    }
    
    MopPar.LoopOverflow = 0;
    
    //转动处理
    //拖布处于运动状态
    if(MopPar.LoopStatus)
    {
        //未达到停止位置
        if(MopPar.Pos != MopPar.StopPos)
        {
            if(MopPar.MopDir == MOTOR_RUN_DIR_ZW)
            {
                MOP_B_RUN(MopPar.MopPwm);
            }
            else
            {
                MOP_F_RUN(MopPar.MopPwm);
            }
        }        
        else   //达到停止位置
        {
            //拖布状态切换为停止状态
            MopPar.LoopStatus = 0;
            MopPar.MopOffCnt = 0;
        }
    }
    else
    {
        //停止时间到，拖布切换方向继续运行
        //拖布旋转方向切换，StartPos  StopPos互换 
        if(MopPar.MopOffCnt >= MOPSTOPTIME)
        {
            uint8_t tmppos;
            
            //切换为运行
            MopPar.LoopStatus = 1;
            
            //切换方向
            if(MopPar.MopDir == MOTOR_RUN_DIR_ZW)
            {
                MopPar.MopDir = MOTOR_RUN_DIR_FW;
            }
            else
            {
                MopPar.MopDir = MOTOR_RUN_DIR_ZW;
            }
            
            // StartPos he  StopPos 互换
            tmppos = MopPar.StartPos;
            MopPar.StartPos = MopPar.StopPos;
            MopPar.StopPos = tmppos;
        }
        else
        {
            MopPar.MopOffCnt++;
            if(MopPar.MopOffCnt <= MOPSTOPTIME / 2)
            {
                MOP_BRAKE();
            }
            else
            {
                MOP_STOP();
            }
            MopPar.LoopStatus = 0;
        }
    }
    
//    if(!MopPar.MopEn)
//    {
//        MOP_STOP();
//        MopPar.LoopStatus = 1;
//        MopPar.LoopOverflow = 0;
//        MopPar.LoopOverflowCnt = 0;
//        RagChangeSingalBak = RagChangeSingal;
//        return;
//    }
//    
//    if(0 == RagChangeSingal)
//    {              
//        if(0 == MopPar.LoopOverflow || 0 == MopPar.LoopStatus)
//        {
//            if(MopPar.MopOffCnt >= MOPSTOPTIME)
//            {    
//                if(MopPar.MopDir == MOTOR_RUN_DIR_ZW)
//                {
//                    MOP_B_RUN(MopPar.MopPwm);
//                }
//                else
//                {
//                    MOP_F_RUN(MopPar.MopPwm);
//                }
//                MopPar.LoopStatus = 1;
//                
//                MopPar.LoopOverflow = 0;
//                MopPar.LoopOverflowCnt = 0;
//            }
//            else
//            {
//                MopPar.MopOffCnt++;
//                
//                if(MopPar.MopOffCnt <= MOPSTOPTIME / 2)
//                {
//                    MOP_BRAKE();
//                }
//                else
//                {
//                    MOP_STOP();
//                }

//                MopPar.LoopStatus = 0;
//                if(!MopPar.LoopInit)
//                {
//                    MopPar.LoopInit = 1;
//                    MopPar.MopLoopCnt++;
//                    if(MopPar.MopDir == MOTOR_RUN_DIR_ZW)
//                    {
//                        MopPar.MopDir = MOTOR_RUN_DIR_FW;
//                    }
//                    else
//                    {
//                        MopPar.MopDir = MOTOR_RUN_DIR_ZW;
//                    }
//                }
//            }
//        }
//        else
//        {
//            MopPar.LoopOverflowCnt = 1;
//        }
//    }
//    else
//    {
//        if(MopPar.LoopStatus)
//        {
//            MopPar.MopOffCnt = 0;
//            MopPar.LoopInit = 0;
//            
//            if(MopPar.LoopOverflowCnt > 0)
//            {
//                MopPar.LoopOverflow = 0;
//            }
//            
//            if(MopPar.MopDir == MOTOR_RUN_DIR_ZW)
//            {
//                MOP_B_RUN(MopPar.MopPwm);
//            }
//            else
//            {
//                MOP_F_RUN(MopPar.MopPwm);
//            }
//        }
//        else
//        {
//            MopPar.LoopOverflow = 1;
//            MopPar.LoopOverflowCnt = 0;
//            if(MopPar.MopOffCnt >= MOPSTOPTIME)
//            {
//                if(MopPar.MopDir == MOTOR_RUN_DIR_ZW)
//                {
//                    MOP_B_RUN(MopPar.MopPwm);
//                }
//                else
//                {
//                    MOP_F_RUN(MopPar.MopPwm);
//                }
//                MopPar.LoopStatus = 1;
//            }
//            else
//            {
//                MopPar.MopOffCnt++;
//                if(MopPar.MopOffCnt <= MOPSTOPTIME / 2)
//                {
//                    MOP_BRAKE();
//                }
//                else
//                {
//                    MOP_STOP();
//                }
//                MopPar.LoopStatus = 0;
//                
//                if(!MopPar.LoopInit)
//                {
//                    MopPar.LoopInit = 1;
//                    MopPar.MopLoopCnt++;
//                    if(MopPar.MopDir == MOTOR_RUN_DIR_ZW)
//                    {
//                        MopPar.MopDir = MOTOR_RUN_DIR_FW;
//                    }
//                    else
//                    {
//                        MopPar.MopDir = MOTOR_RUN_DIR_ZW;
//                    }
//                }
//            }
//        }

//    }
}

