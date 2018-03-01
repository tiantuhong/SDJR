/******************************************************************************

                  °æÈ¨ËùÓÐ (C), 1994-2016, º¼ÖÝ¾ÅÑôÅ·ÄÏ¶àÐ¡¼ÒµçÓÐÏÞ¹«Ë¾

 ******************************************************************************
  ÎÄ ¼þ Ãû   : Robit_Run.c
  °æ ±¾ ºÅ   : V1.0
  ×÷    Õß   : ÌïºêÍ¼
  Éú³ÉÈÕÆÚ   : 2016Äê3ÔÂ16ÈÕ
  ×î½üÐÞ¸Ä   :
  ¹¦ÄÜÃèÊö   : É¨µØ»úÐÐ×ß¿ØÖÆ
  º¯ÊýÁÐ±í   :
  ÐÞ¸ÄÀúÊ·   :
  1.ÈÕ    ÆÚ   : 2016Äê3ÔÂ16ÈÕ
    ×÷    Õß   : ÌïºêÍ¼
    ÐÞ¸ÄÄÚÈÝ   : ´´½¨ÎÄ¼þ

******************************************************************************/

/*----------------------------------------------*
 * °üº¬Í·ÎÄ¼þ                                   *
 *----------------------------------------------*/
#include "stm32f10x.h"
#include <string.h>
#include <math.h>
/*----------------------------------------------*
 * Íâ²¿±äÁ¿ËµÃ÷                                 *
 *----------------------------------------------*/

/*----------------------------------------------*
 * Íâ²¿º¯ÊýÔ­ÐÍËµÃ÷                             *
 *----------------------------------------------*/
void Robit_RunLR(uint8_t LeftMotor_Speed,uint8_t LeftMotor_Dir,uint8_t RightMotor_Speed,uint8_t RightMotor_Dir);
void Robit_RunDir(uint8_t Speed,uint8_t Dir);
void Robit_Stop(void);
/*----------------------------------------------*
 * ÄÚ²¿º¯ÊýÔ­ÐÍËµÃ÷                             *
 *----------------------------------------------*/
int16_t EdageRunAngle(void);
uint16_t CalAngleDif(int16_t Angle1, int16_t Angle2);
/*----------------------------------------------*
 * È«¾Ö±äÁ¿                                     *
 *----------------------------------------------*/
 
Motor_TypeDef LeftMotor,RightMotor;
MotorSpeed_TypeDef LeftMotorSpeed,RightMotorSpeed;
FunctionalState AdjustSpeedEn,MotorLinerModeEn, AccCtrlTimeEn;
int16_t Sys_Startangle;
int16_t AnglerateTarg, AnglerateTargInit;

uint8_t SpeedAdjTime;   //ÂÖ×ÓËÙ¶ÈPIDµ÷½ÚÊ±¼ä

///ËÙ¶ÈµµÎ»»®·ÖÎª0~10  11¸öµµÎ»
uint8_t Robit_RunSpeed;

uint32_t TargMileage,LeftCurMileage,RightCurMileage, TargMileageForArc;
FunctionalState RobitRouteModeEn;
FunctionalState MotorStartupEn,MotorStopCtrlEn,RobitModeCtrlEn;
FunctionalState VaccEn, WaccEn; //Ïß¼ÓËÙÊ¹ÄÜ±êÖ¾ £¬ ½Ç¼ÓËÙ¶ÈÊ¹ÄÜ±êÖ¾
uint8_t VaccInit = 0, WaccInit = 0;

int32_t  LeftMileageAll,RightMileageAll; //×óÓÒÂÖÀÛ¼ÆÀï³ÌÊý£¬Ç°½øÔö¼Ó£¬ºóÍË¼õÐ¡¡£
int32_t  LeftMileageAllPre,RightMileageAllPre; //×óÓÒÂÖÀÛ¼ÆÀï³ÌÊý£¬Ç°½øÔö¼Ó£¬ºóÍË¼õÐ¡¡£

uint8_t  TargCirle;
uint16_t TargAngle; // Éè¶¨Ðý×ª½Ç¶È
uint16_t RotationAngleCur; //µ±Ç°Ðý×ª½Ç¶È
uint16_t StepModeCnt;

///É¨µØ»úÔËÐÐ×´Ì¬  ²ÎÊý

uint8_t RobitStatus;  				// 0--Í£Ö¹×´Ì¬ 1-- ¹¤×÷×´Ì¬
uint8_t RobitRunningMode;           //É¨µØ»ú¹¤×÷Ä£Ê½  0--¿ª»·¹¤×÷·½Ê½ 1--±Õ»·¹¤×÷·½Ê½
uint8_t RobitRunningClosedLoopMode; //É¨µØ»ú±Õ»·¹¤×÷·½Ê½  0--ËÙ¶È±Õ»··½Ê½ 1--ÐÐ³Ì±Õ»·¹¤×÷·½Ê½
uint8_t RobitRunbyRouteMode;		//É¨µØ»úÐÐ³Ì±Õ»··½Ê½  0--Ö±ÏßÐÐÊ»Ä¿±êÀï³ÌÊý1--ÖáÐÄ×ª¶¯Ä¿±ê½Ç¶È  2--ÑØÇ½ÐÐ×ßÄ£Ê½

FlagStatus LeftMotorSwDirFlag,RightMotorSwDirFlag;//É¨µØ»úËÙ¶È±Õ»·Ä£Ê½ÖÐ£¬·½ÏòÇÐ»»±êÖ¾Î»
uint16_t   LeftMotorSpeedBak,RightMotorSpeedBak;//É¨µØ»úËÙ¶È±Õ»·Ä£Ê½ÖÐ£¬·½ÏòÇÐ»»Ê±µÄËÙ¶È±¸·Ý
MOTOR_RUNDIR_TypeDef    LeftMotorDirBak,RightMotorDirBak;//É¨µØ»úËÙ¶È±Õ»·Ä£Ê½ÖÐ£¬·½ÏòÇÐ»»Ê±µÄ·½Ïò±¸·Ý



uint8_t Robit_Dir;    //0---Ç°½ø  1---ºóÍË
int32_t RobotLineSpeed,RobotLineSpeedTarg, RobotAngleRate; //»úÆ÷ÈËÏßËÙ¶È(mm/s)ºÍ½ÇËÙ¶È(ºÁ»¡¶È/s)
int16_t AngularVelocityOdo;                                //Àï³Ì¼Æ¼ÆËãµÄ½ÇËÙ¶È

/*----------------------------------------------*
 * Ä£¿é¼¶±äÁ¿                                   *
 *----------------------------------------------*/
uint8_t LeftMotorSpeedCnt,RightMotorSpeedCnt;
uint32_t LeftMotorSpeedSum,RightMotorSpeedSum;
uint8_t MotorLinerModeCnt;

uint16_t LeftMotorInterCnt,RightMotorInterCnt;
uint16_t LeftMotorErrorCnt,RightMotorErrorCnt;

/*----------------------------------------------*
 * ³£Á¿¶¨Òå                                     *
 *----------------------------------------------*/

const uint16_t Speed_Mileage[9] = 
{
	4500,4242,3969,3674,3354,3000,2598,2121,1500
};

const uint8_t Speed_Angle[9] = 
{
	64, 60, 56, 52, 47, 42, 37, 30, 21
};

const uint8_t Angle_Num[41] = 
{
// 0 , 1 , 2 , 3 , 4 , 5 , 6 , 7 , 8 , 9 , 10 , 11 , 12 , 13 , 14 , 15 , 16 , 17 , 18 , 19 , 20 ,
   19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19 , 19 , 19 , 19 , 19 , 19 , 19 , 19 , 19 , 18 , 18 ,	
// 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31 , 32 , 33 , 34 , 35 , 36 , 37 , 38 , 39 , 40,
   18, 18, 18, 18, 18, 17, 17, 17, 17, 17, 17 , 16 , 16 , 16 , 16 , 16 , 15 , 15 , 15 , 15	
};


/*****************************************************************************
 º¯ Êý Ãû  : RobitMode_Conf
 ¹¦ÄÜÃèÊö  : É¨µØ»ú¹¤×÷Ä£Ê½³õÊ¼»¯
 ÊäÈë²ÎÊý  : void  
 Êä³ö²ÎÊý  : ÎÞ
 ·µ »Ø Öµ  : 
 µ÷ÓÃº¯Êý  : 
 ±»µ÷º¯Êý  : 
 
 ÐÞ¸ÄÀúÊ·      :
  1.ÈÕ    ÆÚ   : 2016Äê4ÔÂ8ÈÕ
    ×÷    Õß   : ÌïºêÍ¼
    ÐÞ¸ÄÄÚÈÝ   : ÐÂÉú³Éº¯Êý

*****************************************************************************/
void RobitMode_Conf(void)
{
	RobitStatus = 0;  				//Ä¬ÈÏÉ¨µØ»úÍ£Ö¹×´Ì¬
	RobitRunningMode = 0;			//Ä¬ÈÏ¹¤×÷ÔÚ¿ª»·×´Ì¬
	RobitRunningClosedLoopMode = 0;	//±Õ»·Ä£Ê½Ä¬ÈÏËÙ¶È±Õ»·
	RobitRunbyRouteMode = 0;		//ÐÐ³Ì±Õ»·Ä£Ê½Ä¬ÈÏÖ±ÏßÐÐÊ»·½Ê½
    MotorLinerModeEn = DISABLE;
    MotorLinerModeCnt = 0;
    SpeedAdjTime = 5;
    
    LeftMotorErrorCnt = 0;
    RightMotorErrorCnt = 0;
    LeftMotorInterCnt = 0;
    RightMotorInterCnt = 0;
    
    LeftMileageAllPre = 0;
	RightMileageAllPre = 0;
    
    RightMotor.SpeedAdjAll = 0;
    LeftMotor.SpeedAdjAll = 0;
    Robit_Dir = 0;
    RobotLineSpeedTarg = 0;
    AnglerateTarg = 0;
}

/*****************************************************************************
 º¯ Êý Ãû  : RobitMode_Ctrl
 ¹¦ÄÜÃèÊö  : É¨µØ»ú¹¤×÷Ä£Ê½´¦Àí,ÅÐ¶ÏÊÇ·ñÓÉ¹¤×÷×´Ì¬½øÈëÍ£Ö¹×´Ì¬
 ÊäÈë²ÎÊý  : void  
 Êä³ö²ÎÊý  : ÎÞ
 ·µ »Ø Öµ  : 
 µ÷ÓÃº¯Êý  : 
 ±»µ÷º¯Êý  : 

1.ÅÐ¶Ï·½Ê½ ×óÓÒÂÖµÄ×´Ì¬¶¼ÊÇÍ£Ö¹×´Ì¬ÈÏÎªÉ¨µØ»úÒÑ¾­½øÈëÍ£Ö¹×ªÌ¬
2.ÓÉÍ£Ö¹->¹¤×÷×´Ì¬µÄÅÐ¶¨ÔÚ¸÷¸ö½øÈë¹¤×÷×´Ì¬Ê±µÄÎ»ÖÃÊ±¸ø¶¨
3.±¾º¯Êý1msµ÷ÓÃÒ»´Î

4.ËÙ¶È±Õ»·Ä£Ê½Ê±£¬·½ÏòÇÐ»»´¦Àí
 
 ÐÞ¸ÄÀúÊ·      :
  1.ÈÕ    ÆÚ   : 2016Äê4ÔÂ8ÈÕ
    ×÷    Õß   : ÌïºêÍ¼
    ÐÞ¸ÄÄÚÈÝ   : ÐÂÉú³Éº¯Êý

*****************************************************************************/
void RobitMode_Ctrl(void)
{
	if(!RobitModeCtrlEn)
		return;
	RobitModeCtrlEn = DISABLE;

	if(LeftMotor.Status == SYS_STOP && RightMotor.Status == SYS_STOP)
	{
		if(RobitStatus != 0)
		{
			RobitStatus = 0;
						
            RobotLineSpeed = 0;
            
            RobotLineSpeedTargPre = 0;
            RobotAngleRatePre = 0;
            
            
            //RightMotorSwDirFlag = 1 »òÕßLeftMotorSwDirFlag = 1Ê±£¬ËµÃ÷¶¨Á¿Ä£¿é»¹Ã»ÓÐÖ´ÐÐÍê³É£¬²»ÄÜÇåÁã¸Ã±êÖ¾Î»
            // 2017 07 27 ÇåÁã±êÖ¾Î»£¬»áµ¼ÖÂÄ³Ð©Çé¿öÏÂ£¬»úÆ÷Ô­µØÐý×ª
            // ²âÊÔ
            if(!(RightMotorSwDirFlag || LeftMotorSwDirFlag))
            {
                //´Ë±êÖ¾Î»ÓÃÓÚÖ¸Ê¾ ¶¨Á¿Ö±ÏßÐÐ×ß/¶¨Á¿½Ç¶ÈÐý×ª/¶¨Á¿»­»¡Ïß/½ÇËÙ¶ÈÏßËÙ¶Èµ÷Õû 
                RobitRouteModeEn = DISABLE;
            }
            
		}
	}
	
    //ÔÚPathPlanMode = 0Ê±£¬Åö×²ºóÍË ·ñÔòÅö×²ºóÖ»Í£Ö¹
	if(RobitStatus == 0 && BumpFlag == ENABLE)
	{
        if(0 == PathPlanMode)
        {
            BumpFlag = DISABLE;
            SetRobitMileageRun(70, 2, 110);
        }
        else
        {
            // ÑØÇ½Ä£Ê½Ê±£¬ÔÚÖ´ÐÐÑØÇ½Ê±»áÖ´ÐÐÌØ¶¨µÄºóÍË³ÌÐò£¬²¢ÇÒ´¦Àí±êÖ¾Î»£¬´Ë´¦²»ÔÙ´¦Àí
//            BumpFlag = DISABLE;
//            BumpOverFlag = ENABLE;
            BumpOverForCloseWallFlag = DISABLE;
        }

	}

	if(RobitStatus == 0 && BumpFlag == DISABLE)
	{
		if(DISABLE == BumpOverFlag)
		{
			BumpOverFlag = ENABLE;
            BumpOverForCloseWallFlag = ENABLE;
//            BumpBak = 0;
		}
	}
	
	if(RobitRunningMode == 1 /*&& RobitRunningClosedLoopMode == 0 */&& LeftMotorSwDirFlag == SET)
	{
		if(LeftMotor.Status == SYS_STOP)
	  	{
			LeftMotorEnterRun(LeftMotorDirBak);		
			
			LeftMotor.TargSpeed = LeftMotorSpeedBak;
			
			LeftMotorSwDirFlag = RESET;
            RobitStatus = 1;
            RobitRunningMode = 1;
 //           RobitRunningClosedLoopMode = 0;
	  	}

		
	}

	if(RobitRunningMode == 1 /*&& RobitRunningClosedLoopMode == 0*/ && RightMotorSwDirFlag == SET)
	{
		
		//Èç¹û´¦ÓÚÍ£Ö¹ÖÐ£¬ÔòÆô¶¯µç»ú
      	if(RightMotor.Status == SYS_STOP)
      	{
			RightMotorEnterRun(RightMotorDirBak);
			
			RightMotor.TargSpeed = RightMotorSpeedBak;

			RightMotorSwDirFlag = RESET;
            RobitStatus = 1;
            RobitRunningMode = 1;
 //           RobitRunningClosedLoopMode = 0;
      	}
	}



//	if(WallSignal[0] == 1 || WallSignal[1] == 1 ||WallSignal[2] == 1 ||WallSignal[3] == 1 ||WallSignal[4] == 1 )
//	{
//		RightMotorStop();
//		LeftMotorStop();	

//		LeftMotorSwDirFlag = RESET;            
//		RightMotorSwDirFlag = RESET;
//	}
}

/*****************************************************************************
 º¯ Êý Ãû  : Motor_Conf
 ¹¦ÄÜÃèÊö  : µç»úÊý¾Ý³õÊ¼»¯
 ÊäÈë²ÎÊý  : void  
 Êä³ö²ÎÊý  : ÎÞ
 ·µ »Ø Öµ  : 
 µ÷ÓÃº¯Êý  : 
 ±»µ÷º¯Êý  : 
 
 ÐÞ¸ÄÀúÊ·      :
  1.ÈÕ    ÆÚ   : 2016Äê4ÔÂ1ÈÕ
    ×÷    Õß   : ÌïºêÍ¼
    ÐÞ¸ÄÄÚÈÝ   : ÐÂÉú³Éº¯Êý

*****************************************************************************/
void Motor_Conf(void)
{
	RightMotor.Status = SYS_STOP;
	LeftMotor.Status = SYS_STOP;

	RightMotor.PWMVal = 0;
	LeftMotor.PWMVal = 0;

	RightMotor.CurSpeed = 0;
	LeftMotor.CurSpeed = 0;

	RightCurMileage = 0;
	LeftCurMileage = 0;

	LeftMileageAll = 0;
	RightMileageAll = 0;
	LeftMileageAllPre = 0;
	RightMileageAllPre = 0;

	RobitRouteModeEn = DISABLE;
}

/*****************************************************************************
 º¯ Êý Ãû  : RightMotorRunInit
 ¹¦ÄÜÃèÊö  : ÓÒÂÖµç»úÇ°³õÊ¼»¯
 ÊäÈë²ÎÊý  : void  
 Êä³ö²ÎÊý  : ÎÞ
 ·µ »Ø Öµ  : 
 µ÷ÓÃº¯Êý  : 
 ±»µ÷º¯Êý  : 
 
 ÐÞ¸ÄÀúÊ·      :
  1.ÈÕ    ÆÚ   : 2016Äê3ÔÂ18ÈÕ
    ×÷    Õß   : ÌïºêÍ¼
    ÐÞ¸ÄÄÚÈÝ   : ÐÂÉú³Éº¯Êý
  2.ÈÕ    ÆÚ   : 2017Äê7ÔÂ25ÈÕ
    ×÷    Õß   : ÌïºêÍ¼
    ÐÞ¸ÄÄÚÈÝ   : ÐÞ¸´¶Â×ª×´Ì¬²»Õý³£µÄBUG£¬¶Â×ª×´Ì¬Ò²¿ÉÒÔ×ª±ä³ÉÆô¶¯×´Ì¬

*****************************************************************************/
void RightMotorRunInit(void)
{
	if(RightMotor.Status == SYS_STOP || RightMotor.Status == SYS_BLOCKING)
	{
		RightMotor.RunEnable = ENABLE;
		RightMotor.RunFlag = RESET;
		RightMotor.SpeedRisedFlag = RESET;
		RightMotor.RunCycle = 0;

		RightMotorSpeedSum = 0;
		RightMotorSpeedCnt = 0;

		RightMotorSpeed.OverflowCnt = 0;
		RightMotorSpeed.PreCountValue = TIM_GetCounter(TIM2);
		RightMotorSpeed.PulseCnt = 0;
		RightMotor.Status = SYS_STARTUP;
	}
}


//ÐÞ¸´¶Â×ª×´Ì¬²»Õý³£µÄBUG£¬¶Â×ª×´Ì¬Ò²¿ÉÒÔ×ª±ä³ÉÆô¶¯×´Ì¬ 2016.07.25
void LeftMotorRunInit(void)
{
	if(LeftMotor.Status == SYS_STOP || LeftMotor.Status == SYS_BLOCKING)
	{
		LeftMotor.RunEnable = ENABLE;
		LeftMotor.RunFlag = RESET;
		LeftMotor.SpeedRisedFlag = RESET;
		LeftMotor.RunCycle = 0;

		LeftMotorSpeedSum = 0;
		LeftMotorSpeedCnt = 0;

		LeftMotorSpeed.OverflowCnt = 0;
		LeftMotorSpeed.PreCountValue = TIM_GetCounter(TIM2);
		LeftMotor.Status = SYS_STARTUP;	
		LeftMotorSpeed.PulseCnt = 0;
	}
}

void CalSpeed(void)
{
    uint16_t MotorSpeedTmp;
	if(RightMotorSpeed.CalEn)
	{
		RightMotorSpeed.CalEn = DISABLE;

		MotorSpeedTmp = (uint16_t)(Motor_K / RightMotorSpeed.CircleTime);

		if(RightMotorSpeedCnt < 16)
		{
			RightMotorSpeedSum += MotorSpeedTmp;
			RightMotor.CurSpeed = MotorSpeedTmp;
			RightMotorSpeedCnt ++;
		}
		else
		{
			RightMotor.CurSpeed = RightMotorSpeedSum >> 4;
			RightMotorSpeedSum += MotorSpeedTmp;
			RightMotorSpeedSum -= RightMotor.CurSpeed;
		}

		RightMotorSpeed.Valid = ENABLE;
        RightMotorSpeed.ValidForAnguar = ENABLE;
	}

	if(LeftMotorSpeed.CalEn)
	{
		LeftMotorSpeed.CalEn = DISABLE;		

		MotorSpeedTmp = (uint16_t)(Motor_K / LeftMotorSpeed.CircleTime);

		if(LeftMotorSpeedCnt < 16)
		{
			LeftMotorSpeedSum += MotorSpeedTmp;
			LeftMotor.CurSpeed = MotorSpeedTmp;
			LeftMotorSpeedCnt ++;
		}
		else
		{
			LeftMotor.CurSpeed = LeftMotorSpeedSum >> 4;
			LeftMotorSpeedSum += MotorSpeedTmp;
			LeftMotorSpeedSum -= LeftMotor.CurSpeed;
		}

		LeftMotorSpeed.Valid = ENABLE;
        LeftMotorSpeed.ValidForAnguar = ENABLE;
	}
    
    
}

//input: ±äÁ¿:×óÂÖËÙ¶È¡¢ÓÒÂÖËÙ¶È  
//       ·½Ïò:Ç°½øÎªÕý£¬ºóÍËÎª¸º 
//       µ¥Î»:RPM
//return:±äÁ¿:½ÇËÙ¶È 
//       ·½Ïò:Ë³Ê±ÕëÎªÕý£¬ÄæÊ±ÕëÎª¸º        
//       µ¥Î»:ºÁ»¡¶È/s
int16_t CalOdoAngularVelocity(int16_t leftSpeed, int16_t rightSpeed)
{
    int16_t Wheelbase = ROBOT_WHEEL_AXIS;
    
    return ((rightSpeed - leftSpeed ) * RPM2MMPERSEC * 1000 / Wheelbase);
}


/*****************************************************************************
 º¯ Êý Ãû  : LeftMotorEnterRun
 ¹¦ÄÜÃèÊö  : ×óÂÖ½øÈëÔË¶¯
 ÊäÈë²ÎÊý  : uint16_t LeftMotor_Speed  
             uint8_t LeftMotor_Dir     
 Êä³ö²ÎÊý  : ÎÞ
 ·µ »Ø Öµ  : 
 µ÷ÓÃº¯Êý  : 
 ±»µ÷º¯Êý  : 
 
 ÐÞ¸ÄÀúÊ·      :
  1.ÈÕ    ÆÚ   : 2016Äê3ÔÂ31ÈÕ
    ×÷    Õß   : ÌïºêÍ¼
    ÐÞ¸ÄÄÚÈÝ   : ÐÂÉú³Éº¯Êý

*****************************************************************************/
void LeftMotorEnterRun(uint8_t LeftMotor_Dir)
{
	if(LeftMotor_Dir > 1)
		LeftMotor_Dir = 1;

	
    LeftMotorRunInit();
	InitVar_PID(1);

	LeftMotor.PWMVal = PWM_STATRUP_VAL;
	LeftMotor.Dir = (MOTOR_RUNDIR_TypeDef)LeftMotor_Dir;
	
	if(LeftMotor_Dir == 0)
	{
//		Motor_L_F_PWM(LeftMotor.PWMVal);
//		Motor_L_B_PWM(0);
		Motor_L_F_Run(LeftMotor.PWMVal);
	}
	else
	{
//		Motor_L_F_PWM(0);
//		Motor_L_B_PWM(LeftMotor.PWMVal);
		Motor_L_B_Run(LeftMotor.PWMVal);
	}
	
}

/*****************************************************************************
 º¯ Êý Ãû  : RightMotorEnterRun
 ¹¦ÄÜÃèÊö  : ÓÒÂÖ½øÈëÔËÐÐ
 ÊäÈë²ÎÊý  : uint8_t RightMotor_Dir  
 Êä³ö²ÎÊý  : ÎÞ
 ·µ »Ø Öµ  : 
 µ÷ÓÃº¯Êý  : 
 ±»µ÷º¯Êý  : 
 
 ÐÞ¸ÄÀúÊ·      :
  1.ÈÕ    ÆÚ   : 2016Äê4ÔÂ1ÈÕ
    ×÷    Õß   : ÌïºêÍ¼
    ÐÞ¸ÄÄÚÈÝ   : ÐÂÉú³Éº¯Êý

*****************************************************************************/
void RightMotorEnterRun(uint8_t RightMotor_Dir)
{
	if(RightMotor_Dir > 1)
		RightMotor_Dir = 1;	
	

    RightMotorRunInit();
	InitVar_PID(0);

	RightMotor.PWMVal = PWM_STATRUP_VAL;
	RightMotor.Dir = (MOTOR_RUNDIR_TypeDef)RightMotor_Dir;
	if(RightMotor_Dir == 0)
	{		
//		Motor_R_F_PWM(RightMotor.PWMVal);
//		Motor_R_B_PWM(0);
		Motor_R_F_Run(RightMotor.PWMVal);
	}
	else
	{
//		Motor_R_F_PWM(0);
//		Motor_R_B_PWM(RightMotor.PWMVal);
		Motor_R_B_Run(RightMotor.PWMVal);
	}
	
}

/*****************************************************************************
 º¯ Êý Ãû  : MotorStartupCtrl
 ¹¦ÄÜÃèÊö  : µç»úÆô¶¯¿ØÖÆ
 ÊäÈë²ÎÊý  : void    
 Êä³ö²ÎÊý  : ÎÞ
 ·µ »Ø Öµ  : 
 µ÷ÓÃº¯Êý  : 
 ±»µ÷º¯Êý  : 
 	1.50msµ÷ÓÃÒ»´Î
 	2.Ã¿´Îµ÷ÓÃÊ±,PWM Õ¼¿Õ±ÈÔö¼ÓÒ»¸öÆô¶¯²½³¤(50)
 ÐÞ¸ÄÀúÊ·      :
  1.ÈÕ    ÆÚ   : 2016Äê4ÔÂ7ÈÕ
    ×÷    Õß   : ÌïºêÍ¼
    ÐÞ¸ÄÄÚÈÝ   : ÐÂÉú³Éº¯Êý

*****************************************************************************/
void MotorStartupCtrl(void)
{
	if(!MotorStartupEn)
		return;

	if(RobitStatus == 0 || (RobitStatus == 1 && RobitRunningMode != 1))
		return;

	MotorStartupEn = DISABLE;
	
	if(LeftMotor.Status == SYS_STARTUP)
	{
		if(LeftMotor.PWMVal <= PWM_STATRUP_VAL_MAX - PWM_STATRUP_VAL_STEP)
			LeftMotor.PWMVal += PWM_STATRUP_VAL_STEP;
		
		if(LeftMotor.Dir == MOTOR_RUN_DIR_ZW)
		{
//			Motor_L_F_PWM(LeftMotor.PWMVal);
//			Motor_L_B_PWM(0);
			Motor_L_F_Run(LeftMotor.PWMVal);
		}
		else
		{
//			Motor_L_F_PWM(0);
//			Motor_L_B_PWM(LeftMotor.PWMVal);
			Motor_L_B_Run(LeftMotor.PWMVal);
		}
		
	}

	if(RightMotor.Status == SYS_STARTUP)
	{
		if(RightMotor.PWMVal <= PWM_STATRUP_VAL_MAX - PWM_STATRUP_VAL_STEP)
			RightMotor.PWMVal += PWM_STATRUP_VAL_STEP;
		
		if(RightMotor.Dir == MOTOR_RUN_DIR_ZW)
		{		
//			Motor_R_F_PWM(RightMotor.PWMVal);
//			Motor_R_B_PWM(0);
			Motor_R_F_Run(RightMotor.PWMVal);
		}
		else
		{
//			Motor_R_F_PWM(0);
//			Motor_R_B_PWM(RightMotor.PWMVal);
			Motor_R_B_Run(RightMotor.PWMVal);
		}
		
	}
}

/*****************************************************************************
 º¯ Êý Ãû  : AccCtrl
 ¹¦ÄÜÃèÊö  : ¼ÓËÙ¶È¿ØÖÆ
 ÊäÈë²ÎÊý  : void  
 Êä³ö²ÎÊý  : ÎÞ
 ·µ »Ø Öµ  : 
 µ÷ÓÃº¯Êý  : 
 ±»µ÷º¯Êý  : 
 				1.5msµ÷ÓÃÒ»´Î
 				2.¼ÓËÙ¶È¿ØÖÆ
 ÐÞ¸ÄÀúÊ·      :
  1.ÈÕ    ÆÚ   : 2017Äê7ÔÂ17ÈÕ
    ×÷    Õß   : ÌïºêÍ¼
    ÐÞ¸ÄÄÚÈÝ   : ÐÂÉú³Éº¯Êý

*****************************************************************************/
void AccCtrl(void)
{
    static uint16_t BaseSpeedBak = 0;
    static int16_t AngleRateTargBak = 0;
    //µç»ú´¦ÓÚÔËÐÐ×´Ì¬
    if(RightMotor.Status != SYS_RUNNING && LeftMotor.Status != SYS_RUNNING)
        return;
    
    //¶¨Ê±Ê±¼äÊÇ·ñµ½()
    if(!AccCtrlTimeEn)
        return;
    AccCtrlTimeEn = DISABLE;
    
    //Ïß¼ÓËÙ¶ÈÊ¹ÄÜ±êÖ¾
    if(VaccEn)
    {
        //Ïß¼ÓËÙ¶È²ÎÊý³õÊ¼»¯
        if(!VaccInit)
        {
            VaccInit = 1;
            BaseSpeedBak = RightMotor.BaseSpeed;
            
            if(RightMotor.CurSpeed <= MOT_RPM_SPEED_MIN && LeftMotor.CurSpeed <= MOT_RPM_SPEED_MIN)
            {
                RightMotor.BaseSpeed = MOT_RPM_SPEED_MIN;
                LeftMotor.BaseSpeed = MOT_RPM_SPEED_MIN;
            }
            else if(RightMotor.CurSpeed + LeftMotor.CurSpeed <= RightMotor.BaseSpeed * 2)
            {
                RightMotor.BaseSpeed = (RightMotor.CurSpeed + LeftMotor.CurSpeed) / 2;
                LeftMotor.BaseSpeed = RightMotor.BaseSpeed;
            }
            else
            {
                //basespeed²»±ä
            }
            
            MotorLinerModeCnt = 10;
        }
        
        //¼ÓËÙ¹ý³Ì
        if(RightMotor.BaseSpeed + RightMotor.Vacc < BaseSpeedBak)
        {
            RightMotor.BaseSpeed += RightMotor.Vacc;
            LeftMotor.BaseSpeed += LeftMotor.Vacc;
        }
        //¼ÓËÙÍê³É
        else
        {
            RightMotor.BaseSpeed = BaseSpeedBak;
            LeftMotor.BaseSpeed = BaseSpeedBak;
            VaccEn = DISABLE;
            VaccInit = 0;
        }
    }
    
    //½Ç¼ÓËÙ¶ÈÊ¹ÄÜ±êÖ¾
    if(WaccEn)
    {
        //½Ç¼ÓËÙ¶È²ÎÊý³õÊ¼»¯
        if(!WaccInit)
        {
            WaccInit = 1;
            AngleRateTargBak = AnglerateTarg;
            MotorLinerModeCnt = 10;
            
            if(AngleRateTargBak > 0)
            {
                if(Sys_Anglerate < ANGLERATE_MIN)
                {
                    AnglerateTarg = ANGLERATE_MIN;                   
                }
                else if(Sys_Anglerate <= AnglerateTarg)
                {
                    AnglerateTarg = Sys_Anglerate;
                }
                else
                {
                    //²»±ä
                }
                
                if(AnglerateTarg < AngleRateTargBak)
                {
                    RightMotor.SpeedAdjAll = AnglerateTarg * RightMotor.SpeedAdjAll / AngleRateTargBak;
                    LeftMotor.SpeedAdjAll = AnglerateTarg * LeftMotor.SpeedAdjAll / AngleRateTargBak;
                }
            }
            else
            {
                if(Sys_Anglerate > -ANGLERATE_MIN)
                {
                    AnglerateTarg = -ANGLERATE_MIN;       
                }
                else if(Sys_Anglerate > AnglerateTarg)
                {
                    AnglerateTarg = Sys_Anglerate;
                }
                else
                {
                    //²»±ä
                }
                
                if(AnglerateTarg > AngleRateTargBak)
                {
                    RightMotor.SpeedAdjAll = AnglerateTarg * RightMotor.SpeedAdjAll / AngleRateTargBak;
                    LeftMotor.SpeedAdjAll = AnglerateTarg * LeftMotor.SpeedAdjAll / AngleRateTargBak;
                }
            }
        }
        
        if(fabs(AnglerateTarg + RightMotor.Wacc) < fabs(AngleRateTargBak))
        {
            AnglerateTarg += RightMotor.Wacc;
        }
        else
        {
            AnglerateTarg = AngleRateTargBak;
            WaccEn = DISABLE;
            WaccInit = 0;
        }
    }
}

/*****************************************************************************
 º¯ Êý Ãû  : MotorRunCtrl
 ¹¦ÄÜÃèÊö  : ×óÓÒÂÖÔËÐÐ¿ØÖÆ
 ÊäÈë²ÎÊý  : void  
 Êä³ö²ÎÊý  : ÎÞ
 ·µ »Ø Öµ  : 
 µ÷ÓÃº¯Êý  : 
 ±»µ÷º¯Êý  : 
 				1.5msµ÷ÓÃÒ»´Î
 				2.×óÓÒÂÖµ÷ËÙ¿ØÖÆ
 ÐÞ¸ÄÀúÊ·      :
  1.ÈÕ    ÆÚ   : 2016Äê4ÔÂ1ÈÕ
    ×÷    Õß   : ÌïºêÍ¼
    ÐÞ¸ÄÄÚÈÝ   : ÐÂÉú³Éº¯Êý

*****************************************************************************/
void MotorRunCtrl(void)
{
	int16_t PWM_Adj;
	uint16_t speed_tmp;
//	uint16_t PWM_min;

	//É¨µØ»úÄ£Ê½ÅÐ¶Ï
	if(RobitStatus == 0 || (RobitStatus == 1 && RobitRunningMode != 1))
		return;    
    
	if(!AdjustSpeedEn)
		return;

	AdjustSpeedEn = DISABLE;
    
    //PID µ÷½ÚÊ±¼äµ÷Õû
    //Éè¶¨ËÙ¶È´óÓÚ1500RPM ÉèÎª5ms 
    //Éè¶¨ËÙ¶ÈÐ¡ÓÚ1300RPM ÉèÎª10ms
    //Éè¶¨ËÙ¶ÈÔÚ1300~1500RPM Ö®¼ä ±£³ÖÔ­À´µÄÊ±¼ä²»±ä 
    //µ÷½ÚÊ±¼ä³õÊ¼ÖµÎª5ms
    if(LeftMotor.TargSpeed >= 1500 && RightMotor.TargSpeed >= 1500)
    {
        SpeedAdjTime = 5;
    }
    else if(!(LeftMotor.TargSpeed >= 1300 && RightMotor.TargSpeed >= 1300))
    {
        SpeedAdjTime = 10;
    }
	
	//×óÂÖ
	if(LeftMotor.Status == SYS_RUNNING)
	{
		if(LeftMotorSpeed.Valid == ENABLE)
		{
			LeftMotorSpeed.Valid = DISABLE;

			//PIDµ÷½Ú
			PWM_Adj = CalculPID(LeftMotor.TargSpeed, LeftMotor.CurSpeed, 1);


			
			if(PWM_Adj > 0)
		  	{
		  		LeftMotor.PWMVal += PWM_Adj;
		  	}
		  	else
		  	{
		  		PWM_Adj = -PWM_Adj;
				
				//PWM ×îÐ¡ÖµÉèÖÃ
//				if(LeftMotor.TargSpeed >= 1500)
//				{
//					PWM_min = PWM_VAL_MIN;
//				}
//				else
//				{
//					PWM_min = PWM_VAL_MIN_Low;
//				}

				
		  		if(LeftMotor.PWMVal > PWM_Adj + PWM_VAL_MIN)
		  		{
		  			LeftMotor.PWMVal -= PWM_Adj;
		  		}
		  		else
		  		{
		  			LeftMotor.PWMVal = PWM_VAL_MIN;
		  		}
		  	}
		    
		  	if(LeftMotor.PWMVal > PWM_VAL_MAX)
		  	{
		  		LeftMotor.PWMVal = PWM_VAL_MAX;
		  	}

			//PWMÉè¶¨
			if(LeftMotor.Dir == MOTOR_RUN_DIR_ZW)
			{
//				Motor_L_F_PWM(LeftMotor.PWMVal);
//				Motor_L_B_PWM(0);
				Motor_L_F_Run(LeftMotor.PWMVal);
			}
			else
			{
//				Motor_L_B_PWM(LeftMotor.PWMVal);
//				Motor_L_F_PWM(0);
				Motor_L_B_Run(LeftMotor.PWMVal);
			}
			
			//µ±Ç°ËÙ¶È´óÓÚÉè¶¨ËÙ¶ÈµÄ7/8 ÈÏÎªËÙ¶ÈÉÏÉý¹ý³ÌÒÑÍê³É
			speed_tmp = LeftMotor.TargSpeed * 7 ;
			speed_tmp >>= 3; 
			if(LeftMotor.CurSpeed > speed_tmp)
				LeftMotor.SpeedRisedFlag = SET;
			else
				LeftMotor.SpeedRisedFlag = RESET;
		}
	}

	//ÓÒÂÖ
	if(RightMotor.Status == SYS_RUNNING)
	{
		if(RightMotorSpeed.Valid == ENABLE)
		{
			RightMotorSpeed.Valid = DISABLE;
			//PIDµ÷½Ú
			PWM_Adj = CalculPID(RightMotor.TargSpeed, RightMotor.CurSpeed, 0);

		
			if(PWM_Adj > 0)
		  	{
		  		RightMotor.PWMVal += PWM_Adj;
		  	}
		  	else
		  	{
		  		PWM_Adj = -PWM_Adj;

				//PWM ×îÐ¡ÖµÉèÖÃ
//				if(RightMotor.TargSpeed >= 1500)
//				{
//					PWM_min = PWM_VAL_MIN;
//				}
//				else
//				{
//					PWM_min = PWM_VAL_MIN_Low;
//				}
				
		  		if(RightMotor.PWMVal > PWM_Adj + PWM_VAL_MIN)
		  		{
		  			RightMotor.PWMVal -= PWM_Adj;
		  		}
		  		else
		  		{
		  			RightMotor.PWMVal = PWM_VAL_MIN;
		  		}
		  	}
		    
		  	if(RightMotor.PWMVal > PWM_VAL_MAX)
		  	{
		  		RightMotor.PWMVal = PWM_VAL_MAX;
		  	}

			//PWMÉè¶¨
			if(RightMotor.Dir == MOTOR_RUN_DIR_ZW)
			{
//				Motor_R_F_PWM(RightMotor.PWMVal);
//				Motor_R_B_PWM(0);
				Motor_R_F_Run(RightMotor.PWMVal);
			}
			else
			{
//				Motor_R_B_PWM(RightMotor.PWMVal);
//				Motor_R_F_PWM(0);
				Motor_R_B_Run(RightMotor.PWMVal);
			}
			
			//µ±Ç°ËÙ¶È´óÓÚÉè¶¨ËÙ¶ÈµÄ7/8 ÈÏÎªËÙ¶ÈÉÏÉý¹ý³ÌÒÑÍê³É
			speed_tmp = RightMotor.TargSpeed * 7 ;
			speed_tmp >>= 3; 
			if(RightMotor.CurSpeed > speed_tmp)
				RightMotor.SpeedRisedFlag = SET;
			else
				RightMotor.SpeedRisedFlag = RESET;
		}		
	}
}

/*****************************************************************************
 º¯ Êý Ãû  : Robit_LinerPathMoving
 ¹¦ÄÜÃèÊö  : É¨µØ»úÖ±ÏßÐÐ×ß¿ØÖÆ
 ÊäÈë²ÎÊý  : void  
 Êä³ö²ÎÊý  : ÎÞ
 ·µ »Ø Öµ  : 
 µ÷ÓÃº¯Êý  : 
 ±»µ÷º¯Êý  : 
 
 ÐÞ¸ÄÀúÊ·      :
  1.ÈÕ    ÆÚ   : 2016Äê4ÔÂ7ÈÕ
    ×÷    Õß   : ÌïºêÍ¼
    ÐÞ¸ÄÄÚÈÝ   : ÐÂÉú³Éº¯Êý

*****************************************************************************/
void Robit_RouteMoving(void)
{
//	uint32_t tmpmileage;
//	uint16_t TmpAngledif;
    int16_t TmpSpeed;
    int16_t SpeedAdjust;
//	int16_t tmpangle;
    int16_t tmpspeedl, tmpspeedr;
//	uint8_t tmpnum;
	static uint8_t AngleCnt = 0,CirleHalf = 0, AngleStopCnt = 0;
//	static uint16_t StartSpeed = 0;

    if(!MotorLinerModeEn)
        return;
    MotorLinerModeEn = DISABLE;
    
	if(!(RobitStatus == 1 && RobitRunningMode == 1 && RobitRunningClosedLoopMode == 1))
		return;
	
    if(!RobitRouteModeEn)	
    return;
	
    MotorLinerModeCnt ++;
	if(RightMotor.Status != SYS_STOP && RightMotor.Status != SYS_STOPING)
	{
			if(TargMileage > RightCurMileage + 2 || RobitRunbyRouteMode == 3 || RobitRunbyRouteMode == 1)
			{
				if(0 == RobitRunbyRouteMode || 2 == RobitRunbyRouteMode || 3 == RobitRunbyRouteMode || 1 == RobitRunbyRouteMode)  // Ö±ÏßÀï³Ì¿ØÖÆ·½Ê½
				{
//					tmpmileage = TargMileage - RightCurMileage;		

						//10msµ÷ÕûÒ»´Î ½ÇËÙ¶Èµ÷½Ú
                    if(MotorLinerModeCnt >= 10)
                    {
                        MotorLinerModeCnt = 0;
                        AngleCnt ++;
                        
                        if(0 == RobitRunbyRouteMode)
//						if(2 == RobitRunbyRouteMode)
                        {
                            //20msµ÷ÕûÒ»´Î ½Ç¶Èµ÷½Ú  Ä¿±ê½Ç¶È=³õÊ¼½Ç¶È
                            if(AngleCnt >= 2)
                            {
                                AngleCnt = 0;
        //						tmpangle = EdageRunAngle();
                                
                                AnglerateTarg = CalculPID(Sys_Startangle,Sys_Angle,3);
        //						AnglerateTarg = CalculPID(tmpangle,Sys_Angle,3);
        
                            }
                        }
                        else if(2 == RobitRunbyRouteMode) // ÑØÇ½ÐÐ×ßÄ£Ê½
//						else if(0 == RobitRunbyRouteMode)
                        {
                            //40msµ÷ÕûÒ»´Î ½Ç¶Èµ÷½Ú  Ä¿±ê½Ç¶È=¼ÆËãÊä³ö½Ç¶È
                            //VL53L0x Ò»´Î²â¾àÐèÒª33ms £¬¸ÄÎª40msµ÷ÕûÒ»´Î
                            if(AngleCnt >= 4)
                            {
                                AngleCnt = 0;
                                boundary(1);
//                                AnglerateTarg = CalculPID(Sys_Startangle,Sys_Angle,3);
        
                            }
                        }
                        
                        if(AnglerateTarg > 20000)
                        {
                            AnglerateTarg = 20000;
                        }
                        else if(AnglerateTarg < -20000)
                        {
                            AnglerateTarg = -20000;
                        }
                        
    //					Sys_Anglerate = ;
                        SpeedAdjust = CalculPID(AnglerateTarg, Sys_Anglerate, 2);//(LeftCurMileage - RightCurMileage);//
                        
                        if(!((RightMotor.SpeedAdjAll >= MOT_RPM_SPEED_MAX * 2 && SpeedAdjust > 0) || (RightMotor.SpeedAdjAll <= - MOT_RPM_SPEED_MAX * 2 && SpeedAdjust < 0)))
                        {
                        //()
                            if(Robit_Dir == 1)
                            {
                                RightMotor.SpeedAdjAll += SpeedAdjust;
                                LeftMotor.SpeedAdjAll -= SpeedAdjust;
                            }
                            else
                            {
                                RightMotor.SpeedAdjAll -= SpeedAdjust;
                                LeftMotor.SpeedAdjAll += SpeedAdjust;
                            }
                        }
                        
                        
                        TmpSpeed = (int16_t)RightMotor.BaseSpeed;
                        if(TmpSpeed + RightMotor.SpeedAdjAll > 0)
                        {
                            if(TmpSpeed + RightMotor.SpeedAdjAll <= MOT_RPM_SPEED_MAX_2)
                            {
                                RightMotor.TargSpeed = RightMotor.BaseSpeed + RightMotor.SpeedAdjAll;
                            }
                            else
                            {
                                RightMotor.TargSpeed = MOT_RPM_SPEED_MAX_2;
                                RightMotor.SpeedAdjAll = MOT_RPM_SPEED_MAX_2 - RightMotor.BaseSpeed;
                                
                                LeftMotor.SpeedAdjAll = -RightMotor.SpeedAdjAll;///1
                            }

                            if(Robit_Dir == 0)
                            {
                                RightMotor.TargDir = MOTOR_RUN_DIR_ZW;
                                tmpspeedr = RightMotor.CurSpeed;
                            }
                            else
                            {
                                RightMotor.TargDir = MOTOR_RUN_DIR_FW;
                                tmpspeedr = -RightMotor.CurSpeed;
                            }
                        }
                        else
                        {
                            
                            
                            if(-(RightMotor.BaseSpeed + RightMotor.SpeedAdjAll) <= MOT_RPM_SPEED_MAX_2)
                            {
                                RightMotor.TargSpeed = -(RightMotor.BaseSpeed + RightMotor.SpeedAdjAll);
                            }
                            else
                            {
                                RightMotor.TargSpeed = MOT_RPM_SPEED_MAX_2;
                                RightMotor.SpeedAdjAll = -(int16_t)(MOT_RPM_SPEED_MAX_2 + RightMotor.BaseSpeed);
                                
                                LeftMotor.SpeedAdjAll = -RightMotor.SpeedAdjAll;///1
                            }
                            
                            if(Robit_Dir == 0)
                            {
                                RightMotor.TargDir = MOTOR_RUN_DIR_FW;
                                tmpspeedr = -RightMotor.CurSpeed;
                            }
                            else
                            {
                                RightMotor.TargDir = MOTOR_RUN_DIR_ZW;
                                tmpspeedr = RightMotor.CurSpeed;
                            }
                        }
                        
                        //Ä¿±ê·½ÏòÓëµ±Ç°·½Ïò²»Ò»ÖÂ£¬ÐèÒªÇÐ»»·½Ïò
                        if(RightMotor.Dir != RightMotor.TargDir && RightMotor.Status == SYS_RUNNING && RightMotor.TargSpeed >= MOT_RPM_SPEED_MIN)
                        {
                            RightMotorStop();
                            RightMotorSwDirFlag = SET;
                            RightMotorSpeedBak = RightMotor.TargSpeed;
                            RightMotorDirBak = RightMotor.TargDir;
                        }
                        
                        TmpSpeed = (int16_t)LeftMotor.BaseSpeed;
                        if(TmpSpeed + LeftMotor.SpeedAdjAll > 0)
                        {
                            if(TmpSpeed + LeftMotor.SpeedAdjAll <= MOT_RPM_SPEED_MAX_2)
                            {
                                LeftMotor.TargSpeed = LeftMotor.BaseSpeed + LeftMotor.SpeedAdjAll;
                            }
                            else
                            {
                                LeftMotor.TargSpeed = MOT_RPM_SPEED_MAX_2;
                                LeftMotor.SpeedAdjAll = MOT_RPM_SPEED_MAX_2 - LeftMotor.BaseSpeed;
                                
                                RightMotor.SpeedAdjAll = -LeftMotor.SpeedAdjAll;///1
                            }
                            
                            if(Robit_Dir == 0)
                            {
                                LeftMotor.TargDir = MOTOR_RUN_DIR_ZW;
                                tmpspeedl = LeftMotor.CurSpeed;
                            }
                            else
                            {
                                LeftMotor.TargDir = MOTOR_RUN_DIR_FW;
                                tmpspeedl = -LeftMotor.CurSpeed;
                            }
                        }
                        else
                        {
                            if(-(LeftMotor.BaseSpeed + LeftMotor.SpeedAdjAll) <= MOT_RPM_SPEED_MAX_2)
                            {
                                LeftMotor.TargSpeed = -(LeftMotor.BaseSpeed + LeftMotor.SpeedAdjAll);
                            }
                            else
                            {
                                LeftMotor.TargSpeed = MOT_RPM_SPEED_MAX_2;
                                LeftMotor.SpeedAdjAll = -(int16_t)(MOT_RPM_SPEED_MAX_2 + LeftMotor.BaseSpeed);
                                
                                RightMotor.SpeedAdjAll = -LeftMotor.SpeedAdjAll;///1
                            }
                            
                            if(Robit_Dir == 0)
                            {
                                LeftMotor.TargDir = MOTOR_RUN_DIR_FW;
                                tmpspeedl = -LeftMotor.CurSpeed;
                            }
                            else
                            {
                                LeftMotor.TargDir = MOTOR_RUN_DIR_ZW;
                                tmpspeedl = LeftMotor.CurSpeed;
                            }
                        }
                        
                        if(LeftMotor.Dir  != LeftMotor.TargDir && LeftMotor.Status == SYS_RUNNING && LeftMotor.TargSpeed >= MOT_RPM_SPEED_MIN)
                        {
                            LeftMotorStop();	
                            LeftMotorSwDirFlag = SET;
                            LeftMotorSpeedBak = LeftMotor.TargSpeed;
                            LeftMotorDirBak = LeftMotor.TargDir;
                        }
                        
                        //¼ÆËãÏßËÙ¶È
                        RobotLineSpeed = (tmpspeedl + tmpspeedr) / 2;
                        
                        if(RightMotor.TargSpeed > MOT_RPM_SPEED_MAX_2)
                        {
                            RightMotor.TargSpeed = MOT_RPM_SPEED_MAX_2;
                        }
                        else if(RightMotor.TargSpeed < MOT_RPM_SPEED_MIN_2)
                        {
                            RightMotor.TargSpeed = MOT_RPM_SPEED_MIN_2;
                        }
                        
                        if(LeftMotor.TargSpeed > MOT_RPM_SPEED_MAX_2)
                        {
                            LeftMotor.TargSpeed = MOT_RPM_SPEED_MAX_2;
                        }
                        else if(LeftMotor.TargSpeed < MOT_RPM_SPEED_MIN_2)
                        {
                            LeftMotor.TargSpeed = MOT_RPM_SPEED_MIN_2;
                        }
                          
                    }

				}				
			}
			else
			{
				RightMotorStop();
				LeftMotorStop();
                //Çå³þ×óÓÒÂÖ»»ÏòÆô¶¯±êÖ¾ £¬·ñÔò»áµ¼ÖÂÎóÆô¶¯
                LeftMotorSwDirFlag = RESET;
                RightMotorSwDirFlag = RESET;
			}
		}
		
//			else //if(tmpmileage >= 100)
//			{
//				TmpSpeed = Speed_Mileage[tmpmileage / Motor_Stop_Step];
//				
//				if(RightMotor.TargSpeed > TmpSpeed)
//				{
//					RightMotor.TargSpeed = TmpSpeed;
//				}
//                LeftMotor.TargSpeed = RightMotor.TargSpeed;
//			}
			
		

	if(LeftMotor.Status != SYS_STOP && LeftMotor.Status != SYS_STOPING)
	{
        if(1 == RobitRunbyRouteMode)
        {

        }
        else if(TargMileage > LeftCurMileage + 2 || RobitRunbyRouteMode == 3)
		{
			
		}
		else
		{
			LeftMotorStop();
			RightMotorStop();
            
            //Çå³þ×óÓÒÂÖ»»ÏòÆô¶¯±êÖ¾ £¬·ñÔò»áµ¼ÖÂÎóÆô¶¯
            LeftMotorSwDirFlag = RESET;
            RightMotorSwDirFlag = RESET;
		}
	}
    
    //Ðý×ª½Ç¶È step Ä£Ê½ »­»¡Ïß½áÊøÌõ¼þÅÐ¶¨
    if(1 == RobitRunbyRouteMode)
    {
        uint16_t TmpAngledif;
        uint32_t TmpLineDis;
        
        //20170717 5S²»ÄÜÍê³Éstep¶¯×÷£¬Í£Ö¹
//        if(++StepModeCnt >= 5000)
//        {
//            RightMotorStop();
//            LeftMotorStop();
//            AngleStopCnt = 0;
//            CirleHalf = 0;
//            StepModeCnt = 0;
//            
//            //Çå³þ×óÓÒÂÖ»»ÏòÆô¶¯±êÖ¾ £¬·ñÔò»áµ¼ÖÂÎóÆô¶¯
//            LeftMotorSwDirFlag = RESET;
//            RightMotorSwDirFlag = RESET;
//        }
        
        // ½Ç¶ÈÅÐ¶¨
        if(Sys_Anglerate > 0)
        {
            TmpAngledif = CalAngleDif(Sys_Angle, Sys_Startangle);					
        }
        else if(Sys_Anglerate < 0)
        {
            TmpAngledif = CalAngleDif(Sys_Startangle, Sys_Angle);
        }
        
        
        if(TmpAngledif > 18000 && TmpAngledif < 35000)
        {
            CirleHalf = 1; 
        }

        
        
        if(TargCirle > 0)
        {
            if(1 == CirleHalf && TmpAngledif <= ANGLE_DELT)
            {
                CirleHalf = 0;
                TargCirle --;
                
            }
        }
        else
        {	
            if(TargAngle >= 35000)
            {
                //Ðý×ª½Ç¶È½Ï´óÊ±£¬ÐèÒª°ëÈ¦±êÖ¾ÖÃÎ»£¬
                if(!CirleHalf)
                {
                    return;
                }
            }
            
            //Ðý×ª½Ç¶ÈÓÃÓÚÐ¡ÓÚ350µÄ¸³Öµ£¬´óÓÚ350¶ÈµÄ¸³Öµ¿ÉÄÜÊÇ³õÊ¼µÄÐ¡·ù¶È·­×ªÆ«²îµ¼ÖÂ
            if(TargAngle < 35000 && TmpAngledif < 35000)
                RotationAngleCur = TmpAngledif;
            
            //ÑØ±ßÄ£Ê½Ê±£¬stepÄ£Ê½µÄ½áÊøÅÐ¶¨Ö´ÐÐ¾ÉµÄ°æ±¾
            if(3 == PathPlanMode || TargMileageForArc == 0 || 4 == PathPlanMode) 
            {
                if(TmpAngledif + (fabs(AnglerateTarg) / 2) >= TargAngle)
                {               
                    if(++AngleStopCnt >= 10)
                    {
                        AngleStopCnt = 0;
                        if(fabs(AnglerateTarg) >= ANGLERATE_MIN * 3)
                        {
                            AnglerateTarg /= 3;
                        }
                        else if(fabs(AnglerateTarg) > ANGLERATE_MIN)
                        {
                            if(AnglerateTarg > 0)
                            {
                                AnglerateTarg = ANGLERATE_MIN;
                            }
                            else if(AnglerateTarg < 0)
                            {
                                AnglerateTarg = -ANGLERATE_MIN;
                            }
                        }
                    }
                    if(TargAngle >= ANGLE_DELT * 2)
                    {
                        if(TmpAngledif >= TargAngle - ANGLE_DELT && TmpAngledif <= TargAngle + ANGLE_DELT)
                        {
                            RightMotorStop();
                            LeftMotorStop();
                            AngleStopCnt = 0;
                            CirleHalf = 0;
                            
                            //Çå³þ×óÓÒÂÖ»»ÏòÆô¶¯±êÖ¾ £¬·ñÔò»áµ¼ÖÂÎóÆô¶¯
                            LeftMotorSwDirFlag = RESET;
                            RightMotorSwDirFlag = RESET;
                            
    //                        AnglerateTarg = 0;
                        }
                    }
                    else
                    {
                        if(TmpAngledif >= TargAngle && TmpAngledif <= TargAngle + ANGLE_DELT)
                        {
                            RightMotorStop();
                            LeftMotorStop();
                            AngleStopCnt = 0;
                            CirleHalf = 0;
                            
                            //Çå³þ×óÓÒÂÖ»»ÏòÆô¶¯±êÖ¾ £¬·ñÔò»áµ¼ÖÂÎóÆô¶¯
                            LeftMotorSwDirFlag = RESET;
                            RightMotorSwDirFlag = RESET;
    //                        AnglerateTarg = 0;
                        }
                    }
                }
                else if(TmpAngledif + (fabs(AnglerateTargInit) / 2) < TargAngle)
                {
                    AnglerateTarg = AnglerateTargInit;
                }
            }
            //ÆäËûÄ£Ê½£¬stepÄ£Ê½½áÊøÖ»ÊÇÉè¶¨½ÇËÙ¶ÈÎª0
            else 
            {
                if(TargAngle >= ANGLE_DELT * 2)
                {
                    if(TmpAngledif >= TargAngle - ANGLE_DELT && TmpAngledif <= TargAngle + ANGLE_DELT)
                    {                           
                        if(AnglerateTarg > 1000)
                            AnglerateTarg = 1000;
 
                        if(StepStatus)
                        {
                            RightMotor.SpeedAdjAll = RightMotor.SpeedAdjAll >> 3;
                            LeftMotor.SpeedAdjAll = LeftMotor.SpeedAdjAll >> 3; 
//                            if(LeftMotor.SpeedAdjAll > 1800 && LeftMotor.SpeedAdjAll > 0)
//                            {
//                                RightMotor.SpeedAdjAll = -1800;
//                                LeftMotor.SpeedAdjAll = 1800;
//                            }
//                            else if(LeftMotor.SpeedAdjAll < -1800 && LeftMotor.SpeedAdjAll < 0)
//                            {
//                                RightMotor.SpeedAdjAll = 1800;
//                                LeftMotor.SpeedAdjAll = -1800;
//                            }
                            StepStatus = 0;
                        }
                    }
                }
                else
                {
                    if(TmpAngledif >= TargAngle && TmpAngledif <= TargAngle + ANGLE_DELT)
                    {
                        if(AnglerateTarg > 1000)
                            AnglerateTarg = 1000;
                        
                        if(StepStatus)
                        {
                            RightMotor.SpeedAdjAll = RightMotor.SpeedAdjAll >> 3;
                            LeftMotor.SpeedAdjAll = LeftMotor.SpeedAdjAll >> 3;
//                            if(LeftMotor.SpeedAdjAll > 1800 && LeftMotor.SpeedAdjAll > 0)
//                            {
//                                RightMotor.SpeedAdjAll = 1800;
//                                LeftMotor.SpeedAdjAll = 1800;
//                            }
//                            else if(LeftMotor.SpeedAdjAll < -1800 && LeftMotor.SpeedAdjAll < 0)
//                            {
//                                RightMotor.SpeedAdjAll = -1800;
//                                LeftMotor.SpeedAdjAll = -1800;
//                            }
                            
                            StepStatus = 0;
                        }
                    }
                }
            }                
        }
        
        //¾àÀëÅÐ¶¨
        //TargMileageForArc = 0Ê±²»×ö¾àÀëÅÐ¶¨
        if(TargMileageForArc != 0)
        {
            if(RightMotor.Dir == LeftMotor.Dir)
            {
                TmpLineDis = (LeftCurMileage + RightCurMileage) >> 1;
            }
            else
            {
                TmpLineDis = (uint32_t)fabs((int32_t)LeftCurMileage - (int32_t)RightCurMileage);
                TmpLineDis >>= 1;
            }
            if(TmpLineDis >= TargMileageForArc + 2)
            {
                LeftMotorStop();
                RightMotorStop();
                
                //Çå³þ×óÓÒÂÖ»»ÏòÆô¶¯±êÖ¾ £¬·ñÔò»áµ¼ÖÂÎóÆô¶¯
                LeftMotorSwDirFlag = RESET;
                RightMotorSwDirFlag = RESET;
            }
        }
    }
}

//¼ÆËã½Ç¶È²îAngle1 - Angle2£¬·µ»ØÖµÔÚ0~36000Ö®¼ä
uint16_t CalAngleDif(int16_t Angle1, int16_t Angle2)
{
	int32_t calresult;
	
	calresult = (int32_t)(Angle1 - Angle2);
	
	if(calresult < 0)
	{
		calresult += 36000;
	}
	else if(calresult > 36000)
	{
		calresult -= 36000;
	}
	return (uint16_t)calresult;
}

/*****************************************************************************
 º¯ Êý Ãû  : Robit_RunLR
 ¹¦ÄÜÃèÊö  : É¨µØ»ú×óÓÒÂÖËÙ¶È¿ØÖÆ
 ÊäÈë²ÎÊý  : uint8_t LeftMotor_Speed           // 0~10
             uint8_t LeftMotor_Dir			   //0/1
             uint8_t RightMotor_Speed  		   //0~10
             uint8_t RightMotor_Dir			   //0/1
 Êä³ö²ÎÊý  : ÎÞ
 ·µ »Ø Öµ  : 
 µ÷ÓÃº¯Êý  : 
 ±»µ÷º¯Êý  : 
 
 ÐÞ¸ÄÀúÊ·      :
  1.ÈÕ    ÆÚ   : 2016Äê3ÔÂ16ÈÕ
    ×÷    Õß   : ÌïºêÍ¼
    ÐÞ¸ÄÄÚÈÝ   : ÐÂÉú³Éº¯Êý

*****************************************************************************/
void Robit_RunLR(uint8_t LeftMotor_Speed,uint8_t LeftMotor_Dir,uint8_t RightMotor_Speed,uint8_t RightMotor_Dir)
{
    
	if(LeftMotor_Speed > 10)
		LeftMotor_Speed = 10;
	if(RightMotor_Speed > 10)
		RightMotor_Speed = 10;
	if(LeftMotor_Dir > 1)
		LeftMotor_Dir = 1;
	if(RightMotor_Dir > 1)
		RightMotor_Dir = 1;

	if(LeftMotor_Speed > 0)
    {
        LeftMotorRunInit();
    }
    
    if(RightMotor_Speed > 0)
    {
        RightMotorRunInit();
    }


	RightMotorEnterRun(RightMotor_Dir);
	LeftMotorEnterRun(LeftMotor_Dir);

}

/*****************************************************************************
 º¯ Êý Ãû  : LeftMotorStop
 ¹¦ÄÜÃèÊö  : ×óÂÖÍ£Ö¹
 ÊäÈë²ÎÊý  : void  
 Êä³ö²ÎÊý  : ÎÞ
 ·µ »Ø Öµ  : 
 µ÷ÓÃº¯Êý  : 
 ±»µ÷º¯Êý  : 
 
 ÐÞ¸ÄÀúÊ·      :
  1.ÈÕ    ÆÚ   : 2016Äê3ÔÂ18ÈÕ
    ×÷    Õß   : ÌïºêÍ¼
    ÐÞ¸ÄÄÚÈÝ   : ÐÂÉú³Éº¯Êý

*****************************************************************************/
void LeftMotorStop(void)
{
//	Motor_L_F_PWM(0);
//	Motor_L_B_PWM(0);	
//	LeftMotor.RunEnable = 0;
    
    //20170721
    //Çå³þ×óÂÖ»»ÏòÆô¶¯±êÖ¾£¬·ÀÖ¹ÎóÆô¶¯·¢Éú£¬ÈôÐèºóÐø»»ÏòÆô¶¯£¬ÔÙ°É±êÖ¾Î»ÖÃÎ»
//    LeftMotorSwDirFlag = RESET;

	if(LeftMotor.Status != SYS_STOP)
		LeftMotor.Status = SYS_STOPING;
}

/*****************************************************************************
 º¯ Êý Ãû  : RightMotorStop
 ¹¦ÄÜÃèÊö  : ÓÒÂÖÍ£Ö¹
 ÊäÈë²ÎÊý  : void  
 Êä³ö²ÎÊý  : ÎÞ
 ·µ »Ø Öµ  : 
 µ÷ÓÃº¯Êý  : 
 ±»µ÷º¯Êý  : 
 
 ÐÞ¸ÄÀúÊ·      :
  1.ÈÕ    ÆÚ   : 2016Äê3ÔÂ18ÈÕ
    ×÷    Õß   : ÌïºêÍ¼
    ÐÞ¸ÄÄÚÈÝ   : ÐÂÉú³Éº¯Êý

*****************************************************************************/
void RightMotorStop(void)
{
//	Motor_R_F_PWM(0);
//	Motor_R_B_PWM(0);

//	RightMotor.RunEnable = 0;
    
    //20170721
    //Çå³þÓÒÂÖ»»ÏòÆô¶¯±êÖ¾£¬·ÀÖ¹ÎóÆô¶¯·¢Éú£¬ÈôÐèºóÐø»»ÏòÆô¶¯£¬ÔÙ°É±êÖ¾Î»ÖÃÎ»
//    LeftMotorSwDirFlag = RESET;

	if(RightMotor.Status != SYS_STOP)
	{
		RightMotor.Status = SYS_STOPING;
		
	}
}

/*
2016.7.25 
1.ÐÞ¸´BUG£¬µç»ú¶Â×ªºó£¬²»ÄÜ½øÈëÔËÐÐ×´Ì¬
*/
void MotorBlockCtrl(void)
{
	if(RightMotor.Status == SYS_BLOCKING)
	{
//		RightMotor.Status = SYS_STARTUP;
		
//		RightMotor.Status = SYS_STOP;
		RightMotorEnterRun(RightMotor.Dir);  // 2016.07.25 ¸ü¸Ä ÐÞ²¹¶Â×ªºó²»ÄÜ½øÈëÔËÐÐ×´Ì¬µÄBUG
	}

	if(LeftMotor.Status == SYS_BLOCKING)
	{
//		LeftMotor.Status = SYS_STARTUP;
		
//		LeftMotor.Status = SYS_STOP;
		LeftMotorEnterRun(LeftMotor.Dir);   //2016.07.25 ¸ü¸Ä ÐÞ¸´¶Â×ªºó²»ÄÜ½øÈëÔËÐÐ×´Ì¬µÄBUG
	}
}

/*****************************************************************************
 º¯ Êý Ãû  : MotorStopCtrl
 ¹¦ÄÜÃèÊö  : ×óÓÒÂÖÍ£Ö¹¿ØÖÆ  ,  1msµ÷ÓÃÒ»´Î
 ÊäÈë²ÎÊý  : void  
 Êä³ö²ÎÊý  : ÎÞ
 ·µ »Ø Öµ  : 
 µ÷ÓÃº¯Êý  : 
 ±»µ÷º¯Êý  : 
 
 ÐÞ¸ÄÀúÊ·      :
  1.ÈÕ    ÆÚ   : 2016Äê3ÔÂ21ÈÕ
    ×÷    Õß   : ÌïºêÍ¼
    ÐÞ¸ÄÄÚÈÝ   : ÐÂÉú³Éº¯Êý

*****************************************************************************/
void MotorStopCtrl(void)
{
	if(!MotorStopCtrlEn)
		return;
	MotorStopCtrlEn = DISABLE;
	
	if(RightMotor.Status == SYS_STOP)
	{
		RightMotor.RunEnable = DISABLE;
		RightMotor.RunFlag = RESET;
		RightMotor.CurSpeed = 0;
        
        RightMotor.PWMVal = 0;
	}
	else if(RightMotor.Status == SYS_STOPING)
	{
//		Motor_R_F_PWM(2400);
//		Motor_R_B_PWM(2400);
		Motor_R_Brake();
		
		RightMotor.PWMVal = 0;
        RightMotor.PWMVal = TIM4_Period;

		RightMotor.RunEnable = DISABLE;
		if(RightMotor.RunFlag == RESET)
		{
			RightMotor.Status = SYS_STOP;
			RightMotor.CurSpeed = 0;
//			Motor_R_F_PWM(0);
//			Motor_R_B_PWM(0);
			Motor_R_Stop();
            RightMotor.PWMVal = 0;
		}
		else
		{
			RightMotor.StopCnt++;
			if(RightMotor.StopCnt > Motor_Stop_Time)
			{
				RightMotor.RunFlag = RESET;
				RightMotor.StopCnt = 0;
			}
		}
	}

	if(LeftMotor.Status == SYS_STOP)
	{
		LeftMotor.RunEnable = DISABLE;
		LeftMotor.RunFlag = RESET;
		LeftMotor.CurSpeed = 0;
	}
	else if(LeftMotor.Status == SYS_STOPING)
	{
//		Motor_L_F_PWM(2400);
//		Motor_L_B_PWM(2400);
		Motor_L_Brake();
		LeftMotor.PWMVal = 0;
		LeftMotor.RunEnable = DISABLE;

		if(LeftMotor.RunFlag == RESET)
		{
			LeftMotor.Status = SYS_STOP;
			LeftMotor.CurSpeed = 0;
//			Motor_L_F_PWM(0);
//			Motor_L_B_PWM(0);
			Motor_L_Stop();
		}
		else
		{
			LeftMotor.StopCnt++;
			if(LeftMotor.StopCnt > Motor_Stop_Time)
			{
				LeftMotor.RunFlag = RESET;
				LeftMotor.StopCnt = 0;
			}
		}
	}
}

/*****************************************************************************
 º¯ Êý Ãû  : Robit_Stop
 ¹¦ÄÜÃèÊö  : É¨µØ»úÍ£Ö¹ÔË¶¯
 ÊäÈë²ÎÊý  : void  
 Êä³ö²ÎÊý  : ÎÞ
 ·µ »Ø Öµ  : 
 µ÷ÓÃº¯Êý  : 
 ±»µ÷º¯Êý  : 
 
 ÐÞ¸ÄÀúÊ·      :
  1.ÈÕ    ÆÚ   : 2016Äê3ÔÂ16ÈÕ
    ×÷    Õß   : ÌïºêÍ¼
    ÐÞ¸ÄÄÚÈÝ   : ÐÂÉú³Éº¯Êý

*****************************************************************************/
void Robit_Stop(void)
{
    LeftMotorStop();
    RightMotorStop();	
}

//kalman1_state CurKalman;
//uint16_t LeftMotorCur, RightMotorCur;
uint8_t rightIncrease,rightReduce;
int8_t rightCnt;
uint8_t CarpetStatus, CarpetCheckEn;
uint8_t Sta_Gyro, UpCnt, DownCnt;

uint32_t earthSum = 0;
uint16_t earthAvg;
uint8_t  earthWaveCnt = 0, earthCnt = 0, earthAvgOk = 0, earthWaveTimeCnt = 0;
uint16_t earthWaveMax,earthWaveMin;

#define  ANGLE_X_THRESHOLD  120

//µØÌº¼ì²â Ö÷Òª¼ì²âÂÖ×ÓµÄµçÁ÷ÖµµÄ±ä»¯
//20ms¼ì²âÒ»´Î
void CarpetCheck(void)
{
    static uint8_t init = 0, earthmaxminOK = 0;
    static uint16_t RunningCnt = 0, EarthCheckPre = 0, AngleCnt = 0; //rightCurPre,leftCurPre, 
//    static int16_t Gyro_x_Bak;
    static int32_t AngleSum = 0;
    uint16_t anglexdelt;
    
    if(!CarpetCheckEn)
        return;
    CarpetCheckEn = 0;
    
    //³õÊ¼»¯
    if(!init)
    {
        init = 1;
//        kalman1_init(&CurKalman, 0, 20, 1, 100);
//        rightCurPre = RightMotorCur;
//        leftCurPre = LeftMotorCur;
        CarpetStatus = 0;
        rightCnt = 0;
        
        Sta_Gyro = 0;
        UpCnt = 0;
        DownCnt = 0;
        
        earthWaveMax = 0;
        earthWaveMin = 0xffff;
        earthmaxminOK = 0;
    }
    
    
//    RightMotorCur = kalman1_filter(&CurKalman, ADC_ConvertedValue_Val[5]);
//    LeftMotorCur = kalman1_filter(&CurKalman, ADC_ConvertedValue_Val[6]);
    
    if(RightMotor.Status == SYS_RUNNING && LeftMotor.Status == SYS_RUNNING)
    {
        RunningCnt++;
    }
    else
    {
        RunningCnt = 0;
    }
    
    if(RunningCnt < 50)
        return;
    
    //·ÀÖ¹Òç³ö
    RunningCnt = 51;
    
    if(RightMotor.Dir != MOTOR_RUN_DIR_ZW || LeftMotor.Dir != MOTOR_RUN_DIR_ZW)
    {
        //Ã¿´Î×ªÍäÊ± ÇåÁãµØ¼ì²¨¶¯¼ÆÊý
        earthWaveCnt = 0;
        return;
    }
    
    // 5S¼ÆÊ±
    if(earthWaveTimeCnt < 250)
    {
        earthWaveTimeCnt++;   
        
        if(earthWaveTimeCnt == 200)
            earthmaxminOK = 1;
    }
    else
    {
        earthWaveTimeCnt = 0;
        earthWaveCnt = 0;
        
        earthWaveMax = 0;
        earthWaveMin = 0xffff;
        earthmaxminOK = 0;
    }
    
    //µØ¼ì²¨¶¯ÔÚ+300~-300·¶Î§ÄÚ
    if(EarthCheckPre < (ADC_ConvertedValue_Val[0] + 500) && (EarthCheckPre + 500) > ADC_ConvertedValue_Val[0])
    {
        //ÇóÆ½¾ùÖµ
        earthSum += ADC_ConvertedValue_Val[0];
        earthCnt++;
        
        if(earthCnt >= 64)
        {
            earthAvg = earthSum >> 6;
            earthCnt = 0;
            earthSum = 0;
            earthAvgOk = 1;
        }       
    }
    //µØ¼ì²¨¶¯½Ï´ó
    else
    {
        earthCnt = 0;
        earthSum = 0;
        //ÀÛ¼Æ¼ÆÊ±
        //´ÓµÚÒ»´Î²¨¶¯¿ªÊ¼¼ÆÊ±£¬¼ÆËã5SÄÚµÄ²¨¶¯Êý Èç´óÓÚ18ÔòÈÏÎªÓÐµØÌº£¬·ñÔòÈÏÎªÃ»ÓÐ
        if(earthWaveCnt == 0)
        {
            earthWaveTimeCnt = 0;
        }
        earthWaveCnt++;    

    }
    
    if(earthWaveMax < ADC_ConvertedValue_Val[0])
    {
        earthWaveMax = ADC_ConvertedValue_Val[0];
    }
    
    if(earthWaveMin > ADC_ConvertedValue_Val[0])
    {
        earthWaveMin = ADC_ConvertedValue_Val[0];
    }
    
    EarthCheckPre = ADC_ConvertedValue_Val[0];
    
    if(IcmDataByKalman.Gyro_x > 2000 || IcmDataByKalman.Gyro_x < -2000)
    {
        Sta_Gyro = 1;
        UpCnt = 0;
    }
    
    if(Sta_Gyro)
    {
        UpCnt++;
        if(UpCnt > 30)
        {
            UpCnt = 0;
            Sta_Gyro = 0;
        }
    }
    
    anglexdelt = CalAngleDif(Sys_Angle2, Sys_AnglexInit);
//    test2 = anglexdelt;
    
    //xÖá½Ç¶ÈÐ¡ÓÚ³õÊ¼Öµ 120 ÈÏÎªÉÏÆÂ¶¯×÷ ANGLE_X_THRESHOLD = 120
    if(anglexdelt > 18000 && anglexdelt < 36000 - ANGLE_X_THRESHOLD && Sta_Gyro)
    {
        CarpetStatus = 2;
        earthAvgOk = 0;
        MopPar.MopEn = DISABLE;
    }
    
    //xÖá½Ç¶È´óÓÚ³õÊ¼Öµ120 ÈÏÎªÏÂÆÂ¶¯×÷
    else if(anglexdelt > ANGLE_X_THRESHOLD && anglexdelt < 18000 && Sta_Gyro)
    {
        CarpetStatus = 1;
        
        earthWaveCnt = 0;
        
        MopPar.MopEn = ENABLE;
    }
    else if(Sta_Gyro == 0)
    {       
        AngleSum += Sys_Angle2;
        if(AngleCnt < 256)
        {
            AngleCnt++;
            Sys_AnglexInit = AngleSum / AngleCnt;
        }
        else
        {
            Sys_AnglexInit = AngleSum >> 8;
            AngleSum -= Sys_AnglexInit;
        }
    }
    
    //²»ÔÚµØÌºÉÏ
    if(1 == CarpetStatus || 0 == CarpetStatus)
    {
        if((earthWaveCnt >= 18 && earthWaveTimeCnt <= 250) || (earthWaveMax > 3000 && earthWaveMax - earthWaveMin > 2000 && earthmaxminOK)) 
        {
            //¿ÉÄÜÔÚµØÌºÉÏ ÅÐ¶ÏÆäËûÌõ¼þ ¡
            CarpetStatus = 2;
            earthAvgOk = 0;
            MopPar.MopEn = DISABLE;
            earthmaxminOK = 0;
//            TDMotorOff();
        }
    }
    //ÔÚµØÌºÉÏ
    else if(2 == CarpetStatus || 0 == CarpetStatus)
    {
        //Æ½¾ùÖµÐ¡ÓÚ1500»òÕßÎÈ¶¨³ÖÐøÊ±¼ä´óÓÚ3S
        if((earthAvgOk && earthAvg < 1500)||(earthWaveMax < 2000 && earthWaveMax - earthWaveMin < 1500 && earthmaxminOK))
        {
            //¿ÉÄÜ²»ÔÚµØÌºÉÏ
            CarpetStatus = 1;
            earthWaveCnt = 0;
            MopPar.MopEn = ENABLE;
            earthmaxminOK = 0;
            
//            TDMotorOn();
        }
    }
    
//    if(fabs(RightMotorCur - rightCurPre) > 20)
//    {
//        //Ôö¼Ó
//        if(RightMotorCur > rightCurPre)
//        {
//            rightIncrease++;
//            rightReduce = 0;
//            rightCnt++;
//        }
//        //¼õÐ¡
//        else
//        {
//            rightIncrease = 0;
//            rightReduce++;
//            rightCnt--;
//        }
//        
//        if(rightIncrease >= 3)
//        {
//            CarpetStatus = 1;
//        }
//        else if(rightReduce >= 3)
//        {
//            CarpetStatus = 2;
//        }
//    }
//    else
//    {
//        rightIncrease = 0;
//        rightReduce = 0;
//        rightCnt = 0;
//    }
//    rightCurPre = RightMotorCur;
}

/*
1.»ùÓÚµ±Ç°µÄÑù»ú³ß´ç£¬³¬Éù²¨ÖÐÐÄ¾à119.25mm,¼ÆËã»úÉíÓëÇ½µÄ¼Ð½ÇÈçÏÂ£º
    y = atan(x / 119.25mm)
-------------------------------------------------------------------
    Ç°ºó³¬Éù²¨Êý¾Ý²î (x)       |     ¼Ð½Ç(½Ç¶È£¬0.01¶È¾«¶È£¬y)  
-------------------------------------------------------------------
           x < 301			   |      y = (62 * x + 216) >> 3
-------------------------------------------------------------------
    301 <= x < 501             |      y = (49 * x + 3944) >> 3
-------------------------------------------------------------------
    501 <= x < 701             |      y = (38 * x + 9600) >> 3
-------------------------------------------------------------------
           x > 701             |       ²»¼ÆËã½Ç¶È£¬Ó¦µ÷Ð¡¼Ð½Ç
-------------------------------------------------------------------

2.¼ÆËãÑù»ú»úÉí×óÇ°·½½ÇÓëÇ½ÃæµÄ¾àÀë(y),Ç°³¬Éù²¨Êý¾ÝD1£¬Ç°ºó³¬Éù²¨Êý¾Ý²îD2£¬³¬Éù²¨Ì½Í·Óë»úÉí±ß½çµÄ¾àÀëD4(39mm)
³¬Éù²¨ÖÐÐÄ¾àD3(119.25mm),Ç°³¬Éù²¨Óë»úÉí×îÇ°·½µÄ¾àÀëL£¨166mm£©,»úÉíÓëÇ½ÃæµÄ¼Ð½Ça.
    y = (D3 * (D1 - D4) / D2 - L) * sin(a).
*/
/*
int16_t DistanceCalu(void)
{
	int16_t tmpdelt, tmpangle;
	int32_t tmpdata; //ÖÐ¼ä±äÁ¿
	
	// 1. ¼ÆËã¼Ð½Ç
	tmpdelt = Ultra_RevTimeVlid[2] - Ultra_RevTimeVlid[1];
	
	if(tmpdelt > 0 && tmpdelt <= 701)
	{
		if(tmpdelt < 301)
		{
			tmpdata = tmpdelt * 62 + 216;
			tmpangle = tmpdata >> 3;
		}
		else if(tmpdelt < 501)
		{
			tmpdata = tmpdelt * 49 + 3944;
			tmpangle = tmpdata >> 3;
		}
		else
		{
			tmpdata = tmpdelt * 38 + 9600;
			tmpangle = tmpdata >> 3;
		}
		test2 = tmpangle ;
		
	// 2. ¼ÆËã¾àÀë
		tmpdata = 701 * (Ultra_RevTimeVlid[1] - 230) / tmpdelt - 976;
		tmpdata = (MF_DSIN(tmpangle) * tmpdata) >> 15;	
		
	}
	else
	{
		tmpdata = 0;
	}
	
	return (int16_t)tmpdata;
}*/

int16_t EdageRunAngle(void)
{
	int16_t Wall_Angle,tmpangle;	
	int16_t Timedelt;
	uint16_t tmpdistance;
	static int32_t tmpangle2 = 0;
	
//	Ultra_Jump[1] = DISABLE;
//	Ultra_Jump[2] = DISABLE;
	if(Ultra_RevTimeVlid[1] > Edage_Distance && Ultra_RevTimeVlid[2] > Edage_Distance)
	{
//		Wall_Angle = (Ultra_RevTimeVlid[1] - Ultra_RevTimeVlid[2]) * 100 / 11;
//		tmpangle2 = Ultra_RevTimeVlid[1] * 100; // 15
//		tmpangle2 >>= 4;
//		if(tmpangle2 > 4000)
//			tmpangle2 = 4000;
		
		if(Ultra_RevTimeVlid[1] >= Ultra_RevTimeVlid[2])
		{
			Timedelt = Ultra_RevTimeVlid[1] - Ultra_RevTimeVlid[2];
			if(Timedelt > 700)
			{
				Wall_Angle = 0;	
			}
			else
			{
				Wall_Angle = Timedelt * 100 / 15;
			}
			
			Timedelt = Timedelt * 7;
			Timedelt /= 5;
//			if(Timedelt < 700)
//			{
				if(Timedelt  + Edage_Distance < Ultra_RevTimeVlid[1])
				{
//					tmpangle2 = Ultra_RevTimeVlid[1] - Timedelt - 400;
//					tmpangle2 *= 100;
//					tmpangle2 >>= 6;					
//					tmpangle2 += 200;
					
					tmpdistance = Wall_Angle / 100;//Ultra_RevTimeVlid[1] - 300 - Timedelt;
					if(tmpdistance > 40)
						tmpdistance = 40;
					
					tmpdistance = Angle_Num[tmpdistance];
					
					tmpangle2 = Ultra_RevTimeVlid[1] - Timedelt - Edage_Distance;
					tmpangle2 = tmpangle2 * tmpdistance ;
					tmpangle2 *= 5;
					tmpangle2 >>= 6;
				}
				else
				{
					tmpangle2 = 0;
				}
				
				if(tmpangle2 > 4000)
				{
					tmpangle2 = 4000;
				}
				
//				tmpangle2 = 0;
//			}
//			else
//			{
//				tmpangle2 = 0;
//			}
			
			tmpangle = Sys_Angle + Wall_Angle + tmpangle2;
			if(tmpangle > 18000)
			{
				tmpangle -= 36000;
			}
			
			return tmpangle;
		}
		else
		{
			Timedelt = Ultra_RevTimeVlid[2] - Ultra_RevTimeVlid[1];
			if(Timedelt > 700)
			{
				Wall_Angle = 0;	
			}
			else
			{
				Wall_Angle = Timedelt * 100 / 15;
			}
			
			Timedelt = Timedelt * 7;
			Timedelt /= 5;
			
//			if(Timedelt < 700)
//			{
				if(Timedelt  + Edage_Distance < Ultra_RevTimeVlid[1])
				{
					tmpdistance = Wall_Angle / 100;//Ultra_RevTimeVlid[1] - 300 - Timedelt;
					if(tmpdistance > 40)
						tmpdistance = 40;
					
					tmpdistance = Angle_Num[tmpdistance];
					
					tmpangle2 = Ultra_RevTimeVlid[1] - Timedelt - Edage_Distance;
					tmpangle2 = tmpangle2 * tmpdistance ;
					tmpangle2 *= 5;
					tmpangle2 >>= 6;					
//					tmpangle2 += 200;
				}
				else
				{
					tmpangle2 = 0;
				}
				
				
				if(tmpangle2 > 4000)
				{
					tmpangle2 = 4000;
				}
//				tmpangle2 = 0;
//			}
//			else
//			{
//				tmpangle2 = 0;
//			}
//			
			tmpangle = Sys_Angle - Wall_Angle + tmpangle2;
			if(tmpangle > 18000)
			{
				tmpangle -= 36000;
			}
			else if(tmpangle < -18000)
			{
				tmpangle += 36000;
			}
			
			return tmpangle;	
		}

	}
	else
	{
		Timedelt = Ultra_RevTimeVlid[1] - Ultra_RevTimeVlid[2];
		Wall_Angle = Timedelt * 100 / 14;
		
//		if(Timedelt )
		tmpangle = Sys_Angle + Wall_Angle;
		

		return tmpangle;
	}
}
/*****************************************************************************
 º¯ Êý Ãû  : TIM2_IRQHandler
 ¹¦ÄÜÃèÊö  : TIM2Òç³öÖÐ¶Ï£¬ÓÃÓÚ×óÓÒÂÖ²âËÙ
 ÊäÈë²ÎÊý  : void  
 Êä³ö²ÎÊý  : ÎÞ
 ·µ »Ø Öµ  : 
 µ÷ÓÃº¯Êý  : 
 ±»µ÷º¯Êý  : 
 
 ÐÞ¸ÄÀúÊ·      :
  1.ÈÕ    ÆÚ   : 2016Äê3ÔÂ17ÈÕ
    ×÷    Õß   : ÌïºêÍ¼
    ÐÞ¸ÄÄÚÈÝ   : ÐÂÉú³Éº¯Êý

*****************************************************************************/
void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2 , TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM2 , TIM_IT_Update);
		
		LeftMotorSpeed.OverflowCnt++;
		RightMotorSpeed.OverflowCnt++;
		
	}	
}


/*****************************************************************************
 º¯ Êý Ãû  : EXTI1_IRQHandler
 ¹¦ÄÜÃèÊö  : ÓÒÂÖËÙ¶È¼ì²âÖÐ¶Ï
 ÊäÈë²ÎÊý  : void  
 Êä³ö²ÎÊý  : ÎÞ
 ·µ »Ø Öµ  : 
 µ÷ÓÃº¯Êý  : 
 ±»µ÷º¯Êý  : 
 
 ÐÞ¸ÄÀúÊ·      :
  1.ÈÕ    ÆÚ   : 2016Äê3ÔÂ8ÈÕ
    ×÷    Õß   : ÌïºêÍ¼
    ÐÞ¸ÄÄÚÈÝ   : ÐÂÉú³Éº¯Êý

*****************************************************************************/
//void EXTI9_5_IRQHandler(void)
void EXTI1_IRQHandler(void)
{
//	uint32_t tmpcircletime,tmpcurcountvalue;
//	uint8_t tmpoverflowcntbuf;
//    uint16_t tmpmotordata;
//	uint8_t tmpsta;
	static uint32_t tmpcntla = 0;//, tmpcntlavld = 0;
//	static uint8_t tmpfallingcnt = 0;
	uint32_t tmpcnt,tmpcntdel,tmpcntdelmin;
	
	if ( EXTI_GetITStatus(EXTI_Line1) != RESET )
	{
		EXTI_ClearITPendingBit(EXTI_Line1);
		
//		///ÔÙ´Î¶ÁÈ¡ÖÐ¶Ï¿ÚµçÆ½£¬ÅÐ¶ÏÊÇ·ñÏÂ½µÑØ£¨µÍµçÆ½£©
//		if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5) != RESET)
//			return;

		// Âö¿í¼ÆËã£¬ÈôÐ¡ÓÚÉè¶¨Öµ£¬ÔòÈÏÎªÊÇÒ»´Î´íÎóµÄ´¥·¢ÐÅºÅ//////////
		if(RightMotor.RunEnable == ENABLE && RightMotor.RunFlag != SET)
		{
			tmpcntla = RightMotorSpeed.PreCountValue;
		}
		
		tmpcnt = TIM_GetCounter(TIM2);
		
		if(tmpcnt >= tmpcntla)
		{
			tmpcntdel = tmpcnt - tmpcntla;
		}
		else
		{
			tmpcntdel = 0xffff - tmpcntla + tmpcnt;			
		}
		
		if(RightMotor.SpeedRisedFlag)
		{
			tmpcntdelmin = Motor_InterTimerMin;//RightMotorSpeed.CircleTime >> 2;
//			tmpcntdelmin /= 5;
		}
		else
		{
			tmpcntdelmin = Motor_InterTimerMin;
		}
		
		tmpcntla = tmpcnt;
		
		if(tmpcntdel < tmpcntdelmin)
//		if(tmpcntdel < RightMotorSpeed.CircleTime)
			return;
		//////////////////////////////
		
		
		
		RightMotorSpeed.PulseCnt ++;
		
		if(RightMotorSpeed.PulseCnt >= 2)
		{
			RightMotorSpeed.PulseCnt = 0;
			return;
		}
		
		
		
        
		//ÓÒÂÖÆô¶¯ºóµÚÒ»´Î½øÖÐ¶Ï
		if(RightMotor.RunEnable == ENABLE && RightMotor.RunFlag != SET)
		{
			RightMotor.RunFlag = SET;  				// ÓÒÂÖÔËÐÐ±êÖ¾ÖÃÎ»
			
			
			if(RightMotor.Status == SYS_STARTUP)
				RightMotor.Status = SYS_RUNNING;  	//ÓÒÂÖ×´Ì¬ÇÐ»»ÖÁÔËÐÐ×´Ì¬
            
//            TIM_Cmd(TIM7, ENABLE);

		}

//		RightMotor.RunCycle ++;
		
		    
		// Àï³Ì¼Æ¼ÆÊý£¬Ç°½øÔö¼Ó£¬ºóÍË¼õÉÙ
		if(RightMotor.Dir == MOTOR_RUN_DIR_ZW)
		{
			RightMileageAll ++;	
		}
		else if(RightMotor.Dir == MOTOR_RUN_DIR_FW)
		{
			RightMileageAll --;	
		}

		//Àï³Ì»ò½Ç¶È¿ØÖÆÄ£Ê½Ê±£¬ÐÐ×ßÀï³ÌÊýÔö¼Ó
		if(RobitRouteModeEn)
		{
			RightCurMileage ++;
		}

		
		RightMotor.StopCnt = 0;  //Í£Ö¹¼ÆÊýÇåÁã
		RightMotor.Block_Cnt = 0;//¶Â×ª¼ÆÊýÇåÁã


        //Ò»´ÎÂö³å¼ÆËãÒ»´ÎËÙ¶È
        if(tmpcnt > RightMotorSpeed.PreCountValue)
        {
            RightMotorSpeed.CircleTime = tmpcnt - RightMotorSpeed.PreCountValue;
        }
        else
        {
            RightMotorSpeed.CircleTime = tmpcnt + 0xffff - RightMotorSpeed.PreCountValue;
        }
        
        RightMotorSpeed.CalEn = ENABLE;
        
        RightMotorSpeed.PreCountValue = tmpcnt;

        
	}



//	if ( EXTI_GetITStatus(EXTI_Line8) != RESET )
//	{
//		EXTI_ClearITPendingBit(EXTI_Line8);
//		
//		Ultra_RevID = 1;

//		if(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_8) == SET)
//		{
//			Ultra_RevLv[1] = 1;	
//		}
//		else
//		{
//			Ultra_RevLv[1] = 0;	
//		}

//		Ultra_Deal();
//	}

//	if ( EXTI_GetITStatus(EXTI_Line9) != RESET )
//	{
//		EXTI_ClearITPendingBit(EXTI_Line9);
//		
//		Ultra_RevID = 2;

//		if(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_9) == SET)
//		{
//			Ultra_RevLv[2] = 1;	
//		}
//		else
//		{
//			Ultra_RevLv[2] = 0;	
//		}

//		Ultra_Deal();
//	}
}



/*****************************************************************************
 º¯ Êý Ãû  : EXTI9_5_IRQHandler
 ¹¦ÄÜÃèÊö  : ×óÂÖËÙ¶È¼ì²âÖÐ¶Ï
 ÊäÈë²ÎÊý  : void  
 Êä³ö²ÎÊý  : ÎÞ
 ·µ »Ø Öµ  : 
 µ÷ÓÃº¯Êý  : 
 ±»µ÷º¯Êý  : 
 
 ÐÞ¸ÄÀúÊ·      :
  1.ÈÕ    ÆÚ   : 2016Äê3ÔÂ8ÈÕ
    ×÷    Õß   : ÌïºêÍ¼
    ÐÞ¸ÄÄÚÈÝ   : ÐÂÉú³Éº¯Êý

*****************************************************************************/
void EXTI9_5_IRQHandler(void)
//void EXTI1_IRQHandler(void)
{
//	uint32_t tmpcircletime,tmpcurcountvalue;
//	uint8_t tmpoverflowcntbuf;
//    uint16_t tmpmotordata;
	
	static uint32_t tmpcntla = 0;
//	static uint8_t tmpfallingcnt = 0;
	uint32_t tmpcnt,tmpcntdel,tmpcntdelmin;
	
    
    
	if ( EXTI_GetITStatus(EXTI_Line5) != RESET )
	{
		EXTI_ClearITPendingBit(EXTI_Line5);	
		
		
		tmpcnt = TIM_GetCounter(TIM2);
		
		if(LeftMotor.RunEnable == ENABLE && LeftMotor.RunFlag != SET)
		{
			tmpcntla = LeftMotorSpeed.PreCountValue;
		}
		
		if(tmpcnt >= tmpcntla)
		{
			tmpcntdel = tmpcnt - tmpcntla;
		}
		else
		{
			tmpcntdel = 0xffff - tmpcntla + tmpcnt;			
		}
		
		if(LeftMotor.SpeedRisedFlag)
		{
			tmpcntdelmin = Motor_InterTimerMin;//LeftMotorSpeed.CircleTime >> 2;
//			tmpcntdelmin /= 5;
		}
		else
		{
			tmpcntdelmin = Motor_InterTimerMin;
		}
		
		tmpcntla = tmpcnt;
		
		
		
		if(tmpcntdel < tmpcntdelmin)
			return;
		
		
		LeftMotorSpeed.PulseCnt ++;
		
		if(LeftMotorSpeed.PulseCnt == 2)
		{
			LeftMotorSpeed.PulseCnt = 0;
			return;
		}
        
        
            
		//×óÂÖÆô¶¯ºóµÚÒ»´Î½øÖÐ¶Ï
		if(LeftMotor.RunEnable == ENABLE && LeftMotor.RunFlag != SET)
		{
			LeftMotor.RunFlag = SET;
			
			if(LeftMotor.Status == SYS_STARTUP)
				LeftMotor.Status = SYS_RUNNING;
			
//			if(tmpcnt >= LeftMotorSpeed.PreCountValue)
//			{
//				tmpcntdel = tmpcnt - LeftMotorSpeed.PreCountValue; 
//			}
//			else
//			{
//				tmpcntdel = LeftMotorSpeed.PreCountValue - tmpcnt;
//			}
            
//            TIM_Cmd(TIM1, ENABLE);
		}

		LeftMotor.RunCycle ++;

		if(LeftMotor.Dir == MOTOR_RUN_DIR_ZW)
		{
			LeftMileageAll ++;	
		}
		else if(LeftMotor.Dir == MOTOR_RUN_DIR_FW)
		{
			LeftMileageAll --;	
		}

		if(RobitRouteModeEn)
		{
			LeftCurMileage ++;
		}
		
		LeftMotor.StopCnt = 0;
		LeftMotor.Block_Cnt = 0;

			
        if(tmpcnt > LeftMotorSpeed.PreCountValue)
        {
            LeftMotorSpeed.CircleTime = tmpcnt - LeftMotorSpeed.PreCountValue;
        }
        else
        {
            LeftMotorSpeed.CircleTime = tmpcnt + 0xffff - LeftMotorSpeed.PreCountValue;
        }
        
        LeftMotorSpeed.CalEn = ENABLE;
        
        LeftMotorSpeed.PreCountValue = tmpcnt;

        
	}
    
    //Ë¯Ãß»½ÐÑ
    if ( EXTI_GetITStatus(EXTI_Line8) != RESET )
	{
		EXTI_ClearITPendingBit(EXTI_Line8);	
        
//        if(SystePwrMode)
//        {
//            __set_FAULTMASK(1);// ??????
//            NVIC_SystemReset();// ??
//            Device_Init();
//        }
//        VL53L0X_IRQ();  
        vl53l0x_Mesure_En = 1; 
    }
    
    if ( EXTI_GetITStatus(EXTI_Line9) != RESET )
	{
		EXTI_ClearITPendingBit(EXTI_Line9);	
        
        if(SystePwrMode)
        {
            __set_FAULTMASK(1);// ??????
            NVIC_SystemReset();// ??
            Device_Init();
        }
    }
}

