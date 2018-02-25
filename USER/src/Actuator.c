/******************************************************************************

                  ��Ȩ���� (C), 1994-2015, ���ݾ���ŷ�϶�С�ҵ����޹�˾

 ******************************************************************************
  �� �� ��   : Actuator.c
  �� �� ��   : V1.0
  ��    ��   : ���ͼ
  ��������   : 2016��4��28��
  ����޸�   :
  ��������   : ��ɨ����ɨ�������������
  �����б�   :
  �޸���ʷ   :
  1.��    ��   : 2016��4��28��
    ��    ��   : ���ͼ
    �޸�����   : �����ļ�

******************************************************************************/

/*----------------------------------------------*
 * ����ͷ�ļ�                                   *
 *----------------------------------------------*/
#include "stm32f10x.h"

/*----------------------------------------------*
 * �ⲿ����˵��                                 *
 *----------------------------------------------*/

/*----------------------------------------------*
 * �ⲿ����ԭ��˵��                             *
 *----------------------------------------------*/

/*----------------------------------------------*
 * �ڲ�����ԭ��˵��                             *
 *----------------------------------------------*/

/*----------------------------------------------*
 * ȫ�ֱ���                                     *
 *----------------------------------------------*/
FunctionalState SprayWaterEn,ActuatorEn,CleanEn,DustEn,RagChangeEn;
uint8_t SparyWaterInit,SparyWaterOnCnt;
uint8_t CleanInit = 0,DustInit = 0;
uint16_t SparyWaterOffCnt,CleanPwm,CleanPwmStep,DustPwm,DustPwmStep;
uint16_t CleanPwmOn, DustPwmOn;
uint8_t fanLevelPre, sideBroomLevelPre, midBroomLevelPre;
/*----------------------------------------------*
 * ģ�鼶����                                   *
 *----------------------------------------------*/
MopTypeDef MopPar;
/*----------------------------------------------*
 * ��������                                     *
 *----------------------------------------------*/

/*----------------------------------------------*
 * �궨��                                       *
 *----------------------------------------------*/
#define MOPSTOPTIME     50      //�ϵؼ��ʱ�� ��λms

void Actuator_Conf(void)
{
	SprayWaterEn = DISABLE;
	RagChangeEn = DISABLE;
	SparyWaterInit = 0;
    DustPowerOff();
    
    //�ϵز�������
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

//����50ms����һ��
void Actuator_Deal(void)
{
	
	if(!ActuatorEn)
		return;
	ActuatorEn = DISABLE;
	
	//��ɨ���������
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
	
	//�������������
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
	
	//Ĩ����������
	//1.��⵽΢�������źţ����ֹͣ��Ĩ���ɸ���
	//2.��΢�����رպ�״̬���������0.5s,Ĩ����ת�ɹ�
	//3.��־λRagChangeEn ENABLE=Ĩ����Ҫ����  DISABLE=Ĩ����Ҫ��װ
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

//1ms ִ��һ�θú���
void MopCtrl(void)
{
    static uint8_t RagChangeSingalBak = 0;//, MopTurnDir, MopPos = 0;
    
    //RagChangeSingal = 1 �źű��ڵ�
    //RagChangeSingal = 0 �ź�δ���ڵ�
    
    
    //��ʼ��λ��
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
            //������
            if(0 == RagChangeSingal)
            {
                //��һ�ν���
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
 
                    //�Ѿ���ת��һȦ
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
            //�½���
            else if(MopPosCheckStart)
            {
                highLvTime = timeCnt;
                timeCnt = 0;
            }
        }
        
        RagChangeSingalBak = RagChangeSingal;
        return;
    }
    
    //λ�ü���
    if(RagChangeSingalBak != RagChangeSingal)
    {
        //������
        //MopTurnDir 1--˳ʱ��ת  0--��ʱ��ת
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
        //�½���
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
        
        //�źŷ����仯�Ǹ����ϲ�λ��
        // MopTurnDir = 1 ˳ʱ�� + �� MopTurnDir = 0 ��ʱ��-
        if(MopPar.TurnDir)
        {
            MopPar.Pos++;
        }
        else
        {
            MopPar.Pos--;
        }
        //��Χ0~7 ��ֹ���(+/-���)
        MopPar.Pos &= 0x07;
        
        RagChangeSingalBak = RagChangeSingal;
        LED4_SW;
    }
    
    //ֹͣ����
    if(!MopPar.MopEn)
    {
        //ÿ���ϲ�ֹͣ��λ�ö�ֹͣ�ڳ�λ��
        
        //��϶λ��
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
        //��λ��
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
    
    //ת������
    //�ϲ������˶�״̬
    if(MopPar.LoopStatus)
    {
        //δ�ﵽֹͣλ��
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
        else   //�ﵽֹͣλ��
        {
            //�ϲ�״̬�л�Ϊֹͣ״̬
            MopPar.LoopStatus = 0;
            MopPar.MopOffCnt = 0;
        }
    }
    else
    {
        //ֹͣʱ�䵽���ϲ��л������������
        //�ϲ���ת�����л���StartPos  StopPos���� 
        if(MopPar.MopOffCnt >= MOPSTOPTIME)
        {
            uint8_t tmppos;
            
            //�л�Ϊ����
            MopPar.LoopStatus = 1;
            
            //�л�����
            if(MopPar.MopDir == MOTOR_RUN_DIR_ZW)
            {
                MopPar.MopDir = MOTOR_RUN_DIR_FW;
            }
            else
            {
                MopPar.MopDir = MOTOR_RUN_DIR_ZW;
            }
            
            // StartPos he  StopPos ����
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

