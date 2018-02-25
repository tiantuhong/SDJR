/******************************************************************************

                  ��Ȩ���� (C), 1994-2016, ���ݾ���ŷ�϶�С�ҵ����޹�˾

 ******************************************************************************
  �� �� ��   : Battery.c
  �� �� ��   : V1.0
  ��    ��   : ���ͼ
  ��������   : 2016��4��25��
  ����޸�   :
  ��������   : ��ع���ģ��
  �����б�   :
  �޸���ʷ   :
  1.��    ��   : 2016��4��25��
    ��    ��   : ���ͼ
    �޸�����   : �����ļ�

******************************************************************************/

/*----------------------------------------------*
 * ����ͷ�ļ�                                   *
 *----------------------------------------------*/
#include "stm32f10x.h"
#include "kalman_filter.h"
/*----------------------------------------------*
 * �ⲿ����˵��                                 *
 *----------------------------------------------*/

/*----------------------------------------------*
 * �ⲿ����ԭ��˵��                             *
 *----------------------------------------------*/

/*----------------------------------------------*
 * �ڲ�����ԭ��˵��                             *
 *----------------------------------------------*/
uint16_t BatteryVolCorrect(uint16_t _vol_);
/*----------------------------------------------*
 * ȫ�ֱ���                                     *
 *----------------------------------------------*/
uint8_t BatterySoc, BatterySocDis;				//���ʣ����� ���100 ��С0
uint8_t BatteryStatus;						//0-δ��� 1-���ڳ�� 2-������
uint8_t BatteryChargeReadyStatus;			//��س��׼��״̬ 0��1��2
uint8_t BatteryConnect;						//0--δ���� 1--����
uint16_t BatteryChargePWMVal;				//������PWMֵ
FunctionalState SocketPower,JackPower,BatteryCheckEn,BatteryChargeEn;
uint8_t BatteryError;
uint16_t BatteryOffCnt;
/*----------------------------------------------*
 * ģ�鼶����                                   *
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
 * ��������                                     *
 *----------------------------------------------*/
uint16_t BatteryVol2Soc[11] = 
{
	// 100%/16.8V   90%/16.24  80%/15.92   70%/15.68  60%/15.48  50%/15.28  40%/15.16  30%/15.08  20% 14.96  10%/14.72  0%/12V
	     0x767,      0x728,     0x704,      0x6e9,     0x6d2,     0x6bc,      0x6b9,     0x6a5,     0x698,     0x67c,    0x54a
};

/*----------------------------------------------*
 * �궨��                                       *
 *----------------------------------------------*/

/*****************************************************************************
 �� �� ��  : Battery_Conf
 ��������  : ��ز�����ʼ��
 �������  : void  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2016��4��25��
    ��    ��   : ���ͼ
    �޸�����   : �����ɺ���

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
 �� �� ��  : BatterySocCheck
 ��������  : ��ص������
 �������  : void  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 

 1. 1s����һ��
 2.���ʱ���رճ���⿪·��ѹ
 
 �޸���ʷ      :
  1.��    ��   : 2016��4��25��
    ��    ��   : ���ͼ
    �޸�����   : �����ɺ���

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
		
		//��ʱ10ms
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
 �� �� ��  : BatteryCharge
 ��������  : ��س�����
 �������  : void  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
1.20ms����һ��
 
 �޸���ʷ      :
  1.��    ��   : 2016��4��25��
    ��    ��   : ���ͼ
    �޸�����   : �����ɺ���

*****************************************************************************/
void BatteryCharge(void)
{
//	uint16_t Tmpcurrent;
	if(!BatteryChargeEn)
		return;
	BatteryChargeEn = DISABLE;
	
	//������Ƿ�����
	if(SocketPower != ENABLE /*&& JackPower != ENABLE*/)
	{
		//�رճ��
//		Set_Charge_PWM(0);
		BatteryStatus = 0;

		BatteryChargeFullEn = ENABLE;
		
		return;
	}

	//����Ƿ�����
//	if(BatteryConnect == 0)
//	{
//		//�رճ��
////		Set_Charge_PWM(0);
//		BatteryStatus = 0;
//		return;	
//	}
	
	//��ص����Ƿ����
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
//				//�رճ��
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
//			//�رճ��
////			Set_Charge_PWM(0);
//			BatteryStatus = 0;
//			return;	
//		}
//	}

	if(BatteryError)
		return;
	
	
	//�Ƿ����ڼ���ص���(����ص���ʱ��رճ�磬��ʱ������紦��)
//	if(BatteryCheckEn)
//		return;
	
	//���ݵ�س��״̬��ͬ����
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
//			//�������״̬���������������趨����
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
//						//��1�벻��⣬�˵����ܵ��ϵ���
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
//							//�رճ��
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
//								//�رճ��
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
//								//�رճ��
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
//			//�ȶ����״̬����������������(0.5A)
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
//			//С�������  20����Ȼ��Ͽ�
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
//				//�رճ��
//				Set_Charge_PWM(0);
//				BatteryStatus = 0;
//				BatteryChargeFullEn = DISABLE;
//			}
//			
//			break;
//		default:
//			//�رճ��
//			Set_Charge_PWM(0);
//			BatteryStatus = 0;
//			break;
//	}



    
}

