/******************************************************************************

                  ��Ȩ���� (C), 1994-2015, ���ݾ���ŷ�϶�С�ҵ����޹�˾

 ******************************************************************************
  �� �� ��   : Display_Key.c
  �� �� ��   : V1.0
  ��    ��   : ���ͼ
  ��������   : 2016��4��8��
  ����޸�   :
  ��������   : LED��ʾ�Ͱ�������
  �����б�   :
  �޸���ʷ   :
  1.��    ��   : 2016��4��8��
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
void Key_Scan(void);

/*----------------------------------------------*
 * �ڲ�����ԭ��˵��                             *
 *----------------------------------------------*/
void Key_Deal(void);

/*----------------------------------------------*
 * ȫ�ֱ���                                     *
 *----------------------------------------------*/
FunctionalState KeyScanEn;
FunctionalState DisplayScanEn;
FlagStatus Flag_Key_LongPress, FlagResetWifi;

/*----------------------------------------------*
 * ģ�鼶����                                   *
 *----------------------------------------------*/
uint8_t Key_Code = 0,g_key_data,g_key_data_Pre,Key_Status = 0,g_key_Continuum_con = 0,g_key_Continuum_tim;
uint8_t DisCnt = 0;

uint8_t g_display_buf[10] = 
{
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00
};

/*----------------------------------------------*
 * ��������                                     *
 *----------------------------------------------*/
const uint8_t Display_Table[] = 
{
	MI_00,MI_01,MI_02,MI_03,MI_04,MI_05,MI_06,MI_07,MI_08,MI_09
};
/*----------------------------------------------*
 * �궨��                                       *
 *----------------------------------------------*/

/*****************************************************************************
 �� �� ��  : Key_Scan
 ��������  : ����ɨ��,20ģʽִ��һ��
 �������  : void  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2016��4��8��
    ��    ��   : ���ͼ
    �޸�����   : �����ɺ���

*****************************************************************************/
void Key_Scan(void)
{

	//�ɼ�����Ч�İ���ֵ
	if(!KeyScanEn)
	 return;
	KeyScanEn = DISABLE;
    
    //û�г�ʼ�����ǰ�������а����ɼ��봦��
    if(!GyroReady)
        return;

    // PD8��ʱ����VL53L0x���ж�������ʹ�ã�����1����
//	if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_8))
//	{
//		Key_Code &= ~0x01; 
//	}
//	else
//	{
//		Key_Code |= 0x01;
//	}

	if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_9))
	{
		Key_Code &= ~0x02;
	}
	else
	{
		Key_Code |= 0x02;
	}

	if(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_10))
	{
		Key_Code &= ~0x04;
	}
	else
	{
		Key_Code |= 0x04;
	}
	
	if(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_8))
	{
		Key_Code &= ~0x08;
	}
	else
	{
		Key_Code |= 0x08;
	}
    
    

	g_key_data = Key_Code;
	//����״̬�ж��봦�� 
	switch(Key_Status)
	{
	 case 0: //�޼�����
		 g_key_Continuum_con = 0;
		 g_key_Continuum_tim = 0;
		 Flag_Key_LongPress = RESET;
		 if((g_key_data > 0)&&(g_key_data_Pre == 0))
		 {
			 //����������
			 Key_Status = 1;
		 }
         
         //����WIFI
         FlagResetWifi = RESET;
		 //��ֵ����
		 //g_key_data_Pre = g_key_data;
		 break;
	 case 1:
		 if(g_key_data & g_key_data_Pre)
		 {
			 Key_Status = 2;
			 g_key_Continuum_con++;
		 }
		 else
		 {
			 Key_Status = 0;
		 }
		 break;
	 case 2:
		 if((g_key_data == 0)&&(g_key_data_Pre > 0))
		 {
			 Key_Status = 0;
			 Key_Deal();
		 }
		 else if(g_key_data && g_key_data_Pre) //��������
		 {
			 g_key_Continuum_con++;
			 if(g_key_Continuum_con >= 40)
			 {
				 Flag_Key_LongPress = SET;
				 g_key_Continuum_tim++;
				 if(g_key_Continuum_tim > 9)
					 g_key_Continuum_tim = 9;	 
				 g_key_Continuum_con = 30 + g_key_Continuum_tim;
				 Key_Deal();
			 }
		 }
		 break;
	 default:
		 Key_Status = 0;
		 break;
	}
	//��ֵ����
	g_key_data_Pre = g_key_data;
 
}

void Key_Deal(void)
{
//    static uint16_t setspeed = 80;
	switch(g_key_data_Pre)
	{
        // �س䰴ť
		case 0x01:
			//
			RemoteSet(sig_to_charge);
//			if(!MopPar.MopEn)
//			{
//				PathPlanStep = 0;
//				UltraFrontDisMin = 0xffff;
//				EndPosGet = DISABLE;
//                
//                //�ϵؿ���
//                MopPar.MopEn = ENABLE;
//                
//                //��ɨ����
//                CleanEn = ENABLE;
//                
//                //��������
//                DustEn = ENABLE;
//                
//                VoiceReq = 1;
//			}
//			else
//			{
//				PathPlanMode = 0;
//				PathPlanStep = 0;
//				SetRobitSpeed(0, 0);
//				
//				//�ر���ɨ
//				CleanEn = DISABLE;
//				
//				//�ر�����
//				DustEn = DISABLE;
//				
//				//�ر��ϵ�
//				RagChangeEn = DISABLE;

//				
//				//�رյ�ˮ
//				SprayWaterEn = DISABLE;
//                
//                //�ر��ϵ�
//                MopPar.MopEn = DISABLE;
//                
//                VoiceReq = 2;   
//			}
            			

			break;
        //��Դ��ť
		case 0x02:
			//
//			LED3_SW;
        
            if(Flag_Key_LongPress)
            {
                if(0 == SystePwrMode)
                {
                    Single_Write_I2C(3, 0x00, 0x80);
                    GPIO_Conf_StopMode();
                    EXIT_Conf_StopMode();
                    ADC_Cmd(ADC1, DISABLE);
                    RCC_APB1PeriphClockCmd(0xffffffff , DISABLE);
                    RCC_APB2PeriphClockCmd(0xffffffff & (~RCC_APB2Periph_GPIOE) & (~RCC_APB2Periph_AFIO), DISABLE);
                    PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);                
                    SystePwrMode = 1;
                }
                
                if(SystePwrMode)
                {
                    __set_FAULTMASK(1);// ??????
                    NVIC_SystemReset();// ??
                    Device_Init();
                }
            }
            else if(GyroReady)
            {
                RemoteSet(sig_to_clean_or_pause);
//                RemoteSet(sig_reset_wifi);
            }
//			if(0 == PathPlanMode)
//			{
//				PathPlanMode = 4;
//				PathPlanStep = 0;
//				BumpCnt = 0;
//				SysInitAngle = Sys_Angle;
//                
//                //�ϵؿ���
//                MopPar.MopEn = ENABLE;
//                
//                //��ɨ����
//                CleanEn = ENABLE;
//                
//                //��������
//                DustEn = ENABLE;
//                
//			}
//			else
//			{
//				PathPlanMode = 0;
//				PathPlanStep = 0;
//				BumpCnt = 0;
//				SetRobitSpeed(0, 0);
//                
//                //�ر���ɨ
//				CleanEn = DISABLE;
//				
//				//�ر�����
//				DustEn = DISABLE;
//                
//                //�ϵؿ���
//                MopPar.MopEn = DISABLE;
//			}
//			VoiceincreasVolum();
//            SetRobitMileageRun(3000, 1, setspeed);
//            

			break;
        // ������ɨ��ť
		case 0x04:
			//
			//LED3_SW;
        
            RemoteSet(sig_spot);
//			if(PathPlanMode == 0)
//			{
//				PathPlanMode = 3;
//				PathPlanStep = 0;
////				UltraFrontDisMin = 0xffff;
//			}
//			else
//			{
//				PathPlanMode = 0;
//				PathPlanStep = 0;
//				SetRobitSpeed(0, 0);
//			}
			break;
        // ����ͻس�һ����
        case 0x05:
            if(Flag_Key_LongPress && !FlagResetWifi)
            {
                //FlagResetWifi ��־���ڷ�ֹ�����󣬶�ν�������WIFI���ã��˱�־�ް�������ʱ����
                FlagResetWifi = SET;
                RemoteSet(sig_reset_wifi);

            }
            break;
		case 0x08:
			//
//			if(PathPlanMode == 0)
//			{
//				PathPlanMode = 3;
//				PathPlanStep = 0;
//				UltraFrontDisMin = 0xffff;
//				BumpCnt = 0;
//				SysInitAngle = Sys_Angle;
//				EndPosGet = DISABLE;
//			}
//			else
//			{
//				PathPlanMode = 0;
//				PathPlanStep = 0;
//				BumpCnt = 0;
//				SetRobitSpeed(0, 0);				
//			}
//            SetRobitVacc(0);
//            SetRobitAngleRun(180, 1, 15, 0);
//            SetRobitMileageRun(4000, 1, 160);
//            RemoteSet(sig_to_clean_or_pause);
//            MCU_Check_CheanPackStatus(CHECK_WIFI_STATUS);
//            MCU_Send_Charge_Status();
//        
            
            if(Flag_Key_LongPress)
            {
                if(0 == SystePwrMode)
                {
                    Single_Write_I2C(3, 0x00, 0x80);
                    GPIO_Conf_StopMode();
                    EXIT_Conf_StopMode();
                    ADC_Cmd(ADC1, DISABLE);
                    RCC_APB1PeriphClockCmd(0xffffffff , DISABLE);
                    RCC_APB2PeriphClockCmd(0xffffffff & (~RCC_APB2Periph_GPIOE) & (~RCC_APB2Periph_AFIO), DISABLE);
                    PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);                
                    SystePwrMode = 1;
                }
                
                //
                if(SystePwrMode)
                {
                    __set_FAULTMASK(1);// ??????
                    NVIC_SystemReset();// ??
                    Device_Init();
                }
            }
            else if(GyroReady)
            {
//                RemoteSet(sig_to_clean_or_pause);
//                RemoteSet(sig_reset_wifi);
                if(PathPlanMode == 0)
                {
                    PathPlanMode = 3;
                    PathPlanStep = 5;
                    SubStep = 0;
                    UltraFrontDisMin = 0xffff;
                    BumpCnt = 0;
                    SysInitAngle = Sys_Angle;
                    EndPosGet = DISABLE;
                    
                    //�ϵؿ���
                    MopPar.MopEn = ENABLE;
                    
                    //��¼��ʼ����ʱ��x��Ƕ�
                    Sys_AnglexInit = Sys_Angle2;
                }
                else
                {
                    PathPlanMode = 0;
                    PathPlanStep = 0;
                    BumpCnt = 0;
                    SetRobitSpeed(0, 0);	

                    //�ر��ϵ�
                    MopPar.MopEn = DISABLE;                    
                }
//                if(MopPar.MopEn)
//                {
//                    PathPlanStep = 0;
//                    UltraFrontDisMin = 0xffff;
//                    EndPosGet = DISABLE;
//                    
//                    //�ϵؿ���
//                    MopPar.MopEn = ENABLE;
//                    
//                    //��ɨ����
//                    CleanEn = ENABLE;
//                    
//                    //��������
//                    DustEn = ENABLE;
//                    
//                    VoiceReq = 1;
//                }
//                else
//                {
//                    PathPlanMode = 0;
//                    PathPlanStep = 0;
//                    SetRobitSpeed(0, 0);
//                    
//                    //�ر���ɨ
//                    CleanEn = DISABLE;
//                    
//                    //�ر�����
//                    DustEn = DISABLE;
//                    
//                    //�ر��ϵ�
//                    RagChangeEn = DISABLE;

//                    
//                    //�رյ�ˮ
//                    SprayWaterEn = DISABLE;
//                    
//                    //�ر��ϵ�
//                    MopPar.MopEn = DISABLE;
//                    
//                    VoiceReq = 2;   
//                }
            }
		default:
			break;
				
	}
}

//#if 0
/*****************************************************************************
 �� �� ��  : Send_8bit_Data
 ��������  : ��TM1628����8λ����,�ӵ�λ��ʼ
 �������  : uchar data_buf  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2015��3��31��
    ��    ��   : ���ͼ
    �޸�����   : �����ɺ���

*****************************************************************************/
void Send_8bit_Data(uint8_t data_buf)
{
	uint8_t i;
	for(i=0;i<8;i++)						//����һ�ֽڵ�8λ����
	{
		if(data_buf&0x01)					//�жϸ�λ�ĸߵ�
		{
			b_1628_Data_High();				//����λ�������ݶ˿�
		}
		else
		{
			b_1628_Data_Low();
		}
		b_1628_Clk_Low();					//�������ݳ�ȥ�ĵ�ƽ�仯ʱ�͵�ƽ���ߵ�ƽ��ʽ
		Delay_nop() ;
		Delay_nop() ;
		Delay_nop() ;
		Delay_nop() ;
		Delay_nop() ;
		Delay_nop() ;
		b_1628_Clk_High();
		data_buf = data_buf>>1;				//��������λ�����ݵĴ��䷽ʽ���ȷ��͵�λ��ʼ�����η���8λ����
	}
}

/*****************************************************************************
 �� �� ��  : Command
 ��������  : ��TM1628��������
 �������  : uchar command  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2015��3��31��
    ��    ��   : ���ͼ
    �޸�����   : �����ɺ���

*****************************************************************************/
void Command(uint8_t command)
{
	b_1628_Stb_High();						//����1628��������Ϊ�Ӹߵ�ƽת��Ϊ�͵�ƽ��ʽ
	b_1628_Stb_Low();
	Send_8bit_Data(command);				//�������ݷ��ͳ���
}

/*****************************************************************************
 �� �� ��  : HextoBcd
 ��������  : ת����Ҫ��ʾ����
 �������  : uint count  
             uhar zero   
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2015��3��31��
    ��    ��   : ���ͼ
    �޸�����   : �����ɺ���

*****************************************************************************/
void HextoBcd ( uint16_t count, uint8_t zero )
{ 
  	uint16_t i,k; 

	if(count == 0 && zero == 0)
	{
		g_display_buf[0] = 0;
		g_display_buf[1] = 0;
		g_display_buf[2] = 0;
		g_display_buf[3] = 0;
		g_display_buf[4] = Display_Table[0];
		return;
	}
		 
  	for ( i=4; i<10; i-- )
  	{
	  	k = count % 10 ;
        g_display_buf[i] = Display_Table[k];
	  	count /= 10;
	}
	if(!zero)
	{
  		
		for ( i=0; i<5; i++ ) 
  		{
	  		if ( g_display_buf[i] == MI_00 )
            {
                g_display_buf[i] = 0;
            }
	  		else
            {
                break;
            }
		}
	}

}

//void SendBit_1621(uint8_t data,uint8_t cnt) //data ��cnt д��HT1621,��λ��ǰ
//{
//	uint8_t i;
//	for(i = 0; i < cnt; i ++)
//	{
//		if((data & 0x80)==0) 
//			DATA_L;
//		else 
//			DATA_H;
//		WR_L;
//		Delay_nop();
//		WR_H;
//		data<<=1;
//	}
//}
//void SendDataBit_1621(uint8_t data,uint8_t cnt) //data ��cnt д��HT1621,��λ��ǰ
//{
//	uint8_t i;
//	for(i = 0; i < cnt; i ++)
//	{
//		if((data & 0x01)==0) 
//			DATA_L;
//		else 
//			DATA_H;
//		WR_L;
//		Delay_nop();
//		WR_H;
//		data>>=1;
//	}
//}

//void SendCmd(uint8_t command)
//{
//	CS_L;
//	SendBit_1621(0x80,3); //д���־��"100"
//	SendBit_1621(command,9); //д��9 λ����,ǰ8 λΪcommand ����,���һλ���
//	CS_H;
//}
////��������д�룬�̶���ַ����4λ����
//void Write_1621(uint8_t addr,uint8_t data)
//{
//	CS_L;
//	SendBit_1621(0xa0,3); //д��"101"
//	SendBit_1621(addr<<2,6); //д��6 λaddr
//	SendDataBit_1621(data,4); //д��data �ĵ�4 λ
//	CS_H;
//}
////��ʼ��ַ������д��cntλ����
//void WriteAll_1621(uint8_t addr,uint8_t *p,uint8_t cnt)
//{
//	uint8_t i;
//	CS_L;
//	SendBit_1621(0xa0,3); //д��"101"
//	SendBit_1621(addr<<2,6); //д��6 λaddr
//	for(i =0; i <cnt; i ++,p++) //����д������
//	{
//		SendDataBit_1621(*p,8);
//	}
//	CS_H;
//}

void Display_Conf(void)
{
//	LCDPOWERON;
//	LCDBGON;
//	SendCmd(BIAS); //????????
//	SendCmd(SYSEN); //???????
//	SendCmd(LCDON); //??LCD ?????
//	b_1628_Power_On();
}

void DisplayDataReady(void)
{
	uint16_t DisplayDataBuf;
	
	DisplayDataBuf = 0;//(uint16_t)(Kp_Set);//(ADC_ConvertedValue_Val[3]);//Sys_Angle / 100;//ADC_ConvertedValue_Val[3];//
	
//	Display_Buf[5] = (WallData[0].OK) | (WallData[1].OK << 1) | (WallData[2].OK << 2) | (WallData[3].OK << 3) | WallData[4].OK << 4 | (RightBump << 5) | LeftBump << 6;
//	
//	Display_Buf[6] = Display_Table[DisplayDataBuf%10];
//	Display_Buf[7] = Display_Table[DisplayDataBuf/10%10];
//	Display_Buf[8] = Display_Table[DisplayDataBuf/100%10];
//	Display_Buf[9] = Display_Table[DisplayDataBuf/1000];
	
	HextoBcd(DisplayDataBuf,0);
	
//	if(BatterySoc >= 100)
//	{
//		Display_Buf[2] = 0x17;
//	}
//	else if(BatterySoc >= 70)
//	{	
//		Display_Buf[2] = 0x16;
//	}
//	else if(BatterySoc >= 20)
//	{
//		Display_Buf[2] = 0x14;
//	}
//	else 
//	{	
//		Display_Buf[2] = 0x10;
//	}
}
//#endif


// 50ms ����һ��
void Display(void)
{
    static uint16_t ledpwm = 300, dir = 1;
	if(!DisplayScanEn)
		return;
	DisplayScanEn = DISABLE;
    

//    LED1_SW;
//    LED2(0);
//    LED3(0);
    if(GyroReady)
    {
//        LED1(1);
        TIM_SetCompare1(TIM1,TIM3_Period);
    }
    else
    {	
        TIM_SetCompare1(TIM1, ledpwm);
        
        if(0 == dir)
        {
            if(ledpwm > 1000)
            {
                ledpwm -= 250;
            }
            else
            {
                dir = 1;
            }
        }
        else
        {
            if(ledpwm < 6000)
            {
                ledpwm += 250;
            }
            else
            {
                dir = 0;
            }
        }
//		LED1(0);

    }
    
    if(RightBump)//LeftBump)//BATTERY_CHARGEING_STA)//UltraObs)//
    {
        LED2(1);
    }
    else
    {
        LED2(0);
    }
    
    if(LeftBump)//RagChangeEn)//BATTERY_COMPLET_STA)//
    {
        LED3(1);
    }
    else
    {
        LED3(0);
    }
    
//    LED1(1);
//    LED2(1);
//	LED3(1);
}

