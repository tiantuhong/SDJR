/******************************************************************************

                  版权所有 (C), 1994-2015, 杭州九阳欧南多小家电有限公司

 ******************************************************************************
  文 件 名   : Display_Key.c
  版 本 号   : V1.0
  作    者   : 田宏图
  生成日期   : 2016年4月8日
  最近修改   :
  功能描述   : LED显示和按键处理
  函数列表   :
  修改历史   :
  1.日    期   : 2016年4月8日
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
void Key_Scan(void);

/*----------------------------------------------*
 * 内部函数原型说明                             *
 *----------------------------------------------*/
void Key_Deal(void);

/*----------------------------------------------*
 * 全局变量                                     *
 *----------------------------------------------*/
FunctionalState KeyScanEn;
FunctionalState DisplayScanEn;
FlagStatus Flag_Key_LongPress, FlagResetWifi;

/*----------------------------------------------*
 * 模块级变量                                   *
 *----------------------------------------------*/
uint8_t Key_Code = 0,g_key_data,g_key_data_Pre,Key_Status = 0,g_key_Continuum_con = 0,g_key_Continuum_tim;
uint8_t DisCnt = 0;

uint8_t g_display_buf[10] = 
{
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00
};

/*----------------------------------------------*
 * 常量定义                                     *
 *----------------------------------------------*/
const uint8_t Display_Table[] = 
{
	MI_00,MI_01,MI_02,MI_03,MI_04,MI_05,MI_06,MI_07,MI_08,MI_09
};
/*----------------------------------------------*
 * 宏定义                                       *
 *----------------------------------------------*/

/*****************************************************************************
 函 数 名  : Key_Scan
 功能描述  : 按键扫描,20模式执行一次
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2016年4月8日
    作    者   : 田宏图
    修改内容   : 新生成函数

*****************************************************************************/
void Key_Scan(void)
{

	//采集到有效的按键值
	if(!KeyScanEn)
	 return;
	KeyScanEn = DISABLE;
    
    //没有初始化完成前，不进行按键采集与处理
    if(!GyroReady)
        return;

    // PD8暂时用于VL53L0x的中断做调试使用，按键1屏蔽
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
	//按键状态判断与处理 
	switch(Key_Status)
	{
	 case 0: //无键按下
		 g_key_Continuum_con = 0;
		 g_key_Continuum_tim = 0;
		 Flag_Key_LongPress = RESET;
		 if((g_key_data > 0)&&(g_key_data_Pre == 0))
		 {
			 //按键被按下
			 Key_Status = 1;
		 }
         
         //重置WIFI
         FlagResetWifi = RESET;
		 //键值更新
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
		 else if(g_key_data && g_key_data_Pre) //长按按键
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
	//键值更新
	g_key_data_Pre = g_key_data;
 
}

void Key_Deal(void)
{
//    static uint16_t setspeed = 80;
	switch(g_key_data_Pre)
	{
        // 回充按钮
		case 0x01:
			//
			RemoteSet(sig_to_charge);
//			if(!MopPar.MopEn)
//			{
//				PathPlanStep = 0;
//				UltraFrontDisMin = 0xffff;
//				EndPosGet = DISABLE;
//                
//                //拖地开启
//                MopPar.MopEn = ENABLE;
//                
//                //清扫开启
//                CleanEn = ENABLE;
//                
//                //吸尘开启
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
//				//关闭清扫
//				CleanEn = DISABLE;
//				
//				//关闭吸尘
//				DustEn = DISABLE;
//				
//				//关闭拖地
//				RagChangeEn = DISABLE;

//				
//				//关闭滴水
//				SprayWaterEn = DISABLE;
//                
//                //关闭拖地
//                MopPar.MopEn = DISABLE;
//                
//                VoiceReq = 2;   
//			}
            			

			break;
        //电源按钮
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
//                //拖地开启
//                MopPar.MopEn = ENABLE;
//                
//                //清扫开启
//                CleanEn = ENABLE;
//                
//                //吸尘开启
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
//                //关闭清扫
//				CleanEn = DISABLE;
//				
//				//关闭吸尘
//				DustEn = DISABLE;
//                
//                //拖地开启
//                MopPar.MopEn = DISABLE;
//			}
//			VoiceincreasVolum();
//            SetRobitMileageRun(3000, 1, setspeed);
//            

			break;
        // 定点清扫按钮
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
        // 定点和回充一起按下
        case 0x05:
            if(Flag_Key_LongPress && !FlagResetWifi)
            {
                //FlagResetWifi 标志用于防止长按后，多次进入重置WIFI设置，此标志无按键按下时清零
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
                    
                    //拖地开启
                    MopPar.MopEn = ENABLE;
                    
                    //记录开始工作时的x轴角度
                    Sys_AnglexInit = Sys_Angle2;
                }
                else
                {
                    PathPlanMode = 0;
                    PathPlanStep = 0;
                    BumpCnt = 0;
                    SetRobitSpeed(0, 0);	

                    //关闭拖地
                    MopPar.MopEn = DISABLE;                    
                }
//                if(MopPar.MopEn)
//                {
//                    PathPlanStep = 0;
//                    UltraFrontDisMin = 0xffff;
//                    EndPosGet = DISABLE;
//                    
//                    //拖地开启
//                    MopPar.MopEn = ENABLE;
//                    
//                    //清扫开启
//                    CleanEn = ENABLE;
//                    
//                    //吸尘开启
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
//                    //关闭清扫
//                    CleanEn = DISABLE;
//                    
//                    //关闭吸尘
//                    DustEn = DISABLE;
//                    
//                    //关闭拖地
//                    RagChangeEn = DISABLE;

//                    
//                    //关闭滴水
//                    SprayWaterEn = DISABLE;
//                    
//                    //关闭拖地
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
 函 数 名  : Send_8bit_Data
 功能描述  : 向TM1628发送8位数据,从低位开始
 输入参数  : uchar data_buf  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2015年3月31日
    作    者   : 田宏图
    修改内容   : 新生成函数

*****************************************************************************/
void Send_8bit_Data(uint8_t data_buf)
{
	uint8_t i;
	for(i=0;i<8;i++)						//发送一字节的8位数据
	{
		if(data_buf&0x01)					//判断该位的高低
		{
			b_1628_Data_High();				//将该位给到数据端口
		}
		else
		{
			b_1628_Data_Low();
		}
		b_1628_Clk_Low();					//发送数据出去的电平变化时低电平到高电平方式
		Delay_nop() ;
		Delay_nop() ;
		Delay_nop() ;
		Delay_nop() ;
		Delay_nop() ;
		Delay_nop() ;
		b_1628_Clk_High();
		data_buf = data_buf>>1;				//再向右移位，数据的传输方式是先发送低位开始，依次发完8位数据
	}
}

/*****************************************************************************
 函 数 名  : Command
 功能描述  : 向TM1628发送命令
 输入参数  : uchar command  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2015年3月31日
    作    者   : 田宏图
    修改内容   : 新生成函数

*****************************************************************************/
void Command(uint8_t command)
{
	b_1628_Stb_High();						//发送1628驱动命令为从高电平转变为低电平方式
	b_1628_Stb_Low();
	Send_8bit_Data(command);				//调用数据发送程序
}

/*****************************************************************************
 函 数 名  : HextoBcd
 功能描述  : 转换成要显示的码
 输入参数  : uint count  
             uhar zero   
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2015年3月31日
    作    者   : 田宏图
    修改内容   : 新生成函数

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

//void SendBit_1621(uint8_t data,uint8_t cnt) //data 高cnt 写入HT1621,高位在前
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
//void SendDataBit_1621(uint8_t data,uint8_t cnt) //data 低cnt 写入HT1621,低位在前
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
//	SendBit_1621(0x80,3); //写入标志码"100"
//	SendBit_1621(command,9); //写入9 位数据,前8 位为command 数据,最后一位随机
//	CS_H;
//}
////单个数据写入，固定地址，低4位数据
//void Write_1621(uint8_t addr,uint8_t data)
//{
//	CS_L;
//	SendBit_1621(0xa0,3); //写入"101"
//	SendBit_1621(addr<<2,6); //写入6 位addr
//	SendDataBit_1621(data,4); //写入data 的低4 位
//	CS_H;
//}
////起始地址，连续写入cnt位数据
//void WriteAll_1621(uint8_t addr,uint8_t *p,uint8_t cnt)
//{
//	uint8_t i;
//	CS_L;
//	SendBit_1621(0xa0,3); //写入"101"
//	SendBit_1621(addr<<2,6); //写入6 位addr
//	for(i =0; i <cnt; i ++,p++) //连续写入数据
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


// 50ms 调用一次
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

