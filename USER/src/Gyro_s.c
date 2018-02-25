/******************************************************************************

                  ��Ȩ���� (C), 1994-2015, ���ݾ���ŷ�϶�С�ҵ����޹�˾

 ******************************************************************************
  �� �� ��   : Gyro.c
  �� �� ��   : V1.0
  ��    ��   : ���ͼ
  ��������   : 2016��5��26��
  ����޸�   :
  ��������   : ���������ݶ�ȡ�����
  �����б�   :
  �޸���ʷ   :
  1.��    ��   : 2016��5��26��
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
int16_t Sys_Angle,Sys_Anglerate;
uint8_t IIcRevData[4];
/*----------------------------------------------*
 * ģ�鼶����                                   *
 *----------------------------------------------*/
FunctionalState GyroInitEn;
/*----------------------------------------------*
 * ��������                                     *
 *----------------------------------------------*/

/*----------------------------------------------*
 * �궨��                                       *
 *----------------------------------------------*/

/*****************************************************************************
 �� �� ��  : Gyro_Conf
 ��������  : ��������ģ��IICͨѶ��ʽ��������ģ��ӻ�ģʽ����ģ������ģʽ��ͨ
             �����ģ��ķ�ʽʵ��
 �������  : void  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2016��5��26��
    ��    ��   : ���ͼ
    �޸�����   : �����ɺ���

*****************************************************************************/
void Gyro_Conf(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
       
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOGYRO, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_Scl;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
  	GPIO_Init(GPIO_SCL, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_Int;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
  	GPIO_Init(GPIO_INT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_Sda;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
  	GPIO_Init(GPIO_SDA, &GPIO_InitStructure);

	// INT����Ϊ�ⲿ�ж� �½����ж�
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource12);
	EXTI_ClearITPendingBit(EXTI_Line12);

	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_Line = EXTI_Line12;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);  													
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;   
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;   
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;	      
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

//	GPIO_WriteBit(GPIO_SDA,GPIO_Pin_Sda,Bit_SET);
//	GPIO_WriteBit(GPIO_SCL,GPIO_Pin_Scl,Bit_SET);
    SCL_H;
    SDA_H;

    Sys_Angle = 0;
    Sys_Anglerate = 0;
    GyroInitEn = DISABLE;
    
    
}

/*******************************************************************************
* Function Name  : I2C_delay
* Description    : �ӳ�ʱ��
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
static void I2C_delay(uint8_t delaytime)
{	
   uint8_t i; /* ��������Ż��ٶ�,��������͵�5����д�� */
   i = delaytime;
   while(i) 
   { 
     i--; 
   } 
}

/*******************************************************************************
* Function Name  : I2C_Start
* Description    : None
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
static FunctionalState I2C_Start(void)
{
    SDA_Out();
	SDA_H;
	SCL_H;
	I2C_delay(LongTime);
	if(!SDA_read)return DISABLE;	/* SDA��Ϊ�͵�ƽ������æ,�˳� */
	SDA_L;
	I2C_delay(LongTime);
	if(SDA_read) return DISABLE;	/* SDA��Ϊ�ߵ�ƽ�����߳���,�˳� */
	SDA_L;
	I2C_delay(LongTime);
	return ENABLE;
}

/*******************************************************************************
* Function Name  : I2C_Stop
* Description    : None
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
static void I2C_Stop(void)
{
    SDA_Out();
	SCL_L;
	I2C_delay(LongTime);
	SDA_L;
	I2C_delay(LongTime);
	SCL_H;
	I2C_delay(LongTime);
	SDA_H;
	I2C_delay(LongTime);
}

/*******************************************************************************
* Function Name  : I2C_Ack
* Description    : None
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
static void I2C_Ack(void)
{
    SDA_Out();
	SCL_L;
	I2C_delay(LongTime);
	SDA_L;
	I2C_delay(LongTime);
	SCL_H;
	I2C_delay(LongTime);
	SCL_L;
	I2C_delay(LongTime);
}



/*******************************************************************************
* Function Name  : I2C_NoAck
* Description    : None
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
static void I2C_NoAck(void)
{
    SDA_Out();
	SCL_L;
	I2C_delay(LongTime);
	SDA_H;
	I2C_delay(LongTime);
	SCL_H;
	I2C_delay(LongTime);
	SCL_L;
	I2C_delay(LongTime);
}

/*******************************************************************************
* Function Name  : I2C_WaitAck
* Description    : None
* Input          : None
* Output         : None
* Return         : ����Ϊ:=1��ACK,=0��ACK
* Attention		 : None
*******************************************************************************/
static FunctionalState I2C_WaitAck(void) 	
{
	SDA_In(); 
	SCL_L;
    
	I2C_delay(LongTime);
//	SDA_H;	
       
	I2C_delay(LongTime);
	SCL_H;
	I2C_delay(ShortTime);
	if(SDA_read)
	{
        SCL_L;
        return DISABLE;
	}
	SCL_L;
	return ENABLE;
}
/*******************************************************************************
* Function Name  : I2C_SendByte
* Description    : ���ݴӸ�λ����λ
* Input          : - SendByte: ���͵�����
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
static void I2C_SendByte(uint8_t SendByte) 
{
    uint8_t i=8;
    
    SDA_Out();
    while(i--)
    {
        SCL_L;
        I2C_delay(LongTime);
        if(SendByte&0x80)
            SDA_H;  
        else 
            SDA_L;   
        SendByte<<=1;
        I2C_delay(MidTime);
		SCL_H;
        I2C_delay(MidTime);
    }
    SCL_L;
}

/*******************************************************************************
* Function Name  : I2C_ReceiveByte
* Description    : ���ݴӸ�λ����λ
* Input          : None
* Output         : None
* Return         : I2C���߷��ص�����
* Attention		 : None
*******************************************************************************/
static uint8_t I2C_ReceiveByte(void)  
{ 
    uint8_t i=8;
    uint8_t ReceiveByte=0;

//    SDA_H;	
    SDA_In();    
    while(i--)
    {
           
      SCL_L;
      I2C_delay(LongTime);
	  I2C_delay(LongTime);
		
	  ReceiveByte<<=1; 
	  
	  SCL_H;
		
      I2C_delay(ShortTime);
		
      if(SDA_read)
      {
        ReceiveByte|=0x01;
      }
	  else
	  {
		ReceiveByte&=~0x01;
	  }
    }
    SCL_L;
    return ReceiveByte;
}
/*******************************************************************************
* Function Name  : I2C_WriteByte
* Description    : дһ�ֽ�����
* Input          : - length: ��д�����ݳ���
*           	   - pBuffer: ��д�����ݵ�ַ
*                  - DeviceAddress: ��д���ַ
* Output         : None
* Return         : ����Ϊ:=1�ɹ�д��,=0ʧ��
* Attention		 : None
*******************************************************************************/           
FunctionalState I2C_WriteByte(uint8_t* pBuffer, uint8_t length,  uint8_t DeviceAddress)
{		
    if(!I2C_Start())
        return DISABLE;
    I2C_SendByte(((DeviceAddress) << 1) | 0x00); /* */
    if(!I2C_WaitAck())
    {
        I2C_Stop();
        return DISABLE;
    }
    while(length)
    {
        I2C_SendByte((uint8_t)(*pBuffer));       
        I2C_WaitAck();	
        length --;
        pBuffer ++;
    }        
    I2C_Stop(); 
    return ENABLE;
}	

/*******************************************************************************
* Function Name  : I2C_ReadByte
* Description    : ��ȡһ������
* Input          : - pBuffer: ��Ŷ�������
*           	   - length: ����������
*                  - ReadAddress: ��������ַ
* Output         : None
* Return         : ����Ϊ:=1�ɹ�����,=0ʧ��
* Attention		 : None
*******************************************************************************/          
FunctionalState I2C_ReadByte(uint8_t* pBuffer,   uint8_t length,   uint8_t ReadAddress)
{		
    if(!I2C_Start())
        return DISABLE;
    I2C_SendByte(((ReadAddress) << 1) | 0x01); /* ���ø���ʼ��ַ+������ַ */ 
    if(!I2C_WaitAck())
    {
        I2C_Stop();
        return DISABLE;
    }
//    I2C_SendByte((uint8_t)(ReadAddress & 0x00FF));   /* ���õ���ʼ��ַ */      
//    I2C_WaitAck();
//    I2C_Start();
//    I2C_SendByte(((ReadAddress & 0x0700) >>7) | DeviceAddress | 0x0001);
//    I2C_WaitAck();
    while(length)
    {
      *pBuffer = I2C_ReceiveByte();
      if(length == 1)I2C_NoAck();
      else I2C_Ack(); 
      pBuffer++;
      length--;
    }
    I2C_Stop();
    return ENABLE;
}
/*****************************************************************************
 �� �� ��  : EXTI15_10_IRQHandler
 ��������  : �������ж�
 �������  : void  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2016��3��8��
    ��    ��   : ���ͼ
    �޸�����   : �����ɺ���
  2.��    ��   : 2016��8��12��
    ��    ��   : ���ͼ
    �޸�����   : ���Ľ�λ

*****************************************************************************/
void EXTI15_10_IRQHandler(void)
{
    static uint8_t initcnt = 0;
    static int16_t Sys_AngleOld,Sys_AngleNew;
    int16_t Temp_Data;
//    uint8_t readdata[4];
    
//	if ( EXTI_GetITStatus(EXTI_Line5) != RESET )
//	{
//		EXTI_ClearITPendingBit(EXTI_Line5);
//		
//	}

	if(EXTI_GetITStatus(EXTI_Line12) != RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line12);
        
        if(GyroInitEn)
        {
            I2C_ReadByte(IIcRevData,4,0x35);
            Sys_AngleNew = (int16_t)(((uint16_t)(IIcRevData[0]) << 8) + IIcRevData[1]);
            
            if(Sys_AngleNew > Sys_AngleOld)
            {
                Temp_Data = Sys_AngleNew - Sys_AngleOld;
            }
            else
            {
                Temp_Data = Sys_AngleOld - Sys_AngleNew;
            }
            
            if(Temp_Data <= 200)
            {
                Sys_Angle = Sys_AngleNew;
                
            }
            Sys_AngleOld = Sys_AngleNew;
            
            Sys_Anglerate = (int16_t)(((uint16_t)(IIcRevData[2]) << 8) + IIcRevData[3]);
        }
        else
        {
//            I2C_WriteByte();
            if(++initcnt >= 2)
                GyroInitEn = ENABLE;
        }
	}
}


