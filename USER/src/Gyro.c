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
uint16_t Sys_Angle,Sys_Anglerate;
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
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource8);
	EXTI_ClearITPendingBit(EXTI_Line8);

	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_Line = EXTI_Line8;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	GPIO_WriteBit(GPIO_SDA,GPIO_Pin_Sda,Bit_SET);
	GPIO_WriteBit(GPIO_SCL,GPIO_Pin_Scl,Bit_SET);
    Sys_Angle = 0;
    Sys_Anglerate = 0;
    GyroInitEn = DISABLE;
}

void IIC_WriteData(uint8_t WriterData)
{
	uint8_t i,tmpdata;

	tmpdata = WriterData;
	
    Delay_nop();Delay_nop();
    Delay_nop();Delay_nop();
	SDA_Out();
	
	for(i=0;i<8;i++)
	{
		Delay_nop();Delay_nop();
		Delay_nop();Delay_nop();
		Delay_nop();Delay_nop();
		Delay_nop();Delay_nop();
		GPIO_WriteBit(GPIO_SCL,GPIO_Pin_Scl,Bit_RESET);
		Delay_nop();Delay_nop();
		Delay_nop();Delay_nop();
		Delay_nop();Delay_nop();
		
		if(tmpdata & 0x80)
		{
			GPIO_WriteBit(GPIO_SDA,GPIO_Pin_Sda,Bit_SET);
		}
		else
		{
			GPIO_WriteBit(GPIO_SDA,GPIO_Pin_Sda,Bit_RESET);
		}
		
		Delay_nop();Delay_nop();
		Delay_nop();Delay_nop();
		Delay_nop();Delay_nop();
		Delay_nop();Delay_nop();
		Delay_nop();Delay_nop();
		Delay_nop();Delay_nop();
			
		GPIO_WriteBit(GPIO_SCL,GPIO_Pin_Scl,Bit_SET);
		
		Delay_nop();Delay_nop();
		

		tmpdata = tmpdata << 1;
	}
}

void IIC_Start(void)
{
	SDA_Out();
	GPIO_WriteBit(GPIO_SDA,GPIO_Pin_Sda,Bit_RESET);
	Delay_nop();Delay_nop();
	Delay_nop();Delay_nop();
	Delay_nop();Delay_nop();
	GPIO_WriteBit(GPIO_SCL,GPIO_Pin_Scl,Bit_RESET);
}

void IIC_End(void)
{
	SDA_Out();
	Delay_nop();Delay_nop();
	Delay_nop();Delay_nop();
	Delay_nop();Delay_nop();
	GPIO_WriteBit(GPIO_SCL,GPIO_Pin_Scl,Bit_SET);
	Delay_nop();Delay_nop();
	Delay_nop();Delay_nop();
	Delay_nop();Delay_nop();
	GPIO_WriteBit(GPIO_SDA,GPIO_Pin_Sda,Bit_SET);
}

void IIC_Ack(void)
{	
	GPIO_WriteBit(GPIO_SCL,GPIO_Pin_Scl,Bit_RESET);
	Delay_nop();Delay_nop();
    Delay_nop();Delay_nop();
    Delay_nop();Delay_nop();
    SDA_Out();
	GPIO_WriteBit(GPIO_SDA,GPIO_Pin_Sda,Bit_RESET);
	Delay_nop();Delay_nop();
	Delay_nop();Delay_nop();
	Delay_nop();Delay_nop();
	Delay_nop();Delay_nop();
	Delay_nop();Delay_nop();
	Delay_nop();Delay_nop();
	Delay_nop();Delay_nop();
	Delay_nop();Delay_nop();
	Delay_nop();Delay_nop();
	Delay_nop();Delay_nop();
	Delay_nop();Delay_nop();
	Delay_nop();Delay_nop();
	Delay_nop();Delay_nop();
	Delay_nop();Delay_nop();
	GPIO_WriteBit(GPIO_SCL,GPIO_Pin_Scl,Bit_SET);
	Delay_nop();Delay_nop();
	Delay_nop();Delay_nop();
	Delay_nop();Delay_nop();
	Delay_nop();Delay_nop();
	Delay_nop();Delay_nop();
    GPIO_WriteBit(GPIO_SCL,GPIO_Pin_Scl,Bit_RESET);
}


uint8_t IIC_ReadAck(void)
{
	
    
	GPIO_WriteBit(GPIO_SCL,GPIO_Pin_Scl,Bit_RESET);
	Delay_nop();Delay_nop();Delay_nop();Delay_nop();
	Delay_nop();Delay_nop();Delay_nop();Delay_nop();
    SDA_In();
    Delay_nop();Delay_nop();Delay_nop();Delay_nop();	
	Delay_nop();Delay_nop();Delay_nop();Delay_nop();
	Delay_nop();Delay_nop();Delay_nop();Delay_nop();
	Delay_nop();Delay_nop();Delay_nop();Delay_nop();
	Delay_nop();Delay_nop();Delay_nop();Delay_nop();
	Delay_nop();Delay_nop();Delay_nop();Delay_nop();
    
	GPIO_WriteBit(GPIO_SCL,GPIO_Pin_Scl,Bit_SET);
    
    Delay_nop();Delay_nop();
	Delay_nop();Delay_nop();
    Delay_nop();Delay_nop();
	Delay_nop();Delay_nop();

    Delay_nop();Delay_nop();
	Delay_nop();Delay_nop();


//	if(GPIO_ReadInputDataBit(GPIO_SDA,GPIO_Pin_Sda) == RESET)
//	{
//		GPIO_WriteBit(GPIO_SCL,GPIO_Pin_Scl,Bit_RESET);
//        return 1;
//	}
//	else
//	{
//        GPIO_WriteBit(GPIO_SCL,GPIO_Pin_Scl,Bit_RESET);
//        return 0;
//	}
    GPIO_WriteBit(GPIO_SCL,GPIO_Pin_Scl,Bit_RESET);
    return 1;
//    while(GPIO_ReadInputDataBit(GPIO_SDA,GPIO_Pin_Sda) != Bit_SET);
//    GPIO_WriteBit(GPIO_SCL,GPIO_Pin_Scl,Bit_RESET);
}

uint8_t IIC_ReadData(void)
{
	uint8_t Revdata,i;

	SDA_In();

	for(i=0;i<8;i++)
	{
		Revdata <<= 1;
		GPIO_WriteBit(GPIO_SCL,GPIO_Pin_Scl,Bit_RESET);
		Delay_nop();Delay_nop();
		Delay_nop();Delay_nop();
		Delay_nop();Delay_nop();
		Delay_nop();
		Delay_nop();Delay_nop();
		Delay_nop();Delay_nop();
		Delay_nop();Delay_nop();
		Delay_nop();
		Delay_nop();Delay_nop();
		Delay_nop();Delay_nop();
		Delay_nop();Delay_nop();
		Delay_nop();Delay_nop();
		Delay_nop();Delay_nop();
		Delay_nop();
		Delay_nop();
		GPIO_WriteBit(GPIO_SCL,GPIO_Pin_Scl,Bit_SET);
		Delay_nop();
		Delay_nop();
		
		if(GPIO_ReadInputDataBit(GPIO_SDA,GPIO_Pin_Sda) == SET)
		{
			Revdata |= 0x01;
		}
		else
		{
			Revdata &= ~0x01;
		}
			
		
	}

	return Revdata;
	
}

/*****************************************************************************
 �� �� ��  : EXTI9_5_IRQHandler
 ��������  : �����ֲ����ж�
 �������  : void  
 �������  : ��
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2016��3��8��
    ��    ��   : ���ͼ
    �޸�����   : �����ɺ���

*****************************************************************************/
void EXTI9_5_IRQHandler(void)
{
    static uint8_t initcnt = 0;
    uint8_t readdata[4];
    
	if ( EXTI_GetITStatus(EXTI_Line5) != RESET )
	{
		EXTI_ClearITPendingBit(EXTI_Line5);
		
	}

	if(EXTI_GetITStatus(EXTI_Line8) != RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line8);
        
        if(GyroInitEn)
        {
            IIC_Start();
            IIC_WriteData(0x6B);
            IIC_ReadAck();
            
            readdata[0] = IIC_ReadData();          
            IIC_Ack();
            readdata[1] = IIC_ReadData();
            IIC_Ack();
            readdata[2] = IIC_ReadData();
            IIC_Ack();
            readdata[3] = IIC_ReadData();
            IIC_Ack();
            
            IIC_End();
            
            IIcRevData[0] = readdata[0];
            IIcRevData[1] = readdata[1];
            IIcRevData[2] = readdata[2];
            IIcRevData[3] = readdata[3];
            //Sys_Angle = ((uint16_t)(readdata[1]) << 8) + readdata[0];
            //Sys_Anglerate = ((uint16_t)(readdata[3]) << 8) + readdata[2];
        }
        else
        {
            IIC_Start();
            IIC_WriteData(0x6A);
            IIC_ReadAck();
            IIC_WriteData(0x00);
            IIC_ReadAck();
            IIC_WriteData(0x00);
            IIC_ReadAck();
            IIC_WriteData(0x00);
            IIC_ReadAck();
            IIC_End();
            if(++initcnt >= 2)
                GyroInitEn = ENABLE;
        }
	}
}


