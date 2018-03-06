/******************************************************************************

                  版权所有 (C), 1994-2016, 杭州九阳欧南多小家电有限公司

 ******************************************************************************
  文 件 名   : Gyro_I2C.c
  版 本 号   : V1.0
  作    者   : tht
  生成日期   : 2016年8月16日
  最近修改   :
  功能描述   : 陀螺仪数据读取，使用硬件I2C
  函数列表   :
  修改历史   :
  1.日    期   : 2016年8月16日
    作    者   : tht
    修改内容   : 创建文件
******************************************************************************/


/*----------------------------------------------*
 * 包含头文件                                   *
 *----------------------------------------------*/
#include "stm32f10x.h"
#include "kalman_filter.h"
#include "imu.h"
#include "IMUSO3.h"
#include <string.h>

#define I2C_DEVICE_NUM      6
/*----------------------------------------------*
 * 外部变量说明                                 *
 *----------------------------------------------*/
//typedef struct
//{
//    uint8_t I2C_SendBuf[10]; //发送缓存
//    
//    uint8_t I2C_SendBytes;      //发送字节数
//    
////    uint8_t I2C_SendPts;        //发送个数
//    
//    uint8_t I2C_Device;         //发送设备
//    
//    uint8_t I2C_Dir;            //发送方向
//    
//    uint8_t I2C_RevBytes;       //接收字节数
//    
//    uint8_t *RevData;           //接收数据
//    
//} I2CData_TypeDef;
/*----------------------------------------------*
 * 外部函数原型说明                             *
 *----------------------------------------------*/

/*----------------------------------------------*
 * 内部函数原型说明                             *
 *----------------------------------------------*/
void Single_Write_I2C(uint8_t Device_Num, uint8_t REG_Address,uint8_t REG_data);
void Write_I2C(uint8_t Device_Num, uint8_t * REG_Data, uint8_t REG_Len);
void Read_I2C(uint8_t Device_Num, uint8_t REG_Address, uint8_t REG_ReadBytes);
int64_t AngleFormatForIcm(int64_t _angle_);
/*----------------------------------------------*
 * 全局变量                                     *
 *----------------------------------------------*/
int16_t Sys_Angle,Sys_Anglerate, Sys_AnglePre, Sys_Angle2, Sys_AnglexInit;
uint8_t IIcRevData[20];
uint8_t IIcRevBytes;

uint8_t I2C_RevDataVL53L0X[10];
FunctionalState GyroReady,GyroDataVlid;
uint16_t Icm_Init_Cnt = 0;
uint8_t Battery_Charge_En, Gyro_Icm_DataReady;

/*----------------------------------------------*
 * 模块级变量                                   *
 *----------------------------------------------*/
FunctionalState GyroInitEn, ADXLInitEn, I2CFrameTrans, GyroIcmInit, BatteryChargeInit;

uint8_t /*I2C_SendBuf[10], I2C_SendBytes, */I2C_SendPts, I2C_ReqDevice[I2C_DEVICE_NUM], I2C_ReqDeviceLockedBak[I2C_DEVICE_NUM];/*, I2C_Dir, I2C_RevBytes*/;
I2CData_TypeDef I2C_Gyro, I2C_ADXL, I2C_Gyro_Icm, I2C_Charge, I2C_VL53L0X, * I2C_Core /*, * I2C_Lock*/;
uint8_t I2C_Status = 0,I2C_Req = 0, I2C_ReqLock = 0,I2C_ReqLocked = 0;// I2C状态 0-空闲 1-正在使用 ， I2C 请求 0-无请求 1-有请求
uint8_t AccBuf[6] = {0};
Gyro_6Axis_TypeDef IcmData, IcmDataByKalman;
kalman1_state GyroKalman[6];
/*----------------------------------------------*
 * 常量定义                                     *
 *----------------------------------------------*/

/*----------------------------------------------*
 * 宏定义                                       *
 *----------------------------------------------*/
#define I2C_DEVICE_ADXL345  1
#define I2C_DEVICE_GYRO     2
#define I2C_DEVICE_GYRO_ICM 3
#define I2C_DEVICE_CHARGE   4

#define I2C_DIR_WRITE       0
#define I2C_DIR_READ        1
#define I2C_DIR_RW          2
#define ADXL345_ADDR        0xA6

/*****************************************************************************
 函 数 名  : Gyro_Conf
 功能描述  : 与陀螺仪模块IIC通讯方式，陀螺仪模块从机模式，本模块主机模式，通
             过软件模拟的方式实现
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2016年5月26日
    作    者   : 田宏图
    修改内容   : 新生成函数

*****************************************************************************/
void Gyro_Conf(void)
{
//	GPIO_InitTypeDef GPIO_InitStructure;
//	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	I2C_InitTypeDef  I2C_InitStructure;
       
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2,ENABLE);
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOGYRO | RCC_APB2Periph_AFIO, ENABLE);


	I2C_DeInit(I2C2);
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = 0x30;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = 400000;  /* 400K?ù?è */
	I2C_Init(I2C2, &I2C_InitStructure);

	I2C_ITConfig(I2C2,I2C_IT_BUF | I2C_IT_EVT,ENABLE);
//	I2C_ClearFlag(I2C2,I2C_FLAG_TXE | I2C_FLAG_RXNE);
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  													
	NVIC_InitStructure.NVIC_IRQChannel = I2C2_EV_IRQn;   
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;   
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;	      
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	I2C_Cmd(I2C2, ENABLE);
	
	/*?êDí1×??ú1ó|′e?￡ê?*/
	I2C_AcknowledgeConfig(I2C2, ENABLE);
	

//	GPIO_WriteBit(GPIO_SDA,GPIO_Pin_Sda,Bit_SET);
//	GPIO_WriteBit(GPIO_SCL,GPIO_Pin_Scl,Bit_SET);
//    SCL_H;
//    SDA_H;

    Sys_Angle = 0;
    Sys_Anglerate = 0;
    GyroInitEn = DISABLE;
	IIcRevBytes = 0;
	GyroReady = DISABLE;
	GyroDataVlid = DISABLE;
    
    I2CFrameTrans = ENABLE;
    ADXLInitEn = DISABLE;
    GyroIcmInit = DISABLE;
    BatteryChargeInit = DISABLE;
    
    I2C_Status = 0;
    I2C_Req = 0;
    
    Battery_Charge_En = 0;
    Gyro_Icm_DataReady = 0;
}

#define CHARGE_DEVICE_ADDR  0x12

const uint8_t ChargeManageInitTable[] = 
{
    0x12,0x10,0x81,
    0x15,0xa0,0x41,         //充电电压16.8V
    0x14,0x00,0x04,         //充电电流1.024A
    0x3f,0x00,0x08,         //输入电流2.048
    0x3e,0x80,0x00,         //停止充电电流128mA
    0xff
};
void ChargeManageInit(void)
{
    static uint8_t ChargeInitTurn = 0;
    uint8_t * reg;
    
    //是否连接充电器
    if(!SocketPower)
    {
        BatteryChargeInit = DISABLE;
        ChargeInitTurn = 0;
        return;
    }
    //充电初始化已完成？
    if(BatteryChargeInit)
        return;
    
    //I2C是否空闲
    if(!I2CFrameTrans)
        return;
    
    if(I2C_Status)
        return;
    
    I2CFrameTrans = DISABLE;
    
    reg = (uint8_t *)ChargeManageInitTable;
    if(ChargeManageInitTable[ChargeInitTurn] != 0xff)
    {
        Write_I2C(I2C_DEVICE_CHARGE, reg + ChargeInitTurn, 3);
        ChargeInitTurn += 3;
    }
    else
    {
//        Read_I2C(I2C_DEVICE_CHARGE, 0xfe, 2);
        BatteryChargeInit = ENABLE;
        I2CFrameTrans = ENABLE;
    }
}



#define ICM_DEVICE_ADDR 0xD0
//#define ICM_DEVICE_ADDR 0xD1
#define ICM_DEVICE_ID   0xAE
#define PWR_MGMT_1  0x6b
#define PWR_MGMT_2  0x6c
#define GYRO_CONFIG 0x1b
#define ACCEL_CONFIG    0x1c
#define ACCEL_CONFIG2   0x1d
#define CONFIG      0x1a
#define SMPLRT_DIV  0x19
#define INT_PIN_CFG 0x37
#define INTENABLE   0x38
#define FIFO_EN     0x23
#define SMPLRT_DIV  0x19
#define WHO_AM_I    0x75
#define ACCEL_XOUT_H    0x3B    //x轴加速度的高位地址,读取数据的起始地址，每次读取14位，包含3轴加速度3轴角速度和温度

const uint8_t ICM_Init_Table[23] = 
{
    PWR_MGMT_1, 0x00,                      //唤醒
    GYRO_CONFIG, GYRO_CONGFIG_DATA,        //陀螺仪量程设置
    ACCEL_CONFIG,0x00,                     //加速度计量程设置
    CONFIG, 0x02,                          //设置采样率  100Hz
    ACCEL_CONFIG2, 0x02,                   //数据滤波器设置
    SMPLRT_DIV, 0x09,                      //10分频 采样率100Hz
    INT_PIN_CFG, 0x90,                     //设置中断
    INTENABLE, 0x01,                       //INT 电平设置
    FIFO_EN, 0x00,                         //关闭FIFO
    PWR_MGMT_1, 0x01,                      //设置时钟
    PWR_MGMT_2, 0x00,                      //启动陀螺仪和加速度计
    0xff                                   //结束标志
};

void ICM_Init(void)
{
    static uint8_t Icm_trans_turn = 0, Icm_Init_Sta = 0;
    
    if(!I2CFrameTrans)
        return;
    if(GyroIcmInit)
        return;
    
    switch(Icm_Init_Sta)
    {
        case 0:
            if(Icm_Init_Cnt >= 100)
                Icm_Init_Sta = 1;
            break;
        case 1:
            //1.复位芯片  电源寄存器RESET = 1
            I2CFrameTrans = DISABLE;
            Single_Write_I2C(I2C_DEVICE_GYRO_ICM, PWR_MGMT_1, 0x80);
            Icm_Init_Sta = 2;
            Icm_Init_Cnt = 0;
            
            break;
        
        case 2:
            //2.延时100ms , 等待
            if(Icm_Init_Cnt >= 100)
                Icm_Init_Sta = 3;
            break;
            
        case 3:
            //3.按照初始化数组，依次设置寄存器，寄存器地址=0xff 寄存器设置完毕
            I2CFrameTrans = DISABLE;

            if(ICM_Init_Table[Icm_trans_turn] != 0xff)
            {
                Single_Write_I2C(I2C_DEVICE_GYRO_ICM, ICM_Init_Table[Icm_trans_turn], ICM_Init_Table[Icm_trans_turn + 1]);
                Icm_trans_turn += 2;
                
            }
            
            //4.读取芯片ID ， =0xAE ,初始化成功，否则失败
            else
            {
                Read_I2C(I2C_DEVICE_GYRO_ICM, WHO_AM_I, 1);
                Icm_Init_Sta = 4;
            }
            
            break;
        default:
            break;
    }
    
}

const uint8_t ADXL_Init[15] = 
{
    0x31,0x0B,              //测量范围,正负16g，13位模式
    0x2C,0x0a,              //速率设定为12.5 参考pdf13页 100Hz
    0x2D,0x08,              //选择电源模式   参考pdf24页
    0x2E,0x80,              //使能 DATA_READY 中断
    0x1e,0x0a,              //X 偏移量 根据测试传感器的状态写入pdf29页
    0x1F,0xff,              //Y 偏移量 根据测试传感器的状态写入pdf29页
    0x20,0xff,              //Z 偏移量 根据测试传感器的状态写入pdf29页
    0xff                    //结束标志
};
//初始化ADXL345，根据需要请参考pdf进行修改************************
void Init_ADXL345(void)
{
    static uint8_t ADXLInitTransTurn = 0;
    
    if(!ADXLInitEn)
    {
        if(I2CFrameTrans)
        {
            I2CFrameTrans = DISABLE;
            if(ADXL_Init[ADXLInitTransTurn] != 0xff)
            {
                I2CFrameTrans = DISABLE;
                Single_Write_I2C(I2C_DEVICE_ADXL345, ADXL_Init[ADXLInitTransTurn], ADXL_Init[ADXLInitTransTurn+1]);
                ADXLInitTransTurn += 2;
            }
            else
            {
                //读取设备ID
                Read_I2C(I2C_DEVICE_ADXL345, 0x00,1);
            }
        }
    }  
}

void ADXL345_Read(void)
{
    if(ADXLInitEn)
    {
        Read_I2C(I2C_DEVICE_ADXL345, 0x32,6);
    }
    else
    {
        Read_I2C(I2C_DEVICE_ADXL345, 0x00,1);
    }
}

//I2C调度，防止2个设备同时请求发生冲突
void I2C_Deal(void)
{  
    
    if(I2C_Status)
        return;
    
    if(!I2C_Req)
        return;
    
    I2C_ReqLock = 1;
//    Test2_Sw;
    
    switch(I2C_ReqDevice[I2C_Req])
    {
        case I2C_DEVICE_GYRO:
            I2C_Core = &I2C_Gyro;
            break;
        case I2C_DEVICE_ADXL345:
            I2C_Core = &I2C_ADXL;
            break;
        case I2C_DEVICE_GYRO_ICM:
            I2C_Core = &I2C_Gyro_Icm;
            break;
        case I2C_DEVICE_CHARGE:           
            I2C_Core = &I2C_Charge;
            break;
        case I2C_DEVICE_VL530XL:
            I2C_Core = &I2C_VL53L0X;
        default:
            break;
    }
//    test1++;
    ///I2C_ReqLock 参数说明：
    ///用于保护I2C_ReqLock = 1 时的代码，若这段代码之间有中断发起了I2C请求，需要另外处理
    
    ///I2C_ReqLocked = 1，说明在I2C_ReqLock = 1 时有中断发起I2C请求 
    ///I2C请求设备名会单独保存在I2C_ReqDevice[0]中，在此处处理，处理完成后I2C_ReqLocked = 0
    if(I2C_ReqLocked)
    {
        uint8_t i;
//        I2C_Req++;
//        I2C_ReqDevice[I2C_Req] = I2C_ReqDevice[0];
//        I2C_ReqLocked = 0;
        
        if(I2C_Req + I2C_ReqLocked < I2C_DEVICE_NUM - 1)
        {
            for(i=0; i<I2C_ReqLocked; i++)
            {
                I2C_ReqDevice[I2C_Req + i] = I2C_ReqDeviceLockedBak[i];
            }
            I2C_Req += I2C_ReqLocked;
            I2C_ReqLocked = 0;
        }
    }
    
    I2C_SendPts = 0;
    I2C_ReqLock = 0;
    I2C_GenerateSTART(I2C2, ENABLE);
    I2C_Status = 1;
    I2C_Req--;
    
}

void Gyro_Icm_DataRead(void)
{
    if(!Gyro_Icm_DataReady)
        return;
    Gyro_Icm_DataReady = 0;
    
    Read_I2C(I2C_DEVICE_GYRO_ICM, ACCEL_XOUT_H, 14);
}

/*****************************************************************************
 函 数 名  : EXTI15_10_IRQHandler
 功能描述  : 陀螺仪中断
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2016年3月8日
    作    者   : 田宏图
    修改内容   : 新生成函数
  2.日    期   : 2016年8月12日
    作    者   : 田宏图
    修改内容   : 更改脚位

*****************************************************************************/
void EXTI3_IRQHandler(void)
{
    
//    uint8_t readdata[4];
    
//	if ( EXTI_GetITStatus(EXTI_Line5) != RESET )
//	{
//		EXTI_ClearITPendingBit(EXTI_Line5);
//		
//	}

	if(EXTI_GetITStatus(EXTI_Line3) != RESET)
	{
        EXTI_ClearITPendingBit(EXTI_Line3);
//        Test2_Sw;
        if(GyroIcmInit)
        {
            //20180305 由中断过程中读取，改为置标志位在主循环中读取，解决和VL53L0X的冲突问题
            Gyro_Icm_DataReady = 1;
            
            //20170828 - 陀螺仪改为6轴陀螺仪ICM20608D, GGPM01不使用
//            Read_I2C(I2C_DEVICE_GYRO_ICM, ACCEL_XOUT_H, 14);
//            Test2_Sw;
        }
        else
        {
            
        }
	}
	
//	if(EXTI_GetITStatus(EXTI_Line7) != RESET)
//	{
//		EXTI_ClearITPendingBit(EXTI_Line7);
//		
//	}
//	
//	if(EXTI_GetITStatus(EXTI_Line8) != RESET)
//	{
//		EXTI_ClearITPendingBit(EXTI_Line8);
//		
//	}
//	
//	if(EXTI_GetITStatus(EXTI_Line9) != RESET)
//	{
//		EXTI_ClearITPendingBit(EXTI_Line9);
//		
//	}
}

/*****************************************************************************
 函 数 名  : EXTI0_IRQHandler
 功能描述  : 陀螺仪中断
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2016年3月8日
    作    者   : 田宏图
    修改内容   : 新生成函数
  2.日    期   : 2016年8月12日
    作    者   : 田宏图
    修改内容   : 更改脚位

*****************************************************************************/
void EXTI0_IRQHandler(void)
{
    if(EXTI_GetITStatus(EXTI_Line0) != RESET)
	{
		EXTI_ClearITPendingBit(EXTI_Line0);
        
//        ADXL345_Read();
    }
}

void Single_Write_I2C(uint8_t Device_Num, uint8_t REG_Address,uint8_t REG_data)
{
//    I2C_SendPts = 0;
    I2CData_TypeDef * i2c_device;
    
    switch(Device_Num)
    {
        case I2C_DEVICE_ADXL345:
            i2c_device = &I2C_ADXL;
            break;
        case I2C_DEVICE_GYRO:
            i2c_device = &I2C_Gyro;
            break;
        case I2C_DEVICE_GYRO_ICM:
            i2c_device = &I2C_Gyro_Icm;
            break;
        case I2C_DEVICE_CHARGE:
            i2c_device = &I2C_Charge;
            break;
        case I2C_DEVICE_VL530XL:
            i2c_device = &I2C_VL53L0X;
            break;
        default:
            break;
    }
    
    i2c_device->I2C_Dir = I2C_DIR_WRITE;
    i2c_device->I2C_Device = Device_Num;
    i2c_device->I2C_SendBuf[0] = REG_Address;
    i2c_device->I2C_SendBuf[1] = REG_data;
    i2c_device->I2C_SendBytes = 2;
//    I2C_Req = 1;
//    if(I2C_Req == I2C_ReqBak
    
    //若I2C_ReqLock =  1 需要单独保存请求设备名，因总共只有2个请求设备，所以只预留一个保存空间
    //并且I2C_ReqLocked = 1
    if(I2C_Req != 0 && I2C_ReqLock)
    {
//        I2C_ReqLocked = 1;
//        I2C_ReqDevice[0] = Device_Num;
        
        if(I2C_ReqLocked < I2C_DEVICE_NUM - 1)
        {
            I2C_ReqLocked++;
            I2C_ReqDeviceLockedBak[I2C_ReqLocked] = Device_Num;
        }
    }
    else
    {
        if(I2C_Req < I2C_DEVICE_NUM - 1)
        {
            I2C_Req++;
            I2C_ReqDevice[I2C_Req] = Device_Num;
        }
    }
    
    
}

void Write_I2C(uint8_t Device_Num, uint8_t * REG_Data, uint8_t REG_Len)
{
    I2CData_TypeDef * i2c_device;
    uint8_t i;
    
    switch(Device_Num)
    {
        case I2C_DEVICE_ADXL345:
            i2c_device = &I2C_ADXL;
            break;
        case I2C_DEVICE_GYRO:
            i2c_device = &I2C_Gyro;
            break;
        case I2C_DEVICE_GYRO_ICM:
            i2c_device = &I2C_Gyro_Icm;
            break;
        case I2C_DEVICE_CHARGE:
            i2c_device = &I2C_Charge;
            break;
        case I2C_DEVICE_VL530XL:
            i2c_device = &I2C_VL53L0X;
            break;
        default:
            break;
    }
    
    i2c_device->I2C_Dir = I2C_DIR_WRITE;
    i2c_device->I2C_Device = Device_Num;
    
    for(i = 0; i < REG_Len; i++)
    {
        i2c_device->I2C_SendBuf[i] = REG_Data[i];
    }
    i2c_device->I2C_SendBytes = REG_Len;
    
    //若I2C_ReqLock =  1 需要单独保存请求设备名，因总共只有2个请求设备，所以只预留一个保存空间
    //并且I2C_ReqLocked = 1
    if(I2C_Req != 0 && I2C_ReqLock)
    {
//        I2C_ReqLocked = 1;
//        I2C_ReqDevice[0] = Device_Num;
        if(I2C_ReqLocked < I2C_DEVICE_NUM - 1)
        {
            I2C_ReqLocked++;
            I2C_ReqDeviceLockedBak[I2C_ReqLocked] = Device_Num;
        }
    }
    else
    {
        if(I2C_Req < I2C_DEVICE_NUM - 1)
        {
            I2C_Req++;
            I2C_ReqDevice[I2C_Req] = Device_Num;
        }
    }
}

//********字节读取*****************************************
void Read_I2C(uint8_t Device_Num, uint8_t REG_Address, uint8_t REG_ReadBytes)
{    
    I2CData_TypeDef * i2c_device;
    switch(Device_Num)
    {
        case I2C_DEVICE_ADXL345:
            i2c_device = &I2C_ADXL;
            break;
        case I2C_DEVICE_GYRO:
            i2c_device = &I2C_Gyro;
            break;
        case I2C_DEVICE_GYRO_ICM:
            i2c_device = &I2C_Gyro_Icm;
            break;
        case I2C_DEVICE_CHARGE:
            i2c_device = &I2C_Charge;
            break;
        case I2C_DEVICE_VL530XL:
            i2c_device = &I2C_VL53L0X;
            break;
        default:
            break;
    }
    i2c_device->I2C_Dir = I2C_DIR_RW;
    i2c_device->I2C_Device = Device_Num;
    i2c_device->I2C_SendBuf[0] = REG_Address;
    i2c_device->I2C_SendBytes = 1;
    i2c_device->I2C_RevBytes = REG_ReadBytes;
    
    //若I2C_ReqLock =  1 需要单独保存请求设备名，因总共只有2个请求设备，所以只预留一个保存空间
    //并且I2C_ReqLocked = 1
    if(I2C_Req != 0 && I2C_ReqLock)
    {
//        I2C_ReqLocked = 1;
//        I2C_ReqDevice[0] = Device_Num;
        if(I2C_ReqLocked < I2C_DEVICE_NUM - 1)
        {
            I2C_ReqLocked++;
            I2C_ReqDeviceLockedBak[I2C_ReqLocked] = Device_Num;
        }
    }
    else
    {
        if(I2C_Req < I2C_DEVICE_NUM - 1)
        {
            I2C_Req++;
            I2C_ReqDevice[I2C_Req] = Device_Num;
        }
    }
}

void ReadOnly_I2C(uint8_t Device_Num, uint8_t *Rev_Reg, uint8_t REG_ReadBytes)
{
    I2CData_TypeDef * i2c_device;
    switch(Device_Num)
    {
        case I2C_DEVICE_ADXL345:
            i2c_device = &I2C_ADXL;
            break;
        case I2C_DEVICE_GYRO:
            i2c_device = &I2C_Gyro;
            break;
        case I2C_DEVICE_GYRO_ICM:
            i2c_device = &I2C_Gyro_Icm;
            break;
        case I2C_DEVICE_CHARGE:
            i2c_device = &I2C_Charge;
            break;
        case I2C_DEVICE_VL530XL:
            i2c_device = &I2C_VL53L0X;
            break;
        default:
            break;
    }
    i2c_device->I2C_Dir = I2C_DIR_READ;
    i2c_device->I2C_Device = Device_Num;
//    i2c_device->I2C_SendBuf[0] = REG_Address;
//    i2c_device->I2C_SendBytes = 1;
    i2c_device->I2C_RevBytes = REG_ReadBytes;
    i2c_device->RevData = Rev_Reg;
    //若I2C_ReqLock =  1 需要单独保存请求设备名，因总共只有2个请求设备，所以只预留一个保存空间
    //并且I2C_ReqLocked = 1
    if(I2C_Req != 0 && I2C_ReqLock)
    {
//        I2C_ReqLocked = 1;
//        I2C_ReqDevice[0] = Device_Num;
        if(I2C_ReqLocked < I2C_DEVICE_NUM - 1)
        {
            I2C_ReqLocked++;
            I2C_ReqDeviceLockedBak[I2C_ReqLocked] = Device_Num;
        }
    }
    else
    {
        if(I2C_Req < I2C_DEVICE_NUM - 1)
        {
            I2C_Req++;
            I2C_ReqDevice[I2C_Req] = Device_Num;
        }
    }
}

/*****************************************************************************
 函 数 名  : I2C2_EV_IRQHandler
 功能描述  : I2C事件中断
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2016年8月17日
    作    者   : tht
    修改内容   : 新生成函数

*****************************************************************************/
void I2C2_EV_IRQHandler(void)
{
	static int16_t Sys_AngleOld,Sys_AngleNew,Sys_AnglePoweron;
    int16_t Temp_Data;
	
	if(I2C_GetITStatus(I2C2,I2C_IT_SB) != RESET)
	{
		//起始条件中断
//		I2C_ClearITPendingBit(I2C2,I2C_IT_SB);
//        test3 = 1;
        
        if(I2C_Core->I2C_Dir == I2C_DIR_READ)
        {
            switch(I2C_Core->I2C_Device)
            {
                case I2C_DEVICE_GYRO:
                    //发送设备地址
                    I2C_Send7bitAddress(I2C2, 0x6A, I2C_Direction_Receiver);
                    
                    break;
                case I2C_DEVICE_ADXL345:
                    I2C_Send7bitAddress(I2C2, ADXL345_ADDR, I2C_Direction_Receiver);                     
                    break;
                case I2C_DEVICE_GYRO_ICM:
                    I2C_Send7bitAddress(I2C2, ICM_DEVICE_ADDR, I2C_Direction_Receiver);
                    break;
                case I2C_DEVICE_CHARGE:
                    I2C_Send7bitAddress(I2C2, CHARGE_DEVICE_ADDR, I2C_Direction_Receiver);
                    break;
                case I2C_DEVICE_VL530XL:
                    I2C_Send7bitAddress(I2C2, VL53L0X_ADDR, I2C_Direction_Receiver);
                    break;
                default:
                    break;
            }
            I2C_AcknowledgeConfig(I2C2, ENABLE);
            IIcRevBytes = 0;
        }
        else if(I2C_Core->I2C_Dir == I2C_DIR_WRITE || I2C_Core->I2C_Dir == I2C_DIR_RW)
        {
            switch(I2C_Core->I2C_Device)
            {
                case I2C_DEVICE_GYRO:
                    break;
                case I2C_DEVICE_ADXL345:
                    I2C_Send7bitAddress(I2C2, ADXL345_ADDR, I2C_Direction_Transmitter);
                    break;
                case I2C_DEVICE_GYRO_ICM:
                    I2C_Send7bitAddress(I2C2, ICM_DEVICE_ADDR, I2C_Direction_Transmitter);
                    break;
                case I2C_DEVICE_CHARGE:
                    I2C_Send7bitAddress(I2C2, CHARGE_DEVICE_ADDR, I2C_Direction_Transmitter);
                    break;
                case I2C_DEVICE_VL530XL:
                    I2C_Send7bitAddress(I2C2, VL53L0X_ADDR, I2C_Direction_Transmitter);
                    break;
                default:
                    break;
            }
           
        }
//        test3 = I2C_Core->I2C_Device;
//        test1 = 0;
	}

	if(I2C_GetITStatus(I2C2,I2C_IT_ADDR) != RESET)
	{
		//清标志位
//        I2C_ReadRegister(I2C2, I2C_Register_SR1);
		I2C_ReadRegister(I2C2, I2C_Register_SR2);
        if(I2C_Core->I2C_Dir == I2C_DIR_READ && I2C_Core->I2C_RevBytes == 1)
        {
            I2C_AcknowledgeConfig(I2C2, DISABLE);	
			I2C_GenerateSTOP(I2C2, ENABLE);

        }
		
	}

	if(I2C_GetITStatus(I2C2,I2C_IT_RXNE) != RESET)
	{
		//接收到一个数据
		IIcRevData[IIcRevBytes] = I2C_ReceiveData(I2C2);
		if(IIcRevBytes >= (I2C_Core->I2C_RevBytes - 1))
		{
			if(I2C_Core->I2C_RevBytes == 1)
            {
//                I2C_AcknowledgeConfig(I2C2, DISABLE);	
//				I2C_GenerateSTOP(I2C2, ENABLE);
            }
            
            I2C_Status = 0;
            
            
            switch(I2C_Core->I2C_Device)
            {
                case I2C_DEVICE_GYRO:
                
                    //最后一个字节
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
                    
                    if(!GyroReady)
                    {
                        SysFirstAngle = Sys_Angle;
                        Sys_AnglePoweron = Sys_Angle;
                    }
                    GyroReady = ENABLE;

                    Temp_Data = Sys_Angle - Sys_AnglePoweron;
                    if(Temp_Data > 18000)
                    {
                        Motion_Pos.theta = Temp_Data - 36000;
                    }
                    else if(Temp_Data < -18000)
                    {
                        Motion_Pos.theta = Temp_Data + 36000;
                    }
                    else 
                    {
                        Motion_Pos.theta = Temp_Data;
                    }
                    GyroDataVlid = ENABLE;
                
                    break;
                case I2C_DEVICE_ADXL345:
                    if(!ADXLInitEn)
                    {
                        //读取到设备ID认为初始化成功
                        if(IIcRevData[0] == 0xE5)
                        {
                            ADXLInitEn = ENABLE;
                        }
                    }
                    // 读取加速度数据
                    else
                    {
                        uint8_t i;
                        for(i = 0;i < 6;i++)
                            AccBuf[i] = IIcRevData[i];
                    }
                    break;
                case I2C_DEVICE_GYRO_ICM:
                    if(!GyroIcmInit)
                    {
                        if(IIcRevData[0] == 0xAE)
                        {
                           GyroIcmInit = ENABLE; 
                           I2CFrameTrans = ENABLE;
                        }
                    }
                    else
                    //读取陀螺仪数据
                    {
                        imu.accADC[0] = IcmData.Acc_x = (int16_t)((int16_t)((uint16_t)IIcRevData[0] << 8) + IIcRevData[1]);
                        imu.accADC[1] = IcmData.Acc_y = (int16_t)((int16_t)((uint16_t)IIcRevData[2] << 8) + IIcRevData[3]);
                        imu.accADC[2] = IcmData.Acc_z = (int16_t)((int16_t)((uint16_t)IIcRevData[4] << 8) + IIcRevData[5]);
                        IcmData.Temp = (uint16_t)((uint16_t)((uint16_t)IIcRevData[6] << 8) + IIcRevData[7]);
                        imu.gyroADC[0] = IcmData.Gyro_x = (int16_t)((int16_t)((uint16_t)IIcRevData[8] << 8) + IIcRevData[9]);
                        imu.gyroADC[1] = IcmData.Gyro_y = (int16_t)((int16_t)((uint16_t)IIcRevData[10] << 8) + IIcRevData[11]);
                        imu.gyroADC[2] = IcmData.Gyro_z = (int16_t)((int16_t)((uint16_t)IIcRevData[12] << 8) + IIcRevData[13]);
                        
//                        if(!GyroReady)
//                        {
////                            SysFirstAngle = Sys_Angle;
////                            Sys_AnglePoweron = Sys_Angle;
//                            kalman1_init(&GyroKalman, IcmData.Gyro_z, 20);
//                        }
//                        GyroReady = ENABLE;
                        
                        IcmData.DataReady = 1;
                    }
                    break;
                case I2C_DEVICE_CHARGE:
                    break;
                case I2C_DEVICE_VL530XL:
                    //读取到的数值
                    {
                        uint8_t i;
                        //先将其存储起来，防止被其他覆盖
                        for(i = 0; i < I2C_VL53L0X.I2C_RevBytes;i++)
                            I2C_RevDataVL53L0X[i] = IIcRevData[i];

                        I2C_VL53L0X.RevData = &I2C_RevDataVL53L0X[0];
                    }
                    break;
                default:
                    break;
            }
			
		}
		else
		{
			IIcRevBytes ++;
			if(IIcRevBytes >= (I2C_Core->I2C_RevBytes - 1))
			{
				I2C_AcknowledgeConfig(I2C2, DISABLE);	
				I2C_GenerateSTOP(I2C2, ENABLE);

			}
		}
	}
    
    //发送接收器空中断
    if(I2C_GetITStatus(I2C2,I2C_IT_TXE) != RESET)
    {
        
        if(I2C_Core->I2C_SendBytes > 0)
        {
            I2C_SendData(I2C2, I2C_Core->I2C_SendBuf[I2C_SendPts]);
            I2C_SendPts++;
            I2C_Core->I2C_SendBytes--;
        }
        else
        {
            
            if(I2C_Core->I2C_Dir == I2C_DIR_RW)
            {
                ///--------1-1-1-1-1-1--1-1-1-
//                I2C_Core->I2C_Device = I2C_DEVICE_ADXL345;
                I2C_Core->I2C_Dir = I2C_DIR_READ;
                I2C_GenerateSTART(I2C2, ENABLE);
            }
            else if(I2C_Core->I2C_Dir == I2C_DIR_WRITE)
            {
//                I2C_GenerateSTOP(I2C2, ENABLE);
//                I2CFrameTrans = ENABLE; 
//                I2C_Status = 0; 
            }

        }
    }
    
    if(I2C_GetITStatus(I2C2,I2C_IT_BTF) != RESET)
    {
        //数据传输完成
        if(I2C_Core->I2C_Dir == I2C_DIR_WRITE)
        {
            I2C_GenerateSTOP(I2C2, ENABLE);
            I2CFrameTrans = ENABLE; 
            I2C_Status = 0;    
        }
        else if(I2C_Core->I2C_Dir == I2C_DIR_READ)
        {
//            I2C_Status = 0; 
        }
        else if(I2C_Core->I2C_Dir == I2C_DIR_RW)
        {

        }
        
        
    }
}

void Gyro_Calibration(void)
{
    static int32_t DataSum[3] = {0}, IcmData_Angle = 0, AccOffSetSum[3] = {0}, IcmData_Angle2 = 0;
    static int16_t ZeroPoint[3], DataCnt[3] = {0}, AccOffSet[3] = {0}, AccOffSetCnt[3] = {0};
    static uint8_t CalibOK[6] = {0}, dataupdatecnt = 0, Acc_Stable_Cnt[3] = {0};
    static int16_t Acc_Pre[3] = {0};
    int16_t * databuf, * dataoutput;
    uint8_t i;

    if(!IcmData.DataReady)
        return;
    IcmData.DataReady = 0;
    
    //前500ms的数据不可靠，丢弃
    if(++dataupdatecnt <= 50)
        return;
    
    dataupdatecnt = 51;
    

//    IMUSO3Thread();
//  
//    if(!imuCaliFlag)
//    {
//        if(IMU_Calibrate())
//        {
//            imuCaliFlag=1;
//            GyroReady = ENABLE;
//        }
//    }

    databuf = (int16_t * )(&IcmData);
    dataoutput = (int16_t *)(&IcmDataByKalman);
    for( i = 0 ; i < 6 ; i++)
    {
        //加速度计
        if(i < 3)
        {
            if(!CalibOK[i])
            {
                if(AccOffSetCnt[i] < 300)
                {
                    AccOffSetSum[i] += databuf[i];
                    AccOffSetCnt[i]++;
                }
                else
                {
                    AccOffSet[i] = AccOffSetSum[i] / AccOffSetCnt[i];
                    AccOffSetCnt[i] = 0;
                    AccOffSetSum[i] = 0;
                    
                    if(i == 2)
                    {
                        AccOffSet[i] = AccOffSet[i] - ACC_ONE_G;
                    }
                    CalibOK[i] = 1;
                    kalman1_init(&GyroKalman[i], databuf[i] - AccOffSet[i], 20, 1, 40);
                }   
            }
            else
            {
                dataoutput[i] = kalman1_filter(&GyroKalman[i], databuf[i]);
                
                if(fabs(Acc_Pre[i] - databuf[i]) < 100)
                {
                    if(Acc_Stable_Cnt[i] < 255)
                        Acc_Stable_Cnt[i]++;
                }
                else
                {
                    Acc_Stable_Cnt[i] = 0;
                }
                Acc_Pre[i] = databuf[i];
            }
        }
        //陀螺仪
        else
        {
            if(!CalibOK[i])
            {
                if(DataCnt[i-3] < 300)
                {
                    DataSum[i-3] += databuf[i];
                    DataCnt[i-3]++;
                }
                else
                {
                    ZeroPoint[i-3] = DataSum[i-3] / DataCnt[i-3];
                    CalibOK[i] = 1;
                    DataSum[i-3] = 0;
                    DataCnt[i-3] = 0;
                    kalman1_init(&GyroKalman[i], databuf[i] - ZeroPoint[i-3], 20, 1, 40);
                    GyroReady = ENABLE;
                }
            }
            else
            {
                dataoutput[i] = kalman1_filter(&GyroKalman[i], databuf[i] - ZeroPoint[i-3]);
//                test2 = dataoutput[5];
                               
                if(dataoutput[i] <= GYRO_ZERO_RANGE && dataoutput[i] >= -GYRO_ZERO_RANGE)
                {
                    //校正零点 当加速度测量值波动在设定范围内时，校正零点                     
                    if(Acc_Stable_Cnt[0] >= 20  && \
                       Acc_Stable_Cnt[1] >= 20  && \
                       Acc_Stable_Cnt[2] >= 20  && \
                       0 == RobitStatus)
                    {
                        DataSum[i-3] += dataoutput[i];
                        DataCnt[i-3]++;
                    }
                    
                    if(DataCnt[i-3] >= 256)
                    {
                        ZeroPoint[i-3] += (DataSum[i-3] >> 8);
                        DataCnt[i-3] = 0;
                        DataSum[i-3] = 0;                        
                    }                                     
                    
                    if(dataoutput[i] <= GYRO_ZERO_DEAD_ZONE && dataoutput[i] >= -GYRO_ZERO_DEAD_ZONE)
                        dataoutput[i] = 0;
                } 

            }
        }
    }
    
    
    //只对Z轴的角速度积分                 
    IcmData_Angle += IcmDataByKalman.Gyro_z;
    IcmData_Angle = AngleFormatForIcm(IcmData_Angle);
    
    
    //角度/角速度方向 顺时针为正，逆时针为负
    Sys_Angle = -((IcmData_Angle * GYRO_RANGE) >> 16); 
    Sys_Anglerate = -((int32_t)((int32_t)IcmDataByKalman.Gyro_z * GYRO_RANGE * 100) >> 16);

    // x轴积分 用于检测地毯
    IcmData_Angle2 += IcmDataByKalman.Gyro_x;
    IcmData_Angle2 = AngleFormatForIcm(IcmData_Angle2);
    Sys_Angle2 = -((IcmData_Angle2 * GYRO_RANGE) >> 16);
}

int64_t AngleFormatForIcm(int64_t _angle_)
{
//    static int64_t temp;
//    temp = GYRO_RANGE;
//    temp = ANGLE_RANGE << 1 ;
    while(_angle_ > ANGLE_RANGE)
    {
        _angle_ -= ANGLE_RANGE << 1; 
    }
    
    while(_angle_ < -ANGLE_RANGE)
    {
        _angle_ += ANGLE_RANGE << 1;
    }
    
    return _angle_;
}

