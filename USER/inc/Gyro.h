#ifndef _GYRO_H
#define _GYRO_H

#define I2C_DEVICE_VL530XL  5
#define VL53L0X_ADDR        0x52

#define	RCC_APB2Periph_GPIOGYRO	RCC_APB2Periph_GPIOB
#define GPIO_INT		GPIOA
#define GPIO_Pin_Int	GPIO_Pin_3
#define GPIO_SCL		GPIOB
#define GPIO_Pin_Scl	GPIO_Pin_11
#define	GPIO_SDA		GPIOB
#define GPIO_Pin_Sda	GPIO_Pin_10

#define SDA_Out()		GPIO_SDA->CRH |= GPIO_CRH_MODE10;GPIO_SDA->CRH &= ~GPIO_CRH_CNF10
#define	SDA_In()		GPIO_SDA->CRH &= ~GPIO_CRH_MODE10;GPIO_SDA->CRH |= GPIO_CRH_CNF10_0

#define SCL_H         GPIO_SCL->BSRR = GPIO_Pin_Scl	 /* GPIO_SetBits(GPIOB , GPIO_Pin_6)   */
#define SCL_L         GPIO_SCL->BRR  = GPIO_Pin_Scl   /* GPIO_ResetBits(GPIOB , GPIO_Pin_6) */
   
#define SDA_H         GPIO_SDA->BSRR = GPIO_Pin_Sda	 /* GPIO_SetBits(GPIOB , GPIO_Pin_7)   */
#define SDA_L         GPIO_SDA->BRR  = GPIO_Pin_Sda	 /* GPIO_ResetBits(GPIOB , GPIO_Pin_7) */

#define SCL_read      (GPIO_SCL->IDR  & GPIO_Pin_Scl)   /* GPIO_ReadInputDataBit(GPIOB , GPIO_Pin_6) */
#define SDA_read      (GPIO_SDA->IDR  & GPIO_Pin_Sda)	 /* GPIO_ReadInputDataBit(GPIOB , GPIO_Pin_7) */

#define Delay_nop()     __nop();__nop();__nop();__nop();

#define LongTime		18
#define MidTime			16
#define ShortTime		13

#define GYRO_RANGE_SET      1                                   // 1->±250°/s   2->±500°/s  3->±1000°/s  4->±2000°/s
#define GYRO_RANGE          (int16_t)(500 << (GYRO_RANGE_SET - 1))
#define GYRO_ZERO_DEAD_ZONE (int16_t)(2 << (4 - GYRO_RANGE_SET))
#define GYRO_CONGFIG_DATA   (int16_t)((GYRO_RANGE_SET - 1) << 3)
#define ANGLE_RANGE         ((int64_t)((int64_t)18000 << 16) / GYRO_RANGE)

#define GYRO_ZERO_RANGE     300
#define ACC_ONE_G           16384

#pragma pack(push, 1)
typedef struct
{
    int16_t Acc_x;       //x轴加速度
    int16_t Acc_y;       //y轴加速度
    int16_t Acc_z;       //z轴加速度   
    int16_t Gyro_x;      //x轴角速度
    int16_t Gyro_y;      //y轴角速度
    int16_t Gyro_z;      //z轴角速度
    int16_t Temp;       //芯片温度
    uint8_t DataReady;   //数据已准备好
}Gyro_6Axis_TypeDef;

typedef struct
{
    uint8_t I2C_SendBuf[10]; //发送缓存
    
    uint8_t I2C_SendBytes;      //发送字节数
    
//    uint8_t I2C_SendPts;        //发送个数
    
    uint8_t I2C_Device;         //发送设备
    
    uint8_t I2C_Dir;            //发送方向
    
    uint8_t I2C_RevBytes;       //接收字节数
    
    uint8_t *RevData;           //接收数据
    
} I2CData_TypeDef;

#pragma pack(pop)

extern int16_t Sys_Angle,Sys_Anglerate,Sys_AnglePre, Sys_Angle2, Sys_AnglexInit;
extern uint8_t IIcRevData[20];
extern FunctionalState GyroReady, GyroDataVlid;
extern uint8_t AccBuf[6];
extern uint8_t Battery_Charge_En;
extern uint16_t Icm_Init_Cnt;
extern Gyro_6Axis_TypeDef IcmDataByKalman;
extern uint8_t I2C_Status, I2C_Req, I2C_ReqLock, IIcRevBytes;
extern I2CData_TypeDef I2C_VL53L0X, *I2C_Core ;

extern void IIC_Test(void);
extern void Gyro_Conf(void);
extern void Init_ADXL345(void);
extern void ICM_Init(void);
extern void ADXL345_Read(void);
extern void I2C_Deal(void);
extern void ChargeManageInit(void);
extern void Gyro_Calibration(void);
extern void Single_Write_I2C(uint8_t Device_Num, uint8_t REG_Address,uint8_t REG_data);
extern void Write_I2C(uint8_t Device_Num, uint8_t * REG_Data, uint8_t REG_Len);
extern void ReadOnly_I2C(uint8_t Device_Num, uint8_t *Rev_Reg, uint8_t REG_ReadBytes);

#endif
