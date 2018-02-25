//?????#include "hal.h"
#include "vl53l0x_platform.h"
#include "vl53l0x_api.h"

//#include "stm32xxx_hal.h"
#include "stm32f10x.h"
#include <string.h>

#define I2C_TIME_OUT_BASE   50
#define I2C_TIME_OUT_BYTE   50
#define VL53L0X_OsDelay(...) Delay_nop() //HAL_Delay(2)



#define HAL_I2C_MODULE_ENABLED
#ifndef HAL_I2C_MODULE_ENABLED
#warning "HAL I2C module must be enable "
#endif
//extern I2C_HandleTypeDef hi2c1;
//#define VL53L0X_pI2cHandle    (&hi2c1)

/* when not customized by application define dummy one */
#ifndef VL53L0X_GetI2cBus
/** This macro can be overloaded by user to enforce i2c sharing in RTOS context
 */
#   define VL53L0X_GetI2cBus(...) (void)0
#endif

#ifndef VL53L0X_PutI2cBus
/** This macro can be overloaded by user to enforce i2c sharing in RTOS context
 */
#   define VL53L0X_PutI2cBus(...) (void)0
#endif

#ifndef VL53L0X_OsDelay
#   define  VL53L0X_OsDelay(...) (void)0
#endif


uint8_t _I2CBuffer[64];

typedef struct 
{
    uint16_t devAddr;
    uint8_t *pdata;
    uint16_t len;
    uint8_t dir;
}I2C_Buff;

I2C_Buff i2c_SendRevBuff[20];
uint8_t i2c_Sta = 0, i2cBuffEmpty = 1, pStart = 0, pEnd = 0;

#define TRANSMIT    1
#define RECEIVE     0

//I2C发送
int HAL_I2C_Master_Transmit(uint16_t DevAddress, uint8_t *pData, uint16_t Size , int _time_out_cnt)
{
    uint16_t start_count, end_count;
    //i2c 空闲
    if(0 == i2c_Sta)
    {
        //发送
        i2c_Sta = 0;
        Write_I2C(I2C_DEVICE_VL530XL, pData, Size);
        
        //获取当前定时器计数
        start_count = TIM_GetCounter(TIM2);
        
        I2C_Deal();
        
        
        //等待传输完成
        while(I2C_Status)
        {
            //获取当前计数
            end_count = TIM_GetCounter(TIM2);
            
            //判断是否超时
            //若超时,则退出,发送失败
            if(end_count > start_count)
            {
                if(end_count - start_count >= _time_out_cnt)
                {
                    return VL53L0X_ERROR_CONTROL_INTERFACE_1;
                }
            }
            else 
            {
                if(0xffff - start_count + end_count >= _time_out_cnt)
                {
                    return VL53L0X_ERROR_CONTROL_INTERFACE_1;
                }
            }
           
        }
        
        //传输完成
        i2c_Sta = 0;
    }
    else
    {
        return VL53L0X_ERROR_CONTROL_INTERFACE_2;
//        i2cBuffEmpty = 0;
//        i2c_SendRevBuff[pEnd].devAddr = DevAddress;
//        i2c_SendRevBuff[pEnd].pdata = pData;
//        i2c_SendRevBuff[pEnd].len = Size;
//        i2c_SendRevBuff[pEnd].dir = TRANSMIT;
//        pEnd++;
//        
//        if(pEnd >= 20)
//            pEnd = 0;
    }
    return 0;
}

//I2C接收
int HAL_I2C_Master_Receive(uint16_t DevAddress, uint8_t *pData, uint16_t Size, int _time_out_cnt)
{
    uint16_t start_count, end_count;
    if(0 == i2c_Sta)
    {
        //读取接收
        i2c_Sta = 0;
        ReadOnly_I2C(I2C_DEVICE_VL530XL, pData, Size);
        
        //获取当前定时器计数
        start_count = TIM_GetCounter(TIM2);
        
        I2C_Deal();
        
        //等待传输完成
        while(I2C_Status)
        {
            //获取当前计数
            end_count = TIM_GetCounter(TIM2);
            
            //判断是否超时  通过读取定时器的计数值实现             
            //若超时,则退出,接收失败
            if(end_count > start_count)
            {
                if(end_count - start_count >= _time_out_cnt)
                {
                    return VL53L0X_ERROR_CONTROL_INTERFACE_3;
                }
            }
            else 
            {
                if(0xffff - start_count + end_count >= _time_out_cnt)
                {
                    return VL53L0X_ERROR_CONTROL_INTERFACE_3;
                }
            }
        }
        
        //传输完成，获取接收到的数据
        memcpy(pData, I2C_VL53L0X.RevData, Size);
        i2c_Sta = 0;
    }
    else
    {
        return VL53L0X_ERROR_CONTROL_INTERFACE_4;
//        i2cBuffEmpty = 0;
//        i2c_SendRevBuff[pEnd].devAddr = DevAddress;
//        i2c_SendRevBuff[pEnd].pdata = pData;
//        i2c_SendRevBuff[pEnd].len = Size;
//        i2c_SendRevBuff[pEnd].dir = RECEIVE;
//        pEnd++;
//        
//        if(pEnd >= 20)
//            pEnd = 0;
    }
    return 0;
}

void i2c_BuffRead(void)
{
    if(i2c_Sta)
    {
        if(pEnd == pStart)
        {
            //I2C空闲状态 
            if(0 == I2C_Status)
            {
                i2c_Sta = 0;
            }
        }
        else
        {
            if(i2c_SendRevBuff[pStart].dir == RECEIVE)
            {
                //接收
                ReadOnly_I2C(I2C_DEVICE_VL530XL, i2c_SendRevBuff[pStart].pdata, i2c_SendRevBuff[pStart].len);
            }
            else
            {
                //发送
                Write_I2C(I2C_DEVICE_VL530XL, i2c_SendRevBuff[pStart].pdata, i2c_SendRevBuff[pStart].len);
            }
            pStart++;
            
            if(pStart >= 20)
                pStart = 0;
        }
    }
}


int _I2CWrite(VL53L0X_DEV Dev, uint8_t *pdata, uint32_t count) {
    int status;
    int i2c_time_out = I2C_TIME_OUT_BASE + count * I2C_TIME_OUT_BYTE;

    status = HAL_I2C_Master_Transmit(Dev->I2cDevAddr, pdata, count, i2c_time_out);
    if (status) {
        //VL6180x_ErrLog("I2C error 0x%x %d len", dev->I2cAddr, len);
        //XNUCLEO6180XA1_I2C1_Init(&hi2c1);
    }
    return status;
}

int _I2CRead(VL53L0X_DEV Dev, uint8_t *pdata, uint32_t count) {
    int status;
    int i2c_time_out = I2C_TIME_OUT_BASE + count * I2C_TIME_OUT_BYTE;

    status = HAL_I2C_Master_Receive(Dev->I2cDevAddr|1, pdata, count, i2c_time_out);
    if (status) {
        //VL6180x_ErrLog("I2C error 0x%x %d len", dev->I2cAddr, len);
        //XNUCLEO6180XA1_I2C1_Init(&hi2c1);
    }
    return status;
}

// the ranging_sensor_comms.dll will take care of the page selection
VL53L0X_Error VL53L0X_WriteMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count) {
    int status_int;
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    if (count > sizeof(_I2CBuffer) - 1) {
        return VL53L0X_ERROR_INVALID_PARAMS;
    }
    _I2CBuffer[0] = index;
    memcpy(&_I2CBuffer[1], pdata, count);
    VL53L0X_GetI2cBus();
    status_int = _I2CWrite(Dev, _I2CBuffer, count + 1);
    if (status_int != 0) {
        Status = status_int;//VL53L0X_ERROR_CONTROL_INTERFACE;
    }
    VL53L0X_PutI2cBus();
    return Status;
}

// the ranging_sensor_comms.dll will take care of the page selection
VL53L0X_Error VL53L0X_ReadMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count) {
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    int32_t status_int;
    VL53L0X_GetI2cBus();
    status_int = _I2CWrite(Dev, &index, 1);
    if (status_int != 0) {
        Status = status_int;//VL53L0X_ERROR_CONTROL_INTERFACE;
        goto done;
    }
    status_int = _I2CRead(Dev, pdata, count);
    if (status_int != 0) {
        Status = status_int;//VL53L0X_ERROR_CONTROL_INTERFACE;
    }
done:
    VL53L0X_PutI2cBus();
    return Status;
}

VL53L0X_Error VL53L0X_WrByte(VL53L0X_DEV Dev, uint8_t index, uint8_t data) {
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    int32_t status_int;

    _I2CBuffer[0] = index;
    _I2CBuffer[1] = data;

    VL53L0X_GetI2cBus();
    status_int = _I2CWrite(Dev, _I2CBuffer, 2);
    if (status_int != 0) {
        Status = status_int;//VL53L0X_ERROR_CONTROL_INTERFACE;
    }
    VL53L0X_PutI2cBus();
    return Status;
}

VL53L0X_Error VL53L0X_WrWord(VL53L0X_DEV Dev, uint8_t index, uint16_t data) {
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    int32_t status_int;

    _I2CBuffer[0] = index;
    _I2CBuffer[1] = data >> 8;
    _I2CBuffer[2] = data & 0x00FF;

    VL53L0X_GetI2cBus();
    status_int = _I2CWrite(Dev, _I2CBuffer, 3);
    if (status_int != 0) {
        Status = status_int;//VL53L0X_ERROR_CONTROL_INTERFACE;
    }
    VL53L0X_PutI2cBus();
    return Status;
}

VL53L0X_Error VL53L0X_WrDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t data) {
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    int32_t status_int;
    _I2CBuffer[0] = index;
    _I2CBuffer[1] = (data >> 24) & 0xFF;
    _I2CBuffer[2] = (data >> 16) & 0xFF;
    _I2CBuffer[3] = (data >> 8)  & 0xFF;
    _I2CBuffer[4] = (data >> 0 ) & 0xFF;
    VL53L0X_GetI2cBus();
    status_int = _I2CWrite(Dev, _I2CBuffer, 5);
    if (status_int != 0) {
        Status = status_int;//VL53L0X_ERROR_CONTROL_INTERFACE;
    }
    VL53L0X_PutI2cBus();
    return Status;
}

VL53L0X_Error VL53L0X_UpdateByte(VL53L0X_DEV Dev, uint8_t index, uint8_t AndData, uint8_t OrData) {
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t data;

    Status = VL53L0X_RdByte(Dev, index, &data);
    if (Status) {
        goto done;
    }
    data = (data & AndData) | OrData;
    Status = VL53L0X_WrByte(Dev, index, data);
done:
    return Status;
}

VL53L0X_Error VL53L0X_RdByte(VL53L0X_DEV Dev, uint8_t index, uint8_t *data) {
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    int32_t status_int;

    VL53L0X_GetI2cBus();
    status_int = _I2CWrite(Dev, &index, 1);
    if( status_int ){
        Status = status_int;//VL53L0X_ERROR_CONTROL_INTERFACE;
        goto done;
    }
    status_int = _I2CRead(Dev, data, 1);
    if (status_int != 0) {
        Status = status_int;//VL53L0X_ERROR_CONTROL_INTERFACE;
    }
done:
    VL53L0X_PutI2cBus();
    return Status;
}

VL53L0X_Error VL53L0X_RdWord(VL53L0X_DEV Dev, uint8_t index, uint16_t *data) {
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    int32_t status_int;

    VL53L0X_GetI2cBus();
    status_int = _I2CWrite(Dev, &index, 1);

    if( status_int ){
        Status = status_int;//VL53L0X_ERROR_CONTROL_INTERFACE;
        goto done;
    }
    status_int = _I2CRead(Dev, _I2CBuffer, 2);
    if (status_int != 0) {
        Status = status_int;//VL53L0X_ERROR_CONTROL_INTERFACE;
        goto done;
    }

    *data = ((uint16_t)_I2CBuffer[0]<<8) + (uint16_t)_I2CBuffer[1];
done:
    VL53L0X_PutI2cBus();
    return Status;
}

VL53L0X_Error VL53L0X_RdDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t *data) {
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    int32_t status_int;

    VL53L0X_GetI2cBus();
    status_int = _I2CWrite(Dev, &index, 1);
    if (status_int != 0) {
        Status = status_int;//VL53L0X_ERROR_CONTROL_INTERFACE;
        goto done;
    }
    status_int = _I2CRead(Dev, _I2CBuffer, 4);
    if (status_int != 0) {
        Status = status_int;//VL53L0X_ERROR_CONTROL_INTERFACE;
        goto done;
    }

    *data = ((uint32_t)_I2CBuffer[0]<<24) + ((uint32_t)_I2CBuffer[1]<<16) + ((uint32_t)_I2CBuffer[2]<<8) + (uint32_t)_I2CBuffer[3];

done:
    VL53L0X_PutI2cBus();
    return Status;
}

VL53L0X_Error VL53L0X_PollingDelay(VL53L0X_DEV Dev) {
    VL53L0X_Error status = VL53L0X_ERROR_NONE;

    // do nothing
    VL53L0X_OsDelay();
    return status;
}

//end of file
