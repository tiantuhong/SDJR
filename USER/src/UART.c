 /******************************************************************************

                  版权所有 (C), 1994-2016, 杭州九阳欧南多小家电有限公司

 ******************************************************************************
  文 件 名   : UART.c
  版 本 号   : V1.0
  作    者   : 田宏图
  生成日期   : 2016年3月16日
  最近修改   :
  功能描述   : UART文件
  函数列表   :
  修改历史   :
  1.日    期   : 2016年3月16日
    作    者   : 田宏图
    修改内容   : 创建文件

******************************************************************************/

/*----------------------------------------------*
 * 包含头文件                                   *
 *----------------------------------------------*/
#include "stm32f10x.h"
#include <string.h>
#include <math.h>
#include "imu.h"
/*----------------------------------------------*
 * 外部变量说明                                 *
 *----------------------------------------------*/
FunctionalState Uart_SendEn, Uart2_SendEn, HandShakeEn;

/*----------------------------------------------*
 * 外部函数原型说明                             *
 *----------------------------------------------*/
void UART_Conf(void);
void UartTimerJudge(void);
void SetRobitAngleRun(uint16_t Angle, uint8_t Dir, uint16_t AngleRate, uint8_t SpinMode);
void SetRobitMileageRun(uint16_t Mileage, uint8_t Dir, uint16_t RunSpeed);
void SetRobitSpeed(int16_t LeftSpeed, int16_t RightSpeed);
void UART2_SendData(void);
void SetRobitVacc(int32_t _vacc_);
void SetRobitWacc(int32_t _wacc_);
void MCU_Check_CheanPackStatus(CheckCode _check_code_);
void MCU_Send_Charge_Status(void);
/*----------------------------------------------*
 * 内部函数原型说明                             *
 *----------------------------------------------*/
int32_t Length2Mileage(int32_t LengthInput);
int32_t Angle2Mileage(int32_t AngleInput);
uint16_t ABSNum(int16_t InputNum);
void SetRobitLineAnglerate(int16_t lineSpeed, int16_t angleRate);
void remove(uint8_t * data, uint8_t len, uint8_t idx);
void add(uint8_t * data, uint8_t len, uint8_t idx);
uint8_t RegisterWrite(RegisterType * reg, uint8_t shiftaddr, uint8_t data_size, uint8_t * data);
uint8_t StatusRegisterWrite(StatusRegisterType * reg, uint8_t shiftaddr, uint8_t data_size, uint8_t * data);
void DrawArc(int16_t lineSpeed, int16_t angleRate, uint16_t angleTarg, uint32_t Mileage);
/*----------------------------------------------*
 * 全局变量                                     *
 *----------------------------------------------*/
//发送变量
uint8_t UART_SendBuf[UART_BUF_LEN],UART2_SendBuf[UART2_BUF_LEN];
uint8_t* UART_SendPtr,*UART2_SendPtr;
uint8_t UART_SendBytes,UART2_SendBytes;
FunctionalState UART_SendOver,UART2_SendOver;
uint16_t UART_Send_Fq;

McuStatus Mcu_Charge_Status;
uint8_t WIFI_Status;

//接收变量
uint8_t UART_RcvBuf[UART_BUF_LEN];
uint8_t UART_RcvPtr;
uint8_t UART_RcvBytes;

//FunctionalState PID_SET;
FunctionalState UartFrameHeadCheckEn, UartFrameTailCheckEn;
BitAction	FlagUartFrameHead, FlagUartFrameTail;
uint8_t FrameHeadDataBuf = 0, FrameTailDataBuf = 0;

RegisterType ControlReg;
StatusRegisterType CleanStatus;
int32_t AngleTarg;
//uint8_t RobotCleanStatus, RobotCleanSubStatus;

uint16_t test1,test2,test3;
int16_t q0buf,q1buf,q2buf,q3buf,axbuf,aybuf,azbuf;

/*----------------------------------------------*
 * 模块级变量                                   *
 *----------------------------------------------*/
uint16_t UartTimerCnt = 0;
FunctionalState UartRcvOneFrameData;
ITStatus UartMaterLost;
uint8_t UartSendCnt,UartBumpSendCnt,UartWheelSendCnt,UartWallSendCnt;

uint8_t UartSendCmd, UartSendDataAddr, UartSendDatlength;
T_UART_ACKFarme UartAck[7];
T_UART_ACKFarme_P UartAck_P;
uint8_t StepModeOn, CtrlMode;

//速度模式中，上一次设置的线速度和角速度，防止相同的线速度和角速度重复设置，初始化为0
int32_t RobotLineSpeedTargPre, RobotAngleRatePre;

uint32_t StepStatus, GarbageStatus;
/*----------------------------------------------*
 * 常量定义                                     *
 *----------------------------------------------*/
#define SPEED_SET   200  // 速度设定值小于该值认为为0，在该值和MOT_RPM_SPEED_MIN之间认为为MOT_RPM_SPEED_MIN


const uint8_t RobotPar[224] = 
{
  0xff,0xff,  //超声波传感器1安装角度
  0xff,0xff,  //超声波传感器1可视角度
  0xff,0xff,  //超声波传感器2安装角度
  0xff,0xff,  //超声波传感器2可视角度
  0xff,0xff,  //超声波传感器3安装角度
  0xff,0xff,  //超声波传感器3可视角度
  0xff,0xff,  //超声波传感器4安装角度
  0xff,0xff,  //超声波传感器4可视角度
  0xff,0xff,  //超声波传感器5安装角度
  0xff,0xff,  //超声波传感器5可视角度
  0xff,0xff,  //超声波传感器6安装角度
  0xff,0xff,  //超声波传感器6可视角度
  0xff,0xff,  //超声波传感器7安装角度
  0xff,0xff,  //超声波传感器7可视角度
  0xff,0xff,  //超声波传感器8安装角度
  0xff,0xff,  //超声波传感器8可视角度
    
  0xff,0xff,  //防跌落传感器1安装角度
  0xff,0xff,  //防跌落传感器1可视角度
  0xff,0xff,  //防跌落传感器2安装角度
  0xff,0xff,  //防跌落传感器2可视角度
  0xff,0xff,  //防跌落传感器3安装角度
  0xff,0xff,  //防跌落传感器3可视角度
  0xff,0xff,  //防跌落传感器4安装角度
  0xff,0xff,  //防跌落传感器4可视角度
  0xff,0xff,  //防跌落传感器5安装角度
  0xff,0xff,  //防跌落传感器5可视角度
  0xff,0xff,  //防跌落传感器6安装角度
  0xff,0xff,  //防跌落传感器6可视角度
  0xff,0xff,  //防跌落传感器7安装角度
  0xff,0xff,  //防跌落传感器7可视角度
  0xff,0xff,  //防跌落传感器8安装角度
  0xff,0xff,  //防跌落传感器8可视角度
  0xff,0xff,  //防跌落传感器9安装角度
  0xff,0xff,  //防跌落传感器9可视角度
  0xff,0xff,  //防跌落传感器10安装角度
  0xff,0xff,  //防跌落传感器10可视角度
  0xff,0xff,  //防跌落传感器11安装角度
  0xff,0xff,  //防跌落传感器11可视角度
  0xff,0xff,  //防跌落传感器12安装角度
  0xff,0xff,  //防跌落传感器12可视角度
  0xff,0xff,  //防跌落传感器13安装角度
  0xff,0xff,  //防跌落传感器13可视角度
  0xff,0xff,  //防跌落传感器14安装角度
  0xff,0xff,  //防跌落传感器14可视角度
  0xff,0xff,  //防跌落传感器15安装角度
  0xff,0xff,  //防跌落传感器15可视角度
  0xff,0xff,  //防跌落传感器16安装角度
  0xff,0xff,  //防跌落传感器16可视角度
    
  0xff,0xff,  //红外传感器1安装角度
  0xff,0xff,  //红外传感器1可视角度
  0xff,0xff,  //红外传感器2安装角度
  0xff,0xff,  //红外传感器2可视角度
  0xff,0xff,  //红外传感器3安装角度
  0xff,0xff,  //红外传感器3可视角度
  0xff,0xff,  //红外传感器4安装角度
  0xff,0xff,  //红外传感器4可视角度
  0xff,0xff,  //红外传感器5安装角度
  0xff,0xff,  //红外传感器5可视角度
  0xff,0xff,  //红外传感器6安装角度
  0xff,0xff,  //红外传感器6可视角度
  0xff,0xff,  //红外传感器7安装角度
  0xff,0xff,  //红外传感器7可视角度
  0xff,0xff,  //红外传感器8安装角度
  0xff,0xff,  //红外传感器8可视角度
  0xff,0xff,  //红外传感器9安装角度
  0xff,0xff,  //红外传感器9可视角度
  0xff,0xff,  //红外传感器10安装角度
  0xff,0xff,  //红外传感器10可视角度
  0xff,0xff,  //红外传感器11安装角度
  0xff,0xff,  //红外传感器11可视角度
  0xff,0xff,  //红外传感器12安装角度
  0xff,0xff,  //红外传感器12可视角度
  0xff,0xff,  //红外传感器13安装角度
  0xff,0xff,  //红外传感器13可视角度
  0xff,0xff,  //红外传感器14安装角度
  0xff,0xff,  //红外传感器14可视角度
  0xff,0xff,  //红外传感器15安装角度
  0xff,0xff,  //红外传感器15可视角度
  0xff,0xff,  //红外传感器16安装角度
  0xff,0xff,  //红外传感器16可视角度
    
  0xff,0xff,  //碰撞传感器1安装角度
  0xff,0xff,  //碰撞传感器1可视角度
  0xff,0xff,  //碰撞传感器2安装角度
  0xff,0xff,  //碰撞传感器2可视角度
  0xff,0xff,  //碰撞传感器3安装角度
  0xff,0xff,  //碰撞传感器3可视角度
  0xff,0xff,  //碰撞传感器4安装角度
  0xff,0xff,  //碰撞传感器4可视角度
  0xff,0xff,  //碰撞传感器5安装角度
  0xff,0xff,  //碰撞传感器5可视角度
  0xff,0xff,  //碰撞传感器6安装角度
  0xff,0xff,  //碰撞传感器7安装角度
  0xff,0xff,  //碰撞传感器7可视角度
  0xff,0xff,  //碰撞传感器8安装角度
  0xff,0xff,  //碰撞传感器8可视角度
  0xff,0xff,  //碰撞传感器9安装角度
  0xff,0xff,  //碰撞传感器9可视角度
  0xff,0xff,  //碰撞传感器10安装角度
  0xff,0xff,  //碰撞传感器10可视角度
  0xff,0xff,  //碰撞传感器11安装角度
  0xff,0xff,  //碰撞传感器11可视角度
  0xff,0xff,  //碰撞传感器12安装角度
  0xff,0xff,  //碰撞传感器12可视角度
  0xff,0xff,  //碰撞传感器13安装角度
  0xff,0xff,  //碰撞传感器13可视角度
  0xff,0xff,  //碰撞传感器14安装角度
  0xff,0xff,  //碰撞传感器14可视角度
  0xff,0xff,  //碰撞传感器15安装角度
  0xff,0xff,  //碰撞传感器15可视角度
  0xff,0xff,  //碰撞传感器16安装角度
  0xff,0xff   //碰撞传感器16可视角度
};


void UART_Conf(void)
{
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
//	DMA_InitTypeDef DMA_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
//	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	
	/* DMA channel1 configuration ----------------------------------------------*/
//	DMA_DeInit(DMA1_Channel4);
//	DMA_InitStructure.DMA_PeripheralBaseAddr = UART_DR_Address;
//	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)UART_SendBuf;
//	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
//	DMA_InitStructure.DMA_BufferSize = UART_BYTES;
//	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
//	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
//	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
//	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
//	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
//	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
//	DMA_Init(DMA1_Channel4, &DMA_InitStructure);

	/* Enable DMA1 channel1 */
//	DMA_Cmd(DMA1_Channel4, ENABLE);

	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART1, &USART_InitStructure); 
    
//    USART_InitStructure.USART_BaudRate = 9600;
	USART_Init(USART2, &USART_InitStructure); 

//	USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);	

//	USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	USART_ITConfig(USART1, USART_IT_TC, ENABLE); 

	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	USART_ITConfig(USART2, USART_IT_TC, ENABLE); 

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  													
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;   
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;   
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;	      
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  													
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;   
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;   
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;	      
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	USART_Cmd(USART1, ENABLE);
	USART_Cmd(USART2, ENABLE);

	
	UartSendCnt = 0;
	UartBumpSendCnt = 0;
	UartWheelSendCnt = 0;
	UartWallSendCnt = 0;
	UART_Send_Fq = 20;
	UART_SendOver = ENABLE;
	UART2_SendOver = ENABLE;
    HandShakeEn = DISABLE;
	
	Kp_Set = 100;
	Kd_Set = 0;
	Ki_Set = 0;
//	PID_SET = DISBALE;

	UartFrameHeadCheckEn = ENABLE;
	UartFrameTailCheckEn = DISABLE;
	FlagUartFrameHead = Bit_RESET;
	FlagUartFrameTail = Bit_RESET;
    
    ControlReg.Reg_Length = CONTROL_REG_LEN;
    CtrlMode = 0;
    RobotLineSpeedTargPre = 0;
    RobotAngleRatePre = 0;
    TargMileageForArc = 0;
    
    
    CleanStatus.Reg.cleanStatus = CPS_IDLE;
    CleanStatus.Reg.cleanStatusSub = CLEAN_SUB_MOD_NULL;
    CleanStatus.Length = 4;
    
    Mcu_Charge_Status.ctrlChargingStatus = 0;
//    WIFI_Status = 0;
}


void Uart_Send_Start(void)
{
  //启动串口发送
  UART_SendPtr = UART_SendBuf;
  USART_SendData(USART1, *UART_SendPtr);
}

void Uart2_Send_Start(void)
{
  //启动串口发送
  UART2_SendPtr = UART2_SendBuf;
  USART_SendData(USART2, *UART2_SendPtr);
}

void USART1_IRQHandler(void)
{
	if(USART_GetITStatus(USART1,USART_IT_RXNE) != RESET)
	{
		uint8_t data;       
		
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);	


		data = (uint8_t)USART_ReceiveData(USART1);

        
        if(ENABLE == UartFrameHeadCheckEn)
		{
            //1.连续2个0xaa
            //2.上一次的数据不是0xa5
            //3.检测到帧头后，使能检测帧尾
            if(0xAA == data && 0xaa == FrameHeadDataBuf)
            {
                FlagUartFrameHead = Bit_SET;
                FlagUartFrameTail = Bit_RESET;
                UartFrameHeadCheckEn = DISABLE;
                UartFrameTailCheckEn = ENABLE;
                FrameHeadDataBuf = 0;
                FrameTailDataBuf = 0;
                
                UART_RcvPtr = 0;
                UART_RcvBytes = 0;
            }
            else
            {
                //如果上一个数据是0xa5,清零 否则存储该值
                if(FrameHeadDataBuf != 0xA5)
                    FrameHeadDataBuf = data;
                else
                    FrameHeadDataBuf = 0;
            }
		}
		
		
		UartTimerCnt = 0;
		UartMaterLost = RESET;

		UART_RcvBuf[UART_RcvPtr] = data;
		UART_RcvPtr++;
		UART_RcvBytes++;
        
        if(ENABLE == UartFrameTailCheckEn)
		{
            //1.连续2个0x55
            //2.上一次字节不是0xa5
            //3.检测到帧尾认为检测到一个完整的帧
            //4.检测到帧尾后，使能帧头检测，失能帧尾检测
            if(0x55 == data && 0x55 == FrameTailDataBuf)
            {
                FlagUartFrameTail = Bit_SET;
                FlagUartFrameHead = Bit_RESET;
                UartFrameHeadCheckEn = ENABLE;
                UartFrameTailCheckEn = DISABLE;
                FrameTailDataBuf = 0;
                FrameHeadDataBuf = 0;
                
                UartRcvOneFrameData = ENABLE;
                
            }
            else
            {
                //若上一字节不为0xa5 更新FrameTailDataBuf,否则清零
                if(0xa5 == FrameTailDataBuf)
                    FrameTailDataBuf = 0;
                else
                    FrameTailDataBuf = data;
            }
        }
        
	}

	if(USART_GetITStatus(USART1,USART_IT_TC) != RESET)
	{
		USART_ClearITPendingBit(USART1, USART_IT_TC);
		
		UART_SendPtr++;
		
		if(--UART_SendBytes > 0)
		{
			USART_SendData(USART1, *UART_SendPtr);
		}
		else
		{
			UART_SendOver = ENABLE;
		}
        
	}
}


#define UART2_REV_BUF   40   
float a[3],w[3],angle[3],T;
uint8_t Uart2_RevBytes, Uart2_RevPtr, Uart2_RevBuf[UART2_REV_BUF], Uart2_TimeCnt, Uart2_RevOneFrameData;

void USART2_IRQHandler(void)
{
	if(USART_GetITStatus(USART2,USART_IT_RXNE) != RESET)
	{
		uint8_t data;
		
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);	

		data = (uint8_t)USART_ReceiveData(USART2);
        
        Uart2_RevBuf[Uart2_RevPtr] = data;
        Uart2_RevPtr++;
        Uart2_RevBytes++;
        Uart2_TimeCnt = 0;		
	}

	if((USART_GetITStatus(USART2,USART_IT_TC) != RESET))
	{
		USART_ClearITPendingBit(USART2, USART_IT_TC);
		
		UART2_SendPtr++;
		
		if(--UART2_SendBytes > 0)
		{
			USART_SendData(USART2, *UART2_SendPtr);
		}
		else
		{
			UART2_SendOver = ENABLE;
		}
	}
}


// 语音串口控制策略
// 1. 定时连续查询播放状态
// 2. VoiceCur.Req = 1 , 发送状态查询+控制指令
// 3. VoiceCur.Req = 0 , 只发送状态查询指令
// 4. 控制指令先发送，状态查询指令后发送
void UART2_SendData(void)
{
	uint8_t i;
	uint8_t CHK;
	T_UART2_SendFarme* SendBufPtr;


	//定时发送时间是否到
	if(!Uart2_SendEn)
		return;

//	if(!UART2_SendOver)
//		return;
    
	Uart2_SendEn = DISABLE;  
    
//    if(VoiceCur.Req)
//    {
//        VoiceCur.Req = 0;
//        
//        SendBufPtr = (T_UART2_SendFarme*)UART2_SendBuf;
//        SendBufPtr->SOI = 0xAA;
//        SendBufPtr->CMD = VoiceCur.Cmd;
//        SendBufPtr->DataLength = VoiceCur.DataLength;
//        
//        for(i=0;i<VoiceCur.DataLength;i++)
//        {
//            SendBufPtr->Data[i] = VoiceCur.Data[i];
//        }
//        
//        //校验和
//        CHK=0;
//        for(i=0;i<SendBufPtr->DataLength + 3;i++)
//        {
//            CHK += UART2_SendBuf[i];
//        }
//        
//        UART2_SendBuf[i] = CHK;
//        
//        //状态查询指令
//        UART2_SendBuf[i + 1] = 0xAA;
//        UART2_SendBuf[i + 2] = 0x01;
//        UART2_SendBuf[i + 3] = 0x00;
//        UART2_SendBuf[i + 4] = 0xab;
//        
//        UART2_SendBytes = VoiceCur.DataLength + 8;
//        
//    }
//    else
    {
        SendBufPtr = (T_UART2_SendFarme*)UART2_SendBuf;
        SendBufPtr->SOI = 0xAA;
        SendBufPtr->CMD = 0x01;//VoiceCur.Cmd;
        SendBufPtr->DataLength = 38;//0;//
        
//        for(i=0;i<VoiceCur.DataLength;i++)
//        {
//            SendBufPtr->Data[i] = VoiceCur.Data[i];
//        }
        
        SendBufPtr->Data[0] = vl53l0x_Results >> 8;
        SendBufPtr->Data[1] = vl53l0x_Results;
        SendBufPtr->Data[2] = PathPlanStep;
        SendBufPtr->Data[3] = RobitRunbyRouteMode;
        SendBufPtr->Data[4] = LeftBump; 
        SendBufPtr->Data[5] = RightBump;
        SendBufPtr->Data[6] = RightMotor.CurSpeed >> 8;
        SendBufPtr->Data[7] = RightMotor.CurSpeed;
        SendBufPtr->Data[8] = LeftMotor.CurSpeed >> 8;
        SendBufPtr->Data[9] = LeftMotor.CurSpeed;
        SendBufPtr->Data[10] = Sys_AnglexInit >> 8;
        SendBufPtr->Data[11] = Sys_AnglexInit;
        SendBufPtr->Data[12] = earthWaveMin >> 8;
        SendBufPtr->Data[13] = earthWaveMin >> 0;
        SendBufPtr->Data[14] = earthWaveMax >> 8;
        SendBufPtr->Data[15] = earthWaveMax;
        SendBufPtr->Data[16] = RightMotorSwDirFlag;
        SendBufPtr->Data[17] = test2 >> 8;
        SendBufPtr->Data[18] = test2;
        SendBufPtr->Data[19] = Sys_Angle2 >> 8;
        SendBufPtr->Data[20] = Sys_Angle2 ;
        SendBufPtr->Data[21] = CarpetStatus;
        SendBufPtr->Data[22] = IcmDataByKalman.Gyro_x >> 8;
        SendBufPtr->Data[23] = IcmDataByKalman.Gyro_x;
        SendBufPtr->Data[24] = earthAvg >> 8;
        SendBufPtr->Data[25] = earthAvg;
        SendBufPtr->Data[26] = earthAvgOk;
        SendBufPtr->Data[27] = ADC_ConvertedValue_Val[3] >> 8;
        SendBufPtr->Data[28] = ADC_ConvertedValue_Val[3];
        SendBufPtr->Data[29] = RightMotor.PWMVal >> 8;
        SendBufPtr->Data[30] = RightMotor.PWMVal;
        SendBufPtr->Data[31] = LeftMotor.PWMVal >> 8;
        SendBufPtr->Data[32] = LeftMotor.PWMVal;
        SendBufPtr->Data[33] = ADC_ConvertedValue_Val[0] >> 8;
        SendBufPtr->Data[34] = ADC_ConvertedValue_Val[0];
        SendBufPtr->Data[35] = ADC_ConvertedValue_Val[1] >> 8;
        SendBufPtr->Data[36] = ADC_ConvertedValue_Val[1];
        SendBufPtr->Data[37] = earthWaveCnt;
     
        CHK=0;
        
        for(i=0;i<SendBufPtr->DataLength + 3;i++)
        {
            CHK += UART2_SendBuf[i];
        }
        
        UART2_SendBuf[i] = CHK;
    //	UART2_SendBuf[i+1] = UART_END_BYTE;
        
        UART2_SendBytes = SendBufPtr->DataLength + 4;
    }
    
	Uart2_Send_Start();
//	Uart2SendCnt++;
	UART2_SendOver = DISABLE;
}

void UART_ResponseMaster(void)
{
	uint8_t i;
	uint8_t CHK;
	T_UART_SendFarme* SendBufPtr;
    int32_t lineSpeed, angleRate;
    int16_t angle;
    int16_t pitch, roll;


	//定时发送时间是否到
	if(!Uart_SendEn)
		return;

	if(!UART_SendOver)
		return;
	
	Uart_SendEn = DISABLE;  

	SendBufPtr = (T_UART_SendFarme*)UART_SendBuf;
	SendBufPtr->SOI[0] = UART_FIRST_BYTE;
    SendBufPtr->SOI[1] = UART_FIRST_BYTE;
	SendBufPtr->DeviceAddr = DEVICE_ADDRESS;
	SendBufPtr->CMD = 0x03 | 0x80;
    SendBufPtr->DataAddr = 0x00;
    SendBufPtr->DataLength = DATA_LENGTH;

	//超声波传感器
    for(i = 0; i < 8; i++)
    {
        //超声波传感器数据 int16_t * 8 = 16Bit  
        SendBufPtr->Data.ultrasound[i] = 0;
    }
	
    //防跌落传感器
    SendBufPtr->Data.dropSensor = (EarthData[1].OK) << 1 | EarthData[0].OK;
    
    //红外传感器
//    SendBufPtr->Data.irSensor =  (WallData[4].OK) << 4 | (WallData[3].OK) << 3 | (WallData[2].OK) << 2 | (WallData[1].OK) << 1 | WallData[0].OK;
    
    //红外传感器- 将超声波传感器作为一个
    SendBufPtr->Data.irSensor = UltraDecelerationObs;//UltraObs;
    
    //碰撞传感器
    //在执行后退的过程中 保持碰撞传感器的状态
    if(BumpFlag || !BumpOverFlag)
    {
        SendBufPtr->Data.collisionSensor = BumpBak;
    }
    else
    {
        SendBufPtr->Data.collisionSensor = RightBump << 1 | LeftBump; //LeftBump
    }
    //增加轻质障碍物检测触发功能，UltraObs = 1 时，上报给算法板的碰撞状态全部置1
    if(UltraObs)
    {
        SendBufPtr->Data.collisionSensor |= 0x03;
    }   
    
    //线速度 角速度 角度 单位转换
    lineSpeed = (int32_t)((float)RobotLineSpeed * RPM2MMPERSEC); //r/min -> mm/s  RPM2MMPERSEC
    angleRate = (int32_t)((float)(-Sys_Anglerate) * 0.1745);   // 0.01du/s -> m弧度/s
    angle = (int16_t)((float)(-Sys_Angle) * 0.1745);           //0.01du -> m弧度
    
    //当前航向角
    SendBufPtr->Data.angularPos = angle;
    
    //左轮累计里程
	SendBufPtr->Data.leftEncoderPos = (int32_t)((float)LeftMileageAll * MILEAGE2MM);//(WallData[0].Delt);//

	//右轮累计里程
	SendBufPtr->Data.rightEncoderPos = (int32_t)((float)RightMileageAll * MILEAGE2MM);//(WallData[0].Delt);//
    
    //线速度
	SendBufPtr->Data.lineVelocity = lineSpeed;//(WallData[0].Delt);//
    
    //角速度
	SendBufPtr->Data.angularVelocity = angleRate;//(WallData[0].Delt);//
    
    //充电载体状态
    SendBufPtr->Data.chargeStatus = BatteryStatus;
    //剩余电量
    SendBufPtr->Data.batteryStatus = BatterySocDis;
    
    //抱起状态寄存器
    SendBufPtr->Data.pickupStatus = 0;//WheelLift;

	//错误寄存器
    SendBufPtr->Data.errorState = 0x00;//0x00;
    
    //沿墙传感器距离传感器 单位mm
    SendBufPtr->Data.wallDistance = PSD_Distance;//ADC_ConvertedValue_Val[7];
    
    //调试预留寄存器
    SendBufPtr->Data.feedbackStatus = StepStatus | GarbageStatus;
      
    //IMU 相关
    pitch = ((int32_t)((float)IcmDataByKalman.Gyro_x * GYRO_RANGE * 17.45) >> 16);
    roll = ((int32_t)((float)IcmDataByKalman.Gyro_y * GYRO_RANGE * 17.45) >> 16);
    
    SendBufPtr->Data.gyro_pitch = pitch;//LeftMotor.SpeedAdjAll;
    SendBufPtr->Data.gyro_roll = roll;//LeftMotor.BaseSpeed;
    SendBufPtr->Data.gyro_yaw = angleRate;//LeftMotor.TargSpeed;
    SendBufPtr->Data.accel_pitch = IcmDataByKalman.Acc_x;//RightMotor.SpeedAdjAll;
    SendBufPtr->Data.accel_roll = IcmDataByKalman.Acc_y;//RightMotor.BaseSpeed;
    SendBufPtr->Data.accel_vert = IcmDataByKalman.Acc_z;//RightMotor.TargSpeed;//16000
    
       
    //预留寄存器
    SendBufPtr->Data.reservedl = 0;
    
    //各执行机构电流
    SendBufPtr->Data.wheelCurrentL = 0; //左轮电流
    SendBufPtr->Data.wheelCurrentR = 0;
    SendBufPtr->Data.sideBroomCurrentL = 0;
    SendBufPtr->Data.sideBroomCurrentR = 0;
    SendBufPtr->Data.midBroomCurrent = 0;
    SendBufPtr->Data.fanSpeed = (DustSpeedMin + (DustPwmOn * DustPwm2Speed_K)) * DustEn;
    
    //预留寄存器
    SendBufPtr->Data.reserved2[0] = 0;
    SendBufPtr->Data.reserved2[1] = 0;
    SendBufPtr->Data.reserved2[2] = 0;
    SendBufPtr->Data.reserved2[3] = 0;
    
    //沿墙状态寄存器
    if(0 == PathPlanMode)
    { 
        SendBufPtr->Data.alongWallState = 0;
    }
    else
    {
        SendBufPtr->Data.alongWallState = 1; 
    }
    
    //左右轮速度控制量
    if(MOTOR_RUN_DIR_ZW == LeftMotor.Dir)
    {
        SendBufPtr->Data.l_speed = LeftMotor.CurSpeed;        
        SendBufPtr->Data.lpwm = LeftMotor.PWMVal;
    }
    else if(MOTOR_RUN_DIR_FW == LeftMotor.Dir)
    {
        SendBufPtr->Data.l_speed = -(int32_t)LeftMotor.CurSpeed;
        SendBufPtr->Data.lpwm = -(int32_t)LeftMotor.PWMVal;
    }
    
    if(MOTOR_RUN_DIR_ZW == RightMotor.Dir)
    {
        SendBufPtr->Data.r_speed = RightMotor.CurSpeed;
        SendBufPtr->Data.rpwm = RightMotor.PWMVal;       
    }
    else if(MOTOR_RUN_DIR_FW == RightMotor.Dir)
    {
        SendBufPtr->Data.r_speed = -(int32_t)RightMotor.CurSpeed;
        SendBufPtr->Data.rpwm = -(int32_t)RightMotor.PWMVal; 
    }
    
    //预留寄存器
    SendBufPtr->Data.reserved3[0] = 0;
    SendBufPtr->Data.reserved3[1] = 0;
    SendBufPtr->Data.reserved3[2] = 0;
    
    //电池相关参数
    SendBufPtr->Data.real_battery_level = BatterySocDis;
    SendBufPtr->Data.battery_voltage = BatteryVolByKalman * BATTERY_AD2mV_K; //14400;//
    
    //打滑异常参数 = 里程计角速度-IMU角速度(Z轴)
    
    
    //计算里程计角速度 
//    if(LeftMotor.Status != SYS_STOPING && RightMotor.Status != SYS_STOPING)
        AngularVelocityOdo = CalOdoAngularVelocity(SendBufPtr->Data.l_speed, SendBufPtr->Data.r_speed);
    //计算偏差
    SendBufPtr->Data.skid_error = AngularVelocityOdo - angleRate;
    
    //预留寄存器
    for(i = 0; i < 8;i++)
    {
        SendBufPtr->Data.reserved4[i] = 0;
    }
    
    SendBufPtr->ErrorCode = 0x00;   
    UART_SendBytes = SendBufPtr->DataLength + 11;
    
    CHK=0;
    for(i = 2;i < UART_SendBytes;i++)
	{
		CHK += UART_SendBuf[i];   
	}
	
	SendBufPtr->CHK = CHK;
//    SendBufPtr->CHK = 0x55;
    UART_SendBytes ++;
    
    //data & chk 添加控制符
    for(i = 10; i < UART_SendBytes; i++)
    {
        //在 0xaa 0x55 0xa5 前添加0xa5
        if(UART_SendBuf[i] == 0xaa || UART_SendBuf[i] == 0x55 || UART_SendBuf[i] == 0xa5)
        {
            add(UART_SendBuf, UART_SendBytes, i);
            UART_SendBytes++;
            i++;
        }
    }
   

	UART_SendBuf[i] = UART_END_BYTE;
	UART_SendBuf[i+1] = UART_END_BYTE;
    
	UART_SendBytes += 2;

	Uart_Send_Start();
	UART_SendOver = DISABLE;
}


/*****************************************************************************
 函 数 名  : UART_SendSignal
 功能描述  : 下位机主动上传传感器数据，当传感器状态变化式
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 

1.本函数在每次检测传感器信号后调用，1ms调用一次
 
 修改历史      :
  1.日    期   : 2016年4月13日
    作    者   : 田宏图
    修改内容   : 新生成函数

*****************************************************************************/
void UART_SendSignal(void)
{
	T_UART_SendFarme_A * SendBufPtr;
	uint8_t CHK,i,j;

//	SendBufPtr = (T_UART_SendFarme*)UART_SendBuf;
//	SendBufPtr->SOI = UART_FIRST_BYTE;
    for(i = 0; i < 7; i++)
    {
        if(UartAck[i].En && UART_SendOver)
        {         
            uint8_t length;
            UartAck[i].En = DISABLE;
            
            SendBufPtr = (T_UART_SendFarme_A*)UART_SendBuf;
            SendBufPtr->SOI[0] = UART_FIRST_BYTE;
            SendBufPtr->SOI[1] = UART_FIRST_BYTE;
            SendBufPtr->DeviceAddr = UartAck[i].DeviceAddr;
//            SendBufPtr->CMD = UartAck[i].Cmd | 0x80;
            SendBufPtr->DataAddr = UartAck[i].DataAddr;
            SendBufPtr->DataLength = UartAck[i].DataLength;
            
            // i=1 data1 的数据长度是非标准的         
            if(i == 1)
            {
                length = 24;
                SendBufPtr->CMD = UartAck[i].Cmd;
            }
            else
            {
                length = UartAck[i].DataLength;
                SendBufPtr->CMD = UartAck[i].Cmd | 0x80;
            }
            
            for(j = 0; j < length; j++)
                SendBufPtr->Data[j] = UartAck[i].Data[j];
            
            //data1 不需要加错误码，其余需要
            if(i != 1)
            {
                SendBufPtr->Data[j] = 0x00;
                UART_SendBytes = length + 11;
            }
            else
            {
                UART_SendBytes = length + 10;
            }
            
            CHK=0;        
            
            for(j = 2;j < UART_SendBytes;j++)
            {
                CHK += UART_SendBuf[j];   
            }
            UART_SendBuf[j] = CHK;
            UART_SendBytes++;
            
            for(j = 10; j < UART_SendBytes; j++)
            {
                //在 0xaa 0x55 0xa5 前添加0xa5
                if(UART_SendBuf[j] == 0xaa || UART_SendBuf[j] == 0x55 || UART_SendBuf[j] == 0xa5)
                {
                    add(UART_SendBuf, UART_SendBytes, j);
                    UART_SendBytes++;
                    j++;
                }
            }
                       

            UART_SendBuf[j] = UART_END_BYTE;
            UART_SendBuf[j+1] = UART_END_BYTE;
            
            UART_SendBytes += 2;

            Uart_Send_Start();
            UART_SendOver = DISABLE;
            break;
        }
    }
    
    if(UartAck_P.En && UART_SendOver)
    {
        UartAck_P.En = DISABLE;
        
        SendBufPtr = (T_UART_SendFarme_A*)UART_SendBuf;
        SendBufPtr->SOI[0] = UART_FIRST_BYTE;
        SendBufPtr->SOI[1] = UART_FIRST_BYTE;
        SendBufPtr->DeviceAddr = DEVICE_ADDRESS;
        SendBufPtr->CMD = UartAck_P.Cmd | 0x80;
        SendBufPtr->DataAddr = UartAck_P.DataAddr;
        SendBufPtr->DataLength = UartAck_P.DataLength;
        
        for(i = 0; i < UartAck_P.DataLength; i++)
            SendBufPtr->Data[i] = UartAck_P.Data[i];
        
        //增加错误码
        SendBufPtr->Data[i] = 0x00;
        
        UART_SendBytes = UartAck_P.DataLength + 11;
        
        CHK=0;
        
        for(i = 2;i < UART_SendBytes;i++)
        {
            CHK += UART_SendBuf[i];   
        }
        UART_SendBuf[i] = CHK;
        UART_SendBytes++;
        
        for(i = 10; i < UART_SendBytes; i++)
        {
            //在 0xaa 0x55 0xa5 前添加0xa5
            if(UART_SendBuf[i] == 0xaa || UART_SendBuf[i] == 0x55 || UART_SendBuf[i] == 0xa5)
            {
                add(UART_SendBuf, UART_SendBytes, i);
                UART_SendBytes++;
                i++;
            }
        }
        
        UART_SendBuf[i] = UART_END_BYTE;
        UART_SendBuf[i+1] = UART_END_BYTE;
        
        UART_SendBytes += 2;

        Uart_Send_Start();
        UART_SendOver = DISABLE;
    }
   
}

/*****************************************************************************
 函 数 名  : UartTimerJudge
 功能描述  : 串口接收判定
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
   1、每帧数据的间隔为50mS,如果1帧时间为40ms，则最多可接收38字节的数据长度.
   2、超过200ms没收到上位机信息，认为上位机离线，则采取停机处理或其它的处理。
   3、本函数每1毫秒调用一次。
 修改历史      :
  1.日    期   : 2016年3月31日
    作    者   : 田宏图
    修改内容   : 新生成函数

*****************************************************************************/
void UartTimerJudge(void)
{ 
  if(UartTimerCnt < 0xFFFF)
  {
    UartTimerCnt ++;

	if(UartTimerCnt >= 100)
    {
        FlagUartFrameTail = Bit_RESET;
        FlagUartFrameHead = Bit_RESET;
        UartFrameHeadCheckEn = ENABLE;
        UartFrameTailCheckEn = DISABLE;
        FrameHeadDataBuf = 0;
        FrameTailDataBuf = 0;
    }
    
    //超时判定，超过1S没有接收到指令 停止机器
    if(UartTimerCnt >= 1000)
    {
        UartMaterLost = SET;
    }
    
//	if(UART_RcvBytes >= 7)
//	{
//		if(UART_RcvBuf[2] == (UART_RcvBytes - 3))
//		{
//			//认为接收到1帧数据
//			UartRcvOneFrameData = ENABLE;
//		}
//	}
	
  }
  
    if(Uart2_TimeCnt < 0xff)
    {
        Uart2_TimeCnt++;
        if(Uart2_TimeCnt > 10)
        {
            if(Uart2_RevBytes > 0)
            {
                //接收到一帧数据
                Uart2_RevOneFrameData = 1;
            }
            //Uart2_RevBytes = 0;
            Uart2_RevPtr = 0;
        }
    }
}

/*****************************************************************************
 函 数 名  : Uart2_RcvData_Proc
 功能描述  : 串口接收到一帧数据后对它进行处理
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
  1、判定数据的合法性(帧头是否正确，数据长度是否吻合，校验和是否正确)
  2、合法程序后续处理。(清除相关标志，响应上位机命令)
 
 修改历史      :
  1.日    期   : 2016年3月31日
    作    者   : 田宏图
    修改内容   : 新生成函数

*****************************************************************************/
void Uart2_RcvData_Proc(void)
{
    uint8_t i, chk;
    T_UART2_SendFarme* RcvDataFrame;
    if(!Uart2_RevOneFrameData)
    {
        return;
    }

    Uart2_RevOneFrameData = 0;
    
    RcvDataFrame = (T_UART2_SendFarme *)Uart2_RevBuf;
    
    //帧头
    if(RcvDataFrame->SOI != 0xaa)
    {
        Uart2_RevBytes = 0;
        return;
    }
    
    //校验和
    chk = 0;
    for(i = 0; i < RcvDataFrame->DataLength + 3; i++)
    {
        chk += Uart2_RevBuf[i];
    }
    
    if(chk != Uart2_RevBuf[i])
    {
        Uart2_RevBytes = 0;
        return;
    }
    
    VoiceHandshake = 1;
    switch(RcvDataFrame->CMD)
    {
        case PLAYSTATUS:
            if(RcvDataFrame->DataLength > 0)
            {
                VoiceStatus = RcvDataFrame->Data[0];
            }
            break;
        default:
            break;
    }
    Uart2_RevBytes = 0;
}

/*****************************************************************************
 函 数 名  : handshakepresec
 功能描述  : 握手程序 每秒执行一次
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : HandshakeData2
 被调函数  : 
*****************************************************************************/
void handshakepresec(void)
{
    if(!HandShakeEn)
        return;
    HandShakeEn = DISABLE;
    
    if(HandshakeData2((uint8_t *)(&UartAck[2])))
    {
        UartAck[2].En = ENABLE;
    }
    else
    {   
        UartAck[2].En = DISABLE;
    }
    
    //借用握手定时，1S查询一次算法板的清扫状态，用于回充    
    MCU_Check_CheanPackStatus(CHECK_CLEAN_STATUS);
}

/*****************************************************************************
 函 数 名  : Uart_RcvData_Proc
 功能描述  : 串口接收到一帧数据后对它进行处理
 输入参数  : void  
 输出参数  : 无
 返 回 值  : 
 调用函数  : 
 被调函数  : 
  1、判定数据的合法性(帧头是否正确，数据长度是否吻合，校验和是否正确)
  2、合法程序后续处理。(清除相关标志，响应上位机命令)
 
 修改历史      :
  1.日    期   : 2016年3月31日
    作    者   : 田宏图
    修改内容   : 新生成函数

*****************************************************************************/
void Uart_RcvData_Proc(void)
{
  T_UART_RcvFarme* RcvDataFrame;
  uint8_t TmpData_c,i;
    
    //主机离线
    if(UartMaterLost)
    {
        //停止工作

        if(BumpFlag || !BumpOverFlag)
            return;
//        SetRobitSpeed(0, 0);
//        PathPlanMode = 0;
//        PathPlanStep = 0;
//        StepModeOn = 0;
//        
//        RobotLineSpeedTargPre = 0;
//        RobotAngleRatePre = 0;

        return;
        
    }
	  
  if(!UartRcvOneFrameData) return;
  UartRcvOneFrameData = DISABLE;
  
  //用结构体访问缓冲区
  RcvDataFrame = (T_UART_RcvFarme*)UART_RcvBuf;
  
  //判定起始字节是否正确
  if(RcvDataFrame->SOI != UART_FIRST_BYTE)
  {
    goto ErrorExit;
  }
  
  //删除接收数据中的无效0xa5
  for(i = 1; i < UART_RcvBytes - 2;i++)
  {
    if(UART_RcvBuf[i] == 0xa5)
    {
        //删除掉数组中的这个0xa5
        remove(UART_RcvBuf, UART_RcvBytes, i);
        UART_RcvBytes--;
        
        //0xa5的下一个字节是受保护的字节，不需要判断
        i++;
    }
  }
  
  //判定校验和是否正确
  TmpData_c = 0;
  for(i = 1 ; i <= UART_RcvBytes - 4; i++)
  {
    TmpData_c += UART_RcvBuf[i];
  }
  if(TmpData_c != UART_RcvBuf[i])
  {
    goto ErrorExit;
  }

  //判断帧尾是否正确
  if(UART_RcvBuf[UART_RcvBytes - 1] != UART_END_BYTE)
  {
	goto ErrorExit;
  }
  
  //设备地址是否正确
  if(RcvDataFrame->DeviceAddr == DEVICE_ADDRESS)
  {  
      //数据处理 
      switch(RcvDataFrame->CMD)
      {
        case 0x01:
            {
                int32_t lineSpeed, angleRate;
                //读取控制寄存器组
                UartAck[0].En = ENABLE;
                UartAck[0].Cmd = 0x01;
                UartAck[0].DeviceAddr = DEVICE_ADDRESS;
                UartAck[0].DataAddr = RcvDataFrame->DataAddr;
                UartAck[0].DataLength = RcvDataFrame->DataLength;
            
                //线速度和角速度单位变换
    //            lineSpeed = RobotLineSpeed * 4 / 15 / 4.707;  //r/min -> mm/s
                lineSpeed = (int32_t)((float)(RobotLineSpeed) * RPM2MMPERSEC); //r/min -> mm/s
                angleRate = (int32_t)((float)(-Sys_Anglerate) * 0.1745);   // 0.01du/s -> m弧度/s
                
                if(UartAck[0].DataAddr + UartAck[0].DataLength <= 8)
                {
                    uint8_t j;
                    for( j = UartAck[0].DataAddr; j < UartAck[0].DataLength + UartAck[0].DataAddr; j++)
                    {
                        if(j < 4)
                            UartAck[0].Data[j - UartAck[0].DataAddr] = (uint8_t)(lineSpeed >> ((3 - j) * 8));
                        else
                            UartAck[0].Data[j - UartAck[0].DataAddr] = (uint8_t)(angleRate >> ((7 - j) * 8));
                    }
                }            
            }
            break;
        case MCU_CONTROL_REG:
            //写入控制寄存器组
            
            // 写入成功，执行操作
            if(RegisterWrite(&ControlReg, RcvDataFrame->DataAddr, RcvDataFrame->DataLength, &(RcvDataFrame->Data[0])))
            {
                //单位转换
                RobotLineSpeedTarg = (int32_t)((float)(ControlReg.Reg.lineVelocity) * MMPERS2RPM);
                RobotAngleRate = (int32_t)((float)(-ControlReg.Reg.angularVelocity) * 5.73);
                AngleTarg = (int32_t)((float)(-ControlReg.Reg.stepPhi) * 5.73);
                
                //更改风扇功率
                if(fanLevelPre != ControlReg.Reg.fanLevel)
                {
                    //fanLevelPre = 0 则启动吸尘
                    if(0 == fanLevelPre)
                    {
                        DustEn = ENABLE; 
                        MopPar.MopEn = ENABLE;                        
                    }
                    
                    if(ControlReg.Reg.fanLevel < 80)
                    {
                        DustPwmOn = 10;
                    }
                    else
                    {
                        DustPwmOn = TIM3_Period / 2;
                    }
                    
                    //fanLevel = 0 关闭吸尘
                    if(0 == ControlReg.Reg.fanLevel)
                    {
                        DustEn = DISABLE;
                        MopPar.MopEn = DISABLE;
                    }
                    
                    //更新
                    fanLevelPre = ControlReg.Reg.fanLevel;
                }
                
                //更改中刷功率
                if(midBroomLevelPre != ControlReg.Reg.midBroomLevel)
                {
                    //midBroomLevelPre = 0 启动中刷
                    if(0 == midBroomLevelPre)
                    {
                        CleanEn = ENABLE;
                    }
                    
                    //midBroomLevel = 0 关闭中刷
                    if(0 == ControlReg.Reg.midBroomLevel)
                    {
                        CleanEn = DISABLE;
                    }
                    
                    //更新
                    midBroomLevelPre = ControlReg.Reg.midBroomLevel;
                }
                
                //更改边刷功率
                if(sideBroomLevelPre != ControlReg.Reg.sideBroomLevel)
                {
                    //sideBroomLevelPre = 0, 启动边刷
                    if(0 == sideBroomLevelPre)
                    {

                    }
                    
                    //sideBroomLevel = 0 关闭边刷
                    if(0 == sideBroomLevelPre)
                    {

                    }
                    
                    sideBroomLevelPre = ControlReg.Reg.sideBroomLevel;
                }
                
                //碰撞发生时，不执行算法板指令
                if(BumpFlag || !BumpOverFlag)
                    return;
                
                switch(ControlReg.Reg.mode)
                {
                    case SPEED_MODE:
                        //角速度和线速度控制模式
                        
                        //1.模式切换时
                        //2.模式未切换但线速度和角速度其中任意一个发生改变
                        //以上两种情况执行重新设置角速度和线速度否则不执行
                        if(CtrlMode != SPEED_MODE || (CtrlMode == SPEED_MODE && (RobotAngleRatePre != RobotAngleRate || RobotLineSpeedTarg != RobotLineSpeedTargPre)))
                        {
                            SetRobitLineAnglerate(RobotLineSpeedTarg, RobotAngleRate);
                            PathPlanMode = 0;
                            StepModeOn = 0;
                            StepStatus = 0;
                            CtrlMode = SPEED_MODE;
                            
                            RightMotor.Vacc = ControlReg.Reg.Vacc * MMPERS2RPM / 200;
                            LeftMotor.Vacc = RightMotor.Vacc;
                            
                            RightMotor.Wacc = (int32_t)((float)(-ControlReg.Reg.Wacc) * 5.73 / 200);
                            LeftMotor.Wacc = RightMotor.Wacc;
                            
                            //加速度设置
                            if(RightMotor.Vacc)
                            {
                                VaccEn = ENABLE;
                                VaccInit = 0;
                            }
                            
                            //暂时屏蔽
    //                        if(RightMotor.Wacc)
    //                        {
    //                            WaccEn = ENABLE;
    //                            WaccInit = 0;
    //                        }
                            
                            
                            RobotAngleRatePre = RobotAngleRate;
                            RobotLineSpeedTargPre = RobotLineSpeedTarg;
                        }
                        break;
                    case ALONG_WALL_MODE:
                        //沿墙行走模式
                        PathPlanMode = 3;
    //                    PathPlanStep = 0;
                        StepModeOn = 0;
                        StepStatus = 0;
                        CtrlMode = ALONG_WALL_MODE;
                        break;
                    case STEP_MODE:
                        //STEP 模式
                        
                        //扫地机已经工作在Step模式时，再次接收到Step模式指令不执行
                        if(StepModeOn /*&& RobitStatus*/)
                        {
                            if(0 == RobitStatus)
                            {
                                //RightMotorSwDirFlag = 1 或 LeftMotorSwDirFlag = 1 说明Step还没有完成 
                                if(!(RightMotorSwDirFlag || LeftMotorSwDirFlag))
                                    StepStatus = 0;
                            }
                            goto ErrorExit;
                        }
                        
                        SetRobitVacc(0);
                        
                        StepModeOn = 1;
                        PathPlanMode = 0;
                        CtrlMode = STEP_MODE;
                        StepStatus = 1;

                        if(0 == AngleTarg  && ControlReg.Reg.stepS != 0)
                        {
                            ControlReg.Reg.stepS = fabs(ControlReg.Reg.stepS);
                            if(RobotLineSpeedTarg > 0)
                            {
                                SetRobitMileageRun(ControlReg.Reg.stepS, 1, (uint16_t)(fabs(ControlReg.Reg.lineVelocity)));
                            }
                            else if(RobotLineSpeedTarg < 0)
                            {                               
                                SetRobitMileageRun(ControlReg.Reg.stepS, 2, (uint16_t)(fabs(ControlReg.Reg.lineVelocity)));
                            }
                            else
                            {
                                //线速度为0 
                                SetRobitSpeed(0, 0);
                            }
                        }
                        else if(AngleTarg != 0 && ControlReg.Reg.stepS == 0)
                        {
                            AngleTarg = fabs(AngleTarg);
                            
                            
                            if(RobotAngleRate > 0)
                            {
                                //向右旋转
                                SetRobitAngleRun(AngleTarg / 100, 2, (uint16_t)RobotAngleRate / 100, 0);
                            }
                            else if(RobotAngleRate < 0)
                            {
                                //向左旋转
                                SetRobitAngleRun(AngleTarg / 100, 1, (uint16_t)(-RobotAngleRate) / 100, 0);
                            }
                            else
                            {
                                SetRobitSpeed(0, 0);
                            }                            
                        }
                        else if(AngleTarg != 0 && ControlReg.Reg.stepS != 0)
                        {
                            //画弧线
                            DrawArc(RobotLineSpeedTarg, RobotAngleRate, fabs(AngleTarg), fabs(ControlReg.Reg.stepS));
    //                        SetRobitLineAnglerate(RobotLineSpeedTarg, RobotAngleRate);
                        }
                        break;
                }
            }
            
            break;
        case MCU_STATUS_REG:
            //读取状态寄存器组 (正常情况该寄存器组是驱动板主动上传给算法板，不需要算法板发读取信号)
            break;
        case 0x04:
            //读取参数寄存器
            UartAck_P.Cmd = 0x04;   
            UartAck_P.DeviceAddr = DEVICE_ADDRESS;
            UartAck_P.DataAddr = RcvDataFrame->DataAddr;
            UartAck_P.DataLength = RcvDataFrame->DataLength;
            
            
            if(UartAck_P.DataAddr + UartAck_P.DataLength <= 224)
            {
                //调整高低位的位置
                uint8_t j;
                for( j = UartAck_P.DataAddr; j < UartAck_P.DataAddr + UartAck_P.DataLength; j++)
                {
                    UartAck_P.Data[j - UartAck_P.DataAddr] = RobotPar[j];
                }
            }
            
            break;
            
        case HAND_SHAKE:
            //握手信息
            if(HandshakeData1((uint8_t *)(&(RcvDataFrame->DeviceAddr)),(uint8_t *)(&UartAck[1])))
            {
                UartAck[1].En = ENABLE;
            }
            else
            {
                UartAck[1].En = DISABLE;
            }

            break;
        default:
            break;
      }
  }
  else if(RcvDataFrame->DeviceAddr == MCU_CLEANER_STATUS_ADDRESS)
  {
      //扫地机状态查询
      if(RcvDataFrame->CMD == MCU_CLEANER_STATUS_CODE_STATUS)
      {
          StatusRegisterWrite(&CleanStatus, RcvDataFrame->DataAddr, RcvDataFrame->DataLength, &(RcvDataFrame->Data[0]));
//          if(RcvDataFrame->DataAddr == 0 && RcvDataFrame->DataLength == 2)
//          {
//              CleanStatus.Reg.cleanStatus = RcvDataFrame->Data[0];
//              CleanStatus.Reg.cleanStatusSub = RcvDataFrame->Data[1];
//          }
//          else if(RcvDataFrame->DataAddr == 2 && RcvDataFrame->DataLength == 1)
//          {
//              CleanStatus.Reg.wifiStatus = RcvDataFrame->Data[0];
//          }
      }
  }
  ErrorExit:
    UART_RcvBytes = 0;
}



int32_t Length2Mileage(int32_t LengthInput)
{
	int32_t tmpmileage;

	//tmpmileage  = LengthInput * 4.707 = LengthInput * 301 /64
//	tmpmileage = LengthInput * 301;
//	tmpmileage = tmpmileage >> 6;
    tmpmileage = (int32_t)((float)LengthInput * LENGTH2MILEAGE);

	return (int32_t)tmpmileage;
}

int32_t Angle2Mileage(int32_t AngleInput)
{
	int32_t tmpmileage;


//   2016-4-19 系数更改 -> 11   需测试
//   2016-4-25  -> 10.9
//   2016-11-16 -> 11.3
	//tmpmileage  = AngleInput * 11.74 = AngleInput * 43 / 4
//	tmpmileage = AngleInput * 113;
//	tmpmileage = tmpmileage / 10;
    
    tmpmileage = (int32_t)((float)AngleInput * ANGLE2MILEAGE);

	return (int32_t)tmpmileage;
}

uint16_t ABSNum(int16_t InputNum)
{
	int16_t tmpnum;

	tmpnum = InputNum;

	if(tmpnum < 0)
	{
		tmpnum = 0 - InputNum; 
	}
	else
	{
		tmpnum = InputNum;	
	}

	return (uint16_t)tmpnum;
}

//Angle     : du
//Dir       : 1-left 2-right
//AngleRate : 0.01du/s
//SpinMode  : 0-轴心旋转 1-单轮前进旋转 2-单轮后退旋转
void SetRobitAngleRun(uint16_t Angle, uint8_t Dir, uint16_t AngleRate, uint8_t SpinMode)
{
	uint16_t TmpData_h,TmpSpeedU;
    int16_t SetSpeedL, SetSpeedR;
	uint8_t TmpDir,Tmpmode;
	int16_t lineSpeed, tmpSpeede;
    
	TmpData_h = Angle;
	TmpSpeedU = AngleRate;
	TmpDir = Dir;
	Tmpmode = SpinMode;
	
	if(Tmpmode > 2)
	{
		Tmpmode = 0;
	}
	
	//目标角度大于0
	if(TmpData_h > 0)
	{
		TargCirle = TmpData_h / 360;
		TargAngle = TmpData_h % 360;
		TargAngle *= 100;
		//角度单位转换 度 转换成脉冲个数
		TmpData_h = Angle2Mileage(TmpData_h);

		//角速度转换成 r/min
		TmpSpeedU = (uint16_t)(Angle2Mileage(TmpSpeedU) * 15) >> 2;//TENmDuPERSEC2RPM
        //TmpSpeedU = (uint16_t)((float)TmpSpeedU * TENmDuPERSEC2RPM);//
        
        if(Tmpmode)
        {
            TmpSpeedU <<= 1;
        }

		//速度范围限定
		if(TmpSpeedU > MOT_RPM_SPEED_MAX)
		{
			TmpSpeedU = MOT_RPM_SPEED_MAX;
		}
		else if(TmpSpeedU < MOT_RPM_SPEED_MIN)
		{
			TmpSpeedU = MOT_RPM_SPEED_MIN;
		}
		else
		{
			TmpSpeedU = TmpSpeedU;
		}
        
        
		
		//目标方向只有1和2     1向左 2 向右
		if(TmpDir == 1)
		{
			if(0 == Tmpmode)
			{
//				RightMotorEnterRun(0);
//				LeftMotorEnterRun(1);
                SetSpeedR = TmpSpeedU;
				SetSpeedL = (int16_t)TmpSpeedU;
				SetSpeedL = -SetSpeedL;
			}
			else if(1 == Tmpmode)
			{
//				RightMotorEnterRun(0);
                SetSpeedR = TmpSpeedU;
				SetSpeedL = 0;
			}		
			else if(2 == Tmpmode)
			{
//				LeftMotorEnterRun(1);
                SetSpeedL = (int16_t)TmpSpeedU;
				SetSpeedL = -SetSpeedL;
				SetSpeedR = 0;
			}
			
		}
		else if(TmpDir == 2)
		{
			if(0 == Tmpmode)
			{
//				RightMotorEnterRun(1);
//				LeftMotorEnterRun(0);
                SetSpeedL = TmpSpeedU;
				SetSpeedR = (int16_t)TmpSpeedU;
				SetSpeedR = -SetSpeedR;
			}
			else if(1 == Tmpmode)
			{
//				LeftMotorEnterRun(0);
                SetSpeedR = 0;
				SetSpeedL = TmpSpeedU;
			}
			else if(2 == Tmpmode)
			{
//				RightMotorEnterRun(1);
                
                SetSpeedR = (int16_t)TmpSpeedU;
				SetSpeedR = -SetSpeedR;
				SetSpeedL = 0;
			}
		}
		
		
		InitVar_PID(2);
		InitVar_PID(3);
//		AnglerateTarg = 0;
		Sys_Startangle = Sys_Angle;
		
		if(0 == Tmpmode)
		{
			TargMileage = TmpData_h;
		}
		else
		{
			TargMileage = TmpData_h << 1;
		}
		
//		LeftMotor.TargSpeed = TmpSpeedU;
//		RightMotor.TargSpeed = TmpSpeedU;
        SetRobitSpeed(SetSpeedL, SetSpeedR);
        
        
        lineSpeed = (SetSpeedL + SetSpeedR) / 2;
        tmpSpeede = (SetSpeedL - SetSpeedR) / 2;
        AnglerateTarg = (float)(tmpSpeede) * RPM2TENmDuPERDSEC;
        AnglerateTargInit = AnglerateTarg;
        if(lineSpeed >= 0 )
        {
            LeftMotor.BaseSpeed = lineSpeed;
            RightMotor.BaseSpeed = lineSpeed;
            RightMotor.SpeedAdjAll = -tmpSpeede;
            LeftMotor.SpeedAdjAll = tmpSpeede;
            Robit_Dir = 0;
        }
        else
        {
            LeftMotor.BaseSpeed = -lineSpeed;
            RightMotor.BaseSpeed = -lineSpeed;
            RightMotor.SpeedAdjAll = tmpSpeede;
            LeftMotor.SpeedAdjAll = -tmpSpeede;
            Robit_Dir = 1;
        }   
        
        
		LeftCurMileage = 0;
		RightCurMileage = 0;
        TargMileageForArc = 0;
        StepModeCnt = 0;

		RobitRouteModeEn = ENABLE;
		RobitStatus = 1;
		RobitRunningMode = 1;
		RobitRunningClosedLoopMode = 1;	
		RobitRunbyRouteMode = 1;
	}
}


// mileage  : mm
// dir      : 1 前进 2 后退
// RunSpeed ：mm/s
void SetRobitMileageRun(uint16_t Mileage, uint8_t Dir, uint16_t RunSpeed)
{
	uint16_t TmpData_h, TmpSpeedU;
    int16_t SetSpeed;
	uint8_t TmpDir;
	
	TmpData_h = Mileage;
	TmpSpeedU = RunSpeed;
	TmpDir = Dir;
	
	//目标里程大于0
	if(TmpData_h > 0)
	{
		//里程单位转换 mm 转换成脉冲个数
		TmpData_h = Length2Mileage(TmpData_h);

		//mm/s -> r/min
		TmpSpeedU = (Length2Mileage(TmpSpeedU) * 15) >> 2;

		//速度范围限定
		if(TmpSpeedU > MOT_RPM_SPEED_MAX)
		{
			TmpSpeedU = MOT_RPM_SPEED_MAX;
		}
		else if(TmpSpeedU < MOT_RPM_SPEED_MIN)
		{
			TmpSpeedU = MOT_RPM_SPEED_MIN;
		}
		else
		{
			TmpSpeedU = TmpSpeedU;
		}
	
		//目标方向只有1和2   1前进 2 后退
		if(TmpDir == 1)
		{
//			RightMotorEnterRun(0);
//			LeftMotorEnterRun(0);
            SetSpeed = (int16_t)TmpSpeedU;
            RightMotor.TargDir = MOTOR_RUN_DIR_ZW;
            LeftMotor.TargDir = MOTOR_RUN_DIR_ZW;
            Robit_Dir = 0;            
		}
		else if(TmpDir == 2)
		{
//			RightMotorEnterRun(1);
//			LeftMotorEnterRun(1);
            SetSpeed = (int16_t)TmpSpeedU;
			SetSpeed = -SetSpeed;
            RightMotor.TargDir = MOTOR_RUN_DIR_FW;
            LeftMotor.TargDir = MOTOR_RUN_DIR_FW;
            Robit_Dir = 1;
		}
        SetRobitSpeed(SetSpeed, SetSpeed);
        LeftMotor.BaseSpeed = TmpSpeedU;
        RightMotor.BaseSpeed = TmpSpeedU;
        RightMotor.SpeedAdjAll = 0;
        LeftMotor.SpeedAdjAll = 0;
        
		InitVar_PID(2);
		InitVar_PID(3);
		AnglerateTarg = 0;
		Sys_Startangle = Sys_Angle;
		TargMileage = TmpData_h;

		LeftCurMileage = 0;
		RightCurMileage = 0;

		RobitRouteModeEn = ENABLE;
		RobitStatus = 1;
		RobitRunningMode = 1;
		RobitRunningClosedLoopMode = 1;	
		RobitRunbyRouteMode = 0;
	}
}

// lineSpeed 线速度 r/min    角速度 (0.01度/s)
void SetRobitLineAnglerate(int16_t lineSpeed, int16_t angleRate)
{
    int16_t tmpSpeedL, tmpSpeedR, tmpSpeede;//, tmpSpeedemin;
    float tmphanglerate;
    
    //将0.01度/s转换为弧度/s 
//    tmphanglerate = (float)angleRate * 3.14 / 180;
//    
//    tmpSpeede = (int16_t)(tmphanglerate * ROBOT_WHEEL_AXIS / 2);
//    tmpSpeede = (Length2Mileage(tmpSpeede) * 15) >> 2;
    
    //
    //0.01du/s角速度转换成 r/min
	//tmpSpeede = (Angle2Mileage(angleRate) * 15 / 100) >> 2;
    tmphanglerate = angleRate;
    tmpSpeede = tmphanglerate * TENmDuPERSEC2RPM ;//(Angle2Mileage(angleRate) * 3 / 5) >> 4;
//    lineSpeed = (Length2Mileage(lineSpeed) * 15) >> 2;
    
    tmpSpeedL = lineSpeed + tmpSpeede;
    tmpSpeedR = lineSpeed - tmpSpeede;
    
    if(tmpSpeedL < SPEED_SET && tmpSpeedL > -SPEED_SET)
    {
        tmpSpeedL = 0;
    }
    
    if(tmpSpeedR < SPEED_SET && tmpSpeedR > -SPEED_SET)
    {
        tmpSpeedR = 0;
    }
       
    SetRobitSpeed(tmpSpeedL, tmpSpeedR); 

    // 根据左右轮速度的实际值更新速度设定值
    //然后更新线速度和角速度的实际设定值(实际设定值和原始设定值会有差异,是由左右轮的限速(700~6000)导致的)
    if(tmpSpeedL >= 0)
    {
        LeftMotor.TargDir = MOTOR_RUN_DIR_ZW;
        tmpSpeedL = LeftMotor.TargSpeed;
    }
    else
    {
        LeftMotor.TargDir = MOTOR_RUN_DIR_FW;
        tmpSpeedL = -LeftMotor.TargSpeed;
    }
    
    if(tmpSpeedR >= 0)
    {
        RightMotor.TargDir = MOTOR_RUN_DIR_ZW;
        tmpSpeedR = RightMotor.TargSpeed;
    }
    else
    {
        RightMotor.TargDir = MOTOR_RUN_DIR_FW;
        tmpSpeedR = -RightMotor.TargSpeed;
    }
    

    lineSpeed = (tmpSpeedL + tmpSpeedR) / 2;
    tmpSpeede = (tmpSpeedL - tmpSpeedR) / 2;
    
    
    //求当前线速度条件下的角速度的最小值
    //线速度在 -MOT_RPM_SPEED_MIN~MOT_RPM_SPEED_MIN 之间时，最小值不为0，其余皆为0
//    if((lineSpeed < MOT_RPM_SPEED_MIN && lineSpeed > -MOT_RPM_SPEED_MIN))
//    {
//        if(angleRate > 0 && lineSpeed < 0 )
//        {
//            tmpSpeedemin = lineSpeed + MOT_RPM_SPEED_MIN;
//        }
//        else if(angleRate < 0 && lineSpeed >= 0)
//        {
//            tmpSpeedemin = lineSpeed - MOT_RPM_SPEED_MIN;
//        }
//        else if(angleRate > 0 && lineSpeed >= 0 )
//        {
//            tmpSpeedemin = MOT_RPM_SPEED_MIN - lineSpeed;
//        }
//        else if(angleRate < 0 && lineSpeed < 0)
//        {
//            tmpSpeedemin = - lineSpeed - MOT_RPM_SPEED_MIN;
//        }
//    }
//    else if(lineSpeed > MOT_RPM_SPEED_MAX)
//    {
//        lineSpeed = MOT_RPM_SPEED_MAX;
//        tmpSpeedemin = 0;
//    }
//    else if(lineSpeed < -MOT_RPM_SPEED_MAX)
//    {
//        lineSpeed = -MOT_RPM_SPEED_MAX;
//        tmpSpeedemin = 0;
//    }
//    else
//    {
//        tmpSpeedemin = 0;
//    }
    
     
    
    if(lineSpeed >= 0 )
    {
        LeftMotor.BaseSpeed = lineSpeed;
        RightMotor.BaseSpeed = lineSpeed;
        RightMotor.SpeedAdjAll = -tmpSpeede;
        LeftMotor.SpeedAdjAll = tmpSpeede;
        Robit_Dir = 0;
    }
    else
    {
        LeftMotor.BaseSpeed = -lineSpeed;
        RightMotor.BaseSpeed = -lineSpeed;
        RightMotor.SpeedAdjAll = tmpSpeede;
        LeftMotor.SpeedAdjAll = -tmpSpeede;
        Robit_Dir = 1;
    }
    
    
    //将速度差转换为0.01度/s   系数的选取和轮子轮径 减速比 一圈栅格数 以及 轴距有关
//    tmphanglerate = (float)tmpSpeedemin * 4 / 15 / 11.3 * 100;
    tmphanglerate = (float)tmpSpeede * RPM2TENmDuPERDSEC ;
    
    
//    test2 = (uint16_t)tmphanglerate;
//    if(ABSNum(angleRate) >= ABSNum(tmphanglerate) || angleRate == 0)
//    {
//        AnglerateTarg = angleRate ;
//    }
//    else
//    {
        AnglerateTarg = (int16_t)tmphanglerate;
//    }

    
    InitVar_PID(2);
    InitVar_PID(3);
    Sys_Startangle = Sys_Angle;
    RobitRouteModeEn = ENABLE;
    RobitStatus = 1;
    RobitRunningMode = 1;
    RobitRunningClosedLoopMode = 1;	
    RobitRunbyRouteMode = 3;
    
}

//lineSpeed: PRM
//angleRate: (0.01度/s)
//angleTarg: 0.01度
//Mileage:   mm
void DrawArc(int16_t lineSpeed, int16_t angleRate, uint16_t angleTarg, uint32_t Mileage)
{   
    
    TargCirle = angleTarg / 36000;
    TargAngle = angleTarg - 36000 * TargCirle;
    
    //里程单位转换 mm 转换成脉冲个数
    TargMileageForArc = Length2Mileage(Mileage);
    
    //设置角速度和线速度
    SetRobitLineAnglerate(lineSpeed, angleRate);
    
    LeftCurMileage = 0;
    RightCurMileage = 0;
    StepModeCnt = 0;
    AnglerateTargInit = AnglerateTarg;
    RobitRunbyRouteMode = 1;
}

void SetRobitSpeed(int16_t LeftSpeed, int16_t RightSpeed)
{
	int16_t TmpLSpeed, TmpRSpeed;
	MOTOR_RUNDIR_TypeDef TmpLDir, TmpRDir;
	
	TmpLSpeed = LeftSpeed;
	TmpRSpeed = RightSpeed;	
	//任一模式下，写速度0扫地机都停止
	if(TmpLSpeed == 0 && TmpRSpeed == 0)
	{
		if(RobitStatus != 0 /*&& RobitRunningMode == 1 && RobitRunningClosedLoopMode == 0*/)
		{
			//RobitStatus = 0;
			RightMotorStop();
			LeftMotorStop();	

			LeftMotorSwDirFlag = RESET;
			RightMotorSwDirFlag = RESET;
			
		}
		return;
	}
	else
	{
		///左轮速度解析处理
		if(TmpLSpeed >= 0)
		{
			TmpLDir = MOTOR_RUN_DIR_ZW;
		}
		else if(TmpLSpeed < 0)
		{
			TmpLDir = MOTOR_RUN_DIR_FW;
		}	

		TmpLSpeed = (int16_t)ABSNum(TmpLSpeed);
		
		//mm/s -> 脉冲/s -> r/min
//		TmpLSpeed = (Length2Mileage(TmpLSpeed) * 15) >> 2;

		
		
		////右轮速度解析处理
		if(TmpRSpeed >= 0)
		{
			TmpRDir = MOTOR_RUN_DIR_ZW;
		}
		else if(TmpRSpeed < 0)
		{
			TmpRDir = MOTOR_RUN_DIR_FW;	
		}

		
		TmpRSpeed = (int16_t)ABSNum(TmpRSpeed);
		
//		TmpRSpeed = (Length2Mileage(TmpRSpeed) * 15) >> 2;


		
		///左轮运行状态处理
		if(TmpLSpeed == 0)
		{
			LeftMotorStop();
            LeftMotor.TargSpeed = 0;
            LeftMotorSwDirFlag = RESET;
		}
		else
		{
            //计算速度，对合法性进行判断
            if(TmpLSpeed > MOT_RPM_SPEED_MAX)
            {
                TmpLSpeed = MOT_RPM_SPEED_MAX;
            }
            else if(TmpLSpeed < MOT_RPM_SPEED_MIN)
            {
                TmpLSpeed = MOT_RPM_SPEED_MIN;
            }
            else
            {
                TmpLSpeed = TmpLSpeed;
            }
            
			////左轮目标方向与当前方向不同且不是停止状态  或者  左轮虽然目标方向与当前方向相同但出于正在停止状态(后面的情况不处理会导致左轮异常，不能启动)
			if((LeftMotor.Dir != TmpLDir && LeftMotor.Status != SYS_STOP) || (LeftMotor.Dir == TmpLDir && LeftMotor.Status == SYS_STOPING))
			{
				LeftMotorStop();	
				LeftMotorSwDirFlag = SET;
				LeftMotorSpeedBak = TmpLSpeed;
				LeftMotorDirBak = TmpLDir;
                LeftMotor.TargSpeed = TmpLSpeed;
				
				/////////20170216-1/////////
				////1.左右轮其中一个轮子需要改变方向，2个轮子都停止后，然后再重新启动////
				////2.上面已停止了左轮的运行/////////////////////////////////////////////
				////3.下面停止右轮的运行/////////////////////////////////////////////////
//				RightMotorStop();	
//				RightMotorSwDirFlag = SET;
//				RightMotorSpeedBak = TmpRSpeed;
//				RightMotorDirBak = TmpRDir;
				/////20170216-1 over////////
			}
			else
			{	
				//设定目标速度
				LeftMotor.TargSpeed = TmpLSpeed;
				
				//若轮子已停止，则启动
				if(LeftMotor.Status == SYS_STOP)
				{
					LeftMotorEnterRun(TmpLDir);					
				}
			}
		}

		//右轮运行状态处理
		if(TmpRSpeed == 0)
		{
			RightMotorStop();
            RightMotor.TargSpeed = 0;
		}
		else
		{	
			if(TmpRSpeed > MOT_RPM_SPEED_MAX)
            {
                TmpRSpeed = MOT_RPM_SPEED_MAX;
            }
            else if(TmpRSpeed < MOT_RPM_SPEED_MIN)
            {
                TmpRSpeed = MOT_RPM_SPEED_MIN;
            }
            else
            {
                TmpRSpeed = TmpRSpeed;
            }
            
            ////右轮目标方向与当前方向不同且不是停止状态  或者  右轮虽然目标方向与当前方向相同但出于正在停止状态(后面的情况不处理会导致右轮异常，不能启动)
			if((RightMotor.Dir != TmpRDir && RightMotor.Status != SYS_STOP)||(RightMotor.Dir == TmpRDir && RightMotor.Status == SYS_STOPING))
			{
				RightMotorStop();
				RightMotorSwDirFlag = SET;
				RightMotorSpeedBak = TmpRSpeed;
                RightMotor.TargSpeed = TmpRSpeed;
				RightMotorDirBak = TmpRDir;
				
				/////////20170216-2/////////
				////1.左右轮其中一个轮子需要改变方向，2个轮子都停止后，然后再重新启动////
				////2.上面已停止了右轮的运行/////////////////////////////////////////////
				////3.下面停止左轮的运行/////////////////////////////////////////////////
//				LeftMotorStop();	
//				LeftMotorSwDirFlag = SET;
//				LeftMotorSpeedBak = TmpLSpeed;
//				LeftMotorDirBak = TmpLDir;
				/////20170216-2 over////////
			}
			else
			{
				RightMotor.TargSpeed = TmpRSpeed;
				
				//如果处于停止中，则启动电机
				if(RightMotor.Status == SYS_STOP)
				{
					RightMotorEnterRun(TmpRDir);
				}
			}
		}
		
		
		//扫地机由停止-> 运行状态
		RobitStatus = 1;
		RobitRunningMode = 1;
		RobitRunningClosedLoopMode = 0;
		
	}
}

//删除数组中某个指定位置的元素
///数组指针 数组长度 需删除的元素在数组中的索引位置
void remove(uint8_t * data, uint8_t len, uint8_t idx)
{
    uint8_t i;
    len --;
    if(idx >= len)
    {
        return;
    }
    
    
    for(i = idx; i < len; i++)
    {
        data[i] = data[i + 1];
    }
}

//在数组的指定索引位置添加特定元素 0xa5 
//数组指针 数组长度 位置索引
void add(uint8_t * data, uint8_t len, uint8_t idx)
{
    uint8_t i;
    len--;
    
    if(idx > len)
    {
        return;
    }
    
    
    for(i = len ; i >= idx; i--)
    {
        data[i + 1] = data[i];
    }
    data[idx] = 0xa5;
}

/**
* @brief  执行写寄存器操作
* @param  reg       : 寄存器参数(首地址与尺寸，初始化时确定)
* @param  shiftaddr : 寄存器写入偏移地址
* @param  data_size : 数据长度
* @param  data      : 数据地址
* @retval 是否成功
*/
uint8_t RegisterWrite(RegisterType * reg, uint8_t shiftaddr, uint8_t data_size, uint8_t * data)
{
    if(shiftaddr >= reg->Reg_Length || shiftaddr + data_size > reg->Reg_Length)
    {
        return 0;
    }
    else
    {
        memcpy((uint8_t * )(&reg->Reg) + shiftaddr, (uint8_t *)(data), data_size);
        return 1;
    }
}

/**
* @brief  执行写寄存器操作
* @param  reg       : 寄存器参数(首地址与尺寸，初始化时确定)
* @param  shiftaddr : 寄存器写入偏移地址
* @param  data_size : 数据长度
* @param  data      : 数据地址
* @retval 是否成功
*/
uint8_t StatusRegisterWrite(StatusRegisterType * reg, uint8_t shiftaddr, uint8_t data_size, uint8_t * data)
{
    if(shiftaddr >= reg->Length || shiftaddr + data_size > reg->Length)
    {
        return 0;
    }
    else
    {
        memcpy((uint8_t * )(&reg->Reg) + shiftaddr, (uint8_t *)(data), data_size);
        return 1;
    }
}

void RemoteSet(RfCtrl signal)
{
    UartAck[3].Cmd = FUNC_ID_READ_REMOTE;
    UartAck[3].DeviceAddr = REMOTE_CTRL_ADDR;
    UartAck[3].DataAddr = 0;
    UartAck[3].DataLength = 4;
    
    UartAck[3].Data[0] = (uint8_t)signal ;
    UartAck[3].Data[1] = (uint8_t)(signal >> 8);
    UartAck[3].Data[2] = (uint8_t)(signal >> 16);
    UartAck[3].Data[3] = (uint8_t)(signal >> 24);
    
    UartAck[3].En = ENABLE;
}

//查询算法板清扫状态或者WIFI状态
void MCU_Check_CheanPackStatus(CheckCode _check_code_)
{
    uint8_t i;
    
    if(CHECK_CLEAN_STATUS == _check_code_)
    {
        i = 4;
    }
    else if(CHECK_WIFI_STATUS == _check_code_)
    {
        i = 6;
    }
    else
    {
        return;
    }
    UartAck[i].Cmd = MCU_CLEANER_STATUS_CODE_CHECK;
    UartAck[i].DeviceAddr = MCU_CLEANER_STATUS_ADDRESS;
    UartAck[i].DataAddr = 0;
    UartAck[i].DataLength = 1;
    UartAck[i].Data[0] = (uint8_t)_check_code_;
    UartAck[i].En = ENABLE;
}

//MCU 上报充电状态 
void MCU_Send_Charge_Status(void)
{
    UartAck[5].Cmd = MCU_CLEANER_STATUS_CODE_STATUS;
    UartAck[5].DeviceAddr = MCU_CLEANER_STATUS_ADDRESS;
    UartAck[5].DataAddr = 0;
    UartAck[5].DataLength = 1;
    UartAck[5].Data[0] = (uint8_t)Mcu_Charge_Status.ctrlChargingStatus;
    UartAck[5].En = ENABLE;
}

// _vacc_ : mm/s2
void SetRobitVacc(int32_t _vacc_)
{
    if(_vacc_)
    {
        RightMotor.Vacc = _vacc_* MMPERS2RPM / 200;
        LeftMotor.Vacc = RightMotor.Vacc;
        VaccEn = ENABLE;
        VaccInit = 0;
    }
    else 
    {
        VaccEn = DISABLE;
        VaccInit = 0;
    }
}

// _wacc_ : 0.01du/s
void SetRobitWacc(int32_t _wacc_)
{
    if(_wacc_)
    {
        RightMotor.Wacc = _wacc_ / 200;
        VaccEn = ENABLE;
        VaccInit = 0;
    }
    else
    {
        WaccEn = DISABLE;
        WaccInit = 0;
    }
}
