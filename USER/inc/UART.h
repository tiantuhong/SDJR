#ifndef _UART_H
#define _UART_H

/*----------------------------------------------*
 * 宏定义                                       *
 *----------------------------------------------*/
#define UART_DR_Address		(uint32_t)(0x40013804)


#define ONE_FRAME_DATA_TIMER	10
#define UART_BUF_LEN        	250 //uart的发送及接收缓冲区大小
#define UART2_BUF_LEN           100
#define DATA_LENGTH				113 //81  //19 //
#define UART_FIRST_BYTE			0xAA
#define UART_END_BYTE			0x55
#define UART_DATA_LEN_MASK    	0x0F
#define UART_MOT_DIR_MASK     	0x80
#define UART_CMD_MASK         	0x70

#define DEVICE_ADDRESS          0x12
#define REMOTE_CTRL_ADDR        0xfd
#define CONTROL_REG_LEN         54

#define FUNC_ID_READ_REMOTE     0

// Control mode
#define SPEED_MODE 0        ///<线速度和角速度控制模式
#define ALONG_WALL_MODE 1   ///<沿墙模式
#define STEP_MODE 3         ///<step:做一个完整的动作，如：顺时针旋转360度，后退1m

// switchControl
#define SW_POWER_DOWN (1 << 2)
#define SW_DISABLE_DROP_PROTECT (1<<4) ///<关闭地检
#define SW_DISABLE_WALL_PROTECT (1<<5) ///<关闭墙检
#define SW_DISABLE_COLLISION_STUCK_PROTECT (1 << 6) // 关闭碰撞超时处理，当碰撞超时后，将会禁用碰撞
#define SW_ENABLE_AUTO_STOP_AT_CHARGE (1 << 8) // 检测到充电信号后，自动刹车停机

// Function Code
#define MCU_CONTROL_REG     2
#define MCU_STATUS_REG      3
#define HAND_SHAKE          7

#define Set_UART_TXD_DMA_Bytes(_BYTES_)	DMA1_Channel4->CNDTR = (uint16_t)(_BYTES_)

typedef enum _RCtrl{
    sig_to_charge = 3000,   //回充
    sig_spot,               //定点清扫
    sig_to_clean_or_pause,  //启动清扫和停止清扫
    sig_debug31,            
    sig_reset_wifi,         //充值wifi
        
    //遥控功能
    //遥控功能会和算法板的路径规划有冲突，所以需要算法板来执行
    sig_forward,            //前进
    sig_back,               //后退
    sig_turn_left,          //左转
    sig_turn_right,         //右转
}RfCtrl;

//清扫状态
typedef enum _CleanPackStatus{
    CPS_IDLE,               //空闲
    CPS_CHARGE,             //充电中
    CPS_SWEEP,              //清扫
    CPS_MOP,                //拖地
    CPS_FAULT,              //故障
    CPS_PAUSE,              //暂停
    CPS_BACK_CHARGE,        //回充
    CPS_RF_CTRL             //遥控
}CleanPackStatus;

//清扫子状态
typedef enum _CleanSubMode{
    CLEAN_SUB_MOD_NULL,     //无子状态
    CLEAN_SUB_MOD_TOTAL,    //全局扫
    CLEAN_SUB_MOD_POINT,    //定点扫
    CLEAN_SUB_MOD_AREA,     //区域扫
    CLEAN_SUB_MOD_CUR_POINT,//定点扫，原地定点
    CLEAN_SUB_MOD_UPDATING, //固件升级中
}CleanSubMode;

//检查内容
typedef enum _CheckCode{
    CHECK_CLEAN_STATUS,     //检查清扫状态
    CHECK_WIFI_STATUS,      //检查网络状态
}CheckCode;

//addr
#define MCU_CLEANER_STATUS_ADDRESS      0x20    //专用通讯地址
//CMD
#define MCU_CLEANER_STATUS_CODE_CHECK     0x00    //请求扫地机状态
#define MCU_CLEANER_STATUS_CODE_STATUS    0x01    //状态码互通

#pragma pack(push, 1)
/**
* @brief 状态寄存器
*/
typedef struct
{
    uint16_t ultrasound[8];         /// 超声波距离值 单位mm
    uint8_t dropSensor;             /// 地检传感器，开关量每一位对应一个地检，1触发
    uint16_t irSensor;            ///<红外传感器，开关量，每一位对应一个，1触发
    uint8_t collisionSensor;        ///<碰撞传感器,开关量，每一位对应一个，1触发
    int32_t angularPos;          ///<IMU 积分的航向角角度，毫弧度为单位，逆时针为正
    uint32_t leftEncoderPos;      ///<当前左边里程计的积分位置，毫米为单位
    uint32_t rightEncoderPos;     ///<当前右边里程计的积分位置，毫米为单位
    uint32_t lineVelocity;        ///<线速度，毫米每秒为单位
    uint32_t angularVelocity;     ///<角速度，毫弧度每秒为单位，逆时针为负
    uint8_t chargeStatus;           ///<充电信号,1 为有充电信号
    uint8_t batteryStatus;          ///<电量,0~100
    uint8_t pickupStatus;           ///<抱起,1 为机器被抱起
    uint16_t errorState;          ///<错误状态,不为0的时候表示有错误
    uint16_t wallDistance;        ///<沿墙传感器离墙距离,毫米为单位
    uint32_t feedbackStatus;
    
    ///<IMU 相关------------------------
    int16_t gyro_pitch;
    int16_t gyro_roll;
    int16_t gyro_yaw;
    int16_t accel_pitch;
    int16_t accel_roll;
    int16_t accel_vert;
    
    uint8_t reservedl;
    
    uint16_t wheelCurrentL;         ///左轮电流，单位mA
    uint16_t wheelCurrentR;         ///右轮电流单位mA
    uint16_t sideBroomCurrentL;     ///左边扫电流，单位mA
    uint16_t sideBroomCurrentR;     ///右边扫电流，单位mA
    uint16_t midBroomCurrent;       ///中刷电流，单位mA
    uint16_t fanSpeed;              ///风扇转速，单位转/分
    
    uint8_t reserved2[4];
    uint8_t alongWallState;         ///沿墙状态,1 表示在沿墙，0-不在沿墙
    int32_t l_speed;                ///左轮当前速度
    int32_t r_speed;                ///右轮当前速度
    int32_t lpwm;                   ///左轮当前的PWM控制量
    int32_t rpwm;                   ///右轮当前的PWM控制量    
    uint8_t reserved3[3];
    uint8_t real_battery_level;      ///高分辨率的电量值，0-100，如1的分辨率
    uint16_t battery_voltage;       ///电池电压，单位mV
    int16_t skid_error;             ///打滑异常，返回的是里程计角度和IMU角度的偏差，单位毫弧度 /角速度的偏差
    uint8_t reserved4[8];
}ChassisStatusRegister;

// errorState
#define ERR_STATUS_IMU              (1<<0)
#define ERR_STATUS_L_SIDE_I         (1<<1)
#define ERR_STATUS_R_SIDE_I         (1<<2)
#define ERR_STATUS_MID_I            (1<<3)
#define ERR_STATUS_L_WHEEL_I        (1<<4)
#define ERR_STATUS_R_WHEEL_I        (1<<5)
#define ERR_STATUS_FAN_SPEED        (1<<6)
#define ERR_STATUS_COLLISION        (1<<7)
#define ERR_STATUS_DROP             (1<<8)

// feedbackStatus
#define STATUS_IS_STEPING           (1<<0)
#define STATUS_GARBAGE_FULL         (1<<3)
#define STATUS_GARBAGE_BOX_PLUGIN   (1<<4)
#define STATUS_STEPING_BREAK        (1<<5)

/**
* @brief 控制寄存器
*/
typedef struct
{
    int32_t lineVelocity;        ///<线速度,毫米/秒为单位
    int32_t angularVelocity;     ///<角速度,毫弧度/秒为单位,逆时针为正
    uint8_t mode;                ///对应工作模式,见control mode
    int32_t Vacc;                ///<线加速度,毫米/秒方为单位
    int32_t Wacc;                ///<角加速度,毫弧度/秒方为单位
    int32_t stepS;               ///<step,距离
    int16_t stepPhi;             ///<step,角度
    uint32_t switchControl;      ///<开关切换
    uint8_t fanLevel;            ///风机风力控制
    uint32_t reserved[2];
    uint8_t sideBroomLevel;
    uint8_t midBroomLevel;
    uint8_t reserved1[4];
    uint32_t reserve[3];
}ChassisControlRegister;

typedef struct
{
    ChassisControlRegister Reg;
    uint8_t Reg_Length;
}RegisterType;

typedef struct _CleanCheckStatus{
    uint8_t cleanStatus;        //扫地机状态 clean packstatus
    uint8_t cleanStatusSub;     //扫地机子状态 cleansubmode
    uint8_t wifiStatus;         //网络状态 0-未联网 1-已联网
    uint8_t rev;                //保留
}CleanCheckStatus;

typedef struct
{
    CleanCheckStatus Reg;
    uint8_t Length;
}StatusRegisterType;


typedef struct _McuStatus{
    uint8_t ctrlChargingStatus; //控制清扫的状态，
    //bit0-是否由底盘控制进行充电中，1-正在由MCU控制充电
    //bit1-充电状态是否结束，0-未结束 1-结束
    //bit2-充电结果，1-充电成功，0-充电失败
    uint8_t rev[3];
}McuStatus;

//串口数据结构
typedef struct{
  uint8_t SOI[2];               //帧同步字节，0xAa 0xa  
  uint8_t DeviceAddr;	        //设备地址
  uint8_t CMD;		            //功能码
  uint16_t DataAddr;            //数据起始地址
  uint32_t DataLength;		    //数据长度
  uint8_t Data[250];             //数据 及 校验和 及帧尾
}T_UART_SendFarme_A;

typedef struct{
  uint8_t SOI[2];               //帧同步字节，0xAa 0xaa  
  uint8_t DeviceAddr;	        //设备地址
  uint8_t CMD;		            //功能码
  uint16_t DataAddr;          //数据起始地址
  uint32_t DataLength;		//数据长度
  ChassisStatusRegister Data;             //数据 及 校验和 及帧尾
  uint8_t ErrorCode;
  uint8_t CHK;
}T_UART_SendFarme;

typedef struct{
  uint8_t SOI;              //帧同步字节，0xaa  
  uint8_t DeviceAddr;	    //设备地址 对于底盘来说是0x12
  uint8_t CMD;              //功能码 0x02 写入数据 0x01 0x03 0x04 读取数据(被动和主动都有)
  uint16_t DataAddr;		//数据起始地址
  uint32_t DataLength;      //数据长度
  uint8_t Data[65];         //帧数据
}T_UART_RcvFarme;  

typedef struct{    
    uint8_t         DeviceAddr;
    uint8_t         Cmd;        //ACK 功能码
    uint16_t        DataAddr;   //ACK 数据起始地址
    uint32_t        DataLength; //ACK 数据长度
    uint8_t         Data[40];   //ACK 数据    
    FunctionalState En;         //ACK 使能标志
}T_UART_ACKFarme;
typedef struct{    
    uint8_t         DeviceAddr;
    uint8_t         Cmd;        //ACK 功能码
    uint16_t        DataAddr;   //ACK 数据起始地址
    uint32_t        DataLength; //ACK 数据长度
    uint8_t         Data[224];   //ACK 数据    
    FunctionalState En;         //ACK 使能标志
}T_UART_ACKFarme_P;

typedef struct{
  uint8_t SOI;              //
  uint8_t CMD;              //
  uint8_t DataLength;          //
  uint8_t Data[42];     //????
}T_UART2_SendFarme;  

#pragma pack(pop)

extern FunctionalState Uart_SendEn, Uart2_SendEn, HandShakeEn;
extern FunctionalState UART_SendOver;
extern uint16_t UART_Send_Fq;
extern RegisterType ControlReg;
extern uint16_t test1,test2,test3;
extern int16_t q0buf,q1buf,q2buf,q3buf,axbuf,aybuf,azbuf;
extern int32_t RobotLineSpeedTargPre, RobotAngleRatePre;
extern uint8_t StepModeOn, CtrlMode;
extern uint8_t RobotCleanStatus;
extern uint32_t GarbageStatus, StepStatus;

extern void UART_Conf(void);
extern void UART_ResponseMaster(void);
extern void UartTimerJudge(void);;
extern void Uart_RcvData_Proc(void);
extern void UART_SendSignal(void);
extern void SetRobitAngleRun(uint16_t Angle, uint8_t Dir, uint16_t AngleRate, uint8_t SpinMode);
extern void SetRobitMileageRun(uint16_t Mileage, uint8_t Dir, uint16_t RunSpeed);
extern void SetRobitSpeed(int16_t LeftSpeed, int16_t RightSpeed);
extern void SetRobitLineAnglerate(int16_t lineSpeed, int16_t angleRate);
extern void UART2_SendData(void);
extern void Uart2_RcvData_Proc(void);
extern void handshakepresec(void);
extern void RemoteSet(RfCtrl signal);
extern int32_t Length2Mileage(int32_t LengthInput);
extern void SetRobitVacc(int32_t _vacc_);
extern void SetRobitWacc(int32_t _wacc_);
extern void DrawArc(int16_t lineSpeed, int16_t angleRate, uint16_t angleTarg, uint32_t Mileage);
extern void MCU_Check_CheanPackStatus(CheckCode _check_code_);
extern void MCU_Send_Charge_Status(void);
#endif
