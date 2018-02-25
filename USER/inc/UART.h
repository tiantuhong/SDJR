#ifndef _UART_H
#define _UART_H

/*----------------------------------------------*
 * �궨��                                       *
 *----------------------------------------------*/
#define UART_DR_Address		(uint32_t)(0x40013804)


#define ONE_FRAME_DATA_TIMER	10
#define UART_BUF_LEN        	250 //uart�ķ��ͼ����ջ�������С
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
#define SPEED_MODE 0        ///<���ٶȺͽ��ٶȿ���ģʽ
#define ALONG_WALL_MODE 1   ///<��ǽģʽ
#define STEP_MODE 3         ///<step:��һ�������Ķ������磺˳ʱ����ת360�ȣ�����1m

// switchControl
#define SW_POWER_DOWN (1 << 2)
#define SW_DISABLE_DROP_PROTECT (1<<4) ///<�رյؼ�
#define SW_DISABLE_WALL_PROTECT (1<<5) ///<�ر�ǽ��
#define SW_DISABLE_COLLISION_STUCK_PROTECT (1 << 6) // �ر���ײ��ʱ��������ײ��ʱ�󣬽��������ײ
#define SW_ENABLE_AUTO_STOP_AT_CHARGE (1 << 8) // ��⵽����źź��Զ�ɲ��ͣ��

// Function Code
#define MCU_CONTROL_REG     2
#define MCU_STATUS_REG      3
#define HAND_SHAKE          7

#define Set_UART_TXD_DMA_Bytes(_BYTES_)	DMA1_Channel4->CNDTR = (uint16_t)(_BYTES_)

typedef enum _RCtrl{
    sig_to_charge = 3000,   //�س�
    sig_spot,               //������ɨ
    sig_to_clean_or_pause,  //������ɨ��ֹͣ��ɨ
    sig_debug31,            
    sig_reset_wifi,         //��ֵwifi
        
    //ң�ع���
    //ң�ع��ܻ���㷨���·���滮�г�ͻ��������Ҫ�㷨����ִ��
    sig_forward,            //ǰ��
    sig_back,               //����
    sig_turn_left,          //��ת
    sig_turn_right,         //��ת
}RfCtrl;

//��ɨ״̬
typedef enum _CleanPackStatus{
    CPS_IDLE,               //����
    CPS_CHARGE,             //�����
    CPS_SWEEP,              //��ɨ
    CPS_MOP,                //�ϵ�
    CPS_FAULT,              //����
    CPS_PAUSE,              //��ͣ
    CPS_BACK_CHARGE,        //�س�
    CPS_RF_CTRL             //ң��
}CleanPackStatus;

//��ɨ��״̬
typedef enum _CleanSubMode{
    CLEAN_SUB_MOD_NULL,     //����״̬
    CLEAN_SUB_MOD_TOTAL,    //ȫ��ɨ
    CLEAN_SUB_MOD_POINT,    //����ɨ
    CLEAN_SUB_MOD_AREA,     //����ɨ
    CLEAN_SUB_MOD_CUR_POINT,//����ɨ��ԭ�ض���
    CLEAN_SUB_MOD_UPDATING, //�̼�������
}CleanSubMode;

//�������
typedef enum _CheckCode{
    CHECK_CLEAN_STATUS,     //�����ɨ״̬
    CHECK_WIFI_STATUS,      //�������״̬
}CheckCode;

//addr
#define MCU_CLEANER_STATUS_ADDRESS      0x20    //ר��ͨѶ��ַ
//CMD
#define MCU_CLEANER_STATUS_CODE_CHECK     0x00    //����ɨ�ػ�״̬
#define MCU_CLEANER_STATUS_CODE_STATUS    0x01    //״̬�뻥ͨ

#pragma pack(push, 1)
/**
* @brief ״̬�Ĵ���
*/
typedef struct
{
    uint16_t ultrasound[8];         /// ����������ֵ ��λmm
    uint8_t dropSensor;             /// �ؼ촫������������ÿһλ��Ӧһ���ؼ죬1����
    uint16_t irSensor;            ///<���⴫��������������ÿһλ��Ӧһ����1����
    uint8_t collisionSensor;        ///<��ײ������,��������ÿһλ��Ӧһ����1����
    int32_t angularPos;          ///<IMU ���ֵĺ���ǽǶȣ�������Ϊ��λ����ʱ��Ϊ��
    uint32_t leftEncoderPos;      ///<��ǰ�����̼ƵĻ���λ�ã�����Ϊ��λ
    uint32_t rightEncoderPos;     ///<��ǰ�ұ���̼ƵĻ���λ�ã�����Ϊ��λ
    uint32_t lineVelocity;        ///<���ٶȣ�����ÿ��Ϊ��λ
    uint32_t angularVelocity;     ///<���ٶȣ�������ÿ��Ϊ��λ����ʱ��Ϊ��
    uint8_t chargeStatus;           ///<����ź�,1 Ϊ�г���ź�
    uint8_t batteryStatus;          ///<����,0~100
    uint8_t pickupStatus;           ///<����,1 Ϊ����������
    uint16_t errorState;          ///<����״̬,��Ϊ0��ʱ���ʾ�д���
    uint16_t wallDistance;        ///<��ǽ��������ǽ����,����Ϊ��λ
    uint32_t feedbackStatus;
    
    ///<IMU ���------------------------
    int16_t gyro_pitch;
    int16_t gyro_roll;
    int16_t gyro_yaw;
    int16_t accel_pitch;
    int16_t accel_roll;
    int16_t accel_vert;
    
    uint8_t reservedl;
    
    uint16_t wheelCurrentL;         ///���ֵ�������λmA
    uint16_t wheelCurrentR;         ///���ֵ�����λmA
    uint16_t sideBroomCurrentL;     ///���ɨ��������λmA
    uint16_t sideBroomCurrentR;     ///�ұ�ɨ��������λmA
    uint16_t midBroomCurrent;       ///��ˢ��������λmA
    uint16_t fanSpeed;              ///����ת�٣���λת/��
    
    uint8_t reserved2[4];
    uint8_t alongWallState;         ///��ǽ״̬,1 ��ʾ����ǽ��0-������ǽ
    int32_t l_speed;                ///���ֵ�ǰ�ٶ�
    int32_t r_speed;                ///���ֵ�ǰ�ٶ�
    int32_t lpwm;                   ///���ֵ�ǰ��PWM������
    int32_t rpwm;                   ///���ֵ�ǰ��PWM������    
    uint8_t reserved3[3];
    uint8_t real_battery_level;      ///�߷ֱ��ʵĵ���ֵ��0-100����1�ķֱ���
    uint16_t battery_voltage;       ///��ص�ѹ����λmV
    int16_t skid_error;             ///���쳣�����ص�����̼ƽǶȺ�IMU�Ƕȵ�ƫ���λ������ /���ٶȵ�ƫ��
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
* @brief ���ƼĴ���
*/
typedef struct
{
    int32_t lineVelocity;        ///<���ٶ�,����/��Ϊ��λ
    int32_t angularVelocity;     ///<���ٶ�,������/��Ϊ��λ,��ʱ��Ϊ��
    uint8_t mode;                ///��Ӧ����ģʽ,��control mode
    int32_t Vacc;                ///<�߼��ٶ�,����/�뷽Ϊ��λ
    int32_t Wacc;                ///<�Ǽ��ٶ�,������/�뷽Ϊ��λ
    int32_t stepS;               ///<step,����
    int16_t stepPhi;             ///<step,�Ƕ�
    uint32_t switchControl;      ///<�����л�
    uint8_t fanLevel;            ///�����������
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
    uint8_t cleanStatus;        //ɨ�ػ�״̬ clean packstatus
    uint8_t cleanStatusSub;     //ɨ�ػ���״̬ cleansubmode
    uint8_t wifiStatus;         //����״̬ 0-δ���� 1-������
    uint8_t rev;                //����
}CleanCheckStatus;

typedef struct
{
    CleanCheckStatus Reg;
    uint8_t Length;
}StatusRegisterType;


typedef struct _McuStatus{
    uint8_t ctrlChargingStatus; //������ɨ��״̬��
    //bit0-�Ƿ��ɵ��̿��ƽ��г���У�1-������MCU���Ƴ��
    //bit1-���״̬�Ƿ������0-δ���� 1-����
    //bit2-�������1-���ɹ���0-���ʧ��
    uint8_t rev[3];
}McuStatus;

//�������ݽṹ
typedef struct{
  uint8_t SOI[2];               //֡ͬ���ֽڣ�0xAa 0xa  
  uint8_t DeviceAddr;	        //�豸��ַ
  uint8_t CMD;		            //������
  uint16_t DataAddr;            //������ʼ��ַ
  uint32_t DataLength;		    //���ݳ���
  uint8_t Data[250];             //���� �� У��� ��֡β
}T_UART_SendFarme_A;

typedef struct{
  uint8_t SOI[2];               //֡ͬ���ֽڣ�0xAa 0xaa  
  uint8_t DeviceAddr;	        //�豸��ַ
  uint8_t CMD;		            //������
  uint16_t DataAddr;          //������ʼ��ַ
  uint32_t DataLength;		//���ݳ���
  ChassisStatusRegister Data;             //���� �� У��� ��֡β
  uint8_t ErrorCode;
  uint8_t CHK;
}T_UART_SendFarme;

typedef struct{
  uint8_t SOI;              //֡ͬ���ֽڣ�0xaa  
  uint8_t DeviceAddr;	    //�豸��ַ ���ڵ�����˵��0x12
  uint8_t CMD;              //������ 0x02 д������ 0x01 0x03 0x04 ��ȡ����(��������������)
  uint16_t DataAddr;		//������ʼ��ַ
  uint32_t DataLength;      //���ݳ���
  uint8_t Data[65];         //֡����
}T_UART_RcvFarme;  

typedef struct{    
    uint8_t         DeviceAddr;
    uint8_t         Cmd;        //ACK ������
    uint16_t        DataAddr;   //ACK ������ʼ��ַ
    uint32_t        DataLength; //ACK ���ݳ���
    uint8_t         Data[40];   //ACK ����    
    FunctionalState En;         //ACK ʹ�ܱ�־
}T_UART_ACKFarme;
typedef struct{    
    uint8_t         DeviceAddr;
    uint8_t         Cmd;        //ACK ������
    uint16_t        DataAddr;   //ACK ������ʼ��ַ
    uint32_t        DataLength; //ACK ���ݳ���
    uint8_t         Data[224];   //ACK ����    
    FunctionalState En;         //ACK ʹ�ܱ�־
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
