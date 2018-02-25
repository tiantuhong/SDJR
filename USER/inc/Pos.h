#ifndef _POS_H
#define _POS_H

typedef struct _pos_t
{
  int32_t x , y ;
  int16_t theta ;
} pos_t ;

#define ROBOT_WHEEL_PERIMETER				20954   //0.01mm
#define ROBOT_WHEEL_PER_LAP_PULSE_COUNT		985.6   //轮子旋转一圈里程计计数
#define ROBOT_WHEEL_AXIS					290.0   // mm 两轮轴距
#define ROBOT_WHEEL_DIAMETER                62.5    //mm 轮子直径
#define ROBOT_MOTOR_PER_LAP_PULUSE_COUNT    16.0    //电机旋转一圈里程计计数

#define RPM2MMPERSEC                        (ROBOT_MOTOR_PER_LAP_PULUSE_COUNT /  ROBOT_WHEEL_PER_LAP_PULSE_COUNT * 3.1416 * ROBOT_WHEEL_DIAMETER / 60)    // r/min -> mm/s 的转换系数
#define MMPERS2RPM                          (ROBOT_WHEEL_PER_LAP_PULSE_COUNT / ROBOT_MOTOR_PER_LAP_PULUSE_COUNT / 3.1416 / ROBOT_WHEEL_DIAMETER * 60)     // mm/s -> r/min 
#define LENGTH2MILEAGE                      (ROBOT_WHEEL_PER_LAP_PULSE_COUNT / 3.1416 / ROBOT_WHEEL_DIAMETER)                                            // mm -> 里程计数
#define ANGLE2MILEAGE                       (3.1416 * ROBOT_WHEEL_AXIS / 360  * LENGTH2MILEAGE)                                                       // 角度->里程计数  （轴心旋转）
#define TENmDuPERSEC2RPM                    (ANGLE2MILEAGE * 60 /  ROBOT_MOTOR_PER_LAP_PULUSE_COUNT / 100)                                               // 角速度(0.01du/s) -> 轮子速度差(r/min)
#define RPM2TENmDuPERDSEC                   (1 / TENmDuPERSEC2RPM)                                                                                       // 轮子速度差(r/min) -> 角速度(0.01du/s)       
#define MILEAGE2MM                          (3.1416 * ROBOT_WHEEL_DIAMETER  / ROBOT_WHEEL_PER_LAP_PULSE_COUNT)

extern pos_t Motion_Pos;

void motion_init(void);
void motion_updata_pos(void);
int32_t MF_DSIN(int16_t x);
int32_t MF_DCOS(int16_t x);


#endif
