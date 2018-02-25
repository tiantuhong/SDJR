#ifndef __PID_H_
#define __PID_H_

/*----------------------------------------------*
 * 宏定义                                       *
 *----------------------------------------------*/

//#define PID_KP  (float)0.80//1//(1.8)
//#define PID_Ki  (float)0
//#define PID_Kd  (float)20
//#define PID_KI  (float)(PID_KP*PID_Ki)

//#define PID_KD  (float)(PID_KP*PID_Kd)

//#define PID_KP  (float)20        //  0.55//1//(1.8)

//#define PID_KP_LOW  (float)2
//#define PID_Ki  (float)0.01
//#define PID_Ki_LOW  (float)0.019
//#define PID_Kd  (float)4  //8//5  // 10

//#define PID_KI  (float)0.1//(PID_KP*PID_Ki)
//#define PID_KI_LOW	(float)(PID_KP_LOW*PID_Ki_LOW)

//#define PID_KD  (float)80//(PID_KP*PID_Kd)
//#define PID_KD_LOW  (float)(PID_KP_LOW*PID_Kd)

////#define PID_KI  (float)0
////#define PID_KD  (float)0
//#define PID_PWM_ADJ_MAX   50//50//PID调节时PWM的最大调整幅值

//#define Kp    (float)0.16    									//PID 比例系数
//#define Ki    (float)0.05   									//PID 积分系数
//#define Kd    (float)50                                      //PID 微分系数
//#define PID_A (float)(Kp*(1+Ki+Kd))
//#define PID_A1 (float)(Kp*(1+Kd))
//#define PID_B (float)(-Kp*(1+2*Kd))
//#define PID_CC (float)(Kp*Kd)


//extern void Clear_Ei_List(void);
extern void InitVar_PID(uint8_t PIDID);
extern signed int CalculPID(int16_t Target_Value,int16_t Cur_Value ,uint8_t PIDID);
extern int16_t ImprovePid(int16_t _error_);
//extern float	 Kp,Kd,Ki;
extern uint16_t Kp_Set,Kd_Set,Ki_Set;

#endif


