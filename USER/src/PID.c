/******************************************************************************

                  ��Ȩ���� (C), 1994-2015, ���ݾ���ŷ�϶�С�ҵ����޹�˾

 ******************************************************************************
  �� �� ��   : PID.c
  �� �� ��   : V1.0
  ��    ��   : ���ͼ
  ��������   : 2015��5��13��
  ����޸�   :
  ��������   : PID����
  �����б�   :
              CalculPID
              Clear_Ei_List
              InitVar_PID
  �޸���ʷ   :
  1.��    ��   : 2015��5��13��
    ��    ��   : ���ͼ
    �޸�����   : �����ļ�

******************************************************************************/

/*----------------------------------------------*
 * ����ͷ�ļ�                                   *
 *----------------------------------------------*/
#include "stm32f10x.h"



#define PID_K	1

#define PID_KP  (float)15 * PID_K      //  0.55//1//(1.8)

#define PID_KP_LOW  (float)2
#define PID_Ki  (float)0.01 * PID_K
#define PID_Ki_LOW  (float)0.019
#define PID_Kd  (float)5 * PID_K //8//5  // 10

#define PID_KI  (float)0.1//(PID_KP*PID_Ki)
#define PID_KI_LOW	(float)(PID_KP_LOW*PID_Ki_LOW)

#define PID_KD  (float)100//(PID_KP*PID_Kd)
#define PID_KD_LOW  (float)(PID_KP_LOW*PID_Kd)

#define PID_KP_Line (float)1.25//0.4
#define PID_Ki_Line (float)0//0.006
#define PID_Kd_Line (float)12.5//800


#define PID_KP_ANGLE    (float)1.5
#define PID_Ki_ANGLE    (float)0
#define PID_Kd_ANGLE    (float)18

#define PID_KP_BOUNDRY  (float)10 //5
#define PID_Ki_BOUNDRY  (float)0
#define PID_Kd_BOUNDRY  (float)400

//#define PID_KI  (float)0
//#define PID_KD  (float)0
#define PID_PWM_ADJ_MAX   50//50//PID����ʱPWM����������ֵ

#define PID_RATE_ADJ_MAX	15000

/*----------------------------------------------*
 * �ⲿ����˵��                                 *
 *----------------------------------------------*/


/*----------------------------------------------*
 * �ⲿ����ԭ��˵��                             *
 *----------------------------------------------*/
void InitVar_PID(uint8_t PIDID);
signed int CalculPID(int16_t Target_Value,int16_t Cur_Value ,uint8_t PIDID);

uint16_t Kp_Set,Kd_Set,Ki_Set;


/*----------------------------------------------*
 * �ڲ�����ԭ��˵��                             *
 *----------------------------------------------*/

/*----------------------------------------------*
 * ȫ�ֱ���                                     *
 *----------------------------------------------*/

/*----------------------------------------------*
 * ģ�鼶����                                   *
 *----------------------------------------------*/
float	 Kp[5],Kd[5],Ki[5];
int32_t	 LastPID_I[5];//LastPID_I_Cur[3];
int16_t Last_Value[5];


/*----------------------------------------------*
 * ��������                                     *
 *----------------------------------------------*/



/*
u = Kp * E(n)  +  Kd * (E(n) - E(n-1))  +  Ki * ��E(i)

E(n)�ǵ�n�β���ֵ������ֵ֮��

Kp * E(n)���Ǳ�������Խ�󣬵����ٶ�Խ�죬ע�����

Kd * (E(n) - E(n-1))��΢�������(E(n) - E(n-1))������������Ĳ�ֵ����ʾ���ı仯�ʣ�
������ø�ϵ���ϴ�������ǰ�����ٶȣ���Ӧ�ٶȵı仯����

Ki * ��E(i)�ǻ����Ϊ�˼��㷽�㣬��E(i)����ѭ�����д洢������ε�ֵ��ÿ�μ���PIDʱ��͡�
��ϵ���ϴ������ڼ�С��̬���

*/


void InitVar_PID(uint8_t PIDID)
{
	if(PIDID > 4)
        return;
	
    LastPID_I[PIDID] = 0;

	Last_Value[PIDID] = 0;	
    
	if(PIDID < 2)
	{
		Kp[PIDID]=PID_KP;
		Kd[PIDID]=PID_KD;
		Ki[PIDID]=PID_KI;	
	}
	else if(PIDID == 2)
	{
		Kp[PIDID]=PID_KP_Line;//(float)Kp_Set * 0.01;//
		Kd[PIDID]=PID_Kd_Line;//(float)Kd_Set * 0.01;//
		Ki[PIDID]=PID_Ki_Line;//(float)Ki_Set * 0.01;//
	}
	else if(PIDID == 3)
	{
		Kp[PIDID]=PID_KP_ANGLE;//(float)Kp_Set * 0.01;//
		Kd[PIDID]=PID_Kd_ANGLE;//(float)Kd_Set * 0.01;//
		Ki[PIDID]=PID_Ki_ANGLE;
	}
    else if(PIDID == 4)
    {
        Kp[PIDID]=PID_KP_BOUNDRY;
        Kd[PIDID]=PID_Kd_BOUNDRY;
        Ki[PIDID]=PID_Ki_BOUNDRY;
        
        Last_Value[PIDID] = PSD_Distance;//ADC_ConvertedValue_Val[7];//
    }


	

}

/*
 λ��ʽPID
 �������:_Err_-->���ε��ٶ����,Err_Min-->�����е��ڵ���Сֵ
 ��������:����PWM��ֵ
 */
signed int CalculPID(int16_t Target_Value,int16_t Cur_Value ,uint8_t PIDID)
{
	float Pid_result,tmp;
	int32_t Cur_Ei;
	int16_t Adj_Max;
	//int16_t Last_Ei;
	//int32_t  Sum_Ei;
	signed int PWM_AdjValue;
	uint16_t Speed_Limit;

	if(PIDID > 4)
        return 0;



	Cur_Ei = Target_Value - Cur_Value;
	
////////�Ƕȼ������������ǶȲ�����Χ -180~+180��
////////�ǶȲΧ (-180~+180)
	if(PIDID == 3)
	{
		if(Cur_Ei > 18000)
		{
			Cur_Ei -= 36000;
		}
		else if(Cur_Ei < -18000)
		{
			Cur_Ei += 36000;
		}
		
	}
	
	Speed_Limit = 1500;//1500;//Target_Value >> 3;

	

	//���������
	tmp = (float)Cur_Ei;
	tmp *= Kp[PIDID];
	Pid_result = tmp;

	
	/*
		//���������
		tmp = (float)Sum_Ei;
		tmp *= Ki;
		Pid_result += tmp;*/
	if(Cur_Ei <= Speed_Limit && Cur_Ei >= -Speed_Limit)
	{
//		tmp = (float)Cur_Ei;
//		tmp *= Ki[PIDID];
//        
//		tmp += LastPID_I[PIDID];
//    
//		LastPID_I[PIDID] = tmp;

//    
//		Pid_result += tmp;
		
		
		//////20161026
		LastPID_I[PIDID] += Cur_Ei;
		tmp = (float)(LastPID_I[PIDID]) * Ki[PIDID];
		Pid_result += tmp;
	} 

	//����΢����(��ֹ����)

	//΢�����У����㵱ǰ�ٶȵı仯���������ٶ���Ŀ���ٶȲ�ı仯��
	Cur_Ei = Last_Value[PIDID] - Cur_Value;
	tmp = (float)Cur_Ei;
	tmp *= Kd[PIDID];  
	Pid_result += tmp;

	Last_Value[PIDID] = Cur_Value;

	
    if(PIDID < 3)
	{
		Adj_Max = PID_PWM_ADJ_MAX;
		
		//��ֵת��ΪPWM����ֵ
		Pid_result /= 100;
	}
	else if(PIDID == 3)
	{
		Adj_Max = PID_RATE_ADJ_MAX;
		Pid_result = Pid_result;
	}
    else if(PIDID == 4)
    {
        Adj_Max = PID_RATE_ADJ_MAX;//500;//
//		Pid_result /= 100;
    }

		
//	}
//	else if(PIDID == 2)
//	{
//		Pid_result /= 1000;
//		Adj_Max = PID_SPEED_ADJ_MAX;
//	}

	if(Pid_result >= 0)
	{
		if(Pid_result > Adj_Max)
		{
		  	PWM_AdjValue = Adj_Max;
		}
		else 
		{
		  	PWM_AdjValue = (int16_t)Pid_result;
		}
	}
	else
	{
		if(Pid_result < (0-Adj_Max))
		{
		  	PWM_AdjValue = 0-Adj_Max;
		}
		else 
		{
		  	PWM_AdjValue = (int16_t)Pid_result;
		}
	} 



	return(PWM_AdjValue);
}


///�Ľ�PID ����ʽ
//du(n) = b1 * du(n-1) + b2 * e(n) + b3 * e(n-1) + b4 * e(n-2)
static int16_t lastres, error_n_1,error_n_2;
#define PID_B1      (float)0.001
#define PID_B2      (float)1.84
#define PID_B3      (float)1.86
#define PID_B4      (float)0.468
int16_t ImprovePid(int16_t _error_)
{
    float result,tmp;
    
    tmp = lastres;
    tmp *= PID_B1;
    result = tmp;
    
    tmp = _error_;
    tmp *= PID_B2;
    result += tmp;
    
    tmp = error_n_1;
    tmp *= PID_B3;
    result += tmp;
    
    tmp = error_n_2;
    tmp *= PID_B4;
    result += tmp;
    
    error_n_2 = error_n_1;
    error_n_1 = _error_;
    lastres = result;
    
    return (int16_t)result;
}
