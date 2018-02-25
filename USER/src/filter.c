#define Threshold_1     8       //��ֵ1����һ�״����˲����������仯�����ڴ�ֵʱ����������
#define Threshold_2     30      //��ֵ2����һ�״����˲���������ֵ���ڴ�ֵʱ�������������ǿ�˲�����
#define K_Base          256     //�˲�ϵ���Ļ�����K = K_x / K_Base

#include <math.h>
#include "stm32f10x.h"

uint16_t K_x = 0; //�˲�ϵ��
u8 new_flag_x = 0;//�������ݱ仯����
u8 num_x = 0;//�˲�������


/*****�������޸ĵ�һ���˲���

���:   NEW_DATA    ������
      OLD_DATA    �ϴ��˲�����
      k           �˲�ϵ��(�������˲�����е�Ȩ��)
      flag        �ϴ����ݱ仯����
����:   result      �˲����
 */
int32_t filter_1_x(int32_t NEW_DATA, int32_t OLD_DATA, uint16_t k, u8 flag)
{

    //���ݱ仯����,new_flag=1��ʾ����,=0��ʾ��С
    if((NEW_DATA-OLD_DATA)>0)
        new_flag_x=1;
    else if((NEW_DATA-OLD_DATA)<0)
        new_flag_x=0;


    if(new_flag_x==flag)  //�˴α仯�������ϴ��Ƿ�һֱ,��ȱ�ʾ�仯����һ��
    {
        num_x++;
        if(fabs((NEW_DATA-OLD_DATA))>Threshold_1)
        //�������仯������Threshold_1��ֵ,������num��������,�Ѵﵽ��������K,��߸�����
            num_x+=5;                           

        if(num_x>Threshold_2)   //??????,?????????????????,??K?
        {
            if(K_x <= K_Base - 30)
            {
                K_x=k+30;          //0.2?K_x????,???????
            }
            num_x=0;
        }
    }
    else 
    {
        num_x=0;
        K_x=3;     //???????K_x?,?????
    }

    OLD_DATA = ((K_Base - K_x) * OLD_DATA + K_x * NEW_DATA) >> 8;
    return OLD_DATA;
}
