#define Threshold_1     8       //阈值1用于一阶带参滤波器，参数变化量大于此值时，计数增加
#define Threshold_2     30      //阈值2用于一阶带参滤波器，计数值大于此值时，增大参数，增强滤波跟随
#define K_Base          256     //滤波系数的基数，K = K_x / K_Base

#include <math.h>
#include "stm32f10x.h"

uint16_t K_x = 0; //滤波系数
u8 new_flag_x = 0;//本次数据变化方向
u8 num_x = 0;//滤波计数器


/*****带参数修改的一阶滤波器

入口:   NEW_DATA    新数据
      OLD_DATA    上次滤波数据
      k           滤波系数(代表在滤波结果中的权重)
      flag        上次数据变化方向
出口:   result      滤波结果
 */
int32_t filter_1_x(int32_t NEW_DATA, int32_t OLD_DATA, uint16_t k, u8 flag)
{

    //数据变化方向,new_flag=1表示增加,=0表示减小
    if((NEW_DATA-OLD_DATA)>0)
        new_flag_x=1;
    else if((NEW_DATA-OLD_DATA)<0)
        new_flag_x=0;


    if(new_flag_x==flag)  //此次变化方向与上次是否一直,相等表示变化方向一致
    {
        num_x++;
        if(fabs((NEW_DATA-OLD_DATA))>Threshold_1)
        //当参数变化量大于Threshold_1的值,计数器num快速增加,已达到快速增大K,提高跟随性
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
