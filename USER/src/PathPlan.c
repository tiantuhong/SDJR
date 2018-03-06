/******************************************************************************

                  版权所有 (C), 1994-2016, 杭州九阳欧南多小家电有限公司

 ******************************************************************************
  文 件 名   : PathPlan.c
  版 本 号   : V1.0
  作    者   : tht
  生成日期   : 2016年8月12日
  最近修改   :
  功能描述   : 路径规划
  函数列表   :
  修改历史   :
  1.日    期   : 2016年8月12日
    作    者   : tht
    修改内容   : 创建文件

******************************************************************************/

/*----------------------------------------------*
 * 包含头文件                                   *
 *----------------------------------------------*/
#include "stm32f10x.h"
/*----------------------------------------------*
 * 外部变量说明                                 *
 *----------------------------------------------*/

/*----------------------------------------------*
 * 外部函数原型说明                             *
 *----------------------------------------------*/

/*----------------------------------------------*
 * 内部函数原型说明                             *
 *----------------------------------------------*/
void RegRechargeStand(void);
void ReplaceDishcloth(void);
void BorderWork(void);
void BorderWork_irobot(void);
void ClosetoWall(void);
uint8_t DataTrendCal(uint16_t data);
void CloseToWall2(void);
void BowShapeClean(void);
/*----------------------------------------------*
 * 全局变量                                     *
 *----------------------------------------------*/
uint8_t PathPlanMode, PathPlanStep, SubStep = 0; // 0=未启用 1=沿边模式
//uint8_t PathPlanStep; 				// 路径规划步骤
uint16_t UltraFrontDisMin = 0xffff;			//前超声波最小距离，
int16_t UltraFrontAngle, SysInitAngle, SysFirstAngle;			// 前超声波最小距离的方向角
uint8_t BumpStaBak,BumpCnt = 0;

uint8_t FlagUltraRising[3],FlagUltraFalling[3],FlagRobotTurnOK[3];
FunctionalState PathPlanEn,UltraSignalCalEn;
/*----------------------------------------------*
 * 模块级变量                                   *
 *----------------------------------------------*/
pos_t StartPos;									//初始位置
FunctionalState StartPosEn, EndPosCheckEn, EndPosGet;      //初始位置确定标志位，结束位置检测使能标志位

uint32_t RobitLineMileage;
int16_t   PathPlanAngle;
/*----------------------------------------------*
 * 常量定义                                     *
 *----------------------------------------------*/

/*----------------------------------------------*
 * 宏定义                                       *
 *----------------------------------------------*/
#define ROBOT_RUN_SPEED		200  // 单位mm/s
#define ROBOT_WALL_DISTANCE	850
#define START_POS_RANGE		8000			//0.01mm
#define END_POS_RANGE		6000			//0.01mm
#define ROBOT_LINE_DISATNCE	10000			// mm
#define PSD_WALL_DIS_MAX    900 //450//            //1641
#define PSD_WALL_DIS_MAX2   1641 //1200//
#define REDSINGALDIS        50//200
#define REDSINGALDIS2       500//250
#define CLOSEWALLSTEPDIS    75

#define ROBOT_CLOSE_WALL_SPEED  160 //mm/s
#define ROBOT_VACC              100 // mm/s2
/*****************************************************************************
 函 数 名  : PathPlan
 功能描述  : 路径规划10ms调用一次
 输入参数  : void  
 输出参数  : 无j
 返 回 值  : 
 调用函数  : 
 被调函数  : 
 
 修改历史      :
  1.日    期   : 2016年8月12日
    作    者   : tht
    修改内容   : 新生成函数

*****************************************************************************/
void PathPlan(void)
{
	int16_t tmpangledif,tmpstartangle;
	uint16_t tmpdistance = 0;
	uint16_t tmpangle;
	static uint16_t delaycnt = 0, RunMileage = 0;
//	static uint8_t  FlagFirstTurn = 0;
	
	if(!PathPlanEn)
		return;
	PathPlanEn = DISABLE;
	
	//
	
	if(1 == PathPlanMode)
	{
//		if(EndPosGet)
//		{
//			return;
//		}
			
		
		switch(PathPlanStep)
		{
			case 0:
				//旋转一圈
				SetRobitAngleRun(360,1,30, 0); // 向左轴心旋转360° 速度30°/s
				PathPlanStep = 1;
			
				StartPosEn = DISABLE;
				EndPosCheckEn = DISABLE;
				EndPosGet = DISABLE;
			
				RobitLineMileage = 0;
				PathPlanAngle = 0;
				break;
			case 1:
				//寻找前超声波距离最小值和角度
				if(UltraFrontDisMin > Ultra_RevTime[0])
				{
					UltraFrontDisMin = Ultra_RevTime[0];
					UltraFrontAngle = Sys_Angle;
				}
				
				// 旋转一圈完成
				if(0 == RobitStatus)
				{
					//计算最小角度和当前角度的差值
					tmpangledif = UltraFrontAngle - Sys_Angle;
					if(tmpangledif > 18000)
					{
						tmpangledif -= 36000;
					}
					else if(tmpangledif < -18000)
					{
						tmpangledif += 36000;
					}
					tmpangledif /= 100;
					if(tmpangledif > 1)
					{
						SetRobitAngleRun(tmpangledif , 2, 30, 0);
					}
					else if(tmpangledif < -1)
					{
						tmpangledif = -tmpangledif;
						SetRobitAngleRun(tmpangledif , 1, 30, 0);
					}
					
					PathPlanStep = 2;
					
					
				}
				break;
			case 2:
				//直行到最近障碍物
				if(0 == RobitStatus)
				{
//					tmpdistance = UltraTime2Dis(UltraFrontDisMin);
//					SetRobitMileageRun(tmpdistance, 1, ROBOT_RUN_SPEED);
					SetRobitMileageRun(ROBOT_LINE_DISATNCE, 1, ROBOT_RUN_SPEED);
					PathPlanStep = 3;
				}
			
				break;
			case 3:
				//障碍物前停止
				if(1 == RobitStatus)
				{
//					if(Ultra_RevTime[0] <= 600)
//					{
//						SetRobitSpeed(0, 0);
//					}
				}
				else
				{
					//此位置为沿边行走的初始位置
					StartPos.x = Motion_Pos.x;
					StartPos.y = Motion_Pos.y;
					StartPosEn = ENABLE;
					
					if(1 == RightBump || 1 == LeftBump)
					{
						//有碰撞，则后退 65mm
						SetRobitMileageRun(65, 2, ROBOT_RUN_SPEED);
						PathPlanStep = 9;
					}
					else
					{
						PathPlanStep = 4;
					
						//旋转90°，尽可能扫地机与墙平行
						SetRobitAngleRun(75 , 1, 60, 0);
						
						FlagUltraRising[1] = 0;
						FlagUltraRising[1] = 0;
						FlagUltraFalling[2] = 0;
						FlagUltraFalling[2] = 0;
						FlagRobotTurnOK[1] = 0;
						FlagRobotTurnOK[2] = 0;
						UltraSignalCalEn = DISABLE;
					}
					
				}
			case 4:
				
				if(1 == RobitStatus)
				{
					if(UltraSignalCalEn)
					{
						//右前超声波 跳变检测
						if(0 == FlagUltraRising[1])
						{
							if(ENABLE == Ultra_Rising[1])
							{
								FlagUltraRising[1] = 1;
							}
						}
						else
						{
							if(ENABLE == Ultra_Falling[1])
							{
								FlagRobotTurnOK[1] = 1;
							}
						}
						
						// 右后超声波 跳变检测
						if(0 == FlagUltraRising[2])
						{
							if(ENABLE == Ultra_Rising[2])
							{
								FlagUltraRising[2] = 1;
							}
						}
						else
						{
							if(ENABLE == Ultra_Falling[2])
							{
								FlagRobotTurnOK[2] = 1;
							}
						}
						
						if(1 == FlagRobotTurnOK[1] && 1 == FlagRobotTurnOK[2])
						{
							// 前后超声波数据差在误差范围内，转弯认为完成
							if(Ultra_RevTimeVlid[1] < Ultra_RevTimeVlid[2] + Edage_Distance /*&& Ultra_RevTimeVlid[2] <= 1140*/)
							{
								SetRobitSpeed(0, 0);
	//							PathPlanStep = 10;
							}
	//						if(1 == FlagUltraRising[1])
	//						{
	//							SetRobitSpeed(0, 0);
	//						}
						}
					}
					
				}
				else if(0 == RobitStatus)
				// 沿墙行走 			
				{
					SetRobitMileageRun(ROBOT_LINE_DISATNCE, 1, ROBOT_RUN_SPEED);  // 直行4m
					
//					//超声波数据在2000以内 沿墙行走，否则保持直线行走
//					if(Ultra_RevTimeVlid[1] + Ultra_RevTimeVlid[2] > 2000)
//					{
//						RobitRunbyRouteMode = 1;
//					}
//					else
//					{
						RobitRunbyRouteMode = 2;
//					}
					PathPlanStep = 5;
					Ultra_Jump[1] = DISABLE;
					Ultra_Jump[2] = DISABLE;

				}
				break; 
			case 5:
				//前方有障碍物停止
				if(1 == RobitStatus)
				{
//					if(Ultra_RevTime[0] <= 600)
//					{
//						SetRobitSpeed(0, 0);	
//						
//					}
					
//					if(Ultra_RevTimeVlid[1] + Ultra_RevTimeVlid[2] > 2000)
//					{
//						RobitRunbyRouteMode = 1;
//					}
//					else
//					{
//						RobitRunbyRouteMode = 2;
//					}
					
					// 记录直行行走时，最长路径的最终方向
					if(RightCurMileage > RobitLineMileage)
					{
						PathPlanAngle = Sys_Angle;
						RobitLineMileage = RightCurMileage;
					}
                    
                    
					
					//20170221 前超声波数据上升沿跳变，后超声波数据无跳变 并且后超声波数据小于设定值 认为有一个凸角弯 
					if(ENABLE == Ultra_Jump[1] && DISABLE == Ultra_Jump[2] && ENABLE == Ultra_Rising[1] /*&& Ultra_RevTimeVlid[2] <= 1600*/)
					{
						//保持直行
//						RobitRunbyRouteMode = 1;
//						Sys_Startangle = Sys_Angle;
						
						SetRobitMileageRun(30, 1, ROBOT_RUN_SPEED);						
						PathPlanStep = 6;
						
					}
				}
				
				if(0 == RobitStatus)
				{
					if(1 == RightBump || 1 == LeftBump)
					{
//						//有碰撞，则后退 65mm
//						SetRobitMileageRun(65, 2, ROBOT_RUN_SPEED);
//						PathPlanStep = 9;
						PathPlanStep = 12;
					}
					else
					{
						PathPlanStep = 4;
						
						if(Ultra_RevTimeVlid[0] < ROBOT_WALL_DISTANCE)
						{
							SetRobitAngleRun(90 , 1, 45, 0);						
							UltraSignalCalEn = ENABLE;
						}
						else
						{
							//20170221 每次15度尝试转弯，超声波数据判定转弯禁止
							SetRobitAngleRun(15 , 1, 30, 0);						
							UltraSignalCalEn = DISABLE;
						}
					}
				}
				
				break;
			case 6:
				
				if(0 == RobitStatus)
				{
					SetRobitAngleRun(85 , 2, 60, 1);
					PathPlanStep = 7;
					Ultra_Jump[1] = DISABLE;
					Ultra_Jump[2] = DISABLE;
				}
				
//				if(ENABLE == Ultra_Jump[1] && ENABLE == Ultra_Jump[2])
//				{
////					SetRobitSpeed(0, 0);
//					SetRobitMileageRun(50, 1, ROBOT_RUN_SPEED);
//					Ultra_Jump[1] = DISABLE;
//					Ultra_Jump[2] = DISABLE;
//				}
				break;
			case 7:
				if(0 == RobitStatus)
				{
//					SetRobitMileageRun(ROBOT_LINE_DISATNCE, 1, ROBOT_RUN_SPEED);
                    //20170227
                    SetRobitMileageRun(210, 1, ROBOT_RUN_SPEED);
					Ultra_Jump[1] = DISABLE;
					Ultra_Jump[2] = DISABLE;
					PathPlanStep = 8;
				}
				else
				{
					if(Ultra_RevTimeVlid[1] <= Edage_Distance)
					{
						SetRobitSpeed(0, 0);
					}
				}

				break;
			case 8:
				
				if(0 == RobitStatus)
				{
					if(1 == RightBump || 1 == LeftBump)
					{
						//有碰撞，则后退 65mm
						SetRobitMileageRun(65, 2, ROBOT_RUN_SPEED);
						PathPlanStep = 9;
					}
					else
					{                       
//						SetRobitMileageRun(ROBOT_LINE_DISATNCE, 1, ROBOT_RUN_SPEED);
                        //20170227
                        Ultra_Jump[1] = DISABLE;
						Ultra_Jump[2] = DISABLE;
						SetRobitMileageRun(280, 2, ROBOT_RUN_SPEED);
						PathPlanStep = 10;
					}
				}
				else
				{
                    //20170227
					if(DISABLE == Ultra_Jump[1] && ENABLE == Ultra_Jump[2] && ENABLE == Ultra_Falling[2])
					{
	//					PathPlanStep = 5;
	//					RobitRunbyRouteMode = 2;
						Ultra_Jump[1] = DISABLE;
						Ultra_Jump[2] = DISABLE;
						SetRobitMileageRun(280, 2, ROBOT_RUN_SPEED);
						PathPlanStep = 10;
					}
					
					if(1 == RightBump || 1 == LeftBump)
					{
						//有碰撞，则后退 65mm
						SetRobitMileageRun(65, 2, ROBOT_RUN_SPEED);
						PathPlanStep = 9;
					}
					
				}				
				break;
			case 9:
				if(0 == RobitStatus)
				{
					if(1 == RightBump || 1 == LeftBump)
					{
						//有碰撞，则后退 65mm
						SetRobitMileageRun(65, 2, ROBOT_RUN_SPEED);
					}
					else
					{
						if(Ultra_RevTimeVlid[0] < ROBOT_WALL_DISTANCE)
						{
							SetRobitAngleRun(90 , 1, 45, 0);
							FlagUltraRising[1] = 0;
							FlagUltraRising[1] = 0;
							FlagUltraFalling[2] = 0;
							FlagUltraFalling[2] = 0;
							FlagRobotTurnOK[1] = 0;
							FlagRobotTurnOK[2] = 0;
							UltraSignalCalEn = ENABLE;
						}
						else
						{
							SetRobitAngleRun(30 , 1, 30, 0);
							UltraSignalCalEn = DISABLE;
						}
						
						PathPlanStep = 4;
					}
				}
				break;
			case 10:
				if(0 == RobitStatus)
				{
					SetRobitMileageRun(ROBOT_LINE_DISATNCE, 1, ROBOT_RUN_SPEED);
					Ultra_Jump[1] = DISABLE;
					Ultra_Jump[2] = DISABLE;
					PathPlanStep = 11;
				}
				break;
			case 11:
				if(1 == RobitStatus)
				{
                    //20170227
					if(ENABLE == Ultra_Jump[2] && DISABLE == Ultra_Jump[1] && ENABLE == Ultra_Falling[2])
					{
						//延时360ms  4次超声波数据，保证超声波沿墙效果
//						delaycnt ++;
//						if(delaycnt >= 36)
//						{
//							delaycnt = 0;
							PathPlanStep = 5;
							RobitRunbyRouteMode = 2;
							Ultra_Jump[1] = DISABLE;
							Ultra_Jump[2] = DISABLE;
//						}
					}
				}
				else
				{
					if(1 == RightBump || 1 == LeftBump)
					{
						//有碰撞，则后退 65mm
						SetRobitMileageRun(65, 2, ROBOT_RUN_SPEED);
						PathPlanStep = 9;
					}
					else
					{
						SetRobitMileageRun(ROBOT_LINE_DISATNCE, 1, ROBOT_RUN_SPEED);
					}
				}
				break;
			case 12:
//				if(delaycnt < 100)
//				{
//					delaycnt ++;
//				}
//				else
//				{
					PathPlanStep = 9;
					
					//// 20170221 取消碰撞暂停设置
//					delaycnt = 100;//0;
					
					//有碰撞，则后退 65mm
					SetRobitMileageRun(65, 2, ROBOT_RUN_SPEED);
//				}
			
				//等待2s
				break;
			
			//以下步骤 沿边结束，行走至弓形规划起始点。
			//1.旋转机器角度至直线角度(PathPlanAngle)+90,			
			//2.沿着此角度直行至碰撞结束			
			//3.弓形规划
			case 13:
				SetRobitSpeed(0, 0);
				PathPlanStep = 14;
				break;
			case 14:
				if(0 == RobitStatus)
				{
					if(PathPlanAngle < 9000)
					{
						Sys_Startangle = PathPlanAngle + 9000;
					}
					else
					{
						Sys_Startangle = PathPlanAngle - 27000;
					}
					
					tmpangle = CalAngleDif(Sys_Startangle,Sys_Angle);
					if(tmpangle <= 1000 || tmpangle >= 35000)
					{
						tmpstartangle = Sys_Startangle;
						SetRobitMileageRun(ROBOT_LINE_DISATNCE, 1, 110);
						Sys_Startangle = tmpstartangle;
						PathPlanStep = 15;
					}
					else
					{
						tmpangle /= 100;
						if(tmpangle <= 180)
						{
							SetRobitAngleRun(tmpangle, 2, 30, 0);
						}
						else
						{
							tmpangle = 360 - tmpangle;
							SetRobitAngleRun(tmpangle, 1, 30, 0);
						}

					}	
					 
				}
				break;
			case 15:
				if(0 == RobitStatus)
				{					
					if(1 == RightBump || 1 == LeftBump)
					{
						//有碰撞，则后退 65mm
						SetRobitMileageRun(65, 2, ROBOT_RUN_SPEED);
					}
					else
					{

					}
					PathPlanStep = 16;
				}
				break;
			case 16:
				if(0 == RobitStatus)
				{					
//					PathPlanMode = 2;
					PathPlanStep = 0;
					BumpCnt = 0;
					SysInitAngle = PathPlanAngle;
				}
				break;
			default:
				break;
		}
		
		//20170228 检测沿边是否完成  
		
		// 1. 初始位置确定 
		if(StartPosEn && (!EndPosCheckEn))
		{
			//横坐标和纵坐标 都不在初始范围内 ，认为机器已经出发 ，可以检测是否闭环
			if((Motion_Pos.x >= StartPos.x + START_POS_RANGE || Motion_Pos.x <= StartPos.x - START_POS_RANGE) || (Motion_Pos.y >= StartPos.y + START_POS_RANGE || Motion_Pos.y <= StartPos.y - START_POS_RANGE))
			{
				EndPosCheckEn = ENABLE;
			}
		}
		
		// 2. 结束位置检测使能时，坐标在结束位置检测范围内，认为完成沿边
		if(EndPosCheckEn && (!EndPosGet))
		{
			if((Motion_Pos.x <= StartPos.x + END_POS_RANGE && Motion_Pos.x >= StartPos.x - END_POS_RANGE) && (Motion_Pos.y <= StartPos.y + END_POS_RANGE && Motion_Pos.y >= StartPos.y - END_POS_RANGE))
			{
				EndPosGet = ENABLE;
//				SetRobitSpeed(0, 0);
				PathPlanStep = 13;
			}
		}
		
	}
	else if(2 == PathPlanMode)
	{
		switch(PathPlanStep)
		{
			case 0:
				//直行4m
				if(0 == RobitStatus)
				{
//					SetRobitMileageRun(ROBOT_LINE_DISATNCE, 1, 110);
					
					if((BumpCnt & 0x01) == 0x01)
					{
//						tmpangle = SysInitAngle + 180;
						if(SysInitAngle > 0)
						{
							Sys_Startangle = SysInitAngle -18000;
						}
						else
						{
							Sys_Startangle = SysInitAngle + 18000;
						}
//						Sys_Startangle = SysInitAngle;
					}
					else
					{
						Sys_Startangle = SysInitAngle;
					}
					
					tmpangle = CalAngleDif(Sys_Startangle,Sys_Angle);
					if(tmpangle <= 1000 || tmpangle >= 35000)
					{
						tmpstartangle = Sys_Startangle;
						SetRobitMileageRun(ROBOT_LINE_DISATNCE, 1, 110);
						Sys_Startangle = tmpstartangle;
						PathPlanStep = 1;
					}
					else
					{
						//20170308 转弯后退 20
						if(RightBump == 1 || LeftBump == 1)
						{
							SetRobitMileageRun(20, 2, 110);
						}
						else
						{
							tmpangle /= 100;
							if(tmpangle <= 180)
							{
								SetRobitAngleRun(tmpangle, 2, 30, 0);
							}
							else
							{
								tmpangle = 360 - tmpangle;
								SetRobitAngleRun(tmpangle, 1, 30, 0);
							}
						}

					}				
					
				}
				break;
			case 1:
				if(0 == RobitStatus)
				{
					if(RightBump == 1 || LeftBump == 1)
					{
//						SetRobitMileageRun(70, 2, 110);
//						PathPlanStep = 2;
						PathPlanStep = 6;
						delaycnt = 0;
						BumpCnt	++;					
					}
					else
					{
						BumpCnt	++;
						PathPlanStep = 2;
					}
					BumpStaBak = (RightBump << 1) | LeftBump;
				}
				break;
			case 2:
				// 右碰撞 左转 左碰撞右转 不碰撞 左转
				// 左转180°直行 ,左转90° 前走一小步,然后继续左转90 然后直行
				if(0 == RobitStatus)
				{
//					if(2 == BumpStaBak || 3 == BumpStaBak || 0 == BumpStaBak)
//					{
//						tmpangledif = CalAngleDif(SysInitAngle, Sys_Angle);
					
					if(RightBump == 1 || LeftBump == 1 || (RightCurMileage < (343 - RunMileage) && LeftCurMileage < (343 - RunMileage)))
					{
						tmpdistance =  343 - RightCurMileage;
						RunMileage += RightCurMileage;
						if(RunMileage > 343)
						{
							RunMileage = 343;
						}
						tmpdistance *= 10;
						tmpdistance /= 47;
						SetRobitMileageRun(tmpdistance, 2, 110);
					}
					else
					{
						if((BumpCnt & 0x01) == 0x01)
						{
//							tmpangledif = 90 - tmpangledif / 100;
//							SetRobitAngleRun(90, 1, 60, 0);
//							PathPlanStep = 4;
//							if(0 == FlagFirstTurn)
//							{
								SetRobitAngleRun(180, 1, 60, 0);
//								FlagFirstTurn = 1;
//							}
//							else
//							{
//								SetRobitAngleRun(180, 2, 60, 0);

//							}
														
							PathPlanStep = 0;
							BumpStaBak = 0;
						}
						else
						{
//							tmpangledif = 90 + tmpangledif / 100 - 180;
							SetRobitAngleRun(90, 2, 60, 0);
							PathPlanStep = 4;
						}
					}
//					}
//					else if(1 == BumpStaBak)
//					{
//						SetRobitAngleRun(90 , 2, 60, 1);
//					}
					
//					else
//					{
//						SetRobitAngleRun(90 , 1, 60, 1);
//					}
					
				}
				break;
			case 3:
				if(0 == RobitStatus)
				{
					if(RightBump == 1 || LeftBump == 1)
					{
						SetRobitMileageRun(20, 2, 110);
					}
					else
					{
						if((BumpCnt & 0x01) == 0x01)
						{
							SetRobitAngleRun(90 , 1, 60, 0);
						}
						else
						{
							SetRobitAngleRun(90 , 2, 60, 0);
						}

						BumpStaBak = 0;
						PathPlanStep = 0;
					}
				}
				break;
			case 4:
				if(0 == RobitStatus)
				{
					if(SysInitAngle > -9000)
					{
						Sys_Startangle = SysInitAngle - 9000;
					}
					else
					{
						Sys_Startangle = SysInitAngle + 27000;
					}
					
					tmpangle = CalAngleDif(Sys_Startangle,Sys_Angle);
					if(tmpangle <= 1000 || tmpangle >= 35000)
					{
						tmpstartangle = Sys_Startangle;
						SetRobitMileageRun(160, 1, 110);
						Sys_Startangle = tmpstartangle;
						PathPlanStep = 3;
					}
					else
					{
						//20170308 转弯后退 20
						if(RightBump == 1 || LeftBump == 1)
						{
							SetRobitMileageRun(20, 2, 110);
						}
						else
						{
							tmpangle /= 100;
							if(tmpangle <= 180)
							{
								SetRobitAngleRun(tmpangle, 2, 30, 0);
							}
							else
							{
								tmpangle = 360 - tmpangle;
								SetRobitAngleRun(tmpangle, 1, 30, 0);
							}
						}

					}	
					
//					SetRobitMileageRun(160, 1, 160);
//					PathPlanStep = 3;
				}
				break;
			case 5:
//				if(0 == RobitStatus)
//				{
//					if(RightBump || LeftBump)
//					{
//						
//					}
//				}
				break;
			case 6:
				if(delaycnt < 100)
				{
					delaycnt ++;
				}
				else
				{
					PathPlanStep = 2;
					delaycnt = 0;
					
					//有碰撞，则后退 65mm
					SetRobitMileageRun(73, 2, ROBOT_RUN_SPEED);
				}
			
				//等待2s
				break;
			default:
				break;
		}
	}
	else if(3 == PathPlanMode)
	{
//		BorderWork_irobot();
//        ClosetoWall();
        CloseToWall2();
//        BorderWork();
//        RegRechargeStand();
	}
	else if(4 == PathPlanMode)
	{
//		ReplaceDishcloth();	
//        RegRechargeStand();
        BowShapeClean();
	}
	else if(5 == PathPlanMode)
	{
		//直行30cm,然后开始工作
		switch(PathPlanStep)
		{
			case 0:
				RagChangeEn = ENABLE;
				SysFirstAngle = Sys_Angle;
				SetRobitMileageRun(300, 1, ROBOT_RUN_SPEED);			
				PathPlanStep = 1;
				break;
			case 1:
				if(0 == RobitStatus)
				{
					//打开清扫
					CleanEn = ENABLE;
					
					//打开吸尘
					DustEn = ENABLE;			
					
					//打开滴水
					SprayWaterEn = ENABLE;
				}
				break;
			default:
				break;
		}
	}
	else if(0 == PathPlanMode)
	{
		PathPlanStep = 0;
		delaycnt = 0;
//		FlagFirstTurn = 0;
	}
	
	
}

//回充控制
void RegRechargeStand(void)
{
	uint8_t tmpsta;
//	static uint8_t Reg_Cnt = 0;
	
	tmpsta = (Indr[1].RevOK) << 2 | (Indr[2].RevOK) << 1 | (Indr[3].RevOK);
	
	switch(PathPlanStep)
	{
		case 0:
			// 第一步，旋转扫地机，接收到充电座信号停止
			SetRobitSpeed(1100,-1100);
			if(0 != tmpsta)
			{
				SetRobitSpeed(0,0);
				PathPlanStep = 1;
			}
			break;
		case 1:
			// 角度校准
//			if(0 == Indr[2].RevOK && Indr[3].RevOK)
//			{
//				SetRobitSpeed(50,-50);
//			}
//			else if(Indr[2].RevOK && 0 == Indr[3].RevOK)
//			{
//				SetRobitSpeed(-50,50);
//			}
//			else if(Indr[2].RevOK && Indr[3].RevOK)
//			{
//				SetRobitSpeed(-50,-50);
//			}
			
			if(3 == tmpsta || 7 == tmpsta)
			{
				SetRobitSpeed(-885,-885);
			}
			else if(4 == tmpsta)
			{
				SetRobitSpeed(-885,-885);
				
			}
			else if(1 == tmpsta)
			{
				SetRobitMileageRun(25, 2, 50);
				PathPlanStep = 3;
				
//				Reg_Cnt = 0;
			}
			else if(2 == tmpsta)
			{
				SetRobitMileageRun(25, 2, 50);
				PathPlanStep = 3;
//				Reg_Cnt = 0;
			}
			else if(5 == tmpsta)
			{
				SetRobitSpeed(0,-1600);
//				Reg_Cnt = 0;
			}
			else if(6 == tmpsta)
			{
				SetRobitSpeed(-1600,0);
//				Reg_Cnt = 0;
			}
			else if(0 == tmpsta)
			{
				PathPlanStep = 0;
//				Reg_Cnt = 0;
			}
			
			if(1 == SocketPower)
			{
//				Reg_Cnt ++;
//				if(Reg_Cnt > 50)
//				{
					PathPlanStep = 2;
//				}
			}
			break;
		case 2:
			SetRobitSpeed(0,0);
			break;
		case 3:
			if(0 == RobitStatus)
			{
				PathPlanStep = 4;
				if(1 == tmpsta)
				{
					SetRobitAngleRun(360,2,30, 0);
				}
				else
				{
					SetRobitAngleRun(360,1,30, 0);
				}
			}
			break;
		case 4:
			if(0 == RobitStatus)
			{
				SetRobitMileageRun(25, 2, 50);
				PathPlanStep = 3;
			}
			else
			{
				if(tmpsta & 0x04)
				{
					PathPlanStep = 1;
				}
			}
			break;
		default:
			break;
	}
}


void ReplaceDishcloth(void)
{
	//1.旋转到初始方向 逆时针90°
	//2.直行30cm
	//3.旋转180°
	//4.丢弃抹布
	//5.直行30cm
	//6.寻找充电座(切换到模式3)
	uint16_t tmpangle;
	int16_t tmpangleinit;
	
	switch(PathPlanStep)
	{
		case 0:
			//
		
			RagChangeEn = ENABLE;
		
			tmpangle = CalAngleDif(Sys_Angle,SysFirstAngle);
			tmpangle /= 100;
			if(tmpangle <= 90)
			{				
				SetRobitAngleRun(tmpangle + 90, 1, 30, 0);
			}
			else if(tmpangle <= 270)
			{
				SetRobitAngleRun(270 - tmpangle, 2, 30, 0);
			}
			else
			{
				SetRobitAngleRun(tmpangle - 270, 1, 30, 0);
			}
			PathPlanStep = 1;
			break;
		case 1:
			//判断旋转后的方向是否正确，然后前进30cm
			if(0 == RobitStatus)
			{
				tmpangle = CalAngleDif(SysFirstAngle,Sys_Angle);
				if(tmpangle >= 10000 || tmpangle <= 8000)
				{
					tmpangle /= 100;
					if(tmpangle >= 100)
					{
						SetRobitAngleRun(tmpangle - 90, 2, 30, 0);
					}
					else if(tmpangle <= 80)
					{
						SetRobitAngleRun(90 - tmpangle, 1, 30, 0);
					}
					PathPlanStep = 1;
				}
				else
				{
					if(SysFirstAngle >= -9000)
					{
						tmpangleinit = SysFirstAngle - 9000;
					}
					else
					{
						tmpangleinit = SysFirstAngle + 27000;
					}
					SetRobitMileageRun(500, 1, ROBOT_RUN_SPEED);
					Sys_Startangle = tmpangleinit;
					
					PathPlanStep = 2;
				}
			}
			break;
		case 2:
			//旋转180°
			if(0 == RobitStatus)
			{
				SetRobitAngleRun(180, 2, 30, 0);
				PathPlanStep = 3;
			}
			break;
		case 3:
			//判断旋转是否完成
			if(0 == RobitStatus)
			{
				tmpangle = CalAngleDif(Sys_Angle, SysFirstAngle);
				if(tmpangle >= 10000 || tmpangle <= 8000)
				{
					tmpangle /= 100;
					if(tmpangle >= 100)
					{
						SetRobitAngleRun(tmpangle - 90, 1, 30, 0);
					}
					else if(tmpangle <= 80)
					{
						SetRobitAngleRun(90 - tmpangle, 2, 30, 0);
					}
					PathPlanStep = 3;
				}
				else
				{
					//旋转完成，丢弃拖布
					RagChangeEn = DISABLE;

					// 直行30cm
					if(SysFirstAngle >= 9000)
					{
						tmpangleinit = SysFirstAngle - 27000;
					}
					else
					{
						tmpangleinit = SysFirstAngle + 9000;
					}
					SetRobitMileageRun(500, 1, ROBOT_RUN_SPEED);
					Sys_Startangle = tmpangleinit;
					PathPlanStep = 4;
				}
			}
			break;
		case 4:
			if(0 == RobitStatus)
			{
				//切换模式到 寻找充电座模式
				PathPlanMode = 3;
				PathPlanStep = 0;
			}
			break;
		default:
			break;
	}

}

void BorderWork(void)
{
    static uint8_t turncnt = 0;
    switch(PathPlanStep)
    {
        case 0:
            if(0 == RobitStatus)
            {
                //直行直到碰到障碍物
                if((!RightBump) && (!LeftBump))
                {    
                    SetRobitMileageRun(ROBOT_LINE_DISATNCE, 1, ROBOT_RUN_SPEED);
                }
                else
                {
                    //后退
 //                   SetRobitMileageRun(60, 2, ROBOT_RUN_SPEED);
                    PathPlanStep = 1;
                }
            }
            break;
        case 1:
            if(0 == RobitStatus)
            {
//                if(RightBump || !LeftBump)
//                {
//                    //转动一个小的角度
//                    SetRobitAngleRun(10, 1, 10, 0);
//                }
//                else
//                {
//                    SetRobitAngleRun(20, 1, 10, 0);
//                }
                SetRobitAngleRun(15, 1, 30, 2);
                PathPlanStep = 2;
            } 
            else 
            {
            }                
            break;
        case 2:
            if(0 == RobitStatus)
            {
                //旋转中碰撞
//                if(RightBump | LeftBump)
//                {
//                    SetRobitMileageRun(10, 2, ROBOT_RUN_SPEED);
//                }
//                else
//                {
//                    //直行15cm
//                    SetRobitMileageRun(150, 1, ROBOT_RUN_SPEED);
//                    PathPlanStep = 3;
//                }
                SetRobitAngleRun(15, 1, 30, 1);
                turncnt++;
                if(turncnt < 3)                   
                    PathPlanStep = 1;
                else
                    PathPlanStep = 3;
            }
            break;
        case 3:
//            if(0 == RobitStatus)
//            {
//                uint8_t angleset;
//                
//                angleset = (6 - (RightCurMileage >> 7)) * 3;
//                
//                if(RightBump | LeftBump)
//                {
//                    SetRobitAngleRun(angleset, 1, 10, 0);
//                    PathPlanStep = 2;
//                }
//                else
//                {
//                    SetRobitAngleRun(1, 2, 10, 0);
//                    PathPlanStep = 2;
//                }
//                
//            }
            break;
        case 4:          
            break;
        default:
            break;

    }
    
}

void BorderWork_irobot(void)
{
    static uint8_t turndir = 0,bumpnum = 0, ModeInit = 0, turncnt = 0, cleandir = 0;
    static int16_t ModeStartAngle = 0,tmpstartangle; 
    static pos_t   pos_bump;
    uint16_t tmpangle;
    
    if(!ModeInit)
    {
        ModeInit = 1;
        ModeStartAngle = Sys_Angle;
        turncnt = 0;
    }
    switch(PathPlanStep)
    {
        case 0:
            ///1-----直行 直到碰到障碍物
            if(0 == RobitStatus)
            {

                if((bumpnum & 0x01) == 0x01)
                {
                    if(ModeStartAngle > 0)
                    {
                        Sys_Startangle = ModeStartAngle -18000;
                    }
                    else
                    {
                        Sys_Startangle = ModeStartAngle + 18000;
                    }
                }
                else
                {
                    Sys_Startangle = ModeStartAngle;
                }   

 
					
                tmpangle = CalAngleDif(Sys_Startangle,Sys_Angle);
                if(tmpangle <= 1000 || tmpangle >= 35000)
                {
                    tmpstartangle = Sys_Startangle;
                    
                    SetRobitMileageRun(ROBOT_LINE_DISATNCE, 1, ROBOT_RUN_SPEED);
                    turncnt ++;
                    Sys_Startangle = tmpstartangle;
                    PathPlanStep = 6;
                }
                else
                {
                    //20170308 转弯后退 20
                    if(RightBump == 1 || LeftBump == 1)
                    {
                        SetRobitMileageRun(20, 2, 110);
                    }
                    else
                    {
                        tmpangle /= 100;
                        if(tmpangle <= 180)
                        {
                            SetRobitAngleRun(tmpangle, 2, 30, 0);
                        }
                        else
                        {
                            tmpangle = 360 - tmpangle;
                            SetRobitAngleRun(tmpangle, 1, 30, 0);
                        }
                    }

                }	
            }
            break;
        case 6:
            if(0 == RobitStatus)
            {
                if(RightBump | LeftBump)
                {
                    SetRobitMileageRun(60, 2, ROBOT_RUN_SPEED);                                      
//                    bumpnum = 0;                  
                }
                PathPlanStep = 1; 
            }
            break;
        case 1:
            if(0 == RobitStatus)
            {
                //轴心转 90 度
                bumpnum ++;
                if(bumpnum % 2)
                {
                    if(cleandir == 0)
                    {
                        turndir = 2;
                    }
                    else
                    {
                        turndir = 1;
                    }
                }
                else
                {
                    if(cleandir == 0)
                    {
                        turndir = 1;
                    }
                    else
                    {
                        turndir = 2;
                    }
                }
                SetRobitAngleRun(90, turndir, 30, 0);
                PathPlanStep = 2;
            }
            break;
        case 2:
            if(0 == RobitStatus)
            {
                // 有碰撞 后退 -
                if(RightBump | LeftBump)
                {
                    //后退 
                    SetRobitMileageRun(15, 2, ROBOT_RUN_SPEED);                   
                }
                // 无碰撞 偏心转 90度
                else
                {
                    //判断轴心转90度是否完成
                    if(cleandir == 0)
                    {
                        if(SysInitAngle > 9000)
                        {
                            Sys_Startangle = ModeStartAngle - 27000;
                        }
                        else
                        {
                            Sys_Startangle = ModeStartAngle + 9000;
                        }
                    }
                    else
                    {
                        if(SysInitAngle > -9000)
                        {
                            Sys_Startangle = ModeStartAngle - 9000;
                        }
                        else
                        {
                            Sys_Startangle = ModeStartAngle + 27000;
                        }
                    }
					
					tmpangle = CalAngleDif(Sys_Startangle,Sys_Angle);
					if(tmpangle <= 1000 || tmpangle >= 35000)
					{
                        ///////
                        SetRobitAngleRun(90, turndir, 30, 1);
                        pos_bump.x = Motion_Pos.x;
                        PathPlanStep = 3;
					}
					else
					{
                        tmpangle /= 100;
                        if(tmpangle <= 180)
                        {
                            SetRobitAngleRun(tmpangle, 2, 30, 0);
                        }
                        else
                        {
                            tmpangle = 360 - tmpangle;
                            SetRobitAngleRun(tmpangle, 1, 30, 0);
                        }

					}
                    
                    
                }
            }
            break;
        case 3:
            if(0 == RobitStatus)
            {
                //偏心转过程中碰撞 后退 10mm
                if(RightBump | LeftBump)
                {
                    SetRobitMileageRun(10, 2, ROBOT_RUN_SPEED);
                    PathPlanStep = 4;
                }
                else
                {
                    //判断旋转是否完成                  
                    
                    //无碰撞 直行
                    //SetRobitMileageRun(ROBOT_LINE_DISATNCE, 1, ROBOT_RUN_SPEED);
                    PathPlanStep = 0;
                }
            }
            break;
        case 4:
            if(0 == RobitStatus)
            {
                // 轴心转 15度
                SetRobitAngleRun(15, turndir, 30, 0);
                PathPlanStep = 5;
            }
            break;
        case 5:
            if(0 == RobitStatus)
            {
                if(RightBump | LeftBump)
                {
                    //轴心转过程中碰撞
                    SetRobitMileageRun(5, 2, ROBOT_RUN_SPEED);
                }
                else
                {
                    //直行 10mm
                    SetRobitMileageRun(10, 1, ROBOT_RUN_SPEED);
                    PathPlanStep = 7;
                }
            }
            break;
        case 7:
            // 偏心转
            if(0 == RobitStatus)
            {
                uint8_t tmpdir;
                if(turndir == 1)
                {
                    tmpdir = 2;
                }
                else
                {
                    tmpdir = 1;
                }
                if(RightBump | LeftBump)
                {
                    SetRobitMileageRun(5, 2, ROBOT_RUN_SPEED);
                }
                else
                {
                    SetRobitAngleRun(90, tmpdir, 30, 1);
                    PathPlanStep = 8;
                }
            }
            break;
        case 8:
            if(0 == RobitStatus)
            {
                //计算横向偏移 
                if((cleandir == 0 && Motion_Pos.x - pos_bump.x > 13700) || (cleandir == 1 && pos_bump.x - Motion_Pos.x > 13700))
                {
                    PathPlanStep = 0;
                }
                else
                {
                    if((turndir == 2 && RightBump) || (turndir == 1 && LeftBump))
                    {
                        PathPlanStep = 9;
                    }
                    else
                    {
                        PathPlanStep = 3;
                    }
//                    if((bumpnum & 0x01) == 0x01)
//					{
//						if(ModeStartAngle > 0)
//						{
//							Sys_Startangle = ModeStartAngle -18000;
//						}
//						else
//						{
//							Sys_Startangle = ModeStartAngle + 18000;
//						}
//					}
//					else
//					{
//						Sys_Startangle = ModeStartAngle;
//					} 
//                    
//                    tmpangle = CalAngleDif(Sys_Startangle,Sys_Angle);
//					if(tmpangle <= 1000 || tmpangle >= 30000)
//					{
//                        PathPlanStep = 9;
//                    }
//                    else
//                    {
//                        PathPlanStep = 3;
//                    }                   
                                       
                }
            }
            break;
        case 9:
            // 旋转90度 逆时针
            
            if(ModeStartAngle > -9000)
            {
                Sys_Startangle = ModeStartAngle - 9000;
            }
            else
            {
                Sys_Startangle = ModeStartAngle + 27000;
            }
            
            tmpangle = CalAngleDif(Sys_Startangle,Sys_Angle);
            if(tmpangle <= 1000 || tmpangle >= 35000)
            {
                uint16_t tmpdistance;
                ///////
                if(turncnt > 1)
                {
                    tmpdistance = 137 * (turncnt - 1);
                    SetRobitMileageRun(tmpdistance, 1, ROBOT_RUN_SPEED);                    
                }
                
                if(cleandir == 0)
                {
                    cleandir = 1;
                    bumpnum = 0;
                    turncnt = 0;
                    PathPlanStep = 0;
                }
                else
                {

                }
                
            }
            else
            {
                tmpangle /= 100;
                if(tmpangle <= 180)
                {
                    SetRobitAngleRun(tmpangle, 2, 30, 0);
                }
                else
                {
                    tmpangle = 360 - tmpangle;
                    SetRobitAngleRun(tmpangle, 1, 30, 0);
                }

            }
            break;
    }
}

int32_t AngleFormat(int32_t angle)
{
    int32_t angletmp;
    angletmp = angle;
    while(angletmp > 18000)
    {
        angletmp -= 36000;
    }
    
    while(angletmp < -18000)
    {
        angletmp += 36000;
    }
    
    return (int16_t)angletmp;
}


void boundary(uint8_t boundarydir)
{
    static uint8_t init = 0;
    uint16_t ADC_Singal;
    int16_t walladcdif;
    

    ADC_Singal = PSD_Distance;//PSD_Distance;//
    
//    if(++boundcnt < 3)
//    {
//        infraredsingal += ADC_Singal;
//        return;
//    }
//    infraredsingal = infraredsingal / (boundcnt - 1);
//    boundcnt = 0;
    
    if(/*RightCurMileage >= 1000 && */!init)
    {
        init = 1;
        InitVar_PID(4);
    }
    
    
    walladcdif = CalculPID(REDSINGALDIS, ADC_Singal, 4);

//    test1 = walladcdif;
//    Sys_Startangle -= walladcdif;

//    Sys_Startangle = AngleFormat(Sys_Startangle);   

//    walladcdif = ImprovePid((int16_t)REDSINGALDIS - (int16_t)ADC_Singal);  

    AnglerateTarg = -walladcdif;
}

void ClosetoWall(void)
{
    switch(PathPlanStep)
    {
        case 0:
            SetRobitMileageRun(10000, 1, 110);
            RobitRunbyRouteMode = 2;
            PathPlanStep = 1;
            break;
        case 1:
            if(0 == RobitStatus)
            {
                //根据停止原因进行策略执行
//                if(RightBump && !LeftBump)
//                {
//                    //左轮单轮后退 10°
//                    SetRobitAngleRun(10, 1, 30, 2); 
//                    PathPlanStep = 4;
//                }
//                else if(RightBump || LeftBump)
//                {
//                    SetRobitMileageRun(70, 2, 110); 
//                    PathPlanStep = 5;                   
//                }
//                else
//                {
//                    PathPlanStep = 0;
//                }
                
                if(BumpBak == 0x03 && Ultra_RevTimeVlid[0] >= 450)
                {
                    //左轮单轮后退 10°
                    SetRobitAngleRun(10, 1, 30, 2); 
                    PathPlanStep = 4;
                }
                else if(BumpBak != 0)
                {
                    SetRobitMileageRun(70, 2, 110); 
                    PathPlanStep = 5; 
                }
                else
                {
                    PathPlanStep = 0;
                }
                
                BumpBak = 0;
                
            }
            else
            {
                //沿边传感器检测不到墙或离墙较远
//                if(ADC_ConvertedValue_Val[7] <= PSD_WALL_DIS_MAX)
                if(PSD_Distance > PSD_WALL_DIS_MAX)
                {
                    //停止前进
                    SetRobitSpeed(0, 0);
                    PathPlanStep = 2;
                }
            }
            break;
        case 2:
            if(0 == RobitStatus)
            {
                SetRobitLineAnglerate(2000, 3400);
                PathPlanStep = 3;
            }
            break;
        case 3:
            if(0 == RobitStatus)
            {
//                if(RightBump && !LeftBump)
//                {
//                    //左轮单轮后退 10°
//                    SetRobitAngleRun(10, 1, 30, 2); 
//                    PathPlanStep = 4;
//                }
//                else 
//                {
//                    SetRobitMileageRun(70, 2, 110); 
//                    PathPlanStep = 5;
//                }
                
                if(BumpBak == 0x03 && Ultra_RevTimeVlid[0] >= 450)
                {
                    //左轮单轮后退 10°
                    SetRobitAngleRun(10, 1, 30, 2); 
                    PathPlanStep = 4;
                }
                else
                {
                    SetRobitMileageRun(70, 2, 110); 
                    PathPlanStep = 5;
                }
                BumpBak = 0;
            }
            else
            {
                if(ADC_ConvertedValue_Val[7] >= REDSINGALDIS)
                {
                    PathPlanStep = 0;
                }
            }
            break;
        case 4:
            if(0 == RobitStatus)
            {
                PathPlanStep = 0;
            }
            break;
        case 5:
            if(0 == RobitStatus)
            {
                //轴心转 90 度
                SetRobitAngleRun(90, 1, 30, 0);
                PathPlanStep = 6;
            }
            break;
        case 6:
            if(0 == RobitStatus)
            {
                PathPlanStep = 0;
            }
            else
            {
                static uint8_t fall = 0;
                uint8_t trend;
                trend = DataTrendCal(ADC_ConvertedValue_Val[7]);
                if(trend == 2)
                {
                    fall = 1;
                }
                
                if(fall && (trend == 1) && ADC_ConvertedValue_Val[7] > 530)
                {
                    fall = 0;
                    SetRobitSpeed(0, 0);
                }
            }
            break;
    }
}

void CloseToWall2(void)
{
    static uint8_t PSD_Trend = 0;
    static uint16_t SmallDis = 0, EscapeWallDistane = PSD_WALL_DIS_MAX;

    PSD_Trend = DataTrendCal(PSD_Distance);
//    test1 = PSD_Trend;
    switch(PathPlanStep)
    {
        case 0:
            //检测碰撞状态 
            if(0 == RobitStatus)
            {            
                //1.仅右侧碰撞           EarthData[0].OK - right  EarthData[1].OK- left   
                if((BumpBak == 0x02)  || (EarthData[0].OK && !EarthData[1].OK))
                {
                    //前方近距离没有障碍物
                    if(Ultra_RevTimeVlid[0] >= ULTRAOBSDIS_FRONT)
                    {
                        PathPlanStep = 1;
                        
                        //在从其他模式切换到沿墙模式时，
                        //若BumpOverFlag = ENABLE 说明碰撞后退已经执行完成
                        //则跳过后退，直接执行下一步动作
                        if(BumpOverForCloseWallFlag)
                        {
                            SubStep = 1;
                        }
                        else
                        {
                            SubStep = 0;
                        }
                    }
                    //前方近距离有障碍物
                    else
                    {
                        PathPlanStep = 3;
                        
                        //在从其他模式切换到沿墙模式时，
                        //若BumpOverFlag = ENABLE 说明碰撞后退已经执行完成
                        //则跳过后退，直接执行下一步动作
                        if(BumpOverForCloseWallFlag)
                        {
                            SubStep = 1;
                        }
                        else
                        {
                            SubStep = 0;
                        }
                    }
                    
                }
                else if(BumpBak != 0 || UltraObs || EarthData[0].OK || EarthData[1].OK)
                {
                    //两个碰撞都触发
                    PathPlanStep = 3;
                    
                    //在从其他模式切换到沿墙模式时，
                    //若BumpOverFlag = ENABLE 说明碰撞后退已经执行完成
                    //则跳过后退，直接执行下一步动作
                    if(BumpOverForCloseWallFlag)
                    {
                        SubStep = 1;
                    }
                    else
                    {
                        SubStep = 0;
                    }
                }
                else
                {
                    //没有碰撞
                    
                    //1沿墙传感器信号是否有墙               - PSD距离小于MAX2 并且在减小
                    if(PSD_Distance >= EscapeWallDistane && !(PSD_Trend == 2 && PSD_Distance <= PSD_WALL_DIS_MAX2))
                    {
                        //离墙较远 ，旋转
                        PathPlanStep = 4;
                        SubStep = 0;
                        RightCurMileage = 0;
                    }
                    else
                    {
                        //离墙较近，执行贴墙程序
                        
                        SetRobitVacc(ROBOT_VACC);
                        
                        SetRobitMileageRun(10000, 1, ROBOT_CLOSE_WALL_SPEED);
                        RobitRunbyRouteMode = 2;
    //                    PathPlanStep = 0;
                    }
                    
                }
                BumpBak = 0;
                
            }
            else
            {
                //判断是否是扫地机有碰撞发生或悬空发生正在停止，若无则进行障碍物脱离判定否则让机器停止
                if(BumpBak == 0 && (!(EarthData[0].OK || EarthData[1].OK)))
                {
                //沿墙过程中，沿墙传感器的信号太小及离墙太远
//                if(ADC_ConvertedValue_Val[7] < 400)      - PSD距离小于MAX2 并且在减小               
                    if(PSD_Distance >= EscapeWallDistane && !(PSD_Trend == 2 && PSD_Distance <= PSD_WALL_DIS_MAX2))
                    {
                        PathPlanStep = 4;
                        SubStep = 0;
                    }
                }
                else
                {
                    SetRobitSpeed(0, 0);
                }
            }
            
            //test
//            PathPlanStep = 5;
            
            break;
        case 1:
            //仅右侧碰撞，且超声波近距离没有测到障碍物
            //认为右侧边碰撞
            // 1.小后退 10mm
            // 2.左轮单轮旋转向左 10度
        
            EscapeWallDistane = PSD_WALL_DIS_MAX;
            if(0 == RobitStatus)
            {
                if(0 == SubStep)
                {
                    //后退 15mm
                    SetRobitVacc(ROBOT_VACC);
                    SetRobitMileageRun(15, 2, ROBOT_CLOSE_WALL_SPEED);
                    SubStep = 1;
                    
                }
                else if(1 == SubStep)
                {
                    //碰撞传感器松开判定
                    if(1)//!(LeftBump || RightBump ))//|| EarthData[0].OK || EarthData[1].OK))
                    {
                        //左轮单轮旋转向左 10度
                        SetRobitVacc(0);
                        SetRobitAngleRun(10, 1, 30, 2);
                        SubStep = 2;
                        
                        //旋转角度清零
                        RotationAngleCur = 0;
                        
                        //清除碰撞标志，置位碰撞处理结束标志
                        BumpFlag = DISABLE;
                        BumpOverFlag = ENABLE;
                        BumpBak = 0;
                    }
                    else
                    {
                        SubStep = 0;
                    }                 
                }
                else if(2 == SubStep)
                {
                    if(!(BumpBak || EarthData[0].OK || EarthData[1].OK))
                    {
                        if(PSD_Distance <= PSD_WALL_DIS_MAX2)
                        {
                            //前进200mm
                            SetRobitVacc(ROBOT_VACC);
                            SetRobitMileageRun(200, 1, ROBOT_CLOSE_WALL_SPEED);
                            SubStep = 3;
                        }
                        else
                        {
                            PathPlanStep = 0;
                            SubStep = 0;
                        }
                    }
                    else
                    {
                        //左轮单轮旋转向左 10度
                        SetRobitVacc(0);
                        SetRobitAngleRun(10, 1, 30, 2);
                        SubStep = 2;
                        
                        //清除碰撞标志，置位碰撞处理结束标志
                        BumpFlag = DISABLE;
                        BumpOverFlag = ENABLE;
                        
                        ///
                        BumpBak = 0;
                        //判断是否由于撞板脱离障碍物颤动导致的bumpbak信号置位 
                        //如果RightBump = 1 不是颤动导致的 清零旋转角度 重新转10度
                        //RightBump = 0 是由颤动导致的，不清零旋转角度，继续旋转
                        if(RightBump)
                        {
                            //旋转角度清零
                            RotationAngleCur = 0;
                        }
                        else
                        {
                            //不清零
                        }
                    }
                }
                else if(3 == SubStep)
                {
                    //执行完成后，切换为PathPlanStep=0
                    PathPlanStep = 0;
                    SubStep = 0;
                }
            }
            else
            {
                //后退距离大于10mm即切换到下一步
                if(1 == SubStep)
                {
//                    if(!(LeftBump || RightBump || EarthData[0].OK || EarthData[1].OK))
//                    {
                        if(RightCurMileage >= Length2Mileage(10))
                        {
                            //左轮单轮旋转向左 10度
                            SetRobitVacc(0);
                            SetRobitAngleRun(10, 1, 30, 2);
                            SubStep = 2;
                            
                            //旋转角度清零
                            RotationAngleCur = 0;
                            
                            //清除碰撞标志，置位碰撞处理结束标志
                            BumpFlag = DISABLE;
                            BumpOverFlag = ENABLE;
                            BumpBak = 0;
                        }
//                    }
                }
                else if(2 == SubStep)
                {
                    if(!(BumpBak || EarthData[0].OK || EarthData[1].OK))
                    {
                        if(RotationAngleCur >= 800)
                        {
                            if(PSD_Distance <= PSD_WALL_DIS_MAX2)
                            {
                                //前进200mm
                                SetRobitVacc(ROBOT_VACC);
                                SetRobitMileageRun(200, 1, ROBOT_CLOSE_WALL_SPEED);
                                SubStep = 3;
                            }
                            else
                            {
                                SetRobitVacc(ROBOT_VACC);                       
                                SetRobitMileageRun(10000, 1, ROBOT_CLOSE_WALL_SPEED);
                                RobitRunbyRouteMode = 2;
                                PathPlanStep = 0;
                                SubStep = 0;
                            }
                        }
                    }

                }                
                else if(3 == SubStep)
                {
                    if(!(BumpBak || EarthData[0].OK || EarthData[1].OK))
                    {
                        //若已经脱离墙，则转到Step = 0 
                        if(PSD_Distance > PSD_WALL_DIS_MAX2)
                        {
                            PathPlanStep = 0;
                            SubStep = 0;
                            SetRobitVacc(ROBOT_VACC);
                            SetRobitMileageRun(10000, 1, ROBOT_CLOSE_WALL_SPEED);
                            RobitRunbyRouteMode = 2;
                        }
                        //前进50mm即切换到Step = 0 , 默认执行沿墙，后续根据Step = 0的条件进行动作
                        else if(RightCurMileage >= Length2Mileage(50))
                        {
                            PathPlanStep = 0;
                            SubStep = 0;
                            SetRobitVacc(ROBOT_VACC);
                            SetRobitMileageRun(10000, 1, ROBOT_CLOSE_WALL_SPEED);
                            RobitRunbyRouteMode = 2;
                        }
                        
                    }
                }
            }
            break;
        case 2:
            //仅右侧碰撞，且超声波测到前方近距离有障碍物
            //1. 后踢40mm
            //2. 向左旋转 30度
            EscapeWallDistane = PSD_WALL_DIS_MAX;
            if(0 == RobitStatus)
            {
                if(0 == SubStep)
                {
                    //1. 后退55mm
                    SetRobitVacc(ROBOT_VACC);
                    SetRobitMileageRun(55, 2, ROBOT_CLOSE_WALL_SPEED);
                    SubStep = 1;
                    
                }
                else if(1 == SubStep)
                {
                    //碰撞传感器松开判定
                    if(!(LeftBump || RightBump || EarthData[0].OK || EarthData[1].OK))
                    {
                        //2.向左旋转60度
                        SetRobitVacc(0);
                        SetRobitAngleRun(60, 1, 45, 0);
                        SubStep = 2;
                        
                        //清除碰撞标志，置位碰撞处理结束标志
                        BumpFlag = DISABLE;
                        BumpOverFlag = ENABLE;
                    }
                    else
                    {
                        SubStep = 0;
                    }
                }
                else if(2 == SubStep)
                {                    
                    
                    //执行完成后，返回PathPlanStep = 0;
                    
                    PathPlanStep = 0;
                    SubStep = 0;
                }
            }
            break;
            
        case 3:
            //两个碰撞都触发
            // 1. 后退大距离 70mm
            // 2. 旋转70度 向左
            EscapeWallDistane = PSD_WALL_DIS_MAX2;
            if(0 == RobitStatus)
            {
                if(0 == SubStep)
                {
                    //后退80mm
                    SetRobitVacc(ROBOT_VACC);
                    SetRobitMileageRun(80, 2, ROBOT_CLOSE_WALL_SPEED); 
                    SubStep = 1;
                    
                }
                else if(1 == SubStep)
                {
                    //碰撞传感器松开判定
                    if(!(BumpBak || EarthData[0].OK || EarthData[1].OK))
                    {
                        //向左旋转70
                        SetRobitVacc(0);
                        SetRobitAngleRun(90, 1, 45, 0);
                        SubStep = 2;
                        
                        //旋转角度清零
                        RotationAngleCur = 0;
                        
                        //清除碰撞标志，置位碰撞处理结束标志
                        BumpFlag = DISABLE;
                        BumpOverFlag = ENABLE;
                        
                    }
                    else
                    {
                        SubStep = 0;
                    }
                    BumpBak = 0;
                }
                else if(2 == SubStep)
                {
                    //以上执行完，返回PathPlanStep = 0;
                    PathPlanStep = 0;
                    SubStep = 0;
                }
            }
            else
            {
                //后退58mm切换到旋转
                if(1 == SubStep)
                {
                    //如果没有碰撞发生
                    if(!(BumpBak || EarthData[0].OK || EarthData[1].OK))
                    {
                        //后退距离大于60mm 就执行旋转动作
                        if(RightCurMileage >= Length2Mileage(58))
                        {
                            //向左旋转70
                            SetRobitVacc(0);
                            SetRobitAngleRun(90, 1, 45, 0);
                            SubStep = 2;
                            
                            //旋转角度清零
                            RotationAngleCur = 0;
                            
                            //清除碰撞标志，置位碰撞处理结束标志
                            BumpFlag = DISABLE;
                            BumpOverFlag = ENABLE;
                        }
                    }
                }
                //旋转角度大于65度 即切换到step = 0 执行沿边
                else if(2 == SubStep)
                {
                    //如果没有碰撞发生
                    if(!(BumpBak || EarthData[0].OK || EarthData[1].OK))
                    {
                        if(RotationAngleCur >= 6500)
                        {
                            SetRobitVacc(ROBOT_VACC);                       
                            SetRobitMileageRun(10000, 1, ROBOT_CLOSE_WALL_SPEED);
                            RobitRunbyRouteMode = 2;
                            
                            PathPlanStep = 0;
                            SubStep = 0;
                        }
                    }
                }
            }
            break;
        case 4:
            //离墙太远
            EscapeWallDistane = PSD_WALL_DIS_MAX;
            if(0 == RobitStatus)
            {
                if(0 == SubStep)
                {
//                    if(RightCurMileage >= Length2Mileage(40))
//                    {
//                        SmallDis = Length2Mileage(10);
//                    }
//                    else
//                    {
//                        if(PSD_Distance <= PSD_WALL_DIS_MAX2)
//                        {
//                            SmallDis = Length2Mileage(100);
//                        }
//                        else
//                        {
//                            SmallDis = Length2Mileage(50) - RightCurMileage;
//                        }
//                    }
                    SmallDis = Length2Mileage(CLOSEWALLSTEPDIS);
					SetRobitVacc(ROBOT_VACC);
                    SetRobitMileageRun(200, 1, ROBOT_CLOSE_WALL_SPEED);
                    SubStep = 1;
                }
                else 
                {
                    //执行完成后，返回PathPlanStep = 0;
                    PathPlanStep = 0;
                    SubStep = 0;
                }
            }
            else
            {
                //若机器在运行过程中脱离墙
                //执行逼近墙旋转
                if(0 == SubStep)
                {
//                    if(RightCurMileage >= Length2Mileage(40))
//                    {
//                        SmallDis = Length2Mileage(10);
//                    }
//                    else
//                    {
//                        if(PSD_Distance <= PSD_WALL_DIS_MAX2)
//                        {
//                            SmallDis = Length2Mileage(100);
//                        }
//                        else
//                        {
//                            SmallDis = Length2Mileage(50) - RightCurMileage;
//                        }
//                    }
                    SmallDis = Length2Mileage(CLOSEWALLSTEPDIS);
                    SetRobitVacc(ROBOT_VACC);
                    SetRobitMileageRun(200, 1, ROBOT_CLOSE_WALL_SPEED);
                    SubStep = 1;
                    
                }
                else if(1 == SubStep)
                {
                    if(BumpBak == 0 && !(EarthData[0].OK || EarthData[1].OK))
                    {
                        //前进50mm即切换到Step = 2, 
                        if(RightCurMileage >= SmallDis)
                        {
                            SubStep = 2;
                            SetRobitVacc(ROBOT_VACC);
//                            SetRobitWacc(2000);
//                            SetRobitLineAnglerate(800, 4561);
                            SetRobitLineAnglerate(600, 3420);
    //                        SetRobitLineAnglerate(1200, 4561);
    //                        SetRobitLineAnglerate(2300, 3400);
                        }
                    }
                    else
                    {
                        SetRobitSpeed(0, 0);
                    }

                }
                else if(2 == SubStep)
                {
                    
                    if((PSD_Distance  <= REDSINGALDIS2) )//|| (PSD_Trend == 1 && PSD_Distance <= PSD_WALL_DIS_MAX))
                    {
                        //执行完成后，返回PathPlanStep = 0;
                        PathPlanStep = 0;
                        SubStep = 0;
                        
                        if(BumpBak == 0 && !(EarthData[0].OK || EarthData[1].OK))
                        {
                            SetRobitVacc(ROBOT_VACC);                       
                            SetRobitMileageRun(10000, 1, ROBOT_CLOSE_WALL_SPEED);
                            RobitRunbyRouteMode = 2;
                        }
                        else
                        {
                            SetRobitSpeed(0, 0);
                        }
                    }
				}
            }
            break;
        case 5:
            SetRobitMileageRun(10000, 1, ROBOT_CLOSE_WALL_SPEED);
            RobitRunbyRouteMode = 2;
//            SetRobitLineAnglerate(0, 8000);
            PathPlanStep = 6;
            break;
        case 6:
            
            break;
    }
}

#define TEST_LINE_DISTANCE  3000
#define TEST_Horizontal_DISTANCE  230  
void BowShapeClean(void)
{
    static uint8_t SubStep = 0;
    uint16_t tmpangle;
    int16_t tmpstartangle;
    
    if(RobitStatus)
        return;
    
    switch(PathPlanStep)
    {
        case 0:
//            if(0 == SubStep)
//            {
                //方向判断--
                if((BumpCnt & 0x01) == 0x01)
                {
//						tmpangle = SysInitAngle + 180;
                    if(SysInitAngle > 0)
                    {
                        Sys_Startangle = SysInitAngle -18000;
                    }
                    else
                    {
                        Sys_Startangle = SysInitAngle + 18000;
                    }
//						Sys_Startangle = SysInitAngle;
                }
                else
                {
                    Sys_Startangle = SysInitAngle;
                }
                
                tmpangle = CalAngleDif(Sys_Startangle,Sys_Angle);
                if(tmpangle <= 1000 || tmpangle >= 35000)
                {
                    tmpstartangle = Sys_Startangle;
                    SetRobitMileageRun(TEST_LINE_DISTANCE, 1, ROBOT_RUN_SPEED);
//                    SubStep = 1;
                    
                    SubStep = 0;
                    PathPlanStep = 1;
                    Sys_Startangle = tmpstartangle;
                }
                else
                {
                    //20170308 转弯后退 20
                    if(RightBump == 1 || LeftBump == 1)
                    {
                        SetRobitMileageRun(20, 2, 110);
                    }
                    else
                    {
                        tmpangle /= 100;
                        if(tmpangle <= 180)
                        {
                            SetRobitAngleRun(tmpangle, 2, 30, 0);
                        }
                        else
                        {
                            tmpangle = 360 - tmpangle;
                            SetRobitAngleRun(tmpangle, 1, 30, 0);
                        }
                    }

                }
                //
                
//            }
//            else
//            {

                //执行下一步
//                SubStep = 0;
//                PathPlanStep = 1;

//            }
            break;
        case 1:
            if(0 == SubStep)
            {
                //判断是需要左转还是右转
                
                if(BumpCnt & 0x01)
                {
                    //左转
                    PathPlanStep = 3;
                }
                else
                {
                    //右转
                    PathPlanStep = 2;
                }
            }

            break;
        case 2:
            //左转
            if(0 == SubStep)
            {
                DrawArc(-ROBOT_RUN_SPEED / 2 * MMPERS2RPM, -ROBOT_RUN_SPEED  * MMPERS2RPM * RPM2TENmDuPERDSEC, 9000, 10000);
                SubStep = 1;
            }
            else if(1 == SubStep)
            {
                DrawArc(ROBOT_RUN_SPEED / 2 * MMPERS2RPM, -ROBOT_RUN_SPEED  * MMPERS2RPM * RPM2TENmDuPERDSEC, 9000, 10000);
                SubStep = 2;
            }
            else
            {
                SubStep = 0;
                PathPlanStep = 0;
                BumpCnt++;
            }
            break;
        case 3:
            if(0 == SubStep)
            {
                SetRobitAngleRun(90, 2, 60, 0);
                SubStep = 1;
            }
            else if(1 == SubStep)
            {
                if(SysInitAngle > -9000)
                {
                    Sys_Startangle = SysInitAngle - 9000;
                }
                else
                {
                    Sys_Startangle = SysInitAngle + 27000;
                }
                
                tmpangle = CalAngleDif(Sys_Startangle,Sys_Angle);
                if(tmpangle <= 1000 || tmpangle >= 35000)
                {
                    tmpstartangle = Sys_Startangle;
                    SetRobitMileageRun(TEST_Horizontal_DISTANCE, 1, ROBOT_RUN_SPEED);
                    Sys_Startangle = tmpstartangle;
                    SubStep = 2;
                }
                else
                {
                    //20170308 转弯后退 20
                    if(RightBump == 1 || LeftBump == 1)
                    {
                        SetRobitMileageRun(20, 2, 110);
                    }
                    else
                    {
                        tmpangle /= 100;
                        if(tmpangle <= 180)
                        {
                            SetRobitAngleRun(tmpangle, 2, 30, 0);
                        }
                        else
                        {
                            tmpangle = 360 - tmpangle;
                            SetRobitAngleRun(tmpangle, 1, 30, 0);
                        }
                    }

                }
                
            }
            else if(2 == SubStep)
            {
                SetRobitAngleRun(90, 2, 60, 0);
                SubStep = 3;
            }
            else
            {
                SubStep = 0;
                PathPlanStep = 0;
                BumpCnt++;
            }
            break;
    }
}


//返回值 1-上升趋势 2-下降趋势 0-不变趋势
uint8_t DataTrendCal(uint16_t data)
{
    uint16_t avg;
    int32_t sum;
    uint8_t i;
    
    static uint16_t datas[8] = {0}, avg_last = 0;
    static uint8_t  cnt = 0, trendcnt = 0;
    static int8_t trend[8] = {0}; 

    if(cnt < 8)
        datas[cnt] = data;
    else
        datas[(cnt&0x07)]= data;
    
    cnt++;
    if(cnt < 8)
        return 0;
    
    if(cnt >= 128)
        cnt = 8;
    
    sum = 0;
    for(i = 0; i < 8 ;i++)
    {
        sum += datas[i];
    }
    
    avg = sum >> 3;
    sum = 0;
    
    if(avg > avg_last)
    {
        trend[trendcnt] = 1;
    }
    else if(avg < avg_last)
    {
        trend[trendcnt] = -1;
    }
    else
    {
        trend[trendcnt] = 0;
    }
    avg_last = avg;
    trendcnt++;
    if(trendcnt >=8)
        trendcnt = 0;
    
    for(i = 0;i<8;i++)
    {
        sum += trend[i];
    }
    
    if(sum > 2 && data > avg)
    {
        return 1;
    }
    else if(sum < -2 && data < avg)
    {   
        return 2;
    }
    else
    {
        return 0;
    }
}

