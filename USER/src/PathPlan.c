/******************************************************************************

                  ��Ȩ���� (C), 1994-2016, ���ݾ���ŷ�϶�С�ҵ����޹�˾

 ******************************************************************************
  �� �� ��   : PathPlan.c
  �� �� ��   : V1.0
  ��    ��   : tht
  ��������   : 2016��8��12��
  ����޸�   :
  ��������   : ·���滮
  �����б�   :
  �޸���ʷ   :
  1.��    ��   : 2016��8��12��
    ��    ��   : tht
    �޸�����   : �����ļ�

******************************************************************************/

/*----------------------------------------------*
 * ����ͷ�ļ�                                   *
 *----------------------------------------------*/
#include "stm32f10x.h"
/*----------------------------------------------*
 * �ⲿ����˵��                                 *
 *----------------------------------------------*/

/*----------------------------------------------*
 * �ⲿ����ԭ��˵��                             *
 *----------------------------------------------*/

/*----------------------------------------------*
 * �ڲ�����ԭ��˵��                             *
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
 * ȫ�ֱ���                                     *
 *----------------------------------------------*/
uint8_t PathPlanMode, PathPlanStep, SubStep = 0; // 0=δ���� 1=�ر�ģʽ
//uint8_t PathPlanStep; 				// ·���滮����
uint16_t UltraFrontDisMin = 0xffff;			//ǰ��������С���룬
int16_t UltraFrontAngle, SysInitAngle, SysFirstAngle;			// ǰ��������С����ķ����
uint8_t BumpStaBak,BumpCnt = 0;

uint8_t FlagUltraRising[3],FlagUltraFalling[3],FlagRobotTurnOK[3];
FunctionalState PathPlanEn,UltraSignalCalEn;
/*----------------------------------------------*
 * ģ�鼶����                                   *
 *----------------------------------------------*/
pos_t StartPos;									//��ʼλ��
FunctionalState StartPosEn, EndPosCheckEn, EndPosGet;      //��ʼλ��ȷ����־λ������λ�ü��ʹ�ܱ�־λ

uint32_t RobitLineMileage;
int16_t   PathPlanAngle;
/*----------------------------------------------*
 * ��������                                     *
 *----------------------------------------------*/

/*----------------------------------------------*
 * �궨��                                       *
 *----------------------------------------------*/
#define ROBOT_RUN_SPEED		200  // ��λmm/s
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
 �� �� ��  : PathPlan
 ��������  : ·���滮10ms����һ��
 �������  : void  
 �������  : ��j
 �� �� ֵ  : 
 ���ú���  : 
 ��������  : 
 
 �޸���ʷ      :
  1.��    ��   : 2016��8��12��
    ��    ��   : tht
    �޸�����   : �����ɺ���

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
				//��תһȦ
				SetRobitAngleRun(360,1,30, 0); // ����������ת360�� �ٶ�30��/s
				PathPlanStep = 1;
			
				StartPosEn = DISABLE;
				EndPosCheckEn = DISABLE;
				EndPosGet = DISABLE;
			
				RobitLineMileage = 0;
				PathPlanAngle = 0;
				break;
			case 1:
				//Ѱ��ǰ������������Сֵ�ͽǶ�
				if(UltraFrontDisMin > Ultra_RevTime[0])
				{
					UltraFrontDisMin = Ultra_RevTime[0];
					UltraFrontAngle = Sys_Angle;
				}
				
				// ��תһȦ���
				if(0 == RobitStatus)
				{
					//������С�ǶȺ͵�ǰ�ǶȵĲ�ֵ
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
				//ֱ�е�����ϰ���
				if(0 == RobitStatus)
				{
//					tmpdistance = UltraTime2Dis(UltraFrontDisMin);
//					SetRobitMileageRun(tmpdistance, 1, ROBOT_RUN_SPEED);
					SetRobitMileageRun(ROBOT_LINE_DISATNCE, 1, ROBOT_RUN_SPEED);
					PathPlanStep = 3;
				}
			
				break;
			case 3:
				//�ϰ���ǰֹͣ
				if(1 == RobitStatus)
				{
//					if(Ultra_RevTime[0] <= 600)
//					{
//						SetRobitSpeed(0, 0);
//					}
				}
				else
				{
					//��λ��Ϊ�ر����ߵĳ�ʼλ��
					StartPos.x = Motion_Pos.x;
					StartPos.y = Motion_Pos.y;
					StartPosEn = ENABLE;
					
					if(1 == RightBump || 1 == LeftBump)
					{
						//����ײ������� 65mm
						SetRobitMileageRun(65, 2, ROBOT_RUN_SPEED);
						PathPlanStep = 9;
					}
					else
					{
						PathPlanStep = 4;
					
						//��ת90�㣬������ɨ�ػ���ǽƽ��
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
						//��ǰ������ ������
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
						
						// �Һ����� ������
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
							// ǰ���������ݲ�����Χ�ڣ�ת����Ϊ���
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
				// ��ǽ���� 			
				{
					SetRobitMileageRun(ROBOT_LINE_DISATNCE, 1, ROBOT_RUN_SPEED);  // ֱ��4m
					
//					//������������2000���� ��ǽ���ߣ����򱣳�ֱ������
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
				//ǰ�����ϰ���ֹͣ
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
					
					// ��¼ֱ������ʱ���·�������շ���
					if(RightCurMileage > RobitLineMileage)
					{
						PathPlanAngle = Sys_Angle;
						RobitLineMileage = RightCurMileage;
					}
                    
                    
					
					//20170221 ǰ�������������������䣬���������������� ���Һ���������С���趨ֵ ��Ϊ��һ��͹���� 
					if(ENABLE == Ultra_Jump[1] && DISABLE == Ultra_Jump[2] && ENABLE == Ultra_Rising[1] /*&& Ultra_RevTimeVlid[2] <= 1600*/)
					{
						//����ֱ��
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
//						//����ײ������� 65mm
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
							//20170221 ÿ��15�ȳ���ת�䣬�����������ж�ת���ֹ
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
						//����ײ������� 65mm
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
						//����ײ������� 65mm
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
						//����ײ������� 65mm
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
						//��ʱ360ms  4�γ��������ݣ���֤��������ǽЧ��
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
						//����ײ������� 65mm
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
					
					//// 20170221 ȡ����ײ��ͣ����
//					delaycnt = 100;//0;
					
					//����ײ������� 65mm
					SetRobitMileageRun(65, 2, ROBOT_RUN_SPEED);
//				}
			
				//�ȴ�2s
				break;
			
			//���²��� �ر߽��������������ι滮��ʼ�㡣
			//1.��ת�����Ƕ���ֱ�߽Ƕ�(PathPlanAngle)+90,			
			//2.���Ŵ˽Ƕ�ֱ������ײ����			
			//3.���ι滮
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
						//����ײ������� 65mm
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
		
		//20170228 ����ر��Ƿ����  
		
		// 1. ��ʼλ��ȷ�� 
		if(StartPosEn && (!EndPosCheckEn))
		{
			//������������� �����ڳ�ʼ��Χ�� ����Ϊ�����Ѿ����� �����Լ���Ƿ�ջ�
			if((Motion_Pos.x >= StartPos.x + START_POS_RANGE || Motion_Pos.x <= StartPos.x - START_POS_RANGE) || (Motion_Pos.y >= StartPos.y + START_POS_RANGE || Motion_Pos.y <= StartPos.y - START_POS_RANGE))
			{
				EndPosCheckEn = ENABLE;
			}
		}
		
		// 2. ����λ�ü��ʹ��ʱ�������ڽ���λ�ü�ⷶΧ�ڣ���Ϊ����ر�
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
				//ֱ��4m
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
						//20170308 ת����� 20
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
				// ����ײ ��ת ����ײ��ת ����ײ ��ת
				// ��ת180��ֱ�� ,��ת90�� ǰ��һС��,Ȼ�������ת90 Ȼ��ֱ��
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
						//20170308 ת����� 20
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
					
					//����ײ������� 65mm
					SetRobitMileageRun(73, 2, ROBOT_RUN_SPEED);
				}
			
				//�ȴ�2s
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
		//ֱ��30cm,Ȼ��ʼ����
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
					//����ɨ
					CleanEn = ENABLE;
					
					//������
					DustEn = ENABLE;			
					
					//�򿪵�ˮ
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

//�س����
void RegRechargeStand(void)
{
	uint8_t tmpsta;
//	static uint8_t Reg_Cnt = 0;
	
	tmpsta = (Indr[1].RevOK) << 2 | (Indr[2].RevOK) << 1 | (Indr[3].RevOK);
	
	switch(PathPlanStep)
	{
		case 0:
			// ��һ������תɨ�ػ������յ�������ź�ֹͣ
			SetRobitSpeed(1100,-1100);
			if(0 != tmpsta)
			{
				SetRobitSpeed(0,0);
				PathPlanStep = 1;
			}
			break;
		case 1:
			// �Ƕ�У׼
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
	//1.��ת����ʼ���� ��ʱ��90��
	//2.ֱ��30cm
	//3.��ת180��
	//4.����Ĩ��
	//5.ֱ��30cm
	//6.Ѱ�ҳ����(�л���ģʽ3)
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
			//�ж���ת��ķ����Ƿ���ȷ��Ȼ��ǰ��30cm
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
			//��ת180��
			if(0 == RobitStatus)
			{
				SetRobitAngleRun(180, 2, 30, 0);
				PathPlanStep = 3;
			}
			break;
		case 3:
			//�ж���ת�Ƿ����
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
					//��ת��ɣ������ϲ�
					RagChangeEn = DISABLE;

					// ֱ��30cm
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
				//�л�ģʽ�� Ѱ�ҳ����ģʽ
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
                //ֱ��ֱ�������ϰ���
                if((!RightBump) && (!LeftBump))
                {    
                    SetRobitMileageRun(ROBOT_LINE_DISATNCE, 1, ROBOT_RUN_SPEED);
                }
                else
                {
                    //����
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
//                    //ת��һ��С�ĽǶ�
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
                //��ת����ײ
//                if(RightBump | LeftBump)
//                {
//                    SetRobitMileageRun(10, 2, ROBOT_RUN_SPEED);
//                }
//                else
//                {
//                    //ֱ��15cm
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
            ///1-----ֱ�� ֱ�������ϰ���
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
                    //20170308 ת����� 20
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
                //����ת 90 ��
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
                // ����ײ ���� -
                if(RightBump | LeftBump)
                {
                    //���� 
                    SetRobitMileageRun(15, 2, ROBOT_RUN_SPEED);                   
                }
                // ����ײ ƫ��ת 90��
                else
                {
                    //�ж�����ת90���Ƿ����
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
                //ƫ��ת��������ײ ���� 10mm
                if(RightBump | LeftBump)
                {
                    SetRobitMileageRun(10, 2, ROBOT_RUN_SPEED);
                    PathPlanStep = 4;
                }
                else
                {
                    //�ж���ת�Ƿ����                  
                    
                    //����ײ ֱ��
                    //SetRobitMileageRun(ROBOT_LINE_DISATNCE, 1, ROBOT_RUN_SPEED);
                    PathPlanStep = 0;
                }
            }
            break;
        case 4:
            if(0 == RobitStatus)
            {
                // ����ת 15��
                SetRobitAngleRun(15, turndir, 30, 0);
                PathPlanStep = 5;
            }
            break;
        case 5:
            if(0 == RobitStatus)
            {
                if(RightBump | LeftBump)
                {
                    //����ת��������ײ
                    SetRobitMileageRun(5, 2, ROBOT_RUN_SPEED);
                }
                else
                {
                    //ֱ�� 10mm
                    SetRobitMileageRun(10, 1, ROBOT_RUN_SPEED);
                    PathPlanStep = 7;
                }
            }
            break;
        case 7:
            // ƫ��ת
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
                //�������ƫ�� 
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
            // ��ת90�� ��ʱ��
            
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
                //����ֹͣԭ����в���ִ��
//                if(RightBump && !LeftBump)
//                {
//                    //���ֵ��ֺ��� 10��
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
                    //���ֵ��ֺ��� 10��
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
                //�رߴ�������ⲻ��ǽ����ǽ��Զ
//                if(ADC_ConvertedValue_Val[7] <= PSD_WALL_DIS_MAX)
                if(PSD_Distance > PSD_WALL_DIS_MAX)
                {
                    //ֹͣǰ��
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
//                    //���ֵ��ֺ��� 10��
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
                    //���ֵ��ֺ��� 10��
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
                //����ת 90 ��
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
            //�����ײ״̬ 
            if(0 == RobitStatus)
            {            
                //1.���Ҳ���ײ           EarthData[0].OK - right  EarthData[1].OK- left   
                if((BumpBak == 0x02)  || (EarthData[0].OK && !EarthData[1].OK))
                {
                    //ǰ��������û���ϰ���
                    if(Ultra_RevTimeVlid[0] >= ULTRAOBSDIS_FRONT)
                    {
                        PathPlanStep = 1;
                        
                        //�ڴ�����ģʽ�л�����ǽģʽʱ��
                        //��BumpOverFlag = ENABLE ˵����ײ�����Ѿ�ִ�����
                        //���������ˣ�ֱ��ִ����һ������
                        if(BumpOverForCloseWallFlag)
                        {
                            SubStep = 1;
                        }
                        else
                        {
                            SubStep = 0;
                        }
                    }
                    //ǰ�����������ϰ���
                    else
                    {
                        PathPlanStep = 3;
                        
                        //�ڴ�����ģʽ�л�����ǽģʽʱ��
                        //��BumpOverFlag = ENABLE ˵����ײ�����Ѿ�ִ�����
                        //���������ˣ�ֱ��ִ����һ������
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
                    //������ײ������
                    PathPlanStep = 3;
                    
                    //�ڴ�����ģʽ�л�����ǽģʽʱ��
                    //��BumpOverFlag = ENABLE ˵����ײ�����Ѿ�ִ�����
                    //���������ˣ�ֱ��ִ����һ������
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
                    //û����ײ
                    
                    //1��ǽ�������ź��Ƿ���ǽ               - PSD����С��MAX2 �����ڼ�С
                    if(PSD_Distance >= EscapeWallDistane && !(PSD_Trend == 2 && PSD_Distance <= PSD_WALL_DIS_MAX2))
                    {
                        //��ǽ��Զ ����ת
                        PathPlanStep = 4;
                        SubStep = 0;
                        RightCurMileage = 0;
                    }
                    else
                    {
                        //��ǽ�Ͻ���ִ����ǽ����
                        
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
                //�ж��Ƿ���ɨ�ػ�����ײ���������շ�������ֹͣ������������ϰ��������ж������û���ֹͣ
                if(BumpBak == 0 && (!(EarthData[0].OK || EarthData[1].OK)))
                {
                //��ǽ�����У���ǽ���������ź�̫С����ǽ̫Զ
//                if(ADC_ConvertedValue_Val[7] < 400)      - PSD����С��MAX2 �����ڼ�С               
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
            //���Ҳ���ײ���ҳ�����������û�в⵽�ϰ���
            //��Ϊ�Ҳ����ײ
            // 1.С���� 10mm
            // 2.���ֵ�����ת���� 10��
        
            EscapeWallDistane = PSD_WALL_DIS_MAX;
            if(0 == RobitStatus)
            {
                if(0 == SubStep)
                {
                    //���� 15mm
                    SetRobitVacc(ROBOT_VACC);
                    SetRobitMileageRun(15, 2, ROBOT_CLOSE_WALL_SPEED);
                    SubStep = 1;
                    
                }
                else if(1 == SubStep)
                {
                    //��ײ�������ɿ��ж�
                    if(1)//!(LeftBump || RightBump ))//|| EarthData[0].OK || EarthData[1].OK))
                    {
                        //���ֵ�����ת���� 10��
                        SetRobitVacc(0);
                        SetRobitAngleRun(10, 1, 30, 2);
                        SubStep = 2;
                        
                        //��ת�Ƕ�����
                        RotationAngleCur = 0;
                        
                        //�����ײ��־����λ��ײ���������־
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
                            //ǰ��200mm
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
                        //���ֵ�����ת���� 10��
                        SetRobitVacc(0);
                        SetRobitAngleRun(10, 1, 30, 2);
                        SubStep = 2;
                        
                        //�����ײ��־����λ��ײ���������־
                        BumpFlag = DISABLE;
                        BumpOverFlag = ENABLE;
                        
                        ///
                        BumpBak = 0;
                        //�ж��Ƿ�����ײ�������ϰ���������µ�bumpbak�ź���λ 
                        //���RightBump = 1 ���ǲ������µ� ������ת�Ƕ� ����ת10��
                        //RightBump = 0 ���ɲ������µģ���������ת�Ƕȣ�������ת
                        if(RightBump)
                        {
                            //��ת�Ƕ�����
                            RotationAngleCur = 0;
                        }
                        else
                        {
                            //������
                        }
                    }
                }
                else if(3 == SubStep)
                {
                    //ִ����ɺ��л�ΪPathPlanStep=0
                    PathPlanStep = 0;
                    SubStep = 0;
                }
            }
            else
            {
                //���˾������10mm���л�����һ��
                if(1 == SubStep)
                {
//                    if(!(LeftBump || RightBump || EarthData[0].OK || EarthData[1].OK))
//                    {
                        if(RightCurMileage >= Length2Mileage(10))
                        {
                            //���ֵ�����ת���� 10��
                            SetRobitVacc(0);
                            SetRobitAngleRun(10, 1, 30, 2);
                            SubStep = 2;
                            
                            //��ת�Ƕ�����
                            RotationAngleCur = 0;
                            
                            //�����ײ��־����λ��ײ���������־
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
                                //ǰ��200mm
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
                        //���Ѿ�����ǽ����ת��Step = 0 
                        if(PSD_Distance > PSD_WALL_DIS_MAX2)
                        {
                            PathPlanStep = 0;
                            SubStep = 0;
                            SetRobitVacc(ROBOT_VACC);
                            SetRobitMileageRun(10000, 1, ROBOT_CLOSE_WALL_SPEED);
                            RobitRunbyRouteMode = 2;
                        }
                        //ǰ��50mm���л���Step = 0 , Ĭ��ִ����ǽ����������Step = 0���������ж���
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
            //���Ҳ���ײ���ҳ������⵽ǰ�����������ϰ���
            //1. ����40mm
            //2. ������ת 30��
            EscapeWallDistane = PSD_WALL_DIS_MAX;
            if(0 == RobitStatus)
            {
                if(0 == SubStep)
                {
                    //1. ����55mm
                    SetRobitVacc(ROBOT_VACC);
                    SetRobitMileageRun(55, 2, ROBOT_CLOSE_WALL_SPEED);
                    SubStep = 1;
                    
                }
                else if(1 == SubStep)
                {
                    //��ײ�������ɿ��ж�
                    if(!(LeftBump || RightBump || EarthData[0].OK || EarthData[1].OK))
                    {
                        //2.������ת60��
                        SetRobitVacc(0);
                        SetRobitAngleRun(60, 1, 45, 0);
                        SubStep = 2;
                        
                        //�����ײ��־����λ��ײ���������־
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
                    
                    //ִ����ɺ󣬷���PathPlanStep = 0;
                    
                    PathPlanStep = 0;
                    SubStep = 0;
                }
            }
            break;
            
        case 3:
            //������ײ������
            // 1. ���˴���� 70mm
            // 2. ��ת70�� ����
            EscapeWallDistane = PSD_WALL_DIS_MAX2;
            if(0 == RobitStatus)
            {
                if(0 == SubStep)
                {
                    //����80mm
                    SetRobitVacc(ROBOT_VACC);
                    SetRobitMileageRun(80, 2, ROBOT_CLOSE_WALL_SPEED); 
                    SubStep = 1;
                    
                }
                else if(1 == SubStep)
                {
                    //��ײ�������ɿ��ж�
                    if(!(BumpBak || EarthData[0].OK || EarthData[1].OK))
                    {
                        //������ת70
                        SetRobitVacc(0);
                        SetRobitAngleRun(90, 1, 45, 0);
                        SubStep = 2;
                        
                        //��ת�Ƕ�����
                        RotationAngleCur = 0;
                        
                        //�����ײ��־����λ��ײ���������־
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
                    //����ִ���꣬����PathPlanStep = 0;
                    PathPlanStep = 0;
                    SubStep = 0;
                }
            }
            else
            {
                //����58mm�л�����ת
                if(1 == SubStep)
                {
                    //���û����ײ����
                    if(!(BumpBak || EarthData[0].OK || EarthData[1].OK))
                    {
                        //���˾������60mm ��ִ����ת����
                        if(RightCurMileage >= Length2Mileage(58))
                        {
                            //������ת70
                            SetRobitVacc(0);
                            SetRobitAngleRun(90, 1, 45, 0);
                            SubStep = 2;
                            
                            //��ת�Ƕ�����
                            RotationAngleCur = 0;
                            
                            //�����ײ��־����λ��ײ���������־
                            BumpFlag = DISABLE;
                            BumpOverFlag = ENABLE;
                        }
                    }
                }
                //��ת�Ƕȴ���65�� ���л���step = 0 ִ���ر�
                else if(2 == SubStep)
                {
                    //���û����ײ����
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
            //��ǽ̫Զ
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
                    //ִ����ɺ󣬷���PathPlanStep = 0;
                    PathPlanStep = 0;
                    SubStep = 0;
                }
            }
            else
            {
                //�����������й���������ǽ
                //ִ�бƽ�ǽ��ת
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
                        //ǰ��50mm���л���Step = 2, 
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
                        //ִ����ɺ󣬷���PathPlanStep = 0;
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
                //�����ж�--
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
                    //20170308 ת����� 20
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

                //ִ����һ��
//                SubStep = 0;
//                PathPlanStep = 1;

//            }
            break;
        case 1:
            if(0 == SubStep)
            {
                //�ж�����Ҫ��ת������ת
                
                if(BumpCnt & 0x01)
                {
                    //��ת
                    PathPlanStep = 3;
                }
                else
                {
                    //��ת
                    PathPlanStep = 2;
                }
            }

            break;
        case 2:
            //��ת
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
                    //20170308 ת����� 20
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


//����ֵ 1-�������� 2-�½����� 0-��������
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

