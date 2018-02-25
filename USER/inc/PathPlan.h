#ifndef _PATHPALN_H
#define _PATHPALN_H



extern uint8_t PathPlanMode, PathPlanStep, SubStep;
extern uint16_t UltraFrontDisMin;
extern int16_t UltraFrontAngle,SysInitAngle,SysFirstAngle;
extern uint8_t BumpCnt;
extern FunctionalState PathPlanEn;
extern FunctionalState StartPosEn, EndPosCheckEn, EndPosGet; 
extern int16_t   PathPlanAngle;

extern void PathPlan(void);
extern void boundary(uint8_t boundarydir);
extern int32_t AngleFormat(int32_t angle);

#endif

