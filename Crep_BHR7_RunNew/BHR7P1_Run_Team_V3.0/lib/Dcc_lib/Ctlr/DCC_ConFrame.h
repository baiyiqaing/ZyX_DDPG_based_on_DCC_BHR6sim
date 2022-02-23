// 20210109 bit
#pragma once
#ifndef DCC_ConFrame_H
#define DCC_ConFrame_H
#ifdef DCC_ConFrame_C
#define Extern 
#else
#define Extern extern
#endif

#include "..\Base\dcc_con_base.h"
#include "..\..\Tra_Generate.h"

Extern dccRobotState stStatePG, stStateSens, stStateConVal, stStateRef, stStateCmd; // initial states
//PG-规划的（下发的轨迹）   Sens-实际的、传感回来的      ConVal-控制量       Ref-期望的值（规划的+控制的，姿态、zmp等）     Cmd-下发的值（也是期望值）（髋的位置、脚的位置等）
//做文章时，就在Conframe里面读就行，因为赋值都在这里面，是全局变量，可以在 zyx read的地方读取这些值
Extern dccJoints stJoints; // armswi
#undef Extern

#ifndef __Gravity
#define __Gravity 9.8
#endif
#ifndef __ControlT
#define __ControlT CONTROL_T
#endif
#ifndef __MRobot
#define __MRobot m_robot
#endif
#ifndef __MArm
#define __MArm 5.0
#endif
#ifndef __MShank
#define __MShank 2.0
#endif
#ifndef __MThigh
#define __MThigh 9.0
#endif
#ifndef __MFoot
#define __MFoot 1.0
#endif
#ifndef __LArm
#define __LArm 0.4
#endif
#ifndef __LShank
#define __LShank 3.2
#endif
#ifndef __LThigh
#define __LThigh 3.2
#endif
#ifndef __AnkleWidth
#define __AnkleWidth 0.16
#endif
#ifndef __AnkleHeight
#define __AnkleHeight 0.112
#endif

void fnvDccControlInit();
void fnvDccControlUpdate(int nKpre);

#endif