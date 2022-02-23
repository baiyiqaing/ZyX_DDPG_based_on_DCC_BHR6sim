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
//PG-�滮�ģ��·��Ĺ켣��   Sens-ʵ�ʵġ����л�����      ConVal-������       Ref-������ֵ���滮��+���Ƶģ���̬��zmp�ȣ�     Cmd-�·���ֵ��Ҳ������ֵ�����ŵ�λ�á��ŵ�λ�õȣ�
//������ʱ������Conframe��������У���Ϊ��ֵ���������棬��ȫ�ֱ����������� zyx read�ĵط���ȡ��Щֵ
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