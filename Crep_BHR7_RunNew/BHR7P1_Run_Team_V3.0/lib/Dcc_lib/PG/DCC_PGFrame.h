// 20210427 bit
#pragma once
#ifndef DCC_PGFrame_H
#define DCC_PGFrame_H
#ifdef DCC_PGFrame_C
#define Extern
#else
#define Extern extern
#endif

#include "..\Base\dcc_con_base.h"
#include "..\..\Tra_Generate.h"

Extern dccRobotState_tra stTraPG;
Extern int nKprogNow; // Current K in the program 
Extern int nKprogArg; // K that has been arranged for movments
Extern dccAnkle stLastAnkPos;

#ifndef __MaxPlanStepN
#define __MaxPlanStepN 500
#endif

typedef struct {
	int nStepNum;
	char cStartSwiLeg;
	char cStartSupLeg;
	int nSupLeg[__MaxPlanStepN];
	double dTimeReady;
	double dTimeEnd;
	double dStepLength[__MaxPlanStepN];
	double dStepWidth[__MaxPlanStepN];
	double dStepHeight[__MaxPlanStepN];
	double dLandHeight[__MaxPlanStepN];
	double dStepTime[__MaxPlanStepN];
	double dSinPercent[__MaxPlanStepN];
	double dPauseTime[__MaxPlanStepN];
	double dHipHeightSin[__MaxPlanStepN];
	double dHipHeightDou[__MaxPlanStepN];
	double dHipHeightPause[__MaxPlanStepN];
	double dStepForward;
	double dStepBackward;
	double dStepLeftside;
	double dStepRightside;
}dccPlanCmd;

Extern dccPlanCmd stPlanCmd;
Extern double dHolding[2];
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
#ifndef __HipHeightHome
#define __HipHeightHome 0.7368294446
#endif
#ifndef __ZMPWidth
#define __ZMPWidth 0.15
#endif
#ifndef __Holding
#define __Holding dHolding
#endif
#ifndef IsHolding
#define IsHolding(a) ((a[0] == __Holding[0]) && (a[1] == __Holding[1]))
#endif
#ifndef NotHolding
#define NotHolding(a) ((a[0] != __Holding[0]) || (a[1] != __Holding[1]))
#endif
#ifndef __dVoid
#define __dVoid 4.33e6
#endif
#ifndef __cVoid
#define __cVoid 'V'
#endif

// for custom PG
double fndGetAnkLateral(char cFootFlag);
void fnvSetNextStep(double dAnkxyz[3], double dStepHeight, double dStepTime[3], double dHipHeight[3], char cFootFlag, int nMidStepFlag, double dZMPWidth, int nKprogNow);
void fnvSwitchZMP(char cSwiftToWhere, double dHipToHeight, double dFreezeTime, double dZMPWidth, int nKprogNow);

// for main loop
void fnvDccGetPG();
void fnvDccPGUpdate(int nKpre);

// include your online PG methods here
#include "DCC_FreeWalk.h"

#define USE_PLANPG_METHOD
#define USE_DCCPGFRAME
#endif