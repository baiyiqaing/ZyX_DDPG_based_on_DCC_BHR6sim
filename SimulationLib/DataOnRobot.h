#pragma once
#ifndef DATAONROBOT_H
#define DATAONROBOT_H

#include "../BHR7P1_Run_Team_7P2_cmake_multithread/lib/Tra_Generate.h"
#include "../BHR7P1_Run_Team_7P2_cmake_multithread/ChzFrame/ChzFrameCpp2C.h"

#ifdef DATAONROBOT_C
#define Extern 
#else 
#define Extern extern 
#endif

Extern double XS_Pitch, XS_Roll, XS_Yaw;
Extern double XS_AccX, XS_AccY, XS_AccZ;
Extern double XS_GyrX, XS_GyrY, XS_GyrZ;
Extern double Body_VX, Body_VY;
Extern double GQ_Roll, GQ_Pitch;
Extern double Joint[3][7], Joint_Arm[3][8];
Extern double Real_LegTorque[3][7];
Extern int PreCon_Mode;
Extern int Walk_On;

extern JointsAngle PreCon_LegJoint, Real_LegJoint;
extern ForceSensor F_RFoot, F_LFoot;
extern double Ref_Waist_Joint[4];
Extern double FootFT[5][6];
extern int K_Preview_Con;
extern double Ref_Leg_Joint[3][7];
extern double Ref_Arm_Joint[3][8];


#ifdef CHZ_FixedWaistMotion
double Ref_Waist_Pos[6];
#endif

#undef Extern

#endif