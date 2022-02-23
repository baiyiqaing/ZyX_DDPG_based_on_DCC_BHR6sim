#pragma once
// #include "Tra_Generate.h"

//#define USE_DIFF_INV

#define THIGH_L 320.0
#define THIGH_1 100.0
#define THIGH_2 160.0
#define THIGH_3 80.0
#define THIGH_4 185.0
#define THIGH_5	140.0
#define SHANK_L 320.0
#define SHANK_1 60.0
#define SHANK_2 300.0 //327.0
#define ANKLE_3 43.97
#define ANKLE_4 16.11
#define ANKLE_5 60.5
#define ANKLE_6 60.5
#define ALPHA_1 150.0

#define AGL_RESET 25.0

// #ifdef USE_CHZ_RUN
	// #define AGL_RESET 25.0
// #endif
	
typedef struct
{
	double q1;
	double q2;
	double q3;
	double q4;
	double q5;
	double q6;
}JOINTS_AGL;

JOINTS_AGL Q;
JOINTS_AGL Phi;
JOINTS_AGL Phi_reset;
double THETA_ZERO[2];
void cal_zero(double * theta_zero, double mech_paras[14]);
JOINTS_AGL cal_reset(double theta_reset, double mech_paras[14], double * theta_zero);
JOINTS_AGL cal_old2diff(JOINTS_AGL q, double mech_paras[14], double * theta_zero);
void cal_PL_PR(double * PLPR, double ankle_joints[2], double mech_paras[3]);
