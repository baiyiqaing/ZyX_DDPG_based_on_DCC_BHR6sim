#pragma once
#include "stdio.h"
#include "stdlib.h"
#include "malloc.h"
#include "math.h"

// #define USE_RESISTANT_COMPLIANCE
// #define STANDING_PUSH_CHECK
// #define USE_MPC

#define CONTROL_T 0.004	

typedef struct
{
	double e;
	double de;
	double dde;
	double el;
	double del;
	double ddel;
	double r;
	double dr;
	double ddr;
	double rl;
	double drl;
	double ddrl;
}RC_comp_onedirection;

typedef struct
{
	RC_comp_onedirection sagi;
	RC_comp_onedirection late;
}RC_comp;

typedef struct
{
	double px;
	double vx;
	double py;
	double vy;
}State_CoM;

RC_comp_onedirection RC_Lateral(RC_comp_onedirection comp_late, double zmp_x_ref, double zmp_x_rel, double Z_c, double T_filter, double K_RC[4], double T_lag[2], double K_VMM[6], double micro[2], double T_start_walk, int k_pre);
RC_comp_onedirection RC_Sagitta(RC_comp_onedirection comp_sagi, double zmp_y_ref, double zmp_y_rel, double Z_c, double T_filter, double K_RC[4], double T_lag[2], double K_VMM[6], double micro[2], double T_start_walk, int k_pre);
void VMM_RC_controller(double Z_c, int k_pre);
State_CoM State_OB(int k_pre);
