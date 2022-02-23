#pragma once
#include <stdio.h>
#include <math.h>
#include "Tra_generate.h"
#include "CoM_Stabilizer.h"

//#define USE_UNEVEN_TRAILBLAZER
//#define USE_HUBO_COMPLIANCE

//#define USE_HUBO_COMBO
#ifdef USE_HUBO_COMBO
	#define USE_UNEVEN_TRAILBLAZER
	#define USE_HUBO_COMPLIANCE
#endif 

typedef struct
{
	double a11;
	double a12;
	double a21;
	double a22;
	double b11;
	double b21;
}SS_2x2;

typedef struct
{
	double roll;
	double droll;
	double pitch;
	double dpitch;
}state_IPSD;

typedef struct 
{
	double e;
	double de;
	double dde;
}SD_UPTATE;

typedef struct
{
	SD_UPTATE x;
	SD_UPTATE y;
	SD_UPTATE z;
	SD_UPTATE pit;
	SD_UPTATE rol;
	SD_UPTATE yaw;
}FOOT_BIAS;

typedef struct
{
	FOOT_BIAS Lfoot;
	FOOT_BIAS Rfoot;
}FEET_BIAS;

typedef struct
{
	double F_x;
	double F_y;
	double F_z;
	double T_pit;
	double T_rol;
	double T_yaw;
	double dF_x;
	double dF_y;
	double dF_z;
	double dT_pit;
	double dT_rol;
	double dT_yaw;
}FOOT_FORCE_TORQUE;

typedef struct
{
	FOOT_FORCE_TORQUE Lfoot;
	FOOT_FORCE_TORQUE Rfoot;
}FEET_FORCE_TORQUE;

state_IPSD State_CP;
state_IPSD Theta_Con;
Horizontal_Current GICP_ref;
Horizontal_Current WICP_rel;
Horizontal_Current GICP_rel;
Horizontal_Current GCOM_ref;
Horizontal_Current WCOM_rel;
Horizontal_Current GCOM_rel;
Horizontal_Current delta_p_hat;
Horizontal_Current Pd_CP;
Horizontal_Current GP_ref;
Horizontal_Current GPd_CP;
Horizontal_Current GP_rel;
Position WZMP_rel;
// GP_rel - GPd_CP
Horizontal_Current Delta_COM;

FEET_BIAS HUBO_foot_com;
SD_UPTATE HUBO_foot_ztrl;
FEET_FORCE_TORQUE FEET_FT_HUBO_ref;
FEET_FORCE_TORQUE FEET_FT_HUBO_rel;


Horizontal_Current Get_Pd_CP(Horizontal_Current GICP_ref, Horizontal_Current GICP_rel, Horizontal_Current p_ref, double k_cp[2]);
SS_2x2 c2d_2x2(SS_2x2 SS_IPSD_c);
Horizontal_Current ZMP_Controller(double k_IPSD[2], double G_ob[2], double K_con[2], Horizontal_Current Gp_rel, Horizontal_Current Gpd_cp);
SD_UPTATE Damping_HUBO(SD_UPTATE damp, double Fd[2], double F[2], double k[4], int k_pre);
FEET_BIAS Compliance_HUBO(FEET_BIAS HUBO_foot_com, Horizontal_Current pd_cp, double kz[4], double kt[4], double filter_Fz_T, int k_pre);
void HUBO19(int k_pre);