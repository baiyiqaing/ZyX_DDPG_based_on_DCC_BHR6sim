#pragma once
#include "Tra_Generate.h"
#include "stdio.h"
#include "stdlib.h"
#include "malloc.h"
#include "math.h"

//#define USE_COM_REGULATOR
//#define TEST_CHEST_POSTURE_CON
//#define USE_MODEL_ZMP
//#define TUNE_K_ROT
#ifdef TUNE_K_ROT
	#define STANDSTILL
	#undef USE_TPC
#endif

#define H_ROBOT 1.0
#define I_ROBOT 0.3

extern PreCon_Tra Tra_COM_real;
extern PreCon_Tra Tra_VCOM_real;
extern PreCon_Tra Tra_RAnkle;
extern PreCon_Tra Tra_LAnkle;
extern PreCon_Tra Tra_COM;
extern PreCon_Tra Tra_VCOM;
extern PreCon_Tra Tra_ZMP;
extern int Signal_SupportLeg[MAX_ARRAY_NUM];
extern double T_com;
extern double T_walk;
extern double Body_VX;
extern double Body_VY;
extern double H_zc;

typedef struct
{
	double x;
	double y;
}Horizontal_Current;

typedef struct
{
	Horizontal_Current CoM_Posi;
	Horizontal_Current CoM_Velo;
	Horizontal_Current ZMP;
}State_DelayedLIPM;

typedef struct
{
	double fx;
	double fy;
	double fz;
	double tx;
	double ty;
	double tz;
}Foot_FT;

typedef struct
{
	Foot_FT Lfoot;
	Foot_FT Rfoot;
}Feet_FT;

typedef struct
{
	double delta_x;
	double delta_y;
	double delta_z;
	double delta_pit;
	double delta_rol;
	double delta_yaw;
}Foot_comp;

typedef struct
{
	Foot_comp Lfoot;
	Foot_comp Rfoot;
}Feet_comp;

typedef struct
{
	double left;
	double righ;
	double forw;
	double back;
}Sup_Polygon;

typedef struct
{
	double x;
	double y;
	double pitch;
	double roll;
	double dx;
	double dy;
	double dpitch;
	double droll;
	double ddx;
	double ddy;
	double ddpitch;
	double ddroll;
	double x_roll;
	double y_pitch;
	double dde_max;
}Model_ZMP_Con_Value;

typedef struct
{
	double F_x;
	double F_y;
	double F_z;
	double e_x;
	double e_y;
	double e_z;
	double de_x;
	double de_z;
	double de_y;
	double dde_x;
	double dde_y;
	double dde_z;
}Foot_Step_Adjust;

typedef struct
{
	Foot_Step_Adjust Lfoot;
	Foot_Step_Adjust Rfoot;
	double delta_L_x;
	double delta_L_y;
}Feet_Step_Adjust;

typedef struct
{
	double zmp_x_re[500];
	double zmp_y_re[500];
	double zmp_x_ave;
	double zmp_y_ave;
	double zmp_x_ori;
	double zmp_y_ori;
	double k_roll;
	double k_pitch;
	int x_ok;
	int x_count;
	int y_ok;
	int y_count;
	int i_x_recover;
	int i_y_recover;
}AutoTuneKrot;

com_state chest_pitch;
com_state chest_roll;
com_state Lfoot_Rol;
com_state Lfoot_Pit;
com_state Lfoot_Ver;
com_state Rfoot_Rol;
com_state Rfoot_Pit;
com_state Rfoot_Ver;
com_state Zfoot_Ctl;
com_state Lfoot_Ctl;
com_state Rfoot_Ctl;
Feet_FT TauFz; // test
Feet_comp FeetComp;
PreCon_Tra Tra_GCOM; // test
PreCon_Tra Tra_BZMP; // test
PreCon_Tra Tra_GCOM_rel; // test
PreCon_Tra Tra_VCOM_rel; // test
PreCon_Tra Tra_BZMP_rel; // test
PreCon_Tra Tra_GICP;
Horizontal_Current Pd_Star;
Sup_Polygon Sup_Poly;
Horizontal_Current M_model;
Model_ZMP_Con_Value Md_ZMP_PosRot;
Feet_Step_Adjust Step_Adjust;
AutoTuneKrot Tune_Krot;

void Cal_ICP_ref(PreCon_Tra Tra_COM, PreCon_Tra Tra_VCOM, int mod);
Horizontal_Current CoM_controller(State_DelayedLIPM state_ref, State_DelayedLIPM state_rel, double k[3], int k_pre);
Sup_Polygon Get_SupPoly(int k_pre);
Horizontal_Current Pd_Exceed_Distributor(Sup_Polygon sup_poly, double Fz);
Model_ZMP_Con_Value Model_ZMP_Con(Model_ZMP_Con_Value md_posrot, Horizontal_Current Mmodel, double k_hori[2], double k_rot[2], double kx_r[2], double p_friction, double H_robot, double I_robot, double Fz);
Feet_Step_Adjust Model_Step_Adjust(Feet_Step_Adjust step_sdjust, Model_ZMP_Con_Value md_posrot, double k[4], double roll, double pitch, int k_pre);
Position Trans2_Gframe_Cur(Position posi_In, int k_pre);
Feet_FT Pd_TauFz(Horizontal_Current pd_star, int k_pre);
com_state Damping_DT(double D, double T, double Fd, double F, com_state damp);
com_state Damping_PD(double P, double D, double Fd, double F, com_state damp);
Feet_comp GRF_Con(Horizontal_Current pd_star, double k_damp[6], int k_pre);
void Chest_Posture_Con(double pitch_d, double roll_d, double pitch, double roll);
void CoM_Sta10(int k_pre);
void AutoTune_K_Rot(int k_pre);
