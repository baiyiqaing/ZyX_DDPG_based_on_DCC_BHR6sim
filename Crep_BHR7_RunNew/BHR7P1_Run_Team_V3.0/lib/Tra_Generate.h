	#pragma once
#ifndef _TRA_GENERATE_H
#define _TRA_GENERATE_H

/*============================== Include Head Files ==============================*/
#include "stdio.h"
#include "stdlib.h"
#include "malloc.h"
#include "math.h"

#include "..\rtx_process.h"
#include "..\control.h"
#include "leeCpp2C.h"
#include "ChzCpp2C.h"

#include "lee_control_lib\leeSRMotion.h"
// #include <leeSRMotion.h>

#define _ext_ extern

#pragma warning(disable : 4996)
#pragma warning(disable : 4305)
#pragma warning(disable : 4244)

/*============================== Define the Motion of the Robot ==============================*/
//////// DCC ////////
#define USE_DCC_CONTROLFRAME
// ============================== now testing =========================================
//#define USE_RUN_Con 						// DCC Control zongkaiguan
//#define RUN_ConVal_AddiOn
//#define USE_Run_PostureCon				// using
//#define USE_Run_TPC						// using
//#define USE_Foot_TPC 		
//#define USE_Run_Compliance_old 			// using
//#define ADDI_Tor_LIPM					// using
//#define USE_Run_LIPM						// using
	//#define USE_Run_LIPM_0				// using
	//#define USE_Run_LIPM_1		
//#define USE_Contact_Con 					// using
// #define USE_DCC_ARMSWING					// using
//#define STANDSTILL  						// stand
//#define ANGLE_PROTECTION
// ============================== now testing =========================================
// #define DCC_SPLINE_MOTION // use w ith STANDSTILL
//#define TPCMPC
// #define USE_Z_Compliance_2nd
// #define USE_Run_Compliance
// #define USE_VMC
// #define USE_Land_RotCon
// #define USE_FLY_PostureCon
// #define USE_FLY_STEPZ // stepz
// #define USE_BALANCE_PRO // zhegekeyiyou
// #ifdef TPCMPC
	// #undef USE_Run_TPC
// #endif
// #ifdef USE_Run_TPC
	// #undef USE_Foot_TPC
// #endif
//#define USE_DIFF_INV // must def this!!!!!!!!

//////// CHZ ////////
//#define USE_CHZ_RUN
//#define USE_CHZ_BIAS
// #define USE_CHZ_AUTOBIAS
#define USE_ZMPMAPPING
//#define USE_CHZ_LANDING
// #define USE_CHZ_LOWLEVELFOOTSTEP
// #define USE_CHZ_FOOTDOWN
// #define USE_CHZ_SWINGFOOT
#ifdef USE_CHZ_RUN
	// #define USE_SDFAST_MF
	// #define USE_CHZ_WAISTCOMP
#endif
#define ANKLE_HEIGHT_CHZ 0.112
//////// CHZ ////////

// #define USE_NQP_BALANCE
// #define USE_ANKLECOMPLIANCE  //stand  // only x
// #define USE_STACOMPLIANCE
//#define stop_judgment
//#define USE_ZCOMPLIANCE 

//#define USE_TPC //stand
// #define USE_WAIST_COMPENSATION  //walk
// #define USE_WAIST_COMP_BHR7TEST
//#define USE_FOOT_DOWN   //walk
//#define USE_ZC_CHANGE
#define y_bias -0.0;

//#define USE_ANKLE_CYCLOID
//#define USE_SLQR_METHOD
// #define USE_WAVE_ARM
//#define FZ_BIAS
#define bias_micro 0.34


/*============================== Define the Physical Parameters ==============================*/
#define CONTROL_T 0.004		        // control period
#define GRAVITY 9.8         
#define THIGH 0.32//0.33                //0.35  // thigh length and shank length
#define ANKLE_WIDTH 0.16            // distance between two ankle joint
#define H_RESET 0.7368294446//0.8013654271085//0.730             //0.8013654271085	    //hip height while reset // 0.8013654271085  // 0.825365427108546
#define H_ANKLE 0.112//0.08              //0.112  // ankle joint height    // 0.112            // 0.136
#define FS_HIGHT -0.0874            // force sensor hight relative to ankle
#define SHOUDLER_WIDTH 0.4
#define FOOT_DISTANCE 0.16

#define ARM1_RESET ( 0.0*DEG2RAD)	// reset angle of arm joint 1 ,  0 deg
#define ARM4_RESET (50.0*DEG2RAD)	// reset angle of arm joint 1 , 50 deg
#define ARM1_WAVE  (20.0*DEG2RAD)	// wave angle of arm joint 1 ,  20 deg
#define ARM4_WAVE  (25.0*DEG2RAD)	// wave angle of arm joint 1 ,  25 deg

#define FZ_MIN_LIMIT (0.10*40.0*9.8)   // 0.1*weight 59.8
#define ALPHA_X 1.4          //0.8 //  coefficient of TPC equation
#define ALPHA_Y 1.8          //0.8 //  coefficient of TPC equation

#define PAI 3.1415926
#define DEG2RAD  0.0174533
#define RAD2DEG 57.2957795
#define MAX_ARRAY_NUM 30000         // the length of the trajectory array

#define H_hip_H 0.71

#define Shank 0.35
#define Thigh 0.35
#define Body 0.55
#define Arm 0.5
#define Waist 0.16
#define ankle_width Waist
#define m_foot 1.12
#define m_robot 44.6

//rc
typedef struct
{
	double e;
	double de;
	double el;
	double del;
	double r;
	double dr;
	double rl;
	double drl;
}com_state;

typedef struct
{
	com_state la;
	com_state sa; 
}DCC_StandingCompliance;
//

typedef struct
{
	double fx[MAX_ARRAY_NUM];
	double fy[MAX_ARRAY_NUM];
	double fz[MAX_ARRAY_NUM];
	double tx[MAX_ARRAY_NUM];
	double ty[MAX_ARRAY_NUM];
	double tz[MAX_ARRAY_NUM];
}FootFTDesired;
typedef struct
{
	double fx;
	double fy;
	double fz;
	double tx;
	double ty;
	double tz;
}FootFT_dcc;
typedef struct
{
	FootFT_dcc Rfoot;
	FootFT_dcc Lfoot;
}FeetFT;

typedef struct
{
	double x;
	double y;
}Tau_direction;
typedef struct
{
	Tau_direction Rfoot;
	Tau_direction Lfoot;
}Tau_struct;
typedef struct
{
	double Rfoot;
	double Lfoot;
}F_struct;
typedef struct
{
	F_struct F;
	Tau_struct Tau;
}Desired_FOOTFT;
typedef struct
{
	double q1, q2, q3, q4, q5, q6, q7;
	double dq1, dq2, dq3, dq4, dq5, dq6, dq7;
	double ddq1, ddq2, ddq3, ddq4, ddq5, ddq6, ddq7;
}Joints_sa;
typedef struct
{
	double q1, q2, q3;
	double dq1, dq2, dq3;
	double ddq1, ddq2, ddq3;
}Joints_la;
typedef struct
{
	Joints_sa sa;
	Joints_la la;
}Joints_state;
typedef struct
{
	double m1;
	double m2;
	double m3;
	double m4;
	double m5;
	double m6;
	double m7;
	double m8;
}m_sa;
typedef struct
{
	double m1;
	double m2;
	double m3;
	double m4;
}m_la;
typedef struct
{
	double q1;
	double q2;
	double q3;
	double q4;
	double q5;
	double q6;
}Joints_ori;

typedef struct
{
	double fx;
	double fy;
	double fz;
	double tx;
	double ty;
	double tz;
}FOOTFT_DCC;

typedef struct {
	double x;
	double y;
	double xl;
	double yl;
	double dx;
	double dy;
	double dxl;
	double dyl;
}Test_Tra;

/*============================== Declare the Variables ==============================*/
typedef struct{
	double _T_step;          // step period, unit：s
	double _L_step;          // step length, unit: m  !!! NOTE: walk_speed = L_step / T_step *3.6, unit: km/h
	int    _N_step;          // steps
	double _ZMP_width;       // the whole width that ZMP moves along x direction
	double _H_hip;           // hip height while walking
	double _H_step;          // foot lifting height
	double _L_step_single;   // the length that ZMP move along y direction in single phase
	double _Pct_dou;         // the percentage of double phase to step period, i.e., T_dou = Pct_dou * T_step
	double _H_zc;            // the COM height in LIP model, which determines 'Gp_0_8.h', 'Gp_0_9.h' or 'Gp_1_0.h' used
	double _H_minhi;         // the height of ankle that the foot lift more than the floor in the end of the single phase, unit: m
	double _H_minlow;        // the height of ankle that the foot lift less than the floor in the end of the double phase, unit: m
}PreCon_Walk_Parameters;

typedef struct{
	double x[MAX_ARRAY_NUM];
	double y[MAX_ARRAY_NUM];
	double z[MAX_ARRAY_NUM];
}PreCon_Tra;

typedef struct
{
	double px;
	double py;
	double pz;
}Position;

typedef struct
{
	double fx;
	double fy;
	double fz;
	double tx;
	double ty;
	double tz;
}ForceSensor;

typedef struct
{
	double qr[7];
	double ql[7];
}JointsAngle;

typedef struct 
{
	double jr[7][MAX_ARRAY_NUM];
	double jl[7][MAX_ARRAY_NUM];
}JointTra;

double Comp_Waist_bhr7test[MAX_ARRAY_NUM];;
double Per_Down;
double Per_Mid;
double T_Down;

double dcc_ql_arm;
double dcc_qr_arm;
double dcc_ql_arm_old;
double dcc_qr_arm_old;
JointsAngle PreCon_LegJoint_last;
int Walk_On_old;
int k_start;
double delta_fz;
double micro_delta_fz;
/*============================== Define the External Global Variables ==============================*/
_ext_ int K_Preview_Con;

_ext_ double T_step;
_ext_ double L_step;
_ext_ int    N_step;

_ext_ PreCon_Tra Tra_ZMP, Tra_ZMPCal;
_ext_ PreCon_Tra Tra_RAnkle, Tra_LAnkle;
_ext_ PreCon_Tra Tra_COM, Tra_VCOM, Tra_ACOM;

_ext_ JointsAngle PreCon_LegJoint, Real_LegJoint;
_ext_ ForceSensor F_RFoot, F_LFoot;

_ext_ double Ref_Leg_Joint[3][7];
_ext_ double Ref_Arm_Joint[3][7];

_ext_ double Comp_Waist[MAX_ARRAY_NUM];
_ext_ int    Signal_SupportLeg[MAX_ARRAY_NUM];
_ext_ int    Signal_NowStep[MAX_ARRAY_NUM]; 

_ext_ Position P_ZMPRFoot_RAnkle, P_ZMPLFoot_LAnkle;
_ext_ Position P_ZMPRFoot_B, P_ZMPLFoot_B;
_ext_ Position P_ZMPRel_B;
_ext_ Position P_ZMPRef_B;
_ext_ Position P_DetaCOM, P_DetaCOM_D, P_DetaCOM_DD;

_ext_ double Deta_RAnkle_Z,		Deta_LAnkle_Z;
_ext_ double Rel_RForce_Z,		Rel_LForce_Z;
_ext_ double Rel_RDiffForce_Z,	Rel_LDiffForce_Z;
_ext_ double Ref_RForce_Z,		Ref_LForce_Z;
_ext_ double Ref_RDiffForce_Z,	Ref_LDiffForce_Z;
_ext_ double Last_Ref_RForce_Z, Last_Ref_LForce_Z;
_ext_ double Last_Rel_RForce_Z, Last_Rel_LForce_Z;
_ext_ int	 VMC_Start_K,		VMC_End_K;

/* Other Fils Global Parameters */
_ext_ int Walk_On;
_ext_ double FootFT[5][6];  
_ext_ double FootFT_temp[5][6];  
_ext_ int PreCon_Mode;


/*============================== Declare the Functions ==============================*/
//int round(double r);
void Init_Walk_Parameters();
void Init_Walk_Tra();

void ZMP_Tra();
void Ankle_Tra();
void Ankle_Tra_Z();
void Ankle_Tra_Cycloid();
void Cycloid_Generate(double px1, double py1, double px2, double py2, double td, double dt);
void ZCOM_Tra();

void ZMP_Tra_MoveLeftSide();
void Ankle_Tra_MoveLeftSide();
void ZMP_Tra_MoveRightSide();
void Ankle_Tra_MoveRightSide();

void XYCOM_Tra_PreConCal(double * xzmp, double * yzmp, int kprecon); 
void XYCOM_Tra_SLQR(double * xzmp, double * yzmp, int kprecon);	
		
void LegJoint_Calculate( PreCon_Tra * rankle, PreCon_Tra * lankle, PreCon_Tra * com, int kprecon);
void Inverse_Kinematics(double x_online, double y_online, double z_online, enum Support_Leg leg_No, double q[7]);

void TSpline_S_V_A(double p0, double v0, double a0, double t0, double p1, double t1, double p2, double v2, double a2, double t2, double step);

void ArmJoint_Tra();
void WaistCompensation_Tra();
void StepSignal_Tra();

Position SingleFootZMP_in_Ankle(const ForceSensor * rfs, const ForceSensor * lfs, enum Support_Leg sup_leg);
Position SingleFootZMP_in_Body(const JointsAngle * real_legjoint, const Position * P_ZMPFoot_Ankle, enum Support_Leg sup_leg);
Position T_mul_P(const double t[4][4], const Position * Pos);
Position Calculate_RealZMP_FromForceSensor(const ForceSensor * rfs, const ForceSensor * lfs, const JointsAngle * real_legjoint);
Position T_mul_P2(const double t[4][4], const Position * Pos);
Position RefZMP_in_Body(const Position *p_zmpref_w, const Position *p_comref_w);
void Calculate_DetaCOM_FromTPC(const Position *p_zmprel_b, const Position *p_zmpref_b);

void VMC_Update();
void VMC_Control();

void PreviewControl_Tra_Generate();

void WaistCompensation_Tra_New();

void ZCOM_Tra_UpDown();

void WaistCompensation_Tra_bhr7test(double per_down, double t_down);
void dcc_ArmInit();
void dcc_ArmSwing();
double cal_fz_bias(double delta_fz_zero, int k_pre);
//void Full_DOF_IK(double* ql, double* qr, double xcom, double ycom, double zcom, double xr, double yr, double zr, double xl, double yl, double zl, double thxl, double thxr);
void Full_DOF_IK(double* ql, double* qr, double xcom, double ycom, double zcom, double thx_com, double thy_com, double xr, double yr, double zr, double xl, double yl, double zl, double thxl, double thxr);

void chz_checkIMUbias();
void chz_footdown_go();
void chz_lowlevelfootstep_go();
void chz_landingforce_go();
void chz_swingfoot_go();
void chz_waistcomp_go();
#endif 