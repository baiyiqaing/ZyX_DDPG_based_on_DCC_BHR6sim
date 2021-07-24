/*****************************************************************************************
* FILE NAME : Tra_Generate . c                                                       *
*                                                                                        *
* PURPOSE   : Walking Pattern Generation by Preview Control for BIT Humanoid Robot       * 
*                                                                                        *
* NOTE      : 1. Change the parameters in " void Init_Walking() " to get expected speed. *
*             2. If you don't understand the meaning of the parameter, try to find 
			     the explanation in file "Tra_Generate.h".
*                                                                                        *
* DEVELOPMENT HISTORY:                                                                   *
*   DATE          NAME            DESCIBTION
* ---------    ----------    ---------------------
  2018-11-29     zhouqq      Re-arrangement the whole file. 


******************************************************************************************/
/*--------------------------------------------------------------------------------------
			   IN MEMORY OF PREDECESSOR   
 This project includes two .c documents and one .h document,
 and it is used to generate joints' trajectories on-line(essentially off-line) 
 for the robot.
 The walking parameters can be modified in the function of "void Init_walking()".
 If this functions should be used on the real robot in the future,
 the performed code in every control period is the "for()" in the main
 function.

  LiTongtong    2015.5.20         Beijing Institute of Technology 
---------------------------------------------------------------------------------------*/ 

#include "Tra_Generate.h"
#include "Resistant_Compliance.h"
#include "BHR7_diff_inv.h"
//#include "BHR7_diff_inv.c"
#include "Gp.h"

#include "chzrun_foot.h"
#include "chzrun_com.h"
#include "chzrun_signal.h"
#include "chzrun_MF.h"
#ifdef USE_SDFAST_MF
	#include "chzrun_MFsd.h"
#endif

#include "Uneven_Trailblazer.h"
#include "DCC_motion.h"
#include "CoM_Stabilizer.h"
#include "QP_balance.h"

#include "DCC_RunCon.h"
#include "TPCMPC.h"
#include "Dcc_lib\Base\dcc_con_base.h"
#include "Dcc_lib\Ctlr\DCC_ConFrame.h"
#include "Dcc_lib\PG\DCC_PGFrame.h"

// angle protection
double dJointsPositionCmd[12];
JointsAngle PreCon_LegJoint_old;
JointsAngle PreCon_LegJoint_old_old;
extern strJointPtotection strDCC_Protection;
double dptJointsNow[23] = { 0.0 }, dptJointsOld[23] = { 0.0 }, dptJointsOldOld[23] = { 0.0 };
double dptJointsProtect[23] = { /*ql*/0.0, 0.0, DccD2R(72.5), DccD2R(-80.0), DccD2R(40.0), 0.0, 0.0, /*qr*/0.0, DccD2R(72.5), DccD2R(-80.0), DccD2R(40.0), 0.0, /*waist*/0.0, 0.0, 0.0, /*al*/0.0, 0.0, 0.0, 0.0, /*ar*/0.0, 0.0, 0.0, 0.0 };
double dHopTol = 5.0 / 57.3;
double q2_addi = 0.0;

// TPCMPC
double x_MPCTPC = 0.0;
double dx_MPCTPC = 0.0;
double y_MPCTPC = 0.0;
double dy_MPCTPC = 0.0;
extern double LIPM_ZMP_x;
extern double LIPM_ZMP_y;
// joint protection
int joint_error = 0;
int k_error = 0;
extern Free_Motion Motion_Protection;
// joint protection

extern Run_ConVal DCC_Run;
extern Run_ConVal FootCompliance_ConVal;
extern Run_ConVal FlyRot_ConVal;
extern Run_Horizontal LIPM_ConVal;
extern Run_Horizontal TPC_Run_ConVal;
extern Run_Rotational BodyRot_ConVal;
extern Run_Rotational delta_Rot_old;
extern Run_FS ADD_Trq_Ref;
extern double MECH_PARAS[14];

extern QP_Rotational IMU_bias;
//Chz
double chz_desired_MF[6];
double chz_Bias_com[5] = {0.0};

double chz_XZMP_filted, chz_YZMP_filted;
double chz_Pitch_filted, chz_Roll_filted;
double chz_log[40];
double chz_Pitch_last = 0.0, chz_Roll_last = 0.0;
double chz_Swing_footroll_r = 0.0, chz_Swing_footroll_l = 0.0;
double chz_Swing_joints[12] = {0.0, 0.0, 0.218166, -0.436332, 0.218166, 0.0, 0.0, 0.0, 0.218166, -0.436332, 0.218166, 0.0,};
double chz_comr[3], chz_comr_last[3];
//Chz

// NQP
double body_agl_d[2] = { 0.0, 0.0 }; // desired pitch roll
extern QPbalance_paras_conval QP_test;
extern NQPbalance_paras_conval NQP_test;
extern RE_NQP NQP_re;

extern Free_Motion  Motion_Position_in;
extern Free_Motion  Motion_Position_out;
extern double mic_rot;

extern RC_comp RC_Comp_Cont;
double Asist_x = 0.0;
double chzpitch = 1.0;

double F_Ext_Late_old[6] = { 0.0 }; // check five times, so we need six mark point
int Count_Check_Release_x_old = 0;
//dcc use for compliance
double BHR6_WEIGHT = 40;
double DCC_kx = 500;
double DCC_ky = 500;
com_state DCC_rx, DCC_ry, DCC_lx, DCC_ly, DCC_rpx, DCC_rpy, DCC_rpz, DCC_lpx, DCC_lpy, DCC_lpz;
double dccx, dccy, dccz;
static FootFTDesired FTRd, FTLd;
double addi_r = 0.0;
double addi_l = 0.0;
double P_Toeoff = 0.75;
double T_Toeoff = 0.2;
double P_Heelif = 0.75;
double T_Heelif = 0.2;
double kTH_r = 0;
double kTH_l = 0;
double Tor_Heelif = 0.0;// 500;
double Tor_Toeoff = 0.0;// -50;
double addir[MAX_ARRAY_NUM];
double addil[MAX_ARRAY_NUM];

static DCC_StandingCompliance DCC_SC;
double F_cal_old = 0.0;
double F_v_old = 0.0;
double T_cal_old = 0.0;
double T_v_old = 0.0;
double roll_body = 0.0;
double F_cal_oldsa = 0.0;
double F_v_oldsa = 0.0;
double T_cal_oldsa = 0.0;
double T_v_oldsa = 0.0;
double pitch_body = 0.0;
double F_cal_x_re;
double F_resi_x_re;
double F_v_x_re;
double e_x_re;
double F_cal_y_re;
double F_resi_y_re;
double F_v_y_re;
double e_y_re;
double T_cal_re;
double T_resi_re;
double T_v_re;
double r_re;

double pitch_footr = 0.0;
double pitch_footl = 0.0;

double q5sup;
double q4sup;
double q3sup;
double qasup;
double qaswi;
double q3swi;
double q4swi;
double dq5sup;
double dq4sup;
double dq3sup;
double dqasup;
double dqaswi;
double dq3swi;
double dq4swi;
double ddq5sup;
double ddq4sup;
double ddq3sup;
double ddqasup;
double ddqaswi;
double ddq3swi;
double ddq4swi;
double q6sup;
double q2sup;
double q2swi;
double dq6sup;
double dq2sup;
double dq2swi;
double ddq6sup;
double ddq2sup;
double ddq2swi;
double pcy[9] = { 0.0 };
double acy[9] = { 0.0 };
double pcz[9] = { 0.0 };
double acz[9] = { 0.0 };
double pcxRsup[5] = { 0.0 };
double acxRsup[5] = { 0.0 };
double pczRsup[5] = { 0.0 };
double aczRsup[5] = { 0.0 };
double pcxLsup[5] = { 0.0 };
double acxLsup[5] = { 0.0 };
double pczLsup[5] = { 0.0 };
double aczLsup[5] = { 0.0 };
Joints_sa j_sa;
Joints_la j_la;
Joints_ori qL;
Joints_ori qL_OLD;
Joints_ori qL_OOLD;
Joints_ori qR;
Joints_ori qR_OLD;
Joints_ori qR_OOLD;
Joints_state Joints;
#define L_sup 1
#define R_sup 2
#define D_sup 3
#define S_sup 4
int supleg = D_sup;
int flag = S_sup;
double theta;
double msa[9] = { 0.0 };
double Isa[9] = { 0.0 };
double mla[5] = { 0.0 };
double Ila[5] = { 0.0 };
Desired_FOOTFT FT_desired;
F_struct cal_F_desired;
Tau_struct cal_Tau_desired;
//extern double FootFT[3][6];
double dccFz_L;
double dccTx_L;
double dccTy_L;
double dccFz_R;
double dccTx_R;
double dccTy_R;

FOOTFT_DCC footftL_DCC;
FOOTFT_DCC footftL_DCC_old;
FOOTFT_DCC footftR_DCC;
FOOTFT_DCC footftR_DCC_old;

extern double XS_Pitch, XS_Roll, XS_Yaw;
extern double XS_AccX, XS_AccY, XS_AccZ;
extern double XS_GyrX, XS_GyrY, XS_GyrZ;
//**************************** Declare the Variables ******************************//
double T_step;
double L_step;
int    N_step, N_predict, N_tcom, N_tzmp;
double ZMP_width;
double L_step_sin;
double H_hip;
double H_step;
double H_zc;
double Pct_dou;
double T_dou;
double T_sin;
double T_predict;
double T_ready;
double T_end;
double T_step1;
double T_stepn;
double T_walk;
double T_com;
double T_zmp;

double Pct_dou1, Pct_dou2;
double Pct_sin1, Pct_sin2;
double H_minhi , H_minlow;

FILE *fp;
double S[3][5000] = {0};  //Tspline

int K_Preview_Con;
int K_NUM=0;
//======================================== Variables for ZMP Trajectories Generation 
PreCon_Tra Tra_ZMP;

//======================================== Variables for Ankle Trajectories Generation 
PreCon_Tra Tra_RAnkle, Tra_LAnkle;

//======================================== Variables for CoM Trajectories Generation By Preview Control 
double Gi;
double Gx[3];
double Gp[500]={0};
double A[9];
double B[3];
double C[3];
double X[3] = {0};
double ux[MAX_ARRAY_NUM] = {0};
double ucx, ex, sigma_ex;

double Y[3] = {0};
double uy[MAX_ARRAY_NUM] = {0};
double ucy, ey, sigma_ey;

PreCon_Tra Tra_COM;
PreCon_Tra Tra_VCOM;
PreCon_Tra Tra_ACOM;
PreCon_Tra Tra_ZMPCal;

//======================================== Variables for CoM Trajectories Generation By Singular LQR Method
double Tk_a, Tk_m, Tk_dt;
double Tk_Kd[3] = {0};
double  Tk_A[9] = {0};
double  Tk_B[3] = {0};

double Tk_ux[MAX_ARRAY_NUM] = {0};
double Tk_uy[MAX_ARRAY_NUM] = {0};

double CycTra[3][MAX_ARRAY_NUM]={0};

//======================================== Variables for Joints Angle
enum Support_Leg { DOUBLE_LEG_SP, RLEG_SP, LLEG_SP };
JointsAngle PreCon_LegJoint = { 0.0 };
JointsAngle Fake_LegJoint;
JointsAngle Real_LegJoint;

double Ref_Leg_Joint[3][7] = {0};

//======================================== Variables for Arm Angle
JointTra Tra_ArmJoint; 
double Ref_Arm_Joint[3][7] = {0};

//======================================== Variables for Waist Compensation and Step signal
double Comp_Waist[MAX_ARRAY_NUM] = {0};
int    Signal_SupportLeg[MAX_ARRAY_NUM] = {0};
int    Signal_NowStep[MAX_ARRAY_NUM] = {0}; 
 
//======================================== Variables for Trunk Position Control 
ForceSensor F_RFoot, F_LFoot;
Position P_RFS_RAnkle, P_LFS_LAnkle;
Position P_ZMPRFoot_RAnkle, P_ZMPLFoot_LAnkle;

Position P_ZMPRFoot_B, P_ZMPLFoot_B;
Position P_ZMPRel_B;
Position p_zmprel_b_old;

Position P_ZMPRef_W, P_COMRef_W;
Position P_ZMPRef_B;

// cal ankle B
Position P_RAnkleRef_W, P_LAnkleRef_W;
Position P_RAnkleRef_B, P_LAnkleRef_B;

Position P_DetaCOM, P_DetaCOM_D, P_DetaCOM_DD;

//======================================== Variables for Viscoelastic Model based Compliance Control 
double Deta_RAnkle_Z,		Deta_LAnkle_Z;
double Rel_RForce_Z,		Rel_LForce_Z;
double Rel_RDiffForce_Z,	Rel_LDiffForce_Z;
double Ref_RForce_Z,		Ref_LForce_Z;
double Ref_RDiffForce_Z,	Ref_LDiffForce_Z;
double Last_Ref_RForce_Z, Last_Ref_LForce_Z;
double Last_Rel_RForce_Z, Last_Rel_LForce_Z;
int	 VMC_Start_K,		VMC_End_K;

//======================================== Variables for Move Side
int N_side;
double L_side;
double roll_footr, roll_footl;

/******************************************* Functions For Walking Pattern Generation x ********************************************/
#ifdef USE_VISUAL_STUDIO
int round(double r)
{	
	int a;
	if(r>0)
		a=floor(r+0.5);
	else
		a=ceil(r-0.5);
	return a;
} 
#endif

 float alp_acc[5 + 8 + 5] = { 0,0,0.25,0.5,0.75/*5*/,1,1,1,1,1,1,1,1/*6*/,0.75,0.5,0.25,0,0/*5*/ };
 //float alp_acc[5 + 6 + 5] = { 0,0,0.25,0.5,0.75/*5*/,1,1,1,1,1,1/*6*/,0.75,0.5,0.25,0,0/*5*/ };
//float alp_acc[5 + 12 + 5] = { 0,0,0.25,0.5,0.75/*5*/,1,1,1,1,1,1,1,1,1,1,1,1/*12*/,0.75,0.5,0.25,0,0/*5*/ };
// float alp_acc[5 + 36 + 5] = { 0,0,0.25,0.5,0.75/*5*/,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1/*36*/,0.75,0.5,0.25,0,0/*5*/ };
//float alp_acc[5 + 90 + 5] = { 0,0,0.25,0.5,0.75/*5*/,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1/*100*/,0.75,0.5,0.25,0,0/*5*/ };
//float alp_acc[2 + 2 + 2] = { 0,0/*2*/,1,1/*4*/,0.0,0.0/*2*/ };

#define WALK_PARA_NUM 10

#ifdef USE_CHZ_RUN
PreCon_Walk_Parameters Walk_Para_Config[WALK_PARA_NUM] = {
            // _T_step  _L_step  _N_step  _ZMP_width  _H_hip  _H_step  _L_step_single  _Pct_dou  _H_zc  _H_minhi  _H_minlow//
 /*?chz? km/h*/   3.0,     0.40,     10  ,      0.175,      0.700,   0.04,      0.0,           0.2,     0.8,    0.002,    -0.001, /* Step */
};
#else
PreCon_Walk_Parameters Walk_Para_Config[WALK_PARA_NUM] = {
            // _T_step  _L_step  _N_step  _ZMP_width  _H_hip  _H_step  _L_step_single  _Pct_dou  _H_zc  _H_minhi  _H_minlow//
/*0.0 km/h*/  	0.72,       1.0 * 0.20,     5+8+5  ,      0.18,      0.65,   0.06,      0.0,           0.2,     0.8,    0.002,    -0.001, /* Step */
// /*1.0 km/h*/  	0.72,       0.2,     5+6+5  ,      0.16,      0.65,   0.04,      0.0,           0.2,     0.8,    0.002,    -0.001, /* Step */
///*1.125 km/h*/  	0.8,       0.25,     5+6+5  ,      0.16,      0.65,   0.04,      0.0,           0.2,     0.8,    0.002,    -0.001, /* Step */

 // /*1.8 km/h*/  0.6,		0.30,	 5+6+5,   0.16,		  0.65,	  0.04,	   0.0,			   0.2,		 0.8,   0.002,	  -0.001, /* Step */

// /*1.5 km/h*/  	0.352,       0.0 * 0.30,     5+12+5  ,      0.17,      0.666,   0.04,      0.0,           0.2,     0.8,    0.002,    -0.001, /* Step */
 // /*1.8 km/h*/  	0.6,       1.0 * 0.30,     5+60+5  ,      0.17,      0.700,   0.04,      0.0,           0.2,     0.8,    0.002,    -0.001, /* Step */
 ///*1.35 km/h*/  0.8,    0.30,     16  ,      0.16,      0.700,   0.04,      0.0,           0.2,     0.8,    0.002,    -0.001, /* Step */
 ///*1.80 km/h*/  0.8,    0.40,     16  ,      0.16,      0.700,   0.04,      0.0,           0.2,     0.8,    0.002,    -0.001, /* Step */
 ///*1.80 km/h*/  0.6,    0.30,     16  ,      0.16,      0.700,   0.04,      0.0,           0.2,     0.8,    0.002,    -0.001, /* Step */
 ///*2.10 km/h*/  0.6,    0.35,     16  ,      0.16,      0.700,   0.04,      0.0,           0.2,     0.8,    0.002,    -0.001, /* Step */
 ///*2.40 km/h*/  0.6,    0.40,     16  ,      0.16,      0.700,   0.04,      0.0,           0.2,     0.8,    0.002,    -0.001, /* Step */
 ///*2.88 km/h*/  0.5,     0.40,     16  ,      0.16,      0.700,   0.04,      0.0,           0.2,     0.8,    0.002,    -0.001, /* Step */
///*3.13 km/h*/  0.46,     0.40,     5+12+5  ,      0.16,      0.630,   0.04,      0.0,           0.2,     0.8,    0.002,    -0.001, /* Step */ 
};
#endif

void Init_Walk_Parameters()
{
	//rc
	
	DCC_SC.la.e = 0.0;
	DCC_SC.la.el = 0.0;
	DCC_SC.la.de = 0.0;
	DCC_SC.la.del = 0.0;
	DCC_SC.la.r = 0.0;
	DCC_SC.la.rl = 0.0;
	DCC_SC.la.dr = 0.0;
	DCC_SC.la.drl = 0.0;
	DCC_SC.sa.e = 0.0;
	DCC_SC.sa.el = 0.0;
	DCC_SC.sa.de = 0.0;
	DCC_SC.sa.del = 0.0;
	DCC_SC.sa.r = 0.0;
	DCC_SC.sa.rl = 0.0;
	DCC_SC.sa.dr = 0.0;
	DCC_SC.sa.drl = 0.0;
	
	p_zmprel_b_old.px = 0.0;
	p_zmprel_b_old.py = 0.0;
	
	int i=0;
	GpValue *pGp;

	T_step = Walk_Para_Config[K_NUM]._T_step;
	L_step = Walk_Para_Config[K_NUM]._L_step;
	N_step = Walk_Para_Config[K_NUM]._N_step;
	#ifdef STANDSTILL
	N_step = 110;
	T_step = 1;
	#endif

	ZMP_width  = Walk_Para_Config[K_NUM]._ZMP_width;
	L_step_sin = Walk_Para_Config[K_NUM]._L_step_single;
	
	H_hip  = Walk_Para_Config[K_NUM]._H_hip;
	H_step = Walk_Para_Config[K_NUM]._H_step;
	H_zc   = Walk_Para_Config[K_NUM]._H_zc;

	Pct_dou   = Walk_Para_Config[K_NUM]._Pct_dou;
	T_dou     =  T_step*Pct_dou;
	T_sin     =  T_step*(1-Pct_dou); 
	T_predict = 2.0;
	T_ready   = 1.8;//2.2; //!!! Must Bigger Than 1.2!!!
	T_end     = 2.0;

	T_step1 = T_step*1.0;
	T_stepn = T_step*1.0;
	T_walk  = T_ready+T_step1+(N_step-2)*T_step+T_stepn+T_end;
	T_com   = T_walk + T_predict;
	T_zmp   = T_com + T_predict;

	#ifdef DCC_SPLINE_MOTION
		//printf("%lf\n", T_com);
		Free_Motion_Assig(ANKLE_WIDTH, H_ANKLE, H_zc, CONTROL_T, T_com);
	#endif
	
	N_predict = round(T_predict/CONTROL_T);
	N_tcom    = round(T_com/CONTROL_T);
	N_tzmp    = round(T_zmp/CONTROL_T);

	Pct_dou1 = 0.3;
	Pct_dou2 = 0.5;
	Pct_sin1 = 0.2;
	Pct_sin2 = 0.5;
	H_minhi  = Walk_Para_Config[K_NUM]._H_minhi;
	H_minlow = Walk_Para_Config[K_NUM]._H_minlow;
		
	/* Preview Control Parameters */
	A[0] = 1.0; A[1] = CONTROL_T; A[2] = CONTROL_T*CONTROL_T/2.0;
	A[3] = 0.0; A[4] = 1.0;       A[5] = CONTROL_T;
	A[6] = 0.0; A[7] = 0.0;       A[8] = 1.0;
	B[0] = CONTROL_T*CONTROL_T*CONTROL_T/6.0; 
	B[1] = CONTROL_T*CONTROL_T/2.0; 
	B[2] = CONTROL_T;
	C[0] = 1.0;                               
	C[1] = 0.0;                     
	C[2] = -H_zc/GRAVITY;

	if(H_zc==1.0)
	{
		Gi= 624.0667;

		Gx[0]= 101130.0;
		Gx[1]= 32980.0;
		Gx[2]= 220.0;
	}
	else if(H_zc==0.9)
		{
			Gi=638.9414;

			Gx[0]=98375;
			Gx[1]=30490;
			Gx[2]=209.0; 
		}
		else
		{
			Gi=654.9315; 

			Gx[0]=95250; 
			Gx[1]=27896;
			Gx[2]=198;  
		}

	pGp = GpSelect(H_zc);
	for(i=0; i<N_predict; i++)
	{
		Gp[i] = pGp->gp[i];
	}
	printf("\n GP File OK!, Gp[0]=%f, Gp[1]=%f\n", Gp[0], Gp[1]);
	
	/* TPC Method */
	P_LFS_LAnkle.px = 0.0;
	P_LFS_LAnkle.py = 0.0;
	P_LFS_LAnkle.pz = FS_HIGHT;
	P_RFS_RAnkle.px = 0.0;
	P_RFS_RAnkle.py = 0.0;
	P_RFS_RAnkle.pz = FS_HIGHT;

	P_DetaCOM.px = 0.0;
	P_DetaCOM.py = 0.0;
	P_DetaCOM.pz = 0.0;
	P_DetaCOM_D.px = 0.0;
	P_DetaCOM_D.py = 0.0;
	P_DetaCOM_D.pz = 0.0;
	P_DetaCOM_DD.px = 0.0;
	P_DetaCOM_DD.py = 0.0;
	P_DetaCOM_DD.pz = 0.0;

	/* Singular LQR Method */
    Tk_a  = sqrt(GRAVITY/H_zc);
	Tk_dt = CONTROL_T;
	Tk_m  = 1/(1+Tk_a*Tk_dt);
	Tk_Kd[0] = (1+Tk_a*Tk_dt)/Tk_dt+Tk_a/(1+Tk_a*Tk_dt);
	Tk_Kd[1] = -(2+Tk_a*Tk_dt)/Tk_dt;
	Tk_Kd[2] = -(2+Tk_a*Tk_dt)/(Tk_a*Tk_dt);

	Tk_A[0] = 1;                Tk_A[1] = 0;               Tk_A[2] = 0; 
	Tk_A[3] = 0;                Tk_A[4] = 1;               Tk_A[5] = Tk_dt;
	Tk_A[6] = -Tk_a*Tk_a*Tk_dt; Tk_A[7] = Tk_a*Tk_a*Tk_dt; Tk_A[8] = 1;
	Tk_B[0] = Tk_dt;            Tk_B[1] = 0;               Tk_B[2] = 0;
	
	Per_Down = 0.7;
	Per_Mid = 0.55;
	T_Down = 0.5;
	dcc_ql_arm_old = 0.0;
	dcc_qr_arm_old = 0.0;
	Walk_On_old = 0;
	k_start = 0;
	delta_fz = 0.0;
	
	
	K_Preview_Con = 0;	
	
	#ifdef USE_ZMPMAPPING
		double tempp_R[4][2] = {{-0.020, -0.042}, {0.036, -0.036}, {-0.020, 0.062}, {0.036, 0.055}};
		double tempq_R[4][2] = {{-0.045, -0.090}, {0.085, -0.090}, {-0.045, 0.135}, {0.085, 0.135}};
		chz_ZMPMapping_getpq(tempp_R, tempq_R, 'R');
		double tempp_L[4][2] = {{-0.042, -0.043}, {0.017, -0.039}, {-0.040, 0.067}, {0.017, 0.065}};
		double tempq_L[4][2] = {{-0.085, -0.090}, {0.045, -0.090}, {-0.085, 0.135}, {0.045, 0.135}};
		chz_ZMPMapping_getpq(tempp_L, tempq_L, 'L');
	#endif
}

void Init_Walk_Tra()
{	
	switch (PreCon_Mode)
	{
		case PRECON_MOVE_FORWARD: // Walk Forward
			
			Init_Walk_Parameters();
			fnvDccControlInit();
			
			ZMP_Tra();
			ZCOM_Tra();	
			
			// diff
			cal_zero(THETA_ZERO, MECH_PARAS);
			Phi_reset = cal_reset(AGL_RESET, MECH_PARAS, THETA_ZERO);
			// diff
			
			#ifndef USE_ANKLE_CYCLOID
				Ankle_Tra();
				#ifdef USE_FOOT_DOWN
					Ankle_Tra_Z();
				#endif 		
			#else
				Ankle_Tra_Cycloid();
			#endif
					
			break;
			
		case PRECON_MOVE_BACK:  // WALK	Back
			
			if(Walk_Para_Config[K_NUM]._L_step>0.0) Walk_Para_Config[K_NUM]._L_step = -Walk_Para_Config[K_NUM]._L_step;
			Init_Walk_Parameters();
			
			ZMP_Tra();
			ZCOM_Tra();	
			#ifndef USE_ANKLE_CYCLOID
				Ankle_Tra();
				#ifdef USE_FOOT_DOWN
					Ankle_Tra_Z();
				#endif 		
			#else
				Ankle_Tra_Cycloid();
			#endif
					
			break;

		case PRECON_MOVE_RIGHT:  // Move Right
			
			N_side = 4;
			L_side = 0.20;
			Walk_Para_Config[K_NUM]._N_step = N_side*2+1;
			
			Init_Walk_Parameters();
				
			ZMP_Tra_MoveRightSide();
			Ankle_Tra_MoveRightSide();
			ZCOM_Tra();	
			//ZCOM_Tra_UpDown();		
			break;
			
		case PRECON_MOVE_LEFT:  // Move Left
			
			N_side = 2;
			L_side = 0.20;
			Walk_Para_Config[K_NUM]._N_step = N_side*2;
			
			Init_Walk_Parameters();
			
			ZMP_Tra_MoveLeftSide();
			Ankle_Tra_MoveLeftSide();
			ZCOM_Tra();	
			
			break;
			
			
		default:// Other
			PreCon_Mode = PRECON_MOVE_NONE;
			break;	
	}
	
	ArmJoint_Tra();
	//WaistCompensation_Tra();
	WaistCompensation_Tra_New();
	StepSignal_Tra();
	WaistCompensation_Tra_bhr7test(Per_Down, T_Down);
	dcc_ArmInit();
	
	printf("\n Pre-Con Initialize OK!\n");
}

void ZMP_Tra()//temp_here
{
	int k, j;
	int k_step;
	int nn_step;
	nn_step = N_step - 1;
	double YZMP_temp_temp = 0;
	/* --------------------------------------------- XZMP x----------------------------------------------*/
	/* !!! Tready Use TSpline */
	for (k = 0; k <= round((T_ready - 1.2) / CONTROL_T); k++)
	{
		Tra_ZMP.x[k] = 0;
	}

	j = 0;
	TSpline_S_V_A(0.0, 0, 0, 0, 0.5*ZMP_width*0.5, 1.2 / 2, 0.5*ZMP_width, 0, 0, 1.2, CONTROL_T);
	for (k = round((T_ready - 1.2) / CONTROL_T); k <= round(T_ready / CONTROL_T); k++)
	{
		Tra_ZMP.x[k] = S[0][j]; //0.0;//
		j++;
		#ifdef STANDSTILL
		Tra_ZMP.x[k] = 0.0;
		#endif
	}

	/* First Step */
	for (k = round(T_ready / CONTROL_T); k <= round((T_ready + T_step1*(1 - Pct_dou)) / CONTROL_T); k++)
	{
		Tra_ZMP.x[k] = 0.5*ZMP_width;
		#ifdef STANDSTILL
		Tra_ZMP.x[k] = 0.0;
		#endif
	}

	j = 0;
	TSpline_S_V_A(0.5*ZMP_width, 0, 0, 0, 0, T_step1*Pct_dou*0.5, -0.5*ZMP_width, 0, 0, T_step1*Pct_dou, CONTROL_T);
	for (k = round((T_ready + T_step1*(1 - Pct_dou)) / CONTROL_T); k <= round((T_ready + T_step1) / CONTROL_T); k++)
	{
		Tra_ZMP.x[k] = S[0][j];
		j++;
		#ifdef STANDSTILL
		Tra_ZMP.x[k] = 0.0;
		#endif
	}

	/* Second Step --> (N_step-1) Step */
	for (k_step = 1; k_step<N_step - 1; k_step++)
	{
		if (k_step % 2 != 0)  // right leg swing  1,3,5...
		{
			for (k = round((T_ready + T_step1 + (k_step - 1)*T_step) / CONTROL_T); k <= round((T_ready + T_step1 + (k_step - 1)*T_step + T_sin) / CONTROL_T); k++)
			{
				Tra_ZMP.x[k] = -0.5*ZMP_width;
		#ifdef STANDSTILL
		Tra_ZMP.x[k] = 0.0;
		#endif
			}

			j = 0;
			TSpline_S_V_A(-0.5*ZMP_width, 0, 0, 0, 0, T_dou / 2, 0.5*ZMP_width, 0, 0, T_dou, CONTROL_T);
			for (k = round((T_ready + T_step1 + (k_step - 1)*T_step + T_sin) / CONTROL_T); k <= round((T_ready + T_step1 + k_step*T_step) / CONTROL_T); k++)
			{
				Tra_ZMP.x[k] = S[0][j];
				j++;
		#ifdef STANDSTILL
		Tra_ZMP.x[k] = 0.0;
		#endif
			}

		}

		if (k_step % 2 == 0)  // left leg swing  0,2,4...
		{
			for (k = round((T_ready + T_step1 + (k_step - 1)*T_step) / CONTROL_T); k <= round((T_ready + T_step1 + (k_step - 1)*T_step + T_sin) / CONTROL_T); k++)
			{
				Tra_ZMP.x[k] = 0.5*ZMP_width;
		#ifdef STANDSTILL
		Tra_ZMP.x[k] = 0.0;
		#endif
			}

			j = 0;
			TSpline_S_V_A(0.5*ZMP_width, 0, 0, 0, 0, T_dou / 2, -0.5*ZMP_width, 0, 0, T_dou, CONTROL_T);
			for (k = round((T_ready + T_step1 + (k_step - 1)*T_step + T_sin) / CONTROL_T); k <= round((T_ready + T_step1 + k_step*T_step) / CONTROL_T); k++)
			{
				Tra_ZMP.x[k] = S[0][j];
				j++;
		#ifdef STANDSTILL
		Tra_ZMP.x[k] = 0.0;
		#endif
			}

		}

	}

	/* Last Step */
	if (N_step % 2 == 0)    // right leg swing
	{
		for (k = round((T_ready + T_step1 + (N_step - 2)*T_step) / CONTROL_T); k <= round((T_ready + T_step1 + (N_step - 2)*T_step + T_stepn*(1 - Pct_dou)) / CONTROL_T); k++)
		{
			Tra_ZMP.x[k] = -0.5*ZMP_width;
		#ifdef STANDSTILL
		Tra_ZMP.x[k] = 0.0;
		#endif
		}

		j = 0;
		TSpline_S_V_A(-0.5*ZMP_width, 0, 0, 0, -0.25*ZMP_width, T_stepn*Pct_dou*0.5, 0, 0, 0, T_stepn*Pct_dou, CONTROL_T);
		for (k = round((T_ready + T_step1 + (N_step - 2)*T_step + T_stepn*(1 - Pct_dou)) / CONTROL_T); k <= round((T_ready + T_step1 + (N_step - 2)*T_step + T_stepn) / CONTROL_T); k++)
		{
			Tra_ZMP.x[k] = S[0][j];
			j++;
		#ifdef STANDSTILL
		Tra_ZMP.x[k] = 0.0;
		#endif
		}

	}
	else               // left leg swing 
	{
		for (k = round((T_ready + T_step1 + (N_step - 2)*T_step) / CONTROL_T); k <= round((T_ready + T_step1 + (N_step - 2)*T_step + T_stepn*(1 - Pct_dou)) / CONTROL_T); k++)
		{
			Tra_ZMP.x[k] = 0.5*ZMP_width;
		#ifdef STANDSTILL
		Tra_ZMP.x[k] = 0.0;
		#endif
		}

		j = 0;
		TSpline_S_V_A(0.5*ZMP_width, 0, 0, 0, 0.25*ZMP_width, T_stepn*Pct_dou*0.5, 0, 0, 0, T_stepn*Pct_dou, CONTROL_T);
		for (k = round((T_ready + T_step1 + (N_step - 2)*T_step + T_stepn*(1 - Pct_dou)) / CONTROL_T); k <= round((T_ready + T_step1 + (N_step - 2)*T_step + T_stepn) / CONTROL_T); k++)
		{
			Tra_ZMP.x[k] = S[0][j];
			j++;
		#ifdef STANDSTILL
		Tra_ZMP.x[k] = 0.0;
		#endif
		}


	}

	for (k = round((T_ready + T_step1 + (N_step - 2)*T_step + T_stepn) / CONTROL_T); k <= round(T_zmp / CONTROL_T); k++)
	{
		Tra_ZMP.x[k] = 0.0;
	}

	// if(NULL==(fp=fopen("xzmp.dat","w")))
	// {
	// printf("xzmp File can not open!\n");
	// }
	// for(k=0;k<=round((T_zmp)/CONTROL_T);k++)     // ZMP_t
	// fprintf(fp,"%lf\n",Tra_ZMP.x[k]);						
	// fclose(fp);

	/* --------------------------------------------- XZMP s----------------------------------------------*/

	/* --------------------------------------------- YZMP x----------------------------------------------*/
	for (k = 0; k <= round(T_ready / CONTROL_T); k++)
	{
		Tra_ZMP.y[k] = 0;
		#ifdef STANDSTILL
		Tra_ZMP.y[k] = 0.0;
		#endif
	}

	/* First Step */
	k_step = 0;
	j = 0;
	TSpline_S_V_A(0, 0, 0, 0, L_step_sin*0.5, T_step1*(1 - Pct_dou)*0.5, L_step_sin, 0, 0, T_step1*(1 - Pct_dou), CONTROL_T);
	for (k = round(T_ready / CONTROL_T); k <= round((T_ready + T_step1*(1 - Pct_dou)) / CONTROL_T); k++)   // front 0.5*T_sin
	{
		Tra_ZMP.y[k] = S[0][j];
		j++;
		#ifdef STANDSTILL
		Tra_ZMP.y[k] = 0.0;
		#endif
	}

	j = 0;
	TSpline_S_V_A(L_step_sin, 0, 0, 0, (L_step_sin + alp_acc[0] * L_step)*0.5, T_step1*Pct_dou*0.5, alp_acc[0] * L_step, 0, 0, T_step1*Pct_dou, CONTROL_T);
	for (k = round((T_ready + T_step1*(1 - Pct_dou)) / CONTROL_T); k <= round((T_ready + T_step1) / CONTROL_T); k++)   // T_dou
	{
		Tra_ZMP.y[k] = S[0][j];
		j++;
		#ifdef STANDSTILL
		Tra_ZMP.y[k] = 0.0;
		#endif
	}
	YZMP_temp_temp += alp_acc[k_step] * L_step;
	/* Second Step --> (N_step-1) Step */
	for (k_step = 1; k_step<(N_step); k_step++)
	{
		j = 0;
		TSpline_S_V_A(YZMP_temp_temp, 0, 0, 0, (2 * YZMP_temp_temp + L_step_sin)*0.5, T_sin / 2, (YZMP_temp_temp + L_step_sin), 0, 0, T_sin, CONTROL_T);
		for (k = round((T_ready + T_step1 + (k_step - 1)*T_step) / CONTROL_T); k <= round((T_ready + T_step1 + (k_step - 1)*T_step + T_sin) / CONTROL_T); k++)   // front 0.5*T_sin
		{
			Tra_ZMP.y[k] = S[0][j];
			j++;
		#ifdef STANDSTILL
		Tra_ZMP.y[k] = 0.0;
		#endif
		}

		j = 0;
		TSpline_S_V_A((YZMP_temp_temp + L_step_sin), 0, 0, 0, (2 * YZMP_temp_temp + L_step_sin + alp_acc[k_step] * L_step)*0.5, T_dou / 2, (YZMP_temp_temp + alp_acc[k_step] * L_step), 0, 0, T_dou, CONTROL_T);
		for (k = round((T_ready + T_step1 + (k_step - 1)*T_step + T_sin) / CONTROL_T); k <= round((T_ready + T_step1 + k_step*T_step) / CONTROL_T); k++)   // T_dou
		{
			Tra_ZMP.y[k] = S[0][j];
			j++;
		#ifdef STANDSTILL
		Tra_ZMP.y[k] = 0.0;
		#endif
		}
		YZMP_temp_temp += alp_acc[k_step] * L_step;
	}

	for (k = round((T_ready + T_step1 + (N_step - 2)*T_step + T_stepn) / CONTROL_T); k <= round(T_zmp / CONTROL_T); k++)   // T_dou
	{
		Tra_ZMP.y[k] = YZMP_temp_temp;
		#ifdef STANDSTILL
		Tra_ZMP.y[k] = 0.0;
		#endif
	}

	// if(NULL==(fp=fopen("yzmp.dat","w")))
	// {
	// printf("yzmp File can not open!\n");
	// }
	// for(k=0;k<=round((T_zmp)/CONTROL_T);k++)     // ZMP_t
	// fprintf(fp,"%lf\n",Tra_ZMP.y[k]);						
	// fclose(fp);

	/* --------------------------------------------- YZMP s----------------------------------------------*/
}

void ZCOM_Tra()
{
	int k, j;
	int k_step;
	
	if(K_NUM == 0)
	{
		#ifndef USE_ZC_CHANGE
		 TSpline_S_V_A(H_RESET,0,0,0,(H_RESET+H_hip)/2,T_ready/2,H_hip,0,0,T_ready,CONTROL_T);
		 j=0;
		 for(k=0;k<=round(T_ready/CONTROL_T);k++)
		 {
			 Tra_COM.z[k]=S[0][j++];
		 }

		 for(k=round(T_ready/CONTROL_T);k<=round((T_ready+T_step1+(N_step-2)*T_step+T_stepn)/CONTROL_T);k++)
		 {
			 Tra_COM.z[k]=H_hip;
		 }

		 for(k=round((T_ready+T_step1+(N_step-2)*T_step+T_stepn)/CONTROL_T);k<=round((T_ready+T_step1+(N_step-2)*T_step+T_stepn+T_end)/CONTROL_T);k++)
		 {
			 Tra_COM.z[k]=H_hip;
		 }
		 for(k=round((T_ready+T_step1+(N_step-2)*T_step+T_stepn+T_end)/CONTROL_T);k<=round(T_com/CONTROL_T);k++)
		 {
			 Tra_COM.z[k]=H_hip;
		 }
		 #else
	/*	
		// TSpline_S_V_A(H_hip,0,0,0,(H_RESET+H_hip)/2,T_end/2,H_RESET,0,0,T_end,CONTROL_T);
		// j=0;
		// for(k=round((T_ready+T_step1+(N_step-2)*T_step+T_stepn)/CONTROL_T);k<=round((T_ready+T_step1+(N_step-2)*T_step+T_stepn+T_end)/CONTROL_T);k++)
		// {
			// Tra_COM.z[k]=S[0][j++];
		// }
		// for(k=round((T_ready+T_step1+(N_step-2)*T_step+T_stepn+T_end)/CONTROL_T);k<=round(T_com/CONTROL_T);k++)
		// {
			// Tra_COM.z[k]=H_RESET;
		// }
	*/	
		
			TSpline_S_V_A(H_RESET,0,0,0,(H_RESET+H_hip)/2,T_ready/2,H_hip,0,0,T_ready,CONTROL_T);
			j=0;
			for(k=0;k<=round(T_ready/CONTROL_T);k++)
			{
				Tra_COM.z[k]=S[0][j++];
			}
			/* First Step */
			j = 0;
			TSpline_S_V_A(H_hip, 0, 0, 0, H_hip_H, T_step1*(1 - Pct_dou)*0.5, H_hip, 0, 0, T_step1*(1 - Pct_dou), CONTROL_T);
			for (k = round(T_ready / CONTROL_T); k <= round((T_ready + T_step1*(1 - Pct_dou)) / CONTROL_T); k++)
			{
				Tra_COM.z[k] = S[0][j++];//H_hip;//
			}
			for (k = round((T_ready + T_step1*(1 - Pct_dou)) / CONTROL_T); k <= round((T_ready + T_step1) / CONTROL_T); k++)
			{
				Tra_COM.z[k] = H_hip;

			}

			/* Second Step --> (N_step-1) Step */
			for (k_step = 1; k_step<N_step - 1; k_step++)
			{
				if (k_step % 2 != 0) // right leg swing  1,3,5...
				{
					j = 0;
					TSpline_S_V_A(H_hip, 0, 0, 0, H_hip_H, T_sin / 2, H_hip, 0, 0, T_sin, CONTROL_T);
					for (k = round((T_ready + T_step1 + (k_step - 1)*T_step) / CONTROL_T); k <= round((T_ready + T_step1 + (k_step - 1)*T_step + T_sin) / CONTROL_T); k++)
					{
						Tra_COM.z[k] = S[0][j++];
					}
				
					for (k = round((T_ready + T_step1 + (k_step - 1)*T_step + T_sin) / CONTROL_T); k <= round((T_ready + T_step1 + (k_step - 1)*T_step + T_step) / CONTROL_T); k++)
					{
						Tra_COM.z[k] = H_hip;
					}
				}

				if (k_step % 2 == 0) // left leg swing   0,2,4...
				{
					
					j = 0;
					TSpline_S_V_A(H_hip, 0, 0, 0, H_hip_H, T_sin / 2, H_hip, 0, 0, T_sin, CONTROL_T);
					for (k = round((T_ready + T_step1 + (k_step - 1)*T_step) / CONTROL_T); k <= round((T_ready + T_step1 + (k_step - 1)*T_step + T_sin) / CONTROL_T); k++)
					{
						Tra_COM.z[k] = S[0][j++];
				
					}
					
					for (k = round((T_ready + T_step1 + (k_step - 1)*T_step + T_sin) / CONTROL_T); k <= round((T_ready + T_step1 + (k_step - 1)*T_step + T_step) / CONTROL_T); k++)
					{
						Tra_COM.z[k] = H_hip;
					}
				}
			}

			/* Last Step */
			if (N_step % 2 == 0)  // right leg swing
			{
				//single phase
				j = 0;
				TSpline_S_V_A(H_hip, 0, 0, 0, H_hip_H, T_stepn*(1 - Pct_dou)*0.5, H_hip, 0, 0, T_stepn*(1 - Pct_dou), CONTROL_T);
				for (k = round((T_ready + T_step1 + (N_step - 2)*T_step) / CONTROL_T); k <= round((T_ready + T_step1 + (N_step - 2)*T_step + T_stepn*(1 - Pct_dou)) / CONTROL_T); k++)
				{
					Tra_COM.z[k] = S[0][j++];
				}
				//double phase
				for (k = round((T_ready + T_step1 + (N_step - 2)*T_step + T_stepn*(1 - Pct_dou)) / CONTROL_T); k <= round((T_ready + T_step1 + (N_step - 2)*T_step + T_stepn) / CONTROL_T); k++)
				{
					Tra_COM.z[k] = H_hip;
				}
			}
			else  // left leg swing
			{
				
				j = 0;
				TSpline_S_V_A(H_hip, 0, 0, 0, H_hip_H, T_stepn*(1 - Pct_dou)*0.5, H_hip, 0, 0, T_stepn*(1 - Pct_dou), CONTROL_T);
				for (k = round((T_ready + T_step1 + (N_step - 2)*T_step) / CONTROL_T); k <= round((T_ready + T_step1 + (N_step - 2)*T_step + T_stepn*(1 - Pct_dou)) / CONTROL_T); k++)
				{
					Tra_COM.z[k] = S[0][j++];
				
				}
				
				for (k = round((T_ready + T_step1 + (N_step - 2)*T_step + T_stepn*(1 - Pct_dou)) / CONTROL_T); k <= round((T_ready + T_step1 + (N_step - 2)*T_step + T_stepn) / CONTROL_T); k++)
				{
					Tra_COM.z[k] = H_hip;
				}
			}
			
			for (k = round((T_ready + T_step1 + (N_step - 2)*T_step + T_stepn) / CONTROL_T); k <= round((T_ready + T_step1 + (N_step - 2)*T_step + T_stepn + T_end) / CONTROL_T); k++)
			{
				Tra_COM.z[k] = H_hip;
			}


			for (k = round((T_ready + T_step1 + (N_step - 2)*T_step + T_stepn + T_end) / CONTROL_T); k <= round(T_com / CONTROL_T); k++)
			{
				Tra_COM.z[k] = H_hip;
			}
			#endif
	}
	else
	{
		if(PreCon_Mode==PRECON_MOVE_RIGHT) // Only for 'Move Right', change ZC in single support period. COMMET: HZZ
		{			
			for (k = 0; k <= round(T_ready / CONTROL_T); k++)
			{
				Tra_COM.z[k] = H_hip;
			}
			/* First Step */
			// single phase 
			j = 0;
			TSpline_S_V_A(H_hip, 0, 0, 0, H_hip_H, T_step1*(1 - Pct_dou)*0.5, H_hip, 0, 0, T_step1*(1 - Pct_dou), CONTROL_T);
			for (k = round(T_ready / CONTROL_T); k <= round((T_ready + T_step1*(1 - Pct_dou)) / CONTROL_T); k++)
			{
				Tra_COM.z[k] = H_hip;//S[0][j++];
				//P_Sig[k] = 1;
			}
			// double phase
			for (k = round((T_ready + T_step1*(1 - Pct_dou)) / CONTROL_T); k <= round((T_ready + T_step1) / CONTROL_T); k++)
			{
				Tra_COM.z[k] = H_hip;

			}

			/* Second Step --> (N_step-1) Step */
			for (k_step = 1; k_step<N_step - 1; k_step++)
			{
				if (k_step % 2 != 0) // right leg swing  1,3,5...
				{
					// single phase
					j = 0;
					TSpline_S_V_A(H_hip, 0, 0, 0, H_hip_H, T_sin / 2, H_hip, 0, 0, T_sin, CONTROL_T);
					for (k = round((T_ready + T_step1 + (k_step - 1)*T_step) / CONTROL_T); k <= round((T_ready + T_step1 + (k_step - 1)*T_step + T_sin) / CONTROL_T); k++)
					{
						Tra_COM.z[k] = S[0][j++];
						//P_Sig[k] = 1;
					}
					// double phase
					for (k = round((T_ready + T_step1 + (k_step - 1)*T_step + T_sin) / CONTROL_T); k <= round((T_ready + T_step1 + (k_step - 1)*T_step + T_step) / CONTROL_T); k++)
					{
						Tra_COM.z[k] = H_hip;
					}
				}

				if (k_step % 2 == 0) // left leg swing   0,2,4...
				{
					// single phase
					j = 0;
					TSpline_S_V_A(H_hip, 0, 0, 0, H_hip_H, T_sin / 2, H_hip, 0, 0, T_sin, CONTROL_T);
					for (k = round((T_ready + T_step1 + (k_step - 1)*T_step) / CONTROL_T); k <= round((T_ready + T_step1 + (k_step - 1)*T_step + T_sin) / CONTROL_T); k++)
					{
						Tra_COM.z[k] = S[0][j++];
						//P_Sig[k] = 1;
					}
					// double phase
					for (k = round((T_ready + T_step1 + (k_step - 1)*T_step + T_sin) / CONTROL_T); k <= round((T_ready + T_step1 + (k_step - 1)*T_step + T_step) / CONTROL_T); k++)
					{
						Tra_COM.z[k] = H_hip;
					}
				}
			}

			/* Last Step */
			if (N_step % 2 == 0)  // right leg swing
			{
				// single phase
				j = 0;
				TSpline_S_V_A(H_hip, 0, 0, 0, H_hip_H, T_stepn*(1 - Pct_dou)*0.5, H_hip, 0, 0, T_stepn*(1 - Pct_dou), CONTROL_T);
				for (k = round((T_ready + T_step1 + (N_step - 2)*T_step) / CONTROL_T); k <= round((T_ready + T_step1 + (N_step - 2)*T_step + T_stepn*(1 - Pct_dou)) / CONTROL_T); k++)
				{
					Tra_COM.z[k] = S[0][j++];
					//P_Sig[k] = 1;
				}
				// double phase
				for (k = round((T_ready + T_step1 + (N_step - 2)*T_step + T_stepn*(1 - Pct_dou)) / CONTROL_T); k <= round((T_ready + T_step1 + (N_step - 2)*T_step + T_stepn) / CONTROL_T); k++)
				{
					Tra_COM.z[k] = H_hip;
				}
			}
			else  // left leg swing
			{
				// single phase
				j = 0;
				TSpline_S_V_A(H_hip, 0, 0, 0, H_hip_H, T_stepn*(1 - Pct_dou)*0.5, H_hip, 0, 0, T_stepn*(1 - Pct_dou), CONTROL_T);
				for (k = round((T_ready + T_step1 + (N_step - 2)*T_step) / CONTROL_T); k <= round((T_ready + T_step1 + (N_step - 2)*T_step + T_stepn*(1 - Pct_dou)) / CONTROL_T); k++)
				{
					Tra_COM.z[k] = S[0][j++];
					//P_Sig[k] = 1;
				}
				// double phase
				for (k = round((T_ready + T_step1 + (N_step - 2)*T_step + T_stepn*(1 - Pct_dou)) / CONTROL_T); k <= round((T_ready + T_step1 + (N_step - 2)*T_step + T_stepn) / CONTROL_T); k++)
				{
					Tra_COM.z[k] = H_hip;
				}
			}
			
			for (k = round((T_ready + T_step1 + (N_step - 2)*T_step + T_stepn) / CONTROL_T); k <= round((T_ready + T_step1 + (N_step - 2)*T_step + T_stepn + T_end) / CONTROL_T); k++)
			{
				Tra_COM.z[k] = H_hip;
			}


			for (k = round((T_ready + T_step1 + (N_step - 2)*T_step + T_stepn + T_end) / CONTROL_T); k <= round(T_com / CONTROL_T); k++)
			{
				Tra_COM.z[k] = H_hip;
			}
		}
		else if(PreCon_Mode==PRECON_MOVE_LEFT) // Only for 'Move Right', change ZC in single support period. COMMET: HZZ
		{			
			for (k = 0; k <= round(T_ready / CONTROL_T); k++)
			{
				Tra_COM.z[k] = H_hip;
			}
			/* First Step */
			// single phase 
			j = 0;
			TSpline_S_V_A(H_hip, 0, 0, 0, H_hip_H, T_step1*(1 - Pct_dou)*0.5, H_hip, 0, 0, T_step1*(1 - Pct_dou), CONTROL_T);
			for (k = round(T_ready / CONTROL_T); k <= round((T_ready + T_step1*(1 - Pct_dou)) / CONTROL_T); k++)
			{
				Tra_COM.z[k] = S[0][j++];//H_hip;//
				//P_Sig[k] = 1;
			}
			// double phase
			for (k = round((T_ready + T_step1*(1 - Pct_dou)) / CONTROL_T); k <= round((T_ready + T_step1) / CONTROL_T); k++)
			{
				Tra_COM.z[k] = H_hip;

			}

			/* Second Step --> (N_step-1) Step */
			for (k_step = 1; k_step<N_step - 1; k_step++)
			{
				if (k_step % 2 != 0) // right leg swing  1,3,5...
				{
					// single phase
					j = 0;
					TSpline_S_V_A(H_hip, 0, 0, 0, H_hip_H, T_sin / 2, H_hip, 0, 0, T_sin, CONTROL_T);
					for (k = round((T_ready + T_step1 + (k_step - 1)*T_step) / CONTROL_T); k <= round((T_ready + T_step1 + (k_step - 1)*T_step + T_sin) / CONTROL_T); k++)
					{
						Tra_COM.z[k] = S[0][j++];
						//P_Sig[k] = 1;
					}
					// double phase
					for (k = round((T_ready + T_step1 + (k_step - 1)*T_step + T_sin) / CONTROL_T); k <= round((T_ready + T_step1 + (k_step - 1)*T_step + T_step) / CONTROL_T); k++)
					{
						Tra_COM.z[k] = H_hip;
					}
				}

				if (k_step % 2 == 0) // left leg swing   0,2,4...
				{
					// single phase
					j = 0;
					TSpline_S_V_A(H_hip, 0, 0, 0, H_hip_H, T_sin / 2, H_hip, 0, 0, T_sin, CONTROL_T);
					for (k = round((T_ready + T_step1 + (k_step - 1)*T_step) / CONTROL_T); k <= round((T_ready + T_step1 + (k_step - 1)*T_step + T_sin) / CONTROL_T); k++)
					{
						Tra_COM.z[k] = S[0][j++];
						//P_Sig[k] = 1;
					}
					// double phase
					for (k = round((T_ready + T_step1 + (k_step - 1)*T_step + T_sin) / CONTROL_T); k <= round((T_ready + T_step1 + (k_step - 1)*T_step + T_step) / CONTROL_T); k++)
					{
						Tra_COM.z[k] = H_hip;
					}
				}
			}

			/* Last Step */
			if (N_step % 2 == 0)  // right leg swing
			{
				// single phase
				j = 0;
				TSpline_S_V_A(H_hip, 0, 0, 0, H_hip_H, T_stepn*(1 - Pct_dou)*0.5, H_hip, 0, 0, T_stepn*(1 - Pct_dou), CONTROL_T);
				for (k = round((T_ready + T_step1 + (N_step - 2)*T_step) / CONTROL_T); k <= round((T_ready + T_step1 + (N_step - 2)*T_step + T_stepn*(1 - Pct_dou)) / CONTROL_T); k++)
				{
					Tra_COM.z[k] = S[0][j++];
					//P_Sig[k] = 1;
				}
				// double phase
				for (k = round((T_ready + T_step1 + (N_step - 2)*T_step + T_stepn*(1 - Pct_dou)) / CONTROL_T); k <= round((T_ready + T_step1 + (N_step - 2)*T_step + T_stepn) / CONTROL_T); k++)
				{
					Tra_COM.z[k] = H_hip;
				}
			}
			else  // left leg swing
			{
				// single phase
				j = 0;
				TSpline_S_V_A(H_hip, 0, 0, 0, H_hip_H, T_stepn*(1 - Pct_dou)*0.5, H_hip, 0, 0, T_stepn*(1 - Pct_dou), CONTROL_T);
				for (k = round((T_ready + T_step1 + (N_step - 2)*T_step) / CONTROL_T); k <= round((T_ready + T_step1 + (N_step - 2)*T_step + T_stepn*(1 - Pct_dou)) / CONTROL_T); k++)
				{
					Tra_COM.z[k] = S[0][j++];
					//P_Sig[k] = 1;
				}
				// double phase
				for (k = round((T_ready + T_step1 + (N_step - 2)*T_step + T_stepn*(1 - Pct_dou)) / CONTROL_T); k <= round((T_ready + T_step1 + (N_step - 2)*T_step + T_stepn) / CONTROL_T); k++)
				{
					Tra_COM.z[k] = H_hip;
				}
			}
			
			for (k = round((T_ready + T_step1 + (N_step - 2)*T_step + T_stepn) / CONTROL_T); k <= round((T_ready + T_step1 + (N_step - 2)*T_step + T_stepn + T_end) / CONTROL_T); k++)
			{
				Tra_COM.z[k] = H_hip;
			}


			for (k = round((T_ready + T_step1 + (N_step - 2)*T_step + T_stepn + T_end) / CONTROL_T); k <= round(T_com / CONTROL_T); k++)
			{
				Tra_COM.z[k] = H_hip;
			}
		}
		else
		{
			for(k=0;k<=round(T_ready/CONTROL_T);k++)
			{
				Tra_COM.z[k]=H_hip;
			}

			for(k=round(T_ready/CONTROL_T);k<=round((T_ready+T_step1+(N_step-2)*T_step+T_stepn)/CONTROL_T);k++)
			{
				Tra_COM.z[k]=H_hip;
			}

			for(k=round((T_ready+T_step1+(N_step-2)*T_step+T_stepn)/CONTROL_T);k<=round((T_ready+T_step1+(N_step-2)*T_step+T_stepn+T_end)/CONTROL_T);k++)
			{
				Tra_COM.z[k]=H_hip;
			}

			for(k=round((T_ready+T_step1+(N_step-2)*T_step+T_stepn+T_end)/CONTROL_T);k<=round(T_com/CONTROL_T);k++)
			{
				Tra_COM.z[k]=H_hip;
			}
		}

	}
	// if(NULL==(fp=fopen("zcom.dat","w")))
	// {
		// printf("zcom File can not open!\n");
	// }
	// for(k=0;k<=round(T_com/CONTROL_T);k++)     // COM_t
		// fprintf(fp,"%lf\n",Tra_COM.z[k]);						
	// fclose(fp);
}


//dcc_rec
void Ankle_Tra()
{
	int k, j;
	int k_step;
	double L_temp_temp;
	double R_temp_temp;
	L_temp_temp = 0;
	R_temp_temp = 0;

	for (k = 0; k <= round(T_ready / CONTROL_T); k++)
	{
		Tra_RAnkle.x[k] = ANKLE_WIDTH / 2;
		Tra_RAnkle.y[k] = 0;

		Tra_LAnkle.x[k] = -ANKLE_WIDTH / 2;
		Tra_LAnkle.y[k] = 0;

		Tra_RAnkle.z[k] = H_ANKLE;
		Tra_LAnkle.z[k] = H_ANKLE;

#ifdef STANDSTILL
		Tra_RAnkle.x[k] = ANKLE_WIDTH / 2;
		Tra_RAnkle.y[k] = 0;

		Tra_LAnkle.x[k] = -ANKLE_WIDTH / 2;
		Tra_LAnkle.y[k] = 0;

		Tra_RAnkle.z[k] = H_ANKLE;
		Tra_LAnkle.z[k] = H_ANKLE;
#endif
	}

	/* First Step */
	// single phase 
	j = 0;
	TSpline_S_V_A(0, 0, 0, 0, alp_acc[0] * L_step / 2, T_step1*(1 - Pct_dou)*0.5, alp_acc[0] * L_step, 0, 0, T_step1*(1 - Pct_dou), CONTROL_T);
	for (k = round(T_ready / CONTROL_T); k <= round((T_ready + T_step1*(1 - Pct_dou)) / CONTROL_T); k++)
	{
		Tra_RAnkle.x[k] = ANKLE_WIDTH / 2;
		Tra_RAnkle.y[k] = 0;

		Tra_LAnkle.x[k] = -ANKLE_WIDTH / 2;
		Tra_LAnkle.y[k] = S[0][j++];
#ifdef STANDSTILL
		Tra_RAnkle.x[k] = ANKLE_WIDTH / 2;
		Tra_RAnkle.y[k] = 0;

		Tra_LAnkle.x[k] = -ANKLE_WIDTH / 2;
		Tra_LAnkle.y[k] = 0;
#endif
	}
	j = 0;
	TSpline_S_V_A(H_ANKLE, 0, 0, 0, H_ANKLE + H_step, T_step1*(1 - Pct_dou)*0.5, H_ANKLE, 0, 0, T_step1*(1 - Pct_dou), CONTROL_T);
	for (k = round(T_ready / CONTROL_T); k <= round((T_ready + T_step1*(1 - Pct_dou)) / CONTROL_T); k++)
	{
		Tra_RAnkle.z[k] = H_ANKLE;
		Tra_LAnkle.z[k] = S[0][j++];
#ifdef STANDSTILL
		Tra_RAnkle.z[k] = H_ANKLE;
		Tra_LAnkle.z[k] = H_ANKLE;
#endif
	}
	// double phase
	for (k = round((T_ready + T_step1*(1 - Pct_dou)) / CONTROL_T); k <= round((T_ready + T_step1) / CONTROL_T); k++)
	{
		Tra_RAnkle.x[k] = ANKLE_WIDTH / 2;
		Tra_RAnkle.y[k] = 0;

		Tra_LAnkle.x[k] = -ANKLE_WIDTH / 2;
		Tra_LAnkle.y[k] = alp_acc[0] * L_step;

		Tra_RAnkle.z[k] = H_ANKLE;
		Tra_LAnkle.z[k] = H_ANKLE;
#ifdef STANDSTILL
		Tra_RAnkle.x[k] = ANKLE_WIDTH / 2;
		Tra_RAnkle.y[k] = 0;

		Tra_LAnkle.x[k] = -ANKLE_WIDTH / 2;
		Tra_LAnkle.y[k] = 0;

		Tra_RAnkle.z[k] = H_ANKLE;
		Tra_LAnkle.z[k] = H_ANKLE;
#endif

	}
	//接下来
	L_temp_temp = alp_acc[0] * L_step;
	R_temp_temp = 0;
	/* Second Step --> (N_step-1) Step */
	for (k_step = 1; k_step<N_step; k_step++)
	{
		if (k_step % 2 != 0) // right leg swing  1,3,5...
		{
			// single phase
			j = 0;
			TSpline_S_V_A(R_temp_temp, 0, 0, 0, L_temp_temp, T_sin / 2, L_temp_temp + alp_acc[k_step] * L_step, 0, 0, T_sin, CONTROL_T);//
			for (k = round((T_ready + T_step1 + (k_step - 1)*T_step) / CONTROL_T); k <= round((T_ready + T_step1 + (k_step - 1)*T_step + T_sin) / CONTROL_T); k++)
			{
				Tra_RAnkle.x[k] = ANKLE_WIDTH / 2;
				Tra_RAnkle.y[k] = S[0][j++];

				Tra_LAnkle.x[k] = -ANKLE_WIDTH / 2;
				Tra_LAnkle.y[k] = L_temp_temp;
#ifdef STANDSTILL
				Tra_RAnkle.x[k] = ANKLE_WIDTH / 2;
				Tra_RAnkle.y[k] = 0;

				Tra_LAnkle.x[k] = -ANKLE_WIDTH / 2;
				Tra_LAnkle.y[k] = 0;
#endif
			}
			R_temp_temp = L_temp_temp + alp_acc[k_step] * L_step;

			j = 0;
			TSpline_S_V_A(H_ANKLE, 0, 0, 0, H_ANKLE + H_step, T_sin / 2, H_ANKLE, 0, 0, T_sin, CONTROL_T);
			for (k = round((T_ready + T_step1 + (k_step - 1)*T_step) / CONTROL_T); k <= round((T_ready + T_step1 + (k_step - 1)*T_step + T_sin) / CONTROL_T); k++)
			{
				Tra_RAnkle.z[k] = S[0][j++];
				Tra_LAnkle.z[k] = H_ANKLE;
#ifdef STANDSTILL
				Tra_RAnkle.z[k] = H_ANKLE;
				Tra_LAnkle.z[k] = H_ANKLE;
#endif
			}
			// double phase
			for (k = round((T_ready + T_step1 + (k_step - 1)*T_step + T_sin) / CONTROL_T); k <= round((T_ready + T_step1 + (k_step - 1)*T_step + T_step) / CONTROL_T); k++)
			{
				Tra_RAnkle.x[k] = ANKLE_WIDTH / 2;
				Tra_RAnkle.y[k] = R_temp_temp;

				Tra_LAnkle.x[k] = -ANKLE_WIDTH / 2;
				Tra_LAnkle.y[k] = L_temp_temp;

				Tra_RAnkle.z[k] = H_ANKLE;
				Tra_LAnkle.z[k] = H_ANKLE;
#ifdef STANDSTILL
				Tra_RAnkle.x[k] = ANKLE_WIDTH / 2;
				Tra_RAnkle.y[k] = 0;

				Tra_LAnkle.x[k] = -ANKLE_WIDTH / 2;
				Tra_LAnkle.y[k] = 0;

				Tra_RAnkle.z[k] = H_ANKLE;
				Tra_LAnkle.z[k] = H_ANKLE;
#endif
			}
		}

		if (k_step % 2 == 0) // left leg swing   0,2,4...
		{
			// single phase
			j = 0;
			TSpline_S_V_A(L_temp_temp, 0, 0, 0, R_temp_temp, T_sin / 2, R_temp_temp + alp_acc[k_step] * L_step, 0, 0, T_sin, CONTROL_T);
			for (k = round((T_ready + T_step1 + (k_step - 1)*T_step) / CONTROL_T); k <= round((T_ready + T_step1 + (k_step - 1)*T_step + T_sin) / CONTROL_T); k++)
			{
				Tra_RAnkle.x[k] = ANKLE_WIDTH / 2;
				Tra_RAnkle.y[k] = R_temp_temp;

				Tra_LAnkle.x[k] = -ANKLE_WIDTH / 2;
				Tra_LAnkle.y[k] = S[0][j++];
#ifdef STANDSTILL
				Tra_RAnkle.x[k] = ANKLE_WIDTH / 2;
				Tra_RAnkle.y[k] = 0;

				Tra_LAnkle.x[k] = -ANKLE_WIDTH / 2;
				Tra_LAnkle.y[k] = 0;
#endif
			}
			L_temp_temp = R_temp_temp + alp_acc[k_step] * L_step;

			j = 0;
			TSpline_S_V_A(H_ANKLE, 0, 0, 0, H_ANKLE + H_step, T_sin / 2, H_ANKLE, 0, 0, T_sin, CONTROL_T);
			for (k = round((T_ready + T_step1 + (k_step - 1)*T_step) / CONTROL_T); k <= round((T_ready + T_step1 + (k_step - 1)*T_step + T_sin) / CONTROL_T); k++)
			{
				Tra_RAnkle.z[k] = H_ANKLE;
				Tra_LAnkle.z[k] = S[0][j++];
#ifdef STANDSTILL
				Tra_RAnkle.z[k] = H_ANKLE;
				Tra_LAnkle.z[k] = H_ANKLE;
#endif
			}
			// double phase
			for (k = round((T_ready + T_step1 + (k_step - 1)*T_step + T_sin) / CONTROL_T); k <= round((T_ready + T_step1 + (k_step - 1)*T_step + T_step) / CONTROL_T); k++)
			{
				Tra_RAnkle.x[k] = ANKLE_WIDTH / 2;
				Tra_RAnkle.y[k] = R_temp_temp;

				Tra_LAnkle.x[k] = -ANKLE_WIDTH / 2;
				Tra_LAnkle.y[k] = L_temp_temp;

				Tra_RAnkle.z[k] = H_ANKLE;
				Tra_LAnkle.z[k] = H_ANKLE;
#ifdef STANDSTILL
				Tra_RAnkle.x[k] = ANKLE_WIDTH / 2;
				Tra_RAnkle.y[k] = 0;

				Tra_LAnkle.x[k] = -ANKLE_WIDTH / 2;
				Tra_LAnkle.y[k] = 0;

				Tra_RAnkle.z[k] = H_ANKLE;
				Tra_LAnkle.z[k] = H_ANKLE;
#endif
			}
		}
	}

	for (k = round((T_ready + T_step1 + (N_step - 2)*T_step + T_stepn) / CONTROL_T); k <= round(T_com / CONTROL_T); k++)
	{
		Tra_RAnkle.x[k] = ANKLE_WIDTH / 2;
		Tra_RAnkle.y[k] = R_temp_temp;

		Tra_LAnkle.x[k] = -ANKLE_WIDTH / 2;
		Tra_LAnkle.y[k] = L_temp_temp;

		Tra_RAnkle.z[k] = H_ANKLE;
		Tra_LAnkle.z[k] = H_ANKLE;
#ifdef STANDSTILL
		Tra_RAnkle.x[k] = ANKLE_WIDTH / 2;
		Tra_RAnkle.y[k] = 0;

		Tra_LAnkle.x[k] = -ANKLE_WIDTH / 2;
		Tra_LAnkle.y[k] = 0;

		Tra_RAnkle.z[k] = H_ANKLE;
		Tra_LAnkle.z[k] = H_ANKLE;
#endif
	}

	// if(NULL==(fp=fopen("ankle.dat","w")))
	// {
	// printf("ankle File can not open!\n");
	// }
	// for(k=0;k<=round(T_com/CONTROL_T);k++)     // COM_t
	// fprintf(fp,"%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n",Tra_RAnkle.x[k],Tra_RAnkle.y[k],Tra_RAnkle.z[k],Tra_LAnkle.x[k],Tra_LAnkle.y[k],Tra_LAnkle.z[k]);						
	// fclose(fp);


}
//dcc_rec
void Ankle_Tra_Z()
{
	int k;
	int k_step;
	int j;
	double mm = 1 / 4.0;

	for (k = 0; k <= round(T_ready / CONTROL_T); k++)
	{
		Tra_LAnkle.z[k] = H_ANKLE;
	}

	for (k = 0; k <= round((T_ready - (1 - Pct_dou2)*T_dou) / CONTROL_T); k++)
	{
		Tra_RAnkle.z[k] = H_ANKLE;
	}
	j = 0;
	TSpline_S_V_A(H_ANKLE, 0, 0, 0, H_ANKLE + H_minlow*0.5, (1 - Pct_dou2)*T_dou*0.5, H_ANKLE + H_minlow, 0, 0, (1 - Pct_dou2)*T_dou, CONTROL_T);
	for (k = round((T_ready - (1 - Pct_dou2)*T_dou) / CONTROL_T); k <= round(T_ready / CONTROL_T); k++)
	{
		Tra_RAnkle.z[k] = S[0][j++];
#ifdef STANDSTILL
		Tra_RAnkle.z[k] = H_ANKLE;
#endif
	}

	/* First step */
	// single phase 
	j = 0;
	TSpline_S_V_A(H_ANKLE, 0, 0, 0, H_ANKLE + H_step, T_step1*(1 - Pct_dou)*mm, H_ANKLE + H_minhi, 0, 0, T_step1*(1 - Pct_dou), CONTROL_T);
	for (k = round(T_ready / CONTROL_T); k <= round((T_ready + T_step1*(1 - Pct_dou)) / CONTROL_T); k++)
	{
		Tra_LAnkle.z[k] = S[0][j++];
#ifdef STANDSTILL
		Tra_LAnkle.z[k] = H_ANKLE;
#endif
	}

	for (k = round(T_ready / CONTROL_T); k <= round((T_ready + Pct_sin1*T_step1*(1 - Pct_dou)) / CONTROL_T); k++)
	{
		Tra_RAnkle.z[k] = H_ANKLE + H_minlow;
#ifdef STANDSTILL
		Tra_RAnkle.z[k] = H_ANKLE;
#endif
	}
	j = 0;
	TSpline_S_V_A(H_ANKLE + H_minlow, 0, 0, 0, H_ANKLE + H_minlow*0.5, (Pct_sin2 - Pct_sin1)*T_step1*(1 - Pct_dou)*0.5, H_ANKLE, 0, 0, (Pct_sin2 - Pct_sin1)*T_step1*(1 - Pct_dou), CONTROL_T);
	for (k = round((T_ready + Pct_sin1*T_step1*(1 - Pct_dou)) / CONTROL_T); k <= round((T_ready + Pct_sin2*T_step1*(1 - Pct_dou)) / CONTROL_T); k++)
	{
		Tra_RAnkle.z[k] = S[0][j++];
#ifdef STANDSTILL
		Tra_RAnkle.z[k] = H_ANKLE;
#endif
	}
	for (k = round((T_ready + Pct_sin2*T_step1*(1 - Pct_dou)) / CONTROL_T); k <= round((T_ready + T_step1*(1 - Pct_dou)) / CONTROL_T); k++)
	{
		Tra_RAnkle.z[k] = H_ANKLE;
#ifdef STANDSTILL
		Tra_RAnkle.z[k] = H_ANKLE;
#endif
	}

	// double phase 
	j = 0;
	TSpline_S_V_A(H_ANKLE + H_minhi, 0, 0, 0, H_ANKLE + H_minhi*0.5, Pct_dou1*T_step1*Pct_dou*0.5, H_ANKLE, 0, 0, Pct_dou1*T_step1*Pct_dou, CONTROL_T);
	for (k = round((T_ready + T_step1*(1 - Pct_dou)) / CONTROL_T); k <= round((T_ready + T_step1*(1 - Pct_dou) + Pct_dou1*T_step1*Pct_dou) / CONTROL_T); k++)
	{
		Tra_LAnkle.z[k] = S[0][j++];
#ifdef STANDSTILL
		Tra_LAnkle.z[k] = H_ANKLE;
#endif
	}
	for (k = round((T_ready + T_step1*(1 - Pct_dou) + Pct_dou1*T_step1*Pct_dou) / CONTROL_T); k <= round((T_ready + T_step1*(1 - Pct_dou) + Pct_dou2*T_step1*Pct_dou) / CONTROL_T); k++)
	{
		Tra_LAnkle.z[k] = H_ANKLE;
#ifdef STANDSTILL
		Tra_LAnkle.z[k] = H_ANKLE;
#endif
	}
	j = 0;
	TSpline_S_V_A(H_ANKLE, 0, 0, 0, H_ANKLE + H_minlow*0.5, (1 - Pct_dou2)*T_step1*Pct_dou*0.5, H_ANKLE + H_minlow, 0, 0, (1 - Pct_dou2)*T_step1*Pct_dou, CONTROL_T);
	for (k = round((T_ready + T_step1*(1 - Pct_dou) + Pct_dou2*T_dou) / CONTROL_T); k <= round((T_ready + T_step) / CONTROL_T); k++)
	{
		Tra_LAnkle.z[k] = S[0][j++];
#ifdef STANDSTILL
		Tra_LAnkle.z[k] = H_ANKLE;
#endif
	}

	for (k = round((T_ready + T_step1*(1 - Pct_dou)) / CONTROL_T); k <= round((T_ready + T_step1) / CONTROL_T); k++)
	{
		Tra_RAnkle.z[k] = H_ANKLE;
	}

	/* Second Step ...(N_step-2) Step */
	for (k_step = 1; k_step<N_step - 1; k_step++)
	{
		if (k_step % 2 != 0) // right leg swing 1,3,5...
		{
			// single phase 
			for (k = round((T_ready + T_step1 + (k_step - 1)*T_step) / CONTROL_T); k <= round((T_ready + T_step1 + (k_step - 1)*T_step + Pct_sin1*T_sin) / CONTROL_T); k++)
			{
				Tra_LAnkle.z[k] = H_ANKLE + H_minlow;
#ifdef STANDSTILL
				Tra_LAnkle.z[k] = H_ANKLE;
#endif
			}
			j = 0;
			TSpline_S_V_A(H_ANKLE + H_minlow, 0, 0, 0, H_ANKLE + H_minlow*0.5, (Pct_sin2 - Pct_sin1)*T_sin*0.5, H_ANKLE, 0, 0, (Pct_sin2 - Pct_sin1)*T_sin, CONTROL_T);
			for (k = round((T_ready + T_step1 + (k_step - 1)*T_step + Pct_sin1*T_sin) / CONTROL_T); k <= round((T_ready + T_step1 + (k_step - 1)*T_step + Pct_sin2*T_sin) / CONTROL_T); k++)
			{
				Tra_LAnkle.z[k] = S[0][j++];
#ifdef STANDSTILL
				Tra_LAnkle.z[k] = H_ANKLE;
#endif
			}
			for (k = round((T_ready + T_step1 + (k_step - 1)*T_step + Pct_sin2*T_sin) / CONTROL_T); k <= round((T_ready + T_step1 + (k_step - 1)*T_step + T_sin) / CONTROL_T); k++)
			{
				Tra_LAnkle.z[k] = H_ANKLE;
#ifdef STANDSTILL
				Tra_LAnkle.z[k] = H_ANKLE;
#endif
			}

			j = 0;
			TSpline_S_V_A(H_ANKLE, 0, 0, 0, H_ANKLE + H_step, T_sin*mm, H_ANKLE + H_minhi, 0, 0, T_sin, CONTROL_T);
			for (k = round((T_ready + T_step1 + (k_step - 1)*T_step) / CONTROL_T); k <= round((T_ready + T_step1 + (k_step - 1)*T_step + T_sin) / CONTROL_T); k++)
			{
				Tra_RAnkle.z[k] = S[0][j++];
#ifdef STANDSTILL
				Tra_RAnkle.z[k] = H_ANKLE;
#endif
			}
			/* double period */
			for (k = round((T_ready + T_step1 + (k_step - 1)*T_step + T_sin) / CONTROL_T); k <= round((T_ready + T_step1 + (k_step - 1)*T_step + T_step) / CONTROL_T); k++)
			{
				Tra_LAnkle.z[k] = H_ANKLE;
#ifdef STANDSTILL
				Tra_LAnkle.z[k] = H_ANKLE;
#endif
			}

			j = 0;
			TSpline_S_V_A(H_ANKLE + H_minhi, 0, 0, 0, H_ANKLE + H_minhi*0.5, Pct_dou1*T_dou*0.5, H_ANKLE, 0, 0, Pct_dou1*T_dou, CONTROL_T);
			for (k = round((T_ready + T_step1 + (k_step - 1)*T_step + T_sin) / CONTROL_T); k <= round((T_ready + T_step1 + (k_step - 1)*T_step + T_sin + Pct_dou1*T_dou) / CONTROL_T); k++)
			{
				Tra_RAnkle.z[k] = S[0][j++];
#ifdef STANDSTILL
				Tra_RAnkle.z[k] = H_ANKLE;
#endif
			}
			for (k = round((T_ready + T_step1 + (k_step - 1)*T_step + T_sin + Pct_dou1*T_dou) / CONTROL_T); k <= round((T_ready + T_step1 + (k_step - 1)*T_step + T_sin + Pct_dou2*T_dou) / CONTROL_T); k++)
			{
				Tra_RAnkle.z[k] = H_ANKLE;
#ifdef STANDSTILL
				Tra_RAnkle.z[k] = H_ANKLE;
#endif
			}
			j = 0;
			TSpline_S_V_A(H_ANKLE, 0, 0, 0, H_ANKLE + H_minlow*0.5, (1 - Pct_dou2)*T_dou*0.5, H_ANKLE + H_minlow, 0, 0, (1 - Pct_dou2)*T_dou, CONTROL_T);
			for (k = round((T_ready + T_step1 + (k_step - 1)*T_step + T_sin + Pct_dou2*T_dou) / CONTROL_T); k <= round((T_ready + T_step1 + (k_step - 1)*T_step + T_step) / CONTROL_T); k++)
			{
				Tra_RAnkle.z[k] = S[0][j++];
#ifdef STANDSTILL
				Tra_RAnkle.z[k] = H_ANKLE;
#endif
			}
		}


		if (k_step % 2 == 0) // left leg swing   0,2,4...
		{
			// single phase
			j = 0;
			TSpline_S_V_A(H_ANKLE, 0, 0, 0, H_ANKLE + H_step, T_sin*mm, H_ANKLE + H_minhi, 0, 0, T_sin, CONTROL_T);
			for (k = round((T_ready + T_step1 + (k_step - 1)*T_step) / CONTROL_T); k <= round((T_ready + T_step1 + (k_step - 1)*T_step + T_sin) / CONTROL_T); k++)
			{
				Tra_LAnkle.z[k] = S[0][j++];
#ifdef STANDSTILL
				Tra_LAnkle.z[k] = H_ANKLE;
#endif
			}

			for (k = round((T_ready + T_step1 + (k_step - 1)*T_step) / CONTROL_T); k <= round((T_ready + T_step1 + (k_step - 1)*T_step + Pct_sin1*T_sin) / CONTROL_T); k++)
			{
				Tra_RAnkle.z[k] = H_ANKLE + H_minlow;
#ifdef STANDSTILL
				Tra_RAnkle.z[k] = H_ANKLE;
#endif
			}
			j = 0;
			TSpline_S_V_A(H_ANKLE + H_minlow, 0, 0, 0, H_ANKLE + H_minlow*0.5, (Pct_sin2 - Pct_sin1)*T_sin*0.5, H_ANKLE, 0, 0, (Pct_sin2 - Pct_sin1)*T_sin, CONTROL_T);
			for (k = round((T_ready + T_step1 + (k_step - 1)*T_step + Pct_sin1*T_sin) / CONTROL_T); k <= round((T_ready + T_step1 + (k_step - 1)*T_step + Pct_sin2*T_sin) / CONTROL_T); k++)
			{
				Tra_RAnkle.z[k] = S[0][j++];
#ifdef STANDSTILL
				Tra_RAnkle.z[k] = H_ANKLE;
#endif
			}
			for (k = round((T_ready + T_step1 + (k_step - 1)*T_step + Pct_sin2*T_sin) / CONTROL_T); k <= round((T_ready + T_step1 + (k_step - 1)*T_step + T_sin) / CONTROL_T); k++)
			{
				Tra_RAnkle.z[k] = H_ANKLE;
#ifdef STANDSTILL
				Tra_RAnkle.z[k] = H_ANKLE;
#endif
			}


			// double phase			
			j = 0;
			TSpline_S_V_A(H_ANKLE + H_minhi, 0, 0, 0, H_ANKLE + H_minhi*0.5, Pct_dou1*T_dou*0.5, H_ANKLE, 0, 0, Pct_dou1*T_dou, CONTROL_T);
			for (k = round((T_ready + T_step1 + (k_step - 1)*T_step + T_sin) / CONTROL_T); k <= round((T_ready + T_step1 + (k_step - 1)*T_step + T_sin + Pct_dou1*T_dou) / CONTROL_T); k++)
			{
				Tra_LAnkle.z[k] = S[0][j++];
#ifdef STANDSTILL
				Tra_LAnkle.z[k] = H_ANKLE;
#endif
			}
			for (k = round((T_ready + T_step1 + (k_step - 1)*T_step + T_sin + Pct_dou1*T_dou) / CONTROL_T); k <= round((T_ready + T_step1 + (k_step - 1)*T_step + T_sin + Pct_dou2*T_dou) / CONTROL_T); k++)
			{
				Tra_LAnkle.z[k] = H_ANKLE;
#ifdef STANDSTILL
				Tra_LAnkle.z[k] = H_ANKLE;
#endif
			}
			j = 0;
			TSpline_S_V_A(H_ANKLE, 0, 0, 0, H_ANKLE + H_minlow*0.5, (1 - Pct_dou2)*T_dou*0.5, H_ANKLE + H_minlow, 0, 0, (1 - Pct_dou2)*T_dou, CONTROL_T);
			for (k = round((T_ready + T_step1 + (k_step - 1)*T_step + T_sin + Pct_dou2*T_dou) / CONTROL_T); k <= round((T_ready + T_step1 + (k_step - 1)*T_step + T_step) / CONTROL_T); k++)
			{
				Tra_LAnkle.z[k] = S[0][j++];
#ifdef STANDSTILL
				Tra_LAnkle.z[k] = H_ANKLE;
#endif
			}

			for (k = round((T_ready + T_step1 + (k_step - 1)*T_step + T_sin) / CONTROL_T); k <= round((T_ready + T_step1 + (k_step - 1)*T_step + T_step) / CONTROL_T); k++)
			{
				Tra_RAnkle.z[k] = H_ANKLE;
			}
		}
	}

	/* Last Step */
	if (N_step % 2 == 0)  // right leg swing 
	{
		// single phase
		for (k = round((T_ready + T_step1 + (N_step - 2)*T_step) / CONTROL_T); k <= round((T_ready + T_step1 + (N_step - 2)*T_step + Pct_sin1*T_stepn*(1 - Pct_dou)) / CONTROL_T); k++)
		{
			Tra_LAnkle.z[k] = H_ANKLE + H_minlow;
#ifdef STANDSTILL
			Tra_LAnkle.z[k] = H_ANKLE;
#endif
		}
		j = 0;
		TSpline_S_V_A(H_ANKLE + H_minlow, 0, 0, 0, H_ANKLE + H_minlow*0.5, (Pct_sin2 - Pct_sin1)*T_stepn*(1 - Pct_dou)*0.5, H_ANKLE, 0, 0, (Pct_sin2 - Pct_sin1)*T_stepn*(1 - Pct_dou), CONTROL_T);
		for (k = round((T_ready + T_step1 + (N_step - 2)*T_step + Pct_sin1*T_stepn*(1 - Pct_dou)) / CONTROL_T); k <= round((T_ready + T_step1 + (N_step - 2)*T_step + Pct_sin2*T_stepn*(1 - Pct_dou)) / CONTROL_T); k++)
		{
			Tra_LAnkle.z[k] = S[0][j++];
#ifdef STANDSTILL
			Tra_LAnkle.z[k] = H_ANKLE;
#endif
		}
		for (k = round((T_ready + T_step1 + (N_step - 2)*T_step + Pct_sin2*T_stepn*(1 - Pct_dou)) / CONTROL_T); k <= round((T_ready + T_step1 + (N_step - 2)*T_step + T_stepn*(1 - Pct_dou)) / CONTROL_T); k++)
		{
			Tra_LAnkle.z[k] = H_ANKLE;
#ifdef STANDSTILL
			Tra_LAnkle.z[k] = H_ANKLE;
#endif
		}

		j = 0;
		TSpline_S_V_A(H_ANKLE, 0, 0, 0, H_ANKLE + H_step, T_stepn*(1 - Pct_dou)*mm, H_ANKLE + H_minhi, 0, 0, T_stepn*(1 - Pct_dou), CONTROL_T);
		for (k = round((T_ready + T_step1 + (N_step - 2)*T_step) / CONTROL_T); k <= round((T_ready + T_step1 + (N_step - 2)*T_step + T_stepn*(1 - Pct_dou)) / CONTROL_T); k++)
		{
			Tra_RAnkle.z[k] = S[0][j++];
#ifdef STANDSTILL
			Tra_RAnkle.z[k] = H_ANKLE;
#endif
		}

		// double phase
		for (k = round((T_ready + T_step1 + (N_step - 2)*T_step + T_stepn*(1 - Pct_dou)) / CONTROL_T); k <= round((T_ready + T_step1 + (N_step - 2)*T_step + T_stepn) / CONTROL_T); k++)
		{
			Tra_LAnkle.z[k] = H_ANKLE;
#ifdef STANDSTILL
			Tra_LAnkle.z[k] = H_ANKLE;
#endif
		}

		j = 0;
		TSpline_S_V_A(H_ANKLE + H_minhi, 0, 0, 0, H_ANKLE + H_minhi*0.5, Pct_dou1*T_stepn*Pct_dou*0.5, H_ANKLE, 0, 0, Pct_dou1*T_stepn*Pct_dou, CONTROL_T);
		for (k = round((T_ready + T_step1 + (N_step - 2)*T_step + T_stepn*(1 - Pct_dou)) / CONTROL_T); k <= round((T_ready + T_step1 + (N_step - 2)*T_step + T_stepn*(1 - Pct_dou) + Pct_dou1*T_stepn*Pct_dou) / CONTROL_T); k++)
		{
			Tra_RAnkle.z[k] = S[0][j++];
#ifdef STANDSTILL
			Tra_RAnkle.z[k] = H_ANKLE;
#endif
		}
		for (k = round((T_ready + T_step1 + (N_step - 2)*T_step + T_stepn*(1 - Pct_dou) + Pct_dou1*T_stepn*Pct_dou) / CONTROL_T); k <= round((T_ready + T_step1 + (N_step - 2)*T_step + T_stepn) / CONTROL_T); k++)
		{
			Tra_RAnkle.z[k] = H_ANKLE;
#ifdef STANDSTILL
			Tra_RAnkle.z[k] = H_ANKLE;
#endif
		}
	}

	if (N_step % 2 != 0)  // left leg swing
	{
		// single phase
		j = 0;
		TSpline_S_V_A(H_ANKLE, 0, 0, 0, H_ANKLE + H_step, T_stepn*(1 - Pct_dou)*mm, H_ANKLE + H_minhi, 0, 0, T_stepn*(1 - Pct_dou), CONTROL_T);
		for (k = round((T_ready + T_step1 + (N_step - 2)*T_step) / CONTROL_T); k <= round((T_ready + T_step1 + (N_step - 2)*T_step + T_stepn*(1 - Pct_dou)) / CONTROL_T); k++)
		{
			Tra_LAnkle.z[k] = S[0][j++];
#ifdef STANDSTILL
			Tra_LAnkle.z[k] = H_ANKLE;
#endif
		}

		for (k = round((T_ready + T_step1 + (N_step - 2)*T_step) / CONTROL_T); k <= round((T_ready + T_step1 + (N_step - 2)*T_step + T_stepn*(1 - Pct_dou)*Pct_sin1) / CONTROL_T); k++)
		{
			Tra_RAnkle.z[k] = H_ANKLE + H_minlow;
#ifdef STANDSTILL
			Tra_RAnkle.z[k] = H_ANKLE;
#endif
		}
		j = 0;
		TSpline_S_V_A(H_ANKLE + H_minlow, 0, 0, 0, H_ANKLE + H_minlow*0.5, T_stepn*(1 - Pct_dou)*(Pct_sin2 - Pct_sin1)*0.5, H_ANKLE, 0, 0, T_stepn*(1 - Pct_dou)*(Pct_sin2 - Pct_sin1), CONTROL_T);
		for (k = round((T_ready + T_step1 + (N_step - 2)*T_step + T_stepn*(1 - Pct_dou)*Pct_sin1) / CONTROL_T); k <= round((T_ready + T_step1 + (N_step - 2)*T_step + T_stepn*(1 - Pct_dou)*Pct_sin2) / CONTROL_T); k++)
		{
			Tra_RAnkle.z[k] = S[0][j++];
#ifdef STANDSTILL
			Tra_RAnkle.z[k] = H_ANKLE;
#endif
		}
		for (k = round((T_ready + T_step1 + (N_step - 2)*T_step + T_stepn*(1 - Pct_dou)*Pct_sin2) / CONTROL_T); k <= round((T_ready + T_step1 + (N_step - 2)*T_step + T_stepn*(1 - Pct_dou)) / CONTROL_T); k++)
		{
			Tra_RAnkle.z[k] = H_ANKLE;
#ifdef STANDSTILL
			Tra_RAnkle.z[k] = H_ANKLE;
#endif
		}

		// double phase
		j = 0;
		TSpline_S_V_A(H_ANKLE + H_minhi, 0, 0, 0, H_ANKLE + H_minhi*0.5, T_stepn*Pct_dou*Pct_dou1*0.5, H_ANKLE, 0, 0, T_stepn*Pct_dou*Pct_dou1, CONTROL_T);
		for (k = round((T_ready + T_step1 + (N_step - 2)*T_step + T_stepn*(1 - Pct_dou)) / CONTROL_T); k <= round((T_ready + T_step1 + (N_step - 2)*T_step + T_stepn*(1 - Pct_dou) + T_stepn*Pct_dou*Pct_dou1) / CONTROL_T); k++)
		{
			Tra_LAnkle.z[k] = S[0][j++];
#ifdef STANDSTILL
			Tra_LAnkle.z[k] = H_ANKLE;
#endif
		}
		for (k = round((T_ready + T_step1 + (N_step - 2)*T_step + T_stepn*(1 - Pct_dou) + T_stepn*Pct_dou*Pct_dou1) / CONTROL_T); k <= round((T_ready + T_step1 + (N_step - 2)*T_step + T_stepn) / CONTROL_T); k++)
		{
			Tra_LAnkle.z[k] = H_ANKLE;
		}

		for (k = round((T_ready + T_step1 + (N_step - 2)*T_step + T_stepn*(1 - Pct_dou)) / CONTROL_T); k <= round((T_ready + T_step1 + (N_step - 2)*T_step + T_stepn) / CONTROL_T); k++)
		{
			Tra_RAnkle.z[k] = H_ANKLE;
		}
	}

	for (k = round((T_ready + T_step1 + (N_step - 2)*T_step + T_stepn) / CONTROL_T); k <= round(T_com / CONTROL_T); k++)
	{
		Tra_LAnkle.z[k] = H_ANKLE;
		Tra_RAnkle.z[k] = H_ANKLE;
	}

	/* 	 if(NULL==(fp=fopen("ankle_z.dat","w")))
	{
	printf("ankle_z.dat can not open!\n");
	}
	for(k=0;k<=round(T_com/CONTROL_T);k++)     // COM_t
	fprintf(fp,"%lf\t%lf\n",Tra_RAnkle.z[k],Tra_LAnkle.z[k]);
	fclose(fp); */

}


/******************************************* Functions For Move Side x ********************************************/
void ZMP_Tra_MoveRightSide()
{
	int j;
	int k;
	int k_step,k_side;
	double zst=0.0,zed=0.0;
	
	/************************************** XZMP x****************************************/	
	/* T_ready - Descend COM Height */
	for(k=0;k<=round((T_ready-1.2)/CONTROL_T);k++)
	{	
		Tra_ZMP.x[k]=0;
	}
	
	j=0;
	TSpline_S_V_A(0.0,0,0,0,0.5*ZMP_width*0.5,1.2/2,0.5*ZMP_width,0,0,1.2,CONTROL_T);
	for(k=round((T_ready-1.2)/CONTROL_T);k<=round(T_ready/CONTROL_T);k++)
	{	
		Tra_ZMP.x[k]=0.0;//S[0][j];
		j++;
	}
	
	/* Move Right Side */
	// First Step : Lift Left Leg but do not move aside
	k_step = 0;
	for(k=round((T_ready+k_step*T_step)/CONTROL_T);k<=round((T_ready+k_step*T_step+T_sin)/CONTROL_T);k++)   // front 0.5*T_sin
	{
		Tra_ZMP.x[k]=0.0;//0.5*ZMP_width;
	}

	j=0;
	zst= 0.0;//0.5*ZMP_width;
	zed=-0.5*ZMP_width;
	TSpline_S_V_A(zst,0,0,0,(zst+zed)*0.5,T_dou/2,zed,0,0,T_dou,CONTROL_T);
	for(k=round((T_ready+k_step*T_step+T_sin)/CONTROL_T);k<=round((T_ready+(k_step+1)*T_step)/CONTROL_T);k++)   // T_dou
	{
		Tra_ZMP.x[k]=0.0;//S[0][j];
		j++;
	}
	
	// 0,1,2..N_side-2
	k_step = 1;
	for(k_side=0;k_side<N_side-1;k_side++)
	{
		if(k_step%2!=0)  // right step 0,2,4...
		{
			for(k=round((T_ready+k_step*T_step)/CONTROL_T);k<=round((T_ready+k_step*T_step+T_sin)/CONTROL_T);k++)   // front 0.5*T_sin
			{
				Tra_ZMP.x[k]=-0.5*ZMP_width+k_side*L_side;
			}

			j=0;
			zst=-0.5*ZMP_width+k_side*L_side;
			zed= 0.5*ZMP_width+(k_side+1)*L_side;
			TSpline_S_V_A(zst,0,0,0,(zst+zed)*0.5,T_dou/2,zed,0,0,T_dou,CONTROL_T);
			for(k=round((T_ready+k_step*T_step+T_sin)/CONTROL_T);k<=round((T_ready+(k_step+1)*T_step)/CONTROL_T);k++)   // T_dou
			{
				Tra_ZMP.x[k]=S[0][j];
				j++;
			}

		}
		k_step=k_step+1;
		if(k_step%2==0)  // left step 1,3,5...
		{
			for(k=round((T_ready+k_step*T_step)/CONTROL_T);k<=round((T_ready+k_step*T_step+T_sin)/CONTROL_T);k++)   // front 0.5*T_sin
			{
				Tra_ZMP.x[k]=0.5*ZMP_width+(k_side+1)*L_side;
			}
			
			j=0;
			zst= 0.5*ZMP_width+(k_side+1)*L_side;
			zed=-0.5*ZMP_width+(k_side+1)*L_side;
			TSpline_S_V_A(zst,0,0,0,(zst+zed)*0.5,T_dou/2,zed,0,0,T_dou,CONTROL_T);
			for(k=round((T_ready+k_step*T_step+T_sin)/CONTROL_T);k<=round((T_ready+(k_step+1)*T_step)/CONTROL_T);k++)   // T_dou
			{
				Tra_ZMP.x[k]=S[0][j];
				j++;
			}

		}
		k_step=k_step+1;
	}
	// Last : N_side-1
	k_side = N_side-1;
	if(k_step%2!=0)  // right step 0,2,4...
	{
		for(k=round((T_ready+k_step*T_step)/CONTROL_T);k<=round((T_ready+k_step*T_step+T_sin)/CONTROL_T);k++)   // front 0.5*T_sin
		{
			Tra_ZMP.x[k]=-0.5*ZMP_width+k_side*L_side;
		}

		j=0;
		zst=-0.5*ZMP_width+k_side*L_side;
		zed= 0.5*ZMP_width+(k_side+1)*L_side;
		TSpline_S_V_A(zst,0,0,0,(zst+zed)*0.5,T_dou/2,zed,0,0,T_dou,CONTROL_T);
		for(k=round((T_ready+k_step*T_step+T_sin)/CONTROL_T);k<=round((T_ready+(k_step+1)*T_step)/CONTROL_T);k++)   // T_dou
		{
			Tra_ZMP.x[k]=S[0][j];
			j++;
		}

	}
	k_step=k_step+1;
	if(k_step%2==0)  // left step 1,3,5...
	{
		for(k=round((T_ready+k_step*T_step)/CONTROL_T);k<=round((T_ready+k_step*T_step+T_sin)/CONTROL_T);k++)   // front 0.5*T_sin
		{
			Tra_ZMP.x[k]= 0.5*ZMP_width+(k_side+1)*L_side;
		}
		
		j=0;
		zst= 0.5*ZMP_width+(k_side+1)*L_side;
		zed=-0.5*ZMP_width+(k_side+1)*L_side+0.5*ZMP_width;
		TSpline_S_V_A(zst,0,0,0,(zst+zed)*0.5,T_dou/2,zed,0,0,T_dou,CONTROL_T);
		for(k=round((T_ready+k_step*T_step+T_sin)/CONTROL_T);k<=round((T_ready+(k_step+1)*T_step)/CONTROL_T);k++)   // T_dou
		{
			Tra_ZMP.x[k]=S[0][j];
			j++;
		}

	}
	/* Reset */
	for((k=round((T_ready+N_step*T_step)/CONTROL_T));k<=round(T_zmp/CONTROL_T);k++)
	{
		Tra_ZMP.x[k]= N_side*L_side;
	}

	
/* 	if(NULL==(fp=fopen("moveRside_xzmp.dat","w")))
	{
		printf("moveRside_xzmp File can not open!\n");
	}
	for(k=0;k<=round((T_zmp)/CONTROL_T);k++)     // ZMP_t
	fprintf(fp,"%lf\n",Tra_ZMP.x[k]);						
	fclose(fp); */
	
/************************************** XZMP s****************************************/

/************************************** YZMP x****************************************/
	for(k=0;k<=round(T_zmp/CONTROL_T);k++)
	{
		Tra_ZMP.y[k]=0.0;
	}
	
/* 	if(NULL==(fp=fopen("moveRside_yzmp.dat","w")))
	{
		printf("moveRside_yzmp File can not open!\n");
	}
	for(k=0;k<round((T_zmp)/CONTROL_T);k++)     // ZMP_t
	fprintf(fp,"%lf\n",Tra_ZMP.y[k]);						
	fclose(fp); */
/************************************** YZMP s****************************************/
		
}

void Ankle_Tra_MoveRightSide()
{
	int k;
	int j;
	int k_step,k_side;
	double akx_s,akx_e;
	
	/* T_ready - Descend COM Height */
	for(k=0;k<=round(T_ready/CONTROL_T);k++)
	{		
		Tra_RAnkle.x[k]=ANKLE_WIDTH/2;
		Tra_RAnkle.y[k]=0;		
		Tra_RAnkle.z[k]=H_ANKLE;
		
		Tra_LAnkle.x[k]=-ANKLE_WIDTH/2;
		Tra_LAnkle.y[k]=0;
		Tra_LAnkle.z[k]=H_ANKLE;
	}
	
/************************************** Ankle Tra X x****************************************/
	/* Move Right Side */
	// First Step : Lift Left Leg but do not move aside
	k_step = 0;
	for(k=round((T_ready+k_step*T_step)/CONTROL_T);k<=round((T_ready+(k_step+1)*T_step)/CONTROL_T);k++)   
	{
		Tra_RAnkle.x[k]=ANKLE_WIDTH/2;
		Tra_LAnkle.x[k]=-ANKLE_WIDTH/2;
	}

	// 0...(N_side-1)
	k_step = 1;
	for(k_side=0;k_side<N_side;k_side++)
	{
		if(k_step%2!=0)  // right step 0,2,4...
		{
			j=0;
			akx_s = 0.5*ANKLE_WIDTH+k_side*L_side;
			akx_e = 0.5*ANKLE_WIDTH+(k_side+1)*L_side;
			TSpline_S_V_A(akx_s,0,0,0,(akx_s+akx_e)*0.5,T_sin/2,akx_e,0,0,T_sin,CONTROL_T);		
			for(k=round((T_ready+k_step*T_step)/CONTROL_T);k<=round((T_ready+k_step*T_step+T_sin)/CONTROL_T);k++)   // front T_sin
			{
				Tra_RAnkle.x[k] = S[0][j];				
				Tra_LAnkle.x[k] =-ANKLE_WIDTH*0.5+k_side*L_side;
				j=j+1;				
			}
			for (k=round((T_ready+k_step*T_step+T_sin) / CONTROL_T); k<=round((T_ready+k_step*T_step+T_step) / CONTROL_T); k++)   // double period
			{
				Tra_RAnkle.x[k] = ANKLE_WIDTH*0.5+(k_side+1)*L_side;
				Tra_LAnkle.x[k] = -ANKLE_WIDTH*0.5+k_side*L_side;
			}
		}
		k_step=k_step+1;
		if(k_step%2==0)  // left step 1,3,5...
		{
			j=0;
			akx_s = -0.5*ANKLE_WIDTH+k_side*L_side;
			akx_e = -0.5*ANKLE_WIDTH+(k_side+1)*L_side;
			TSpline_S_V_A(akx_s,0,0,0,(akx_s+akx_e)*0.5,T_sin/2,akx_e,0,0,T_sin,CONTROL_T);		
			for(k=round((T_ready+k_step*T_step)/CONTROL_T);k<=round((T_ready+k_step*T_step+T_sin)/CONTROL_T);k++)   // front 0.5*T_sin
			{
				Tra_RAnkle.x[k] = ANKLE_WIDTH*0.5+(k_side+1)*L_side;		
				Tra_LAnkle.x[k] = S[0][j];		
				j=j+1;
			}
			for(k=round((T_ready+k_step*T_step+T_sin)/CONTROL_T);k<=round((T_ready+(k_step+1)*T_step)/CONTROL_T);k++)   // T_dou
			{
				Tra_RAnkle.x[k] = ANKLE_WIDTH*0.5+(k_side+1)*L_side;
				Tra_LAnkle.x[k] = -ANKLE_WIDTH*0.5+(k_side+1)*L_side;
			}

		}
		k_step=k_step+1;
		
	}
	for(k=round((T_ready+N_step*T_step)/CONTROL_T);k<=round(T_com/CONTROL_T);k++)   
	{
		Tra_RAnkle.x[k] = ANKLE_WIDTH*0.5+N_side*L_side;
		Tra_LAnkle.x[k] = -ANKLE_WIDTH*0.5+N_side*L_side;
	}
	
/************************************** Ankle Tra X s****************************************/
/************************************** Ankle Tra Y x****************************************/
	for(k=round(T_ready/CONTROL_T);k<=round(T_com/CONTROL_T);k++)
	{
		Tra_RAnkle.y[k]=0;	
		Tra_LAnkle.y[k]=0;
	}
/************************************** Ankle Tra Y s****************************************/
/************************************** Ankle Tra Z x****************************************/
	// First Step : Lift Left Leg but do not move aside
	k_step=0;
	j=0;
	TSpline_S_V_A(H_ANKLE,0,0,0,H_ANKLE+H_step,T_sin/2,H_ANKLE,0,0,T_sin,CONTROL_T);
	for(k=round((T_ready+k_step*T_step)/CONTROL_T);k<=round((T_ready+k_step*T_step+T_sin)/CONTROL_T);k++)   // front T_sin
	{
		Tra_RAnkle.z[k]=H_ANKLE;
		Tra_LAnkle.z[k]=H_ANKLE;//S[0][j++];
	}
	for(k=round((T_ready+k_step*T_step+T_sin)/CONTROL_T);k<=round((T_ready+(k_step+1)*T_step)/CONTROL_T);k++)   // T_dou
	{
		Tra_RAnkle.z[k]=H_ANKLE;
		Tra_LAnkle.z[k]=H_ANKLE;
	}
	
	// 0...(N_side-1)
	k_step=1;	
	for(k_side=0;k_side<N_side;k_side++)
	{
		if(k_step%2!=0)  // right step 1,3,5...
		{
			j=0;
			TSpline_S_V_A(H_ANKLE,0,0,0,H_ANKLE+H_step,T_sin/2,H_ANKLE,0,0,T_sin,CONTROL_T);
			for(k=round((T_ready+k_step*T_step)/CONTROL_T);k<=round((T_ready+k_step*T_step+T_sin)/CONTROL_T);k++)   // front T_sin
			{
				Tra_RAnkle.z[k]=S[0][j++];
				Tra_LAnkle.z[k]=H_ANKLE;
			}
			for(k=round((T_ready+k_step*T_step+T_sin)/CONTROL_T);k<=round((T_ready+(k_step+1)*T_step)/CONTROL_T);k++)   // T_dou
			{
				Tra_RAnkle.z[k]=H_ANKLE;
				Tra_LAnkle.z[k]=H_ANKLE;
			}
		}
		k_step=k_step+1;
		
		if(k_step%2==0)  // left step 0,2,4...
		{
			j=0;
			TSpline_S_V_A(H_ANKLE,0,0,0,H_ANKLE+H_step,T_sin/2,H_ANKLE,0,0,T_sin,CONTROL_T);
			for(k=round((T_ready+k_step*T_step)/CONTROL_T);k<=round((T_ready+k_step*T_step+T_sin)/CONTROL_T);k++)   // front T_sin
			{
				Tra_RAnkle.z[k]=H_ANKLE;
				Tra_LAnkle.z[k]=S[0][j++];
			}
			for(k=round((T_ready+k_step*T_step+T_sin)/CONTROL_T);k<=round((T_ready+(k_step+1)*T_step)/CONTROL_T);k++)   // T_dou
			{
				Tra_RAnkle.z[k]=H_ANKLE;
				Tra_LAnkle.z[k]=H_ANKLE;
			}
		}
		k_step=k_step+1;
		
	}	
	for(k=round((T_ready+N_step*T_step)/CONTROL_T);k<=round(T_com/CONTROL_T);k++)   
	{
		Tra_RAnkle.z[k]=H_ANKLE;
		Tra_LAnkle.z[k]=H_ANKLE;
	}
	

/************************************** Ankle Tra Z s****************************************/

/* 	if(NULL==(fp=fopen("moveRside_ankle_tra.dat","w")))
	{
		printf("moveRside_ankle_tra File can not open!\n");
	}
	for(k=0;k<round((T_ready+N_step*T_step+T_end+pp_compensation)/CONTROL_T);k++)     // COM_t
		fprintf(fp,"%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n",Tra_RAnkle.x[k],Tra_RAnkle.y[k],Tra_RAnkle.z[k],Tra_LAnkle.x[k],Tra_LAnkle.y[k],Tra_LAnkle.z[k]);						
	fclose(fp); */

}

void ZMP_Tra_MoveLeftSide()
{
	int j;
	int k;
	int k_step,k_side;
	double zst=0.0,zed=0.0;
	
	/************************************** XZMP x****************************************/	
	/* T_ready - Descend COM Height */
	for(k=0;k<=round((T_ready-1.2)/CONTROL_T);k++)
	{	
		Tra_ZMP.x[k]=0;
	}
	
	j=0;
	TSpline_S_V_A(0.0,0,0,0,0.5*ZMP_width*0.5,1.2/2,0.5*ZMP_width,0,0,1.2,CONTROL_T);
	for(k=round((T_ready-1.2)/CONTROL_T);k<=round(T_ready/CONTROL_T);k++)
	{	
		Tra_ZMP.x[k]=0.0;//S[0][j];
		j++;
	}
	
	/* Move Left Side */
	k_step = 0;
	// 0,1,2..N_side-2
	for(k_side=0;k_side<N_side-1;k_side++)
	{
		if(k_step%2==0)  // left step 0,2,4...
		{
			for(k=round((T_ready+k_step*T_step)/CONTROL_T);k<=round((T_ready+k_step*T_step+T_sin)/CONTROL_T);k++)   // front 0.5*T_sin
			{
				Tra_ZMP.x[k]=0.5*ZMP_width-k_side*L_side;
			}

			j=0;
			zst=0.5*ZMP_width-k_side*L_side;
			zed=-0.5*ZMP_width-(k_side+1)*L_side;
			TSpline_S_V_A(zst,0,0,0,(zst+zed)*0.5,T_dou/2,zed,0,0,T_dou,CONTROL_T);
			for(k=round((T_ready+k_step*T_step+T_sin)/CONTROL_T);k<=round((T_ready+(k_step+1)*T_step)/CONTROL_T);k++)   // T_dou
			{
				Tra_ZMP.x[k]=S[0][j];
				j++;
			}

		}
		k_step=k_step+1;
		if(k_step%2!=0)  // right step 1,3,5...
		{
			for(k=round((T_ready+k_step*T_step)/CONTROL_T);k<=round((T_ready+k_step*T_step+T_sin)/CONTROL_T);k++)   // front 0.5*T_sin
			{
				Tra_ZMP.x[k]=-0.5*ZMP_width-(k_side+1)*L_side;
			}
			
			j=0;
			zst=-0.5*ZMP_width-(k_side+1)*L_side;
			zed=0.5*ZMP_width-(k_side+1)*L_side;
			TSpline_S_V_A(zst,0,0,0,(zst+zed)*0.5,T_dou/2,zed,0,0,T_dou,CONTROL_T);
			for(k=round((T_ready+k_step*T_step+T_sin)/CONTROL_T);k<=round((T_ready+(k_step+1)*T_step)/CONTROL_T);k++)   // T_dou
			{
				Tra_ZMP.x[k]=S[0][j];
				j++;
			}

		}
		k_step=k_step+1;
	}
	// Last : N_side-1
	k_side = N_side-1;
	if(k_step%2==0)  // left step 0,2,4...
	{
		for(k=round((T_ready+k_step*T_step)/CONTROL_T);k<=round((T_ready+k_step*T_step+T_sin)/CONTROL_T);k++)   // front 0.5*T_sin
		{
			Tra_ZMP.x[k]=0.5*ZMP_width-k_side*L_side;
		}

		j=0;
		zst=0.5*ZMP_width-k_side*L_side;
		zed=-0.5*ZMP_width-(k_side+1)*L_side;
		TSpline_S_V_A(zst,0,0,0,(zst+zed)*0.5,T_dou/2,zed,0,0,T_dou,CONTROL_T);
		for(k=round((T_ready+k_step*T_step+T_sin)/CONTROL_T);k<=round((T_ready+(k_step+1)*T_step)/CONTROL_T);k++)   // T_dou
		{
			Tra_ZMP.x[k]=S[0][j];
			j++;
		}

	}
	k_step=k_step+1;
	if(k_step%2!=0)  // right step 1,3,5...
	{
		for(k=round((T_ready+k_step*T_step)/CONTROL_T);k<=round((T_ready+k_step*T_step+T_sin)/CONTROL_T);k++)   // front 0.5*T_sin
		{
			Tra_ZMP.x[k]=-0.5*ZMP_width-(k_side+1)*L_side;
		}
		
		j=0;
		zst=-0.5*ZMP_width-(k_side+1)*L_side;
		zed=0.5*ZMP_width-(k_side+1)*L_side-0.5*ZMP_width;
		TSpline_S_V_A(zst,0,0,0,(zst+zed)*0.5,T_dou/2,zed,0,0,T_dou,CONTROL_T);
		for(k=round((T_ready+k_step*T_step+T_sin)/CONTROL_T);k<=round((T_ready+(k_step+1)*T_step)/CONTROL_T);k++)   // T_dou
		{
			Tra_ZMP.x[k]=S[0][j];
			j++;
		}

	}
	/* Reset */
	for((k=round((T_ready+N_step*T_step)/CONTROL_T));k<=round(T_zmp/CONTROL_T);k++)
	{
		Tra_ZMP.x[k]=-N_side*L_side;
	}

	
/* 	if(NULL==(fp=fopen("moveLside_xzmp.dat","w")))
	{
		printf("moveLside_xzmp File can not open!\n");
	}
	for(k=0;k<round((T_zmp)/CONTROL_T);k++)     // ZMP_t
	fprintf(fp,"%lf\n",Tra_ZMP.x[k]);						
	fclose(fp); */
	
/************************************** XZMP s****************************************/

/************************************** YZMP x****************************************/
	for(k=0;k<round(T_zmp/CONTROL_T);k++)
	{
		Tra_ZMP.y[k]=0.0;
	}
	
/* 	if(NULL==(fp=fopen("moveLside_yzmp.dat","w")))
	{
		printf("moveLside_yzmp File can not open!\n");
	}
	for(k=0;k<round((T_zmp)/CONTROL_T);k++)     // ZMP_t
	fprintf(fp,"%lf\n",Tra_ZMP.y[k]);						
	fclose(fp); */
/************************************** YZMP s****************************************/
		
}

void Ankle_Tra_MoveLeftSide()
{
	int k;
	int j;
	int k_step,k_side;
	double akx_s,akx_e;
	
	/* T_ready - Descend COM Height */
	for(k=0;k<round(T_ready/CONTROL_T);k++)
	{		
		Tra_RAnkle.x[k]=ANKLE_WIDTH/2;
		Tra_RAnkle.y[k]=0;		
		Tra_RAnkle.z[k]=H_ANKLE;
		
		Tra_LAnkle.x[k]=-ANKLE_WIDTH/2;
		Tra_LAnkle.y[k]=0;
		Tra_LAnkle.z[k]=H_ANKLE;
	}
	
/************************************** Ankle Tra X x****************************************/
	/* Move Left Side */
	k_step = 0; 
	// 0...(N_side-1)
	for(k_side=0;k_side<N_side;k_side++)
	{
		if(k_step%2==0)  // left step 0,2,4...
		{
			j=0;
			akx_s = -0.5*ANKLE_WIDTH-k_side*L_side;
			akx_e = -0.5*ANKLE_WIDTH-(k_side+1)*L_side;
			TSpline_S_V_A(akx_s,0,0,0,(akx_s+akx_e)*0.5,T_sin/2,akx_e,0,0,T_sin,CONTROL_T);		
			for(k=round((T_ready+k_step*T_step)/CONTROL_T);k<round((T_ready+k_step*T_step+T_sin)/CONTROL_T);k++)   // front T_sin
			{
				Tra_RAnkle.x[k] = ANKLE_WIDTH*0.5-k_side*L_side;			
				Tra_LAnkle.x[k] = S[0][j];	
				j=j+1;				
			}
			for (k=round((T_ready+k_step*T_step+T_sin) / CONTROL_T); k<round((T_ready+k_step*T_step+T_step) / CONTROL_T); k++)   // double period
			{
				Tra_RAnkle.x[k] = ANKLE_WIDTH*0.5-k_side*L_side;
				Tra_LAnkle.x[k] = -ANKLE_WIDTH*0.5-(k_side+1)*L_side;
			}
		}
		k_step=k_step+1;
		if(k_step%2!=0)  // right step 1,3,5...
		{
			j=0;
			akx_s = 0.5*ANKLE_WIDTH-k_side*L_side;
			akx_e = 0.5*ANKLE_WIDTH-(k_side+1)*L_side;
			TSpline_S_V_A(akx_s,0,0,0,(akx_s+akx_e)*0.5,T_sin/2,akx_e,0,0,T_sin,CONTROL_T);		
			for(k=round((T_ready+k_step*T_step)/CONTROL_T);k<round((T_ready+k_step*T_step+T_sin)/CONTROL_T);k++)   // front 0.5*T_sin
			{
				Tra_RAnkle.x[k] = S[0][j];			
				Tra_LAnkle.x[k] = -ANKLE_WIDTH*0.5-(k_side+1)*L_side;	
				j=j+1;
			}
			for(k=round((T_ready+k_step*T_step+T_sin)/CONTROL_T);k<round((T_ready+(k_step+1)*T_step)/CONTROL_T);k++)   // T_dou
			{
				Tra_RAnkle.x[k] = ANKLE_WIDTH*0.5-(k_side+1)*L_side;
				Tra_LAnkle.x[k] = -ANKLE_WIDTH*0.5-(k_side+1)*L_side;
			}

		}
		k_step=k_step+1;
		
	}
	for(k=round((T_ready+N_step*T_step)/CONTROL_T);k<round(T_com/CONTROL_T);k++)   
	{
		Tra_RAnkle.x[k] = ANKLE_WIDTH*0.5-N_side*L_side;
		Tra_LAnkle.x[k] = -ANKLE_WIDTH*0.5-N_side*L_side;
	}
	
/************************************** Ankle Tra X s****************************************/
/************************************** Ankle Tra Y x****************************************/
	for(k=round(T_ready/CONTROL_T);k<round(T_com/CONTROL_T);k++)
	{
		Tra_RAnkle.y[k]=0;	
		Tra_LAnkle.y[k]=0;
	}
/************************************** Ankle Tra Y s****************************************/
/************************************** Ankle Tra Z x****************************************/
	k_step=0;
	for(k_side=0;k_side<N_side;k_side++)
	{
		if(k_step%2==0)  // left step 0,2,4...
		{
			j=0;
			TSpline_S_V_A(H_ANKLE,0,0,0,H_ANKLE+H_step,T_sin/2,H_ANKLE,0,0,T_sin,CONTROL_T);
			for(k=round((T_ready+k_step*T_step)/CONTROL_T);k<round((T_ready+k_step*T_step+T_sin)/CONTROL_T);k++)   // front T_sin
			{
				Tra_RAnkle.z[k]=H_ANKLE;
				Tra_LAnkle.z[k]=S[0][j++];
			}
			for(k=round((T_ready+k_step*T_step+T_sin)/CONTROL_T);k<round((T_ready+(k_step+1)*T_step)/CONTROL_T);k++)   // T_dou
			{
				Tra_RAnkle.z[k]=H_ANKLE;
				Tra_LAnkle.z[k]=H_ANKLE;
			}
		}
		k_step=k_step+1;
		if(k_step%2!=0)  // right step 1,3,5...
		{
			j=0;
			TSpline_S_V_A(H_ANKLE,0,0,0,H_ANKLE+H_step,T_sin/2,H_ANKLE,0,0,T_sin,CONTROL_T);
			for(k=round((T_ready+k_step*T_step)/CONTROL_T);k<round((T_ready+k_step*T_step+T_sin)/CONTROL_T);k++)   // front T_sin
			{
				Tra_RAnkle.z[k]=S[0][j++];
				Tra_LAnkle.z[k]=H_ANKLE;
			}
			for(k=round((T_ready+k_step*T_step+T_sin)/CONTROL_T);k<round((T_ready+(k_step+1)*T_step)/CONTROL_T);k++)   // T_dou
			{
				Tra_RAnkle.z[k]=H_ANKLE;
				Tra_LAnkle.z[k]=H_ANKLE;
			}
		}
		k_step=k_step+1;
	}	
	for(k=round((T_ready+N_step*T_step)/CONTROL_T);k<round(T_com/CONTROL_T);k++)   
	{
		Tra_RAnkle.z[k]=H_ANKLE;
		Tra_LAnkle.z[k]=H_ANKLE;
	}
	

/************************************** Ankle Tra Z s****************************************/

/* 	if(NULL==(fp=fopen("moveLside_ankle_tra.dat","w")))
	{
		printf("moveLside_ankle_tra File can not open!\n");
	}
	for(k=0;k<round((T_ready+N_step*T_step+T_end+pp_compensation)/CONTROL_T);k++)     // COM_t
		fprintf(fp,"%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n",Tra_RAnkle.x[k],Tra_RAnkle.y[k],Tra_RAnkle.z[k],Tra_LAnkle.x[k],Tra_LAnkle.y[k],Tra_LAnkle.z[k]);						
	fclose(fp); */

}

/******************************************* Functions For Move Side s ********************************************/


// dcc_rec
void Ankle_Tra_Cycloid()
{
	int k, j;
	int k_step;
	double x1, y1, x2, y2, td, dt;
	for (k = 0; k <= round(T_ready / CONTROL_T); k++)
	{
		Tra_RAnkle.x[k] = ANKLE_WIDTH / 2;
		Tra_RAnkle.y[k] = 0;
		Tra_RAnkle.z[k] = H_ANKLE;

		Tra_LAnkle.x[k] = -ANKLE_WIDTH / 2;
		Tra_LAnkle.y[k] = 0;
		Tra_LAnkle.z[k] = H_ANKLE;
#ifdef STANDSTILL
		Tra_RAnkle.x[k] = ANKLE_WIDTH / 2;
		Tra_RAnkle.y[k] = 0;

		Tra_LAnkle.x[k] = -ANKLE_WIDTH / 2;
		Tra_LAnkle.y[k] = 0;

		Tra_RAnkle.z[k] = H_ANKLE;
		Tra_LAnkle.z[k] = H_ANKLE;
#endif

	}

	/* First Step */
	// single phase
	j = 0;
	x1 = -ANKLE_WIDTH / 2;
	y1 = 0;
	x2 = -ANKLE_WIDTH / 2;
	y2 = L_step;
	td = T_step1*(1 - Pct_dou);
	dt = 0.004;
	Cycloid_Generate(x1, y1, x2, y2, td, dt);
	for (k = round(T_ready / CONTROL_T); k <= round((T_ready + T_step1*(1 - Pct_dou)) / CONTROL_T); k++)   // single period
	{
		Tra_RAnkle.x[k] = ANKLE_WIDTH / 2;
		Tra_RAnkle.y[k] = 0;
		Tra_RAnkle.z[k] = H_ANKLE;

		Tra_LAnkle.x[k] = CycTra[0][j];
		Tra_LAnkle.y[k] = CycTra[1][j];
		Tra_LAnkle.z[k] = CycTra[2][j] + H_ANKLE;
#ifdef STANDSTILL
		Tra_RAnkle.x[k] = ANKLE_WIDTH / 2;
		Tra_RAnkle.y[k] = 0;

		Tra_LAnkle.x[k] = -ANKLE_WIDTH / 2;
		Tra_LAnkle.y[k] = 0;

		Tra_RAnkle.z[k] = H_ANKLE;
		Tra_LAnkle.z[k] = H_ANKLE;
#endif

		j++;
	}
	// double phase 
	for (k = round((T_ready + T_step1*(1 - Pct_dou)) / CONTROL_T); k <= round((T_ready + T_step1) / CONTROL_T); k++)   // single period
	{
		Tra_RAnkle.x[k] = ANKLE_WIDTH / 2;
		Tra_RAnkle.y[k] = 0;
		Tra_RAnkle.z[k] = H_ANKLE;

		Tra_LAnkle.x[k] = -ANKLE_WIDTH / 2;
		Tra_LAnkle.y[k] = L_step;
		Tra_LAnkle.z[k] = H_ANKLE;
#ifdef STANDSTILL
		Tra_RAnkle.x[k] = ANKLE_WIDTH / 2;
		Tra_RAnkle.y[k] = 0;

		Tra_LAnkle.x[k] = -ANKLE_WIDTH / 2;
		Tra_LAnkle.y[k] = 0;

		Tra_RAnkle.z[k] = H_ANKLE;
		Tra_LAnkle.z[k] = H_ANKLE;
#endif
	}

	/* Second Step ...(N_step-2) Step */
	for (k_step = 1; k_step<N_step - 1; k_step++)
	{
		if (k_step % 2 != 0) // right leg swing  1,3,5...
		{
			// single phase
			j = 0;
			x1 = ANKLE_WIDTH / 2;
			y1 = (k_step - 1)*L_step;
			x2 = ANKLE_WIDTH / 2;
			y2 = (k_step + 1)*L_step;
			td = T_sin;
			dt = 0.004;
			Cycloid_Generate(x1, y1, x2, y2, td, dt);
			for (k = round((T_ready + T_step1 + (k_step - 1)*T_step) / CONTROL_T); k <= round((T_ready + T_step1 + (k_step - 1)*T_step + T_sin) / CONTROL_T); k++)   // single period
			{
				Tra_RAnkle.x[k] = CycTra[0][j];
				Tra_RAnkle.y[k] = CycTra[1][j];
				Tra_RAnkle.z[k] = CycTra[2][j] + H_ANKLE;

				Tra_LAnkle.x[k] = -ANKLE_WIDTH / 2;
				Tra_LAnkle.y[k] = k_step*L_step;
				Tra_LAnkle.z[k] = H_ANKLE;
#ifdef STANDSTILL
				Tra_RAnkle.x[k] = ANKLE_WIDTH / 2;
				Tra_RAnkle.y[k] = 0;

				Tra_LAnkle.x[k] = -ANKLE_WIDTH / 2;
				Tra_LAnkle.y[k] = 0;

				Tra_RAnkle.z[k] = H_ANKLE;
				Tra_LAnkle.z[k] = H_ANKLE;
#endif

				j++;
			}
			// double phase
			for (k = round((T_ready + T_step1 + (k_step - 1)*T_step + T_sin) / CONTROL_T); k <= round((T_ready + T_step1 + (k_step - 1)*T_step + T_step) / CONTROL_T); k++)   // single period
			{
				Tra_RAnkle.x[k] = ANKLE_WIDTH / 2;
				Tra_RAnkle.y[k] = (k_step + 1)*L_step;
				Tra_RAnkle.z[k] = H_ANKLE;

				Tra_LAnkle.x[k] = -ANKLE_WIDTH / 2;
				Tra_LAnkle.y[k] = k_step*L_step;
				Tra_LAnkle.z[k] = H_ANKLE;
#ifdef STANDSTILL
				Tra_RAnkle.x[k] = ANKLE_WIDTH / 2;
				Tra_RAnkle.y[k] = 0;

				Tra_LAnkle.x[k] = -ANKLE_WIDTH / 2;
				Tra_LAnkle.y[k] = 0;

				Tra_RAnkle.z[k] = H_ANKLE;
				Tra_LAnkle.z[k] = H_ANKLE;
#endif
			}
		}

		if (k_step % 2 == 0) // left leg swing  2,4,6...
		{
			// single phase
			j = 0;
			x1 = -ANKLE_WIDTH / 2;
			y1 = (k_step - 1)*L_step;
			x2 = -ANKLE_WIDTH / 2;
			y2 = (k_step + 1)*L_step;
			td = T_sin;
			dt = 0.004;
			Cycloid_Generate(x1, y1, x2, y2, td, dt);
			for (k = round((T_ready + T_step1 + (k_step - 1)*T_step) / CONTROL_T); k <= round((T_ready + T_step1 + (k_step - 1)*T_step + T_sin) / CONTROL_T); k++)   // single period
			{
				Tra_RAnkle.x[k] = ANKLE_WIDTH / 2;
				Tra_RAnkle.y[k] = k_step*L_step;
				Tra_RAnkle.z[k] = H_ANKLE;

				Tra_LAnkle.x[k] = CycTra[0][j];
				Tra_LAnkle.y[k] = CycTra[1][j];
				Tra_LAnkle.z[k] = CycTra[2][j] + H_ANKLE;
#ifdef STANDSTILL
				Tra_RAnkle.x[k] = ANKLE_WIDTH / 2;
				Tra_RAnkle.y[k] = 0;

				Tra_LAnkle.x[k] = -ANKLE_WIDTH / 2;
				Tra_LAnkle.y[k] = 0;

				Tra_RAnkle.z[k] = H_ANKLE;
				Tra_LAnkle.z[k] = H_ANKLE;
#endif

				j++;
			}
			// double phase
			for (k = round((T_ready + T_step1 + (k_step - 1)*T_step + T_sin) / CONTROL_T); k <= round((T_ready + T_step1 + (k_step - 1)*T_step + T_step) / CONTROL_T); k++)   // single period
			{
				Tra_RAnkle.x[k] = ANKLE_WIDTH / 2;
				Tra_RAnkle.y[k] = k_step*L_step;
				Tra_RAnkle.z[k] = H_ANKLE;

				Tra_LAnkle.x[k] = -ANKLE_WIDTH / 2;
				Tra_LAnkle.y[k] = (k_step + 1)*L_step;
				Tra_LAnkle.z[k] = H_ANKLE;
#ifdef STANDSTILL
				Tra_RAnkle.x[k] = ANKLE_WIDTH / 2;
				Tra_RAnkle.y[k] = 0;

				Tra_LAnkle.x[k] = -ANKLE_WIDTH / 2;
				Tra_LAnkle.y[k] = 0;

				Tra_RAnkle.z[k] = H_ANKLE;
				Tra_LAnkle.z[k] = H_ANKLE;
#endif

			}
		}
	}

	/* Last Step */
	if (N_step % 2 == 0)  // swing right leg
	{
		// single phase
		j = 0;
		x1 = ANKLE_WIDTH / 2;
		y1 = (N_step - 2)*L_step;
		x2 = ANKLE_WIDTH / 2;
		y2 = (N_step - 1)*L_step;
		td = T_stepn*(1 - Pct_dou);
		dt = 0.004;
		Cycloid_Generate(x1, y1, x2, y2, td, dt);
		for (k = round((T_ready + T_step1 + (N_step - 2)*T_step) / CONTROL_T); k <= round((T_ready + T_step1 + (N_step - 2)*T_step + T_stepn*(1 - Pct_dou)) / CONTROL_T); k++)   // single period
		{
			Tra_RAnkle.x[k] = CycTra[0][j];
			Tra_RAnkle.y[k] = CycTra[1][j];
			Tra_RAnkle.z[k] = CycTra[2][j] + H_ANKLE;

			Tra_LAnkle.x[k] = -ANKLE_WIDTH / 2;
			Tra_LAnkle.y[k] = (N_step - 1)*L_step;
			Tra_LAnkle.z[k] = H_ANKLE;
#ifdef STANDSTILL
			Tra_RAnkle.x[k] = ANKLE_WIDTH / 2;
			Tra_RAnkle.y[k] = 0;

			Tra_LAnkle.x[k] = -ANKLE_WIDTH / 2;
			Tra_LAnkle.y[k] = 0;

			Tra_RAnkle.z[k] = H_ANKLE;
			Tra_LAnkle.z[k] = H_ANKLE;
#endif

			j++;
		}
		// double phase
		for (k = round((T_ready + T_step1 + (N_step - 2)*T_step + T_stepn*(1 - Pct_dou)) / CONTROL_T); k <= round((T_ready + T_step1 + (N_step - 2)*T_step + T_stepn) / CONTROL_T); k++)   // single period
		{
			Tra_RAnkle.x[k] = ANKLE_WIDTH / 2;
			Tra_RAnkle.y[k] = (N_step - 1)*L_step;
			Tra_RAnkle.z[k] = H_ANKLE;

			Tra_LAnkle.x[k] = -ANKLE_WIDTH / 2;
			Tra_LAnkle.y[k] = (N_step - 1)*L_step;
			Tra_LAnkle.z[k] = H_ANKLE;
#ifdef STANDSTILL
			Tra_RAnkle.x[k] = ANKLE_WIDTH / 2;
			Tra_RAnkle.y[k] = 0;

			Tra_LAnkle.x[k] = -ANKLE_WIDTH / 2;
			Tra_LAnkle.y[k] = 0;

			Tra_RAnkle.z[k] = H_ANKLE;
			Tra_LAnkle.z[k] = H_ANKLE;
#endif
		}
	}
	if (N_step % 2 != 0)  // swing left leg
	{
		// single phase
		j = 0;
		x1 = -ANKLE_WIDTH / 2;
		y1 = (N_step - 2)*L_step;
		x2 = -ANKLE_WIDTH / 2;
		y2 = (N_step - 1)*L_step;
		td = T_stepn*(1 - Pct_dou);
		dt = 0.004;
		Cycloid_Generate(x1, y1, x2, y2, td, dt);
		for (k = round((T_ready + T_step1 + (N_step - 2)*T_step) / CONTROL_T); k <= round((T_ready + T_step1 + (N_step - 2)*T_step + T_stepn*(1 - Pct_dou)) / CONTROL_T); k++)   // single period
		{
			Tra_RAnkle.x[k] = ANKLE_WIDTH / 2;
			Tra_RAnkle.y[k] = (N_step - 1)*L_step;
			Tra_RAnkle.z[k] = H_ANKLE;

			Tra_LAnkle.x[k] = CycTra[0][j];
			Tra_LAnkle.y[k] = CycTra[1][j];
			Tra_LAnkle.z[k] = CycTra[2][j] + H_ANKLE;
#ifdef STANDSTILL
			Tra_RAnkle.x[k] = ANKLE_WIDTH / 2;
			Tra_RAnkle.y[k] = 0;

			Tra_LAnkle.x[k] = -ANKLE_WIDTH / 2;
			Tra_LAnkle.y[k] = 0;

			Tra_RAnkle.z[k] = H_ANKLE;
			Tra_LAnkle.z[k] = H_ANKLE;
#endif

			j++;
		}
		// double phase
		for (k = round((T_ready + T_step1 + (N_step - 2)*T_step + T_stepn*(1 - Pct_dou)) / CONTROL_T); k <= round((T_ready + T_step1 + (N_step - 2)*T_step + T_stepn) / CONTROL_T); k++)   // single period
		{
			Tra_RAnkle.x[k] = ANKLE_WIDTH / 2;
			Tra_RAnkle.y[k] = (N_step - 1)*L_step;
			Tra_RAnkle.z[k] = H_ANKLE;

			Tra_LAnkle.x[k] = -ANKLE_WIDTH / 2;
			Tra_LAnkle.y[k] = (N_step - 1)*L_step;
			Tra_LAnkle.z[k] = H_ANKLE;
#ifdef STANDSTILL
			Tra_RAnkle.x[k] = ANKLE_WIDTH / 2;
			Tra_RAnkle.y[k] = 0;

			Tra_LAnkle.x[k] = -ANKLE_WIDTH / 2;
			Tra_LAnkle.y[k] = 0;

			Tra_RAnkle.z[k] = H_ANKLE;
			Tra_LAnkle.z[k] = H_ANKLE;
#endif
		}
	}

	for (k = round((T_ready + T_step1 + (N_step - 2)*T_step + T_stepn) / CONTROL_T); k <= round(T_com / CONTROL_T); k++)
	{
		Tra_RAnkle.x[k] = ANKLE_WIDTH / 2;
		Tra_RAnkle.y[k] = (N_step - 1)*L_step;
		Tra_RAnkle.z[k] = H_ANKLE;

		Tra_LAnkle.x[k] = -ANKLE_WIDTH / 2;
		Tra_LAnkle.y[k] = (N_step - 1)*L_step;
		Tra_LAnkle.z[k] = H_ANKLE;
#ifdef STANDSTILL
		Tra_RAnkle.x[k] = ANKLE_WIDTH / 2;
		Tra_RAnkle.y[k] = 0;

		Tra_LAnkle.x[k] = -ANKLE_WIDTH / 2;
		Tra_LAnkle.y[k] = 0;

		Tra_RAnkle.z[k] = H_ANKLE;
		Tra_LAnkle.z[k] = H_ANKLE;
#endif
	}

	if (L_step != 0.0)
	{
		for (k = 0; k <= round(T_com / CONTROL_T); k++)
		{
			if ((k>round((T_ready + T_step1) / CONTROL_T)) && (k<round((T_ready + T_step1 + (N_step - 2)*T_step) / CONTROL_T)))
			{
				Tra_RAnkle.z[k] = H_ANKLE + fabs(Tra_RAnkle.z[k] - H_ANKLE)*(H_step*PAI / L_step / 2);
				Tra_LAnkle.z[k] = H_ANKLE + fabs(Tra_LAnkle.z[k] - H_ANKLE)*(H_step*PAI / L_step / 2);
#ifdef STANDSTILL
				Tra_RAnkle.z[k] = H_ANKLE;
				Tra_LAnkle.z[k] = H_ANKLE;
#endif
			}
			else
			{
				Tra_RAnkle.z[k] = H_ANKLE + fabs(Tra_RAnkle.z[k] - H_ANKLE)*(H_step*PAI / L_step);
				Tra_LAnkle.z[k] = H_ANKLE + fabs(Tra_LAnkle.z[k] - H_ANKLE)*(H_step*PAI / L_step);
#ifdef STANDSTILL
				Tra_RAnkle.z[k] = H_ANKLE;
				Tra_LAnkle.z[k] = H_ANKLE;
#endif	
			}
		}
		printf("\n Cycloid Ankle Z Modified!\n");
	}

	if (NULL == (fp = fopen("ankle_tra_cycloid.dat", "w")))
	{
		printf("ankle_tra_cycloid File can not open!\n");
	}
	for (k = 0; k<round(T_com / CONTROL_T); k++)     // COM_t
													 // fprintf(fp,"%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n",Tra_RAnkle.x[k],Tra_RAnkle.y[k],Tra_RAnkle.z[k],Tra_LAnkle.x[k],Tra_LAnkle.y[k],Tra_LAnkle.z[k]);						
		fclose(fp);

}

void Cycloid_Generate(double px1, double py1, double px2, double py2, double td, double dt)
{
	int n;
	double et;
	double t;
	
	double seta;
	double r;

	Position P_cyc, P_rt;
	double T_rot_seta[4][4];

	int i;
	
	n = round(td/dt);
	et = (2*PAI)/n;
	r = sqrt((py2-py1)*(py2-py1)+(px2-px1)*(px2-px1))/(2*PAI);

	seta = atan2(py2-py1, px2-px1);
	
	T_rot_seta[0][0] =  cos(seta); T_rot_seta[0][1] = -sin(seta); T_rot_seta[0][2] =  0.0; T_rot_seta[0][3] = px1;
	T_rot_seta[1][0] =  sin(seta); T_rot_seta[1][1] =  cos(seta); T_rot_seta[1][2] =  0.0; T_rot_seta[1][3] = py1;
	T_rot_seta[2][0] =        0.0; T_rot_seta[2][1] =        0.0; T_rot_seta[2][2] =  1.0; T_rot_seta[2][3] = 0.0;
	T_rot_seta[3][0] =        0.0; T_rot_seta[3][1] =        0.0; T_rot_seta[3][2] =  0.0; T_rot_seta[3][3] = 1.0;

	for(i=0;i<=n;i++)
	{
		t = i*et;
		P_cyc.px = r*(t-sin(t));
		P_cyc.py = 0*t;
		P_cyc.pz = r*(1-cos(t));

		P_rt.px = T_rot_seta[0][0]*P_cyc.px + T_rot_seta[0][1]*P_cyc.py + T_rot_seta[0][2]*P_cyc.pz + T_rot_seta[0][3]*1.0;
		P_rt.py = T_rot_seta[1][0]*P_cyc.px + T_rot_seta[1][1]*P_cyc.py + T_rot_seta[1][2]*P_cyc.pz + T_rot_seta[1][3]*1.0;
		P_rt.pz = T_rot_seta[2][0]*P_cyc.px + T_rot_seta[2][1]*P_cyc.py + T_rot_seta[2][2]*P_cyc.pz + T_rot_seta[2][3]*1.0;

		CycTra[0][i] = P_rt.px;
		CycTra[1][i] = P_rt.py;
		CycTra[2][i] = P_rt.pz;
	}
	
}

//dcc_rec
void XYCOM_Tra_PreConCal(double * xzmp, double * yzmp, int kprecon) // Pass the address of the array 
{
	int j;
	double x0, x1, x2, Px;
	double y0, y1, y2, Py;

	/* Calculate XCOM Trajectory */
	ucx = 0.0;
	for (j = 0; j<N_predict; j++)
	{
		ucx += Gp[j] * xzmp[kprecon + j];
	}

	if (kprecon == 0)
	{
		X[0] = 0.0;
		X[1] = 0.0;
		X[2] = 0.0;

		ex = 0.0;
		sigma_ex = 0.0;
		Px = C[0] * X[0] + C[1] * X[1] + C[2] * X[2];

		ux[kprecon] = -Gi*sigma_ex - Gx[0] * X[0] - Gx[1] * X[1] - Gx[2] * X[2] - ucx;

	}
	else
	{
		x0 = A[0] * X[0] + A[1] * X[1] + A[2] * X[2] + B[0] * ux[kprecon - 1];
		x1 = A[3] * X[0] + A[4] * X[1] + A[5] * X[2] + B[1] * ux[kprecon - 1];
		x2 = A[6] * X[0] + A[7] * X[1] + A[8] * X[2] + B[2] * ux[kprecon - 1];

		X[0] = x0;
		X[1] = x1;
		X[2] = x2;

		Px = C[0] * X[0] + C[1] * X[1] + C[2] * X[2];
		ex = Px - xzmp[kprecon];
		sigma_ex = sigma_ex + ex;
		ux[kprecon] = -Gi*sigma_ex - Gx[0] * X[0] - Gx[1] * X[1] - Gx[2] * X[2] - ucx;
	}

	Tra_ZMPCal.x[kprecon] = Px;
	Tra_COM.x[kprecon] = X[0];
	Tra_VCOM.x[kprecon] = X[1];
	Tra_ACOM.x[kprecon] = X[2];

	/* Calculate YCOM Trajectory */
	ucy = 0.0;
	for (j = 0; j<N_predict; j++)
	{
		ucy += Gp[j] * yzmp[kprecon + j];
	}

	if (kprecon == 0)
	{
		Y[0] = 0.0;
		Y[1] = 0.0;
		Y[2] = 0.0;

		ey = 0.0;
		sigma_ey = 0.0;
		Py = C[0] * Y[0] + C[1] * Y[1] + C[2] * Y[2];

		uy[kprecon] = -Gi*sigma_ey - Gx[0] * Y[0] - Gx[1] * Y[1] - Gx[2] * Y[2] - ucy;

	}
	else
	{
		y0 = A[0] * Y[0] + A[1] * Y[1] + A[2] * Y[2] + B[0] * uy[kprecon - 1];
		y1 = A[3] * Y[0] + A[4] * Y[1] + A[5] * Y[2] + B[1] * uy[kprecon - 1];
		y2 = A[6] * Y[0] + A[7] * Y[1] + A[8] * Y[2] + B[2] * uy[kprecon - 1];

		Y[0] = y0;
		Y[1] = y1;
		Y[2] = y2;

		Py = C[0] * Y[0] + C[1] * Y[1] + C[2] * Y[2];
		ey = Py - yzmp[kprecon];
		sigma_ey = sigma_ey + ey;
		uy[kprecon] = -Gi*sigma_ey - Gx[0] * Y[0] - Gx[1] * Y[1] - Gx[2] * Y[2] - ucy;
	}

	Tra_ZMPCal.y[kprecon] = Py;
	Tra_COM.y[kprecon] = Y[0];
	Tra_VCOM.y[kprecon] = Y[1];
	Tra_ACOM.y[kprecon] = Y[2];

#ifdef STANDSTILL
	Tra_COM.x[kprecon] = 0.0;
	Tra_COM.y[kprecon] = 0.0;
#endif
}

double e, sigma_e, P_online, uc, u;
double PLine[30000] = { 0 };
double e_y, YP, _uy;
double YPLine[30000] = { 0 };
int Nl;
//dcc_rec
void XYCOM_Tra_PreConCal_New(double zmpx[50000], double zmpy[50000], int k)			//Generate COM trajectories
{
	//int k;
	int j;
	int i = 0;

	Nl = N_predict;
	/* Generate XCOM trajectory */

	if (k == 0)
	{
		X[0] = 0;
		X[1] = 0;
		X[2] = 0;
		e = 0;
		sigma_e = 0;
		P_online = C[0] * X[0] + C[1] * X[1] + C[2] * X[2];
	}
	if (k>0)
	{
		for (j = 0; j<Nl; j++)
		{
			if (j == 0)
				uc = Gp[0] * zmpx[k + j];
			else

				uc = uc + Gp[j] * zmpx[k + j];

		}


		e = P_online - zmpx[k];
		sigma_e = sigma_e + e;
		u = -Gi*sigma_e - Gx[0] * X[0] - Gx[1] * X[1] - Gx[2] * X[2] - uc;
		P_online = C[0] * X[0] + C[1] * X[1] + C[2] * X[2];
		X[0] = A[0] * X[0] + A[1] * X[1] + A[2] * X[2] + B[0] * u;
		X[1] = A[3] * X[0] + A[4] * X[1] + A[5] * X[2] + B[1] * u;
		X[2] = A[6] * X[0] + A[7] * X[1] + A[8] * X[2] + B[2] * u;
		Tra_ZMPCal.x[k] = P_online;
		Tra_COM.x[k] = X[0];
		Tra_VCOM.x[k] = X[1];
		Tra_ACOM.x[k] = X[2];
		//printf("%12.16lf\n",Gp[0]);

	}


	/* Generate YCOM trajectory */

	if (k == 0)
	{
		Y[0] = 0;
		Y[1] = 0;
		Y[2] = 0;
		e_y = 0;
		sigma_ey = 0;
		YP = C[0] * Y[0] + C[1] * Y[1] + C[2] * Y[2];
	}
	if (k>0)
	{
		for (j = 0; j<Nl; j++)
		{
			if (j == 0)
				ucy = Gp[0] * zmpy[k + j];
			else
				ucy = ucy + Gp[j] * zmpy[k + j];
		}


		e_y = YP - zmpy[k];
		sigma_ey = sigma_ey + e_y;
		_uy = -Gi*sigma_ey - Gx[0] * Y[0] - Gx[1] * Y[1] - Gx[2] * Y[2] - ucy;
		YP = C[0] * Y[0] + C[1] * Y[1] + C[2] * Y[2];
		Y[0] = A[0] * Y[0] + A[1] * Y[1] + A[2] * Y[2] + B[0] * _uy;
		Y[1] = A[3] * Y[0] + A[4] * Y[1] + A[5] * Y[2] + B[1] * _uy;
		Y[2] = A[6] * Y[0] + A[7] * Y[1] + A[8] * Y[2] + B[2] * _uy;
		Tra_ZMPCal.y[k] = YP;
		Tra_COM.y[k] = Y[0] + y_bias;
		Tra_VCOM.y[k] = Y[1];
		Tra_ACOM.y[k] = Y[2];

	}
#ifdef STANDSTILL
	Tra_COM.x[k] = 0.0;
	Tra_COM.y[k] = 0.0;
#endif
	//printf("XCOM_online[%d]=%lf\n",k,XCOM_online[k]);

}

void XYCOM_Tra_SLQR(double * xzmp, double * yzmp, int kprecon)			
{
	double S;
	double uffx;
	double x0, x1, x2;
	
	double Sy;
	double uffy;
	double y0, y1, y2;
	
	int j=0;

	/* Generate XCOM trajectory */
	S = xzmp[kprecon+N_predict];
	for(j = 2; j <= N_predict; j++)
	{
		S = xzmp[kprecon+N_predict-j+1] + Tk_m * S;
	}
	uffx = xzmp[kprecon+1]/Tk_dt - Tk_a*(2+Tk_a*Tk_dt)*Tk_m*Tk_m*S;
	
	if(kprecon == 0)
	{
		X[0]=0.0;
		X[1]=0.0;
		X[2]=0.0;
		
		Tk_ux[kprecon] = -(Tk_Kd[0]*X[0] + Tk_Kd[1]*X[1] + Tk_Kd[2]*X[2]) + uffx;
	}	
	else
	{
		x0 = Tk_A[0]*X[0] + Tk_A[1]*X[1] + Tk_A[2]*X[2] + Tk_B[0]*Tk_ux[kprecon-1];
		x1 = Tk_A[3]*X[0] + Tk_A[4]*X[1] + Tk_A[5]*X[2] + Tk_B[1]*Tk_ux[kprecon-1];
		x2 = Tk_A[6]*X[0] + Tk_A[7]*X[1] + Tk_A[8]*X[2] + Tk_B[2]*Tk_ux[kprecon-1];
		X[0] = x0;
		X[1] = x1;
		X[2] = x2;

		Tk_ux[kprecon] = -(Tk_Kd[0]*X[0] + Tk_Kd[1]*X[1] + Tk_Kd[2]*X[2]) + uffx;
		//printf("%f, %f, %f,-- %f, %f, %f, %f\n", X[0], X[1], X[2], Tk_Kd[0], Tk_Kd[1], Tk_Kd[2], uffx);
	}
	Tra_ZMPCal.x[kprecon] = X[0];
	Tra_COM.x[kprecon]  = X[1];
	Tra_VCOM.x[kprecon] = X[2];

	if(kprecon==0)
	{
		Tra_ACOM.x[kprecon] = 0.0;		
	}
	else
	{
		Tra_ACOM.x[kprecon] = (Tra_VCOM.x[kprecon]-Tra_VCOM.x[kprecon-1])/Tk_dt;
	}


	/* Generate YCOM trajectory */
	Sy = yzmp[kprecon+N_predict];
	for(j = 2; j <= N_predict; j++)
	{
		Sy = yzmp[kprecon+N_predict-j+1] + Tk_m * Sy;
	}
	uffy = yzmp[kprecon+1]/Tk_dt - Tk_a*(2+Tk_a*Tk_dt)*Tk_m*Tk_m*Sy;
	
	if(kprecon == 0)
	{
		Y[0]=0.0;
		Y[1]=0.0;
		Y[2]=0.0;
		
		Tk_uy[kprecon] = -(Tk_Kd[0]*Y[0] + Tk_Kd[1]*Y[1] + Tk_Kd[2]*Y[2]) + uffy;
	}	
	else
	{
		y0 = Tk_A[0]*Y[0] + Tk_A[1]*Y[1] + Tk_A[2]*Y[2] + Tk_B[0]*Tk_uy[kprecon-1];
		y1 = Tk_A[3]*Y[0] + Tk_A[4]*Y[1] + Tk_A[5]*Y[2] + Tk_B[1]*Tk_uy[kprecon-1];
		y2 = Tk_A[6]*Y[0] + Tk_A[7]*Y[1] + Tk_A[8]*Y[2] + Tk_B[2]*Tk_uy[kprecon-1];

		Y[0] = y0;
		Y[1] = y1;
		Y[2] = y2;

		Tk_uy[kprecon] = -(Tk_Kd[0]*Y[0] + Tk_Kd[1]*Y[1] + Tk_Kd[2]*Y[2]) + uffy;
	}
	Tra_ZMPCal.y[kprecon] = Y[0];
	Tra_COM.y[kprecon]  = Y[1];
	Tra_VCOM.y[kprecon] = Y[2];

	if(kprecon==0)
	{
		Tra_ACOM.y[kprecon] = 0.0;		
	}
	else
	{
		Tra_ACOM.y[kprecon] = (Tra_VCOM.y[kprecon]-Tra_VCOM.y[kprecon-1])/Tk_dt;
	}
}



void LegJoint_Calculate( PreCon_Tra * rankle, PreCon_Tra * lankle, PreCon_Tra * com, int kprecon) // Pass the address of the struct
{
	double ql[7], qr[7];
	Inverse_Kinematics(rankle->x[kprecon] - com->x[kprecon], rankle->y[kprecon] - com->y[kprecon], rankle->z[kprecon] - com->z[kprecon], RLEG_SP, qr);
	Inverse_Kinematics(lankle->x[kprecon] - com->x[kprecon], lankle->y[kprecon] - com->y[kprecon], lankle->z[kprecon] - com->z[kprecon], LLEG_SP, ql);

	Full_DOF_IK(ql, qr, com->x[kprecon], com->y[kprecon], com->z[kprecon], roll_body, pitch_body, rankle->x[kprecon], rankle->y[kprecon], rankle->z[kprecon], lankle->x[kprecon], lankle->y[kprecon], lankle->z[kprecon], pitch_footl, pitch_footr);
	for(int i = 1; i <= 6; i++) PreCon_LegJoint.ql[i] = ql[i], PreCon_LegJoint.qr[i] = qr[i];
	//printf("%.10f, %.10f, %.10f\n", rankle->x[kprecon] - com->x[kprecon], rankle->y[kprecon] - com->y[kprecon], rankle->z[kprecon] - com->z[kprecon]);
	//Inverse_Kinematics2( rankle->x[kprecon] - com->x[kprecon], rankle->y[kprecon] - com->y[kprecon], rankle->z[kprecon] - com->z[kprecon], RLEG_SP, PreCon_LegJoint.qr );
	//printf("%.10f, %.10f, %.10f, %.10f, %.10f, %.10f\n", PreCon_LegJoint.qr[1], PreCon_LegJoint.qr[2], PreCon_LegJoint.qr[3], PreCon_LegJoint.qr[4], PreCon_LegJoint.qr[5], PreCon_LegJoint.qr[6]);
	
	//Inverse_Kinematics2( lankle->x[kprecon] - com->x[kprecon], lankle->y[kprecon] - com->y[kprecon], lankle->z[kprecon] - com->z[kprecon], LLEG_SP, PreCon_LegJoint.ql );
}

void Inverse_Kinematics(double x_online, double y_online, double z_online, enum Support_Leg leg_No, double q[])
{
	double WaistLength = ANKLE_WIDTH;
	double ThighLength = THIGH;
	double CrusLength = THIGH;
	double r[3];
	double RT[3][3];
	double C;
	double PI = 3.1415926;
	double q5_temp = 0;
	if (z_online <= -0.7) z_online = -0.7; // must <= H_reset-H_ankle
	if (z_online >= -0.20) z_online = -0.20;

	if (leg_No == 1)
	{
		//if (x_online<0.085) x_online=0.085;
		r[0] = WaistLength / 2 * cos(roll_body) - x_online;
		r[1] = -y_online;
		r[2] = -(z_online + sin(roll_body)*WaistLength / 2);
	}
	if (leg_No == 2)
	{
		//if (x_online>-0.085) x_online=-0.085;
		r[0] = -WaistLength / 2 * cos(roll_body) - x_online;
		r[1] = -y_online;
		r[2] = -(z_online - sin(roll_body)*WaistLength / 2);
	}
	C = sqrt(pow(r[0], 2.0) + pow(r[1], 2.0) + pow(r[2], 2.0));
	q[4] = acos((pow(ThighLength, 2.0) + pow(CrusLength, 2.0) - pow(C, 2.0)) / (2 * ThighLength*CrusLength)) - PI;
	q[6] = -atan2(r[0], r[2]);
	q5_temp = asin(ThighLength*sin(PI + q[4]) / C);
	q[5] = atan2(r[1], sqrt(pow(r[0], 2.0) + pow(r[2], 2.0))) + q5_temp;
	RT[0][0] = cos(q[6]);
	RT[0][1] = cos(q[4])*sin(q[5])*sin(q[6]) + cos(q[5])*sin(q[4])*sin(q[6]);
	RT[0][2] = sin(q[4])*sin(q[5])*sin(q[6]) - cos(q[4])*cos(q[5])*sin(q[6]);
	RT[1][0] = 0;
	RT[1][1] = cos(q[4])*cos(q[5]) - sin(q[4])*sin(q[5]);
	RT[1][2] = cos(q[4])*sin(q[5]) + cos(q[5])*sin(q[4]);
	RT[2][0] = sin(q[6]);
	RT[2][1] = -cos(q[4])*cos(q[6])*sin(q[5]) - cos(q[5])*cos(q[6])*sin(q[4]);
	RT[2][2] = cos(q[4])*cos(q[5])*cos(q[6]) - cos(q[6])*sin(q[4])*sin(q[5]);
	q[3] = atan2(RT[2][1], RT[2][2]) - pitch_body;
	q[1] = atan2(RT[1][0], RT[0][0]);
	q[2] = atan2(-RT[2][0], RT[1][0] * sin(q[1]) + RT[0][0] * cos(q[1])) - roll_body;
}


void Full_DOF_IK(double* ql, double* qr, double xcom, double ycom, double zcom, double thx_com, double thy_com, double xr, double yr, double zr, double xl, double yl, double zl, double thxl, double thxr)
{
	int i;
	BPosture COM, Hip, Ankle;
	TStruct temp;

	COM.Posture.p[0] = thx_com;
	COM.Posture.p[1] = thy_com;
	COM.Posture.p[2] = 0.0;
	COM.Posture.p[3] = ycom * 1000.0;
	COM.Posture.p[4] = -xcom * 1000.0;
	COM.Posture.p[5] = zcom * 1000.0;
	COM.T = get_t(&COM.Posture);

	// 右腿
	Ankle.Posture.p[0] = 0.0;
	Ankle.Posture.p[1] = -thxr;
	Ankle.Posture.p[2] = 0.0;
	Ankle.Posture.p[3] = yr * 1000.0;
	Ankle.Posture.p[4] = -xr * 1000.0;
	Ankle.Posture.p[5] = zr * 1000.0;
	Ankle.T = get_t(&Ankle.Posture);

	Hip.T = mt_mul(COM.T, get_move(0, -BASE_HIP_Y, 0));
	temp = mt_mul(t_inv(&Hip.T), Ankle.T);
	for (i = 3; i <= 5; i++) qr[i] = -qr[i];
	ik_leg(temp.t, &(qr[1]), &(qr[1]));
	for (i = 3; i <= 5; i++) qr[i] = -qr[i];

	// 左腿
	Ankle.Posture.p[0] = 0.0;
	Ankle.Posture.p[1] = -thxl;
	Ankle.Posture.p[2] = 0.0;
	Ankle.Posture.p[3] = yl * 1000.0;
	Ankle.Posture.p[4] = -xl * 1000.0;
	Ankle.Posture.p[5] = zl * 1000.0;
	Ankle.T = get_t(&Ankle.Posture);

	Hip.T = mt_mul(COM.T, get_move(0, BASE_HIP_Y, 0));
	temp = mt_mul(t_inv(&Hip.T), Ankle.T);
	for (i = 3; i <= 5; i++) ql[i] = -ql[i];
	// for (i = 2; i <= 6; i++) ql[i] = -ql[i];
	ik_leg(temp.t, &(ql[1]), &(ql[1]));
	for (i = 3; i <= 5; i++) ql[i] = -ql[i];
	// for (i = 2; i <= 6; i++) ql[i] = -ql[i];
}


void TSpline_S_V_A(double p0, double v0, double a0, double t0, double p1, double t1, double p2, double v2, double a2, double t2, double step)
{

	double P[10];
	//double S[3][500];		
	int k = 0;
	int i = 0;

	P[0] = (a2*(pow(t0, 3.0)*pow(t1, 2.0) - t2*pow(t0, 3.0)*t1)) / (6 * (pow(t0, 3.0) - 2 * pow(t0, 2.0)*t1 - t2*pow(t0, 2.0) + t0*pow(t1, 2.0) + 2 * t2*t0*t1 - t2*pow(t1, 2.0))) - (v0*(2 * pow(t0, 3.0)*pow(t1, 2.0) + t2*pow(t0, 3.0)*t1 - pow(t0, 2.0)*pow(t1, 3.0) - 3 * t2*pow(t0, 2.0)*pow(t1, 2.0) + t2*t0*pow(t1, 3.0))) / (pow(t0, 4.0) - 3 * pow(t0, 3.0)*t1 - t2*pow(t0, 3.0) + 3 * pow(t0, 2.0)*pow(t1, 2.0) + 3 * t2*pow(t0, 2.0)*t1 - t0*pow(t1, 3.0) - 3 * t2*t0*pow(t1, 2.0) + t2*pow(t1, 3.0)) - (p0*(4 * pow(t0, 3.0)*pow(t1, 2.0) + 2 * t2*pow(t0, 3.0)*t1 - 4 * pow(t0, 2.0)*pow(t1, 3.0) - 6 * t2*pow(t0, 2.0)*pow(t1, 2.0) + t0*pow(t1, 4.0) + 4 * t2*t0*pow(t1, 3.0) - t2*pow(t1, 4.0))) / (-pow(t0, 5.0) + 4 * pow(t0, 4.0)*t1 + t2*pow(t0, 4.0) - 6 * pow(t0, 3.0)*pow(t1, 2.0) - 4 * t2*pow(t0, 3.0)*t1 + 4 * pow(t0, 2.0)*pow(t1, 3.0) + 6 * t2*pow(t0, 2.0)*pow(t1, 2.0) - t0*pow(t1, 4.0) - 4 * t2*t0*pow(t1, 3.0) + t2*pow(t1, 4.0)) + (a0*(2 * pow(t0, 3.0)*pow(t1, 2.0) + t2*pow(t0, 3.0)*t1 - 3 * t2*pow(t0, 2.0)*pow(t1, 2.0))) / (6 * (pow(t0, 3.0) - 2 * pow(t0, 2.0)*t1 - t2*pow(t0, 2.0) + t0*pow(t1, 2.0) + 2 * t2*t0*t1 - t2*pow(t1, 2.0))) - (p1*(pow(t0, 5.0) - 4 * pow(t0, 4.0)*t1 - t2*pow(t0, 4.0) + 2 * pow(t0, 3.0)*pow(t1, 2.0) + 2 * t2*pow(t0, 3.0)*t1)) / (-pow(t0, 5.0) + 4 * pow(t0, 4.0)*t1 + t2*pow(t0, 4.0) - 6 * pow(t0, 3.0)*pow(t1, 2.0) - 4 * t2*pow(t0, 3.0)*t1 + 4 * pow(t0, 2.0)*pow(t1, 3.0) + 6 * t2*pow(t0, 2.0)*pow(t1, 2.0) - t0*pow(t1, 4.0) - 4 * t2*t0*pow(t1, 3.0) + t2*pow(t1, 4.0)) + (pow(t0, 3.0)*t1*v2) / (pow(t0, 3.0) - 2 * pow(t0, 2.0)*t1 - t2*pow(t0, 2.0) + t0*pow(t1, 2.0) + 2 * t2*t0*t1 - t2*pow(t1, 2.0)) - (2 * p1*pow(t0, 3.0)*t1) / (pow(t0, 3.0)*t1 - pow(t0, 3.0)*t2 - 2 * pow(t0, 2.0)*pow(t1, 2.0) + pow(t0, 2.0)*t1*t2 + pow(t0, 2.0)*pow(t2, 2.0) + t0*pow(t1, 3.0) + t0*pow(t1, 2.0)*t2 - 2 * t0*t1*pow(t2, 2.0) - pow(t1, 3.0)*t2 + pow(t1, 2.0)*pow(t2, 2.0)) + (2 * p2*pow(t0, 3.0)*t1) / (pow(t0, 3.0)*t1 - pow(t0, 3.0)*t2 - 2 * pow(t0, 2.0)*pow(t1, 2.0) + pow(t0, 2.0)*t1*t2 + pow(t0, 2.0)*pow(t2, 2.0) + t0*pow(t1, 3.0) + t0*pow(t1, 2.0)*t2 - 2 * t0*t1*pow(t2, 2.0) - pow(t1, 3.0)*t2 + pow(t1, 2.0)*pow(t2, 2.0));
	P[1] = (2 * p1*(pow(t0, 3.0) + 3 * t1*pow(t0, 2.0))) / (pow(t0, 3.0)*t1 - pow(t0, 3.0)*t2 - 2 * pow(t0, 2.0)*pow(t1, 2.0) + pow(t0, 2.0)*t1*t2 + pow(t0, 2.0)*pow(t2, 2.0) + t0*pow(t1, 3.0) + t0*pow(t1, 2.0)*t2 - 2 * t0*t1*pow(t2, 2.0) - pow(t1, 3.0)*t2 + pow(t1, 2.0)*pow(t2, 2.0)) - (2 * p2*(pow(t0, 3.0) + 3 * t1*pow(t0, 2.0))) / (pow(t0, 3.0)*t1 - pow(t0, 3.0)*t2 - 2 * pow(t0, 2.0)*pow(t1, 2.0) + pow(t0, 2.0)*t1*t2 + pow(t0, 2.0)*pow(t2, 2.0) + t0*pow(t1, 3.0) + t0*pow(t1, 2.0)*t2 - 2 * t0*t1*pow(t2, 2.0) - pow(t1, 3.0)*t2 + pow(t1, 2.0)*pow(t2, 2.0)) - (v0*(-5 * pow(t0, 3.0)*t1 - t2*pow(t0, 3.0) + 3 * t2*pow(t0, 2.0)*t1 + t0*pow(t1, 3.0) + 3 * t2*t0*pow(t1, 2.0) - t2*pow(t1, 3.0))) / (pow(t0, 4.0) - 3 * pow(t0, 3.0)*t1 - t2*pow(t0, 3.0) + 3 * pow(t0, 2.0)*pow(t1, 2.0) + 3 * t2*pow(t0, 2.0)*t1 - t0*pow(t1, 3.0) - 3 * t2*t0*pow(t1, 2.0) + t2*pow(t1, 3.0)) - (a2*(pow(t0, 3.0)*t1 - t2*pow(t0, 3.0) + 3 * pow(t0, 2.0)*pow(t1, 2.0) - 3 * t2*pow(t0, 2.0)*t1)) / (6 * (pow(t0, 3.0) - 2 * pow(t0, 2.0)*t1 - t2*pow(t0, 2.0) + t0*pow(t1, 2.0) + 2 * t2*t0*t1 - t2*pow(t1, 2.0))) - (a0*(5 * pow(t0, 3.0)*t1 + t2*pow(t0, 3.0) + 3 * pow(t0, 2.0)*pow(t1, 2.0) - 3 * t2*pow(t0, 2.0)*t1 - 6 * t2*t0*pow(t1, 2.0))) / (6 * (pow(t0, 3.0) - 2 * pow(t0, 2.0)*t1 - t2*pow(t0, 2.0) + t0*pow(t1, 2.0) + 2 * t2*t0*t1 - t2*pow(t1, 2.0))) - (v2*(pow(t0, 3.0) + 3 * t1*pow(t0, 2.0))) / (pow(t0, 3.0) - 2 * pow(t0, 2.0)*t1 - t2*pow(t0, 2.0) + t0*pow(t1, 2.0) + 2 * t2*t0*t1 - t2*pow(t1, 2.0)) - (2 * p0*(-5 * pow(t0, 3.0)*t1 - t2*pow(t0, 3.0) + 3 * pow(t0, 2.0)*pow(t1, 2.0) + 3 * t2*pow(t0, 2.0)*t1)) / (-pow(t0, 5.0) + 4 * pow(t0, 4.0)*t1 + t2*pow(t0, 4.0) - 6 * pow(t0, 3.0)*pow(t1, 2.0) - 4 * t2*pow(t0, 3.0)*t1 + 4 * pow(t0, 2.0)*pow(t1, 3.0) + 6 * t2*pow(t0, 2.0)*pow(t1, 2.0) - t0*pow(t1, 4.0) - 4 * t2*t0*pow(t1, 3.0) + t2*pow(t1, 4.0)) + (2 * p1*(-5 * pow(t0, 3.0)*t1 - t2*pow(t0, 3.0) + 3 * pow(t0, 2.0)*pow(t1, 2.0) + 3 * t2*pow(t0, 2.0)*t1)) / (-pow(t0, 5.0) + 4 * pow(t0, 4.0)*t1 + t2*pow(t0, 4.0) - 6 * pow(t0, 3.0)*pow(t1, 2.0) - 4 * t2*pow(t0, 3.0)*t1 + 4 * pow(t0, 2.0)*pow(t1, 3.0) + 6 * t2*pow(t0, 2.0)*pow(t1, 2.0) - t0*pow(t1, 4.0) - 4 * t2*t0*pow(t1, 3.0) + t2*pow(t1, 4.0));
	P[2] = (6 * p2*(pow(t0, 2.0) + t1*t0)) / (pow(t0, 3.0)*t1 - pow(t0, 3.0)*t2 - 2 * pow(t0, 2.0)*pow(t1, 2.0) + pow(t0, 2.0)*t1*t2 + pow(t0, 2.0)*pow(t2, 2.0) + t0*pow(t1, 3.0) + t0*pow(t1, 2.0)*t2 - 2 * t0*t1*pow(t2, 2.0) - pow(t1, 3.0)*t2 + pow(t1, 2.0)*pow(t2, 2.0)) - (6 * p1*(pow(t0, 2.0) + t1*t0)) / (pow(t0, 3.0)*t1 - pow(t0, 3.0)*t2 - 2 * pow(t0, 2.0)*pow(t1, 2.0) + pow(t0, 2.0)*t1*t2 + pow(t0, 2.0)*pow(t2, 2.0) + t0*pow(t1, 3.0) + t0*pow(t1, 2.0)*t2 - 2 * t0*t1*pow(t2, 2.0) - pow(t1, 3.0)*t2 + pow(t1, 2.0)*pow(t2, 2.0)) + (3 * v2*(pow(t0, 2.0) + t1*t0)) / (pow(t0, 3.0) - 2 * pow(t0, 2.0)*t1 - t2*pow(t0, 2.0) + t0*pow(t1, 2.0) + 2 * t2*t0*t1 - t2*pow(t1, 2.0)) + (6 * p0*(-pow(t0, 3.0) - pow(t0, 2.0)*t1 + t0*pow(t1, 2.0) + t2*t0*t1)) / (-pow(t0, 5.0) + 4 * pow(t0, 4.0)*t1 + t2*pow(t0, 4.0) - 6 * pow(t0, 3.0)*pow(t1, 2.0) - 4 * t2*pow(t0, 3.0)*t1 + 4 * pow(t0, 2.0)*pow(t1, 3.0) + 6 * t2*pow(t0, 2.0)*pow(t1, 2.0) - t0*pow(t1, 4.0) - 4 * t2*t0*pow(t1, 3.0) + t2*pow(t1, 4.0)) - (6 * p1*(-pow(t0, 3.0) - pow(t0, 2.0)*t1 + t0*pow(t1, 2.0) + t2*t0*t1)) / (-pow(t0, 5.0) + 4 * pow(t0, 4.0)*t1 + t2*pow(t0, 4.0) - 6 * pow(t0, 3.0)*pow(t1, 2.0) - 4 * t2*pow(t0, 3.0)*t1 + 4 * pow(t0, 2.0)*pow(t1, 3.0) + 6 * t2*pow(t0, 2.0)*pow(t1, 2.0) - t0*pow(t1, 4.0) - 4 * t2*t0*pow(t1, 3.0) + t2*pow(t1, 4.0)) + (a2*(pow(t0, 2.0)*t1 - t2*pow(t0, 2.0) + t0*pow(t1, 2.0) - t2*t0*t1)) / (2 * (pow(t0, 3.0) - 2 * pow(t0, 2.0)*t1 - t2*pow(t0, 2.0) + t0*pow(t1, 2.0) + 2 * t2*t0*t1 - t2*pow(t1, 2.0))) + (3 * v0*(-pow(t0, 3.0) - 2 * pow(t0, 2.0)*t1 + t0*pow(t1, 2.0) + 2 * t2*t0*t1)) / (pow(t0, 4.0) - 3 * pow(t0, 3.0)*t1 - t2*pow(t0, 3.0) + 3 * pow(t0, 2.0)*pow(t1, 2.0) + 3 * t2*pow(t0, 2.0)*t1 - t0*pow(t1, 3.0) - 3 * t2*t0*pow(t1, 2.0) + t2*pow(t1, 3.0)) + (a0*(pow(t0, 3.0) + 3 * pow(t0, 2.0)*t1 - 3 * t2*t0*t1 - t2*pow(t1, 2.0))) / (2 * (pow(t0, 3.0) - 2 * pow(t0, 2.0)*t1 - t2*pow(t0, 2.0) + t0*pow(t1, 2.0) + 2 * t2*t0*t1 - t2*pow(t1, 2.0)));
	P[3] = (2 * p1*(3 * t0 + t1)) / (pow(t0, 3.0)*t1 - pow(t0, 3.0)*t2 - 2 * pow(t0, 2.0)*pow(t1, 2.0) + pow(t0, 2.0)*t1*t2 + pow(t0, 2.0)*pow(t2, 2.0) + t0*pow(t1, 3.0) + t0*pow(t1, 2.0)*t2 - 2 * t0*t1*pow(t2, 2.0) - pow(t1, 3.0)*t2 + pow(t1, 2.0)*pow(t2, 2.0)) - (v0*(-5 * pow(t0, 2.0) + 2 * t2*t0 + pow(t1, 2.0) + 2 * t2*t1)) / (pow(t0, 4.0) - 3 * pow(t0, 3.0)*t1 - t2*pow(t0, 3.0) + 3 * pow(t0, 2.0)*pow(t1, 2.0) + 3 * t2*pow(t0, 2.0)*t1 - t0*pow(t1, 3.0) - 3 * t2*t0*pow(t1, 2.0) + t2*pow(t1, 3.0)) - (2 * p2*(3 * t0 + t1)) / (pow(t0, 3.0)*t1 - pow(t0, 3.0)*t2 - 2 * pow(t0, 2.0)*pow(t1, 2.0) + pow(t0, 2.0)*t1*t2 + pow(t0, 2.0)*pow(t2, 2.0) + t0*pow(t1, 3.0) + t0*pow(t1, 2.0)*t2 - 2 * t0*t1*pow(t2, 2.0) - pow(t1, 3.0)*t2 + pow(t1, 2.0)*pow(t2, 2.0)) - (2 * p0*(-4 * pow(t0, 2.0) + t0*t1 + t2*t0 + pow(t1, 2.0) + t2*t1)) / (-pow(t0, 5.0) + 4 * pow(t0, 4.0)*t1 + t2*pow(t0, 4.0) - 6 * pow(t0, 3.0)*pow(t1, 2.0) - 4 * t2*pow(t0, 3.0)*t1 + 4 * pow(t0, 2.0)*pow(t1, 3.0) + 6 * t2*pow(t0, 2.0)*pow(t1, 2.0) - t0*pow(t1, 4.0) - 4 * t2*t0*pow(t1, 3.0) + t2*pow(t1, 4.0)) + (2 * p1*(-4 * pow(t0, 2.0) + t0*t1 + t2*t0 + pow(t1, 2.0) + t2*t1)) / (-pow(t0, 5.0) + 4 * pow(t0, 4.0)*t1 + t2*pow(t0, 4.0) - 6 * pow(t0, 3.0)*pow(t1, 2.0) - 4 * t2*pow(t0, 3.0)*t1 + 4 * pow(t0, 2.0)*pow(t1, 3.0) + 6 * t2*pow(t0, 2.0)*pow(t1, 2.0) - t0*pow(t1, 4.0) - 4 * t2*t0*pow(t1, 3.0) + t2*pow(t1, 4.0)) - (v2*(3 * t0 + t1)) / (pow(t0, 3.0) - 2 * pow(t0, 2.0)*t1 - t2*pow(t0, 2.0) + t0*pow(t1, 2.0) + 2 * t2*t0*t1 - t2*pow(t1, 2.0)) - (a2*(3 * t0*t1 - 3 * t0*t2 - t1*t2 + pow(t1, 2.0))) / (6 * (pow(t0, 3.0) - 2 * pow(t0, 2.0)*t1 - t2*pow(t0, 2.0) + t0*pow(t1, 2.0) + 2 * t2*t0*t1 - t2*pow(t1, 2.0))) + (a0*(-6 * pow(t0, 2.0) - 3 * t0*t1 + 3 * t2*t0 + pow(t1, 2.0) + 5 * t2*t1)) / (6 * (pow(t0, 3.0) - 2 * pow(t0, 2.0)*t1 - t2*pow(t0, 2.0) + t0*pow(t1, 2.0) + 2 * t2*t0*t1 - t2*pow(t1, 2.0)));
	P[4] = (2 * p2) / (pow(t0, 3.0)*t1 - pow(t0, 3.0)*t2 - 2 * pow(t0, 2.0)*pow(t1, 2.0) + pow(t0, 2.0)*t1*t2 + pow(t0, 2.0)*pow(t2, 2.0) + t0*pow(t1, 3.0) + t0*pow(t1, 2.0)*t2 - 2 * t0*t1*pow(t2, 2.0) - pow(t1, 3.0)*t2 + pow(t1, 2.0)*pow(t2, 2.0)) - (2 * p1) / (pow(t0, 3.0)*t1 - pow(t0, 3.0)*t2 - 2 * pow(t0, 2.0)*pow(t1, 2.0) + pow(t0, 2.0)*t1*t2 + pow(t0, 2.0)*pow(t2, 2.0) + t0*pow(t1, 3.0) + t0*pow(t1, 2.0)*t2 - 2 * t0*t1*pow(t2, 2.0) - pow(t1, 3.0)*t2 + pow(t1, 2.0)*pow(t2, 2.0)) + v2 / (pow(t0, 3.0) - 2 * pow(t0, 2.0)*t1 - t2*pow(t0, 2.0) + t0*pow(t1, 2.0) + 2 * t2*t0*t1 - t2*pow(t1, 2.0)) - (a0*(t1 - 3 * t0 + 2 * t2)) / (6 * (pow(t0, 3.0) - 2 * pow(t0, 2.0)*t1 - t2*pow(t0, 2.0) + t0*pow(t1, 2.0) + 2 * t2*t0*t1 - t2*pow(t1, 2.0))) + (a2*(t1 - t2)) / (6 * (pow(t0, 3.0) - 2 * pow(t0, 2.0)*t1 - t2*pow(t0, 2.0) + t0*pow(t1, 2.0) + 2 * t2*t0*t1 - t2*pow(t1, 2.0))) + (v0*(t1 - 2 * t0 + t2)) / (pow(t0, 4.0) - 3 * pow(t0, 3.0)*t1 - t2*pow(t0, 3.0) + 3 * pow(t0, 2.0)*pow(t1, 2.0) + 3 * t2*pow(t0, 2.0)*t1 - t0*pow(t1, 3.0) - 3 * t2*t0*pow(t1, 2.0) + t2*pow(t1, 3.0)) + (p0*(2 * t1 - 3 * t0 + t2)) / (-pow(t0, 5.0) + 4 * pow(t0, 4.0)*t1 + t2*pow(t0, 4.0) - 6 * pow(t0, 3.0)*pow(t1, 2.0) - 4 * t2*pow(t0, 3.0)*t1 + 4 * pow(t0, 2.0)*pow(t1, 3.0) + 6 * t2*pow(t0, 2.0)*pow(t1, 2.0) - t0*pow(t1, 4.0) - 4 * t2*t0*pow(t1, 3.0) + t2*pow(t1, 4.0)) - (p1*(2 * t1 - 3 * t0 + t2)) / (-pow(t0, 5.0) + 4 * pow(t0, 4.0)*t1 + t2*pow(t0, 4.0) - 6 * pow(t0, 3.0)*pow(t1, 2.0) - 4 * t2*pow(t0, 3.0)*t1 + 4 * pow(t0, 2.0)*pow(t1, 3.0) + 6 * t2*pow(t0, 2.0)*pow(t1, 2.0) - t0*pow(t1, 4.0) - 4 * t2*t0*pow(t1, 3.0) + t2*pow(t1, 4.0));
	P[5] = (p2*(pow(t1, 4.0)*t2 - t0*pow(t1, 4.0) - 4 * pow(t1, 3.0)*pow(t2, 2.0) + 4 * t0*pow(t1, 3.0)*t2 + 4 * pow(t1, 2.0)*pow(t2, 3.0) - 6 * t0*pow(t1, 2.0)*pow(t2, 2.0) + 2 * t0*t1*pow(t2, 3.0))) / (pow(t1, 4.0)*t2 - t0*pow(t1, 4.0) - 4 * pow(t1, 3.0)*pow(t2, 2.0) + 4 * t0*pow(t1, 3.0)*t2 + 6 * pow(t1, 2.0)*pow(t2, 3.0) - 6 * t0*pow(t1, 2.0)*pow(t2, 2.0) - 4 * t1*pow(t2, 4.0) + 4 * t0*t1*pow(t2, 3.0) + pow(t2, 5.0) - t0*pow(t2, 4.0)) - (a2*(2 * pow(t1, 2.0)*pow(t2, 3.0) - 3 * t0*pow(t1, 2.0)*pow(t2, 2.0) + t0*t1*pow(t2, 3.0))) / (6 * (-pow(t1, 2.0)*t2 + t0*pow(t1, 2.0) + 2 * t1*pow(t2, 2.0) - 2 * t0*t1*t2 - pow(t2, 3.0) + t0*pow(t2, 2.0))) - (v2*(-pow(t1, 3.0)*pow(t2, 2.0) + t0*pow(t1, 3.0)*t2 + 2 * pow(t1, 2.0)*pow(t2, 3.0) - 3 * t0*pow(t1, 2.0)*pow(t2, 2.0) + t0*t1*pow(t2, 3.0))) / (-pow(t1, 3.0)*t2 + t0*pow(t1, 3.0) + 3 * pow(t1, 2.0)*pow(t2, 2.0) - 3 * t0*pow(t1, 2.0)*t2 - 3 * t1*pow(t2, 3.0) + 3 * t0*t1*pow(t2, 2.0) + pow(t2, 4.0) - t0*pow(t2, 3.0)) + (p1*(2 * pow(t1, 2.0)*pow(t2, 3.0) - 4 * t1*pow(t2, 4.0) + 2 * t0*t1*pow(t2, 3.0) + pow(t2, 5.0) - t0*pow(t2, 4.0))) / (pow(t1, 4.0)*t2 - t0*pow(t1, 4.0) - 4 * pow(t1, 3.0)*pow(t2, 2.0) + 4 * t0*pow(t1, 3.0)*t2 + 6 * pow(t1, 2.0)*pow(t2, 3.0) - 6 * t0*pow(t1, 2.0)*pow(t2, 2.0) - 4 * t1*pow(t2, 4.0) + 4 * t0*t1*pow(t2, 3.0) + pow(t2, 5.0) - t0*pow(t2, 4.0)) - (a0*(pow(t1, 2.0)*pow(t2, 3.0) - t0*t1*pow(t2, 3.0))) / (6 * (-pow(t1, 2.0)*t2 + t0*pow(t1, 2.0) + 2 * t1*pow(t2, 2.0) - 2 * t0*t1*t2 - pow(t2, 3.0) + t0*pow(t2, 2.0))) - (t1*pow(t2, 3.0)*v0) / (-pow(t1, 2.0)*t2 + t0*pow(t1, 2.0) + 2 * t1*pow(t2, 2.0) - 2 * t0*t1*t2 - pow(t2, 3.0) + t0*pow(t2, 2.0)) + (2 * p0*t1*pow(t2, 3.0)) / (pow(t0, 2.0)*pow(t1, 2.0) - 2 * pow(t0, 2.0)*t1*t2 + pow(t0, 2.0)*pow(t2, 2.0) - t0*pow(t1, 3.0) + t0*pow(t1, 2.0)*t2 + t0*t1*pow(t2, 2.0) - t0*pow(t2, 3.0) + pow(t1, 3.0)*t2 - 2 * pow(t1, 2.0)*pow(t2, 2.0) + t1*pow(t2, 3.0)) - (2 * p1*t1*pow(t2, 3.0)) / (pow(t0, 2.0)*pow(t1, 2.0) - 2 * pow(t0, 2.0)*t1*t2 + pow(t0, 2.0)*pow(t2, 2.0) - t0*pow(t1, 3.0) + t0*pow(t1, 2.0)*t2 + t0*t1*pow(t2, 2.0) - t0*pow(t2, 3.0) + pow(t1, 3.0)*t2 - 2 * pow(t1, 2.0)*pow(t2, 2.0) + t1*pow(t2, 3.0));
	P[6] = (2 * p1*(pow(t2, 3.0) + 3 * t1*pow(t2, 2.0))) / (pow(t0, 2.0)*pow(t1, 2.0) - 2 * pow(t0, 2.0)*t1*t2 + pow(t0, 2.0)*pow(t2, 2.0) - t0*pow(t1, 3.0) + t0*pow(t1, 2.0)*t2 + t0*t1*pow(t2, 2.0) - t0*pow(t2, 3.0) + pow(t1, 3.0)*t2 - 2 * pow(t1, 2.0)*pow(t2, 2.0) + t1*pow(t2, 3.0)) - (2 * p0*(pow(t2, 3.0) + 3 * t1*pow(t2, 2.0))) / (pow(t0, 2.0)*pow(t1, 2.0) - 2 * pow(t0, 2.0)*t1*t2 + pow(t0, 2.0)*pow(t2, 2.0) - t0*pow(t1, 3.0) + t0*pow(t1, 2.0)*t2 + t0*t1*pow(t2, 2.0) - t0*pow(t2, 3.0) + pow(t1, 3.0)*t2 - 2 * pow(t1, 2.0)*pow(t2, 2.0) + t1*pow(t2, 3.0)) + (v2*(-pow(t1, 3.0)*t2 + t0*pow(t1, 3.0) - 3 * t0*pow(t1, 2.0)*t2 + 5 * t1*pow(t2, 3.0) - 3 * t0*t1*pow(t2, 2.0) + t0*pow(t2, 3.0))) / (-pow(t1, 3.0)*t2 + t0*pow(t1, 3.0) + 3 * pow(t1, 2.0)*pow(t2, 2.0) - 3 * t0*pow(t1, 2.0)*t2 - 3 * t1*pow(t2, 3.0) + 3 * t0*t1*pow(t2, 2.0) + pow(t2, 4.0) - t0*pow(t2, 3.0)) - (2 * p1*(3 * pow(t1, 2.0)*pow(t2, 2.0) - 5 * t1*pow(t2, 3.0) + 3 * t0*t1*pow(t2, 2.0) - t0*pow(t2, 3.0))) / (pow(t1, 4.0)*t2 - t0*pow(t1, 4.0) - 4 * pow(t1, 3.0)*pow(t2, 2.0) + 4 * t0*pow(t1, 3.0)*t2 + 6 * pow(t1, 2.0)*pow(t2, 3.0) - 6 * t0*pow(t1, 2.0)*pow(t2, 2.0) - 4 * t1*pow(t2, 4.0) + 4 * t0*t1*pow(t2, 3.0) + pow(t2, 5.0) - t0*pow(t2, 4.0)) + (2 * p2*(3 * pow(t1, 2.0)*pow(t2, 2.0) - 5 * t1*pow(t2, 3.0) + 3 * t0*t1*pow(t2, 2.0) - t0*pow(t2, 3.0))) / (pow(t1, 4.0)*t2 - t0*pow(t1, 4.0) - 4 * pow(t1, 3.0)*pow(t2, 2.0) + 4 * t0*pow(t1, 3.0)*t2 + 6 * pow(t1, 2.0)*pow(t2, 3.0) - 6 * t0*pow(t1, 2.0)*pow(t2, 2.0) - 4 * t1*pow(t2, 4.0) + 4 * t0*t1*pow(t2, 3.0) + pow(t2, 5.0) - t0*pow(t2, 4.0)) + (a0*(3 * pow(t1, 2.0)*pow(t2, 2.0) + t1*pow(t2, 3.0) - 3 * t0*t1*pow(t2, 2.0) - t0*pow(t2, 3.0))) / (6 * (-pow(t1, 2.0)*t2 + t0*pow(t1, 2.0) + 2 * t1*pow(t2, 2.0) - 2 * t0*t1*t2 - pow(t2, 3.0) + t0*pow(t2, 2.0))) + (a2*(3 * pow(t1, 2.0)*pow(t2, 2.0) - 6 * t0*pow(t1, 2.0)*t2 + 5 * t1*pow(t2, 3.0) - 3 * t0*t1*pow(t2, 2.0) + t0*pow(t2, 3.0))) / (6 * (-pow(t1, 2.0)*t2 + t0*pow(t1, 2.0) + 2 * t1*pow(t2, 2.0) - 2 * t0*t1*t2 - pow(t2, 3.0) + t0*pow(t2, 2.0))) + (v0*(pow(t2, 3.0) + 3 * t1*pow(t2, 2.0))) / (-pow(t1, 2.0)*t2 + t0*pow(t1, 2.0) + 2 * t1*pow(t2, 2.0) - 2 * t0*t1*t2 - pow(t2, 3.0) + t0*pow(t2, 2.0));
	P[7] = (6 * p0*(pow(t2, 2.0) + t1*t2)) / (pow(t0, 2.0)*pow(t1, 2.0) - 2 * pow(t0, 2.0)*t1*t2 + pow(t0, 2.0)*pow(t2, 2.0) - t0*pow(t1, 3.0) + t0*pow(t1, 2.0)*t2 + t0*t1*pow(t2, 2.0) - t0*pow(t2, 3.0) + pow(t1, 3.0)*t2 - 2 * pow(t1, 2.0)*pow(t2, 2.0) + t1*pow(t2, 3.0)) - (6 * p1*(pow(t2, 2.0) + t1*t2)) / (pow(t0, 2.0)*pow(t1, 2.0) - 2 * pow(t0, 2.0)*t1*t2 + pow(t0, 2.0)*pow(t2, 2.0) - t0*pow(t1, 3.0) + t0*pow(t1, 2.0)*t2 + t0*t1*pow(t2, 2.0) - t0*pow(t2, 3.0) + pow(t1, 3.0)*t2 - 2 * pow(t1, 2.0)*pow(t2, 2.0) + t1*pow(t2, 3.0)) + (a0*(-pow(t1, 2.0)*t2 - t1*pow(t2, 2.0) + t0*t1*t2 + t0*pow(t2, 2.0))) / (2 * (-pow(t1, 2.0)*t2 + t0*pow(t1, 2.0) + 2 * t1*pow(t2, 2.0) - 2 * t0*t1*t2 - pow(t2, 3.0) + t0*pow(t2, 2.0))) - (3 * v2*(-pow(t1, 2.0)*t2 + 2 * t1*pow(t2, 2.0) - 2 * t0*t1*t2 + pow(t2, 3.0))) / (-pow(t1, 3.0)*t2 + t0*pow(t1, 3.0) + 3 * pow(t1, 2.0)*pow(t2, 2.0) - 3 * t0*pow(t1, 2.0)*t2 - 3 * t1*pow(t2, 3.0) + 3 * t0*t1*pow(t2, 2.0) + pow(t2, 4.0) - t0*pow(t2, 3.0)) - (3 * v0*(pow(t2, 2.0) + t1*t2)) / (-pow(t1, 2.0)*t2 + t0*pow(t1, 2.0) + 2 * t1*pow(t2, 2.0) - 2 * t0*t1*t2 - pow(t2, 3.0) + t0*pow(t2, 2.0)) + (a2*(t0*pow(t1, 2.0) - 3 * t1*pow(t2, 2.0) + 3 * t0*t1*t2 - pow(t2, 3.0))) / (2 * (-pow(t1, 2.0)*t2 + t0*pow(t1, 2.0) + 2 * t1*pow(t2, 2.0) - 2 * t0*t1*t2 - pow(t2, 3.0) + t0*pow(t2, 2.0))) - (6 * p1*(-pow(t1, 2.0)*t2 + t1*pow(t2, 2.0) - t0*t1*t2 + pow(t2, 3.0))) / (pow(t1, 4.0)*t2 - t0*pow(t1, 4.0) - 4 * pow(t1, 3.0)*pow(t2, 2.0) + 4 * t0*pow(t1, 3.0)*t2 + 6 * pow(t1, 2.0)*pow(t2, 3.0) - 6 * t0*pow(t1, 2.0)*pow(t2, 2.0) - 4 * t1*pow(t2, 4.0) + 4 * t0*t1*pow(t2, 3.0) + pow(t2, 5.0) - t0*pow(t2, 4.0)) + (6 * p2*(-pow(t1, 2.0)*t2 + t1*pow(t2, 2.0) - t0*t1*t2 + pow(t2, 3.0))) / (pow(t1, 4.0)*t2 - t0*pow(t1, 4.0) - 4 * pow(t1, 3.0)*pow(t2, 2.0) + 4 * t0*pow(t1, 3.0)*t2 + 6 * pow(t1, 2.0)*pow(t2, 3.0) - 6 * t0*pow(t1, 2.0)*pow(t2, 2.0) - 4 * t1*pow(t2, 4.0) + 4 * t0*t1*pow(t2, 3.0) + pow(t2, 5.0) - t0*pow(t2, 4.0));
	P[8] = (2 * p1*(t1 + 3 * t2)) / (pow(t0, 2.0)*pow(t1, 2.0) - 2 * pow(t0, 2.0)*t1*t2 + pow(t0, 2.0)*pow(t2, 2.0) - t0*pow(t1, 3.0) + t0*pow(t1, 2.0)*t2 + t0*t1*pow(t2, 2.0) - t0*pow(t2, 3.0) + pow(t1, 3.0)*t2 - 2 * pow(t1, 2.0)*pow(t2, 2.0) + t1*pow(t2, 3.0)) - (2 * p0*(t1 + 3 * t2)) / (pow(t0, 2.0)*pow(t1, 2.0) - 2 * pow(t0, 2.0)*t1*t2 + pow(t0, 2.0)*pow(t2, 2.0) - t0*pow(t1, 3.0) + t0*pow(t1, 2.0)*t2 + t0*t1*pow(t2, 2.0) - t0*pow(t2, 3.0) + pow(t1, 3.0)*t2 - 2 * pow(t1, 2.0)*pow(t2, 2.0) + t1*pow(t2, 3.0)) - (v2*(pow(t1, 2.0) + 2 * t0*t1 - 5 * pow(t2, 2.0) + 2 * t0*t2)) / (-pow(t1, 3.0)*t2 + t0*pow(t1, 3.0) + 3 * pow(t1, 2.0)*pow(t2, 2.0) - 3 * t0*pow(t1, 2.0)*t2 - 3 * t1*pow(t2, 3.0) + 3 * t0*t1*pow(t2, 2.0) + pow(t2, 4.0) - t0*pow(t2, 3.0)) - (a2*(pow(t1, 2.0) - 3 * t1*t2 + 5 * t0*t1 - 6 * pow(t2, 2.0) + 3 * t0*t2)) / (6 * (-pow(t1, 2.0)*t2 + t0*pow(t1, 2.0) + 2 * t1*pow(t2, 2.0) - 2 * t0*t1*t2 - pow(t2, 3.0) + t0*pow(t2, 2.0))) - (a0*(t0*t1 + 3 * t0*t2 - 3 * t1*t2 - pow(t1, 2.0))) / (6 * (-pow(t1, 2.0)*t2 + t0*pow(t1, 2.0) + 2 * t1*pow(t2, 2.0) - 2 * t0*t1*t2 - pow(t2, 3.0) + t0*pow(t2, 2.0))) + (v0*(t1 + 3 * t2)) / (-pow(t1, 2.0)*t2 + t0*pow(t1, 2.0) + 2 * t1*pow(t2, 2.0) - 2 * t0*t1*t2 - pow(t2, 3.0) + t0*pow(t2, 2.0)) - (2 * p1*(pow(t1, 2.0) + t1*t2 + t0*t1 - 4 * pow(t2, 2.0) + t0*t2)) / (pow(t1, 4.0)*t2 - t0*pow(t1, 4.0) - 4 * pow(t1, 3.0)*pow(t2, 2.0) + 4 * t0*pow(t1, 3.0)*t2 + 6 * pow(t1, 2.0)*pow(t2, 3.0) - 6 * t0*pow(t1, 2.0)*pow(t2, 2.0) - 4 * t1*pow(t2, 4.0) + 4 * t0*t1*pow(t2, 3.0) + pow(t2, 5.0) - t0*pow(t2, 4.0)) + (2 * p2*(pow(t1, 2.0) + t1*t2 + t0*t1 - 4 * pow(t2, 2.0) + t0*t2)) / (pow(t1, 4.0)*t2 - t0*pow(t1, 4.0) - 4 * pow(t1, 3.0)*pow(t2, 2.0) + 4 * t0*pow(t1, 3.0)*t2 + 6 * pow(t1, 2.0)*pow(t2, 3.0) - 6 * t0*pow(t1, 2.0)*pow(t2, 2.0) - 4 * t1*pow(t2, 4.0) + 4 * t0*t1*pow(t2, 3.0) + pow(t2, 5.0) - t0*pow(t2, 4.0));
	P[9] = (2 * p0) / (pow(t0, 2.0)*pow(t1, 2.0) - 2 * pow(t0, 2.0)*t1*t2 + pow(t0, 2.0)*pow(t2, 2.0) - t0*pow(t1, 3.0) + t0*pow(t1, 2.0)*t2 + t0*t1*pow(t2, 2.0) - t0*pow(t2, 3.0) + pow(t1, 3.0)*t2 - 2 * pow(t1, 2.0)*pow(t2, 2.0) + t1*pow(t2, 3.0)) - v0 / (-pow(t1, 2.0)*t2 + t0*pow(t1, 2.0) + 2 * t1*pow(t2, 2.0) - 2 * t0*t1*t2 - pow(t2, 3.0) + t0*pow(t2, 2.0)) - (2 * p1) / (pow(t0, 2.0)*pow(t1, 2.0) - 2 * pow(t0, 2.0)*t1*t2 + pow(t0, 2.0)*pow(t2, 2.0) - t0*pow(t1, 3.0) + t0*pow(t1, 2.0)*t2 + t0*t1*pow(t2, 2.0) - t0*pow(t2, 3.0) + pow(t1, 3.0)*t2 - 2 * pow(t1, 2.0)*pow(t2, 2.0) + t1*pow(t2, 3.0)) + (p1*(t0 + 2 * t1 - 3 * t2)) / (pow(t1, 4.0)*t2 - t0*pow(t1, 4.0) - 4 * pow(t1, 3.0)*pow(t2, 2.0) + 4 * t0*pow(t1, 3.0)*t2 + 6 * pow(t1, 2.0)*pow(t2, 3.0) - 6 * t0*pow(t1, 2.0)*pow(t2, 2.0) - 4 * t1*pow(t2, 4.0) + 4 * t0*t1*pow(t2, 3.0) + pow(t2, 5.0) - t0*pow(t2, 4.0)) - (p2*(t0 + 2 * t1 - 3 * t2)) / (pow(t1, 4.0)*t2 - t0*pow(t1, 4.0) - 4 * pow(t1, 3.0)*pow(t2, 2.0) + 4 * t0*pow(t1, 3.0)*t2 + 6 * pow(t1, 2.0)*pow(t2, 3.0) - 6 * t0*pow(t1, 2.0)*pow(t2, 2.0) - 4 * t1*pow(t2, 4.0) + 4 * t0*t1*pow(t2, 3.0) + pow(t2, 5.0) - t0*pow(t2, 4.0)) + (v2*(t0 + t1 - 2 * t2)) / (-pow(t1, 3.0)*t2 + t0*pow(t1, 3.0) + 3 * pow(t1, 2.0)*pow(t2, 2.0) - 3 * t0*pow(t1, 2.0)*t2 - 3 * t1*pow(t2, 3.0) + 3 * t0*t1*pow(t2, 2.0) + pow(t2, 4.0) - t0*pow(t2, 3.0)) + (a2*(2 * t0 + t1 - 3 * t2)) / (6 * (-pow(t1, 2.0)*t2 + t0*pow(t1, 2.0) + 2 * t1*pow(t2, 2.0) - 2 * t0*t1*t2 - pow(t2, 3.0) + t0*pow(t2, 2.0))) + (a0*(t0 - t1)) / (6 * (-pow(t1, 2.0)*t2 + t0*pow(t1, 2.0) + 2 * t1*pow(t2, 2.0) - 2 * t0*t1*t2 - pow(t2, 3.0) + t0*pow(t2, 2.0)));

	for (i = (int)(t0 / step); i < (int)(t1 / step); i++)
	{
		S[0][k] = P[0] + P[1] * i*step + P[2] * i*step*i*step + P[3] * i*step*step*step*i*i + P[4] * i*i*i*i*step*step*step*step;
		S[1][k] = P[1] + 2 * P[2] * i*step + 3 * P[3] * i*i*step*step + 4 * P[4] * i*i*i*step*step*step;
		S[2][k] = 2 * P[2] + 6 * P[3] * i*step + 12 * P[4] * i*i*step*step;
		k = k + 1;
	}

	for (i = (int)(t1 / step); i <=(int)(t2 / step); i++)
	{
		S[0][k] = P[5] + P[6] * i*step + P[7] * i*i*step*step + P[8] * i*i*i*step*step*step + P[9] * i*i*i*i*step*step*step*step;
		S[1][k] = P[6] + 2 * P[7] * i*step + 3 * P[8] * i*i*step*step + 4 * P[9] * i*i*i*step*step*step;
		S[2][k] = 2 * P[7] + 6 * P[8] * i*step + 12 * P[9] * i*i*step*step;
		k = k + 1;
	}

}

void ArmJoint_Tra()
{
	int k;
	double arm4_rate = ARM4_WAVE / ARM1_WAVE; 
	
#ifdef USE_WAVE_ARM
	int j;
	int k_step;
	/* T-ready */
	j = 0;
	TSpline_S_V_A(ARM4_RESET, 0, 0, 0, (ARM4_RESET + ARM4_WAVE)*0.5, T_ready*0.5, ARM4_WAVE, 0, 0, T_ready, CONTROL_T);
	for (k = 0; k<=round(T_ready / CONTROL_T); k++)
	{
		Tra_ArmJoint.jr[1][k] = 0.0; // right 
		Tra_ArmJoint.jr[2][k] = 0.0;
		Tra_ArmJoint.jr[3][k] = 0.0;
		Tra_ArmJoint.jr[4][k] = S[0][j];

		Tra_ArmJoint.jl[1][k] = 0.0; // left 
		Tra_ArmJoint.jl[2][k] = 0.0;
		Tra_ArmJoint.jl[3][k] = 0.0;
		Tra_ArmJoint.jl[4][k] = S[0][j];

		j++;
	}

	/* First Step */
	// right hand
	j = 0;
	TSpline_S_V_A(0, 0, 0, 0, ARM1_WAVE / 2, T_step1*0.5, ARM1_WAVE, 0, 0, T_step1, CONTROL_T);
	for(k=round(T_ready/CONTROL_T);k<=round((T_ready+T_step1)/CONTROL_T);k++)  
	{
		Tra_ArmJoint.jr[1][k] = S[0][j]; // right 
		Tra_ArmJoint.jr[2][k] = 0.0;
		Tra_ArmJoint.jr[3][k] = 0.0;
		Tra_ArmJoint.jr[4][k] = ARM4_WAVE + arm4_rate*S[0][j];
		
		j++;
	} 
	// left hand
	j = 0;
	TSpline_S_V_A(0, 0, 0, 0, -ARM1_WAVE / 2, T_step1*0.5, -ARM1_WAVE, 0, 0, T_step1, CONTROL_T);
	for(k=round(T_ready/CONTROL_T);k<=round((T_ready+T_step1)/CONTROL_T);k++)  
	{
		Tra_ArmJoint.jl[1][k] = S[0][j]; // left
		Tra_ArmJoint.jl[2][k] = 0.0;
		Tra_ArmJoint.jl[3][k] = 0.0;
		Tra_ArmJoint.jl[4][k] = ARM4_WAVE + arm4_rate*S[0][j];
		
		j++;
	}

	/* Second Step --> (N_step-1) Step */
	for(k_step=1;k_step<N_step-1;k_step++)  
	{
		if(k_step%2!=0)  // right leg swing  1,3,5...
		{
			// right hand
			j = 0;
			TSpline_S_V_A(ARM1_WAVE, 0, 0, 0, 0, T_step / 2, -ARM1_WAVE, 0, 0, T_step, CONTROL_T);
			for(k=round((T_ready+T_step1+(k_step-1)*T_step)/CONTROL_T);k<=round((T_ready+T_step1+(k_step-1)*T_step+T_step)/CONTROL_T);k++)   
			{
				Tra_ArmJoint.jr[1][k] = S[0][j]; // right 
				Tra_ArmJoint.jr[2][k] = 0.0;
				Tra_ArmJoint.jr[3][k] = 0.0;
				Tra_ArmJoint.jr[4][k] = ARM4_WAVE + arm4_rate*S[0][j];
				
				j++;
			}
			
			// left hand
			j = 0;
			TSpline_S_V_A(-ARM1_WAVE, 0, 0, 0, 0, T_step / 2, ARM1_WAVE, 0, 0, T_step, CONTROL_T);
			for(k=round((T_ready+T_step1+(k_step-1)*T_step)/CONTROL_T);k<=round((T_ready+T_step1+(k_step-1)*T_step+T_step)/CONTROL_T);k++)   
			{
				Tra_ArmJoint.jl[1][k] = S[0][j]; // left 
				Tra_ArmJoint.jl[2][k] = 0.0;
				Tra_ArmJoint.jl[3][k] = 0.0;
				Tra_ArmJoint.jl[4][k] = ARM4_WAVE + arm4_rate*S[0][j];
				
				j++;
			}
		}

		if(k_step%2==0)  // left leg swing  0,2,4...
		{
			// right hand
			j = 0;
			TSpline_S_V_A(-ARM1_WAVE, 0, 0, 0, 0, T_step / 2, ARM1_WAVE, 0, 0, T_step, CONTROL_T);	
			for(k=round((T_ready+T_step1+(k_step-1)*T_step)/CONTROL_T);k<=round((T_ready+T_step1+(k_step-1)*T_step+T_step)/CONTROL_T);k++)   
			{
				Tra_ArmJoint.jr[1][k] = S[0][j]; // right 
				Tra_ArmJoint.jr[2][k] = 0.0;
				Tra_ArmJoint.jr[3][k] = 0.0;
				Tra_ArmJoint.jr[4][k] = ARM4_WAVE + arm4_rate*S[0][j];
				
				j++;
			}

			// left hand
			j = 0;
			TSpline_S_V_A(ARM1_WAVE, 0, 0, 0, 0, T_step / 2, -ARM1_WAVE, 0, 0, T_step, CONTROL_T);
			for(k=round((T_ready+T_step1+(k_step-1)*T_step)/CONTROL_T);k<=round((T_ready+T_step1+(k_step-1)*T_step+T_step)/CONTROL_T);k++)   
			{
				Tra_ArmJoint.jl[1][k] = S[0][j]; // left 
				Tra_ArmJoint.jl[2][k] = 0.0;
				Tra_ArmJoint.jl[3][k] = 0.0;
				Tra_ArmJoint.jl[4][k] = ARM4_WAVE + arm4_rate*S[0][j];
				
				j++;
			}
		}		
	}
	
	/* Last Step */
	if(N_step%2==0)    // right leg swing
	{
		// right hand
		j = 0;
		TSpline_S_V_A(ARM1_WAVE, 0, 0, 0, ARM1_WAVE*0.5, T_stepn / 2, 0, 0, 0, T_stepn, CONTROL_T);
		for(k=round((T_ready+T_step1+(N_step-2)*T_step)/CONTROL_T);k<=round((T_ready+T_step1+(N_step-2)*T_step+T_stepn)/CONTROL_T);k++)
		{
			Tra_ArmJoint.jr[1][k] = S[0][j]; // right 
			Tra_ArmJoint.jr[2][k] = 0.0;
			Tra_ArmJoint.jr[3][k] = 0.0;
			Tra_ArmJoint.jr[4][k] = ARM4_WAVE + arm4_rate*S[0][j];
			
			j++;
		}
		
		// left hand
		j = 0;
		TSpline_S_V_A(-ARM1_WAVE, 0, 0, 0, -ARM1_WAVE*0.5, T_stepn / 2, 0, 0, 0, T_stepn, CONTROL_T);
		for(k=round((T_ready+T_step1+(N_step-2)*T_step)/CONTROL_T);k<=round((T_ready+T_step1+(N_step-2)*T_step+T_stepn)/CONTROL_T);k++)
		{
			Tra_ArmJoint.jl[1][k] = S[0][j]; // left 
			Tra_ArmJoint.jl[2][k] = 0.0;
			Tra_ArmJoint.jl[3][k] = 0.0;
			Tra_ArmJoint.jl[4][k] = ARM4_WAVE + arm4_rate*S[0][j];
			
			j++;
		}
	}
	else   // left leg swing 
	{
		// right hand
		j = 0;
		TSpline_S_V_A(-ARM1_WAVE, 0, 0, 0, -ARM1_WAVE*0.5, T_stepn / 2, 0, 0, 0, T_stepn, CONTROL_T);
		for(k=round((T_ready+T_step1+(N_step-2)*T_step)/CONTROL_T);k<=round((T_ready+T_step1+(N_step-2)*T_step+T_stepn)/CONTROL_T);k++)
		{
			Tra_ArmJoint.jr[1][k] = S[0][j]; // right 
			Tra_ArmJoint.jr[2][k] = 0.0;
			Tra_ArmJoint.jr[3][k] = 0.0;
			Tra_ArmJoint.jr[4][k] = ARM4_WAVE + arm4_rate*S[0][j];
			
			j++;
		}
	
		// left hand
		j = 0;
		TSpline_S_V_A(ARM1_WAVE, 0, 0, 0, ARM1_WAVE*0.5, T_stepn / 2, 0, 0, 0, T_stepn, CONTROL_T);
		for(k=round((T_ready+T_step1+(N_step-2)*T_step)/CONTROL_T);k<=round((T_ready+T_step1+(N_step-2)*T_step+T_stepn)/CONTROL_T);k++)
		{
			Tra_ArmJoint.jl[1][k] = S[0][j]; // left 
			Tra_ArmJoint.jl[2][k] = 0.0;
			Tra_ArmJoint.jl[3][k] = 0.0;
			Tra_ArmJoint.jl[4][k] = ARM4_WAVE + arm4_rate*S[0][j];
			
			j++;
		}
	}
	
	j = 0;
	TSpline_S_V_A(ARM4_WAVE, 0, 0, 0, (ARM4_RESET + ARM4_WAVE)*0.5, T_end / 2, ARM4_RESET, 0, 0, T_end, CONTROL_T);
	for (k = round((T_ready+T_step1+(N_step-2)*T_step+T_stepn)/CONTROL_T); k<=round(T_walk / CONTROL_T); k++)
	{
		Tra_ArmJoint.jr[1][k] = 0.0; // right 
		Tra_ArmJoint.jr[2][k] = 0.0;
		Tra_ArmJoint.jr[3][k] = 0.0;
		Tra_ArmJoint.jr[4][k] = S[0][j];

		Tra_ArmJoint.jl[1][k] = 0.0; // left 
		Tra_ArmJoint.jl[2][k] = 0.0;
		Tra_ArmJoint.jl[3][k] = 0.0;
		Tra_ArmJoint.jl[4][k] = S[0][j];

		j++;
	}
	
	for (k = round(T_walk/CONTROL_T); k<=round(T_com/CONTROL_T); k++)
	{
		Tra_ArmJoint.jr[1][k] = 0.0; // right 
		Tra_ArmJoint.jr[2][k] = 0.0;
		Tra_ArmJoint.jr[3][k] = 0.0;
		Tra_ArmJoint.jr[4][k] = ARM4_RESET;

		Tra_ArmJoint.jl[1][k] = 0.0; // left 
		Tra_ArmJoint.jl[2][k] = 0.0;
		Tra_ArmJoint.jl[3][k] = 0.0;
		Tra_ArmJoint.jl[4][k] = ARM4_RESET;
	}
	
	//Other joint angle should Set to Zero
	for (k = 0; k<=round(T_com/CONTROL_T); k++)
	{
		Tra_ArmJoint.jr[0][k] = 0.0; // right 
		Tra_ArmJoint.jr[5][k] = 0.0;
		Tra_ArmJoint.jr[6][k] = 0.0;

		Tra_ArmJoint.jl[0][k] = 0.0; // left 
		Tra_ArmJoint.jl[5][k] = 0.0;
		Tra_ArmJoint.jl[6][k] = 0.0;
	}
#else
		for (k = 0; k<=round(T_com/CONTROL_T); k++)
		{
			Tra_ArmJoint.jr[1][k] = 0.0; // right 
			Tra_ArmJoint.jr[2][k] = 0.0;
			Tra_ArmJoint.jr[3][k] = 0.0;
			Tra_ArmJoint.jr[4][k] = ARM4_RESET;

			Tra_ArmJoint.jl[1][k] = 0.0; // left 
			Tra_ArmJoint.jl[2][k] = 0.0;
			Tra_ArmJoint.jl[3][k] = 0.0;
			Tra_ArmJoint.jl[4][k] = ARM4_RESET;
			
			Tra_ArmJoint.jr[0][k] = 0.0; // right 
			Tra_ArmJoint.jr[5][k] = 0.0;
			Tra_ArmJoint.jr[6][k] = 0.0;

			Tra_ArmJoint.jl[0][k] = 0.0; // left 
			Tra_ArmJoint.jl[5][k] = 0.0;
			Tra_ArmJoint.jl[6][k] = 0.0;
		}
#endif/*end: USE_WAVE_ARM*/
	
	// if(NULL==(fp=fopen("arm_joints.dat","w")))
	// {
		// printf("arm_joints.dat File can not open!\n");
	// }
	// for(k=0;k<=round(T_com/CONTROL_T);k++)     
		// fprintf(fp,"%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n",Tra_ArmJoint.jr[1][k],Tra_ArmJoint.jr[2][k],Tra_ArmJoint.jr[3][k],Tra_ArmJoint.jr[4][k],Tra_ArmJoint.jl[1][k],Tra_ArmJoint.jl[2][k],Tra_ArmJoint.jl[3][k],Tra_ArmJoint.jl[4][k]);
	// fclose(fp);

}

void WaistCompensation_Tra()
{
	int k;
	
#ifdef USE_WAIST_COMPENSATION	
	int j, k_step;
	double comp_angle;
	comp_angle = DEG2RAD*1.0;
	
	for(k=0;k<=round((T_ready-(1-Pct_dou2)*T_dou)/CONTROL_T);k++)
	{
		Comp_Waist[k] = 0.0;
	}
	j=0;
	TSpline_S_V_A( 0, 0, 0, 0,comp_angle*0.5, (1-Pct_dou2)*T_dou*0.5, comp_angle,0,0,(1-Pct_dou2)*T_dou, CONTROL_T);	
	for(k=round((T_ready-(1-Pct_dou2)*T_dou)/CONTROL_T);k<=round(T_ready/CONTROL_T);k++)
	{
		Comp_Waist[k] = S[0][j++];
	}
	
	/* First Step */
		// single phase 
	for(k=round(T_ready/CONTROL_T);k<=round((T_ready+T_step1*(1-Pct_dou)+T_step1*Pct_dou*Pct_dou1)/CONTROL_T);k++)  
	{
		Comp_Waist[k] = comp_angle;
	}		
	   // double phase 
	j=0;
	TSpline_S_V_A( comp_angle, 0, 0, 0, 0, T_step1*Pct_dou*(1-Pct_dou1)*0.5, -comp_angle,0,0,T_step1*Pct_dou*(1-Pct_dou1), CONTROL_T);	
	for(k=round((T_ready+T_step1*(1-Pct_dou)+T_step1*Pct_dou*Pct_dou1)/CONTROL_T);k<=round((T_ready+T_step1)/CONTROL_T);k++)   
	{
		Comp_Waist[k] = S[0][j++];	
	}
	
	/* Second Step --> (N_step-1) Step */
	for(k_step=1;k_step<N_step-1;k_step++)  
	{	
		if(k_step%2!=0) // right leg swing  1,3,5...
		{
			// single phase
			for(k=round((T_ready+T_step1+(k_step-1)*T_step)/CONTROL_T);k<=round((T_ready+T_step1+(k_step-1)*T_step+T_sin+T_dou*Pct_dou1)/CONTROL_T);k++)   
			{
				Comp_Waist[k] = -comp_angle;
			}
			// double phase
			j=0;
			TSpline_S_V_A(-comp_angle, 0, 0, 0, 0, T_dou*(1-Pct_dou1)*0.5, comp_angle, 0, 0, T_dou*(1-Pct_dou1), CONTROL_T);
			for(k=round((T_ready+T_step1+(k_step-1)*T_step+T_sin+T_dou*Pct_dou1)/CONTROL_T);k<=round((T_ready+T_step1+(k_step-1)*T_step+T_step)/CONTROL_T);k++)  
			{
				Comp_Waist[k] = S[0][j++];	
			}
		}
		
		if(k_step%2==0) // left leg swing   0,2,4...
		{
			// single phase
			for(k=round((T_ready+T_step1+(k_step-1)*T_step)/CONTROL_T);k<=round((T_ready+T_step1+(k_step-1)*T_step+T_sin+T_dou*Pct_dou1)/CONTROL_T);k++)   
			{
				Comp_Waist[k] = comp_angle;
			}
			// double phase
			j=0;
			TSpline_S_V_A( comp_angle, 0, 0, 0, 0, T_dou*(1-Pct_dou1)*0.5, -comp_angle, 0, 0, T_dou*(1-Pct_dou1), CONTROL_T);
			for(k=round((T_ready+T_step1+(k_step-1)*T_step+T_sin+T_dou*Pct_dou1)/CONTROL_T);k<=round((T_ready+T_step1+(k_step-1)*T_step+T_step)/CONTROL_T);k++)   
			{
				Comp_Waist[k] = S[0][j++];	
			}
		}
	}
	
	/* Last Step */
	if(N_step%2==0)  // right leg swing
	{
		// single phase
		for(k=round((T_ready+T_step1+(N_step-2)*T_step)/CONTROL_T);k<=round((T_ready+T_step1+(N_step-2)*T_step+T_stepn*(1-Pct_dou)+T_stepn*Pct_dou1)/CONTROL_T);k++)   
		{
			Comp_Waist[k] = -comp_angle;
		}
		// double phase
		j=0;
		TSpline_S_V_A(-comp_angle, 0, 0, 0, -comp_angle*0.5, T_stepn*Pct_dou*(1-Pct_dou1)*0.5, 0, 0, 0, T_stepn*Pct_dou*(1-Pct_dou1), CONTROL_T);
		for(k=round((T_ready+T_step1+(N_step-2)*T_step+T_stepn*(1-Pct_dou)+T_stepn*Pct_dou*Pct_dou1)/CONTROL_T);k<=round((T_ready+T_step1+(N_step-2)*T_step+T_stepn)/CONTROL_T);k++)   
		{
			Comp_Waist[k] = S[0][j++];
		}
	}
	else  // left leg swing
	{
		// single phase
		for(k=round((T_ready+T_step1+(N_step-2)*T_step)/CONTROL_T);k<=round((T_ready+T_step1+(N_step-2)*T_step+T_stepn*(1-Pct_dou)+T_stepn*Pct_dou1)/CONTROL_T);k++)   
		{
			Comp_Waist[k] = comp_angle;
		}
		// double phase
		j=0;
		TSpline_S_V_A( comp_angle, 0, 0, 0, comp_angle*0.5, T_stepn*Pct_dou*(1-Pct_dou1)*0.5, 0, 0, 0, T_stepn*Pct_dou*(1-Pct_dou1), CONTROL_T);
		for(k=round((T_ready+T_step1+(N_step-2)*T_step+T_stepn*(1-Pct_dou)+T_stepn*Pct_dou*Pct_dou1)/CONTROL_T);k<=round((T_ready+T_step1+(N_step-2)*T_step+T_stepn)/CONTROL_T);k++)   
		{
			Comp_Waist[k] = S[0][j++];
		}
	}
	
	for(k=round((T_ready+T_step1+(N_step-2)*T_step+T_stepn)/CONTROL_T);k<=round(T_com/CONTROL_T);k++)   
	{
		Comp_Waist[k] = 0.0;
	}

#else	
	for(k=0;k<=round(T_com/CONTROL_T);k++)   
	{
		Comp_Waist[k] = 0.0;
	}
#endif /* end : USE_WAIST_COMPENSATION */

	// if(NULL==(fp=fopen("waist_compensation.dat","w")))
	// {
		// printf("waist_compensation File can not open!\n");
	// }
	// for(k=0;k<=round(T_com/CONTROL_T);k++)     // COM_t
		// fprintf(fp,"%lf\n",Comp_Waist[k]);						
	// fclose(fp);
}

void WaistCompensation_Tra_New()
{
	int k, j;
	int k_step;
	double comp_angle;
	double first_step_gain; // When 'Move Right', this gain should be less than 'Walk' and 'Move Left'. 
	double t_ready_time;    // Tready - t_ready_time
	
	comp_angle = DEG2RAD*1.0;
	
	// Only for Move Right
	if(PreCon_Mode == PRECON_MOVE_RIGHT) first_step_gain = 0.1; //0.5;//
	else first_step_gain = 1.0;
	
	t_ready_time = T_dou;
	for(k=0;k<=round((T_ready-t_ready_time)/CONTROL_T);k++)
	{
		Comp_Waist[k] = 0.0;
	}
	j=0;
	TSpline_S_V_A( 0, 0, 0, 0,first_step_gain*comp_angle*0.5, t_ready_time*0.5, first_step_gain*comp_angle,0,0,t_ready_time, CONTROL_T);	
	for(k=round((T_ready-t_ready_time)/CONTROL_T);k<=round(T_ready/CONTROL_T);k++)
	{
		Comp_Waist[k] = S[0][j++];
	}
	
	/* First Step */
		// single phase 
	for(k=round(T_ready/CONTROL_T);k<=round((T_ready+T_step1*(1-Pct_dou))/CONTROL_T);k++)  
	{
		Comp_Waist[k] = first_step_gain*comp_angle;
	}		
	   // double phase 
	j=0;
	TSpline_S_V_A( first_step_gain*comp_angle, 0, 0, 0, 0, T_step1*Pct_dou*0.5, -comp_angle,0,0,T_step1*Pct_dou, CONTROL_T);	
	for(k=round((T_ready+T_step1*(1-Pct_dou))/CONTROL_T);k<=round((T_ready+T_step1)/CONTROL_T);k++)   
	{
		Comp_Waist[k] = S[0][j++];	
	}
	
	/* Second Step --> (N_step-1) Step */
	for(k_step=1;k_step<N_step-1;k_step++)  
	{	
		if(k_step%2!=0) // right leg swing  1,3,5...
		{
			// single phase
			for(k=round((T_ready+T_step1+(k_step-1)*T_step)/CONTROL_T);k<=round((T_ready+T_step1+(k_step-1)*T_step+T_sin)/CONTROL_T);k++)   
			{
				Comp_Waist[k] = -comp_angle;
			}
			// double phase
			j=0;
			TSpline_S_V_A(-comp_angle, 0, 0, 0, 0, T_dou*0.5, comp_angle, 0, 0, T_dou, CONTROL_T);
			for(k=round((T_ready+T_step1+(k_step-1)*T_step+T_sin)/CONTROL_T);k<=round((T_ready+T_step1+(k_step-1)*T_step+T_step)/CONTROL_T);k++)  
			{
				Comp_Waist[k] = S[0][j++];	
			}
		}
		
		if(k_step%2==0) // left leg swing   0,2,4...
		{
			// single phase
			for(k=round((T_ready+T_step1+(k_step-1)*T_step)/CONTROL_T);k<=round((T_ready+T_step1+(k_step-1)*T_step+T_sin)/CONTROL_T);k++)   
			{
				Comp_Waist[k] = comp_angle;
			}
			// double phase
			j=0;
			TSpline_S_V_A( comp_angle, 0, 0, 0, 0, T_dou*0.5, -comp_angle, 0, 0, T_dou, CONTROL_T);
			for(k=round((T_ready+T_step1+(k_step-1)*T_step+T_sin)/CONTROL_T);k<=round((T_ready+T_step1+(k_step-1)*T_step+T_step)/CONTROL_T);k++)   
			{
				Comp_Waist[k] = S[0][j++];	
			}
		}
	}
	
	/* Last Step */
	if(N_step%2==0)  // right leg swing
	{
		// single phase
		for(k=round((T_ready+T_step1+(N_step-2)*T_step)/CONTROL_T);k<=round((T_ready+T_step1+(N_step-2)*T_step+T_stepn*(1-Pct_dou))/CONTROL_T);k++)   
		{
			Comp_Waist[k] = -comp_angle;
		}
		// double phase
		j=0;
		TSpline_S_V_A(-comp_angle, 0, 0, 0, -comp_angle*0.5, T_stepn*Pct_dou*0.5, 0, 0, 0, T_stepn*Pct_dou, CONTROL_T);
		for(k=round((T_ready+T_step1+(N_step-2)*T_step+T_stepn*(1-Pct_dou))/CONTROL_T);k<=round((T_ready+T_step1+(N_step-2)*T_step+T_stepn)/CONTROL_T);k++)   
		{
			Comp_Waist[k] = S[0][j++];
		}
	}
	else  // left leg swing
	{
		// single phase
		for(k=round((T_ready+T_step1+(N_step-2)*T_step)/CONTROL_T);k<=round((T_ready+T_step1+(N_step-2)*T_step+T_stepn*(1-Pct_dou))/CONTROL_T);k++)   
		{
			Comp_Waist[k] = comp_angle;
		}
		// double phase
		j=0;
		TSpline_S_V_A( comp_angle, 0, 0, 0, comp_angle*0.5, T_stepn*Pct_dou*0.5, 0, 0, 0, T_stepn*Pct_dou, CONTROL_T);
		for(k=round((T_ready+T_step1+(N_step-2)*T_step+T_stepn*(1-Pct_dou))/CONTROL_T);k<=round((T_ready+T_step1+(N_step-2)*T_step+T_stepn)/CONTROL_T);k++)   
		{
			Comp_Waist[k] = S[0][j++];
		}
	}
	
	for(k=round((T_ready+T_step1+(N_step-2)*T_step+T_stepn)/CONTROL_T);k<=round(T_com/CONTROL_T);k++)   
	{
		Comp_Waist[k] = 0.0;
	}



	// if(NULL==(fp=fopen("waist_compensation.dat","w")))
	// {
		// printf("waist_compensation File can not open!\n");
	// }
	// for(k=0;k<=round(T_com/CONTROL_T);k++)     // COM_t
		// fprintf(fp,"%lf\n",Comp_Waist[k]);						
	// fclose(fp);
}


/*------------------------------------------------------------------------------------------------
Note: Singal_SupportLeg[] : 0 -> double support, 1 -> right leg support, 2 -> left leg support
	  Singal_NowStep[]    : 1 ~ N_step 
------------------------------------------------------------------------------------------------*/
void StepSignal_Tra()
{
	int k;
	int k_step;
	
	for(k=0;k<=round(T_ready/CONTROL_T);k++)
	{
		Signal_SupportLeg[k] = DOUBLE_LEG_SP;
		Signal_NowStep[k] = 0;
#ifdef STANDSTILL
		Signal_SupportLeg[k] = DOUBLE_LEG_SP;
#endif
	}
	
	/* First Step */
		// single phase 
	for(k=round(T_ready/CONTROL_T);k<=round((T_ready+T_step1*(1-Pct_dou))/CONTROL_T);k++)  
	{
		Signal_SupportLeg[k] = RLEG_SP;
		Signal_NowStep[k] = 1;
#ifdef STANDSTILL
		Signal_SupportLeg[k] = DOUBLE_LEG_SP;
#endif
	}		
	   // double phase
	for(k=round((T_ready+T_step1*(1-Pct_dou))/CONTROL_T);k<=round((T_ready+T_step1)/CONTROL_T);k++)   
	{
		Signal_SupportLeg[k] = DOUBLE_LEG_SP;
		Signal_NowStep[k] = 1;		
#ifdef STANDSTILL
		Signal_SupportLeg[k] = DOUBLE_LEG_SP;
#endif
	}
	
	/* Second Step --> (N_step-1) Step */
	for(k_step=1;k_step<N_step-1;k_step++)  
	{	
		if(k_step%2!=0) // right leg swing  1,3,5...
		{
			// single phase
			for(k=round((T_ready+T_step1+(k_step-1)*T_step)/CONTROL_T);k<=round((T_ready+T_step1+(k_step-1)*T_step+T_sin)/CONTROL_T);k++)   
			{
				Signal_SupportLeg[k] = LLEG_SP;
				Signal_NowStep[k] = k_step+1;	
#ifdef STANDSTILL
				Signal_SupportLeg[k] = DOUBLE_LEG_SP;
#endif
			}
			// double phase
			for(k=round((T_ready+T_step1+(k_step-1)*T_step+T_sin)/CONTROL_T);k<=round((T_ready+T_step1+(k_step-1)*T_step+T_step)/CONTROL_T);k++)  
			{
				Signal_SupportLeg[k] = DOUBLE_LEG_SP;
				Signal_NowStep[k] = k_step+1;	
#ifdef STANDSTILL
				Signal_SupportLeg[k] = DOUBLE_LEG_SP;
#endif
			}
		}
		
		if(k_step%2==0) // left leg swing   0,2,4...
		{
			// single phase
			for(k=round((T_ready+T_step1+(k_step-1)*T_step)/CONTROL_T);k<=round((T_ready+T_step1+(k_step-1)*T_step+T_sin)/CONTROL_T);k++)   
			{
				Signal_SupportLeg[k] = RLEG_SP;
				Signal_NowStep[k] = k_step+1;
#ifdef STANDSTILL
				Signal_SupportLeg[k] = DOUBLE_LEG_SP;
#endif
			}
			// double phase
			for(k=round((T_ready+T_step1+(k_step-1)*T_step+T_sin)/CONTROL_T);k<=round((T_ready+T_step1+(k_step-1)*T_step+T_step)/CONTROL_T);k++)   
			{
				Signal_SupportLeg[k] = DOUBLE_LEG_SP;
				Signal_NowStep[k] = k_step+1;	
#ifdef STANDSTILL
				Signal_SupportLeg[k] = DOUBLE_LEG_SP;
#endif
			}
		}
	}
	
	/* Last Step */
	if(N_step%2==0)  // right leg swing
	{
		// single phase
		for(k=round((T_ready+T_step1+(N_step-2)*T_step)/CONTROL_T);k<=round((T_ready+T_step1+(N_step-2)*T_step+T_stepn*(1-Pct_dou))/CONTROL_T);k++)   
		{
			Signal_SupportLeg[k] = LLEG_SP;
			Signal_NowStep[k] = N_step;	
#ifdef STANDSTILL
			Signal_SupportLeg[k] = DOUBLE_LEG_SP;
#endif
		}
		// double phase
		for(k=round((T_ready+T_step1+(N_step-2)*T_step+T_stepn*(1-Pct_dou))/CONTROL_T);k<=round((T_ready+T_step1+(N_step-2)*T_step+T_stepn)/CONTROL_T);k++)   
		{
			Signal_SupportLeg[k] = DOUBLE_LEG_SP;
			Signal_NowStep[k] = N_step;	
#ifdef STANDSTILL
			Signal_SupportLeg[k] = DOUBLE_LEG_SP;
#endif
		}
	}
	else  // left leg swing
	{
		// single phase
		for(k=round((T_ready+T_step1+(N_step-2)*T_step)/CONTROL_T);k<=round((T_ready+T_step1+(N_step-2)*T_step+T_stepn*(1-Pct_dou))/CONTROL_T);k++)   
		{
			Signal_SupportLeg[k] = RLEG_SP;
			Signal_NowStep[k] = N_step;
#ifdef STANDSTILL
			Signal_SupportLeg[k] = DOUBLE_LEG_SP;
#endif
		}
		// double phase
		for(k=round((T_ready+T_step1+(N_step-2)*T_step+T_stepn*(1-Pct_dou))/CONTROL_T);k<=round((T_ready+T_step1+(N_step-2)*T_step+T_stepn)/CONTROL_T);k++)   
		{
			Signal_SupportLeg[k] = DOUBLE_LEG_SP;
			Signal_NowStep[k] = N_step;	
#ifdef STANDSTILL
			Signal_SupportLeg[k] = DOUBLE_LEG_SP;
#endif
		}
	}
	
	for(k=round((T_ready+T_step1+(N_step-2)*T_step+T_stepn)/CONTROL_T);k<=round(T_walk/CONTROL_T);k++)   
	{
		Signal_SupportLeg[k] = DOUBLE_LEG_SP;
		Signal_NowStep[k] = N_step;	
#ifdef STANDSTILL
		Signal_SupportLeg[k] = DOUBLE_LEG_SP;
#endif
	}
	
	for(k=round(T_walk/CONTROL_T);k<=round(T_com/CONTROL_T);k++)   
	{
		Signal_SupportLeg[k] = DOUBLE_LEG_SP-1;
		Signal_NowStep[k] = N_step;	
#ifdef STANDSTILL
		Signal_SupportLeg[k] = DOUBLE_LEG_SP;
#endif
	}

	// if(NULL==(fp=fopen("signal.dat","w")))
	// {
		// printf("signal File can not open!\n");
	// }
	// for(k=0;k<=round(T_com/CONTROL_T);k++)     // COM_t
		// fprintf(fp,"%d\t%d\n",Signal_SupportLeg[k], Signal_NowStep[k]);						
	// fclose(fp);
	
}

/******************************************* Functions For Walking Pattern Generation s ********************************************/

/******************************************* Functions For Trunk Position Control x ********************************************/
Position SingleFootZMP_in_Ankle(const ForceSensor * rfs, const ForceSensor * lfs, enum Support_Leg sup_leg)
{
	Position SingleFootZMP;
	ForceSensor lfs_temp, rfs_temp;
	lfs_temp.fx = 0.0 * lfs->fx;
	lfs_temp.fy = 0.0 * lfs->fy;
	lfs_temp.fz = lfs->fz;
	lfs_temp.tx = lfs->tx;
	lfs_temp.ty = lfs->ty;
	lfs_temp.tz = lfs->tz;

	rfs_temp.fx = 0.0 * rfs->fx;
	rfs_temp.fy = 0.0 * rfs->fy;
	rfs_temp.fz = rfs->fz;
	rfs_temp.tx = rfs->tx;
	rfs_temp.ty = rfs->ty;
	rfs_temp.tz = rfs->tz;

	switch (sup_leg)
	{
	case RLEG_SP:
		lfs_temp.fx = 0.0;
		lfs_temp.fy = 0.0;
		lfs_temp.fz = 0.0;
		lfs_temp.tx = 0.0;
		lfs_temp.ty = 0.0;
		lfs_temp.tz = 0.0;
		break;
	case LLEG_SP:
		rfs_temp.fx = 0.0;
		rfs_temp.fy = 0.0;
		rfs_temp.fz = 0.0;
		rfs_temp.tx = 0.0;
		rfs_temp.ty = 0.0;
		rfs_temp.tz = 0.0;
		break;
	case DOUBLE_LEG_SP:
		break;
	default:
		break;
	}
	if (abs(lfs_temp.fz + rfs_temp.fz)<FZ_MIN_LIMIT)
	{
		SingleFootZMP.px = 0.0;
		SingleFootZMP.py = 0.0;
	}
	else
	{
		SingleFootZMP.px = ((-lfs_temp.ty - P_LFS_LAnkle.pz*lfs_temp.fx + P_LFS_LAnkle.px*lfs_temp.fz) + (-rfs_temp.ty - P_RFS_RAnkle.pz*rfs_temp.fx + P_RFS_RAnkle.px*rfs_temp.fz)) / (lfs_temp.fz + rfs_temp.fz);
		SingleFootZMP.py = ((lfs_temp.tx - P_LFS_LAnkle.pz*lfs_temp.fy + P_LFS_LAnkle.py*lfs_temp.fz) + (rfs_temp.tx - P_RFS_RAnkle.pz*rfs_temp.fy + P_RFS_RAnkle.py*rfs_temp.fz)) / (lfs_temp.fz + rfs_temp.fz);
	}
	SingleFootZMP.pz = -H_ANKLE;
	
	#ifdef USE_ZMPMAPPING
		if(sup_leg == LLEG_SP)
		{
			//printf("%.8f %.8f ", SingleFootZMP.px, SingleFootZMP.py);
			chz_ZMPMapping_map(&SingleFootZMP.px, &SingleFootZMP.py, SingleFootZMP.px, SingleFootZMP.py, 'L');
			//printf("%.8f %.8f\n", SingleFootZMP.px, SingleFootZMP.py);
			if(SingleFootZMP.px >  0.045) SingleFootZMP.px =  0.045;
			if(SingleFootZMP.px < -0.085) SingleFootZMP.px = -0.085;
		}
		else if(sup_leg == RLEG_SP)
		{
			chz_ZMPMapping_map(&SingleFootZMP.px, &SingleFootZMP.py, SingleFootZMP.px, SingleFootZMP.py, 'R');
			//printf("%.8f %.8f\n", SingleFootZMP.px, SingleFootZMP.py);
			if(SingleFootZMP.px >  0.085) SingleFootZMP.px =  0.085;
			if(SingleFootZMP.px < -0.045) SingleFootZMP.px = -0.045;
		}
	if(SingleFootZMP.py > 0.135) SingleFootZMP.py = 0.135;
	if(SingleFootZMP.py < -0.09) SingleFootZMP.py = -0.09;
	#endif
	return SingleFootZMP;
}

Position SingleFootZMP_in_Body(const JointsAngle * real_legjoint, const Position * P_ZMPFoot_Ankle, enum Support_Leg sup_leg)
{
	double T_Ankle_B[4][4];
	double r1, r2, r3, r4, r5, r6;

	Position p_singlefootzmp_b;

	double d01x = ANKLE_WIDTH*0.5;//half hip width 
	double d01z = H_zc - THIGH * 2 - H_ANKLE;//0.088; //0.242;//
	double d34 = THIGH;  //thigh length
	double d45 = THIGH;  //crus length

	if(sup_leg == RLEG_SP)
	{
		r1 = real_legjoint->qr[1];
		r2 = real_legjoint->qr[2];
		r3 = real_legjoint->qr[3];
		r4 = real_legjoint->qr[4];
		r5 = real_legjoint->qr[5];
		r6 = real_legjoint->qr[6];
	}
	// else if(sup_leg == LLEG_SP) 
	{
		r1 = real_legjoint->ql[1];
		r2 = real_legjoint->ql[2];
		r3 = real_legjoint->ql[3];
		r4 = real_legjoint->ql[4];
		r5 = real_legjoint->ql[5];
		r6 = real_legjoint->ql[6];
	}

	d01x = (sup_leg == LLEG_SP) ? (-d01x) : (d01x);

	T_Ankle_B[0][0] = cos(r1)*cos(r2)*cos(r6) - sin(r6)*(cos(r5)*(cos(r4)*(sin(r1)*sin(r3) + cos(r1)*cos(r3)*sin(r2)) + sin(r4)*(cos(r3)*sin(r1) - cos(r1)*sin(r2)*sin(r3))) + sin(r5)*(cos(r4)*(cos(r3)*sin(r1) - cos(r1)*sin(r2)*sin(r3)) - sin(r4)*(sin(r1)*sin(r3) + cos(r1)*cos(r3)*sin(r2))));
	T_Ankle_B[0][1] = sin(r5)*(cos(r4)*(sin(r1)*sin(r3) + cos(r1)*cos(r3)*sin(r2)) + sin(r4)*(cos(r3)*sin(r1) - cos(r1)*sin(r2)*sin(r3))) - cos(r5)*(cos(r4)*(cos(r3)*sin(r1) - cos(r1)*sin(r2)*sin(r3)) - sin(r4)*(sin(r1)*sin(r3) + cos(r1)*cos(r3)*sin(r2)));
	T_Ankle_B[0][2] = cos(r6)*(cos(r5)*(cos(r4)*(sin(r1)*sin(r3) + cos(r1)*cos(r3)*sin(r2)) + sin(r4)*(cos(r3)*sin(r1) - cos(r1)*sin(r2)*sin(r3))) + sin(r5)*(cos(r4)*(cos(r3)*sin(r1) - cos(r1)*sin(r2)*sin(r3)) - sin(r4)*(sin(r1)*sin(r3) + cos(r1)*cos(r3)*sin(r2)))) + cos(r1)*cos(r2)*sin(r6);
	T_Ankle_B[0][3] = d01x - d34*(sin(r1)*sin(r3) + cos(r1)*cos(r3)*sin(r2)) - d45*(cos(r4)*(sin(r1)*sin(r3) + cos(r1)*cos(r3)*sin(r2)) + sin(r4)*(cos(r3)*sin(r1) - cos(r1)*sin(r2)*sin(r3)));

	T_Ankle_B[1][0] = sin(r6)*(cos(r5)*(cos(r4)*(cos(r1)*sin(r3) - cos(r3)*sin(r1)*sin(r2)) + sin(r4)*(cos(r1)*cos(r3) + sin(r1)*sin(r2)*sin(r3))) + sin(r5)*(cos(r4)*(cos(r1)*cos(r3) + sin(r1)*sin(r2)*sin(r3)) - sin(r4)*(cos(r1)*sin(r3) - cos(r3)*sin(r1)*sin(r2)))) + cos(r2)*cos(r6)*sin(r1);
	T_Ankle_B[1][1] = cos(r5)*(cos(r4)*(cos(r1)*cos(r3) + sin(r1)*sin(r2)*sin(r3)) - sin(r4)*(cos(r1)*sin(r3) - cos(r3)*sin(r1)*sin(r2))) - sin(r5)*(cos(r4)*(cos(r1)*sin(r3) - cos(r3)*sin(r1)*sin(r2)) + sin(r4)*(cos(r1)*cos(r3) + sin(r1)*sin(r2)*sin(r3)));
	T_Ankle_B[1][2] = cos(r2)*sin(r1)*sin(r6) - cos(r6)*(cos(r5)*(cos(r4)*(cos(r1)*sin(r3) - cos(r3)*sin(r1)*sin(r2)) + sin(r4)*(cos(r1)*cos(r3) + sin(r1)*sin(r2)*sin(r3))) + sin(r5)*(cos(r4)*(cos(r1)*cos(r3) + sin(r1)*sin(r2)*sin(r3)) - sin(r4)*(cos(r1)*sin(r3) - cos(r3)*sin(r1)*sin(r2))));
	T_Ankle_B[1][3] = d34*(cos(r1)*sin(r3) - cos(r3)*sin(r1)*sin(r2)) + d45*(cos(r4)*(cos(r1)*sin(r3) - cos(r3)*sin(r1)*sin(r2)) + sin(r4)*(cos(r1)*cos(r3) + sin(r1)*sin(r2)*sin(r3)));

	T_Ankle_B[2][0] = -cos(r6)*sin(r2) - cos(r3 + r4 + r5)*cos(r2)*sin(r6);
	T_Ankle_B[2][1] = sin(r3 + r4 + r5)*cos(r2);
	T_Ankle_B[2][2] = cos(r3 + r4 + r5)*cos(r2)*cos(r6) - sin(r2)*sin(r6);
	T_Ankle_B[2][3] = -d01z - d45*(cos(r2)*cos(r3)*cos(r4) - cos(r2)*sin(r3)*sin(r4)) - d34*cos(r2)*cos(r3);

	T_Ankle_B[3][0] = 0.0;
	T_Ankle_B[3][1] = 0.0;
	T_Ankle_B[3][2] = 0.0;
	T_Ankle_B[3][3] = 1.0;

	p_singlefootzmp_b = T_mul_P(T_Ankle_B, P_ZMPFoot_Ankle);
	return p_singlefootzmp_b;
}

Position T_mul_P(const double t[4][4], const Position * Pos)
{
	Position p_out;
	double p_temp[4] = { Pos->px, Pos->py, Pos->pz, 1.0 };
	double p_temp_sum[4];
	int i1, i2;
	for (i1 = 0; i1 < 4; i1++)
	{
		p_temp_sum[i1] = 0.0;
		for (i2 = 0; i2 < 4; i2++)
		{
			p_temp_sum[i1] += t[i1][i2] * p_temp[i2];
		}
	}
	p_out.px = p_temp_sum[0];
	p_out.py = p_temp_sum[1];
	p_out.pz = p_temp_sum[2];
	return p_out;
}


double cal_fz_bias(double delta_fz_zero, int k_pre)
{
	if (k_pre >= 1 && k_pre <= 20)
	{
		delta_fz_zero += (FootFT[1][2] - FootFT[2][2]);
		delta_fz_zero *= 0.5;
		//printf("k");
	}
	
	//printf("%lf\n", delta_fz_zero);
	return delta_fz_zero;
}

Position Calculate_RealZMP_FromForceSensor(const ForceSensor * rfs, const ForceSensor * lfs, const JointsAngle * real_legjoint)
{
	Position p_zmprel_b;
	
	P_ZMPRFoot_RAnkle = SingleFootZMP_in_Ankle(rfs, lfs, RLEG_SP);
	P_ZMPLFoot_LAnkle = SingleFootZMP_in_Ankle(rfs, lfs, LLEG_SP);

	P_ZMPRFoot_B = SingleFootZMP_in_Body(real_legjoint, &P_ZMPRFoot_RAnkle, RLEG_SP);
	P_ZMPLFoot_B = SingleFootZMP_in_Body(real_legjoint, &P_ZMPLFoot_LAnkle, LLEG_SP);
	if (abs(lfs->fz + rfs->fz)<FZ_MIN_LIMIT)
	{
		p_zmprel_b.px = 0.0;
		p_zmprel_b.py = 0.0;
	}
	else
	{
		p_zmprel_b.px = lfs->fz / (lfs->fz + rfs->fz)*P_ZMPLFoot_B.px + rfs->fz / (lfs->fz + rfs->fz)*P_ZMPRFoot_B.px;
		p_zmprel_b.py = lfs->fz / (lfs->fz + rfs->fz)*P_ZMPLFoot_B.py + rfs->fz / (lfs->fz + rfs->fz)*P_ZMPRFoot_B.py;
		//p_zmprel_b.px -= 0.05;
	}
	p_zmprel_b.pz = -H_zc;

	return p_zmprel_b;	
}


Position T_mul_P2(const double t[4][4], const Position * Pos)
{
	Position p_out;
	double p_temp[4] = { Pos->px, Pos->py,Pos->pz,1.0 };
	double p_temp_sum[4];
	int i1, i2;
	for (i1 = 0; i1 < 4; i1++)
	{
		p_temp_sum[i1] = 0.0;
		for (i2 = 0; i2 < 4; i2++)
		{
			p_temp_sum[i1] += t[i1][i2] * p_temp[i2];
		}
	}
	p_out.px = p_temp_sum[0];
	p_out.py = p_temp_sum[1];
	p_out.pz = p_temp_sum[2];
	return p_out;
}

Position RefZMP_in_Body(const Position *p_zmpref_w, const Position *p_comref_w)
{
	double T_World_B[4][4];
	Position p_zmpref_b;

	T_World_B[0][0] = 1.0;
	T_World_B[0][1] = 0.0;
	T_World_B[0][2] = 0.0;
	T_World_B[0][3] = -p_comref_w->px;

	T_World_B[1][0] = 0.0;
	T_World_B[1][1] = 1.0;
	T_World_B[1][2] = 0.0;
	T_World_B[1][3] = -p_comref_w->py;

	T_World_B[2][0] = 0.0;
	T_World_B[2][1] = 0.0;
	T_World_B[2][2] = 1.0;
	T_World_B[2][3] = -p_comref_w->pz;

	T_World_B[3][0] = 0.0;
	T_World_B[3][1] = 0.0;
	T_World_B[3][2] = 0.0;
	T_World_B[3][3] = 1.0;

	p_zmpref_b = T_mul_P2(T_World_B, p_zmpref_w);
	return p_zmpref_b;
}

void Calculate_DetaCOM_FromTPC(const Position *p_zmprel_b, const Position *p_zmpref_b)
{
	int K_TPC_END;
	double Kzero = 1.0;
	K_TPC_END = round( (T_ready + T_step1 + (N_step - 2)*T_step + T_stepn+0.6*T_end) / CONTROL_T);
	if(K_Preview_Con>K_TPC_END) Kzero = 0.0;
		
	double ZMP_Dif_X = p_zmprel_b->px - p_zmpref_b->px;
	double ZMP_Dif_Y = p_zmprel_b->py + 0.0 * 0.002 - p_zmpref_b->py;

	double xk[3] = { 15.8,-54.4,-32.9 };
	double yk[3] = { 15.8,-54.4,-32.9 };
	double T = 0.004;

	//CoM-modification limitation
	double COM_Mod_Limit_X[2] = { -0.036,0.036 }; //Unit: m
	double COM_Mod_Limit_Y[2] = { -0.02,0.02 };

	double x_old, dx_old, ddx_old;
	double y_old, dy_old, ddy_old;
	
	x_old = P_DetaCOM.px;
	dx_old = P_DetaCOM_D.px;
	ddx_old = P_DetaCOM_DD.px;
	y_old = P_DetaCOM.py;
	dy_old = P_DetaCOM_D.py;
	ddy_old = P_DetaCOM_DD.py;
	

	P_DetaCOM_DD.px = Kzero*ALPHA_X*xk[0] * ZMP_Dif_X + xk[1] * P_DetaCOM.px + xk[2] * P_DetaCOM_D.px;
	P_DetaCOM_D.px = P_DetaCOM_D.px + P_DetaCOM_DD.px*T;
	P_DetaCOM.px = P_DetaCOM.px + P_DetaCOM_D.px*T;

	P_DetaCOM_DD.py = Kzero*ALPHA_Y*yk[0] * ZMP_Dif_Y + yk[1] * P_DetaCOM.py + yk[2] * P_DetaCOM_D.py;
	P_DetaCOM_D.py = P_DetaCOM_D.py + P_DetaCOM_DD.py*T;
	P_DetaCOM.py = P_DetaCOM.py + P_DetaCOM_D.py*T;

	if (P_DetaCOM.px < COM_Mod_Limit_X[0])
	{
		P_DetaCOM.px = COM_Mod_Limit_X[0];
		P_DetaCOM_D.px = (P_DetaCOM.px - x_old) / T;
		P_DetaCOM_DD.px = (P_DetaCOM_D.px - dx_old) / T;
	}
	else if (P_DetaCOM.px > COM_Mod_Limit_X[1])
		{
			P_DetaCOM.px = COM_Mod_Limit_X[1];
			P_DetaCOM_D.px = (P_DetaCOM.px - x_old) / T;
			P_DetaCOM_DD.px = (P_DetaCOM_D.px - dx_old) / T;
		}

	if (P_DetaCOM.py < COM_Mod_Limit_Y[0])
	{
		P_DetaCOM.py = COM_Mod_Limit_Y[0];
		P_DetaCOM_D.py = (P_DetaCOM.py - y_old) / T;
		P_DetaCOM_DD.py = (P_DetaCOM_D.py - dy_old) / T;
	}
	else if (P_DetaCOM.py > COM_Mod_Limit_Y[1])
		{
			P_DetaCOM.py = COM_Mod_Limit_Y[1];
			P_DetaCOM_D.py = (P_DetaCOM.py - y_old) / T;
			P_DetaCOM_DD.py = (P_DetaCOM_D.py - dy_old) / T;
		}

	P_DetaCOM.pz = 0.0;
}

/******************************************* Functions For Trunk Position Control s ********************************************/

/******************************************* Functions For Viscoelastic Model based Compliance x ********************************************/
void VMC_Update()
{
	double com_x = Tra_COM.x[K_Preview_Con];
	double com_y = Tra_COM.y[K_Preview_Con];
	double r_ankle_x = Tra_RAnkle.x[K_Preview_Con];
	double r_ankle_y = Tra_RAnkle.y[K_Preview_Con];
	double l_ankle_x = Tra_LAnkle.x[K_Preview_Con];
	double l_ankle_y = Tra_LAnkle.y[K_Preview_Con];
	double weight = 440.0;//59.8*9.8;
	double sr, sl;

	sr = sqrt(pow(r_ankle_x - com_x, 2.0) + pow(r_ankle_y - com_y, 2.0));
	sl = sqrt(pow(l_ankle_x - com_x, 2.0) + pow(l_ankle_y - com_y, 2.0));

	Last_Ref_RForce_Z = Ref_RForce_Z; 
	Last_Ref_LForce_Z = Ref_LForce_Z;

	Last_Rel_RForce_Z = Rel_RForce_Z;
	Last_Rel_LForce_Z = Rel_LForce_Z;

	// Rel_RForce_Z = FootFT[1][2];
	// Rel_LForce_Z = FootFT[2][2];

	switch (Signal_SupportLeg[K_Preview_Con])
	{
	case DOUBLE_LEG_SP: //double support
		Ref_RForce_Z = sl*weight / (sl + sr);
		Ref_LForce_Z = sr*weight / (sl + sr);
		break;

	case RLEG_SP: //right support
		Ref_RForce_Z = weight;
		Ref_LForce_Z = 0.0;
		break;

	case LLEG_SP: //left support
		Ref_RForce_Z = 0.0;
		Ref_LForce_Z = weight;
		break;
	default:
		Ref_RForce_Z = Rel_RForce_Z;
		Ref_LForce_Z = Rel_LForce_Z;
		break;
	}

	Ref_RDiffForce_Z = (Ref_RForce_Z - Last_Ref_RForce_Z) / CONTROL_T;
	Ref_LDiffForce_Z = (Ref_LForce_Z - Last_Ref_LForce_Z) / CONTROL_T;

	Rel_RDiffForce_Z = (Rel_RForce_Z - Last_Rel_RForce_Z) / CONTROL_T;
	Rel_LDiffForce_Z = (Rel_LForce_Z - Last_Rel_LForce_Z) / CONTROL_T;
}

void VMC_Control()
{
	//先右后左
	double k0 = 0.1;//0.2;
	//double k[3] = {-0.0161,80.7020,15.4343};
	//double k[3] = {-0.0161,70.7020,15.4343};
	//double k[3] = {-0.018945, 49.372143, 14.081092};
	double k[4] = { -0.010348,-0.000031 * 0,36.408055,11.281818 };
	double T = CONTROL_T;
	double f_err[2] = { 0.0,0.0 };
	double df_err[2] = { 0.0,0.0 };
	double real_Fz[2];
	double ref_Fz[2];
	double real_dFz[2];
	double ref_dFz[2];

	static double delta_z[2] = { 0.0,0.0 };
	static double d_delta_z[2] = { 0.0,0.0 };
	static double dd_delta_z[2] = { 0.0,0.0 };

	static double last_delta_z[2] = { 0.0,0.0 };
	static double last_d_delta_z[2] = { 0.0,0.0 };
	static double last_dd_delta_z[2] = { 0.0,0.0 };

	double delta_limit[2] = { -0.02,0.02 };

	int i;

	VMC_Start_K = round((T_ready + T_step1) / T);
	VMC_End_K = round((T_ready + T_step1 + (N_step - 2)*T_step + T_stepn) / T);
	if (K_Preview_Con >= VMC_End_K || K_Preview_Con <= VMC_Start_K)
	{
		k0 = 0;
	}
	//printf("k0 = %f\n",k0);

	real_Fz[0]	= Rel_RForce_Z;			real_Fz[1]	= Rel_LForce_Z;
	real_dFz[0] = Rel_RDiffForce_Z;		real_dFz[1] = Rel_LDiffForce_Z;
	ref_Fz[0]	= Ref_RForce_Z;			ref_Fz[1]	= Ref_LForce_Z;
	ref_dFz[0]	= Ref_RDiffForce_Z;		ref_dFz[1]	= Ref_LDiffForce_Z;

	for (i = 0; i<2; i++)
	{
		last_dd_delta_z[i] = dd_delta_z[i];
		last_d_delta_z[i] = d_delta_z[i];
		last_delta_z[i] = delta_z[i];

		f_err[i] = (real_Fz[i] - ref_Fz[i]);
		df_err[i] = (real_dFz[i] - ref_dFz[i]);

		dd_delta_z[i] = -(k0*k[0] * f_err[i] + k0*k[1] * df_err[i] + k[2] * delta_z[i] + k[3] * d_delta_z[i]);
		d_delta_z[i] += dd_delta_z[i] * T;
		delta_z[i] += d_delta_z[i] * T;

		if (delta_z[i] > delta_limit[1])
		{
			delta_z[i] = delta_limit[1];
			d_delta_z[i] = (delta_z[i] - last_delta_z[i]) / T;
			dd_delta_z[i] = (d_delta_z[i] - last_d_delta_z[i]) / T;
		}
		else if (delta_z[i] < delta_limit[0])
		{
			delta_z[i] = delta_limit[0];
			d_delta_z[i] = (delta_z[i] - last_delta_z[i]) / T;
			dd_delta_z[i] = (d_delta_z[i] - last_d_delta_z[i]) / T;
		}
	}

	Deta_RAnkle_Z = delta_z[0];
	Deta_LAnkle_Z = delta_z[1];
}
/******************************************* Functions For Viscoelastic Model based Compliance s ********************************************/

void WaistCompensation_Tra_bhr7test(double per_down, double t_down)
{
	int k, j;
	int k_step;
	double comp_angle = 0;// Bug?
	double first_step_gain; 
	double t_ready_time;    
	double v_dou = 0.003 * comp_angle * 2 / T_dou; // Bug?

	comp_angle = DEG2RAD*1.0;

	// Only for Move Right
	//if (PreCon_Mode == PRECON_MOVE_RIGHT) first_step_gain = 0.1; //0.5;//
	//else first_step_gain = 1.0;
	first_step_gain = 1.0;

	t_ready_time = T_dou;
	for (k = 0; k <= round((T_ready - t_ready_time) / CONTROL_T); k++)
	{
		Comp_Waist_bhr7test[k] = 0.0;
	}
	j = 0;
	TSpline_S_V_A(0, 0, 0, 0, first_step_gain*comp_angle*0.5, t_ready_time*0.5, first_step_gain*comp_angle, 0, 0, t_ready_time, CONTROL_T);
	for (k = round((T_ready - t_ready_time) / CONTROL_T); k <= round(T_ready / CONTROL_T); k++)
	{
		Comp_Waist_bhr7test[k] = S[0][j++];
	}

	/* First Step */
	// single phase 
	for (k = round(T_ready / CONTROL_T); k <= round((T_ready + T_step1*(1 - Pct_dou) * t_down) / CONTROL_T); k++)
	{
		Comp_Waist_bhr7test[k] = first_step_gain*comp_angle;
	}
	j = 0;
	TSpline_S_V_A(first_step_gain*comp_angle, 0, 0, 0, Per_Mid * first_step_gain*comp_angle * (1 + per_down), T_step1*(1 - Pct_dou)*(1 - t_down)*0.5, first_step_gain*comp_angle * per_down, -v_dou, 0, T_step1*(1 - Pct_dou)*(1 - t_down), CONTROL_T);
	for (k = round((T_ready + T_step1*(1 - Pct_dou) * t_down) / CONTROL_T); k <= round((T_ready + T_step1*(1 - Pct_dou)) / CONTROL_T); k++)
	{
		Comp_Waist_bhr7test[k] = S[0][j++];
	}
	// double phase 
	j = 0;
	TSpline_S_V_A(first_step_gain*comp_angle * per_down, -v_dou, 0, 0, 0, T_step1*Pct_dou*0.5, -first_step_gain*comp_angle, 0, 0, T_step1*Pct_dou, CONTROL_T);
	for (k = round((T_ready + T_step1*(1 - Pct_dou)) / CONTROL_T); k <= round((T_ready + T_step1) / CONTROL_T); k++)
	{
		Comp_Waist_bhr7test[k] = S[0][j++];
	}

	/* Second Step --> (N_step-1) Step */
	for (k_step = 1; k_step<N_step - 1; k_step++)
	{
		if (k_step % 2 != 0) // right leg swing  1,3,5...
		{
			// single phase
			for (k = round((T_ready + T_step1 + (k_step - 1)*T_step) / CONTROL_T); k <= round((T_ready + T_step1 + (k_step - 1)*T_step + T_sin * t_down) / CONTROL_T); k++)
			{
				Comp_Waist_bhr7test[k] = -comp_angle;
			}
			j = 0;
			TSpline_S_V_A(-comp_angle, 0, 0, 0, -Per_Mid * comp_angle * (1 + per_down), T_sin*(1 - t_down)*0.5, -comp_angle * per_down, v_dou, 0, T_sin*(1 - t_down), CONTROL_T);
			for (k = round((T_ready + T_step1 + (k_step - 1)*T_step + T_sin * t_down) / CONTROL_T); k <= round((T_ready + T_step1 + (k_step - 1)*T_step + T_sin) / CONTROL_T); k++)
			{
				Comp_Waist_bhr7test[k] = S[0][j++];
			}
			// double phase
			j = 0;
			TSpline_S_V_A(-comp_angle * per_down, v_dou, 0, 0, 0, T_dou*0.5, comp_angle, 0, 0, T_dou, CONTROL_T);
			for (k = round((T_ready + T_step1 + (k_step - 1)*T_step + T_sin) / CONTROL_T); k <= round((T_ready + T_step1 + (k_step - 1)*T_step + T_step) / CONTROL_T); k++)
			{
				Comp_Waist_bhr7test[k] = S[0][j++];
			}
		}

		if (k_step % 2 == 0) // left leg swing   0,2,4...
		{
			// single phase
			for (k = round((T_ready + T_step1 + (k_step - 1)*T_step) / CONTROL_T); k <= round((T_ready + T_step1 + (k_step - 1)*T_step + T_sin * t_down) / CONTROL_T); k++)
			{
				Comp_Waist_bhr7test[k] = comp_angle;
			}
			j = 0;
			TSpline_S_V_A(comp_angle, 0, 0, 0, Per_Mid * comp_angle * (1 + per_down), T_sin*(1 - t_down)*0.5, comp_angle * per_down, -v_dou, 0, T_sin*(1 - t_down), CONTROL_T);
			for (k = round((T_ready + T_step1 + (k_step - 1)*T_step + T_sin * t_down) / CONTROL_T); k <= round((T_ready + T_step1 + (k_step - 1)*T_step + T_sin) / CONTROL_T); k++)
			{
				Comp_Waist_bhr7test[k] = S[0][j++];
			}
			// double phase
			j = 0;
			TSpline_S_V_A(comp_angle * per_down, -v_dou, 0, 0, 0, T_dou*0.5, -comp_angle, 0, 0, T_dou, CONTROL_T);
			for (k = round((T_ready + T_step1 + (k_step - 1)*T_step + T_sin) / CONTROL_T); k <= round((T_ready + T_step1 + (k_step - 1)*T_step + T_step) / CONTROL_T); k++)
			{
				Comp_Waist_bhr7test[k] = S[0][j++];
			}
		}
	}

	/* Last Step */
	if (N_step % 2 == 0)  // right leg swing
	{
		// single phase
		for (k = round((T_ready + T_step1 + (N_step - 2)*T_step) / CONTROL_T); k <= round((T_ready + T_step1 + (N_step - 2)*T_step + T_stepn*(1 - Pct_dou) * t_down) / CONTROL_T); k++)
		{
			Comp_Waist_bhr7test[k] = -comp_angle;
		}
		j = 0;
		TSpline_S_V_A(-comp_angle, 0, 0, 0, -Per_Mid * comp_angle*(1 + per_down), T_stepn*(1 - Pct_dou) * (1 - t_down)*0.5, -comp_angle*per_down, v_dou, 0, T_stepn*(1 - Pct_dou) * (1 - t_down), CONTROL_T);
		for (k = round((T_ready + T_step1 + (N_step - 2)*T_step + T_stepn*(1 - Pct_dou) * t_down) / CONTROL_T); k <= round((T_ready + T_step1 + (N_step - 2)*T_step + T_stepn*(1 - Pct_dou)) / CONTROL_T); k++)
		{
			Comp_Waist_bhr7test[k] = S[0][j++];
		}
		// double phase
		j = 0;
		TSpline_S_V_A(-comp_angle*per_down, v_dou, 0, 0, -comp_angle*per_down*0.5, T_stepn*Pct_dou*0.5, 0, 0, 0, T_stepn*Pct_dou, CONTROL_T);
		for (k = round((T_ready + T_step1 + (N_step - 2)*T_step + T_stepn*(1 - Pct_dou)) / CONTROL_T); k <= round((T_ready + T_step1 + (N_step - 2)*T_step + T_stepn) / CONTROL_T); k++)
		{
			Comp_Waist_bhr7test[k] = S[0][j++];
		}
	}
	else  // left leg swing
	{
		// single phase
		for (k = round((T_ready + T_step1 + (N_step - 2)*T_step) / CONTROL_T); k <= round((T_ready + T_step1 + (N_step - 2)*T_step + T_stepn*(1 - Pct_dou) * t_down) / CONTROL_T); k++)
		{
			Comp_Waist_bhr7test[k] = comp_angle;
		}
		j = 0;
		TSpline_S_V_A(comp_angle, 0, 0, 0, Per_Mid * comp_angle*(1 + per_down), T_stepn*(1 - Pct_dou) * (1 - t_down)*0.5, comp_angle*per_down, -v_dou, 0, T_stepn*(1 - Pct_dou) * (1 - t_down), CONTROL_T);
		for (k = round((T_ready + T_step1 + (N_step - 2)*T_step + T_stepn*(1 - Pct_dou) * t_down) / CONTROL_T); k <= round((T_ready + T_step1 + (N_step - 2)*T_step + T_stepn*(1 - Pct_dou)) / CONTROL_T); k++)
		{
			Comp_Waist_bhr7test[k] = S[0][j++];
		}
		// double phase
		j = 0;
		TSpline_S_V_A(comp_angle*per_down, -v_dou, 0, 0, comp_angle*per_down*0.5, T_stepn*Pct_dou*0.5, 0, 0, 0, T_stepn*Pct_dou, CONTROL_T);
		for (k = round((T_ready + T_step1 + (N_step - 2)*T_step + T_stepn*(1 - Pct_dou)) / CONTROL_T); k <= round((T_ready + T_step1 + (N_step - 2)*T_step + T_stepn) / CONTROL_T); k++)
		{
			Comp_Waist_bhr7test[k] = S[0][j++];
		}
	}

	for (k = round((T_ready + T_step1 + (N_step - 2)*T_step + T_stepn) / CONTROL_T); k <= round(T_com / CONTROL_T); k++)
	{
		Comp_Waist_bhr7test[k] = 0.0;
	}

}

//// in init
//WaistCompensation_Tra_bhr7test(Per_Down, T_Down);
//// after IK
//#ifdef USE_WAIST_COMP_BHR7TEST
//// Right 
//PreCon_LegJoint.qr[2] -= Per_Mid * 1.2 * Comp_Waist_bhr7test[K_Preview_Con];
//PreCon_LegJoint.qr[6] -= 0.3 * 0.3 * Comp_Waist_bhr7test[K_Preview_Con];
//// Left 
//PreCon_LegJoint.ql[2] -= Per_Mid * 1.2 * Comp_Waist_bhr7test[K_Preview_Con];
//PreCon_LegJoint.ql[6] -= 0.3 * 0.3 * Comp_Waist_bhr7test[K_Preview_Con];
//#endif 

void dcc_ArmInit()
{
	PreCon_LegJoint_last.ql[3] = 10 / 57.3;
	PreCon_LegJoint_last.ql[4] = -20 / 57.3;
	PreCon_LegJoint_last.ql[5] = 10 / 57.3;
	PreCon_LegJoint_last.qr[3] = 10 / 57.3;
	PreCon_LegJoint_last.qr[4] = -20 / 57.3;
	PreCon_LegJoint_last.qr[5] = 10 / 57.3;
}

void dcc_ArmSwing()
{
	double K_L = 0.8049;
	double K_P;
	double dql[7], dqr[7];
	double dql_arm, dqr_arm;
	double dql_arm_L, dqr_arm_L;
	double dql_arm_P, dqr_arm_P;
	int i;
	double v_M = 500; //450;
	double L_arm = 0.6;
	double M_arm = 8.0;//12.0
	double L_thigh = 0.3;
	double L_shank = 0.3;
	double M_thigh = 5.0;
	double M_shank = 5.0;
	double M_foot = 1.0;

	K_P = 1.0 - K_L;

	for (i = 1; i < 7; i++)
	{
		dqr[i] = (PreCon_LegJoint.ql[i] - PreCon_LegJoint_last.ql[i]) / CONTROL_T;
		dql[i] = (PreCon_LegJoint.qr[i] - PreCon_LegJoint_last.qr[i]) / CONTROL_T;
	}
	dql_arm_L = 2 * ANKLE_WIDTH / (SHOUDLER_WIDTH * L_arm * (M_arm + v_M)) * ((0.5 * M_thigh + M_shank + M_foot) * L_thigh * dql[3] + (0.5 * M_shank + M_foot) * L_shank * dql[4]);
	dqr_arm_L = 2 * ANKLE_WIDTH / (SHOUDLER_WIDTH * L_arm * (M_arm + v_M)) * ((0.5 * M_thigh + M_shank + M_foot) * L_thigh * dqr[3] + (0.5 * M_shank + M_foot) * L_shank * dqr[4]);
	dqr_arm_P = -2 / (L_arm * (M_arm + v_M)) * ((0.5 * M_thigh + M_shank + M_foot) * L_thigh * dql[3] + (0.5 * M_shank + M_foot) * L_shank * dql[4]);
	dql_arm_P = -2 / (L_arm * (M_arm + v_M)) * ((0.5 * M_thigh + M_shank + M_foot) * L_thigh * dqr[3] + (0.5 * M_shank + M_foot) * L_shank * dqr[4]);

	dql_arm = K_L*dql_arm_L + K_P*dql_arm_P;
	dqr_arm = K_L*dqr_arm_L + K_P*dqr_arm_P;

	dcc_ql_arm = dcc_ql_arm_old + dql_arm;
	dcc_qr_arm = dcc_qr_arm_old + dqr_arm;
	dcc_ql_arm_old = dcc_ql_arm;
	dcc_qr_arm_old = dcc_qr_arm;

	for (i = 1; i < 7; i++)
	{
		PreCon_LegJoint_last.ql[i] = PreCon_LegJoint.ql[i];
		PreCon_LegJoint_last.qr[i] = PreCon_LegJoint.qr[i];
	}
}


//cal_desired_FOOTFT_dcc
void CheckSupLeg(int k_precon)
{
	if (Tra_RAnkle.z[k_precon] > Tra_LAnkle.z[k_precon])
	{
		supleg = L_sup;
	}
	else if (Tra_RAnkle.z[k_precon] < Tra_LAnkle.z[k_precon])
	{
		supleg = R_sup;
	}
	else
	{
		supleg = D_sup;
	}

	if (Tra_RAnkle.y[k_precon] == Tra_LAnkle.y[k_precon])
	{
		//printf("yequl\n");
		if (Tra_RAnkle.z[k_precon] == Tra_LAnkle.z[k_precon])
		{
			flag = S_sup;
			// printf("zequl\n");
		}
	}
}

//input ori joints of L & R and output q dq ddq for sa & la dynamics in right format 
Joints_state LegEmploy()
{

	qR_OOLD.q1 = qR_OLD.q1;
	qR_OOLD.q2 = qR_OLD.q2;
	qR_OOLD.q3 = qR_OLD.q3;
	qR_OOLD.q4 = qR_OLD.q4;
	qR_OOLD.q5 = qR_OLD.q5;
	qR_OOLD.q6 = qR_OLD.q6;
	qL_OOLD.q1 = qL_OLD.q1;
	qL_OOLD.q2 = qL_OLD.q2;
	qL_OOLD.q3 = qL_OLD.q3;
	qL_OOLD.q4 = qL_OLD.q4;
	qL_OOLD.q5 = qL_OLD.q5;
	qL_OOLD.q6 = qL_OLD.q6;

	qR_OLD.q1 = qR.q1;
	qR_OLD.q2 = qR.q2;
	qR_OLD.q3 = qR.q3;
	qR_OLD.q4 = qR.q4;
	qR_OLD.q5 = qR.q5;
	qR_OLD.q6 = qR.q6;
	qL_OLD.q1 = qL.q1;
	qL_OLD.q2 = qL.q2;
	qL_OLD.q3 = qL.q3;
	qL_OLD.q4 = qL.q4;
	qL_OLD.q5 = qL.q5;
	qL_OLD.q6 = qL.q6;

	qR.q1 = PreCon_LegJoint.qr[1];
	qR.q2 = PreCon_LegJoint.qr[2];
	qR.q3 = PreCon_LegJoint.qr[3];
	qR.q4 = PreCon_LegJoint.qr[4];
	qR.q5 = PreCon_LegJoint.qr[5];
	qR.q6 = PreCon_LegJoint.qr[6];
	qL.q1 = PreCon_LegJoint.ql[1];
	qL.q2 = PreCon_LegJoint.ql[2];
	qL.q3 = PreCon_LegJoint.ql[3];
	qL.q4 = PreCon_LegJoint.ql[4];
	qL.q5 = PreCon_LegJoint.ql[5];
	qL.q6 = PreCon_LegJoint.ql[6];

	Joints_state Joints_ret;
	if (supleg == R_sup)
	{
		j_sa.q1 = qR.q5;
		j_sa.q2 = qR.q4;
		j_sa.q3 = qR.q3;
		j_sa.q4 = 0.0;
		j_sa.q5 = 0.0;
		j_sa.q6 = qL.q3;
		j_sa.q7 = qL.q4;
		j_sa.dq1 = (qR.q5 - qR_OLD.q5) / 0.004;
		j_sa.dq2 = (qR.q4 - qR_OLD.q4) / 0.004;
		j_sa.dq3 = (qR.q3 - qR_OLD.q3) / 0.004;
		j_sa.dq4 = 0.0;
		j_sa.dq5 = 0.0;
		j_sa.dq6 = (qL.q3 - qL_OLD.q3) / 0.004;
		j_sa.dq7 = (qL.q4 - qL_OLD.q4) / 0.004;
		j_sa.ddq1 = (qR.q5 - 2 * qR_OLD.q5 + qR_OOLD.q5) / 0.004;
		j_sa.ddq2 = (qR.q4 - 2 * qR_OLD.q4 + qR_OOLD.q4) / 0.004;
		j_sa.ddq3 = (qR.q3 - 2 * qR_OLD.q3 + qR_OOLD.q3) / 0.004;
		j_sa.ddq4 = 0.0;
		j_sa.ddq5 = 0.0;
		j_sa.ddq6 = (qL.q3 - 2 * qL_OLD.q3 + qL_OOLD.q3) / 0.004;
		j_sa.ddq7 = (qL.q4 - 2 * qL_OLD.q4 + qL_OOLD.q4) / 0.004;

		j_la.q1 = qR.q6;
		j_la.q2 = qR.q2;
		j_la.q3 = qL.q2;
		j_la.dq1 = (qR.q6 - qR_OLD.q6) / 0.004;
		j_la.dq2 = (qR.q2 - qR_OLD.q2) / 0.004;
		j_la.dq3 = (qL.q2 - qL_OLD.q2) / 0.004;
		j_la.ddq1 = (qR.q6 - 2 * qR_OLD.q6 + qR_OOLD.q6) / 0.004;
		j_la.ddq2 = (qR.q2 - 2 * qR_OLD.q2 + qR_OOLD.q2) / 0.004;
		j_la.ddq3 = (qL.q2 - 2 * qL_OLD.q2 + qL_OOLD.q2) / 0.004;

		flag = R_sup;
	}
	else if (supleg == L_sup)
	{
		j_sa.q1 = qL.q5;
		j_sa.q2 = qL.q4;
		j_sa.q3 = qL.q3;
		j_sa.q4 = 0.0;
		j_sa.q5 = 0.0;
		j_sa.q6 = qR.q3;
		j_sa.q7 = qR.q4;
		j_sa.dq1 = (qL.q5 - qL_OLD.q5) / 0.004;
		j_sa.dq2 = (qL.q4 - qL_OLD.q4) / 0.004;
		j_sa.dq3 = (qL.q3 - qL_OLD.q3) / 0.004;
		j_sa.dq4 = 0.0;
		j_sa.dq5 = 0.0;
		j_sa.dq6 = (qR.q3 - qR_OLD.q3) / 0.004;
		j_sa.dq7 = (qR.q4 - qR_OLD.q4) / 0.004;
		j_sa.ddq1 = (qL.q5 - 2 * qL_OLD.q5 + qL_OOLD.q5) / 0.004;
		j_sa.ddq2 = (qL.q4 - 2 * qL_OLD.q4 + qL_OOLD.q4) / 0.004;
		j_sa.ddq3 = (qL.q3 - 2 * qL_OLD.q3 + qL_OOLD.q3) / 0.004;
		j_sa.ddq4 = 0.0;
		j_sa.ddq5 = 0.0;
		j_sa.ddq6 = (qR.q3 - 2 * qR_OLD.q3 + qR_OOLD.q3) / 0.004;
		j_sa.ddq7 = (qR.q4 - 2 * qR_OLD.q4 + qR_OOLD.q4) / 0.004;

		j_la.q1 = qL.q6;
		j_la.q2 = qL.q2;
		j_la.q3 = qR.q2;
		j_la.dq1 = (qL.q6 - qL_OLD.q6) / 0.004;
		j_la.dq2 = (qL.q2 - qL_OLD.q2) / 0.004;
		j_la.dq3 = (qR.q2 - qR_OLD.q2) / 0.004;
		j_la.ddq1 = (qL.q6 - 2 * qL_OLD.q6 + qL_OOLD.q6) / 0.004;
		j_la.ddq2 = (qL.q2 - 2 * qL_OLD.q2 + qL_OOLD.q2) / 0.004;
		j_la.ddq3 = (qR.q2 - 2 * qR_OLD.q2 + qR_OOLD.q2) / 0.004;

		flag = L_sup;
	}
	else if (supleg == D_sup)
	{
		if (flag == L_sup)
		{
			j_sa.q1 = qR.q5;
			j_sa.q2 = qR.q4;
			j_sa.q3 = qR.q3;
			j_sa.q4 = 0.0;
			j_sa.q5 = 0.0;
			j_sa.q6 = qL.q3;
			j_sa.q7 = qL.q4;
			j_sa.dq1 = (qR.q5 - qR_OLD.q5) / 0.004;
			j_sa.dq2 = (qR.q4 - qR_OLD.q4) / 0.004;
			j_sa.dq3 = (qR.q3 - qR_OLD.q3) / 0.004;
			j_sa.dq4 = 0.0;
			j_sa.dq5 = 0.0;
			j_sa.dq6 = (qL.q3 - qL_OLD.q3) / 0.004;
			j_sa.dq7 = (qL.q4 - qL_OLD.q4) / 0.004;
			j_sa.ddq1 = (qR.q5 - 2 * qR_OLD.q5 + qR_OOLD.q5) / 0.004;
			j_sa.ddq2 = (qR.q4 - 2 * qR_OLD.q4 + qR_OOLD.q4) / 0.004;
			j_sa.ddq3 = (qR.q3 - 2 * qR_OLD.q3 + qR_OOLD.q3) / 0.004;
			j_sa.ddq4 = 0.0;
			j_sa.ddq5 = 0.0;
			j_sa.ddq6 = (qL.q3 - 2 * qL_OLD.q3 + qL_OOLD.q3) / 0.004;
			j_sa.ddq7 = (qL.q4 - 2 * qL_OLD.q4 + qL_OOLD.q4) / 0.004;

			j_la.q1 = qR.q6;
			j_la.q2 = qR.q2;
			j_la.q3 = qL.q2;
			j_la.dq1 = (qR.q6 - qR_OLD.q6) / 0.004;
			j_la.dq2 = (qR.q2 - qR_OLD.q2) / 0.004;
			j_la.dq3 = (qL.q2 - qL_OLD.q2) / 0.004;
			j_la.ddq1 = (qR.q6 - 2 * qR_OLD.q6 + qR_OOLD.q6) / 0.004;
			j_la.ddq2 = (qR.q2 - 2 * qR_OLD.q2 + qR_OOLD.q2) / 0.004;
			j_la.ddq3 = (qL.q2 - 2 * qL_OLD.q2 + qL_OOLD.q2) / 0.004;
		}
		else if (flag == R_sup)
		{
			j_sa.q1 = qL.q5;
			j_sa.q2 = qL.q4;
			j_sa.q3 = qL.q3;
			j_sa.q4 = 0.0;
			j_sa.q5 = 0.0;
			j_sa.q6 = qR.q3;
			j_sa.q7 = qR.q4;
			j_sa.dq1 = (qL.q5 - qL_OLD.q5) / 0.004;
			j_sa.dq2 = (qL.q4 - qL_OLD.q4) / 0.004;
			j_sa.dq3 = (qL.q3 - qL_OLD.q3) / 0.004;
			j_sa.dq4 = 0.0;
			j_sa.dq5 = 0.0;
			j_sa.dq6 = (qR.q3 - qR_OLD.q3) / 0.004;
			j_sa.dq7 = (qR.q4 - qR_OLD.q4) / 0.004;
			j_sa.ddq1 = (qL.q5 - 2 * qL_OLD.q5 + qL_OOLD.q5) / 0.004;
			j_sa.ddq2 = (qL.q4 - 2 * qL_OLD.q4 + qL_OOLD.q4) / 0.004;
			j_sa.ddq3 = (qL.q3 - 2 * qL_OLD.q3 + qL_OOLD.q3) / 0.004;
			j_sa.ddq4 = 0.0;
			j_sa.ddq5 = 0.0;
			j_sa.ddq6 = (qR.q3 - 2 * qR_OLD.q3 + qR_OOLD.q3) / 0.004;
			j_sa.ddq7 = (qR.q4 - 2 * qR_OLD.q4 + qR_OOLD.q4) / 0.004;

			j_la.q1 = qL.q6;
			j_la.q2 = qL.q2;
			j_la.q3 = qR.q2;
			j_la.dq1 = (qL.q6 - qL_OLD.q6) / 0.004;
			j_la.dq2 = (qL.q2 - qL_OLD.q2) / 0.004;
			j_la.dq3 = (qR.q2 - qR_OLD.q2) / 0.004;
			j_la.ddq1 = (qL.q6 - 2 * qL_OLD.q6 + qL_OOLD.q6) / 0.004;
			j_la.ddq2 = (qL.q2 - 2 * qL_OLD.q2 + qL_OOLD.q2) / 0.004;
			j_la.ddq3 = (qR.q2 - 2 * qR_OLD.q2 + qR_OOLD.q2) / 0.004;
		}
		else
		{
			//printf("%d %d LegEmploy false in D\n", supleg, flag);
		}
	}
	else if (supleg == S_sup)
	{
		j_sa.q1 = qL.q5;
		j_sa.q2 = qL.q4;
		j_sa.q3 = qL.q3;
		j_sa.q4 = 0.0;
		j_sa.q5 = 0.0;
		j_sa.q6 = qR.q3;
		j_sa.q7 = qR.q4;
		j_sa.dq1 = (qL.q5 - qL_OLD.q5) / 0.004;
		j_sa.dq2 = (qL.q4 - qL_OLD.q4) / 0.004;
		j_sa.dq3 = (qL.q3 - qL_OLD.q3) / 0.004;
		j_sa.dq4 = 0.0;
		j_sa.dq5 = 0.0;
		j_sa.dq6 = (qR.q3 - qR_OLD.q3) / 0.004;
		j_sa.dq7 = (qR.q4 - qR_OLD.q4) / 0.004;
		j_sa.ddq1 = (qL.q5 - 2 * qL_OLD.q5 + qL_OOLD.q5) / 0.004;
		j_sa.ddq2 = (qL.q4 - 2 * qL_OLD.q4 + qL_OOLD.q4) / 0.004;
		j_sa.ddq3 = (qL.q3 - 2 * qL_OLD.q3 + qL_OOLD.q3) / 0.004;
		j_sa.ddq4 = 0.0;
		j_sa.ddq5 = 0.0;
		j_sa.ddq6 = (qR.q3 - 2 * qR_OLD.q3 + qR_OOLD.q3) / 0.004;
		j_sa.ddq7 = (qR.q4 - 2 * qR_OLD.q4 + qR_OOLD.q4) / 0.004;

		j_la.q1 = qL.q6;
		j_la.q2 = qL.q2;
		j_la.q3 = qR.q2;
		j_la.dq1 = (qL.q6 - qL_OLD.q6) / 0.004;
		j_la.dq2 = (qL.q2 - qL_OLD.q2) / 0.004;
		j_la.dq3 = (qR.q2 - qR_OLD.q2) / 0.004;
		j_la.ddq1 = (qL.q6 - 2 * qL_OLD.q6 + qL_OOLD.q6) / 0.004;
		j_la.ddq2 = (qL.q2 - 2 * qL_OLD.q2 + qL_OOLD.q2) / 0.004;
		j_la.ddq3 = (qR.q2 - 2 * qR_OLD.q2 + qR_OOLD.q2) / 0.004;
	}
	else
	{
		printf("%d LegEmploy false\n", supleg);
	}

	Joints_ret.la = j_la;
	Joints_ret.sa = j_sa;
	return Joints_ret;
}

F_struct Cal_Desired_Fext(double *msa, double *acz, double *pcy, double *acy, double *pcz, double com_xi, double k)
{
	double g = 9.8;
	double Dphase_adj = 3.5;
	theta = atan(Waist / L_step);
	F_struct cal_F_desired_ret;
	if (supleg == L_sup)
	{
		cal_F_desired_ret.Lfoot = acz[1] * msa[1] + acz[2] * msa[2] + acz[3] * msa[3] + acz[4] * msa[4] + acz[5] * msa[5] + acz[6] * msa[6] + acz[7] * msa[7] + acz[7] * msa[8] + g*(msa[1] + msa[2] + msa[3] + msa[4] + msa[5] + msa[6] + msa[7] + msa[8]);
		cal_F_desired_ret.Rfoot = -acz[7] * msa[8];
	}
	else if (supleg == R_sup)
	{
		cal_F_desired_ret.Rfoot = acz[1] * msa[1] + acz[2] * msa[2] + acz[3] * msa[3] + acz[4] * msa[4] + acz[5] * msa[5] + acz[6] * msa[6] + acz[7] * msa[7] + acz[7] * msa[8] + g*(msa[1] + msa[2] + msa[3] + msa[4] + msa[5] + msa[6] + msa[7] + msa[8]);
		cal_F_desired_ret.Lfoot = -acz[7] * msa[8];
	}
	else if (supleg == D_sup)
	{
		if (flag == L_sup)
		{
			cal_F_desired_ret.Lfoot = -g*(msa[1] * pcy[1] + msa[2] * pcy[2] + msa[3] * pcy[3] + msa[4] * pcy[4] + msa[5] * pcy[5] + msa[6] * pcy[6] + msa[7] * pcy[7] + msa[8] * pcy[8]) / L_step;
			cal_F_desired_ret.Rfoot = acz[1] * msa[1] + acz[2] * msa[2] + acz[3] * msa[3] + acz[4] * msa[4] + acz[5] * msa[5] + acz[6] * msa[6] + acz[7] * msa[7] + acz[7] * msa[8] + g*(msa[1] + msa[2] + msa[3] + msa[4] + msa[5] + msa[6] + msa[7] + msa[8]) - cal_F_desired_ret.Lfoot;
		}
		else if (flag == R_sup)
		{
			cal_F_desired_ret.Rfoot = -g*(msa[1] * pcy[1] + msa[2] * pcy[2] + msa[3] * pcy[3] + msa[4] * pcy[4] + msa[5] * pcy[5] + msa[6] * pcy[6] + msa[7] * pcy[7] + msa[8] * pcy[8]) / L_step;
			cal_F_desired_ret.Lfoot = acz[1] * msa[1] + acz[2] * msa[2] + acz[3] * msa[3] + acz[4] * msa[4] + acz[5] * msa[5] + acz[6] * msa[6] + acz[7] * msa[7] + acz[7] * msa[8] + g*(msa[1] + msa[2] + msa[3] + msa[4] + msa[5] + msa[6] + msa[7] + msa[8]) - cal_F_desired_ret.Rfoot;
		}
		else if (flag == S_sup)
		{
			cal_F_desired_ret.Lfoot = ((Waist / 2 - com_xi)*(g*(msa[1] + msa[2] + msa[3] + msa[4] + msa[5] + msa[6] + msa[7] + msa[8]))) / Waist;
			cal_F_desired_ret.Rfoot = ((Waist / 2 + com_xi)*(g*(msa[1] + msa[2] + msa[3] + msa[4] + msa[5] + msa[6] + msa[7] + msa[8]))) / Waist;
			//printf("%f", com_xi);
		}
		else
		{
			printf("%d false in D", flag);
		}

	}

	if (cal_F_desired_ret.Lfoot > 800)cal_F_desired_ret.Lfoot = 800.0;
	if (cal_F_desired_ret.Lfoot <-800)cal_F_desired_ret.Lfoot = -800.0;
	if (cal_F_desired_ret.Rfoot > 800)cal_F_desired_ret.Rfoot = 800.0;
	if (cal_F_desired_ret.Rfoot <-800)cal_F_desired_ret.Rfoot = -800.0;

	if (k <= 1)
	{
		cal_F_desired_ret.Lfoot = ((Waist / 2)*(g*(msa[1] + msa[2] + msa[3] + msa[4] + msa[5] + msa[6] + msa[7] + msa[8]))) / Waist;
		cal_F_desired_ret.Rfoot = ((Waist / 2)*(g*(msa[1] + msa[2] + msa[3] + msa[4] + msa[5] + msa[6] + msa[7] + msa[8]))) / Waist;
	}
	return cal_F_desired_ret;
}

Tau_struct Cal_Desired_Tau(double Taux, double TauyR_sup, double TauyL_sup, double kL, double kR, double k)
{
	Tau_struct cal_Tau_desired_ret;
	if (supleg == L_sup)
	{
		cal_Tau_desired_ret.Lfoot.x = Taux;
		cal_Tau_desired_ret.Rfoot.x = 0.0;
		cal_Tau_desired_ret.Lfoot.y = TauyL_sup;
		cal_Tau_desired_ret.Rfoot.y = 0.0;
	}
	else if (supleg == R_sup)
	{
		cal_Tau_desired_ret.Lfoot.x = 0.0;
		cal_Tau_desired_ret.Rfoot.x = Taux;
		cal_Tau_desired_ret.Lfoot.y = 0.0;
		cal_Tau_desired_ret.Rfoot.y = TauyR_sup;
	}
	else if (supleg == D_sup)
	{
		if (flag == L_sup)
		{
			cal_Tau_desired_ret.Lfoot.x = Taux;
			cal_Tau_desired_ret.Rfoot.x = 0.0;
			cal_Tau_desired_ret.Lfoot.y = kL * TauyR_sup;
			cal_Tau_desired_ret.Rfoot.y = kR * TauyR_sup;
		}
		else if (flag == R_sup)
		{
			cal_Tau_desired_ret.Lfoot.x = 0.0;
			cal_Tau_desired_ret.Rfoot.x = Taux;
			cal_Tau_desired_ret.Lfoot.y = kL * TauyL_sup;
			cal_Tau_desired_ret.Rfoot.y = kR * TauyL_sup;
		}
		else if (flag == S_sup)
		{
			cal_Tau_desired_ret.Lfoot.x = 0.0 * Taux;
			cal_Tau_desired_ret.Rfoot.x = 0.0 * Taux;
			cal_Tau_desired_ret.Lfoot.y = 0.0;
			cal_Tau_desired_ret.Rfoot.y = 0.0;
		}
	}
	if (k <= 2)
	{
		cal_Tau_desired_ret.Lfoot.x = 0.0;
		cal_Tau_desired_ret.Rfoot.x = 0.0;
		cal_Tau_desired_ret.Lfoot.y = 0.0;
		cal_Tau_desired_ret.Rfoot.y = 0.0;
	}
	return cal_Tau_desired_ret;
}

void Cal_Desired_FootFT(int kprecon)
{
	double kL;
	double kR;
	double taux_sup;
	double tauyR_sup;
	double tauyL_sup;
	double g = 9.8;

	msa[1] = m_robot / 39.0*5.0;
	msa[2] = m_robot / 39.0*5.0;
	msa[3] = m_robot / 39.0*15.0;
	msa[4] = m_robot / 39.0*1.5;
	msa[5] = m_robot / 39.0*1.5;
	msa[6] = m_robot / 39.0*5.0;
	msa[7] = m_robot / 39.0*5.0;
	msa[8] = m_robot / 39.0*1.0;
	Isa[1] = (msa[1] * Thigh * Thigh / 12);
	Isa[2] = (msa[2] * Shank * Shank / 12);
	Isa[3] = (msa[3] * Body * Body / 12);
	Isa[4] = (msa[4] * Arm * Arm / 12);
	Isa[5] = (msa[5] * Arm * Arm / 12);
	Isa[6] = (msa[6] * Thigh * Thigh / 12);
	Isa[7] = (msa[7] * Shank * Shank / 12);
	Isa[8] = 0.0;
	mla[1] = 1.25*10.5;
	mla[2] = 1.25*13.7;
	mla[3] = 1.25*10.5;
	mla[4] = 1.25*1.0;
	Isa[1] = (mla[1] * (Thigh + Shank) * (Thigh + Shank) / 12);
	Isa[2] = (mla[2] * Body * Body / 12);
	Isa[3] = (mla[3] * (Thigh + Shank) * (Thigh + Shank) / 12);
	Isa[4] = 0.0;

	Joints_state Joints;

	CheckSupLeg(kprecon);
	//input ori joints of L & R and output q dq ddq for sa & la dynamics in right format 
	Joints = LegEmploy();

	//cal p & c for sa
	q5sup = Joints.sa.q1;
	q4sup = Joints.sa.q2;
	q3sup = Joints.sa.q3;
	qasup = Joints.sa.q4;
	qaswi = Joints.sa.q5;
	q3swi = Joints.sa.q6;
	q4swi = Joints.sa.q7;
	dq5sup = Joints.sa.dq1;
	dq4sup = Joints.sa.dq2;
	dq3sup = Joints.sa.dq3;
	dqasup = Joints.sa.dq4;
	dqaswi = Joints.sa.dq5;
	dq3swi = Joints.sa.dq6;
	dq4swi = Joints.sa.dq7;
	ddq5sup = Joints.sa.ddq1;
	ddq4sup = Joints.sa.ddq2;
	ddq3sup = Joints.sa.ddq3;
	ddqasup = Joints.sa.ddq4;
	ddqaswi = Joints.sa.ddq5;
	ddq3swi = Joints.sa.ddq6;
	ddq4swi = Joints.sa.ddq7;
	pcy[1] = (33 * sin(q5sup)) / 200;
	pcy[2] = (33 * sin(q5sup)) / 100 + (33 * cos(q4sup)*sin(q5sup)) / 200 + (33 * cos(q5sup)*sin(q4sup)) / 200;
	pcy[3] = (33 * sin(q5sup)) / 100 + (33 * cos(q4sup)*sin(q5sup)) / 100 + (33 * cos(q5sup)*sin(q4sup)) / 100 + (11 * cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) / 40 + (11 * sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) / 40;
	pcy[4] = (33 * sin(q5sup)) / 100 + (33 * cos(q4sup)*sin(q5sup)) / 100 + (33 * cos(q5sup)*sin(q4sup)) / 100 - (cos(qasup)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4 - (sin(qasup)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4 + (11 * cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) / 20 + (11 * sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) / 20;
	pcy[5] = (33 * sin(q5sup)) / 100 + (33 * cos(q4sup)*sin(q5sup)) / 100 + (33 * cos(q5sup)*sin(q4sup)) / 100 - (cos(qaswi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4 - (sin(qaswi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4 + (11 * cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) / 20 + (11 * sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) / 20;
	pcy[6] = (33 * sin(q5sup)) / 100 + (33 * cos(q4sup)*sin(q5sup)) / 100 + (33 * cos(q5sup)*sin(q4sup)) / 100 - (33 * cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 200 + (33 * sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 200;
	pcy[7] = (33 * sin(q5sup)) / 100 + (33 * sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))) / 200 + (33 * cos(q4sup)*sin(q5sup)) / 100 + (33 * cos(q5sup)*sin(q4sup)) / 100 - (33 * cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 100 + (33 * sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 100 - (33 * cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))) / 200;
	pcy[8] = (33 * sin(q5sup)) / 100 + (33 * sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))) / 100 + (33 * cos(q4sup)*sin(q5sup)) / 100 + (33 * cos(q5sup)*sin(q4sup)) / 100 - (33 * cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 100 + (33 * sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 100 - (33 * cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))) / 100;
	pcz[1] = (33 * cos(q5sup)) / 200;
	pcz[2] = (33 * cos(q5sup)) / 100 + (33 * cos(q4sup)*cos(q5sup)) / 200 - (33 * sin(q4sup)*sin(q5sup)) / 200;
	pcz[3] = (33 * cos(q5sup)) / 100 + (33 * cos(q4sup)*cos(q5sup)) / 100 - (33 * sin(q4sup)*sin(q5sup)) / 100 + (11 * cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) / 40 - (11 * sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) / 40;
	pcz[4] = (33 * cos(q5sup)) / 100 + (33 * cos(q4sup)*cos(q5sup)) / 100 - (33 * sin(q4sup)*sin(q5sup)) / 100 - (cos(qasup)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4 + (sin(qasup)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4 + (11 * cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) / 20 - (11 * sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) / 20;
	pcz[5] = (33 * cos(q5sup)) / 100 + (33 * cos(q4sup)*cos(q5sup)) / 100 - (33 * sin(q4sup)*sin(q5sup)) / 100 - (cos(qaswi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4 + (sin(qaswi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4 + (11 * cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) / 20 - (11 * sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) / 20;
	pcz[6] = (33 * cos(q5sup)) / 100 + (33 * cos(q4sup)*cos(q5sup)) / 100 - (33 * sin(q4sup)*sin(q5sup)) / 100 - (33 * cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 200 - (33 * sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 200;
	pcz[7] = (33 * cos(q5sup)) / 100 + (33 * cos(q4sup)*cos(q5sup)) / 100 - (33 * sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))) / 200 - (33 * sin(q4sup)*sin(q5sup)) / 100 - (33 * cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 100 - (33 * sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 100 - (33 * cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))) / 200;
	pcz[8] = (33 * cos(q5sup)) / 100 + (33 * cos(q4sup)*cos(q5sup)) / 100 - (33 * sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))) / 100 - (33 * sin(q4sup)*sin(q5sup)) / 100 - (33 * cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 100 - (33 * sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 100 - (33 * cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))) / 100;

	acy[1] = (dq5sup*dq5sup)*sin(q5sup)*(-3.3E+1 / 2.0E+2) + ddq5sup*cos(q5sup)*(3.3E+1 / 2.0E+2);
	acy[2] = -dq5sup*(dq4sup*(cos(q4sup)*sin(q5sup)*(3.3E+1 / 2.0E+2) + cos(q5sup)*sin(q4sup)*(3.3E+1 / 2.0E+2)) + dq5sup*(sin(q5sup)*(3.3E+1 / 1.0E+2) + cos(q4sup)*sin(q5sup)*(3.3E+1 / 2.0E+2) + cos(q5sup)*sin(q4sup)*(3.3E+1 / 2.0E+2))) + ddq4sup*(cos(q4sup)*cos(q5sup)*(3.3E+1 / 2.0E+2) - sin(q4sup)*sin(q5sup)*(3.3E+1 / 2.0E+2)) + ddq5sup*(cos(q5sup)*(3.3E+1 / 1.0E+2) + cos(q4sup)*cos(q5sup)*(3.3E+1 / 2.0E+2) - sin(q4sup)*sin(q5sup)*(3.3E+1 / 2.0E+2)) - dq4sup*(dq4sup*(cos(q4sup)*sin(q5sup)*(3.3E+1 / 2.0E+2) + cos(q5sup)*sin(q4sup)*(3.3E+1 / 2.0E+2)) + dq5sup*(cos(q4sup)*sin(q5sup)*(3.3E+1 / 2.0E+2) + cos(q5sup)*sin(q4sup)*(3.3E+1 / 2.0E+2)));
	acy[3] = ddq5sup*(cos(q5sup)*(3.3E+1 / 1.0E+2) + cos(q4sup)*cos(q5sup)*(3.3E+1 / 1.0E+2) - sin(q4sup)*sin(q5sup)*(3.3E+1 / 1.0E+2) + cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))*(1.1E+1 / 4.0E+1) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))*(1.1E+1 / 4.0E+1)) - dq3sup*(dq3sup*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))*(1.1E+1 / 4.0E+1) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))*(1.1E+1 / 4.0E+1)) + dq4sup*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))*(1.1E+1 / 4.0E+1) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))*(1.1E+1 / 4.0E+1)) + dq5sup*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))*(1.1E+1 / 4.0E+1) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))*(1.1E+1 / 4.0E+1))) - dq4sup*(dq3sup*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))*(1.1E+1 / 4.0E+1) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))*(1.1E+1 / 4.0E+1)) + dq4sup*(cos(q4sup)*sin(q5sup)*(3.3E+1 / 1.0E+2) + cos(q5sup)*sin(q4sup)*(3.3E+1 / 1.0E+2) + cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))*(1.1E+1 / 4.0E+1) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))*(1.1E+1 / 4.0E+1)) + dq5sup*(cos(q4sup)*sin(q5sup)*(3.3E+1 / 1.0E+2) + cos(q5sup)*sin(q4sup)*(3.3E+1 / 1.0E+2) + cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))*(1.1E+1 / 4.0E+1) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))*(1.1E+1 / 4.0E+1))) + ddq3sup*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))*(1.1E+1 / 4.0E+1) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))*(1.1E+1 / 4.0E+1)) + ddq4sup*(cos(q4sup)*cos(q5sup)*(3.3E+1 / 1.0E+2) - sin(q4sup)*sin(q5sup)*(3.3E+1 / 1.0E+2) + cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))*(1.1E+1 / 4.0E+1) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))*(1.1E+1 / 4.0E+1)) - dq5sup*(dq5sup*(sin(q5sup)*(3.3E+1 / 1.0E+2) + cos(q4sup)*sin(q5sup)*(3.3E+1 / 1.0E+2) + cos(q5sup)*sin(q4sup)*(3.3E+1 / 1.0E+2) + cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))*(1.1E+1 / 4.0E+1) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))*(1.1E+1 / 4.0E+1)) + dq3sup*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))*(1.1E+1 / 4.0E+1) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))*(1.1E+1 / 4.0E+1)) + dq4sup*(cos(q4sup)*sin(q5sup)*(3.3E+1 / 1.0E+2) + cos(q5sup)*sin(q4sup)*(3.3E+1 / 1.0E+2) + cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))*(1.1E+1 / 4.0E+1) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))*(1.1E+1 / 4.0E+1)));
	acy[4] = dq3sup*(dq3sup*((cos(qasup)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4.0 + (sin(qasup)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4.0 - cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))*(1.1E+1 / 2.0E+1) - sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))*(1.1E+1 / 2.0E+1)) + dq4sup*((cos(qasup)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4.0 + (sin(qasup)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4.0 - cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))*(1.1E+1 / 2.0E+1) - sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))*(1.1E+1 / 2.0E+1)) + dq5sup*((cos(qasup)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4.0 + (sin(qasup)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4.0 - cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))*(1.1E+1 / 2.0E+1) - sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))*(1.1E+1 / 2.0E+1)) + dqasup*((cos(qasup)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4.0 + (sin(qasup)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4.0)) - ddq3sup*((cos(qasup)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4.0 - (sin(qasup)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4.0 - cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))*(1.1E+1 / 2.0E+1) + sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))*(1.1E+1 / 2.0E+1)) + dqasup*(dq3sup*((cos(qasup)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4.0 + (sin(qasup)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4.0) + dq4sup*((cos(qasup)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4.0 + (sin(qasup)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4.0) + dq5sup*((cos(qasup)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4.0 + (sin(qasup)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4.0) + dqasup*((cos(qasup)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4.0 + (sin(qasup)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4.0)) + ddq4sup*(cos(q4sup)*cos(q5sup)*(3.3E+1 / 1.0E+2) - sin(q4sup)*sin(q5sup)*(3.3E+1 / 1.0E+2) - (cos(qasup)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4.0 + (sin(qasup)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4.0 + cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))*(1.1E+1 / 2.0E+1) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))*(1.1E+1 / 2.0E+1)) + dq4sup*(dq3sup*((cos(qasup)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4.0 + (sin(qasup)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4.0 - cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))*(1.1E+1 / 2.0E+1) - sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))*(1.1E+1 / 2.0E+1)) - dq4sup*(cos(q4sup)*sin(q5sup)*(3.3E+1 / 1.0E+2) + cos(q5sup)*sin(q4sup)*(3.3E+1 / 1.0E+2) - (cos(qasup)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4.0 - (sin(qasup)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4.0 + cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))*(1.1E+1 / 2.0E+1) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))*(1.1E+1 / 2.0E+1)) - dq5sup*(cos(q4sup)*sin(q5sup)*(3.3E+1 / 1.0E+2) + cos(q5sup)*sin(q4sup)*(3.3E+1 / 1.0E+2) - (cos(qasup)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4.0 - (sin(qasup)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4.0 + cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))*(1.1E+1 / 2.0E+1) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))*(1.1E+1 / 2.0E+1)) + dqasup*((cos(qasup)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4.0 + (sin(qasup)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4.0)) + ddq5sup*(cos(q5sup)*(3.3E+1 / 1.0E+2) + cos(q4sup)*cos(q5sup)*(3.3E+1 / 1.0E+2) - sin(q4sup)*sin(q5sup)*(3.3E+1 / 1.0E+2) - (cos(qasup)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4.0 + (sin(qasup)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4.0 + cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))*(1.1E+1 / 2.0E+1) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))*(1.1E+1 / 2.0E+1)) - ddqasup*((cos(qasup)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4.0 - (sin(qasup)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4.0) + dq5sup*(dq3sup*((cos(qasup)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4.0 + (sin(qasup)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4.0 - cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))*(1.1E+1 / 2.0E+1) - sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))*(1.1E+1 / 2.0E+1)) - dq4sup*(cos(q4sup)*sin(q5sup)*(3.3E+1 / 1.0E+2) + cos(q5sup)*sin(q4sup)*(3.3E+1 / 1.0E+2) - (cos(qasup)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4.0 - (sin(qasup)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4.0 + cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))*(1.1E+1 / 2.0E+1) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))*(1.1E+1 / 2.0E+1)) + dqasup*((cos(qasup)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4.0 + (sin(qasup)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4.0) - dq5sup*(sin(q5sup)*(3.3E+1 / 1.0E+2) + cos(q4sup)*sin(q5sup)*(3.3E+1 / 1.0E+2) + cos(q5sup)*sin(q4sup)*(3.3E+1 / 1.0E+2) - (cos(qasup)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4.0 - (sin(qasup)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4.0 + cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))*(1.1E+1 / 2.0E+1) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))*(1.1E+1 / 2.0E+1)));
	acy[5] = dq3sup*(dq3sup*((cos(qaswi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4.0 + (sin(qaswi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4.0 - cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))*(1.1E+1 / 2.0E+1) - sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))*(1.1E+1 / 2.0E+1)) + dq4sup*((cos(qaswi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4.0 + (sin(qaswi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4.0 - cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))*(1.1E+1 / 2.0E+1) - sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))*(1.1E+1 / 2.0E+1)) + dq5sup*((cos(qaswi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4.0 + (sin(qaswi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4.0 - cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))*(1.1E+1 / 2.0E+1) - sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))*(1.1E+1 / 2.0E+1)) + dqaswi*((cos(qaswi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4.0 + (sin(qaswi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4.0)) - ddq3sup*((cos(qaswi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4.0 - (sin(qaswi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4.0 - cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))*(1.1E+1 / 2.0E+1) + sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))*(1.1E+1 / 2.0E+1)) + dqaswi*(dq3sup*((cos(qaswi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4.0 + (sin(qaswi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4.0) + dq4sup*((cos(qaswi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4.0 + (sin(qaswi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4.0) + dq5sup*((cos(qaswi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4.0 + (sin(qaswi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4.0) + dqaswi*((cos(qaswi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4.0 + (sin(qaswi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4.0)) + ddq4sup*(cos(q4sup)*cos(q5sup)*(3.3E+1 / 1.0E+2) - sin(q4sup)*sin(q5sup)*(3.3E+1 / 1.0E+2) - (cos(qaswi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4.0 + (sin(qaswi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4.0 + cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))*(1.1E+1 / 2.0E+1) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))*(1.1E+1 / 2.0E+1)) + dq4sup*(dq3sup*((cos(qaswi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4.0 + (sin(qaswi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4.0 - cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))*(1.1E+1 / 2.0E+1) - sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))*(1.1E+1 / 2.0E+1)) - dq4sup*(cos(q4sup)*sin(q5sup)*(3.3E+1 / 1.0E+2) + cos(q5sup)*sin(q4sup)*(3.3E+1 / 1.0E+2) - (cos(qaswi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4.0 - (sin(qaswi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4.0 + cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))*(1.1E+1 / 2.0E+1) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))*(1.1E+1 / 2.0E+1)) - dq5sup*(cos(q4sup)*sin(q5sup)*(3.3E+1 / 1.0E+2) + cos(q5sup)*sin(q4sup)*(3.3E+1 / 1.0E+2) - (cos(qaswi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4.0 - (sin(qaswi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4.0 + cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))*(1.1E+1 / 2.0E+1) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))*(1.1E+1 / 2.0E+1)) + dqaswi*((cos(qaswi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4.0 + (sin(qaswi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4.0)) + ddq5sup*(cos(q5sup)*(3.3E+1 / 1.0E+2) + cos(q4sup)*cos(q5sup)*(3.3E+1 / 1.0E+2) - sin(q4sup)*sin(q5sup)*(3.3E+1 / 1.0E+2) - (cos(qaswi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4.0 + (sin(qaswi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4.0 + cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))*(1.1E+1 / 2.0E+1) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))*(1.1E+1 / 2.0E+1)) - ddqaswi*((cos(qaswi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4.0 - (sin(qaswi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4.0) + dq5sup*(dq3sup*((cos(qaswi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4.0 + (sin(qaswi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4.0 - cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))*(1.1E+1 / 2.0E+1) - sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))*(1.1E+1 / 2.0E+1)) - dq4sup*(cos(q4sup)*sin(q5sup)*(3.3E+1 / 1.0E+2) + cos(q5sup)*sin(q4sup)*(3.3E+1 / 1.0E+2) - (cos(qaswi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4.0 - (sin(qaswi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4.0 + cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))*(1.1E+1 / 2.0E+1) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))*(1.1E+1 / 2.0E+1)) + dqaswi*((cos(qaswi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4.0 + (sin(qaswi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4.0) - dq5sup*(sin(q5sup)*(3.3E+1 / 1.0E+2) + cos(q4sup)*sin(q5sup)*(3.3E+1 / 1.0E+2) + cos(q5sup)*sin(q4sup)*(3.3E+1 / 1.0E+2) - (cos(qaswi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4.0 - (sin(qaswi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4.0 + cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))*(1.1E+1 / 2.0E+1) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))*(1.1E+1 / 2.0E+1)));
	acy[6] = -dq3swi*(-dq3swi*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 2.0E+2) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 2.0E+2)) + dq3sup*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 2.0E+2) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 2.0E+2)) + dq4sup*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 2.0E+2) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 2.0E+2)) + dq5sup*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 2.0E+2) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 2.0E+2))) + dq3sup*(-dq3swi*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 2.0E+2) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 2.0E+2)) + dq3sup*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 2.0E+2) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 2.0E+2)) + dq4sup*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 2.0E+2) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 2.0E+2)) + dq5sup*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 2.0E+2) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 2.0E+2))) + ddq3swi*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 2.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 2.0E+2)) - ddq3sup*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 2.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 2.0E+2)) - dq4sup*(dq3swi*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 2.0E+2) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 2.0E+2)) - dq3sup*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 2.0E+2) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 2.0E+2)) + dq4sup*(cos(q4sup)*sin(q5sup)*(3.3E+1 / 1.0E+2) + cos(q5sup)*sin(q4sup)*(3.3E+1 / 1.0E+2) - cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 2.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 2.0E+2)) + dq5sup*(cos(q4sup)*sin(q5sup)*(3.3E+1 / 1.0E+2) + cos(q5sup)*sin(q4sup)*(3.3E+1 / 1.0E+2) - cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 2.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 2.0E+2))) - ddq4sup*(cos(q4sup)*cos(q5sup)*(-3.3E+1 / 1.0E+2) + sin(q4sup)*sin(q5sup)*(3.3E+1 / 1.0E+2) + cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 2.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 2.0E+2)) - dq5sup*(dq3swi*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 2.0E+2) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 2.0E+2)) - dq3sup*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 2.0E+2) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 2.0E+2)) + dq4sup*(cos(q4sup)*sin(q5sup)*(3.3E+1 / 1.0E+2) + cos(q5sup)*sin(q4sup)*(3.3E+1 / 1.0E+2) - cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 2.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 2.0E+2)) + dq5sup*(sin(q5sup)*(3.3E+1 / 1.0E+2) + cos(q4sup)*sin(q5sup)*(3.3E+1 / 1.0E+2) + cos(q5sup)*sin(q4sup)*(3.3E+1 / 1.0E+2) - cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 2.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 2.0E+2))) - ddq5sup*(cos(q5sup)*(-3.3E+1 / 1.0E+2) - cos(q4sup)*cos(q5sup)*(3.3E+1 / 1.0E+2) + sin(q4sup)*sin(q5sup)*(3.3E+1 / 1.0E+2) + cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 2.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 2.0E+2));
	acy[7] = -dq5sup*(dq5sup*(sin(q5sup)*(3.3E+1 / 1.0E+2) + sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 2.0E+2) + cos(q4sup)*sin(q5sup)*(3.3E+1 / 1.0E+2) + cos(q5sup)*sin(q4sup)*(3.3E+1 / 1.0E+2) - cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 1.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 1.0E+2) - cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 2.0E+2)) - dq4swi*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 2.0E+2) - cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 2.0E+2)) - dq3swi*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 2.0E+2) - cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 1.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 1.0E+2) - cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 2.0E+2)) + dq3sup*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 2.0E+2) - cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 1.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 1.0E+2) - cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 2.0E+2)) + dq4sup*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 2.0E+2) + cos(q4sup)*sin(q5sup)*(3.3E+1 / 1.0E+2) + cos(q5sup)*sin(q4sup)*(3.3E+1 / 1.0E+2) - cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 1.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 1.0E+2) - cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 2.0E+2))) + ddq4swi*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 2.0E+2) + cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 2.0E+2)) + dq3swi*(-dq4swi*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 2.0E+2) - cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 2.0E+2)) - dq3swi*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 2.0E+2) - cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 1.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 1.0E+2) - cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 2.0E+2)) + dq3sup*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 2.0E+2) - cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 1.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 1.0E+2) - cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 2.0E+2)) + dq4sup*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 2.0E+2) - cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 1.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 1.0E+2) - cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 2.0E+2)) + dq5sup*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 2.0E+2) - cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 1.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 1.0E+2) - cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 2.0E+2))) - dq3sup*(-dq4swi*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 2.0E+2) - cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 2.0E+2)) - dq3swi*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 2.0E+2) - cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 1.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 1.0E+2) - cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 2.0E+2)) + dq3sup*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 2.0E+2) - cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 1.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 1.0E+2) - cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 2.0E+2)) + dq4sup*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 2.0E+2) - cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 1.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 1.0E+2) - cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 2.0E+2)) + dq5sup*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 2.0E+2) - cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 1.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 1.0E+2) - cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 2.0E+2))) + ddq3swi*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 2.0E+2) + cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 1.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 1.0E+2) + cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 2.0E+2)) - ddq3sup*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 2.0E+2) + cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 1.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 1.0E+2) + cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 2.0E+2)) - ddq4sup*(cos(q4sup)*cos(q5sup)*(-3.3E+1 / 1.0E+2) + sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 2.0E+2) + sin(q4sup)*sin(q5sup)*(3.3E+1 / 1.0E+2) + cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 1.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 1.0E+2) + cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 2.0E+2)) - dq4sup*(-dq4swi*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 2.0E+2) - cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 2.0E+2)) - dq3swi*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 2.0E+2) - cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 1.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 1.0E+2) - cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 2.0E+2)) + dq3sup*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 2.0E+2) - cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 1.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 1.0E+2) - cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 2.0E+2)) + dq4sup*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 2.0E+2) + cos(q4sup)*sin(q5sup)*(3.3E+1 / 1.0E+2) + cos(q5sup)*sin(q4sup)*(3.3E+1 / 1.0E+2) - cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 1.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 1.0E+2) - cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 2.0E+2)) + dq5sup*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 2.0E+2) + cos(q4sup)*sin(q5sup)*(3.3E+1 / 1.0E+2) + cos(q5sup)*sin(q4sup)*(3.3E+1 / 1.0E+2) - cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 1.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 1.0E+2) - cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 2.0E+2))) + dq4swi*(-dq3swi*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 2.0E+2) - cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 2.0E+2)) - dq4swi*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 2.0E+2) - cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 2.0E+2)) + dq3sup*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 2.0E+2) - cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 2.0E+2)) + dq4sup*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 2.0E+2) - cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 2.0E+2)) + dq5sup*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 2.0E+2) - cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 2.0E+2))) - ddq5sup*(cos(q5sup)*(-3.3E+1 / 1.0E+2) - cos(q4sup)*cos(q5sup)*(3.3E+1 / 1.0E+2) + sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 2.0E+2) + sin(q4sup)*sin(q5sup)*(3.3E+1 / 1.0E+2) + cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 1.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 1.0E+2) + cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 2.0E+2));
	acy[8] = -dq5sup*(dq5sup*(sin(q5sup)*(3.3E+1 / 1.0E+2) + sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 1.0E+2) + cos(q4sup)*sin(q5sup)*(3.3E+1 / 1.0E+2) + cos(q5sup)*sin(q4sup)*(3.3E+1 / 1.0E+2) - cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 1.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 1.0E+2) - cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 1.0E+2)) - dq4swi*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 1.0E+2) - cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 1.0E+2)) - dq3swi*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 1.0E+2) - cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 1.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 1.0E+2) - cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 1.0E+2)) + dq3sup*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 1.0E+2) - cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 1.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 1.0E+2) - cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 1.0E+2)) + dq4sup*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 1.0E+2) + cos(q4sup)*sin(q5sup)*(3.3E+1 / 1.0E+2) + cos(q5sup)*sin(q4sup)*(3.3E+1 / 1.0E+2) - cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 1.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 1.0E+2) - cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 1.0E+2))) + ddq4swi*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 1.0E+2) + cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 1.0E+2)) + dq3swi*(-dq4swi*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 1.0E+2) - cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 1.0E+2)) - dq3swi*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 1.0E+2) - cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 1.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 1.0E+2) - cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 1.0E+2)) + dq3sup*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 1.0E+2) - cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 1.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 1.0E+2) - cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 1.0E+2)) + dq4sup*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 1.0E+2) - cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 1.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 1.0E+2) - cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 1.0E+2)) + dq5sup*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 1.0E+2) - cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 1.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 1.0E+2) - cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 1.0E+2))) - dq3sup*(-dq4swi*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 1.0E+2) - cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 1.0E+2)) - dq3swi*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 1.0E+2) - cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 1.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 1.0E+2) - cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 1.0E+2)) + dq3sup*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 1.0E+2) - cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 1.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 1.0E+2) - cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 1.0E+2)) + dq4sup*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 1.0E+2) - cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 1.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 1.0E+2) - cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 1.0E+2)) + dq5sup*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 1.0E+2) - cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 1.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 1.0E+2) - cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 1.0E+2))) + ddq3swi*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 1.0E+2) + cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 1.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 1.0E+2) + cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 1.0E+2)) - ddq3sup*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 1.0E+2) + cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 1.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 1.0E+2) + cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 1.0E+2)) - ddq4sup*(cos(q4sup)*cos(q5sup)*(-3.3E+1 / 1.0E+2) + sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 1.0E+2) + sin(q4sup)*sin(q5sup)*(3.3E+1 / 1.0E+2) + cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 1.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 1.0E+2) + cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 1.0E+2)) - dq4sup*(-dq4swi*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 1.0E+2) - cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 1.0E+2)) - dq3swi*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 1.0E+2) - cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 1.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 1.0E+2) - cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 1.0E+2)) + dq3sup*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 1.0E+2) - cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 1.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 1.0E+2) - cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 1.0E+2)) + dq4sup*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 1.0E+2) + cos(q4sup)*sin(q5sup)*(3.3E+1 / 1.0E+2) + cos(q5sup)*sin(q4sup)*(3.3E+1 / 1.0E+2) - cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 1.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 1.0E+2) - cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 1.0E+2)) + dq5sup*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 1.0E+2) + cos(q4sup)*sin(q5sup)*(3.3E+1 / 1.0E+2) + cos(q5sup)*sin(q4sup)*(3.3E+1 / 1.0E+2) - cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 1.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 1.0E+2) - cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 1.0E+2))) + dq4swi*(-dq3swi*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 1.0E+2) - cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 1.0E+2)) - dq4swi*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 1.0E+2) - cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 1.0E+2)) + dq3sup*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 1.0E+2) - cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 1.0E+2)) + dq4sup*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 1.0E+2) - cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 1.0E+2)) + dq5sup*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 1.0E+2) - cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 1.0E+2))) - ddq5sup*(cos(q5sup)*(-3.3E+1 / 1.0E+2) - cos(q4sup)*cos(q5sup)*(3.3E+1 / 1.0E+2) + sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 1.0E+2) + sin(q4sup)*sin(q5sup)*(3.3E+1 / 1.0E+2) + cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 1.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 1.0E+2) + cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 1.0E+2));
	acz[1] = (q5sup*sin(q5sup)*(-3.3E+1 / 2.0E+2) - (dq5sup*dq5sup)*cos(q5sup)*(3.3E+1 / 2.0E+2));
	acz[2] = (dq4sup*(cos(q4sup)*sin(q5sup)*(3.3E+1 / 2.0E+2) + cos(q5sup)*sin(q4sup)*(3.3E+1 / 2.0E+2)) - ddq5sup*(sin(q5sup)*(3.3E+1 / 1.0E+2) + cos(q4sup)*sin(q5sup)*(3.3E+1 / 2.0E+2) + cos(q5sup)*sin(q4sup)*(3.3E+1 / 2.0E+2)) - dq4sup*(dq4sup*(cos(q4sup)*cos(q5sup)*(3.3E+1 / 2.0E+2) - sin(q4sup)*sin(q5sup)*(3.3E+1 / 2.0E+2)) + dq5sup*(cos(q4sup)*cos(q5sup)*(3.3E+1 / 2.0E+2) - sin(q4sup)*sin(q5sup)*(3.3E+1 / 2.0E+2))) - dq5sup*(dq4sup*(cos(q4sup)*cos(q5sup)*(3.3E+1 / 2.0E+2) - sin(q4sup)*sin(q5sup)*(3.3E+1 / 2.0E+2)) + dq5sup*(cos(q5sup)*(3.3E+1 / 1.0E+2) + cos(q4sup)*cos(q5sup)*(3.3E+1 / 2.0E+2) - sin(q4sup)*sin(q5sup)*(3.3E+1 / 2.0E+2))));
	acz[3] = (dq5sup*(sin(q5sup)*(3.3E+1 / 1.0E+2) + cos(q4sup)*sin(q5sup)*(3.3E+1 / 1.0E+2) + cos(q5sup)*sin(q4sup)*(3.3E+1 / 1.0E+2) + cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))*(1.1E+1 / 4.0E+1) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))*(1.1E+1 / 4.0E+1)) - dq3sup*(dq3sup*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))*(1.1E+1 / 4.0E+1) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))*(1.1E+1 / 4.0E+1)) + dq4sup*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))*(1.1E+1 / 4.0E+1) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))*(1.1E+1 / 4.0E+1)) + dq5sup*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))*(1.1E+1 / 4.0E+1) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))*(1.1E+1 / 4.0E+1))) - dq4sup*(dq3sup*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))*(1.1E+1 / 4.0E+1) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))*(1.1E+1 / 4.0E+1)) + dq4sup*(cos(q4sup)*cos(q5sup)*(3.3E+1 / 1.0E+2) - sin(q4sup)*sin(q5sup)*(3.3E+1 / 1.0E+2) + cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))*(1.1E+1 / 4.0E+1) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))*(1.1E+1 / 4.0E+1)) + dq5sup*(cos(q4sup)*cos(q5sup)*(3.3E+1 / 1.0E+2) - sin(q4sup)*sin(q5sup)*(3.3E+1 / 1.0E+2) + cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))*(1.1E+1 / 4.0E+1) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))*(1.1E+1 / 4.0E+1))) - ddq3sup*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))*(1.1E+1 / 4.0E+1) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))*(1.1E+1 / 4.0E+1)) - dq5sup*(dq5sup*(cos(q5sup)*(3.3E+1 / 1.0E+2) + cos(q4sup)*cos(q5sup)*(3.3E+1 / 1.0E+2) - sin(q4sup)*sin(q5sup)*(3.3E+1 / 1.0E+2) + cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))*(1.1E+1 / 4.0E+1) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))*(1.1E+1 / 4.0E+1)) + dq3sup*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))*(1.1E+1 / 4.0E+1) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))*(1.1E+1 / 4.0E+1)) + dq4sup*(cos(q4sup)*cos(q5sup)*(3.3E+1 / 1.0E+2) - sin(q4sup)*sin(q5sup)*(3.3E+1 / 1.0E+2) + cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))*(1.1E+1 / 4.0E+1) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))*(1.1E+1 / 4.0E+1))) - ddq4sup*(cos(q4sup)*sin(q5sup)*(3.3E+1 / 1.0E+2) + cos(q5sup)*sin(q4sup)*(3.3E+1 / 1.0E+2) + cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))*(1.1E+1 / 4.0E+1) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))*(1.1E+1 / 4.0E+1)));
	acz[4] = (dq3sup*(dq3sup*((cos(qasup)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4.0 - (sin(qasup)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4.0 - cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))*(1.1E+1 / 2.0E+1) + sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))*(1.1E+1 / 2.0E+1)) + dq4sup*((cos(qasup)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4.0 - (sin(qasup)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4.0 - cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))*(1.1E+1 / 2.0E+1) + sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))*(1.1E+1 / 2.0E+1)) + dq5sup*((cos(qasup)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4.0 - (sin(qasup)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4.0 - cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))*(1.1E+1 / 2.0E+1) + sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))*(1.1E+1 / 2.0E+1)) + dqasup*((cos(qasup)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4.0 - (sin(qasup)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4.0)) + ddq3sup*((cos(qasup)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4.0 + (sin(qasup)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4.0 - cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))*(1.1E+1 / 2.0E+1) - sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))*(1.1E+1 / 2.0E+1)) + dqasup*(dq3sup*((cos(qasup)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4.0 - (sin(qasup)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4.0) + dq4sup*((cos(qasup)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4.0 - (sin(qasup)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4.0) + dq5sup*((cos(qasup)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4.0 - (sin(qasup)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4.0) + dqasup*((cos(qasup)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4.0 - (sin(qasup)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4.0)) - ddq4sup*(cos(q4sup)*sin(q5sup)*(3.3E+1 / 1.0E+2) + cos(q5sup)*sin(q4sup)*(3.3E+1 / 1.0E+2) - (cos(qasup)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4.0 - (sin(qasup)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4.0 + cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))*(1.1E+1 / 2.0E+1) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))*(1.1E+1 / 2.0E+1)) + dq4sup*(dq3sup*((cos(qasup)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4.0 - (sin(qasup)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4.0 - cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))*(1.1E+1 / 2.0E+1) + sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))*(1.1E+1 / 2.0E+1)) - dq4sup*(cos(q4sup)*cos(q5sup)*(3.3E+1 / 1.0E+2) - sin(q4sup)*sin(q5sup)*(3.3E+1 / 1.0E+2) - (cos(qasup)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4.0 + (sin(qasup)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4.0 + cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))*(1.1E+1 / 2.0E+1) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))*(1.1E+1 / 2.0E+1)) - dq5sup*(cos(q4sup)*cos(q5sup)*(3.3E+1 / 1.0E+2) - sin(q4sup)*sin(q5sup)*(3.3E+1 / 1.0E+2) - (cos(qasup)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4.0 + (sin(qasup)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4.0 + cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))*(1.1E+1 / 2.0E+1) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))*(1.1E+1 / 2.0E+1)) + dqasup*((cos(qasup)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4.0 - (sin(qasup)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4.0)) + ddqasup*((cos(qasup)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4.0 + (sin(qasup)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4.0) - ddq5sup*(sin(q5sup)*(3.3E+1 / 1.0E+2) + cos(q4sup)*sin(q5sup)*(3.3E+1 / 1.0E+2) + cos(q5sup)*sin(q4sup)*(3.3E+1 / 1.0E+2) - (cos(qasup)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4.0 - (sin(qasup)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4.0 + cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))*(1.1E+1 / 2.0E+1) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))*(1.1E+1 / 2.0E+1)) + dq5sup*(dq3sup*((cos(qasup)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4.0 - (sin(qasup)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4.0 - cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))*(1.1E+1 / 2.0E+1) + sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))*(1.1E+1 / 2.0E+1)) - dq4sup*(cos(q4sup)*cos(q5sup)*(3.3E+1 / 1.0E+2) - sin(q4sup)*sin(q5sup)*(3.3E+1 / 1.0E+2) - (cos(qasup)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4.0 + (sin(qasup)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4.0 + cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))*(1.1E+1 / 2.0E+1) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))*(1.1E+1 / 2.0E+1)) - dq5sup*(cos(q5sup)*(3.3E+1 / 1.0E+2) + cos(q4sup)*cos(q5sup)*(3.3E+1 / 1.0E+2) - sin(q4sup)*sin(q5sup)*(3.3E+1 / 1.0E+2) - (cos(qasup)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4.0 + (sin(qasup)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4.0 + cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))*(1.1E+1 / 2.0E+1) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))*(1.1E+1 / 2.0E+1)) + dqasup*((cos(qasup)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4.0 - (sin(qasup)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4.0)));
	acz[5] = (dq3sup*(dq3sup*((cos(qaswi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4.0 - (sin(qaswi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4.0 - cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))*(1.1E+1 / 2.0E+1) + sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))*(1.1E+1 / 2.0E+1)) + dq4sup*((cos(qaswi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4.0 - (sin(qaswi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4.0 - cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))*(1.1E+1 / 2.0E+1) + sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))*(1.1E+1 / 2.0E+1)) + dq5sup*((cos(qaswi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4.0 - (sin(qaswi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4.0 - cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))*(1.1E+1 / 2.0E+1) + sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))*(1.1E+1 / 2.0E+1)) + dqaswi*((cos(qaswi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4.0 - (sin(qaswi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4.0)) + ddq3sup*((cos(qaswi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4.0 + (sin(qaswi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4.0 - cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))*(1.1E+1 / 2.0E+1) - sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))*(1.1E+1 / 2.0E+1)) + dqaswi*(dq3sup*((cos(qaswi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4.0 - (sin(qaswi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4.0) + dq4sup*((cos(qaswi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4.0 - (sin(qaswi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4.0) + dq5sup*((cos(qaswi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4.0 - (sin(qaswi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4.0) + dqaswi*((cos(qaswi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4.0 - (sin(qaswi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4.0)) - ddq4sup*(cos(q4sup)*sin(q5sup)*(3.3E+1 / 1.0E+2) + cos(q5sup)*sin(q4sup)*(3.3E+1 / 1.0E+2) - (cos(qaswi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4.0 - (sin(qaswi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4.0 + cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))*(1.1E+1 / 2.0E+1) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))*(1.1E+1 / 2.0E+1)) + dq4sup*(dq3sup*((cos(qaswi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4.0 - (sin(qaswi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4.0 - cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))*(1.1E+1 / 2.0E+1) + sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))*(1.1E+1 / 2.0E+1)) - dq4sup*(cos(q4sup)*cos(q5sup)*(3.3E+1 / 1.0E+2) - sin(q4sup)*sin(q5sup)*(3.3E+1 / 1.0E+2) - (cos(qaswi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4.0 + (sin(qaswi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4.0 + cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))*(1.1E+1 / 2.0E+1) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))*(1.1E+1 / 2.0E+1)) - dq5sup*(cos(q4sup)*cos(q5sup)*(3.3E+1 / 1.0E+2) - sin(q4sup)*sin(q5sup)*(3.3E+1 / 1.0E+2) - (cos(qaswi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4.0 + (sin(qaswi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4.0 + cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))*(1.1E+1 / 2.0E+1) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))*(1.1E+1 / 2.0E+1)) + dqaswi*((cos(qaswi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4.0 - (sin(qaswi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4.0)) + ddqaswi*((cos(qaswi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4.0 + (sin(qaswi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4.0) - ddq5sup*(sin(q5sup)*(3.3E+1 / 1.0E+2) + cos(q4sup)*sin(q5sup)*(3.3E+1 / 1.0E+2) + cos(q5sup)*sin(q4sup)*(3.3E+1 / 1.0E+2) - (cos(qaswi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4.0 - (sin(qaswi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4.0 + cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))*(1.1E+1 / 2.0E+1) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))*(1.1E+1 / 2.0E+1)) + dq5sup*(dq3sup*((cos(qaswi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4.0 - (sin(qaswi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4.0 - cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))*(1.1E+1 / 2.0E+1) + sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))*(1.1E+1 / 2.0E+1)) - dq4sup*(cos(q4sup)*cos(q5sup)*(3.3E+1 / 1.0E+2) - sin(q4sup)*sin(q5sup)*(3.3E+1 / 1.0E+2) - (cos(qaswi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4.0 + (sin(qaswi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4.0 + cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))*(1.1E+1 / 2.0E+1) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))*(1.1E+1 / 2.0E+1)) - dq5sup*(cos(q5sup)*(3.3E+1 / 1.0E+2) + cos(q4sup)*cos(q5sup)*(3.3E+1 / 1.0E+2) - sin(q4sup)*sin(q5sup)*(3.3E+1 / 1.0E+2) - (cos(qaswi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4.0 + (sin(qaswi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4.0 + cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))*(1.1E+1 / 2.0E+1) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))*(1.1E+1 / 2.0E+1)) + dqaswi*((cos(qaswi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))) / 4.0 - (sin(qaswi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))) / 4.0)));
	acz[6] = (q3swi*(-dq3swi*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 2.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 2.0E+2)) + dq3sup*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 2.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 2.0E+2)) + dq4sup*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 2.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 2.0E+2)) + dq5sup*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 2.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 2.0E+2))) + dq3sup*(-dq3swi*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 2.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 2.0E+2)) + dq3sup*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 2.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 2.0E+2)) + dq4sup*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 2.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 2.0E+2)) + dq5sup*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 2.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 2.0E+2))) - ddq3swi*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 2.0E+2) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 2.0E+2)) + ddq3sup*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 2.0E+2) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 2.0E+2)) + dq4sup*(-dq3swi*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 2.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 2.0E+2)) + dq3sup*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 2.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 2.0E+2)) + dq4sup*(cos(q4sup)*cos(q5sup)*(-3.3E+1 / 1.0E+2) + sin(q4sup)*sin(q5sup)*(3.3E+1 / 1.0E+2) + cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 2.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 2.0E+2)) + dq5sup*(cos(q4sup)*cos(q5sup)*(-3.3E+1 / 1.0E+2) + sin(q4sup)*sin(q5sup)*(3.3E+1 / 1.0E+2) + cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 2.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 2.0E+2))) - ddq4sup*(cos(q4sup)*sin(q5sup)*(3.3E+1 / 1.0E+2) + cos(q5sup)*sin(q4sup)*(3.3E+1 / 1.0E+2) - cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 2.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 2.0E+2)) + dq5sup*(-dq3swi*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 2.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 2.0E+2)) + dq3sup*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 2.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 2.0E+2)) + dq4sup*(cos(q4sup)*cos(q5sup)*(-3.3E+1 / 1.0E+2) + sin(q4sup)*sin(q5sup)*(3.3E+1 / 1.0E+2) + cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 2.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 2.0E+2)) + dq5sup*(cos(q5sup)*(-3.3E+1 / 1.0E+2) - cos(q4sup)*cos(q5sup)*(3.3E+1 / 1.0E+2) + sin(q4sup)*sin(q5sup)*(3.3E+1 / 1.0E+2) + cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 2.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 2.0E+2))) - ddq5sup*(sin(q5sup)*(3.3E+1 / 1.0E+2) + cos(q4sup)*sin(q5sup)*(3.3E+1 / 1.0E+2) + cos(q5sup)*sin(q4sup)*(3.3E+1 / 1.0E+2) - cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 2.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 2.0E+2)));
	acz[7] = (dq5sup*(sin(q5sup)*(3.3E+1 / 1.0E+2) + sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 2.0E+2) + cos(q4sup)*sin(q5sup)*(3.3E+1 / 1.0E+2) + cos(q5sup)*sin(q4sup)*(3.3E+1 / 1.0E+2) - cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 1.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 1.0E+2) - cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 2.0E+2)) + dq5sup*(-dq4swi*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 2.0E+2) + cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 2.0E+2)) - dq3swi*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 2.0E+2) + cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 1.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 1.0E+2) + cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 2.0E+2)) + dq3sup*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 2.0E+2) + cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 1.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 1.0E+2) + cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 2.0E+2)) + dq4sup*(cos(q4sup)*cos(q5sup)*(-3.3E+1 / 1.0E+2) + sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 2.0E+2) + sin(q4sup)*sin(q5sup)*(3.3E+1 / 1.0E+2) + cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 1.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 1.0E+2) + cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 2.0E+2)) + dq5sup*(cos(q5sup)*(-3.3E+1 / 1.0E+2) - cos(q4sup)*cos(q5sup)*(3.3E+1 / 1.0E+2) + sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 2.0E+2) + sin(q4sup)*sin(q5sup)*(3.3E+1 / 1.0E+2) + cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 1.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 1.0E+2) + cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 2.0E+2))) + ddq4swi*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 2.0E+2) - cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 2.0E+2)) - dq3swi*(-dq4swi*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 2.0E+2) + cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 2.0E+2)) - dq3swi*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 2.0E+2) + cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 1.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 1.0E+2) + cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 2.0E+2)) + dq3sup*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 2.0E+2) + cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 1.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 1.0E+2) + cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 2.0E+2)) + dq4sup*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 2.0E+2) + cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 1.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 1.0E+2) + cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 2.0E+2)) + dq5sup*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 2.0E+2) + cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 1.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 1.0E+2) + cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 2.0E+2))) + dq3sup*(-dq4swi*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 2.0E+2) + cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 2.0E+2)) - dq3swi*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 2.0E+2) + cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 1.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 1.0E+2) + cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 2.0E+2)) + dq3sup*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 2.0E+2) + cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 1.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 1.0E+2) + cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 2.0E+2)) + dq4sup*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 2.0E+2) + cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 1.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 1.0E+2) + cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 2.0E+2)) + dq5sup*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 2.0E+2) + cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 1.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 1.0E+2) + cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 2.0E+2))) + ddq3swi*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 2.0E+2) - cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 1.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 1.0E+2) - cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 2.0E+2)) - ddq3sup*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 2.0E+2) - cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 1.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 1.0E+2) - cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 2.0E+2)) - ddq4sup*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 2.0E+2) + cos(q4sup)*sin(q5sup)*(3.3E+1 / 1.0E+2) + cos(q5sup)*sin(q4sup)*(3.3E+1 / 1.0E+2) - cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 1.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 1.0E+2) - cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 2.0E+2)) + dq4sup*(-dq4swi*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 2.0E+2) + cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 2.0E+2)) - dq3swi*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 2.0E+2) + cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 1.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 1.0E+2) + cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 2.0E+2)) + dq3sup*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 2.0E+2) + cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 1.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 1.0E+2) + cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 2.0E+2)) + dq4sup*(cos(q4sup)*cos(q5sup)*(-3.3E+1 / 1.0E+2) + sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 2.0E+2) + sin(q4sup)*sin(q5sup)*(3.3E+1 / 1.0E+2) + cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 1.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 1.0E+2) + cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 2.0E+2)) + dq5sup*(cos(q4sup)*cos(q5sup)*(-3.3E+1 / 1.0E+2) + sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 2.0E+2) + sin(q4sup)*sin(q5sup)*(3.3E+1 / 1.0E+2) + cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 1.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 1.0E+2) + cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 2.0E+2))) - dq4swi*(-dq3swi*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 2.0E+2) + cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 2.0E+2)) - dq4swi*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 2.0E+2) + cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 2.0E+2)) + dq3sup*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 2.0E+2) + cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 2.0E+2)) + dq4sup*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 2.0E+2) + cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 2.0E+2)) + dq5sup*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 2.0E+2) + cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 2.0E+2))));
	// acz[7] = -ddq5sup*(sin(q5sup)*(3.3E+1 / 1.0E+2) + sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 1.0E+2) + cos(q4sup)*sin(q5sup)*(3.3E+1 / 1.0E+2) + cos(q5sup)*sin(q4sup)*(3.3E+1 / 1.0E+2) - cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 1.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 1.0E+2) - cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 1.0E+2)) + dq5sup*(-dq4swi*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 1.0E+2) + cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 1.0E+2)) - dq3swi*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 1.0E+2) + cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 1.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 1.0E+2) + cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 1.0E+2)) + dq3sup*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 1.0E+2) + cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 1.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 1.0E+2) + cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 1.0E+2)) + dq4sup*(cos(q4sup)*cos(q5sup)*(-3.3E+1 / 1.0E+2) + sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 1.0E+2) + sin(q4sup)*sin(q5sup)*(3.3E+1 / 1.0E+2) + cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 1.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 1.0E+2) + cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 1.0E+2)) + dq5sup*(cos(q5sup)*(-3.3E+1 / 1.0E+2) - cos(q4sup)*cos(q5sup)*(3.3E+1 / 1.0E+2) + sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 1.0E+2) + sin(q4sup)*sin(q5sup)*(3.3E+1 / 1.0E+2) + cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 1.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 1.0E+2) + cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 1.0E+2))) + ddq4swi*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 1.0E+2) - cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 1.0E+2)) - dq3swi*(-dq4swi*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 1.0E+2) + cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 1.0E+2)) - dq3swi*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 1.0E+2) + cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 1.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 1.0E+2) + cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 1.0E+2)) + dq3sup*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 1.0E+2) + cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 1.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 1.0E+2) + cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 1.0E+2)) + dq4sup*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 1.0E+2) + cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 1.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 1.0E+2) + cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 1.0E+2)) + dq5sup*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 1.0E+2) + cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 1.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 1.0E+2) + cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 1.0E+2))) + dq3sup*(-dq4swi*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 1.0E+2) + cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 1.0E+2)) - dq3swi*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 1.0E+2) + cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 1.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 1.0E+2) + cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 1.0E+2)) + dq3sup*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 1.0E+2) + cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 1.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 1.0E+2) + cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 1.0E+2)) + dq4sup*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 1.0E+2) + cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 1.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 1.0E+2) + cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 1.0E+2)) + dq5sup*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 1.0E+2) + cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 1.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 1.0E+2) + cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 1.0E+2))) + ddq3swi*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 1.0E+2) - cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 1.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 1.0E+2) - cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 1.0E+2)) - ddq3sup*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 1.0E+2) - cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 1.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 1.0E+2) - cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 1.0E+2)) - ddq4sup*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 1.0E+2) + cos(q4sup)*sin(q5sup)*(3.3E+1 / 1.0E+2) + cos(q5sup)*sin(q4sup)*(3.3E+1 / 1.0E+2) - cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 1.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 1.0E+2) - cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 1.0E+2)) + dq4sup*(-dq4swi*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 1.0E+2) + cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 1.0E+2)) - dq3swi*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 1.0E+2) + cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 1.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 1.0E+2) + cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 1.0E+2)) + dq3sup*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 1.0E+2) + cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 1.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 1.0E+2) + cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 1.0E+2)) + dq4sup*(cos(q4sup)*cos(q5sup)*(-3.3E+1 / 1.0E+2) + sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 1.0E+2) + sin(q4sup)*sin(q5sup)*(3.3E+1 / 1.0E+2) + cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 1.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 1.0E+2) + cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 1.0E+2)) + dq5sup*(cos(q4sup)*cos(q5sup)*(-3.3E+1 / 1.0E+2) + sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 1.0E+2) + sin(q4sup)*sin(q5sup)*(3.3E+1 / 1.0E+2) + cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)))*(3.3E+1 / 1.0E+2) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)))*(3.3E+1 / 1.0E+2) + cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 1.0E+2))) - dq4swi*(-dq3swi*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 1.0E+2) + cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 1.0E+2)) - dq4swi*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 1.0E+2) + cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 1.0E+2)) + dq3sup*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 1.0E+2) + cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 1.0E+2)) + dq4sup*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 1.0E+2) + cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 1.0E+2)) + dq5sup*(sin(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))) - sin(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))))*(3.3E+1 / 1.0E+2) + cos(q4swi)*(cos(q3swi)*(cos(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup)) - sin(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup))) + sin(q3swi)*(cos(q3sup)*(cos(q4sup)*sin(q5sup) + cos(q5sup)*sin(q4sup)) + sin(q3sup)*(cos(q4sup)*cos(q5sup) - sin(q4sup)*sin(q5sup))))*(3.3E+1 / 1.0E+2)))); 

	//cal p & c for la
	q6sup = Joints.la.q1;
	q2sup = Joints.la.q2;
	q2swi = Joints.la.q3;
	dq6sup = Joints.la.dq1;
	dq2sup = Joints.la.dq2;
	dq2swi = Joints.la.dq3;
	ddq6sup = Joints.la.ddq1;
	ddq2sup = Joints.la.ddq2;
	ddq2swi = Joints.la.ddq3;
	// pcxRsup[1] = -(33 * sin(q6sup)) / 100;
	// pcxRsup[2] = (17 * sin(q2sup)*sin(q6sup)) / 200 - (17 * cos(q2sup)*cos(q6sup)) / 200 - (11 * cos(q2sup)*sin(q6sup)) / 40 - (11 * cos(q6sup)*sin(q2sup)) / 40 - (33 * sin(q6sup)) / 50;
	// pcxRsup[3] = (17 * sin(q2sup)*sin(q6sup)) / 100 - (17 * cos(q2sup)*cos(q6sup)) / 100 - (33 * sin(q6sup)) / 50 + (33 * cos(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 100 - (33 * sin(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 100;
	// pcxRsup[4] = (17 * sin(q2sup)*sin(q6sup)) / 100 - (17 * cos(q2sup)*cos(q6sup)) / 100 - (33 * sin(q6sup)) / 50 + (33 * cos(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 50 - (33 * sin(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 50;
	// acxRsup[1] = (33 * dq6sup * dq6sup * sin(q6sup)) / 100 - (33 * ddq6sup*cos(q6sup)) / 100;
	// acxRsup[2] = dq2sup*(dq2sup*((17 * cos(q2sup)*cos(q6sup)) / 200 + (11 * cos(q2sup)*sin(q6sup)) / 40 + (11 * cos(q6sup)*sin(q2sup)) / 40 - (17 * sin(q2sup)*sin(q6sup)) / 200) + dq6sup*((17 * cos(q2sup)*cos(q6sup)) / 200 + (11 * cos(q2sup)*sin(q6sup)) / 40 + (11 * cos(q6sup)*sin(q2sup)) / 40 - (17 * sin(q2sup)*sin(q6sup)) / 200)) + dq6sup*(dq2sup*((17 * cos(q2sup)*cos(q6sup)) / 200 + (11 * cos(q2sup)*sin(q6sup)) / 40 + (11 * cos(q6sup)*sin(q2sup)) / 40 - (17 * sin(q2sup)*sin(q6sup)) / 200) + dq6sup*((33 * sin(q6sup)) / 50 + (17 * cos(q2sup)*cos(q6sup)) / 200 + (11 * cos(q2sup)*sin(q6sup)) / 40 + (11 * cos(q6sup)*sin(q2sup)) / 40 - (17 * sin(q2sup)*sin(q6sup)) / 200)) + ddq2sup*((17 * cos(q2sup)*sin(q6sup)) / 200 - (11 * cos(q2sup)*cos(q6sup)) / 40 + (17 * cos(q6sup)*sin(q2sup)) / 200 + (11 * sin(q2sup)*sin(q6sup)) / 40) + ddq6sup*((17 * cos(q2sup)*sin(q6sup)) / 200 - (11 * cos(q2sup)*cos(q6sup)) / 40 - (33 * cos(q6sup)) / 50 + (17 * cos(q6sup)*sin(q2sup)) / 200 + (11 * sin(q2sup)*sin(q6sup)) / 40);
	// acxRsup[3] = ddq6sup*((17 * cos(q2sup)*sin(q6sup)) / 100 - (33 * cos(q6sup)) / 50 + (17 * cos(q6sup)*sin(q2sup)) / 100 + (33 * cos(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 100 + (33 * sin(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 100) + dq2swi*(dq2sup*((33 * cos(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 100 - (33 * sin(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 100) - dq2swi*((33 * cos(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 100 - (33 * sin(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 100) + dq6sup*((33 * cos(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 100 - (33 * sin(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 100)) + dq2sup*(dq2swi*((33 * cos(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 100 - (33 * sin(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 100) + dq2sup*((17 * cos(q2sup)*cos(q6sup)) / 100 - (17 * sin(q2sup)*sin(q6sup)) / 100 - (33 * cos(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 100 + (33 * sin(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 100) + dq6sup*((17 * cos(q2sup)*cos(q6sup)) / 100 - (17 * sin(q2sup)*sin(q6sup)) / 100 - (33 * cos(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 100 + (33 * sin(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 100)) - ddq2swi*((33 * cos(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 100 + (33 * sin(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 100) + ddq2sup*((17 * cos(q2sup)*sin(q6sup)) / 100 + (17 * cos(q6sup)*sin(q2sup)) / 100 + (33 * cos(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 100 + (33 * sin(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 100) + dq6sup*(dq6sup*((33 * sin(q6sup)) / 50 + (17 * cos(q2sup)*cos(q6sup)) / 100 - (17 * sin(q2sup)*sin(q6sup)) / 100 - (33 * cos(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 100 + (33 * sin(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 100) + dq2swi*((33 * cos(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 100 - (33 * sin(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 100) + dq2sup*((17 * cos(q2sup)*cos(q6sup)) / 100 - (17 * sin(q2sup)*sin(q6sup)) / 100 - (33 * cos(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 100 + (33 * sin(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 100));
	// acxRsup[4] = ddq6sup*((17 * cos(q2sup)*sin(q6sup)) / 100 - (33 * cos(q6sup)) / 50 + (17 * cos(q6sup)*sin(q2sup)) / 100 + (33 * cos(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 50 + (33 * sin(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 50) + dq2swi*(dq2sup*((33 * cos(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 50 - (33 * sin(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 50) - dq2swi*((33 * cos(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 50 - (33 * sin(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 50) + dq6sup*((33 * cos(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 50 - (33 * sin(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 50)) + dq2sup*(dq2swi*((33 * cos(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 50 - (33 * sin(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 50) + dq2sup*((17 * cos(q2sup)*cos(q6sup)) / 100 - (17 * sin(q2sup)*sin(q6sup)) / 100 - (33 * cos(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 50 + (33 * sin(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 50) + dq6sup*((17 * cos(q2sup)*cos(q6sup)) / 100 - (17 * sin(q2sup)*sin(q6sup)) / 100 - (33 * cos(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 50 + (33 * sin(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 50)) - ddq2swi*((33 * cos(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 50 + (33 * sin(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 50) + ddq2sup*((17 * cos(q2sup)*sin(q6sup)) / 100 + (17 * cos(q6sup)*sin(q2sup)) / 100 + (33 * cos(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 50 + (33 * sin(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 50) + dq6sup*(dq6sup*((33 * sin(q6sup)) / 50 + (17 * cos(q2sup)*cos(q6sup)) / 100 - (17 * sin(q2sup)*sin(q6sup)) / 100 - (33 * cos(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 50 + (33 * sin(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 50) + dq2swi*((33 * cos(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 50 - (33 * sin(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 50) + dq2sup*((17 * cos(q2sup)*cos(q6sup)) / 100 - (17 * sin(q2sup)*sin(q6sup)) / 100 - (33 * cos(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 50 + (33 * sin(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 50));
	// pczRsup[1] = (33 * cos(q6sup)) / 100;
	// pczRsup[2] = (33 * cos(q6sup)) / 50 + (11 * cos(q2sup)*cos(q6sup)) / 40 - (17 * cos(q2sup)*sin(q6sup)) / 200 - (17 * cos(q6sup)*sin(q2sup)) / 200 - (11 * sin(q2sup)*sin(q6sup)) / 40;
	// pczRsup[3] = (33 * cos(q6sup)) / 50 - (17 * cos(q2sup)*sin(q6sup)) / 100 - (17 * cos(q6sup)*sin(q2sup)) / 100 - (33 * cos(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 100 - (33 * sin(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 100;
	// pczRsup[4] = (33 * cos(q6sup)) / 50 - (17 * cos(q2sup)*sin(q6sup)) / 100 - (17 * cos(q6sup)*sin(q2sup)) / 100 - (33 * cos(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 50 - (33 * sin(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 50;
	// aczRsup[1] = -(33 * ddq6sup*sin(q6sup)) / 100 - (33 * dq6sup * dq6sup * cos(q6sup)) / 100;
	// aczRsup[2] = dq2sup*(dq2sup*((17 * cos(q2sup)*sin(q6sup)) / 200 - (11 * cos(q2sup)*cos(q6sup)) / 40 + (17 * cos(q6sup)*sin(q2sup)) / 200 + (11 * sin(q2sup)*sin(q6sup)) / 40) + dq6sup*((17 * cos(q2sup)*sin(q6sup)) / 200 - (11 * cos(q2sup)*cos(q6sup)) / 40 + (17 * cos(q6sup)*sin(q2sup)) / 200 + (11 * sin(q2sup)*sin(q6sup)) / 40)) + dq6sup*(dq2sup*((17 * cos(q2sup)*sin(q6sup)) / 200 - (11 * cos(q2sup)*cos(q6sup)) / 40 + (17 * cos(q6sup)*sin(q2sup)) / 200 + (11 * sin(q2sup)*sin(q6sup)) / 40) + dq6sup*((17 * cos(q2sup)*sin(q6sup)) / 200 - (11 * cos(q2sup)*cos(q6sup)) / 40 - (33 * cos(q6sup)) / 50 + (17 * cos(q6sup)*sin(q2sup)) / 200 + (11 * sin(q2sup)*sin(q6sup)) / 40)) - ddq2sup*((17 * cos(q2sup)*cos(q6sup)) / 200 + (11 * cos(q2sup)*sin(q6sup)) / 40 + (11 * cos(q6sup)*sin(q2sup)) / 40 - (17 * sin(q2sup)*sin(q6sup)) / 200) - ddq6sup*((33 * sin(q6sup)) / 50 + (17 * cos(q2sup)*cos(q6sup)) / 200 + (11 * cos(q2sup)*sin(q6sup)) / 40 + (11 * cos(q6sup)*sin(q2sup)) / 40 - (17 * sin(q2sup)*sin(q6sup)) / 200);
	// aczRsup[3] = dq2sup*(dq2sup*((17 * cos(q2sup)*sin(q6sup)) / 100 + (17 * cos(q6sup)*sin(q2sup)) / 100 + (33 * cos(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 100 + (33 * sin(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 100) - dq2swi*((33 * cos(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 100 + (33 * sin(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 100) + dq6sup*((17 * cos(q2sup)*sin(q6sup)) / 100 + (17 * cos(q6sup)*sin(q2sup)) / 100 + (33 * cos(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 100 + (33 * sin(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 100)) - dq2swi*(dq2sup*((33 * cos(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 100 + (33 * sin(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 100) - dq2swi*((33 * cos(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 100 + (33 * sin(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 100) + dq6sup*((33 * cos(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 100 + (33 * sin(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 100)) - ddq6sup*((33 * sin(q6sup)) / 50 + (17 * cos(q2sup)*cos(q6sup)) / 100 - (17 * sin(q2sup)*sin(q6sup)) / 100 - (33 * cos(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 100 + (33 * sin(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 100) - ddq2swi*((33 * cos(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 100 - (33 * sin(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 100) + dq6sup*(dq6sup*((17 * cos(q2sup)*sin(q6sup)) / 100 - (33 * cos(q6sup)) / 50 + (17 * cos(q6sup)*sin(q2sup)) / 100 + (33 * cos(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 100 + (33 * sin(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 100) - dq2swi*((33 * cos(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 100 + (33 * sin(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 100) + dq2sup*((17 * cos(q2sup)*sin(q6sup)) / 100 + (17 * cos(q6sup)*sin(q2sup)) / 100 + (33 * cos(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 100 + (33 * sin(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 100)) - ddq2sup*((17 * cos(q2sup)*cos(q6sup)) / 100 - (17 * sin(q2sup)*sin(q6sup)) / 100 - (33 * cos(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 100 + (33 * sin(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 100);
	// aczRsup[4] = dq2sup*(dq2sup*((17 * cos(q2sup)*sin(q6sup)) / 100 + (17 * cos(q6sup)*sin(q2sup)) / 100 + (33 * cos(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 50 + (33 * sin(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 50) - dq2swi*((33 * cos(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 50 + (33 * sin(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 50) + dq6sup*((17 * cos(q2sup)*sin(q6sup)) / 100 + (17 * cos(q6sup)*sin(q2sup)) / 100 + (33 * cos(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 50 + (33 * sin(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 50)) - dq2swi*(dq2sup*((33 * cos(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 50 + (33 * sin(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 50) - dq2swi*((33 * cos(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 50 + (33 * sin(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 50) + dq6sup*((33 * cos(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 50 + (33 * sin(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 50)) - ddq6sup*((33 * sin(q6sup)) / 50 + (17 * cos(q2sup)*cos(q6sup)) / 100 - (17 * sin(q2sup)*sin(q6sup)) / 100 - (33 * cos(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 50 + (33 * sin(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 50) - ddq2swi*((33 * cos(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 50 - (33 * sin(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 50) + dq6sup*(dq6sup*((17 * cos(q2sup)*sin(q6sup)) / 100 - (33 * cos(q6sup)) / 50 + (17 * cos(q6sup)*sin(q2sup)) / 100 + (33 * cos(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 50 + (33 * sin(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 50) - dq2swi*((33 * cos(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 50 + (33 * sin(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 50) + dq2sup*((17 * cos(q2sup)*sin(q6sup)) / 100 + (17 * cos(q6sup)*sin(q2sup)) / 100 + (33 * cos(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 50 + (33 * sin(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 50)) - ddq2sup*((17 * cos(q2sup)*cos(q6sup)) / 100 - (17 * sin(q2sup)*sin(q6sup)) / 100 - (33 * cos(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 50 + (33 * sin(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 50);
	// pcxLsup[1] = -(33 * sin(q6sup)) / 100;
	// pcxLsup[2] = (17 * cos(q2sup)*cos(q6sup)) / 200 - (33 * sin(q6sup)) / 50 - (11 * cos(q2sup)*sin(q6sup)) / 40 - (11 * cos(q6sup)*sin(q2sup)) / 40 - (17 * sin(q2sup)*sin(q6sup)) / 200;
	// pcxLsup[3] = (17 * cos(q2sup)*cos(q6sup)) / 100 - (33 * sin(q6sup)) / 50 - (17 * sin(q2sup)*sin(q6sup)) / 100 + (33 * cos(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 100 - (33 * sin(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 100;
	// pcxLsup[4] = (17 * cos(q2sup)*cos(q6sup)) / 100 - (33 * sin(q6sup)) / 50 - (17 * sin(q2sup)*sin(q6sup)) / 100 + (33 * cos(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 50 - (33 * sin(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 50;
	// acxLsup[1] = (33 * dq6sup * dq6sup * sin(q6sup)) / 100 - (33 * ddq6sup*cos(q6sup)) / 100;
	// acxLsup[2] = dq2sup*(dq2sup*((11 * cos(q2sup)*sin(q6sup)) / 40 - (17 * cos(q2sup)*cos(q6sup)) / 200 + (11 * cos(q6sup)*sin(q2sup)) / 40 + (17 * sin(q2sup)*sin(q6sup)) / 200) + dq6sup*((11 * cos(q2sup)*sin(q6sup)) / 40 - (17 * cos(q2sup)*cos(q6sup)) / 200 + (11 * cos(q6sup)*sin(q2sup)) / 40 + (17 * sin(q2sup)*sin(q6sup)) / 200)) + dq6sup*(dq2sup*((11 * cos(q2sup)*sin(q6sup)) / 40 - (17 * cos(q2sup)*cos(q6sup)) / 200 + (11 * cos(q6sup)*sin(q2sup)) / 40 + (17 * sin(q2sup)*sin(q6sup)) / 200) + dq6sup*((33 * sin(q6sup)) / 50 - (17 * cos(q2sup)*cos(q6sup)) / 200 + (11 * cos(q2sup)*sin(q6sup)) / 40 + (11 * cos(q6sup)*sin(q2sup)) / 40 + (17 * sin(q2sup)*sin(q6sup)) / 200)) - ddq2sup*((11 * cos(q2sup)*cos(q6sup)) / 40 + (17 * cos(q2sup)*sin(q6sup)) / 200 + (17 * cos(q6sup)*sin(q2sup)) / 200 - (11 * sin(q2sup)*sin(q6sup)) / 40) - ddq6sup*((33 * cos(q6sup)) / 50 + (11 * cos(q2sup)*cos(q6sup)) / 40 + (17 * cos(q2sup)*sin(q6sup)) / 200 + (17 * cos(q6sup)*sin(q2sup)) / 200 - (11 * sin(q2sup)*sin(q6sup)) / 40);
	// acxLsup[3] = dq2swi*(dq2sup*((33 * cos(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 100 - (33 * sin(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 100) - dq2swi*((33 * cos(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 100 - (33 * sin(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 100) + dq6sup*((33 * cos(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 100 - (33 * sin(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 100)) - ddq6sup*((33 * cos(q6sup)) / 50 + (17 * cos(q2sup)*sin(q6sup)) / 100 + (17 * cos(q6sup)*sin(q2sup)) / 100 - (33 * cos(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 100 - (33 * sin(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 100) - dq2sup*(dq2sup*((17 * cos(q2sup)*cos(q6sup)) / 100 - (17 * sin(q2sup)*sin(q6sup)) / 100 + (33 * cos(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 100 - (33 * sin(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 100) - dq2swi*((33 * cos(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 100 - (33 * sin(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 100) + dq6sup*((17 * cos(q2sup)*cos(q6sup)) / 100 - (17 * sin(q2sup)*sin(q6sup)) / 100 + (33 * cos(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 100 - (33 * sin(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 100)) - ddq2swi*((33 * cos(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 100 + (33 * sin(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 100) - ddq2sup*((17 * cos(q2sup)*sin(q6sup)) / 100 + (17 * cos(q6sup)*sin(q2sup)) / 100 - (33 * cos(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 100 - (33 * sin(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 100) + dq6sup*(dq6sup*((33 * sin(q6sup)) / 50 - (17 * cos(q2sup)*cos(q6sup)) / 100 + (17 * sin(q2sup)*sin(q6sup)) / 100 - (33 * cos(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 100 + (33 * sin(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 100) + dq2swi*((33 * cos(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 100 - (33 * sin(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 100) - dq2sup*((17 * cos(q2sup)*cos(q6sup)) / 100 - (17 * sin(q2sup)*sin(q6sup)) / 100 + (33 * cos(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 100 - (33 * sin(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 100));
	// acxLsup[4] = dq2swi*(dq2sup*((33 * cos(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 50 - (33 * sin(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 50) - dq2swi*((33 * cos(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 50 - (33 * sin(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 50) + dq6sup*((33 * cos(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 50 - (33 * sin(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 50)) - ddq6sup*((33 * cos(q6sup)) / 50 + (17 * cos(q2sup)*sin(q6sup)) / 100 + (17 * cos(q6sup)*sin(q2sup)) / 100 - (33 * cos(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 50 - (33 * sin(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 50) - dq2sup*(dq2sup*((17 * cos(q2sup)*cos(q6sup)) / 100 - (17 * sin(q2sup)*sin(q6sup)) / 100 + (33 * cos(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 50 - (33 * sin(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 50) - dq2swi*((33 * cos(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 50 - (33 * sin(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 50) + dq6sup*((17 * cos(q2sup)*cos(q6sup)) / 100 - (17 * sin(q2sup)*sin(q6sup)) / 100 + (33 * cos(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 50 - (33 * sin(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 50)) - ddq2swi*((33 * cos(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 50 + (33 * sin(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 50) - ddq2sup*((17 * cos(q2sup)*sin(q6sup)) / 100 + (17 * cos(q6sup)*sin(q2sup)) / 100 - (33 * cos(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 50 - (33 * sin(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 50) + dq6sup*(dq6sup*((33 * sin(q6sup)) / 50 - (17 * cos(q2sup)*cos(q6sup)) / 100 + (17 * sin(q2sup)*sin(q6sup)) / 100 - (33 * cos(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 50 + (33 * sin(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 50) + dq2swi*((33 * cos(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 50 - (33 * sin(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 50) - dq2sup*((17 * cos(q2sup)*cos(q6sup)) / 100 - (17 * sin(q2sup)*sin(q6sup)) / 100 + (33 * cos(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 50 - (33 * sin(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 50));
	// pczLsup[1] = (33 * cos(q6sup)) / 100;
	// pczLsup[2] = (33 * cos(q6sup)) / 50 + (11 * cos(q2sup)*cos(q6sup)) / 40 + (17 * cos(q2sup)*sin(q6sup)) / 200 + (17 * cos(q6sup)*sin(q2sup)) / 200 - (11 * sin(q2sup)*sin(q6sup)) / 40;
	// pczLsup[3] = (33 * cos(q6sup)) / 50 + (17 * cos(q2sup)*sin(q6sup)) / 100 + (17 * cos(q6sup)*sin(q2sup)) / 100 - (33 * cos(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 100 - (33 * sin(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 100;
	// pczLsup[4] = (33 * cos(q6sup)) / 50 + (17 * cos(q2sup)*sin(q6sup)) / 100 + (17 * cos(q6sup)*sin(q2sup)) / 100 - (33 * cos(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 50 - (33 * sin(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 50;
	// aczLsup[1] = -(33 * ddq6sup*sin(q6sup)) / 100 - (33 * dq6sup * dq6sup * cos(q6sup)) / 100;
	// aczLsup[2] = -dq2sup*(dq2sup*((11 * cos(q2sup)*cos(q6sup)) / 40 + (17 * cos(q2sup)*sin(q6sup)) / 200 + (17 * cos(q6sup)*sin(q2sup)) / 200 - (11 * sin(q2sup)*sin(q6sup)) / 40) + dq6sup*((11 * cos(q2sup)*cos(q6sup)) / 40 + (17 * cos(q2sup)*sin(q6sup)) / 200 + (17 * cos(q6sup)*sin(q2sup)) / 200 - (11 * sin(q2sup)*sin(q6sup)) / 40)) - dq6sup*(dq2sup*((11 * cos(q2sup)*cos(q6sup)) / 40 + (17 * cos(q2sup)*sin(q6sup)) / 200 + (17 * cos(q6sup)*sin(q2sup)) / 200 - (11 * sin(q2sup)*sin(q6sup)) / 40) + dq6sup*((33 * cos(q6sup)) / 50 + (11 * cos(q2sup)*cos(q6sup)) / 40 + (17 * cos(q2sup)*sin(q6sup)) / 200 + (17 * cos(q6sup)*sin(q2sup)) / 200 - (11 * sin(q2sup)*sin(q6sup)) / 40)) - ddq2sup*((11 * cos(q2sup)*sin(q6sup)) / 40 - (17 * cos(q2sup)*cos(q6sup)) / 200 + (11 * cos(q6sup)*sin(q2sup)) / 40 + (17 * sin(q2sup)*sin(q6sup)) / 200) - ddq6sup*((33 * sin(q6sup)) / 50 - (17 * cos(q2sup)*cos(q6sup)) / 200 + (11 * cos(q2sup)*sin(q6sup)) / 40 + (11 * cos(q6sup)*sin(q2sup)) / 40 + (17 * sin(q2sup)*sin(q6sup)) / 200);
	// aczLsup[3] = ddq2sup*((17 * cos(q2sup)*cos(q6sup)) / 100 - (17 * sin(q2sup)*sin(q6sup)) / 100 + (33 * cos(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 100 - (33 * sin(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 100) - dq2swi*(dq2sup*((33 * cos(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 100 + (33 * sin(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 100) - dq2swi*((33 * cos(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 100 + (33 * sin(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 100) + dq6sup*((33 * cos(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 100 + (33 * sin(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 100)) - dq2sup*(dq2swi*((33 * cos(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 100 + (33 * sin(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 100) + dq2sup*((17 * cos(q2sup)*sin(q6sup)) / 100 + (17 * cos(q6sup)*sin(q2sup)) / 100 - (33 * cos(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 100 - (33 * sin(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 100) + dq6sup*((17 * cos(q2sup)*sin(q6sup)) / 100 + (17 * cos(q6sup)*sin(q2sup)) / 100 - (33 * cos(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 100 - (33 * sin(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 100)) - ddq2swi*((33 * cos(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 100 - (33 * sin(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 100) - dq6sup*(dq6sup*((33 * cos(q6sup)) / 50 + (17 * cos(q2sup)*sin(q6sup)) / 100 + (17 * cos(q6sup)*sin(q2sup)) / 100 - (33 * cos(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 100 - (33 * sin(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 100) + dq2swi*((33 * cos(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 100 + (33 * sin(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 100) + dq2sup*((17 * cos(q2sup)*sin(q6sup)) / 100 + (17 * cos(q6sup)*sin(q2sup)) / 100 - (33 * cos(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 100 - (33 * sin(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 100)) - ddq6sup*((33 * sin(q6sup)) / 50 - (17 * cos(q2sup)*cos(q6sup)) / 100 + (17 * sin(q2sup)*sin(q6sup)) / 100 - (33 * cos(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 100 + (33 * sin(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 100);
	// aczLsup[4] = ddq2sup*((17 * cos(q2sup)*cos(q6sup)) / 100 - (17 * sin(q2sup)*sin(q6sup)) / 100 + (33 * cos(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 50 - (33 * sin(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 50) - dq2swi*(dq2sup*((33 * cos(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 50 + (33 * sin(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 50) - dq2swi*((33 * cos(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 50 + (33 * sin(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 50) + dq6sup*((33 * cos(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 50 + (33 * sin(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 50)) - dq2sup*(dq2swi*((33 * cos(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 50 + (33 * sin(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 50) + dq2sup*((17 * cos(q2sup)*sin(q6sup)) / 100 + (17 * cos(q6sup)*sin(q2sup)) / 100 - (33 * cos(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 50 - (33 * sin(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 50) + dq6sup*((17 * cos(q2sup)*sin(q6sup)) / 100 + (17 * cos(q6sup)*sin(q2sup)) / 100 - (33 * cos(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 50 - (33 * sin(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 50)) - ddq2swi*((33 * cos(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 50 - (33 * sin(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 50) - dq6sup*(dq6sup*((33 * cos(q6sup)) / 50 + (17 * cos(q2sup)*sin(q6sup)) / 100 + (17 * cos(q6sup)*sin(q2sup)) / 100 - (33 * cos(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 50 - (33 * sin(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 50) + dq2swi*((33 * cos(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 50 + (33 * sin(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 50) + dq2sup*((17 * cos(q2sup)*sin(q6sup)) / 100 + (17 * cos(q6sup)*sin(q2sup)) / 100 - (33 * cos(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 50 - (33 * sin(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 50)) - ddq6sup*((33 * sin(q6sup)) / 50 - (17 * cos(q2sup)*cos(q6sup)) / 100 + (17 * sin(q2sup)*sin(q6sup)) / 100 - (33 * cos(q2swi)*(cos(q2sup)*sin(q6sup) + cos(q6sup)*sin(q2sup))) / 50 + (33 * sin(q2swi)*(cos(q2sup)*cos(q6sup) - sin(q2sup)*sin(q6sup))) / 50);

	//cal F desired from p & a
	cal_F_desired = Cal_Desired_Fext(msa, acz, pcy, acy, pcz, Tra_COM.x[kprecon], kprecon);
	//printf("%f, %f\n", cal_F_desired.Rfoot, cal_F_desired.Lfoot);
	//cal F desired from p & a
	taux_sup = ddq4swi*(Isa[7] + Isa[8] - msa[7] * 2.7225E-2 - msa[8] * 1.089E-1 + msa[7] * cos(q3swi + q4swi - q3sup - q4sup)*5.445E-2 + msa[8] * cos(q3swi + q4swi - q3sup - q4sup)*1.089E-1 + msa[7] * cos(q3swi + q4swi - q3sup)*5.445E-2 + msa[8] * cos(q3swi + q4swi - q3sup)*1.089E-1 - msa[7] * cos(q4swi)*5.445E-2 - msa[8] * cos(q4swi)*1.089E-1) - (g*(msa[5] * sin(q3sup + q4sup + q5sup + qaswi)*-5.0E+1 - msa[4] * sin(q3sup + q4sup + q5sup + qasup)*5.0E+1 + msa[2] * sin(q4sup + q5sup)*3.3E+1 + msa[3] * sin(q4sup + q5sup)*6.6E+1 + msa[4] * sin(q4sup + q5sup)*6.6E+1 + msa[5] * sin(q4sup + q5sup)*6.6E+1 + msa[6] * sin(q4sup + q5sup)*6.6E+1 + msa[7] * sin(q4sup + q5sup)*6.6E+1 + msa[8] * sin(q4sup + q5sup)*6.6E+1 - msa[7] * sin(-q3swi - q4swi + q3sup + q4sup + q5sup)*3.3E+1 - msa[8] * sin(-q3swi - q4swi + q3sup + q4sup + q5sup)*6.6E+1 + msa[1] * sin(q5sup)*3.3E+1 + msa[2] * sin(q5sup)*6.6E+1 + msa[3] * sin(q5sup)*6.6E+1 + msa[4] * sin(q5sup)*6.6E+1 + msa[5] * sin(q5sup)*6.6E+1 + msa[6] * sin(q5sup)*6.6E+1 + msa[7] * sin(q5sup)*6.6E+1 + msa[8] * sin(q5sup)*6.6E+1 - msa[6] * sin(-q3swi + q3sup + q4sup + q5sup)*3.3E+1 - msa[7] * sin(-q3swi + q3sup + q4sup + q5sup)*6.6E+1 - msa[8] * sin(-q3swi + q3sup + q4sup + q5sup)*6.6E+1 + msa[3] * sin(q3sup + q4sup + q5sup)*5.5E+1 + msa[4] * sin(q3sup + q4sup + q5sup)*1.1E+2 + msa[5] * sin(q3sup + q4sup + q5sup)*1.1E+2)) / 2.0E+2 + ddq4sup*(Isa[2] + Isa[3] + Isa[4] + Isa[5] + Isa[6] + Isa[7] + Isa[8] + msa[2] * 2.7225E-2 + msa[3] * 1.84525E-1 + msa[4] * 4.739E-1 + msa[5] * 4.739E-1 + msa[6] * 1.36125E-1 + msa[7] * 2.45025E-1 + msa[8] * 3.267E-1 - msa[7] * cos(q3swi + q4swi - q3sup - q4sup)*5.445E-2 - msa[8] * cos(q3swi + q4swi - q3sup - q4sup)*1.089E-1 - msa[7] * cos(q3swi + q4swi - q3sup)*1.089E-1 - msa[8] * cos(q3swi + q4swi - q3sup)*2.178E-1 - msa[6] * cos(-q3swi + q3sup + q4sup)*5.445E-2 - msa[7] * cos(-q3swi + q3sup + q4sup)*1.089E-1 - msa[8] * cos(-q3swi + q3sup + q4sup)*1.089E-1 + msa[3] * cos(q3sup + q4sup)*9.075E-2 + msa[4] * cos(q3sup + q4sup)*1.815E-1 + msa[5] * cos(q3sup + q4sup)*1.815E-1 - msa[5] * cos(q3sup + qaswi)*(3.3E+1 / 2.0E+2) - msa[4] * cos(q3sup + qasup)*(3.3E+1 / 2.0E+2) + msa[2] * cos(q4sup)*5.445E-2 + msa[3] * cos(q3sup)*1.815E-1 + msa[7] * cos(q4swi)*1.089E-1 + msa[3] * cos(q4sup)*1.089E-1 + msa[4] * cos(q3sup)*(3.63E+2 / 1.0E+3) + msa[8] * cos(q4swi)*2.178E-1 + msa[4] * cos(q4sup)*1.089E-1 + msa[5] * cos(q3sup)*(3.63E+2 / 1.0E+3) + msa[5] * cos(q4sup)*1.089E-1 + msa[6] * cos(q4sup)*1.089E-1 + msa[7] * cos(q4sup)*1.089E-1 + msa[8] * cos(q4sup)*1.089E-1 - msa[5] * cos(qaswi)*(1.1E+1 / 4.0E+1) - msa[4] * cos(qasup)*(1.1E+1 / 4.0E+1) - msa[6] * cos(q3swi - q3sup)*1.089E-1 - msa[7] * cos(q3swi - q3sup)*2.178E-1 - msa[8] * cos(q3swi - q3sup)*2.178E-1 - msa[5] * cos(q3sup + q4sup + qaswi)*(3.3E+1 / 4.0E+2) - msa[4] * cos(q3sup + q4sup + qasup)*(3.3E+1 / 4.0E+2)) + ddq5sup*(Isa[1] + Isa[2] + Isa[3] + Isa[4] + Isa[5] + Isa[6] + Isa[7] + Isa[8] + msa[1] * 2.7225E-2 + msa[2] * 1.36125E-1 + msa[3] * 2.93425E-1 + msa[4] * 5.828E-1 + msa[5] * 5.828E-1 + msa[6] * 2.45025E-1 + msa[7] * 3.53925E-1 + msa[8] * 4.356E-1 - msa[7] * cos(q3swi + q4swi - q3sup - q4sup)*1.089E-1 - msa[8] * cos(q3swi + q4swi - q3sup - q4sup)*2.178E-1 - msa[7] * cos(q3swi + q4swi - q3sup)*1.089E-1 - msa[8] * cos(q3swi + q4swi - q3sup)*2.178E-1 - msa[6] * cos(-q3swi + q3sup + q4sup)*1.089E-1 - msa[7] * cos(-q3swi + q3sup + q4sup)*2.178E-1 - msa[8] * cos(-q3swi + q3sup + q4sup)*2.178E-1 + msa[3] * cos(q3sup + q4sup)*1.815E-1 + msa[4] * cos(q3sup + q4sup)*(3.63E+2 / 1.0E+3) + msa[5] * cos(q3sup + q4sup)*(3.63E+2 / 1.0E+3) - msa[5] * cos(q3sup + qaswi)*(3.3E+1 / 2.0E+2) - msa[4] * cos(q3sup + qasup)*(3.3E+1 / 2.0E+2) + msa[2] * cos(q4sup)*1.089E-1 + msa[3] * cos(q3sup)*1.815E-1 + msa[7] * cos(q4swi)*1.089E-1 + msa[3] * cos(q4sup)*2.178E-1 + msa[4] * cos(q3sup)*(3.63E+2 / 1.0E+3) + msa[8] * cos(q4swi)*2.178E-1 + msa[4] * cos(q4sup)*2.178E-1 + msa[5] * cos(q3sup)*(3.63E+2 / 1.0E+3) + msa[5] * cos(q4sup)*2.178E-1 + msa[6] * cos(q4sup)*2.178E-1 + msa[7] * cos(q4sup)*2.178E-1 + msa[8] * cos(q4sup)*2.178E-1 - msa[5] * cos(qaswi)*(1.1E+1 / 4.0E+1) - msa[4] * cos(qasup)*(1.1E+1 / 4.0E+1) - msa[6] * cos(q3swi - q3sup)*1.089E-1 - msa[7] * cos(q3swi - q3sup)*2.178E-1 - msa[8] * cos(q3swi - q3sup)*2.178E-1 - msa[5] * cos(q3sup + q4sup + qaswi)*(3.3E+1 / 2.0E+2) - msa[4] * cos(q3sup + q4sup + qasup)*(3.3E+1 / 2.0E+2)) + ddq3swi*(Isa[6] + Isa[7] + Isa[8] - msa[6] * 2.7225E-2 - msa[7] * 1.36125E-1 - msa[8] * 2.178E-1 + msa[7] * cos(q3swi + q4swi - q3sup - q4sup)*5.445E-2 + msa[8] * cos(q3swi + q4swi - q3sup - q4sup)*1.089E-1 + msa[7] * cos(q3swi + q4swi - q3sup)*5.445E-2 + msa[8] * cos(q3swi + q4swi - q3sup)*1.089E-1 + msa[6] * cos(-q3swi + q3sup + q4sup)*5.445E-2 + msa[7] * cos(-q3swi + q3sup + q4sup)*1.089E-1 + msa[8] * cos(-q3swi + q3sup + q4sup)*1.089E-1 - msa[7] * cos(q4swi)*1.089E-1 - msa[8] * cos(q4swi)*2.178E-1 + msa[6] * cos(q3swi - q3sup)*5.445E-2 + msa[7] * cos(q3swi - q3sup)*1.089E-1 + msa[8] * cos(q3swi - q3sup)*1.089E-1) - ddqaswi*(-Isa[5] - msa[5] / 1.6E+1 + msa[5] * cos(q3sup + qaswi)*(3.3E+1 / 4.0E+2) + msa[5] * cos(qaswi)*(1.1E+1 / 8.0E+1) + msa[5] * cos(q3sup + q4sup + qaswi)*(3.3E+1 / 4.0E+2)) - ddqasup*(-Isa[4] - msa[4] / 1.6E+1 + msa[4] * cos(q3sup + qasup)*(3.3E+1 / 4.0E+2) + msa[4] * cos(qasup)*(1.1E+1 / 8.0E+1) + msa[4] * cos(q3sup + q4sup + qasup)*(3.3E+1 / 4.0E+2)) + ddq3sup*(Isa[3] + Isa[4] + Isa[5] + Isa[6] + Isa[7] + Isa[8] + msa[3] * 7.5625E-2 + msa[4] * (7.3E+1 / 2.0E+2) + msa[5] * (7.3E+1 / 2.0E+2) + msa[6] * 2.7225E-2 + msa[7] * 1.36125E-1 + msa[8] * 2.178E-1 - msa[7] * cos(q3swi + q4swi - q3sup - q4sup)*5.445E-2 - msa[8] * cos(q3swi + q4swi - q3sup - q4sup)*1.089E-1 - msa[7] * cos(q3swi + q4swi - q3sup)*5.445E-2 - msa[8] * cos(q3swi + q4swi - q3sup)*1.089E-1 - msa[6] * cos(-q3swi + q3sup + q4sup)*5.445E-2 - msa[7] * cos(-q3swi + q3sup + q4sup)*1.089E-1 - msa[8] * cos(-q3swi + q3sup + q4sup)*1.089E-1 + msa[3] * cos(q3sup + q4sup)*9.075E-2 + msa[4] * cos(q3sup + q4sup)*1.815E-1 + msa[5] * cos(q3sup + q4sup)*1.815E-1 - msa[5] * cos(q3sup + qaswi)*(3.3E+1 / 4.0E+2) - msa[4] * cos(q3sup + qasup)*(3.3E+1 / 4.0E+2) + msa[3] * cos(q3sup)*9.075E-2 + msa[7] * cos(q4swi)*1.089E-1 + msa[4] * cos(q3sup)*1.815E-1 + msa[8] * cos(q4swi)*2.178E-1 + msa[5] * cos(q3sup)*1.815E-1 - msa[5] * cos(qaswi)*(1.1E+1 / 4.0E+1) - msa[4] * cos(qasup)*(1.1E+1 / 4.0E+1) - msa[6] * cos(q3swi - q3sup)*5.445E-2 - msa[7] * cos(q3swi - q3sup)*1.089E-1 - msa[8] * cos(q3swi - q3sup)*1.089E-1 - msa[5] * cos(q3sup + q4sup + qaswi)*(3.3E+1 / 4.0E+2) - msa[4] * cos(q3sup + q4sup + qasup)*(3.3E+1 / 4.0E+2)) - (dq3swi*dq3swi)*msa[7] * sin(q3swi + q4swi - q3sup)*5.445E-2 - (dq3swi*dq3swi)*msa[8] * sin(q3swi + q4swi - q3sup)*1.089E-1 - (dq4swi*dq4swi)*msa[7] * sin(q3swi + q4swi - q3sup)*5.445E-2 - (dq4swi*dq4swi)*msa[8] * sin(q3swi + q4swi - q3sup)*1.089E-1 + (dq3swi*dq3swi)*msa[6] * sin(-q3swi + q3sup + q4sup)*5.445E-2 + (dq3swi*dq3swi)*msa[7] * sin(-q3swi + q3sup + q4sup)*1.089E-1 - (dq3sup*dq3sup)*msa[7] * sin(q3swi + q4swi - q3sup)*5.445E-2 + (dq3swi*dq3swi)*msa[8] * sin(-q3swi + q3sup + q4sup)*1.089E-1 - (dq3sup*dq3sup)*msa[8] * sin(q3swi + q4swi - q3sup)*1.089E-1 + (dq3sup*dq3sup)*msa[6] * sin(-q3swi + q3sup + q4sup)*5.445E-2 + (dq3sup*dq3sup)*msa[7] * sin(-q3swi + q3sup + q4sup)*1.089E-1 + (dq4sup*dq4sup)*msa[6] * sin(-q3swi + q3sup + q4sup)*5.445E-2 + (dq3sup*dq3sup)*msa[8] * sin(-q3swi + q3sup + q4sup)*1.089E-1 + (dq4sup*dq4sup)*msa[7] * sin(-q3swi + q3sup + q4sup)*1.089E-1 + (dq4sup*dq4sup)*msa[8] * sin(-q3swi + q3sup + q4sup)*1.089E-1 - (dq3sup*dq3sup)*msa[3] * sin(q3sup + q4sup)*9.075E-2 - (dq3sup*dq3sup)*msa[4] * sin(q3sup + q4sup)*1.815E-1 - (dq4sup*dq4sup)*msa[3] * sin(q3sup + q4sup)*9.075E-2 - (dq3sup*dq3sup)*msa[5] * sin(q3sup + q4sup)*1.815E-1 - (dq4sup*dq4sup)*msa[4] * sin(q3sup + q4sup)*1.815E-1 - (dq4sup*dq4sup)*msa[5] * sin(q3sup + q4sup)*1.815E-1 + (dq3sup*dq3sup)*msa[5] * sin(q3sup + qaswi)*(3.3E+1 / 4.0E+2) + (dq3sup*dq3sup)*msa[4] * sin(q3sup + qasup)*(3.3E+1 / 4.0E+2) + (dqaswi*dqaswi)*msa[5] * sin(q3sup + qaswi)*(3.3E+1 / 4.0E+2) + (dqasup*dqasup)*msa[4] * sin(q3sup + qasup)*(3.3E+1 / 4.0E+2) + (dq4swi*dq4swi)*msa[7] * sin(q4swi)*5.445E-2 + (dq4swi*dq4swi)*msa[8] * sin(q4swi)*1.089E-1 - (dq3sup*dq3sup)*msa[3] * sin(q3sup)*9.075E-2 - (dq3sup*dq3sup)*msa[4] * sin(q3sup)*1.815E-1 - (dq4sup*dq4sup)*msa[2] * sin(q4sup)*5.445E-2 - (dq3sup*dq3sup)*msa[5] * sin(q3sup)*1.815E-1 - (dq4sup*dq4sup)*msa[3] * sin(q4sup)*1.089E-1 - (dq4sup*dq4sup)*msa[4] * sin(q4sup)*1.089E-1 - (dq4sup*dq4sup)*msa[5] * sin(q4sup)*1.089E-1 - (dq4sup*dq4sup)*msa[6] * sin(q4sup)*1.089E-1 - (dq4sup*dq4sup)*msa[7] * sin(q4sup)*1.089E-1 - (dq4sup*dq4sup)*msa[8] * sin(q4sup)*1.089E-1 + (dqaswi*dqaswi)*msa[5] * sin(qaswi)*(1.1E+1 / 8.0E+1) + (dqasup*dqasup)*msa[4] * sin(qasup)*(1.1E+1 / 8.0E+1) - (dq3swi*dq3swi)*msa[6] * sin(q3swi - q3sup)*5.445E-2 - (dq3swi*dq3swi)*msa[7] * sin(q3swi - q3sup)*1.089E-1 - (dq3swi*dq3swi)*msa[8] * sin(q3swi - q3sup)*1.089E-1 - (dq3sup*dq3sup)*msa[6] * sin(q3swi - q3sup)*5.445E-2 - (dq3sup*dq3sup)*msa[7] * sin(q3swi - q3sup)*1.089E-1 - (dq3sup*dq3sup)*msa[8] * sin(q3swi - q3sup)*1.089E-1 + (dq3sup*dq3sup)*msa[5] * sin(q3sup + q4sup + qaswi)*(3.3E+1 / 4.0E+2) + (dq4sup*dq4sup)*msa[5] * sin(q3sup + q4sup + qaswi)*(3.3E+1 / 4.0E+2) + (dq3sup*dq3sup)*msa[4] * sin(q3sup + q4sup + qasup)*(3.3E+1 / 4.0E+2) + (dq4sup*dq4sup)*msa[4] * sin(q3sup + q4sup + qasup)*(3.3E+1 / 4.0E+2) + (dqaswi*dqaswi)*msa[5] * sin(q3sup + q4sup + qaswi)*(3.3E+1 / 4.0E+2) + (dqasup*dqasup)*msa[4] * sin(q3sup + q4sup + qasup)*(3.3E+1 / 4.0E+2) - (dq3swi*dq3swi)*msa[7] * sin(q3swi + q4swi - q3sup - q4sup)*5.445E-2 - (dq3swi*dq3swi)*msa[8] * sin(q3swi + q4swi - q3sup - q4sup)*1.089E-1 - (dq4swi*dq4swi)*msa[7] * sin(q3swi + q4swi - q3sup - q4sup)*5.445E-2 - (dq4swi*dq4swi)*msa[8] * sin(q3swi + q4swi - q3sup - q4sup)*1.089E-1 - (dq3sup*dq3sup)*msa[7] * sin(q3swi + q4swi - q3sup - q4sup)*5.445E-2 - (dq3sup*dq3sup)*msa[8] * sin(q3swi + q4swi - q3sup - q4sup)*1.089E-1 - (dq4sup*dq4sup)*msa[7] * sin(q3swi + q4swi - q3sup - q4sup)*5.445E-2 - (dq4sup*dq4sup)*msa[8] * sin(q3swi + q4swi - q3sup - q4sup)*1.089E-1 - dq3swi*dq4swi*msa[7] * sin(q3swi + q4swi - q3sup)*1.089E-1 - dq3swi*dq4swi*msa[8] * sin(q3swi + q4swi - q3sup)*2.178E-1 + dq3swi*dq3sup*msa[7] * sin(q3swi + q4swi - q3sup)*1.089E-1 + dq3swi*dq3sup*msa[8] * sin(q3swi + q4swi - q3sup)*2.178E-1 + dq3swi*dq4sup*msa[7] * sin(q3swi + q4swi - q3sup)*1.089E-1 + dq4swi*dq3sup*msa[7] * sin(q3swi + q4swi - q3sup)*1.089E-1 + dq3swi*dq4sup*msa[8] * sin(q3swi + q4swi - q3sup)*2.178E-1 + dq3swi*dq5sup*msa[7] * sin(q3swi + q4swi - q3sup)*1.089E-1 + dq4swi*dq3sup*msa[8] * sin(q3swi + q4swi - q3sup)*2.178E-1 + dq4swi*dq4sup*msa[7] * sin(q3swi + q4swi - q3sup)*1.089E-1 + dq3swi*dq5sup*msa[8] * sin(q3swi + q4swi - q3sup)*2.178E-1 + dq4swi*dq4sup*msa[8] * sin(q3swi + q4swi - q3sup)*2.178E-1 + dq4swi*dq5sup*msa[7] * sin(q3swi + q4swi - q3sup)*1.089E-1 - dq3swi*dq3sup*msa[6] * sin(-q3swi + q3sup + q4sup)*1.089E-1 + dq4swi*dq5sup*msa[8] * sin(q3swi + q4swi - q3sup)*2.178E-1 - dq3swi*dq3sup*msa[7] * sin(-q3swi + q3sup + q4sup)*2.178E-1 - dq3swi*dq4sup*msa[6] * sin(-q3swi + q3sup + q4sup)*1.089E-1 - dq3swi*dq3sup*msa[8] * sin(-q3swi + q3sup + q4sup)*2.178E-1 - dq3swi*dq4sup*msa[7] * sin(-q3swi + q3sup + q4sup)*2.178E-1 - dq3swi*dq5sup*msa[6] * sin(-q3swi + q3sup + q4sup)*1.089E-1 - dq3sup*dq4sup*msa[7] * sin(q3swi + q4swi - q3sup)*1.089E-1 - dq3swi*dq4sup*msa[8] * sin(-q3swi + q3sup + q4sup)*2.178E-1 - dq3swi*dq5sup*msa[7] * sin(-q3swi + q3sup + q4sup)*2.178E-1 - dq3sup*dq4sup*msa[8] * sin(q3swi + q4swi - q3sup)*2.178E-1 - dq3sup*dq5sup*msa[7] * sin(q3swi + q4swi - q3sup)*1.089E-1 - dq3swi*dq5sup*msa[8] * sin(-q3swi + q3sup + q4sup)*2.178E-1 - dq3sup*dq5sup*msa[8] * sin(q3swi + q4swi - q3sup)*2.178E-1 + dq3sup*dq4sup*msa[6] * sin(-q3swi + q3sup + q4sup)*1.089E-1 + dq3sup*dq4sup*msa[7] * sin(-q3swi + q3sup + q4sup)*2.178E-1 + dq3sup*dq5sup*msa[6] * sin(-q3swi + q3sup + q4sup)*1.089E-1 + dq3sup*dq4sup*msa[8] * sin(-q3swi + q3sup + q4sup)*2.178E-1 + dq3sup*dq5sup*msa[7] * sin(-q3swi + q3sup + q4sup)*2.178E-1 + dq4sup*dq5sup*msa[6] * sin(-q3swi + q3sup + q4sup)*1.089E-1 + dq3sup*dq5sup*msa[8] * sin(-q3swi + q3sup + q4sup)*2.178E-1 + dq4sup*dq5sup*msa[7] * sin(-q3swi + q3sup + q4sup)*2.178E-1 + dq4sup*dq5sup*msa[8] * sin(-q3swi + q3sup + q4sup)*2.178E-1 - dq3sup*dq4sup*msa[3] * sin(q3sup + q4sup)*1.815E-1 - dq3sup*dq4sup*msa[4] * sin(q3sup + q4sup)*(3.63E+2 / 1.0E+3) - dq3sup*dq5sup*msa[3] * sin(q3sup + q4sup)*1.815E-1 - dq3sup*dq4sup*msa[5] * sin(q3sup + q4sup)*(3.63E+2 / 1.0E+3) - dq3sup*dq5sup*msa[4] * sin(q3sup + q4sup)*(3.63E+2 / 1.0E+3) - dq4sup*dq5sup*msa[3] * sin(q3sup + q4sup)*1.815E-1 - dq3sup*dq5sup*msa[5] * sin(q3sup + q4sup)*(3.63E+2 / 1.0E+3) - dq4sup*dq5sup*msa[4] * sin(q3sup + q4sup)*(3.63E+2 / 1.0E+3) - dq4sup*dq5sup*msa[5] * sin(q3sup + q4sup)*(3.63E+2 / 1.0E+3) + dq3sup*dq4sup*msa[5] * sin(q3sup + qaswi)*(3.3E+1 / 2.0E+2) + dq3sup*dq5sup*msa[5] * sin(q3sup + qaswi)*(3.3E+1 / 2.0E+2) + dq3sup*dq4sup*msa[4] * sin(q3sup + qasup)*(3.3E+1 / 2.0E+2) + dq3sup*dq5sup*msa[4] * sin(q3sup + qasup)*(3.3E+1 / 2.0E+2) + dq3sup*dqaswi*msa[5] * sin(q3sup + qaswi)*(3.3E+1 / 2.0E+2) + dq4sup*dqaswi*msa[5] * sin(q3sup + qaswi)*(3.3E+1 / 2.0E+2) + dq5sup*dqaswi*msa[5] * sin(q3sup + qaswi)*(3.3E+1 / 2.0E+2) + dq3sup*dqasup*msa[4] * sin(q3sup + qasup)*(3.3E+1 / 2.0E+2) + dq4sup*dqasup*msa[4] * sin(q3sup + qasup)*(3.3E+1 / 2.0E+2) + dq5sup*dqasup*msa[4] * sin(q3sup + qasup)*(3.3E+1 / 2.0E+2) + dq3swi*dq4swi*msa[7] * sin(q4swi)*1.089E-1 + dq3swi*dq4swi*msa[8] * sin(q4swi)*2.178E-1 - dq4swi*dq3sup*msa[7] * sin(q4swi)*1.089E-1 - dq4swi*dq3sup*msa[8] * sin(q4swi)*2.178E-1 - dq4swi*dq4sup*msa[7] * sin(q4swi)*1.089E-1 - dq4swi*dq4sup*msa[8] * sin(q4swi)*2.178E-1 - dq4swi*dq5sup*msa[7] * sin(q4swi)*1.089E-1 - dq4swi*dq5sup*msa[8] * sin(q4swi)*2.178E-1 - dq3sup*dq4sup*msa[3] * sin(q3sup)*1.815E-1 - dq3sup*dq4sup*msa[4] * sin(q3sup)*(3.63E+2 / 1.0E+3) - dq3sup*dq5sup*msa[3] * sin(q3sup)*1.815E-1 - dq3sup*dq4sup*msa[5] * sin(q3sup)*(3.63E+2 / 1.0E+3) - dq3sup*dq5sup*msa[4] * sin(q3sup)*(3.63E+2 / 1.0E+3) - dq4sup*dq5sup*msa[2] * sin(q4sup)*1.089E-1 - dq3sup*dq5sup*msa[5] * sin(q3sup)*(3.63E+2 / 1.0E+3) - dq4sup*dq5sup*msa[3] * sin(q4sup)*2.178E-1 - dq4sup*dq5sup*msa[4] * sin(q4sup)*2.178E-1 - dq4sup*dq5sup*msa[5] * sin(q4sup)*2.178E-1 - dq4sup*dq5sup*msa[6] * sin(q4sup)*2.178E-1 - dq4sup*dq5sup*msa[7] * sin(q4sup)*2.178E-1 - dq4sup*dq5sup*msa[8] * sin(q4sup)*2.178E-1 + dq3sup*dqaswi*msa[5] * sin(qaswi)*(1.1E+1 / 4.0E+1) + dq4sup*dqaswi*msa[5] * sin(qaswi)*(1.1E+1 / 4.0E+1) + dq5sup*dqaswi*msa[5] * sin(qaswi)*(1.1E+1 / 4.0E+1) + dq3sup*dqasup*msa[4] * sin(qasup)*(1.1E+1 / 4.0E+1) + dq4sup*dqasup*msa[4] * sin(qasup)*(1.1E+1 / 4.0E+1) + dq5sup*dqasup*msa[4] * sin(qasup)*(1.1E+1 / 4.0E+1) + dq3swi*dq3sup*msa[6] * sin(q3swi - q3sup)*1.089E-1 + dq3swi*dq3sup*msa[7] * sin(q3swi - q3sup)*2.178E-1 + dq3swi*dq4sup*msa[6] * sin(q3swi - q3sup)*1.089E-1 + dq3swi*dq3sup*msa[8] * sin(q3swi - q3sup)*2.178E-1 + dq3swi*dq4sup*msa[7] * sin(q3swi - q3sup)*2.178E-1 + dq3swi*dq5sup*msa[6] * sin(q3swi - q3sup)*1.089E-1 + dq3swi*dq4sup*msa[8] * sin(q3swi - q3sup)*2.178E-1 + dq3swi*dq5sup*msa[7] * sin(q3swi - q3sup)*2.178E-1 + dq3swi*dq5sup*msa[8] * sin(q3swi - q3sup)*2.178E-1 - dq3sup*dq4sup*msa[6] * sin(q3swi - q3sup)*1.089E-1 - dq3sup*dq4sup*msa[7] * sin(q3swi - q3sup)*2.178E-1 - dq3sup*dq5sup*msa[6] * sin(q3swi - q3sup)*1.089E-1 - dq3sup*dq4sup*msa[8] * sin(q3swi - q3sup)*2.178E-1 - dq3sup*dq5sup*msa[7] * sin(q3swi - q3sup)*2.178E-1 - dq3sup*dq5sup*msa[8] * sin(q3swi - q3sup)*2.178E-1 + dq3sup*dq4sup*msa[5] * sin(q3sup + q4sup + qaswi)*(3.3E+1 / 2.0E+2) + dq3sup*dq5sup*msa[5] * sin(q3sup + q4sup + qaswi)*(3.3E+1 / 2.0E+2) + dq4sup*dq5sup*msa[5] * sin(q3sup + q4sup + qaswi)*(3.3E+1 / 2.0E+2) + dq3sup*dq4sup*msa[4] * sin(q3sup + q4sup + qasup)*(3.3E+1 / 2.0E+2) + dq3sup*dq5sup*msa[4] * sin(q3sup + q4sup + qasup)*(3.3E+1 / 2.0E+2) + dq4sup*dq5sup*msa[4] * sin(q3sup + q4sup + qasup)*(3.3E+1 / 2.0E+2) + dq3sup*dqaswi*msa[5] * sin(q3sup + q4sup + qaswi)*(3.3E+1 / 2.0E+2) + dq4sup*dqaswi*msa[5] * sin(q3sup + q4sup + qaswi)*(3.3E+1 / 2.0E+2) + dq5sup*dqaswi*msa[5] * sin(q3sup + q4sup + qaswi)*(3.3E+1 / 2.0E+2) + dq3sup*dqasup*msa[4] * sin(q3sup + q4sup + qasup)*(3.3E+1 / 2.0E+2) + dq4sup*dqasup*msa[4] * sin(q3sup + q4sup + qasup)*(3.3E+1 / 2.0E+2) + dq5sup*dqasup*msa[4] * sin(q3sup + q4sup + qasup)*(3.3E+1 / 2.0E+2) - dq3swi*dq4swi*msa[7] * sin(q3swi + q4swi - q3sup - q4sup)*1.089E-1 - dq3swi*dq4swi*msa[8] * sin(q3swi + q4swi - q3sup - q4sup)*2.178E-1 + dq3swi*dq3sup*msa[7] * sin(q3swi + q4swi - q3sup - q4sup)*1.089E-1 + dq3swi*dq3sup*msa[8] * sin(q3swi + q4swi - q3sup - q4sup)*2.178E-1 + dq3swi*dq4sup*msa[7] * sin(q3swi + q4swi - q3sup - q4sup)*1.089E-1 + dq4swi*dq3sup*msa[7] * sin(q3swi + q4swi - q3sup - q4sup)*1.089E-1 + dq3swi*dq4sup*msa[8] * sin(q3swi + q4swi - q3sup - q4sup)*2.178E-1 + dq3swi*dq5sup*msa[7] * sin(q3swi + q4swi - q3sup - q4sup)*1.089E-1 + dq4swi*dq3sup*msa[8] * sin(q3swi + q4swi - q3sup - q4sup)*2.178E-1 + dq4swi*dq4sup*msa[7] * sin(q3swi + q4swi - q3sup - q4sup)*1.089E-1 + dq3swi*dq5sup*msa[8] * sin(q3swi + q4swi - q3sup - q4sup)*2.178E-1 + dq4swi*dq4sup*msa[8] * sin(q3swi + q4swi - q3sup - q4sup)*2.178E-1 + dq4swi*dq5sup*msa[7] * sin(q3swi + q4swi - q3sup - q4sup)*1.089E-1 + dq4swi*dq5sup*msa[8] * sin(q3swi + q4swi - q3sup - q4sup)*2.178E-1 - dq3sup*dq4sup*msa[7] * sin(q3swi + q4swi - q3sup - q4sup)*1.089E-1 - dq3sup*dq4sup*msa[8] * sin(q3swi + q4swi - q3sup - q4sup)*2.178E-1 - dq3sup*dq5sup*msa[7] * sin(q3swi + q4swi - q3sup - q4sup)*1.089E-1 - dq3sup*dq5sup*msa[8] * sin(q3swi + q4swi - q3sup - q4sup)*2.178E-1 - dq4sup*dq5sup*msa[7] * sin(q3swi + q4swi - q3sup - q4sup)*1.089E-1 - dq4sup*dq5sup*msa[8] * sin(q3swi + q4swi - q3sup - q4sup)*2.178E-1;
	tauyR_sup = ddq2sup*(Ila[2] + Ila[3] + Ila[4] + mla[2] * 8.285E-2 + mla[3] * 1.378E-1 + mla[4] * 4.645E-1 + mla[2] * cos(q2sup)*1.815E-1 + mla[3] * sin(q2swi)*1.122E-1 + mla[4] * sin(q2swi)*2.244E-1 - mla[2] * sin(q2sup)*5.61E-2 - mla[3] * sin(q2sup)*1.122E-1 - mla[4] * sin(q2sup)*1.122E-1 - mla[3] * cos(q2swi - q2sup)*2.178E-1 - mla[4] * cos(q2swi - q2sup)*4.356E-1) + ddq6sup*(Ila[1] + Ila[2] + Ila[3] + Ila[4] + mla[1] * 1.089E-1 + mla[2] * 5.1845E-1 + mla[3] * 5.734E-1 + mla[4] * 9.001E-1 + mla[2] * cos(q2sup)*(3.63E+2 / 1.0E+3) + mla[3] * sin(q2swi)*1.122E-1 + mla[4] * sin(q2swi)*2.244E-1 - mla[2] * sin(q2sup)*1.122E-1 - mla[3] * sin(q2sup)*2.244E-1 - mla[4] * sin(q2sup)*2.244E-1 - mla[3] * cos(q2swi - q2sup)*4.356E-1 - mla[4] * cos(q2swi - q2sup)*8.712E-1) + ddq2swi*(Ila[3] + Ila[4] - mla[3] * 1.089E-1 - mla[4] * 4.356E-1 - mla[3] * sin(q2swi)*5.61E-2 - mla[4] * sin(q2swi)*1.122E-1 + mla[3] * cos(q2swi - q2sup)*2.178E-1 + mla[4] * cos(q2swi - q2sup)*4.356E-1) - (g*(mla[3] * sin(-q2swi + q2sup + q6sup)*-6.6E+1 - mla[4] * sin(-q2swi + q2sup + q6sup)*1.32E+2 + mla[2] * cos(q2sup + q6sup)*1.7E+1 + mla[3] * cos(q2sup + q6sup)*3.4E+1 + mla[4] * cos(q2sup + q6sup)*3.4E+1 + mla[2] * sin(q2sup + q6sup)*5.5E+1 + mla[1] * sin(q6sup)*6.6E+1 + mla[2] * sin(q6sup)*1.32E+2 + mla[3] * sin(q6sup)*1.32E+2 + mla[4] * sin(q6sup)*1.32E+2)) / 2.0E+2 - (dq2swi*dq2swi)*mla[3] * cos(q2swi)*5.61E-2 - (dq2swi*dq2swi)*mla[4] * cos(q2swi)*1.122E-1 - (dq2sup*dq2sup)*mla[2] * cos(q2sup)*5.61E-2 - (dq2sup*dq2sup)*mla[3] * cos(q2sup)*1.122E-1 - (dq2sup*dq2sup)*mla[4] * cos(q2sup)*1.122E-1 - (dq2sup*dq2sup)*mla[2] * sin(q2sup)*1.815E-1 - (dq2swi*dq2swi)*mla[3] * sin(q2swi - q2sup)*2.178E-1 - (dq2swi*dq2swi)*mla[4] * sin(q2swi - q2sup)*4.356E-1 - (dq2sup*dq2sup)*mla[3] * sin(q2swi - q2sup)*2.178E-1 - (dq2sup*dq2sup)*mla[4] * sin(q2swi - q2sup)*4.356E-1 + dq2swi*dq2sup*mla[3] * cos(q2swi)*1.122E-1 + dq2swi*dq2sup*mla[4] * cos(q2swi)*2.244E-1 + dq2swi*dq6sup*mla[3] * cos(q2swi)*1.122E-1 + dq2swi*dq6sup*mla[4] * cos(q2swi)*2.244E-1 - dq2sup*dq6sup*mla[2] * cos(q2sup)*1.122E-1 - dq2sup*dq6sup*mla[3] * cos(q2sup)*2.244E-1 - dq2sup*dq6sup*mla[4] * cos(q2sup)*2.244E-1 - dq2sup*dq6sup*mla[2] * sin(q2sup)*(3.63E+2 / 1.0E+3) + dq2swi*dq2sup*mla[3] * sin(q2swi - q2sup)*4.356E-1 + dq2swi*dq2sup*mla[4] * sin(q2swi - q2sup)*8.712E-1 + dq2swi*dq6sup*mla[3] * sin(q2swi - q2sup)*4.356E-1 + dq2swi*dq6sup*mla[4] * sin(q2swi - q2sup)*8.712E-1 - dq2sup*dq6sup*mla[3] * sin(q2swi - q2sup)*4.356E-1 - dq2sup*dq6sup*mla[4] * sin(q2swi - q2sup)*8.712E-1;
	tauyL_sup = ddq2sup*(Ila[2] + Ila[3] + Ila[4] + mla[2] * 8.285E-2 + mla[3] * 1.378E-1 + mla[4] * 4.645E-1 + mla[2] * cos(q2sup)*1.815E-1 - mla[3] * sin(q2swi)*1.122E-1 - mla[4] * sin(q2swi)*2.244E-1 + mla[2] * sin(q2sup)*5.61E-2 + mla[3] * sin(q2sup)*1.122E-1 + mla[4] * sin(q2sup)*1.122E-1 - mla[3] * cos(q2swi - q2sup)*2.178E-1 - mla[4] * cos(q2swi - q2sup)*4.356E-1) + ddq6sup*(Ila[1] + Ila[2] + Ila[3] + Ila[4] + mla[1] * 1.089E-1 + mla[2] * 5.1845E-1 + mla[3] * 5.734E-1 + mla[4] * 9.001E-1 + mla[2] * cos(q2sup)*(3.63E+2 / 1.0E+3) - mla[3] * sin(q2swi)*1.122E-1 - mla[4] * sin(q2swi)*2.244E-1 + mla[2] * sin(q2sup)*1.122E-1 + mla[3] * sin(q2sup)*2.244E-1 + mla[4] * sin(q2sup)*2.244E-1 - mla[3] * cos(q2swi - q2sup)*4.356E-1 - mla[4] * cos(q2swi - q2sup)*8.712E-1) + ddq2swi*(Ila[3] + Ila[4] - mla[3] * 1.089E-1 - mla[4] * 4.356E-1 + mla[3] * sin(q2swi)*5.61E-2 + mla[4] * sin(q2swi)*1.122E-1 + mla[3] * cos(q2swi - q2sup)*2.178E-1 + mla[4] * cos(q2swi - q2sup)*4.356E-1) + (g*(mla[3] * sin(-q2swi + q2sup + q6sup)*6.6E+1 + mla[4] * sin(-q2swi + q2sup + q6sup)*1.32E+2 + mla[2] * cos(q2sup + q6sup)*1.7E+1 + mla[3] * cos(q2sup + q6sup)*3.4E+1 + mla[4] * cos(q2sup + q6sup)*3.4E+1 - mla[2] * sin(q2sup + q6sup)*5.5E+1 - mla[1] * sin(q6sup)*6.6E+1 - mla[2] * sin(q6sup)*1.32E+2 - mla[3] * sin(q6sup)*1.32E+2 - mla[4] * sin(q6sup)*1.32E+2)) / 2.0E+2 + (dq2swi*dq2swi)*mla[3] * cos(q2swi)*5.61E-2 + (dq2swi*dq2swi)*mla[4] * cos(q2swi)*1.122E-1 + (dq2sup*dq2sup)*mla[2] * cos(q2sup)*5.61E-2 + (dq2sup*dq2sup)*mla[3] * cos(q2sup)*1.122E-1 + (dq2sup*dq2sup)*mla[4] * cos(q2sup)*1.122E-1 - (dq2sup*dq2sup)*mla[2] * sin(q2sup)*1.815E-1 - (dq2swi*dq2swi)*mla[3] * sin(q2swi - q2sup)*2.178E-1 - (dq2swi*dq2swi)*mla[4] * sin(q2swi - q2sup)*4.356E-1 - (dq2sup*dq2sup)*mla[3] * sin(q2swi - q2sup)*2.178E-1 - (dq2sup*dq2sup)*mla[4] * sin(q2swi - q2sup)*4.356E-1 - dq2swi*dq2sup*mla[3] * cos(q2swi)*1.122E-1 - dq2swi*dq2sup*mla[4] * cos(q2swi)*2.244E-1 - dq2swi*dq6sup*mla[3] * cos(q2swi)*1.122E-1 - dq2swi*dq6sup*mla[4] * cos(q2swi)*2.244E-1 + dq2sup*dq6sup*mla[2] * cos(q2sup)*1.122E-1 + dq2sup*dq6sup*mla[3] * cos(q2sup)*2.244E-1 + dq2sup*dq6sup*mla[4] * cos(q2sup)*2.244E-1 - dq2sup*dq6sup*mla[2] * sin(q2sup)*(3.63E+2 / 1.0E+3) + dq2swi*dq2sup*mla[3] * sin(q2swi - q2sup)*4.356E-1 + dq2swi*dq2sup*mla[4] * sin(q2swi - q2sup)*8.712E-1 + dq2swi*dq6sup*mla[3] * sin(q2swi - q2sup)*4.356E-1 + dq2swi*dq6sup*mla[4] * sin(q2swi - q2sup)*8.712E-1 - dq2sup*dq6sup*mla[3] * sin(q2swi - q2sup)*4.356E-1 - dq2sup*dq6sup*mla[4] * sin(q2swi - q2sup)*8.712E-1;
	kL = cal_F_desired.Lfoot / (cal_F_desired.Rfoot + cal_F_desired.Lfoot);
	kR = cal_F_desired.Rfoot / (cal_F_desired.Rfoot + cal_F_desired.Lfoot);
	cal_Tau_desired = Cal_Desired_Tau(taux_sup, tauyR_sup, tauyL_sup, kL, kR, kprecon);
	//printf("%f, %f, %f\n", taux_sup, tauyR_sup, tauyL_sup);
	//printf("%f, %f, %f, %F\n", cal_Tau_desired.Rfoot.x, cal_Tau_desired.Rfoot.y, cal_Tau_desired.Lfoot.x, cal_Tau_desired.Lfoot.y);
	FT_desired.F = cal_F_desired;
	FT_desired.Tau = cal_Tau_desired;
	//printf("%f, %f\n", cal_F_desired.Rfoot, cal_F_desired.Lfoot);


	dccFz_L = FT_desired.F.Lfoot;
	dccTx_L = FT_desired.Tau.Lfoot.x;
	dccTy_L = FT_desired.Tau.Lfoot.y;
	dccFz_R = FT_desired.F.Rfoot;
	dccTx_R = FT_desired.Tau.Rfoot.x;
	dccTy_R = FT_desired.Tau.Rfoot.y;
}

//dcc compliance
com_state FootXTControl(int kprecon, com_state lx, double F, double Fd, double used_for_leg)
{
	double Kd;
	double Dd;
	double Md;
	double el, del, e;

	double kdsup = 1800.0;//2500.0;		//1000; //3500;//6500;
	double ddsup = 100.0 ;//120.0;		//80;   //120;	//350;
	double kdswi = 300.0 ;//350.0;		//150;  //350;	//1000;
	double ddswi = 60.0  ;//75.0;		//40;   //75;	//150;
	double kddou = 500.0 ;//600.0; 		//2800;	//600;  //2800;//4000;
	double dddou = 60.0  ;//70.0; 		//110;		//70;   //110;	//300;
	
	if (used_for_leg == L_sup)
	{
		if (supleg == L_sup)
		{
			Kd = kdsup;// 1500
			Dd = ddsup;// 200
			Md = 0.0;
		}
		else if (supleg == R_sup)
		{
			Kd = kdswi;// 600
			Dd = ddswi;// 100
			Md = 0.0;
		}
		else
		{
			Kd = kddou;// 1200
			Dd = dddou;// 150
			Md = 0.0;
		}
	}
	else if (used_for_leg == R_sup)
	{
		if (supleg == R_sup)
		{
			Kd = kdsup;// 200
			Dd = ddsup;// 120
			Md = 0.0;
		}
		else if (supleg == L_sup)
		{
			Kd = kdswi;// 200
			Dd = ddswi;// 120
			Md = 0.0;
		}
		else
		{
			Kd = kddou;// 200
			Dd = dddou;// 120
			Md = 0.0;
		}
	}
	else
	{
		printf("error in FootXTControl\n");
		Kd = kdsup;// 200
		Dd = ddsup;// 120
		Md = 0.0;
	}
	el = lx.el;
	del = lx.del;
	e = (F - Fd + (Md / 0.004 / 0.004 + Dd / 0.004)*el + Md / 0.004*del) / (Md / 0.004 / 0.004 + Dd / 0.004 + Kd);
	lx.e = e;
	lx.del = (e - el) / 0.004;
	lx.el = e;

	return lx;
}
com_state FootYTControl(int kprecon, com_state lx, double F, double Fd, double used_for_leg)
{
	double Kd;
	double Dd;
	double Md;
	double el, del, e;

	double kdsup = 3000.0;//5000;	//1000;	//5000;	//7000;
	double ddsup = 120.0 ;//190;		//80;  	//190;	//380;	
	double kdswi = 300.0 ;//300;		//150; 	//300;	//1200;
	double ddswi = 60.0  ;//85;		//40;  	//85;	//180;	
	double kddou = 1500.0;//3200;	//600; 	//3200;	//5500;
	double dddou = 100.0 ;//170; 	//70;  	//170; 	//350; 
	if (used_for_leg == L_sup)
	{
		if (supleg == L_sup)
		{
			Kd = kdsup;// 300
			Dd = ddsup;// 80
			Md = 0.0;
		}
		else if (supleg == R_sup)
		{
			Kd = kdswi;// 50
			Dd = ddswi;// 70
			Md = 0.0;
		}
		else
		{
			Kd = kddou;// 250
			Dd = dddou;// 80
			Md = 0.0;
		}
	}
	else if (used_for_leg == R_sup)
	{
		if (supleg == R_sup)
		{
			Kd = kdsup;// 
			Dd = ddsup;// 
			Md = 0.0;
		}
		else if (supleg == L_sup)
		{
			Kd = kdswi;// 
			Dd = ddswi;// 
			Md = 0.0;
		}
		else
		{
			Kd = kddou;// 
			Dd = dddou;// 
			Md = 0.0;
		}
	}
	else
	{
		printf("error in FootXTControl\n");
		Kd = kdsup;// 
		Dd = ddsup;// 
		Md = 0.0;
	}
	el = lx.el;
	del = lx.del;
	e = (F - Fd + (Md / 0.004 / 0.004 + Dd / 0.004)*el + Md / 0.004*del) / (Md / 0.004 / 0.004 + Dd / 0.004 + Kd);
	lx.e = e;
	lx.del = (e - el) / 0.004;
	lx.el = e;

	return lx;
}
com_state FootZFControl(int kprecon, com_state lx, double F, double Fd, double used_for_leg)
{
	double Kd;
	double Dd;
	double Md;
	double el, del, e;

	double kdsup = 8000;//400000
	double ddsup = 4500;//8000
	double kdswi = 1500;//2500;
	double ddswi = 1600;//2000;
	double kddou = 6000;//180000
	double dddou = 4200;//5000
	if (used_for_leg == L_sup)
	{
		if (supleg == L_sup)
		{
			Kd = kdsup;// 1200
			Dd = ddsup;// 25000
			Md = 0.0;
		}
		else if (supleg == R_sup)
		{
			Kd = kdswi;// 600
			Dd = ddswi;// 18000
			Md = 0.0;
		}
		else
		{
			Kd = kddou;// 1000
			Dd = dddou;// 22000
			Md = 0.0;
		}
	}
	else if (used_for_leg == R_sup)
	{
		if (supleg == R_sup)
		{
			Kd = kdsup;// 
			Dd = ddsup;// 
			Md = 0.0;
		}
		else if (supleg == L_sup)
		{
			Kd = kdswi;// 
			Dd = ddswi;// 
			Md = 0.0;
		}
		else
		{
			Kd = kddou;// 
			Dd = dddou;// 
			Md = 0.0;
		}
	}
	else
	{
		printf("error in FootXTControl\n");
		Kd = kddou;// 
		Dd = dddou;// 
		Md = 0.0;
	}
	el = lx.el;
	del = lx.del;
	e = (F - Fd + (Md / 0.004 / 0.004 + Dd / 0.004)*el + Md / 0.004*del) / (Md / 0.004 / 0.004 + Dd / 0.004 + Kd);
	lx.e = e;
	lx.del = (e - el) / 0.004;
	lx.el = e;

	return lx;
}
com_state FootXFControl(int kprecon, com_state lx, double F, double Fd)
{
	double Kd = 2500;// 1300
	double Dd = 1000;// 500
	double Md = 0.0;
	double el, del, e;
	el = lx.el;
	del = lx.del;
	e = (F - Fd + (Md / 0.004 / 0.004 + Dd / 0.004)*el + Md / 0.004*del) / (Md / 0.004 / 0.004 + Dd / 0.004 + Kd);
	lx.e = e;
	lx.del = (e - el) / 0.004;
	lx.el = e;

	return lx;
}
com_state FootYFControl(int kprecon, com_state lx, double F, double Fd)
{
	double Kd = 2500;// 1300
	double Dd = 1000;// 500
	double Md = 0.0;
	double el, del, e;
	el = lx.el;
	del = lx.del;
	e = (F - Fd + (Md / 0.004 / 0.004 + Dd / 0.004)*el + Md / 0.004*del) / (Md / 0.004 / 0.004 + Dd / 0.004 + Kd);
	lx.e = e;
	lx.del = (e - el) / 0.004;
	lx.el = e;

	return lx;
}

void dcccompliance_FootFTControl(int kprecon)
{
// #ifdef SIMULATION
	// double FootFT[3][6];
	// FootFT[1][2] = F_RFoot.fz;
	// FootFT[2][2] = F_LFoot.fz;
// #endif
	double fastre_x = 0.0;
	double fastre_y = 0.0;
	double pbody_R = 0.5;//
	double pbody_L = 0.5;//
	double qbody_R = 0.2;//outer
	double qbody_L = 0.2;//outer
	double addi = 1.0;
	
	#ifdef USE_CHZ_RUN
	if (chzrun_signal[kprecon][2] == 1) // r sup
	{
		FT_desired.Tau.Rfoot.x = 0.0;//-chzrun_MF[kprecon][0];
		FT_desired.Tau.Lfoot.x = 0.0;
	}
	else if (chzrun_signal[kprecon][2] == 2) // l sup
	{
		FT_desired.Tau.Lfoot.x = 0.0;//-chzrun_MF[kprecon][0];
		FT_desired.Tau.Rfoot.x = 0.0;
	}
	else if (chzrun_signal[kprecon][2] == 3) // fly
	{
		FT_desired.Tau.Lfoot.x = 0.0;
		FT_desired.Tau.Rfoot.x = 0.0;
	}
	else // d sup
	{
		FT_desired.Tau.Lfoot.x = 0.0;//-0.5 * chzrun_MF[kprecon][0];
		FT_desired.Tau.Rfoot.x = 0.0;//-0.5 * chzrun_MF[kprecon][0];
	}
	
	if (chzrun_signal[kprecon][2] == 1) // r sup
	{
		FT_desired.Tau.Rfoot.y = 0.0;//-chzrun_MF[kprecon][2];
		FT_desired.Tau.Lfoot.y = 0.0;
	}
	else if (chzrun_signal[kprecon][2] == 2) // l sup
	{
		FT_desired.Tau.Lfoot.y = 0.0;//-chzrun_MF[kprecon][2];
		FT_desired.Tau.Rfoot.y = 0.0;
	}
	else if (chzrun_signal[kprecon][2] == 3) // fly
	{
		FT_desired.Tau.Lfoot.y = 0.0;
		FT_desired.Tau.Rfoot.y = 0.0;
	}
	else // d sup
	{
		FT_desired.Tau.Lfoot.y = 0.0;//-0.5 * chzrun_MF[kprecon][2];
		FT_desired.Tau.Rfoot.y = 0.0;//-0.5 * chzrun_MF[kprecon][2];
	}
	#endif
	
	
	DCC_rx = FootXTControl(kprecon, DCC_rx, (F_RFoot.tx - fastre_x*DCC_kx*DCC_rx.e) + 0.1 * 20, -pbody_R*FT_desired.Tau.Rfoot.x, R_sup);
	DCC_ry = FootYTControl(kprecon, DCC_ry, (F_RFoot.ty - fastre_y*DCC_ky*DCC_ry.e), -qbody_R*FT_desired.Tau.Rfoot.y, R_sup);
	//DCC_rpx = FootXFControl(kprecon, DCC_rpx, F_RFoot.fx, 0.0);
	//DCC_rpy = FootYFControl(kprecon, DCC_rpy, F_RFoot.fy, 0.0);
	DCC_rpz = FootZFControl(kprecon, DCC_rpz, (FootFT[1][2]), 1.0*FT_desired.F.Rfoot, R_sup);

	DCC_lx = FootXTControl(kprecon, DCC_lx, (F_LFoot.tx - fastre_x*DCC_kx*DCC_lx.e) + 0.1 * 20, -pbody_L*FT_desired.Tau.Lfoot.x, L_sup);
	DCC_ly = FootYTControl(kprecon, DCC_ly, (F_LFoot.ty - fastre_y*DCC_ky*DCC_ly.e), -qbody_L*FT_desired.Tau.Lfoot.y, L_sup);
	//DCC_lpx = FootXFControl(kprecon, DCC_lpx, F_LFoot.fx, 0.0);
	//DCC_lpy = FootYFControl(kprecon, DCC_lpy, F_LFoot.fy, 0.0);
	DCC_lpz = FootZFControl(kprecon, DCC_lpz, (FootFT[2][2]), 1.0*FT_desired.F.Lfoot, L_sup);

	if (DCC_rx.e > 0.1)DCC_rx.e = 0.1;
	if (DCC_lx.e > 0.1)DCC_lx.e = 0.1;
	if (DCC_rx.e < -0.1)DCC_rx.e = -0.1;
	if (DCC_lx.e < -0.1)DCC_lx.e = -0.1;
	if (DCC_ry.e > 0.1)DCC_ry.e = 0.1;
	if (DCC_ly.e > 0.1)DCC_ly.e = 0.1;
	if (DCC_ry.e < -0.1)DCC_ry.e = -0.1;
	if (DCC_ly.e < -0.1)DCC_ly.e = -0.1;
	if (DCC_rpz.e > 0.04)DCC_rpz.e = 0.04;
	if (DCC_lpz.e > 0.04)DCC_lpz.e = 0.04;
	if (DCC_rpz.e < -0.02)DCC_rpz.e = -0.02;
	if (DCC_lpz.e < -0.02)DCC_lpz.e = -0.02;

	dccx = DCC_lpx.e;
	dccy = DCC_lpy.e;
	dccz = DCC_lpz.e;
}


// rc

void Stand_Compliance_la()
{
	double T_rls = 0.5;
	double F_cal;
	double F_real;
	double A_F_cal = 0.2, B_F_cal = 0.8;
	double dF_cal;
	double F_resi;
	double F_v;
	// double kf = 1.3, kdf = 0.0, kx = 5.0, kdx = 0.0;
	// double kp = 200.0, kd = 300.0;
	// double t_cal = 0.22;//输入力延迟
	// double t = 1.6;//抗力延迟
	double kf = 1.4, kdf = 0.0, kx = 15.0, kdx = 0.0;
	// double kp = 70.0, kd = 145.0;
	double kp = 200.0, kd = 145.0;
	double T_cal = 0.15;//输入力延迟
	double T = 1.1;//抗力延迟
	
	//good
	// double F_cal;
	// double F_real;
	// double A_F_cal = 0.2, B_F_cal = 0.8;
	// double dF_cal;
	// double F_resi;
	// double F_v;
	// double kf = 1.3, kdf = 0.0, kx = 15.0, kdx = 0.0;
	// double kp = 70.0, kd = 135.0;
	// double T_cal = 0.22;//输入力延迟
	// double T = 1.3;//抗力延迟

	F_real = (0.5*(FootFT[1][2] - FootFT[2][2])*ANKLE_WIDTH - m_robot*GRAVITY*DCC_SC.la.e) / H_hip;
	//F_real = (0.5*(F_RFoot.fz - F_LFoot.fz)*ANKLE_WIDTH - m_robot*GRAVITY*DCC_SC.la.e) / H_hip;
	if(F_real > 80) F_real = 80;//20
	if(F_real < -80) F_real = -80;
	
	if (Count_Check_Release_x_old++ == T_rls * 0.2 / CONTROL_T) // continues decline of F_ext for five times, then release is judged
	{
		Count_Check_Release_x_old = 0;
		for (int i = 5; i > 0; i--)
		{
			F_Ext_Late_old[i] = F_Ext_Late_old[i - 1];
		}
	}
	
	F_cal = (F_real + T_cal / 0.004*F_cal_old) / (1 + T_cal/0.004);
	F_Ext_Late_old[0] = F_cal;
	
	dF_cal = (F_cal - F_cal_old) / 0.004;
	F_cal_old = F_cal;
	F_resi = -kf*F_cal - kdf*dF_cal - kx*DCC_SC.la.e - kdx*DCC_SC.la.de;

	F_v = (F_resi + T/0.004*F_v_old) / (1 + T/0.004);
	F_v_old = F_v;
	
	#ifdef stop_judgment
	if((abs(F_Ext_Late_old[5]) > abs(F_Ext_Late_old[4])) && (abs(F_Ext_Late_old[4]) > abs(F_Ext_Late_old[3])) && (abs(F_Ext_Late_old[3]) > abs(F_Ext_Late_old[2])) && (abs(F_Ext_Late_old[2]) > abs(F_Ext_Late_old[1])) && (abs(F_Ext_Late_old[1]) > abs(F_Ext_Late_old[0])))
	{
		F_v = -F_cal; // if release, stop resisting
		F_v_old = F_v;
	}
	#endif
	DCC_SC.la.e = 1.0*(F_cal + F_v + kd / 0.004*DCC_SC.la.el) / (kp + kd / 0.004);
	// if(DCC_SC.la.e > 0.06) DCC_SC.la.e = 0.06;
	// if(DCC_SC.la.e < -0.06) DCC_SC.la.e = -0.06;
	if(DCC_SC.la.e >  1.0 * 0.05) DCC_SC.la.e =  1.0 * 0.05;
	if(DCC_SC.la.e < -1.0 * 0.05) DCC_SC.la.e = -1.0 * 0.05;
	DCC_SC.la.de = (DCC_SC.la.el - DCC_SC.la.e) / 0.004;
	DCC_SC.la.el = DCC_SC.la.e;
	//printf("%6f\n", DCC_SC.la.de);
	//printf("%6f, %6f, %6f\n", F_cal, F_resi, F_v);
	//printf("%6f\n", DCC_SC.la.e);
	F_cal_x_re  = F_cal;
	F_resi_x_re = F_resi;
	F_v_x_re    = F_v;
	e_x_re      = DCC_SC.la.e;

	//body_roll
	double Tr_cal;
	//double dTr_cal;
	double T_resi;
	double T_v;
	// double kt = 1.2, kdt = 0.0, kr = 7.0, kdr = 0.0;
	// double kptr = 40.0, kdtr = 6.0;
	// double Tr = 0.5;// 1.6;//抗力延迟
	double kt = 2.0, kdt = 0.0, kr = 7.0, kdr = 0.0;
	// double kptr = 30.0, kdtr = 4.0;
	double kptr = 60.0, kdtr = 8.0;
	double Tr = 0.7;// 1.6;//抗力延迟

	Tr_cal = F_cal * 0.3;
	T_resi = -kt*Tr_cal - kr*DCC_SC.la.r;

	T_v = (T_resi + Tr / 0.004*T_v_old) / (1 + Tr / 0.004);
	T_v_old = T_v;
	
	#ifdef stop_judgment
	if((abs(F_Ext_Late_old[5]) > abs(F_Ext_Late_old[4])) && (abs(F_Ext_Late_old[4]) > abs(F_Ext_Late_old[3])) && (abs(F_Ext_Late_old[3]) > abs(F_Ext_Late_old[2])) && (abs(F_Ext_Late_old[2]) > abs(F_Ext_Late_old[1])) && (abs(F_Ext_Late_old[1]) > abs(F_Ext_Late_old[0])))
	{
		T_v = -Tr_cal; // if release, stop resisting
		T_v_old = T_v;
	}
	#endif

	DCC_SC.la.r = 1.0*(Tr_cal + T_v + kdtr / 0.004*DCC_SC.la.rl) / (kptr + kdtr / 0.004);
	// if (DCC_SC.la.r > 10 / 57.3) DCC_SC.la.r = 10 / 57.3;
	// if (DCC_SC.la.r < -10 / 57.3) DCC_SC.la.r = -10 / 57.3;
	if (DCC_SC.la.r >  1.0 * 12 / 57.3) DCC_SC.la.r =  1.0 * 12 / 57.3;
	if (DCC_SC.la.r < -1.0 * 12 / 57.3) DCC_SC.la.r = -1.0 * 12 / 57.3;
	DCC_SC.la.dr = (DCC_SC.la.r - DCC_SC.la.rl) / 0.004;
	DCC_SC.la.rl = DCC_SC.la.r;
	//printf("%6f, %6f, %6f\n", Tr_cal, T_resi, T_v);
	//printf("%6f\n", DCC_SC.la.r*57.3);
	// T_cal_re = Tr_cal;
	// T_resi_re = T_resi;
	// T_v_re = T_v;
	// r_re = DCC_SC.la.r;
}

void Stand_Compliance_sa()
{
	double F_cal;
	double F_real;
	double A_F_cal = 0.2, B_F_cal = 0.8;
	double dF_cal;
	double F_resi;
	double F_v;
	double kf = 1.2, kdf = 0.0, kx = 25.0, kdx = 0.0;
	// double kp = 85.0, kd = 220.0;
	double kp = 200.0, kd = 200.0;
	double T_cal = 0.13;//输入力延迟
	double T = 0.8;//抗力延迟
	
	//good
	// double F_cal;
	// double F_real;
	// double A_F_cal = 0.2, B_F_cal = 0.8;
	// double dF_cal;
	// double F_resi;
	// double F_v;
	// double kf = 1.05, kdf = 0.0, kx = 25.0, kdx = 0.0;
	// double kp = 85.0, kd = 200.0;
	// double T_cal = 0.20;//输入力延迟
	// double T = 0.8;//抗力延迟

	F_real = 1.0*(F_LFoot.tx + F_RFoot.tx - m_robot*GRAVITY*DCC_SC.sa.e) / H_hip - 0.0*(F_LFoot.fy + F_RFoot.fy);
	//F_real = (0.5*(F_RFoot.fz - F_LFoot.fz)*ANKLE_WIDTH - m_robot*GRAVITY*DCC_SC.la.e) / H_hip;
	if (F_real > 22) F_real = 22;//15
	if (F_real < -22) F_real = -22;
	F_cal = (F_real + T_cal / 0.004*F_cal_oldsa) / (1 + T_cal / 0.004);

	dF_cal = (F_cal - F_cal_oldsa) / 0.004;
	F_cal_oldsa = F_cal;
	F_resi = -kf*F_cal - kdf*dF_cal - kx*DCC_SC.sa.e - kdx*DCC_SC.sa.de;

	F_v = (F_resi + T / 0.004*F_v_oldsa) / (1 + T / 0.004);
	F_v_oldsa = F_v;

	DCC_SC.sa.e = 1.0*(F_cal + F_v + kd / 0.004*DCC_SC.sa.el) / (kp + kd / 0.004);
	// if (DCC_SC.sa.e > 0.05) DCC_SC.sa.e = 0.05;
	// if (DCC_SC.sa.e < -0.05) DCC_SC.sa.e = -0.05;
	if (DCC_SC.sa.e >  1.0 * 0.04) DCC_SC.sa.e =  1.0 * 0.04;
	if (DCC_SC.sa.e < -1.0 * 0.04) DCC_SC.sa.e = -1.0 * 0.04;
	DCC_SC.sa.de = (DCC_SC.sa.el - DCC_SC.sa.e) / 0.004;
	DCC_SC.sa.el = DCC_SC.sa.e;
	//printf("%6f\n", DCC_SC.la.de);
	//printf("%6f, %6f, %6f\n", F_cal, F_resi, F_v);
	//printf("%6f\n", DCC_SC.la.e);
	F_cal_y_re  = F_cal;
	F_resi_y_re = F_resi;
	F_v_y_re    = F_v;
	e_y_re      = DCC_SC.sa.e;

	//body_roll
	double Tr_cal;
	//double dTr_cal;
	double T_resi;
	double T_v;
	double kt = 1.3, kdt = 0.0, kr = 20.0, kdr = 0.0;
	// double kptr = 25.0, kdtr = 6.0;
	double kptr = 60.0, kdtr = 8.0;
	double Tr = 0.6;// 1.6;//抗力延迟

	Tr_cal = F_cal * 0.3;
	T_resi = -kt*Tr_cal - kr*DCC_SC.sa.r;

	T_v = (T_resi + Tr / 0.004*T_v_oldsa) / (1 + Tr / 0.004);
	T_v_oldsa = T_v;

	DCC_SC.sa.r = 1.0*(Tr_cal + T_v + kdtr / 0.004*DCC_SC.sa.rl) / (kptr + kdtr / 0.004);
	// if (DCC_SC.sa.r > 20 / 57.3) DCC_SC.sa.r = 20 / 57.3;
	// if (DCC_SC.sa.r < -20 / 57.3) DCC_SC.sa.r = -20 / 57.3;
	if (DCC_SC.sa.r >  1.0 * 15 / 57.3) DCC_SC.sa.r =  1.0 * 15 / 57.3;
	if (DCC_SC.sa.r < -1.0 * 15 / 57.3) DCC_SC.sa.r = -1.0 * 15 / 57.3;
	DCC_SC.sa.dr = (DCC_SC.sa.r - DCC_SC.sa.rl) / 0.004;
	DCC_SC.sa.rl = DCC_SC.sa.r;
	//printf("%6f, %6f, %6f, %6f \n\n", Tr_cal, T_resi, T_v, DCC_SC.sa.r);
	//printf("%6f\n", DCC_SC.la.r*57.3);
	T_cal_re = Tr_cal;
	T_resi_re = T_resi;
	T_v_re = T_v;
	r_re = DCC_SC.sa.r;
}

void chz_checkIMUbias()
{
	static int k = 200;
	static double chz_IMU_bias[20];
	if(K_Preview_Con == 1) for(int i = 0; i <= 19; i++) chz_IMU_bias[i] = 0.0;
	if(K_Preview_Con <= k)
	{
		chz_IMU_bias[0] += XS_AccX / (1.0 * k);
		chz_IMU_bias[1] += XS_GyrX / (1.0 * k);
		chz_IMU_bias[2] += XS_AccY / (1.0 * k);
		chz_IMU_bias[3] += XS_GyrY / (1.0 * k);
		chz_IMU_bias[4] += XS_AccZ / (1.0 * k);
		chz_IMU_bias[5] += XS_GyrZ / (1.0 * k);
		XS_AccX = 0.0; XS_GyrX = 0.0;
		XS_AccY = 0.0; XS_GyrY = 0.0;
		XS_AccZ = 0.0; XS_GyrZ = 0.0;
	}
	else
	{
		XS_AccX -= chz_IMU_bias[0];
		XS_GyrX -= chz_IMU_bias[1];
		XS_AccY -= chz_IMU_bias[2];
		XS_GyrY -= chz_IMU_bias[3];
		XS_AccZ -= chz_IMU_bias[4];
		XS_GyrZ -= chz_IMU_bias[5];
	}
}

void chz_footdown_go()
{
	static double ifcoll_l, ifcoll_r;
	//U
	#ifdef USE_CHZ_RUN
	if(K_Preview_Con >= 10 && chzrun_signal[K_Preview_Con - 5][0] == 0 && chzrun_signal[K_Preview_Con - 4][0] == 1)
	#else
	if(K_Preview_Con >= 10 && Signal_SupportLeg[K_Preview_Con - 5] == 0 && Signal_SupportLeg[K_Preview_Con - 4] == 2)
	#endif
	{
		chz_FootDown_setmode('R', 'U');
		ifcoll_r = 0;
	}
	#ifdef USE_CHZ_RUN
	if(K_Preview_Con >= 10 && chzrun_signal[K_Preview_Con - 5][1] == 0 && chzrun_signal[K_Preview_Con - 4][1] == 1)
	#else
	if(K_Preview_Con >= 10 && Signal_SupportLeg[K_Preview_Con - 5] == 0 && Signal_SupportLeg[K_Preview_Con - 4] == 1)
	#endif
	{
		chz_FootDown_setmode('L', 'U');
		ifcoll_l = 0;
	}
	//D
	#ifdef USE_CHZ_RUN
	if(K_Preview_Con >= 10 && chzrun_signal[K_Preview_Con - 1][0] == 1 && chzrun_signal[K_Preview_Con][0] == 0)
	#else
	if(K_Preview_Con >= 10 && Signal_SupportLeg[K_Preview_Con - 1] == 2 && Signal_SupportLeg[K_Preview_Con] == 0)
	#endif
	{
		chz_FootDown_setmode('R', 'D');
	}
	#ifdef USE_CHZ_RUN
	if(K_Preview_Con >= 10 && chzrun_signal[K_Preview_Con - 1][1] == 1 && chzrun_signal[K_Preview_Con][1] == 0)
	#else
	if(K_Preview_Con >= 10 && Signal_SupportLeg[K_Preview_Con - 1] == 1 && Signal_SupportLeg[K_Preview_Con] == 0)
	#endif
	{
		chz_FootDown_setmode('L', 'D');
	}

	//P
	#ifdef USE_CHZ_RUN
	if(chzrun_signal[K_Preview_Con + 20][0] == 0) ifcoll_r = 1;
	if(chzrun_signal[K_Preview_Con + 20][1] == 0) ifcoll_l = 1;
	#else
	if(Signal_SupportLeg[K_Preview_Con + 20] == 0) ifcoll_r = ifcoll_l = 1;
	#endif
	if(ifcoll_r && F_RFoot.fz > 100.0) chz_FootDown_setmode('R', 'P');
	if(ifcoll_l && F_LFoot.fz > 100.0) chz_FootDown_setmode('L', 'P');
	
	double footdowncon_l, footdowncon_r;
	footdowncon_r = chz_FootDown_updatecon('R');
	footdowncon_l = chz_FootDown_updatecon('L');
	
	Tra_RAnkle.z[K_Preview_Con] += footdowncon_r;
	Tra_LAnkle.z[K_Preview_Con] += footdowncon_l;
	
	// chz_log[15] = footdowncon_r;
	// chz_log[16] = footdowncon_l;
	// chz_log[17] = Tra_RAnkle.z[K_Preview_Con];
	// chz_log[18] = Tra_LAnkle.z[K_Preview_Con];
}

void chz_lowlevelfootstep_go()
{
	static int ifcoll_l = 0, ifcoll_r = 0;
	#ifdef USE_CHZ_RUN
	if(K_Preview_Con > 20 && chzrun_signal[K_Preview_Con - 15][0] == 0 && chzrun_signal[K_Preview_Con - 14][0] == 1)
	#else
	if(K_Preview_Con > 20 && Signal_SupportLeg[K_Preview_Con - 15] == 0 && Signal_SupportLeg[K_Preview_Con - 14] == 2)
	#endif
		chz_LowLevelFoot_setmode('X', 'R', 'C'), chz_LowLevelFoot_setmode('Y', 'R', 'C'), ifcoll_r = 0;
	
	#ifdef USE_CHZ_RUN
	if(K_Preview_Con > 20 && chzrun_signal[K_Preview_Con - 15][1] == 0 && chzrun_signal[K_Preview_Con - 14][1] == 1)
	#else
	if(K_Preview_Con > 20 && Signal_SupportLeg[K_Preview_Con - 15] == 0 && Signal_SupportLeg[K_Preview_Con - 14] == 1)
	#endif
		chz_LowLevelFoot_setmode('X', 'L', 'C'), chz_LowLevelFoot_setmode('Y', 'L', 'C'), ifcoll_l = 0;
	
	#ifdef USE_CHZ_RUN
	if(K_Preview_Con > 11 && chzrun_signal[K_Preview_Con - 1][0] == 1 && chzrun_signal[K_Preview_Con][0] == 0)
	#else
	if(K_Preview_Con > 11 && Signal_SupportLeg[K_Preview_Con + 10] == 0)
	#endif
		chz_LowLevelFoot_setmode('X', 'R', 'P'), chz_LowLevelFoot_setmode('Y', 'R', 'P');
		
	#ifdef USE_CHZ_RUN
	if(K_Preview_Con > 11 && chzrun_signal[K_Preview_Con - 1][1] == 1 && chzrun_signal[K_Preview_Con][1] == 0)
	#else
	if(K_Preview_Con > 11 && Signal_SupportLeg[K_Preview_Con + 10] == 0)
	#endif
		chz_LowLevelFoot_setmode('X', 'L', 'P'), chz_LowLevelFoot_setmode('Y', 'L', 'P');
	
	#ifdef USE_CHZ_RUN
	if(chzrun_signal[K_Preview_Con + 10][0] == 0) ifcoll_r = 1;
	if(chzrun_signal[K_Preview_Con + 10][1] == 0) ifcoll_l = 1;
	#else
	if(Signal_SupportLeg[K_Preview_Con + 10] == 0) ifcoll_r = ifcoll_l = 1;
	#endif
	if(ifcoll_r && F_RFoot.fz > 50.0) chz_LowLevelFoot_setmode('X', 'R', 'P'), chz_LowLevelFoot_setmode('Y', 'R', 'P');
	if(ifcoll_l && F_LFoot.fz > 50.0) chz_LowLevelFoot_setmode('X', 'L', 'P'), chz_LowLevelFoot_setmode('Y', 'R', 'P');
	
	double chzdcc_micro_x = 0.8 * 0.5;
	double chzdcc_micro_y = 0.8 * 0.5;
	double des_x , des_y;
	
	////////  Pitch Roll based  ////////
	// double chz_pitch_lim = 1.0 / 57.3;
	// double chz_roll_lim = 1.0 / 57.3;
	// if(chz_Roll_filted > chz_roll_lim) des_x = chzdcc_micro_x * (chz_Roll_filted - chz_roll_lim);
	// else if(chz_Roll_filted < -chz_roll_lim) des_x = chzdcc_micro_x * (chz_Roll_filted + chz_roll_lim);
	// else des_x = 0.0;
	// if(chz_Pitch_filted > chz_pitch_lim) des_y = chzdcc_micro_y * (chz_Pitch_filted - chz_pitch_lim);
	// else if(chz_Pitch_filted < -chz_pitch_lim) des_y = chzdcc_micro_y * (chz_Pitch_filted + chz_pitch_lim);
	// else des_y = 0.0;
	//////////////
	
	////////  CoMr based  ////////
	static double T = 4e-3, w = 3.5;//sqrt(9.8 / 0.8)
	static double chz_x_lim = 0.01;
	static double chz_y_lim = 0.01;
	static double chz_comr_filted[2], chz_vcomr_filted[2];
	if(K_Preview_Con == 101) for(int i = 0; i <= 1; i++) chz_LPFilters_init(i + 4, chz_comr[i]);
	if(K_Preview_Con >= 101) for(int i = 0; i <= 1; i++) chz_comr_filted[i] = chz_LPFilters_update(i + 4, chz_comr[i]);
	if(K_Preview_Con == 102) for(int i = 0; i <= 1; i++) chz_LPFilters_init(i + 6, 0.0);
	if(K_Preview_Con >= 102) for(int i = 0; i <= 1; i++) chz_vcomr_filted[i] = chz_LPFilters_update(i + 6, (chz_comr[i] - chz_comr_last[i]) / T);
	des_x = 0.6 * (chz_comr_filted[0] + 1 / w * chz_vcomr_filted[0]);
	des_y = 0.6 * (chz_comr_filted[1] + 1 / w * chz_vcomr_filted[1]);
	if(des_x > chz_x_lim) des_x -= chz_x_lim;
	else if(des_x < -chz_x_lim) des_x += chz_x_lim;
	else des_x = 0.0;
	if(des_y > chz_y_lim) des_y -= chz_y_lim;
	else if(des_y < -chz_y_lim) des_y += chz_y_lim;
	else des_y = 0.0;
	//////////////
	
	des_x = fmin(des_x, 0.07); des_x = fmax(des_x, -0.07);
	chz_LowLevelFoot_setxdes('X', des_x, des_x);
	des_y = fmin(des_y, 0.07); des_y = fmax(des_y, -0.07);
	chz_LowLevelFoot_setxdes('Y', des_y, des_y);
	
	double Xconr, Xconl, Yconr, Yconl;
	chz_LowLevelFoot_updatexcon('X', &Xconr, &Xconl);
	chz_LowLevelFoot_updatexcon('Y', &Yconr, &Yconl);
	Xconr = fmin(Xconr, 0.07);
	Xconr = fmax(Xconr, -0.05);
	Xconl = fmin(Xconl, 0.05);
	Xconl = fmax(Xconl, -0.05);
	
	Yconr = fmin(Yconr, 0.05);
	Yconr = fmax(Yconr, -0.05);
	Yconl = fmin(Yconl, 0.05);
	Yconl = fmax(Yconl, -0.05);
	
	Tra_RAnkle.x[K_Preview_Con] += 1.0 * Xconr;
	Tra_LAnkle.x[K_Preview_Con] += 1.0 * Xconl;
	// Tra_RAnkle.y[K_Preview_Con] += 0.55 * Yconr;
	// Tra_LAnkle.y[K_Preview_Con] += 0.55 * Yconl;
	
	// chz_log[6] = des_x;
	// chz_log[7] = Xconr;
	// chz_log[8] = Xconl;
	// chz_log[9] = des_y;
	// chz_log[10] = Yconr;
	// chz_log[11] = Yconl;
	// chz_log[12] = chzrun_signal[K_Preview_Con][2];
	
	// chz_log[34] = chz_comr[0];
	// chz_log[35] = chz_comr[1];
	// chz_log[36] = chz_vcomr_filted[0];
	// chz_log[37] = chz_vcomr_filted[1];
}

void chz_landingforce_go()
{
	static double chzmrobot = 43.0;
	static double g = 9.8;
	static int Landing_num_L = 9999999, Landing_num_R = 9999999;
	
	double fzr_real = max(F_RFoot.fz - chzmrobot * (g + Tra_ACOM.z[K_Preview_Con]), 0.0);
	double fzl_real = max(F_LFoot.fz - chzmrobot * (g + Tra_ACOM.z[K_Preview_Con]), 0.0);
	
	chz_LandingForce_update(fmin(fzr_real, 400.0), Tra_RAnkle.z[K_Preview_Con] - ANKLE_HEIGHT_CHZ, 'R', 'Z');
	chz_LandingForce_update(fmin(fzl_real, 400.0), Tra_LAnkle.z[K_Preview_Con] - ANKLE_HEIGHT_CHZ, 'L', 'Z');
	chz_LandingForce_update(fmin(-F_RFoot.fy, 200.0), -Tra_RAnkle.y[K_Preview_Con], 'R', 'Y');
	chz_LandingForce_update(fmin(-F_LFoot.fy, 200.0), -Tra_LAnkle.y[K_Preview_Con], 'L', 'Y');
	chz_LandingForce_update(fmin(F_RFoot.tx, 30.0), 20.0 * (Tra_RAnkle.z[K_Preview_Con] - ANKLE_HEIGHT_CHZ), 'R', 'P');
	chz_LandingForce_update(fmin(F_LFoot.tx, 30.0), 20.0 * (Tra_LAnkle.z[K_Preview_Con] - ANKLE_HEIGHT_CHZ), 'L', 'P');
	// chz_LandingForce_update(fmin(5.0, 10.0), 5.0 * (Tra_RAnkle.z[K_Preview_Con] - 0.112), 'R', 'P');
	// chz_LandingForce_update(fmin(5.0, 10.0), 5.0 * (Tra_LAnkle.z[K_Preview_Con] - 0.112), 'L', 'P');
	
	#ifdef USE_CHZ_RUN
	if(chzrun_signal[K_Preview_Con + 21][0] == 0 && chzrun_signal[K_Preview_Con + 20][0] == 1)
	#else
	if(Tra_RAnkle.z[K_Preview_Con + 21] < ANKLE_HEIGHT_CHZ + 1e-6 && Tra_RAnkle.z[K_Preview_Con + 20] > ANKLE_HEIGHT_CHZ + 1e-6)
	#endif
	{
		chz_LandingForce_setmode('O', 'R', 'Z');
		chz_LandingForce_setmode('O', 'R', 'Y');
		chz_LandingForce_setmode('O', 'R', 'P');
		Landing_num_R = K_Preview_Con + 21;
	}
	#ifdef USE_CHZ_RUN
	if(chzrun_signal[K_Preview_Con + 21][1] == 0 && chzrun_signal[K_Preview_Con + 20][1] == 1)
	#else
	if(Tra_LAnkle.z[K_Preview_Con + 21] < ANKLE_HEIGHT_CHZ + 1e-6 && Tra_LAnkle.z[K_Preview_Con + 20] > ANKLE_HEIGHT_CHZ + 1e-6)
	#endif
	{
		chz_LandingForce_setmode('O', 'L', 'Z');
		chz_LandingForce_setmode('O', 'L', 'Y');
		chz_LandingForce_setmode('O', 'L', 'P');
		Landing_num_L = K_Preview_Con + 21;
	}
	if(K_Preview_Con == Landing_num_R + 25)
	{
		chz_LandingForce_setmode('I', 'R', 'Z');
		chz_LandingForce_setmode('I', 'R', 'Y');
		chz_LandingForce_setmode('I', 'R', 'P');
		Landing_num_R = 9999999;
	}
	if(K_Preview_Con == Landing_num_L + 25)
	{
		chz_LandingForce_setmode('I', 'L', 'Z');
		chz_LandingForce_setmode('I', 'L', 'Y');
		chz_LandingForce_setmode('I', 'L', 'P');
		Landing_num_L = 9999999;
	}

	double chz_conrz = chz_LandingForce_getdelz('R', 'Z');
	double chz_conlz = chz_LandingForce_getdelz('L', 'Z');
	double chz_conry = chz_LandingForce_getdelz('R', 'Y');
	double chz_conly = chz_LandingForce_getdelz('L', 'Y');
	double chz_conrp = chz_LandingForce_getdelz('R', 'P');
	double chz_conlp = chz_LandingForce_getdelz('L', 'P');
	
	Tra_RAnkle.z[K_Preview_Con] += chz_conrz;
	Tra_LAnkle.z[K_Preview_Con] += chz_conlz;
	// Tra_RAnkle.y[K_Preview_Con] -= 0.15 * chz_conry;
	// Tra_LAnkle.y[K_Preview_Con] -= 0.15 * chz_conly;
	// pitch_footr += 0.15 * chz_conrp;
	// pitch_footl += 0.15 * chz_conlp;
	
	// chz_log[0] = chz_conrz; chz_log[1] = chz_conlz;
	// chz_log[2] = chz_conry; chz_log[3] = chz_conly;
	// chz_log[4] = chz_conrp; chz_log[5] = chz_conlp;
}

void chz_swingfoot_go()
{	
	static double ifcoll_r = 0, ifcoll_l = 0, rprop = 0.5;
	if(K_Preview_Con == 1)
	{
		double Pos_lim[6][2] = {{-0.03, 0.03}, {-0.04, 0.04}, {-0.05, 0.05}, {-0.20, 0.20}, {-0.20, 0.20}, {-0.20, 0.20}};
		chz_SwingFoot_setposlim(Pos_lim);
		chz_SwingFoot_setmode('R', 0);
		chz_SwingFoot_setmode('L', 0);
	}
	// // test //
	// rprop = 0.0;
	
	if(K_Preview_Con < 100) return;
	#ifdef USE_CHZ_RUN
		if (chzrun_signal[K_Preview_Con][2] == 0) rprop = 0.5;
		else if(chzrun_signal[K_Preview_Con][2] == 1) rprop = 1.0;
		else if (chzrun_signal[K_Preview_Con][2] == 2) rprop = 0.0;
		else
		{
			int templ = K_Preview_Con, tempr = K_Preview_Con;
			while (chzrun_signal[templ][2] == 3) templ--;
			while (chzrun_signal[tempr][2] == 3) tempr++;
			rprop = (K_Preview_Con - templ) / (tempr - templ);
		}
	#else
		if (Signal_SupportLeg[K_Preview_Con] == DOUBLE_LEG_SP) rprop = 0.5;
		else if(Signal_SupportLeg[K_Preview_Con] == RLEG_SP) rprop = 1.0;
		else rprop = 0.0;
	#endif
	chz_SwingFoot_setprop(rprop);

	#ifdef USE_CHZ_RUN
	if(K_Preview_Con > 20 && chzrun_signal[K_Preview_Con - 10][0] == 0 && chzrun_signal[K_Preview_Con - 9][0] == 1)
	#else
	if(K_Preview_Con > 20 && Signal_SupportLeg[K_Preview_Con - 10] == 0 && Signal_SupportLeg[K_Preview_Con - 9] == 2)
	#endif
		chz_SwingFoot_setmode('R', 1), ifcoll_r = 1;
	#ifdef USE_CHZ_RUN
	if(K_Preview_Con > 20 && chzrun_signal[K_Preview_Con - 10][1] == 0 && chzrun_signal[K_Preview_Con - 9][1] == 1)
	#else
	if(K_Preview_Con > 20 && Signal_SupportLeg[K_Preview_Con - 10] == 0 && Signal_SupportLeg[K_Preview_Con - 9] == 1)
	#endif
		chz_SwingFoot_setmode('L', 1), ifcoll_l = 1;

	#ifdef USE_CHZ_RUN
	if(ifcoll_r && chzrun_signal[K_Preview_Con - 9][0] == 0 && chzrun_signal[K_Preview_Con - 10][0] == 1)
	#else
	if(ifcoll_r && Signal_SupportLeg[K_Preview_Con - 9] == 0 && Signal_SupportLeg[K_Preview_Con - 10] == 2)
	#endif
		chz_SwingFoot_setmode('R', 0), ifcoll_r = 0;
	#ifdef USE_CHZ_RUN
	if(ifcoll_l && chzrun_signal[K_Preview_Con - 9][1] == 0 && chzrun_signal[K_Preview_Con - 10][1] == 1)
	#else
	if(ifcoll_l && Signal_SupportLeg[K_Preview_Con - 9] == 0 && Signal_SupportLeg[K_Preview_Con - 10] == 1)
	#endif
		chz_SwingFoot_setmode('L', 0), ifcoll_l = 0;
	
	#ifdef USE_CHZ_RUN
	if(ifcoll_r && chzrun_signal[K_Preview_Con + 9][0] == 0 && F_RFoot.fz > 60.0)
	#else
	if(ifcoll_r && Signal_SupportLeg[K_Preview_Con + 9] == 0 && F_RFoot.fz > 60.0)
	#endif
	chz_SwingFoot_setmode('R', 0), ifcoll_r = 0;
	
	#ifdef USE_CHZ_RUN	
	if(ifcoll_l && chzrun_signal[K_Preview_Con + 9][1] == 0 && F_LFoot.fz > 60.0)
	#else
	if(ifcoll_l && Signal_SupportLeg[K_Preview_Con + 9] == 0 && F_LFoot.fz > 60.0)
	#endif
	chz_SwingFoot_setmode('L', 0), ifcoll_l = 0;

	static double CoMd[6], RFootd[6], LFootd[6], CoMr[6];
	CoMd[0] = Tra_COM.y[K_Preview_Con];
	CoMd[1] = -Tra_COM.x[K_Preview_Con];
	CoMd[2] = Tra_COM.z[K_Preview_Con];
	CoMd[3] = roll_body;
	CoMd[4] = pitch_body;
	CoMd[5] = 0.0;
	RFootd[0] = Tra_RAnkle.y[K_Preview_Con];
	RFootd[1] = -Tra_RAnkle.x[K_Preview_Con];
	RFootd[2] = Tra_RAnkle.z[K_Preview_Con];
	RFootd[3] = 0.0;
	RFootd[4] = -pitch_footr;
	RFootd[5] = 0.0;
	LFootd[0] = Tra_LAnkle.y[K_Preview_Con];
	LFootd[1] = -Tra_LAnkle.x[K_Preview_Con];
	LFootd[2] = Tra_LAnkle.z[K_Preview_Con];
	LFootd[3] = 0.0;
	LFootd[4] = -pitch_footl;
	LFootd[5] = 0.0;
	for(int i = 0; i <= 2; i++) CoMr[i] = CoMd[i];
	CoMr[3] = XS_Roll;
	CoMr[4] = -XS_Pitch;//-20.0 / 57.3;
	CoMr[5] = 0.0;
	
	//test
	// CoMd[0] = 0.0; CoMd[1] = 0.0; CoMd[2] = 0.112 + 0.64 * cos(12.5 / 57.3);
	// CoMd[3] = 0.0; CoMd[4] = 0.0; CoMd[5] = 0.0;
	// RFootd[0] = 0.0; RFootd[1] = -0.08; RFootd[2] = 0.112;
	// RFootd[3] = 0.0; RFootd[4] = 0.0; RFootd[5] = 0.0;
	// LFootd[0] = 0.0; LFootd[1] = 0.08; LFootd[2] = 0.112;
	// LFootd[3] = 0.0; LFootd[4] = 0.0; LFootd[5] = 0.0;
	// CoMr[0] = 0.0; CoMr[1] = 0.0; CoMr[2] = 0.112 + 0.64 * cos(12.5 / 57.3);
	// CoMr[3] = 0.0; CoMr[4] = 0.0; CoMr[5] = 0.0;
	// for(int i = 0; i <= 11; i++) joints[i] = 0.0;
	// joints[2] = 12.5 / 57.3;
	// joints[3] = -25.0 / 57.3;
	// joints[4] = 12.5 / 57.3;
	// joints[8] = 12.5 / 57.3;
	// joints[9] = -25.0 / 57.3;
	// joints[10] = 12.5 / 57.3;
	//test
	
	static double RCon[6], LCon[6];
	chz_SwingFoot_update(CoMd, RFootd, LFootd, CoMr, chz_Swing_joints);
	if(K_Preview_Con > 1) for(int i = 0; i <= 2; i++) chz_comr_last[i] = chz_comr[i];
	chz_SwingFoot_outputcomr(chz_comr);
	chz_comr[0] -= Tra_COM.x[K_Preview_Con];
	chz_comr[1] -= Tra_COM.y[K_Preview_Con];
	chz_comr[2] -= Tra_COM.z[K_Preview_Con];
	if(K_Preview_Con == 1) for(int i = 0; i <= 2; i++) chz_comr_last[i] = chz_comr[i];
	chz_SwingFoot_outputcon(RCon, LCon);
	chz_log[21] = -124.0;
	for (int i = 0; i <= 5; i++) chz_log[22 + 0 + i] = RCon[i];
	for(int i = 0; i <= 5; i++) chz_log[22 + 6 + i] = LCon[i];
	// chz_log[22 + 12] = -0.01 * ifcoll_r;
	// chz_log[22 + 13] = -0.01 * ifcoll_l;
	//for(int i = 0; i <= 2; i++) chz_log[22 + 12 + i] = chz_comr[i];
	
	//Tra_RAnkle.x[K_Preview_Con] += -RCon[1];
	//Tra_RAnkle.y[K_Preview_Con] += RCon[0];
	Tra_RAnkle.z[K_Preview_Con] += RCon[2];
	pitch_footr += -RCon[4];
	chz_Swing_footroll_r = RCon[3];
	//Tra_LAnkle.x[K_Preview_Con] += -LCon[1];
	//Tra_LAnkle.y[K_Preview_Con] += LCon[0];
	Tra_LAnkle.z[K_Preview_Con] += LCon[2];
	pitch_footl += -LCon[4];
	chz_Swing_footroll_l = LCon[3];
	if(Tra_RAnkle.x[K_Preview_Con] - Tra_LAnkle.x[K_Preview_Con] < 0.13)
	{
		double temp = (Tra_RAnkle.x[K_Preview_Con] + Tra_LAnkle.x[K_Preview_Con]) / 2.0;
		Tra_RAnkle.x[K_Preview_Con] = temp + 0.13 / 2.0;
		Tra_LAnkle.x[K_Preview_Con] = temp - 0.13 / 2.0;
	}
}

void chz_waistcomp_go()
{
	if(K_Preview_Con == 1)
	{
		chz_WaistComp_setparam('R', 1.0 / 57.3, 40);
		chz_WaistComp_setparam('L', 1.0 / 57.3, 40);
	}
	
	static double ifinit = 1, ifend = 1;
	if(ifinit && chzrun_signal[K_Preview_Con][1] == 1)
	{
		ifinit = 0;
		chz_WaistComp_setmode('R', 'A');
	}
	if(ifinit && chzrun_signal[K_Preview_Con][2] == 1)
	{
		ifinit = 0;
		chz_WaistComp_setmode('L', 'A');
	}
	if(!ifinit && ifend && chzrun_signal[K_Preview_Con][0] == 0 && chzrun_signal[K_Preview_Con][2] == 0)
	{
		ifend = 0;
		chz_WaistComp_setmode('L', 'Z');
		chz_WaistComp_setmode('R', 'Z');
	}
	if(chzrun_signal[K_Preview_Con - 1][0] == 1 && chzrun_signal[K_Preview_Con - 0][0] == 0)
		chz_WaistComp_setmode('R', 'A');
	if(chzrun_signal[K_Preview_Con - 1][0] == 0 && chzrun_signal[K_Preview_Con - 0][0] == 1)
		chz_WaistComp_setmode('R', 'Z');
	if(chzrun_signal[K_Preview_Con - 1][1] == 1 && chzrun_signal[K_Preview_Con - 0][1] == 0)
		chz_WaistComp_setmode('L', 'A');
	if(chzrun_signal[K_Preview_Con - 1][1] == 0 && chzrun_signal[K_Preview_Con - 0][1] == 1)
		chz_WaistComp_setmode('L', 'Z');

	static double WaistConr = 0.0, WaistConl = 0.0;
	chz_WaistComp_updatecon(&WaistConr, &WaistConl);
	
	PreCon_LegJoint.qr[2] -= 0.5 * WaistConr;
	//PreCon_LegJoint.ql[2] += 0.00 * 0.5 * WaistConl;
	
	PreCon_LegJoint.qr[6] -= 0.5 * WaistConr;
	//PreCon_LegJoint.ql[6] += 0.00 * 0.5 * WaistConl;
	
	chz_log[13] = WaistConr;
	chz_log[14] = WaistConl;
}

void PreviewControl_Tra_Generate()
{
	int i,j;
#ifndef USE_DCCPGFRAME
	#ifndef USE_SLQR_METHOD
		//XYCOM_Tra_PreConCal( Tra_ZMP.x , Tra_ZMP.y , K_Preview_Con );
		XYCOM_Tra_PreConCal_New( Tra_ZMP.x , Tra_ZMP.y , K_Preview_Con );
	#else
		XYCOM_Tra_SLQR( Tra_ZMP.x , Tra_ZMP.y , K_Preview_Con );	
	#endif	
#endif
	
	//chz_checkIMUbias();
	
	if (K_Preview_Con < 50)
	{
		check_IMU_bias(K_Preview_Con, XS_Pitch, XS_Roll);
	}
	
	// must renew in every cycle
	roll_body = 0.0;
	pitch_body = 0.0;
	pitch_footr = 0.0;
	pitch_footl = 0.0;
	
	#ifdef USE_CHZ_RUN
		// printf("\n--------------\nUse CHZ RUN\n-------------\n");
		Tra_ZMP.x[K_Preview_Con] = chzrun_com[K_Preview_Con][0];
		Tra_ZMP.y[K_Preview_Con] = chzrun_com[K_Preview_Con][1];
		
		Tra_COM.x[K_Preview_Con] = chzrun_com[K_Preview_Con][2];
		Tra_VCOM.x[K_Preview_Con] = chzrun_com[K_Preview_Con][3];
		Tra_ACOM.x[K_Preview_Con] = chzrun_com[K_Preview_Con][4];
		
		Tra_COM.y[K_Preview_Con] = chzrun_com[K_Preview_Con][5];
		Tra_VCOM.y[K_Preview_Con] = chzrun_com[K_Preview_Con][6];
		Tra_ACOM.y[K_Preview_Con] = chzrun_com[K_Preview_Con][7];
		
		Tra_COM.z[K_Preview_Con] = chzrun_com[K_Preview_Con][8];
		Tra_VCOM.z[K_Preview_Con] = chzrun_com[K_Preview_Con][9];
		
		#ifdef USE_SDFAST_MF
			if(chzrun_signal[K_Preview_Con][2] == 0) Tra_ACOM.z[K_Preview_Con] = 0.0;
			else Tra_ACOM.z[K_Preview_Con] = chzrun_MFsd[K_Preview_Con][2] / m_robot - 9.8;
			for(int i = 0; i <= 5; i++) chz_desired_MF[i] = chzrun_MFsd[K_Preview_Con][i];
		#else
			Tra_ACOM.z[K_Preview_Con] = chzrun_com[K_Preview_Con][15];
			chz_desired_MF[0] = chzrun_MF[K_Preview_Con][1];
			chz_desired_MF[1] = -chzrun_MF[K_Preview_Con][3];
			chz_desired_MF[2] = Tra_ACOM.z[K_Preview_Con] * (m_robot + 9.8);
			chz_desired_MF[3] = chzrun_MF[K_Preview_Con][2];
			chz_desired_MF[4] = chzrun_MF[K_Preview_Con][0];
			chz_desired_MF[5] = 0.0;
		#endif
		if(chzrun_signal[K_Preview_Con][2] == 3) Tra_ACOM.z[K_Preview_Con] = -9.8;
		
		Tra_RAnkle.x[K_Preview_Con] = chzrun_foot[K_Preview_Con][0];
		Tra_LAnkle.x[K_Preview_Con] = chzrun_foot[K_Preview_Con][1];
		Tra_RAnkle.y[K_Preview_Con] = chzrun_foot[K_Preview_Con][2];
		Tra_LAnkle.y[K_Preview_Con] = chzrun_foot[K_Preview_Con][3];
		Tra_RAnkle.z[K_Preview_Con] = chzrun_foot[K_Preview_Con][4] + ANKLE_HEIGHT_CHZ;
		Tra_LAnkle.z[K_Preview_Con] = chzrun_foot[K_Preview_Con][5] + ANKLE_HEIGHT_CHZ;
	#endif
	
	#ifdef USE_CHZ_BIAS
		static double n_bias = 300;
		if(K_Preview_Con == 0)
		{
			chz_Bias_com[0] = -0.0; // x
			chz_Bias_com[1] = 0.0; // y
			chz_Bias_com[2] = 0.0;
			chz_Bias_com[3] = deg2rad(0.0);
			chz_Bias_com[4] = deg2rad(0.0);
		}
		Tra_COM.x[K_Preview_Con] += chz_Bias_com[0] * chz_Bias_getval(1.0 * K_Preview_Con / n_bias);
		Tra_COM.y[K_Preview_Con] += chz_Bias_com[1] * chz_Bias_getval(1.0 * K_Preview_Con / n_bias);
		Tra_COM.z[K_Preview_Con] += chz_Bias_com[2] * chz_Bias_getval(1.0 * K_Preview_Con / n_bias);
	#endif
	
	#ifdef USE_CHZ_AUTOBIAS
		static double bias_x = 0.0, bias_y = 0.0;
		static double bias_pitch = 0.0, bias_roll = 0.0;
		static double com_sta[3];
		static double ankle_sta[6];
		if(K_Preview_Con == 0)
		{
			com_sta[0] = Tra_COM.x[K_Preview_Con];
			com_sta[1] = Tra_COM.y[K_Preview_Con];
			com_sta[2] = Tra_COM.z[K_Preview_Con];
			ankle_sta[0] = Tra_RAnkle.x[K_Preview_Con];
			ankle_sta[1] = Tra_RAnkle.y[K_Preview_Con];
			ankle_sta[2] = Tra_RAnkle.z[K_Preview_Con];
			ankle_sta[3] = Tra_LAnkle.x[K_Preview_Con];
			ankle_sta[4] = Tra_LAnkle.y[K_Preview_Con];
			ankle_sta[5] = Tra_LAnkle.z[K_Preview_Con];
		}
		Tra_COM.x[K_Preview_Con] = com_sta[0];
		Tra_COM.y[K_Preview_Con] = com_sta[1];
		Tra_COM.z[K_Preview_Con] = com_sta[2];
		Tra_RAnkle.x[K_Preview_Con] = ankle_sta[0];
		Tra_RAnkle.y[K_Preview_Con] = ankle_sta[1];
		Tra_RAnkle.z[K_Preview_Con] = ankle_sta[2];
		Tra_LAnkle.x[K_Preview_Con] = ankle_sta[3];
		Tra_LAnkle.y[K_Preview_Con] = ankle_sta[4];
		Tra_LAnkle.z[K_Preview_Con] = ankle_sta[5];
		pitch_body = 0.0;
		roll_body = 0.0;
		
		if(K_Preview_Con <= 8000)
		{
			bias_pitch += 0.0005 * (XS_Pitch);
			bias_roll += -0.0005 * (XS_Roll);
			//bias_x += -0.0008 / (4.0 * 43.0 * 9.8 / 0.16) * (F_RFoot.fz - F_LFoot.fz);
			bias_x += -0.00008 * (P_ZMPRel_B.px);
			bias_y += -0.00008 * (P_ZMPRel_B.py);
		}
		
		bias_pitch = fmin(fmax(bias_pitch, -0.05), 0.05);
		bias_roll = fmin(fmax(bias_roll, -0.05), 0.05);
		bias_x = fmin(fmax(bias_x, -0.04), 0.04);
		bias_y = fmin(fmax(bias_y, -0.04), 0.04);
		
		Tra_COM.x[K_Preview_Con] += bias_x;
		Tra_COM.y[K_Preview_Con] += bias_y;
		pitch_body += bias_pitch;
		roll_body += bias_roll;
		if(K_Preview_Con == 8000)
		{
			printf("%.8f\n%.8f\n%.8f\n%.8f\n", bias_x, bias_y, bias_pitch, bias_roll);
		}
	#endif
	
	//printf("%lf\n", delta_fz);
	LegJoint_Calculate( &Tra_RAnkle, &Tra_LAnkle, &Tra_COM, K_Preview_Con);
	P_ZMPRef_W.px = Tra_ZMP.x[K_Preview_Con];
	P_ZMPRef_W.py = Tra_ZMP.y[K_Preview_Con];
	P_ZMPRef_W.pz = 0.0;
	P_COMRef_W.px = Tra_COM.x[K_Preview_Con];
	P_COMRef_W.py = Tra_COM.y[K_Preview_Con];
	P_COMRef_W.pz = Tra_COM.z[K_Preview_Con];
	P_ZMPRef_B = RefZMP_in_Body(&P_ZMPRef_W, &P_COMRef_W);//********//
	P_ZMPRel_B = Calculate_RealZMP_FromForceSensor(&F_RFoot, &F_LFoot, &PreCon_LegJoint);

	// cal ankle B
	P_RAnkleRef_W.px = Tra_RAnkle.x[K_Preview_Con];
	P_RAnkleRef_W.py = Tra_RAnkle.y[K_Preview_Con];
	P_RAnkleRef_W.pz = Tra_RAnkle.z[K_Preview_Con];
	P_RAnkleRef_B = RefZMP_in_Body(&P_RAnkleRef_W, &P_COMRef_W);
	P_LAnkleRef_W.px = Tra_LAnkle.x[K_Preview_Con];
	P_LAnkleRef_W.py = Tra_LAnkle.y[K_Preview_Con];
	P_LAnkleRef_W.pz = Tra_LAnkle.z[K_Preview_Con];
	P_LAnkleRef_B = RefZMP_in_Body(&P_LAnkleRef_W, &P_COMRef_W);
	//printf("%lf\n", F_LFoot.fz);

	//chz filter for IMU and ZMP
	if(K_Preview_Con == 100) 
	{
		chz_LPFilters_init(0, 0.0), chz_LPFilters_init(1, 0.0);
		chz_LPFilters_init(2, 0.0), chz_LPFilters_init(3, 0.0);
	}
	if(K_Preview_Con >= 101) 
	{
		chz_XZMP_filted = chz_LPFilters_update(0, P_ZMPRel_B.px - P_ZMPRef_B.px);
		chz_YZMP_filted = chz_LPFilters_update(1, P_ZMPRel_B.py - P_ZMPRef_B.py);
		chz_Pitch_filted = chz_LPFilters_update(2, -XS_Pitch - chz_Pitch_last);
		chz_Roll_filted = chz_LPFilters_update(3, XS_Roll - chz_Roll_last);
	}
	
	#ifdef USE_CHZ_RUN
		pitch_footr = chzrun_foot[K_Preview_Con][6];
		pitch_footl = chzrun_foot[K_Preview_Con][7];
		pitch_body = chzrun_com[K_Preview_Con][11];
		roll_body = chzrun_com[K_Preview_Con][13];
	#endif
	
	#ifdef USE_CHZ_BIAS
		pitch_body += chz_Bias_com[3] * chz_Bias_getval(1.0 * K_Preview_Con / n_bias);
		roll_body += chz_Bias_com[4] * chz_Bias_getval(1.0 * K_Preview_Con / n_bias);
	#endif
	
	chz_Pitch_last = pitch_body;
	chz_Roll_last = roll_body;
	
	#ifdef USE_CHZ_LOWLEVELFOOTSTEP
		chz_lowlevelfootstep_go();
	#endif
	
	#ifdef USE_CHZ_FOOTDOWN
		chz_footdown_go();
	#endif
	
	#ifdef DCC_SPLINE_MOTION
		Tra_COM.x[K_Preview_Con] =	  Motion_Position_out.OutPut.Com_x[K_Preview_Con];
		Tra_COM.y[K_Preview_Con] =	  Motion_Position_out.OutPut.Com_y[K_Preview_Con];
		Tra_COM.z[K_Preview_Con] =	  Motion_Position_out.OutPut.Com_z[K_Preview_Con];
		Tra_RAnkle.x[K_Preview_Con] = Motion_Position_out.OutPut.Ankle_R_x[K_Preview_Con];
		Tra_RAnkle.y[K_Preview_Con] = Motion_Position_out.OutPut.Ankle_R_y[K_Preview_Con];
		Tra_RAnkle.z[K_Preview_Con] = Motion_Position_out.OutPut.Ankle_R_z[K_Preview_Con];
		Tra_LAnkle.x[K_Preview_Con] = Motion_Position_out.OutPut.Ankle_L_x[K_Preview_Con];
		Tra_LAnkle.y[K_Preview_Con] = Motion_Position_out.OutPut.Ankle_L_y[K_Preview_Con];
		Tra_LAnkle.z[K_Preview_Con] = Motion_Position_out.OutPut.Ankle_L_z[K_Preview_Con];
		roll_body = Motion_Position_out.OutPut.Body_Roll[K_Preview_Con];
		pitch_body = Motion_Position_out.OutPut.Body_Pitch[K_Preview_Con];
		Tra_ZMP.x[K_Preview_Con] = Motion_Position_out.OutPut.ZMP_x[K_Preview_Con];
		Tra_ZMP.y[K_Preview_Con] = Motion_Position_out.OutPut.ZMP_y[K_Preview_Con];
		P_ZMPRef_W.px = Tra_ZMP.x[K_Preview_Con];
		P_ZMPRef_W.py = Tra_ZMP.y[K_Preview_Con];
		P_ZMPRef_B = RefZMP_in_Body(&P_ZMPRef_W, &P_COMRef_W);
	#endif
	
	#ifdef USE_ZCOMPLIANCE
		Tra_RAnkle.z[K_Preview_Con] += DCC_rpz.e;
		Tra_LAnkle.z[K_Preview_Con] += DCC_lpz.e;
	#endif

	#ifdef USE_VMC
		VMC_Update();
		VMC_Control();
		Tra_RAnkle.z[K_Preview_Con] += Deta_RAnkle_Z;
		Tra_LAnkle.z[K_Preview_Con] += Deta_LAnkle_Z;
	#endif
	#ifdef FZ_BIAS
		if(K_Preview_Con >= 1 && K_Preview_Con <= 20 )
		{
			delta_fz = cal_fz_bias(delta_fz, K_Preview_Con);
		}
		micro_delta_fz = bias_micro * delta_fz;
	#endif

	// rc
	#ifdef USE_STACOMPLIANCE
		Stand_Compliance_la();
		roll_body = DCC_SC.la.r;
		Tra_COM.x[K_Preview_Con] += DCC_SC.la.e;
		Stand_Compliance_sa();
		pitch_body = -DCC_SC.sa.r;
		Tra_COM.y[K_Preview_Con] += DCC_SC.sa.e;
		
	#endif
	
	#ifdef USE_RESISTANT_COMPLIANCE
		VMM_RC_controller(1.0, K_Preview_Con);
		roll_body = 1.0 * RC_Comp_Cont.late.r;
		pitch_body = -0.0 * RC_Comp_Cont.sagi.r;		
		Tra_COM.x[K_Preview_Con] += (1.0 * RC_Comp_Cont.late.e - 0.35 * RC_Comp_Cont.late.r);
		Tra_COM.y[K_Preview_Con] += 0.0 * (1.0 * RC_Comp_Cont.sagi.e + 0.2 * RC_Comp_Cont.sagi.r);
	#endif
	
	#ifdef USE_COM_REGULATOR
		Tra_RAnkle.z[K_Preview_Con] += 1.0 * FeetComp.Rfoot.delta_z;
		Tra_LAnkle.z[K_Preview_Con] += 1.0 * FeetComp.Lfoot.delta_z;
		CoM_Sta10(K_Preview_Con);
		roll_body = 0.0 * mic_rot * Md_ZMP_PosRot.roll;
		pitch_body =0.0 *  mic_rot * Md_ZMP_PosRot.pitch;
		Tra_COM.x[K_Preview_Con] +=  0.0 * (Md_ZMP_PosRot.x + mic_rot *Md_ZMP_PosRot.x_roll);
		Tra_COM.y[K_Preview_Con] +=  0.0 * (Md_ZMP_PosRot.y + mic_rot *Md_ZMP_PosRot.y_pitch);
	#endif
	#ifdef USE_MODEL_ZMP
		Tra_RAnkle.x[K_Preview_Con] += 0.0 * Step_Adjust.Rfoot.e_x;
		Tra_RAnkle.y[K_Preview_Con] += 0.0 * Step_Adjust.Rfoot.e_y;
		Tra_RAnkle.z[K_Preview_Con] += 0.0 * Step_Adjust.Rfoot.e_z;
		Tra_LAnkle.x[K_Preview_Con] += 0.0 * Step_Adjust.Lfoot.e_x;
		Tra_LAnkle.y[K_Preview_Con] += 0.0 * Step_Adjust.Lfoot.e_y;
		Tra_LAnkle.z[K_Preview_Con] += 0.0 * Step_Adjust.Lfoot.e_z;
	#endif
	
	#ifdef TEST_CHEST_POSTURE_CON 
		Chest_Posture_Con(0.0 * GQ_omegaY, 0.0 * GQ_omegaX, -GQ_Pitch, GQ_Roll);
		roll_body += 0.0 * chest_roll.e;
		pitch_body += 0.0 * chest_pitch.e;
	#endif
	#ifdef TUNE_K_ROT
		AutoTune_K_Rot(K_Preview_Con);
	#endif

	
	#ifdef USE_UNEVEN_TRAILBLAZER
		HUBO19(K_Preview_Con);
		Tra_COM.x[K_Preview_Con] += 1.0 * Delta_COM.x;
		Tra_COM.y[K_Preview_Con] += 1.25 * Delta_COM.y;
	#endif
	#ifdef USE_HUBO_COMPLIANCE
		Tra_LAnkle.z[K_Preview_Con] += 1.0 * HUBO_foot_com.Lfoot.z.e;
		Tra_RAnkle.z[K_Preview_Con] += 1.0 * HUBO_foot_com.Rfoot.z.e;
	#endif
	#ifdef USE_TEST_FOOTFT_CON
		Cal_FootFT_Test(K_Preview_Con);
		FootFT_Con(); 
		Tra_RAnkle.z[K_Preview_Con] += Rfoot_zctrl.e;
		Tra_LAnkle.z[K_Preview_Con] += Lfoot_zctrl.e;
	#endif
	
	#ifdef USE_TPC															  //printf("P_ZMPRef_B:%f,%f,%f\n",P_ZMPRef_B.px,P_ZMPRef_B.py,P_ZMPRef_B.pz);
		Calculate_DetaCOM_FromTPC(&P_ZMPRel_B, &P_ZMPRef_B);
		
		Tra_COM.x[K_Preview_Con] += 1.0 * P_DetaCOM.px;
		Tra_COM.y[K_Preview_Con] += 1.0 * P_DetaCOM.py;
	#endif
	
	
		//printf("%f\n", pitch_body);
		// NQP
	#ifdef USE_NQP_BALANCE
		body_agl_d[0] = -pitch_body;
		body_agl_d[1] = roll_body;
		NQP_balance_con(K_Preview_Con, XS_Pitch, XS_Roll, body_agl_d);
		Tra_COM.x[K_Preview_Con] += 1.0 * NQP_test.e.x;
		Tra_COM.y[K_Preview_Con] += 1.0 * NQP_test.e.y;
		roll_body  += 1.0 * NQP_test.phi.roll;
		pitch_body += 1.0 * NQP_test.phi.pitch;
	#endif
	
	//printf("%f, %f, ", pitch_body * 57.3, XS_Pitch * 57.3);
	#ifdef USE_CHZ_RUN
		chzpitch = -1.0;
	#endif
	
	#ifdef USE_RUN_Con
		#ifdef USE_CHZ_RUN
			DCC_RunningControl(chzpitch * chzrun_com[K_Preview_Con - 10][11], roll_body, XS_Pitch - IMU_bias.pitch, XS_Roll - IMU_bias.roll, 0.0, 0.0, K_Preview_Con);
		#else
			DCC_RunningControl(chzpitch * pitch_body, roll_body, XS_Pitch - IMU_bias.pitch, XS_Roll - IMU_bias.roll, 0.0, 0.0, K_Preview_Con);
		#endif
	#ifdef RUN_ConVal_AddiOn
		Tra_COM.x[K_Preview_Con] += DCC_Run.BodyPos.x;
		Tra_COM.y[K_Preview_Con] += DCC_Run.BodyPos.y;
		pitch_body -= DCC_Run.BodyRot.pitch;
		roll_body  += DCC_Run.BodyRot.roll;
		Tra_RAnkle.z[K_Preview_Con] += 1.0 * DCC_Run.RfootPos.z;
		Tra_LAnkle.z[K_Preview_Con] += 1.0 * DCC_Run.LfootPos.z;
		// TPCFoot
		Tra_RAnkle.x[K_Preview_Con] -= 1.0 * DCC_Run.RfootPos.x;
		Tra_RAnkle.y[K_Preview_Con] -= 1.0 * DCC_Run.RfootPos.y;
		Tra_LAnkle.x[K_Preview_Con] -= 1.0 * DCC_Run.LfootPos.x;
		Tra_LAnkle.y[K_Preview_Con] -= 1.0 * DCC_Run.LfootPos.y;
	#endif
	#endif


	//printf("%f\n", pitch_body * 57.3);
		fnvDccControlUpdate(K_Preview_Con); // pitch??
		pitch_body = -pitch_body;
		//printf("%f\n", pitch_body);
// TPCMPC
#ifdef TPCMPC
	// P_ZMPRel_B.px = 0.1;
		double dVeZmpRefx_250x1[nNumPre][1];
		double dVeZmpRefy_250x1[nNumPre][1];
		double dVeStatex_3x1[nStateNum][1];
		double dVeStatey_3x1[nStateNum][1];

		double TPCxLimit[6] = { -0.03, 0.03, -1.5, 1.5, -10.0, 10.0 };
		double TPCyLimit[6] = { -0.03, 0.03, -1.5, 1.5, -10.0, 10.0 };

		double TPCMPC_x_bias = 0.0;
		double TPCMPC_y_bias = -0.0 * 0.03;

		for (int i = 0; i < nNumPre; i++) {
			//dVeZmpRefx_250x1[i][0] = Tra_ZMP.x[K_Preview_Con + i] - Tra_COM.x[K_Preview_Con + i] - P_ZMPRel_B.px;
			//dVeZmpRefy_250x1[i][0] = Tra_ZMP.y[K_Preview_Con + i] - Tra_COM.y[K_Preview_Con + i] - P_ZMPRel_B.py;
			dVeZmpRefx_250x1[i][0] = Tra_ZMP.x[K_Preview_Con] + LIPM_ZMP_x - Tra_COM.x[K_Preview_Con] - P_ZMPRel_B.px;
			dVeZmpRefy_250x1[i][0] = Tra_ZMP.y[K_Preview_Con] + LIPM_ZMP_y - Tra_COM.y[K_Preview_Con] - P_ZMPRel_B.py;
			//dVeZmpRefx_250x1[i][0] = 0.0;
			//dVeZmpRefy_250x1[i][0] = 0.0;
		}
		dVeStatex_3x1[0][0] = P_ZMPRel_B.px - (P_ZMPRef_B.px + LIPM_ZMP_x) + TPCMPC_x_bias;
		dVeStatex_3x1[1][0] = x_MPCTPC;
		dVeStatex_3x1[2][0] = dx_MPCTPC;
		dVeStatey_3x1[0][0] = P_ZMPRel_B.py - (P_ZMPRef_B.py + LIPM_ZMP_y) + TPCMPC_y_bias;
		dVeStatey_3x1[1][0] = y_MPCTPC;
		dVeStatey_3x1[2][0] = dy_MPCTPC;

		fnvTPCMPCCalConval(dVeZmpRefx_250x1, dVeZmpRefy_250x1, dVeStatex_3x1, dVeStatey_3x1);
		fnvIntegLimit(&x_MPCTPC, &dx_MPCTPC, dTPCMPCConval[0], TPCxLimit, CONTROL_T);
		fnvIntegLimit(&y_MPCTPC, &dy_MPCTPC, dTPCMPCConval[1], TPCyLimit, CONTROL_T);

		Tra_COM.x[K_Preview_Con] = 1.0 * x_MPCTPC;
		Tra_COM.y[K_Preview_Con] = 1.0 * y_MPCTPC;
#endif

	#ifdef USE_CHZ_LANDING
		chz_landingforce_go();
	#endif
	
	#ifdef USE_CHZ_SWINGFOOT
		chz_swingfoot_go();
	#endif
	
	
	///////////////////////////////////////////////////////////////////////////////////
	// protection
	PreCon_LegJoint_old = PreCon_LegJoint;
	LegJoint_Calculate( &Tra_RAnkle, &Tra_LAnkle, &Tra_COM, K_Preview_Con);
	//if (K_Preview_Con == 500) PreCon_LegJoint.ql[3] = asin(-0.1); // check hop protect
	/*if (K_Preview_Con > 1400) {
		q2_addi += 0.0001;
		PreCon_LegJoint.ql[2] -= q2_addi;
		PreCon_LegJoint.ql[6] += q2_addi;
		PreCon_LegJoint.qr[2] += q2_addi;
		PreCon_LegJoint.qr[6] -= q2_addi;
	}*/
	///////////////////////////////////////////////////////////////////////////////////

	#ifdef USE_CHZ_SWINGFOOT
		PreCon_LegJoint.qr[6] += chz_Swing_footroll_r;
		PreCon_LegJoint.ql[6] += chz_Swing_footroll_l;
	#endif
	
	#ifdef USE_RUN_Con
	#ifdef RUN_ConVal_AddiOn
		PreCon_LegJoint.qr[5] += DCC_Run.RfootRot.pitch;
		PreCon_LegJoint.qr[6] += DCC_Run.RfootRot.roll;
		PreCon_LegJoint.ql[5] += DCC_Run.LfootRot.pitch;
		PreCon_LegJoint.ql[6] += DCC_Run.LfootRot.roll;
	#endif
	#endif
#ifdef USE_DCC_CONTROLFRAME
		PreCon_LegJoint.qr[6] += roll_footr;
		PreCon_LegJoint.ql[6] += roll_footl;
#endif
	#ifdef USE_COM_REGULATOR	
		PreCon_LegJoint.qr[5] += FeetComp.Rfoot.delta_pit;
		PreCon_LegJoint.qr[6] += FeetComp.Rfoot.delta_rol;
							   
		PreCon_LegJoint.ql[5] += FeetComp.Lfoot.delta_pit;
		PreCon_LegJoint.ql[6] += FeetComp.Lfoot.delta_rol;
	#endif
	
	#ifdef USE_CHZ_WAISTCOMP
		chz_waistcomp_go();
	#endif
	
	#ifdef USE_WAIST_COMPENSATION
		// Right 
		PreCon_LegJoint.qr[2] -= 1.2 * 1.0 * Comp_Waist[K_Preview_Con]; //2.88good 0.8 * 1.2 *//0.7 * 1.2
		PreCon_LegJoint.qr[6] -= 0.7 * 0.5 * Comp_Waist[K_Preview_Con]; //2.88good 0.4 * 0.3 *//0.5 * 0.3
		// Left                                                         //2.88good 
		PreCon_LegJoint.ql[2] -= 1.2 * 1.0 * Comp_Waist[K_Preview_Con]; //2.88good 0.8 * 1.2 *//0.7 * 1.2
		PreCon_LegJoint.ql[6] -= 0.6 * 0.5 * Comp_Waist[K_Preview_Con]; //2.88good 0.5 * 0.3 *//1.0 * 0.3
		
	#endif
	#ifdef USE_WAIST_COMP_BHR7TEST
		// Right 
		PreCon_LegJoint.qr[2] -= 0.8 * 1.2 * Comp_Waist_bhr7test[K_Preview_Con];
		PreCon_LegJoint.qr[6] -= 0.4 * 0.3 * Comp_Waist_bhr7test[K_Preview_Con];
		// Left                  
		PreCon_LegJoint.ql[2] -= 0.8 * 1.2 * Comp_Waist_bhr7test[K_Preview_Con];
		PreCon_LegJoint.ql[6] -= 0.4 * 0.3 * Comp_Waist_bhr7test[K_Preview_Con];
	#endif 
	
	#ifdef USE_HUBO_COMPLIANCE	
		PreCon_LegJoint.qr[5] += HUBO_foot_com.Rfoot.pit.e;
		PreCon_LegJoint.qr[6] += HUBO_foot_com.Rfoot.rol.e;
							   
		PreCon_LegJoint.ql[5] += HUBO_foot_com.Lfoot.pit.e;
		PreCon_LegJoint.ql[6] += HUBO_foot_com.Lfoot.rol.e;
	#endif
	
	#ifdef USE_DCC_ARMSWING
		dcc_ArmSwing(); //offline if read before PreCon_LegJoint passing 
	#endif
	
	
	
	#ifdef USE_ANKLECOMPLIANCE
			PreCon_LegJoint.qr[5] += 1.0*DCC_rx.e;
			PreCon_LegJoint.qr[6] += 0.0 * DCC_ry.e;
		
			PreCon_LegJoint.ql[5] += 1.0*DCC_lx.e;
			PreCon_LegJoint.ql[6] += 0.0 * DCC_ly.e;
	#endif
	
	#ifdef USE_CHZ_SWINGFOOT
		for(int i = 0; i <= 5; i++)
		{
			chz_Swing_joints[i] = PreCon_LegJoint.qr[i + 1];
			chz_Swing_joints[i + 6] = PreCon_LegJoint.ql[i + 1];
		}
	#endif
	// protection
#ifdef ANGLE_PROTECTION
	for (int i = 0; i < 6; i++) {
		dptJointsNow[i] = PreCon_LegJoint.ql[i + 1];
		dptJointsNow[i + 6] = PreCon_LegJoint.qr[i + 1];
		dptJointsOld[i] = PreCon_LegJoint_old.ql[i + 1];
		dptJointsOld[i + 6] = PreCon_LegJoint_old.qr[i + 1];
	}
	for (int i = 12; i < 23; i++) {
		dptJointsNow[i] = 0.0;
	}
	fnvJointHopCheck(dptJointsNow, dptJointsOld, dptJointsProtect, dHopTol, K_Preview_Con, CONTROL_T); // hop check
	fnvFootCollisionCheck(dptJointsNow, dptJointsOld, dptJointsProtect, dHopTol, K_Preview_Con, CONTROL_T); // collision check
	if (strDCC_Protection.error_flag == 2 || strDCC_Protection.error_flag == 3) { // hop || collision
		Walk_On = 0;
		PreCon_LegJoint.ql[1] = strDCC_Protection.ql_1[K_Preview_Con - strDCC_Protection.k_hop];
		PreCon_LegJoint.ql[2] = strDCC_Protection.ql_2[K_Preview_Con - strDCC_Protection.k_hop];
		PreCon_LegJoint.ql[3] = strDCC_Protection.ql_3[K_Preview_Con - strDCC_Protection.k_hop];
		PreCon_LegJoint.ql[4] = strDCC_Protection.ql_4[K_Preview_Con - strDCC_Protection.k_hop];
		PreCon_LegJoint.ql[5] = strDCC_Protection.ql_5[K_Preview_Con - strDCC_Protection.k_hop];
		PreCon_LegJoint.ql[6] = strDCC_Protection.ql_6[K_Preview_Con - strDCC_Protection.k_hop];
		PreCon_LegJoint.qr[1] = strDCC_Protection.qr_1[K_Preview_Con - strDCC_Protection.k_hop];
		PreCon_LegJoint.qr[2] = strDCC_Protection.qr_2[K_Preview_Con - strDCC_Protection.k_hop];
		PreCon_LegJoint.qr[3] = strDCC_Protection.qr_3[K_Preview_Con - strDCC_Protection.k_hop];
		PreCon_LegJoint.qr[4] = strDCC_Protection.qr_4[K_Preview_Con - strDCC_Protection.k_hop];
		PreCon_LegJoint.qr[5] = strDCC_Protection.qr_5[K_Preview_Con - strDCC_Protection.k_hop];
		PreCon_LegJoint.qr[6] = strDCC_Protection.qr_6[K_Preview_Con - strDCC_Protection.k_hop];
	}
#endif
	for (int i = 0; i < 6; i++) {
		dJointsPositionCmd[i] = PreCon_LegJoint.qr[i + 1];
		dJointsPositionCmd[i + 6] = PreCon_LegJoint.ql[i + 1];
	}
	// diff
#ifdef USE_DIFF_INV
	Q.q1 = PreCon_LegJoint.ql[1];
	Q.q2 = PreCon_LegJoint.ql[2];
	Q.q3 = PreCon_LegJoint.ql[3];
	Q.q4 = PreCon_LegJoint.ql[4];
	Q.q5 = PreCon_LegJoint.ql[5];
	Q.q6 = PreCon_LegJoint.ql[6];
	Phi = cal_old2diff(Q, MECH_PARAS, THETA_ZERO);
	if (K_Preview_Con == 0)
	{
		Phi_reset.q5 = Phi.q5;
		Phi_reset.q6 = Phi.q6;
	}
	PreCon_LegJoint.ql[1] = Phi.q1 - Phi_reset.q1;
	PreCon_LegJoint.ql[2] = Phi.q2 - Phi_reset.q2;
	PreCon_LegJoint.ql[3] = Phi.q3 - Phi_reset.q3;
	PreCon_LegJoint.ql[4] = Phi.q4 - Phi_reset.q4;
	PreCon_LegJoint.ql[5] = Phi.q5 - Phi_reset.q5;
	PreCon_LegJoint.ql[6] = Phi.q6 - Phi_reset.q6;

	Q.q1 = PreCon_LegJoint.qr[1];
	Q.q2 = PreCon_LegJoint.qr[2];
	Q.q3 = PreCon_LegJoint.qr[3];
	Q.q4 = PreCon_LegJoint.qr[4];
	Q.q5 = PreCon_LegJoint.qr[5];
	Q.q6 = PreCon_LegJoint.qr[6];
	Phi = cal_old2diff(Q, MECH_PARAS, THETA_ZERO);
	if (K_Preview_Con == 0)
	{
		Phi_reset.q5 = Phi.q5;
		Phi_reset.q6 = Phi.q6;
	}
	PreCon_LegJoint.qr[1] = Phi.q1 - Phi_reset.q1;
	PreCon_LegJoint.qr[2] = Phi.q2 - Phi_reset.q2;
	PreCon_LegJoint.qr[3] = Phi.q3 - Phi_reset.q3;
	PreCon_LegJoint.qr[4] = Phi.q4 - Phi_reset.q4;
	PreCon_LegJoint.qr[5] = Phi.q5 - Phi_reset.q5;
	PreCon_LegJoint.qr[6] = Phi.q6 - Phi_reset.q6;
#endif
	// diff
	
	for (i = 1; i<3; i++)
	{
		for (j = 1; j<7; j++)
		{
			if (i == 1) // Right 
			{
				Ref_Leg_Joint[i][j] = PreCon_LegJoint.qr[j];
				//Ref_Arm_Joint[i][j] = Tra_ArmJoint.jr[j][K_Preview_Con];
			}
			if (i == 2) // Left
			{
				Ref_Leg_Joint[i][j] = PreCon_LegJoint.ql[j];
				//Ref_Arm_Joint[i][j] = Tra_ArmJoint.jl[j][K_Preview_Con];
			}
		}
	}
	//printf("%lf", Ref_Arm_Joint[1][1]);
	#ifdef USE_DCC_ARMSWING
	Ref_Arm_Joint[1][1] = dcc_qr_arm;
	Ref_Arm_Joint[2][1] = dcc_ql_arm;
	#endif 

	#ifdef USE_XYCOMPLIANCE
		//	Ref_Leg_Joint[1][2] += DCC_rpx.e;
		//	Ref_Leg_Joint[1][3] += DCC_rpy.e;
		//
		//	Ref_Leg_Joint[2][2] += DCC_lpx.e;
		//	Ref_Leg_Joint[2][3] += DCC_lpy.e;
	#endif

	K_Preview_Con++;
	
	if( K_Preview_Con >= N_tcom )
	{
		Walk_On = 0;
		K_Preview_Con = 0;
		
		K_NUM++;
		if(K_NUM >= WALK_PARA_NUM) K_NUM = WALK_PARA_NUM;
		
		printf("\n Pre_Con Walk end! \n");
	}
}


