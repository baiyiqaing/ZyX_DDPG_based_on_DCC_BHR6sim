#pragma	once

// #define USE_RUN_Con

// #define USE_Run_LIPM
// #define USE_Run_TPC
// #define USE_Land_RotCon // LandRot
// #define USE_Run_PostureCon
// #define USE_BALANCE_PRO
// #define USE_Run_Compliance
// #define USE_FLY_PostureCon
// #define USE_FLY_STEPZ // stepz

typedef struct
{
	double x;
	double y;
	double z;
	double pitch;
	double roll;
	double yaw;
}Run_FS;

typedef struct
{
	double x;
	double y;
	double dx;
	double dy;
	double ddx;
	double ddy;
}Run_Horizontal;

typedef struct
{
	double pitch;
	double roll;
	double dpitch;
	double droll;
	double ddpitch;
	double ddroll;
}Run_Rotational;

typedef struct
{
	double x;
	double y;
	double z;
	double dx;
	double dy;
	double dz;
	double ddx;
	double ddy;
	double ddz;
}Run_Positional;

typedef struct
{
	Run_Horizontal BodyPos;
	Run_Rotational BodyRot;
	Run_Positional RfootPos;
	Run_Rotational RfootRot;
	Run_Positional LfootPos;
	Run_Rotational LfootRot;
}Run_ConVal;

Run_Horizontal LIPM_Con(Run_Horizontal LIPM_conval, Run_Horizontal TPC_Run_conval, Run_Rotational body_rot_ref, Run_Rotational body_rot_rel, double Zc, double paras[6]);
Run_Horizontal TPC_Run(Run_Horizontal TPC_Run_conval, Run_Horizontal LIPM_conval, Run_Horizontal zmp_ref, Run_Horizontal zmp_rel, double paras[6], double limit[2], double F_sum);
Run_ConVal TPC_Foot(Run_ConVal ConVal_in, Run_Horizontal LIPM_conval, Run_Horizontal zmp_ref, Run_Horizontal zmp_rel, double F_check[3], double paras[6], double limit[2], int k_pre);
Run_ConVal Compliance_Run_old(Run_ConVal FootCompliance_ConVal, Run_FS Rfoot_ref, Run_FS Lfoot_ref, Run_FS Rfoot_rel, Run_FS Lfoot_rel, double paras[6], double limit[3]);
Run_ConVal Compliance_Run(Run_ConVal FootCompliance_ConVal, Run_FS Rfoot_ref, Run_FS Lfoot_ref, Run_FS Rfoot_rel, Run_FS Lfoot_rel, double paras[6], double limit[2]);
Run_FS DCC_BalancePro(Run_Horizontal LIPM_conval, Run_Rotational body_rot_ref, Run_Rotational body_rot_rel, double Balance_Pro[2], double prolimit[2], double F_sum);
Run_Rotational PostureCon_Run(Run_Rotational bodyrot_conval, Run_Rotational body_rot_ref, Run_Rotational body_rot_rel, double paras[6], double limit[2], double F_sum);
Run_ConVal Fly_Rot_Con(Run_ConVal FlyRot_conval, Run_Rotational Rdel_theta_rel, Run_Rotational Ldel_theta_rel, Run_Rotational del_theta_rel, double paras_body[6], double limit_body[2], double wake_body[2], double paras_foot[2], double limit_foot[2], double paras_step[12], double limit_step[3], double F_sum, double Fz_R, double Fz_L, int k_pre);
Run_ConVal LandingBalance_Rot(Run_ConVal Ref_RotAndPos_ConVal, Run_Rotational Rdel_theta_rel, Run_Rotational Ldel_theta_rel, Run_Rotational Rdel_dtheta_rel, Run_Rotational Ldel_dtheta_rel, Run_Horizontal zmp_ref, Run_Horizontal zmp_rel, Run_FS Rfoot_rel, Run_FS Lfoot_rel, double paras[11], double limit[2], double wake[3], double F_sum, int k_pre);
Run_ConVal ContactCon(Run_ConVal contactconval, Run_ConVal vd, Run_FS Rfoot_ref, Run_FS Lfoot_ref, Run_FS Rfoot_rel, Run_FS Lfoot_rel, double paras[5], double limit[1], char mode, double paras_stepdown[4], int k_pre);
void DCC_RunningControl(double pitch_ref, double roll_ref, double  pitch_sen, double roll_sen, double pitch_con_old, double roll_con_old, int k_pre);

typedef struct
{
	double ZMP_Lag_T;
	double IMU_Lag_T;
	double Fz_Lag_T;

	double pitch_bias;
	double zmpbias_micro;
	double zmpx_bias;
	double zmpy_bias;
	double m_robot_bias;
	double roll_ampli;
	double tau_pitch_bias;
	double tau_roll_bias;

	double Zc;
	double paras_LIPM[6];

	double paras_TPC[6];
	double limit_TPC[2];

	double paras_GRFC_old[6];
	double limit_GRFC_old[3];

	double paras_GRFC[6];
	double limit_GRFC[2];

	double paras_Rot[6];
	double limit_Rot[2];

	double Balance_Pro[2];
	double Micro_Pro[2];
	double limit_Pro[2];

	double paras_FlyBody[6];
	double limit_FlyBody[2];
	double wake_FlyBody[2];

	double paras_FlyFoot[4];
	double limit_FlyFoot[2];
	double wake_FlyFoot[2];
	double paras_Step[12];
	double limit_Step[3];
															 // LandRot
	double paras_Land[11];
	double limit_Land[2];
	double wake_Land[3];																   // contact
	double paras_Cont[5];
	double limit_Cont[1];
	double paras_Stepdown[4];
	int k_down;
	int k_down_fore;
	double H_ankle;

	double addi_pitch;
	double addi_Rfoot_pitch;
	double addi_Lfoot_pitch;
	double roll_bias_foot;
	double pitch_bias_foot;
}DCCRunParms;

extern DCCRunParms dccRunParms;
