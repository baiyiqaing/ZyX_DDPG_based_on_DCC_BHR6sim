#include "DCC_RunCon.h"
#include "Tra_Generate.h"
#include "DCC_filters.h"
#include "Dcc_lib\Base\dcc_con_base.h"
#include "math.h" // LandRot
extern int chzrun_signal[30000][3];
extern double chzrun_MF[30000][4];
extern double pitch_footr;
extern double pitch_footl;

Run_ConVal DCC_Run;
Run_ConVal FootCompliance_ConVal;
Run_ConVal FlyRot_ConVal;
Run_Horizontal LIPM_ConVal;
Run_Horizontal TPC_Run_ConVal;
Run_ConVal Ref_RotAndPos_ConVal; // LandRot
Run_Rotational BodyRot_ConVal = { 0.0 };
Run_Rotational delta_Rot_old;
Run_FS ADD_Trq_Ref;
Run_Rotational IMU_filtered;
Run_Horizontal ZMP_filtered;
Run_ConVal stepz_pitch, stepz_roll; // stepz
Run_Rotational Rdel_theta_rel, Ldel_theta_rel, del_theta_rel;
double FzR_filtered = 0.0;
double FzL_filtered = 0.0;
double TxR_filtered = 0.0;
double TxL_filtered = 0.0;
double TyR_filtered = 0.0;
double TyL_filtered = 0.0;
double FzR_ref = 0.0;
double FzL_ref = 0.0;
double FzR_rel = 0.0;
double FzL_rel = 0.0;
double bias_x_sum[] = { 0.0 };
double bias_y_sum[] = { 0.0 };
double Moment_pitch_re;
double Moment_roll_re;
double footpitch_re;
double footroll_re;
double pitch_sen_bias = 0.0;
double roll_sen_bias  = 0.0;

double Rz, Lz; // for test sake

double deltax_re  = 0.0;
double deltay_re  = 0.0;
double deltadx_re = 0.0;
double deltady_re = 0.0;

double ddpitch_re;
double dpitch_re;
double pitch_re;

// contact
double sum_delta_R_Fz = 0.0;
double sum_delta_L_Fz = 0.0;
Run_ConVal ContactConVal = { 0.0 };
char mode_StepDown = 'S';

//Chz
extern double chz_XZMP_filted;
//Chz

// re 
Run_FS Rfoot_ref_re, Lfoot_ref_re, Rfoot_rel_re, Lfoot_rel_re;
Run_Horizontal zmp_rel_re, zmp_ref_re, delta_com_re, delta_vcom_re;

// TPCMPC
double LIPM_ZMP_x = 0.0;
double LIPM_ZMP_y = 0.0;
extern double x_MPCTPC;
extern double y_MPCTPC;
extern double dx_MPCTPC;
extern double dy_MPCTPC;
double Trq_Ref_pitch_re;
double Trq_Rel_pitch_re;

// TPCFoot
Run_ConVal TPCFoot_ConVal = { 0.0 };
Run_ConVal TPCFoot_ConVal_re = { 0.0 };
//
double ref_pitch_re = 0.0;
double rel_pitch_re = 0.0;
double con_pitch_re = 0.0;
double ref_roll_re = 0.0;
double rel_roll_re = 0.0;
double con_roll_re = 0.0;

// double trqLIPM_max[] = { -19.0, 29.0, 15.0 }; // stepgood very important!!
double trqLIPM_max[] = { -19.0, 25.0, 15.0 }; // very important!!

// balance pro
double additor_flag = 0.0;
double additor_pit = 0.0;
double additor_rol = 0.0;
double ktau_pitch  = 600.0;
double kdtau_pitch = 60.0;
double ktau_roll   = 420.0;
double kdtau_roll  = 42.0;
double Pittau_limit[2] = { -25, 27 };
double Roltau_limit[2] = { -20, 20 };

// TPCMPC
Run_Horizontal LIPM_Con(Run_Horizontal LIPM_conval, Run_Horizontal TPC_Run_conval, Run_Rotational body_rot_ref, Run_Rotational body_rot_rel, double Zc, double paras[6])
{
	double kp_x = paras[0];	double kp_y = paras[0 + 3];
	double kv_x = paras[1];	double kv_y = paras[1 + 3];
	double kz_x = paras[2];	double kz_y = paras[2 + 3];
	double LIPMLimit_x[2] = { -0.3, 0.3 };
	double LIPMLimit_y[2] = { -0.4, 0.4 };

	// double slowLIPM_x;
	// double slowLIPM_y;

	double delta_com_x;  double delta_com_y;
	double delta_vcom_x; double delta_vcom_y;
	
	double Posture = 0.0;
	#ifdef USE_Run_PostureCon
		Posture = 1.0;
	#endif
	double delta_pitch = body_rot_rel.pitch - (body_rot_ref.pitch + Posture * BodyRot_ConVal.pitch);
	double delta_roll = body_rot_rel.roll - (body_rot_ref.roll + Posture * BodyRot_ConVal.roll);
	double ddelta_pitch = 0.0;
	double ddelta_roll = 0.0;
	
	// re
	ref_pitch_re = body_rot_ref.pitch;
	rel_pitch_re = body_rot_rel.pitch;
	con_pitch_re = BodyRot_ConVal.pitch;
	ref_roll_re =   body_rot_ref.roll;
	rel_roll_re =   body_rot_rel.roll;
	con_roll_re = BodyRot_ConVal.roll;

	ddelta_pitch = (delta_pitch - delta_Rot_old.pitch) / CONTROL_T;
	ddelta_roll = (delta_roll - delta_Rot_old.roll) / CONTROL_T;
	delta_Rot_old.dpitch = Filter_TimeLag(delta_Rot_old.dpitch, ddelta_pitch, CONTROL_T, 0.01);
	delta_Rot_old.droll = Filter_TimeLag(delta_Rot_old.droll, ddelta_roll, CONTROL_T, 0.01);
	delta_Rot_old.pitch = delta_pitch;
	delta_Rot_old.roll = delta_roll;

	delta_com_x = TPC_Run_conval.x + Zc * sin(delta_roll);
	delta_com_y = TPC_Run_conval.y - Zc * sin(delta_pitch);
	delta_vcom_x = TPC_Run_conval.dx + Zc * delta_Rot_old.droll;
	delta_vcom_y = TPC_Run_conval.dy - Zc * delta_Rot_old.dpitch;
	deltax_re = Zc * delta_roll;
	deltay_re = Zc * delta_pitch;
	deltadx_re = Zc * delta_Rot_old.droll;
	deltady_re = Zc * delta_Rot_old.dpitch;
#ifdef TPCMPC
	delta_com_x = x_MPCTPC + 1.0 * Zc * delta_roll;
	delta_com_y = y_MPCTPC + 1.0 * Zc * delta_pitch;
	delta_vcom_x = dx_MPCTPC + 1.0 * Zc * delta_Rot_old.droll;
	delta_vcom_y = dy_MPCTPC + 1.0 * Zc * delta_Rot_old.dpitch;
#endif
#ifdef USE_Run_LIPM_0
	LIPM_conval.x = kp_x * delta_com_x + kv_x * delta_vcom_x + 1.0 * kz_x * (P_ZMPRel_B.px - P_ZMPRef_B.px);
	LIPM_conval.y = kp_y * delta_com_y + kv_y * delta_vcom_y + 1.0 * kz_y * (P_ZMPRel_B.py - P_ZMPRef_B.py);
#endif
#ifdef USE_Run_LIPM_1
	LIPM_conval.dx = kp_x * delta_com_x + kv_x * delta_vcom_x - kz_x * LIPM_conval.x;
	LIPM_conval.x  = LIPM_conval.x + LIPM_conval.dx * CONTROL_T;
	LIPM_conval.dy = kp_y * delta_com_y + kv_y * delta_vcom_y - kz_y * LIPM_conval.y;
	LIPM_conval.y  = LIPM_conval.y + LIPM_conval.dy * CONTROL_T;
#endif

	delta_com_re.x = delta_com_x;
	delta_com_re.y = delta_com_y;
	delta_vcom_re.x = delta_vcom_x;
	delta_vcom_re.y = delta_vcom_y;

	LIPM_conval.x = fndLimit(LIPM_conval.x, LIPMLimit_x);
	LIPM_conval.y = fndLimit(LIPM_conval.y, LIPMLimit_y);
	LIPM_ZMP_x = LIPM_conval.x;
	LIPM_ZMP_y = LIPM_conval.y;
	

	additor_pit = -ktau_pitch * (body_rot_rel.pitch - body_rot_ref.pitch) - kdtau_pitch * delta_Rot_old.dpitch;
    additor_rol = -ktau_roll * (body_rot_rel.roll - body_rot_ref.roll) - kdtau_roll * delta_Rot_old.droll;
	additor_pit = fndLimit(additor_pit, Pittau_limit);
	additor_rol = fndLimit(additor_rol, Roltau_limit);
	
	return LIPM_conval;
}

Run_Horizontal TPC_Run(Run_Horizontal TPC_Run_conval, Run_Horizontal LIPM_conval, Run_Horizontal zmp_ref, Run_Horizontal zmp_rel, double paras[6], double limit[2], double F_sum)
{
	double kz_x = paras[0];	double kz_y = paras[0 + 3];
	double kp_x = paras[1];	double kp_y = paras[1 + 3];
	double kd_x = paras[2]; double kd_y = paras[2 + 3];

	double limit_x = limit[0]; double limit_y = limit[1];

	// rec 20201125
	if (F_sum < 0.2 * m_robot * 9.8) { // fly
		kz_x = 0.0;
		kz_y = 0.0;
	}

	double tpc_x;   double tpc_y;
	double tpc_dx;  double tpc_dy;
	double tpc_ddx; double tpc_ddy;
	double x_old = TPC_Run_conval.x;	 double y_old = TPC_Run_conval.y;
	double dx_old = TPC_Run_conval.dx;	 double dy_old = TPC_Run_conval.dy;
	double ddx_old = TPC_Run_conval.ddx; double ddy_old = TPC_Run_conval.ddy;

	#ifdef USE_Run_LIPM
		zmp_ref.x += LIPM_conval.x;
		zmp_ref.y += LIPM_conval.y;
	#endif
	zmp_rel_re.x = zmp_rel.x;
	zmp_rel_re.y = zmp_rel.y;
	zmp_ref_re.x = zmp_ref.x;
	zmp_ref_re.y = zmp_ref.y;

	tpc_ddx = kz_x * (zmp_rel.x - zmp_ref.x) - kp_x * x_old - kd_x * dx_old;
	if (tpc_ddx > 100.0) {
		tpc_ddx = 100.0; }
	if (tpc_ddx < -100.0) {
		tpc_ddx = -100.0; }
	tpc_dx = dx_old + tpc_ddx * CONTROL_T;
	if (tpc_dx > 10.0) {
		tpc_dx = 10.0;
		tpc_ddx = (tpc_dx - dx_old) / CONTROL_T;}
	if (tpc_dx < -10.0) {
		tpc_dx = -10.0;
		tpc_ddx = (tpc_dx - dx_old) / CONTROL_T;}
	tpc_x = x_old + tpc_dx * CONTROL_T;
	if (tpc_x > limit_x) {
		tpc_x = limit_x;
		tpc_dx = (tpc_x - x_old) / CONTROL_T;}
	if (tpc_x < -limit_x) {
		tpc_x = -limit_x;
		tpc_dx = (tpc_x - x_old) / CONTROL_T;}

	tpc_ddy = kz_y * (zmp_rel.y - zmp_ref.y) - kp_y * y_old - kd_y * dy_old;
	if (tpc_ddy > 100.0) {
		tpc_ddy = 100.0; }
	if (tpc_ddy < -100.0) {
		tpc_ddy = -100.0; }
	tpc_dy = dy_old + tpc_ddy * CONTROL_T;
	if (tpc_dy > 10.0) {
		tpc_dy = 10.0;
		tpc_ddy = (tpc_dy - dy_old) / CONTROL_T;
	}
	if (tpc_dy < -10.0) {
		tpc_dy = -10.0;
		tpc_ddy = (tpc_dy - dy_old) / CONTROL_T;
	}
	tpc_y = y_old + tpc_dy * CONTROL_T;
	if (tpc_y > limit_y) {
		tpc_y = limit_y;
		tpc_dy = (tpc_y - y_old) / CONTROL_T;}
	if (tpc_y < -limit_y) {
		tpc_y = -limit_y;
		tpc_dy = (tpc_y - y_old) / CONTROL_T;}

	TPC_Run_conval.x   = tpc_x;
	TPC_Run_conval.dx  = tpc_dx;
	TPC_Run_conval.ddx = tpc_ddx;
	TPC_Run_conval.y   = tpc_y;
	TPC_Run_conval.dy  = tpc_dy;
	TPC_Run_conval.ddy = tpc_ddy;
	
	#ifndef USE_Run_TPC
		TPC_Run_conval.x = 0.0;
		TPC_Run_conval.y = 0.0;
	#endif

	return TPC_Run_conval;
}

void getSafeInt(double *x, double *dx, double *ddx, double T, double limit[2])
{
	double old_x = *x;
	double old_dx = *dx;

	*dx = *dx + (*ddx)*T;
	*x = *x + (*dx)*T;
	if (*x > limit[1])
	{
		*x = limit[1];
		*dx = (*x - old_x) / T;
	}
	if(*x < limit[0])
	{
		*x = limit[0];
		*dx = (*x - old_x) / T;
	}
}

// TPCFoot
Run_ConVal TPC_Foot(Run_ConVal ConVal_in, Run_Horizontal LIPM_conval, Run_Horizontal zmp_ref, Run_Horizontal zmp_rel, double F_check[3], double paras[6], double limit[2], int k_pre)
{
	Run_ConVal ConVal_out = { 0.0 };
	double F_sum = F_check[0];
	double F_R = F_check[1];
	double F_L = F_check[2];
	double kz_x = paras[0];	double kz_y = paras[0 + 3];
	double kp_x = paras[1];	double kp_y = paras[1 + 3];
	double kd_x = paras[2]; double kd_y = paras[2 + 3];
	double dLimits_x[6] = { -limit[0], limit[0], -10.0, 10.0, -50.0, 50.0 };
	double dLimits_y[6] = { -limit[1], limit[1], -10.0, 10.0, -50.0, 50.0 };
	double ddxR = 0.0, ddyR = 0.0, ddxL = 0.0, ddyL = 0.0;

	double limit_x = limit[0]; double limit_y = limit[1];

		//printf("hehe0\n");
	if (F_R > 0.1 * m_robot * GRAVITY && F_L > 0.1 * m_robot * GRAVITY) { // D sup in reality
		// Rfoot
		ddxR = kz_x * (zmp_rel.x - zmp_ref.x) - kp_x * ConVal_in.RfootPos.x - kd_x * ConVal_in.RfootPos.dx;
		ddyR = kz_y * (zmp_rel.y - zmp_ref.y) - kp_y * ConVal_in.RfootPos.y - kd_y * ConVal_in.RfootPos.dy;
		fnvIntegLimit(&ConVal_in.RfootPos.x, &ConVal_in.RfootPos.dx, ddxR, dLimits_x, CONTROL_T);
		fnvIntegLimit(&ConVal_in.RfootPos.y, &ConVal_in.RfootPos.dy, ddyR, dLimits_y, CONTROL_T);
		// Lfoot
		ddxL = kz_x * (zmp_rel.x - zmp_ref.x) - kp_x * ConVal_in.LfootPos.x - kd_x * ConVal_in.LfootPos.dx;
		ddyL = kz_y * (zmp_rel.y - zmp_ref.y) - kp_y * ConVal_in.LfootPos.y - kd_y * ConVal_in.LfootPos.dy;
		fnvIntegLimit(&ConVal_in.LfootPos.x, &ConVal_in.LfootPos.dx, ddxL, dLimits_x, CONTROL_T);
		fnvIntegLimit(&ConVal_in.LfootPos.y, &ConVal_in.LfootPos.dy, ddyL, dLimits_y, CONTROL_T);
	}
	else { // Other
		if (Tra_RAnkle.z[k_pre] <= H_ANKLE + 1e-6 && Tra_LAnkle.z[k_pre] > H_ANKLE + 1e-6) { // R sup
			if (F_R > 0.1 * m_robot * GRAVITY) { // R touch
				// Rfoot
				ddxR = kz_x * (zmp_rel.x - zmp_ref.x) - kp_x * ConVal_in.RfootPos.x - kd_x * ConVal_in.RfootPos.dx;
				ddyR = kz_y * (zmp_rel.y - zmp_ref.y) - kp_y * ConVal_in.RfootPos.y - kd_y * ConVal_in.RfootPos.dy;
				fnvIntegLimit(&ConVal_in.RfootPos.x, &ConVal_in.RfootPos.dx, ddxR, dLimits_x, CONTROL_T);
				fnvIntegLimit(&ConVal_in.RfootPos.y, &ConVal_in.RfootPos.dy, ddyR, dLimits_y, CONTROL_T);
				// Lfoot
				ddxL = -kp_x * ConVal_in.LfootPos.x - kd_x * ConVal_in.LfootPos.dx;
				ddyL = -kp_y * ConVal_in.LfootPos.y - kd_y * ConVal_in.LfootPos.dy;
				fnvIntegLimit(&ConVal_in.LfootPos.x, &ConVal_in.LfootPos.dx, ddxL, dLimits_x, CONTROL_T);
				fnvIntegLimit(&ConVal_in.LfootPos.y, &ConVal_in.LfootPos.dy, ddyL, dLimits_y, CONTROL_T);
			}
			else {
				// Rfoot
				ddxR = -kp_x * ConVal_in.RfootPos.x - kd_x * ConVal_in.RfootPos.dx;
				ddyR = -kp_y * ConVal_in.RfootPos.y - kd_y * ConVal_in.RfootPos.dy;
				fnvIntegLimit(&ConVal_in.RfootPos.x, &ConVal_in.RfootPos.dx, ddxR, dLimits_x, CONTROL_T);
				fnvIntegLimit(&ConVal_in.RfootPos.y, &ConVal_in.RfootPos.dy, ddyR, dLimits_y, CONTROL_T);
				// Lfoot
				ddxL = -kp_x * ConVal_in.LfootPos.x - kd_x * ConVal_in.LfootPos.dx;
				ddyL = -kp_y * ConVal_in.LfootPos.y - kd_y * ConVal_in.LfootPos.dy;
				fnvIntegLimit(&ConVal_in.LfootPos.x, &ConVal_in.LfootPos.dx, ddxL, dLimits_x, CONTROL_T);
				fnvIntegLimit(&ConVal_in.LfootPos.y, &ConVal_in.LfootPos.dy, ddyL, dLimits_y, CONTROL_T);
			}
		}
		else if (Tra_RAnkle.z[k_pre] > H_ANKLE + 1e-6 && Tra_LAnkle.z[k_pre] <= H_ANKLE + 1e-6) { // L sup
			if (F_L > 0.1 * m_robot * GRAVITY) { // L touch
				// Rfoot
				ddxR = -kp_x * ConVal_in.RfootPos.x - kd_x * ConVal_in.RfootPos.dx;
				ddyR = -kp_y * ConVal_in.RfootPos.y - kd_y * ConVal_in.RfootPos.dy;
				fnvIntegLimit(&ConVal_in.RfootPos.x, &ConVal_in.RfootPos.dx, ddxR, dLimits_x, CONTROL_T);
				fnvIntegLimit(&ConVal_in.RfootPos.y, &ConVal_in.RfootPos.dy, ddyR, dLimits_y, CONTROL_T);
				// Lfoot
				ddxL = kz_x * (zmp_rel.x - zmp_ref.x) - kp_x * ConVal_in.LfootPos.x - kd_x * ConVal_in.LfootPos.dx;
				ddyL = kz_y * (zmp_rel.y - zmp_ref.y) - kp_y * ConVal_in.LfootPos.y - kd_y * ConVal_in.LfootPos.dy;
				fnvIntegLimit(&ConVal_in.LfootPos.x, &ConVal_in.LfootPos.dx, ddxL, dLimits_x, CONTROL_T);
				fnvIntegLimit(&ConVal_in.LfootPos.y, &ConVal_in.LfootPos.dy, ddyL, dLimits_y, CONTROL_T);
			}
			else {
				// Rfoot
				ddxR = -kp_x * ConVal_in.RfootPos.x - kd_x * ConVal_in.RfootPos.dx;
				ddyR = -kp_y * ConVal_in.RfootPos.y - kd_y * ConVal_in.RfootPos.dy;
				fnvIntegLimit(&ConVal_in.RfootPos.x, &ConVal_in.RfootPos.dx, ddxR, dLimits_x, CONTROL_T);
				fnvIntegLimit(&ConVal_in.RfootPos.y, &ConVal_in.RfootPos.dy, ddyR, dLimits_y, CONTROL_T);
				// Lfoot
				ddxL = -kp_x * ConVal_in.LfootPos.x - kd_x * ConVal_in.LfootPos.dx;
				ddyL = -kp_y * ConVal_in.LfootPos.y - kd_y * ConVal_in.LfootPos.dy;
				fnvIntegLimit(&ConVal_in.LfootPos.x, &ConVal_in.LfootPos.dx, ddxL, dLimits_x, CONTROL_T);
				fnvIntegLimit(&ConVal_in.LfootPos.y, &ConVal_in.LfootPos.dy, ddyL, dLimits_y, CONTROL_T);
			}
		}
		else if (Tra_RAnkle.z[k_pre] > H_ANKLE + 1e-6 && Tra_LAnkle.z[k_pre] > H_ANKLE + 1e-6) { // Fly
			// Rfoot
			ddxR = -kp_x * ConVal_in.RfootPos.x - kd_x * ConVal_in.RfootPos.dx;
			ddyR = -kp_y * ConVal_in.RfootPos.y - kd_y * ConVal_in.RfootPos.dy;
			fnvIntegLimit(&ConVal_in.RfootPos.x, &ConVal_in.RfootPos.dx, ddxR, dLimits_x, CONTROL_T);
			fnvIntegLimit(&ConVal_in.RfootPos.y, &ConVal_in.RfootPos.dy, ddyR, dLimits_y, CONTROL_T);
			// Lfoot
			ddxL = -kp_x * ConVal_in.LfootPos.x - kd_x * ConVal_in.LfootPos.dx;
			ddyL = -kp_y * ConVal_in.LfootPos.y - kd_y * ConVal_in.LfootPos.dy;
			fnvIntegLimit(&ConVal_in.LfootPos.x, &ConVal_in.LfootPos.dx, ddxL, dLimits_x, CONTROL_T);
			fnvIntegLimit(&ConVal_in.LfootPos.y, &ConVal_in.LfootPos.dy, ddyL, dLimits_y, CONTROL_T);
		}
		else { // D sup 
		//printf("hehed\n");
			if (F_sum > 0.2 * m_robot * GRAVITY) { // touch
				// Rfoot
				ddxR = kz_x * (zmp_rel.x - zmp_ref.x) - kp_x * ConVal_in.RfootPos.x - kd_x * ConVal_in.RfootPos.dx;
				ddyR = kz_y * (zmp_rel.y - zmp_ref.y) - kp_y * ConVal_in.RfootPos.y - kd_y * ConVal_in.RfootPos.dy;
				fnvIntegLimit(&ConVal_in.RfootPos.x, &ConVal_in.RfootPos.dx, ddxR, dLimits_x, CONTROL_T);
				fnvIntegLimit(&ConVal_in.RfootPos.y, &ConVal_in.RfootPos.dy, ddyR, dLimits_y, CONTROL_T);
				// Lfoot
				ddxL = kz_x * (zmp_rel.x - zmp_ref.x) - kp_x * ConVal_in.LfootPos.x - kd_x * ConVal_in.LfootPos.dx;
				ddyL = kz_y * (zmp_rel.y - zmp_ref.y) - kp_y * ConVal_in.LfootPos.y - kd_y * ConVal_in.LfootPos.dy;
				fnvIntegLimit(&ConVal_in.LfootPos.x, &ConVal_in.LfootPos.dx, ddxL, dLimits_x, CONTROL_T);
				fnvIntegLimit(&ConVal_in.LfootPos.y, &ConVal_in.LfootPos.dy, ddyL, dLimits_y, CONTROL_T);
			}
			else {
				// Rfoot
				ddxR = -kp_x * ConVal_in.RfootPos.x - kd_x * ConVal_in.RfootPos.dx;
				ddyR = -kp_y * ConVal_in.RfootPos.y - kd_y * ConVal_in.RfootPos.dy;
				fnvIntegLimit(&ConVal_in.RfootPos.x, &ConVal_in.RfootPos.dx, ddxR, dLimits_x, CONTROL_T);
				fnvIntegLimit(&ConVal_in.RfootPos.y, &ConVal_in.RfootPos.dy, ddyR, dLimits_y, CONTROL_T);
				// Lfoot
				ddxL = -kp_x * ConVal_in.LfootPos.x - kd_x * ConVal_in.LfootPos.dx;
				ddyL = -kp_y * ConVal_in.LfootPos.y - kd_y * ConVal_in.LfootPos.dy;
				fnvIntegLimit(&ConVal_in.LfootPos.x, &ConVal_in.LfootPos.dx, ddxL, dLimits_x, CONTROL_T);
				fnvIntegLimit(&ConVal_in.LfootPos.y, &ConVal_in.LfootPos.dy, ddyL, dLimits_y, CONTROL_T);
			}
		}
	}

	ConVal_out = ConVal_in;
	
	// re
	TPCFoot_ConVal_re = ConVal_out;
	

	return ConVal_out;
}

// dd
double Thresh_pit[2] = { -5.0, 5.0 };
double Thresh_rol[2] = { -3.0, 3.0 };
// TPCMPC
Run_ConVal Compliance_Run_old(Run_ConVal FootCompliance_ConVal, Run_FS Rfoot_ref, Run_FS Lfoot_ref, Run_FS Rfoot_rel, Run_FS Lfoot_rel, double paras[6], double limit[3])
{
	double kp_pitch = paras[0]; double kd_pitch = paras[1];
	double kp_roll = paras[2]; double kd_roll = paras[3];
	double kp_zctrl = paras[4]; double kd_zctrl = paras[5];

	// for(int i = 0; i <= 5; i++) printf("%.8f ", paras[i]);
	// printf("\n");

	double limit_pitch = limit[0];
	double limit_roll = limit[1];
	double limit_zctrl = limit[2];

	// FootCompliance_ConVal.RfootRot.dpitch = kp_pitch * (Rfoot_rel.pitch - Rfoot_ref.pitch) - kd_pitch * FootCompliance_ConVal.RfootRot.pitch;
	FootCompliance_ConVal.RfootRot.dpitch = kp_pitch * fndThreshold((Rfoot_rel.pitch - Rfoot_ref.pitch), Thresh_pit) - kd_pitch * FootCompliance_ConVal.RfootRot.pitch;
	if (FootCompliance_ConVal.RfootRot.dpitch >  10.0) FootCompliance_ConVal.RfootRot.dpitch = 10.0;
	if (FootCompliance_ConVal.RfootRot.dpitch < -10.0) FootCompliance_ConVal.RfootRot.dpitch = -10.0;
	FootCompliance_ConVal.RfootRot.pitch = FootCompliance_ConVal.RfootRot.pitch + FootCompliance_ConVal.RfootRot.dpitch * CONTROL_T;
	if (FootCompliance_ConVal.RfootRot.pitch >  limit_pitch) FootCompliance_ConVal.RfootRot.pitch = limit_pitch;
	if (FootCompliance_ConVal.RfootRot.pitch < -limit_pitch) FootCompliance_ConVal.RfootRot.pitch = -limit_pitch;

	FootCompliance_ConVal.RfootRot.droll = kp_roll * fndThreshold((Rfoot_rel.roll - Rfoot_ref.roll), Thresh_rol) - kd_roll * FootCompliance_ConVal.RfootRot.roll;
	if (FootCompliance_ConVal.RfootRot.droll >  10.0) FootCompliance_ConVal.RfootRot.droll = 10.0;
	if (FootCompliance_ConVal.RfootRot.droll < -10.0) FootCompliance_ConVal.RfootRot.droll = -10.0;
	FootCompliance_ConVal.RfootRot.roll = FootCompliance_ConVal.RfootRot.roll + FootCompliance_ConVal.RfootRot.droll * CONTROL_T;
	if (FootCompliance_ConVal.RfootRot.roll >  limit_roll) FootCompliance_ConVal.RfootRot.roll = limit_roll;
	if (FootCompliance_ConVal.RfootRot.roll < -limit_roll) FootCompliance_ConVal.RfootRot.roll = -limit_roll;

	FootCompliance_ConVal.RfootPos.dz = (Rfoot_rel.z - Rfoot_ref.z) / kd_zctrl - kp_zctrl / kd_zctrl * FootCompliance_ConVal.RfootPos.z;
	if (FootCompliance_ConVal.RfootPos.dz >  10.0) FootCompliance_ConVal.RfootPos.dz = 10.0;
	if (FootCompliance_ConVal.RfootPos.dz < -10.0) FootCompliance_ConVal.RfootPos.dz = -10.0;
	FootCompliance_ConVal.RfootPos.z = FootCompliance_ConVal.RfootPos.z + FootCompliance_ConVal.RfootPos.dz * CONTROL_T;
	if (FootCompliance_ConVal.RfootPos.z >  limit_zctrl) FootCompliance_ConVal.RfootPos.z = limit_zctrl;
	if (FootCompliance_ConVal.RfootPos.z < -limit_zctrl) FootCompliance_ConVal.RfootPos.z = -limit_zctrl;

	FootCompliance_ConVal.LfootRot.dpitch = kp_pitch * fndThreshold((Lfoot_rel.pitch - Lfoot_ref.pitch), Thresh_pit) - kd_pitch * FootCompliance_ConVal.LfootRot.pitch;
	if (FootCompliance_ConVal.LfootRot.dpitch >  10.0) FootCompliance_ConVal.LfootRot.dpitch = 10.0;
	if (FootCompliance_ConVal.LfootRot.dpitch < -10.0) FootCompliance_ConVal.LfootRot.dpitch = -10.0;
	FootCompliance_ConVal.LfootRot.pitch = FootCompliance_ConVal.LfootRot.pitch + FootCompliance_ConVal.LfootRot.dpitch * CONTROL_T;
	if (FootCompliance_ConVal.LfootRot.pitch >  limit_pitch) FootCompliance_ConVal.LfootRot.pitch = limit_pitch;
	if (FootCompliance_ConVal.LfootRot.pitch < -limit_pitch) FootCompliance_ConVal.LfootRot.pitch = -limit_pitch;

	FootCompliance_ConVal.LfootRot.droll = kp_roll * fndThreshold((Lfoot_rel.roll - Lfoot_ref.roll), Thresh_rol) - kd_roll * FootCompliance_ConVal.LfootRot.roll;
	if (FootCompliance_ConVal.LfootRot.droll >  10.0) FootCompliance_ConVal.LfootRot.droll = 10.0;
	if (FootCompliance_ConVal.LfootRot.droll < -10.0) FootCompliance_ConVal.LfootRot.droll = -10.0;
	FootCompliance_ConVal.LfootRot.roll = FootCompliance_ConVal.LfootRot.roll + FootCompliance_ConVal.LfootRot.droll * CONTROL_T;
	if (FootCompliance_ConVal.LfootRot.roll >  limit_roll) FootCompliance_ConVal.LfootRot.roll = limit_roll;
	if (FootCompliance_ConVal.LfootRot.roll < -limit_roll) FootCompliance_ConVal.LfootRot.roll = -limit_roll;

	FootCompliance_ConVal.LfootPos.dz = (Lfoot_rel.z - Lfoot_ref.z) / kd_zctrl - kp_zctrl / kd_zctrl * FootCompliance_ConVal.LfootPos.z;
	if (FootCompliance_ConVal.LfootPos.dz >  10.0) FootCompliance_ConVal.LfootPos.dz = 10.0;
	if (FootCompliance_ConVal.LfootPos.dz < -10.0) FootCompliance_ConVal.LfootPos.dz = -10.0;
	FootCompliance_ConVal.LfootPos.z = FootCompliance_ConVal.LfootPos.z + FootCompliance_ConVal.LfootPos.dz * CONTROL_T;
	if (FootCompliance_ConVal.LfootPos.z >  limit_zctrl) FootCompliance_ConVal.LfootPos.z = limit_zctrl;
	if (FootCompliance_ConVal.LfootPos.z < -limit_zctrl) FootCompliance_ConVal.LfootPos.z = -limit_zctrl;

	// printf("%.6f, %.6f\n", FootCompliance_ConVal.LfootRot.pitch, FootCompliance_ConVal.LfootRot.roll);
	return FootCompliance_ConVal;
}
// rec
Run_ConVal Compliance_Run(Run_ConVal FootCompliance_ConVal, Run_FS Rfoot_ref, Run_FS Lfoot_ref, Run_FS Rfoot_rel, Run_FS Lfoot_rel, double paras[6], double limit[2])
{
	double kf_pitch = paras[0]; double kf_roll = paras[0 + 3];
	double kp_pitch = paras[1]; double kp_roll = paras[1 + 3];
	double kd_pitch = paras[2]; double kd_roll = paras[2 + 3];
	double limit_pitch = limit[0];
	double limit_roll = limit[1];

	Run_ConVal FootCompliance_ConVal_old;
	FootCompliance_ConVal_old.RfootRot.pitch = FootCompliance_ConVal.RfootRot.pitch;
	FootCompliance_ConVal_old.RfootRot.roll = FootCompliance_ConVal.RfootRot.roll;
	FootCompliance_ConVal_old.LfootRot.pitch = FootCompliance_ConVal.LfootRot.pitch;
	FootCompliance_ConVal_old.LfootRot.roll = FootCompliance_ConVal.LfootRot.roll;

	if (Rfoot_rel.pitch >  100.0) Rfoot_rel.pitch =  100.0;
	if (Rfoot_rel.roll  >  100.0) Rfoot_rel.roll  =  100.0;
	if (Lfoot_rel.pitch < -100.0) Lfoot_rel.pitch = -100.0;
	if (Lfoot_rel.roll  < -100.0) Lfoot_rel.roll  = -100.0;
	// for(int i = 0; i <= 5; i++) printf("%.8f ", paras[i]);
	// printf("\n");

	FootCompliance_ConVal.RfootRot.ddpitch = kf_pitch * (Rfoot_rel.pitch - Rfoot_ref.pitch) - kp_pitch * FootCompliance_ConVal.RfootRot.pitch - kd_pitch * FootCompliance_ConVal.RfootRot.dpitch;
	if (FootCompliance_ConVal.RfootRot.ddpitch >  20.0) FootCompliance_ConVal.RfootRot.ddpitch =  20.0;
	if (FootCompliance_ConVal.RfootRot.ddpitch < -20.0) FootCompliance_ConVal.RfootRot.ddpitch = -20.0;
	FootCompliance_ConVal.RfootRot.dpitch = FootCompliance_ConVal.RfootRot.dpitch + FootCompliance_ConVal.RfootRot.ddpitch * CONTROL_T;
	if (FootCompliance_ConVal.RfootRot.dpitch >  5.0) FootCompliance_ConVal.RfootRot.dpitch =  5.0;
	if (FootCompliance_ConVal.RfootRot.dpitch < -5.0) FootCompliance_ConVal.RfootRot.dpitch = -5.0;
	FootCompliance_ConVal.RfootRot.pitch = FootCompliance_ConVal.RfootRot.pitch + FootCompliance_ConVal.RfootRot.dpitch * CONTROL_T;
	if (FootCompliance_ConVal.RfootRot.pitch > limit_pitch) {
		FootCompliance_ConVal.RfootRot.pitch = limit_pitch;
		FootCompliance_ConVal.RfootRot.dpitch = (FootCompliance_ConVal.RfootRot.pitch - FootCompliance_ConVal_old.RfootRot.pitch) / CONTROL_T;
	}
	if (FootCompliance_ConVal.RfootRot.pitch < -limit_pitch) {
		FootCompliance_ConVal.RfootRot.pitch = -limit_pitch;
		FootCompliance_ConVal.RfootRot.dpitch = (FootCompliance_ConVal.RfootRot.pitch - FootCompliance_ConVal_old.RfootRot.pitch) / CONTROL_T;
	}

	FootCompliance_ConVal.RfootRot.ddroll = kf_roll * (Rfoot_rel.roll - Rfoot_ref.roll) - kp_roll * FootCompliance_ConVal.RfootRot.roll - kd_roll * FootCompliance_ConVal.RfootRot.droll;
	if (FootCompliance_ConVal.RfootRot.ddroll >  20.0) FootCompliance_ConVal.RfootRot.ddroll =  20.0;
	if (FootCompliance_ConVal.RfootRot.ddroll < -20.0) FootCompliance_ConVal.RfootRot.ddroll = -20.0;
	FootCompliance_ConVal.RfootRot.droll = FootCompliance_ConVal.RfootRot.droll + FootCompliance_ConVal.RfootRot.ddroll * CONTROL_T;
	if (FootCompliance_ConVal.RfootRot.droll >  5.0) FootCompliance_ConVal.RfootRot.droll =  5.0;
	if (FootCompliance_ConVal.RfootRot.droll < -5.0) FootCompliance_ConVal.RfootRot.droll = -5.0;
	FootCompliance_ConVal.RfootRot.roll = FootCompliance_ConVal.RfootRot.roll + FootCompliance_ConVal.RfootRot.droll * CONTROL_T;
	if (FootCompliance_ConVal.RfootRot.roll > limit_roll) {
		FootCompliance_ConVal.RfootRot.roll = limit_roll;
		FootCompliance_ConVal.RfootRot.droll = (FootCompliance_ConVal.RfootRot.roll - FootCompliance_ConVal_old.RfootRot.roll) / CONTROL_T;
	}
	if (FootCompliance_ConVal.RfootRot.roll < -limit_roll) {
		FootCompliance_ConVal.RfootRot.roll = -limit_roll;
		FootCompliance_ConVal.RfootRot.droll = (FootCompliance_ConVal.RfootRot.roll - FootCompliance_ConVal_old.RfootRot.roll) / CONTROL_T;
	}

	FootCompliance_ConVal.LfootRot.ddpitch = kf_pitch * (Lfoot_rel.pitch - Lfoot_ref.pitch) - kp_pitch * FootCompliance_ConVal.LfootRot.pitch - kd_pitch * FootCompliance_ConVal.LfootRot.dpitch;
	if (FootCompliance_ConVal.LfootRot.ddpitch >  20.0) FootCompliance_ConVal.LfootRot.ddpitch =  20.0;
	if (FootCompliance_ConVal.LfootRot.ddpitch < -20.0) FootCompliance_ConVal.LfootRot.ddpitch = -20.0;
	FootCompliance_ConVal.LfootRot.dpitch = FootCompliance_ConVal.LfootRot.dpitch + FootCompliance_ConVal.LfootRot.ddpitch * CONTROL_T;
	if (FootCompliance_ConVal.LfootRot.dpitch >  5.0) FootCompliance_ConVal.LfootRot.dpitch =  5.0;
	if (FootCompliance_ConVal.LfootRot.dpitch < -5.0) FootCompliance_ConVal.LfootRot.dpitch = -5.0;
	FootCompliance_ConVal.LfootRot.pitch = FootCompliance_ConVal.LfootRot.pitch + FootCompliance_ConVal.LfootRot.dpitch * CONTROL_T;
	if (FootCompliance_ConVal.LfootRot.pitch > limit_pitch) {
		FootCompliance_ConVal.LfootRot.pitch = limit_pitch;	 
		FootCompliance_ConVal.LfootRot.dpitch = (FootCompliance_ConVal.LfootRot.pitch - FootCompliance_ConVal_old.LfootRot.pitch) / CONTROL_T;
	}
	if (FootCompliance_ConVal.LfootRot.pitch < -limit_pitch) {
		FootCompliance_ConVal.LfootRot.pitch = -limit_pitch;
		FootCompliance_ConVal.LfootRot.dpitch = (FootCompliance_ConVal.LfootRot.pitch - FootCompliance_ConVal_old.LfootRot.pitch) / CONTROL_T;
	}

	FootCompliance_ConVal.LfootRot.ddroll = kf_roll * (Lfoot_rel.roll - Lfoot_ref.roll) - kp_roll * FootCompliance_ConVal.LfootRot.roll - kd_roll * FootCompliance_ConVal.LfootRot.droll;
	if (FootCompliance_ConVal.LfootRot.ddroll >  20.0) FootCompliance_ConVal.LfootRot.ddroll =  20.0;
	if (FootCompliance_ConVal.LfootRot.ddroll < -20.0) FootCompliance_ConVal.LfootRot.ddroll = -20.0;
	FootCompliance_ConVal.LfootRot.droll = FootCompliance_ConVal.LfootRot.droll + FootCompliance_ConVal.LfootRot.ddroll * CONTROL_T;
	if (FootCompliance_ConVal.LfootRot.droll >  5.0) FootCompliance_ConVal.LfootRot.droll =  5.0;
	if (FootCompliance_ConVal.LfootRot.droll < -5.0) FootCompliance_ConVal.LfootRot.droll = -5.0;
	FootCompliance_ConVal.LfootRot.roll = FootCompliance_ConVal.LfootRot.roll + FootCompliance_ConVal.LfootRot.droll * CONTROL_T;
	if (FootCompliance_ConVal.LfootRot.roll > limit_roll) {
		FootCompliance_ConVal.LfootRot.roll = limit_roll;
		FootCompliance_ConVal.LfootRot.droll = (FootCompliance_ConVal.LfootRot.roll - FootCompliance_ConVal_old.LfootRot.roll) / CONTROL_T;
	}
	if (FootCompliance_ConVal.LfootRot.roll < -limit_roll) {
		FootCompliance_ConVal.LfootRot.roll = -limit_roll;
		FootCompliance_ConVal.LfootRot.droll = (FootCompliance_ConVal.LfootRot.roll - FootCompliance_ConVal_old.LfootRot.roll) / CONTROL_T;
	}
	ddpitch_re = FootCompliance_ConVal.RfootRot.ddpitch;
	dpitch_re = FootCompliance_ConVal.RfootRot.dpitch;
	pitch_re = FootCompliance_ConVal.RfootRot.pitch;
	// printf("%.6f, %.6f\n", FootCompliance_ConVal.LfootRot.pitch, FootCompliance_ConVal.LfootRot.roll);
	return FootCompliance_ConVal;
}

Run_Rotational PostureCon_Run(Run_Rotational bodyrot_conval, Run_Rotational body_rot_ref, Run_Rotational body_rot_rel, double paras[6], double limit[2], double F_sum)
{
	double  k_pitch = paras[0];    double  k_roll = paras[0 + 3];
	double kp_pitch = paras[1];	   double kp_roll = paras[1 + 3];
	double kd_pitch = paras[2];	   double kd_roll = paras[2 + 3];
	double limit_pitch = limit[0]; double limit_roll  = limit[1];

	double pitch_old = bodyrot_conval.pitch;
	double roll_old  = bodyrot_conval.roll;

	if (F_sum < 0.2 * m_robot)
	{
		k_pitch = 0.0;
		k_roll  = 0.0;
	}

	bodyrot_conval.ddpitch = k_pitch*(body_rot_ref.pitch - body_rot_rel.pitch) - kp_pitch * bodyrot_conval.pitch - kd_pitch * bodyrot_conval.dpitch;
	if (bodyrot_conval.ddpitch >  50.0) bodyrot_conval.ddpitch =  50.0;
	if (bodyrot_conval.ddpitch < -50.0) bodyrot_conval.ddpitch = -50.0;
	bodyrot_conval.dpitch = bodyrot_conval.dpitch + bodyrot_conval.ddpitch * CONTROL_T;
	if (bodyrot_conval.dpitch >  5.0) bodyrot_conval.dpitch =  5.0;
	if (bodyrot_conval.dpitch < -5.0) bodyrot_conval.dpitch = -5.0;
	bodyrot_conval.pitch = bodyrot_conval.pitch + bodyrot_conval.dpitch * CONTROL_T;
	if (bodyrot_conval.pitch > limit_pitch) {
		bodyrot_conval.pitch = limit_pitch;
		bodyrot_conval.dpitch = (bodyrot_conval.pitch - pitch_old) / CONTROL_T; }
	if (bodyrot_conval.pitch < -limit_pitch) {
		bodyrot_conval.pitch = -limit_pitch;
		bodyrot_conval.dpitch = (bodyrot_conval.pitch - pitch_old) / CONTROL_T; }

		
		// printf("%f, %f\n", bodyrot_conval.pitch, body_rot_ref.pitch - body_rot_rel.pitch);
	bodyrot_conval.ddroll = k_roll*(body_rot_ref.roll - body_rot_rel.roll) - kp_roll * bodyrot_conval.roll - kd_roll * bodyrot_conval.droll;
	if (bodyrot_conval.ddroll >  50.0) bodyrot_conval.ddroll = 50.0;
	if (bodyrot_conval.ddroll < -50.0) bodyrot_conval.ddroll = -50.0;
	bodyrot_conval.droll = bodyrot_conval.droll + bodyrot_conval.ddroll * CONTROL_T;
	if (bodyrot_conval.droll >  5.0) bodyrot_conval.droll = 5.0;
	if (bodyrot_conval.droll < -5.0) bodyrot_conval.droll = -5.0;
	bodyrot_conval.roll = bodyrot_conval.roll + bodyrot_conval.droll * CONTROL_T;
	if (bodyrot_conval.roll > limit_roll) {
		bodyrot_conval.roll = limit_roll;
		bodyrot_conval.droll = (bodyrot_conval.roll - roll_old) / CONTROL_T; }
	if (bodyrot_conval.roll < -limit_roll) {
		bodyrot_conval.roll = -limit_roll;
		bodyrot_conval.droll = (bodyrot_conval.roll - roll_old) / CONTROL_T; }

	return bodyrot_conval;
}

Run_FS DCC_BalancePro(Run_Horizontal LIPM_conval, Run_Rotational body_rot_ref, Run_Rotational body_rot_rel, double Balance_Pro[2], double prolimit[2], double F_sum)
{
	Run_FS ADD_trq_ref, Ref_trq_LIPM, Ref_trq_Balance;

	ADD_trq_ref.pitch = 0.0;
	ADD_trq_ref.roll  = 0.0;
	Ref_trq_LIPM.pitch = 0.0;
	Ref_trq_LIPM.roll  = 0.0;
	Ref_trq_LIPM.z = 0.0;
	Ref_trq_Balance.pitch = 0.0;
	Ref_trq_Balance.roll  = 0.0;
	Ref_trq_Balance.z = 0.0;
	
	Run_Horizontal deltaZMP;
	deltaZMP.x = LIPM_conval.x + P_ZMPRef_B.px - 0.6 * P_ZMPRel_B.px;
	deltaZMP.y = LIPM_conval.y + P_ZMPRef_B.py - 0.35 * P_ZMPRel_B.py;

	if (F_sum > 0.2 * m_robot * 9.8) {		// touch down
		// LIPM
		#ifdef ADDI_Tor_LIPM
			Ref_trq_LIPM.pitch = 1.0 * deltaZMP.y * m_robot * 9.8;
			Ref_trq_LIPM.roll = -1.0 * deltaZMP.x * m_robot * 9.8;
			//Ref_trq_LIPM.z = (LIPM_conval.x)* m_robot * 9.8 / ankle_width;
			// printf("%f\n", Ref_trq_LIPM.z);
			// printf("%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t\n", ADD_Trq_Ref.pitch, ADD_Trq_Ref.roll);
		#endif

		// Balance Pro
		#ifdef USE_BALANCE_PRO
			Ref_trq_Balance.pitch = Balance_Pro[0] * (body_rot_ref.pitch - body_rot_rel.pitch);
			if (Ref_trq_Balance.pitch >  prolimit[0]) Ref_trq_Balance.pitch =  prolimit[0];
			if (Ref_trq_Balance.pitch < -prolimit[0]) Ref_trq_Balance.pitch = -prolimit[0];
			Ref_trq_Balance.roll  = Balance_Pro[1] * (body_rot_ref.roll  - body_rot_rel.roll );
			if (Ref_trq_Balance.roll  >  prolimit[1]) Ref_trq_Balance.roll  =  prolimit[1];
			if (Ref_trq_Balance.roll  < -prolimit[1]) Ref_trq_Balance.roll  = -prolimit[1];
		#endif
	}
	else if (F_sum < 0.1 * m_robot * 9.8) {	// fly
		// LIPM
		#ifdef ADDI_Tor_LIPM
			Ref_trq_LIPM.pitch = 0.0;
			Ref_trq_LIPM.roll  = 0.0;
		#endif

		// Balance Pro
		#ifdef USE_BALANCE_PRO
			Ref_trq_Balance.pitch = 0.0;
			Ref_trq_Balance.roll  = 0.0;
		#endif
	}
	else {									// transition
		// LIPM
		#ifdef ADDI_Tor_LIPM
			Ref_trq_LIPM.pitch = 0.0;
			Ref_trq_LIPM.roll  = 0.0;
		#endif

		// Balance Pro
		#ifdef USE_BALANCE_PRO
			Ref_trq_Balance.pitch = 0.0;
			Ref_trq_Balance.roll  = 0.0;
		#endif
	}

	ADD_trq_ref.pitch = Ref_trq_LIPM.pitch + Ref_trq_Balance.pitch;
	ADD_trq_ref.roll  = Ref_trq_LIPM.roll  + Ref_trq_Balance.roll;
	ADD_trq_ref.z  = Ref_trq_LIPM.z  + Ref_trq_Balance.z;
	
	if(ADD_trq_ref.pitch <= trqLIPM_max[0]) ADD_trq_ref.pitch = trqLIPM_max[0];
	if(ADD_trq_ref.pitch >= trqLIPM_max[1]) ADD_trq_ref.pitch = trqLIPM_max[1];
	if(ADD_trq_ref.roll >= trqLIPM_max[2]) ADD_trq_ref.roll = trqLIPM_max[2];
	if(ADD_trq_ref.roll <= -trqLIPM_max[2]) ADD_trq_ref.roll = -trqLIPM_max[2];
	
	

	return ADD_trq_ref;
}

// stepz
Run_ConVal Fly_Rot_Con(Run_ConVal FlyRot_conval, Run_Rotational Rdel_theta_rel, Run_Rotational Ldel_theta_rel, Run_Rotational del_theta_rel, double paras_body[6], double limit_body[2], double wake_body[2], double paras_foot[2], double limit_foot[2], double paras_step[12], double limit_step[3], double F_sum, double Fz_R, double Fz_L, int k_pre)
{
	Run_Rotational FootRotAverage;
	FootRotAverage.pitch = del_theta_rel.pitch;
	FootRotAverage.roll = del_theta_rel.roll;

	double km_pitch = paras_body[0]; double km_roll = paras_body[0 + 3];
	double kp_pitch = paras_body[1]; double kp_roll = paras_body[1 + 3];
	double kd_pitch = paras_body[2]; double kd_roll = paras_body[2 + 3];

	double kfp_R = paras_foot[0];
	double kfp_L = paras_foot[0];
	double kfd = paras_foot[1];

	double kth_pitch_R = paras_step[0]; double kth_roll_R = paras_step[0 + 3];
	double kth_pitch_L = paras_step[0]; double kth_roll_L = paras_step[0 + 3];
	double ktp_pitch = paras_step[1]; double ktp_roll = paras_step[1 + 3];
	double ktd_pitch = paras_step[2]; double ktd_roll = paras_step[2 + 3];
	double rth_pitch_R = paras_step[6]; double rth_roll_R = paras_step[6 + 3];
	double rth_pitch_L = paras_step[6]; double rth_roll_L = paras_step[6 + 3];
	double rtp_pitch = paras_step[7]; double rtp_roll = paras_step[7 + 3];
	double rtd_pitch = paras_step[8]; double rtd_roll = paras_step[8 + 3];
	double L_stepRL = Tra_RAnkle.y[k_pre] - Tra_LAnkle.y[k_pre];
	double L_width = ankle_width;

	// touch down check
	if (F_sum > 0.1 * m_robot * 9.8) { // touch down
		km_pitch = 0.0;
		km_roll = 0.0;
		//hehe = 0;
	}
	else {
		// hehe = 1;
	}
	if (Fz_R > 0.1 * m_robot * 9.8) { // R touch down
		kfp_R = 0.5;
	}
	if (Fz_L > 0.1 * m_robot * 9.8) { // L touch down
		kfp_L = 0.5;
	}
	if (FootRotAverage.pitch < wake_body[0] && FootRotAverage.pitch > -wake_body[0]) {
		km_pitch = 0.0;
	}
	if (FootRotAverage.roll  < wake_body[1] && FootRotAverage.roll  > -wake_body[1]) {
		km_roll = 0.0;
	}

#ifdef USE_FLY_STEPZ // stepz
	// foreleg check
	if (L_stepRL > 0) { // R fore
		kth_pitch_L = 0.0;
		kth_roll_L = 0.0;
		rth_pitch_L = 0.0;
		rth_roll_L = 0.0;

	}
	else if (L_stepRL < 0) { // L fore
		kth_pitch_R = 0.0;
		kth_roll_R = 0.0;
		rth_pitch_R = 0.0;
		rth_roll_R = 0.0;
		L_stepRL = -L_stepRL;
	}
	else {
		kth_pitch_R = 0.0;
		kth_pitch_L = 0.0;
		kth_roll_R = 0.0;
		kth_roll_L = 0.0;
		rth_pitch_R = 0.0;
		rth_pitch_L = 0.0;
		rth_roll_R = 0.0;
		rth_roll_L = 0.0;
	}

	// step check
	// Signal_SupportLeg[k_pre] = 1; // for test sake
	if (Signal_SupportLeg[k_pre] == 1) { // r sup
		kth_pitch_R = 0.0;
		kth_roll_R = 0.0;
	}
	else if (Signal_SupportLeg[k_pre] == 2) { // l sup
		kth_pitch_L = 0.0;
		kth_roll_L = 0.0;
	}
	// step z
	double Rstep_pitch_old = stepz_pitch.RfootPos.z;
	double Rstep_roll_old  = stepz_roll.RfootPos.z;
	double Rstep_Rotpitch_old = stepz_pitch.RfootRot.pitch;
	double Rstep_Rotroll_old  = stepz_roll.RfootRot.roll;
	double Lstep_pitch_old = stepz_pitch.LfootPos.z;
	double Lstep_roll_old  = stepz_roll.LfootPos.z;
	double Lstep_Rotpitch_old = stepz_pitch.LfootRot.pitch;
	double Lstep_Rotroll_old  = stepz_roll.LfootRot.roll;

	stepz_pitch.RfootPos.ddz = -kth_pitch_R * L_stepRL * sin(FootRotAverage.pitch) - ktp_pitch * stepz_pitch.RfootPos.z - ktd_pitch * stepz_pitch.RfootPos.dz; // R pitch
	if (stepz_pitch.RfootPos.ddz >  100.0) stepz_pitch.RfootPos.ddz = 100.0;
	if (stepz_pitch.RfootPos.ddz < -100.0) stepz_pitch.RfootPos.ddz = -100.0;
	stepz_pitch.RfootPos.dz = stepz_pitch.RfootPos.dz + stepz_pitch.RfootPos.ddz * CONTROL_T;
	if (stepz_pitch.RfootPos.dz >  10.0) stepz_pitch.RfootPos.dz = 10.0;
	if (stepz_pitch.RfootPos.dz < -10.0) stepz_pitch.RfootPos.dz = -10.0;
	stepz_pitch.RfootPos.z = stepz_pitch.RfootPos.z + stepz_pitch.RfootPos.dz * CONTROL_T;
	if (stepz_pitch.RfootPos.z >  limit_step[0] * 0.5) { stepz_pitch.RfootPos.z = limit_step[0] * 0.5;
		stepz_pitch.RfootPos.dz = (stepz_pitch.RfootPos.z - Rstep_pitch_old) / CONTROL_T;}
	if (stepz_pitch.RfootPos.z < 0.0) { stepz_pitch.RfootPos.z = 0.0;
		stepz_pitch.RfootPos.dz = (stepz_pitch.RfootPos.z - Rstep_pitch_old) / CONTROL_T;}

	stepz_pitch.RfootRot.ddpitch = -rth_pitch_R * FootRotAverage.pitch - rtp_pitch * stepz_pitch.RfootRot.pitch - rtd_pitch * stepz_pitch.RfootRot.dpitch;
	if (stepz_pitch.RfootRot.ddpitch >  50.0) stepz_pitch.RfootRot.ddpitch = 50.0;
	if (stepz_pitch.RfootRot.ddpitch < -50.0) stepz_pitch.RfootRot.ddpitch = -50.0;
	stepz_pitch.RfootRot.dpitch = stepz_pitch.RfootRot.dpitch + stepz_pitch.RfootRot.ddpitch * CONTROL_T;
	if (stepz_pitch.RfootRot.dpitch >  5.0) stepz_pitch.RfootRot.dpitch = 5.0;
	if (stepz_pitch.RfootRot.dpitch < -5.0) stepz_pitch.RfootRot.dpitch = -5.0;
	stepz_pitch.RfootRot.pitch = stepz_pitch.RfootRot.pitch + stepz_pitch.RfootRot.dpitch * CONTROL_T;
	if (stepz_pitch.RfootRot.pitch >  limit_step[1]) { stepz_pitch.RfootRot.pitch = limit_step[1];
		stepz_pitch.RfootRot.dpitch = (stepz_pitch.RfootRot.pitch - Rstep_Rotpitch_old) / CONTROL_T;}
	if (stepz_pitch.RfootRot.pitch < -limit_step[1]) { stepz_pitch.RfootRot.pitch = -limit_step[1];
		stepz_pitch.RfootRot.dpitch = (stepz_pitch.RfootRot.pitch - Rstep_Rotpitch_old) / CONTROL_T;}

	stepz_pitch.LfootPos.ddz = -kth_pitch_L * L_stepRL * sin(FootRotAverage.pitch) - ktp_pitch * stepz_pitch.LfootPos.z - ktd_pitch * stepz_pitch.LfootPos.dz; // L pitch
	if (stepz_pitch.LfootPos.ddz >  100.0) stepz_pitch.LfootPos.ddz = 100.0;
	if (stepz_pitch.LfootPos.ddz < -100.0) stepz_pitch.LfootPos.ddz = -100.0;
	stepz_pitch.LfootPos.dz = stepz_pitch.LfootPos.dz + stepz_pitch.LfootPos.ddz * CONTROL_T;
	if (stepz_pitch.LfootPos.dz >  10.0) stepz_pitch.LfootPos.dz = 10.0;
	if (stepz_pitch.LfootPos.dz < -10.0) stepz_pitch.LfootPos.dz = -10.0;
	stepz_pitch.LfootPos.z = stepz_pitch.LfootPos.z + stepz_pitch.LfootPos.dz * CONTROL_T;
	if (stepz_pitch.LfootPos.z >  limit_step[0] * 0.5) { stepz_pitch.LfootPos.z = limit_step[0] * 0.5;
		stepz_pitch.LfootPos.dz = (stepz_pitch.LfootPos.z - Lstep_pitch_old) / CONTROL_T;}
	if (stepz_pitch.LfootPos.z < 0.0) { stepz_pitch.LfootPos.z = 0.0;
		stepz_pitch.LfootPos.dz = (stepz_pitch.LfootPos.z - Lstep_pitch_old) / CONTROL_T;}

	stepz_pitch.LfootRot.ddpitch = -rth_pitch_L * FootRotAverage.pitch - rtp_pitch * stepz_pitch.LfootRot.pitch - rtd_pitch * stepz_pitch.LfootRot.dpitch;
	if (stepz_pitch.LfootRot.ddpitch >  50.0) stepz_pitch.LfootRot.ddpitch = 50.0;
	if (stepz_pitch.LfootRot.ddpitch < -50.0) stepz_pitch.LfootRot.ddpitch = -50.0;
	stepz_pitch.LfootRot.dpitch = stepz_pitch.LfootRot.dpitch + stepz_pitch.LfootRot.ddpitch * CONTROL_T;
	if (stepz_pitch.LfootRot.dpitch >  5.0) stepz_pitch.LfootRot.dpitch = 5.0;
	if (stepz_pitch.LfootRot.dpitch < -5.0) stepz_pitch.LfootRot.dpitch = -5.0;
	stepz_pitch.LfootRot.pitch = stepz_pitch.LfootRot.pitch + stepz_pitch.LfootRot.dpitch * CONTROL_T;
	if (stepz_pitch.LfootRot.pitch >  limit_step[1]) { stepz_pitch.LfootRot.pitch = limit_step[1];
		stepz_pitch.LfootRot.dpitch = (stepz_pitch.LfootRot.pitch - Lstep_Rotpitch_old) / CONTROL_T;}
	if (stepz_pitch.LfootRot.pitch < -limit_step[1]) { stepz_pitch.LfootRot.pitch = -limit_step[1];
		stepz_pitch.LfootRot.dpitch = (stepz_pitch.LfootRot.pitch - Lstep_Rotpitch_old) / CONTROL_T;}

	stepz_roll.RfootPos.ddz = -kth_roll_R * L_stepRL * sin(FootRotAverage.roll) - ktp_roll * stepz_roll.RfootPos.z - ktd_roll * stepz_roll.RfootPos.dz; // R roll
	if (stepz_roll.RfootPos.ddz >  100.0) stepz_roll.RfootPos.ddz = 100.0;
	if (stepz_roll.RfootPos.ddz < -100.0) stepz_roll.RfootPos.ddz = -100.0;
	stepz_roll.RfootPos.dz = stepz_roll.RfootPos.dz + stepz_roll.RfootPos.ddz * CONTROL_T;
	if (stepz_roll.RfootPos.dz >  10.0) stepz_roll.RfootPos.dz = 10.0;
	if (stepz_roll.RfootPos.dz < -10.0) stepz_roll.RfootPos.dz = -10.0;
	stepz_roll.RfootPos.z = stepz_roll.RfootPos.z + stepz_roll.RfootPos.dz * CONTROL_T;
	if (stepz_roll.RfootPos.z >  limit_step[0] * 0.5) { stepz_roll.RfootPos.z = limit_step[0] * 0.5;
		stepz_roll.RfootPos.dz = (stepz_roll.RfootPos.z - Rstep_roll_old) / CONTROL_T;}
	if (stepz_roll.RfootPos.z < 0.0) { stepz_roll.RfootPos.z = 0.0;
		stepz_roll.RfootPos.dz = (stepz_roll.RfootPos.z - Rstep_roll_old) / CONTROL_T;}

	stepz_roll.RfootRot.ddroll = -rth_roll_R * FootRotAverage.roll - rtp_roll * stepz_roll.RfootRot.roll - rtd_roll * stepz_roll.RfootRot.droll;
	if (stepz_roll.RfootRot.ddroll >  50.0) stepz_roll.RfootRot.ddroll = 50.0;
	if (stepz_roll.RfootRot.ddroll < -50.0) stepz_roll.RfootRot.ddroll = -50.0;
	stepz_roll.RfootRot.droll = stepz_roll.RfootRot.droll + stepz_roll.RfootRot.ddroll * CONTROL_T;
	if (stepz_roll.RfootRot.droll >  5.0) stepz_roll.RfootRot.droll = 5.0;
	if (stepz_roll.RfootRot.droll < -5.0) stepz_roll.RfootRot.droll = -5.0;
	stepz_roll.RfootRot.roll = stepz_roll.RfootRot.roll + stepz_roll.RfootRot.droll * CONTROL_T;
	if (stepz_roll.RfootRot.roll >  limit_step[2]) { stepz_roll.RfootRot.roll = limit_step[2];
		stepz_roll.RfootRot.droll = (stepz_roll.RfootRot.roll - Rstep_Rotroll_old) / CONTROL_T;}
	if (stepz_roll.RfootRot.roll < -limit_step[2]) { stepz_roll.RfootRot.roll = -limit_step[2];
		stepz_roll.RfootRot.droll = (stepz_roll.RfootRot.roll - Rstep_Rotroll_old) / CONTROL_T;}

	stepz_roll.LfootPos.ddz = -kth_roll_L * L_stepRL * sin(FootRotAverage.roll) - ktp_roll * stepz_roll.LfootPos.z - ktd_roll * stepz_roll.LfootPos.dz; // L roll
	if (stepz_roll.LfootPos.ddz >  100.0) stepz_roll.LfootPos.ddz = 100.0;
	if (stepz_roll.LfootPos.ddz < -100.0) stepz_roll.LfootPos.ddz = -100.0;
	stepz_roll.LfootPos.dz = stepz_roll.LfootPos.dz + stepz_roll.LfootPos.ddz * CONTROL_T;
	if (stepz_roll.LfootPos.dz >  10.0) stepz_roll.LfootPos.dz = 10.0;
	if (stepz_roll.LfootPos.dz < -10.0) stepz_roll.LfootPos.dz = -10.0;
	stepz_roll.LfootPos.z = stepz_roll.LfootPos.z + stepz_roll.LfootPos.dz * CONTROL_T;
	if (stepz_roll.LfootPos.z >  limit_step[0] * 0.5) { stepz_roll.LfootPos.z = limit_step[0] * 0.5;
		stepz_roll.LfootPos.dz = (stepz_roll.LfootPos.z - Lstep_roll_old) / CONTROL_T;}
	if (stepz_roll.LfootPos.z < 0.0) { stepz_roll.LfootPos.z = 0.0;
		stepz_roll.LfootPos.dz = (stepz_roll.LfootPos.z - Lstep_roll_old) / CONTROL_T;}

	stepz_roll.LfootRot.ddroll = -rth_roll_L * FootRotAverage.roll - rtp_roll * stepz_roll.LfootRot.roll - rtd_roll * stepz_roll.LfootRot.droll;
	if (stepz_roll.LfootRot.ddroll >  50.0) stepz_roll.LfootRot.ddroll = 50.0;
	if (stepz_roll.LfootRot.ddroll < -50.0) stepz_roll.LfootRot.ddroll = -50.0;
	stepz_roll.LfootRot.droll = stepz_roll.LfootRot.droll + stepz_roll.LfootRot.ddroll * CONTROL_T;
	if (stepz_roll.LfootRot.droll >  5.0) stepz_roll.LfootRot.droll = 5.0;
	if (stepz_roll.LfootRot.droll < -5.0) stepz_roll.LfootRot.droll = -5.0;
	stepz_roll.LfootRot.roll = stepz_roll.LfootRot.roll + stepz_roll.LfootRot.droll * CONTROL_T;
	if (stepz_roll.LfootRot.roll >  limit_step[2]) { stepz_roll.LfootRot.roll = limit_step[2];
		stepz_roll.LfootRot.droll = (stepz_roll.LfootRot.roll - Lstep_Rotroll_old) / CONTROL_T;}
	if (stepz_roll.LfootRot.roll < -limit_step[2]) { stepz_roll.LfootRot.roll = -limit_step[2];
		stepz_roll.LfootRot.droll = (stepz_roll.LfootRot.roll - Lstep_Rotroll_old) / CONTROL_T;}

	FlyRot_conval.RfootPos.z = stepz_pitch.RfootPos.z + 1.0 * stepz_roll.RfootPos.z;
	FlyRot_conval.LfootPos.z = stepz_pitch.LfootPos.z + 1.0 * stepz_roll.LfootPos.z;

	Rz = stepz_pitch.RfootPos.z; Lz = stepz_pitch.LfootPos.z; // for test sake

	// Rdel_theta_rel.pitch -= stepz_pitch.RfootRot.pitch;
	// Ldel_theta_rel.pitch -= stepz_pitch.LfootRot.pitch;
	// Rdel_theta_rel.roll -= stepz_roll.RfootRot.roll;
	// Ldel_theta_rel.roll -= stepz_roll.LfootRot.roll;

#endif

	double pitch_old = FlyRot_conval.BodyRot.pitch;
	double roll_old = FlyRot_conval.BodyRot.roll;

	// body
	FlyRot_conval.BodyRot.ddpitch = km_pitch * FootRotAverage.pitch - kp_pitch * FlyRot_conval.BodyRot.pitch - kd_pitch * FlyRot_conval.BodyRot.dpitch;
	if (FlyRot_conval.BodyRot.ddpitch >  100.0) FlyRot_conval.BodyRot.ddpitch = 100.0;
	if (FlyRot_conval.BodyRot.ddpitch < -100.0) FlyRot_conval.BodyRot.ddpitch = -100.0;
	FlyRot_conval.BodyRot.dpitch = FlyRot_conval.BodyRot.dpitch + FlyRot_conval.BodyRot.ddpitch * CONTROL_T;
	if (FlyRot_conval.BodyRot.dpitch >  10.0) FlyRot_conval.BodyRot.dpitch = 10.0;
	if (FlyRot_conval.BodyRot.dpitch < -10.0) FlyRot_conval.BodyRot.dpitch = -10.0;
	FlyRot_conval.BodyRot.pitch = FlyRot_conval.BodyRot.pitch + FlyRot_conval.BodyRot.dpitch * CONTROL_T;
	if (FlyRot_conval.BodyRot.pitch >  limit_body[0]) {
		FlyRot_conval.BodyRot.pitch = limit_body[0];
		FlyRot_conval.BodyRot.dpitch = (FlyRot_conval.BodyRot.pitch - pitch_old) / CONTROL_T;
	}
	if (FlyRot_conval.BodyRot.pitch < -limit_body[0]) {
		FlyRot_conval.BodyRot.pitch = -limit_body[0];
		FlyRot_conval.BodyRot.droll = (FlyRot_conval.BodyRot.roll - roll_old) / CONTROL_T;
	}

	FlyRot_conval.BodyRot.ddroll = km_roll  * FootRotAverage.roll - kp_roll  * FlyRot_conval.BodyRot.roll - kd_roll  * FlyRot_conval.BodyRot.droll;
	if (FlyRot_conval.BodyRot.ddroll >  100.0) FlyRot_conval.BodyRot.ddroll = 100.0;
	if (FlyRot_conval.BodyRot.ddroll < -100.0) FlyRot_conval.BodyRot.ddroll = -100.0;
	FlyRot_conval.BodyRot.droll = FlyRot_conval.BodyRot.droll + FlyRot_conval.BodyRot.ddroll * CONTROL_T;
	if (FlyRot_conval.BodyRot.droll >  10.0) FlyRot_conval.BodyRot.droll = 10.0;
	if (FlyRot_conval.BodyRot.droll < -10.0) FlyRot_conval.BodyRot.droll = -10.0;
	FlyRot_conval.BodyRot.roll = FlyRot_conval.BodyRot.roll + FlyRot_conval.BodyRot.droll * CONTROL_T;
	if (FlyRot_conval.BodyRot.roll >  limit_body[1]) {
		FlyRot_conval.BodyRot.roll = limit_body[1];
		FlyRot_conval.BodyRot.droll = (FlyRot_conval.BodyRot.roll - roll_old) / CONTROL_T;
	}
	if (FlyRot_conval.BodyRot.roll < -limit_body[1]) {
		FlyRot_conval.BodyRot.roll = -limit_body[1];
		FlyRot_conval.BodyRot.droll = (FlyRot_conval.BodyRot.roll - roll_old) / CONTROL_T;
	}

	// foot
	FlyRot_conval.RfootRot.dpitch = -kfp_R * Rdel_theta_rel.pitch - kfd * FlyRot_conval.RfootRot.pitch;
	if (FlyRot_conval.RfootRot.dpitch >  10.0) FlyRot_conval.RfootRot.dpitch = 10.0;
	if (FlyRot_conval.RfootRot.dpitch < -10.0) FlyRot_conval.RfootRot.dpitch = -10.0;
	FlyRot_conval.RfootRot.pitch = FlyRot_conval.RfootRot.pitch + FlyRot_conval.RfootRot.dpitch * CONTROL_T;
	if (FlyRot_conval.RfootRot.pitch >  limit_foot[0]) FlyRot_conval.RfootRot.pitch = limit_foot[0];
	if (FlyRot_conval.RfootRot.pitch < -limit_foot[0]) FlyRot_conval.RfootRot.pitch = -limit_foot[0];
	FlyRot_conval.RfootRot.droll = -kfp_R * Rdel_theta_rel.roll - kfd * FlyRot_conval.RfootRot.roll;
	if (FlyRot_conval.RfootRot.droll >  10.0) FlyRot_conval.RfootRot.droll = 10.0;
	if (FlyRot_conval.RfootRot.droll < -10.0) FlyRot_conval.RfootRot.droll = -10.0;
	FlyRot_conval.RfootRot.roll = FlyRot_conval.RfootRot.roll + FlyRot_conval.RfootRot.droll * CONTROL_T;
	if (FlyRot_conval.RfootRot.roll >  limit_foot[1]) FlyRot_conval.RfootRot.roll = limit_foot[1];
	if (FlyRot_conval.RfootRot.roll < -limit_foot[1]) FlyRot_conval.RfootRot.roll = -limit_foot[1];

	FlyRot_conval.LfootRot.dpitch = -kfp_L * Ldel_theta_rel.pitch - kfd * FlyRot_conval.LfootRot.pitch;
	if (FlyRot_conval.LfootRot.dpitch >  10.0) FlyRot_conval.LfootRot.dpitch = 10.0;
	if (FlyRot_conval.LfootRot.dpitch < -10.0) FlyRot_conval.LfootRot.dpitch = -10.0;
	FlyRot_conval.LfootRot.pitch = FlyRot_conval.LfootRot.pitch + FlyRot_conval.LfootRot.dpitch * CONTROL_T;
	if (FlyRot_conval.LfootRot.pitch >  limit_foot[0]) FlyRot_conval.LfootRot.pitch = limit_foot[0];
	if (FlyRot_conval.LfootRot.pitch < -limit_foot[0]) FlyRot_conval.LfootRot.pitch = -limit_foot[0];
	FlyRot_conval.LfootRot.droll = -kfp_L * Ldel_theta_rel.roll - kfd * FlyRot_conval.LfootRot.roll;
	if (FlyRot_conval.LfootRot.droll >  10.0) FlyRot_conval.LfootRot.droll = 10.0;
	if (FlyRot_conval.LfootRot.droll < -10.0) FlyRot_conval.LfootRot.droll = -10.0;
	FlyRot_conval.LfootRot.roll = FlyRot_conval.LfootRot.roll + FlyRot_conval.LfootRot.droll * CONTROL_T;
	if (FlyRot_conval.LfootRot.roll >  limit_foot[1]) FlyRot_conval.LfootRot.roll = limit_foot[1];
	if (FlyRot_conval.LfootRot.roll < -limit_foot[1]) FlyRot_conval.LfootRot.roll = -limit_foot[1];

	return FlyRot_conval;
}

// LandRot
int sense_on_flag = 0;
int control_on_flag = 0;
int fore_step_k = 25; // 0.1s
int control_confirmmax_k = 5; // 0.02s
int control_confirm_k = 0;
int control_onmax_k = 50; // 0.2  
int control_releasemax_k = 5; // 0.02s
int control_release_k = 0; // 0.02s
int control_on_k = 0; 
Run_ConVal LandingBalance_Rot(Run_ConVal Ref_RotAndPos_ConVal, Run_Rotational Rdel_theta_rel, Run_Rotational Ldel_theta_rel, Run_Rotational Rdel_dtheta_rel, Run_Rotational Ldel_dtheta_rel, Run_Horizontal zmp_ref, Run_Horizontal zmp_rel, Run_FS Rfoot_rel, Run_FS Lfoot_rel, double paras[11], double limit[2], double wake[3], double F_sum, int k_pre)
{
	Run_Rotational Moment;
	Run_Rotational SupFoot_del_theta, SupFoot_del_dtheta;
	double SupFoot_Fz = 0.0;
	double LandRotConOn_pitch = 1.0;
	double LandRotConOn_roll  = 1.0;

	// cal Moment
	double Kzmp_y    = paras[0]; double Kzmp_x   = paras[0 + 3]; // zmp input
	double Kth_pitch = paras[1]; double Kth_roll = paras[1 + 3]; // rot input
	double Kom_pitch = paras[2]; double Kom_roll = paras[2 + 3]; // ome input
	// cal Rot desired
	double I_robot = paras[6];	// Inertia
	double kp = paras[7];		// kp
	double kd = paras[8];		// kd
	// cal horizontal
	double K_Pitch2Y = paras[9];
	double K_Roll2X  = paras[10];

	// check supleg for Moment calculation
	if (Rfoot_rel.z > 0.1 * m_robot * 9.8 && Lfoot_rel.z <= 0.1 * m_robot * 9.8) { // r sup
		SupFoot_del_theta.pitch = Rdel_theta_rel.pitch;
		SupFoot_del_theta.roll  = Rdel_theta_rel.roll; 
		SupFoot_del_dtheta.pitch = Rdel_dtheta_rel.pitch;
		SupFoot_del_dtheta.roll  = Rdel_dtheta_rel.roll;
		SupFoot_Fz = Rfoot_rel.z;
	}
	else if (Lfoot_rel.z > 0.1 * m_robot * 9.8 && Rfoot_rel.z <= 0.1 * m_robot * 9.8) { // l sup
		SupFoot_del_theta.pitch = Ldel_theta_rel.pitch;
		SupFoot_del_theta.roll  = Ldel_theta_rel.roll; 
		SupFoot_del_dtheta.pitch = Ldel_dtheta_rel.pitch;
		SupFoot_del_dtheta.roll  = Ldel_dtheta_rel.roll;
		SupFoot_Fz = Lfoot_rel.z;
	}
	else if (Lfoot_rel.z > 0.1 * m_robot * 9.8 && Rfoot_rel.z > 0.1 * m_robot * 9.8) { // d sup
		SupFoot_del_theta.pitch = 0.5 * (Rdel_theta_rel.pitch + Ldel_theta_rel.pitch);
		SupFoot_del_theta.roll  = 0.5 * (Rdel_theta_rel.roll  + Ldel_theta_rel.roll);
		SupFoot_del_dtheta.pitch = 0.5 * (Rdel_dtheta_rel.pitch + Ldel_dtheta_rel.pitch);
		SupFoot_del_dtheta.roll  = 0.5 * (Rdel_dtheta_rel.roll  + Ldel_dtheta_rel.roll);
		SupFoot_Fz = F_sum;
	}
	else { // fly
		SupFoot_del_theta.pitch  = 0.0;
		SupFoot_del_theta.roll   = 0.0;
		SupFoot_del_dtheta.pitch = 0.0;
		SupFoot_del_dtheta.roll  = 0.0; 
		SupFoot_Fz = 0.0;
	}

	// check force for Ref_Rot calculation
	footpitch_re = SupFoot_del_theta.pitch; 
	footroll_re  = SupFoot_del_theta.roll; 
	if (SupFoot_del_theta.pitch < wake[0] && SupFoot_del_theta.pitch > wake[1]) {
		// Kzmp_y = 0.0; // check if is needed
		LandRotConOn_pitch = 0.0;
	}
	if (fabs(SupFoot_del_theta.roll ) < wake[2]) {
		// Kzmp_x = 0.0; // check if is needed
		LandRotConOn_roll  = 0.0;
	}
	if (F_sum < 0.1 * m_robot * 9.8) { // fly for test sake
		// Kzmp_y = 0.0; // check if is needed
		// Kzmp_x = 0.0; // check if is needed
		LandRotConOn_pitch = 0.0;
		LandRotConOn_roll= 0.0;
	}
	
	// check touch down state for Ref_Rot calculation
	// if ((Tra_RAnkle.z[k_pre + fore_step_k] < H_ANKLE || Tra_LAnkle.z[k_pre + fore_step_k] < H_ANKLE) && sense_on_flag == 1) { // sense on
	if ( chzrun_signal[k_pre + fore_step_k][2] < 3 && sense_on_flag == 1) { // sense on
		if (F_sum > 0.1 * m_robot * 9.8) { // touch down
			control_confirm_k++;
			control_on_k = 0;
		}
		else {
			control_confirm_k = 0;
			control_on_k = 0;
		}
	}
	if (control_confirm_k == control_confirmmax_k) { // control confirm
		control_confirm_k = 0;
		sense_on_flag = 0; // sense off when control
		control_on_flag = 1;
	}
	if (control_on_flag == 1) {
		control_on_k++;
		if (control_on_k < control_onmax_k) {
			control_on_flag = 1;
		}
		else {
			control_on_k = 0;
			control_on_flag = 0;
		}
	}
	if (sense_on_flag == 0) {
		// if ((Tra_RAnkle.z[k_pre] > H_ANKLE || Tra_LAnkle.z[k_pre] > H_ANKLE)) {
		if (chzrun_signal[k_pre][2] > 2) {
			if ( F_sum < 0.1 * m_robot * 9.8) {
				control_release_k++;
			}
			else {
				control_release_k = 0;
			}
		}
	}
	if (control_release_k == control_releasemax_k) { // sense on
		control_release_k = 0;
		sense_on_flag = 1;
	}
	if (control_on_flag != 1 && chzrun_signal[k_pre][2] != 0) {
		LandRotConOn_pitch = 0.0;
		LandRotConOn_roll  = 0.0;
	}
		
	Moment.pitch =  Kzmp_y * (zmp_rel.y - zmp_ref.y) * SupFoot_Fz - Kth_pitch * SupFoot_del_theta.pitch - 0.0 * Kom_pitch * SupFoot_del_dtheta.pitch; // this three paras needs checking
	Moment.roll  = -Kzmp_x * (zmp_rel.x - zmp_ref.x) * SupFoot_Fz - Kth_roll  * SupFoot_del_theta.roll  - 0.0 * Kom_roll  * SupFoot_del_dtheta.roll;	// this three paras needs checking
	Moment_pitch_re = Moment.pitch;
	Moment_roll_re  = Moment.roll;

	double LandRot_pitch_old = Ref_RotAndPos_ConVal.BodyRot.pitch;
	double LandRot_roll_old  = Ref_RotAndPos_ConVal.BodyRot.roll;

	// pitch
	Ref_RotAndPos_ConVal.BodyRot.ddpitch = -LandRotConOn_pitch * Moment.pitch / I_robot - kp / I_robot * Ref_RotAndPos_ConVal.BodyRot.pitch - kd / I_robot * Ref_RotAndPos_ConVal.BodyRot.dpitch;
	if (Ref_RotAndPos_ConVal.BodyRot.ddpitch >  50.0) Ref_RotAndPos_ConVal.BodyRot.ddpitch =  50.0;
	if (Ref_RotAndPos_ConVal.BodyRot.ddpitch < -50.0) Ref_RotAndPos_ConVal.BodyRot.ddpitch = -50.0;
	Ref_RotAndPos_ConVal.BodyRot.dpitch = Ref_RotAndPos_ConVal.BodyRot.dpitch + Ref_RotAndPos_ConVal.BodyRot.ddpitch * CONTROL_T;
	if (Ref_RotAndPos_ConVal.BodyRot.dpitch >  5.0) Ref_RotAndPos_ConVal.BodyRot.dpitch =  5.0;
	if (Ref_RotAndPos_ConVal.BodyRot.dpitch < -5.0) Ref_RotAndPos_ConVal.BodyRot.dpitch = -5.0;
	Ref_RotAndPos_ConVal.BodyRot.pitch = Ref_RotAndPos_ConVal.BodyRot.pitch + Ref_RotAndPos_ConVal.BodyRot.dpitch * CONTROL_T;
	if (Ref_RotAndPos_ConVal.BodyRot.pitch >  limit[0]) { Ref_RotAndPos_ConVal.BodyRot.pitch =  limit[0];
		Ref_RotAndPos_ConVal.BodyRot.dpitch = (Ref_RotAndPos_ConVal.BodyRot.pitch - LandRot_pitch_old) / CONTROL_T; }
	if (Ref_RotAndPos_ConVal.BodyRot.pitch < -limit[0]) { Ref_RotAndPos_ConVal.BodyRot.pitch = -limit[0];
		Ref_RotAndPos_ConVal.BodyRot.dpitch = (Ref_RotAndPos_ConVal.BodyRot.pitch - LandRot_pitch_old) / CONTROL_T; }

	// roll
	Ref_RotAndPos_ConVal.BodyRot.ddroll  = -LandRotConOn_roll * Moment.roll  / I_robot - kp / I_robot * Ref_RotAndPos_ConVal.BodyRot.roll  - kd / I_robot * Ref_RotAndPos_ConVal.BodyRot.droll;
	if (Ref_RotAndPos_ConVal.BodyRot.ddroll >  50.0) Ref_RotAndPos_ConVal.BodyRot.ddroll =  50.0;
	if (Ref_RotAndPos_ConVal.BodyRot.ddroll < -50.0) Ref_RotAndPos_ConVal.BodyRot.ddroll = -50.0;
	Ref_RotAndPos_ConVal.BodyRot.droll = Ref_RotAndPos_ConVal.BodyRot.droll + Ref_RotAndPos_ConVal.BodyRot.ddroll * CONTROL_T;
	if (Ref_RotAndPos_ConVal.BodyRot.droll >  5.0) Ref_RotAndPos_ConVal.BodyRot.droll =  5.0;
	if (Ref_RotAndPos_ConVal.BodyRot.droll < -5.0) Ref_RotAndPos_ConVal.BodyRot.droll = -5.0;
	Ref_RotAndPos_ConVal.BodyRot.roll = Ref_RotAndPos_ConVal.BodyRot.roll + Ref_RotAndPos_ConVal.BodyRot.droll * CONTROL_T;
	if (Ref_RotAndPos_ConVal.BodyRot.roll >  limit[1]) { Ref_RotAndPos_ConVal.BodyRot.roll =  limit[1];
		Ref_RotAndPos_ConVal.BodyRot.droll = (Ref_RotAndPos_ConVal.BodyRot.roll - LandRot_roll_old) / CONTROL_T; }
	if (Ref_RotAndPos_ConVal.BodyRot.roll < -limit[1]) { Ref_RotAndPos_ConVal.BodyRot.roll = -limit[1];
		Ref_RotAndPos_ConVal.BodyRot.droll = (Ref_RotAndPos_ConVal.BodyRot.roll - LandRot_roll_old) / CONTROL_T; }

	// hori
	Ref_RotAndPos_ConVal.BodyPos.y = K_Pitch2Y * Ref_RotAndPos_ConVal.BodyRot.pitch;
	Ref_RotAndPos_ConVal.BodyPos.x = K_Roll2X  * Ref_RotAndPos_ConVal.BodyRot.roll;

	return Ref_RotAndPos_ConVal;

}

// contact
Run_ConVal ContactCon(Run_ConVal contactconval, Run_ConVal vd, Run_FS Rfoot_ref, Run_FS Lfoot_ref, Run_FS Rfoot_rel, Run_FS Lfoot_rel, double paras[5], double limit[1], char mode, double paras_stepdown[4], int k_pre)
{
	double Kpz = paras[0];
	double Kdz = paras[1];
	// double Kfz = paras[2];
	double Kmz = paras[2];
	double Kiz = paras[3];
	double Kez = paras[4];
	Run_ConVal ed;

	double delta_R_Fz = Rfoot_rel.z - Rfoot_ref.z;
	double delta_L_Fz = Lfoot_rel.z - Lfoot_ref.z;
	sum_delta_R_Fz = sum_delta_R_Fz + delta_R_Fz;
	sum_delta_L_Fz = sum_delta_L_Fz + delta_L_Fz;

	double Rz_old = contactconval.RfootPos.z;
	double Lz_old = contactconval.LfootPos.z;

	// mode: Landing, Standing, Jumping, Air
	double h_ankle = paras_stepdown[0];
	double step_dowm = paras_stepdown[1];
	double Kez_Landing = paras_stepdown[2];
	int k_max = (int)paras_stepdown[3];
	double RKez = 0.0;
	double LKez = 0.0;
	
	if(k_pre <= 800){
		Kdz = 1.5 * Kdz;
	}
	
	if (mode == 'L') { // Landing
		int k = 0;
		while (k < k_max) {
			k++;
			if (Tra_RAnkle.z[k_pre] > h_ankle && Tra_RAnkle.z[k_pre + k] < h_ankle + 1e-6) { // R Landing
				RKez = Kez_Landing;
				ed.RfootPos.z = step_dowm;
				// printf("R_landing\n");
				break;
			}
			if (Tra_LAnkle.z[k_pre] > h_ankle && Tra_LAnkle.z[k_pre + k] < h_ankle + 1e-6) { // L Landing
				LKez = Kez_Landing;
				ed.LfootPos.z = step_dowm;
				// printf("L_landing\n");
				break;
			}
		}
		// printf("%f, %f\n", RKez, LKez);
	}
	else if (mode == 'S') { // Standing
		// void
	}
	else if (mode == 'J') { // Jumping
		// release energy
	}
	else if (mode == 'A') { // Air
		// retreat
	}
	else {
	}

	ed.RfootPos.z = (delta_R_Fz + sqrt(delta_R_Fz * delta_R_Fz + delta_R_Fz / 9.8 * Kpz * (-vd.RfootPos.z - contactconval.RfootPos.dz) * (-vd.RfootPos.z - contactconval.RfootPos.dz))) / Kpz;
	// contactconval.RfootPos.ddz = Kfz * delta_R_Fz - Kpz * contactconval.RfootPos.z - Kdz * contactconval.RfootPos.dz + Kez * (ed.RfootPos.z - contactconval.RfootPos.z) + 0.0 * Kiz * sum_delta_R_Fz + RKez * (step_dowm - contactconval.RfootPos.z);
	// contactconval.RfootPos.ddz = 1 / Kmz * delta_R_Fz - Kpz / Kmz * contactconval.RfootPos.z - Kdz / Kmz * contactconval.RfootPos.dz + Kez * (ed.RfootPos.z - contactconval.RfootPos.z) + 0.0 * Kiz * sum_delta_R_Fz + RKez * (step_dowm - contactconval.RfootPos.z);
	contactconval.RfootPos.ddz = 1 / Kmz * delta_R_Fz - Kpz / Kmz * contactconval.RfootPos.z - Kdz / Kmz * contactconval.RfootPos.dz;
	// contactconval.RfootPos.ddz = Kmz * delta_R_Fz - Kpz * contactconval.RfootPos.z - Kdz * contactconval.RfootPos.dz;
	if (contactconval.RfootPos.ddz >  100.0) contactconval.RfootPos.ddz =  100.0;
	if (contactconval.RfootPos.ddz < -100.0) contactconval.RfootPos.ddz = -100.0;
	contactconval.RfootPos.dz = contactconval.RfootPos.dz + contactconval.RfootPos.ddz * CONTROL_T;
	if (contactconval.RfootPos.dz >  10.0) contactconval.RfootPos.dz =  10.0;
	if (contactconval.RfootPos.dz < -10.0) contactconval.RfootPos.dz = -10.0;
	contactconval.RfootPos.z = contactconval.RfootPos.z + contactconval.RfootPos.dz * CONTROL_T;
	if (contactconval.RfootPos.z > limit[0]) { contactconval.RfootPos.z = limit[0];
		contactconval.RfootPos.dz = (contactconval.RfootPos.z - Rz_old) / CONTROL_T; }
	if (contactconval.RfootPos.z < -0.2 * limit[0]) { contactconval.RfootPos.z = -0.2 * limit[0];
		contactconval.RfootPos.dz = (contactconval.RfootPos.z - Rz_old) / CONTROL_T; }

	ed.LfootPos.z = (delta_L_Fz + sqrt(delta_L_Fz * delta_L_Fz + delta_L_Fz / 9.8 * Kpz * (-vd.LfootPos.z - contactconval.LfootPos.dz) * (-vd.LfootPos.z - contactconval.LfootPos.dz))) / Kpz;
	// contactconval.LfootPos.ddz = Kfz * delta_L_Fz - Kpz * contactconval.LfootPos.z - Kdz * contactconval.LfootPos.dz + Kez * (ed.LfootPos.z - contactconval.LfootPos.z) + 0.0 * Kiz * sum_delta_L_Fz + LKez * (step_dowm - contactconval.LfootPos.z);
	// contactconval.LfootPos.ddz = 1 / Kmz * delta_L_Fz - Kpz / Kmz * contactconval.LfootPos.z - Kdz / Kmz * contactconval.LfootPos.dz + Kez * (ed.LfootPos.z - contactconval.LfootPos.z) + 0.0 * Kiz * sum_delta_L_Fz + LKez * (step_dowm - contactconval.LfootPos.z);
	contactconval.LfootPos.ddz = 1 / Kmz * delta_L_Fz - Kpz / Kmz * contactconval.LfootPos.z - Kdz / Kmz * contactconval.LfootPos.dz;
	// contactconval.LfootPos.ddz = Kmz * delta_L_Fz - Kpz * contactconval.LfootPos.z - Kdz * contactconval.LfootPos.dz;
	if (contactconval.LfootPos.ddz >  100.0) contactconval.LfootPos.ddz =  100.0;
	if (contactconval.LfootPos.ddz < -100.0) contactconval.LfootPos.ddz = -100.0;
	contactconval.LfootPos.dz = contactconval.LfootPos.dz + contactconval.LfootPos.ddz * CONTROL_T;
	if (contactconval.LfootPos.dz >  10.0) contactconval.LfootPos.dz =  10.0;
	if (contactconval.LfootPos.dz < -10.0) contactconval.LfootPos.dz = -10.0;
	contactconval.LfootPos.z = contactconval.LfootPos.z + contactconval.LfootPos.dz * CONTROL_T;
	if (contactconval.LfootPos.z > limit[0]) { contactconval.LfootPos.z = limit[0];
		contactconval.LfootPos.dz = (contactconval.LfootPos.z - Lz_old) / CONTROL_T; }
	if (contactconval.LfootPos.z < -0.2 * limit[0]) { contactconval.LfootPos.z = -0.2 * limit[0];
		contactconval.LfootPos.dz = (contactconval.LfootPos.z - Lz_old) / CONTROL_T; }

	return contactconval;
}

void DCC_RunningControl(double pitch_ref, double roll_ref, double  pitch_sen, double roll_sen, double pitch_con_old, double roll_con_old, int k_pre)
{
	Run_Horizontal zmp_ref;
	Run_Horizontal zmp_rel;
	Run_Rotational body_rot_ref;
	Run_Rotational body_rot_rel;
	Run_Rotational Rdel_the_temp, Ldel_the_temp, Rdel_dtheta_rel, Ldel_dtheta_rel; // LandRot
	Run_FS Rfoot_ref, Lfoot_ref, Rfoot_rel, Lfoot_rel;
	//DCCRunParms *p = &dccRunParms;
	// contact
	Run_ConVal Vd_ankle = { 0.0 }; // Bug?
	double F_sum;
	double paras_GRFC_old_temp[6] = { 0.0 };
	//for(int i = 0;i<6;i++) {paras_GRFC_old_temp[i] = p->paras_GRFC_old[i];}
	double ZMP_Lag_T = 0.005;
	double IMU_Lag_T = 0.04;
	double Fz_Lag_T = 0.00;
	
	double pitch_bias = + 0.65 / 57.3;
	double zmpbias_micro = 0.85;
	double zmpx_bias = -0.0 * 0.02;  // for walk // -0.015; // for run
	double zmpy_bias = -0.025; // * (k_pre < 3500? 1.0: (exp((-1.0 * k_pre + 3500.0) / 1000.0))); // for walk // -0.02;  // for run -0.02
	// double zmpy_bias = -0.01;
	double m_robot_bias = 4.5;
	double roll_ampli = 2.5;
	double tau_pitch_bias = -0.0 * 2.8;
	double tau_roll_bias = 0.0;
	
	double Zc = 0.7;
	double paras_LIPM[6] = {
		0.6 * 20.0, 10.4, 15.5,		// x -> kp, kv, kz
		1.5 * 12.0, 10.4, 22.5		// y -> kp, kv, kz
	};

	double paras_TPC[6] = {
		22.0, 85.4, 20.0,		// x -> kzmp, kp, kd 
		18.0, 72.4, 28.0		// y -> kzmp, kp, kd 
	};
	double limit_TPC[2] = { 0.04, 0.03 }; // x, y

	double paras_GRFC_old[6] = {
		0.75 * 250.0, 0.85 * 70.0,			// pitch -> kp, kd 
		0.75 * 350.0, 0.85 * 75.0,			// roll  -> kp, kd 
		1e4, 8e3 //1e4,   1e4 	//5e4,   8e3				// zctrl -> kp, kd 
	};
	double limit_GRFC_old[3] = { 6.0 / 57.3, 5.0 / 57.3, 0.015 }; // pitch, roll, zctrl

	double paras_GRFC[6] = {
		0.1, 10.0, 8.0,			// pitch -> kf, kp, kd 
		0.1, 10.0, 8.0			// roll  -> kf, kp, kd 
	};
	double limit_GRFC[3] = { 30.0 / 57.3, 20.0 / 57.3 }; // pitch, roll

	double paras_Rot[6] = {
		 1.2 * 100.0, 50.0, 15.0,		// pitch -> kzmp, kp, kd 
		 1.2 * 100.0, 50.0, 15.0		// roll  -> kzmp, kp, kd 
	};
	double limit_Rot[2] = { 20.0 / 57.3, 15.0 / 57.3 };// pitch, roll

	double Balance_Pro[2] = { 10.0, 10.0 }; // pitch, roll
	double Micro_Pro[2] = {0.2,0.2};
	double limit_Pro[2]   = { 50.0, 50.0 }; // pitch, roll

	double paras_FlyBody[6] = { 
		145.9, 58.3, 16.7,		// pitch -> km, kp, kd
		// 120.9, 58.3, 16.7,		// pitch -> km, kp, kd
		145.9, 58.3, 16.7		// roll  -> km, kp, kd
		// 120.9, 58.3, 16.7		// roll  -> km, kp, kd
	};
	double limit_FlyBody[2] = { 10.0 / 57.3, 10.0 / 57.3 }; // pitch, roll
	double wake_FlyBody[2]  = { 0.0 / 57.3,   0.0 / 57.3 }; // pitch, roll
	
	double paras_FlyFoot[4] = { 
		0.1 * 48.3, 3.0 * 19.7,		// pitch -> kp, kd
		0.1 * 48.3, 3.0 * 19.7		// roll  -> kp, kd
	};
	double limit_FlyFoot[2] = { 5.0 / 57.3, 5.0 / 57.3 }; // pitch, roll
	double wake_FlyFoot[2]  = { 0.0 / 57.3, 0.0 / 57.3 }; // pitch, roll
	// stepz
	double paras_Step[12] = {
		60.0, 2.5 * 25.0, 1.25 * 9.0,		// z pitch -> kth, kp, kd
		60.0, 2.5 * 25.0, 1.25 * 9.0,		// z roll  -> kth, kp, kd
		20.0, 25.0, 9.0,					// r pitch -> rth, kp, kd
		20.0, 25.0, 9.0						// r roll  -> rth, kp, kd
	};
	double limit_Step[3] = { 0.06, 10.0 * 57.3, 10 * 57.3 }; // z, pitch, roll
	// LandRot
	double paras_Land[11] = {
		1.2, 180.0, 275.0,		// pitch -> Kzmp, Kth, kom
		1.0, 150.0, 275.0,		// roll  -> Kzmp, Kth, kom
		0.3, 175.0, 28.0,		// I_robot, kp, kd
		0.04, 0.08				// kP2TY, kR2X 
	};
	double limit_Land[2] = { 15.0 / 57.3, 10.0 / 57.3 }; // pitch, roll
	double wake_Land[3] = { (1.0 + 5.3) / 57.3, -(1.0 + 7.0) / 57.3, (1.0 + 3.5) / 57.3}; // pitch+ pitch-, roll

	// contact
	double paras_Cont[5] = {
		100.0, 18000.0, 260, 0.00004, 0.36 // Kp, Kd, Km, Ki, Ke
	};
	double limit_Cont[1] = { 0.1 };
	double paras_Stepdown[4] = {
		0.112, 0.004, 250.0, 50     // H_ankle, StepDown, KeStepDown, k_down
	};
	int k_down = (int)paras_Stepdown[3];
	int k_down_fore = 55; // no bigger than k_down
	double H_ankle = paras_Stepdown[0];

	double addi_pitch = 0.0 * 5.0 * 57.3;
	double addi_Rfoot_pitch;
	double addi_Lfoot_pitch;
	double roll_bias_foot = 0.0;
	double pitch_bias_foot = 28.0;
	
	 //zmp bias
	 //zmpx_bias = -1.0 * chz_XZMP_filted;
	
	// zmp
	zmp_ref.x = P_ZMPRef_B.px;
	zmp_ref.y = P_ZMPRef_B.py;
	ZMP_filtered.x = Filter_TimeLag(ZMP_filtered.x, P_ZMPRel_B.px + zmpbias_micro * zmpx_bias, CONTROL_T, ZMP_Lag_T); // 0807
	ZMP_filtered.y = Filter_TimeLag(ZMP_filtered.y, P_ZMPRel_B.py + zmpbias_micro * zmpy_bias, CONTROL_T, ZMP_Lag_T);
	IMU_filtered.pitch = Filter_TimeLag(IMU_filtered.pitch, pitch_sen, CONTROL_T, IMU_Lag_T);
	IMU_filtered.roll  = Filter_TimeLag(IMU_filtered.roll , roll_sen , CONTROL_T, IMU_Lag_T);
	// FzR_filtered = Filter_TimeLag(FzR_filtered, FootFT[1][2], CONTROL_T, Fz_Lag_T);
	// FzL_filtered = Filter_TimeLag(FzL_filtered, FootFT[2][2], CONTROL_T, Fz_Lag_T);
	FzR_filtered = Filter_TimeLag(FzR_filtered, F_RFoot.fz, CONTROL_T, Fz_Lag_T);
	FzL_filtered = Filter_TimeLag(FzL_filtered, F_LFoot.fz, CONTROL_T, Fz_Lag_T);
	TxR_filtered = Filter_TimeLag(TxR_filtered, F_RFoot.tx, CONTROL_T, Fz_Lag_T);
	TxL_filtered = Filter_TimeLag(TxL_filtered, F_LFoot.tx, CONTROL_T, Fz_Lag_T);
	TyR_filtered = Filter_TimeLag(TyR_filtered, F_RFoot.ty, CONTROL_T, Fz_Lag_T);
	TyL_filtered = Filter_TimeLag(TyL_filtered, F_LFoot.ty, CONTROL_T, Fz_Lag_T);
	zmp_rel.x = ZMP_filtered.x;
	zmp_rel.y = ZMP_filtered.y;
	pitch_sen = IMU_filtered.pitch;
	roll_sen  = IMU_filtered.roll;
	F_RFoot.fz = FzR_filtered;
	F_LFoot.fz = FzL_filtered;
	F_RFoot.tx = TxR_filtered;
	F_RFoot.ty = TyR_filtered;
	F_LFoot.tx = TxL_filtered;
	F_LFoot.ty = TxL_filtered;
	
	FzR_rel = F_RFoot.fz;
	FzL_rel = F_LFoot.fz;
	// if ( 1 || (FzR_rel > 0.1 * m_robot * 9.8 && FzL_rel > 0.1 * m_robot * 9.8)) // dou
	// {
		// paras_GRFC[4] = 5e4;
		// paras_GRFC[5] = 8e3;
	// }
	

	// rot
	body_rot_ref.pitch = pitch_ref + pitch_bias;
	body_rot_ref.roll  = roll_ref;
	if (k_pre == 0) {
		pitch_sen_bias = pitch_sen;
		roll_sen_bias  = roll_sen;
	}
	else if (k_pre < 20) {
		pitch_sen_bias = 0.5 * (pitch_sen_bias + pitch_sen);
		roll_sen_bias  = 0.5 * (roll_sen_bias  + roll_sen);
	}
	
	body_rot_rel.pitch = pitch_sen - pitch_sen_bias;
	body_rot_rel.roll  = roll_sen  - roll_sen_bias ;
	// printf("%f, %f\n", pitch_sen, roll_sen);

	// del
	// LandRot
	Rdel_the_temp.pitch 	= body_rot_rel.pitch - (pitch_ref + DCC_Run.BodyRot.pitch) - DCC_Run.RfootRot.pitch - pitch_footr;	// R foot pitch // check
	Rdel_dtheta_rel.pitch 	= (Rdel_the_temp.pitch - Rdel_theta_rel.pitch) / CONTROL_T;									// R foot dpitch
	Rdel_theta_rel.pitch 	= Rdel_the_temp.pitch;																			
	Rdel_the_temp.roll 		= body_rot_rel.roll - (roll_ref + DCC_Run.BodyRot.roll) + DCC_Run.RfootRot.roll;							// R foot roll
	Rdel_dtheta_rel.roll 	= (Rdel_the_temp.roll - Rdel_theta_rel.roll) / CONTROL_T;										// R foot droll
	Rdel_theta_rel.roll 	= Rdel_the_temp.roll;																			
	Ldel_the_temp.pitch 	= body_rot_rel.pitch - (pitch_ref + DCC_Run.BodyRot.pitch) - DCC_Run.LfootRot.pitch - pitch_footl;	// L foot pitch
	Ldel_dtheta_rel.pitch 	= (Ldel_the_temp.pitch - Ldel_theta_rel.pitch) / CONTROL_T;									// L foot dpitch
	Ldel_theta_rel.pitch 	= Ldel_the_temp.pitch;																			
	Ldel_the_temp.roll 		= body_rot_rel.roll - (roll_ref + DCC_Run.BodyRot.roll) + DCC_Run.LfootRot.roll;							// L foot roll
	Ldel_dtheta_rel.roll 	= (Ldel_the_temp.roll - Ldel_theta_rel.roll) / CONTROL_T;										// L foot droll
	Ldel_theta_rel.roll 	= Ldel_the_temp.roll;																			
	del_theta_rel.pitch 	= body_rot_rel.pitch - (pitch_ref + DCC_Run.BodyRot.pitch);
	del_theta_rel.roll 		= body_rot_rel.roll - (roll_ref + DCC_Run.BodyRot.roll);
	
	// comp
	Rfoot_rel.pitch = TxR_filtered + tau_pitch_bias;		Lfoot_rel.pitch = TxL_filtered + tau_pitch_bias;
	Rfoot_rel.roll	= TyR_filtered - tau_roll_bias;		    Lfoot_rel.roll	= TyL_filtered + tau_roll_bias;
	Rfoot_rel.z		= FzR_filtered;		Lfoot_rel.z		= FzL_filtered;
	
	F_sum = Rfoot_rel.z + Lfoot_rel.z;

	#ifdef USE_Run_LIPM
		LIPM_ConVal = LIPM_Con(LIPM_ConVal, TPC_Run_ConVal, body_rot_ref, body_rot_rel, p->Zc, p->paras_LIPM);									// <- LIPM
	#endif
		ADD_Trq_Ref = DCC_BalancePro(LIPM_ConVal, body_rot_ref, body_rot_rel, Balance_Pro, limit_Pro, F_sum);								// <- Balance Pro

	#ifdef USE_CHZ_RUN
		// ankle
		if (F_sum > 0.2 * m_robot * 9.8) { // if touch down

			if (chzrun_signal[k_pre][2] == 1) // r sup
			{
				// paras_GRFC_old_temp[4] = 0.5 * p->paras_GRFC_old[4];
				// paras_GRFC_old_temp[5] = 0.5 * p->paras_GRFC_old[5];
				if (F_RFoot.fz > 0.2 * m_robot * 9.8) { // if r touch down
					Rfoot_ref.pitch = -0.0 * chzrun_MF[k_pre][0] + 1.0 * ADD_Trq_Ref.pitch + additor_flag * additor_pit;
					Lfoot_ref.pitch = 0.0;
					Rfoot_ref.roll = 1.0 * p->roll_ampli * chzrun_MF[k_pre][2] + 1.0 * ADD_Trq_Ref.roll + additor_flag * additor_rol;
					Lfoot_ref.roll = 0.0;
					Rfoot_ref.z = m_robot * 9.8 + m_robot * Tra_ACOM.z[k_pre];
					Lfoot_ref.z = 0.0;
				}
				else {
					Rfoot_ref.pitch = 0.0;
					Lfoot_ref.pitch = 0.0;
					Rfoot_ref.roll = 0.0;
					Lfoot_ref.roll = 0.0;
					Rfoot_ref.z = 0.0;
					Lfoot_ref.z = 0.0;
				}
			}
			else if (chzrun_signal[k_pre][2] == 2) // l sup
			{
				// paras_GRFC_old_temp[4] = 0.5 * p->paras_GRFC_old[4];
				// paras_GRFC_old_temp[5] = 0.5 * p->paras_GRFC_old[5];
				if (F_LFoot.fz > 0.2 * m_robot * 9.8) { // if l touch down
					Rfoot_ref.pitch = 0.0;
					Lfoot_ref.pitch = -0.0 * chzrun_MF[k_pre][0] + 1.0 * ADD_Trq_Ref.pitch + additor_flag * additor_pit;
					Rfoot_ref.roll = 0.0;
					Lfoot_ref.roll = 1.0 * p->roll_ampli * chzrun_MF[k_pre][2] + 1.0 * ADD_Trq_Ref.roll + additor_flag * additor_rol;
					Rfoot_ref.z = 0.0;
					Lfoot_ref.z = m_robot * 9.8 + m_robot * Tra_ACOM.z[k_pre];
				}
				else {
					Rfoot_ref.pitch = 0.0;
					Lfoot_ref.pitch = 0.0;
					Rfoot_ref.roll = 0.0;
					Lfoot_ref.roll = 0.0;
					Rfoot_ref.z = 0.0;
					Lfoot_ref.z = 0.0;
				}
			}
			else if (chzrun_signal[k_pre][2] == 3) // fly
			{
				// paras_GRFC_old_temp[4] = 0.25 * p->paras_GRFC_old[4];
				// paras_GRFC_old_temp[5] = 0.25 * p->paras_GRFC_old[5];
				Rfoot_ref.pitch = 0.0;
				Lfoot_ref.pitch = 0.0;
				Rfoot_ref.roll = 0.0;
				Lfoot_ref.roll = 0.0;
				Rfoot_ref.z = 0.0;
				Lfoot_ref.z = 0.0;
			}
			else // d sup
			{
				// paras_GRFC_old_temp[4] = p->paras_GRFC_old[4];
				// paras_GRFC_old_temp[5] = p->paras_GRFC_old[5];
				Rfoot_ref.pitch = (Tra_ZMP.x[k_pre] + 0.08) / 0.16 * (-0.0 * chzrun_MF[k_pre][0] + 1.0 * ADD_Trq_Ref.pitch + additor_flag * additor_pit);
				Lfoot_ref.pitch = (-Tra_ZMP.x[k_pre] + 0.08) / 0.16 * (-0.0 * chzrun_MF[k_pre][0] + 1.0 * ADD_Trq_Ref.pitch + additor_flag * additor_pit);
				Rfoot_ref.roll = (Tra_ZMP.x[k_pre] + 0.08) / 0.16 * (1.0 * chzrun_MF[k_pre][2] + 1.0 * ADD_Trq_Ref.roll + additor_flag * additor_rol);
				Lfoot_ref.roll = (-Tra_ZMP.x[k_pre] + 0.08) / 0.16 * (1.0 * chzrun_MF[k_pre][2] + 1.0 * ADD_Trq_Ref.roll + additor_flag * additor_rol);
				Rfoot_ref.z = (Tra_ZMP.x[k_pre] + 0.08) / 0.16 * m_robot * 9.8;
				Lfoot_ref.z = (-Tra_ZMP.x[k_pre] + 0.08) / 0.16 * m_robot * 9.8;
			}
		}
		else // if fly
		{
			// paras_GRFC_old_temp[4] = 0.25 * p->paras_GRFC_old[4];
			// paras_GRFC_old_temp[5] = 0.25 * p->paras_GRFC_old[5];
			Rfoot_ref.pitch = 0.0;
			Lfoot_ref.pitch = 0.0;
			Rfoot_ref.roll = 0.0;
			Lfoot_ref.roll = 0.0;
			Rfoot_ref.z = 0.0;
			Lfoot_ref.z = 0.0;
		}
		
		// z
		// if (chzrun_signal[k_pre][2] == 1) // r sup
		// {
			// Rfoot_ref.z = (p->m_robot_bias + m_robot) * 9.8 + (p->m_robot_bias + m_robot) * Tra_ACOM.z[k_pre];
			// Lfoot_ref.z = 0.0;
		// }
		// else if (chzrun_signal[k_pre][2] == 2) // l sup
		// {
			// Rfoot_ref.z = 0.0;
			// Lfoot_ref.z = (p->m_robot_bias + m_robot) * 9.8 + (p->m_robot_bias + m_robot) * Tra_ACOM.z[k_pre];
		// }
		// else if (chzrun_signal[k_pre][2] == 3) // fly
		// {
			// Rfoot_ref.z = 0.0;
			// Lfoot_ref.z = 0.0;
		// }
		// else // d sup
		// {
			// Rfoot_ref.z = (Tra_ZMP.x[k_pre] + 0.08) / 0.16 * (p->m_robot_bias + m_robot) * (Tra_ACOM.z[k_pre]+9.8);
			// Lfoot_ref.z = (-Tra_ZMP.x[k_pre] + 0.08) / 0.16 * (p->m_robot_bias + m_robot) * (Tra_ACOM.z[k_pre]+9.8);
		// }
		
	#else // precon
		if (Tra_LAnkle.z[k_pre] > H_ANKLE && Tra_RAnkle.z[k_pre] < H_ANKLE + 1e-6) { // R sup
			if (F_RFoot.fz > 0.1 * m_robot * 9.8) { // R touch
				Rfoot_ref.pitch = ADD_Trq_Ref.pitch + pitch_bias_foot + additor_flag * additor_pit;
				Rfoot_ref.roll  = ADD_Trq_Ref.roll + additor_flag * additor_rol;
				Lfoot_ref.pitch = 0.0;
				Lfoot_ref.roll  = 0.0;
				Rfoot_ref.z = (Tra_ZMP.x[k_pre] + 0.08) / 0.16 * m_robot * 9.8 ;
				Lfoot_ref.z = (-Tra_ZMP.x[k_pre] + 0.08) / 0.16 * m_robot * 9.8;
			}
			else {
				Rfoot_ref.pitch = 0.0;
				Rfoot_ref.roll  = 0.0;
				Lfoot_ref.pitch = 0.0;
				Lfoot_ref.roll  = 0.0;
				Rfoot_ref.z = (Tra_ZMP.x[k_pre] + 0.08) / 0.16 * m_robot * 9.8 ;
				Lfoot_ref.z = (-Tra_ZMP.x[k_pre] + 0.08) / 0.16 * m_robot * 9.8;
			}
		}
		else if (Tra_LAnkle.z[k_pre] < H_ANKLE + 1e-6 && Tra_RAnkle.z[k_pre] > H_ANKLE) { // L sup
			if (F_LFoot.fz > 0.1 * m_robot * 9.8) { // L touch
				Lfoot_ref.pitch = ADD_Trq_Ref.pitch + pitch_bias_foot + additor_flag * additor_pit;
				Lfoot_ref.roll  = ADD_Trq_Ref.roll + additor_flag * additor_rol;
				Rfoot_ref.pitch = 0.0;
				Rfoot_ref.roll  = 0.0;
				Rfoot_ref.z = (Tra_ZMP.x[k_pre] + 0.08) / 0.16 * m_robot * 9.8 ;
				Lfoot_ref.z = (-Tra_ZMP.x[k_pre] + 0.08) / 0.16 * m_robot * 9.8;
			}
			else {
				Rfoot_ref.pitch = 0.0;
				Rfoot_ref.roll  = 0.0;
				Lfoot_ref.pitch = 0.0;
				Lfoot_ref.roll  = 0.0;
				Rfoot_ref.z = (Tra_ZMP.x[k_pre] + 0.08) / 0.16 * m_robot * 9.8 ;
				Lfoot_ref.z = (-Tra_ZMP.x[k_pre] + 0.08) / 0.16 * m_robot * 9.8;
			}
		}
		else { // D sup
			Rfoot_ref.pitch = (Tra_ZMP.x[k_pre] + 0.08) / 0.16 * (ADD_Trq_Ref.pitch + additor_flag * additor_pit) + pitch_bias_foot;
			Rfoot_ref.roll  = (Tra_ZMP.x[k_pre] + 0.08) / 0.16 * (ADD_Trq_Ref.roll + additor_flag * additor_rol);
			Lfoot_ref.pitch = (-Tra_ZMP.x[k_pre] + 0.08) / 0.16 * (ADD_Trq_Ref.pitch + additor_flag * additor_pit) + pitch_bias_foot;
			Lfoot_ref.roll  = (-Tra_ZMP.x[k_pre] + 0.08) / 0.16 * (ADD_Trq_Ref.roll + additor_flag * additor_rol);
			Rfoot_ref.z = (Tra_ZMP.x[k_pre] + 0.08) / 0.16 * m_robot * 9.8;// + ADD_Trq_Ref.z;
			Lfoot_ref.z = (-Tra_ZMP.x[k_pre] + 0.08) / 0.16 * m_robot * 9.8;// - ADD_Trq_Ref.z;
			if (F_RFoot.fz < 0.35 * m_robot * 9.8) { // R up
				Rfoot_ref.pitch = 0.0;
				Rfoot_ref.roll  = 0.0;
			}
			if (F_LFoot.fz < 0.35 * m_robot * 9.8) { // R up
				Lfoot_ref.pitch = 0.0;
				Lfoot_ref.roll  = 0.0;
			}
		}
		
	#endif
	FzR_ref = Rfoot_ref.z;
	FzL_ref = Lfoot_ref.z;

	TPC_Run_ConVal = TPC_Run(TPC_Run_ConVal, LIPM_ConVal, zmp_ref, zmp_rel, paras_TPC, limit_TPC, F_sum);			// <- TPC Con
	// TPCFoot
#ifdef USE_Foot_TPC
	double F_check[3] = { 0.0 };
	F_check[0] = F_sum;
	F_check[1] = Rfoot_rel.z;
	F_check[2] = Lfoot_rel.z;
	TPCFoot_ConVal = TPC_Foot(TPCFoot_ConVal, LIPM_ConVal, zmp_ref, zmp_rel, F_check, p->paras_TPC, p->limit_TPC, k_pre);   // <- TPC FOOT
#endif
#ifdef USE_Run_Compliance_old
	FootCompliance_ConVal = Compliance_Run_old(FootCompliance_ConVal, Rfoot_ref, Lfoot_ref, Rfoot_rel, Lfoot_rel, paras_GRFC_old_temp, p->limit_GRFC_old);					// <- Compliance old
#endif
#ifdef USE_Run_Compliance
	FootCompliance_ConVal = Compliance_Run(FootCompliance_ConVal, Rfoot_ref, Lfoot_ref, Rfoot_rel, Lfoot_rel, p->paras_GRFC, p->limit_GRFC);						// <- Compliance 
#endif
																																									// LandRot
	Ref_RotAndPos_ConVal = LandingBalance_Rot( Ref_RotAndPos_ConVal, Rdel_theta_rel, Ldel_theta_rel, Rdel_dtheta_rel, Ldel_dtheta_rel, zmp_ref, zmp_rel, Rfoot_rel, Lfoot_rel, paras_Land, limit_Land, wake_Land, F_sum, k_pre); // <- Land Balance Rot con
	#ifdef USE_Land_RotCon
		body_rot_ref.pitch += Ref_RotAndPos_ConVal.BodyRot.pitch; // check
		body_rot_ref.roll  += Ref_RotAndPos_ConVal.BodyRot.roll;  // check
	#endif
	BodyRot_ConVal = PostureCon_Run(BodyRot_ConVal, body_rot_ref, body_rot_rel, paras_Rot, limit_Rot, F_sum);												// <- Posture Con
	// stepz
	FlyRot_ConVal = Fly_Rot_Con(FlyRot_ConVal, Rdel_theta_rel, Ldel_theta_rel, del_theta_rel, paras_FlyBody, limit_FlyBody, wake_FlyBody, paras_FlyFoot, limit_FlyFoot, paras_Step, limit_Step, F_sum, Rfoot_rel.z, Lfoot_rel.z, k_pre); // <- Fly Con
#ifdef USE_Contact_Con
	// Rfoot_ref.z = m_robot * 9.8 * 0.5 + 50 * cos(2 * 3.14 * 1 * k_pre * CONTROL_T); 
	// Lfoot_ref.z = m_robot * 9.8 * 0.5 + 50 * cos(2 * 3.14 * 1 * k_pre * CONTROL_T);// for test sake

	#ifdef USE_CHZ_RUN // for run
	if (Tra_RAnkle.z[k_pre] > p->H_ankle && Tra_LAnkle.z[k_pre] > p->H_ankle && mode_StepDown != 'L') {  // Air
		mode_StepDown = 'A';
		if (Tra_RAnkle.z[k_pre + p->k_down] < (p->H_ankle + 1e-6) || Tra_LAnkle.z[k_pre + p->k_down] < (p->H_ankle + 1e-6)) { // Landing
			mode_StepDown = 'L';
		}
	}
	else if (Tra_RAnkle.z[k_pre + p->k_down_fore] < (p->H_ankle + 1e-6) || Tra_LAnkle.z[k_pre + p->k_down_fore] < (p->H_ankle + 1e-6)) { // Standing
		mode_StepDown = 'S';
	}
	#else // for walk
	if (Tra_RAnkle.z[k_pre] > p->H_ankle || Tra_LAnkle.z[k_pre] > p->H_ankle && mode_StepDown != 'L') {  // Standing
		mode_StepDown = 'S';
		if ((Tra_RAnkle.z[k_pre] > p->H_ankle && Tra_RAnkle.z[k_pre + p->k_down] < (p->H_ankle + 1e-6)) || (Tra_LAnkle.z[k_pre] > p->H_ankle && Tra_LAnkle.z[k_pre + p->k_down] < (p->H_ankle + 1e-6))) { // Landing
			mode_StepDown = 'L';
		}
	}
	else if (Tra_RAnkle.z[k_pre + p->k_down_fore] < (p->H_ankle + 1e-6) && Tra_LAnkle.z[k_pre + p->k_down_fore] < (p->H_ankle + 1e-6)) { // Standing
		mode_StepDown = 'S';
	}
	// printf("%c\n", mode_StepDown);
	#endif
	ContactConVal = ContactCon(ContactConVal, Vd_ankle, Rfoot_ref, Lfoot_ref, Rfoot_rel, Lfoot_rel, p->paras_Cont, p->limit_Cont, mode_StepDown, p->paras_Stepdown, k_pre);
#endif
		
	// init
	DCC_Run.BodyPos.x		= 0.0;
	DCC_Run.BodyPos.y		= 0.0;
	DCC_Run.BodyRot.pitch	= 0.0;
	DCC_Run.BodyRot.roll	= 0.0;
	DCC_Run.RfootPos.x		= 0.0;
	DCC_Run.LfootPos.x		= 0.0;
	DCC_Run.RfootPos.y		= 0.0;
	DCC_Run.LfootPos.y		= 0.0;
	DCC_Run.RfootPos.z		= 0.0;
	DCC_Run.LfootPos.z		= 0.0;
	DCC_Run.RfootRot.pitch	= 0.0;
	DCC_Run.LfootRot.pitch	= 0.0;
	DCC_Run.RfootRot.roll	= 0.0;
	DCC_Run.LfootRot.roll	= 0.0;
	// body pos conval
	#ifdef USE_Run_TPC
		DCC_Run.BodyPos.x = 1.0 * TPC_Run_ConVal.x;
		DCC_Run.BodyPos.y = 1.0 * TPC_Run_ConVal.y;
	#endif
	// TPCFoot
	#ifdef USE_Foot_TPC
		DCC_Run.RfootPos.x = TPCFoot_ConVal.RfootPos.x;
		DCC_Run.RfootPos.y = TPCFoot_ConVal.RfootPos.y;
		DCC_Run.LfootPos.x = TPCFoot_ConVal.LfootPos.x;
		DCC_Run.LfootPos.y = TPCFoot_ConVal.LfootPos.y;
	#endif
	// LandRot
	#ifdef USE_Land_RotCon 
		DCC_Run.BodyPos.x += 1.0 * Ref_RotAndPos_ConVal.BodyPos.x;
		DCC_Run.BodyPos.y += 1.0 * Ref_RotAndPos_ConVal.BodyPos.y;
	#endif
	// body rot conval
	#ifdef USE_Run_PostureCon
		DCC_Run.BodyRot.pitch += BodyRot_ConVal.pitch;
		DCC_Run.BodyRot.roll  += 1.0 * BodyRot_ConVal.roll ;
	#endif
	// body fly conval
	#ifdef USE_FLY_PostureCon
		DCC_Run.BodyRot.pitch += FlyRot_ConVal.BodyRot.pitch;
		DCC_Run.BodyRot.roll  += FlyRot_ConVal.BodyRot.roll ;
		DCC_Run.RfootRot.pitch += FlyRot_ConVal.RfootRot.pitch;
		DCC_Run.RfootRot.roll  += FlyRot_ConVal.RfootRot.roll;
		DCC_Run.LfootRot.pitch += FlyRot_ConVal.LfootRot.pitch;
		DCC_Run.LfootRot.roll  += FlyRot_ConVal.LfootRot.roll;
	#endif
	// stepz conval
	#ifdef USE_FLY_STEPZ
		DCC_Run.RfootPos.z += FlyRot_ConVal.RfootPos.z;
		DCC_Run.LfootPos.z += FlyRot_ConVal.LfootPos.z;
	#endif
	// foot rot conval
	#if defined(USE_Run_Compliance) || defined(USE_Run_Compliance_old)
		DCC_Run.RfootRot.pitch += 1.0 * FootCompliance_ConVal.RfootRot.pitch;
		DCC_Run.RfootRot.roll  += 1.0 * FootCompliance_ConVal.RfootRot.roll ;
		DCC_Run.LfootRot.pitch += 1.0 * FootCompliance_ConVal.LfootRot.pitch;
		DCC_Run.LfootRot.roll  += 1.0 * FootCompliance_ConVal.LfootRot.roll ;
	#endif
	// contact conval
	#ifdef USE_Contact_Con
		DCC_Run.RfootPos.z += 1.0 * ContactConVal.RfootPos.z;
		DCC_Run.LfootPos.z += 1.0 * ContactConVal.LfootPos.z;
	#endif
	#ifdef USE_Run_Compliance_old
		DCC_Run.RfootPos.z += 0.0 * FootCompliance_ConVal.RfootPos.z;
		DCC_Run.LfootPos.z += 0.0 * FootCompliance_ConVal.LfootPos.z;
	#endif
	// re
	Rfoot_ref_re = Rfoot_ref;
	Lfoot_ref_re = Lfoot_ref;
	Rfoot_rel_re = Rfoot_rel;
	Lfoot_rel_re = Lfoot_rel;

}