#include "Resistant_Compliance.h"
#include "Tra_generate.h"
//#include "cal_MPC_control.h"

extern PreCon_Tra Tra_ZMP;
extern Position P_ZMPRel_B;

extern double F_mpc_x_re;
extern double state_x1_re;
extern double state_x2_re;

double F_Ext_Late[6] = { 0.0 }; // check five times, so we need six mark point 
double F_Ext_Sagi[6] = { 0.0 };
double F_Vir_Late = 0.0;	double Tau_Vir_Late = 0.0;
double F_Vir_Sagi = 0.0;	double Tau_Vir_Sagi = 0.0;
double F_mpc_Sagi = 0.0;
double F_mpc_Late = 0.0;
int Count_Check_Release_x = 0;
int Count_Check_Release_y = 0;

RC_comp_onedirection RC_Comp_Late;
RC_comp_onedirection RC_Comp_Sagi;
RC_comp RC_Comp_Cont;
PreCon_Tra F_ext_re;		PreCon_Tra Tau_ext_re;
PreCon_Tra F_vir_re;		PreCon_Tra Tau_vir_re;
PreCon_Tra e_re;			PreCon_Tra r_re;
State_CoM state_com;

RC_comp_onedirection RC_Lateral(RC_comp_onedirection comp_late, double zmp_x_ref, double zmp_x_rel, double Z_c, double T_filter, double K_RC[4], double T_lag[2], double K_VMM[6], double micro[2], double T_start_walk, int k_pre)
{
	double F_ext;				double Tau_ext;
	double F_resi;				double Tau_resi;
	double K_1_e = K_RC[0];		double K_1_r = K_RC[2];
	double K_2_e = K_RC[1];		double K_2_r = K_RC[3];
	double K_p_e = K_VMM[0];	double K_p_r = K_VMM[3];
	double K_d_e = K_VMM[1];	double K_d_r = K_VMM[4];
	double K_m_e = K_VMM[2];	double K_m_r = K_VMM[5];
	double T_lag_e = T_lag[0];	double T_lag_r = T_lag[1];
	double RC_micro = micro[0]; double MPC_micro = 1.0; //micro[1];
	double Tau_micro = 1.0;
	double T_rls = 0.5; // cancel resist force in T_rls seconds after release the external force
	if (k_pre < T_start_walk / CONTROL_T)
	{
		RC_micro  = 0.01;
		MPC_micro = 0.05;
		Tau_micro = 0.01;
	}
	
	// F -------------------------------------------------------------------------------------------------
	F_ext = 1.0 * (0.0 * m_robot * comp_late.ddel + m_robot * GRAVITY / Z_c * (zmp_x_rel - zmp_x_ref - 0.0 * comp_late.el));
	if (F_ext >  80) F_ext = 80;
	if (F_ext < -80) F_ext = -80;
	if (Count_Check_Release_x++ == T_rls * 0.2 / CONTROL_T) // continues decline of F_ext for five times, then release is judged
	{
		Count_Check_Release_x = 0;
		for (int i = 5; i > 0; i--)
		{
			F_Ext_Late[i] = F_Ext_Late[i - 1];
		}
	}
	F_Ext_Late[0] = (F_ext + T_filter / CONTROL_T * F_Ext_Late[0]) / (1 + T_filter / CONTROL_T);
	F_ext = F_Ext_Late[0];

	F_resi = -K_1_e * F_ext - K_2_e * comp_late.el;
	F_Vir_Late = (F_resi + T_lag_e / CONTROL_T * F_Vir_Late) / (1 + T_lag_e / CONTROL_T);
	F_resi = F_Vir_Late;

	#ifdef STANDING_PUSH_CHECK
		if ((abs(F_Ext_Late[5]) > abs(F_Ext_Late[4])) && (abs(F_Ext_Late[4]) > abs(F_Ext_Late[3])) && (abs(F_Ext_Late[3]) > abs(F_Ext_Late[2])) && (abs(F_Ext_Late[2]) > abs(F_Ext_Late[1])) && (abs(F_Ext_Late[1]) > abs(F_Ext_Late[0])))
		{
			F_resi = -F_ext; // if release, stop resisting
			F_Vir_Late = F_resi;
		}
	#endif

	//comp_late.dde = (F_mpc_x - K_p_e * comp_late.el - K_d_e * comp_late.del) / K_m_e;
	//comp_late.dde = (RC_micro * (F_ext + F_resi) - K_p_e * comp_late.el - K_d_e * comp_late.del) / K_m_e;
	//comp_late.dde = (RC_micro * (F_ext + F_resi) + /*MPC_micro * F_mpc_x*/ - K_p_e * comp_late.el - K_d_e * comp_late.del) / K_m_e;
	// comp_late.dde = (RC_micro * (F_ext + F_resi) + 1.0 * MPC_micro * F_MPC.x - K_p_e * comp_late.el - K_d_e * comp_late.del) / K_m_e;
	// comp_late.de = comp_late.del + comp_late.dde * CONTROL_T;
	// if (comp_late.de >  0.1) comp_late.de = 0.1;
	// if (comp_late.de < -0.1) comp_late.de = -0.1;
	// comp_late.e = comp_late.el + comp_late.de * CONTROL_T;
	// if (comp_late.e >  0.07) comp_late.e = 0.07;
	// if (comp_late.e < -0.07) comp_late.e = -0.07;

	// comp_late.ddel = comp_late.dde;
	// comp_late.del = comp_late.de;
	// comp_late.el = comp_late.e;
	
	comp_late.e = (RC_micro * (F_ext + F_resi) / K_p_e  + (K_d_e / CONTROL_T / K_p_e + K_m_e / CONTROL_T / CONTROL_T / K_p_e) * comp_late.el + K_m_e / CONTROL_T / K_p_e * comp_late.del) / (1 + K_d_e / CONTROL_T / K_p_e + K_m_e / CONTROL_T / CONTROL_T / K_p_e);
	if (comp_late.e >  0.07) comp_late.e =  0.07;
	if (comp_late.e < -0.07) comp_late.e = -0.07;
	comp_late.de = (comp_late.e - comp_late.el) / CONTROL_T;
	comp_late.dde = (comp_late.de - comp_late.del) / CONTROL_T;
	comp_late.ddel = comp_late.dde;
	comp_late.del = comp_late.de;
	comp_late.el = comp_late.e;

	// re
	F_ext_re.x[k_pre] = F_ext;
	F_vir_re.x[k_pre] = F_resi;
	e_re.x[k_pre] = comp_late.e;
	// re

	// Tau -----------------------------------------------------------------------------------------------
	Tau_ext = F_ext * 0.3 * Z_c;

	Tau_resi = -K_1_r * Tau_ext - K_2_r * comp_late.rl;
	Tau_Vir_Late = (Tau_resi + T_lag_r / CONTROL_T * Tau_Vir_Late) / (1 + T_lag_r / CONTROL_T);
	Tau_resi = Tau_Vir_Late;

	// comp_late.ddr = (Tau_micro * (Tau_ext + Tau_resi) - K_p_r * comp_late.rl - K_d_r * comp_late.drl) / K_m_r;
	// comp_late.dr = comp_late.drl + comp_late.ddr * CONTROL_T;
	// if (comp_late.dr >  20 / 57.3) comp_late.dr = 20 / 57.3;
	// if (comp_late.dr < -20 / 57.3) comp_late.dr = -20 / 57.3;
	// comp_late.r = comp_late.rl + comp_late.dr * CONTROL_T;
	// if (comp_late.r >  8 / 57.3) comp_late.r = 8 / 57.3;
	// if (comp_late.r < -8 / 57.3) comp_late.r = -8 / 57.3;

	// comp_late.ddrl = comp_late.ddr;
	// comp_late.drl = comp_late.dr;
	// comp_late.rl = comp_late.r;

	comp_late.r = 1.0 * ((RC_micro * (Tau_ext + Tau_resi) / K_p_r + (K_d_r / CONTROL_T / K_p_r + K_m_r / CONTROL_T / CONTROL_T / K_p_r) * comp_late.rl + K_m_r / CONTROL_T / K_p_r * comp_late.drl) / (1 + K_d_r / CONTROL_T / K_p_r + K_m_r / CONTROL_T / CONTROL_T / K_p_r));
	if (comp_late.r >  12 / 57.3) comp_late.r =  12 / 57.3;
	if (comp_late.r < -12 / 57.3) comp_late.r = -12 / 57.3;
	comp_late.dr = (comp_late.r - comp_late.rl) / CONTROL_T;
	comp_late.ddr = (comp_late.dr - comp_late.drl) / CONTROL_T;
	comp_late.ddrl = comp_late.ddr;
	comp_late.drl = comp_late.dr;
	comp_late.rl = comp_late.r;
	
	// re
	Tau_ext_re.x[k_pre] = Tau_ext;
	Tau_vir_re.x[k_pre] = Tau_resi;
	r_re.x[k_pre] = comp_late.r;
	// re

	return comp_late;
}

RC_comp_onedirection RC_Sagitta(RC_comp_onedirection comp_sagi, double zmp_y_ref, double zmp_y_rel, double Z_c, double T_filter, double K_RC[4], double T_lag[2], double K_VMM[6], double micro[2], double T_start_walk, int k_pre)
{
	double F_ext;				double Tau_ext;
	double F_resi;				double Tau_resi;
	double K_1_e = K_RC[0];		double K_1_r = K_RC[2];
	double K_2_e = K_RC[1];		double K_2_r = K_RC[3];
	double K_p_e = K_VMM[0];	double K_p_r = K_VMM[3];
	double K_d_e = K_VMM[1];	double K_d_r = K_VMM[4];
	double K_m_e = K_VMM[2];	double K_m_r = K_VMM[5];
	double T_lag_e = T_lag[0];	double T_lag_r = T_lag[1];
	double RC_micro = micro[0]; double MPC_micro = micro[1];
	double Tau_micro = 1.0;
	double T_rls = 0.5; // cancel resist force in T_rls seconds after release the external force
	if (k_pre < T_start_walk / CONTROL_T)
	{
		RC_micro  = 0.01;
		MPC_micro = 0.05;
		Tau_micro = 0.01;
	}

	// F ------------------------------------------------------t-------------------------------------------
	F_ext = 0.0 * m_robot * comp_sagi.ddel + m_robot * GRAVITY / Z_c * (zmp_y_rel - zmp_y_ref - 0.0 * comp_sagi.el) + 0.3 * (-2.0 + 24.5 * 0.5 * (Ref_Leg_Joint[1][4] + Ref_Leg_Joint[2][4]));
	if (F_ext >  80) F_ext = 80;
	if (F_ext < -80) F_ext = -80;
	if (Count_Check_Release_y++ == T_rls * 0.2 / CONTROL_T) // continues decline of F_ext for five times, then release is judged
	{
		Count_Check_Release_y = 0;
		for (int i = 5; i > 0; i--)
		{
			F_Ext_Sagi[i] = F_Ext_Sagi[i - 1];
		}
	}
	F_Ext_Sagi[0] = (F_ext + T_filter / CONTROL_T * F_Ext_Sagi[0]) / (1 + T_filter / CONTROL_T);
	F_ext = F_Ext_Sagi[0];

	F_resi = -K_1_e * F_ext - K_2_e * comp_sagi.el;
	F_Vir_Sagi = (F_resi + T_lag_e / CONTROL_T * F_Vir_Sagi) / (1 + T_lag_e / CONTROL_T);
	F_resi = F_Vir_Sagi;

	#ifdef STANDING_PUSH_CHECK
		if ((abs(F_Ext_Sagi[5]) > abs(F_Ext_Sagi[4])) && (abs(F_Ext_Sagi[4]) > abs(F_Ext_Sagi[3])) && (abs(F_Ext_Sagi[3]) > abs(F_Ext_Sagi[2])) && (abs(F_Ext_Sagi[2]) > abs(F_Ext_Sagi[1])) && (abs(F_Ext_Sagi[1]) > abs(F_Ext_Sagi[0])))
		{
			F_resi = -F_ext; // if release, stop resisting
			F_Vir_Sagi = F_resi;
		}
	#endif

	//comp_sagi.dde = (F_mpc_y - K_p_e * comp_sagi.el - K_d_e * comp_sagi.del) / K_m_e;
	//comp_sagi.dde = (RC_micro * (F_ext + F_resi)  - K_p_e * comp_sagi.el - K_d_e * comp_sagi.del) / K_m_e;
	//comp_sagi.dde = (RC_micro * (F_ext + F_resi) + /*MPC_micro * F_mpc_y*/ - K_p_e * comp_sagi.el - K_d_e * comp_sagi.del) / K_m_e;
	//comp_sagi.dde = (RC_micro * (F_ext + F_resi) + 1.0 * MPC_micro * F_MPC.y - K_p_e * comp_sagi.el - K_d_e * comp_sagi.del) / K_m_e;
	// comp_sagi.de = comp_sagi.del + comp_sagi.dde * CONTROL_T;
	// if (comp_sagi.de >  0.1) comp_sagi.de = 0.1;
	// if (comp_sagi.de < -0.1) comp_sagi.de = -0.1;
	// comp_sagi.e = comp_sagi.el + comp_sagi.de * CONTROL_T;
	// if (comp_sagi.e >  0.05) comp_sagi.e = 0.05;
	// if (comp_sagi.e < -0.05) comp_sagi.e = -0.05;

	// comp_sagi.ddel = comp_sagi.dde;
	// comp_sagi.del = comp_sagi.de;
	// comp_sagi.el = comp_sagi.e;
	
	comp_sagi.e = (RC_micro * (F_ext + F_resi) / K_p_e  + (K_d_e / CONTROL_T / K_p_e + K_m_e / CONTROL_T / CONTROL_T / K_p_e) * comp_sagi.el + K_m_e / CONTROL_T / K_p_e * comp_sagi.del) / (1 + K_d_e / CONTROL_T / K_p_e + K_m_e / CONTROL_T / CONTROL_T / K_p_e);
	if (comp_sagi.e >  0.1) comp_sagi.e =  0.1;
	if (comp_sagi.e < -0.1) comp_sagi.e = -0.1;
	comp_sagi.de = (comp_sagi.e - comp_sagi.el) / CONTROL_T;
	comp_sagi.dde = (comp_sagi.de - comp_sagi.del) / CONTROL_T;
	comp_sagi.ddel = comp_sagi.dde;
	comp_sagi.del = comp_sagi.de;
	comp_sagi.el = comp_sagi.e;

	// re
	F_ext_re.y[k_pre] = F_ext;
	F_vir_re.y[k_pre] = F_resi;
	e_re.y[k_pre] = comp_sagi.e;
	// re

	// Tau -----------------------------------------------------------------------------------------------
	Tau_ext = F_ext * 0.3 * Z_c;

	Tau_resi = -K_1_r * Tau_ext - K_2_r * comp_sagi.rl;
	Tau_Vir_Sagi = (Tau_resi + T_lag_r / CONTROL_T * Tau_Vir_Sagi) / (1 + T_lag_r / CONTROL_T);
	Tau_resi = Tau_Vir_Sagi;

	// comp_sagi.ddr = (Tau_micro * (Tau_ext + Tau_resi) - K_p_r * comp_sagi.rl - K_d_r * comp_sagi.drl) / K_m_r;
	// comp_sagi.dr = comp_sagi.drl + comp_sagi.ddr * CONTROL_T;
	// if (comp_sagi.dr >  20 / 57.3) comp_sagi.dr = 20 / 57.3;
	// if (comp_sagi.dr < -20 / 57.3) comp_sagi.dr = -20 / 57.3;
	// comp_sagi.r = comp_sagi.rl + comp_sagi.dr * CONTROL_T;
	// if (comp_sagi.r >  10 / 57.3) comp_sagi.r =  10 / 57.3;
	// if (comp_sagi.r < -10 / 57.3) comp_sagi.r = -10 / 57.3;

	// comp_sagi.ddrl = comp_sagi.ddr;
	// comp_sagi.drl = comp_sagi.dr;
	// comp_sagi.rl = comp_sagi.r;
	
	comp_sagi.r = 1.0 * ((RC_micro * (Tau_ext + Tau_resi) / K_p_r + (K_d_r / CONTROL_T / K_p_r + K_m_r / CONTROL_T / CONTROL_T / K_p_r) * comp_sagi.rl + K_m_r / CONTROL_T / K_p_r * comp_sagi.drl) / (1 + K_d_r / CONTROL_T / K_p_r + K_m_r / CONTROL_T / CONTROL_T / K_p_r));
	if (comp_sagi.r >  20 / 57.3) comp_sagi.r =  20 / 57.3;
	if (comp_sagi.r < -20 / 57.3) comp_sagi.r = -20 / 57.3;
	comp_sagi.dr = (comp_sagi.r - comp_sagi.rl) / CONTROL_T;
	comp_sagi.ddr = (comp_sagi.dr - comp_sagi.drl) / CONTROL_T;
	comp_sagi.ddrl = comp_sagi.ddr;
	comp_sagi.drl = comp_sagi.dr;
	comp_sagi.rl = comp_sagi.r;

	// re
	Tau_ext_re.y[k_pre] = Tau_ext;
	Tau_vir_re.y[k_pre] = Tau_resi;
	r_re.y[k_pre] = comp_sagi.r;
	// re

	return comp_sagi;
}

void VMM_RC_controller(double Z_c, int k_pre)
{
	double T_start_walk = 4.0;
	double mpc_micro_x[2] = { 0.4, 0.01 * 0.4 }; // { RC_micro, MPC_micro } { 1.0, 1.0 * 0.065 };	 // { 1.0, 1.0 } for standing on rotating slope
	double mpc_micro_y[2] = { 0.4, 0.01 * 0.4 }; // { RC_micro, MPC_micro } { 1.0, 1.0 * 0.1   };	     // { 1.0, 1.0 } for standing on rotating slope	
	double T_filter_late = 0.10; // Time lag of input Force for lateral
	double T_filter_sagi = 0.10; // Time lag of input Force for sagitta
	double K_RC_late[4] = { 1.60, 50.0, 1.50, 30.0 }; // {K_1_e, K_2_e, K_1_r, K_2_r} for lateral  { 1.6 * 1.20, 120.0, 2.2 * 1.17, 100.0 }; // for standing on rotating slope
	double K_RC_sagi[4] = { 1.2, 50.0, 1.1, 30.0 }; // {K_1_e, K_2_e, K_1_r, K_2_r} for sagitta
	double T_lag_sagi[2] = { 1.6, 1.0 }; // {T_lag_e, T_lag_r} for lateral
	double T_lag_late[2] = { 1.6, 0.8 }; // {T_lag_e, T_lag_r} for saggita
	double K_VMM_late[6] = { 1.2 * 620.0, 1.2 * 290.0, 2.0, 1.2 * 70.0, 1.2 * 13.0, 0.0086 }; //500.0, 250.0, 2.0, 60.0, 12.0, 0.0086//720.0, 320.0, 2.0, 85.0, 15.0, 0.0086// {K_p_e, K_d_e, K_m_e, K_p_r, K_d_r, K_m_r} for lateral
	double K_VMM_sagi[6] = { 1.2 * 720.0, 1.2 * 400.0, 2.0, 1.2 * 50.0, 1.2 * 9.0, 0.0086 }; //500.0, 250.0, 2.0, 60.0, 12.0, 0.0086//720.0, 320.0, 2.0, 85.0, 15.0, 0.0086// {K_p_e, K_d_e, K_m_e, K_p_r, K_d_r, K_m_r} for sagitta
	double K_model[3] = { 260.0, 120.0, 22.0 };
	double mech_paras[2] = { 1.0, 22.0};
	double R = 1e-7;
	double T_pre = 1.0;
	double rol = 1e-2;
	
	double zmp_x_ref = Tra_ZMP.x[k_pre] - Tra_COM.x[k_pre];
	double zmp_y_ref = Tra_ZMP.y[k_pre] - Tra_COM.y[k_pre];
#ifdef STANDSTILL
	zmp_x_ref = 0.0;
	zmp_y_ref = 0.0;
#endif
	double zmp_x_rel = P_ZMPRel_B.px;
	double zmp_y_rel = P_ZMPRel_B.py;
#ifdef USE_MPC
	// double micro = mpc_micro_x[1];
	// F_MPC = cal_MPC_con(K_model, mech_paras, R, T_pre, rol, F_MPC, k_pre, micro);
	// F_mpc_x_re  = F_MPC.x;
	// state_x1_re = F_MPC.state_x1;
	// state_x2_re = F_MPC.state_x2;
#endif
	RC_Comp_Late = RC_Lateral(RC_Comp_Late, zmp_x_ref, zmp_x_rel, Z_c, T_filter_late, K_RC_late, T_lag_late, K_VMM_late, mpc_micro_x, T_start_walk, k_pre);
	RC_Comp_Sagi = RC_Sagitta(RC_Comp_Sagi, zmp_y_ref, zmp_y_rel, Z_c, T_filter_sagi, K_RC_sagi, T_lag_sagi, K_VMM_sagi, mpc_micro_y, T_start_walk, k_pre);
	RC_Comp_Cont.late = RC_Comp_Late;
	RC_Comp_Cont.sagi = RC_Comp_Sagi;

	state_com =  State_OB(k_pre);
}

State_CoM State_OB(int k_pre) // wrong! this state should be observed from zmp, not cal from e 
{
	State_CoM state_com;
	state_com.px = Tra_COM.x[k_pre] + RC_Comp_Cont.late.e;
	state_com.py = Tra_COM.y[k_pre] + RC_Comp_Cont.sagi.e;
	state_com.vx = Tra_VCOM.x[k_pre] + RC_Comp_Cont.late.de;
	state_com.vy = Tra_VCOM.y[k_pre] + RC_Comp_Cont.sagi.de;

	return state_com;
}


//#ifdef USE_RESISTANT_COMPLIANCE
//VMM_RC_controller(K_Preview_Con);
//roll_body = RC_Comp_Cont.late.r;
//pitch_body = -RC_Comp_Cont.sagi.r;
//Tra_COM.x[K_Preview_Con] += RC_Comp_Cont.late.e;
//Tra_COM.y[K_Preview_Con] += RC_Comp_Cont.sagi.e;
//#endif