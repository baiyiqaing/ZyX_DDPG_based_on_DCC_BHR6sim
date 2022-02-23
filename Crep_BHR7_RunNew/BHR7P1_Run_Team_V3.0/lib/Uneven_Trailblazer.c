#include <math.h>
#include "Uneven_Trailblazer.h"

enum Support_Leg { DOUBLE_LEG_SP, RLEG_SP, LLEG_SP };
extern double GQ_Roll;
extern double GQ_Pitch;

Horizontal_Current Get_Pd_CP(Horizontal_Current GICP_ref, Horizontal_Current GICP_rel, Horizontal_Current p_ref, double k_cp[2])
{
	Horizontal_Current pd_cp;
	double k_cp_x = k_cp[0];
	double k_cp_y = k_cp[1];
	double delta_px_temp = k_cp_x * (GICP_rel.x - GICP_ref.x);
	double delta_py_temp = k_cp_y * (GICP_rel.y - GICP_ref.y);
	if (delta_px_temp >  0.5) delta_px_temp =  0.5;
	if (delta_px_temp < -0.5) delta_px_temp = -0.5;
	if (delta_py_temp >  0.5) delta_py_temp =  0.5;
	if (delta_py_temp < -0.5) delta_py_temp = -0.5;
	pd_cp.x = p_ref.x + delta_px_temp;
	pd_cp.y = p_ref.y + delta_py_temp;

	return pd_cp; // in W_frame
}

SS_2x2 c2d_2x2(SS_2x2 SS_IPSD_c)
{
	SS_2x2 SS_IPSD_d;

	SS_IPSD_d.a11 = SS_IPSD_c.a11 * CONTROL_T + 1;
	SS_IPSD_d.a12 = SS_IPSD_c.a12 * CONTROL_T;
	SS_IPSD_d.a21 = SS_IPSD_c.a21 * CONTROL_T;
	SS_IPSD_d.a22 = SS_IPSD_c.a22 * CONTROL_T + 1;

	SS_IPSD_d.b11 = SS_IPSD_c.b11 * CONTROL_T;
	SS_IPSD_d.b21 = SS_IPSD_c.b21 * CONTROL_T;

	return  SS_IPSD_d;
}

Horizontal_Current ZMP_Controller(double k_IPSD[2], double G_ob[2], double K_con[2], Horizontal_Current Gp_rel, Horizontal_Current Gpd_cp)
{
	SS_2x2 SS_IPSD_c;
	SS_2x2 SS_IPSD_d;
	double kp = k_IPSD[0];
	double kd = k_IPSD[1];
	double G1 = G_ob[0];
	double G2 = G_ob[1];
	double k1 = K_con[0];
	double k2 = K_con[1];
	Horizontal_Current delta_com_con;

	// c2d
	SS_IPSD_c.a11 = 0.0;
	SS_IPSD_c.a12 = 1.0;
	SS_IPSD_c.a21 = (m_robot * GRAVITY * H_zc - kp) / m_robot / H_zc / H_zc;
	SS_IPSD_c.a22 = - kd / m_robot / H_zc / H_zc;
	SS_IPSD_c.b11 = 0.0;
	SS_IPSD_c.b21 = kp / m_robot / H_zc / H_zc;

	SS_IPSD_d = c2d_2x2(SS_IPSD_c);

	// delta_p from theta
	delta_p_hat.x =      kp / m_robot / GRAVITY * State_CP.roll + kd / m_robot / GRAVITY * State_CP.droll - kp / m_robot / GRAVITY * Theta_Con.roll;	// last "-" if right
	delta_p_hat.y = - (kp / m_robot / GRAVITY * State_CP.pitch + kd / m_robot / GRAVITY * State_CP.dpitch - kp / m_robot / GRAVITY * Theta_Con.pitch);	// last "-" if right

	// observer
	State_CP.roll   = SS_IPSD_d.a11 * State_CP.roll  + SS_IPSD_d.a12 * State_CP.droll  + SS_IPSD_d.b11 * Theta_Con.roll  + G1 * (Gp_rel.x - Gpd_cp.x - delta_p_hat.x);
	State_CP.droll  = SS_IPSD_d.a21 * State_CP.roll  + SS_IPSD_d.a22 * State_CP.droll  + SS_IPSD_d.b21 * Theta_Con.roll  + G2 * (Gp_rel.x - Gpd_cp.x - delta_p_hat.x);
	State_CP.pitch  = SS_IPSD_d.a11 * State_CP.pitch + SS_IPSD_d.a12 * State_CP.dpitch + SS_IPSD_d.b11 * Theta_Con.pitch - G1 * (Gp_rel.y - Gpd_cp.y - delta_p_hat.y); // last "+" if right
	State_CP.dpitch = SS_IPSD_d.a21 * State_CP.pitch + SS_IPSD_d.a22 * State_CP.dpitch + SS_IPSD_d.b21 * Theta_Con.pitch - G2 * (Gp_rel.y - Gpd_cp.y - delta_p_hat.y); // last "+" if right

	Theta_Con.roll  = - k1 * State_CP.roll  - k2 * State_CP.droll;
	Theta_Con.pitch = - k1 * State_CP.pitch - k2 * State_CP.dpitch;

	delta_com_con.x =	1.0 * Theta_Con.roll  * H_zc;
	delta_com_con.y = - 1.0 * Theta_Con.pitch * H_zc;
	if (delta_com_con.x >  0.04) delta_com_con.x =  0.04;
	if (delta_com_con.x < -0.04) delta_com_con.x = -0.04;
	if (delta_com_con.y >  0.06) delta_com_con.y =  0.06;
	if (delta_com_con.y < -0.06) delta_com_con.y = -0.06;

	return delta_com_con;
}

SD_UPTATE Damping_HUBO(SD_UPTATE damp, double Fd[2], double F[2], double k[4], int k_pre)
{
	double F_ref  = Fd[0];
	double dF_ref = Fd[1];
	double F_rel  = F[0];
	double dF_rel = F[1];
	double kp = k[0];
	double kd = k[1];
	double kr = k[2];
	double micro_dou_kr = k[3];

	if (Signal_SupportLeg[k_pre] == DOUBLE_LEG_SP)
	{
		kr = micro_dou_kr * kr;
	}

	damp.de = kp * (F_rel - F_ref) + kd * (dF_rel - dF_ref) - kr * damp.e;
	damp.e = damp.e + damp.de * CONTROL_T;

	return damp;
}

FEET_BIAS Compliance_HUBO(FEET_BIAS HUBO_foot_com, Horizontal_Current pd_cp, double kz[4], double kt[4], double filter_Fz_T, int k_pre)
{
	double alpha;
	double FL_z_temp;
	double FR_z_temp;
	double TL_rol_temp;
	double TR_rol_temp;
	double TL_pit_temp;
	double TR_pit_temp;
	double FdL_z[2];  //ref
	double FdR_z[2];
	double Fd_ztrl[2];
	double TdL_rol[2];
	double TdR_rol[2];
	double TdL_pit[2];
	double TdR_pit[2];
	double FrL_z[2];  // rel
	double FrR_z[2];
	double Fr_ztrl[2];
	double TrL_rol[2];
	double TrR_rol[2];
	double TrL_pit[2];
	double TrR_pit[2];
	double dou_flag = 0.0;
	double left_flag = 0.0;
	double rigt_flag = 0.0;

	if (Signal_SupportLeg[k_pre] == DOUBLE_LEG_SP)
	{
		dou_flag  = 1.0;
		left_flag = 0.0;
		rigt_flag = 0.0;
	}
	else if (Signal_SupportLeg[k_pre] == LLEG_SP)
	{
		dou_flag  = 0.0;
		left_flag = 1.0;
		rigt_flag = 0.0;
	}
	else if (Signal_SupportLeg[k_pre] == RLEG_SP)
	{
		dou_flag  = 0.0;
		left_flag = 0.0;
		rigt_flag = 1.0;
	}

	// alpha = (Tra_RAnkle.x[k_pre] - Tra_ZMP.x[k_pre]) / (Tra_RAnkle.x[k_pre] - Tra_LAnkle.x[k_pre]);
	// alpha = (Tra_RAnkle.x[k_pre] - pd_cp.x) / (Tra_RAnkle.x[k_pre] - Tra_LAnkle.x[k_pre]);
	alpha = (0.5 * (FOOT_DISTANCE + 0.0) - pd_cp.x) / (FOOT_DISTANCE + 0.0);
	if (alpha > 1) alpha = 1;
	else if (alpha < 0) alpha = 0;

	// cal FOOTFT ref
	FL_z_temp   = alpha * m_robot * GRAVITY - dou_flag * 2 * m_robot * GRAVITY * GQ_Roll / ankle_width;
	if (FL_z_temp >  600.0) FL_z_temp =  600.0;
	if (FL_z_temp < -600.0) FL_z_temp = -600.0;
	FR_z_temp   = (1 - alpha) * m_robot * GRAVITY + dou_flag * 2 * m_robot * GRAVITY * GQ_Roll / ankle_width;
	if (FR_z_temp >  600.0) FR_z_temp =  600.0;
	if (FR_z_temp < -600.0) FR_z_temp = -600.0;
	TL_rol_temp = left_flag * (FEET_FT_HUBO_ref.Lfoot.F_z * (Tra_ZMP.x[k_pre] - pd_cp.x) - m_robot * GRAVITY * GQ_Roll);
	if (TL_rol_temp >  100.0) TL_rol_temp =  100.0;
	if (TL_rol_temp < -100.0) TL_rol_temp = -100.0;
	TR_rol_temp = rigt_flag * (FEET_FT_HUBO_ref.Rfoot.F_z * (Tra_ZMP.x[k_pre] - pd_cp.x) - m_robot * GRAVITY * GQ_Roll);
	if (TR_rol_temp >  100.0) TR_rol_temp =  100.0;
	if (TR_rol_temp < -100.0) TR_rol_temp = -100.0;
	TL_pit_temp = (FEET_FT_HUBO_ref.Lfoot.F_z * (pd_cp.y - Tra_ZMP.y[k_pre]) + left_flag * m_robot * GRAVITY * GQ_Pitch);
	if (TL_pit_temp >  100.0) TL_pit_temp =  100.0;
	if (TL_pit_temp < -100.0) TL_pit_temp = -100.0;
	TR_pit_temp = (FEET_FT_HUBO_ref.Rfoot.F_z * (pd_cp.y - Tra_ZMP.y[k_pre]) + rigt_flag * m_robot * GRAVITY * GQ_Pitch);
	if (TR_pit_temp >  100.0) TR_pit_temp =  100.0;
	if (TR_pit_temp < -100.0) TR_pit_temp = -100.0;
	//re
	FEET_FT_HUBO_ref.Lfoot.dF_z   = (FL_z_temp   - FEET_FT_HUBO_ref.Lfoot.F_z)   / CONTROL_T;
	FEET_FT_HUBO_ref.Rfoot.dF_z   = (FR_z_temp   - FEET_FT_HUBO_ref.Rfoot.F_z)   / CONTROL_T;
	FEET_FT_HUBO_ref.Lfoot.dT_rol = (TL_rol_temp - FEET_FT_HUBO_ref.Lfoot.T_rol) / CONTROL_T;
	FEET_FT_HUBO_ref.Rfoot.dT_rol = (TR_rol_temp - FEET_FT_HUBO_ref.Rfoot.T_rol) / CONTROL_T;
	FEET_FT_HUBO_ref.Lfoot.dT_pit = (TL_pit_temp - FEET_FT_HUBO_ref.Lfoot.T_pit) / CONTROL_T;
	FEET_FT_HUBO_ref.Rfoot.dT_pit = (TR_pit_temp - FEET_FT_HUBO_ref.Rfoot.T_pit) / CONTROL_T;
	FEET_FT_HUBO_ref.Lfoot.F_z   = FL_z_temp;
	FEET_FT_HUBO_ref.Rfoot.F_z   = FR_z_temp;
	FEET_FT_HUBO_ref.Lfoot.T_rol = TL_rol_temp;
	FEET_FT_HUBO_ref.Rfoot.T_rol = TR_rol_temp;
	FEET_FT_HUBO_ref.Lfoot.T_pit = TL_pit_temp;
	FEET_FT_HUBO_ref.Rfoot.T_pit = TR_pit_temp;
	//re
	FdL_z[0]   = FEET_FT_HUBO_ref.Lfoot.F_z;   FdL_z[1]   = FEET_FT_HUBO_ref.Lfoot.dF_z;
	FdR_z[0]   = FEET_FT_HUBO_ref.Rfoot.F_z;   FdR_z[1]   = FEET_FT_HUBO_ref.Rfoot.dF_z;
	Fd_ztrl[0] = FdL_z[0] - FdR_z[0];
	Fd_ztrl[1] = FdL_z[1] - FdR_z[1];
	TdL_rol[0] = FEET_FT_HUBO_ref.Lfoot.T_rol; TdL_rol[1] = FEET_FT_HUBO_ref.Lfoot.dT_rol;
	TdR_rol[0] = FEET_FT_HUBO_ref.Rfoot.T_rol; TdR_rol[1] = FEET_FT_HUBO_ref.Rfoot.dT_rol;
	TdL_pit[0] = FEET_FT_HUBO_ref.Lfoot.T_pit; TdL_pit[1] = FEET_FT_HUBO_ref.Lfoot.dT_pit;
	TdR_pit[0] = FEET_FT_HUBO_ref.Rfoot.T_pit; TdR_pit[1] = FEET_FT_HUBO_ref.Rfoot.dT_pit;
	// cal FOOTFT ref

	// cal FOOTFT rel
	FL_z_temp   = F_LFoot.fz;
	if (FL_z_temp >  600.0) FL_z_temp =  600.0;
	if (FL_z_temp < -600.0) FL_z_temp = -600.0;
	FR_z_temp   = F_RFoot.fz;
	if (FR_z_temp >  600.0) FR_z_temp =  600.0;
	if (FR_z_temp < -600.0) FR_z_temp = -600.0;
	TL_rol_temp = F_LFoot.ty;
	if (TL_rol_temp >  100.0) TL_rol_temp =  100.0;
	if (TL_rol_temp < -100.0) TL_rol_temp = -100.0;
	TR_rol_temp = F_RFoot.ty;
	if (TR_rol_temp >  100.0) TR_rol_temp =  100.0;
	if (TR_rol_temp < -100.0) TR_rol_temp = -100.0;
	TL_pit_temp = F_LFoot.tx;
	if (TL_pit_temp >  100.0) TL_pit_temp =  100.0;
	if (TL_pit_temp < -100.0) TL_pit_temp = -100.0;
	TR_pit_temp = F_RFoot.tx;
	if (TR_pit_temp >  100.0) TR_pit_temp =  100.0;
	if (TR_pit_temp < -100.0) TR_pit_temp = -100.0;
	FL_z_temp = (filter_Fz_T / CONTROL_T * FEET_FT_HUBO_rel.Lfoot.F_z + FL_z_temp) / (1 + filter_Fz_T / CONTROL_T); // Fz filter
	FR_z_temp = (filter_Fz_T / CONTROL_T * FEET_FT_HUBO_rel.Rfoot.F_z + FR_z_temp) / (1 + filter_Fz_T / CONTROL_T); // Fz filter
	// re
	FEET_FT_HUBO_rel.Lfoot.dF_z   = (FL_z_temp   - FEET_FT_HUBO_rel.Lfoot.F_z) / CONTROL_T;
	FEET_FT_HUBO_rel.Rfoot.dF_z   = (FR_z_temp   - FEET_FT_HUBO_rel.Rfoot.F_z) / CONTROL_T;
	FEET_FT_HUBO_rel.Lfoot.dT_rol = (TL_rol_temp - FEET_FT_HUBO_rel.Lfoot.T_rol) / CONTROL_T;
	FEET_FT_HUBO_rel.Rfoot.dT_rol = (TR_rol_temp - FEET_FT_HUBO_rel.Rfoot.T_rol) / CONTROL_T;
	FEET_FT_HUBO_rel.Lfoot.dT_pit = (TL_pit_temp - FEET_FT_HUBO_rel.Lfoot.T_pit) / CONTROL_T;
	FEET_FT_HUBO_rel.Rfoot.dT_pit = (TR_pit_temp - FEET_FT_HUBO_rel.Rfoot.T_pit) / CONTROL_T;
	FEET_FT_HUBO_rel.Lfoot.F_z   = FL_z_temp;
	FEET_FT_HUBO_rel.Rfoot.F_z   = FR_z_temp;
	FEET_FT_HUBO_rel.Lfoot.T_rol = TL_rol_temp;
	FEET_FT_HUBO_rel.Rfoot.T_rol = TR_rol_temp;
	FEET_FT_HUBO_rel.Lfoot.T_pit = TL_pit_temp;
	FEET_FT_HUBO_rel.Rfoot.T_pit = TR_pit_temp;
	//re
	FrL_z[0]   = FEET_FT_HUBO_rel.Lfoot.F_z;   FrL_z[1]   = FEET_FT_HUBO_rel.Lfoot.dF_z;
	FrR_z[0]   = FEET_FT_HUBO_rel.Rfoot.F_z;   FrR_z[1]   = FEET_FT_HUBO_rel.Rfoot.dF_z;
	Fr_ztrl[0] = FrL_z[0] - FrR_z[0];
	Fr_ztrl[1] = FrL_z[1] - FrR_z[1];
	TrL_rol[0] = FEET_FT_HUBO_rel.Lfoot.T_rol; TrL_rol[1] = FEET_FT_HUBO_rel.Lfoot.dT_rol;
	TrR_rol[0] = FEET_FT_HUBO_rel.Rfoot.T_rol; TrR_rol[1] = FEET_FT_HUBO_rel.Rfoot.dT_rol;
	TrL_pit[0] = FEET_FT_HUBO_rel.Lfoot.T_pit; TrL_pit[1] = FEET_FT_HUBO_rel.Lfoot.dT_pit;
	TrR_pit[0] = FEET_FT_HUBO_rel.Rfoot.T_pit; TrR_pit[1] = FEET_FT_HUBO_rel.Rfoot.dT_pit;
	// cal FOOTFT rel
		
	// GRFC 
	HUBO_foot_com.Lfoot.rol = Damping_HUBO(HUBO_foot_com.Lfoot.rol, TdL_rol, TrL_rol, kt, k_pre);
	HUBO_foot_com.Rfoot.rol = Damping_HUBO(HUBO_foot_com.Rfoot.rol, TdR_rol, TrR_rol, kt, k_pre);
	if (HUBO_foot_com.Lfoot.rol.e >  10 / 57.3) HUBO_foot_com.Lfoot.rol.e =  10 / 57.3;
	if (HUBO_foot_com.Lfoot.rol.e < -10 / 57.3) HUBO_foot_com.Lfoot.rol.e = -10 / 57.3;
	if (HUBO_foot_com.Rfoot.rol.e >  10 / 57.3) HUBO_foot_com.Rfoot.rol.e =  10 / 57.3;
	if (HUBO_foot_com.Rfoot.rol.e < -10 / 57.3) HUBO_foot_com.Rfoot.rol.e = -10 / 57.3;
	HUBO_foot_com.Lfoot.pit = Damping_HUBO(HUBO_foot_com.Lfoot.pit, TdL_pit, TrL_pit, kt, k_pre);
	HUBO_foot_com.Rfoot.pit = Damping_HUBO(HUBO_foot_com.Rfoot.pit, TdR_pit, TrR_pit, kt, k_pre);
	if (HUBO_foot_com.Lfoot.pit.e >  10 / 57.3) HUBO_foot_com.Lfoot.pit.e = 10 / 57.3;
	if (HUBO_foot_com.Lfoot.pit.e < -10 / 57.3) HUBO_foot_com.Lfoot.pit.e = -10 / 57.3;
	if (HUBO_foot_com.Rfoot.pit.e >  10 / 57.3) HUBO_foot_com.Rfoot.pit.e = 10 / 57.3;
	if (HUBO_foot_com.Rfoot.pit.e < -10 / 57.3) HUBO_foot_com.Rfoot.pit.e = -10 / 57.3;
	HUBO_foot_ztrl = Damping_HUBO(HUBO_foot_ztrl, Fd_ztrl, Fr_ztrl, kz, k_pre);
	HUBO_foot_com.Lfoot.z.e =  0.5 * HUBO_foot_ztrl.e;
	HUBO_foot_com.Rfoot.z.e = -0.5 * HUBO_foot_ztrl.e;
	if (HUBO_foot_com.Lfoot.z.e >  0.15) HUBO_foot_com.Lfoot.z.e =  0.15;
	if (HUBO_foot_com.Lfoot.z.e < -0.01) HUBO_foot_com.Lfoot.z.e = -0.01;
	if (HUBO_foot_com.Rfoot.z.e >  0.15) HUBO_foot_com.Rfoot.z.e =  0.15;
	if (HUBO_foot_com.Rfoot.z.e < -0.01) HUBO_foot_com.Rfoot.z.e = -0.01;
	
	return HUBO_foot_com;
}

void HUBO19(int k_pre)
{
	double y_bias_ = -0.005;                                 // stand
	double filter_ICP[2]	= { 0.0, 1.0 };					// = { 0.0, 1.0 };				// A, B
	double filter_ZMP[2]	= { 0.3, 0.7 };					// = { 0.0, 1.0 };				// A, B
	double filter_Fz_T		=   0.005;						// =   0.01;				    // TF_z
	double filter_COM_T[2]  = { 0.1, 0.1 };				// = { 0.1, 0.5 };				// Tx, Ty
	double k_IPSD[2]		= { 5744, 69.375 };				// = { 5744, 69.375 };			// kp, kd
	double G_ob[2]			= { 3.5e-2, 2.8e-2 };			// = { 1.0e-2, 0.7e-2 };		// L1, L2 
	double K_con[2]			= { 0.3, 0.75}; // { 0.50, 1.2 }; //{ 0.50, 0.748 };				// = { 0.10, 0.648 };			// K1, K2 	
	double k_cp[2]			= { 2.05, 1.45 };				// = { 2.4, 4.5 };				// k_cp_x, k_cp_y 
	double ome				=	3.14;						// = 3.14						// 
	double kZ[4]			= { 1e-4, 2e-7, 3e-2, 0.2 };	// = { 3e-4, 2e-7, 3e-3, 0.2 }; // kp, kd, kr, micro_dou 
	double kT[4]			= { 6e-3, 9e-6, 2e-1, 0.2 };	// = { 8e-3, 5e-6, 5e-2, 0.2 }; // kp, kd, kr, micro_dou 
	Horizontal_Current WICP_temp; // filter
	Horizontal_Current WZMP_temp; // filter
	Position GICP_temp;
	Position ICP_temp;
	Position GCOM_temp;
	Position COM_temp;
	Horizontal_Current p_ref;
	Position pd_cp_temp;
	Position gpd_cp_temp;
	Position gp_rel_temp;
	Position wzmp_ref_temp;
	Position gp_ref_temp;
	Horizontal_Current delta_com_temp;

	// frilters
	double A_ICP = filter_ICP[0];
	double B_ICP = filter_ICP[1];
	double A_ZMP = filter_ZMP[0];
	double B_ZMP = filter_ZMP[1];
	// frilters

	// cal GICP_ref
	Cal_ICP_ref(Tra_COM, Tra_VCOM, 'G');
	// re
	GICP_ref.x = Tra_GICP.x[k_pre];
	GICP_ref.y = Tra_GICP.y[k_pre];
	// re

	// cal GICP_rel
	COM_temp.px = Tra_COM.x[k_pre];
	COM_temp.py = Tra_COM.y[k_pre];
	GCOM_temp = Trans2_Gframe_Cur(COM_temp, k_pre);
	GCOM_ref.x = GCOM_temp.px;
	GCOM_ref.y = GCOM_temp.py;
	// re
	GCOM_rel.x = GCOM_temp.px + H_zc * sin(GQ_Roll);
	GCOM_rel.y = GCOM_temp.py - H_zc * sin(-GQ_Pitch + y_bias_);
	WCOM_rel.x = COM_temp.px + H_zc * sin(GQ_Roll);
	WCOM_rel.y = COM_temp.py - H_zc * sin(-GQ_Pitch + y_bias_);
	WICP_temp.x = WCOM_rel.x + (Tra_VCOM.x[k_pre] + Body_VX) / ome; // filter 
	WICP_temp.y = WCOM_rel.y + (Tra_VCOM.y[k_pre] + Body_VY) / ome; // filter
	WICP_rel.x = A_ICP * WICP_rel.x	+ B_ICP * WICP_temp.x; // filter
	WICP_rel.y = A_ICP * WICP_rel.y + B_ICP * WICP_temp.y; // filter
	ICP_temp.px = WICP_rel.x;
	ICP_temp.py = WICP_rel.y;
	GICP_temp = Trans2_Gframe_Cur(ICP_temp, k_pre); // filter should be before trans
	GICP_rel.x = GICP_temp.px;
	GICP_rel.y = GICP_temp.py;
	// re

	// cal Pd_CP & GPd_CP
	p_ref.x = Tra_ZMP.x[k_pre];
	p_ref.y = Tra_ZMP.y[k_pre];
	// re
	//Pd_CP =  Get_Pd_CP(GICP_ref, GICP_rel, p_ref, k_cp); // <- CP CONTROLLER  // tune_dcc
	// Pd_CP.x = -0.0 * 0.08;
	// Pd_CP.y =  0.0 * 0.0;
	Pd_CP.x = p_ref.x; // don't use CP con
	Pd_CP.y = p_ref.y;
	// re
	pd_cp_temp.px = Pd_CP.x;
	pd_cp_temp.py = Pd_CP.y;
	gpd_cp_temp = Trans2_Gframe_Cur(pd_cp_temp, k_pre);
	// re
	GPd_CP.x = gpd_cp_temp.px;
	GPd_CP.y = gpd_cp_temp.py;
	// re

	// cal GP_rel & Gp_ref 
	WZMP_temp.x = P_ZMPRel_B.px + 1.0 * Tra_COM.x[k_pre]; // maybe has error // tune_dcc
	WZMP_temp.y = P_ZMPRel_B.py + 1.0 * Tra_COM.y[k_pre];	// maybe has error
	WZMP_rel.px = A_ZMP * WZMP_rel.px + B_ZMP * WZMP_temp.x; // fillter
	WZMP_rel.py = A_ZMP * WZMP_rel.py + B_ZMP * WZMP_temp.y; // fillter
	gp_rel_temp = Trans2_Gframe_Cur(WZMP_rel, k_pre);
	// re
	GP_rel.x = gp_rel_temp.px;
	GP_rel.y = gp_rel_temp.py;
	// re
	wzmp_ref_temp.px = Tra_ZMP.x[k_pre];
	wzmp_ref_temp.py = Tra_ZMP.y[k_pre];
	gp_ref_temp = Trans2_Gframe_Cur(wzmp_ref_temp, k_pre);
	// re
	GP_ref.x = gp_ref_temp.px;
	GP_ref.y = gp_ref_temp.py;
	// re

	delta_com_temp = ZMP_Controller(k_IPSD, G_ob, K_con, GP_rel, GPd_CP); // <- ZMP CONTROLLER 
	Delta_COM.x = (filter_COM_T[0] / CONTROL_T * Delta_COM.x + delta_com_temp.x) / (1 + filter_COM_T[0] / CONTROL_T); // com filter
	Delta_COM.y = (filter_COM_T[1] / CONTROL_T * Delta_COM.y + delta_com_temp.y) / (1 + filter_COM_T[1] / CONTROL_T); // com filter
	HUBO_foot_com = Compliance_HUBO(HUBO_foot_com, Pd_CP, kZ, kT, filter_Fz_T, k_pre); // <- COMPLIANCE CONTROLLER

}



// before IK
//HUBO19(K_Preview_Con);
//#ifdef USE_UNEVEN_TRAILBLAZER
//Tra_COM.x[K_Preview_Con] += 1.0 * Delta_COM.x;
//Tra_COM.y[K_Preview_Con] += 1.0 * Delta_COM.y;
//#endif
//#ifdef USE_HUBO_COMPLIANCE
//Tra_LAnkle.z[K_Preview_Con] += 1.0 * HUBO_foot_com.Lfoot.z.e;
//Tra_RAnkle.z[K_Preview_Con] += 1.0 * HUBO_foot_com.Rfoot.z.e;
//#endif

// after IK
//#ifdef USE_HUBO_COMPLIANCE	
//Ref_Leg_Joint[1][5] += HUBO_foot_com.Rfoot.pit.e;
//Ref_Leg_Joint[1][6] += HUBO_foot_com.Rfoot.rol.e;
//
//Ref_Leg_Joint[2][5] += HUBO_foot_com.Lfoot.pit.e;
//Ref_Leg_Joint[2][6] += HUBO_foot_com.Lfoot.rol.e;
//#endif