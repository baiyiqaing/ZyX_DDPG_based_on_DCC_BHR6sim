#include "CoM_Stabilizer.h"

double mic_rot = 1.0; // micro the rotation motion
enum Support_Leg { DOUBLE_LEG_SP, RLEG_SP, LLEG_SP };
extern int N_tcom;
extern double GQ_Roll;
extern double GQ_Pitch;
extern double roll_body;
extern double pitch_body;
extern double S[3][5000];
double Body_VX = 0.0;
double Body_VY = 0.0;

void Cal_ICP_ref(PreCon_Tra Tra_COM, PreCon_Tra Tra_VCOM, int mod)
{
	Position Tra_COM_temp;
	Position Tra_GCOM_temp;
	double Omega;
	Omega = sqrt(GRAVITY / H_zc);

	for (int i = 0; i < T_com / CONTROL_T; i++)
	{
		Tra_COM_temp.px = Tra_COM.x[i];
		Tra_COM_temp.py = Tra_COM.y[i];
		Tra_GCOM_temp = Trans2_Gframe_Cur(Tra_COM_temp, i);
		if (mod == 'G')
		{
			Tra_GICP.x[i] = Tra_GCOM_temp.px + 1 / Omega * Tra_VCOM.x[i];
			Tra_GICP.y[i] = Tra_GCOM_temp.py + 1 / Omega * Tra_VCOM.y[i];
		}
		else if (mod == 'W')
		{
			Tra_GICP.x[i] = Tra_COM_temp.px + 1 / Omega * Tra_VCOM.x[i];
			Tra_GICP.y[i] = Tra_COM_temp.py + 1 / Omega * Tra_VCOM.y[i];
		}
	}

}

Horizontal_Current CoM_controller(State_DelayedLIPM state_ref, State_DelayedLIPM state_rel, double k[3], int k_pre)
{
	Horizontal_Current pd_star;

	double k1 = k[0]; 
	double k2 = k[1]; 
	double k3 = k[2]; 

	pd_star.x = Tra_ZMP.x[k_pre] + k1 * (state_rel.CoM_Posi.x - state_ref.CoM_Posi.x) + k2 * (state_rel.CoM_Velo.x - state_ref.CoM_Velo.x) + k3 * (state_rel.ZMP.x - state_ref.ZMP.x);
	pd_star.y = Tra_ZMP.y[k_pre] + k1 * (state_rel.CoM_Posi.y - state_ref.CoM_Posi.y) + k2 * (state_rel.CoM_Velo.y - state_ref.CoM_Velo.y) + k3 * (state_rel.ZMP.y - state_ref.ZMP.y);

	if (k_pre < 50)
	{
		pd_star.x = 0.0;
		pd_star.y = 0.0;
	}
	else if (k_pre >= T_walk / CONTROL_T)
	{
		pd_star.x = 0.0;
		pd_star.y = Tra_ZMP.y[N_tcom];
	}

	return pd_star;
}

Sup_Polygon Get_SupPoly(int k_pre)
{
	Sup_Polygon sup_poly;
	if (Signal_SupportLeg[k_pre] == LLEG_SP)
	{
		sup_poly.left = -0.5 * ANKLE_WIDTH - 0.07;
		sup_poly.righ = -0.5 * ANKLE_WIDTH + 0.05;
		sup_poly.forw = Tra_LAnkle.y[k_pre] + 0.15;
		sup_poly.back = Tra_LAnkle.y[k_pre] - 0.09;
	}
	else if (Signal_SupportLeg[k_pre] == RLEG_SP)
	{
		sup_poly.left = 0.5 * ANKLE_WIDTH - 0.05;
		sup_poly.righ = 0.5 * ANKLE_WIDTH + 0.07;
		sup_poly.forw = Tra_RAnkle.y[k_pre] + 0.15;
		sup_poly.back = Tra_RAnkle.y[k_pre] - 0.09;
	}
	else if (Signal_SupportLeg[k_pre] == DOUBLE_LEG_SP)
	{
		sup_poly.left = -0.5 * ANKLE_WIDTH - 0.07;
		sup_poly.righ =  0.5 * ANKLE_WIDTH + 0.07;
		if (Tra_LAnkle.y[k_pre] >= Tra_RAnkle.y[k_pre])
		{
			sup_poly.forw = Tra_LAnkle.y[k_pre] + 0.15;
			sup_poly.back = Tra_RAnkle.y[k_pre] - 0.09;
		}
		else if (Tra_LAnkle.y[k_pre] < Tra_RAnkle.y[k_pre])
		{
			sup_poly.forw = Tra_RAnkle.y[k_pre] + 0.15;
			sup_poly.back = Tra_LAnkle.y[k_pre] - 0.09;
		}
	}

	if (k_pre > (T_walk / CONTROL_T - 10))
	{
		sup_poly.left = -0.5 * ANKLE_WIDTH - 0.07;
		sup_poly.righ =  0.5 * ANKLE_WIDTH + 0.07;
		sup_poly.forw = Tra_RAnkle.y[N_tcom] + 0.15;
		sup_poly.back = Tra_RAnkle.y[N_tcom] - 0.09;
	}

	return sup_poly;
}

Horizontal_Current Pd_Exceed_Distributor(Sup_Polygon sup_poly, double Fz)
{
	Horizontal_Current Mmodel;
	// x
	if (Pd_Star.x > sup_poly.righ)
	{
		Mmodel.y = -(Pd_Star.x - sup_poly.righ) * Fz;
		Pd_Star.x = sup_poly.righ;
	}
	else if (Pd_Star.x < sup_poly.left)
	{
		Mmodel.y = -(Pd_Star.x - sup_poly.left) * Fz;
		Pd_Star.x = sup_poly.left;
	}
	else
	{
		Mmodel.y = 0.0;
	}
	if (Mmodel.y >  80.0) Mmodel.y =  80.0;
	if (Mmodel.y < -80.0) Mmodel.y = -80.0;
	// y
	if (Pd_Star.y > sup_poly.forw)
	{
		Mmodel.x = (Pd_Star.y - sup_poly.forw) * Fz;
		Pd_Star.y = sup_poly.forw;
	}
	else if (Pd_Star.y < sup_poly.back)
	{
		Mmodel.x = (Pd_Star.y - sup_poly.back) * Fz;
		Pd_Star.y = sup_poly.back;
	}
	else
	{
		Mmodel.x = 0.0;
	}
	if (Mmodel.x >  80.0) Mmodel.x = 80.0;
	if (Mmodel.x < -80.0) Mmodel.x = -80.0;

	return Mmodel;
}

Model_ZMP_Con_Value Model_ZMP_Con(Model_ZMP_Con_Value md_posrot, Horizontal_Current Mmodel, double k_hori[2], double k_rot[2], double kx_r[2], double p_friction, double H_robot, double I_robot, double Fz)
{
	double k_roll  = kx_r[0];
	double k_pitch = kx_r[1];
	md_posrot.dde_max = 10; // p_friction * Fz / m_robot;

	md_posrot.ddx = - Mmodel.y / m_robot / H_robot; 
	md_posrot.ddy = Mmodel.x / m_robot / H_robot;

	// x
	if (abs(md_posrot.ddx) > abs(md_posrot.dde_max))
	{
		md_posrot.ddx = md_posrot.ddx / abs(md_posrot.ddx) * abs(md_posrot.dde_max) - k_hori[0] * md_posrot.x - k_hori[1] * md_posrot.dx;
		md_posrot.ddroll = -(Mmodel.y + (md_posrot.ddx / abs(md_posrot.ddx) * abs(md_posrot.dde_max)) * m_robot * H_robot) / I_robot - k_rot[0] * md_posrot.roll - k_rot[1] * md_posrot.droll;
	}
	else
	{
		md_posrot.ddx = md_posrot.ddx - k_hori[0] * md_posrot.x - k_hori[1] * md_posrot.dx; // if Mmodle == 0, just a damping recover to zero  
		md_posrot.ddroll = -k_rot[0] * md_posrot.roll - k_rot[1] * md_posrot.droll;			// if Mmodle < Mmax, just a damping recover to zero
	}
	md_posrot.dx = md_posrot.dx + md_posrot.ddx * CONTROL_T;
	md_posrot.x  = md_posrot.x  + md_posrot.dx  * CONTROL_T;
	if (md_posrot.x >  0.05) md_posrot.x =  0.05;
	if (md_posrot.x < -0.05) md_posrot.x = -0.05;
	md_posrot.droll = md_posrot.droll + md_posrot.ddroll * CONTROL_T;
	md_posrot.roll  = md_posrot.roll  + md_posrot.droll  * CONTROL_T;
	if (md_posrot.roll >  10 / 57.3) md_posrot.roll =  10 / 57.3;
	if (md_posrot.roll < -10 / 57.3) md_posrot.roll = -10 / 57.3;
	md_posrot.x_roll = -k_roll * md_posrot.roll;
	// x

	// y
	if (abs(md_posrot.ddy) > abs(md_posrot.dde_max))
	{
		md_posrot.ddy = md_posrot.ddy / abs(md_posrot.ddy) * abs(md_posrot.dde_max) - k_hori[0] * md_posrot.y - k_hori[1] * md_posrot.dy;
		md_posrot.ddpitch = -(Mmodel.x - (md_posrot.ddy / abs(md_posrot.ddy) * abs(md_posrot.dde_max)) * m_robot * H_robot) / I_robot - k_rot[0] * md_posrot.pitch - k_rot[1] * md_posrot.dpitch;
	}
	else
	{
		md_posrot.ddy = md_posrot.ddy - k_hori[0] * md_posrot.y - k_hori[1] * md_posrot.dy; // if Mmodle == 0, just a damping recover to zero  
		md_posrot.ddpitch = -k_rot[0] * md_posrot.pitch - k_rot[1] * md_posrot.dpitch;		// if Mmodle < Mmax, just a damping recover to zero
	}
	md_posrot.dy = md_posrot.dy + md_posrot.ddy * CONTROL_T;
	md_posrot.y  = md_posrot.y  + md_posrot.dy  * CONTROL_T;
	if (md_posrot.y >  0.05) md_posrot.y =  0.05;
	if (md_posrot.y < -0.05) md_posrot.y = -0.05;
	md_posrot.dpitch = md_posrot.dpitch + md_posrot.ddpitch * CONTROL_T;
	md_posrot.pitch  = md_posrot.pitch  + md_posrot.dpitch  * CONTROL_T;
	if (md_posrot.pitch >  15 / 57.3) md_posrot.pitch =  15 / 57.3;
	if (md_posrot.pitch < -15 / 57.3) md_posrot.pitch = -15 / 57.3;
	md_posrot.y_pitch = k_pitch * md_posrot.pitch;
	// y

	return md_posrot;

}

Feet_Step_Adjust Model_Step_Adjust(Feet_Step_Adjust step_sdjust, Model_ZMP_Con_Value md_posrot, double k[4], double roll, double pitch, int k_pre)
{
	double om = 3.14;
	double delta_zL;
	double delta_zR;

	double kf = k[0]; // feed back gain for virtual force to follow step delta_L
	double kp = k[1]; // VMM paras portional
	double kd = k[2]; // VMM paras damping
	double km = k[3]; // VMM paras mass
	double kd_L = kd;
	double kd_R = kd;

	step_sdjust.delta_L_x = md_posrot.x + md_posrot.dx / om;
	step_sdjust.delta_L_y = md_posrot.y + md_posrot.dy / om; 

	// cal F_foot
	if (Signal_SupportLeg[k_pre] == RLEG_SP)
	{

		delta_zL = -ankle_width * sin(roll) - ((Tra_LAnkle.y[k_pre] + step_sdjust.Lfoot.F_y) - (Tra_RAnkle.y[k_pre] + step_sdjust.Rfoot.F_y)) * sin(pitch);
		delta_zR = 0.0;
		if (delta_zL >  0.15) delta_zL =  0.15;
		if (delta_zL < -0.03) delta_zL = -0.03;
		if (delta_zR >  0.15) delta_zR =  0.15;
		if (delta_zR < -0.03) delta_zR = -0.03;

		step_sdjust.Lfoot.F_x = kf * (step_sdjust.delta_L_x - step_sdjust.Lfoot.e_x);
		step_sdjust.Lfoot.F_y = kf * (step_sdjust.delta_L_y - step_sdjust.Lfoot.e_y);
		step_sdjust.Lfoot.F_z = kf * (delta_zL - step_sdjust.Lfoot.e_z);
		step_sdjust.Rfoot.F_x = kf * (0.0 - step_sdjust.Rfoot.e_x);
		step_sdjust.Rfoot.F_y = kf * (0.0 - step_sdjust.Rfoot.e_y);
		step_sdjust.Rfoot.F_z = kf * (delta_zR - step_sdjust.Rfoot.e_z);
	}
	else if (Signal_SupportLeg[k_pre] == LLEG_SP)
	{
		delta_zL = 0.0;
		delta_zR = ankle_width * sin(roll) - ((Tra_RAnkle.y[k_pre] + step_sdjust.Rfoot.F_y) - (Tra_LAnkle.y[k_pre] + step_sdjust.Lfoot.F_y)) * sin(pitch);
		if (delta_zL >  0.15) delta_zL = 0.15;
		if (delta_zL < -0.03) delta_zL = -0.03;
		if (delta_zR >  0.15) delta_zR = 0.15;
		if (delta_zR < -0.03) delta_zR = -0.03;

		step_sdjust.Lfoot.F_x = kf * (0.0 - step_sdjust.Lfoot.e_x);
		step_sdjust.Lfoot.F_y = kf * (0.0 - step_sdjust.Lfoot.e_y);
		step_sdjust.Lfoot.F_z = kf * (delta_zL - step_sdjust.Lfoot.e_z);
		step_sdjust.Rfoot.F_x = kf * (step_sdjust.delta_L_x - step_sdjust.Rfoot.e_x);
		step_sdjust.Rfoot.F_y = kf * (step_sdjust.delta_L_y - step_sdjust.Rfoot.e_y);
		step_sdjust.Rfoot.F_z = kf * (delta_zR - step_sdjust.Rfoot.e_z);
	}
	else if (Signal_SupportLeg[k_pre] == DOUBLE_LEG_SP)
	{
		delta_zL = 0.0;
		delta_zR = 0.0;

		step_sdjust.Lfoot.F_x = 0.0;
		step_sdjust.Lfoot.F_y = 0.0;
		step_sdjust.Lfoot.F_z = kf * (0.0 - step_sdjust.Lfoot.e_z);
		step_sdjust.Rfoot.F_x = 0.0;
		step_sdjust.Rfoot.F_y = 0.0;
		step_sdjust.Rfoot.F_z = kf * (0.0 - step_sdjust.Rfoot.e_z);
	}
	// cal F_foot


	//if (Signal_SupportLeg[k_pre] == RLEG_SP)
	//{
	//	kd_R = kd * 1.0;
	//}
	//else if (Signal_SupportLeg[k_pre] == LLEG_SP)
	//{
	//	kd_L = kd * 1.0;
	//}
	//else if (Signal_SupportLeg[k_pre] == DOUBLE_LEG_SP)
	//{
	//	kd_L = kd * 1.0;
	//	kd_R = kd * 1.0;
	//}
	//else
	//{ }
	// x update
	// L
	step_sdjust.Lfoot.dde_x = (step_sdjust.Lfoot.F_x - kp * step_sdjust.Lfoot.e_x - kd_L * step_sdjust.Lfoot.de_x) / km;
	step_sdjust.Lfoot.de_x = step_sdjust.Lfoot.de_x + step_sdjust.Lfoot.dde_x * CONTROL_T;
	if (step_sdjust.Lfoot.de_x >  2.5) step_sdjust.Lfoot.de_x =  2.5;
	if (step_sdjust.Lfoot.de_x < -2.5) step_sdjust.Lfoot.de_x = -2.5;
	step_sdjust.Lfoot.e_x = step_sdjust.Lfoot.e_x + step_sdjust.Lfoot.de_x * CONTROL_T;
	if (step_sdjust.Lfoot.e_x >  0.1) step_sdjust.Lfoot.e_x =  0.1;
	if (step_sdjust.Lfoot.e_x < -0.1) step_sdjust.Lfoot.e_x = -0.1;
	// L
	// R
	step_sdjust.Rfoot.dde_x = (step_sdjust.Rfoot.F_x - kp * step_sdjust.Rfoot.e_x - kd_R * step_sdjust.Rfoot.de_x) / km;
	step_sdjust.Rfoot.de_x = step_sdjust.Rfoot.de_x + step_sdjust.Rfoot.dde_x * CONTROL_T;
	if (step_sdjust.Rfoot.de_x >  2.5) step_sdjust.Rfoot.de_x = 2.5;
	if (step_sdjust.Rfoot.de_x < -2.5) step_sdjust.Rfoot.de_x = -2.5;
	step_sdjust.Rfoot.e_x = step_sdjust.Rfoot.e_x + step_sdjust.Rfoot.de_x * CONTROL_T;
	if (step_sdjust.Rfoot.e_x >  0.1) step_sdjust.Rfoot.e_x = 0.1;
	if (step_sdjust.Rfoot.e_x < -0.1) step_sdjust.Rfoot.e_x = -0.1;
	// R
	// x update

	// y update
	// L
	step_sdjust.Lfoot.dde_y = (step_sdjust.Lfoot.F_y - kp * step_sdjust.Lfoot.e_y - kd_L * step_sdjust.Lfoot.de_y) / km;
	step_sdjust.Lfoot.de_y = step_sdjust.Lfoot.de_y + step_sdjust.Lfoot.dde_y * CONTROL_T;
	if (step_sdjust.Lfoot.de_y >  2.5) step_sdjust.Lfoot.de_y = 2.5;
	if (step_sdjust.Lfoot.de_y < -2.5) step_sdjust.Lfoot.de_y = -2.5;
	step_sdjust.Lfoot.e_y = step_sdjust.Lfoot.e_y + step_sdjust.Lfoot.de_y * CONTROL_T;
	if (step_sdjust.Lfoot.e_y >  0.1) step_sdjust.Lfoot.e_y = 0.1;
	if (step_sdjust.Lfoot.e_y < -0.1) step_sdjust.Lfoot.e_y = -0.1;
	// L
	// R
	step_sdjust.Rfoot.dde_y = (step_sdjust.Rfoot.F_y - kp * step_sdjust.Rfoot.e_y - kd_R * step_sdjust.Rfoot.de_y) / km;
	step_sdjust.Rfoot.de_y = step_sdjust.Rfoot.de_y + step_sdjust.Rfoot.dde_y * CONTROL_T;
	if (step_sdjust.Rfoot.de_y >  2.5) step_sdjust.Rfoot.de_y = 2.5;
	if (step_sdjust.Rfoot.de_y < -2.5) step_sdjust.Rfoot.de_y = -2.5;
	step_sdjust.Rfoot.e_y = step_sdjust.Rfoot.e_y + step_sdjust.Rfoot.de_y * CONTROL_T;
	if (step_sdjust.Rfoot.e_y >  0.1) step_sdjust.Rfoot.e_y = 0.1;
	if (step_sdjust.Rfoot.e_y < -0.1) step_sdjust.Rfoot.e_y = -0.1;
	// R
	// y update

	// z update
	// L
	step_sdjust.Lfoot.dde_z = (step_sdjust.Lfoot.F_z - kp * step_sdjust.Lfoot.e_z - 3 * kd_L * step_sdjust.Lfoot.de_z) / km;
	step_sdjust.Lfoot.de_z = step_sdjust.Lfoot.de_z + step_sdjust.Lfoot.dde_z * CONTROL_T;
	if (step_sdjust.Lfoot.de_z >  2.5) step_sdjust.Lfoot.de_z = 2.5;
	if (step_sdjust.Lfoot.de_z < -2.5) step_sdjust.Lfoot.de_z = -2.5;
	step_sdjust.Lfoot.e_z = step_sdjust.Lfoot.e_z + step_sdjust.Lfoot.de_z * CONTROL_T;
	if (step_sdjust.Lfoot.e_z >  0.1) step_sdjust.Lfoot.e_z = 0.1;
	if (step_sdjust.Lfoot.e_z < -0.1) step_sdjust.Lfoot.e_z = -0.1;
	// L
	// R
	step_sdjust.Rfoot.dde_z = (step_sdjust.Rfoot.F_z - kp * step_sdjust.Rfoot.e_z - 3 * kd_R * step_sdjust.Rfoot.de_z) / km;
	step_sdjust.Rfoot.de_z = step_sdjust.Rfoot.de_z + step_sdjust.Rfoot.dde_z * CONTROL_T;
	if (step_sdjust.Rfoot.de_z >  2.5) step_sdjust.Rfoot.de_z = 2.5;
	if (step_sdjust.Rfoot.de_z < -2.5) step_sdjust.Rfoot.de_z = -2.5;
	step_sdjust.Rfoot.e_z = step_sdjust.Rfoot.e_z + step_sdjust.Rfoot.de_z * CONTROL_T;
	if (step_sdjust.Rfoot.e_z >  0.1) step_sdjust.Rfoot.e_z = 0.1;
	if (step_sdjust.Rfoot.e_z < -0.1) step_sdjust.Rfoot.e_z = -0.1;
	// R
	// z update

	return step_sdjust;
}

Position Trans2_Gframe_Cur(Position posi_in, int k_pre)
{
	Position posi_out;
	if (Signal_SupportLeg[k_pre] == RLEG_SP)
	{
		posi_out.px = posi_in.px - Tra_RAnkle.x[k_pre];
		posi_out.py = posi_in.py - Tra_RAnkle.y[k_pre];
	}
	else if (Signal_SupportLeg[k_pre] == LLEG_SP)
	{
		posi_out.px = posi_in.px - Tra_LAnkle.x[k_pre];
		posi_out.py = posi_in.py - Tra_LAnkle.y[k_pre];
	}
	else
	{
		posi_out.px = posi_in.px - 0.5 * (Tra_RAnkle.x[k_pre] + Tra_LAnkle.x[k_pre]);
		posi_out.py = posi_in.py - 0.5 * (Tra_RAnkle.y[k_pre] + Tra_LAnkle.y[k_pre]);
	}

	return posi_out;
}

Feet_FT Pd_TauFz(Horizontal_Current pd_star, int k_pre)
{
	int Sup_Leg = Signal_SupportLeg[k_pre];
	double alpha;
	double tau0x;
	double tau0y;
	Feet_FT TauFz;
	if (Signal_SupportLeg[k_pre] == LLEG_SP)
	{
		alpha = 0.0;

		TauFz.Rfoot.fz = alpha*m_robot*GRAVITY;
		TauFz.Lfoot.fz = (1 - alpha)*m_robot*GRAVITY;

		TauFz.Lfoot.ty = -(pd_star.x + 0.5*ankle_width) * m_robot*GRAVITY;
		TauFz.Lfoot.tx = (pd_star.y - Tra_LAnkle.y[k_pre]) * m_robot*GRAVITY;

		TauFz.Rfoot.ty = 0.0;
		TauFz.Rfoot.tx = 0.0;
	}
	else if (Signal_SupportLeg[k_pre] == RLEG_SP)
	{
		alpha = 1.0;

		TauFz.Rfoot.fz = alpha*m_robot*GRAVITY;
		TauFz.Lfoot.fz = (1 - alpha)*m_robot*GRAVITY;

		TauFz.Lfoot.ty = 0.0;
		TauFz.Lfoot.tx = 0.0;

		TauFz.Rfoot.ty = (0.5*ankle_width - pd_star.x) * m_robot*GRAVITY;
		TauFz.Rfoot.tx = (pd_star.y - Tra_RAnkle.y[k_pre]) * m_robot*GRAVITY;
	}
	else if (Signal_SupportLeg[k_pre] == DOUBLE_LEG_SP)
	{
		alpha = (pd_star.x + 0.5*FOOT_DISTANCE) / FOOT_DISTANCE;
		if (alpha > 1) alpha = 1;
		else if (alpha < 0) alpha = 0;

		TauFz.Rfoot.fz = alpha*m_robot*GRAVITY;
		TauFz.Lfoot.fz = (1 - alpha)*m_robot*GRAVITY;

		tau0x = (Tra_RAnkle.y[k_pre] - pd_star.y)*TauFz.Rfoot.fz + (Tra_LAnkle.y[k_pre] - pd_star.y)*TauFz.Lfoot.fz;
		tau0y = (pd_star.x + 0.5*ankle_width)*TauFz.Lfoot.fz - (0.5*ankle_width - pd_star.x)*TauFz.Rfoot.fz;

		TauFz.Rfoot.tx = -alpha*tau0x;
		TauFz.Lfoot.tx = -(1 - alpha)*tau0x;

		if (tau0y > 0)
		{
			TauFz.Rfoot.ty = -tau0y;
			TauFz.Lfoot.ty = 0.0;
		}
		else if (tau0y < 0)
		{
			TauFz.Rfoot.ty = 0.0;
			TauFz.Lfoot.ty = -tau0y;
		}
		else
		{
			TauFz.Rfoot.ty = 0.0;
			TauFz.Lfoot.ty = 0.0;
		}
	}
	else
	{
		printf("%d false in cal_desired_footft\n", Sup_Leg);
	}

	if (k_pre >= (T_walk - T_step) / CONTROL_T)
	{
		TauFz.Rfoot.tx = 0.0;
		TauFz.Lfoot.tx = 0.0;
		TauFz.Rfoot.ty = 0.0;
		TauFz.Lfoot.ty = 0.0;
		TauFz.Rfoot.fz = 0.5*m_robot*GRAVITY;
		TauFz.Lfoot.fz = 0.5*m_robot*GRAVITY;
	}

	return TauFz;
}

com_state Damping_DT(double D, double T, double Fd, double F, com_state damp) // global variable damp is required
{
	damp.de = 1 / D*(F - Fd) - 1 / T*damp.el;
	damp.e = damp.el + CONTROL_T*damp.de;
	damp.el = damp.e;

	return damp;
}

com_state Damping_PD(double P, double D, double Fd, double F, com_state damp) // global variable damp is required
{
	damp.e = (F - Fd + D / CONTROL_T * damp.el) / (P + D / CONTROL_T);
	damp.el = damp.e;

	return damp;
}

Feet_comp GRF_Con(Horizontal_Current pd_star, double k_damp[6], int k_pre)
{
	Feet_comp feetcomp;
	double D_pit = k_damp[0];
	double T_pit = k_damp[1];
	double D_rol = k_damp[2];
	double T_rol = k_damp[3];
	double D_ctl = k_damp[4];
	double T_ctl = k_damp[5];

	TauFz = Pd_TauFz(pd_star, k_pre);
	// test grfcon
	TauFz.Lfoot.ty = 0.0;
	TauFz.Lfoot.tx = 0.0;
	TauFz.Rfoot.ty = 0.0;
	TauFz.Rfoot.tx = 0.0;
	TauFz.Rfoot.fz = 0.5*m_robot*GRAVITY;
	TauFz.Lfoot.fz = 0.5*m_robot*GRAVITY;
	// test grfcon
	Lfoot_Rol = Damping_DT(D_rol, T_rol, TauFz.Lfoot.ty, F_LFoot.ty, Lfoot_Rol);
	Lfoot_Pit = Damping_DT(D_pit, T_pit, TauFz.Lfoot.tx, F_LFoot.tx, Lfoot_Pit);
	Rfoot_Rol = Damping_DT(D_rol, T_rol, TauFz.Rfoot.ty, F_RFoot.ty, Rfoot_Rol);
	Rfoot_Pit = Damping_DT(D_pit, T_pit, TauFz.Rfoot.tx, F_RFoot.tx, Rfoot_Pit);
	Zfoot_Ctl = Damping_DT(D_ctl, T_ctl, (TauFz.Lfoot.fz - TauFz.Rfoot.fz), (F_LFoot.fz - F_RFoot.fz), Zfoot_Ctl);
	Lfoot_Ctl.e = 0.5 * Zfoot_Ctl.e;
	Rfoot_Ctl.e = -0.5 * Zfoot_Ctl.e;

	if (Lfoot_Pit.e >  0.5) Lfoot_Pit.e = 0.5;
	if (Rfoot_Pit.e >  0.5) Rfoot_Pit.e = 0.5;
	if (Lfoot_Pit.e < -0.5) Lfoot_Pit.e = -0.5;
	if (Rfoot_Pit.e < -0.5) Rfoot_Pit.e = -0.5;
	if (Lfoot_Rol.e >  0.5) Lfoot_Rol.e = 0.5;
	if (Rfoot_Rol.e >  0.5) Rfoot_Rol.e = 0.5;
	if (Lfoot_Rol.e < -0.5) Lfoot_Rol.e = -0.5;
	if (Rfoot_Rol.e < -0.5) Rfoot_Rol.e = -0.5;
	if (Lfoot_Ctl.e >  0.05)Lfoot_Ctl.e = 0.05;
	if (Rfoot_Ctl.e >  0.05)Rfoot_Ctl.e = 0.05;
	if (Lfoot_Ctl.e < -0.05)Lfoot_Ctl.e = -0.05;
	if (Rfoot_Ctl.e < -0.05)Rfoot_Ctl.e = -0.05;

	feetcomp.Lfoot.delta_pit = Lfoot_Pit.e;
	feetcomp.Rfoot.delta_pit = Rfoot_Pit.e;
	feetcomp.Lfoot.delta_rol = Lfoot_Rol.e;
	feetcomp.Rfoot.delta_rol = Rfoot_Rol.e;
	feetcomp.Lfoot.delta_z = Lfoot_Ctl.e;
	feetcomp.Rfoot.delta_z = Rfoot_Ctl.e;

	return feetcomp;
}

void Chest_Posture_Con(double pitch_d, double roll_d, double pitch, double roll)
{
	double kc_pitch = 7.0;
	double Tc_pitch = 60.0;
	double kc_roll  = 7.0;
	double Tc_roll  = 60.0;

	chest_pitch.de = kc_pitch*(pitch_d - pitch) - 1 / Tc_pitch*chest_pitch.el;
	chest_pitch.e = chest_pitch.el + CONTROL_T*chest_pitch.de;
	chest_pitch.el = chest_pitch.e;

	chest_roll.de = kc_roll*(roll_d - roll) - 1 / Tc_roll*chest_roll.el;
	chest_roll.e = chest_roll.el + CONTROL_T*chest_roll.de;
	chest_roll.el = chest_roll.e;

	if (chest_pitch.e >  10 / 57.3)chest_pitch.e = 10 / 57.3;
	if (chest_pitch.e < -10 / 57.3)chest_pitch.e = -10 / 57.3;
	if (chest_roll.e  >  10 / 57.3)chest_roll.e = 10 / 57.3;
	if (chest_roll.e  < -10 / 57.3)chest_roll.e = -10 / 57.3;
}

void CoM_Sta10(int k_pre) // state_rel is required
{
	State_DelayedLIPM state_ref;
	State_DelayedLIPM state_rel;

	Position COM_rel;
	Position COM_ref;
	Position ZMP_rel;
	Position ZMP_ref;

	double Fz;
	Fz = F_LFoot.fz + F_RFoot.fz;

	COM_ref.px = Tra_COM.x[k_pre];
	COM_ref.py = Tra_COM.y[k_pre];
	ZMP_ref.px = Tra_ZMP.x[k_pre] - COM_ref.px;
	ZMP_ref.py = Tra_ZMP.y[k_pre] - COM_ref.py;

	COM_ref = Trans2_Gframe_Cur(COM_ref, k_pre);

	COM_rel.px = COM_ref.px + H_zc * sin(GQ_Roll);
	COM_rel.py = COM_ref.py - H_zc * sin(-GQ_Pitch);
	ZMP_rel.px = P_ZMPRel_B.px;
	ZMP_rel.py = P_ZMPRel_B.py;

	// re
	Tra_GCOM.x[k_pre] = COM_ref.px;
	Tra_GCOM.y[k_pre] = COM_ref.py;
	Tra_VCOM.x[k_pre] = Tra_VCOM.x[k_pre];
	Tra_VCOM.y[k_pre] = Tra_VCOM.y[k_pre];
	Tra_BZMP.x[k_pre] = ZMP_ref.px;
	Tra_BZMP.y[k_pre] = ZMP_ref.py;

	Tra_GCOM_rel.x[k_pre] = COM_rel.px;
	Tra_GCOM_rel.y[k_pre] = COM_rel.py;
	Tra_VCOM_rel.x[k_pre] = Body_VX;
	Tra_VCOM_rel.y[k_pre] = Body_VY;
	Tra_BZMP_rel.x[k_pre] = ZMP_rel.px;
	Tra_BZMP_rel.y[k_pre] = ZMP_rel.py;
	// re

	state_ref.CoM_Posi.x = COM_ref.px;
	state_ref.CoM_Posi.y = COM_ref.py;
	state_ref.CoM_Velo.x = Tra_VCOM.x[k_pre];
	state_ref.CoM_Velo.y = Tra_VCOM.y[k_pre];
	state_ref.ZMP.x = ZMP_ref.px;
	state_ref.ZMP.y = ZMP_ref.py;

	state_rel.CoM_Posi.x = COM_rel.px;
	state_rel.CoM_Posi.y = COM_rel.py;
	state_rel.CoM_Velo.x = Body_VX;
	state_rel.CoM_Velo.y = Body_VY;
	state_rel.ZMP.x = ZMP_ref.px;
	state_rel.ZMP.y = ZMP_ref.py;

	double k_pd_s[3] = { 1.5, 0.3, 0.02 };							// fb gains for com regulator
	double k_hori[2] = { 10.0, 5.0 };								// recover gains for model zmp horizontal { 10.0, 5.0 }
	double k_rot[2]  = { 10.0, 5.0 };								// recover gains for model zmp rotational { 10.0, 5.0 }
	double kp_r[2]   = { 0.2795, 0.3057 };							// upper body position bias from rotation k_roll, k_pitch
	double k_step[4] = { 1500.0, 50.0, 120.0, 2.5 };				// step adjust parameters: kf, kp, kd, km
	double k_damp[6] = { 260.0, 0.25, 250.0, 0.28, 7000.0, 0.8 };	// GRF control parameters: D_pit, T_pit, D_rol, T_rol, D_ctl, T_ctl { 300.0, 0.25, 350.0, 0.28, 7000.0, 0.8 }
	double p_friction = 0.2;                                        // friction coefficient

	Pd_Star = CoM_controller(state_ref, state_rel, k_pd_s, k_pre);
	Sup_Poly = Get_SupPoly(k_pre); 
#ifdef USE_MODEL_ZMP
	M_model = Pd_Exceed_Distributor(Sup_Poly, Fz);																	// direction of Mmodel from pd* & pd* exceed
	Md_ZMP_PosRot = Model_ZMP_Con(Md_ZMP_PosRot, M_model, k_hori, k_rot, kp_r, p_friction, H_ROBOT, I_ROBOT, Fz);	// direction of ddx ddtheta from Mmodel & friction exceed
	Step_Adjust = Model_Step_Adjust(Step_Adjust, Md_ZMP_PosRot, k_step, GQ_Roll, -GQ_Pitch, k_pre);
#endif
	FeetComp = GRF_Con(Pd_Star, k_damp, k_pre); 
}

void AutoTune_K_Rot(int k_pre)
{
	int n_ready = 50;
	double zmp_x_sum = 0.0;
	double zmp_y_sum = 0.0;

	if (k_pre >= n_ready && k_pre < n_ready + 500)
	{
		// preparing zmp_x_ori & zmp_y_ori
		Tune_Krot.zmp_x_re[k_pre - n_ready] = P_ZMPRel_B.px;
		Tune_Krot.zmp_y_re[k_pre - n_ready] = P_ZMPRel_B.py;
	}
	else if (k_pre == n_ready + 500)
	{
		for (int i = 0; i < 500; i++)
		{
			zmp_x_sum += Tune_Krot.zmp_x_re[i];
			zmp_y_sum += Tune_Krot.zmp_y_re[i];
		}
		Tune_Krot.zmp_x_ori = zmp_x_sum / 500.0;
		Tune_Krot.zmp_y_ori = zmp_y_sum / 500.0;
		// prepared zmp_x_ori &  zmp_y_ori

		Tune_Krot.zmp_x_re[0] = P_ZMPRel_B.px;
		Tune_Krot.zmp_y_re[0] = P_ZMPRel_B.py;
	}
	else if (k_pre > n_ready + 500)
	{
		// preparing zmp_x_ave & zmp_y_ave
		Tune_Krot.zmp_x_re[(k_pre - n_ready) % 500] = P_ZMPRel_B.px;
		Tune_Krot.zmp_y_re[(k_pre - n_ready) % 500] = P_ZMPRel_B.py;
		for (int i = 0; i < 500; i++)
		{
			zmp_x_sum += Tune_Krot.zmp_x_re[i];
			zmp_y_sum += Tune_Krot.zmp_y_re[i];
		}
		Tune_Krot.zmp_x_ave = zmp_x_sum / 500.0;
		Tune_Krot.zmp_y_ave = zmp_y_sum / 500.0;
		// prepared zmp_x_ave & zmp_y_ave

		// tune x
		if (Tune_Krot.x_ok == 0 && Tune_Krot.y_ok == 0)
		{
			TSpline_S_V_A(0.0, 0.0, 0.0, 0.0, 4.0 / 57.3, 5.0, 8.0 / 57.3, 0.0, 0.0, 10.0, CONTROL_T);
			roll_body = S[0][k_pre - n_ready - 501];
			if (k_pre > n_ready + 500 + 10 / CONTROL_T)
			{
				roll_body = 8.0 / 57.3;
				if (abs(Tune_Krot.zmp_x_ave - Tune_Krot.zmp_x_ori) > 0.001)
				{
					Tune_Krot.x_ok = 0;
					if (Tune_Krot.x_ok == 0) Tune_Krot.k_roll += 0.0001;
					Tra_COM.x[k_pre] = -Tune_Krot.k_roll * roll_body;
					if (Tra_COM.x[k_pre] > 0.06) Tune_Krot.x_ok = 1;
				}
				else if (abs(Tune_Krot.zmp_x_ave - Tune_Krot.zmp_x_ori) <= 0.001 && Tune_Krot.x_count < n_ready)
				{
					Tune_Krot.x_count++;
					Tra_COM.x[k_pre] = -Tune_Krot.k_roll * roll_body;
				}
				else if (abs(Tune_Krot.zmp_x_ave - Tune_Krot.zmp_x_ori) <= 0.001 && Tune_Krot.x_count == n_ready)
				{
					Tune_Krot.x_ok = 1;
					Tra_COM.x[k_pre] = -Tune_Krot.k_roll * roll_body;
				}
				else;
			}
		}
		else if (Tune_Krot.x_ok == 1 && Tune_Krot.y_ok == 0 && Tune_Krot.i_x_recover < 1250)
		{
			TSpline_S_V_A(8.0 / 57.3, 0.0, 0.0, 0.0, 4.0 / 57.3, 2.5, 0.0, 0.0, 0.0, 5.0, CONTROL_T);
			roll_body = S[0][Tune_Krot.i_x_recover++];
			Tra_COM.x[k_pre] = -Tune_Krot.k_roll * roll_body;
			Tune_Krot.x_count = k_pre; // recording k_pre for y
		} 
		// x ok !!!!!
		
		// tune y
		else if (Tune_Krot.x_ok == 1 && Tune_Krot.y_ok == 0 && Tune_Krot.i_x_recover == 1250)
		{
			roll_body = 0.0;
			TSpline_S_V_A(0.0, 0.0, 0.0, 0.0, -7.5 / 57.3, 5.0, -15.0 / 57.3, 0.0, 0.0, 10.0, CONTROL_T);
			pitch_body = S[0][k_pre - Tune_Krot.x_count];
			if (k_pre > Tune_Krot.x_count + 10 / CONTROL_T)
			{
				pitch_body = -15.0 / 57.3;
				if (abs(Tune_Krot.zmp_y_ave - Tune_Krot.zmp_y_ori) > 0.001)
				{
					Tune_Krot.y_ok = 0;
					if (Tune_Krot.y_ok == 0) Tune_Krot.k_pitch += 0.0001;
					Tra_COM.y[k_pre] = Tune_Krot.k_pitch * pitch_body;
					if (Tra_COM.y[k_pre] < -0.08) Tune_Krot.y_ok = 1;
				}
				else if (abs(Tune_Krot.zmp_y_ave - Tune_Krot.zmp_y_ori) <= 0.001 && Tune_Krot.y_count < n_ready)
				{
					Tune_Krot.y_count++;
					Tra_COM.y[k_pre] = Tune_Krot.k_pitch * pitch_body;
				}
				else if (abs(Tune_Krot.zmp_y_ave - Tune_Krot.zmp_y_ori) <= 0.001 && Tune_Krot.y_count == n_ready)
				{
					Tune_Krot.y_ok = 1;
					Tra_COM.y[k_pre] = Tune_Krot.k_pitch * pitch_body;
				}
				else;
			}
		}
		else if (Tune_Krot.x_ok == 1 && Tune_Krot.y_ok == 1 && Tune_Krot.i_y_recover < 1250)
		{
			TSpline_S_V_A(-15.0 / 57.3, 0.0, 0.0, 0.0, -7.5 / 57.3, 2.5, 0.0, 0.0, 0.0, 5.0, CONTROL_T);
			pitch_body = S[0][Tune_Krot.i_y_recover++];
			Tra_COM.y[k_pre] = Tune_Krot.k_pitch * pitch_body;
		}
		// y ok !!!!!
	}
	else;
		
}

// before IK
//#ifdef USE_COM_REGULATOR
//Tra_RAnkle.z[K_Preview_Con] += FeetComp.Rfoot.delta_z;
//Tra_LAnkle.z[K_Preview_Con] += FeetComp.Lfoot.delta_z;
//CoM_Sta10(K_Preview_Con);
//roll_body = Md_ZMP_PosRot.roll;
//pitch_body = Md_ZMP_PosRot.pitch;
//Tra_COM.x[K_Preview_Con] += (Md_ZMP_PosRot.x + Md_ZMP_PosRot.x_roll);
//Tra_COM.y[K_Preview_Con] += (Md_ZMP_PosRot.y + Md_ZMP_PosRot.y_pitch);
//#endif
//#ifdef USE_MODEL_ZMP
//Tra_RAnkle.x[K_Preview_Con] += Step_Adjust.Rfoot.e_x;
//Tra_RAnkle.y[K_Preview_Con] += Step_Adjust.Rfoot.e_y;
//Tra_RAnkle.z[K_Preview_Con] += Step_Adjust.Rfoot.e_z;
//Tra_LAnkle.x[K_Preview_Con] += Step_Adjust.Lfoot.e_x;
//Tra_LAnkle.y[K_Preview_Con] += Step_Adjust.Lfoot.e_y;
//Tra_LAnkle.z[K_Preview_Con] += Step_Adjust.Lfoot.e_z;
//#endif
//#ifdef TEST_CHEST_POSTURE_CON 
//Chest_Posture_Con(0.0 * GQ_omegaY, 0.0 * GQ_omegaX, -GQ_Pitch, GQ_Roll);
//roll_body += chest_roll.e;
//pitch_body += chest_pitch.e;
//#endif
//#ifdef TUNE_K_ROT
//AutoTune_K_Rot(K_Preview_Con);
//#endif

//
//// after IK
//#ifdef USE_COM_REGULATOR
//Ref_Leg_Joint[1][5] += FeetComp.Rfoot.delta_pit;
//Ref_Leg_Joint[1][6] += FeetComp.Rfoot.delta_rol;
//
//Ref_Leg_Joint[2][5] += FeetComp.Lfoot.delta_pit;
//Ref_Leg_Joint[2][6] += FeetComp.Lfoot.delta_rol;
//#endif