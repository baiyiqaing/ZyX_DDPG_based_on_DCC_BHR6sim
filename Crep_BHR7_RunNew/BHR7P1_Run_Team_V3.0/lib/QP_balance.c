#include "QP_balance.h"
#include "Tra_Generate.h"
#include "micro.h"

extern Position P_ZMPRef_B;
extern Position P_ZMPRel_B;

QPbalance_paras_conval QP_test;
NQPbalance_paras_conval NQP_test;
RE_NQP NQP_re;
QP_Rotational IMU_bias = { 0.0, 0.0};

// void Transfer_Leg(int sup_signal, double * q, double * qR_rel, double * qL_rel)
// {
	// if (sup_signal == RLEG_SP)
	// {
		// for (int i = 0; i < 6; i++)
		// {
			// q[i] = qR_rel[i];
			// q[i + 6] = qL_rel[i];
		// }
	// }
	// else if (sup_signal == LLEG_SP)
	// {
		// for (int i = 0; i < 6; i++)
		// {
			// q[i] = qL_rel[i];
			// q[i + 6] = qR_rel[i];
		// }
	// }
	// else
	// {
		// for (int i = 0; i < 6; i++)
		// {
			// q[i] = qR_rel[i];
			// q[i + 6] = qL_rel[i];
		// }
	// }
// }

NQPbalance_paras_conval cal_NQP_conval(NQPbalance_paras_conval in_val, double K_paras[26], double paras[8], double agl[4], double limit[4])
{
	NQPbalance_paras_conval out_val;
	QP_Rotational Md;
	//int map;
	
	double kMp_x		= K_paras[0];	     double kMp_y		 = K_paras[0 + 13];
	double kMd_x		= K_paras[1];	     double kMd_y		 = K_paras[1 + 13];
	double kp_x			= K_paras[2];	     double kp_y		 = K_paras[2 + 13];
	double kd_x			= K_paras[3];	     double kd_y		 = K_paras[3 + 13];
	double kf_x			= K_paras[4];	     double kf_y		 = K_paras[4 + 13];
	double micro_fall_x = K_paras[5];	     double micro_fall_y = K_paras[5 + 13];
	double k_tpc_x_phi	= K_paras[6];	     double k_tpc_y_phi  = K_paras[6 + 13];
	double kp_x_phi		= K_paras[7];	     double kp_y_phi	 = K_paras[7 + 13];
	double kd_x_phi		= K_paras[8];	     double kd_y_phi	 = K_paras[8 + 13];
	double k_tpc_x_e	= K_paras[9];	     double k_tpc_y_e	 = K_paras[9 + 13];
	double kp_x_e		= K_paras[10];       double kp_y_e		 = K_paras[10 + 13];
	double kd_x_e		= K_paras[11];       double kd_y_e		 = K_paras[11 + 13];
	double micro_e_x	= K_paras[12];       double micro_e_y	 = K_paras[12 + 13];
	
	double L_Leg		= paras[0];
	double c_Leg		= paras[1];
	double m_Leg		= paras[2];
	double Hz			= paras[3];
	double c_body		= paras[4];
	double I_body		= paras[5];
	double m_body		= paras[6];
	double M_all		= paras[7];

	double micro_con_x = 1.0;
	double micro_con_y = 1.0;

	double agl_wake_x	= agl[0];			double agl_wake_y	 = agl[0 + 2];
	double agl_shut_x	= agl[1];			double agl_shut_y	 = agl[1 + 2];
	double phi_limit_x	= limit[0];		    double phi_limit_y	 = limit[0 + 2];
	double e_limit_x	= limit[1];		    double e_limit_y	 = limit[1 + 2];

	if (in_val.del_the.pitch < agl_wake_y && in_val.del_the.pitch > -agl_wake_y) { micro_fall_y = 0.0; NQP_re.M_flag.y = 1; }
	else if (in_val.del_the.pitch > agl_shut_y || in_val.del_the.pitch < -agl_shut_y) { micro_con_y = 0.0; NQP_re.M_flag.y = 0; }
	else NQP_re.M_flag.y = 2;
	if (in_val.del_the.roll < agl_wake_x && in_val.del_the.roll > -agl_wake_x) { micro_fall_x = 0.0; NQP_re.M_flag.x = 1; }
	else if (in_val.del_the.roll > agl_shut_x || in_val.del_the.roll < -agl_shut_x) { micro_con_x = 0.0; NQP_re.M_flag.x = 0; }
	else NQP_re.M_flag.x = 2;
	
	// if (in_val.del_the.pitch > agl_shut_y || in_val.del_the.pitch < -agl_shut_y) { micro_con_y = 0.0; NQP_re.M_flag.y = 0; }
	// else
	// {
		// map = (int)(499.0 + in_val.del_the.pitch * 50.0 / agl_wake_y);
		// micro_fall_y = micro_1[map];
		// // printf("%f, %d, %f \n", in_val.del_the.pitch * 57.3, map, micro_fall_y);
		// NQP_re.M_flag.y = 1;

	// }
	// if (in_val.del_the.roll > agl_shut_x || in_val.del_the.roll < -agl_shut_x) { micro_con_x = 0.0; NQP_re.M_flag.x = 0; }
	// else
	// {
		// map = (int)(499.0 + in_val.del_the.roll * 50.0 / agl_wake_x);
		// micro_fall_x = micro_1[map];
		// // printf("%f, %d, %f \n", in_val.del_the.roll * 57.3, map, micro_fall_x);
		// NQP_re.M_flag.x = 1;
	// }
	
	Md.pitch = ((-1.0 * micro_fall_y * M_all * 9.8 * Hz - kMp_y + kf_y + kp_y) * in_val.del_the.pitch + (kd_y - kMd_y) * in_val.ddel_the.pitch);
	NQP_re.M_fall.pitch = Md.pitch;
	Md.roll = ((-1.0 * micro_fall_x * M_all * 9.8 * Hz - kMp_x + kf_x + kp_x) * in_val.del_the.roll + (kd_x - kMd_x) * in_val.ddel_the.roll);
	NQP_re.M_fall.roll = Md.roll;
	// printf("%f\n", Md.pitch);
	// printf("del_the: %f, con_flag: %d, M: %f, con: %f\n", in_val.del_the.pitch * 57.3, NQP_re.M_flag, micro_fall, micro_con);
	// printf("%f\n", in_val.del_the.pitch * 57.3);
	// printf("%f\n", in_val.del_the.roll * 57.3);
	out_val.ddphi.pitch =  micro_con_y * (-Md.pitch / I_body - k_tpc_y_phi * in_val.del_zmp.y * 0.0 - kp_y_phi * in_val.phi.pitch - kd_y_phi * in_val.dphi.pitch);
	out_val.dde.y = micro_con_y * (micro_fall_y * micro_e_y * m_body * c_body / (m_body + c_Leg / L_Leg * m_Leg) * out_val.ddphi.pitch + k_tpc_y_e * in_val.del_zmp.y - kp_y_e * in_val.e.y - kd_y_e * in_val.de.y);
	out_val.ddphi.roll = micro_con_x * (-Md.roll / I_body + k_tpc_x_phi * in_val.del_zmp.x - kp_x_phi * in_val.phi.roll - kd_x_phi * in_val.dphi.roll);
	out_val.dde.x = micro_con_x * (-micro_fall_x * micro_e_x * m_body * c_body / (m_body + c_Leg / L_Leg * m_Leg) * out_val.ddphi.roll + k_tpc_x_e * in_val.del_zmp.x - kp_x_e * in_val.e.x - kd_x_e * in_val.de.x);
	
	// printf("%f, %f \n", out_val.dde.y, out_val.ddphi.pitch);
	// printf("%f, %f \n", out_val.dde.x, out_val.ddphi.roll);
	// printf("%f\n", k_tpc_x_e);
	
	
	if (out_val.ddphi.pitch >  200.0) out_val.ddphi.pitch =  200.0;
	if (out_val.ddphi.pitch < -200.0) out_val.ddphi.pitch = -200.0;
	out_val.dphi.pitch = in_val.dphi.pitch + out_val.ddphi.pitch * CONTROL_T;
	if (out_val.dphi.pitch >  50.0) { out_val.dphi.pitch =  50.0; out_val.ddphi.pitch = (out_val.dphi.pitch - in_val.dphi.pitch) / CONTROL_T; }
	if (out_val.dphi.pitch < -50.0) { out_val.dphi.pitch = -50.0; out_val.ddphi.pitch = (out_val.dphi.pitch - in_val.dphi.pitch) / CONTROL_T; }
	out_val.phi.pitch = in_val.phi.pitch + out_val.dphi.pitch * CONTROL_T;
	if (out_val.phi.pitch >  phi_limit_y) { out_val.phi.pitch =  phi_limit_y; out_val.dphi.pitch = (out_val.phi.pitch - in_val.phi.pitch) / CONTROL_T; }
	if (out_val.phi.pitch < -phi_limit_y) { out_val.phi.pitch = -phi_limit_y; out_val.dphi.pitch = (out_val.phi.pitch - in_val.phi.pitch) / CONTROL_T; }
	if (out_val.ddphi.roll >  200.0) out_val.ddphi.roll = 200.0;
	if (out_val.ddphi.roll < -200.0) out_val.ddphi.roll = -200.0;
	out_val.dphi.roll = in_val.dphi.roll + out_val.ddphi.roll * CONTROL_T;
	if (out_val.dphi.roll >  50.0) { out_val.dphi.roll =  50.0; out_val.ddphi.roll = (out_val.dphi.roll - in_val.dphi.roll) / CONTROL_T; }
	if (out_val.dphi.roll < -50.0) { out_val.dphi.roll = -50.0; out_val.ddphi.roll = (out_val.dphi.roll - in_val.dphi.roll) / CONTROL_T; }
	out_val.phi.roll = in_val.phi.roll + out_val.dphi.roll * CONTROL_T;
	if (out_val.phi.roll >  phi_limit_x) { out_val.phi.roll =  phi_limit_x; out_val.dphi.roll = (out_val.phi.roll - in_val.phi.roll) / CONTROL_T; }
	if (out_val.phi.roll < -phi_limit_x) { out_val.phi.roll = -phi_limit_x; out_val.dphi.roll = (out_val.phi.roll - in_val.phi.roll) / CONTROL_T; }

	if (out_val.dde.y >  40.0) out_val.dde.y =  40.0;
	if (out_val.dde.y < -40.0) out_val.dde.y = -40.0;
	out_val.de.y = in_val.de.y + out_val.dde.y * CONTROL_T;
	if (out_val.de.y >  1.0) { out_val.de.y =  1.0; out_val.dde.y = (out_val.de.y - in_val.de.y) / CONTROL_T; }
	if (out_val.de.y < -1.0) { out_val.de.y = -1.0; out_val.dde.y = (out_val.de.y - in_val.de.y) / CONTROL_T; }
	out_val.e.y  = in_val.e.y + out_val.de.y * CONTROL_T;
	if (out_val.e.y >  e_limit_y) { out_val.e.y =  e_limit_y; out_val.de.y = (out_val.e.y - in_val.e.y) / CONTROL_T; }
	if (out_val.e.y < -e_limit_y) { out_val.e.y = -e_limit_y; out_val.de.y = (out_val.e.y - in_val.e.y) / CONTROL_T; }
	if (out_val.dde.x >  40.0) out_val.dde.x = 40.0;
	if (out_val.dde.x < -40.0) out_val.dde.x = -40.0;
	out_val.de.x = in_val.de.x + out_val.dde.x * CONTROL_T;
	if (out_val.de.x >  1.0) { out_val.de.x =  1.0; out_val.dde.x = (out_val.de.x - in_val.de.x) / CONTROL_T; }
	if (out_val.de.x < -1.0) { out_val.de.x = -1.0; out_val.dde.x = (out_val.de.x - in_val.de.x) / CONTROL_T; }
	out_val.e.x = in_val.e.x + out_val.de.x * CONTROL_T;
	if (out_val.e.x >  e_limit_x) { out_val.e.x =  e_limit_x; out_val.de.x = (out_val.e.x - in_val.e.x) / CONTROL_T; }
	if (out_val.e.x < -e_limit_x) { out_val.e.x = -e_limit_x; out_val.de.x = (out_val.e.x - in_val.e.x) / CONTROL_T; }

	out_val.del_zmp  = in_val.del_zmp;
	out_val.del_the  = in_val.del_the;
	out_val.ddel_the = in_val.ddel_the;
	
	
	
	// printf("%f, ", in_val.del_the.roll);
	// printf("%f\n", out_val.phi.roll);
	return out_val;

}

void check_IMU_bias(int k_pre, double sens_pitch, double sens_roll)
{
	if(k_pre <= 1) 
	{
		IMU_bias.pitch = sens_pitch;
		IMU_bias.roll  = sens_roll;
	}
	else
	{
		IMU_bias.pitch = (IMU_bias.pitch + sens_pitch) * 0.5;
		IMU_bias.roll  = (IMU_bias.roll  + sens_roll)  * 0.5;
	}
}

void NQP_balance_con(int k_pre, double sens_pitch, double sens_roll, double body_agl_d[2])
{
	double K_paras[26] = {
		// x & roll
		40.0, 4.0,					  // kM to recover posture	
	    0.0, 0.0, m_robot * 0.7,	  // compliance & decline
		0.2,					      // micro M fall
		0.0, 90.0, 30.9,			  // tpc phi // 100, 120.0, 30.9,	
		20.8, 54.4, 32.9,			  // tpc e   // 40.5, 175.4, 22.9,
		1.5,						  // micro e
		// y & pitch
		85.0, 4.0,					  // kM to recover posture	
		0.0, 0.0, m_robot * 0.7,	  // compliance & decline
		0.1,					      // micro M fall
		0.0, 100.0, 22.9,			  // tpc phi // 400, 180.0, 16.9,	
		20.8, 54.4, 32.9,			  // tpc e   // 50.8, 150.4, 20.9,	
		1.5    						  // micro e
	};

	double paras[8] = {
		0.32 * 2.0,                   // L_Leg
		0.40,						  // c_Leg
		9.0 * 2.0,					  // m_Leg
		0.7,						  // Zc
		0.45 * 0.5, 				  // c_body
		14.0 * 0.45 * 0.45 * 0.0833,  // I_body
		14.0,						  // m_body
		40.7						  // M_all
	};

	double agl_config[4] = { 3.25 / 57.3, 15.0 / 57.3,   // x & roll:  delta_theta: min (mech flex), max
							 4.50 / 57.3, 15.0 / 57.3 }; // y & pitch: delta_theta: min (mech flex), max

	double limits[4] = { 11.0 / 57.3, 0.05,			// x & roll:  phi, e 
					     20.0 / 57.3, 0.1 };        // y & pitch: phi, e 

	NQPbalance_paras_conval NQP_test_old;
	NQP_test_old = NQP_test;

	// cal delta_pitch & ddelta_pitch
	double delta_pitch;
	double delta_roll;
	double pitch_d = body_agl_d[0];
	double roll_d  = body_agl_d[1];
	// printf("%f\n", sens_pitch);
	delta_pitch = sens_pitch - NQP_test_old.phi.pitch - pitch_d - IMU_bias.pitch;
	delta_roll  = sens_roll  - NQP_test_old.phi.roll  -  roll_d - IMU_bias.roll;
	//printf("%f, %f\n", sens_pitch, pitch_d);LegJoint_Calculate

	NQP_test_old.ddel_the.roll  = (delta_roll  - NQP_test_old.del_the.roll ) / CONTROL_T;
	NQP_test_old.ddel_the.pitch = (delta_pitch - NQP_test_old.del_the.pitch) / CONTROL_T;
	NQP_test_old.del_the.roll   = delta_roll ;
	NQP_test_old.del_the.pitch  = delta_pitch;
	// printf("%f\n", NQP_test_old.del_the.roll * 57.3);
	// cal delta_pitch & ddelta_pitch

	// cal delta_zmp
	#ifdef USE_NQP_BALANCE
	P_ZMPRef_B.py = P_ZMPRef_B.py - 0.003; // 0.052;
	#endif
	NQP_test_old.del_zmp.x = P_ZMPRel_B.px - P_ZMPRef_B.px;
	NQP_test_old.del_zmp.y = P_ZMPRel_B.py - P_ZMPRef_B.py;
	// cal delta_zmp
	
	// init IMU_bias
	if (k_pre < 50)
	{
		check_IMU_bias(k_pre, sens_pitch, sens_roll);
	}
	// init IMU_bias
	
	NQP_test = cal_NQP_conval(NQP_test_old, K_paras, paras, agl_config, limits); // <- balance controller, get phi & e
	// printf("%f, ", NQP_test.phi.pitch * 57.3);
	// re
	// printf("%f\n", NQP_test.del_the.roll * 57.3);
	NQP_re.e.x				= NQP_test.e.x		      ;	
	NQP_re.de.x				= NQP_test.de.x			  ;
	NQP_re.dde.x			= NQP_test.dde.x		  ;
	NQP_re.phi.roll			= NQP_test.phi.roll		  ;
	NQP_re.dphi.roll		= NQP_test.dphi.roll	  ;
	NQP_re.ddphi.roll		= NQP_test.ddphi.roll	  ;
	NQP_re.del_zmp.x		= NQP_test.del_zmp.x	  ;
	NQP_re.del_the.roll		= NQP_test.del_the.roll   ;
	NQP_re.ddel_the.roll	= NQP_test.ddel_the.roll  ;
	NQP_re.e.y				= NQP_test.e.y		      ;	
	NQP_re.de.y				= NQP_test.de.y			  ;
	NQP_re.dde.y			= NQP_test.dde.y		  ;
	NQP_re.phi.pitch		= NQP_test.phi.pitch	  ;
	NQP_re.dphi.pitch		= NQP_test.dphi.pitch	  ;
	NQP_re.ddphi.pitch		= NQP_test.ddphi.pitch	  ;
	NQP_re.del_zmp.y		= NQP_test.del_zmp.y	  ;
	NQP_re.del_the.pitch	= NQP_test.del_the.pitch  ;
	NQP_re.ddel_the.pitch	= NQP_test.ddel_the.pitch ;
	// re
}

//COM_STATIC COM_static_con(double q[6] /* R345 L345 */)
//{
//	if (Signal_SupportLeg == RLEG_SP)
//	{
//		double sq3 = q[0];
//		double sq4 = q[1];
//		double sq5 = q[2];
//		double fq3 = q[3];
//		double fq4 = q[4];
//		double fq5 = q[5];
//	}
//	else if (Signal_SupportLeg == LLEG_SP)
//	{
//		double fq3 = q[0];
//		double fq4 = q[1];
//		double fq5 = q[2];
//		double sq3 = q[3];
//		double sq4 = q[4];
//		double sq5 = q[5];
//	}
//	else
//	H_ankle = paras(1); L_Leg = paras(2); L_body = paras(3);
//
//
//
//	p = [0, 0, (8 * sin(sq5)) / 25, (8 * sin(sq5)) / 25 + (8 * cos(sq4)*sin(sq5)) / 25 + (8 * cos(sq5)*sin(sq4)) / 25, (8 * sin(sq5)) / 25 + (8 * cos(sq4)*sin(sq5)) / 25 + (8 * cos(sq5)*sin(sq4)) / 25 + L_body*(cos(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4)) + sin(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5))), (8 * sin(sq5)) / 25 + (8 * cos(sq4)*sin(sq5)) / 25 + (8 * cos(sq5)*sin(sq4)) / 25 - L_Leg*(cos(fq3)*(cos(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4)) + sin(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5))) - sin(fq3)*(cos(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5)) - sin(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4)))), (8 * sin(sq5)) / 25 + (8 * cos(sq4)*sin(sq5)) / 25 + (8 * cos(sq5)*sin(sq4)) / 25 - L_Leg*(cos(fq4)*(cos(fq3)*(cos(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4)) + sin(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5))) - sin(fq3)*(cos(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5)) - sin(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4)))) - sin(fq4)*(cos(fq3)*(cos(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5)) - sin(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4))) + sin(fq3)*(cos(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4)) + sin(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5))))) - (8 * cos(fq3)*(cos(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4)) + sin(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5)))) / 25 + (8 * sin(fq3)*(cos(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5)) - sin(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4)))) / 25, (8 * sin(sq5)) / 25 - (8 * cos(fq4)*(cos(fq3)*(cos(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4)) + sin(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5))) - sin(fq3)*(cos(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5)) - sin(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4))))) / 25 + (8 * sin(fq4)*(cos(fq3)*(cos(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5)) - sin(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4))) + sin(fq3)*(cos(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4)) + sin(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5))))) / 25 + (8 * cos(sq4)*sin(sq5)) / 25 + (8 * cos(sq5)*sin(sq4)) / 25 - (8 * cos(fq3)*(cos(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4)) + sin(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5)))) / 25 + (8 * sin(fq3)*(cos(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5)) - sin(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4)))) / 25 - H_ankle*(cos(fq5)*(cos(fq4)*(cos(fq3)*(cos(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4)) + sin(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5))) - sin(fq3)*(cos(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5)) - sin(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4)))) - sin(fq4)*(cos(fq3)*(cos(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5)) - sin(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4))) + sin(fq3)*(cos(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4)) + sin(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5))))) - sin(fq5)*(cos(fq4)*(cos(fq3)*(cos(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5)) - sin(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4))) + sin(fq3)*(cos(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4)) + sin(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5)))) + sin(fq4)*(cos(fq3)*(cos(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4)) + sin(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5))) - sin(fq3)*(cos(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5)) - sin(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4))))));
//	0, H_ankle, H_ankle + (8 * cos(sq5)) / 25, H_ankle + (8 * cos(sq5)) / 25 + (8 * cos(sq4)*cos(sq5)) / 25 - (8 * sin(sq4)*sin(sq5)) / 25, H_ankle + (8 * cos(sq5)) / 25 + (8 * cos(sq4)*cos(sq5)) / 25 - (8 * sin(sq4)*sin(sq5)) / 25 + L_body*(cos(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5)) - sin(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4))), H_ankle + (8 * cos(sq5)) / 25 + (8 * cos(sq4)*cos(sq5)) / 25 - (8 * sin(sq4)*sin(sq5)) / 25 - L_Leg*(cos(fq3)*(cos(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5)) - sin(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4))) + sin(fq3)*(cos(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4)) + sin(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5)))), H_ankle + (8 * cos(sq5)) / 25 + (8 * cos(sq4)*cos(sq5)) / 25 - (8 * sin(sq4)*sin(sq5)) / 25 - L_Leg*(cos(fq4)*(cos(fq3)*(cos(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5)) - sin(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4))) + sin(fq3)*(cos(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4)) + sin(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5)))) + sin(fq4)*(cos(fq3)*(cos(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4)) + sin(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5))) - sin(fq3)*(cos(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5)) - sin(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4))))) - (8 * cos(fq3)*(cos(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5)) - sin(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4)))) / 25 - (8 * sin(fq3)*(cos(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4)) + sin(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5)))) / 25, H_ankle + (8 * cos(sq5)) / 25 - (8 * cos(fq4)*(cos(fq3)*(cos(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5)) - sin(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4))) + sin(fq3)*(cos(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4)) + sin(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5))))) / 25 + (8 * cos(sq4)*cos(sq5)) / 25 - (8 * sin(fq4)*(cos(fq3)*(cos(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4)) + sin(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5))) - sin(fq3)*(cos(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5)) - sin(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4))))) / 25 - (8 * sin(sq4)*sin(sq5)) / 25 - (8 * cos(fq3)*(cos(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5)) - sin(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4)))) / 25 - (8 * sin(fq3)*(cos(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4)) + sin(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5)))) / 25 - H_ankle*(cos(fq5)*(cos(fq4)*(cos(fq3)*(cos(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5)) - sin(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4))) + sin(fq3)*(cos(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4)) + sin(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5)))) + sin(fq4)*(cos(fq3)*(cos(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4)) + sin(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5))) - sin(fq3)*(cos(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5)) - sin(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4))))) + sin(fq5)*(cos(fq4)*(cos(fq3)*(cos(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4)) + sin(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5))) - sin(fq3)*(cos(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5)) - sin(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4)))) - sin(fq4)*(cos(fq3)*(cos(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5)) - sin(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4))) + sin(fq3)*(cos(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4)) + sin(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5))))));
//	1, 1, 1, 1, 1, 1, 1, 1];
//
//	c = [0, (L_Leg*sin(sq5)) / 2, (L_Leg*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4))) / 2 + L_Leg*sin(sq5), L_Leg*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4)) + L_Leg*sin(sq5), L_Leg*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4)) + L_Leg*sin(sq5) + (L_body*(cos(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4)) + sin(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5)))) / 2, L_Leg*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4)) - (L_Leg*(cos(fq3)*(cos(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4)) + sin(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5))) - sin(fq3)*(cos(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5)) - sin(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4))))) / 2 + L_Leg*sin(sq5), L_Leg*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4)) - (L_Leg*(cos(fq4)*(cos(fq3)*(cos(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4)) + sin(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5))) - sin(fq3)*(cos(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5)) - sin(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4)))) - sin(fq4)*(cos(fq3)*(cos(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5)) - sin(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4))) + sin(fq3)*(cos(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4)) + sin(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5)))))) / 2 - L_Leg*(cos(fq3)*(cos(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4)) + sin(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5))) - sin(fq3)*(cos(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5)) - sin(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4)))) + L_Leg*sin(sq5), L_Leg*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4)) - L_Leg*(cos(fq4)*(cos(fq3)*(cos(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4)) + sin(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5))) - sin(fq3)*(cos(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5)) - sin(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4)))) - sin(fq4)*(cos(fq3)*(cos(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5)) - sin(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4))) + sin(fq3)*(cos(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4)) + sin(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5))))) - H_ankle*(cos(fq5)*(cos(fq4)*(cos(fq3)*(cos(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4)) + sin(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5))) - sin(fq3)*(cos(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5)) - sin(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4)))) - sin(fq4)*(cos(fq3)*(cos(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5)) - sin(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4))) + sin(fq3)*(cos(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4)) + sin(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5))))) - sin(fq5)*(cos(fq4)*(cos(fq3)*(cos(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5)) - sin(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4))) + sin(fq3)*(cos(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4)) + sin(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5)))) + sin(fq4)*(cos(fq3)*(cos(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4)) + sin(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5))) - sin(fq3)*(cos(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5)) - sin(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4)))))) - L_Leg*(cos(fq3)*(cos(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4)) + sin(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5))) - sin(fq3)*(cos(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5)) - sin(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4)))) + L_Leg*sin(sq5);
//	0, H_ankle + (L_Leg*cos(sq5)) / 2, H_ankle + (L_Leg*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5))) / 2 + L_Leg*cos(sq5), H_ankle + L_Leg*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5)) + L_Leg*cos(sq5), H_ankle + L_Leg*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5)) + L_Leg*cos(sq5) + (L_body*(cos(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5)) - sin(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4)))) / 2, H_ankle + L_Leg*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5)) + L_Leg*cos(sq5) - (L_Leg*(cos(fq3)*(cos(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5)) - sin(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4))) + sin(fq3)*(cos(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4)) + sin(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5))))) / 2, H_ankle - (L_Leg*(cos(fq4)*(cos(fq3)*(cos(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5)) - sin(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4))) + sin(fq3)*(cos(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4)) + sin(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5)))) + sin(fq4)*(cos(fq3)*(cos(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4)) + sin(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5))) - sin(fq3)*(cos(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5)) - sin(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4)))))) / 2 + L_Leg*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5)) + L_Leg*cos(sq5) - L_Leg*(cos(fq3)*(cos(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5)) - sin(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4))) + sin(fq3)*(cos(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4)) + sin(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5)))), H_ankle - L_Leg*(cos(fq4)*(cos(fq3)*(cos(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5)) - sin(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4))) + sin(fq3)*(cos(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4)) + sin(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5)))) + sin(fq4)*(cos(fq3)*(cos(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4)) + sin(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5))) - sin(fq3)*(cos(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5)) - sin(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4))))) + L_Leg*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5)) - H_ankle*(cos(fq5)*(cos(fq4)*(cos(fq3)*(cos(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5)) - sin(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4))) + sin(fq3)*(cos(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4)) + sin(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5)))) + sin(fq4)*(cos(fq3)*(cos(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4)) + sin(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5))) - sin(fq3)*(cos(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5)) - sin(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4))))) + sin(fq5)*(cos(fq4)*(cos(fq3)*(cos(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4)) + sin(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5))) - sin(fq3)*(cos(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5)) - sin(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4)))) - sin(fq4)*(cos(fq3)*(cos(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5)) - sin(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4))) + sin(fq3)*(cos(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4)) + sin(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5)))))) + L_Leg*cos(sq5) - L_Leg*(cos(fq3)*(cos(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5)) - sin(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4))) + sin(fq3)*(cos(sq3)*(cos(sq4)*sin(sq5) + cos(sq5)*sin(sq4)) + sin(sq3)*(cos(sq4)*cos(sq5) - sin(sq4)*sin(sq5))));
//	1, 1, 1, 1, 1, 1, 1, 1];
//
//
//}

