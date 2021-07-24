#pragma once

// #define USE_NQP_BALANCE

typedef struct
{
	double x;
	double y;
}QP_Horizontal;

typedef	struct
{
	double pitch;
	double roll;
}QP_Rotational;

typedef struct
{
	QP_Rotational del_the; // angle foot2ground pitch roll
	QP_Rotational ddel_the; // pitch roll
	QP_Rotational ddphi; // pitch roll
	QP_Rotational dphi; // pitch roll
	QP_Rotational phi; // pitch roll
	QP_Horizontal dde; // x y
	QP_Horizontal de; // x y
	QP_Horizontal e; // x y
	QP_Horizontal del_zmp; //x y
}NQPbalance_paras_conval;

typedef struct
{
	QP_Rotational del_the; // angle foot2ground pitch roll
	QP_Rotational ddel_the; // pitch roll
	QP_Rotational ddphi; // pitch roll
	QP_Rotational dphi; // pitch roll
	QP_Rotational phi; // pitch roll
	QP_Horizontal dde; // x y
	QP_Horizontal de; // x y
	QP_Horizontal e; // x y
	QP_Horizontal del_zmp; //x y
	QP_Rotational M_fall;
	QP_Horizontal M_flag;
}RE_NQP;

typedef struct
{
	double q[12]; // sup 1-6 swi 1-6
	double com[3]; // x y z
	double e[2]; // x y
	double phi[2]; // pitch roll
}QPbalance_paras_conval;

typedef struct
{
	QP_Horizontal dde;
	QP_Rotational ddphi;
	QP_Horizontal com;
	QP_Horizontal com_d;
	QP_Horizontal del_zmp;
}COM_STATIC;

//void Transfer_Leg(int sup_signal, double * q, double * qR_rel, double * qL_rel);
NQPbalance_paras_conval cal_NQP_conval(NQPbalance_paras_conval in_val, double K_paras[26], double paras[8], double agl[4], double limit[4]);
void check_IMU_bias(int k_pre, double sens_pitch, double sens_roll);
void NQP_balance_con(int k_pre, double sens_pitch, double sens_roll, double body_agl_d[2]);

//#ifdef USE_NQP_BALANCE @@@ controverdict with other posture con: other conval should be added in agl_d, @@@ must renew roll_body & pitch_body every cycle
//NQP_balance_con(K_Preview_Con, -GQ_Pitch, GQ_Roll, body_agl_d);
//Tra_COM.y[K_Preview_Con] += 1.0 * NQP_test.e.y;
//pitch_body += 1.0 * NQP_test.phi.pitch;
//#endif