#include "BHR7_diff_inv.h"
// #include "get_PL_PR.h"
#include <math.h>
#include <stdio.h>

double MECH_PARAS[14] = { THIGH_L * 1e-3, THIGH_1 * 1e-3, THIGH_2 * 1e-3, THIGH_3 * 1e-3, THIGH_4 * 1e-3, THIGH_5 * 1e-3, SHANK_L * 1e-3, SHANK_1 * 1e-3, SHANK_2 * 1e-3, ANKLE_3 * 1e-3, ANKLE_4 * 1e-3, ANKLE_5 * 1e-3, ANKLE_6 * 1e-3, ALPHA_1 / 57.3 };


JOINTS_AGL cal_old2diff(JOINTS_AGL q, double mech_paras[14], double * theta_zero)
{
	double L1 = mech_paras[0];
	double r1 = mech_paras[1];
	double r2 = mech_paras[2];
	double r3 = mech_paras[3];
	double r4 = mech_paras[4];
	double r5 = mech_paras[5];
	double L2 = mech_paras[6];
	double a1 = mech_paras[7];
	double a2 = mech_paras[8];
	double a3 = mech_paras[9];
	double a4 = mech_paras[10];
	double a5 = mech_paras[11];
	double a6 = mech_paras[12];
	double alpha = mech_paras[13];
	double theta1 = theta_zero[0];
	double theta2 = theta_zero[1];
	double mech_paras_for_PLPR[3] = { a3, a4, a5 };
	double joints_for_PLPR[2] = { q.q5, q.q6 };
	double theta;
	double q4_cal, phi4_cal;
	double theta3, theta4, theta5, theta6, theta7;
	double r;
	double PLPR[6];
	double KL[3] = { -a6 / 2.0, 0.0, L2 };
	double KR[3] = {  a6 / 2.0, 0.0, L2 };
	double PL_rec[3];
	double PR_rec[3];
	double L = a2;
	double DL, DR, HL, HR;
	double betaL1, betaR1, betaL2, betaR2;
	double phiL_cal, phiR_cal;

	JOINTS_AGL phi;
	phi.q1 = q.q1;
	phi.q2 = q.q2;
	phi.q3 = q.q3;
	q.q4 = -q.q4;

	theta = 3.1415926 - alpha;
	q4_cal = q.q4 + theta;
	theta3 = acos((L1 * L1 + r4 * r4 - r5 * r5) / (2.0 * L1 * r4));
	theta4 = 3.1415926 - theta3;
	theta5 = q4_cal - theta3;
	r = sqrt(r4 * r4 + r3 * r3 - 2.0 * r4 * r3 * cos(theta5));
	theta6 = acos((r * r + r4 * r4 - r3 * r3) / (2.0 * r * r4));
	theta7 = acos((r1 * r1 + r * r - r2 * r2) / (2.0 * r1 * r));
	phi4_cal = 2.0 * 3.1415926 - theta4 - theta6 - theta7;
	phi.q4 = -(phi4_cal - theta1);

	cal_PL_PR(PLPR, joints_for_PLPR, mech_paras_for_PLPR);
	PL_rec[0] = KL[0]; PL_rec[1] = PLPR[1]; PL_rec[2] = PLPR[2];
	PR_rec[0] = KR[0]; PR_rec[1] = PLPR[4]; PR_rec[2] = PLPR[5];
	DL = sqrt(PL_rec[1] * PL_rec[1] + (KL[2] - PL_rec[2]) * (KL[2] - PL_rec[2]));
	DR = sqrt(PR_rec[1] * PR_rec[1] + (KR[2] - PR_rec[2]) * (KR[2] - PR_rec[2]));
	HL = sqrt(L * L - (PLPR[0] - PL_rec[0]) * (PLPR[0] - PL_rec[0]));
	HR = sqrt(L * L - (PLPR[3] - PR_rec[0]) * (PLPR[3] - PR_rec[0]));
	betaL1 = atan2(PL_rec[1], (KL[2] - PL_rec[2]));
	betaR1 = atan2(PR_rec[1], (KR[2] - PR_rec[2]));
	betaL2 = acos((a1 * a1 + DL * DL - HL * HL) / 2.0 / a1 / DL);
	betaR2 = acos((a1 * a1 + DR * DR - HR * HR) / 2.0 / a1 / DR);
	phiL_cal = 3.1415926 - betaL1 - betaL2;
	phiR_cal = 3.1415926 - betaR1 - betaR2;
	phi.q5 = -(phiL_cal - theta2 + q.q4);
	phi.q6 = -(phiR_cal - theta2 + q.q4);
	
	return phi;
}

void cal_zero(double * theta_zero, double mech_paras[14])
{
	JOINTS_AGL q = {0,0,0,0,0,0};
	//double theta1, theta2;
	double L1 = mech_paras[0];
	double r1 = mech_paras[1];
	double r2 = mech_paras[2];
	double r3 = mech_paras[3];
	double r4 = mech_paras[4];
	double r5 = mech_paras[5];
	double L2 = mech_paras[6];
	double a1 = mech_paras[7];
	double a2 = mech_paras[8];
	double a3 = mech_paras[9];
	double a4 = mech_paras[10];
	double a5 = mech_paras[11];
	double a6 = mech_paras[12];
	double alpha = mech_paras[13];
	double mech_paras_for_PLPR[3] = { a3, a4, a5 };
	double joints_for_PLPR[2] = { q.q5, q.q6 };
	double theta;
	double q4_cal, phi4_cal;
	double theta3, theta4, theta5, theta6, theta7;
	double r;
	double PLPR[6];
	double KL[3] = { -a6 / 2.0, 0.0, L2 };
	double KR[3] = {  a6 / 2.0, 0.0, L2 };
	double PL_rec[3];
	double PR_rec[3];
	double L = a2;
	double DL, DR, HL, HR;
	double betaL1, betaR1, betaL2, betaR2;
	double phiL_cal, phiR_cal;

	q.q1 = 0.0;
	q.q2 = 0.0;
	q.q3 = 0.0;
	q.q4 = 0.0;
	q.q5 = 0.0;
	q.q6 = 0.0;

	theta = 3.1415926 - alpha;
	q4_cal = q.q4 + theta;
	theta3 = acos((L1 * L1 + r4 * r4 - r5 * r5) / (2.0 * L1 * r4));
	theta4 = 3.1415926 - theta3;
	theta5 = q4_cal - theta3;
	r = sqrt(r4 * r4 + r3 * r3 - 2.0 * r4 * r3 * cos(theta5));
	theta6 = acos((r * r + r4 * r4 - r3 * r3) / (2.0 * r * r4));
	theta7 = acos((r1 * r1 + r * r - r2 * r2) / (2.0 * r1 * r));
	phi4_cal = 2.0 * 3.1415926 - theta4 - theta6 - theta7;

	cal_PL_PR(PLPR, joints_for_PLPR, mech_paras_for_PLPR);
	PL_rec[0] = KL[0]; PL_rec[1] = PLPR[1]; PL_rec[2] = PLPR[2];
	PR_rec[0] = KR[0]; PR_rec[1] = PLPR[4]; PR_rec[2] = PLPR[5];
	DL = sqrt(PL_rec[1] * PL_rec[1] + (KL[2] - PL_rec[2]) * (KL[2] - PL_rec[2]));
	DR = sqrt(PR_rec[1] * PR_rec[1] + (KR[2] - PR_rec[2]) * (KR[2] - PR_rec[2]));
	HL = sqrt(L * L - (PLPR[0] - PL_rec[0]) * (PLPR[0] - PL_rec[0]));
	HR = sqrt(L * L - (PLPR[3] - PR_rec[0]) * (PLPR[3] - PR_rec[0]));
	betaL1 = atan2(PL_rec[1], (KL[2] - PL_rec[2]));
	betaR1 = atan2(PR_rec[1], (KR[2] - PR_rec[2]));
	betaL2 = acos((a1 * a1 + DL * DL - HL * HL) / 2.0 / a1 / DL);
	betaR2 = acos((a1 * a1 + DR * DR - HR * HR) / 2.0 / a1 / DR);
	phiL_cal = 3.1415926 - betaL1 - betaL2;
	phiR_cal = 3.1415926 - betaR1 - betaR2;

	theta_zero[0] = phi4_cal;
	theta_zero[1] = 0.5 * (phiL_cal + phiR_cal);
}


JOINTS_AGL cal_reset(double theta_reset, double mech_paras[14], double * theta_zero)
{
	JOINTS_AGL q = {0,0,0,0,0,0};
	JOINTS_AGL phi;

	double L1 = mech_paras[0];
	double r1 = mech_paras[1];
	double r2 = mech_paras[2];
	double r3 = mech_paras[3];
	double r4 = mech_paras[4];
	double r5 = mech_paras[5];
	double L2 = mech_paras[6];
	double a1 = mech_paras[7];
	double a2 = mech_paras[8];
	double a3 = mech_paras[9];
	double a4 = mech_paras[10];
	double a5 = mech_paras[11];
	double a6 = mech_paras[12];
	double alpha = mech_paras[13];
	double theta1 = theta_zero[0];
	double theta2 = theta_zero[1];
	double mech_paras_for_PLPR[3] = { a3, a4, a5 };
	double joints_for_PLPR[2] = { q.q5, q.q6 };
	double theta;
	double q4_cal, phi4_cal;
	double theta3, theta4, theta5, theta6, theta7;
	double r;
	double PLPR[6];
	double KL[3] = { -a6 / 2.0, 0.0, L2 };
	double KR[3] = {  a6 / 2.0, 0.0, L2 };
	double PL_rec[3];
	double PR_rec[3];
	double L = a2;
	double DL, DR, HL, HR;
	double betaL1, betaR1, betaL2, betaR2;
	double phiL_cal, phiR_cal;

	q.q1 = 0.0;
	q.q2 = 0.0;
	q.q3 = 0.5 * theta_reset / 57.3;
	q.q4 = theta_reset / 57.3;
	q.q5 = 0.5 * theta_reset / 57.3;
	q.q6 = 0.0; 

	phi.q1 = q.q1;
	phi.q2 = q.q2;
	phi.q3 = q.q3;

	theta = 3.1415926 - alpha;
	q4_cal = q.q4 + theta;
	theta3 = acos((L1 * L1 + r4 * r4 - r5 * r5) / (2.0 * L1 * r4));
	theta4 = 3.1415926 - theta3;
	theta5 = q4_cal - theta3;
	r = sqrt(r4 * r4 + r3 * r3 - 2.0 * r4 * r3 * cos(theta5));
	theta6 = acos((r * r + r4 * r4 - r3 * r3) / (2.0 * r * r4));
	theta7 = acos((r1 * r1 + r * r - r2 * r2) / (2.0 * r1 * r));
	phi4_cal = 2.0 * 3.1415926 - theta4 - theta6 - theta7;
	phi.q4 = -(phi4_cal - theta1);

	cal_PL_PR(PLPR, joints_for_PLPR, mech_paras_for_PLPR);
	PL_rec[0] = KL[0]; PL_rec[1] = PLPR[1]; PL_rec[2] = PLPR[2];
	PR_rec[0] = KR[0]; PR_rec[1] = PLPR[4]; PR_rec[2] = PLPR[5];
	DL = sqrt(PL_rec[1] * PL_rec[1] + (KL[2] - PL_rec[2]) * (KL[2] - PL_rec[2]));
	DR = sqrt(PR_rec[1] * PR_rec[1] + (KR[2] - PR_rec[2]) * (KR[2] - PR_rec[2]));
	HL = sqrt(L * L - (PLPR[0] - PL_rec[0]) * (PLPR[0] - PL_rec[0]));
	HR = sqrt(L * L - (PLPR[3] - PR_rec[0]) * (PLPR[3] - PR_rec[0]));
	betaL1 = atan2(PL_rec[1], (KL[2] - PL_rec[2]));
	betaR1 = atan2(PR_rec[1], (KR[2] - PR_rec[2]));
	betaL2 = acos((a1 * a1 + DL * DL - HL * HL) / 2.0 / a1 / DL);
	betaR2 = acos((a1 * a1 + DR * DR - HR * HR) / 2.0 / a1 / DR);
	phiL_cal = 3.1415926 - betaL1 - betaL2;
	phiR_cal = 3.1415926 - betaR1 - betaR2;

	return phi;
}

void cal_PL_PR(double * PLPR, double ankle_joints[2], double mech_paras[3])
{
	double q5 = ankle_joints[0];
	double q6 = ankle_joints[1];
	double a3 = mech_paras[0];
	double a4 = mech_paras[1];
	double a5 = mech_paras[2];
	
	double Tx[4][4] = {	1.0, 0.0, 0.0, 0.0,
						0.0, cos(q5), -sin(q5), 0.0,
						0.0, sin(q5), cos(q5), 0.0,
						0.0, 0.0, 0.0, 1.0	};
	double Ty[4][4] = {	cos(q6), 0.0, sin(q6), 0.0,
						0.0, 1.0, 0.0, 0.0,
						-sin(q6), 0.0, cos(q6), 0.0,
						0.0, 0.0, 0.0, 1.0	};
						
	double PL_strait[4] = {-a5 / 2.0, a3, -a4, 1.0};
	double PR_strait[4] = { a5 / 2.0, a3, -a4, 1.0};	

	double T_temp[4][4];
	double PL[4];
	double PR[4];
	
	for(int i = 0; i < 4; i++)
	{
		for(int j = 0; j < 4; j++)
		{
			T_temp[i][j] = Tx[i][0] * Ty[0][j] + Tx[i][1] * Ty[1][j] + Tx[i][2] * Ty[2][j] + Tx[i][3] * Ty[3][j];
		}
	}
	
	for(int i = 0; i < 4; i++)
	{
		PL[i] = T_temp[i][0] * PL_strait[0] + T_temp[i][1] * PL_strait[1] + T_temp[i][2] * PL_strait[2] + T_temp[i][3] * PL_strait[3]; 
		PR[i] = T_temp[i][0] * PR_strait[0] + T_temp[i][1] * PR_strait[1] + T_temp[i][2] * PR_strait[2] + T_temp[i][3] * PR_strait[3]; 
	}
	
	for(int i = 0; i < 3; i++)
	{
		* (PLPR + i) = PL[i];
		* (PLPR + i + 3) = PR[i];
	}
}