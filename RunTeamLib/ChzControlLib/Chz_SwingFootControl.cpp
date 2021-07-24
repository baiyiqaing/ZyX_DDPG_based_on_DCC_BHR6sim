#include "Chz_Kinematics.h"
#include "Chz_RobotParam.h"
#include "Chz_Spline.h"
#include "Chz_SwingFootControl.h"
extern "C"
{
	extern double chz_log[40];
}
namespace Chz
{
	extern Kinematics Kinematic_Calc;
	extern RobotParam Robot;
}

void Chz::SwingFootController::GetRealPos(VectorXd CoM_r1, VectorXd Joints)
{
	CoM_r = CoM_r1;

	Vector3d Ledge_rel[4], Redge_rel[4];
	Vector3d Ledge_d[4], Redge_d[4];
	Kinematic_Calc.FK_FootfromChunk(RPos_r, CoM_r, Joints, 'R');
	Kinematic_Calc.FK_FootfromChunk(LPos_r, CoM_r, Joints, 'L');
	
	Kinematic_Calc.FK_FootEdgefromFoot(Redge_rel, RPos_r, 'R');
	Kinematic_Calc.FK_FootEdgefromFoot(Ledge_rel, LPos_r, 'L');
	Kinematic_Calc.FK_FootEdgefromFoot(Redge_d, RPos_d, 'R');
	Kinematic_Calc.FK_FootEdgefromFoot(Ledge_d, LPos_d, 'L');
	
	double zminl = inf, zminr = inf; int posl = -1, posr = -1;
	ChzF(i, 0, 3) if (Redge_rel[i](2) < zminr) zminr = Redge_rel[i](2), posr = i;
	ChzF(i, 0, 3) if (Ledge_rel[i](2) < zminl) zminl = Ledge_rel[i](2), posl = i;
	
	Vector3d Err = rprop * (Redge_d[posr] - Redge_rel[posr]) + (1.0 - rprop) * (Ledge_d[posl] - Ledge_rel[posl]);
	ChzF(i, 0, 2) CoM_r(i) += Err(i), RPos_r(i) += Err(i), LPos_r(i) += Err(i);
}

void Chz::SwingFootController::SetFootPD(char c, double mp, double me, double ts)
{
	double wn, xi;
	double lnmp = log(mp);
	const double pi = acos(-1.0);
	xi = sqrt(lnmp * lnmp / (pi * pi + lnmp * lnmp));
	wn = -1.0 / (ts * xi) * log(me * sqrt(1 - xi * xi));
	if (c == 'A')
	{
		Kp_air_pos = Kp_air_ang = wn * wn;
		Kd_air_pos = Kd_air_ang = 2.0 * wn * xi;
	}
	else if (c == 'G')
	{
		Kp_grd = wn * wn;
		Kd_grd = 2.0 * wn * xi;
	}
	else if(c == 'R')
	{
		Kp_ret = wn * wn;
		Kd_ret = 2.0 * wn * xi;
	}
	else if(c == 'L')
	{
		Kp_lim = wn * wn;
		Kd_lim = 2.0 * wn * xi;
	}
}
void Chz::SwingFootController::SetFootPD_deg2(double& Kp, double& Kd, double tp)
{
	const double pi = acos(-1.0);
	Kp = 2.0 * pi * pi / tp / tp;
	Kd = 2.0 * sqrt(2.0 * Kp);
}

Chz::SwingFootController::SwingFootController()
{
	rprop = 0.5;
	Ifconr = Ifconl = 0;
	ChzF(i, 0, 2) RCon[i] = LCon[i] = VectorXd::Zero(6);
	//SetFootPD('A', 0.06, 0.10, 0.10);
	SetFootPD_deg2(Kp_air_pos, Kd_air_pos, 0.2);
	SetFootPD_deg2(Kp_air_ang, Kd_air_ang, 0.09);
	//SetFootPD('G', 0.06, 0.15, 1.50);
	SetFootPD_deg2(Kp_grd, Kd_grd, 0.45);
	SetFootPD('R', 0.1, 0.2, 1.2);
	SetFootPD('L', 0.05, 0.05, 0.10);
}

void Chz::SwingFootController::SetProp(double rprop1)
{
	rprop = rprop1;
}

void Chz::SwingFootController::SetMode(char lr, int Ifcon1)
{
	if (lr == 'L') Ifconl = Ifcon1;
	else if (lr == 'R') Ifconr = Ifcon1;
	else return;
}

void Chz::SwingFootController::SetPoslim(double Poslim[6][2])
{
	ChzF(i, 0, 5) ChzF(j, 0, 1) Pos_lim[i][j] = Poslim[i][j];
}

void Chz::SwingFootController::Update(VectorXd CoM_d1, VectorXd RPos_d1, VectorXd LPos_d1, VectorXd CoM_r1, VectorXd Joints)
{
	const double T = 4e-3;
	CoM_d = CoM_d1;
	RPos_d = RPos_d1;
	LPos_d = LPos_d1;
	GetRealPos(CoM_r1, Joints);
	
	VectorXd Err_R = RPos_d - RPos_r, Err_L = LPos_d - LPos_r;
	
	for (int i = 0; i <= 5; i++) chz_log[0 + i] = CoM_r1(i);
	for (int i = 0; i <= 5; i++) chz_log[6 + i] = Err_R(i);
	for (int i = 0; i <= 5; i++) chz_log[12 + i] = Err_L(i);
	
	// cout << Err_R << Err_L << endl;
	// ChzF(i, 0, 2) Err_R(i) = min(max(Err_R(i), -4e-2), 4e-2);
	// ChzF(i, 3, 5) Err_R(i) = min(max(Err_R(i), -2e-1), 2e-1);
	// ChzF(i, 0, 2) Err_L(i) = min(max(Err_L(i), -4e-2), 4e-2);
	// ChzF(i, 3, 5) Err_L(i) = min(max(Err_L(i), -2e-1), 2e-1);
	
	if (!Ifconr)
	{ 
		ChzF(i, 0, 2)
			if (Err_R(i) > 2e-2) Err_R(i) -= 2e-2;
			else if (Err_R(i) < -2e-2) Err_R(i) += 2e-2;
			else Err_R(i) = 0.0;
		ChzF(i, 3, 5)
			if (Err_R(i) > 0.1) Err_R(i) -= 0.1;
			else if (Err_R(i) < -0.1) Err_R(i) += 0.1;
			else Err_R(i) = 0.0;
	}
	if (!Ifconl)
	{
		ChzF(i, 0, 2)
			if (Err_L(i) > 2e-2) Err_L(i) -= 2e-2;
			else if (Err_L(i) < -2e-2) Err_L(i) += 2e-2;
			else Err_L(i) = 0.0;
		ChzF(i, 3, 5)
			if (Err_L(i) > 0.1) Err_L(i) -= 0.1;
			else if (Err_L(i) < -0.1) Err_L(i) += 0.1;
			else Err_L(i) = 0.0;
	}
	
	if (Ifconr == Ifconl)
	{
		if (Err_L(1) - Err_R(1) < -3e-2)
		{
			double Err_avg = 0.5 * (Err_L(1) + Err_R(1));
			Err_L(1) = Err_avg + 1.5e-2;
			Err_R(2) = Err_avg - 1.5e-2;
		}
	}
	else if (Ifconl) Err_L(1) = max(Err_L(1), Err_R(1) - 3e-2);
	else if (Ifconr) Err_R(1) = min(Err_R(1), Err_L(1) + 3e-2);
	
	// Following Err
	if (Ifconr)
	{
		ChzF(i, 0, 2) RCon[2](i) = Kp_air_pos * Err_R(i) - Kd_air_pos * RCon[1](i);
		ChzF(i, 3, 5) RCon[2](i) = Kp_air_ang * Err_R(i) - Kd_air_ang * RCon[1](i);
	}
	else RCon[2] = Kp_grd * (0.0 * Err_R - RCon[0]) - Kd_grd * RCon[1];
	if (Ifconl)
	{
		ChzF(i, 0, 2) LCon[2](i) = Kp_air_pos * Err_L(i) - Kd_air_pos * LCon[1](i);
		ChzF(i, 3, 5) LCon[2](i) = Kp_air_ang * Err_L(i) - Kd_air_ang * LCon[1](i);
	}
	else LCon[2] = Kp_grd * (0.0 * Err_L - LCon[0]) - Kd_grd * LCon[1];
	// Return zero
	// RCon[2] += Kp_ret * (-RCon[0]) + Kd_ret * (-RCon[1]);
	// LCon[2] += Kp_ret * (-LCon[0]) + Kd_ret * (-LCon[1]);
	// Stay within limit
	ChzF(i, 0, 5) if(RCon[0](i) < Pos_lim[i][0])
		RCon[2](i) += Kd_lim * (Pos_lim[i][0] - RCon[0](i));
	ChzF(i, 0, 5) if(RCon[0](i) > Pos_lim[i][1])
		RCon[2](i) += Kd_lim * (Pos_lim[i][0] - RCon[0](i));
	ChzF(i, 0, 5) if(LCon[0](i) < Pos_lim[i][0])
		LCon[2](i) += Kd_lim * (Pos_lim[i][0] - LCon[0](i));
	ChzF(i, 0, 5) if(LCon[0](i) > Pos_lim[i][1])
		LCon[2](i) += Kd_lim * (Pos_lim[i][0] - LCon[0](i));
	
	RCon[1] += RCon[2] * T; RCon[0] += RCon[1] * T;
	LCon[1] += LCon[2] * T; LCon[0] += LCon[1] * T;

	chz_log[22 + 12] = -0.01 * Ifconr;
	chz_log[22 + 13] = -0.01 * Ifconl;
}
void Chz::SwingFootController::OutputCoMr(Vector3d& CoMr1)
{
	ChzF(i, 0, 2) CoMr1(i) = CoM_r(i);
}
void Chz::SwingFootController::Outputcon(VectorXd& RCon1, VectorXd& LCon1)
{
	RCon1 = RCon[0];
	LCon1 = LCon[0];
}

