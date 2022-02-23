#include "Chz_Calculator.h"
#include "Chz_Footstep.h"
#include "Chz_RobotParam.h"
#include "Chz_Spline.h"
namespace Chz
{
	extern Calculator Calc;
}

Chz::FootstepController::FootstepController(RobotParam* r)
{
	Robot = r;
	h = Robot->hpend;
	w = sqrt(g / h);
	q_u = 2.0;
	Kp_Foot = 10.0; Kd_Foot = 30.0; Ka_Foot = 0.4;

	LFoot_delx = Eigen::Vector3d::Zero();
	RFoot_delx = Eigen::Vector3d::Zero();
	CoM_x = Eigen::Vector2d::Zero();
}

void Chz::FootstepController::OutputCon(double& RCon, double& LCon)
{
	RCon = RFoot_delx(0); LCon = LFoot_delx(0);
}

void Chz::FootstepController::SetInitDownpos(double down)
{
	downpos = down;
}

void Chz::FootstepController::SetInitCoM(Eigen::Vector2d comini)
{
	CoM_x = comini;
}

void Chz::FootstepController::SetPeriod(double ttotal, double tdown)
{
	T = ttotal;
	T_down = tdown;
}

void Chz::FootstepController::SetStaFoot(bool i, double Sta_x, double lift)
{
	If_Left = i;
	u_now = Sta_x;
	liftpos = lift;
}

void Chz::FootstepController::SetXid(const double xi_d)
{
	xid = xi_d;
}

void Chz::FootstepController::UpdateCon(Eigen::Vector2d CoM_xnew)
{
	t_now += Robot->CONTROL_PRID;
	UpdateCoMPos(CoM_xnew);
	Updatexxi0(Eigen::VectorXd::Zero(1));
	Updateu0(Eigen::VectorXd::Zero(1));
	UpdateFootPos();
}

void Chz::FootstepController::UpdateSup(double down)
{
	t_now = 0.0;
	if (If_Left)
		u_now = downpos + RFoot_delx(0), liftpos = LFoot_delx(0);
	else
		u_now = downpos + LFoot_delx(0), liftpos = RFoot_delx(0);
	downpos = down;
	If_Left = !If_Left;
}

void Chz::FootstepController::UpdateCoMPos(Eigen::Vector2d CoM_new)
{
	CoM_x = CoM_new;
	x_now(0) = CoM_x(0);
	x_now(1) = CoM_x(1);
	xi_now = x_now(0) + 1 / w * x_now(1);
}

double Chz::FootstepController::XPredict(double t_pre, Eigen::VectorXd y)
{
	int n = (int)(y.size() - 1);
	double exp1 = exp(w * t_pre), exp2 = 1.0 / exp1;
	double xres = u_now - 0.5 * exp1 * (u_now - x_now(1) / w - x_now(0)) + exp2 * (u_now + x_now(1) / w - x_now(0));
	ChzF(i, 0, n)
	{
		double resi = -0.5 * y(i) * Calc.Fact(i) * Calc.dPow(w, -i) * (exp1 + exp2 * Calc.Pow_minus1(i));
		ChzF(j, i, n)
			if ((j - i) % 2 == 0)
				resi += Calc.dPow(t_pre, i) * y(i) * Calc.Perm(j, i) / Calc.dPow(w, j - i);
		xres += resi;
	}
	return xres;
}

double Chz::FootstepController::XiPredict(double t_pre, Eigen::VectorXd y)
{
	int n = (int)(y.size() - 1);
	double exp1 = exp(w * t_pre), exp2 = 1.0 / exp1;
	double xires = exp1 * (xi_now - u_now) + u_now;
	ChzF(i, 0, n)
	{
		double resi = -exp1 * y(i) * Calc.Fact(i) * Calc.dPow(w, -i);
		ChzF(j, i, n)
			resi += Calc.dPow(t_pre, i) * y(i) * Calc.Perm(j, i) / Calc.dPow(w, j - i);
		xires += resi;
	}
	return xires;
}

void Chz::FootstepController::UpdateFootPos()
{
	Eigen::Vector3d* foots[2];
	if (If_Left) foots[0] = &RFoot_delx, foots[1] = &LFoot_delx;
	else foots[1] = &RFoot_delx, foots[0] = &LFoot_delx;

	double xd[3], a[7];
	double con_val = u0 - downpos;

	con_val = std::min(con_val, x0(0) - downpos + 0.20);
	con_val = std::max(con_val, x0(0) - downpos - 0.20);
	if (If_Left) con_val = std::min(con_val, u_now - downpos - 0.08);
	else con_val = std::max(con_val, u_now - downpos + 0.08);

	GenerateSpline6(a, liftpos, 0.0, 0.0, con_val, 0.0, 0.0, T_down);
	GetSpline6(xd, a, std::min(t_now, T_down));

	(*foots[1])(2) = Kp_Foot * (u_now - (*foots[1])(0)) + Kd_Foot * (0.0 - (*foots[1])(1));

	(*foots[0])(2) = Kp_Foot * (xd[0] - (*foots[0])(0)) + Kd_Foot * (xd[1] - (*foots[0])(1)) + Ka_Foot * xd[2];

	ChzF(i, 0, 1)
	{
		(*foots[i])(1) += (*foots[i])(2) * Robot->CONTROL_PRID;
		//(*foots[i])(0) += (*foots[i])(1) * Robot->CONTROL_PRID;
	}

	(*foots[0])(0) = xd[0];
	//(*foots[1])(0) = 0.0;
}

void Chz::FootstepController::Updateu0(Eigen::VectorXd y)
{
	Updatexxi0(y);
	double f1 = 0.0, f2 = 0.0, exp1 = exp(w * T_down);
	int n = (int)y.size() - 1;
	ChzF(i, 0, n)
	{
		f1 += Calc.Fact(i) * y(i) * Calc.dPow(w, -i);
		ChzF(j, i, n)
			f2 += Calc.dPow(T_down, i) * y(j) * Calc.Perm(j, i) / Calc.dPow(w, j - i);
	}
	double a = 1.0 - exp1, b = exp1 * (xi0 - f1) + f2;
	u0 = (q_u * downpos - a * (b - xid)) / (a * a + q_u);
}

void Chz::FootstepController::Updatexxi0(Eigen::VectorXd y)
{
	x0(0) = XPredict(T - t_now, y);
	xi0 = XiPredict(T - t_now, y);
	x0(1) = (xi0 - x0(0)) * w;
}
