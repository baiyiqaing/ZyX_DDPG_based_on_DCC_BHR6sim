#include "Chz_FootDown.h"
extern "C" double chz_log[20];

Chz::FootDownController::FootDownController(double hup1, double hdown1)
{
	Hup = hup1;
	Hdown = hdown1;
	Hd = 0.0;
	SetPD('U', 0.01, 0.10, 0.30);
	SetPD('D', 0.01, 0.01, 0.10);
	SetPD('R', 0.01, 0.10, 0.30);
	Hcon[0] = Hcon[1] = Hcon[2] = 0.0;
	Npause_max = 15;
	SetMode('R');
}

void Chz::FootDownController::SetPD(char udr, double mp, double me, double ts)
{
	const double pi = acos(-1.0);
	double wn, xi;
	double lnmp = log(mp);
	xi = sqrt(lnmp * lnmp / (pi * pi + lnmp * lnmp));
	wn = -1.0 / (ts * xi) * log(me * sqrt(1 - xi * xi));
	if (udr == 'U')
	{
		Kp_up = wn * wn;
		Kd_up = 2.0 * wn * xi;
	}
	if (udr == 'D')
	{
		Kp_down = wn * wn;
		Kd_down = 2.0 * wn * xi;
	}
	if (udr == 'R')
	{
		Kp_return = wn * wn;
		Kd_return = 2.0 * wn * xi;
	}
}

void Chz::FootDownController::SetMode(char mode1)
{
	if (mode1 == Mode) return;
	if (Mode == 'D' && mode1 == 'U') return;
	if (Mode == 'P' && mode1 != 'R') return;
	if (Mode == 'R' && mode1 != 'U') return;
	
	if (mode1 == 'U')
	{
		Kp_now = Kp_up; Kd_now = Kd_up;
		Hd = Hup;
	}
	if (mode1 == 'D')
	{
		Kp_now = Kp_down; Kd_now = Kd_down;
		Hd = Hdown;
	}
	if (mode1 == 'P')
	{
		Npause = Npause_max;
		Kp_now = Kp_down; Kd_now = Kd_down;
		Kp_now *= 0.5;
		Kd_now *= 2.0;
		Hd = Hcon[0];
	}
	if (mode1 == 'R')
	{
		Kp_now = Kp_return; Kd_now = Kd_return;
		Hd = 0.0;
	}
	Mode = mode1;
}

double Chz::FootDownController::UpdateCon()
{
	// if(Mode == 'U')
		// chz_log[19] = 1;
	// else if(Mode == 'D')
		// chz_log[19] = 2;
	// else if(Mode == 'P')
		// chz_log[19] = 3;
	// else if(Mode == 'R')
		// chz_log[19] = 0;
	// else chz_log[19] = -1;
	
	static double T = 4e-3;
	Hcon[2] = Kp_now * (Hd - Hcon[0]) + Kd_now * (0.0 - Hcon[1]);
	Hcon[1] += Hcon[2] * T;
	Hcon[0] += Hcon[1] * T;
	if (Mode == 'P')
	{
		Npause--;
		if (Npause == 0) SetMode('R');
	}
	return Hcon[0];
}

