#include "Chz_WaistComp.h"

Chz::WaistComp::WaistComp()
{
	Waistcon = Waistcon_bak = Waistcon_des = 0.0;
	Mode = 'P';
	Compang = 0.0;
	Intenum_max = 50;
	Intenum = 0;
	GenerateSpline6(Inte_a, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0);
}

void Chz::WaistComp::SetParam(double compang1, int intenum_max1)
{
	Compang = compang1;
	Intenum_max = intenum_max1;
}

void Chz::WaistComp::SetMode(char mode1)
{
	if (mode1 != 'P' && Mode != 'P') return;
	Mode = mode1;
	Waistcon_bak = Waistcon;
	Intenum = 0;
}

double Chz::WaistComp::UpdateCon()
{
	if (Mode == 'A') Waistcon_des = Compang;
	else if (Mode == 'D') Waistcon_des = -Compang;
	else if (Mode == 'Z') Waistcon_des = 0.0;
	else
	{
		Waistcon_des = 0.0;
		Waistcon_bak = Waistcon;
		return Waistcon;
	}
	Intenum++;
	if (Intenum > Intenum_max) 
		SetMode('P');
	else
	{
		double t = 1.0 * Intenum / Intenum_max;
		double x[3];
		GetSpline6(x, Inte_a, t);
		Waistcon = Waistcon_bak + x[0] * (Waistcon_des - Waistcon_bak);
	}
	return Waistcon;
}
