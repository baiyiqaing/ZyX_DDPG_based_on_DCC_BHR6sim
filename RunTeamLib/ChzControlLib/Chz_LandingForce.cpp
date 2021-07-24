#include "Chz_LandingForce.h"
#include "Chz_Spline.h"

//extern "C"
//{
	//extern double chz_log[20];
//}
Chz::LandingForceController::LandingForceController(double E, double fmin1, double fmin2, double x1, double y1, double x2, double y2, double k2) : E0(E), fmin_mode(fmin1), fmin_con(fmin2), Inte_num_max(50), X1(x1), Y1(y1), X2(x2), Y2(y2), K2(k2)
{
	mode = "WAIT";
}
void Chz::LandingForceController::SetMode(std::string s)
{
	if(mode == s)
		return;
	if (s == "WAIT") 
	{ 
		mode = "WAIT";
		W = E0;
		z1 = z2;
		z1_last = z1;
		return; 
	}
	if (s == "PO") { mode = "PO"; return; }
	if (s == "PC") { mode = "PC"; return; }
	if (s == "INTE")
	{
		mode = "INTE";
		Inte_num = 0;
		delz_inte = zcon_real;
	}
}

void Chz::LandingForceController::Update(double f, double z2next)
{
	// chz_log[6] = f;
	// if(mode == "WAIT") chz_log[7] = 0.0;
	// else if(mode == "PO") chz_log[7] = 1.0;
	// else if(mode == "PC") chz_log[7] = 2.0;
	// else if(mode == "INTE") chz_log[7] = 3.0;
	
	f = std::max(f, 0.01);
	if (mode == "WAIT")
	{
		z2 = z2next;
		z1_last = z1;
		z1 = z2;
		zcon_real = 0.0;
		return;
	}
	if (mode == "PO")
	{
		z2 = z2next;
		z1_last = z1;
		z1 = z2;
		zcon_real = 0.0;
		if(f > fmin_mode) SetMode("PC");
		return;
	}
	if (mode == "PC")
	{
		if(f < fmin_con) f = 0.01;
		if (abs(z2next - z2) < eps && abs(z2next) < eps) { SetMode("INTE"); return; }
		double delz1 = z1 - z1_last;
		double delz2_1 = z2next - z2;
		W += f * delz1;
		double Wo_1 = W + f * delz2_1;
		double delz_pc;
		if (Wo_1 < 0) delz_pc = std::min(- Wo_1 / f, 0.4e-3);
		else delz_pc = 0.0;
		double delz1_1 = delz2_1 + delz_pc;
		double z1_next = z1 + delz1_1;
		//printf("f = %lf, z1 = %lf, z1_next = %lf, z2 = %lf\n, z2_next = %lf\n", f, z1, z1_next, z2, z2next);
		z1_last = z1;
		z2 = z2next;
		z1 = z1_next;
		
		//zcon_real = z1 - z2;
		zcon_real = GetLimitedValue(z1 - z2, X1, Y1, X2, Y2, K2);
		
		return;
	}
	if (mode == "INTE")
	{
		double tempt = 1.0 * Inte_num / (1.0 * Inte_num_max);
		static double a[7], x[3];
		Chz::GenerateSpline6(a, delz_inte, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
		Chz::GetSpline6(x, a, tempt);

		z2 = z2next;
		z1_last = z1;
		z1 = x[0] + z2;
		zcon_real = z1 - z2;
		Inte_num++;
		if (Inte_num > Inte_num_max) SetMode("WAIT");
	}
}

double Chz::LandingForceController::Getdelz()
{
	return zcon_real;
}
