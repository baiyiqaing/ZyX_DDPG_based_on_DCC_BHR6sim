#pragma once
#include "Chz_Base.h"
namespace Chz
{
	class LandingForceController
	{
		const double E0;
		const double fmin_mode;
		const double fmin_con;
		
		double W, Wo;
		double z1, z1_last, z2;
		double zcon_real;
		double X1, Y1, X2, Y2, K2;//limit
		
		std::string mode;

		int Inte_num;
		const int Inte_num_max;
		double delz_inte;
	public:
		LandingForceController(double E, double fmin1, double fmin2, double x1, double y1, double x2, double y2, double k2);
		void SetMode(std::string s);
		void Update(double f, double z2next);
		double Getdelz();
	};
}