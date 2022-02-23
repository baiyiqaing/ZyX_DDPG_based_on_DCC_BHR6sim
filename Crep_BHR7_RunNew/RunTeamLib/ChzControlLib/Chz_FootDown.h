#pragma once
#include "Chz_Base.h"

using namespace std;
namespace Chz
{
	class FootDownController
	{
		
		double Hup, Hdown, Hd;
		double Kp_up, Kd_up, Kp_down, Kd_down, Kp_return, Kd_return;

		double Kp_now, Kd_now;
		double Hcon[3];
		int Npause, Npause_max;

		char Mode;
	public:
		FootDownController(double hup1, double hdown1);
		void SetPD(char udr, double mp, double me, double ts);
		void SetMode(char mode1);
		double UpdateCon();
	};
}