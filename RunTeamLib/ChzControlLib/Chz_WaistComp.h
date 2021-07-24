#pragma once
#include "Chz_Base.h"
#include "Chz_Spline.h"
namespace Chz
{
	class WaistComp
	{
		double Waistcon, Waistcon_bak, Waistcon_des;
		char Mode;
		double Compang;
		double Inte_a[6];
		int Intenum_max, Intenum;
	public:
		WaistComp();
		void SetParam(double compang1, int intenum_max1);
		void SetMode(char mode1);
		double UpdateCon();
	};
}
