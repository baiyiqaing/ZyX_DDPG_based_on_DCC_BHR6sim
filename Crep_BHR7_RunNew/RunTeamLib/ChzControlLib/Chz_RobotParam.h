#pragma once
#include "Chz_Base.h"
namespace Chz
{
	class RobotComponent
	{
	public:
		double l, lc, m, Ix, Iy, Iz;
		RobotComponent();
		RobotComponent(double l1, double lc1, double m1, double Ix1, double Iy1, double Iz1);
	};

	class RobotParam
	{
	public:
		const double CONTROL_PRID;
		int CompNum;
		std::map<std::string, RobotComponent> Comp;
		double waist_width;
		double lqian, lhou, lin, lout;
		double hpend;
		double h_ankle;
	public:
		RobotParam();
	};
}