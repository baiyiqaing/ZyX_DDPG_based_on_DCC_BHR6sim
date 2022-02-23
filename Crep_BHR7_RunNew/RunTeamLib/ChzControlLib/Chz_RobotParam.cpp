#include "Chz_RobotParam.h"

Chz::RobotComponent::RobotComponent(){}

Chz::RobotComponent::RobotComponent(double l1, double lc1, double m1, double Ix1, double Iy1, double Iz1) : l(l1), lc(lc1), m(m1), Ix(Ix1), Iy(Iy1), Iz(Iz1) {}


Chz::RobotParam::RobotParam() :CONTROL_PRID(4e-3)
{
	CompNum = 5;
	Comp["Body"] = RobotComponent(0.44, 0.34, 14.0, 0.2548, 0.49, 0.0);
	Comp["LeftThigh"] = RobotComponent(0.32, 0.118, 9.0, 0.0978, 0.0978, 0.0);
	Comp["RightThigh"] = RobotComponent(0.32, 0.118, 9.0, 0.0978, 0.0978, 0.0);
	Comp["LeftShank"] = RobotComponent(0.32, 0.264, 2.0, 0.015831, 0.015831, 0.0);
	Comp["RightShank"] = RobotComponent(0.32, 0.264, 2.0, 0.015831, 0.015831, 0.0);
	waist_width = 0.16;
	hpend = 0.8;
	lqian = 0.135;
	lhou = 0.085;
	lin = 0.05;
	lout = 0.09;
	h_ankle = 0.112;
}

