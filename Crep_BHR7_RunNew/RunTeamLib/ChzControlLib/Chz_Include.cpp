#include "Chz_Include.h"

namespace Chz
{
	Calculator Calc;
	RobotParam Robot;
	FootstepController FTCon(&Robot);
	Kinematics Kinematic_Calc(&Robot);

	LandingForceController LandingFCon_LZ(0.01, 50.0, 30.0, 6e-3, 6e-3, 10e-3, 9e-3, 0.0);
	LandingForceController LandingFCon_RZ(0.01, 50.0, 30.0, 6e-3, 6e-3, 10e-3, 9e-3, 0.0);
	LandingForceController LandingFCon_LY(0.01, 40.0, 20.0, 4e-3, 4e-3, 7e-3, 6e-3, 0.0);
	LandingForceController LandingFCon_RY(0.01, 50.0, 20.0, 4e-3, 4e-3, 7e-3, 6e-3, 0.0);
	LandingForceController LandingFCon_LP(0.01, 2.5, 2.0, 1e-2, 1e-2, 3e-2, 2.5e-2, 0.0);
	LandingForceController LandingFCon_RP(0.01, 2.5, 2.0, 1e-2, 1e-2, 3e-2, 2.5e-2, 0.0);

	FootDownController FootDown_L(8e-3, -10e-3), FootDown_R(8e-3, -10e-3);
	
	PerspectiveMapping2d ZMPMapping_L;
	PerspectiveMapping2d ZMPMapping_R;
	
	LowPassFilter5 LPFilters[10] = 
	{LowPassFilter5(0.2), LowPassFilter5(0.2), LowPassFilter5(20.0), LowPassFilter5(20.0), LowPassFilter5(20.0),
	LowPassFilter5(20.0), LowPassFilter5(1.0), LowPassFilter5(1.0), LowPassFilter5(0.2), LowPassFilter5(0.2)};
	
	LowLevelFootStepController FootConX('X'), FootConY('Y');
	WaistComp Qbodyr, Qbodyl;
	
	SwingFootController SwingFootCon;
}
 