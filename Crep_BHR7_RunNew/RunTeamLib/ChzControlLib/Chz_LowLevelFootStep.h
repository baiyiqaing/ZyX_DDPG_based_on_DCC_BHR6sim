#pragma once
#include "Chz_Base.h"
namespace Chz
{
	class LowLevelFootStepController
	{
		double RXCon[3], LXCon[3];
		char Rmode, Lmode;
		
		char xy;
		double RXdes, LXdes;

		double KpFoot_R, KdFoot_R;
		double KpFoot_L, KdFoot_L;

		void SetFootPD(char lr, double mp, double me, double ts);
	public:
		LowLevelFootStepController(char yz1);
		void Setmode(char lr, char mode1);
		void SetXdes(double RXdes1, double LXdes1);
		void UpdateXCon(double* RXCon1, double* LXCon1);
	};
}