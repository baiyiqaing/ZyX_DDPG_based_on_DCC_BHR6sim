#include "Chz_LowLevelFootStep.h"
extern "C" double chz_log[20];

void Chz::LowLevelFootStepController::SetFootPD(char lr, double mp, double me, double ts)
{
	const double pi = acos(-1.0);
	double wn, xi;
	double lnmp = log(mp);
	xi = sqrt(lnmp * lnmp / (pi * pi + lnmp * lnmp));
	wn = -1.0 / (ts * xi) * log(me * sqrt(1 - xi * xi));
	if (lr == 'R')
	{
		KpFoot_R = wn * wn;
		KdFoot_R = 2.0 * wn * xi;
	}
	if (lr == 'L')
	{
		KpFoot_L = wn * wn;
		KdFoot_L = 2.0 * wn * xi;
	}
}

Chz::LowLevelFootStepController::LowLevelFootStepController(char xy1)
{
	xy = xy1;
	ChzF(i, 0, 2) LXCon[i] = RXCon[i] = 0.0;
	Setmode('R', 'P');
	Setmode('L', 'P');
}

void Chz::LowLevelFootStepController::Setmode(char lr, char mode1)
{
	if(lr == 'R')
	{
		if(Rmode == mode1) return;
		Rmode = mode1;
		if (Rmode == 'P') SetFootPD('R', 0.01, 0.01, 0.05), RXdes = RXCon[0];
		//if (Rmode == 'P') SetFootPD('R', 0.01, 0.15, 1.60), RXdes = 0.0;
		if (Rmode == 'C') SetFootPD('R', 0.05, 0.10, 0.30);
	}
	if(lr == 'L')
	{
		if(Lmode == mode1) return;
		Lmode = mode1;
		if (Lmode == 'P') SetFootPD('L', 0.01, 0.01, 0.05), LXdes = LXCon[0];
		//if (Lmode == 'P') SetFootPD('L', 0.01, 0.15, 1.60), LXdes = 0.0;
		if (Lmode == 'C') SetFootPD('L', 0.05, 0.10, 0.30);
	}
}

void Chz::LowLevelFootStepController::SetXdes(double RXdes1, double LXdes1)
{
	const double coll_limit = 0.16 - 0.055 * 2.0 - 0.01;
	if(Rmode == 'C') RXdes = RXdes1; 
	if(Lmode == 'C') LXdes = LXdes1;
	RXdes = std::min(RXdes, 0.05);
	RXdes = std::max(RXdes, -0.05);
	LXdes = std::min(LXdes, 0.05);
	LXdes = std::max(LXdes, -0.05);
	if(xy == 'X')
	{
		if (Rmode == 'C' && Lmode == 'C')
		{
			double temp = RXdes - LXdes + coll_limit;
			if (temp < 0) RXdes += -temp / 2.0, LXdes -= -temp / 2.0;
		}
		else if (Lmode == 'C')
			LXdes = std::min(LXdes, RXdes + coll_limit);
		else if(Rmode == 'C')
			RXdes = std::max(RXdes, LXdes - coll_limit);
	}
}

void Chz::LowLevelFootStepController::UpdateXCon(double* RXCon1, double* LXCon1)
{
	static double T = 4e-3;
	RXCon[2] = -KpFoot_R * (RXCon[0] - RXdes) - KdFoot_R * RXCon[1];
	LXCon[2] = -KpFoot_L * (LXCon[0] - LXdes) - KdFoot_L * LXCon[1];

	RXCon[1] += RXCon[2] * T; RXCon[0] += RXCon[1] * T;
	LXCon[1] += LXCon[2] * T; LXCon[0] += LXCon[1] * T;

	*RXCon1 = RXCon[0];
	*LXCon1 = LXCon[0];
	
	// chz_log[12] = Rmode == 'C'? 1.0: 0.0;
	// chz_log[13] = Lmode == 'C'? 1.0: 0.0;
}
