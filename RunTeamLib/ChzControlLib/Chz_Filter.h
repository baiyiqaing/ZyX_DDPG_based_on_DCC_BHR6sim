#pragma once
#include "Chz_Base.h"

namespace Chz
{
	class LowPassFilter5
	{
		const double Wn;
		static const int n = 5;
		double a[n + 1], b[n + 1];
		double xin[n + 1], xout[n + 1];
	public:
		LowPassFilter5(double wn1);
		void Init(double xin1);
		double Update(double xinnew);
	};
}