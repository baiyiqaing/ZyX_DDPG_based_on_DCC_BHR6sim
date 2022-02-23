#pragma once
#include "Chz_Base.h"

namespace Chz
{
	void GenerateSpline6(double* b, double x0, double v0, double a0, double x2, double v2, double a2, double t2);
	void GenerateSpline7(double* a, double x0, double v0, double a0, double t1, double x1, double t2, double x2, double v2, double a2);
	void GenerateSplineBezier(double* a, double* xk, double* tk, int n);

	//y = Y1 / X1 * x (x < X1); y = K * (x - X2) + Y2 (x > X2); y = GetSpline6(*) (X1 <= x <= X2);
	double GetLimitedValue(double x, double X1, double Y1, double X2, double Y2, double K);

	void GetSpline6(double* x, double* a, double t);
	void GetSpline7(double* x, double* a, double t);
	void GetSplineBezier(double* x, double* a, int n, double t, char mode);
}