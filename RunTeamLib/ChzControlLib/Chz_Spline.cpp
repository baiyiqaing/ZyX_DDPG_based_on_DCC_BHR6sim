#include "Chz_Calculator.h"
#include "Chz_Spline.h"
namespace Chz
{
	extern Calculator Calc;
}
void Chz::GenerateSpline6(double* b, double x0, double v0, double a0, double x2, double v2, double a2, double t2)
{
	b[0] = x0; b[1] = v0; b[2] = a0 / 2.0;
	b[3] = -(20.0 * x0 - 20.0 * x2 + 12.0 * t2 * v0 + 8.0 * t2 * v2 + 3.0 * a0 * t2 * t2 - a2 * t2 * t2) / (2.0 * Calc.dPow(t2, 3));
	b[4] = (30.0 * x0 - 30.0 * x2 + 16.0 * t2 * v0 + 14.0 * t2 * v2 + 3.0 * a0 * t2 * t2 - 2.0 * a2 * t2 * t2) / (2.0 * Calc.dPow(t2, 4));
	b[5] = -(12.0 * x0 - 12.0 * x2 + 6.0 * t2 * v0 + 6.0 * t2 * v2 + a0 * t2 * t2 - a2 * t2 * t2) / (2.0 * Calc.dPow(t2, 5));
}

void Chz::GenerateSpline7(double* a, double x0, double v0, double a0, double t1, double x1, double t2, double x2, double v2, double a2)
{
	a[0] = x0; a[1] = v0; a[2] = a0 / 2.0;
	a[3] = 1.0 / (t1 * t1 * t1) * 1.0 / (t2 * t2 * t2) * 1.0 / pow(t1 - t2, 3.0) * ((t1 * t1 * t1 * t1 * t1 * t1) * x0 * 2.0E+1 - (t2 * t2 * t2 * t2 * t2 * t2) * x0 * 2.0 - (t1 * t1 * t1 * t1 * t1 * t1) * x2 * 2.0E+1 + (t2 * t2 * t2 * t2 * t2 * t2) * x1 * 2.0 - t1 * (t2 * t2 * t2 * t2 * t2 * t2) * v0 * 2.0 + (t1 * t1 * t1 * t1 * t1 * t1) * t2 * v0 * 1.2E+1 + (t1 * t1 * t1 * t1 * t1 * t1) * t2 * v2 * 8.0 - (t1 * t1 * t1 * t1 * t1) * t2 * x0 * 4.8E+1 + (t1 * t1 * t1 * t1 * t1) * t2 * x2 * 4.8E+1 - a0 * (t1 * t1) * (t2 * t2 * t2 * t2 * t2 * t2) + a0 * (t1 * t1 * t1 * t1) * (t2 * t2 * t2 * t2) * 6.0 - a0 * (t1 * t1 * t1 * t1 * t1) * (t2 * t2 * t2) * 8.0 + a0 * (t1 * t1 * t1 * t1 * t1 * t1) * (t2 * t2) * 3.0 - a2 * (t1 * t1 * t1 * t1) * (t2 * t2 * t2 * t2) + a2 * (t1 * t1 * t1 * t1 * t1) * (t2 * t2 * t2) * 2.0 - a2 * (t1 * t1 * t1 * t1 * t1 * t1) * (t2 * t2) + (t1 * t1 * t1 * t1) * (t2 * t2 * t2) * v0 * 2.0E+1 - (t1 * t1 * t1 * t1 * t1) * (t2 * t2) * v0 * 3.0E+1 + (t1 * t1 * t1 * t1) * (t2 * t2 * t2) * v2 * 1.0E+1 - (t1 * t1 * t1 * t1 * t1) * (t2 * t2) * v2 * 1.8E+1 + (t1 * t1 * t1 * t1) * (t2 * t2) * x0 * 3.0E+1 - (t1 * t1 * t1 * t1) * (t2 * t2) * x2 * 3.0E+1) * (-1.0 / 2.0);
	a[4] = (1.0 / (t1 * t1 * t1) * 1.0 / (t2 * t2 * t2 * t2) * 1.0 / pow(t1 - t2, 3.0) * ((t1 * t1 * t1 * t1 * t1 * t1) * x0 * 3.0E+1 - (t2 * t2 * t2 * t2 * t2 * t2) * x0 * 6.0 - (t1 * t1 * t1 * t1 * t1 * t1) * x2 * 3.0E+1 + (t2 * t2 * t2 * t2 * t2 * t2) * x1 * 6.0 - t1 * (t2 * t2 * t2 * t2 * t2 * t2) * v0 * 6.0 + (t1 * t1 * t1 * t1 * t1 * t1) * t2 * v0 * 1.6E+1 + (t1 * t1 * t1 * t1 * t1 * t1) * t2 * v2 * 1.4E+1 - (t1 * t1 * t1 * t1 * t1) * t2 * x0 * 5.4E+1 + (t1 * t1 * t1 * t1 * t1) * t2 * x2 * 5.4E+1 - a0 * (t1 * t1) * (t2 * t2 * t2 * t2 * t2 * t2) * 3.0 + a0 * (t1 * t1 * t1) * (t2 * t2 * t2 * t2 * t2) * 6.0 - a0 * (t1 * t1 * t1 * t1 * t1) * (t2 * t2 * t2) * 6.0 + a0 * (t1 * t1 * t1 * t1 * t1 * t1) * (t2 * t2) * 3.0 - a2 * (t1 * t1 * t1) * (t2 * t2 * t2 * t2 * t2) + a2 * (t1 * t1 * t1 * t1 * t1) * (t2 * t2 * t2) * 3.0 - a2 * (t1 * t1 * t1 * t1 * t1 * t1) * (t2 * t2) * 2.0 + (t1 * t1 * t1) * (t2 * t2 * t2 * t2) * v0 * 2.0E+1 - (t1 * t1 * t1 * t1 * t1) * (t2 * t2) * v0 * 3.0E+1 + (t1 * t1 * t1) * (t2 * t2 * t2 * t2) * v2 * 1.0E+1 - (t1 * t1 * t1 * t1 * t1) * (t2 * t2) * v2 * 2.4E+1 + (t1 * t1 * t1) * (t2 * t2 * t2) * x0 * 3.0E+1 - (t1 * t1 * t1) * (t2 * t2 * t2) * x2 * 3.0E+1)) / 2.0;
	a[5] = 1.0 / (t1 * t1 * t1) * 1.0 / (t2 * t2 * t2 * t2 * t2) * 1.0 / pow(t1 - t2, 3.0) * ((t1 * t1 * t1 * t1 * t1 * t1) * x0 * 1.2E+1 - (t2 * t2 * t2 * t2 * t2 * t2) * x0 * 6.0 - (t1 * t1 * t1 * t1 * t1 * t1) * x2 * 1.2E+1 + (t2 * t2 * t2 * t2 * t2 * t2) * x1 * 6.0 - t1 * (t2 * t2 * t2 * t2 * t2 * t2) * v0 * 6.0 + (t1 * t1 * t1 * t1 * t1 * t1) * t2 * v0 * 6.0 + (t1 * t1 * t1 * t1 * t1 * t1) * t2 * v2 * 6.0 - a0 * (t1 * t1) * (t2 * t2 * t2 * t2 * t2 * t2) * 3.0 + a0 * (t1 * t1 * t1) * (t2 * t2 * t2 * t2 * t2) * 8.0 - a0 * (t1 * t1 * t1 * t1) * (t2 * t2 * t2 * t2) * 6.0 + a0 * (t1 * t1 * t1 * t1 * t1 * t1) * (t2 * t2) - a2 * (t1 * t1 * t1) * (t2 * t2 * t2 * t2 * t2) * 2.0 + a2 * (t1 * t1 * t1 * t1) * (t2 * t2 * t2 * t2) * 3.0 - a2 * (t1 * t1 * t1 * t1 * t1 * t1) * (t2 * t2) + (t1 * t1 * t1) * (t2 * t2 * t2 * t2) * v0 * 3.0E+1 - (t1 * t1 * t1 * t1) * (t2 * t2 * t2) * v0 * 3.0E+1 + (t1 * t1 * t1) * (t2 * t2 * t2 * t2) * v2 * 1.8E+1 - (t1 * t1 * t1 * t1) * (t2 * t2 * t2) * v2 * 2.4E+1 + (t1 * t1 * t1) * (t2 * t2 * t2) * x0 * 4.8E+1 - (t1 * t1 * t1 * t1) * (t2 * t2) * x0 * 5.4E+1 - (t1 * t1 * t1) * (t2 * t2 * t2) * x2 * 4.8E+1 + (t1 * t1 * t1 * t1) * (t2 * t2) * x2 * 5.4E+1) * (-1.0 / 2.0);
	a[6] = (1.0 / (t1 * t1 * t1) * 1.0 / (t2 * t2 * t2 * t2 * t2) * 1.0 / pow(t1 - t2, 3.0) * ((t1 * t1 * t1 * t1 * t1) * x0 * 1.2E+1 - (t2 * t2 * t2 * t2 * t2) * x0 * 2.0 - (t1 * t1 * t1 * t1 * t1) * x2 * 1.2E+1 + (t2 * t2 * t2 * t2 * t2) * x1 * 2.0 - t1 * (t2 * t2 * t2 * t2 * t2) * v0 * 2.0 + (t1 * t1 * t1 * t1 * t1) * t2 * v0 * 6.0 + (t1 * t1 * t1 * t1 * t1) * t2 * v2 * 6.0 - (t1 * t1 * t1 * t1) * t2 * x0 * 3.0E+1 + (t1 * t1 * t1 * t1) * t2 * x2 * 3.0E+1 - a0 * (t1 * t1) * (t2 * t2 * t2 * t2 * t2) + a0 * (t1 * t1 * t1) * (t2 * t2 * t2 * t2) * 3.0 - a0 * (t1 * t1 * t1 * t1) * (t2 * t2 * t2) * 3.0 + a0 * (t1 * t1 * t1 * t1 * t1) * (t2 * t2) - a2 * (t1 * t1 * t1) * (t2 * t2 * t2 * t2) + a2 * (t1 * t1 * t1 * t1) * (t2 * t2 * t2) * 2.0 - a2 * (t1 * t1 * t1 * t1 * t1) * (t2 * t2) + (t1 * t1 * t1) * (t2 * t2 * t2) * v0 * 1.2E+1 - (t1 * t1 * t1 * t1) * (t2 * t2) * v0 * 1.6E+1 + (t1 * t1 * t1) * (t2 * t2 * t2) * v2 * 8.0 - (t1 * t1 * t1 * t1) * (t2 * t2) * v2 * 1.4E+1 + (t1 * t1 * t1) * (t2 * t2) * x0 * 2.0E+1 - (t1 * t1 * t1) * (t2 * t2) * x2 * 2.0E+1)) / 2.0;

	return;
}

void Chz::GenerateSplineBezier(double* a, double* Bk, double* tk, int n)
{
	Eigen::VectorXd Bk_star(n - 5), as(n - 5);
	Eigen::MatrixXd M(n - 5, n - 5);
	ChzF(i, 0, n - 6)
	{
		Bk_star(i) = Bk[i];
		ChzF(j, 0, 2) Bk_star(i) -= Calc.Comb(n, j) * a[j] * Calc.dPow(tk[i], j) * Calc.dPow(1.0 - tk[i], n - j);
		ChzF(j, n - 2, n) Bk_star(i) -= Calc.Comb(n, j) * a[j] * Calc.dPow(tk[i], j) * Calc.dPow(1.0 - tk[i], n - j);
	}
	ChzF(i, 0, n - 6) ChzF(j, 0, n - 6)
		M(i, j) = Calc.Comb(n, j + 3) * Calc.dPow(tk[i], 3 + j) * Calc.dPow(1 - tk[i], n - 3 - j);
	as = M.inverse() * Bk_star;
	ChzF(i, 3, n - 3) a[i] = as(i - 3);
}

double Chz::GetLimitedValue(double x, double X1, double Y1, double X2, double Y2, double K)
{
	double sig = 1.0, y;
	if (abs(x) < eps) return x;
	if (x < -eps) sig = -1.0, x *= -1.0;
	if (x < X1) y = Y1 / X1 * x;
	else if (x > X2) y = K * (x - X2) + Y2;
	else
	{
		static double b[7], tempy[3];
		GenerateSpline6(b, Y1, Y1 / X1, 0.0, Y2, K, 0.0, X2 - X1);
		GetSpline6(tempy, b, x - X1);
		y = tempy[0];
	}
	return sig * y;
}

void Chz::GetSpline6(double* x, double* a, double t)
{
	x[0] = x[1] = x[2] = 0.0;
	ChzF(i, 0, 5)
	{
		x[0] += a[i] * Calc.dPow(t, i);
		if (i >= 1) x[1] += a[i] * i * Calc.dPow(t, i - 1);
		if (i >= 2) x[2] += a[i] * i * (i - 1) * Calc.dPow(t, i - 2);
	}
	return;
}

void Chz::GetSpline7(double* x, double* a, double t)
{
	x[0] = x[1] = x[2] = 0.0;
	ChzF(i, 0, 6)
	{
		x[0] += a[i] * Calc.dPow(t, i);
		if (i >= 1) x[1] += a[i] * i * Calc.dPow(t, i - 1);
		if (i >= 2) x[2] += a[i] * i * (i - 1) * Calc.dPow(t, i - 2);
	}
	return;
}

void Chz::GetSplineBezier(double* x, double* a, int n, double t, char mode)
{
	double ss[2][30];
	ChzF(i, 0, n) ss[0][i] = Calc.dPow(t, i), ss[1][i] = Calc.dPow(1 - t, i);
	double Cniai[30];
	ChzF(i, 0, n) Cniai[i] = Calc.Comb(n, i) * a[i];
	x[0] = 0.0;
	if (mode == 'a') x[1] = x[2] = 0.0;
	if (mode == 'j') x[1] = x[2] = x[3] = 0.0;
	ChzF(i, 0, n)
	{
		x[0] += Cniai[i] * ss[0][i] * ss[1][n - i];
		if (mode == 'a' || mode == 'j')
		{
			if (i >= 1) x[1] += Cniai[i] * i * ss[0][i - 1] * ss[1][n - i];
			if (i <= n - 1) x[1] -= Cniai[i] * ss[0][i] * (n - i) * ss[1][n - i - 1];

			if (i >= 2) x[2] += Cniai[i] * i * (i - 1) * ss[0][i - 2] * ss[1][n - i];
			if (i >= 1 && i <= n - 1) x[2] -= 2.0 * Cniai[i] * i * ss[0][i - 1] * (n - i) * ss[1][n - i - 1];
			if (i <= n - 2) x[2] += Cniai[i] * ss[0][i] * (n - i) * (n - i - 1) * ss[1][n - i - 2];
		}
		if (mode == 'j')
		{
			if (i >= 3) x[3] += Cniai[i] * i * (i - 1) * (i - 2) * ss[0][i - 3] * ss[1][n - i];
			if (i >= 2 && i <= n - 1) x[3] -= 3.0 * Cniai[i] * i * (i - 1) * ss[0][i - 2] * (n - i) * ss[1][n - i - 1];
			if (i >= 1 && i <= n - 2) x[3] += 3.0 * Cniai[i] * i * ss[0][i - 1] * (n - i) * (n - i - 1) * ss[1][n - i - 2];
			if (i <= n - 3) x[3] -= Cniai[i] * ss[0][i] * (n - i) * (n - i - 1) * (n - i - 2) * ss[1][n - i - 3];
		}
	}
}

