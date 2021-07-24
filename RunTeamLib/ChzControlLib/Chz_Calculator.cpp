#include "Chz_Include.h"

Chz::Calculator::Calculator()
{
	Factorial[0] = 1;
	ChzF(i, 1, MaxDiscreteCalNum) Factorial[i] = Factorial[i - 1] * i;
	ChzF(i, 0, MaxDiscreteCalNum)
		ChzF(j, 0, MaxDiscreteCalNum)
			if(j <= i)
			{
				Permutation[i][j] = Factorial[i] / Factorial[j];
				Combination[i][j] = Factorial[i] / Factorial[j] / Factorial[i - j];
			}
}

double Chz::Calculator::dPow(double a, int b)
{
	if (b < 0) return 1.0 / dPow(a, -b);
	if (b == 0) return 1.0;
	double res = 1.0;
	while (b > 1)
	{
		if (b % 2) res *= a, b--;
		a *= a;
		b /= 2;
	}
	return res * a;
}

int Chz::Calculator::Pow_minus1(int b)
{
	if (b % 2)
		return -1;
	else
		return 1;
}

ll Chz::Calculator::Fact(int i)
{
	return Factorial[i];
}

ll Chz::Calculator::Perm(int m, int n)
{
	return Permutation[m][n];
}

ll Chz::Calculator::Comb(int m, int n)
{
	return Combination[m][n];
}

