#pragma once
#include "Chz_Base.h"

namespace Chz
{
	const int MaxDiscreteCalNum = 20;
	class Calculator
	{
		ll Factorial[MaxDiscreteCalNum + 1];
		ll Permutation[MaxDiscreteCalNum + 1][MaxDiscreteCalNum + 1];
		ll Combination[MaxDiscreteCalNum + 1][MaxDiscreteCalNum + 1];
	public:
		Calculator();
		double dPow(double a, int b);
		int Pow_minus1(int b);

		ll Fact(int i);
		ll Perm(int m, int n);
		ll Comb(int m, int n);
	};
}