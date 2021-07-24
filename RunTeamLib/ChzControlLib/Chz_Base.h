#pragma once
#define _CRT_SECURE_NO_WARNINGS

#include <cmath>
#include <cstdlib>
#include <algorithm>
#include <iostream>
#include <map>
#include <random>
#include <string>

//#define __chz_eigen

#ifdef __chz_eigen
#include "Chz_Eigen.h"
#define Eigen ChzEigen
#else
#include "../Eigen/Dense"
#endif

#define ChzF(a, b, c) for(int a = (b); a <= (c); a++)
typedef long long ll;

namespace Chz
{
	const double inf = 1e20;
	const double eps = 1e-7;
	const double pi = acos(-1.0);
	const double e = exp(1.0);
	const double g = 9.8;
	const double mu = 0.6;
}

namespace Chz
{
	void FootStep_Test();
	void Kinematics_Test();
	void PerspectiveMapping_Test();
}