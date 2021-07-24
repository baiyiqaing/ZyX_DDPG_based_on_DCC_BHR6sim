#pragma once
#include "Chz_Base.h"

namespace Chz
{
	class PerspectiveMapping2d
	{
		Eigen::Vector3d p[4], q[4];
		Eigen::Matrix3d H;
	public:
		PerspectiveMapping2d();
		void CalH();
		Eigen::Vector2d Getq(Eigen::Vector2d p);
		void Setpq(Eigen::Vector2d p1[], Eigen::Vector2d q1[]);
	};
}