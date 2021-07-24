#include "Chz_PerspectiveMapping.h"

Chz::PerspectiveMapping2d::PerspectiveMapping2d()
{
	H = Eigen::Matrix3d::Identity();
}

void Chz::PerspectiveMapping2d::CalH()
{
	Eigen::Matrix3d Ap, Aq, F;
	Eigen::Vector3d ab, cd;
	Ap << p[2](0) - p[0](0), p[1](0) - p[0](0), p[0](0),
		p[2](1) - p[0](1), p[1](1) - p[0](1), p[0](1),
		0.0, 0.0, 1.0;
	Aq << q[2](0) - q[0](0), q[1](0) - q[0](0), q[0](0),
		q[2](1) - q[0](1), q[1](1) - q[0](1), q[0](1),
		0.0, 0.0, 1.0;
	//std::cout << Ap << std::endl << Aq << std::endl;
	ab = Ap.inverse() * p[3];
	cd = Aq.inverse() * q[3];
	double a = ab(0), b = ab(1), c = cd(0), d = cd(1);
	double s = a + b - 1.0, t = c + d - 1.0;
	F << b * c * s, 0, 0,
		0, a* d* s, 0,
		b* (c * s - a * t), a* (d * s - b * t), a* b* t;
	H = Aq * F * Ap.inverse();
}

Eigen::Vector2d Chz::PerspectiveMapping2d::Getq(Eigen::Vector2d p)
{
	Eigen::Vector3d p_, q_;
	p_ << p(0), p(1), 1.0;
	q_ = H * p_;
	Eigen::Vector2d q;
	q(0) = q_(0) / q_(2);
	q(1) = q_(1) / q_(2);
	return q;
}

void Chz::PerspectiveMapping2d::Setpq(Eigen::Vector2d p1[], Eigen::Vector2d q1[])
{
	ChzF(i, 0, 3)
	{
		p[i] << p1[i](0), p1[i](1), 1.0;
		q[i] << q1[i](0), q1[i](1), 1.0;
	}
}
