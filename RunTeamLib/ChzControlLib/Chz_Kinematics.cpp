#include "Chz_Base.h"
#include "Chz_Kinematics.h"
#include "Chz_RobotParam.h"

double Chz::Kinematics::lee_aprox(double value, double standard)
{
	if (abs(value - standard) < lee_MIN_ERROR)
	{
		if (standard < 0) return (standard + lee_MIN_ERROR);
		else if (standard == 0) return (standard);
		else return (standard - lee_MIN_ERROR);
	}
	else return value;
}

double Chz::Kinematics::lee_aprox_all(double value)
{
	value = lee_aprox(value, 1.0);
	value = lee_aprox(value, 0.0);
	value = lee_aprox(value, -1.0);
	return value;
}

int Chz::Kinematics::lee_res_choose(const double* ang, const double* ref, const int joint_num, const int res_num)
{
	int op_num = -1;
	int i1, i2;
	double dif = 0.0;
	double dif_min = 9999.9;
	for (i1 = 0; i1 < res_num; i1++)
	{
		dif = 0.0;
		for (i2 = 0; i2 < joint_num; i2++)
		{
			dif += abs(ref[i2] - ang[i2 * res_num + i1]);
		}

		if (dif_min > dif)
		{
			dif_min = dif;
			op_num = i1;
		}
	}
	return op_num;
}

Matrix3d Chz::Kinematics::RotfromTh(double th, char axis)
{
	Matrix3d Mres = Matrix3d::Identity();
	if (axis == 'X')
		Mres << 1.0, 0.0, 0.0, 0.0, cos(th), -sin(th), 0.0, sin(th), cos(th);
	else if (axis == 'Y')
		Mres << cos(th), 0.0, sin(th), 0.0, 1.0, 0.0, -sin(th), 0.0, cos(th);
	else if (axis == 'Z')
		Mres << cos(th), -sin(th), 0.0, sin(th), cos(th), 0.0, 0.0, 0.0, 1.0;
	return Mres;
}

Matrix3d Chz::Kinematics::RotfromEuler(Vector3d Euler)
{
	return RotfromTh(Euler(2), 'Z') * RotfromTh(Euler(1), 'Y') * RotfromTh(Euler(0), 'X');
}

Chz::Kinematics::Kinematics(RobotParam* robot) :Robot(robot) 
{
	lee_MIN_ERROR = 1e-6;
}

void Chz::Kinematics::FK_ChunkfromFoot(Eigen::VectorXd& CoM, Eigen::VectorXd Foot, Eigen::VectorXd Joints, const char c)
{
	double l1, l2, q1, q2, q3, q4, q5, q6, thx_foot, thy_foot, thz_foot, p_waist;
	thx_foot = Foot(3); thy_foot = Foot(4); thz_foot = Foot(5);
	if (c == 'L')
	{
		l1 = Robot->Comp["LeftThigh"].l; l2 = Robot->Comp["LeftShank"].l;
		p_waist = 0.5 * Robot->waist_width;
		q1 = Joints(6); q2 = Joints(7); q3 = Joints(8); q4 = Joints(9); q5 = Joints(10); q6 = Joints(11);
	}
	else if (c == 'R')
	{
		l1 = Robot->Comp["RightThigh"].l; l2 = Robot->Comp["RightShank"].l;
		p_waist = -0.5 * Robot->waist_width;
		q1 = Joints(0); q2 = Joints(1); q3 = Joints(2); q4 = Joints(3); q5 = Joints(4); q6 = Joints(5);
	}
	else return;
	q3 = -q3; q4 = -q4; q5 = -q5;

	double r11, r21, r31, r32, r33;

	double sinq1, sinq2, sinq3, sinq4, sinq5, sinq6, sinthx_foot, sinthy_foot, sinthz_foot;
	double cosq1, cosq2, cosq3, cosq4, cosq5, cosq6, costhx_foot, costhy_foot, costhz_foot;

	sinq1 = sin(q1); sinq2 = sin(q2); sinq3 = sin(q3); sinq4 = sin(q4); sinq5 = sin(q5); sinq6 = sin(q6);
	cosq1 = cos(q1); cosq2 = cos(q2); cosq3 = cos(q3); cosq4 = cos(q4); cosq5 = cos(q5); cosq6 = cos(q6);

	sinthx_foot = sin(thx_foot); sinthy_foot = sin(thy_foot); sinthz_foot = sin(thz_foot);
	costhx_foot = cos(thx_foot); costhy_foot = cos(thy_foot); costhz_foot = cos(thz_foot);

	double temp1 = cosq3 * sinq1 + cosq1 * sinq2 * sinq3;
	double temp2 = cosq1 * cosq3 - sinq1 * sinq2 * sinq3;
	double temp3 = sinq1 * sinq3 - cosq1 * cosq3 * sinq2;
	double temp4 = cosq1 * sinq3 + cosq3 * sinq1 * sinq2;
	double temp5 = cosq2 * cosq3 * cosq4 - cosq2 * sinq3 * sinq4;
	double temp6 = cosq2 * cosq3 * sinq4 + cosq2 * cosq4 * sinq3;
	double temp7 = cosq4 * temp3 + sinq4 * temp1;
	double temp8 = cosq4 * temp1 - sinq4 * temp3;
	double temp9 = -p_waist + l1 * temp3 + l2 * temp7;
	double temp10 = l2 * temp5 + l1 * cosq2 * cosq3;
	double temp11 = cosq4 * temp4 + sinq4 * temp2;
	double temp12 = l1 * temp4 + l2 * temp11;
	double temp13 = cosq4 * temp2 - sinq4 * temp4;
	double temp14 = cosq5 * temp6 + sinq5 * temp5;
	double temp15 = sinq5 * temp11 - cosq5 * temp13;
	double temp16 = cosq5 * cosq6 * temp7 + cosq6 * sinq5 * temp8 - cosq1 * cosq2 * sinq6;
	double temp17 = sinq2 * sinq6 - cosq5 * cosq6 * temp5 + cosq6 * sinq5 * temp6;
	double temp18 = cosq5 * cosq6 * temp11 + cosq6 * sinq5 * temp13 + cosq2 * sinq1 * sinq6;
	double temp19 = temp16 * temp9 - temp10 * temp17 + temp12 * temp18;
	double temp20 = cosq5 * sinq6 * temp7 + cosq1 * cosq2 * cosq6 + sinq5 * sinq6 * temp8;
	double temp21 = cosq6 * sinq2 + cosq5 * sinq6 * temp5 - sinq5 * sinq6 * temp6;
	double temp22 = cosq5 * sinq6 * temp11 + sinq5 * sinq6 * temp13 - cosq2 * cosq6 * sinq1;
	double temp23 = temp9 * temp20 + temp10 * temp21 + temp12 * temp22;
	double temp24 = temp14 * temp10 + (sinq5 * temp7 - cosq5 * temp8) * temp9 + temp12 * temp15;

	CoM = VectorXd::Zero(6);
	CoM(0) = Foot(0) + (sinthx_foot * sinthz_foot + costhx_foot * costhz_foot * sinthy_foot) * temp19 - (costhx_foot * sinthz_foot - costhz_foot * sinthx_foot * sinthy_foot) * temp23 - costhy_foot * costhz_foot * temp24;

	CoM(1) = Foot(1) - (costhz_foot * sinthx_foot - costhx_foot * sinthy_foot * sinthz_foot) * temp19 + (costhx_foot * costhz_foot + sinthx_foot * sinthy_foot * sinthz_foot) * temp23 - costhy_foot * sinthz_foot * temp24;

	CoM(2) = Foot(2) + sinthy_foot * temp24 + costhx_foot * costhy_foot * temp19 + costhy_foot * sinthx_foot * temp23;

	r11 = (sinthx_foot * sinthz_foot + costhx_foot * costhz_foot * sinthy_foot) * temp18 - (costhx_foot * sinthz_foot - costhz_foot * sinthx_foot * sinthy_foot) * temp22 - costhy_foot * costhz_foot * temp15;

	r21 = -(costhz_foot * sinthx_foot - costhx_foot * sinthy_foot * sinthz_foot) * temp18 + (costhx_foot * costhz_foot + sinthx_foot * sinthy_foot * sinthz_foot) * temp22 - costhy_foot * sinthz_foot * temp15;

	r31 = sinthy_foot * temp15 + costhx_foot * costhy_foot * temp18 + costhy_foot * sinthx_foot * temp22;

	r32 = sinthy_foot * (sinq5 * temp7 - cosq5 * temp8) + costhy_foot * sinthx_foot * temp20 + costhx_foot * costhy_foot * temp16;

	r33 = sinthy_foot * temp14 - costhx_foot * costhy_foot * temp17 + costhy_foot * sinthx_foot * temp21;

	CoM(3) = -atan2(r32, r33);
	CoM(4) = atan2(-r31, sqrt(r32 * r32 + r33 * r33));
	CoM(5) = atan2(r21, r11);
}

void Chz::Kinematics::FK_FootfromChunk(Eigen::VectorXd& Foot, Eigen::VectorXd CoM, Eigen::VectorXd Joints, const char c)
{
	double l1, l2, q1, q2, q3, q4, q5, q6, thx_com, thy_com, thz_com, p_waist;
	thx_com = CoM(3); thy_com = CoM(4); thz_com = CoM(5);
	if (c == 'L')
	{
		l1 = Robot->Comp["LeftThigh"].l; l2 = Robot->Comp["LeftShank"].l;
		p_waist = 0.5 * Robot->waist_width;
		q1 = Joints(6); q2 = Joints(7); q3 = Joints(8); q4 = Joints(9); q5 = Joints(10); q6 = Joints(11);
	}
	else if (c == 'R')
	{
		l1 = Robot->Comp["RightThigh"].l; l2 = Robot->Comp["RightShank"].l;
		p_waist = -0.5 * Robot->waist_width;
		q1 = Joints(0); q2 = Joints(1); q3 = Joints(2); q4 = Joints(3); q5 = Joints(4); q6 = Joints(5);
	}
	else return;

	q3 = -q3; q4 = -q4; q5 = -q5;
	double r11, r21, r31, r32, r33;
	double sinq1, sinq2, sinq3, sinq4, sinq5, sinq6, sinthx_com, sinthy_com, sinthz_com;
	double cosq1, cosq2, cosq3, cosq4, cosq5, cosq6, costhx_com, costhy_com, costhz_com;

	sinq1 = sin(q1); sinq2 = sin(q2); sinq3 = sin(q3); sinq4 = sin(q4); sinq5 = sin(q5); sinq6 = sin(q6);
	cosq1 = cos(q1); cosq2 = cos(q2); cosq3 = cos(q3); cosq4 = cos(q4); cosq5 = cos(q5); cosq6 = cos(q6);

	sinthx_com = sin(thx_com); sinthy_com = sin(thy_com); sinthz_com = sin(thz_com);
	costhx_com = cos(thx_com); costhy_com = cos(thy_com); costhz_com = cos(thz_com);

	double temp1 = cosq3 * sinq1 + cosq1 * sinq2 * sinq3;
	double temp2 = cosq1 * cosq3 - sinq1 * sinq2 * sinq3;
	double temp3 = sinq1 * sinq3 - cosq1 * cosq3 * sinq2;
	double temp4 = cosq1 * sinq3 + cosq3 * sinq1 * sinq2;
	double temp5 = costhx_com * sinthz_com - costhz_com * sinthx_com * sinthy_com;
	double temp6 = sinthx_com * sinthz_com + costhx_com * costhz_com * sinthy_com;
	double temp7 = costhx_com * costhz_com + sinthx_com * sinthy_com * sinthz_com;
	double temp8 = costhz_com * sinthx_com - costhx_com * sinthy_com * sinthz_com;
	double temp9 = temp1 * temp5 - costhy_com * costhz_com * temp2 + cosq2 * sinq3 * temp6;
	double temp10 = -temp3 * temp5 + costhy_com * costhz_com * temp4 + cosq2 * cosq3 * temp6;
	double temp11 = temp3 * temp7 - cosq2 * cosq3 * temp8 + costhy_com * sinthz_com * temp4;
	double temp12 = temp1 * temp7 + costhy_com * sinthz_com * temp2 + cosq2 * sinq3 * temp8;
	double temp13 = -sinthy_com * temp4 + costhy_com * sinthx_com * temp3 + cosq2 * cosq3 * costhx_com * costhy_com;
	double temp14 = sinthy_com * temp2 - costhy_com * sinthx_com * temp1 + cosq2 * costhx_com * costhy_com * sinq3;
	double temp15 = cosq4 * temp13 - sinq4 * temp14;
	double temp16 = cosq4 * temp14 + sinq4 * temp13;

	Foot = VectorXd::Zero(6);
	Foot(0) = CoM(0) + l2 * (sinq4 * temp9 - cosq4 * temp10) - p_waist * temp5 - l1 * temp10;
	Foot(1) = CoM(1) + p_waist * temp7 - l2 * (cosq4 * temp11 + sinq4 * temp12) - l1 * temp11;
	Foot(2) = CoM(2) - l1 * temp13 - l2 * temp15 + p_waist * costhy_com * sinthx_com;

	r11 = -cosq5 * (cosq4 * temp9 + sinq4 * temp10) + sinq5 * (sinq4 * temp9 - cosq4 * temp10);

	r21 = cosq5 * (cosq4 * temp12 - sinq4 * temp11) - sinq5 * (cosq4 * temp11 + sinq4 * temp12);

	r31 = -cosq5 * temp16 - sinq5 * temp15;

	r32 = cosq6 * (cosq2 * sinq1 * sinthy_com + costhx_com * costhy_com * sinq2 + cosq1 * cosq2 * costhy_com * sinthx_com) + cosq5 * sinq6 * temp15 - sinq5 * sinq6 * temp16;

	r33 = -sinq6 * (cosq2 * sinq1 * sinthy_com + costhx_com * costhy_com * sinq2 + cosq1 * cosq2 * costhy_com * sinthx_com) + cosq5 * cosq6 * temp15 - cosq6 * sinq5 * temp16;

	Foot(3) = atan2(r32, r33);
	Foot(4) = atan2(-r31, sqrt(r32 * r32 + r33 * r33));
	Foot(5) = atan2(r21, r11);
}

void Chz::Kinematics::FK_FootQianHoufromChunk(Eigen::Vector3d& Pqian, Eigen::Vector3d& Phou, Eigen::VectorXd CoM, Eigen::VectorXd Joints, char c)
{
	double l1, l2, q1, q2, q3, q4, q5, q6, thx_com, thy_com, thz_com, p_waist;
	thx_com = CoM(3); thy_com = CoM(4); thz_com = CoM(5);
	if (c == 'L')
	{
		l1 = Robot->Comp["LeftThigh"].l; l2 = Robot->Comp["LeftShank"].l;
		p_waist = 0.5 * Robot->waist_width;
		q1 = Joints(6); q2 = Joints(7); q3 = Joints(8); q4 = Joints(9); q5 = Joints(10); q6 = Joints(11);
	}
	else if (c == 'R')
	{
		l1 = Robot->Comp["RightThigh"].l; l2 = Robot->Comp["RightShank"].l;
		p_waist = -0.5 * Robot->waist_width;
		q1 = Joints(0); q2 = Joints(1); q3 = Joints(2); q4 = Joints(3); q5 = Joints(4); q6 = Joints(5);
	}
	else return;
	q3 = -q3; q4 = -q4; q5 = -q5;

	double sinq1, sinq2, sinq3, sinq4, sinq5, sinq6, sinthx_com, sinthy_com, sinthz_com;
	double cosq1, cosq2, cosq3, cosq4, cosq5, cosq6, costhx_com, costhy_com, costhz_com;

	sinq1 = sin(q1); sinq2 = sin(q2); sinq3 = sin(q3); sinq4 = sin(q4); sinq5 = sin(q5); sinq6 = sin(q6);
	cosq1 = cos(q1); cosq2 = cos(q2); cosq3 = cos(q3); cosq4 = cos(q4); cosq5 = cos(q5); cosq6 = cos(q6);

	sinthx_com = sin(thx_com); sinthy_com = sin(thy_com); sinthz_com = sin(thz_com);
	costhx_com = cos(thx_com); costhy_com = cos(thy_com); costhz_com = cos(thz_com);

	double lqian = Robot->lqian, hank = Robot->h_ankle;

	double temp1 = cosq3 * sinq1 + cosq1 * sinq2 * sinq3;
	double temp2 = cosq1 * cosq3 - sinq1 * sinq2 * sinq3;
	double temp3 = sinq1 * sinq3 - cosq1 * cosq3 * sinq2;
	double temp4 = cosq1 * sinq3 + cosq3 * sinq1 * sinq2;
	double temp5 = costhx_com * sinthz_com - costhz_com * sinthx_com * sinthy_com;
	double temp6 = sinthx_com * sinthz_com + costhx_com * costhz_com * sinthy_com;
	double temp7 = costhx_com * costhz_com + sinthx_com * sinthy_com * sinthz_com;
	double temp8 = costhz_com * sinthx_com - costhx_com * sinthy_com * sinthz_com;
	double temp9 = temp1 * temp5 - costhy_com * costhz_com * temp2 + cosq2 * sinq3 * temp6;
	double temp10 = -temp3 * temp5 + costhy_com * costhz_com * temp4 + cosq2 * cosq3 * temp6;
	double temp11 = temp3 * temp7 - cosq2 * cosq3 * temp8 + costhy_com * sinthz_com * temp4;
	double temp12 = temp1 * temp7 + costhy_com * sinthz_com * temp2 + cosq2 * sinq3 * temp8;
	double temp13 = -sinthy_com * temp4 + costhy_com * sinthx_com * temp3 + cosq2 * cosq3 * costhx_com * costhy_com;
	double temp14 = sinthy_com * temp2 - costhy_com * sinthx_com * temp1 + cosq2 * costhx_com * costhy_com * sinq3;
	double temp15 = cosq4 * temp13 - sinq4 * temp14;
	double temp16 = cosq4 * temp14 + sinq4 * temp13;
	double temp17 = sinq4 * temp9 - cosq4 * temp10;
	double temp18 = cosq4 * temp9 + sinq4 * temp10;
	double temp19 = cosq4 * temp11 + sinq4 * temp12;

	Pqian(0) = CoM(0) + hank * (-sinq6 * (-sinq2 * temp6 + cosq1 * cosq2 * temp5 + cosq2 * costhy_com * costhz_com * sinq1) + cosq5 * cosq6 * temp17 + cosq6 * sinq5 * temp18) + l2 * temp17 - p_waist * temp5 - l1 * temp10 - lqian * (cosq5 * temp18 - sinq5 * temp17);
	Pqian(1) = CoM(1) - hank * (sinq6 * (sinq2 * temp8 - cosq1 * cosq2 * temp7 + cosq2 * costhy_com * sinq1 * sinthz_com) + cosq5 * cosq6 * temp19 + cosq6 * sinq5 * (cosq4 * temp12 - sinq4 * temp11)) + p_waist * temp7 - l2 * temp19 - l1 * temp11 + lqian * (cosq5 * (cosq4 * temp12 - sinq4 * temp11) - sinq5 * temp19);
	Pqian(2) = CoM(2) - lqian * (cosq5 * temp16 + sinq5 * temp15) + hank * (sinq6 * (cosq2 * sinq1 * sinthy_com + costhx_com * costhy_com * sinq2 + cosq1 * cosq2 * costhy_com * sinthx_com) - cosq5 * cosq6 * temp15 + cosq6 * sinq5 * temp16) - l1 * temp13 - l2 * temp15 + p_waist * costhy_com * sinthx_com;

	lqian = -Robot->lhou;

	Phou(0) = CoM(0) + hank * (-sinq6 * (-sinq2 * temp6 + cosq1 * cosq2 * temp5 + cosq2 * costhy_com * costhz_com * sinq1) + cosq5 * cosq6 * temp17 + cosq6 * sinq5 * temp18) + l2 * temp17 - p_waist * temp5 - l1 * temp10 - lqian * (cosq5 * temp18 - sinq5 * temp17);
	Phou(1) = CoM(1) - hank * (sinq6 * (sinq2 * temp8 - cosq1 * cosq2 * temp7 + cosq2 * costhy_com * sinq1 * sinthz_com) + cosq5 * cosq6 * temp19 + cosq6 * sinq5 * (cosq4 * temp12 - sinq4 * temp11)) + p_waist * temp7 - l2 * temp19 - l1 * temp11 + lqian * (cosq5 * (cosq4 * temp12 - sinq4 * temp11) - sinq5 * temp19);
	Phou(2) = CoM(2) - lqian * (cosq5 * temp16 + sinq5 * temp15) + hank * (sinq6 * (cosq2 * sinq1 * sinthy_com + costhx_com * costhy_com * sinq2 + cosq1 * cosq2 * costhy_com * sinthx_com) - cosq5 * cosq6 * temp15 + cosq6 * sinq5 * temp16) - l1 * temp13 - l2 * temp15 + p_waist * costhy_com * sinthx_com;
}

void Chz::Kinematics::FK_FootEdgefromFoot(Vector3d Pedges[4], VectorXd Foot, char lr)
{
	double lqian = Robot->lqian, lhou = Robot->lhou, lin = Robot->lin, lout = Robot->lout, hank = Robot->h_ankle;
	Vector3d Pedgs_rel[4];
	if (lr == 'L')
	{
		Pedgs_rel[0] = Vector3d(-lhou, -lin, -hank);
		Pedgs_rel[1] = Vector3d(lqian, -lin, -hank);
		Pedgs_rel[2] = Vector3d(-lhou, lout, -hank);
		Pedgs_rel[3] = Vector3d(lqian, lout, -hank);
	}
	if (lr == 'R')
	{
		Pedgs_rel[0] = Vector3d(-lhou, lin, -hank);
		Pedgs_rel[1] = Vector3d(lqian, lin, -hank);
		Pedgs_rel[2] = Vector3d(-lhou, -lout, -hank);
		Pedgs_rel[3] = Vector3d(lqian, -lout, -hank);
	}
	Matrix3d RotFoot = RotfromEuler(Vector3d(Foot(3), Foot(4), Foot(5)));
	Vector3d PFoot = Vector3d(Foot(0), Foot(1), Foot(2));
	ChzF(i, 0, 3) Pedges[i] = RotFoot * Pedgs_rel[i] + PFoot;
}

int Chz::Kinematics::IK_FullDOF(const double T[4][4], double ang[6], const double ref[6])
{
	double d34, d45;
	double nx, ox, ax, px;
	double ny, oy, ay, py;
	double nz, oz, az, pz;

	double r1, r2, r3, r4, r5;
	double sinr1, cosr1;
	double sinr2, cosr2;
	double sinr3, cosr3;
	double sinr4, cosr4;
	double sinr5, cosr5;
	double sinr6, cosr6;

	double angle[6][8];

	double temp[6];

	int i1, i2;

	d34 = -Robot->Comp["LeftShank"].l;
	d45 = -Robot->Comp["LeftThigh"].l;

	nx = T[0][0]; ox = T[0][1]; ax = T[0][2]; px = T[0][3];
	ny = T[1][0]; oy = T[1][1]; ay = T[1][2]; py = T[1][3];
	nz = T[2][0]; oz = T[2][1]; az = T[2][2]; pz = T[2][3];

	temp[0] = (d34 * d34 + d45 * d45 - (px * px + py * py + pz * pz)) / (abs(2.0 * d34 * d45));
	angle[3][0] = pi - abs(acos(temp[0]));
	angle[3][1] = angle[3][0];
	angle[3][2] = -angle[3][0];
	angle[3][3] = -angle[3][0];
	angle[3][4] = angle[3][0];
	angle[3][5] = angle[3][0];
	angle[3][6] = -angle[3][0];
	angle[3][7] = -angle[3][0];

	temp[0] = lee_aprox_all(1.0 / sqrt(pow(ny, 2.0) * pow(pz, 2.0) - 2.0 * ny * pz * nz * py + nz * nz * py * py + nx * nx * pz * pz - 2.0 * nx * pz * px * nz + px * px * nz * nz) * (ny * pz - nz * py));
	temp[1] = lee_aprox_all(1.0 / sqrt(ny * ny * pz * pz - 2.0 * ny * pz * nz * py + nz * nz * py * py + nx * nx * pz * pz - 2.0 * nx * pz * px * nz + px * px * nz * nz) * (nx * pz - px * nz));
	temp[2] = lee_aprox_all(-1.0 / sqrt(ny * ny * pz * pz - 2.0 * ny * pz * nz * py + nz * nz * py * py + nx * nx * pz * pz - 2.0 * nx * pz * px * nz + px * px * nz * nz) * (ny * pz - nz * py));
	temp[3] = lee_aprox_all(-1.0 / sqrt(ny * ny * pz * pz - 2.0 * ny * pz * nz * py + nz * nz * py * py + nx * nx * pz * pz - 2.0 * nx * pz * px * nz + px * px * nz * nz) * (nx * pz - px * nz));

	angle[0][0] = atan2(temp[0], temp[1]);
	angle[0][4] = atan2(temp[2], temp[3]);
	for (i1 = 1; i1 < 4; i1++) angle[0][i1] = angle[0][0];
	for (i1 = 5; i1 < 8; i1++) angle[0][i1] = angle[0][4];

	angle[1][0] = -atan(1.0 / sqrt(ny * ny * pz * pz - 2.0 * ny * pz * nz * py + nz * nz * py * py + nx * nx * pz * pz - 2.0 * nx * pz * px * nz + px * px * nz * nz) * (-px * ny + nx * py));
	angle[1][1] = angle[1][0] + pi - 2.0 * pi * (angle[1][0] > 0.0 ? 1.0 : 0.0);
	angle[1][2] = angle[1][0];
	angle[1][3] = angle[1][1];
	angle[1][4] = -angle[1][0];
	angle[1][5] = angle[1][4] + pi - 2.0 * pi * (angle[1][4] > 0.0 ? 1.0 : 0.0);
	angle[1][6] = angle[1][4];
	angle[1][7] = angle[1][5];

	for (i1 = 0; i1 < 8; i1++)
	{
		r1 = angle[0][i1]; sinr1 = sin(r1); cosr1 = cos(r1);
		r2 = angle[1][i1]; sinr2 = sin(r2); cosr2 = cos(r2);
		r4 = angle[3][i1]; sinr4 = sin(r4); cosr4 = cos(r4);
		sinr3 = lee_aprox_all((d34 * cosr1 * px + d45 * cosr4 * cosr1 * px + d34 * sinr1 * py + d45 * cosr4 * sinr1 * py + sqrt(d45 * d45 * (-1 + cosr4 * cosr4) * (-2 * d45 * cosr4 * d34 + 2 * cosr1 * px * sinr1 * py - d45 * d45 - py * py * cosr1 * cosr1 + px * px * cosr1 * cosr1 + py * py - d34 * d34))) / (2 * d45 * cosr4 * d34 + d34 * d34 + d45 * d45));
		cosr3 = lee_aprox_all(-(d45 * d45 * cosr4 * cosr4 * cosr1 * px + d45 * d45 * cosr4 * cosr4 * sinr1 * py + d45 * cosr4 * sqrt(d45 * d45 * (-1 + cosr4 * cosr4) * (-2 * d45 * cosr4 * d34 + 2 * cosr1 * px * sinr1 * py - d45 * d45 - py * py * cosr1 * cosr1 + px * px * cosr1 * cosr1 + py * py - d34 * d34)) + d34 * sqrt(d45 * d45 * (-1 + cosr4 * cosr4) * (-2 * d45 * cosr4 * d34 + 2 * cosr1 * px * sinr1 * py - d45 * d45 - py * py * cosr1 * cosr1 + px * px * cosr1 * cosr1 + py * py - d34 * d34)) - cosr1 * px * d45 * d45 - sinr1 * py * d45 * d45) / (2 * d45 * cosr4 * d34 + d34 * d34 + d45 * d45) / d45 / sinr4);
		temp[0] = atan2(sinr3, cosr3);
		sinr3 = lee_aprox_all((d34 * cosr1 * px + d45 * cosr4 * cosr1 * px + d34 * sinr1 * py + d45 * cosr4 * sinr1 * py - sqrt(d45 * d45 * (-1 + cosr4 * cosr4) * (-2 * d45 * cosr4 * d34 + 2 * cosr1 * px * sinr1 * py - d45 * d45 - py * py * cosr1 * cosr1 + px * px * cosr1 * cosr1 + py * py - d34 * d34))) / (2 * d45 * cosr4 * d34 + d34 * d34 + d45 * d45));
		cosr3 = lee_aprox_all(-(d45 * d45 * cosr4 * cosr4 * cosr1 * px + d45 * d45 * cosr4 * cosr4 * sinr1 * py - d45 * cosr4 * sqrt(d45 * d45 * (-1 + cosr4 * cosr4) * (-2 * d45 * cosr4 * d34 + 2 * cosr1 * px * sinr1 * py - d45 * d45 - py * py * cosr1 * cosr1 + px * px * cosr1 * cosr1 + py * py - d34 * d34)) - d34 * sqrt(d45 * d45 * (-1 + cosr4 * cosr4) * (-2 * d45 * cosr4 * d34 + 2 * cosr1 * px * sinr1 * py - d45 * d45 - py * py * cosr1 * cosr1 + px * px * cosr1 * cosr1 + py * py - d34 * d34)) - cosr1 * px * d45 * d45 - sinr1 * py * d45 * d45) / (2 * d45 * cosr4 * d34 + d34 * d34 + d45 * d45) / d45 / sinr4);
		temp[1] = atan2(sinr3, cosr3);
		sinr3 = lee_aprox_all((d45 * d45 * cosr4 * cosr4 * sinr2 * sinr1 * px + d45 * d45 * cosr4 * cosr4 * cosr2 * pz - d45 * d45 * cosr4 * cosr4 * cosr1 * sinr2 * py + d45 * cosr4 * sqrt(d45 * d45 * (-1 + cosr4 * cosr4) * (-2 * d45 * cosr4 * d34 + 2 * sinr2 * sinr1 * px * cosr2 * pz + py * py * cosr1 * cosr1 + px * px * cosr2 * cosr2 * cosr1 * cosr1 - px * px * cosr2 * cosr2 - 2 * cosr1 * px * sinr1 * py - px * px * cosr1 * cosr1 - py * py * cosr2 * cosr2 * cosr1 * cosr1 - d45 * d45 + px * px + 2 * cosr1 * px * sinr1 * py * cosr2 * cosr2 - 2 * cosr1 * sinr2 * py * cosr2 * pz - d34 * d34 + pz * pz * cosr2 * cosr2)) + d34 * sqrt(d45 * d45 * (-1 + cosr4 * cosr4) * (-2 * d45 * cosr4 * d34 + 2 * sinr2 * sinr1 * px * cosr2 * pz + py * py * cosr1 * cosr1 + px * px * cosr2 * cosr2 * cosr1 * cosr1 - px * px * cosr2 * cosr2 - 2 * cosr1 * px * sinr1 * py - px * px * cosr1 * cosr1 - py * py * cosr2 * cosr2 * cosr1 * cosr1 - d45 * d45 + px * px + 2 * cosr1 * px * sinr1 * py * cosr2 * cosr2 - 2 * cosr1 * sinr2 * py * cosr2 * pz - d34 * d34 + pz * pz * cosr2 * cosr2)) - sinr2 * sinr1 * px * d45 * d45 + sinr2 * cosr1 * py * d45 * d45 - cosr2 * pz * d45 * d45) / (2 * d45 * cosr4 * d34 + d34 * d34 + d45 * d45) / d45 / sinr4);
		cosr3 = lee_aprox_all((-cosr1 * d34 * sinr2 * py + d45 * cosr4 * sinr2 * sinr1 * px + d34 * cosr2 * pz + d45 * cosr4 * cosr2 * pz + d34 * sinr2 * sinr1 * px - cosr1 * d45 * cosr4 * sinr2 * py + sqrt(d45 * d45 * (-1 + cosr4 * cosr4) * (-2 * d45 * cosr4 * d34 + 2 * sinr2 * sinr1 * px * cosr2 * pz + py * py * cosr1 * cosr1 + px * px * cosr2 * cosr2 * cosr1 * cosr1 - px * px * cosr2 * cosr2 - 2 * cosr1 * px * sinr1 * py - px * px * cosr1 * cosr1 - py * py * cosr2 * cosr2 * cosr1 * cosr1 - d45 * d45 + px * px + 2 * cosr1 * px * sinr1 * py * cosr2 * cosr2 - 2 * cosr1 * sinr2 * py * cosr2 * pz - d34 * d34 + pz * pz * cosr2 * cosr2))) / (2 * d45 * cosr4 * d34 + d34 * d34 + d45 * d45));
		temp[2] = atan2(sinr3, cosr3);
		sinr3 = lee_aprox_all((d45 * d45 * cosr4 * cosr4 * sinr2 * sinr1 * px + d45 * d45 * cosr4 * cosr4 * cosr2 * pz - d45 * d45 * cosr4 * cosr4 * cosr1 * sinr2 * py - d45 * cosr4 * sqrt(d45 * d45 * (-1 + cosr4 * cosr4) * (-2 * d45 * cosr4 * d34 + 2 * sinr2 * sinr1 * px * cosr2 * pz + py * py * cosr1 * cosr1 + px * px * cosr2 * cosr2 * cosr1 * cosr1 - px * px * cosr2 * cosr2 - 2 * cosr1 * px * sinr1 * py - px * px * cosr1 * cosr1 - py * py * cosr2 * cosr2 * cosr1 * cosr1 - d45 * d45 + px * px + 2 * cosr1 * px * sinr1 * py * cosr2 * cosr2 - 2 * cosr1 * sinr2 * py * cosr2 * pz - d34 * d34 + pz * pz * cosr2 * cosr2)) - d34 * sqrt(d45 * d45 * (-1 + cosr4 * cosr4) * (-2 * d45 * cosr4 * d34 + 2 * sinr2 * sinr1 * px * cosr2 * pz + py * py * cosr1 * cosr1 + px * px * cosr2 * cosr2 * cosr1 * cosr1 - px * px * cosr2 * cosr2 - 2 * cosr1 * px * sinr1 * py - px * px * cosr1 * cosr1 - py * py * cosr2 * cosr2 * cosr1 * cosr1 - d45 * d45 + px * px + 2 * cosr1 * px * sinr1 * py * cosr2 * cosr2 - 2 * cosr1 * sinr2 * py * cosr2 * pz - d34 * d34 + pz * pz * cosr2 * cosr2)) - sinr2 * sinr1 * px * d45 * d45 + sinr2 * cosr1 * py * d45 * d45 - cosr2 * pz * d45 * d45) / (2 * d45 * cosr4 * d34 + d34 * d34 + d45 * d45) / d45 / sinr4);
		cosr3 = lee_aprox_all((-cosr1 * d34 * sinr2 * py + d45 * cosr4 * sinr2 * sinr1 * px + d34 * cosr2 * pz + d45 * cosr4 * cosr2 * pz + d34 * sinr2 * sinr1 * px - cosr1 * d45 * cosr4 * sinr2 * py - sqrt(d45 * d45 * (-1 + cosr4 * cosr4) * (-2 * d45 * cosr4 * d34 + 2 * sinr2 * sinr1 * px * cosr2 * pz + py * py * cosr1 * cosr1 + px * px * cosr2 * cosr2 * cosr1 * cosr1 - px * px * cosr2 * cosr2 - 2 * cosr1 * px * sinr1 * py - px * px * cosr1 * cosr1 - py * py * cosr2 * cosr2 * cosr1 * cosr1 - d45 * d45 + px * px + 2 * cosr1 * px * sinr1 * py * cosr2 * cosr2 - 2 * cosr1 * sinr2 * py * cosr2 * pz - d34 * d34 + pz * pz * cosr2 * cosr2))) / (2 * d45 * cosr4 * d34 + d34 * d34 + d45 * d45));
		temp[3] = atan2(sinr3, cosr3);
		if (abs(temp[0] - temp[2]) < lee_MIN_ERROR || abs(temp[0] - temp[3]) < lee_MIN_ERROR)
		{
			angle[2][i1] = temp[0];
		}
		else
		{
			angle[2][i1] = temp[1];
		}
	}

	//[ Get joint 5 ]
	for (i1 = 0; i1 < 8; i1++)
	{
		r1 = angle[0][i1]; sinr1 = sin(r1); cosr1 = cos(r1);
		r2 = angle[1][i1]; sinr2 = sin(r2); cosr2 = cos(r2);
		r3 = angle[2][i1]; sinr3 = sin(r3); cosr3 = cos(r3);
		r4 = angle[3][i1]; sinr4 = sin(r4); cosr4 = cos(r4);
		//sinr345
		temp[0] = lee_aprox_all(-sinr2 * sinr1 * nx + sinr2 * cosr1 * ny - cosr2 * nz);
		//cosr345
		temp[1] = lee_aprox_all(cosr1 * nx + sinr1 * ny);
		temp[3] = atan2(temp[0], temp[1]);
		angle[4][i1] = temp[3] - r3 - r4;
		if (angle[4][i1] > pi)
			angle[4][i1] -= 2.0 * pi;
		else if (angle[4][i1] < -pi)
			angle[4][i1] += 2.0 * pi;
	}

	//[ Get joint 6 ]
	for (i1 = 0; i1 < 8; i1++)
	{
		r1 = angle[0][i1]; sinr1 = sin(r1); cosr1 = cos(r1);
		r2 = angle[1][i1]; sinr2 = sin(r2); cosr2 = cos(r2);
		r3 = angle[2][i1]; sinr3 = sin(r3); cosr3 = cos(r3);
		r4 = angle[3][i1]; sinr4 = sin(r4); cosr4 = cos(r4);
		r5 = angle[4][i1]; sinr5 = sin(r5); cosr5 = cos(r5);
		sinr6 = 1.0 / 2.0 * ox * sin(r5 + r4 + r1 + r3) - 1.0 / 2.0 * ox * sin(-r5 - r4 + r1 - r3) + 1.0 / 4.0 * ox * cos(-r5 - r4 - r3 + r1 - r2) - 1.0 / 4.0 * ox * cos(r5 + r4 + r3 + r1 + r2) + 1.0 / 4.0 * ox * cos(r5 + r4 + r3 + r1 - r2) - 1.0 / 4.0 * ox * cos(-r5 - r4 - r3 + r1 + r2) + 1.0 / 2.0 * oy * cos(-r5 - r4 + r1 - r3) - 1.0 / 2.0 * oy * cos(r5 + r4 + r1 + r3) - 1.0 / 4.0 * oy * sin(r5 + r4 + r3 + r1 + r2) + 1.0 / 4.0 * oy * sin(-r5 - r4 - r3 + r1 - r2) - 1.0 / 4.0 * oy * sin(-r5 - r4 - r3 + r1 + r2) + 1.0 / 4.0 * oy * sin(r5 + r4 + r3 + r1 - r2) + 1.0 / 2.0 * oz * cos(-r5 - r4 + r2 - r3) + 1.0 / 2.0 * oz * cos(r5 + r4 + r2 + r3);
		cosr6 = 1.0 / 2.0 * ax * sin(r5 + r4 + r1 + r3) - 1.0 / 2.0 * ax * sin(-r5 - r4 + r1 - r3) + 1.0 / 4.0 * ax * cos(r1 - r2 - r3 - r5 - r4) - 1.0 / 4.0 * ax * cos(r1 + r2 + r3 + r5 + r4) + 1.0 / 4.0 * ax * cos(r1 - r2 + r3 + r5 + r4) - 1.0 / 4.0 * ax * cos(r1 + r2 - r3 - r5 - r4) + 1.0 / 2.0 * ay * cos(-r5 - r4 + r1 - r3) - 1.0 / 2.0 * ay * cos(r5 + r4 + r1 + r3) - 1.0 / 4.0 * ay * sin(r1 + r2 + r3 + r5 + r4) + 1.0 / 4.0 * ay * sin(r1 - r2 - r3 - r5 - r4) - 1.0 / 4.0 * ay * sin(r1 + r2 - r3 - r5 - r4) + 1.0 / 4.0 * ay * sin(r1 - r2 + r3 + r5 + r4) + 1.0 / 2.0 * az * cos(r2 - r3 - r5 - r4) + 1.0 / 2.0 * az * cos(r2 + r3 + r5 + r4); sinr6 = lee_aprox_all(sinr6);
		sinr6 = lee_aprox_all(sinr6);
		cosr6 = lee_aprox_all(cosr6);
		angle[5][i1] = atan2(sinr6, cosr6);
	}
	//Select fit one 
	i1 = lee_res_choose(&angle[0][0], ref, 6, 8);



	if (i1 == -1)
		return 0;
	else
	{
		for (i2 = 0; i2 < 6; i2++)
			ang[i2] = angle[i2][i1];
		return 1;
	}
}
