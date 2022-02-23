#pragma once
#include "Chz_Base.h"

using namespace Eigen;
namespace Chz
{
	class RobotParam;
	class Kinematics
	{
		RobotParam* Robot;

		double lee_MIN_ERROR;
		double lee_aprox(double value, double standard);
		double lee_aprox_all(double value);
		int lee_res_choose(const double* ang, const double* ref, const int joint_num, const int res_num);

		Matrix3d RotfromTh(double th, char axis);
		Matrix3d RotfromEuler(Vector3d Euler);
	public:
		Kinematics(RobotParam* robot);
		void FK_ChunkfromFoot(VectorXd& CoM, VectorXd Foot, VectorXd Joints, char c);
		void FK_FootfromChunk(VectorXd& Foot, VectorXd CoM, VectorXd Joints, char c);
		void FK_FootQianHoufromChunk(Vector3d& Pqian, Vector3d& Phou, VectorXd CoM, VectorXd Joints, char c);
		void FK_FootEdgefromFoot(Vector3d Pedges[4], VectorXd Foot, char lr);
		// 0 inner hou; 1 inner qian; 2 outer hou; 3 outer qian 
		int IK_FullDOF(const double T[4][4], double ang[6], const double ref[6]);
	};
}