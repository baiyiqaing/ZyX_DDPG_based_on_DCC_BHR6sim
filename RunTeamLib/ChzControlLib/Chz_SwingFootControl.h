#pragma once
#include "Chz_Base.h"

namespace Chz
{
	using namespace std;
	using namespace Eigen;
	class SwingFootController
	{
		double rprop;

		int Ifconr, Ifconl;
		double Kp_air_pos, Kd_air_pos, Kp_air_ang, Kd_air_ang;
		double Kp_grd, Kd_grd, Kp_ret, Kd_ret, Kp_lim, Kd_lim;
		double Pos_lim[6][2];

		VectorXd RPos_r, LPos_r;
		VectorXd RPos_d, LPos_d;
		VectorXd CoM_d, CoM_r;
		VectorXd RCon[3], LCon[3];

		void GetRealPos(VectorXd CoM_r1, VectorXd Joints);
		void SetFootPD(char c, double mp, double me, double ts);
		void SetFootPD_deg2(double& Kp, double& Kd, double tp);
	public:
		SwingFootController();
		void SetProp(double rprop1);
		void SetMode(char lr, int Ifcon1);
		void SetPoslim(double Poslim[6][2]);
		void Update(VectorXd CoM_d1, VectorXd RPos_d1, VectorXd LPos_d1, VectorXd CoM_r1, VectorXd Joints);
		void OutputCoMr(Vector3d& CoMr1);
		void Outputcon(VectorXd& RCon1, VectorXd& LCon1);
	};
}
