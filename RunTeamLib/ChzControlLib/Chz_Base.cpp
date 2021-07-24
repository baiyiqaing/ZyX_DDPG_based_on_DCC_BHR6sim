#include "Chz_Include.h"
Chz::PerspectiveMapping2d ZMPMapping;
#define Chz_SIMULATION_C

#ifdef Chz_SIMULATION_C
namespace Chz
{
	extern Calculator Calc;
	extern FootstepController FTCon;
	extern Kinematics Kinematic_Calc;
}

void Chz::FootStep_Test()
{
	Eigen::Vector2d CoM = Eigen::Vector2d(-0.03, -0.1);
	double t = 0.0, t_real = 0.0, T = 0.5, w = sqrt(Chz::g / 0.8);
	double lfoot_des = 0.08, rfoot_des = -0.08;
	double lfoot_del = 0.0, rfoot_del = 0.0;
	double CONTROL_PRID = 0.004;

	bool If_left = 0;

	FTCon.SetPeriod(T, 0.95 * T);
	FTCon.SetStaFoot(If_left, -0.08, 0.0);
	FTCon.SetInitCoM(CoM);
	FTCon.SetInitDownpos(0.08);
	FTCon.SetXid(-0.025);

	int K_Pre_Con = 0;
	static double Log[10000][30];


	while (1)
	{
		t_real = t_real + CONTROL_PRID;
		t = t + CONTROL_PRID;
		double a;
		if(If_left)
			a = w * w * (CoM(0) - (lfoot_des + lfoot_del));
		else
			a = w * w * (CoM(0) - (rfoot_des + rfoot_del));

		if (t_real < 1.1 && t_real > 1.0) a += 1.5;

		CoM(1) += a * CONTROL_PRID;
		CoM(0) += CoM(1) * CONTROL_PRID;

		FTCon.UpdateCon(CoM);
		FTCon.OutputCon(rfoot_del, lfoot_del);
		if (t > T)
		{
			t -= T;
			if (If_left)
			{
				FTCon.UpdateSup(0.08);
				FTCon.SetXid(-0.025);
			}
			else
			{
				FTCon.UpdateSup(-0.08);
				FTCon.SetXid(0.025);
			}
			If_left = !If_left;
		}

		Log[K_Pre_Con][0] = CoM(0);
		Log[K_Pre_Con][1] = CoM(0) + 1.0 / w * CoM(1);
		Log[K_Pre_Con][2] = rfoot_des + rfoot_del;
		Log[K_Pre_Con][3] = lfoot_des + lfoot_del;
		Log[K_Pre_Con][4] = 1.0 * If_left;
		K_Pre_Con++;
		if (K_Pre_Con > 3000)
			break;
	}

	FILE* f = fopen("FootStep/Log.dat", "w");
	ChzF(i, 0, 5000) 
		ChzF(j, 0, 4) fprintf(f, "%.16f%c", Log[i][j], j == 4 ? '\n' : '\t');
}

void Chz::Kinematics_Test()
{
	Eigen::VectorXd CoM = Eigen::VectorXd::Zero(6);
	CoM << 0.504177843, -0.014448555, 0.655630365, 0.0, 0.031782344, 0.0;
	Eigen::VectorXd Foot = Eigen::VectorXd::Zero(6);
	Eigen::VectorXd Joints = Eigen::VectorXd::Zero(12);
	Joints << -0.004842121, -0.027839249, 0.85144646, -1.28853685, 0.609322209, 0.028257103, 0, 0, 0, 0, 0, 0;
	/*Chz::Kinematic_Calc.FK_FootfromChunk(Foot, CoM, Joints, 'R');
	std::cout << Foot << std::endl;
	Chz::Kinematic_Calc.FK_ChunkfromFoot(CoM, Foot, Joints, 'R');
	std::cout << CoM << std::endl;*/

	Eigen::Vector3d Pqian, Phou;
	Chz::Kinematic_Calc.FK_FootQianHoufromChunk(Pqian, Phou , CoM, Joints, 'R');
	std::cout << Pqian << Phou << std::endl;
}

void Chz::PerspectiveMapping_Test()
{
	Eigen::Vector2d p[4], q[4];
	p[0] << -1.0, 3.0;
	p[1] << 1.0, 9.0;
	p[2] << 2.0, 0.0;
	p[3] << 4.0, 8.0;

	q[0] << 0.0, 0.0;
	q[1] << 0.0, 3.0;
	q[2] << 2.0, 0.0;
	q[3] << 2.0, 3.0;

	ZMPMapping.Setpq(p, q);
	ZMPMapping.CalH();

	Eigen::Vector2d v1(4.0, 8.0);
	v1 = ZMPMapping.Getq(v1);
	std::cout << v1 << std::endl;
}

int main()
{
	//Chz::FootStep_Test();
	//Chz::Kinematics_Test();

	Matrix3d A1;
	A1 << 0.0, 0.056, -0.02,
		0.104, 0.006, -0.042,
		0.0, 0.0, 1.0;
	Matrix3d A2 = A1.inverse();
	cout << A2 << endl;
}
#endif