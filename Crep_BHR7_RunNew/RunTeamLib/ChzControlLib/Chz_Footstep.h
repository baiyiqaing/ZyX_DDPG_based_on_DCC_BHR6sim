#pragma once
#include "Chz_Base.h"

namespace Chz
{
	class RobotParam;
	class FootstepController
	{
		RobotParam* Robot;

		bool If_Left;//现在哪个是支撑腿 1为左

		Eigen::Vector3d RFoot_delx; //右脚调节量
		Eigen::Vector3d LFoot_delx; //左脚调节量
		Eigen::Vector2d CoM_x; //现在的水平CoM位置速度

		double h, w, T;
		double T_down; //每周期落脚的时间

		double u_now; //现在落脚(ZMP)位置，同时也是支撑脚位置
		double u0; //期望落脚(ZMP)位置
		double liftpos; //当前的摆动腿抬起之前的位置调节量，用于插值计算脚加速度控制量
		double downpos; //原来规划的落脚点位置

		double t_now; //现在时间
		Eigen::Vector2d x_now; //现在质心水平位置和速度
		double xi_now; //现在DCM

		Eigen::Vector2d x0; //t = T_down时刻质心水平位置和速度
		double xi0; //t = T_down时刻DCM
		double xid; //期望DCM

		double q_u;
		double Kp_Foot, Kd_Foot, Ka_Foot;

		void UpdateCoMPos(Eigen::Vector2d CoM_xnew);
		double XPredict(double t_pre, Eigen::VectorXd y); //根据x_now u_now预测t_pre后的x
		double XiPredict(double t_pre, Eigen::VectorXd y); //根据xi_now u_now预测t_pre后的xi
		void UpdateFootPos();
		void Updateu0(Eigen::VectorXd y); //根据x0 xi0 xid计算u0
		void Updatexxi0(Eigen::VectorXd y);  //根据x_now xi_now u_now预测x0 xi0

	public:
		FootstepController(RobotParam* r);
		void OutputCon(double& RCon, double& LCon);
		void SetInitDownpos(double down);
		void SetInitCoM(Eigen::Vector2d comini);
		void SetPeriod(double ttotal, double tdown);
		void SetStaFoot(bool i, double Sta_x, double lift); //设定开始的支撑腿 1为左
		void SetXid(const double xi_d);
		void UpdateCon(Eigen::Vector2d CoM_xnew);
		void UpdateSup(double down); //切换支撑腿，同时更新downpos（不包含调节量）
	};
}