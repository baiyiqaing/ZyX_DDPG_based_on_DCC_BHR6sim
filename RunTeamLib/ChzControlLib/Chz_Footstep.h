#pragma once
#include "Chz_Base.h"

namespace Chz
{
	class RobotParam;
	class FootstepController
	{
		RobotParam* Robot;

		bool If_Left;//�����ĸ���֧���� 1Ϊ��

		Eigen::Vector3d RFoot_delx; //�ҽŵ�����
		Eigen::Vector3d LFoot_delx; //��ŵ�����
		Eigen::Vector2d CoM_x; //���ڵ�ˮƽCoMλ���ٶ�

		double h, w, T;
		double T_down; //ÿ������ŵ�ʱ��

		double u_now; //�������(ZMP)λ�ã�ͬʱҲ��֧�Ž�λ��
		double u0; //�������(ZMP)λ��
		double liftpos; //��ǰ�İڶ���̧��֮ǰ��λ�õ����������ڲ�ֵ����ż��ٶȿ�����
		double downpos; //ԭ���滮����ŵ�λ��

		double t_now; //����ʱ��
		Eigen::Vector2d x_now; //��������ˮƽλ�ú��ٶ�
		double xi_now; //����DCM

		Eigen::Vector2d x0; //t = T_downʱ������ˮƽλ�ú��ٶ�
		double xi0; //t = T_downʱ��DCM
		double xid; //����DCM

		double q_u;
		double Kp_Foot, Kd_Foot, Ka_Foot;

		void UpdateCoMPos(Eigen::Vector2d CoM_xnew);
		double XPredict(double t_pre, Eigen::VectorXd y); //����x_now u_nowԤ��t_pre���x
		double XiPredict(double t_pre, Eigen::VectorXd y); //����xi_now u_nowԤ��t_pre���xi
		void UpdateFootPos();
		void Updateu0(Eigen::VectorXd y); //����x0 xi0 xid����u0
		void Updatexxi0(Eigen::VectorXd y);  //����x_now xi_now u_nowԤ��x0 xi0

	public:
		FootstepController(RobotParam* r);
		void OutputCon(double& RCon, double& LCon);
		void SetInitDownpos(double down);
		void SetInitCoM(Eigen::Vector2d comini);
		void SetPeriod(double ttotal, double tdown);
		void SetStaFoot(bool i, double Sta_x, double lift); //�趨��ʼ��֧���� 1Ϊ��
		void SetXid(const double xi_d);
		void UpdateCon(Eigen::Vector2d CoM_xnew);
		void UpdateSup(double down); //�л�֧���ȣ�ͬʱ����downpos����������������
	};
}