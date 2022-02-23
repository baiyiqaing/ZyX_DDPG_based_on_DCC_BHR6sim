// 2019/10/29 ���߹������ȳ��������������ĸ߶ȿɱ�����Ĺ켣
// version 0.00 ��һ�γ���
// 2019/11/05 �޸�Ϊ���ػ��������ߵ���
#pragma once
#include <cmath>
#include <iostream>
/* 
˼·��
���ݲ�ͬ��֧��״̬����ʵ���ȳ���Ȼ����п��ƣ�Ȼ����ּ�������ƺ��ʵ���ȳ����ڼ������Ӧ�����ĸ߶����
���룺֧��״̬������λ�ã���ǰ�滮����λ�����ٶȣ�x, y, z����
���̣�
	1. ����֧��״̬��Ӧ������ģ�ͳ��� l �������ٶ� dl ��
	2. ���г��ȿ��� ddl = -kp*(l-l0)-kd*dl�����ּ��㳤�� dl = dl + ddl*T; l = l + dl*T ��
	3. �������ĸ߶� zc = sqrt(l^2-x^2-y^2) ��
�����
*/
#define LLLC_DS 0
#define LLLC_RS 1
#define LLLC_LS 2
namespace llegcontrol
{
	class LegControl 
	{
	public:
		int init_flag;
		double T;
		double zc;
		double dzc;
		double L;
		double dL;
		double ddL;
		double dddL;
		double ref_L;
		double kp;
		double kd;
		double ref_time;
		double L_act;
		double dL_act;

		//int state;
		//double ankle_pos[2][3];
		//double com[3];
		//double dcom[3];

		LegControl();
		void init(double,double, double com_z, double dcom_z);
		void get_com_height(int state, int last_state, double l_ankle[3], double r_ankle[3], double com[3], double dcom[3], double _ref_time);
		void get_controller();
		void control_leg_length();
	};
}