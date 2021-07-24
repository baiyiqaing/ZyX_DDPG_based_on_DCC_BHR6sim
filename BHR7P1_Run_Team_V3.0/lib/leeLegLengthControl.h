// 2019/10/29 行走过程中腿长控制以生成质心高度可变的质心轨迹
// version 0.00 第一次尝试
// 2019/11/05 修改为工控机程序在线调节
#pragma once
#include <cmath>
#include <iostream>
/* 
思路：
根据不同的支撑状态计算实际腿长，然后进行控制，然后积分计算出控制后的实际腿长，在计算出对应的质心高度输出
输入：支撑状态，两踝位置，当前规划质心位置与速度（x, y, z方向）
过程：
	1. 计算支撑状态对应倒立摆模型长度 l 与收缩速度 dl ；
	2. 进行长度控制 ddl = -kp*(l-l0)-kd*dl，积分计算长度 dl = dl + ddl*T; l = l + dl*T ；
	3. 计算质心高度 zc = sqrt(l^2-x^2-y^2) 。
输出：
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