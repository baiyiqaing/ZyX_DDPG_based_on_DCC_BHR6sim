#include "leeLegLengthControl.h"
using namespace std;
llegcontrol::LegControl::LegControl()
{
	//int i;
	//state = 0;
	init_flag = 0;
	T = 0;
	zc = 0;
	dzc = 0;
	L = 0;
	dL = 0;
	ddL = 0;
	dddL = 0;
	ref_L = 0;
	kp = 0;
	kd = 0;
	ref_time = 0;
	//for (i = 0; i < 3; i++)
	//{
	//	ankle_pos[0][i] = 0.0;
	//	com[i] = 0.0;
	//	dcom[i] = 0.0;
	//}
}

void llegcontrol::LegControl::init(double cycle_time,double ref_leg_length,double com_z,double dcom_z)
{
	init_flag += 1;
	T = cycle_time;
	ref_L = ref_leg_length;
	L = ref_L;
	dL = 0;
	cout << ref_L << endl;
	zc = com_z;
	dzc = dcom_z;

	cout << "ref_L = " << ref_L << ", comz = " << com_z << ", comz_v = " <<dzc<<endl;
}

void llegcontrol::LegControl::get_com_height(int state, int last_state, double l_ankle[3], double r_ankle[3], double com[3], double dcom[3], double _ref_time)
{
	double support_pos[3];
	double x, y, z;
	double dx, dy, dz;
	switch (state)
	{
	case LLLC_DS:
		for (int i = 0; i < 3; i++)	support_pos[i] = 0.5*(l_ankle[i] + r_ankle[i]);
		break;

	case LLLC_LS:
		for (int i = 0; i < 3; i++)	support_pos[i] = l_ankle[i];
		break;

	case LLLC_RS:
		for (int i = 0; i < 3; i++)	support_pos[i] = r_ankle[i];
		break;

	default:
		std::cout << "state = "<<state<<","<<"please check support state set!!!" << std::endl;
		break;
	}
	//cout << state << endl;
	x = com[0]- support_pos[0];		y = com[1]- support_pos[1];		z = zc- support_pos[2];
	dx = dcom[0];	dy = dcom[1];	dz = dzc;
	L_act = sqrt(pow(x, 2.0) + pow(y, 2.0) + pow(z, 2.0));
	if (init_flag == 1)
	{
		init(this->T, L_act, com[2], dcom[2]);
	}
	//cout << last_state << "," << state << "," << zc;
	if (last_state != state)
	{
		L = sqrt(pow(x, 2.0) + pow(y, 2.0) + pow(z, 2.0));
		dL = (x*dx + y*dy + z*dz) / L;
		cout<<"support satte transition"<<endl;
	}
	//else
	{
		ref_time = _ref_time;
		get_controller();
		control_leg_length();
		double last_zc = zc;
		zc = sqrt(L*L - x*x - y*y) + support_pos[2];
		dzc = (L*dL - x*dx - y*dy) / (zc - support_pos[2]);
		zc = last_zc + T*dzc;
	}
	//cout << "," << zc <<endl;
}

void llegcontrol::LegControl::get_controller()
{	
	double ts = fmax(0.1, ref_time);
	double mp = 0.001;
	double me = 0.001;
	double wn, xi;
	double pi = 3.141592657;

	//xi = sqrt(1.0 / ((pi / log(mp))*(pi / log(mp)) + 1.0));
	//wn = 3.0 / ts / xi;

	double lnmp = log(mp);
	xi = sqrt(lnmp*lnmp / (pi*pi + lnmp*lnmp));
	wn = -1 / (ts*xi)*log(me*sqrt(1 - xi*xi));

	kp = wn*wn;
	kd = 2.0*wn*xi;

	//cout << "xi = " << xi << ",";
	//cout << "wn = " << wn << ",";
	//cout << "ts = " << ts << ",";
	//cout << "kp = " << kp << ",";
	//cout << "kd = " << kd << endl;
}

void llegcontrol::LegControl::control_leg_length()
{
	double pt = 0.01;
	dddL = (-kp*(L - ref_L) - (pt*kp + kd)*dL - (pt*kd + 1)*ddL) / pt;
	//ddL = -kp*(L - ref_L) - kd*dL;
	//ddL = fmax(-9.8 / 2.0, ddL);
	ddL += dddL*T;
	dL += ddL*T;
	L += dL*T;
}