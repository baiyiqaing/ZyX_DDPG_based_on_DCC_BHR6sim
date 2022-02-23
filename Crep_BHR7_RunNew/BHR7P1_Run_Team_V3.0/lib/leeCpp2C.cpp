#include "leeCpp2C.h"

llegcontrol::LegControl Controller;//目前工控机RTX C 调用C++时， C++中不能用定义静态局部类（static class），否则会出错
double lee_leg_length_control(double cycle_time, double ref_leg_length, int state, int last_state, double l_ankle[3], double r_ankle[3], double com[3], double dcom[3], double _ref_time,double *res)
{
	using namespace llegcontrol;
	using namespace std;

	if (Controller.init_flag == 0)
		Controller.init(cycle_time, ref_leg_length,com[2],dcom[2]);
	Controller.get_com_height(state, last_state,l_ankle, r_ankle, com, dcom, _ref_time);
	res[0] = Controller.L;
	res[1] = Controller.dL;
	res[2] = Controller.zc;
	res[3] = Controller.dzc;
	return Controller.zc;
}