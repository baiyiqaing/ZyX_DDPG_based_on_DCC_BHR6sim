#ifdef __cplusplus
#include "leeLegLengthControl.h"
extern "C"
{
#endif // __cplusplus
	double lee_leg_length_control(double cycle_time, double ref_leg_length, int state, int last_state, double l_ankle[3], double r_ankle[3], double com[3], double dcom[3], double _ref_time, double *res);
#ifdef __cplusplus
}
#endif // __cplusplus
