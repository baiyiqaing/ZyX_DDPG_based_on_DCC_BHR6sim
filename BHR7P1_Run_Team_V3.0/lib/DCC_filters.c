#include "DCC_filters.h"
#include "math.h"

double Filter_TimeLag(double filtered_in, double data_in, double control_t, double Lag_T)
{
	double filtered_out;
	
	filtered_out = (control_t * data_in + Lag_T * filtered_in) / (control_t + Lag_T);

	return filtered_out;
}

