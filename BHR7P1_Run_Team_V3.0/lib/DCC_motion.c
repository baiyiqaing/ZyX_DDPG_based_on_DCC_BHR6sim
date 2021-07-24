#include <math.h>
#include "DCC_motion.h"
#include <stdio.h>
#include "Tra_Generate.h"

extern PreCon_Tra Tra_COM;
extern PreCon_Tra Tra_RAnkle, Tra_LAnkle;
extern double roll_body;
extern double pitch_body;
Free_Motion Motion_Protection;

Free_Motion  Motion_Position_in;
Free_Motion  Motion_Position_out;

void TSpline_PVA(double p0, double v0, double a0, double t0, double p1, double t1, double p2, double v2, double a2, double t2, double control_t, double *S)
{

	double P[10];	
	int k = 0;
	int i = 0;

	P[0] = (a2*(pow(t0, 3.0)*pow(t1, 2.0) - t2*pow(t0, 3.0)*t1)) / (6 * (pow(t0, 3.0) - 2 * pow(t0, 2.0)*t1 - t2*pow(t0, 2.0) + t0*pow(t1, 2.0) + 2 * t2*t0*t1 - t2*pow(t1, 2.0))) - (v0*(2 * pow(t0, 3.0)*pow(t1, 2.0) + t2*pow(t0, 3.0)*t1 - pow(t0, 2.0)*pow(t1, 3.0) - 3 * t2*pow(t0, 2.0)*pow(t1, 2.0) + t2*t0*pow(t1, 3.0))) / (pow(t0, 4.0) - 3 * pow(t0, 3.0)*t1 - t2*pow(t0, 3.0) + 3 * pow(t0, 2.0)*pow(t1, 2.0) + 3 * t2*pow(t0, 2.0)*t1 - t0*pow(t1, 3.0) - 3 * t2*t0*pow(t1, 2.0) + t2*pow(t1, 3.0)) - (p0*(4 * pow(t0, 3.0)*pow(t1, 2.0) + 2 * t2*pow(t0, 3.0)*t1 - 4 * pow(t0, 2.0)*pow(t1, 3.0) - 6 * t2*pow(t0, 2.0)*pow(t1, 2.0) + t0*pow(t1, 4.0) + 4 * t2*t0*pow(t1, 3.0) - t2*pow(t1, 4.0))) / (-pow(t0, 5.0) + 4 * pow(t0, 4.0)*t1 + t2*pow(t0, 4.0) - 6 * pow(t0, 3.0)*pow(t1, 2.0) - 4 * t2*pow(t0, 3.0)*t1 + 4 * pow(t0, 2.0)*pow(t1, 3.0) + 6 * t2*pow(t0, 2.0)*pow(t1, 2.0) - t0*pow(t1, 4.0) - 4 * t2*t0*pow(t1, 3.0) + t2*pow(t1, 4.0)) + (a0*(2 * pow(t0, 3.0)*pow(t1, 2.0) + t2*pow(t0, 3.0)*t1 - 3 * t2*pow(t0, 2.0)*pow(t1, 2.0))) / (6 * (pow(t0, 3.0) - 2 * pow(t0, 2.0)*t1 - t2*pow(t0, 2.0) + t0*pow(t1, 2.0) + 2 * t2*t0*t1 - t2*pow(t1, 2.0))) - (p1*(pow(t0, 5.0) - 4 * pow(t0, 4.0)*t1 - t2*pow(t0, 4.0) + 2 * pow(t0, 3.0)*pow(t1, 2.0) + 2 * t2*pow(t0, 3.0)*t1)) / (-pow(t0, 5.0) + 4 * pow(t0, 4.0)*t1 + t2*pow(t0, 4.0) - 6 * pow(t0, 3.0)*pow(t1, 2.0) - 4 * t2*pow(t0, 3.0)*t1 + 4 * pow(t0, 2.0)*pow(t1, 3.0) + 6 * t2*pow(t0, 2.0)*pow(t1, 2.0) - t0*pow(t1, 4.0) - 4 * t2*t0*pow(t1, 3.0) + t2*pow(t1, 4.0)) + (pow(t0, 3.0)*t1*v2) / (pow(t0, 3.0) - 2 * pow(t0, 2.0)*t1 - t2*pow(t0, 2.0) + t0*pow(t1, 2.0) + 2 * t2*t0*t1 - t2*pow(t1, 2.0)) - (2 * p1*pow(t0, 3.0)*t1) / (pow(t0, 3.0)*t1 - pow(t0, 3.0)*t2 - 2 * pow(t0, 2.0)*pow(t1, 2.0) + pow(t0, 2.0)*t1*t2 + pow(t0, 2.0)*pow(t2, 2.0) + t0*pow(t1, 3.0) + t0*pow(t1, 2.0)*t2 - 2 * t0*t1*pow(t2, 2.0) - pow(t1, 3.0)*t2 + pow(t1, 2.0)*pow(t2, 2.0)) + (2 * p2*pow(t0, 3.0)*t1) / (pow(t0, 3.0)*t1 - pow(t0, 3.0)*t2 - 2 * pow(t0, 2.0)*pow(t1, 2.0) + pow(t0, 2.0)*t1*t2 + pow(t0, 2.0)*pow(t2, 2.0) + t0*pow(t1, 3.0) + t0*pow(t1, 2.0)*t2 - 2 * t0*t1*pow(t2, 2.0) - pow(t1, 3.0)*t2 + pow(t1, 2.0)*pow(t2, 2.0));
	P[1] = (2 * p1*(pow(t0, 3.0) + 3 * t1*pow(t0, 2.0))) / (pow(t0, 3.0)*t1 - pow(t0, 3.0)*t2 - 2 * pow(t0, 2.0)*pow(t1, 2.0) + pow(t0, 2.0)*t1*t2 + pow(t0, 2.0)*pow(t2, 2.0) + t0*pow(t1, 3.0) + t0*pow(t1, 2.0)*t2 - 2 * t0*t1*pow(t2, 2.0) - pow(t1, 3.0)*t2 + pow(t1, 2.0)*pow(t2, 2.0)) - (2 * p2*(pow(t0, 3.0) + 3 * t1*pow(t0, 2.0))) / (pow(t0, 3.0)*t1 - pow(t0, 3.0)*t2 - 2 * pow(t0, 2.0)*pow(t1, 2.0) + pow(t0, 2.0)*t1*t2 + pow(t0, 2.0)*pow(t2, 2.0) + t0*pow(t1, 3.0) + t0*pow(t1, 2.0)*t2 - 2 * t0*t1*pow(t2, 2.0) - pow(t1, 3.0)*t2 + pow(t1, 2.0)*pow(t2, 2.0)) - (v0*(-5 * pow(t0, 3.0)*t1 - t2*pow(t0, 3.0) + 3 * t2*pow(t0, 2.0)*t1 + t0*pow(t1, 3.0) + 3 * t2*t0*pow(t1, 2.0) - t2*pow(t1, 3.0))) / (pow(t0, 4.0) - 3 * pow(t0, 3.0)*t1 - t2*pow(t0, 3.0) + 3 * pow(t0, 2.0)*pow(t1, 2.0) + 3 * t2*pow(t0, 2.0)*t1 - t0*pow(t1, 3.0) - 3 * t2*t0*pow(t1, 2.0) + t2*pow(t1, 3.0)) - (a2*(pow(t0, 3.0)*t1 - t2*pow(t0, 3.0) + 3 * pow(t0, 2.0)*pow(t1, 2.0) - 3 * t2*pow(t0, 2.0)*t1)) / (6 * (pow(t0, 3.0) - 2 * pow(t0, 2.0)*t1 - t2*pow(t0, 2.0) + t0*pow(t1, 2.0) + 2 * t2*t0*t1 - t2*pow(t1, 2.0))) - (a0*(5 * pow(t0, 3.0)*t1 + t2*pow(t0, 3.0) + 3 * pow(t0, 2.0)*pow(t1, 2.0) - 3 * t2*pow(t0, 2.0)*t1 - 6 * t2*t0*pow(t1, 2.0))) / (6 * (pow(t0, 3.0) - 2 * pow(t0, 2.0)*t1 - t2*pow(t0, 2.0) + t0*pow(t1, 2.0) + 2 * t2*t0*t1 - t2*pow(t1, 2.0))) - (v2*(pow(t0, 3.0) + 3 * t1*pow(t0, 2.0))) / (pow(t0, 3.0) - 2 * pow(t0, 2.0)*t1 - t2*pow(t0, 2.0) + t0*pow(t1, 2.0) + 2 * t2*t0*t1 - t2*pow(t1, 2.0)) - (2 * p0*(-5 * pow(t0, 3.0)*t1 - t2*pow(t0, 3.0) + 3 * pow(t0, 2.0)*pow(t1, 2.0) + 3 * t2*pow(t0, 2.0)*t1)) / (-pow(t0, 5.0) + 4 * pow(t0, 4.0)*t1 + t2*pow(t0, 4.0) - 6 * pow(t0, 3.0)*pow(t1, 2.0) - 4 * t2*pow(t0, 3.0)*t1 + 4 * pow(t0, 2.0)*pow(t1, 3.0) + 6 * t2*pow(t0, 2.0)*pow(t1, 2.0) - t0*pow(t1, 4.0) - 4 * t2*t0*pow(t1, 3.0) + t2*pow(t1, 4.0)) + (2 * p1*(-5 * pow(t0, 3.0)*t1 - t2*pow(t0, 3.0) + 3 * pow(t0, 2.0)*pow(t1, 2.0) + 3 * t2*pow(t0, 2.0)*t1)) / (-pow(t0, 5.0) + 4 * pow(t0, 4.0)*t1 + t2*pow(t0, 4.0) - 6 * pow(t0, 3.0)*pow(t1, 2.0) - 4 * t2*pow(t0, 3.0)*t1 + 4 * pow(t0, 2.0)*pow(t1, 3.0) + 6 * t2*pow(t0, 2.0)*pow(t1, 2.0) - t0*pow(t1, 4.0) - 4 * t2*t0*pow(t1, 3.0) + t2*pow(t1, 4.0));
	P[2] = (6 * p2*(pow(t0, 2.0) + t1*t0)) / (pow(t0, 3.0)*t1 - pow(t0, 3.0)*t2 - 2 * pow(t0, 2.0)*pow(t1, 2.0) + pow(t0, 2.0)*t1*t2 + pow(t0, 2.0)*pow(t2, 2.0) + t0*pow(t1, 3.0) + t0*pow(t1, 2.0)*t2 - 2 * t0*t1*pow(t2, 2.0) - pow(t1, 3.0)*t2 + pow(t1, 2.0)*pow(t2, 2.0)) - (6 * p1*(pow(t0, 2.0) + t1*t0)) / (pow(t0, 3.0)*t1 - pow(t0, 3.0)*t2 - 2 * pow(t0, 2.0)*pow(t1, 2.0) + pow(t0, 2.0)*t1*t2 + pow(t0, 2.0)*pow(t2, 2.0) + t0*pow(t1, 3.0) + t0*pow(t1, 2.0)*t2 - 2 * t0*t1*pow(t2, 2.0) - pow(t1, 3.0)*t2 + pow(t1, 2.0)*pow(t2, 2.0)) + (3 * v2*(pow(t0, 2.0) + t1*t0)) / (pow(t0, 3.0) - 2 * pow(t0, 2.0)*t1 - t2*pow(t0, 2.0) + t0*pow(t1, 2.0) + 2 * t2*t0*t1 - t2*pow(t1, 2.0)) + (6 * p0*(-pow(t0, 3.0) - pow(t0, 2.0)*t1 + t0*pow(t1, 2.0) + t2*t0*t1)) / (-pow(t0, 5.0) + 4 * pow(t0, 4.0)*t1 + t2*pow(t0, 4.0) - 6 * pow(t0, 3.0)*pow(t1, 2.0) - 4 * t2*pow(t0, 3.0)*t1 + 4 * pow(t0, 2.0)*pow(t1, 3.0) + 6 * t2*pow(t0, 2.0)*pow(t1, 2.0) - t0*pow(t1, 4.0) - 4 * t2*t0*pow(t1, 3.0) + t2*pow(t1, 4.0)) - (6 * p1*(-pow(t0, 3.0) - pow(t0, 2.0)*t1 + t0*pow(t1, 2.0) + t2*t0*t1)) / (-pow(t0, 5.0) + 4 * pow(t0, 4.0)*t1 + t2*pow(t0, 4.0) - 6 * pow(t0, 3.0)*pow(t1, 2.0) - 4 * t2*pow(t0, 3.0)*t1 + 4 * pow(t0, 2.0)*pow(t1, 3.0) + 6 * t2*pow(t0, 2.0)*pow(t1, 2.0) - t0*pow(t1, 4.0) - 4 * t2*t0*pow(t1, 3.0) + t2*pow(t1, 4.0)) + (a2*(pow(t0, 2.0)*t1 - t2*pow(t0, 2.0) + t0*pow(t1, 2.0) - t2*t0*t1)) / (2 * (pow(t0, 3.0) - 2 * pow(t0, 2.0)*t1 - t2*pow(t0, 2.0) + t0*pow(t1, 2.0) + 2 * t2*t0*t1 - t2*pow(t1, 2.0))) + (3 * v0*(-pow(t0, 3.0) - 2 * pow(t0, 2.0)*t1 + t0*pow(t1, 2.0) + 2 * t2*t0*t1)) / (pow(t0, 4.0) - 3 * pow(t0, 3.0)*t1 - t2*pow(t0, 3.0) + 3 * pow(t0, 2.0)*pow(t1, 2.0) + 3 * t2*pow(t0, 2.0)*t1 - t0*pow(t1, 3.0) - 3 * t2*t0*pow(t1, 2.0) + t2*pow(t1, 3.0)) + (a0*(pow(t0, 3.0) + 3 * pow(t0, 2.0)*t1 - 3 * t2*t0*t1 - t2*pow(t1, 2.0))) / (2 * (pow(t0, 3.0) - 2 * pow(t0, 2.0)*t1 - t2*pow(t0, 2.0) + t0*pow(t1, 2.0) + 2 * t2*t0*t1 - t2*pow(t1, 2.0)));
	P[3] = (2 * p1*(3 * t0 + t1)) / (pow(t0, 3.0)*t1 - pow(t0, 3.0)*t2 - 2 * pow(t0, 2.0)*pow(t1, 2.0) + pow(t0, 2.0)*t1*t2 + pow(t0, 2.0)*pow(t2, 2.0) + t0*pow(t1, 3.0) + t0*pow(t1, 2.0)*t2 - 2 * t0*t1*pow(t2, 2.0) - pow(t1, 3.0)*t2 + pow(t1, 2.0)*pow(t2, 2.0)) - (v0*(-5 * pow(t0, 2.0) + 2 * t2*t0 + pow(t1, 2.0) + 2 * t2*t1)) / (pow(t0, 4.0) - 3 * pow(t0, 3.0)*t1 - t2*pow(t0, 3.0) + 3 * pow(t0, 2.0)*pow(t1, 2.0) + 3 * t2*pow(t0, 2.0)*t1 - t0*pow(t1, 3.0) - 3 * t2*t0*pow(t1, 2.0) + t2*pow(t1, 3.0)) - (2 * p2*(3 * t0 + t1)) / (pow(t0, 3.0)*t1 - pow(t0, 3.0)*t2 - 2 * pow(t0, 2.0)*pow(t1, 2.0) + pow(t0, 2.0)*t1*t2 + pow(t0, 2.0)*pow(t2, 2.0) + t0*pow(t1, 3.0) + t0*pow(t1, 2.0)*t2 - 2 * t0*t1*pow(t2, 2.0) - pow(t1, 3.0)*t2 + pow(t1, 2.0)*pow(t2, 2.0)) - (2 * p0*(-4 * pow(t0, 2.0) + t0*t1 + t2*t0 + pow(t1, 2.0) + t2*t1)) / (-pow(t0, 5.0) + 4 * pow(t0, 4.0)*t1 + t2*pow(t0, 4.0) - 6 * pow(t0, 3.0)*pow(t1, 2.0) - 4 * t2*pow(t0, 3.0)*t1 + 4 * pow(t0, 2.0)*pow(t1, 3.0) + 6 * t2*pow(t0, 2.0)*pow(t1, 2.0) - t0*pow(t1, 4.0) - 4 * t2*t0*pow(t1, 3.0) + t2*pow(t1, 4.0)) + (2 * p1*(-4 * pow(t0, 2.0) + t0*t1 + t2*t0 + pow(t1, 2.0) + t2*t1)) / (-pow(t0, 5.0) + 4 * pow(t0, 4.0)*t1 + t2*pow(t0, 4.0) - 6 * pow(t0, 3.0)*pow(t1, 2.0) - 4 * t2*pow(t0, 3.0)*t1 + 4 * pow(t0, 2.0)*pow(t1, 3.0) + 6 * t2*pow(t0, 2.0)*pow(t1, 2.0) - t0*pow(t1, 4.0) - 4 * t2*t0*pow(t1, 3.0) + t2*pow(t1, 4.0)) - (v2*(3 * t0 + t1)) / (pow(t0, 3.0) - 2 * pow(t0, 2.0)*t1 - t2*pow(t0, 2.0) + t0*pow(t1, 2.0) + 2 * t2*t0*t1 - t2*pow(t1, 2.0)) - (a2*(3 * t0*t1 - 3 * t0*t2 - t1*t2 + pow(t1, 2.0))) / (6 * (pow(t0, 3.0) - 2 * pow(t0, 2.0)*t1 - t2*pow(t0, 2.0) + t0*pow(t1, 2.0) + 2 * t2*t0*t1 - t2*pow(t1, 2.0))) + (a0*(-6 * pow(t0, 2.0) - 3 * t0*t1 + 3 * t2*t0 + pow(t1, 2.0) + 5 * t2*t1)) / (6 * (pow(t0, 3.0) - 2 * pow(t0, 2.0)*t1 - t2*pow(t0, 2.0) + t0*pow(t1, 2.0) + 2 * t2*t0*t1 - t2*pow(t1, 2.0)));
	P[4] = (2 * p2) / (pow(t0, 3.0)*t1 - pow(t0, 3.0)*t2 - 2 * pow(t0, 2.0)*pow(t1, 2.0) + pow(t0, 2.0)*t1*t2 + pow(t0, 2.0)*pow(t2, 2.0) + t0*pow(t1, 3.0) + t0*pow(t1, 2.0)*t2 - 2 * t0*t1*pow(t2, 2.0) - pow(t1, 3.0)*t2 + pow(t1, 2.0)*pow(t2, 2.0)) - (2 * p1) / (pow(t0, 3.0)*t1 - pow(t0, 3.0)*t2 - 2 * pow(t0, 2.0)*pow(t1, 2.0) + pow(t0, 2.0)*t1*t2 + pow(t0, 2.0)*pow(t2, 2.0) + t0*pow(t1, 3.0) + t0*pow(t1, 2.0)*t2 - 2 * t0*t1*pow(t2, 2.0) - pow(t1, 3.0)*t2 + pow(t1, 2.0)*pow(t2, 2.0)) + v2 / (pow(t0, 3.0) - 2 * pow(t0, 2.0)*t1 - t2*pow(t0, 2.0) + t0*pow(t1, 2.0) + 2 * t2*t0*t1 - t2*pow(t1, 2.0)) - (a0*(t1 - 3 * t0 + 2 * t2)) / (6 * (pow(t0, 3.0) - 2 * pow(t0, 2.0)*t1 - t2*pow(t0, 2.0) + t0*pow(t1, 2.0) + 2 * t2*t0*t1 - t2*pow(t1, 2.0))) + (a2*(t1 - t2)) / (6 * (pow(t0, 3.0) - 2 * pow(t0, 2.0)*t1 - t2*pow(t0, 2.0) + t0*pow(t1, 2.0) + 2 * t2*t0*t1 - t2*pow(t1, 2.0))) + (v0*(t1 - 2 * t0 + t2)) / (pow(t0, 4.0) - 3 * pow(t0, 3.0)*t1 - t2*pow(t0, 3.0) + 3 * pow(t0, 2.0)*pow(t1, 2.0) + 3 * t2*pow(t0, 2.0)*t1 - t0*pow(t1, 3.0) - 3 * t2*t0*pow(t1, 2.0) + t2*pow(t1, 3.0)) + (p0*(2 * t1 - 3 * t0 + t2)) / (-pow(t0, 5.0) + 4 * pow(t0, 4.0)*t1 + t2*pow(t0, 4.0) - 6 * pow(t0, 3.0)*pow(t1, 2.0) - 4 * t2*pow(t0, 3.0)*t1 + 4 * pow(t0, 2.0)*pow(t1, 3.0) + 6 * t2*pow(t0, 2.0)*pow(t1, 2.0) - t0*pow(t1, 4.0) - 4 * t2*t0*pow(t1, 3.0) + t2*pow(t1, 4.0)) - (p1*(2 * t1 - 3 * t0 + t2)) / (-pow(t0, 5.0) + 4 * pow(t0, 4.0)*t1 + t2*pow(t0, 4.0) - 6 * pow(t0, 3.0)*pow(t1, 2.0) - 4 * t2*pow(t0, 3.0)*t1 + 4 * pow(t0, 2.0)*pow(t1, 3.0) + 6 * t2*pow(t0, 2.0)*pow(t1, 2.0) - t0*pow(t1, 4.0) - 4 * t2*t0*pow(t1, 3.0) + t2*pow(t1, 4.0));
	P[5] = (p2*(pow(t1, 4.0)*t2 - t0*pow(t1, 4.0) - 4 * pow(t1, 3.0)*pow(t2, 2.0) + 4 * t0*pow(t1, 3.0)*t2 + 4 * pow(t1, 2.0)*pow(t2, 3.0) - 6 * t0*pow(t1, 2.0)*pow(t2, 2.0) + 2 * t0*t1*pow(t2, 3.0))) / (pow(t1, 4.0)*t2 - t0*pow(t1, 4.0) - 4 * pow(t1, 3.0)*pow(t2, 2.0) + 4 * t0*pow(t1, 3.0)*t2 + 6 * pow(t1, 2.0)*pow(t2, 3.0) - 6 * t0*pow(t1, 2.0)*pow(t2, 2.0) - 4 * t1*pow(t2, 4.0) + 4 * t0*t1*pow(t2, 3.0) + pow(t2, 5.0) - t0*pow(t2, 4.0)) - (a2*(2 * pow(t1, 2.0)*pow(t2, 3.0) - 3 * t0*pow(t1, 2.0)*pow(t2, 2.0) + t0*t1*pow(t2, 3.0))) / (6 * (-pow(t1, 2.0)*t2 + t0*pow(t1, 2.0) + 2 * t1*pow(t2, 2.0) - 2 * t0*t1*t2 - pow(t2, 3.0) + t0*pow(t2, 2.0))) - (v2*(-pow(t1, 3.0)*pow(t2, 2.0) + t0*pow(t1, 3.0)*t2 + 2 * pow(t1, 2.0)*pow(t2, 3.0) - 3 * t0*pow(t1, 2.0)*pow(t2, 2.0) + t0*t1*pow(t2, 3.0))) / (-pow(t1, 3.0)*t2 + t0*pow(t1, 3.0) + 3 * pow(t1, 2.0)*pow(t2, 2.0) - 3 * t0*pow(t1, 2.0)*t2 - 3 * t1*pow(t2, 3.0) + 3 * t0*t1*pow(t2, 2.0) + pow(t2, 4.0) - t0*pow(t2, 3.0)) + (p1*(2 * pow(t1, 2.0)*pow(t2, 3.0) - 4 * t1*pow(t2, 4.0) + 2 * t0*t1*pow(t2, 3.0) + pow(t2, 5.0) - t0*pow(t2, 4.0))) / (pow(t1, 4.0)*t2 - t0*pow(t1, 4.0) - 4 * pow(t1, 3.0)*pow(t2, 2.0) + 4 * t0*pow(t1, 3.0)*t2 + 6 * pow(t1, 2.0)*pow(t2, 3.0) - 6 * t0*pow(t1, 2.0)*pow(t2, 2.0) - 4 * t1*pow(t2, 4.0) + 4 * t0*t1*pow(t2, 3.0) + pow(t2, 5.0) - t0*pow(t2, 4.0)) - (a0*(pow(t1, 2.0)*pow(t2, 3.0) - t0*t1*pow(t2, 3.0))) / (6 * (-pow(t1, 2.0)*t2 + t0*pow(t1, 2.0) + 2 * t1*pow(t2, 2.0) - 2 * t0*t1*t2 - pow(t2, 3.0) + t0*pow(t2, 2.0))) - (t1*pow(t2, 3.0)*v0) / (-pow(t1, 2.0)*t2 + t0*pow(t1, 2.0) + 2 * t1*pow(t2, 2.0) - 2 * t0*t1*t2 - pow(t2, 3.0) + t0*pow(t2, 2.0)) + (2 * p0*t1*pow(t2, 3.0)) / (pow(t0, 2.0)*pow(t1, 2.0) - 2 * pow(t0, 2.0)*t1*t2 + pow(t0, 2.0)*pow(t2, 2.0) - t0*pow(t1, 3.0) + t0*pow(t1, 2.0)*t2 + t0*t1*pow(t2, 2.0) - t0*pow(t2, 3.0) + pow(t1, 3.0)*t2 - 2 * pow(t1, 2.0)*pow(t2, 2.0) + t1*pow(t2, 3.0)) - (2 * p1*t1*pow(t2, 3.0)) / (pow(t0, 2.0)*pow(t1, 2.0) - 2 * pow(t0, 2.0)*t1*t2 + pow(t0, 2.0)*pow(t2, 2.0) - t0*pow(t1, 3.0) + t0*pow(t1, 2.0)*t2 + t0*t1*pow(t2, 2.0) - t0*pow(t2, 3.0) + pow(t1, 3.0)*t2 - 2 * pow(t1, 2.0)*pow(t2, 2.0) + t1*pow(t2, 3.0));
	P[6] = (2 * p1*(pow(t2, 3.0) + 3 * t1*pow(t2, 2.0))) / (pow(t0, 2.0)*pow(t1, 2.0) - 2 * pow(t0, 2.0)*t1*t2 + pow(t0, 2.0)*pow(t2, 2.0) - t0*pow(t1, 3.0) + t0*pow(t1, 2.0)*t2 + t0*t1*pow(t2, 2.0) - t0*pow(t2, 3.0) + pow(t1, 3.0)*t2 - 2 * pow(t1, 2.0)*pow(t2, 2.0) + t1*pow(t2, 3.0)) - (2 * p0*(pow(t2, 3.0) + 3 * t1*pow(t2, 2.0))) / (pow(t0, 2.0)*pow(t1, 2.0) - 2 * pow(t0, 2.0)*t1*t2 + pow(t0, 2.0)*pow(t2, 2.0) - t0*pow(t1, 3.0) + t0*pow(t1, 2.0)*t2 + t0*t1*pow(t2, 2.0) - t0*pow(t2, 3.0) + pow(t1, 3.0)*t2 - 2 * pow(t1, 2.0)*pow(t2, 2.0) + t1*pow(t2, 3.0)) + (v2*(-pow(t1, 3.0)*t2 + t0*pow(t1, 3.0) - 3 * t0*pow(t1, 2.0)*t2 + 5 * t1*pow(t2, 3.0) - 3 * t0*t1*pow(t2, 2.0) + t0*pow(t2, 3.0))) / (-pow(t1, 3.0)*t2 + t0*pow(t1, 3.0) + 3 * pow(t1, 2.0)*pow(t2, 2.0) - 3 * t0*pow(t1, 2.0)*t2 - 3 * t1*pow(t2, 3.0) + 3 * t0*t1*pow(t2, 2.0) + pow(t2, 4.0) - t0*pow(t2, 3.0)) - (2 * p1*(3 * pow(t1, 2.0)*pow(t2, 2.0) - 5 * t1*pow(t2, 3.0) + 3 * t0*t1*pow(t2, 2.0) - t0*pow(t2, 3.0))) / (pow(t1, 4.0)*t2 - t0*pow(t1, 4.0) - 4 * pow(t1, 3.0)*pow(t2, 2.0) + 4 * t0*pow(t1, 3.0)*t2 + 6 * pow(t1, 2.0)*pow(t2, 3.0) - 6 * t0*pow(t1, 2.0)*pow(t2, 2.0) - 4 * t1*pow(t2, 4.0) + 4 * t0*t1*pow(t2, 3.0) + pow(t2, 5.0) - t0*pow(t2, 4.0)) + (2 * p2*(3 * pow(t1, 2.0)*pow(t2, 2.0) - 5 * t1*pow(t2, 3.0) + 3 * t0*t1*pow(t2, 2.0) - t0*pow(t2, 3.0))) / (pow(t1, 4.0)*t2 - t0*pow(t1, 4.0) - 4 * pow(t1, 3.0)*pow(t2, 2.0) + 4 * t0*pow(t1, 3.0)*t2 + 6 * pow(t1, 2.0)*pow(t2, 3.0) - 6 * t0*pow(t1, 2.0)*pow(t2, 2.0) - 4 * t1*pow(t2, 4.0) + 4 * t0*t1*pow(t2, 3.0) + pow(t2, 5.0) - t0*pow(t2, 4.0)) + (a0*(3 * pow(t1, 2.0)*pow(t2, 2.0) + t1*pow(t2, 3.0) - 3 * t0*t1*pow(t2, 2.0) - t0*pow(t2, 3.0))) / (6 * (-pow(t1, 2.0)*t2 + t0*pow(t1, 2.0) + 2 * t1*pow(t2, 2.0) - 2 * t0*t1*t2 - pow(t2, 3.0) + t0*pow(t2, 2.0))) + (a2*(3 * pow(t1, 2.0)*pow(t2, 2.0) - 6 * t0*pow(t1, 2.0)*t2 + 5 * t1*pow(t2, 3.0) - 3 * t0*t1*pow(t2, 2.0) + t0*pow(t2, 3.0))) / (6 * (-pow(t1, 2.0)*t2 + t0*pow(t1, 2.0) + 2 * t1*pow(t2, 2.0) - 2 * t0*t1*t2 - pow(t2, 3.0) + t0*pow(t2, 2.0))) + (v0*(pow(t2, 3.0) + 3 * t1*pow(t2, 2.0))) / (-pow(t1, 2.0)*t2 + t0*pow(t1, 2.0) + 2 * t1*pow(t2, 2.0) - 2 * t0*t1*t2 - pow(t2, 3.0) + t0*pow(t2, 2.0));
	P[7] = (6 * p0*(pow(t2, 2.0) + t1*t2)) / (pow(t0, 2.0)*pow(t1, 2.0) - 2 * pow(t0, 2.0)*t1*t2 + pow(t0, 2.0)*pow(t2, 2.0) - t0*pow(t1, 3.0) + t0*pow(t1, 2.0)*t2 + t0*t1*pow(t2, 2.0) - t0*pow(t2, 3.0) + pow(t1, 3.0)*t2 - 2 * pow(t1, 2.0)*pow(t2, 2.0) + t1*pow(t2, 3.0)) - (6 * p1*(pow(t2, 2.0) + t1*t2)) / (pow(t0, 2.0)*pow(t1, 2.0) - 2 * pow(t0, 2.0)*t1*t2 + pow(t0, 2.0)*pow(t2, 2.0) - t0*pow(t1, 3.0) + t0*pow(t1, 2.0)*t2 + t0*t1*pow(t2, 2.0) - t0*pow(t2, 3.0) + pow(t1, 3.0)*t2 - 2 * pow(t1, 2.0)*pow(t2, 2.0) + t1*pow(t2, 3.0)) + (a0*(-pow(t1, 2.0)*t2 - t1*pow(t2, 2.0) + t0*t1*t2 + t0*pow(t2, 2.0))) / (2 * (-pow(t1, 2.0)*t2 + t0*pow(t1, 2.0) + 2 * t1*pow(t2, 2.0) - 2 * t0*t1*t2 - pow(t2, 3.0) + t0*pow(t2, 2.0))) - (3 * v2*(-pow(t1, 2.0)*t2 + 2 * t1*pow(t2, 2.0) - 2 * t0*t1*t2 + pow(t2, 3.0))) / (-pow(t1, 3.0)*t2 + t0*pow(t1, 3.0) + 3 * pow(t1, 2.0)*pow(t2, 2.0) - 3 * t0*pow(t1, 2.0)*t2 - 3 * t1*pow(t2, 3.0) + 3 * t0*t1*pow(t2, 2.0) + pow(t2, 4.0) - t0*pow(t2, 3.0)) - (3 * v0*(pow(t2, 2.0) + t1*t2)) / (-pow(t1, 2.0)*t2 + t0*pow(t1, 2.0) + 2 * t1*pow(t2, 2.0) - 2 * t0*t1*t2 - pow(t2, 3.0) + t0*pow(t2, 2.0)) + (a2*(t0*pow(t1, 2.0) - 3 * t1*pow(t2, 2.0) + 3 * t0*t1*t2 - pow(t2, 3.0))) / (2 * (-pow(t1, 2.0)*t2 + t0*pow(t1, 2.0) + 2 * t1*pow(t2, 2.0) - 2 * t0*t1*t2 - pow(t2, 3.0) + t0*pow(t2, 2.0))) - (6 * p1*(-pow(t1, 2.0)*t2 + t1*pow(t2, 2.0) - t0*t1*t2 + pow(t2, 3.0))) / (pow(t1, 4.0)*t2 - t0*pow(t1, 4.0) - 4 * pow(t1, 3.0)*pow(t2, 2.0) + 4 * t0*pow(t1, 3.0)*t2 + 6 * pow(t1, 2.0)*pow(t2, 3.0) - 6 * t0*pow(t1, 2.0)*pow(t2, 2.0) - 4 * t1*pow(t2, 4.0) + 4 * t0*t1*pow(t2, 3.0) + pow(t2, 5.0) - t0*pow(t2, 4.0)) + (6 * p2*(-pow(t1, 2.0)*t2 + t1*pow(t2, 2.0) - t0*t1*t2 + pow(t2, 3.0))) / (pow(t1, 4.0)*t2 - t0*pow(t1, 4.0) - 4 * pow(t1, 3.0)*pow(t2, 2.0) + 4 * t0*pow(t1, 3.0)*t2 + 6 * pow(t1, 2.0)*pow(t2, 3.0) - 6 * t0*pow(t1, 2.0)*pow(t2, 2.0) - 4 * t1*pow(t2, 4.0) + 4 * t0*t1*pow(t2, 3.0) + pow(t2, 5.0) - t0*pow(t2, 4.0));
	P[8] = (2 * p1*(t1 + 3 * t2)) / (pow(t0, 2.0)*pow(t1, 2.0) - 2 * pow(t0, 2.0)*t1*t2 + pow(t0, 2.0)*pow(t2, 2.0) - t0*pow(t1, 3.0) + t0*pow(t1, 2.0)*t2 + t0*t1*pow(t2, 2.0) - t0*pow(t2, 3.0) + pow(t1, 3.0)*t2 - 2 * pow(t1, 2.0)*pow(t2, 2.0) + t1*pow(t2, 3.0)) - (2 * p0*(t1 + 3 * t2)) / (pow(t0, 2.0)*pow(t1, 2.0) - 2 * pow(t0, 2.0)*t1*t2 + pow(t0, 2.0)*pow(t2, 2.0) - t0*pow(t1, 3.0) + t0*pow(t1, 2.0)*t2 + t0*t1*pow(t2, 2.0) - t0*pow(t2, 3.0) + pow(t1, 3.0)*t2 - 2 * pow(t1, 2.0)*pow(t2, 2.0) + t1*pow(t2, 3.0)) - (v2*(pow(t1, 2.0) + 2 * t0*t1 - 5 * pow(t2, 2.0) + 2 * t0*t2)) / (-pow(t1, 3.0)*t2 + t0*pow(t1, 3.0) + 3 * pow(t1, 2.0)*pow(t2, 2.0) - 3 * t0*pow(t1, 2.0)*t2 - 3 * t1*pow(t2, 3.0) + 3 * t0*t1*pow(t2, 2.0) + pow(t2, 4.0) - t0*pow(t2, 3.0)) - (a2*(pow(t1, 2.0) - 3 * t1*t2 + 5 * t0*t1 - 6 * pow(t2, 2.0) + 3 * t0*t2)) / (6 * (-pow(t1, 2.0)*t2 + t0*pow(t1, 2.0) + 2 * t1*pow(t2, 2.0) - 2 * t0*t1*t2 - pow(t2, 3.0) + t0*pow(t2, 2.0))) - (a0*(t0*t1 + 3 * t0*t2 - 3 * t1*t2 - pow(t1, 2.0))) / (6 * (-pow(t1, 2.0)*t2 + t0*pow(t1, 2.0) + 2 * t1*pow(t2, 2.0) - 2 * t0*t1*t2 - pow(t2, 3.0) + t0*pow(t2, 2.0))) + (v0*(t1 + 3 * t2)) / (-pow(t1, 2.0)*t2 + t0*pow(t1, 2.0) + 2 * t1*pow(t2, 2.0) - 2 * t0*t1*t2 - pow(t2, 3.0) + t0*pow(t2, 2.0)) - (2 * p1*(pow(t1, 2.0) + t1*t2 + t0*t1 - 4 * pow(t2, 2.0) + t0*t2)) / (pow(t1, 4.0)*t2 - t0*pow(t1, 4.0) - 4 * pow(t1, 3.0)*pow(t2, 2.0) + 4 * t0*pow(t1, 3.0)*t2 + 6 * pow(t1, 2.0)*pow(t2, 3.0) - 6 * t0*pow(t1, 2.0)*pow(t2, 2.0) - 4 * t1*pow(t2, 4.0) + 4 * t0*t1*pow(t2, 3.0) + pow(t2, 5.0) - t0*pow(t2, 4.0)) + (2 * p2*(pow(t1, 2.0) + t1*t2 + t0*t1 - 4 * pow(t2, 2.0) + t0*t2)) / (pow(t1, 4.0)*t2 - t0*pow(t1, 4.0) - 4 * pow(t1, 3.0)*pow(t2, 2.0) + 4 * t0*pow(t1, 3.0)*t2 + 6 * pow(t1, 2.0)*pow(t2, 3.0) - 6 * t0*pow(t1, 2.0)*pow(t2, 2.0) - 4 * t1*pow(t2, 4.0) + 4 * t0*t1*pow(t2, 3.0) + pow(t2, 5.0) - t0*pow(t2, 4.0));
	P[9] = (2 * p0) / (pow(t0, 2.0)*pow(t1, 2.0) - 2 * pow(t0, 2.0)*t1*t2 + pow(t0, 2.0)*pow(t2, 2.0) - t0*pow(t1, 3.0) + t0*pow(t1, 2.0)*t2 + t0*t1*pow(t2, 2.0) - t0*pow(t2, 3.0) + pow(t1, 3.0)*t2 - 2 * pow(t1, 2.0)*pow(t2, 2.0) + t1*pow(t2, 3.0)) - v0 / (-pow(t1, 2.0)*t2 + t0*pow(t1, 2.0) + 2 * t1*pow(t2, 2.0) - 2 * t0*t1*t2 - pow(t2, 3.0) + t0*pow(t2, 2.0)) - (2 * p1) / (pow(t0, 2.0)*pow(t1, 2.0) - 2 * pow(t0, 2.0)*t1*t2 + pow(t0, 2.0)*pow(t2, 2.0) - t0*pow(t1, 3.0) + t0*pow(t1, 2.0)*t2 + t0*t1*pow(t2, 2.0) - t0*pow(t2, 3.0) + pow(t1, 3.0)*t2 - 2 * pow(t1, 2.0)*pow(t2, 2.0) + t1*pow(t2, 3.0)) + (p1*(t0 + 2 * t1 - 3 * t2)) / (pow(t1, 4.0)*t2 - t0*pow(t1, 4.0) - 4 * pow(t1, 3.0)*pow(t2, 2.0) + 4 * t0*pow(t1, 3.0)*t2 + 6 * pow(t1, 2.0)*pow(t2, 3.0) - 6 * t0*pow(t1, 2.0)*pow(t2, 2.0) - 4 * t1*pow(t2, 4.0) + 4 * t0*t1*pow(t2, 3.0) + pow(t2, 5.0) - t0*pow(t2, 4.0)) - (p2*(t0 + 2 * t1 - 3 * t2)) / (pow(t1, 4.0)*t2 - t0*pow(t1, 4.0) - 4 * pow(t1, 3.0)*pow(t2, 2.0) + 4 * t0*pow(t1, 3.0)*t2 + 6 * pow(t1, 2.0)*pow(t2, 3.0) - 6 * t0*pow(t1, 2.0)*pow(t2, 2.0) - 4 * t1*pow(t2, 4.0) + 4 * t0*t1*pow(t2, 3.0) + pow(t2, 5.0) - t0*pow(t2, 4.0)) + (v2*(t0 + t1 - 2 * t2)) / (-pow(t1, 3.0)*t2 + t0*pow(t1, 3.0) + 3 * pow(t1, 2.0)*pow(t2, 2.0) - 3 * t0*pow(t1, 2.0)*t2 - 3 * t1*pow(t2, 3.0) + 3 * t0*t1*pow(t2, 2.0) + pow(t2, 4.0) - t0*pow(t2, 3.0)) + (a2*(2 * t0 + t1 - 3 * t2)) / (6 * (-pow(t1, 2.0)*t2 + t0*pow(t1, 2.0) + 2 * t1*pow(t2, 2.0) - 2 * t0*t1*t2 - pow(t2, 3.0) + t0*pow(t2, 2.0))) + (a0*(t0 - t1)) / (6 * (-pow(t1, 2.0)*t2 + t0*pow(t1, 2.0) + 2 * t1*pow(t2, 2.0) - 2 * t0*t1*t2 - pow(t2, 3.0) + t0*pow(t2, 2.0)));

	for (i = (int)(t0 / control_t); i < (int)(t1 / control_t); i++)
	{
		S[k] = P[0] + P[1] * i*control_t + P[2] * i*control_t*i*control_t + P[3] * i*control_t*control_t*control_t*i*i + P[4] * i*i*i*i*control_t*control_t*control_t*control_t;
		k = k + 1;
	}

	for (i = (int)(t1 / control_t); i <= (int)(t2 / control_t); i++)
	{
		S[k] = P[5] + P[6] * i*control_t + P[7] * i*i*control_t*control_t + P[8] * i*i*i*control_t*control_t*control_t + P[9] * i*i*i*i*control_t*control_t*control_t*control_t;
		k = k + 1;
	}

}

void Free_Ankle_Com(double control_t, int n_stop, int n_tcom, double walk_t)
{
	int N = Motion_Position_in.Mark_Number;
	for (int i = 0; i < N - 1; i++)
	{
		TSpline_PVA(Motion_Position_in.Mark.Com_x[i], 0.0, 0.0, Motion_Position_in.Mark_Time[i], 0.5 * (Motion_Position_in.Mark.Com_x[i] + Motion_Position_in.Mark.Com_x[i + 1]), 0.5 * (Motion_Position_in.Mark_Time[i] + Motion_Position_in.Mark_Time[i + 1]), Motion_Position_in.Mark.Com_x[i + 1], 0.0, 0.0, Motion_Position_in.Mark_Time[i + 1], control_t, (Motion_Position_out.OutPut.Com_x + (int)(Motion_Position_in.Mark_Time[i] / control_t)));
		TSpline_PVA(Motion_Position_in.Mark.Com_y[i], 0.0, 0.0, Motion_Position_in.Mark_Time[i], 0.5 * (Motion_Position_in.Mark.Com_y[i] + Motion_Position_in.Mark.Com_y[i + 1]), 0.5 * (Motion_Position_in.Mark_Time[i] + Motion_Position_in.Mark_Time[i + 1]), Motion_Position_in.Mark.Com_y[i + 1], 0.0, 0.0, Motion_Position_in.Mark_Time[i + 1], control_t, (Motion_Position_out.OutPut.Com_y + (int)(Motion_Position_in.Mark_Time[i] / control_t)));
		TSpline_PVA(Motion_Position_in.Mark.Com_z[i], 0.0, 0.0, Motion_Position_in.Mark_Time[i], 0.5 * (Motion_Position_in.Mark.Com_z[i] + Motion_Position_in.Mark.Com_z[i + 1]), 0.5 * (Motion_Position_in.Mark_Time[i] + Motion_Position_in.Mark_Time[i + 1]), Motion_Position_in.Mark.Com_z[i + 1], 0.0, 0.0, Motion_Position_in.Mark_Time[i + 1], control_t, (Motion_Position_out.OutPut.Com_z + (int)(Motion_Position_in.Mark_Time[i] / control_t)));
		TSpline_PVA(Motion_Position_in.Mark.Ankle_L_x[i], 0.0, 0.0, Motion_Position_in.Mark_Time[i], 0.5 * (Motion_Position_in.Mark.Ankle_L_x[i] + Motion_Position_in.Mark.Ankle_L_x[i + 1]), 0.5 * (Motion_Position_in.Mark_Time[i] + Motion_Position_in.Mark_Time[i + 1]), Motion_Position_in.Mark.Ankle_L_x[i + 1], 0.0, 0.0, Motion_Position_in.Mark_Time[i + 1], control_t, (Motion_Position_out.OutPut.Ankle_L_x + (int)(Motion_Position_in.Mark_Time[i] / control_t)));
		TSpline_PVA(Motion_Position_in.Mark.Ankle_L_y[i], 0.0, 0.0, Motion_Position_in.Mark_Time[i], 0.5 * (Motion_Position_in.Mark.Ankle_L_y[i] + Motion_Position_in.Mark.Ankle_L_y[i + 1]), 0.5 * (Motion_Position_in.Mark_Time[i] + Motion_Position_in.Mark_Time[i + 1]), Motion_Position_in.Mark.Ankle_L_y[i + 1], 0.0, 0.0, Motion_Position_in.Mark_Time[i + 1], control_t, (Motion_Position_out.OutPut.Ankle_L_y + (int)(Motion_Position_in.Mark_Time[i] / control_t)));
		TSpline_PVA(Motion_Position_in.Mark.Ankle_L_z[i], 0.0, 0.0, Motion_Position_in.Mark_Time[i], 0.5 * (Motion_Position_in.Mark.Ankle_L_z[i] + Motion_Position_in.Mark.Ankle_L_z[i + 1]), 0.5 * (Motion_Position_in.Mark_Time[i] + Motion_Position_in.Mark_Time[i + 1]), Motion_Position_in.Mark.Ankle_L_z[i + 1], 0.0, 0.0, Motion_Position_in.Mark_Time[i + 1], control_t, (Motion_Position_out.OutPut.Ankle_L_z + (int)(Motion_Position_in.Mark_Time[i] / control_t)));
		TSpline_PVA(Motion_Position_in.Mark.Ankle_R_x[i], 0.0, 0.0, Motion_Position_in.Mark_Time[i], 0.5 * (Motion_Position_in.Mark.Ankle_R_x[i] + Motion_Position_in.Mark.Ankle_R_x[i + 1]), 0.5 * (Motion_Position_in.Mark_Time[i] + Motion_Position_in.Mark_Time[i + 1]), Motion_Position_in.Mark.Ankle_R_x[i + 1], 0.0, 0.0, Motion_Position_in.Mark_Time[i + 1], control_t, (Motion_Position_out.OutPut.Ankle_R_x + (int)(Motion_Position_in.Mark_Time[i] / control_t)));
		TSpline_PVA(Motion_Position_in.Mark.Ankle_R_y[i], 0.0, 0.0, Motion_Position_in.Mark_Time[i], 0.5 * (Motion_Position_in.Mark.Ankle_R_y[i] + Motion_Position_in.Mark.Ankle_R_y[i + 1]), 0.5 * (Motion_Position_in.Mark_Time[i] + Motion_Position_in.Mark_Time[i + 1]), Motion_Position_in.Mark.Ankle_R_y[i + 1], 0.0, 0.0, Motion_Position_in.Mark_Time[i + 1], control_t, (Motion_Position_out.OutPut.Ankle_R_y + (int)(Motion_Position_in.Mark_Time[i] / control_t)));
		TSpline_PVA(Motion_Position_in.Mark.Ankle_R_z[i], 0.0, 0.0, Motion_Position_in.Mark_Time[i], 0.5 * (Motion_Position_in.Mark.Ankle_R_z[i] + Motion_Position_in.Mark.Ankle_R_z[i + 1]), 0.5 * (Motion_Position_in.Mark_Time[i] + Motion_Position_in.Mark_Time[i + 1]), Motion_Position_in.Mark.Ankle_R_z[i + 1], 0.0, 0.0, Motion_Position_in.Mark_Time[i + 1], control_t, (Motion_Position_out.OutPut.Ankle_R_z + (int)(Motion_Position_in.Mark_Time[i] / control_t)));
		TSpline_PVA(Motion_Position_in.Mark.Body_Roll[i], 0.0, 0.0, Motion_Position_in.Mark_Time[i], 0.5 * (Motion_Position_in.Mark.Body_Roll[i] + Motion_Position_in.Mark.Body_Roll[i + 1]), 0.5 * (Motion_Position_in.Mark_Time[i] + Motion_Position_in.Mark_Time[i + 1]), Motion_Position_in.Mark.Body_Roll[i + 1], 0.0, 0.0, Motion_Position_in.Mark_Time[i + 1], control_t, (Motion_Position_out.OutPut.Body_Roll + (int)(Motion_Position_in.Mark_Time[i] / control_t)));
		TSpline_PVA(Motion_Position_in.Mark.Body_Pitch[i], 0.0, 0.0, Motion_Position_in.Mark_Time[i], 0.5 * (Motion_Position_in.Mark.Body_Pitch[i] + Motion_Position_in.Mark.Body_Pitch[i + 1]), 0.5 * (Motion_Position_in.Mark_Time[i] + Motion_Position_in.Mark_Time[i + 1]), Motion_Position_in.Mark.Body_Pitch[i + 1], 0.0, 0.0, Motion_Position_in.Mark_Time[i + 1], control_t, (Motion_Position_out.OutPut.Body_Pitch + (int)(Motion_Position_in.Mark_Time[i] / control_t)));
		TSpline_PVA(Motion_Position_in.Mark.ZMP_x[i], 0.0, 0.0, Motion_Position_in.Mark_Time[i], 0.5 * (Motion_Position_in.Mark.ZMP_x[i] + Motion_Position_in.Mark.ZMP_x[i + 1]), 0.5 * (Motion_Position_in.Mark_Time[i] + Motion_Position_in.Mark_Time[i + 1]), Motion_Position_in.Mark.ZMP_x[i + 1], 0.0, 0.0, Motion_Position_in.Mark_Time[i + 1], control_t, (Motion_Position_out.OutPut.ZMP_x + (int)(Motion_Position_in.Mark_Time[i] / control_t)));
		TSpline_PVA(Motion_Position_in.Mark.ZMP_y[i], 0.0, 0.0, Motion_Position_in.Mark_Time[i], 0.5 * (Motion_Position_in.Mark.ZMP_y[i] + Motion_Position_in.Mark.ZMP_y[i + 1]), 0.5 * (Motion_Position_in.Mark_Time[i] + Motion_Position_in.Mark_Time[i + 1]), Motion_Position_in.Mark.ZMP_y[i + 1], 0.0, 0.0, Motion_Position_in.Mark_Time[i + 1], control_t, (Motion_Position_out.OutPut.ZMP_y + (int)(Motion_Position_in.Mark_Time[i] / control_t)));
	}
	printf("%lf\n", walk_t);
	printf("%d, %d\n", n_stop, n_tcom);
	for (int j = n_stop; j < n_tcom; j++)
	{
		Motion_Position_out.OutPut.Com_x[j] = Motion_Position_out.OutPut.Com_x[n_stop - 1];
		Motion_Position_out.OutPut.Com_y[j] = Motion_Position_out.OutPut.Com_y[n_stop - 1];
		Motion_Position_out.OutPut.Com_z[j] = Motion_Position_out.OutPut.Com_z[n_stop - 1];
		Motion_Position_out.OutPut.Ankle_R_x[j] = Motion_Position_out.OutPut.Ankle_R_x[n_stop - 1];
		Motion_Position_out.OutPut.Ankle_R_y[j] = Motion_Position_out.OutPut.Ankle_R_y[n_stop - 1];
		Motion_Position_out.OutPut.Ankle_R_z[j] = Motion_Position_out.OutPut.Ankle_R_z[n_stop - 1];
		Motion_Position_out.OutPut.Ankle_L_x[j] = Motion_Position_out.OutPut.Ankle_L_x[n_stop - 1];
		Motion_Position_out.OutPut.Ankle_L_y[j] = Motion_Position_out.OutPut.Ankle_L_y[n_stop - 1];
		Motion_Position_out.OutPut.Ankle_L_z[j] = Motion_Position_out.OutPut.Ankle_L_z[n_stop - 1];
		Motion_Position_out.OutPut.Body_Roll[j] = Motion_Position_out.OutPut.Body_Roll[n_stop - 1];
		Motion_Position_out.OutPut.Body_Pitch[j] = Motion_Position_out.OutPut.Body_Pitch[n_stop - 1];
		Motion_Position_out.OutPut.ZMP_x[j] = Motion_Position_out.OutPut.ZMP_x[n_stop - 1];
		Motion_Position_out.OutPut.ZMP_y[j] = Motion_Position_out.OutPut.ZMP_y[n_stop - 1];
	}
}

void Free_Motion_Protection(double Ankle_width, double Ankle_height, double H_zc, double control_t, double walk_t, int k_pre)
{
	double com_x0 = Tra_COM.x[k_pre - 1];
	double com_y0 = Tra_COM.y[k_pre - 1];
	double com_z0 = Tra_COM.z[k_pre - 1];
	double Rankle_x0 = Tra_RAnkle.x[k_pre - 1];
	double Rankle_y0 = Tra_RAnkle.y[k_pre - 1];
	double Rankle_z0 = Tra_RAnkle.z[k_pre - 1];
	double Lankle_x0 = Tra_LAnkle.x[k_pre - 1];
	double Lankle_y0 = Tra_LAnkle.y[k_pre - 1];
	double Lankle_z0 = Tra_LAnkle.z[k_pre - 1];
	double body_roll0 = roll_body;
	double body_pitch0 = pitch_body;

	double com_x[] = { com_x0, 0.0 };
	double com_y[] = { com_y0, com_y0 };
	double com_z[] = { com_z0, 0.55 };

	double ankle_L_x[] = { Lankle_x0, -0.5 * Ankle_width };
	double ankle_L_y[] = { Lankle_y0, com_y0 };
	double ankle_L_z[] = { Lankle_z0, Ankle_height };

	double ankle_R_x[] = { Rankle_x0, 0.5 * Ankle_width };
	double ankle_R_y[] = { Rankle_y0, com_y0 };
	double ankle_R_z[] = { Rankle_z0, Ankle_height };

	double body_roll[] = { body_roll0, 0.0 };
	double body_pitch[] = { body_pitch0, -15 / 57.3 };

	double mark_time[] = { 0.0, 2.0 };// { 0.0, 2.0, 4.0, 82.0, 84.0, 86.0 };

	int mark_number = 2;

	int n_stop = (int)(mark_time[mark_number - 1] / control_t);
	int n_tcom = (int)(walk_t / control_t);

	for (int i = 0; i < mark_number; i++)
	{
		Motion_Protection.Mark.Com_x[i] = com_x[i];
		Motion_Protection.Mark.Com_y[i] = com_y[i];
		Motion_Protection.Mark.Com_z[i] = com_z[i];

		Motion_Protection.Mark.Ankle_L_x[i] = ankle_L_x[i];
		Motion_Protection.Mark.Ankle_L_y[i] = ankle_L_y[i];
		Motion_Protection.Mark.Ankle_L_z[i] = ankle_L_z[i];

		Motion_Protection.Mark.Ankle_R_x[i] = ankle_R_x[i];
		Motion_Protection.Mark.Ankle_R_y[i] = ankle_R_y[i];
		Motion_Protection.Mark.Ankle_R_z[i] = ankle_R_z[i];

		Motion_Protection.Mark.Body_Roll[i] = body_roll[i];
		Motion_Protection.Mark.Body_Pitch[i] = body_pitch[i];

		Motion_Protection.Mark_Time[i] = mark_time[i];

		Motion_Protection.Mark_Number = mark_number;
	}
	Motion_Position_in = Motion_Protection;

	Free_Ankle_Com(control_t, n_stop, n_tcom, walk_t);
	Motion_Protection = Motion_Position_out;
}

void Free_Motion_Assig(double Ankle_width, double Ankle_height, double H_zc, double control_t, double walk_t)
{
	#ifdef SINGLE_SUP_STILL
	double com_x[] = { 0.0, -0.097, -0.097, -0.097, -0.097, 0.0 };
	double com_y[] = { 0.0, 0.0, -0.015, -0.015, 0.0, 0.0 };
	double com_z[] = { 0.7368294446, 0.7, 0.7, 0.7, 0.7, 0.7 };

	double ankle_L_x[] = { -0.5 * Ankle_width, -0.5 * Ankle_width, -0.5 * Ankle_width, -0.5 * Ankle_width, -0.5 * Ankle_width, -0.5 * Ankle_width };
	double ankle_L_y[] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
	double ankle_L_z[] = { Ankle_height, Ankle_height, Ankle_height, Ankle_height, Ankle_height, Ankle_height };

	double ankle_R_x[] = { 0.5 * Ankle_width, 0.5 * Ankle_width, 0.5 * Ankle_width, 0.5 * Ankle_width, 0.5 * Ankle_width, 0.5 * Ankle_width };
	double ankle_R_y[] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	double ankle_R_z[] = { Ankle_height, Ankle_height, Ankle_height + 0.06, Ankle_height + 0.06, Ankle_height, Ankle_height };

	double body_roll[] = { 0.0, -2.0 / 57.3, -2.0 / 57.3, -2.0 / 57.3, -2.0 / 57.3, 0.0 };
	double body_pitch[] = { 0.0, 0.0, 0.0 / 57.3, 0.0 / 57.3, 0.0, 0.0};
	
	double ZMP_x[] = { 0.0, -0.097, -0.097, -0.097, -0.097, 0.0};
	double ZMP_y[] = { -0.0, 0.0, 0.0, 0.0, 0.0, -0.0};

	double mark_time[] = { 0.0, 1.5, 3.0, 17.0, 18.5, 20.0 };// { 0.0, 2.0, 4.0, 82.0, 84.0, 86.0 };

	int mark_number = 6;
	#endif
	
	#ifdef FLY_STEPZ_CHECK
	double com_x[] = { 0.0, 0.0, 0.0, 0.0 };
	double com_y[] = { 0.0, 0.0, 0.0, 0.0 };
	double com_z[] = { 0.7368294446, 0.67, 0.67, 0.67 };

	double ankle_L_x[] = { -0.5 * Ankle_width, -0.5 * Ankle_width, -0.5 * Ankle_width, -0.5 * Ankle_width };
	double ankle_L_y[] = { 0.0, 0.0, 0.2, 0.2 };
	double ankle_L_z[] = { Ankle_height, Ankle_height, Ankle_height, Ankle_height };

	double ankle_R_x[] = { 0.5 * Ankle_width, 0.5 * Ankle_width, 0.5 * Ankle_width, 0.5 * Ankle_width };
	double ankle_R_y[] = { 0.0, 0.0, -0.2, -0.2 };
	double ankle_R_z[] = { Ankle_height, Ankle_height, Ankle_height, Ankle_height };

	double body_roll[] = { 0.0, 0.0, 0.0, 0.0 };
	double body_pitch[] = { 0.0, 0.0, 0.0, 0.0 };
	
	double ZMP_x[] = { 0.0, 0.0, 0.0, 0.0 };
	double ZMP_y[] = { 0.0, 0.0, 0.0, 0.0 };

	double mark_time[] = { 0.0, 1.5, 3.0, 50.0 };// { 0.0, 2.0, 4.0, 82.0, 84.0, 86.0 };

	int mark_number = 4;
	#endif
		
		
	int n_stop = (int)(mark_time[mark_number - 1] / control_t);
	int n_tcom = (int)(walk_t / control_t);

	for (int i = 0; i < mark_number; i++)
	{
		Motion_Position_in.Mark.Com_x[i] = com_x[i];
		Motion_Position_in.Mark.Com_y[i] = com_y[i];
		Motion_Position_in.Mark.Com_z[i] = com_z[i];

		Motion_Position_in.Mark.Ankle_L_x[i] = ankle_L_x[i];
		Motion_Position_in.Mark.Ankle_L_y[i] = ankle_L_y[i];
		Motion_Position_in.Mark.Ankle_L_z[i] = ankle_L_z[i];

		Motion_Position_in.Mark.Ankle_R_x[i] = ankle_R_x[i];
		Motion_Position_in.Mark.Ankle_R_y[i] = ankle_R_y[i];
		Motion_Position_in.Mark.Ankle_R_z[i] = ankle_R_z[i];

		Motion_Position_in.Mark.Body_Roll[i] = body_roll[i];
		Motion_Position_in.Mark.Body_Pitch[i] = body_pitch[i];
		
		Motion_Position_in.Mark.ZMP_x[i] = ZMP_x[i];
		Motion_Position_in.Mark.ZMP_y[i] = ZMP_y[i];

		Motion_Position_in.Mark_Time[i] = mark_time[i];

		Motion_Position_in.Mark_Number = mark_number;
	}

	Free_Ankle_Com(control_t, n_stop, n_tcom, walk_t);

}
