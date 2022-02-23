#include "dcc_con_base.h"
#include "math.h"

/** Limitation
* InputVal: Data, Limit[min, max]
* OutputVal: DataLimited
*/
double fndLimit(double dInputData, double dLimit[2]) {
	double dOutputData;
	if (dLimit[0] == dLimit[1]) {
		return dInputData;
	}
	else {
		dOutputData = fmax(dLimit[0], dInputData);
		dOutputData = fmin(dLimit[1], dOutputData);
		return dOutputData;
	}
}
// rec dcc
/** Limitation when added to another val
* InputVal: Data, DataAddedTo, Limit[min, max]
* OutputVal: DataLimited, &dSurPassedVal
*/
double fndAddLimit(double dInputData, double dAddedData, double * dSurPassedVal, double dLimit[2]) {
	double dOutputData;
	if (dInputData + dAddedData >= dLimit[1]) {
		dOutputData = dLimit[1] - dAddedData;
	}
	else if (dInputData + dAddedData <= dLimit[0]) {
		dOutputData = dLimit[0] - dAddedData;
	}
	else dOutputData = dInputData;
	if(dSurPassedVal != NULL) *dSurPassedVal = dInputData - dOutputData;
	return dOutputData;
}
double fndThreshold(double dInVal, double dThreshold[2]) {
	double dOutVal;
	if (dInVal >= dThreshold[1]) dOutVal = dInVal - dThreshold[1];
	else if (dInVal <= dThreshold[0]) dOutVal = dInVal - dThreshold[0];
	else dOutVal = 0.0;
	return dOutVal;
}
void fnvInitInteg(IntegValLimit *strInputData) {
	strInputData->p = 0.0;
	strInputData->dp = 0.0;
	strInputData->ddp = 0.0;
}
/** Interation acc with limitation in default struct
* InputVal: DataOld[p, dp, ddp], Acc, Limits[PosMin, PosMax, VelMin, VelMax, AccMin, AccMax], CONTROL_T
* OutputVal: DataUpdated[p, dp, ddp]
*/
IntegValLimit fnstrIntegLimit(IntegValLimit strOldData, double dAcc, double dLimits[6],  double dControlT) {
	IntegValLimit strUpdatedData;
	double dPositionLimit[2];
	double dVelocityLimit[2];
	double dAccelerateLimit[2];
	for (int i = 0; i < 2; i++) {
		dPositionLimit[i] = dLimits[i];
		dVelocityLimit[i] = dLimits[i + 2];
		dAccelerateLimit[i] = dLimits[i + 4];
	}
	dAcc = fndLimit(dAcc, dAccelerateLimit);
	strUpdatedData.dp = strOldData.dp + dAcc * dControlT;
	strUpdatedData.dp = fndLimit(strUpdatedData.dp, dVelocityLimit);
	strUpdatedData.p = strOldData.p + strUpdatedData.dp * dControlT;
	strUpdatedData.p = fndLimit(strUpdatedData.p, dPositionLimit);
	strUpdatedData.dp = (strUpdatedData.p - strOldData.p) / dControlT;
	strUpdatedData.ddp = dAcc;
	return strUpdatedData;
}
/** Interation jerk with limitation
* InputVal: &PosOld, &VeloOld, &AccOld, Jerk, Limits[PosMin, PosMax, VelMin, VelMax, AccMin, AccMax], CONTROL_T
* OutputVal: &PosUpdated, &VeloUpdated, &AccUpdated
*/
void fnvJerkLimit(double *dPosIn, double *dVelIn, double *dAccIn, double dJerk, double dLimits[6], double dControlT) {
	double dPositionLimit[2];
	double dVelocityLimit[2];
	double dAccelerateLimit[2];
	double dAccOld = *dAccIn;
	double dVelOld = *dVelIn;
	double dPosOld = *dPosIn;
	double dJerkIn = dJerk;
	for (int i = 0; i < 2; i++) {
		dPositionLimit[i] = dLimits[i];
		dVelocityLimit[i] = dLimits[i + 2];
		dAccelerateLimit[i] = dLimits[i + 4];
	}
	*dAccIn = dAccOld + dJerkIn * dControlT;
	*dAccIn = fndLimit(*dAccIn, dAccelerateLimit);
	*dVelIn = dVelOld + *dAccIn * dControlT;
	*dVelIn = fndLimit(*dVelIn, dVelocityLimit);
	*dAccIn = (*dVelIn - dVelOld) / dControlT;
	*dPosIn = dPosOld + *dVelIn * dControlT;
	*dPosIn = fndLimit(*dPosIn, dPositionLimit);
	*dVelIn = (*dPosIn - dPosOld) / dControlT;
}
/** Interation acc with limitation 
* InputVal: &PosOld, &VeloOld, Acc, Limits[PosMin, PosMax, VelMin, VelMax, AccMin, AccMax], CONTROL_T
* OutputVal: &PosUpdated, &VeloUpdated
*/
void fnvIntegLimit(double *dPosIn, double *dVelIn, double dAcc, double dLimits[6], double dControlT) {
	double dPositionLimit[2];
	double dVelocityLimit[2];
	double dAccelerateLimit[2];
	double dVelOld = *dVelIn;
	double dPosOld = *dPosIn;
	double dAccIn = dAcc;
	for (int i = 0; i < 2; i++) {
		dPositionLimit[i] = dLimits[i];
		dVelocityLimit[i] = dLimits[i + 2];
		dAccelerateLimit[i] = dLimits[i + 4];
	}
	dAccIn = fndLimit(dAcc, dAccelerateLimit);
	*dVelIn = dVelOld + dAccIn * dControlT;
	*dVelIn = fndLimit(*dVelIn, dVelocityLimit);
	*dPosIn = dPosOld + *dVelIn * dControlT;
	*dPosIn = fndLimit(*dPosIn, dPositionLimit);
	*dVelIn = (*dPosIn - dPosOld) / dControlT;
}
/** Interation velo with limitation
* InputVal: &PosOld, Velo, Limits[PosMin, PosMax, VelMin, VelMax], CONTROL_T
* OutputVal: &PosUpdated
*/
void fnvVeloLimit(double *dPosIn, double dVel, double dLimits[4], double dControlT) {
	double dPositionLimit[2];
	double dVelocityLimit[2];
	double dPosOld = *dPosIn;
	double dVelIn = dVel;
	for (int i = 0; i < 2; i++) {
		dPositionLimit[i] = dLimits[i];
		dVelocityLimit[i] = dLimits[i + 2];
	}
	dVelIn = fndLimit(dVel, dVelocityLimit);
	*dPosIn = dPosOld + dVelIn * dControlT;
	*dPosIn = fndLimit(*dPosIn, dPositionLimit);
}
/** TimeLag filter
* InputVal: DataOld, DataIn, CONTROL_T, LagT
* OutputVal: &DataUpdated
*/
double fndFilterTimeLag(double filtered_in, double data_in, double control_t, double Lag_T)
{
	double filtered_out;
	filtered_out = (control_t * data_in + Lag_T * filtered_in) / (control_t + Lag_T);
	return filtered_out;
}
/** 5th Spline
* InputVal: x0, v0, a0, t0, x1, v1, a1, t1, CONTROL_T, ModeFlag
* OutputVal: &DataOut
*/
void fnvFifthSpline(double *pos_out, int maxProgN, double x0, double v0, double a0, double t0, double x1, double v1, double a1, double t1, double control_t, char mode_flag)
{
	double b_spline[6] = { 0 };
	b_spline[0] = (1.0 / pow(t0 - t1, 5.0)*((t0*t0*t0*t0*t0)*x1*2.0 - (t1*t1*t1*t1*t1)*x0*2.0 + t0*(t1*t1*t1*t1*t1)*v0*2.0 - (t0*t0*t0*t0*t0)*t1*v1*2.0 + t0*(t1*t1*t1*t1)*x0*1.0E+1 - (t0*t0*t0*t0)*t1*x1*1.0E+1 - a0*(t0*t0)*(t1*t1*t1*t1*t1) + a0*(t0*t0*t0)*(t1*t1*t1*t1)*2.0 - a0*(t0*t0*t0*t0)*(t1*t1*t1) + a1*(t0*t0*t0)*(t1*t1*t1*t1) - a1*(t0*t0*t0*t0)*(t1*t1*t1)*2.0 + a1*(t0*t0*t0*t0*t0)*(t1*t1) - (t0*t0)*(t1*t1*t1*t1)*v0*1.0E+1 + (t0*t0*t0)*(t1*t1*t1)*v0*8.0 - (t0*t0*t0)*(t1*t1*t1)*v1*8.0 + (t0*t0*t0*t0)*(t1*t1)*v1*1.0E+1 - (t0*t0)*(t1*t1*t1)*x0*2.0E+1 + (t0*t0*t0)*(t1*t1)*x1*2.0E+1)) / 2.0;
	b_spline[1] = (1.0 / pow(t0 - t1, 5.0)*((t0*t0*t0*t0*t0)*v1*2.0 - (t1*t1*t1*t1*t1)*v0*2.0 + a0*t0*(t1*t1*t1*t1*t1)*2.0 - a1*(t0*t0*t0*t0*t0)*t1*2.0 + t0*(t1*t1*t1*t1)*v0*1.0E+1 - (t0*t0*t0*t0)*t1*v1*1.0E+1 - a0*(t0*t0)*(t1*t1*t1*t1) - a0*(t0*t0*t0)*(t1*t1*t1)*4.0 + a0*(t0*t0*t0*t0)*(t1*t1)*3.0 - a1*(t0*t0)*(t1*t1*t1*t1)*3.0 + a1*(t0*t0*t0)*(t1*t1*t1)*4.0 + a1*(t0*t0*t0*t0)*(t1*t1) + (t0*t0)*(t1*t1*t1)*v0*1.6E+1 - (t0*t0*t0)*(t1*t1)*v0*2.4E+1 + (t0*t0)*(t1*t1*t1)*v1*2.4E+1 - (t0*t0*t0)*(t1*t1)*v1*1.6E+1 + (t0*t0)*(t1*t1)*x0*6.0E+1 - (t0*t0)*(t1*t1)*x1*6.0E+1)) / 2.0;
	b_spline[2] = 1.0 / pow(t0 - t1, 5.0)*(a0*(t1*t1*t1*t1*t1) - a1*(t0*t0*t0*t0*t0) + a0*t0*(t1*t1*t1*t1)*4.0 + a0*(t0*t0*t0*t0)*t1*3.0 - a1*t0*(t1*t1*t1*t1)*3.0 - a1*(t0*t0*t0*t0)*t1*4.0 + t0*(t1*t1*t1)*v0*3.6E+1 - (t0*t0*t0)*t1*v0*2.4E+1 + t0*(t1*t1*t1)*v1*2.4E+1 - (t0*t0*t0)*t1*v1*3.6E+1 + t0*(t1*t1)*x0*6.0E+1 + (t0*t0)*t1*x0*6.0E+1 - t0*(t1*t1)*x1*6.0E+1 - (t0*t0)*t1*x1*6.0E+1 - a0*(t0*t0)*(t1*t1*t1)*8.0 + a1*(t0*t0*t0)*(t1*t1)*8.0 - (t0*t0)*(t1*t1)*v0*1.2E+1 + (t0*t0)*(t1*t1)*v1*1.2E+1)*(-1.0 / 2.0);
	b_spline[3] = (1.0 / pow(t0 - t1, 5.0)*(a0*(t0*t0*t0*t0) + a0*(t1*t1*t1*t1)*3.0 - a1*(t0*t0*t0*t0)*3.0 - a1*(t1*t1*t1*t1) - (t0*t0*t0)*v0*8.0 - (t0*t0*t0)*v1*1.2E+1 + (t1*t1*t1)*v0*1.2E+1 + (t1*t1*t1)*v1*8.0 + (t0*t0)*x0*2.0E+1 - (t0*t0)*x1*2.0E+1 + (t1*t1)*x0*2.0E+1 - (t1*t1)*x1*2.0E+1 + a0*(t0*t0*t0)*t1*4.0 - a1*t0*(t1*t1*t1)*4.0 + t0*(t1*t1)*v0*2.8E+1 - (t0*t0)*t1*v0*3.2E+1 + t0*(t1*t1)*v1*3.2E+1 - (t0*t0)*t1*v1*2.8E+1 - a0*(t0*t0)*(t1*t1)*8.0 + a1*(t0*t0)*(t1*t1)*8.0 + t0*t1*x0*8.0E+1 - t0*t1*x1*8.0E+1)) / 2.0;
	b_spline[4] = 1.0 / pow(t0 - t1, 5.0)*(t0*x0*3.0E+1 - t0*x1*3.0E+1 + t1*x0*3.0E+1 - t1*x1*3.0E+1 + a0*(t0*t0*t0)*2.0 + a0*(t1*t1*t1)*3.0 - a1*(t0*t0*t0)*3.0 - a1*(t1*t1*t1)*2.0 - (t0*t0)*v0*1.4E+1 - (t0*t0)*v1*1.6E+1 + (t1*t1)*v0*1.6E+1 + (t1*t1)*v1*1.4E+1 - a0*t0*(t1*t1)*4.0 - a0*(t0*t0)*t1 + a1*t0*(t1*t1) + a1*(t0*t0)*t1*4.0 - t0*t1*v0*2.0 + t0*t1*v1*2.0)*(-1.0 / 2.0);
	b_spline[5] = (1.0 / pow(t0 - t1, 5.0)*(x0*1.2E+1 - x1*1.2E+1 - t0*v0*6.0 - t0*v1*6.0 + t1*v0*6.0 + t1*v1*6.0 + a0*(t0*t0) + a0*(t1*t1) - a1*(t0*t0) - a1*(t1*t1) - a0*t0*t1*2.0 + a1*t0*t1*2.0)) / 2.0;
	int k0 = (int)floor(t0 / control_t + 1e-8);
	int k1 = (int)floor(t1 / control_t + 1e-8);

	if (mode_flag == 'T') { // traditional: start from k = 0
		int k = 0;
		for (int i = k0; i < k1; i++) {
			pos_out[k] = b_spline[0] + b_spline[1] * i * control_t + b_spline[2] * i * control_t * i * control_t + b_spline[3] * i * control_t * i * control_t * i * control_t + b_spline[4] * i * control_t * i * control_t * i * control_t * i * control_t + b_spline[5] * i * control_t * i * control_t * i * control_t * i * control_t * i * control_t;
			k++;
		}
		for (int i = k1; i < maxProgN + k0; i++) {
			pos_out[k] = pos_out[k1 - k0 - 1];
			k++;
		}
	}
	else if (mode_flag == 'N') { // novel: start from k = k0
		for (int i = k0; i < k1; i++) {
			pos_out[i] = b_spline[0] + b_spline[1] * i * control_t + b_spline[2] * i * control_t * i * control_t + b_spline[3] * i * control_t * i * control_t * i * control_t + b_spline[4] * i * control_t * i * control_t * i * control_t * i * control_t + b_spline[5] * i * control_t * i * control_t * i * control_t * i * control_t * i * control_t;
		}
		for (int i = k1; i < maxProgN; i++) {
			pos_out[i] = pos_out[k1 - 1];
		}
	}
	if (x0 == x1 && v0 == v1 && a0 == a1) {
		for (int i = k0; i < maxProgN; i++) {
			pos_out[i] = x0;
		}
	}
}
/** dou4th Spline
* InputVal: x0, v0, a0, t0, x1, t1, x2, v2, a2, t2, CONTROL_T, ModeFlag
* OutputVal: &DataOut
*/
void fnvDouFourthSpline(double *pos_out, int maxProgN, double x0, double v0, double a0, double t0, double x1, double t1, double x2, double v2, double a2, double t2, double control_t, char mode_flag)
{
	double b_spline[10] = { 0 };
	double v1 = 0.0;
	b_spline[0] = ((t0*t0*t0*t0)*x1*2.0 + (t1*t1*t1*t1)*x0*2.0 - t0*(t1*t1*t1*t1)*v0*2.0 - (t0*t0*t0*t0)*t1*v1*2.0 - t0*(t1*t1*t1)*x0*8.0 - (t0*t0*t0)*t1*x1*8.0 + a0*(t0*t0)*(t1*t1*t1*t1) - a0*(t0*t0*t0)*(t1*t1*t1)*2.0 + a0*(t0*t0*t0*t0)*(t1*t1) + (t0*t0)*(t1*t1*t1)*v0*8.0 - (t0*t0*t0)*(t1*t1)*v0*6.0 + (t0*t0*t0)*(t1*t1)*v1*2.0 + (t0*t0)*(t1*t1)*x0*1.2E+1) / ((t0 - t1)*(t0*(t1*t1)*3.0 - (t0*t0)*t1*3.0 + t0*t0*t0 - t1*t1*t1)*2.0);
	b_spline[1] = ((t0*t0*t0*t0)*v1 + (t1*t1*t1*t1)*v0 - a0*t0*(t1*t1*t1*t1) - a0*(t0*t0*t0*t0)*t1 - t0*(t1*t1*t1)*v0*4.0 + (t0*t0*t0)*t1*v0*6.0 + (t0*t0*t0)*t1*v1*2.0 - (t0*t0)*t1*x0*1.2E+1 + (t0*t0)*t1*x1*1.2E+1 + a0*(t0*t0)*(t1*t1*t1) + a0*(t0*t0*t0)*(t1*t1) - (t0*t0)*(t1*t1)*v0*3.0 - (t0*t0)*(t1*t1)*v1*3.0) / ((t0 - t1)*(t0*(t1*t1)*3.0 - (t0*t0)*t1*3.0 + t0*t0*t0 - t1*t1*t1));
	b_spline[2] = (a0*(t0*t0*t0*t0) + a0*(t1*t1*t1*t1) - (t0*t0*t0)*v0*6.0 - (t0*t0*t0)*v1*6.0 + (t0*t0)*x0*1.2E+1 - (t0*t0)*x1*1.2E+1 + a0*t0*(t1*t1*t1)*2.0 + a0*(t0*t0*t0)*t1*2.0 + t0*(t1*t1)*v0*1.8E+1 - (t0*t0)*t1*v0*1.2E+1 + t0*(t1*t1)*v1*6.0 - a0*(t0*t0)*(t1*t1)*6.0 + t0*t1*x0*2.4E+1 - t0*t1*x1*2.4E+1) / ((t0 - t1)*(t0*(t1*t1)*3.0 - (t0*t0)*t1*3.0 + t0*t0*t0 - t1*t1*t1)*2.0);
	b_spline[3] = -(t0*x0*8.0 - t0*x1*8.0 + t1*x0*4.0 - t1*x1*4.0 + a0*(t0*t0*t0) + a0*(t1*t1*t1) - (t0*t0)*v0*5.0 - (t0*t0)*v1*3.0 + (t1*t1)*v0*3.0 + (t1*t1)*v1 - a0*t0*(t1*t1) - a0*(t0*t0)*t1 + t0*t1*v0*2.0 + t0*t1*v1*2.0) / ((t0 - t1)*(t0*(t1*t1)*3.0 - (t0*t0)*t1*3.0 + t0*t0*t0 - t1*t1*t1));
	b_spline[4] = (x0*6.0 - x1*6.0 - t0*v0*4.0 - t0*v1*2.0 + t1*v0*4.0 + t1*v1*2.0 + a0*(t0*t0) + a0*(t1*t1) - a0*t0*t1*2.0) / ((t0 - t1)*(t0*(t1*t1)*3.0 - (t0*t0)*t1*3.0 + t0*t0*t0 - t1*t1*t1)*2.0); 
	b_spline[5] = ((t1*t1*t1*t1)*x2*2.0 + (t2*t2*t2*t2)*x1*2.0 - t1*(t2*t2*t2*t2)*v1*2.0 - (t1*t1*t1*t1)*t2*v2*2.0 - t1*(t2*t2*t2)*x1*8.0 - (t1*t1*t1)*t2*x2*8.0 + a2*(t1*t1)*(t2*t2*t2*t2) - a2*(t1*t1*t1)*(t2*t2*t2)*2.0 + a2*(t1*t1*t1*t1)*(t2*t2) + (t1*t1)*(t2*t2*t2)*v1*2.0 - (t1*t1)*(t2*t2*t2)*v2*6.0 + (t1*t1*t1)*(t2*t2)*v2*8.0 + (t1*t1)*(t2*t2)*x2*1.2E+1) / ((t1 - t2)*(t1*(t2*t2)*3.0 - (t1*t1)*t2*3.0 + t1*t1*t1 - t2*t2*t2)*2.0);
	b_spline[6] = ((t1*t1*t1*t1)*v2 + (t2*t2*t2*t2)*v1 - a2*t1*(t2*t2*t2*t2) - a2*(t1*t1*t1*t1)*t2 + t1*(t2*t2*t2)*v1*2.0 + t1*(t2*t2*t2)*v2*6.0 - (t1*t1*t1)*t2*v2*4.0 + t1*(t2*t2)*x1*1.2E+1 - t1*(t2*t2)*x2*1.2E+1 + a2*(t1*t1)*(t2*t2*t2) + a2*(t1*t1*t1)*(t2*t2) - (t1*t1)*(t2*t2)*v1*3.0 - (t1*t1)*(t2*t2)*v2*3.0) / ((t1 - t2)*(t1*(t2*t2)*3.0 - (t1*t1)*t2*3.0 + t1*t1*t1 - t2*t2*t2));
	b_spline[7] = (a2*(t1*t1*t1*t1) + a2*(t2*t2*t2*t2) - (t2*t2*t2)*v1*6.0 - (t2*t2*t2)*v2*6.0 - (t2*t2)*x1*1.2E+1 + (t2*t2)*x2*1.2E+1 + a2*t1*(t2*t2*t2)*2.0 + a2*(t1*t1*t1)*t2*2.0 + (t1*t1)*t2*v1*6.0 - t1*(t2*t2)*v2*1.2E+1 + (t1*t1)*t2*v2*1.8E+1 - a2*(t1*t1)*(t2*t2)*6.0 - t1*t2*x1*2.4E+1 + t1*t2*x2*2.4E+1) / ((t1 - t2)*(t1*(t2*t2)*3.0 - (t1*t1)*t2*3.0 + t1*t1*t1 - t2*t2*t2)*2.0);
	b_spline[8] = -(t1*x1*-4.0 + t1*x2*4.0 - t2*x1*8.0 + t2*x2*8.0 + a2*(t1*t1*t1) + a2*(t2*t2*t2) + (t1*t1)*v1 + (t1*t1)*v2*3.0 - (t2*t2)*v1*3.0 - (t2*t2)*v2*5.0 - a2*t1*(t2*t2) - a2*(t1*t1)*t2 + t1*t2*v1*2.0 + t1*t2*v2*2.0) / ((t1 - t2)*(t1*(t2*t2)*3.0 - (t1*t1)*t2*3.0 + t1*t1*t1 - t2*t2*t2));
	b_spline[9] = (x1*-6.0 + x2*6.0 + t1*v1*2.0 + t1*v2*4.0 - t2*v1*2.0 - t2*v2*4.0 + a2*(t1*t1) + a2*(t2*t2) - a2*t1*t2*2.0) / ((t1 - t2)*(t1*(t2*t2)*3.0 - (t1*t1)*t2*3.0 + t1*t1*t1 - t2*t2*t2)*2.0); 
	int k0 = (int)floor(t0 / control_t + 1e-8);
	int k1 = (int)floor(t1 / control_t + 1e-8);
	int k2 = (int)floor(t2 / control_t + 1e-8);

	if (mode_flag == 'T') { // traditional: start from k = 0
		int k = 0;
		for (int i = k0; i < k1; i++) {
			pos_out[k] = b_spline[0] + b_spline[1] * i * control_t + b_spline[2] * i * control_t * i * control_t + b_spline[3] * i * control_t * i * control_t * i * control_t + b_spline[4] * i * control_t * i * control_t * i * control_t * i * control_t;
			k++;
		}
		for (int i = k1; i < k2; i++) {
			pos_out[k] = b_spline[5] + b_spline[6] * i * control_t + b_spline[7] * i * control_t * i * control_t + b_spline[8] * i * control_t * i * control_t * i * control_t + b_spline[9] * i * control_t * i * control_t * i * control_t * i * control_t;
			k++;
		}
		for (int i = k2; i < maxProgN + k0; i++) {
			pos_out[k] = pos_out[k2 - k0 - 1];
			k++;
		}
	}
	else if (mode_flag == 'N') { // novel: start from k = k0
		for (int i = k0; i < k1; i++) {
			pos_out[i] = b_spline[0] + b_spline[1] * i * control_t + b_spline[2] * i * control_t * i * control_t + b_spline[3] * i * control_t * i * control_t * i * control_t + b_spline[4] * i * control_t * i * control_t * i * control_t * i * control_t;
		}
		for (int i = k1; i < k2; i++) {
			pos_out[i] = b_spline[5] + b_spline[6] * i * control_t + b_spline[7] * i * control_t * i * control_t + b_spline[8] * i * control_t * i * control_t * i * control_t + b_spline[9] * i * control_t * i * control_t * i * control_t * i * control_t;
		}
		for (int i = k2; i < maxProgN; i++) {
			pos_out[i] = pos_out[k2 - 1];
		}
	}
	if (x0 == x1 && x1 == x2) {
		for (int i = k0; i < maxProgN; i++) {
			pos_out[i] = x0;
		}
	}
}
/** Easy Spline: from the last position of the data to the final position with no started & ended velo & acc
* InputVal: maxProgN(maxmun steps of the whole program), t0, t1, x1, CONTROL_T
* OutputVal: &DataOut
*/
void fnvEzSpline(double *pos_out, int maxProgN, double t0_in, double t1_in, double x1_in, double control_t)
{
	// int according to control period
	int k0 = (int)floor(t0_in / control_t + 1e-8);
	int k1 = (int)floor(t1_in / control_t + 1e-8);

	// map to 0s (error would be remarkable if time is too large)
	double t0 = 0.0;
	double t1 = (k1 - k0) * control_t;

	double b_spline[6] = { 0 };
	double x0 = pos_out[k0];
	double v0 = 0.0;
	double a0 = 0.0;
	double x1 = x1_in;
	double v1 = 0.0;
	double a1 = 0.0;
	double tempt = 0.0;
	int tempk = 0;
	if (k1 == k0) { // jump
		for (int i = k0; i < maxProgN; i++) {
			pos_out[i] = x1;
		}
	}
	else { // spline to x1
		b_spline[0] = (1.0 / pow(t0 - t1, 5.0)*((t0*t0*t0*t0*t0)*x1*2.0 - (t1*t1*t1*t1*t1)*x0*2.0 + t0*(t1*t1*t1*t1*t1)*v0*2.0 - (t0*t0*t0*t0*t0)*t1*v1*2.0 + t0*(t1*t1*t1*t1)*x0*1.0E+1 - (t0*t0*t0*t0)*t1*x1*1.0E+1 - a0*(t0*t0)*(t1*t1*t1*t1*t1) + a0*(t0*t0*t0)*(t1*t1*t1*t1)*2.0 - a0*(t0*t0*t0*t0)*(t1*t1*t1) + a1*(t0*t0*t0)*(t1*t1*t1*t1) - a1*(t0*t0*t0*t0)*(t1*t1*t1)*2.0 + a1*(t0*t0*t0*t0*t0)*(t1*t1) - (t0*t0)*(t1*t1*t1*t1)*v0*1.0E+1 + (t0*t0*t0)*(t1*t1*t1)*v0*8.0 - (t0*t0*t0)*(t1*t1*t1)*v1*8.0 + (t0*t0*t0*t0)*(t1*t1)*v1*1.0E+1 - (t0*t0)*(t1*t1*t1)*x0*2.0E+1 + (t0*t0*t0)*(t1*t1)*x1*2.0E+1)) / 2.0;
		b_spline[1] = (1.0 / pow(t0 - t1, 5.0)*((t0*t0*t0*t0*t0)*v1*2.0 - (t1*t1*t1*t1*t1)*v0*2.0 + a0*t0*(t1*t1*t1*t1*t1)*2.0 - a1*(t0*t0*t0*t0*t0)*t1*2.0 + t0*(t1*t1*t1*t1)*v0*1.0E+1 - (t0*t0*t0*t0)*t1*v1*1.0E+1 - a0*(t0*t0)*(t1*t1*t1*t1) - a0*(t0*t0*t0)*(t1*t1*t1)*4.0 + a0*(t0*t0*t0*t0)*(t1*t1)*3.0 - a1*(t0*t0)*(t1*t1*t1*t1)*3.0 + a1*(t0*t0*t0)*(t1*t1*t1)*4.0 + a1*(t0*t0*t0*t0)*(t1*t1) + (t0*t0)*(t1*t1*t1)*v0*1.6E+1 - (t0*t0*t0)*(t1*t1)*v0*2.4E+1 + (t0*t0)*(t1*t1*t1)*v1*2.4E+1 - (t0*t0*t0)*(t1*t1)*v1*1.6E+1 + (t0*t0)*(t1*t1)*x0*6.0E+1 - (t0*t0)*(t1*t1)*x1*6.0E+1)) / 2.0;
		b_spline[2] = 1.0 / pow(t0 - t1, 5.0)*(a0*(t1*t1*t1*t1*t1) - a1*(t0*t0*t0*t0*t0) + a0*t0*(t1*t1*t1*t1)*4.0 + a0*(t0*t0*t0*t0)*t1*3.0 - a1*t0*(t1*t1*t1*t1)*3.0 - a1*(t0*t0*t0*t0)*t1*4.0 + t0*(t1*t1*t1)*v0*3.6E+1 - (t0*t0*t0)*t1*v0*2.4E+1 + t0*(t1*t1*t1)*v1*2.4E+1 - (t0*t0*t0)*t1*v1*3.6E+1 + t0*(t1*t1)*x0*6.0E+1 + (t0*t0)*t1*x0*6.0E+1 - t0*(t1*t1)*x1*6.0E+1 - (t0*t0)*t1*x1*6.0E+1 - a0*(t0*t0)*(t1*t1*t1)*8.0 + a1*(t0*t0*t0)*(t1*t1)*8.0 - (t0*t0)*(t1*t1)*v0*1.2E+1 + (t0*t0)*(t1*t1)*v1*1.2E+1)*(-1.0 / 2.0);
		b_spline[3] = (1.0 / pow(t0 - t1, 5.0)*(a0*(t0*t0*t0*t0) + a0*(t1*t1*t1*t1)*3.0 - a1*(t0*t0*t0*t0)*3.0 - a1*(t1*t1*t1*t1) - (t0*t0*t0)*v0*8.0 - (t0*t0*t0)*v1*1.2E+1 + (t1*t1*t1)*v0*1.2E+1 + (t1*t1*t1)*v1*8.0 + (t0*t0)*x0*2.0E+1 - (t0*t0)*x1*2.0E+1 + (t1*t1)*x0*2.0E+1 - (t1*t1)*x1*2.0E+1 + a0*(t0*t0*t0)*t1*4.0 - a1*t0*(t1*t1*t1)*4.0 + t0*(t1*t1)*v0*2.8E+1 - (t0*t0)*t1*v0*3.2E+1 + t0*(t1*t1)*v1*3.2E+1 - (t0*t0)*t1*v1*2.8E+1 - a0*(t0*t0)*(t1*t1)*8.0 + a1*(t0*t0)*(t1*t1)*8.0 + t0*t1*x0*8.0E+1 - t0*t1*x1*8.0E+1)) / 2.0;
		b_spline[4] = 1.0 / pow(t0 - t1, 5.0)*(t0*x0*3.0E+1 - t0*x1*3.0E+1 + t1*x0*3.0E+1 - t1*x1*3.0E+1 + a0*(t0*t0*t0)*2.0 + a0*(t1*t1*t1)*3.0 - a1*(t0*t0*t0)*3.0 - a1*(t1*t1*t1)*2.0 - (t0*t0)*v0*1.4E+1 - (t0*t0)*v1*1.6E+1 + (t1*t1)*v0*1.6E+1 + (t1*t1)*v1*1.4E+1 - a0*t0*(t1*t1)*4.0 - a0*(t0*t0)*t1 + a1*t0*(t1*t1) + a1*(t0*t0)*t1*4.0 - t0*t1*v0*2.0 + t0*t1*v1*2.0)*(-1.0 / 2.0);
		b_spline[5] = (1.0 / pow(t0 - t1, 5.0)*(x0*1.2E+1 - x1*1.2E+1 - t0*v0*6.0 - t0*v1*6.0 + t1*v0*6.0 + t1*v1*6.0 + a0*(t0*t0) + a0*(t1*t1) - a1*(t0*t0) - a1*(t1*t1) - a0*t0*t1*2.0 + a1*t0*t1*2.0)) / 2.0;

		for (int i = k0; i < k1; i++) { // spline
			tempt = tempk * control_t;
			pos_out[i] = b_spline[0] + b_spline[1] * tempt + b_spline[2] * tempt * tempt + b_spline[3] * tempt * tempt * tempt + b_spline[4] * tempt * tempt * tempt * tempt + b_spline[5] * tempt * tempt * tempt * tempt * tempt;
			tempk++;
		}
		for (int i = k1; i < maxProgN; i++) { // hold on
			pos_out[i] = pos_out[k1 - 1];
		}
	}
}
