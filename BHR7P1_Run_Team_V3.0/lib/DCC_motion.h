#pragma once

#define SINGLE_SUP_STILL
// #define FLY_STEPZ_CHECK

typedef struct
{
	double Ankle_L_x[50000];
	double Ankle_L_y[50000];
	double Ankle_L_z[50000];
	double Ankle_R_x[50000];
	double Ankle_R_y[50000];
	double Ankle_R_z[50000];
	double Com_x[50000];
	double Com_y[50000];
	double Com_z[50000];
	double Body_Roll[50000];
	double Body_Pitch[50000];
	double ZMP_x[50000];
	double ZMP_y[50000];
}Motion_Position_Ankle_COM;


typedef struct
{
	Motion_Position_Ankle_COM OutPut;
	Motion_Position_Ankle_COM Mark;
	int Mark_Number;
	double Mark_Time[50000];
}Free_Motion;

void TSpline_PVA(double p0, double v0, double a0, double t0, double p1, double t1, double p2, double v2, double a2, double t2, double control_t, double *S);
void Free_Ankle_Com(double control_t, int n_stop, int n_tcom, double walk_t);
void Free_Motion_Protection(double Ankle_width, double Ankle_height, double H_zc, double control_t, double walk_t, int k_pre);
void Free_Motion_Assig(double Ankle_width, double Ankle_height, double H_zc, double control_t, double walk_t);

// ******* after cal T_com in init_parameters *****************************************************************
// #ifdef SINGLE_SUP_STILL
// //printf("%lf\n", T_com);
// Free_Motion_Assig(ANKLE_WIDTH, H_ANKLE, H_zc, CONTROL_T, T_com);
// #endif
// ******* after cal com in tra_generate **********************************************************************
// #ifdef SINGLE_SUP_STILL
// Tra_COM.x[K_Preview_Con] = Motion_Position.OutPut.Com_x[K_Preview_Con];
// Tra_COM.y[K_Preview_Con] = Motion_Position.OutPut.Com_y[K_Preview_Con];
// Tra_COM.z[K_Preview_Con] = Motion_Position.OutPut.Com_z[K_Preview_Con];
// Tra_RAnkle.x[K_Preview_Con] = Motion_Position.OutPut.Ankle_R_x[K_Preview_Con];
// Tra_RAnkle.y[K_Preview_Con] = Motion_Position.OutPut.Ankle_R_y[K_Preview_Con];
// Tra_RAnkle.z[K_Preview_Con] = Motion_Position.OutPut.Ankle_R_z[K_Preview_Con];
// Tra_LAnkle.x[K_Preview_Con] = Motion_Position.OutPut.Ankle_L_x[K_Preview_Con];
// Tra_LAnkle.y[K_Preview_Con] = Motion_Position.OutPut.Ankle_L_y[K_Preview_Con];
// Tra_LAnkle.z[K_Preview_Con] = Motion_Position.OutPut.Ankle_L_z[K_Preview_Con];
// roll_body = Motion_Position.OutPut.Body_Roll[K_Preview_Con];
// pitch_body = Motion_Position.OutPut.Body_Pitch[K_Preview_Con];
// #endif