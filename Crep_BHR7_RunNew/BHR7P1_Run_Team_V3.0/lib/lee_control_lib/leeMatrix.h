/*--------------------------------------------------------------/
■ Lee, 2019-02-24 增加2阶矩阵求逆函数 m2_inv
/*-------------------------------------------------------------*/

#pragma once
#define _MAX_MATRIX_DIM 8
#include <stdio.h>

typedef struct MStruct
{
	double m[_MAX_MATRIX_DIM][_MAX_MATRIX_DIM];
	int dim[2];
}MStruct;

 MStruct m_mul(MStruct * m1, MStruct * m2);
 MStruct init_matrix(int m,int n);
 MStruct init_matrix0(int m, int n);
 void show_matrix(MStruct * m);

 MStruct get_3lcross_m(double vec[3]);
 MStruct cross_3(double vec1[3], double vec2[3]);

 MStruct m_sum(MStruct * m1, MStruct * m2);
 MStruct m_sub(MStruct * m1, MStruct * m2);
 MStruct create_unit_matrix(int m);
 MStruct create_zero_matrix(int m);
 MStruct comb_matrix_h(MStruct * m1, MStruct * m2);
 MStruct comb_matrix_l(MStruct * m1, MStruct * m2);
 MStruct m_pinv(MStruct * m);
 MStruct m_inv(MStruct *m);
 MStruct m_T(MStruct *m);
 MStruct m2_inv(MStruct *m);

 double _lsign(double num);

 //实时插值函数
 //3次多项式
 void realtime_1D_interpolation(double *p, double * v, double p1, double v1, double t, double ST, double CT);
 void realtime_3D_interpolation(double *p, double * v, double *p1, double *v1, double t, double ST, double CT);
 void realtime_1D_interpolation_5(double *p, double * v, double * a, double p1, double v1, double a1, double t, double ST, double CT);

 //离线插值函数
 //3次多项式
 void offline_1D_interpolation(double *p, double * v, double p0, double v0, double p1, double v1, double t, double ST);
 void offline_3D_interpolation(double * p, double * v, double * p0, double * v0, double * p1, double * v1, double t, double ST);