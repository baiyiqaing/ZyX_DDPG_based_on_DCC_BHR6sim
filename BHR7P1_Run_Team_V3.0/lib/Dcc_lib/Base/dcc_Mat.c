#include <stdio.h>
#include <stdlib.h>
#include "dcc_Mat.h"

#define dcc_PI 3.1415926

/**
Recommended naming format: Matrix[m][n] => double dMaYourName_mxn[m][n];
					       Vector[k][1] => double dVeYourName_kx1[k][1];
						   NumGain      => double dGaYourName[1][1];
warning: You can't use input as output
*/



void dcc_fnvMatDisp(const double *dInMat, int nRow, int nColumn) 
{
	printf("Ans = \n");
	for (int i = 0; i < nRow; i++) {
		for (int j = 0; j < nColumn; j++) {
			printf("%lf\t", *(dInMat + i * nColumn + j));
		}
		printf("\n");
	}
}


void dcc_fnvMatTrans(const double *dInMat, int nRow, int nColumn, double *dOutMat)
{
	for (int i = 0; i < nRow; i++) {
		for (int j = 0; j < nColumn; j++) {
			*(dOutMat + j * nRow + i) = *(dInMat + i * nColumn + j);
		}
	}
}

void dcc_fnvMatMet(const double *dInMat1, const double *dInMat2, int nRow1, int nColumn1, int nColumn2, const char cMethod, double *dOutMat)
{
	double dVal_temp;
	if (cMethod == '+') { // add
		if (nColumn1 != nColumn2) {
			printf("Error: nColumn2\n");
			return;
		}
		for (int i = 0; i < nRow1; i++) {
			for (int j = 0; j < nColumn1; j++) {
				*(dOutMat + i * nColumn1 + j) = *(dInMat1 + i * nColumn1 + j) + *(dInMat2 + i * nColumn1 + j);
			}
		}
	}
	else if (cMethod == '-') { // sub
		if (nColumn1 != nColumn2) {
			printf("Error: nColumn2\n");
			return;
		}
		for (int i = 0; i < nRow1; i++) {
			for (int j = 0; j < nColumn1; j++) {
				*(dOutMat + i * nColumn1 + j) = *(dInMat1 + i * nColumn1 + j) - *(dInMat2 + i * nColumn1 + j);
			}
		}
	}
	else if (cMethod == '*') { 
		if (nColumn1 == 1 && nColumn2 == 1) { // vector dot
			dVal_temp = 0.0;
			for (int i = 0; i < nRow1; i++) {
				dVal_temp += (*(dInMat1 + i)) * (*(dInMat2 + i));
			}
			*dOutMat = dVal_temp;
		}
		else if (nColumn2 == 0) { // num gain
			for (int i = 0; i < nRow1; i++) {
				for (int j = 0; j < nColumn1; j++) {
					*(dOutMat + i * nColumn1 + j) = (*(dInMat1 + i * nColumn1 + j)) * (*(dInMat2));
				}
			}
		}
		else if (nColumn1 > 1 ) { // matrix multiple
			for (int i = 0; i < nRow1; i++) {
				for (int j = 0; j < nColumn2; j++) {
					dVal_temp = 0.0;
					for (int k = 0; k < nColumn1; k++) {
						dVal_temp += (*(dInMat1 + i * nColumn1 + k)) * (*(dInMat2 + k * nColumn2 + j));
					}
					*(dOutMat + i * nColumn2 + j) = dVal_temp;
				}
			}
		}
	}
	else if (cMethod == 'x') { // 
		if (nColumn1 == 1 && nColumn2 == 1 && nRow1 == 3) { // vector cross
			int i_temp, k_temp;
			for (int i = 0; i < 3; i++) {
				i_temp = i + 1;
				if (i_temp == 3) i_temp = 0;
				k_temp = 3 - i - i_temp;
				*(dOutMat + i) = (*(dInMat1 + i_temp)) * (*(dInMat2 + k_temp)) - (*(dInMat1 + k_temp)) * (*(dInMat2 + i_temp));
			}
		}
		else {
			printf("Error: vector cross\n");
			return;
		}
	}
}

double dcc_fndMatDet(const double *dInMat, int nOrder)
{
	double Det = 0.0;
	int i_exp, i, j;
	double *dptMaSmaller = (double *)malloc(sizeof(double) * (nOrder - 1) * (nOrder - 1)); // open stack for a smaller matrix
	dptMaSmaller[0] = 1.0;
	// determaint of a 2x2 matrix
	if (nOrder == 2) {
		Det = (*(dInMat)) * (*(dInMat + 3)) - (*(dInMat + 1)) * (*(dInMat + 2));
	}
	// determaint of a matrix exceeding 2x2 dimension
	else {
		for (i_exp = 0; i_exp < nOrder; i_exp++) { // expend the determaint in row i_mom
			for (i = 0; i < nOrder - 1; i++) { // build the smaller matrix
				for (j = 0; j < nOrder - 1; j++) {
					int i_mom = i < i_exp ? i : i + 1;
					int j_mom = j + 1;
					*(dptMaSmaller + i * (nOrder - 1) + j) = *(dInMat + i_mom * nOrder + j_mom);
				}
			}
			if (i_exp % 2 == 0) {
				Det += (*(dInMat + i_exp * nOrder)) * dcc_fndMatDet(dptMaSmaller, nOrder - 1);
			}
			else {
				Det -= (*(dInMat + i_exp * nOrder)) * dcc_fndMatDet(dptMaSmaller, nOrder - 1);
			}
		}
	}
	free(dptMaSmaller);
	return Det;
}

void dcc_fnvMatAdj(const double *dInMat, int nOrder, double *dOutMat)
{
	int i_exp, j_exp, i, j;
	double *dptMaSmaller = (double *)malloc(sizeof(double) * (nOrder - 1) * (nOrder - 1));
	double *dptMaAdj = (double *)malloc(sizeof(double) * nOrder * nOrder);
	for (i_exp = 0; i_exp < nOrder; i_exp++) { // calculate the algebraic complement at row i_exp, column j_exp
		for (j_exp = 0; j_exp < nOrder; j_exp++) {
			for (i = 0; i < nOrder - 1; i++) { // build the smaller matrix
				for (j = 0; j < nOrder - 1; j++) {
					int i_mom = i < i_exp ? i : i + 1;
					int j_mom = j < j_exp ? j : j + 1;
					*(dptMaSmaller + i * (nOrder - 1) + j) = *(dInMat + i_mom * nOrder + j_mom);
				}
			}
			if ((i_exp + j_exp) % 2 == 0) {
				*(dptMaAdj + i_exp * nOrder + j_exp) = dcc_fndMatDet(dptMaSmaller, nOrder - 1);
			}
			else {
				*(dptMaAdj + i_exp * nOrder + j_exp) = -dcc_fndMatDet(dptMaSmaller, nOrder - 1);
			}
			
		}
	}
	dcc_fnvMatTrans(dptMaAdj, nOrder, nOrder, dOutMat);
	free(dptMaSmaller);
	free(dptMaAdj);
}

void dcc_fnvMatInv(const double *dInMat, int nOrder, double *dOutMat)
{
	double dDet = dcc_fndMatDet(dInMat, nOrder);
	double dTemp[1][1];
	dTemp[0][0] = 1 / dDet;
	dcc_fnvMatAdj(dInMat, nOrder, dOutMat);
	dcc_fnvMatMet(dOutMat, &dTemp[0][0], nOrder, nOrder, 0, '*', dOutMat);
}

void dcc_fnvMatCopy(const double *dInMat, int nRow, int nColumn, double *dOutMat)
{
	for (int i = 0; i < nRow; i++) {
		for (int j = 0; j < nColumn; j++) {
			*(dOutMat + i * nColumn + j) = *(dInMat + i * nColumn + j);
		}
	}
}

void dcc_fnvDiag(const double *dInVec, int nRow, int nColumn, double *dOutMat)
{
	if (nColumn == 1) {
		for (int i = 0; i < nRow; i++) {
			for (int j = 0; j < nRow; j++) {
				if (i == j) {
					*(dOutMat + i * nRow + j) = *(dInVec + i);
				}
				else {
					*(dOutMat + i * nRow + j) = 0.0;
				}
			}
		}
	}
	else {
		printf("Error: diag requires a vector\n");
	}
}

void dcc_fnvEye(double dGain, int nOrder, double *dOutMat)
{
	for (int i = 0; i < nOrder; i++) {
		for (int j = 0; j < nOrder; j++) {
			if (i == j) {
				*(dOutMat + i * nOrder + j) = dGain;
			}
			else {
				*(dOutMat + i * nOrder + j) = 0.0;
			}
		}
	}
}




