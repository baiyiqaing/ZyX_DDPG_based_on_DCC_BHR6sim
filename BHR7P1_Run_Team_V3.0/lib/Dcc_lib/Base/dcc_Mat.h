#pragma once
#pragma warning(disable:4024)
#pragma warning(disable:4047)

void dcc_fnvMatDisp(const double *dInMat, int nRow, int nColumn);
void dcc_fnvMatTrans(const double *dInMat, int nRow, int nColumn, double *dOutMat);
void dcc_fnvMatMet(const double *dInMat1, const double *dInMat2, int nRow1, int nColumn1, int nColumn2, const char cMethod, double *dOutMat);
double dcc_fndMatDet(const double *dInMat, int nOrder);
void dcc_fnvMatAdj(const double *dInMat, int nOrder, double *dOutMat);
void dcc_fnvMatInv(const double *dInMat, int nOrder, double *dOutMat);
void dcc_fnvMatCopy(const double *dInMat, int nRow, int nColumn, double *dOutMat);
void dcc_fnvDiag(const double *dInVec, int nRow, int nColumn, double *dOutMat);
void dcc_fnvEye(double dGain, int nOrder, double *dOutMat);

