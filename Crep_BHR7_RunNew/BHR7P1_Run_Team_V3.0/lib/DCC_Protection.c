#include "DCC_Protection.h"
#include "Dcc_lib\dcc_con_base.h"

#define nJointsNum 23
strJointPtotection strDCC_Protection = { 0.0 };
double dProtectionTime = 5.0;
double dJointsSpeedHop[nJointsNum] = { 0.0 };
int nErrorFlag = 0;

/*
Describe:	check if joints angle got a hop or nan, if dose, a sequence of joints angle tra will be generated.
Inputs:		dptJointsNow: [ ql1, ql2, ql3, ql4, ql5, ql6, qr1, qr2, qr3, qr4, qr5, qr6, qw1, qw2, qw3, qla1, qla2, qla3, qla4, qra1, qra2, qra3, qra4 ]
			dptJointsOld: [ sequence is the same ]
			dptJointsOldOld: [ sequence is the same ]
			dptJointsProtect: [ sequence is the same ]
			dHopTol: maxmun angle difference to be determained as a hop [rad]
			nKNow: k_pre
Outputs:	Protection angle tra will be generated in strDCC_Protection, each joint is start from 0.
*/
void fnvJointHopCheck(double *dptJointsNow, double *dptJointsOld, double *dptJointsOldOld, double *dptJointsProtect, double dHopTol, int nKNow, double control_t) {
	for (int i = 0; i < nJointsNum; i++) { // check hop or nan
		if (*(dptJointsNow + i) - *(dptJointsOld + i) > dHopTol || *(dptJointsNow + i) - *(dptJointsOld + i) < -dHopTol || isnan(*(dptJointsNow + i))) {
			nErrorFlag = 1;
			strDCC_Protection.k_hop = nKNow;
			break;
		}
	}
	if (nErrorFlag == 1) { // if hop
		nErrorFlag = 2;
		for (int i = 0; i < nJointsNum; i++) { // cal hop speed
			*(dJointsSpeedHop + i) = (*(dptJointsOld + i) - *(dptJointsOldOld + i)) / control_t;
		}
		fnvJointsProtection(dptJointsOld, dJointsSpeedHop, dptJointsProtect, control_t); // protection tra is generated and nErrorFlag becomes 2
	}
}

/*
Describe:	if hop occurs, use this function.
			if you need to add more joint, rectify theS struct def in DCC_Protection.h.
InPuts:		dptJointsHop: [ ql1, ql2, ql3, ql4, ql5, ql6, qr1, qr2, qr3, qr4, qr5, qr6, qw1, qw2, qw3, qla1, qla2, qla3, qla4, qra1, qra2, qra3, qra4 ]
			dptJointsSpeedHop: [ sequence is the same ]
			dptJointsProtect: [ sequence is the same ]
*/
void fnvJointsProtection(double *dptJointsHop, double *dptJointsSpeedHop, double *dptJointsProtect, double control_t) {
	for (int i = 0; i < nJointsNum; i++) {
		fnvFifthSpline(strDCC_Protection.ql_1 + MAXMUMMUN * i, *(dptJointsHop + i), *(dptJointsSpeedHop + i), 0.0, 0.0, *(dptJointsProtect + i), 0.0, 0.0, dProtectionTime, control_t, 'T');
	}
	printf("Joint position hop!!!\n----------------------------- PROTECTION EXECUTED -----------------------------\n");
}
