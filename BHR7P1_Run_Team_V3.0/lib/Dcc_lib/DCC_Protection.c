#include "DCC_Protection.h"
#include "Base\dcc_con_base.h"
#include "math.h"

#define nJointsNum 23
strJointPtotection strDCC_Protection = { 0.0 };
double dProtectionTime = 2.0;
double dJointsSpeedHop[nJointsNum] = { 0.0 };

#define LEGLEN 0.3
#define ANKLEWIDTH 0.16
#define WIDTHFOOTINS 0.05
#define WIDTHKNEEINS 0.0695

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
void fnvJointHopCheck(double *dptJointsNow, double *dptJointsOld, double *dptJointsProtect, double dHopTol, int nKNow, double control_t) {
	for (int i = 0; i < nJointsNum; i++) { // check hop or nan
		if ((*(dptJointsNow + i) - *(dptJointsOld + i) > dHopTol || *(dptJointsNow + i) - *(dptJointsOld + i) < -dHopTol || isnan(*(dptJointsNow + i))) && (strDCC_Protection.error_flag == 0)) {
			strDCC_Protection.error_flag = 1;
			strDCC_Protection.k_hop = nKNow;
			break;
		}
	}
	if (strDCC_Protection.error_flag == 1) { // if hop
		strDCC_Protection.error_flag = 2;
		for (int i = 0; i < nJointsNum; i++) { // cal hop speed
			*(dJointsSpeedHop + i) = 0.0; //  (*(dptJointsOld + i) - *(dptJointsOldOld + i)) / control_t;
		}
		fnvJointsProtection(dptJointsOld, dJointsSpeedHop, dptJointsProtect, control_t); // protection tra is generated and strDCC_Protection.error_flag becomes 2
	}
}

void fnvFootCollisionCheck(double *dptJointsNow, double *dptJointsOld, double *dptJointsProtect, double dHopTol, int nKNow, double control_t) {
	double dLegLeft, dLegRight, dFootHoriPosL, dFootHoriPosRight, dFootHoriPosLeft, dKneeHoriPosRight, dKneeHoriPosLeft;
	dLegLeft  = 2 * LEGLEN * cos(0.5 * dptJointsNow[3]); // ql4
	dLegRight = 2 * LEGLEN * cos(0.5 * dptJointsNow[9]); // qr4
	dFootHoriPosLeft  = -0.5 * ANKLEWIDTH - dLegLeft  * sin(dptJointsNow[1]) + WIDTHFOOTINS; // lfoot
	dFootHoriPosRight =  0.5 * ANKLEWIDTH - dLegRight * sin(dptJointsNow[7]) - WIDTHFOOTINS; // rfoot
	dKneeHoriPosLeft  = -0.5 * ANKLEWIDTH - LEGLEN * sin(dptJointsNow[1]) + WIDTHKNEEINS; // lknee
	dKneeHoriPosRight =  0.5 * ANKLEWIDTH - LEGLEN * sin(dptJointsNow[7]) - WIDTHKNEEINS; // rknee
	//printf("%f\t%f\n", dFootHoriPosRight - dFootHoriPosLeft, dKneeHoriPosRight - dKneeHoriPosLeft );
	if (((dFootHoriPosLeft >= dFootHoriPosRight) || (dKneeHoriPosLeft >= dKneeHoriPosRight)) && (strDCC_Protection.error_flag == 0)) { 
		strDCC_Protection.error_flag = 1;
		strDCC_Protection.k_hop = nKNow;
	}
	if (strDCC_Protection.error_flag == 1) { // if crash
		strDCC_Protection.error_flag = 3;
		for (int i = 0; i < nJointsNum; i++) { // cal speed
			*(dJointsSpeedHop + i) = 0.0; // (*(dptJointsOld + i) - *(dptJointsOldOld + i)) / control_t;
		}
		fnvJointsProtection(dptJointsOld, dJointsSpeedHop, dptJointsProtect, control_t); // protection tra is generated and strDCC_Protection.error_flag becomes 3
	}
}

/*
Describe:	if hop occurs, use this function.
			if you need to add more joint, rectify the struct def in DCC_Protection.h.
InPuts:		dptJointsHop: [ ql1, ql2, ql3, ql4, ql5, ql6, qr1, qr2, qr3, qr4, qr5, qr6, qw1, qw2, qw3, qla1, qla2, qla3, qla4, qra1, qra2, qra3, qra4 ]
			dptJointsSpeedHop: [ sequence is the same ]
			dptJointsProtect: [ sequence is the same ]
*/
void fnvJointsProtection(double *dptJointsHop, double *dptJointsSpeedHop, double *dptJointsProtect, double control_t) {
	int nKEnd = (int)(dProtectionTime / control_t);
	for (int i = 0; i < nJointsNum; i++) {
		fnvFifthSpline(strDCC_Protection.ql_1 + MAXMUMMUN * i, MAXMUMMUN, *(dptJointsHop + i), *(dptJointsSpeedHop + i), 0.0, 0.0, *(dptJointsProtect + i), 0.0, 0.0, dProtectionTime, control_t, 'T');
		for (int j = nKEnd; j < MAXMUMMUN; j++) {
			*(strDCC_Protection.ql_1 + MAXMUMMUN * i + j) = *(dptJointsProtect + i);
		}
	}
	if (strDCC_Protection.error_flag == 2) {
		printf("Joint position hop!!!\n----------------------------- PROTECTION EXECUTED -----------------------------\n");
	}
	else if (strDCC_Protection.error_flag == 3) {
		printf("Collision happened!!!\n----------------------------- PROTECTION EXECURED -----------------------------\n");
	}
}

