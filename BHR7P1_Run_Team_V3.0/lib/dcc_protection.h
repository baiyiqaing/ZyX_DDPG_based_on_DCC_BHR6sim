#pragma once

#define MAXMUMMUN 10000

struct {
	// Protection sequence
	double ql_1[MAXMUMMUN];
	double ql_2[MAXMUMMUN];
	double ql_3[MAXMUMMUN];
	double ql_4[MAXMUMMUN];
	double ql_5[MAXMUMMUN];
	double ql_6[MAXMUMMUN];

	double qr_1[MAXMUMMUN];
	double qr_2[MAXMUMMUN];
	double qr_3[MAXMUMMUN];
	double qr_4[MAXMUMMUN];
	double qr_5[MAXMUMMUN];
	double qr_6[MAXMUMMUN];

	double qw_1[MAXMUMMUN];
	double qw_2[MAXMUMMUN];
	double qw_3[MAXMUMMUN];

	double qla_1[MAXMUMMUN];
	double qla_2[MAXMUMMUN];
	double qla_3[MAXMUMMUN];
	double qla_4[MAXMUMMUN];

	double qra_1[MAXMUMMUN];
	double qra_2[MAXMUMMUN];
	double qra_3[MAXMUMMUN];
	double qra_4[MAXMUMMUN];

	int k_hop;
};

void fnvJointHopCheck(double *dptJointsNow, double *dptJointsOld, double *dptJointsOldOld, double *dptJointsProtect, double dHopTol, int nKNow, double CONTROL_T);
void fnvJointsProtection(double *dptJointsHop, double *dptJointsSpeedHop, double *dptJointsProtect, double CONTROL_T);