#ifndef MJ_DATAEXCHENGE_C
#define MJ_DATAEXCHENGE_C
#include "Mj_DataExchange.h"
#include "BHR7P1_Run_Team_V3.0\lib\Tra_Generate.h"
#include "BHR7P1_Run_Team_V3.0\lib\DCC_RunCon.h"
#include "BHR7P1_Run_Team_V3.0\lib\Dcc_lib\PG\DCC_PGFrame.h"
#include "BHR7P1_Run_Team_V3.0\lib\Dcc_lib\Base\dcc_con_base.h"

extern double XS_Pitch, XS_Roll, XS_Yaw;
extern double XS_AccX, XS_AccY, XS_AccZ;
extern double XS_GyrX, XS_GyrY, XS_GyrZ;
extern double Body_VX, Body_VY;
extern double GQ_Roll, GQ_Pitch;
extern double Joint[3][7], Joint_Arm[3][8];
extern double Real_LegTorque[3][7];
extern int PreCon_Mode;
extern int Walk_On;

extern JointsAngle PreCon_LegJoint, Real_LegJoint;
extern ForceSensor F_RFoot, F_LFoot;
extern double Ref_Waist_Joint[4];
extern double FootFT[5][6];
extern int K_Preview_Con;
extern double Ref_Leg_Joint[3][7];
extern double Ref_Arm_Joint[3][7];


int fndMjDataExchange(double dptJointsPosition[50], double dptJointsVelocity[50], double dptGyroSensor[2][3], double dptFootFT[10][6], int k_pre, double dptCmdJointsPosition[50]) {
    // IMU mapping
	XS_Pitch = dptGyroSensor[0][0];
	// if (XS_Pitch > 0) XS_Pitch = XS_Pitch - 3.14;
	// else if (XS_Pitch < 0) XS_Pitch = XS_Pitch + 3.14;
	// else XS_Pitch = 0.0;
    XS_Roll = dptGyroSensor[0][1];
	XS_Yaw = dptGyroSensor[0][2];
	// printf("%lf\t%lf\n", XS_Pitch * 57.3, XS_Roll * 57.3);
	for (int i = 0; i < 6; i++) {
		Real_LegJoint.qr[i] = dptJointsPosition[i];
		Real_LegJoint.ql[i] = dptJointsPosition[i + 6];
    }
    // FootFT mapping
	double dTrqLimit[2] = {-800.0, 800.0};
	double dFrcLimit[2] = {-1800.0, 1800.0};
	F_RFoot.fx = dptFootFT[1][0];
	F_RFoot.fy = dptFootFT[1][1];
	F_RFoot.fz = fndLimit(dptFootFT[1][2], dFrcLimit);
	F_RFoot.tx = fndLimit(dptFootFT[1][4], dTrqLimit); // pit
	F_RFoot.ty = fndLimit(-dptFootFT[1][3], dTrqLimit); // rol
	F_RFoot.tz = dptFootFT[1][5];
	F_LFoot.fx = dptFootFT[0][0];
	F_LFoot.fy = dptFootFT[0][1];
	F_LFoot.fz = fndLimit(dptFootFT[0][2], dFrcLimit);
	F_LFoot.tx = fndLimit(dptFootFT[0][4], dTrqLimit); // pit
	F_LFoot.ty = fndLimit(-dptFootFT[0][3], dTrqLimit); // rol
	F_LFoot.tz = dptFootFT[0][5];
	// printf("%lf\t%lf\n", FootFT[1][4], FootFT[2][4]);

    K_Preview_Con = k_pre;

	// your control should be added here s--------------------------------------
	if (k_pre == 0) { // init
		fnvDccGetPG();		// PlanPG
		PreCon_Mode = PRECON_MOVE_FORWARD;
		//printf("%f\n", dccRunParms.paras_Rot);
		Init_Walk_Parameters();
		Init_Walk_Tra();
	}
#ifdef USE_DCCPGFRAME
	fnvDccPGUpdate(K_Preview_Con);
	if (k_pre % __MaxKprog == 0) printf("========================================= New Circle ===================================== !!\n");
#endif
	PreviewControl_Tra_Generate();
	// your control should be added here e--------------------------------------

	for (int i = 0; i < 6; i++) {
		dptCmdJointsPosition[i + 3] = -PreCon_LegJoint.ql[i + 1];
		dptCmdJointsPosition[i + 6 + 3] = -PreCon_LegJoint.qr[i + 1];
	}
	dptCmdJointsPosition[0] = 0.0;
	dptCmdJointsPosition[1] = -Ref_Arm_Joint[2][1];
	dptCmdJointsPosition[2] = -Ref_Arm_Joint[1][1];

    return 0;
}

#endif