#ifndef MJ_DATAEXCHENGE_C
#define MJ_DATAEXCHENGE_C
#include "Mj_DataExchange.h"
#include "DataOnRobot.h"
#include "../../Dcc_lib/Base/dcc_con_base.h"

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
	FootFT[1][0] = dptFootFT[1][0];
	FootFT[1][1] = dptFootFT[1][1];
	FootFT[1][2] = fndLimit(dptFootFT[1][2], dFrcLimit);
	FootFT[1][3] = fndLimit(dptFootFT[1][4], dTrqLimit); // pit
	FootFT[1][4] = fndLimit(-dptFootFT[1][3], dTrqLimit); // rol
	FootFT[1][5] = dptFootFT[1][5];
	FootFT[2][0] = dptFootFT[0][0];
	FootFT[2][1] = dptFootFT[0][1];
	FootFT[2][2] = fndLimit(dptFootFT[0][2], dFrcLimit);
	FootFT[2][3] = fndLimit(dptFootFT[0][4], dTrqLimit); // pit
	FootFT[2][4] = fndLimit(-dptFootFT[0][3], dTrqLimit); // rol
	FootFT[2][5] = dptFootFT[0][5];
	// printf("%lf\t%lf\n", FootFT[1][4], FootFT[2][4]);

    K_Preview_Con = k_pre;

	// your control should be added here s--------------------------------------
	if (k_pre == 0) {
		FrameReset();
	}
	if (!FrameUpdate());
	// your control should be added here e--------------------------------------

	for (int i = 0; i < 6; i++) {
		dptCmdJointsPosition[i + 3] = -Ref_Leg_Joint[2][i + 1];
		dptCmdJointsPosition[i + 6 + 3] = -Ref_Leg_Joint[1][i + 1];
	}
	dptCmdJointsPosition[0] = -Ref_Waist_Joint[1];
	dptCmdJointsPosition[1] = -Ref_Arm_Joint[2][1];
	dptCmdJointsPosition[2] = -Ref_Arm_Joint[1][1];

	#ifdef CHZ_FixedWaistMotion
		dptCmdJointsPosition[15] = Ref_Waist_Pos[2] - 0.88;
		dptCmdJointsPosition[16] = Ref_Waist_Pos[1];
		dptCmdJointsPosition[17] = Ref_Waist_Pos[0];
		for (int i = 0; i < 3; i++) dptCmdJointsPosition[20 - i] = Ref_Waist_Pos[i + 3];

		dptCmdJointsPosition[15] += 0.10;
		double temp1 = 0.0;
		if (k_pre > 500) 
			temp1 = sin(2.0 * acos(-1.0) * (1.0 / 5.0) * 0.004 * (k_pre - 500));

		for (int i = 0; i < 3; i++) dptCmdJointsPosition[20 - i] = 0.0;
		//dptCmdJointsPosition[19] += 0.2 * temp1; // Pitch
		//dptCmdJointsPosition[20] += 0.2 * temp1; // Roll

	#endif

    return 0;
}

#endif