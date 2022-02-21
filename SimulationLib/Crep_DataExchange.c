#include "Crep_DataExchange.h"
#include "../BHR7P1_Run_Team_7P2_cmake_multithread/lib/Tra_Generate.h"
//#include "BHR7P1_Run_Team_V3.0\lib\DCC_RunCon.h"

#include "../BHR7P1_Run_Team_7P2_cmake_multithread/ChzFrame/ChzFrameCpp2C.h"

double XS_Pitch, XS_Roll, XS_Yaw;
double XS_AccX, XS_AccY, XS_AccZ;
double XS_GyrX, XS_GyrY, XS_GyrZ;
double Body_VX, Body_VY;
double GQ_Roll, GQ_Pitch;
extern JointsAngle PreCon_LegJoint, Real_LegJoint;
extern ForceSensor F_RFoot, F_LFoot;
//extern DCCRunParms dccRunParms;
double FootFT[5][6];
extern int K_Preview_Con;
int PreCon_Mode;
int Walk_On;

extern double Ref_Leg_Joint[3][7];
extern double Ref_Arm_Joint[3][8];

double Joint[3][7], Joint_Arm[3][8];
double Real_LegTorque[3][7];

#ifdef CHZ_FixedWaistMotion
double Ref_Waist_Pos[6];
#endif

int fndDataExchange(double dptJointsPosition[50], double dptJointsVelocity[50], double dptJointsTorque[50], double dptdBodyCaredPosition[20][3], double dptdBodyCaredOrientation[20][3], double dptdBodyCaredVelocoty[20][3], double dptBodyCaredAngularSpeed[20][3], double dptGyroSensor[2][3], double dptAccSensor[3], double dptFootFT[10][6], int k_pre, double dptCmdJointsPosition[50], double dptCmdJointsTorque[50], char *cptMode_flag) {

	// IMU mapping
	XS_Pitch = dptdBodyCaredOrientation[0][0];
	if (XS_Pitch > 0) XS_Pitch = XS_Pitch - 3.14;
	else if (XS_Pitch < 0) XS_Pitch = XS_Pitch + 3.14;
	else XS_Pitch = 0.0;
	GQ_Pitch = dptdBodyCaredOrientation[0][0];
	XS_Roll = dptdBodyCaredOrientation[0][1];
	GQ_Roll = dptdBodyCaredOrientation[0][1];
	XS_Yaw = dptGyroSensor[0][2];
	XS_AccX = -dptGyroSensor[1][1];
	XS_GyrY = dptGyroSensor[1][0];
	XS_GyrZ = dptGyroSensor[1][2];
	XS_AccX = -dptAccSensor[1]; // buhao
	XS_AccY = dptAccSensor[0]; // buhao
	XS_AccZ = dptAccSensor[2]; // buhao
							   //printf("%f\n", XS_Pitch * 57.3);
	// printf("%lf\t%lf\n", XS_Pitch * 57.3, XS_Roll * 57.3);
	for (int i = 0; i < 6; i++) {
		Real_LegJoint.qr[i] = dptJointsPosition[i];
		Real_LegJoint.ql[i] = dptJointsPosition[i + 6];
		Real_LegTorque[1][i + 1] = dptJointsTorque[i];
		Real_LegTorque[2][i + 1] = dptJointsTorque[i + 6];
	}
	// FootFT mapping

	FootFT[1][0] = dptFootFT[0][0];
	FootFT[1][1] = dptFootFT[0][1];
	FootFT[1][2] = dptFootFT[0][2];
	FootFT[1][3] = dptFootFT[0][3];
	FootFT[1][4] = dptFootFT[0][4];
	FootFT[1][5] = dptFootFT[0][5];
	FootFT[2][0] = dptFootFT[1][0];
	FootFT[2][1] = dptFootFT[1][1];
	FootFT[2][2] = dptFootFT[1][2];
	FootFT[2][3] = dptFootFT[1][3];
	FootFT[2][4] = dptFootFT[1][4];
	FootFT[2][5] = dptFootFT[1][5];

	K_Preview_Con = k_pre;
	if (k_pre == 0) {
		FrameReset();
	}
	if (!FrameUpdate());// printf("ChzFrame Update Failed in %d cycle.\n", k_pre);
	for (int i = 0; i < 6; i++) {
		dptCmdJointsPosition[i] = Ref_Leg_Joint[1][i + 1];
		dptCmdJointsPosition[i + 6] = Ref_Leg_Joint[2][i + 1];
	}
	dptCmdJointsPosition[12] = Ref_Waist_Joint[1];
	dptCmdJointsPosition[13] = Ref_Arm_Joint[1][1];
	dptCmdJointsPosition[14] = Ref_Arm_Joint[2][1];

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