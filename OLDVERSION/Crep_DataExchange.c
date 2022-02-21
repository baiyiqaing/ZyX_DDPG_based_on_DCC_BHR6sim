#include "Crep_DataExchange.h"
#include "BHR7P1_Run_Team_V3.0\lib\Tra_Generate.h"
#include "BHR7P1_Run_Team_V3.0\lib\DCC_RunCon.h"
#include "BHR7P1_Run_Team_V3.0\lib\Dcc_lib\PG\DCC_PGFrame.h"

double XS_Pitch, XS_Roll, XS_Yaw;
double XS_AccX, XS_AccY, XS_AccZ;
double XS_GyrX, XS_GyrY, XS_GyrZ;
double Body_VX, Body_VY;
double GQ_Roll, GQ_Pitch;
extern JointsAngle PreCon_LegJoint, Real_LegJoint;
extern double Ref_Arm_Joint[3][7]; 
extern ForceSensor F_RFoot, F_LFoot;
extern DCCRunParms dccRunParms;
extern Position P_ZMPRef_W, P_COMRef_W, P_ZMPRef_B;
double FootFT[5][6];
extern int K_Preview_Con;
int PreCon_Mode;
int Walk_On;

int fndDataExchange(double dptJointsPosition[50], double dptJointsVelocity[50], double dptdBodyCaredPosition[20][3], double dptdBodyCaredOrientation[20][3], double dptdBodyCaredVelocoty[20][3], double dptBodyCaredAngularSpeed[20][3], double dptGyroSensor[2][3], double dptAccSensor[3], double dptFootFT[10][6], int k_pre, double dptCmdJointsPosition[50], double dptCmdJointsTorque[50], char *cptMode_flag) {

	// IMU mapping
	XS_Pitch = dptdBodyCaredOrientation[0][0];
	if (XS_Pitch > 0) XS_Pitch = XS_Pitch - 3.14;
	else if (XS_Pitch < 0) XS_Pitch = XS_Pitch + 3.14;
	else XS_Pitch = 0.0;
	XS_Pitch = -XS_Pitch;
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

	Body_VX = -dptdBodyCaredVelocoty[0][1];
	Body_VY = dptdBodyCaredVelocoty[0][0];
	
	for (int i = 0; i < 6; i++) {
		Real_LegJoint.qr[i] = dptJointsPosition[i];
		Real_LegJoint.ql[i] = dptJointsPosition[i + 6];
	}
	// FootFT mapping
	F_RFoot.fx = dptFootFT[0][0];
	F_RFoot.fy = dptFootFT[0][1];
	F_RFoot.fz = dptFootFT[0][2];
	F_RFoot.tx = dptFootFT[0][3];
	F_RFoot.ty = dptFootFT[0][4];
	F_RFoot.tz = dptFootFT[0][5];
	F_LFoot.fx = dptFootFT[1][0];
	F_LFoot.fy = dptFootFT[1][1];
	F_LFoot.fz = dptFootFT[1][2];
	F_LFoot.tx = dptFootFT[1][3];
	F_LFoot.ty = dptFootFT[1][4];
	F_LFoot.tz = dptFootFT[1][5];
	K_Preview_Con = k_pre;
	if (k_pre == 0) { // init
		fnvDccGetPG();		// PlanPG
		PreCon_Mode = PRECON_MOVE_FORWARD;
		NRT_ReadDCCRunParms(&dccRunParms, L"DCCRunConConfig.json");
		//printf("%f\n", dccRunParms.paras_Rot);
		Init_Walk_Parameters();
		Init_Walk_Tra();
	}
	//printf("%d\n", k_pre);
#ifdef USE_DCCPGFRAME
	fnvDccPGUpdate(K_Preview_Con);
	if (k_pre % __MaxKprog == 0) printf("========================================= New Circle ===================================== !!\n");
#endif
	PreviewControl_Tra_Generate();
	for (int i = 0; i < 6; i++) {
		dptCmdJointsPosition[i] = PreCon_LegJoint.qr[i + 1];
		dptCmdJointsPosition[i + 6] = PreCon_LegJoint.ql[i + 1];
	}
	dptCmdJointsPosition[12] = Ref_Arm_Joint[1][1];
	dptCmdJointsPosition[13] = Ref_Arm_Joint[2][1];
	//printf("%lf", Ref_Arm_Joint[1][1]);
	return 0;
}