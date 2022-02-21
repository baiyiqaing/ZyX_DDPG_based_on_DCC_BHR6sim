#pragma once
#ifdef __cplusplus
extern "C" {
	extern double XS_Pitch, XS_Roll, XS_Yaw;
	extern double XS_AccX, XS_AccY, XS_AccZ;
	extern double XS_GyrX, XS_GyrY, XS_GyrZ;
	extern double Body_VX, Body_VY;
	extern double GQ_Roll, GQ_Pitch;
	extern double Real_LegTorque[3][7];

	int fndDataExchange(double dptJointsPosition[50], double dptJointsVelocity[50], double dptJointsTorque[50], double dptdBodyCaredPosition[20][3], double dptdBodyCaredOrientation[20][3], double dptdBodyCaredVelocoty[20][3], double dptBodyCaredAngularSpeed[20][3], double dptGyroSensor[2][3], double dptAccSensor[3], double dptFootFT[10][6], int k_pre, double dptCmdJointsPosition[50], double dptCmdJointsTorque[50], char *cptMode_flag);
}
#endif