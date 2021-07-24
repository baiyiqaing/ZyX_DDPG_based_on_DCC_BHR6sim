#pragma once

#ifdef __cplusplus
extern "C" {
	int fndDataExchange(double dptJointsPosition[50], double dptJointsVelocity[50], double dptdBodyCaredPosition[20][3], double dptdBodyCaredOrientation[20][3], double dptdBodyCaredVelocoty[20][3], double dptBodyCaredAngularSpeed[20][3], double dptGyroSensor[2][3], double dptAccSensor[3], double dptFootFT[10][6], int k_pre, double dptCmdJointsPosition[50], double dptCmdJointsTorque[50], char *cptMode_flag);
}
#endif