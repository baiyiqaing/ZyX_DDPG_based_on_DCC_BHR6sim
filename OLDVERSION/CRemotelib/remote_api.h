/**
C-rep: V-rep remote api
Header file
bit 20201028
*/
#pragma once

class CRobotConfig
{
public:
	CRobotConfig(int nJointNumber_in, int nBodyNumber_in, int nForceSensorNumber_in, int nMaxColumnForDataRecord_in, double dCONTROL_T_in, double dPROGRAM_T_in);
	~CRobotConfig();
	void fnvReadJoints();
	void fnvSendJoints();
	void fnvReadForceSensor();
	void fnvReadIMU();
	void fnvStartSimulation();
	void fnvTrigger();
	void fnvEndCheck();
	void fnvPushForce(double dPushTime[2], double dPushForce[3]);
	void fnvEndSimulation();
	void fnvDataRecording();
	void fnvDataWriteFile(const char *cptFileName);
	int nConnectedOk_flag;
	int nEndSimulation_flag;
	int k_pre;
	double dSimulationTime;
	double dJointsPosition[50] = { 0.0 }; // for BHR6: Right 1-6, Left 1-6.
	double dJointsVelocity[50] = { 0.0 }; // for BHR6: Right 1-6, Left 1-6.
	double dBodyCaredPosition[20][3] = { 0.0 }; // for BHR6: Trunk, Right foot, Left foot.
	double dBodyCaredVelocoty[20][3] = { 0.0 }; // for BHR6: Trunk, Right foot, Left foot.
	double dBodyCaredOrientation[20][3] = { 0.0 }; // for BHR6: Trunk, Right foot, Left foot.
	double dBodyCaredAngularSpeed[20][3] = { 0.0 }; // for BHR6: Trunk, Right foot, Left foot.
	double dGyroSensor[2][3] = { 0.0 }; // Orientation, Angular speed.
	double dAccSensor[3] = { 0.0 }; 
	double dFootFT[10][6] = { 0.0 }; // Right FT, Left FT.
	double dCmdJointsPosition[50] = { 0.0 }; // for BHR6: Right 1-6, Left 1-6.
	double dCmdJointsTorque[50] = { 0.0 }; // for BHR6: Right 1-6, Left 1-6.
	char *cMode_flag = {};

protected:
	int nPort;
	int nClientID;
	double dCONTROL_T;
	double dPROGRAM_T;
	int nJointNumber;
	int nBodyNumber;
	int nForceSensorNumber;
	double dJointDirection[50];
	double dInitCmdJointBias[50];
	int nJointHandle[50];
	int nForceSeneorHandle[2];
	int nBodyCareHandle[20];
	int nGyroHandle;
	int nAccSendsorHandle;
	void fnvGetObjectHandle();
	void fnvInit();
	FILE *FILEptDataRecord;
	int nMaxColumn;
	int nDataStartCount;
	float fDataRecord[10000000L];
};