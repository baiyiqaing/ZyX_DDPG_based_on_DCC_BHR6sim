/** 
C-rep: V-rep remote api
V-rep server port: 19997
bit 20201028 Made by DCC,
bit 20201226 for runteam
*/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <conio.h>

#include <extApi.h>
//#include <extApi.c>
#include <extApiPlatform.h>
//#include <extApiPlatform.c>

#include "remote_api.h"
#include "..\Crep_DataExchange.h"
extern "C" {
	#include "..\BHR7P1_Run_Team_V3.0\lib\Dcc_lib\Base\dcc_get_key.h"
}
#define BHR7P1
//#define BHR6

// ****************************************************** Include Your Headers Here **********************************************************************
#include "..\BHR7P1_Run_Team_V3.0\lib\Tra_Generate.h"
#include "..\BHR7P1_Run_Team_V3.0\lib\DCC_RunCon.h"
#include "..\BHR7P1_Run_Team_V3.0\lib\Dcc_lib\Base\dcc_con_base.h"
// **************************************************** Extern Your Data To Save Here ********************************************************************
extern "C" {
	double XS_Pitch, XS_Roll;

	extern double ref_pitch_re;
	extern double rel_pitch_re;
	extern double con_pitch_re;
	extern double ref_roll_re;
	extern double rel_roll_re;
	extern double con_roll_re;
	extern double additor_pit;
	extern double additor_rol;

	extern Run_Horizontal TPC_Run_ConVal;
	extern double dx_MPCTPC;
	extern double dy_MPCTPC;
	extern Run_FS Rfoot_ref_re, Lfoot_ref_re, Rfoot_rel_re, Lfoot_rel_re;
	extern Run_Horizontal zmp_rel_re, zmp_ref_re, delta_com_re, delta_vcom_re;
	extern double LIPM_ZMP_x;
	extern double LIPM_ZMP_y;
	extern double FzR_rel;
	extern double FzR_ref;
	extern double FzL_rel;
	extern double FzL_ref;
	extern Run_ConVal DCC_Run;
	extern Run_ConVal FootCompliance_ConVal;
	extern Run_Rotational BodyRot_ConVal;
	extern Run_ConVal ContactConVal;

	extern strJointPtotection strDCC_Protection;
	extern double dJointsPositionCmd[12];

	// suppoly
	extern double dSupPoly[8];
	// SupSig
	extern int nSupSig;
	// Limp
	extern double dLipmRe[8]; // { delta_cx, delta_vx, delta_cy, delta_vy, lipm_px, lipm_py, px_ref, py_ref } 
	// MZmp // rec dcc
	extern double dMZmpRe[7]; // { sur_px, sur_py, md_cx, md_cy }
	// AddiTrq Lipm
	extern double dAddiTrqRe[6]; // { frz, trx, try, flz, tlx, tly  }
	// CalFootFTRef
	extern double dFootFTRefRe[6]; // { frz, trx, try, flz, tlx, tly  }
	// GrfCon
	extern double dFootFTRelRe[6]; // { frz, trx, try, flz, tlx, tly  }
	extern double dGrfConValRe[6]; // { prz, rrx, rry, plz, rlx, rly }
	extern double dKfKpRe[8];
	// Tpc
	extern double dTpcRe[2]; // { tpc_cx, tpc_cy }
	// arm
	extern double dArm[2];  // { Ra, La }
}
ForceSensor F_RFoot, F_LFoot;
Position P_ZMPRel_B;
// *******************************************************************************************************************************************************


// *********************************************** Basic Config of The Simulation and V-rep **************************************************************
#define CONTROL_T 0.004
#define JointsNumberInTotal 14
#define BodyCaredInTotal 3
#define ForceSersorsIntotal 2
#define MaxColumnForDataRecord 400
const char *_cptJointsName_[] = { "Right_Leg_1", "Right_Leg_2", "Right_Leg_3", "Right_Leg_4", "Right_Leg_5", "Right_Leg_6",
								   "Left_Leg_1",  "Left_Leg_2",  "Left_Leg_3",  "Left_Leg_4",  "Left_Leg_5",  "Left_Leg_6",
								   "Rigt_arm_1", "Left_arm_1"};
const char *_cptForceSensorName_[] = { "Rigt_fs", "Left_fs" };
const char *_cptBodyCaredName_[] = { "Body", "Right_Foot", "Left_Foot" };
const char *_cptGyroName_ = { "GyroSensor_referance" };
const char *_cptAccSeneorName_ = { "Accelerometer_referance" };
const char *_cptGyroOrientationValName_[] = { "AnglX", "AnglY", "AnglZ" };
const char *_cptGyroAngularSpeedValName_[] = { "gyroX", "gyroY", "gyroZ" };
const char *_cptAccSensorValName_[] = { "accX", "accY", "accZ" };
const char *_cptPushForceName_[] = { "forceX", "forceY", "forceZ" };
/** Please add this code in the Non-threaded child script of your GyroSensor
	sim.setFloatSignal('gyroX', gyroData[1])
	sim.setFloatSignal('gyroY', gyroData[2])
	sim.setFloatSignal('gyroZ', gyroData[3])
	sim.setFloatSignal('AnglX',euler[1])
	sim.setFloatSignal('AnglY',euler[2])
	sim.setFloatSignal('AnglZ',euler[3])
*/
/** Please add this code in the Non-threaded child script of your Accelerometer
	sim.setFloatSignal('accX', accel[1])
	sim.setFloatSignal('accY', accel[2])
	sim.setFloatSignal('accZ', accel[3])
*/
#ifdef BHR6
const double _dJointDirection_[] = { 1.0, 1.0, -1.0, -1.0, -1.0, 1.0,   // Right leg direction
				                     1.0, 1.0, -1.0, -1.0, -1.0, 1.0};  // Left  leg direction
const double _dInitCmdJointPosition_[] = { DccD2R(0.0), DccD2R(0.0), DccD2R(10.0), DccD2R(-20.0), DccD2R(10.0), DccD2R(0.0),
										   DccD2R(0.0), DccD2R(0.0), DccD2R(10.0), DccD2R(-20.0), DccD2R(10.0), DccD2R(0.0) };
const double _dInitCmdJointBias_[] = { DccD2R(0.0), DccD2R(0.0), DccD2R(-0.0), DccD2R(0.0), DccD2R(-0.0), DccD2R(0.0),
									   DccD2R(0.0), DccD2R(0.0), DccD2R(-0.0), DccD2R(0.0), DccD2R(-0.0), DccD2R(0.0) };
#endif
#ifdef BHR7P1
const double _dJointDirection_[] = { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,   // Right leg direction
				                     1.0, 1.0, 1.0, 1.0, 1.0, 1.0,  // Left  leg direction
								     1.0, 1.0 };
const double _dInitCmdJointPosition_[] = { DccD2R(0.0), DccD2R(0.0), DccD2R(12.5), DccD2R(-25.0), DccD2R(12.5), DccD2R(0.0), 
										   DccD2R(0.0), DccD2R(0.0), DccD2R(12.5), DccD2R(-25.0), DccD2R(12.5), DccD2R(0.0),
                                           DccD2R(0.0), DccD2R(0.0) };
const double _dInitCmdJointBias_[] = { DccD2R(0.0), DccD2R(0.0), DccD2R(-12.5), DccD2R(25.0), DccD2R(-12.5), DccD2R(0.0),
									   DccD2R(0.0), DccD2R(0.0), DccD2R(-12.5), DccD2R(25.0), DccD2R(-12.5), DccD2R(0.0),
									   DccD2R(0.0), DccD2R(0.0) };
#endif
// ************************************************************** Good Luck ******************************************************************************



CRobotConfig::CRobotConfig(int nJointNumber_in, int nBodyNumber_in, int nForceSensorNumber_in, int nMaxColumnForDataRecord_in, double dCONTROL_T_in, double dPROGRAM_T_in) {
	nJointNumber = nJointNumber_in;
	nBodyNumber = nBodyNumber_in;
	nForceSensorNumber = nForceSensorNumber_in;
	dCONTROL_T = dCONTROL_T_in;
	dPROGRAM_T = dPROGRAM_T_in;
	nEndSimulation_flag = 0;
	k_pre = 0;
	dSimulationTime = 0.0;
	cMode_flag = "Position";
	nMaxColumn = nMaxColumnForDataRecord_in;
	nDataStartCount = 0;
	fnvInit();
}

CRobotConfig::~CRobotConfig() {

}

void CRobotConfig::fnvInit() {
	nPort = 19997;
	simxFinish(-1);
	nClientID = simxStart("127.0.0.1", nPort, true, true, 5000, (int)(dCONTROL_T * 1000));
	if (nClientID != -1) {
		printf("V-rep connected.");
		nConnectedOk_flag = 1;
	}
	else {
		printf("V-rep can't be connected.");
		nConnectedOk_flag = 0;
	}
	fnvKeyInit();
}

void CRobotConfig::fnvStartSimulation() {
	simxSynchronous(nClientID, true);
	simxStartSimulation(nClientID, simx_opmode_blocking);
	fnvGetObjectHandle();
	for (int i = 0; i < nJointNumber; i++) {
		dJointDirection[i] = _dJointDirection_[i];
		dCmdJointsPosition[i] = _dInitCmdJointPosition_[i];
		dInitCmdJointBias[i] = _dInitCmdJointBias_[i];
	}
	for (int i = 0; i < nJointNumber; i++) {
		simxSetObjectIntParameter(nClientID, *(nJointHandle + i), 2001, 1, simx_opmode_oneshot); // In position mode when begining
	}
	for (int i = 0; i < 10; i++) { // Pre read the returned value for 5 times
		fnvReadJoints();
		fnvReadForceSensor();
		fnvReadIMU();
		simxSynchronousTrigger(nClientID);
	}
	printf("Press Space To Stop ...\n");
}

void CRobotConfig::fnvTrigger() {
	if (simxGetConnectionId(nClientID) != -1) {
		simxSynchronousTrigger(nClientID);
		k_pre++;
		dSimulationTime = k_pre * dCONTROL_T;
	}
	else {
		fnvEndSimulation();
	}
}

void CRobotConfig::fnvEndCheck() {
	if (k_pre * dCONTROL_T > dPROGRAM_T) {
		nEndSimulation_flag = 1;
	}
	else {
		nEndSimulation_flag = 0;
	}
}

void CRobotConfig::fnvEndSimulation() {
	if (nEndSimulation_flag == 1) {
		simxStopSimulation(nClientID, simx_opmode_blocking);
		simxFinish(nClientID);
	}
}

void CRobotConfig::fnvGetObjectHandle() {
	for (int i = 0; i < nJointNumber; i++) {
		simxGetObjectHandle(nClientID, _cptJointsName_[i], nJointHandle + i, simx_opmode_oneshot_wait);
	}
	for (int i = 0; i < nForceSensorNumber; i++) {
		simxGetObjectHandle(nClientID, _cptForceSensorName_[i], nForceSeneorHandle + i, simx_opmode_oneshot_wait);
	}
	for (int i = 0; i < nBodyNumber; i++) {
		simxGetObjectHandle(nClientID, _cptBodyCaredName_[i], nBodyCareHandle + i, simx_opmode_oneshot_wait);
	}
	simxGetObjectHandle(nClientID, _cptGyroName_, &nGyroHandle, simx_opmode_oneshot_wait);
	simxGetObjectHandle(nClientID, _cptAccSeneorName_, &nAccSendsorHandle, simx_opmode_oneshot_wait);
}

void CRobotConfig::fnvReadJoints() {
	float fJointsPosition[50] = { 0.0 };
	for (int i = 0; i < nJointNumber; i++) {
		simxGetJointPosition(nClientID, *(nJointHandle + i), (fJointsPosition + i), simx_opmode_oneshot);
		*(dJointsVelocity + i) = ((double)*(fJointsPosition + i) * dJointDirection[i] - *(dJointsPosition + i)) / dCONTROL_T;
		*(dJointsPosition + i) = (double)*(fJointsPosition + i) * dJointDirection[i];
	}
}

void CRobotConfig::fnvSendJoints() {
	if (cMode_flag == "Position") { // In position mode
		float fJointPosition[50] = { 0.0 };
		for (int i = 0; i < nJointNumber; i++) {
			simxSetObjectIntParameter(nClientID, *(nJointHandle + i), 2001, 1, simx_opmode_oneshot); 
			fJointPosition[i] = (float)(dCmdJointsPosition[i] + dInitCmdJointBias[i]) * dJointDirection[i];
			simxSetJointTargetPosition(nClientID, *(nJointHandle + i), fJointPosition[i], simx_opmode_oneshot);
		}
	}
	else if (cMode_flag == "Torque") { // In torque mode
		float fJointTorque[50] = { 0.0 };
		for (int i = 0; i < nJointNumber; i++) {
			simxSetObjectIntParameter(nClientID, *(nJointHandle + i), 2001, 0, simx_opmode_oneshot);
			if (dCmdJointsTorque[i] > 0.0) {
				simxSetJointTargetVelocity(nClientID, *(nJointHandle + i), 999.9, simx_opmode_oneshot);
			}
			else {
				simxSetJointTargetVelocity(nClientID, *(nJointHandle + i), -999.9, simx_opmode_oneshot);
			}
			fJointTorque[i] = (float)(fabs(dCmdJointsTorque[i]));
			simxSetJointForce(nClientID, *(nJointHandle + i), fJointTorque[i], simx_opmode_oneshot);
		}
	}
	else {
		printf("Your cMode_flag is wrong, please check the motor mode.");
	}
}

void CRobotConfig::fnvReadForceSensor() {
	float fForce_temp[3];
	float fTorque_temp[3];
	unsigned char *cptState = (unsigned char*)malloc(sizeof(unsigned char));
	for (int i = 0; i < nForceSensorNumber; i++) {
		simxReadForceSensor(nClientID, nForceSeneorHandle[i], cptState, fForce_temp, fTorque_temp, simx_opmode_streaming);
		for (int j = 0; j < 3; j++) {
			dFootFT[i][j] = (double)fForce_temp[j];
			dFootFT[i][j + 3] = (double)fTorque_temp[j];
		}
	}
	free(cptState);
}

void CRobotConfig::fnvReadIMU() {
	float fBodyCaredPosition_temp[3] = { 0.0 };
	float fBodyCaredOrientation_temp[3] = { 0.0 };
	float fGyroSensorOrientation_temp[3] = { 0.0 };
	float fGyroSensorAngularSpeed_temp[3] = { 0.0 };
	float fAccSensor_temp[3] = { 0.0 };
	for (int i = 0; i < nBodyNumber; i++) {
		simxGetObjectPosition(nClientID, *(nBodyCareHandle + i), -1, fBodyCaredPosition_temp, simx_opmode_streaming);
		simxGetObjectOrientation(nClientID, *(nBodyCareHandle + i), -1, fBodyCaredOrientation_temp, simx_opmode_streaming);
		for (int j = 0; j < 3; j++) {
			dBodyCaredVelocoty[i][j] = ((double)*(fBodyCaredPosition_temp + j) - dBodyCaredPosition[i][j]) / dCONTROL_T;
			dBodyCaredPosition[i][j] = (double)*(fBodyCaredPosition_temp + j);
			dBodyCaredAngularSpeed[i][j] = ((double)*(fBodyCaredOrientation_temp + j) - dBodyCaredOrientation[i][j]) / dCONTROL_T;
			dBodyCaredOrientation[i][j] = (double)*(fBodyCaredOrientation_temp + j);
		}
	}
	for (int i = 0; i < 3; i++) {
		simxGetFloatSignal(nClientID, _cptGyroOrientationValName_[i], (fGyroSensorOrientation_temp + i), simx_opmode_streaming);
		simxGetFloatSignal(nClientID, _cptGyroAngularSpeedValName_[i], (fGyroSensorAngularSpeed_temp + i), simx_opmode_streaming);
		simxGetFloatSignal(nClientID, _cptAccSensorValName_[i], (fAccSensor_temp + i), simx_opmode_streaming);
		dGyroSensor[0][i] = (double)fGyroSensorOrientation_temp[i];
		dGyroSensor[1][i] = (double)fGyroSensorAngularSpeed_temp[i];
		dAccSensor[i] = (double)fAccSensor_temp[i];
	}
}

void CRobotConfig::fnvPushForce(double dPushTime[2], double dPushForce[3]) {
	float fPushForce_temp[3];
	if (dSimulationTime > dPushTime[0] && dSimulationTime < dPushTime[1]) {
		for (int i = 0; i < 3; i++) {
			fPushForce_temp[i] = (float)dPushForce[i];
			simxSetFloatSignal(nClientID, _cptPushForceName_[i], fPushForce_temp[i], simx_opmode_streaming);
		}
	}
	else {
		for (int i = 0; i < 3; i++) {
			//simxClearFloatSignal(nClientID, _cptPushForceName_[i], simx_opmode_streaming);
			simxSetFloatSignal(nClientID, _cptPushForceName_[i], 0.0, simx_opmode_streaming);
		}
	}
}

char LogName[MAX_RESP_MSGSTR_SIZE][30] = { 0 };
/* Save response data*/

void SetLogName_default(int cnt, int k, char* logname, char mode[], int i)
{
	if (cnt != 1) return;
	strcat(LogName[k], logname);
	if (!strcmp(mode, "num"))
	{
		char stemp[2] = "1"; stemp[0] = (char)(i + '1');
		strcat(LogName[k], stemp);
	}
	else if (!strcmp(mode, "pos"))
	{
		char stemp[2] = "1"; stemp[0] = (char)(i + 'x');
		strcat(LogName[k], stemp);
	}
	else if (!strcmp(mode, "rot"))
	{
		if (i == 0) strcat(LogName[k], ".pitch");
		if (i == 1) strcat(LogName[k], ".row");
		if (i == 2) strcat(LogName[k], ".yaw");
	}
	else if (!strcmp(mode, "posrot"))
	{
		if (i == 0) strcat(LogName[k], ".x");
		if (i == 1) strcat(LogName[k], ".y");
		if (i == 2) strcat(LogName[k], ".z");
		if (i == 3) strcat(LogName[k], ".pitch");
		if (i == 4) strcat(LogName[k], ".row");
		if (i == 5) strcat(LogName[k], ".yaw");
	}
}

#define SetLogName(a, b, c) SetLogName_default(cycle_cnt, nDataStartTemp, a, b, c)

void CRobotConfig::fnvDataRecording() {
	int nDataStartTemp = 0;
	int i, j;
	float *fptDataBuff = NULL;
	static int cycle_cnt = 0;
	fptDataBuff = &fDataRecord[nDataStartCount];

	if (cycle_cnt++ >= 10) cycle_cnt = 10;

	// ********************************************************** Default Data ****************************************************************
	for (i = 0; i < 1; i++, nDataStartTemp++) SetLogName("K_pre", "none", i), fptDataBuff[nDataStartTemp] = k_pre; // Control step.
	for (i = 0; i < 1; i++, nDataStartTemp++) SetLogName("Time", "none", i), fptDataBuff[nDataStartTemp] = dSimulationTime; // Control step.
	for (i = 0; i < nJointNumber; i++, nDataStartTemp++) SetLogName("JointsPos_rel", "num", i), fptDataBuff[nDataStartTemp] = dJointsPosition[i]; // Real joint angle.
	for (i = 0; i < nJointNumber; i++, nDataStartTemp++) SetLogName("JointsVel_rel", "num", i), fptDataBuff[nDataStartTemp] = dJointsVelocity[i]; // Real joint speed.
	for (i = 0; i < nJointNumber; i++, nDataStartTemp++) SetLogName("JointsPos_cmd", "num", i), fptDataBuff[nDataStartTemp] = dJointsPositionCmd[i]; // Cmd rjoint angle.
	for (i = 0; i < nBodyNumber; i++) {
		for(j = 0; j < 3; j++, nDataStartTemp++) SetLogName("BodyPos", "pos", j), fptDataBuff[nDataStartTemp] = dBodyCaredPosition[i][j]; // Bodys position.
	}
	for (i = 0; i < nBodyNumber; i++) {
		for (j = 0; j < 3; j++, nDataStartTemp++) SetLogName("BodyVel", "pos", j), fptDataBuff[nDataStartTemp] = dBodyCaredVelocoty[i][j]; // Bodys velocity.
	}
	for (i = 0; i < nBodyNumber; i++) {
		for (j = 0; j < 3; j++, nDataStartTemp++) SetLogName("BodyOri", "rot", j), fptDataBuff[nDataStartTemp] = dBodyCaredOrientation[i][j]; // Bodys orientation.
	}
	for (i = 0; i < nBodyNumber; i++) {
		for (j = 0; j < 3; j++, nDataStartTemp++) SetLogName("BodyOme", "rot", j), fptDataBuff[nDataStartTemp] = dBodyCaredAngularSpeed[i][j]; // Bodys angular speed.
	}
	for (i = 0; i < 2; i++) {
		for (j = 0; j < 3; j++, nDataStartTemp++) fptDataBuff[nDataStartTemp] = dGyroSensor[i][j]; // IMU: orientation, Angular speed.
	}
	for (i = 0; i < 3; i++, nDataStartTemp++) SetLogName("Accm", "pos", i), fptDataBuff[nDataStartTemp] = dAccSensor[i]; // Accelermeter
	for (i = 0; i < nForceSensorNumber; i++) {
		for (j = 0; j < 6; j++, nDataStartTemp++) fptDataBuff[nDataStartTemp] = dFootFT[i][j]; // Force sensor data: F x-y-z, T x-y-z;
	}

	for (i = 0; i < 1; i++, nDataStartTemp++) fptDataBuff[nDataStartTemp] = 123;
	for (i = 0; i < 1; i++, nDataStartTemp++) SetLogName("delta_cx", "none", 0), fptDataBuff[nDataStartTemp] = dLipmRe[0];
	for (i = 0; i < 1; i++, nDataStartTemp++) SetLogName("delta_vx", "none", 0), fptDataBuff[nDataStartTemp] = dLipmRe[1];
	for (i = 0; i < 1; i++, nDataStartTemp++) SetLogName("delta_cy", "none", 0), fptDataBuff[nDataStartTemp] = dLipmRe[2];
	for (i = 0; i < 1; i++, nDataStartTemp++) SetLogName("delta_vy", "none", 0), fptDataBuff[nDataStartTemp] = dLipmRe[3];
	for (i = 0; i < 1; i++, nDataStartTemp++) SetLogName("lipm_px", "none", 0), fptDataBuff[nDataStartTemp]	 = dLipmRe[4];
	for (i = 0; i < 1; i++, nDataStartTemp++) SetLogName("px_sens", "none", 0), fptDataBuff[nDataStartTemp] = dLipmRe[6];
	for (i = 0; i < 1; i++, nDataStartTemp++) SetLogName("px_ref", "none", 0), fptDataBuff[nDataStartTemp]   = dLipmRe[5];
	for (i = 0; i < 1; i++, nDataStartTemp++) SetLogName("px_ref", "none", 0), fptDataBuff[nDataStartTemp] = dLipmRe[7];
	for (i = 0; i < 1; i++, nDataStartTemp++) SetLogName("lipm_py", "none", 0), fptDataBuff[nDataStartTemp]  = dLipmRe[8];
	for (i = 0; i < 1; i++, nDataStartTemp++) SetLogName("py_sens", "none", 0), fptDataBuff[nDataStartTemp] = dLipmRe[10];
	for (i = 0; i < 1; i++, nDataStartTemp++) SetLogName("py_ref", "none", 0), fptDataBuff[nDataStartTemp]   = dLipmRe[9];
	for (i = 0; i < 1; i++, nDataStartTemp++) SetLogName("py_ref", "none", 0), fptDataBuff[nDataStartTemp] = dLipmRe[11];
	for (i = 0; i < 1; i++, nDataStartTemp++) fptDataBuff[nDataStartTemp] = 321;
	for (i = 0; i < 1; i++, nDataStartTemp++) SetLogName("tpc_x", "none", 0), fptDataBuff[nDataStartTemp] = dTpcRe[0]; 
	for (i = 0; i < 1; i++, nDataStartTemp++) SetLogName("tpc_y", "none", 0), fptDataBuff[nDataStartTemp] = dTpcRe[1];

	for (i = 0; i < 1; i++, nDataStartTemp++) fptDataBuff[nDataStartTemp] = 456;
	for (i = 0; i < 1; i++, nDataStartTemp++) SetLogName("frz", "none", 0), fptDataBuff[nDataStartTemp] = dAddiTrqRe[0];
	for (i = 0; i < 1; i++, nDataStartTemp++) SetLogName("trx", "none", 0), fptDataBuff[nDataStartTemp] = dAddiTrqRe[1];
	for (i = 0; i < 1; i++, nDataStartTemp++) SetLogName("try", "none", 0), fptDataBuff[nDataStartTemp] = dAddiTrqRe[2];
	for (i = 0; i < 1; i++, nDataStartTemp++) SetLogName("flz", "none", 0), fptDataBuff[nDataStartTemp] = dAddiTrqRe[3];
	for (i = 0; i < 1; i++, nDataStartTemp++) SetLogName("tlx", "none", 0), fptDataBuff[nDataStartTemp] = dAddiTrqRe[4];
	for (i = 0; i < 1; i++, nDataStartTemp++) SetLogName("tly", "none", 0), fptDataBuff[nDataStartTemp] = dAddiTrqRe[5];
	for (i = 0; i < 1; i++, nDataStartTemp++) SetLogName("xmid_rel", "none", 0), fptDataBuff[nDataStartTemp] = dAddiTrqRe[6];
	for (i = 0; i < 1; i++, nDataStartTemp++) SetLogName("alpha_rel", "none", 0), fptDataBuff[nDataStartTemp] = dAddiTrqRe[7];
	for (i = 0; i < 1; i++, nDataStartTemp++) fptDataBuff[nDataStartTemp] = 654;
	for (i = 0; i < 1; i++, nDataStartTemp++) SetLogName("frz", "none", 0), fptDataBuff[nDataStartTemp] = dFootFTRelRe[0];
	for (i = 0; i < 1; i++, nDataStartTemp++) SetLogName("frz", "none", 0), fptDataBuff[nDataStartTemp] = dFootFTRefRe[0];
	for (i = 0; i < 1; i++, nDataStartTemp++) SetLogName("cvrz", "none", 0), fptDataBuff[nDataStartTemp] = dGrfConValRe[0];
	for (i = 0; i < 1; i++, nDataStartTemp++) SetLogName("trx", "none", 0), fptDataBuff[nDataStartTemp] = dFootFTRelRe[1];
	for (i = 0; i < 1; i++, nDataStartTemp++) SetLogName("trx", "none", 0), fptDataBuff[nDataStartTemp] = dFootFTRefRe[1];
	for (i = 0; i < 1; i++, nDataStartTemp++) SetLogName("cvrx", "none", 0), fptDataBuff[nDataStartTemp] = dGrfConValRe[1];
	for (i = 0; i < 1; i++, nDataStartTemp++) SetLogName("try", "none", 0), fptDataBuff[nDataStartTemp] = dFootFTRelRe[2];
	for (i = 0; i < 1; i++, nDataStartTemp++) SetLogName("try", "none", 0), fptDataBuff[nDataStartTemp] = dFootFTRefRe[2];
	for (i = 0; i < 1; i++, nDataStartTemp++) SetLogName("cvry", "none", 0), fptDataBuff[nDataStartTemp] = dGrfConValRe[2];
	for (i = 0; i < 1; i++, nDataStartTemp++) SetLogName("flz", "none", 0), fptDataBuff[nDataStartTemp] = dFootFTRelRe[3];
	for (i = 0; i < 1; i++, nDataStartTemp++) SetLogName("flz", "none", 0), fptDataBuff[nDataStartTemp] = dFootFTRefRe[3];
	for (i = 0; i < 1; i++, nDataStartTemp++) SetLogName("cvlz", "none", 0), fptDataBuff[nDataStartTemp] = dGrfConValRe[3];
	for (i = 0; i < 1; i++, nDataStartTemp++) SetLogName("tlx", "none", 0), fptDataBuff[nDataStartTemp] = dFootFTRelRe[4];
	for (i = 0; i < 1; i++, nDataStartTemp++) SetLogName("tlx", "none", 0), fptDataBuff[nDataStartTemp] = dFootFTRefRe[4];
	for (i = 0; i < 1; i++, nDataStartTemp++) SetLogName("cvlx", "none", 0), fptDataBuff[nDataStartTemp] = dGrfConValRe[4];
	for (i = 0; i < 1; i++, nDataStartTemp++) SetLogName("tly", "none", 0), fptDataBuff[nDataStartTemp] = dFootFTRelRe[5];
	for (i = 0; i < 1; i++, nDataStartTemp++) SetLogName("tly", "none", 0), fptDataBuff[nDataStartTemp] = dFootFTRefRe[5];
	for (i = 0; i < 1; i++, nDataStartTemp++) SetLogName("cvly", "none", 0), fptDataBuff[nDataStartTemp] = dGrfConValRe[5];
	for (i = 0; i < 1; i++, nDataStartTemp++) fptDataBuff[nDataStartTemp] = 789;
	for (i = 0; i < 1; i++, nDataStartTemp++) SetLogName("Rkfpit", "none", 0), fptDataBuff[nDataStartTemp] = dKfKpRe[0];
	for (i = 0; i < 1; i++, nDataStartTemp++) SetLogName("Lkfpit", "none", 0), fptDataBuff[nDataStartTemp] = dKfKpRe[1];
	for (i = 0; i < 1; i++, nDataStartTemp++) SetLogName("Rkfrol", "none", 0), fptDataBuff[nDataStartTemp] = dKfKpRe[2];
	for (i = 0; i < 1; i++, nDataStartTemp++) SetLogName("Lkfrol", "none", 0), fptDataBuff[nDataStartTemp] = dKfKpRe[3];
	for (i = 0; i < 1; i++, nDataStartTemp++) SetLogName("Rkppit", "none", 0), fptDataBuff[nDataStartTemp] = dKfKpRe[4];
	for (i = 0; i < 1; i++, nDataStartTemp++) SetLogName("Lkppit", "none", 0), fptDataBuff[nDataStartTemp] = dKfKpRe[5];
	for (i = 0; i < 1; i++, nDataStartTemp++) SetLogName("Rkprol", "none", 0), fptDataBuff[nDataStartTemp] = dKfKpRe[6];
	for (i = 0; i < 1; i++, nDataStartTemp++) SetLogName("Lkprol", "none", 0), fptDataBuff[nDataStartTemp] = dKfKpRe[7];
	for (i = 0; i < 1; i++, nDataStartTemp++) fptDataBuff[nDataStartTemp] = -654;
	for (i = 0; i < 1; i++, nDataStartTemp++) SetLogName("xmid_pg", "none", 0), fptDataBuff[nDataStartTemp] = dFootFTRelRe[6];
	for (i = 0; i < 1; i++, nDataStartTemp++) SetLogName("alpha_pg", "none", 0), fptDataBuff[nDataStartTemp] = dFootFTRelRe[7];
	for (i = 0; i < 1; i++, nDataStartTemp++) SetLogName("SupSig", "none", 0), fptDataBuff[nDataStartTemp] = nSupSig;
	for (i = 0; i < 1; i++, nDataStartTemp++) fptDataBuff[nDataStartTemp] = -123;
	for (i = 0; i < 1; i++, nDataStartTemp++) SetLogName("Bforw", "none", 0), fptDataBuff[nDataStartTemp] = dSupPoly[0];
	for (i = 0; i < 1; i++, nDataStartTemp++) SetLogName("Bback", "none", 0), fptDataBuff[nDataStartTemp] = dSupPoly[1];
	for (i = 0; i < 1; i++, nDataStartTemp++) SetLogName("Bleft", "none", 0), fptDataBuff[nDataStartTemp] = dSupPoly[2];
	for (i = 0; i < 1; i++, nDataStartTemp++) SetLogName("Brigh", "none", 0), fptDataBuff[nDataStartTemp] = dSupPoly[3];
	for (i = 0; i < 1; i++, nDataStartTemp++) SetLogName("Wforw", "none", 0), fptDataBuff[nDataStartTemp] = dSupPoly[0 + 4];
	for (i = 0; i < 1; i++, nDataStartTemp++) SetLogName("Wback", "none", 0), fptDataBuff[nDataStartTemp] = dSupPoly[1 + 4];
	for (i = 0; i < 1; i++, nDataStartTemp++) SetLogName("Wleft", "none", 0), fptDataBuff[nDataStartTemp] = dSupPoly[2 + 4];
	for (i = 0; i < 1; i++, nDataStartTemp++) SetLogName("Wrigh", "none", 0), fptDataBuff[nDataStartTemp] = dSupPoly[3 + 4];
	for (i = 0; i < 1; i++, nDataStartTemp++) fptDataBuff[nDataStartTemp] = 12321;
	for (i = 0; i < 1; i++, nDataStartTemp++) SetLogName("Wrigh", "none", 0), fptDataBuff[nDataStartTemp] = dMZmpRe[0];
	for (i = 0; i < 1; i++, nDataStartTemp++) SetLogName("Wrigh", "none", 0), fptDataBuff[nDataStartTemp] = dMZmpRe[1];
	for (i = 0; i < 1; i++, nDataStartTemp++) SetLogName("Wrigh", "none", 0), fptDataBuff[nDataStartTemp] = dMZmpRe[2];
	for (i = 0; i < 1; i++, nDataStartTemp++) SetLogName("Wrigh", "none", 0), fptDataBuff[nDataStartTemp] = dMZmpRe[3];
	for (i = 0; i < 1; i++, nDataStartTemp++) SetLogName("Wrigh", "none", 0), fptDataBuff[nDataStartTemp] = dMZmpRe[4];
	for (i = 0; i < 1; i++, nDataStartTemp++) SetLogName("Wrigh", "none", 0), fptDataBuff[nDataStartTemp] = dMZmpRe[5];
	for (i = 0; i < 1; i++, nDataStartTemp++) SetLogName("Wrigh", "none", 0), fptDataBuff[nDataStartTemp] = dMZmpRe[6];
	for (i = 0; i < 1; i++, nDataStartTemp++) fptDataBuff[nDataStartTemp] = 433;
	for (i = 0; i < 1; i++, nDataStartTemp++) SetLogName("Rarm", "none", 0), fptDataBuff[nDataStartTemp] = dArm[0];
	for (i = 0; i < 1; i++, nDataStartTemp++) SetLogName("Larm", "none", 0), fptDataBuff[nDataStartTemp] = dArm[1];

	// *********************************************************** Custom Data ****************************************************************
	/**
		for (i = 0; i < nMaxNum; i++, nDataStartTemp++) fptDataBuff[nDataStartTemp] = dDataName[nNum];  // Your data name.
	*/




	if (nDataStartTemp > nMaxColumn) {
		printf("Error: Too many data to save!!\n");
	}
	nDataStartCount += nMaxColumn;
}

void CRobotConfig::fnvDataWriteFile(const char *cptFileName) {
	int i, j;
	if ((fopen_s(&FILEptDataRecord, cptFileName, "w")) == NULL) {
		printf("Failed to write recording data file!!\n");
	}
	for (i = 0; i < nDataStartCount / nMaxColumn; i++) {
		for (j = 0; j < nMaxColumn; j++) {
			fprintf(FILEptDataRecord, "%f\t", fDataRecord[i * nMaxColumn + j]);
		}
		fprintf(FILEptDataRecord, "\n");
	}
	fclose(FILEptDataRecord);
	printf("Data saved in %s !!\n", cptFileName); 
	printf("Press Enter To Leave ...");
	getchar();
}

int main() {

	// ****************************************** Control period and simulation duration ******************************************************
	const char *cptFileName = "DCC_C-rep.dat";
	double dPROGRAM_T = 20.0;
	double dPushTime_1[2] = { 0.3, 0.4 }; double dPushForce_1[3] = { 0.0 * 50.0, -0.0 * (100.0 + 10.0), 0.0 }; // x, -y, z in V-rep frame
	CRobotConfig *CptRobotBHR6 = new CRobotConfig(JointsNumberInTotal, BodyCaredInTotal, ForceSersorsIntotal, MaxColumnForDataRecord, CONTROL_T, dPROGRAM_T); // Create the robot configuration class, control period and simulation duration are required.
	// *************************************************** Give it a big shot !!! *************************************************************
	
	if (CptRobotBHR6->nConnectedOk_flag == 1) { // Connection check.
		CptRobotBHR6->fnvStartSimulation(); // Start simulation if VS is connected with V-rep, objects name in V-rep and joints configuration are required as concluded in ** Basic Config **.
	}
	while (!CptRobotBHR6->nEndSimulation_flag) { // Simulation duration check.
		CptRobotBHR6->fnvPushForce(dPushTime_1, dPushForce_1); // Push
		CptRobotBHR6->fnvReadJoints(); // Read real joints angle and speed.
		CptRobotBHR6->fnvReadIMU(); // Read body informations and IMU data.
		CptRobotBHR6->fnvReadForceSensor(); // Read force sensors.
	/** This value are obtained now:
		CptRobotBHR6->dJointsPosition[jointnum];
		CptRobotBHR6->dJointsVelocity[jointnum];
		CptRobotBHR6->dBodyCaredPosition[bodynum][direction];
		CptRobotBHR6->dBodyCaredVelocoty[bodynum][direction];
		CptRobotBHR6->dBodyCaredOrientation[bodynum][direction];
		CptRobotBHR6->dBodyCaredAngularSpeed[bodynum][direction];
		CptRobotBHR6->dGyroSensor[][direction];
		CptRobotBHR6->dAccSensor[direction];
		CptRobotBHR6->dFootFT[sensornum][direction];
		CptRobotBHR6->k_pre
	*/
	// ****************************************************** Your controllers ****************************************************************
	/** You can write your controllers or add a main loop function for data exchanging here, like
			fndDataExchange() 
		do.
	*/	
		
		fndDataExchange(CptRobotBHR6->dJointsPosition, CptRobotBHR6->dJointsVelocity, CptRobotBHR6->dBodyCaredPosition, CptRobotBHR6->dBodyCaredOrientation, CptRobotBHR6->dBodyCaredVelocoty, CptRobotBHR6->dBodyCaredAngularSpeed, CptRobotBHR6->dGyroSensor, CptRobotBHR6->dAccSensor, CptRobotBHR6->dFootFT, CptRobotBHR6->k_pre, CptRobotBHR6->dCmdJointsPosition, CptRobotBHR6->dCmdJointsTorque, CptRobotBHR6->cMode_flag);
	

		//printf("%f\n", F_RFoot.fz);


	// ******************************************************** Control ended *****************************************************************
	/** This value are required before this :
		CptRobotBHR6->dCmdJointsPosition[jointnum];
		CptRobotBHR6->dCmdJointsTorque[jointnum];
		CptRobotBHR6->cMode_flag;
		CptRobotBHR6->nEndSimulation_flag;
	*/
		CptRobotBHR6->fnvSendJoints(); // Send joints commands.
		CptRobotBHR6->fnvDataRecording(); // Record data.
		CptRobotBHR6->fnvTrigger(); // Drive the simulation in V-rep for one period.
		CptRobotBHR6->fnvEndCheck(); // Check if the simulation should be ended.	
		fnvCheckKey(0);
		if (_kbhit()) { // Space end.
			if (_getch() == ' ') CptRobotBHR6->nEndSimulation_flag = 1;
		}
	}
	CptRobotBHR6->fnvEndSimulation();
	CptRobotBHR6->fnvDataWriteFile(cptFileName);

	return 0;
}