// 20210427 bit
#ifndef DCC_PGFrame_C
#define DCC_PGFrame_C
#include "DCC_PGFrame.h"

// control platform ====================================================================================================
#define __BHR7RUN

// include the files that containing the required globals for PG & Sens & ConVal, and extern them ======================
#ifdef __BHR7RUN
#include "..\..\Tra_Generate.h"
extern double chzrun_com[30000][16];
extern double chzrun_foot[30000][8];
extern double chzrun_MF[30000][4];
extern double chzrun_signal[30000][3];
extern PreCon_Tra Tra_ZMP;
extern Position P_ZMPRel_B, P_ZMPRef_B;
extern Position P_RAnkleRef_W, P_LAnkleRef_W;
extern Position P_RAnkleRef_B, P_LAnkleRef_B;
extern PreCon_Tra Tra_COM, Tra_ACOM;
extern ForceSensor F_RFoot, F_LFoot;
extern PreCon_Tra Tra_RAnkle, Tra_LAnkle;
extern double pitch_footr, pitch_footl, roll_footr, roll_footl;
extern double pitch_body, roll_body;
extern double XS_Pitch, XS_Roll;
extern double Ref_Arm_Joint[3][7];
#endif
// include your PG file here ===========================================================================================
#include "MPC_unconstrained_coder_forPG\PGMPC.h"

// using PlanPG here =========================================================================================================
#define __StartLegName	/**/	'L'
#define __TimeReady		/**** Time span before start ****/	0.5 // should be longer than TimePrev
#define __TimeEnd		/***** Time span after end ******/	1.0 // should be longer than TimePrev
#define __StepLength	/***** Length of each step ******/	0.3,	0.3,	0.3,	0.3,	0.3,	0.3,	0.3,	0.3,	0.3,	0.0
#define __StepWidth		/**** Width bias of each step ***/	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0
#define __StepHeight	/***** Height of each step ******/	0.04,	0.04,	0.04,	0.04,	0.04,	0.04,	0.04,	0.04,	0.04,	0.04
#define __LandHeight	/** Height of end of each step **/	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0
#define __StepTime		/*** Time span of each step *****/	0.8,	0.8,	0.8,	0.8,	0.8,	0.8,	0.8,	0.8,	0.8,	0.8
#define __SinPercent	/*** Single support propotion ***/	0.8,	0.8,	0.8,	0.8,	0.8,	0.8,	0.8,	0.8,	0.8,	0.8
#define __PauseTime		/* Pause time of double support */	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0,	0.0
#define __HipHeightSin	/* Hip height of single support */	0.69,	0.69,	0.69,	0.69,	0.69,	0.69,	0.69,	0.69,	0.69,	0.69
#define __HipHeightDou	/* Hip height of double support */	0.68,	0.68,	0.68,	0.68,	0.68,	0.68,	0.68,	0.68,	0.68,	0.68
#define __HipHeightPau	/**** Hip height when pause *****/	0.65,	0.65,	0.65,	0.65,	0.65,	0.65,	0.65,	0.65,	0.65,	0.65
#define __TSwitchZMP	/** Time span of switching ZMP **/	0.5
#define __THipHeight	/** Time span of changing hip ***/	0.5

// globals ======================================================================================================
enum supLeg {
	DouSup, RightSup, LeftSup, Fly
};
FILE *FptRePG;
int RePGFileOpenFlag = 0; // 0 -> haven't been opened, 1 -> opened, 2 -> write over and closed
double dVeZmpRefx_500x1[nNumPre][1] = { 0.0 }, dVeZmpRefy_500x1[nNumPre][1] = { 0.0 }, dVeStatex_3x1[nStateNum][1] = { 0.0 }, dVeStatey_3x1[nStateNum][1] = { 0.0 }; // for CalCoM MPC
																																									 // funs for walking ======================================================================================================
																																									 /** Init struct of dccRobotState_tra
																																									 InVal:	dccRobotState_tra * stTraName,
																																									 OutVal: [dccRobotState_tra, ]
																																									 */
void fnvTraInit(dccRobotState_tra * stTraName) {
	memset(stTraName, 0, sizeof(dccRobotState_tra));
	stLastAnkPos.Rfoot.pos.x = 0.5 * __AnkleWidth;
	stLastAnkPos.Rfoot.pos.y = 0.0;
	stLastAnkPos.Rfoot.pos.z = __AnkleHeight;
	stLastAnkPos.Lfoot.pos.x = -0.5 * __AnkleWidth;
	stLastAnkPos.Lfoot.pos.y = 0.0;
	stLastAnkPos.Lfoot.pos.z = __AnkleHeight;
}

/** Init stTraPG for PlanPG
InVal:	[stTraPG, ]
OutVal: [stTraPG, ]
*/
void fnvInitPlanPGTra() {
	fnvTraInit(&stTraPG);
	for (int i = 0; i < __MaxKprog; i++) {
		// init ankle
		stTraPG.Ankle.W.Rfoot.pos.x[i] = 0.5 * __AnkleWidth;
		stTraPG.Ankle.W.Rfoot.pos.y[i] = 0.0;
		stTraPG.Ankle.W.Rfoot.pos.z[i] = __AnkleHeight;
		stTraPG.Ankle.W.Lfoot.pos.x[i] = -0.5 * __AnkleWidth;
		stTraPG.Ankle.W.Lfoot.pos.y[i] = 0.0;
		stTraPG.Ankle.W.Lfoot.pos.z[i] = __AnkleHeight;
		// init hip
		stTraPG.Base.pos.x[i] = 0.0;
		stTraPG.Base.pos.y[i] = 0.0;
		stTraPG.Base.pos.z[i] = __HipHeightHome;
		stTraPG.Base.rot.pit[i] = 0.0;
		stTraPG.Base.rot.rol[i] = 0.0;
		stTraPG.Base.rot.yaw[i] = 0.0;
		// init zmp
		stTraPG.ZMP.W.x[i] = 0.0;
		stTraPG.ZMP.W.y[i] = 0.0;
		// init flag
		stTraPG.SupLeg[i] = DouSup;
	}
}

/** Map current Kprog to the circulated vector
InVal:	int nKprogNow,
OutVal: int nKprogCir,
*/
int fnnCircuKMap(int nKprogNow) {
	return (int)(nKprogNow % __MaxKprog);
}

/** Map current time to the circulated time
InVal:	double dTimeNow,
OutVal: double dTimeCir,
Inc:	fnnCircuKMap,
*/
double fndCircuTMap(double dTimeNow) {
	int nKprogNowTemp = (int)(dTimeNow / __ControlT);
	int nKprogCircu = fnnCircuKMap(nKprogNowTemp);
	return (double)(nKprogCircu * __ControlT);
}

/** Spline world ZMP to a referenced value from current ZMP, and keep it
InVal:	double dTimeStart, double dTimeEnd, double dZMPxy[2], int nKprogNow, [stTraPG.ZMP.W, ]
OutVal: [stTraPG.ZMP.W, ]
Tips:	1. span of (dTimeEnd - dTimeNow) / __ControlT should be smaller than __MaxKprog
2. when dZMPxy == __Holding, ZMP remain unchanged
3. ZMP trajectory is updated before __KLeftToNow up to nKprogNow in circle
4. if ZMP is not changing during a time span, you don't realy need to use this
Inc:	fnvFifthSpline, fnnCircuKMap,
*/
void fnvGetZMPPG(double dTimeStart, double dTimeEnd, double dZMPxy[2], int nKprogNow) {
	double dPGZMPxTemp[__MaxKprog], dPGZMPyTemp[__MaxKprog];
	if (NotHolding(dZMPxy)) { // not holding
		fnvFifthSpline(dPGZMPxTemp, __MaxKprog, stTraPG.ZMP.W.x[fnnCircuKMap((int)(dTimeStart / __ControlT) - 1)], 0.0, 0.0, dTimeStart, dZMPxy[0], 0.0, 0.0, dTimeEnd, __ControlT, 'T');
		fnvFifthSpline(dPGZMPyTemp, __MaxKprog, stTraPG.ZMP.W.y[fnnCircuKMap((int)(dTimeStart / __ControlT) - 1)], 0.0, 0.0, dTimeStart, dZMPxy[1], 0.0, 0.0, dTimeEnd, __ControlT, 'T');
		int k = 0;
		for (int i = (int)(dTimeStart / __ControlT); i < (nKprogNow + __MaxKprog - __KLeftToNow); i++) {
			stTraPG.ZMP.W.x[fnnCircuKMap(i)] = dPGZMPxTemp[k];
			stTraPG.ZMP.W.y[fnnCircuKMap(i)] = dPGZMPyTemp[k];
			k++;
		}
	}
}

/** Spline world anklexy position to a referenced value from current anklexy position, then anklez makes a step, and keep it
InVal:	double dTimeStart, double dTimeEnd, double dAnkxyz[3](the landing position of the ankle), double dStepHeight(the maxi height of the step), char cFootFlag(swing foot flag, 'R' -> right, 'L' -> left), int nKprogNow, [stTraPG.Ankle.W, ]
OutVal: [stTraPG.Ankle.W, ]
Tips:	1. span of (dTimeEnd - dTimeNow) / __ControlT should be smaller than __MaxKprog
2. when dAnkxyz == __Holding, ankle remain unchanged
3. ankle trajectory is updated before __KLeftToNow up to nKprogNow in circle
4. if ankle is not changing during a time span, you don't realy need to use this
Inc:	fnvFifthSpline, fnvDouFourthSpline, fnnCircuKMap,
*/
void fnvGetAnklePG(double dTimeStart, double dTimeEnd, double dAnkxyz[3], double dStepHeight, char cFootFlag, int nKprogNow) {
	double dPGAnklexTemp[__MaxKprog], dPGAnkleyTemp[__MaxKprog], dPGAnklezTemp[__MaxKprog];
	double dAnklexTemp, dAnkleyTemp, dAnklezTemp;
	if (NotHolding(dAnkxyz)) { // not holding
		if (cFootFlag == 'R') {
			dAnklexTemp = stTraPG.Ankle.W.Rfoot.pos.x[fnnCircuKMap((int)(dTimeStart / __ControlT) - 1)];
			dAnkleyTemp = stTraPG.Ankle.W.Rfoot.pos.y[fnnCircuKMap((int)(dTimeStart / __ControlT) - 1)];
			dAnklezTemp = stTraPG.Ankle.W.Rfoot.pos.z[fnnCircuKMap((int)(dTimeStart / __ControlT) - 1)];
		}
		else if (cFootFlag == 'L') {
			dAnklexTemp = stTraPG.Ankle.W.Lfoot.pos.x[fnnCircuKMap((int)(dTimeStart / __ControlT) - 1)];
			dAnkleyTemp = stTraPG.Ankle.W.Lfoot.pos.y[fnnCircuKMap((int)(dTimeStart / __ControlT) - 1)];
			dAnklezTemp = stTraPG.Ankle.W.Lfoot.pos.z[fnnCircuKMap((int)(dTimeStart / __ControlT) - 1)];
		}
		else printf("Wrong Start Leg Flag in fnvGetAnklePG !!\n");
		fnvFifthSpline(dPGAnklexTemp, __MaxKprog, dAnklexTemp, 0.0, 0.0, dTimeStart, dAnkxyz[0], 0.0, 0.0, dTimeEnd, __ControlT, 'T');
		fnvFifthSpline(dPGAnkleyTemp, __MaxKprog, dAnkleyTemp, 0.0, 0.0, dTimeStart, dAnkxyz[1], 0.0, 0.0, dTimeEnd, __ControlT, 'T');
		fnvDouFourthSpline(dPGAnklezTemp, __MaxKprog, dAnklezTemp, 0.0, 0.0, dTimeStart, dStepHeight + __AnkleHeight, 0.5 * (dTimeStart + dTimeEnd), dAnkxyz[2] + __AnkleHeight, 0.0, 0.0, dTimeEnd, __ControlT, 'T');
		int k = 0;
		for (int i = (int)(dTimeStart / __ControlT); i < (nKprogNow + __MaxKprog - __KLeftToNow); i++) {
			if (cFootFlag == 'R') {
				stTraPG.Ankle.W.Rfoot.pos.x[fnnCircuKMap(i)] = dPGAnklexTemp[k];
				stTraPG.Ankle.W.Rfoot.pos.y[fnnCircuKMap(i)] = dPGAnkleyTemp[k];
				stTraPG.Ankle.W.Rfoot.pos.z[fnnCircuKMap(i)] = dPGAnklezTemp[k];
				if (i < (int)(dTimeEnd / __ControlT)) stTraPG.SupLeg[fnnCircuKMap(i)] = LeftSup;
			}
			else if (cFootFlag == 'L') {
				stTraPG.Ankle.W.Lfoot.pos.x[fnnCircuKMap(i)] = dPGAnklexTemp[k];
				stTraPG.Ankle.W.Lfoot.pos.y[fnnCircuKMap(i)] = dPGAnkleyTemp[k];
				stTraPG.Ankle.W.Lfoot.pos.z[fnnCircuKMap(i)] = dPGAnklezTemp[k];
				if (i < (int)(dTimeEnd / __ControlT)) stTraPG.SupLeg[fnnCircuKMap(i)] = RightSup;
			}
			k++;
		}
	}
}

/** Spline hip height from current hip height, and keep it
InVal:	double dTimeStart, double dTimeEnd, double dHipHeightMaj(the first target hip height), double dHipHeightMin(the second target hip height), int nKprogNow, [stTraPG.Base.pos.z, ]
OutVal: [stTraPG.Base.pos.z, ]
Tips:	1. span of (dTimeEnd - dTimeNow) / __ControlT should be smaller than __MaxKprog
2. when dHipHeightMin == __dVoid, the hip height is splined to dHipHeightMaj only
3. when dHipHeightMaj == __dVoid, the hip height holds on
4. if hip height is not changing during a time span, you don't realy need to use this
Inc:	fnvFifthSpline, fnvDouFourthSpline, fnnCircuKMap,
*/
void fnvGetHipzPG(double dTimeStart, double dTimeEnd, double dHipHeightMaj, double dHipHeightMin, int nKprogNow) {
	double dPGHipzTemp[__MaxKprog];
	if (dHipHeightMaj != __dVoid) {
		if (dHipHeightMin == __dVoid) { // change hip height for one time
			fnvFifthSpline(dPGHipzTemp, __MaxKprog, stTraPG.Base.pos.z[fnnCircuKMap((int)(dTimeStart / __ControlT) - 1)], 0.0, 0.0, dTimeStart, dHipHeightMaj, 0.0, 0.0, dTimeEnd, __ControlT, 'T');
		}
		else { // change hip height for SinSup 
			fnvDouFourthSpline(dPGHipzTemp, __MaxKprog, stTraPG.Base.pos.z[fnnCircuKMap((int)(dTimeStart / __ControlT) - 1)], 0.0, 0.0, dTimeStart, dHipHeightMaj, 0.5 * (dTimeStart + dTimeEnd), dHipHeightMin, 0.0, 0.0, dTimeEnd, __ControlT, 'T');
		}
		int k = 0;
		for (int i = (int)(dTimeStart / __ControlT); i < (nKprogNow + __MaxKprog - __KLeftToNow); i++) {
			stTraPG.Base.pos.z[fnnCircuKMap(i)] = dPGHipzTemp[k];
			k++;
		}
	}
}

double fndGetAnkLateral(char cFootFlag) {
	if (cFootFlag == 'R') return 1.0;
	else if (cFootFlag == 'L') return -1.0;
	else printf("Wrong FootFlag in fndGetAnkLateral !!\n");
}

/** Plan the next step, the ankle trajectory, ZMP trajectory, hip height will be obtained, and the arranged program k will be added according to the time of the next step
InVal:	double dAnkxyz[3](the landing position of the ankle), double dStepHeight(the maxi height of the step), double dStepTime[3](step time, single support percent, pause time), double dHipHeight[3](single phase hip height, double phase hip height, pause hip height), char cFootFlag(swing foot flag, 'R' -> right, 'L' -> left), int nMidStepFlag(not the last step), int nKprogNow, [stTraPG.ZMP.W, stTraPG.Ankle.W, stTraPG.Base.pos.z, nKprogArg]
OutVal: [stTraPG.ZMP.W, stTraPG.Ankle.W, stTraPG.Base.pos.z, nKprogArg, ]
Tips:	1. ATTENTION!! nKprogArg is added in this function, which is also required in this function
2. nKprogNow matters the trajectory update, which should be set as 0 for an offline pattern generation for current program k for an online pattern generation
3. the pause time should be more than 2s, for savety sake
4. it's recommended that cFootFlag should be alterant, the last step length should be zero
Inc:	fnvGetZMPPG, fnvGetAnklePG, fnvGetHipzPG,
*/
void fnvSetNextStep(double dAnkxyz[3], double dStepHeight, double dStepTime[3], double dHipHeight[3], char cFootFlag, int nMidStepFlag, double dZMPWidth, int nKprogNow) {
	double dAnkx /*Next Foot x pos W*/ = dAnkxyz[0], dAnky /*Next Foot y pos W*/ = dAnkxyz[1], dAnkz /*Next Foot z pos W*/ = dAnkxyz[2];
	double dHipSin /*Hip Height for SinSup*/ = dHipHeight[0], dHipDou /*Hip Height for DouSup*/ = dHipHeight[1], dHipPau /*Hip Height for Pause*/ = dHipHeight[2];
	double dStepT /*Next Step time*/ = dStepTime[0], dSinT /*Next SinSup time*/ = dStepTime[0] * dStepTime[1], dPauseT /*Pause time after the DouSup, should be longer than 2 * __THipHeight and __TSwitchZMP*/ = dStepTime[2];
	double dTimeArg = (((double)(nKprogArg + 1)) * __ControlT);
	double dMidStepPos[2];
	// SinSup
	double dZMPxyTemp[2] = { dAnkx + 0.5 * fndGetAnkLateral(cFootFlag) * (dZMPWidth - __AnkleWidth), dAnky };
	fnvGetAnklePG(dTimeArg, dTimeArg + dSinT, dAnkxyz, dStepHeight, cFootFlag, nKprogNow); // ank
	fnvGetHipzPG(dTimeArg, dTimeArg + dSinT, dHipSin, dHipDou, nKprogNow); // hip
																		   // DouSup
	if (dPauseT < 1e-8) { // don't pause
		if (nMidStepFlag == 1) fnvGetZMPPG(dTimeArg + dSinT, dTimeArg + dStepT, dZMPxyTemp, nKprogNow); // zmp
	}
	else { // get pause
		if (cFootFlag == 'R') { // Left sup
			dMidStepPos[0] = 0.5 * (stLastAnkPos.Lfoot.pos.x + dAnkx);
			dMidStepPos[1] = 0.5 * (stLastAnkPos.Lfoot.pos.y + dAnky);
		}
		else if (cFootFlag == 'L') { // Right sup
			dMidStepPos[0] = 0.5 * (stLastAnkPos.Rfoot.pos.x + dAnkx);
			dMidStepPos[1] = 0.5 * (stLastAnkPos.Rfoot.pos.y + dAnky);
		}
		else printf("Wrong Start Leg Flag in fnvSetNextStep !!\n");
		fnvGetZMPPG(dTimeArg + dSinT, dTimeArg + dStepT, dMidStepPos, nKprogNow); // zmp
		fnvGetZMPPG(dTimeArg + dStepT, dTimeArg + dStepT + dPauseT, dMidStepPos, nKprogNow); // Pause zmp
		fnvGetZMPPG(dTimeArg + dStepT + dPauseT, dTimeArg + dStepT + dPauseT + __TSwitchZMP, dZMPxyTemp, nKprogNow); // SwitchZMP zmp
		fnvGetHipzPG(dTimeArg + dStepT, dTimeArg + dStepT + __THipHeight, dHipPau, __dVoid, nKprogNow); // Pause hip
		fnvGetHipzPG(dTimeArg + dStepT + dPauseT - __THipHeight, dTimeArg + dStepT + dPauseT, dHipDou, __dVoid, nKprogNow); // Pause hip
	}
	nKprogArg += (int)((dStepT + dPauseT) / __ControlT);
}

/** Switching ZMP, changing hip height, freezing posture in double support phase or standstill
InVal:	char cSwiftToWhere('L' -> to left ankle, 'R' -> to right ankle, 'M' -> to the midle of two ankle), double dHipToHeight(the target hip height from current height), double dFreezeTime(if freeze, the time span), int nKprogNow, [stTraPG.ZMP.W, stTraPG.Base.pos.z, nKprogArg]
OutVal: [stTraPG.ZMP.W, stTraPG.Base.pos.z, nKprogArg, ]
Tips:	1. ATTENTION!! nKprogArg is added in this function, which is also required in this function
2. nKprogNow matters the trajectory update, which should be set as 0 for an offline pattern generation for current program k for an online pattern generation
3. when dFreezeTime == __dVoid, switch ZMP and change hip height, if dHipToHeight == _dVoid, the hip height remains unchanged
4. when cSwiftToWhere == __cVoid and dHipToHeight == __dVoid, the robot freeze for a time span of dFreezeTime
Inc:	fnvGetZMPPG, fnvGetHipzPG,
*/
void fnvSwitchZMP(char cSwiftToWhere, double dHipToHeight, double dFreezeTime, double dZMPWidth, int nKprogNow) {
	double dTimeArg = ((double)nKprogArg * __ControlT);
	if (dFreezeTime == __dVoid) { // using switch
		if (cSwiftToWhere == 'R') { // switch zmp to right ankle
			double dAnkPosTemp[2] = { stTraPG.Ankle.W.Rfoot.pos.x[fnnCircuKMap(nKprogArg)] + 0.5 * (dZMPWidth - __AnkleWidth), stTraPG.Ankle.W.Rfoot.pos.y[fnnCircuKMap(nKprogArg)] };
			fnvGetZMPPG(dTimeArg, dTimeArg + __TSwitchZMP, dAnkPosTemp, nKprogNow); // SwitchZMP zmp
		}
		else if (cSwiftToWhere == 'L') { // switch zmp to left ankle
			double dAnkPosTemp[2] = { stTraPG.Ankle.W.Lfoot.pos.x[fnnCircuKMap(nKprogArg)] - 0.5 * (dZMPWidth - __AnkleWidth), stTraPG.Ankle.W.Lfoot.pos.y[fnnCircuKMap(nKprogArg)] };
			fnvGetZMPPG(dTimeArg, dTimeArg + __TSwitchZMP, dAnkPosTemp, nKprogNow); // SwitchZMP zmp
		}
		else if (cSwiftToWhere == 'M') { // switch zmp to midle of two ankle
			double dAnkPosTemp[2] = { 0.5 * (stTraPG.Ankle.W.Rfoot.pos.x[fnnCircuKMap(nKprogArg)] + stTraPG.Ankle.W.Lfoot.pos.x[fnnCircuKMap(nKprogArg)]), 0.5 * (stTraPG.Ankle.W.Rfoot.pos.y[fnnCircuKMap(nKprogArg)] + stTraPG.Ankle.W.Lfoot.pos.y[fnnCircuKMap(nKprogArg)]) };
			fnvGetZMPPG(dTimeArg, dTimeArg + __TSwitchZMP, dAnkPosTemp, nKprogNow); // SwitchZMP zmp
		}
		else if (cSwiftToWhere == __cVoid);
		else printf("Wrong Switch Flag in fnvSwiftZMP !!\n");
		fnvGetHipzPG(dTimeArg, dTimeArg + __THipHeight, dHipToHeight, __dVoid, nKprogNow);
		nKprogArg += (int)((MaxOf(__TSwitchZMP, __THipHeight)) / __ControlT);
	}
	else { // using pause
		nKprogArg += (int)(dFreezeTime / __ControlT);
	}
}

/**
InVal:	[stTraPG.ZMP.W, nKprogNow, ]
OutVal: []
Tips:	1.
Inc:	fnnCircuKMap, fnvPGMPCCalConval,
*/
void fnvCalCoM() {
	for (int i = 0; i < nNumPre; i++) { // obtain ZMP reference in 500 ConCir
		dVeZmpRefx_500x1[i][0] = stTraPG.ZMP.W.x[fnnCircuKMap(nKprogNow + i)];
		dVeZmpRefy_500x1[i][0] = stTraPG.ZMP.W.y[fnnCircuKMap(nKprogNow + i)];
	}
	fnvPGMPCCalConval(dVeZmpRefx_500x1, dVeZmpRefy_500x1, dVeStatex_3x1, dVeStatey_3x1); // cal jerk ConVal with MPC
	double dLimits[6] = { 0.0, 0.0, -10.0, 10.0, -100.0, 100.0 }; // limitations
	fnvJerkLimit(dVeStatex_3x1, dVeStatex_3x1 + 1, dVeStatex_3x1 + 2, dPGMPCConval[0], dLimits, __ControlT); // uptate the state in direction x
	fnvJerkLimit(dVeStatey_3x1, dVeStatey_3x1 + 1, dVeStatey_3x1 + 2, dPGMPCConval[1], dLimits, __ControlT); // uptate the state in direction y
	for (int i = 0; i < 3; i++) { // pass the state
		stTraPG.Base.pos.x[fnnCircuKMap(nKprogNow)] = dVeStatex_3x1[0][0];
		stTraPG.Base.pos.dx[fnnCircuKMap(nKprogNow)] = dVeStatex_3x1[1][0];
		stTraPG.Base.pos.ddx[fnnCircuKMap(nKprogNow)] = dVeStatex_3x1[2][0];
		stTraPG.Base.pos.y[fnnCircuKMap(nKprogNow)] = dVeStatey_3x1[0][0];
		stTraPG.Base.pos.dy[fnnCircuKMap(nKprogNow)] = dVeStatey_3x1[1][0];
		stTraPG.Base.pos.ddy[fnnCircuKMap(nKprogNow)] = dVeStatey_3x1[2][0];
	}
}

// PlanPG method ======================================================================================================
/** Init stPlanCmd for PlanPG
InVal:	upto -> "using PlanPG here"
OutVal: [stPlanCmd, ]
*/
void fnvInitPlanPG() {
	double dStepLengthTemp[] = { __StepLength };
	double dDataTemp[][__MaxPlanStepN] = { { __StepLength },{ __StepWidth },{ __StepHeight },{ __LandHeight },{ __StepTime },{ __SinPercent },{ __PauseTime },{ __HipHeightSin },{ __HipHeightDou },{ __HipHeightPau } };
	stPlanCmd.nStepNum = sizeof(dStepLengthTemp) / sizeof(dStepLengthTemp[0]);
	stPlanCmd.cStartSwiLeg = __StartLegName;
	if (stPlanCmd.cStartSwiLeg == 'R') stPlanCmd.cStartSupLeg = 'L';
	else if (stPlanCmd.cStartSwiLeg == 'L') stPlanCmd.cStartSupLeg = 'R';
	else printf("Wrong Start Leg Flag in fnvInitPlanPG !!\n");
	stPlanCmd.dTimeReady = __TimeReady;
	stPlanCmd.dTimeEnd = __TimeEnd;
	stPlanCmd.dStepForward = 0.0;
	stPlanCmd.dStepBackward = 0.0;
	stPlanCmd.dStepRightside = 0.5 * __AnkleWidth;
	stPlanCmd.dStepLeftside = -0.5 * __AnkleWidth;
	for (int j = 0; j < 10; j++) {
		for (int i = 0; i < stPlanCmd.nStepNum; i++) {
			*(stPlanCmd.dStepLength + i + j * __MaxPlanStepN) = dDataTemp[j][i];
		}
	}
	for (int i = 0; i < stPlanCmd.nStepNum; i++) {
		if (stPlanCmd.cStartSwiLeg == 'R') { // right start to step
			stPlanCmd.nSupLeg[i] = (int)(-(i % 2) + 2); // left sup firstly
		}
		else if (stPlanCmd.cStartSwiLeg == 'L') { // left start to step
			stPlanCmd.nSupLeg[i] = (int)((i % 2) + 1); // right sup firstly
		}
		else printf("Wrong Start Leg Flag in fnvInitPlanPG !!\n");
	}
}

/** Get stTraPG for planPG, this is an offline pattern generation that can be used at any program k(nKprogNow)
InVal:	[stPlanCmd, stTraPG, nKprogNow, nKprogArg, ]
OutVal: [stTraPG, nKprogArg, ]
Tips:	1. Calculate ZMP, ankle trajectory, hip height trajectory once in the begining of the program(init)
Inc:	fnvSwitchZMP, fnvSetNextStep,
*/
void fnvGetPlanPG() {
	fnvInitPlanPG(); // init stPlanCmd
	char cFootFlag;
	double dAnkTemp[3]/*{ x, y, z }*/, dStepHeightTemp/*z*/, dStepTime[3]/*{ StepT, SinSupPercent, PauseT }*/, dHipHeightTemp[3]/*{ HipzSinSup, HipzDouSup, HipzPause }*/;
	fnvSwitchZMP(__cVoid, __dVoid, __TimeReady, __ZMPWidth, nKprogNow); // ready
	fnvSwitchZMP(stPlanCmd.cStartSupLeg, stPlanCmd.dHipHeightDou[0], __dVoid, __ZMPWidth, nKprogNow); // switch zmp to the start leg and squate
	for (int nStep = 0; nStep < stPlanCmd.nStepNum; nStep++) { // cirle steps
		if (stPlanCmd.nSupLeg[nStep] == 1) { // right foot support
			cFootFlag = 'L'; //  left foot swing
			dAnkTemp[0] = stPlanCmd.dStepLeftside + stPlanCmd.dStepWidth[nStep]; // step position lateral
			stPlanCmd.dStepLeftside = dAnkTemp[0]; // update dStepLeftside
		}
		else if (stPlanCmd.nSupLeg[nStep] == 2) { // left foot support
			cFootFlag = 'R'; // right foot swing
			dAnkTemp[0] = stPlanCmd.dStepRightside + stPlanCmd.dStepWidth[nStep]; // step position lateral
			stPlanCmd.dStepRightside = dAnkTemp[0]; // update dStepRightside
		}
		else printf("Wrong nSupLeg in fnvGetPlanPG !!\n");
		dAnkTemp[1] = stPlanCmd.dStepForward + stPlanCmd.dStepLength[nStep]; // step position saggital
		stPlanCmd.dStepBackward = stPlanCmd.dStepForward; // update dStepForward
		stPlanCmd.dStepForward = dAnkTemp[1]; // update dStepForward
		dAnkTemp[2] = stPlanCmd.dLandHeight[nStep]; // step height of landing
		dStepHeightTemp = stPlanCmd.dStepHeight[nStep]; // step maxi height
		for (int i = 0; i < 3; i++) {
			dStepTime[i] = *(stPlanCmd.dStepTime + nStep + i * __MaxPlanStepN); // step time, SinSup time percent, Pause time
			dHipHeightTemp[i] = *(stPlanCmd.dHipHeightSin + nStep + i * __MaxPlanStepN); // hip height SinSup, hip height DouSup, hip height Pause
		}
		fnvSetNextStep(dAnkTemp, dStepHeightTemp, dStepTime, dHipHeightTemp, cFootFlag, (nStep < stPlanCmd.nStepNum - 1), __ZMPWidth, nKprogNow);
	}
	fnvSwitchZMP('M', __HipHeightHome, __dVoid, __ZMPWidth, nKprogNow); // switch zmp to the midle and squateup
	fnvSwitchZMP(__cVoid, __dVoid, __TimeEnd, __ZMPWidth, nKprogNow);  // end
}

/** Recording stTraPG generated by planPG method
InVal:	[stTraPG, ]
Inc:	fnnCircuKMap,
Create:	DccPlanPG.dat
*/
void fnvRecordPG() {
	if (RePGFileOpenFlag == 0) { // firstly call, open the file
		RePGFileOpenFlag = 1;
		if ((FptRePG = fopen("DccPlanPG.dat", "w")) == NULL) {
			printf("Failed to Open DccPlanPG.dat in fnvRecordPG !!\n");
			RePGFileOpenFlag == 0;
		}
	}
	if (RePGFileOpenFlag == 1) {
		int nKprogCirTemp = fnnCircuKMap(nKprogNow);
		fprintf(FptRePG, "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n", stTraPG.ZMP.W.x[fnnCircuKMap(nKprogCirTemp)], stTraPG.ZMP.W.y[fnnCircuKMap(nKprogCirTemp)], stTraPG.Base.pos.x[fnnCircuKMap(nKprogCirTemp)], stTraPG.Base.pos.y[fnnCircuKMap(nKprogCirTemp)], stTraPG.Base.pos.z[fnnCircuKMap(nKprogCirTemp)], stTraPG.Ankle.W.Rfoot.pos.x[fnnCircuKMap(nKprogCirTemp)], stTraPG.Ankle.W.Rfoot.pos.y[fnnCircuKMap(nKprogCirTemp)], stTraPG.Ankle.W.Rfoot.pos.z[fnnCircuKMap(nKprogCirTemp)], stTraPG.Ankle.W.Lfoot.pos.x[fnnCircuKMap(nKprogCirTemp)], stTraPG.Ankle.W.Lfoot.pos.y[fnnCircuKMap(nKprogCirTemp)], stTraPG.Ankle.W.Lfoot.pos.z[fnnCircuKMap(nKprogCirTemp)]);
		if (nKprogNow >= nKprogArg) { // close the file
			RePGFileOpenFlag = 2;
			fclose(FptRePG);
		}
	}
}
// Init of the whole PGframe
void fnvInitPG() {
	nKprogNow = 0;
	nKprogArg = 0;
	stTraPG.WalkFlag = 0;
	fnvInitPlanPGTra();
#ifdef USE_FREEWALK_METHOD
	fnvFreeWalkInit();	// FreeWalk
#endif
}

// 
void fnvDccGetPG() {
	fnvInitPG();
#ifdef USE_PLANPG_METHOD
	fnvGetPlanPG(); // Plan PG method
#endif
	printf("DCC PG Finished !!\n");
}

//
void fnvExecuteTra() {
	Tra_RAnkle.x[nKprogNow] = stTraPG.Ankle.W.Rfoot.pos.x[nKprogNow];
	Tra_RAnkle.y[nKprogNow] = stTraPG.Ankle.W.Rfoot.pos.y[nKprogNow];
	Tra_RAnkle.z[nKprogNow] = stTraPG.Ankle.W.Rfoot.pos.z[nKprogNow];
	Tra_LAnkle.x[nKprogNow] = stTraPG.Ankle.W.Lfoot.pos.x[nKprogNow];
	Tra_LAnkle.y[nKprogNow] = stTraPG.Ankle.W.Lfoot.pos.y[nKprogNow];
	Tra_LAnkle.z[nKprogNow] = stTraPG.Ankle.W.Lfoot.pos.z[nKprogNow];
	Tra_COM.x[nKprogNow] = stTraPG.Base.pos.x[nKprogNow];
	Tra_COM.y[nKprogNow] = stTraPG.Base.pos.y[nKprogNow];
	Tra_COM.z[nKprogNow] = stTraPG.Base.pos.z[nKprogNow];
	Tra_ZMP.x[nKprogNow] = stTraPG.ZMP.W.x[nKprogNow] * __AnkleWidth / __ZMPWidth;
	Tra_ZMP.y[nKprogNow] = stTraPG.ZMP.W.y[nKprogNow];
	Signal_SupportLeg[nKprogNow] = stTraPG.SupLeg[nKprogNow];
#ifdef USE_PLANPG_METHOD
	if (nKprogNow < nKprogArg) stTraPG.WalkFlag = 1;
	else stTraPG.WalkFlag = 0, printf("PlanPG Over !!\n");
#endif
}

//
void fnvDccPGUpdate(int nKpre) {
	nKprogNow = nKpre; // update nKprogNow
	// online PG methods *********************************
#ifdef USE_FREEWALK_METHOD
	fnvDccFreeWalk(1, 1, 1); // cmd, stepT, stepL
#endif
	// ***************************************************
	fnvCalCoM(); // cal CoM
	fnvExecuteTra(); // execute the trajectory
	fnvRecordPG(); // recording the trajectory
}



#endif