// dcc 20210524 bit
// for a real time free walk controlled by key board
#ifndef DCC_FREEWALK_C
#define DCC_FREEWALK_C
#include <conio.h>
#include "DCC_FreeWalk.h"
#include "DCC_PGFrame.h"
#include "..\Base\dcc_get_key.h"
#include "..\Base\dcc_con_base.h"

#define __StartLegName			'L'
#define __InitalStepTime		0.6
#define __StepTimeGap			0.02
#define __StepTimeLimit			{ 0.35, 1.0 }
#define __SingleSupPer			0.8
#define __InitalStepLength		0.15 
#define __StepLengthGap			0.05
#define __StepLengthLimit		{ 0.05, 0.35 }
#define __InitalStepWidth		0.05 
#define __StepWidthLimit		{ 0.05, 0.35 }
#define __StepHeight			0.04
#define __InitalHipHeight		0.68
#define __HipHeightLimit		{ 0.69, 0.6 }
#define __ZMPBiasGap			0.001

double dStepTimeAdj;
double dStepLengthAdj;
double dStepTimeLimit[2] = __StepTimeLimit;
double dStepLengthLimit[2] = __StepLengthLimit;
double dStepWidthLimit[2] = __StepWidthLimit;

int nCurrentMotion;
enum locomotioncmd {
	StandStill, March, Walk
};
char *cptLocomotionName[] = { "StandStill",  "March", "Walk" };
enum locodirectioncmd {
	None, Forward, Backward, Leftside, Rightside, FowRight, FowLeft, BakRight, BakLeft
};
char *cptLocoDirectionName[] = { "", "Forward",  "Backward", "Leftside", "Rightside", "FowRight", "FowLeft", "BakRight", "BakLeft" };
enum controlcmd {
	NoCmd, ZmpThiner, ZmpFatter
};
char *cptZmpName[] = { "",  "ZmpThiner", "ZmpFatter" };
enum speedcmd {
	SpeedKeep, SpeedUp, SpeedDown
};
char *cptLocoSpeedName[] = { "",  "SpeedUp", "SpeedDown" };
enum steplencmd {
	StepKeep, StepLonger, StepShorter
};
char *cptLocoStepName[] = { "",  "StepLonger", "StepShorter" };

void fnvFreeWalkInit() {
	stFreeWalk.dStepTime = __InitalStepTime;
	stFreeWalk.dStepLength = 0.0;
	stFreeWalk.dStepwidth = 0.0;
	stFreeWalk.dStepForward = 0.0;
	stFreeWalk.dStepBackward = 0.0;
	stFreeWalk.dStepLeftside = -0.5 * __AnkleWidth;
	stFreeWalk.dStepRightside = 0.5 * __AnkleWidth;
	stFreeWalk.dZMPWidthBias = 0.0;
	stFreeWalk.cFootFlag = __StartLegName;
	stFreeWalk.dYawAngle = 0.0;
	stFreeWalk.nLocoCmd[0] = StandStill;
	stFreeWalk.nLocoCmd[1] = StandStill;
	stFreeWalk.nListen = 1;
	dStepTimeAdj = 0.0;
	dStepLengthAdj = 0.0;
	nCurrentMotion = StandStill;
}

void fnvGetMotionCmd(int nCmdDispFlag) {
	// locomotion
	if (stFreeWalk.nLocoCmd[1] == StandStill && nDicInPressFlag[Key_R]) {
		stFreeWalk.nLocoCmd[0] = March; // r input
		stFreeWalk.nLocoDirection[0] = None;
	}
	else if (stFreeWalk.nLocoCmd[1] == March && nDicInPressFlag[Key_R]) {
		stFreeWalk.nLocoCmd[0] = StandStill; // r input
		stFreeWalk.nLocoDirection[0] = None;
	}
	else if ((stFreeWalk.nLocoCmd[1] == March || stFreeWalk.nLocoCmd[1] == Walk) && (nSusInPressFlag[Key_A] || nSusInPressFlag[Key_S] || nSusInPressFlag[Key_D] || nSusInPressFlag[Key_W])) { // a, s, d, w input
		if (!nSusInPressFlag[Key_A] && !nSusInPressFlag[Key_S] && !nSusInPressFlag[Key_D] && nSusInPressFlag[Key_W]) { // forward
			stFreeWalk.nLocoCmd[0] = Walk;
			stFreeWalk.nLocoDirection[0] = Forward;
		}
		else if (!nSusInPressFlag[Key_A] && nSusInPressFlag[Key_S] && !nSusInPressFlag[Key_D] && !nSusInPressFlag[Key_W]) { // backward
			stFreeWalk.nLocoCmd[0] = Walk;
			stFreeWalk.nLocoDirection[0] = Backward;
		}
		else if (nSusInPressFlag[Key_A] && !nSusInPressFlag[Key_S] && !nSusInPressFlag[Key_D] && !nSusInPressFlag[Key_W]) { // leftside
			stFreeWalk.nLocoCmd[0] = Walk;
			stFreeWalk.nLocoDirection[0] = Leftside;
		}
		else if (!nSusInPressFlag[Key_A] && !nSusInPressFlag[Key_S] && nSusInPressFlag[Key_D] && !nSusInPressFlag[Key_W]) { // rightside
			stFreeWalk.nLocoCmd[0] = Walk;
			stFreeWalk.nLocoDirection[0] = Rightside;
		}
		else if (nSusInPressFlag[Key_A] && !nSusInPressFlag[Key_S] && !nSusInPressFlag[Key_D] && nSusInPressFlag[Key_W]) { // fowleft
			stFreeWalk.nLocoCmd[0] = Walk;
			stFreeWalk.nLocoDirection[0] = FowLeft;
		}
		else if (!nSusInPressFlag[Key_A] && !nSusInPressFlag[Key_S] && nSusInPressFlag[Key_D] && nSusInPressFlag[Key_W]) { // fowright
			stFreeWalk.nLocoCmd[0] = Walk;
			stFreeWalk.nLocoDirection[0] = FowRight;
		}
		else if (nSusInPressFlag[Key_A] && nSusInPressFlag[Key_S] && !nSusInPressFlag[Key_D] && !nSusInPressFlag[Key_W]) { // bakleft
			stFreeWalk.nLocoCmd[0] = Walk;
			stFreeWalk.nLocoDirection[0] = BakLeft;
		}
		else if (!nSusInPressFlag[Key_A] && nSusInPressFlag[Key_S] && nSusInPressFlag[Key_D] && !nSusInPressFlag[Key_W]) { // bakleft
			stFreeWalk.nLocoCmd[0] = Walk;
			stFreeWalk.nLocoDirection[0] = BakRight;
		}
		else {
			stFreeWalk.nLocoCmd[0] = March;
			stFreeWalk.nLocoDirection[0] = None;
		}
	}
	else { // no keys input
		if (stFreeWalk.nLocoCmd[1] == March || stFreeWalk.nLocoCmd[1] == Walk) {
			stFreeWalk.nLocoCmd[0] = March;
			stFreeWalk.nLocoDirection[0] = None;
		}
		if (stFreeWalk.nLocoCmd[1] == StandStill) {
			stFreeWalk.nLocoCmd[1] == StandStill;
			stFreeWalk.nLocoDirection[0] = None;
		}
	}
	// speed control & zmp control 
	if (stFreeWalk.nLocoCmd[1] == Walk || stFreeWalk.nLocoCmd[1] == March) { // when walking or march
		if (nDicInPressFlag[Key_J]) stFreeWalk.nSpeedCmd[0] = SpeedDown; // speed down
		else if (nDicInPressFlag[Key_U]) stFreeWalk.nSpeedCmd[0] = SpeedUp; // speed up
		else stFreeWalk.nSpeedCmd[0] = SpeedKeep; // no speed control key input
		if (nDicInPressFlag[Key_K]) stFreeWalk.nStepLenCmd[0] = StepShorter; // step shorter
		else if (nDicInPressFlag[Key_I]) stFreeWalk.nStepLenCmd[0] = StepLonger; // step longer
		else stFreeWalk.nStepLenCmd[0] = StepKeep; // no step control key input
		if (nDicInPressFlag[Key_Z]) stFreeWalk.nZmpCmd[0] = ZmpThiner; // zmp thiner
		else if (nDicInPressFlag[Key_X]) stFreeWalk.nZmpCmd[0] = ZmpFatter; // zmp wider
		else stFreeWalk.nZmpCmd[0] = NoCmd; // no zmp control key input
	}
	// disp
	if (nCmdDispFlag == 1) {
		if (stFreeWalk.nLocoCmd[0] != stFreeWalk.nLocoCmd[1]) printf("============================= %s ===============================\n", cptLocomotionName[stFreeWalk.nLocoCmd[0]]);
		if (stFreeWalk.nLocoDirection[0] != stFreeWalk.nLocoDirection[1]) printf("Direction: %s\n ", cptLocoDirectionName[stFreeWalk.nLocoDirection[0]]);
		if (stFreeWalk.nSpeedCmd[0] != SpeedKeep) printf("%s\n ", cptLocoSpeedName[stFreeWalk.nSpeedCmd[0]]);
		if (stFreeWalk.nStepLenCmd[0] != StepKeep) printf("%s\n ", cptLocoStepName[stFreeWalk.nStepLenCmd[0]]);
		if (stFreeWalk.nZmpCmd[0] != NoCmd) printf("%s -> %lf\n ", cptZmpName[stFreeWalk.nZmpCmd[0]], __ZMPWidth + stFreeWalk.dZMPWidthBias);
	}
	// update 
	stFreeWalk.nLocoCmd[1] = stFreeWalk.nLocoCmd[0];
	stFreeWalk.nLocoDirection[1] = stFreeWalk.nLocoDirection[0];
	stFreeWalk.nCtrlCmd[1] = stFreeWalk.nCtrlCmd[0];
	stFreeWalk.nSpeedCmd[1] = stFreeWalk.nSpeedCmd[0];
	stFreeWalk.nStepLenCmd[1] = stFreeWalk.nStepLenCmd[0];
	stFreeWalk.nZmpCmd[1] = stFreeWalk.nZmpCmd[0];
}

void fnvFreeWalkSpeedAdj() {
	if (stFreeWalk.nSpeedCmd[0] == SpeedUp) dStepTimeAdj -= __StepTimeGap;
	else if (stFreeWalk.nSpeedCmd[0] == SpeedDown) dStepTimeAdj += __StepTimeGap;
}

void fnvFreeWalkStepAdj() {
	if (stFreeWalk.nStepLenCmd[0] == StepLonger) dStepLengthAdj += __StepLengthGap;
	else if (stFreeWalk.nStepLenCmd[0] == StepShorter) dStepLengthAdj -= __StepLengthGap;
}

void fnvFreeWalkZmpAdj() {
	if (stFreeWalk.nZmpCmd[0] == ZmpFatter) stFreeWalk.dZMPWidthBias += __ZMPBiasGap;
	else if (stFreeWalk.nZmpCmd[0] == ZmpThiner) stFreeWalk.dZMPWidthBias -= __ZMPBiasGap;
}

void fnvGetFreeWalkParas() {
	if (stFreeWalk.nLocoCmd[0] == StandStill) {
		dStepTimeAdj = 0.0;
		dStepLengthAdj = 0.0;
	}
	else if (stFreeWalk.nLocoCmd[0] == March) {
		stFreeWalk.dStepTime = fndLimit(__InitalStepTime + dStepTimeAdj, dStepTimeLimit);
		stFreeWalk.dStepLength = 0.0;
		stFreeWalk.dStepwidth = 0.0;
		stFreeWalk.dStepwidth = -fndGetAnkLateral(stFreeWalk.cFootFlag) * (stFreeWalk.dStepRightside - stFreeWalk.dStepLeftside - __AnkleWidth); // follow with the step after moving side
	}
	else if (stFreeWalk.nLocoCmd[0] == Walk) {
		stFreeWalk.dStepTime = fndLimit(__InitalStepTime + dStepTimeAdj, dStepTimeLimit);
		stFreeWalk.dStepLength = 0.0;
		stFreeWalk.dStepwidth = 0.0;
		if ((stFreeWalk.nLocoDirection[0] == Forward) || (stFreeWalk.nLocoDirection[0] == FowLeft) || (stFreeWalk.nLocoDirection[0] == FowRight)) {
			stFreeWalk.dStepLength = fndLimit(__InitalStepLength + dStepLengthAdj, dStepLengthLimit);
			stFreeWalk.dStepwidth = -fndGetAnkLateral(stFreeWalk.cFootFlag) * (stFreeWalk.dStepRightside - stFreeWalk.dStepLeftside - __AnkleWidth); // follow with the step after moving side
		}
		if ((stFreeWalk.nLocoDirection[0] == Backward) || (stFreeWalk.nLocoDirection[0] == BakLeft) || (stFreeWalk.nLocoDirection[0] == BakRight)) {
			stFreeWalk.dStepLength = -fndLimit(__InitalStepLength + dStepLengthAdj, dStepLengthLimit);
			stFreeWalk.dStepwidth = -fndGetAnkLateral(stFreeWalk.cFootFlag) * (stFreeWalk.dStepRightside - stFreeWalk.dStepLeftside - __AnkleWidth); // follow with the step after moving side
		}
		if ((stFreeWalk.nLocoDirection[0] == Leftside)  || (stFreeWalk.nLocoDirection[0] == BakLeft)  || (stFreeWalk.nLocoDirection[0] == FowLeft))  stFreeWalk.dStepwidth = -fndLimit(__InitalStepWidth + dStepLengthAdj, dStepWidthLimit);
		if ((stFreeWalk.nLocoDirection[0] == Rightside) || (stFreeWalk.nLocoDirection[0] == BakRight) || (stFreeWalk.nLocoDirection[0] == FowRight)) stFreeWalk.dStepwidth = fndLimit(__InitalStepWidth + dStepLengthAdj, dStepWidthLimit);
		if (stFreeWalk.cFootFlag == 'R') if ((stFreeWalk.dStepRightside + stFreeWalk.dStepwidth - stFreeWalk.dStepLeftside) < __AnkleWidth) stFreeWalk.dStepwidth = __AnkleWidth + stFreeWalk.dStepLeftside - stFreeWalk.dStepRightside;
		if (stFreeWalk.cFootFlag == 'L') if ((stFreeWalk.dStepRightside - stFreeWalk.dStepLeftside - stFreeWalk.dStepwidth) < __AnkleWidth) stFreeWalk.dStepwidth = -__AnkleWidth + stFreeWalk.dStepRightside - stFreeWalk.dStepLeftside;
	}
	else printf("Wrong Locomotion flag in fnvFreeWalkPG_OnLine !!\n");
}

void fnvCheckListen(int nDipsFlag) {
	if (nKprogNow > nKprogArg) stFreeWalk.nListen = 1;
	else stFreeWalk.nListen = 0;
	if (nDipsFlag) printf("ListenFlag: %d,\tnKprogNow: %d,\tnKprogArg: %d\n", stFreeWalk.nListen, nKprogNow, nKprogArg);
}

void fnvActivateMotion() {
	nKprogArg = nKprogNow;
}

char fncOpsiteFoot(char cFootFlag) {
	if (cFootFlag == 'R') return 'L';
	else if (cFootFlag == 'L') return 'R';
	else printf("Wrong FootFlag in fncOpsiteFoot !!\n");
}

void fnvUpdateFootFlag(double dAnkxyz[3]) {
	if (stFreeWalk.cFootFlag == 'L') {
		stFreeWalk.cFootFlag = 'R';
		stFreeWalk.dStepLeftside = dAnkxyz[0];
	}
	else if (stFreeWalk.cFootFlag == 'R') {
		stFreeWalk.cFootFlag = 'L';
		stFreeWalk.dStepRightside = dAnkxyz[0];
	}
	else printf("Wrong footflag in fnvFreeWalkPG_OnLine !!\n");
	stFreeWalk.dStepBackward = stFreeWalk.dStepForward;
	stFreeWalk.dStepForward = dAnkxyz[1];
}

void fnvFreeWalkPG_OnLine(int nSpeedDispFlag, int nStepDispFlag, int nKprogNow) {
	double dAnkxyzTemp[3], dStepHeightTemp, dStepTimeTemp[3], dHipHeightTemp[3];
	int nMidStepFlagTemp;
	fnvCheckListen(0);
	dAnkxyzTemp[2] = 0.0;
	dStepHeightTemp = __StepHeight;
	dStepTimeTemp[0] = stFreeWalk.dStepTime;
	dStepTimeTemp[1] = __SingleSupPer;
	dStepTimeTemp[2] = 0.0;
	dHipHeightTemp[0] = __InitalHipHeight;
	dHipHeightTemp[1] = __InitalHipHeight;
	dHipHeightTemp[2] = __InitalHipHeight;
	dAnkxyzTemp[1] = stFreeWalk.dStepForward;
	if (stFreeWalk.cFootFlag == 'R') dAnkxyzTemp[0] = stFreeWalk.dStepRightside;
	else if(stFreeWalk.cFootFlag == 'L') dAnkxyzTemp[0] = stFreeWalk.dStepLeftside;
	else printf("Wrong FootFlag in fnvFreeWalkPG_OnLine !!\n");
	if (stFreeWalk.nListen == 1) {
		if ((nCurrentMotion == StandStill) && (stFreeWalk.nLocoCmd[0] == March)) { // first step
			fnvActivateMotion();
			nCurrentMotion = March;
			fnvSwitchZMP(fncOpsiteFoot(stFreeWalk.cFootFlag), __InitalHipHeight, __dVoid, __ZMPWidth + stFreeWalk.dZMPWidthBias, nKprogNow);
			fnvSetNextStep(dAnkxyzTemp, dStepHeightTemp, dStepTimeTemp, dHipHeightTemp, stFreeWalk.cFootFlag, 1, __ZMPWidth + stFreeWalk.dZMPWidthBias, nKprogNow);
			if (nSpeedDispFlag) printf("%lf s\t", dStepTimeTemp[0]);
			if (nStepDispFlag) printf("%lf m, %lf m\t", stFreeWalk.dStepLength, 0.0);
			if (nSpeedDispFlag || nStepDispFlag) printf("%c\n", stFreeWalk.cFootFlag);
			fnvUpdateFootFlag(dAnkxyzTemp);
		}
		else if ((nCurrentMotion == March || nCurrentMotion == Walk) && (stFreeWalk.nLocoCmd[0] == March || stFreeWalk.nLocoCmd[0] == Walk)) { // mid step
			fnvActivateMotion();
			nCurrentMotion = stFreeWalk.nLocoCmd[0];
			dAnkxyzTemp[0] += stFreeWalk.dStepwidth;
			dAnkxyzTemp[1] += stFreeWalk.dStepLength;
			fnvSetNextStep(dAnkxyzTemp, dStepHeightTemp, dStepTimeTemp, dHipHeightTemp, stFreeWalk.cFootFlag, 1, __ZMPWidth + stFreeWalk.dZMPWidthBias, nKprogNow);
			if (nSpeedDispFlag) printf("%lf s\t", dStepTimeTemp[0]);
			if (nStepDispFlag) printf("%lf m, %lf m\t", stFreeWalk.dStepLength, stFreeWalk.dStepwidth);
			if (nSpeedDispFlag || nStepDispFlag) printf("%c\n", stFreeWalk.cFootFlag);
			fnvUpdateFootFlag(dAnkxyzTemp);
		}
		else if ((nCurrentMotion == March) && (stFreeWalk.nLocoCmd[0] == StandStill)) {
			fnvActivateMotion();
			nCurrentMotion = StandStill;
			fnvSetNextStep(dAnkxyzTemp, dStepHeightTemp, dStepTimeTemp, dHipHeightTemp, stFreeWalk.cFootFlag, 0, __ZMPWidth + stFreeWalk.dZMPWidthBias, nKprogNow);
			fnvSwitchZMP('M', __InitalHipHeight, __dVoid, __ZMPWidth + stFreeWalk.dZMPWidthBias, nKprogNow);
			if (nSpeedDispFlag) printf("%lf s\t", dStepTimeTemp[0]);
			if (nStepDispFlag) printf("%lf m, %lf m\t", stFreeWalk.dStepLength, stFreeWalk.dStepwidth);
			if (nSpeedDispFlag || nStepDispFlag) printf("%c\n", stFreeWalk.cFootFlag);
			fnvUpdateFootFlag(dAnkxyzTemp);
		}
	}
}

void fnvDccFreeWalk(int nCmdDispFlag, int nSpeedDispFlag, int nStepDispFlag) {
	fnvGetMotionCmd(nCmdDispFlag);
	fnvFreeWalkSpeedAdj();
	fnvFreeWalkStepAdj();
	fnvFreeWalkZmpAdj();
	fnvGetFreeWalkParas();
	fnvFreeWalkPG_OnLine(nSpeedDispFlag, nStepDispFlag, nKprogNow);
}

#endif