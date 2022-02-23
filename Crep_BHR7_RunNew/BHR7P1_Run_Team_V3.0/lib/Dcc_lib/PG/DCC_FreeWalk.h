// dcc 20210524 bit
#ifndef DCC_FREEWALK_H
#define DCC_FREEWALK_H
#ifdef DCC_FREEWALK_C
#define Extern
#else
#define Extern extern
#endif

typedef struct {
	double dStepLength;
	double dStepwidth;
	double dYawAngle;
	double dStepTime;
	double dStepForward;
	double dStepBackward;
	double dStepLeftside;
	double dStepRightside;
	double dZMPWidthBias;
	char cFootFlag;
	int nLocoCmd[2]; // cmd flags for locomotion mode
	int nLocoDirection[2]; // flags for locomotion direction
	int nCtrlCmd[2]; // cmd flags for controls
	int nSpeedCmd[2]; // cmd flags for speed
	int nStepLenCmd[2]; // cmd flags for step length
	int nZmpCmd[2]; // cmd falgs for zmp width
	int nListen;
}dccFreeWalkCmd;

Extern dccFreeWalkCmd stFreeWalk;

void fnvFreeWalkInit();
void fnvDccFreeWalk(int nCmdDispFlag, int nSpeedDispFlag, int nStepDispFlag);

#undef Extern

//#define USE_FREEWALK_METHOD

#endif
