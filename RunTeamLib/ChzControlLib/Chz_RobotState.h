#pragma once
typedef struct
{
	double x;
	double y;
	double z;
	double dx;
	double dy;
	double dz;
	double ddx;
	double ddy;
	double ddz;
}PosRotXYZ;

typedef struct
{
	int K_Con;
	double T_Con;
	int stepstate;
	PosRotXYZ compos, comang, lfootpos, rfootpos, lfootang, rfootang;
	double legang[12], dlegang[12], ddlegang[12];
	double armang[12], darmang[12], ddarmang[12];
	PosRotXYZ lfootF, rfootF, lfootT, rfootT;
}RobotState;

void GetMeasuredState(RobotState* MState);
void GetPlannedState(RobotState* PState);
void SetControlState(const RobotState* PState);

void Con1(RobotState* PState, const RobotState* MState);