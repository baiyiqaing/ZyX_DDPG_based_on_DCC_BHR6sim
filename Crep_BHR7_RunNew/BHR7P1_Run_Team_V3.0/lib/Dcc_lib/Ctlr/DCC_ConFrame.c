// This file requires the vals from PG and Sens, then modifies the trajectories. 
// 20210109 bit
#ifndef DCC_ConFrame_C
#define DCC_ConFrame_C
#include "DCC_ConFrame.h"

// control platform ====================================================================================================
#define __BHR7RUN
double CoMRe[3];
double RankRe[3];
double LankRe[3];

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
// clear
extern dccPositional stLimpConVal;
extern dccSpacial stMZmpConVal; // rec dcc
extern dccFootFT stLimpAddiTrq;
extern dccAnkle stGrfConVal;
extern dccPositional stTpcConVal;
extern dccRotational stSupPosConVal;
extern dccPositional stSurPassedZmp; // rec dcc

#endif

// include your controllers file here ==================================================================================
#ifdef __BHR7RUN
#include "DCC_RunningCon.h"
#endif

#define LingNumIn1 3
#define LingNumIn2 3
#define LingNumOut 3
int nLingNum_in[3] = { LingNumIn1, LingNumIn2, LingNumOut };
int nMethods[2] = { 1, 0 };
int nFuzzyBase_in[LingNumIn1][LingNumIn2] = {
	{ 0, 1, 1 },
	{ 1, 1, 2 },
	{ 2, 2, 2 },
};
double dMemConfig_in1[LingNumIn1][3] = {{ 0.0, 10.0, 50.0 },{ 10.0, 50.0, 150.0 },{ 50.0, 150.0, 1000.0 }}; // delF
double dMemConfig_in2[LingNumIn1][3] = {{ 0.0, 0.01, 0.02 },{ 0.01, 0.02, 0.04 },{ 0.02, 0.04, 0.08 }}; // H_step_z
double dMemConfig_out[LingNumIn1][3] = {{ 0.02, 0.03, 0.04 },{ 0.03, 0.04, 0.05},{ 0.04, 0.05, 0.06 }}; // Kfz

// parameters for the filters ==========================================================================================
#ifdef __BHR7RUN
double dTLagZMP = 0.01;
double dTLagIMU = 0.01;
double dTLagFrc = 0.0001;
double dTLagTrq = 0.0001;
double dJointsInit[6] = { 0.0, 0.0, 12.5, -25.0, 12.5, 0.0 }; // armswi
#endif

void fnvDccPositionalConValClear(dccPositional * stConVal2bCleared) {
	for (int i = 0; i < 9; i++) *(&stConVal2bCleared->x + i) = 0.0;
}

void fnvDccRotationalConValClear(dccRotational* stConVal2bCleared) {
	for (int i = 0; i < 9; i++) *(&stConVal2bCleared->pit + i) = 0.0;
}

void fnvDccSpacialConValClear(dccSpacial* stConVal2bCleared) {
	for (int i = 0; i < 18; i++) *(&stConVal2bCleared->pos.x + i) = 0.0;
}

void fnvStateInit(dccRobotState * stStateName) { // init all state
	for (int i = 0; i < 9; i++) {
		*(&stStateName->Ankle.B.Rfoot.pos.x + i) = -0.0;
		*(&stStateName->Ankle.B.Lfoot.pos.x + i) = 0.0;
		*(&stStateName->Ankle.B.Rfoot.rot.pit + i) = 0.0;
		*(&stStateName->Ankle.B.Lfoot.rot.pit + i) = 0.0;
		*(&stStateName->Ankle.W.Rfoot.pos.x + i) = 0.0;
		*(&stStateName->Ankle.W.Lfoot.pos.x + i) = 0.0;
		*(&stStateName->Ankle.W.Rfoot.rot.pit + i) = 0.0;
		*(&stStateName->Ankle.W.Lfoot.rot.pit + i) = 0.0;
		*(&stStateName->Base.pos.x + i) = 0.0;
		*(&stStateName->Base.rot.pit + i) = 0.0;
		*(&stStateName->ZMP.B.x + i) = 0.0;
		*(&stStateName->ZMP.W.x + i) = 0.0;
	}
	for (int i = 0; i < 6; i++) {
		*(&stStateName->FootFT.Rfoot.fx + i) = 0.0;
		*(&stStateName->FootFT.Lfoot.fx + i) = 0.0;
	}
	stStateName->SupLeg = 0;
	stStateName->FootFT.Fsum = 0.0;
}

void fnvDccControlInit() {
	fnvStateInit(&stStatePG);
	fnvStateInit(&stStateSens);
	fnvStateInit(&stStateConVal);
	fnvStateInit(&stStateRef);
	fnvStateInit(&stStateCmd);
	fnvDccPositionalConValClear(&stLimpConVal);
	fnvDccPositionalConValClear(&stTpcConVal);
	fnvDccPositionalConValClear(&stSurPassedZmp);
	fnvDccRotationalConValClear(&stSupPosConVal);
	fnvDccSpacialConValClear(&stMZmpConVal);
	stStateCmd.Ankle.B.Rfoot.pos.x = 0.08;
	stStateCmd.Ankle.B.Lfoot.pos.x = -0.08;
	for (int i = 0; i < 6; i++) { // armswi
		*(stJoints.Lq + i) = DccD2R(dJointsInit[i]);
		*(stJoints.Rq + i) = DccD2R(dJointsInit[i]);
		*(stJoints.Ldq + i) = 0.0;
		*(stJoints.Rdq + i) = 0.0;
		stJoints.La = 0.0;
		stJoints.Ra = 0.0;
	}
	fnvFuzzyConInit(nFuzzyBase_in, dMemConfig_in1, dMemConfig_in2, dMemConfig_out, nLingNum_in, nMethods); // init the fuzzy controller
}

#ifdef __BHR7RUN
void fnvGetStatePG(int nKpre) { // get data from pattern generate
	stStatePG.ZMP.W.x = Tra_ZMP.x[nKpre];
	stStatePG.ZMP.W.y = Tra_ZMP.y[nKpre];
	stStatePG.ZMP.B.x = P_ZMPRef_B.px;
	stStatePG.ZMP.B.y = P_ZMPRef_B.py;
	stStatePG.Base.pos.dx = (Tra_COM.x[nKpre] - stStatePG.Base.pos.x) / CONTROL_T;
	stStatePG.Base.pos.dy = (Tra_COM.y[nKpre] - stStatePG.Base.pos.y) / CONTROL_T;
	stStatePG.Base.pos.x = Tra_COM.x[nKpre];
	stStatePG.Base.pos.y = Tra_COM.y[nKpre];
	stStatePG.Base.pos.z = Tra_COM.z[nKpre];
	stStatePG.Base.rot.dpit = (pitch_body - stStatePG.Base.rot.pit) / CONTROL_T;
	stStatePG.Base.rot.drol = (roll_body  - stStatePG.Base.rot.rol) / CONTROL_T;
	stStatePG.Base.rot.pit = -pitch_body;
	stStatePG.Base.rot.rol = roll_body;
	stStateRef.Base.rot.pit = stStatePG.Base.rot.pit;
	stStateRef.Base.rot.rol = stStatePG.Base.rot.rol;
	stStatePG.Ankle.W.Rfoot.pos.x = Tra_RAnkle.x[nKpre];
	stStatePG.Ankle.W.Rfoot.pos.y = Tra_RAnkle.y[nKpre];
	stStatePG.Ankle.W.Rfoot.pos.z = Tra_RAnkle.z[nKpre];
	stStatePG.Ankle.W.Rfoot.rot.pit = pitch_footr;
	stStatePG.Ankle.W.Rfoot.rot.rol = 0.0;
	stStatePG.Ankle.W.Lfoot.pos.x = Tra_LAnkle.x[nKpre];
	stStatePG.Ankle.W.Lfoot.pos.y = Tra_LAnkle.y[nKpre];
	stStatePG.Ankle.W.Lfoot.pos.z = Tra_LAnkle.z[nKpre];
	stStatePG.Ankle.W.Lfoot.rot.pit = pitch_footl;
	stStatePG.Ankle.W.Lfoot.rot.rol = 0.0;
	stStatePG.Ankle.B.Rfoot.pos.x = P_RAnkleRef_B.px;
	stStatePG.Ankle.B.Rfoot.pos.y = P_RAnkleRef_B.py;
	stStatePG.Ankle.B.Rfoot.pos.z = P_RAnkleRef_B.pz;
	stStatePG.Ankle.B.Lfoot.pos.x = P_LAnkleRef_B.px;
	stStatePG.Ankle.B.Lfoot.pos.y = P_LAnkleRef_B.py;
	stStatePG.Ankle.B.Lfoot.pos.z = P_LAnkleRef_B.pz;
#ifdef USE_CHZ_RUN
	stStatePG.Base.pos.ddz = Tra_ACOM.z[nKpre];
	stStatePG.FootFT.Rfoot.tx = -chzrun_MF[nKpre][0];
	stStatePG.FootFT.Rfoot.ty =  chzrun_MF[nKpre][2];
	stStatePG.FootFT.Lfoot.tx = -chzrun_MF[nKpre][0];
	stStatePG.FootFT.Lfoot.ty =  chzrun_MF[nKpre][2];
	stStatePG.SupLeg = chzrun_signal[nKpre][2]; 
#else 
	stStatePG.Base.pos.ddz = 0.0;
	stStatePG.SupLeg = Signal_SupportLeg[nKpre];
#endif
}

void fnvGetStateSens(int nKpre) { // get data from sensors
	stStateSens.ZMP.B.x = fndFilterTimeLag(stStateSens.ZMP.B.x, P_ZMPRel_B.px, CONTROL_T, dTLagZMP);
	stStateSens.ZMP.B.y = fndFilterTimeLag(stStateSens.ZMP.B.y, P_ZMPRel_B.py, CONTROL_T, dTLagZMP);
	if (isnan(stStateSens.ZMP.B.x)) stStateSens.ZMP.B.x = 0.0;
	if (isnan(stStateSens.ZMP.B.y)) stStateSens.ZMP.B.y = 0.0;
	stStateSens.Base.rot.dpit = (fndFilterTimeLag(stStateSens.Base.rot.pit, XS_Pitch, CONTROL_T, dTLagIMU) - stStateSens.Base.rot.pit) / CONTROL_T;
	stStateSens.Base.rot.drol = (fndFilterTimeLag(stStateSens.Base.rot.rol, XS_Roll,  CONTROL_T, dTLagIMU) - stStateSens.Base.rot.rol) / CONTROL_T;
	stStateSens.Base.rot.pit = fndFilterTimeLag(stStateSens.Base.rot.pit, XS_Pitch, CONTROL_T, dTLagIMU);
	stStateSens.Base.rot.rol = fndFilterTimeLag(stStateSens.Base.rot.rol, XS_Roll,  CONTROL_T, dTLagIMU); ;
	for (int i = 0; i < 3; i++) {
		*(&stStateSens.FootFT.Lfoot.fx + i) = fndFilterTimeLag(*(&stStateSens.FootFT.Lfoot.fx + i), *(&F_LFoot.fx + i), CONTROL_T, dTLagFrc);
		*(&stStateSens.FootFT.Rfoot.fx + i) = fndFilterTimeLag(*(&stStateSens.FootFT.Rfoot.fx + i), *(&F_RFoot.fx + i), CONTROL_T, dTLagFrc);
		*(&stStateSens.FootFT.Lfoot.tx + i) = fndFilterTimeLag(*(&stStateSens.FootFT.Lfoot.tx + i), *(&F_LFoot.tx + i), CONTROL_T, dTLagTrq);
		*(&stStateSens.FootFT.Rfoot.tx + i) = fndFilterTimeLag(*(&stStateSens.FootFT.Rfoot.tx + i), *(&F_RFoot.tx + i), CONTROL_T, dTLagTrq);
	}
	stStateSens.FootFT.Fsum = stStateSens.FootFT.Lfoot.fz + stStateSens.FootFT.Rfoot.fz;
	for (int i = 0; i < 6; i++) { // armswi
		stJoints.Ldq[i] = (PreCon_LegJoint.ql[i + 1] - stJoints.Lq[i]) / __ControlT;
		stJoints.Rdq[i] = (PreCon_LegJoint.qr[i + 1] - stJoints.Rq[i]) / __ControlT;
		stJoints.Lq[i] = PreCon_LegJoint.ql[i + 1];
		stJoints.Rq[i] = PreCon_LegJoint.qr[i + 1];
	}
}
void fnvGetConVal() { // add your controllers here and sum up the control value
	fnvStateInit(&stStateConVal); // dcc rec
	fnvDccRunCon();
}
void fnvAddConVal(int nKpre) { // modify the trajactories
	for (int i = 0; i < 3; i++) {
		*(&stStateCmd.Base.pos.x + i) = *(&stStatePG.Base.pos.x + i) + *(&stStateConVal.Base.pos.x + i);
		*(&stStateCmd.Base.rot.pit + i) = *(&stStatePG.Base.rot.pit + i) + *(&stStateConVal.Base.rot.pit + i);
	}
	for (int i = 0; i < 3; i++) {
		*(&stStateCmd.Ankle.W.Rfoot.pos.x + i) = *(&stStatePG.Ankle.W.Rfoot.pos.x + i) + *(&stStateConVal.Ankle.B.Rfoot.pos.x + i);
		*(&stStateCmd.Ankle.W.Lfoot.pos.x + i) = *(&stStatePG.Ankle.W.Lfoot.pos.x + i) + *(&stStateConVal.Ankle.B.Lfoot.pos.x + i);
		*(&stStateCmd.Ankle.W.Rfoot.rot.pit + i) = *(&stStatePG.Ankle.W.Rfoot.rot.pit + i) + *(&stStateConVal.Ankle.B.Rfoot.rot.pit + i);
		*(&stStateCmd.Ankle.W.Lfoot.rot.pit + i) = *(&stStatePG.Ankle.W.Lfoot.rot.pit + i) + *(&stStateConVal.Ankle.B.Lfoot.rot.pit + i);
		*(&stStateCmd.Ankle.B.Rfoot.pos.x + i) = *(&stStatePG.Ankle.B.Rfoot.pos.x + i) + *(&stStateConVal.Ankle.B.Rfoot.pos.x + i);
		*(&stStateCmd.Ankle.B.Lfoot.pos.x + i) = *(&stStatePG.Ankle.B.Lfoot.pos.x + i) + *(&stStateConVal.Ankle.B.Lfoot.pos.x + i);
	}
	CoMRe[0] = Tra_COM.x[nKpre] = stStateCmd.Base.pos.x;
	CoMRe[1] = Tra_COM.y[nKpre] = stStateCmd.Base.pos.y;
	CoMRe[2] = Tra_COM.z[nKpre] = stStateCmd.Base.pos.z;
	pitch_body = stStateCmd.Base.rot.pit;
	roll_body  = stStateCmd.Base.rot.rol;
	RankRe[0] = Tra_RAnkle.x[nKpre] = stStateCmd.Ankle.W.Rfoot.pos.x;
	RankRe[1] = Tra_RAnkle.y[nKpre] = stStateCmd.Ankle.W.Rfoot.pos.y;
	RankRe[2] = Tra_RAnkle.z[nKpre] = stStateCmd.Ankle.W.Rfoot.pos.z;
	pitch_footr = stStateCmd.Ankle.W.Rfoot.rot.pit;
	roll_footr = stStateCmd.Ankle.W.Rfoot.rot.rol;
	LankRe[0] = Tra_LAnkle.x[nKpre] = stStateCmd.Ankle.W.Lfoot.pos.x;
	LankRe[1] = Tra_LAnkle.y[nKpre] = stStateCmd.Ankle.W.Lfoot.pos.y;
	LankRe[2] = Tra_LAnkle.z[nKpre] = stStateCmd.Ankle.W.Lfoot.pos.z;
	pitch_footl = stStateCmd.Ankle.W.Lfoot.rot.pit;
	roll_footl = stStateCmd.Ankle.W.Lfoot.rot.rol;
	Ref_Arm_Joint[2][1] = stJoints.La; // armswi
	Ref_Arm_Joint[1][1] = stJoints.Ra;
	// printf("%lf", Ref_Arm_Joint[1][1]);
}
#endif

// zyx read--读取这些值

void fnvDccControlUpdate(int nKpre) {
	fnvGetStatePG(nKpre);	// -> stStatePG
	fnvGetStateSens(nKpre); // -> stStateSens
	fnvGetConVal();			// -> stStateConVal
#ifdef USE_DCC_CONTROLFRAME
	fnvAddConVal(nKpre);	// -> stStateCmd
#endif
}

#endif