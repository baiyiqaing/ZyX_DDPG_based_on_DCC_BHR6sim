// For running project of BHR7
// 20210111 bit
#ifndef DCC_RunningCon_C
#define DCC_RunningCon_C
#include "DCC_RunningCon.h"

// dcc_fuzzy
#define FuzzyDispFlag 0

// convals
dccPositional stLimpConVal = { 0.0 };
dccSpacial stMZmpConVal = { 0.0 }; // rec dcc
dccFootFT stLimpAddiTrq = { 0.0 };
dccAnkle stGrfConVal = { 0.0 };
dccPositional stTpcConVal = { 0.0 };
dccRotational stSupPosConVal = { 0.0 };
dccPositional stSurPassedZmp = { 0.0 }; // rec dcc

// suppoly // rec dcc
dccSupPoly stSupPoly_B = { 0.0 };
dccSupPoly stSupPoly_W = { 0.0 };

// globals
enum supsignal {
	DouSup, RightSup, LeftSup, Fly
};
double dZc = 0.7;
double dFTouchDown = 0.1 * __MRobot * __Gravity;
double dFUpToFly  = 0.05 * __MRobot * __Gravity;
#define __IUpper 14.0 * 0.4 * 0.4 * 0.083

/** Cal Support Polygon
* InputVal: dFootGeom_in[d4], #AnkleCmd_B_old, AnkleCmd_W_old
* OutputVal: #stSupPoly_B[dccSupPoly], stSupPoly_W[dccSupPoly], 
*/
// rec dcc
void fnvGetSupPoly(double dFootGeom_in[4]) {
	if (stStatePG.SupLeg == RightSup) {
		stSupPoly_B.forw = stStateCmd.Ankle.B.Rfoot.pos.y + dFootGeom_in[0];
		stSupPoly_B.back = stStateCmd.Ankle.B.Rfoot.pos.y - dFootGeom_in[1];
		stSupPoly_B.left = stStateCmd.Ankle.B.Rfoot.pos.x - dFootGeom_in[2];
		stSupPoly_B.righ = stStateCmd.Ankle.B.Rfoot.pos.x + dFootGeom_in[3];
		stSupPoly_W.forw = stStateCmd.Ankle.W.Rfoot.pos.y + dFootGeom_in[0];
		stSupPoly_W.back = stStateCmd.Ankle.W.Rfoot.pos.y - dFootGeom_in[1];
		stSupPoly_W.left = stStateCmd.Ankle.W.Rfoot.pos.x - dFootGeom_in[2];
		stSupPoly_W.righ = stStateCmd.Ankle.W.Rfoot.pos.x + dFootGeom_in[3];
	}
	else if (stStatePG.SupLeg == LeftSup) {
		stSupPoly_B.forw = stStateCmd.Ankle.B.Lfoot.pos.y + dFootGeom_in[0];
		stSupPoly_B.back = stStateCmd.Ankle.B.Lfoot.pos.y - dFootGeom_in[1];
		stSupPoly_B.left = stStateCmd.Ankle.B.Lfoot.pos.x - dFootGeom_in[3];
		stSupPoly_B.righ = stStateCmd.Ankle.B.Lfoot.pos.x + dFootGeom_in[2];
		stSupPoly_W.forw = stStateCmd.Ankle.W.Lfoot.pos.y + dFootGeom_in[0];
		stSupPoly_W.back = stStateCmd.Ankle.W.Lfoot.pos.y - dFootGeom_in[1];
		stSupPoly_W.left = stStateCmd.Ankle.W.Lfoot.pos.x - dFootGeom_in[3];
		stSupPoly_W.righ = stStateCmd.Ankle.W.Lfoot.pos.x + dFootGeom_in[2];
	}
	else if (stStatePG.SupLeg == DouSup) {
		if (stStateCmd.Ankle.B.Rfoot.pos.y >= stStateCmd.Ankle.B.Lfoot.pos.y) { // R forward
			stSupPoly_B.forw = stStateCmd.Ankle.B.Rfoot.pos.y + dFootGeom_in[0];
			stSupPoly_B.back = stStateCmd.Ankle.B.Lfoot.pos.y - dFootGeom_in[1];
			stSupPoly_W.forw = stStateCmd.Ankle.W.Rfoot.pos.y + dFootGeom_in[0];
			stSupPoly_W.back = stStateCmd.Ankle.W.Lfoot.pos.y - dFootGeom_in[1];
		}
		else { // L forward
			stSupPoly_B.forw = stStateCmd.Ankle.B.Lfoot.pos.y + dFootGeom_in[0];
			stSupPoly_B.back = stStateCmd.Ankle.B.Rfoot.pos.y - dFootGeom_in[1];
			stSupPoly_W.forw = stStateCmd.Ankle.W.Lfoot.pos.y + dFootGeom_in[0];
			stSupPoly_W.back = stStateCmd.Ankle.W.Rfoot.pos.y - dFootGeom_in[1];
		}
		stSupPoly_B.left = stStateCmd.Ankle.B.Lfoot.pos.x - dFootGeom_in[3];
		stSupPoly_B.righ = stStateCmd.Ankle.B.Rfoot.pos.x + dFootGeom_in[3];
		stSupPoly_W.left = stStateCmd.Ankle.W.Lfoot.pos.x - dFootGeom_in[3];
		stSupPoly_W.righ = stStateCmd.Ankle.W.Rfoot.pos.x + dFootGeom_in[3];
	}
	else {
		for (int i = 0; i < 4; i++) {
			*(&stSupPoly_B.left + i) = 0.0;
			*(&stSupPoly_W.left + i) = 0.0;
		}
	}
	// re
	for (int i = 0; i < 4; i++) {
		dSupPoly[i] = *(&stSupPoly_B.forw + i);
		dSupPoly[i + 4] = *(&stSupPoly_W.forw + i);
	}	
}

/** LIPM controller
* InputVal: paras[d6], limits[d4], #Base_rot_Sens, Base_drot_Sens, Base_rot_Ref, Base_drot_Ref, TpcConVal_old, ZMP_B_Sens, ZMP_B_PG 
* OutoutVal: #ZMP_B_Ref, stSurPassedZmp
*/
// rec dcc
void fnvLipmCon(double dParasLipm_in[6], double dLimitsLipm_in[4]) {
	double kp_x = dParasLipm_in[0];	double kp_y = dParasLipm_in[0 + 3];
	double kv_x = dParasLipm_in[1];	double kv_y = dParasLipm_in[1 + 3];
	double kz_x = dParasLipm_in[2];	double kz_y = dParasLipm_in[2 + 3];
	double LIPMLimit_x[2] = { stSupPoly_B.left + dLimitsLipm_in[0], stSupPoly_B.righ - dLimitsLipm_in[1] };
	double LIPMLimit_y[2] = { stSupPoly_B.back + dLimitsLipm_in[2], stSupPoly_B.forw - dLimitsLipm_in[3] };
	// cal posture
	double delta_pit = stStateSens.Base.rot.pit - stStateRef.Base.rot.pit;
	double delta_rol = stStateSens.Base.rot.rol - stStateRef.Base.rot.rol;
	double delta_dpit = stStateSens.Base.rot.dpit - stStateRef.Base.rot.dpit;
	double delta_drol = stStateSens.Base.rot.drol - stStateRef.Base.rot.drol;
	// cal state
	double delta_cx = stTpcConVal.x + dZc * sin(delta_rol);
	double delta_cy = stTpcConVal.y - dZc * sin(delta_pit);
	double delta_vx = stTpcConVal.dx + dZc * sin(delta_drol);
	double delta_vy = stTpcConVal.dy - dZc * sin(delta_dpit);
#ifdef USE_LIPM
	// cal ConVal
	stLimpConVal.x = fndAddLimit(kp_x * delta_cx + kv_x * delta_vx + kz_x * (stStateSens.ZMP.B.x - stStatePG.ZMP.B.x), stStatePG.ZMP.B.x, &stSurPassedZmp.x, LIPMLimit_x);
	stLimpConVal.y = fndAddLimit(kp_y * delta_cy + kv_y * delta_vy + kz_y * (stStateSens.ZMP.B.y - stStatePG.ZMP.B.y), stStatePG.ZMP.B.y, &stSurPassedZmp.y, LIPMLimit_y);
	// StateRef update
	stStateRef.ZMP.B.x = stStatePG.ZMP.B.x + stLimpConVal.x;
	stStateRef.ZMP.B.y = stStatePG.ZMP.B.y + stLimpConVal.y;
#endif
	// re
	dLipmRe[0] = delta_cx;
	dLipmRe[1] = delta_vx;
	dLipmRe[2] = delta_cy;
	dLipmRe[3] = delta_vy;
	for (int i = 0; i < 2; i++) {
		dLipmRe[4 + 4 * i] = *(&stLimpConVal.x + i);
		dLipmRe[5 + 4 * i] = *(&stStateRef.ZMP.B.x + i);
		dLipmRe[6 + 4 * i] = *(&stStateSens.ZMP.B.x + i);
		dLipmRe[7 + 4 * i] = *(&stStatePG.ZMP.B.x + i);
	}
}

void fnvModelZmpCon(double dParasMZmp_in[6], double dLimitMZmp_in[2], char cMethodFlag) {
	double micro_x = dParasMZmp_in[0], micro_y = dParasMZmp_in[3];
	double kp_x = dParasMZmp_in[1], kd_x = dParasMZmp_in[2], kp_y = dParasMZmp_in[4], kd_y = dParasMZmp_in[5];
	double dLimit_x[6] = { -dLimitMZmp_in[0], dLimitMZmp_in[0], -10.0, 10.0, -100.0, 100.0 };
	double dLimit_y[6] = { -dLimitMZmp_in[1], dLimitMZmp_in[1], -10.0, 10.0, -100.0, 100.0 };
	double dPosBaseTarget_x, dPosBaseTarget_y;
	double kcom_x = 50.0, kcom_y = 50.0;
	double dLimit_cx[6] = { -0.03, 0.03, -50.0, 50.0, -500.0, 500.0 };
	double dLimit_cy[6] = { -0.2, 0.2, -50.0, 50.0, -500.0, 500.0 };
	if (cMethodFlag == 'a') { // cal acc method
		stMZmpConVal.pos.ddx = (micro_x * stSurPassedZmp.x * __MRobot * __Gravity) / __MRobot / dZc - kp_x * stMZmpConVal.pos.x - kd_x * stMZmpConVal.pos.dx;
		stMZmpConVal.pos.ddy = (micro_y * stSurPassedZmp.y * __MRobot * __Gravity) / __MRobot / dZc - kp_y * stMZmpConVal.pos.y - kd_y * stMZmpConVal.pos.dy;
		// update
		fnvIntegLimit(&stMZmpConVal.pos.x, &stMZmpConVal.pos.dx, stMZmpConVal.pos.ddx, dLimit_x, __ControlT);
		fnvIntegLimit(&stMZmpConVal.pos.y, &stMZmpConVal.pos.dy, stMZmpConVal.pos.ddy, dLimit_y, __ControlT);
#ifdef USE_MODELZMP
		stStateConVal.Base.pos.x += stMZmpConVal.pos.x;
		stStateConVal.Base.pos.y += stMZmpConVal.pos.y;
		dMZmpRe[2] = stMZmpConVal.pos.x;
		dMZmpRe[3] = stMZmpConVal.pos.y;
#endif
	}
	else {
		if (fabs(stSurPassedZmp.x) > 1e-6) dPosBaseTarget_x = -0.1 * dZc * sin(stStateSens.Base.rot.rol);
		else dPosBaseTarget_x = 0.0;
		if (fabs(stSurPassedZmp.y) > 1e-6) dPosBaseTarget_y = 0.1 * dZc * sin(stStateSens.Base.rot.pit);
		else dPosBaseTarget_y = 0.0;
		stMZmpConVal.pos.ddx = kcom_x * (dPosBaseTarget_x - stMZmpConVal.pos.x) - 10.0 * stMZmpConVal.pos.dx;
		stMZmpConVal.pos.ddy = kcom_y * (dPosBaseTarget_y - stMZmpConVal.pos.y) - 10.0 * stMZmpConVal.pos.dy;
		stMZmpConVal.rot.ddrol = -(-(250.0 - 30.0) * micro_x * stSurPassedZmp.x * __Gravity + stMZmpConVal.pos.ddx * dZc) * __MRobot / __IUpper - kp_x * stMZmpConVal.rot.rol - kd_x * stMZmpConVal.rot.drol;
		stMZmpConVal.rot.ddpit =  (-(250.0 - 30.0) * micro_y * stSurPassedZmp.y * __Gravity + stMZmpConVal.pos.ddy * dZc) * __MRobot / __IUpper - kp_y * stMZmpConVal.rot.pit - kd_y * stMZmpConVal.rot.dpit;
		// update
		fnvIntegLimit(&stMZmpConVal.pos.x, &stMZmpConVal.pos.dx, stMZmpConVal.pos.ddx, dLimit_cx, __ControlT);
		fnvIntegLimit(&stMZmpConVal.pos.y, &stMZmpConVal.pos.dy, stMZmpConVal.pos.ddy, dLimit_cy, __ControlT);
		fnvIntegLimit(&stMZmpConVal.rot.rol, &stMZmpConVal.rot.drol, stMZmpConVal.rot.ddrol, dLimit_x, __ControlT);
		fnvIntegLimit(&stMZmpConVal.rot.pit, &stMZmpConVal.rot.dpit, stMZmpConVal.rot.ddpit, dLimit_y, __ControlT);
		stMZmpConVal.pos.z = -0.5 * sqrt(stMZmpConVal.pos.x * stMZmpConVal.pos.x + stMZmpConVal.pos.y * stMZmpConVal.pos.y);
#ifdef USE_MODELZMP
		stStateConVal.Base.pos.x += stMZmpConVal.pos.x;
		stStateConVal.Base.pos.y += stMZmpConVal.pos.y;
		stStateConVal.Base.pos.z += stMZmpConVal.pos.z;
		stStateRef.Base.rot.rol += stMZmpConVal.rot.rol;
		stStateRef.Base.rot.pit += stMZmpConVal.rot.pit;
		dMZmpRe[2] = stMZmpConVal.pos.x;
		dMZmpRe[3] = stMZmpConVal.pos.y;
		dMZmpRe[4] = stMZmpConVal.pos.z;
		dMZmpRe[5] = stMZmpConVal.rot.pit * 57.3;
		dMZmpRe[6] = stMZmpConVal.rot.rol * 57.3;
#endif
	}

	// re
	dMZmpRe[0] = stSurPassedZmp.x;
	dMZmpRe[1] = stSurPassedZmp.y;
}

/** LIPM Additorque phases check & torque distribution
* InputVal: paras[d2], limits[d4], #ZMP_B_Ref, ZMP_B_Sens, SupSignal, FootFT_Sens, Ankle_B_Cmd_Old
* OutputVal: #LipmAddiTrq
*/
void fnvLipmAddiTrq(double dParasAddiTrq_in[2], double dLimitAddiTrq_in[4]) {
	double Trq_x_limit[2] = { dLimitAddiTrq_in[0], dLimitAddiTrq_in[1] }; // pit
	double Trq_y_limit[2] = { dLimitAddiTrq_in[2], dLimitAddiTrq_in[3] }; // rol
	double micro_px = dParasAddiTrq_in[0]; double micro_py = dParasAddiTrq_in[1];
	double xmidpoint = 0.0;
	double alpha = 0.5;
	double Limit_alpha[2] = { 0.0, 1.0 };
#ifdef USE_ADDITRQ
	// cal delzmp
	double delta_px = stLimpConVal.x;//stStateRef.ZMP.B.x - micro_px * stStateSens.ZMP.B.x;
	double delta_py = stLimpConVal.y;//stStateRef.ZMP.B.y - micro_py * stStateSens.ZMP.B.y;
	int RealSupFlag = Fly;
	// phases check
	if (stStateSens.FootFT.Fsum > dFTouchDown) { // touch down
		if (stStatePG.SupLeg == RightSup) { // r sup
			if (stStateSens.FootFT.Rfoot.fz > 0.5 * dFTouchDown) { // r touch down
				RealSupFlag = RightSup;
			}
			else { // fly
				RealSupFlag = Fly;
			}
		}
		else if (stStatePG.SupLeg == LeftSup) { // l sup
			if (stStateSens.FootFT.Lfoot.fz > 0.5 * dFTouchDown) { // l touch down
				RealSupFlag = LeftSup;
			}
			else { // fly
				RealSupFlag = Fly;
			}
		}
		else if (stStatePG.SupLeg == DouSup) { // dou sup
			if ((stStateSens.FootFT.Rfoot.fz > 0.5 * dFTouchDown) && (stStateSens.FootFT.Lfoot.fz > 0.5 * dFTouchDown)) { // dou touch down
				RealSupFlag = DouSup;
			}
			else if ((stStateSens.FootFT.Rfoot.fz > 0.5 * dFTouchDown) && (stStateSens.FootFT.Lfoot.fz <= 0.5 * dFTouchDown)) { // r touch down
				RealSupFlag = RightSup;
			}
			else if ((stStateSens.FootFT.Rfoot.fz <= 0.5 * dFTouchDown) && (stStateSens.FootFT.Lfoot.fz > 0.5 * dFTouchDown)) { // l touch down
				RealSupFlag = LeftSup;
			}
			else { // fly
				RealSupFlag = Fly;
			}
		}
		else { // fly phase
			RealSupFlag = Fly;
		}
	}
	else { // no touch down
		RealSupFlag = Fly;
	}
	// distribution
	if (RealSupFlag == RightSup) {
		stLimpAddiTrq.Rfoot.ty = fndLimit(-delta_px * __MRobot * __Gravity, Trq_y_limit);
		stLimpAddiTrq.Rfoot.tx = fndLimit(delta_py * __MRobot * __Gravity, Trq_x_limit);
		stLimpAddiTrq.Rfoot.fz = 0.0;
		stLimpAddiTrq.Lfoot.ty = 0.0;
		stLimpAddiTrq.Lfoot.tx = 0.0;
		stLimpAddiTrq.Lfoot.fz = 0.0;
		//printf("RightSup\n");
	}
	else if (RealSupFlag == LeftSup) {
		stLimpAddiTrq.Rfoot.ty = 0.0;
		stLimpAddiTrq.Rfoot.tx = 0.0;
		stLimpAddiTrq.Rfoot.fy = 0.0;
		stLimpAddiTrq.Lfoot.ty = fndLimit(-delta_px * __MRobot * __Gravity, Trq_y_limit);
		stLimpAddiTrq.Lfoot.tx = fndLimit(delta_py * __MRobot * __Gravity, Trq_x_limit);
		stLimpAddiTrq.Lfoot.fz = 0.0;
		//printf("LeftSup\n");
	}
	else if (RealSupFlag == DouSup) {
		xmidpoint = (stStateSens.ZMP.B.x*stStateCmd.Ankle.B.Lfoot.pos.x*stStateCmd.Ankle.B.Lfoot.pos.x - 2 * stStateSens.ZMP.B.x*stStateCmd.Ankle.B.Lfoot.pos.x*stStateCmd.Ankle.B.Rfoot.pos.x - stStateCmd.Ankle.B.Lfoot.pos.x*stStateCmd.Ankle.B.Lfoot.pos.y*stStateCmd.Ankle.B.Rfoot.pos.y + stStateSens.ZMP.B.y*stStateCmd.Ankle.B.Lfoot.pos.x*stStateCmd.Ankle.B.Lfoot.pos.y + stStateCmd.Ankle.B.Lfoot.pos.x*stStateCmd.Ankle.B.Rfoot.pos.y*stStateCmd.Ankle.B.Rfoot.pos.y - stStateSens.ZMP.B.y*stStateCmd.Ankle.B.Lfoot.pos.x*stStateCmd.Ankle.B.Rfoot.pos.y + stStateSens.ZMP.B.x*stStateCmd.Ankle.B.Rfoot.pos.x*stStateCmd.Ankle.B.Rfoot.pos.x + stStateCmd.Ankle.B.Rfoot.pos.x*stStateCmd.Ankle.B.Lfoot.pos.y*stStateCmd.Ankle.B.Lfoot.pos.y - stStateCmd.Ankle.B.Rfoot.pos.x*stStateCmd.Ankle.B.Lfoot.pos.y*stStateCmd.Ankle.B.Rfoot.pos.y - stStateSens.ZMP.B.y*stStateCmd.Ankle.B.Rfoot.pos.x*stStateCmd.Ankle.B.Lfoot.pos.y + stStateSens.ZMP.B.y*stStateCmd.Ankle.B.Rfoot.pos.x*stStateCmd.Ankle.B.Rfoot.pos.y) / (stStateCmd.Ankle.B.Lfoot.pos.x*stStateCmd.Ankle.B.Lfoot.pos.x - 2 * stStateCmd.Ankle.B.Lfoot.pos.x*stStateCmd.Ankle.B.Rfoot.pos.x + stStateCmd.Ankle.B.Rfoot.pos.x*stStateCmd.Ankle.B.Rfoot.pos.x + stStateCmd.Ankle.B.Lfoot.pos.y*stStateCmd.Ankle.B.Lfoot.pos.y - 2 * stStateCmd.Ankle.B.Lfoot.pos.y*stStateCmd.Ankle.B.Rfoot.pos.y + stStateCmd.Ankle.B.Rfoot.pos.y*stStateCmd.Ankle.B.Rfoot.pos.y);
		alpha = fndLimit((xmidpoint - stStateCmd.Ankle.B.Lfoot.pos.x) / (stStateCmd.Ankle.B.Rfoot.pos.x - stStateCmd.Ankle.B.Lfoot.pos.x), Limit_alpha); // force propotion to right foot
		stLimpAddiTrq.Rfoot.ty = (alpha) * fndLimit(-delta_px * __MRobot * __Gravity, Trq_y_limit);
		stLimpAddiTrq.Rfoot.tx = (alpha) * fndLimit(delta_py * __MRobot * __Gravity, Trq_x_limit);
		stLimpAddiTrq.Rfoot.fz = 0.0;
		stLimpAddiTrq.Lfoot.ty = (1 - alpha) * fndLimit(-delta_px * __MRobot * __Gravity, Trq_y_limit);
		stLimpAddiTrq.Lfoot.tx = (1 - alpha) * fndLimit(delta_py * __MRobot * __Gravity, Trq_x_limit);
		stLimpAddiTrq.Lfoot.fz = 0.0;
		//printf("DouSup\n");
		//printf("alpha = %f\n", alpha);
	}
	else if (RealSupFlag == Fly) {
		for (int i = 0; i < 3; i++) {
			*(&stLimpAddiTrq.Rfoot.fz + i) = 0.0;
			*(&stLimpAddiTrq.Lfoot.fz + i) = 0.0;
		}
		//printf("Fly\n");
	}
	else {
		printf("Error in fnsLipmAddiTrq !!!\n");
		for (int i = 0; i < 3; i++) {
			*(&stLimpAddiTrq.Rfoot.fz + i) = 0.0;
			*(&stLimpAddiTrq.Lfoot.fz + i) = 0.0;
		}
	}
#endif
	// re
	for (int i = 0; i < 3; i++) {
		dAddiTrqRe[i] = *(&stLimpAddiTrq.Rfoot.fz + i);
		dAddiTrqRe[i + 3] = *(&stLimpAddiTrq.Lfoot.fz + i);
	}
	dAddiTrqRe[6] = xmidpoint;
	dAddiTrqRe[7] = alpha;
}

/** Cal Ref FootFT 
* InputVal: dAdditrqPit, #FootFT_PG, LipmAddiTrq, FootFT_Sens, SupSignal, ZMP_B_Ref, Ankle_B_Cmd_old
* OutputVal: #FootFT_Ref
*/
void fnvCalFootFTRef(double dAdditrq[3]) {
	int RealSupFlag = Fly;
	double xmidpoint = 0.0;
	double alpha = 0.5;
	double Limit_alpha[2] = { 0.0, 1.0 }; 
	// phases ckeck
	if (stStatePG.SupLeg == RightSup) { // r sup
		RealSupFlag = RightSup;
		if (stStateSens.FootFT.Fsum < dFUpToFly) RealSupFlag = Fly;
		//printf("RightSup\n");
	}
	else if (stStatePG.SupLeg == LeftSup) { // l sup
		RealSupFlag = LeftSup;
		if (stStateSens.FootFT.Fsum < dFUpToFly) RealSupFlag = Fly;
		//printf("LeftSup\n");
	}
	else if (stStatePG.SupLeg == DouSup) { // dou sup
		RealSupFlag = DouSup;
		if (stStateSens.FootFT.Fsum < dFUpToFly) RealSupFlag = Fly;
		//printf("DouSup\n");
	}
	else { // fly
		RealSupFlag = Fly;
		//printf("Fly\n");
	}
	// distribution
	if (RealSupFlag == RightSup) {
		stStateRef.FootFT.Rfoot.tx = 0.0 * stStatePG.FootFT.Rfoot.tx + stLimpAddiTrq.Rfoot.tx + dAdditrq[0];
		stStateRef.FootFT.Rfoot.ty = 0.0 * stStatePG.FootFT.Rfoot.ty + stLimpAddiTrq.Rfoot.ty + dAdditrq[1];
		stStateRef.FootFT.Rfoot.fz = __MRobot * (__Gravity + 1.0 * stStatePG.Base.pos.ddz);
		stStateRef.FootFT.Lfoot.tx = 0.0;
		stStateRef.FootFT.Lfoot.ty = 0.0;
		stStateRef.FootFT.Lfoot.fz = 0.0;
	}
	else if (RealSupFlag == LeftSup) {
		stStateRef.FootFT.Rfoot.tx = 0.0;
		stStateRef.FootFT.Rfoot.ty = 0.0;
		stStateRef.FootFT.Rfoot.fz = 0.0;
		stStateRef.FootFT.Lfoot.tx = 0.0 * stStatePG.FootFT.Lfoot.tx + stLimpAddiTrq.Lfoot.tx + dAdditrq[0];
		stStateRef.FootFT.Lfoot.ty = 0.0 * stStatePG.FootFT.Lfoot.ty + stLimpAddiTrq.Lfoot.ty + dAdditrq[2];
		stStateRef.FootFT.Lfoot.fz = __MRobot * (__Gravity + 1.0 * stStatePG.Base.pos.ddz);
	}
	else if (RealSupFlag == DouSup) {
		xmidpoint = (stStatePG.ZMP.B.x*stStateCmd.Ankle.B.Lfoot.pos.x*stStateCmd.Ankle.B.Lfoot.pos.x - 2 * stStatePG.ZMP.B.x*stStateCmd.Ankle.B.Lfoot.pos.x*stStateCmd.Ankle.B.Rfoot.pos.x - stStateCmd.Ankle.B.Lfoot.pos.x*stStateCmd.Ankle.B.Lfoot.pos.y*stStateCmd.Ankle.B.Rfoot.pos.y + stStatePG.ZMP.B.y*stStateCmd.Ankle.B.Lfoot.pos.x*stStateCmd.Ankle.B.Lfoot.pos.y + stStateCmd.Ankle.B.Lfoot.pos.x*stStateCmd.Ankle.B.Rfoot.pos.y*stStateCmd.Ankle.B.Rfoot.pos.y - stStatePG.ZMP.B.y*stStateCmd.Ankle.B.Lfoot.pos.x*stStateCmd.Ankle.B.Rfoot.pos.y + stStatePG.ZMP.B.x*stStateCmd.Ankle.B.Rfoot.pos.x*stStateCmd.Ankle.B.Rfoot.pos.x + stStateCmd.Ankle.B.Rfoot.pos.x*stStateCmd.Ankle.B.Lfoot.pos.y*stStateCmd.Ankle.B.Lfoot.pos.y - stStateCmd.Ankle.B.Rfoot.pos.x*stStateCmd.Ankle.B.Lfoot.pos.y*stStateCmd.Ankle.B.Rfoot.pos.y - stStatePG.ZMP.B.y*stStateCmd.Ankle.B.Rfoot.pos.x*stStateCmd.Ankle.B.Lfoot.pos.y + stStatePG.ZMP.B.y*stStateCmd.Ankle.B.Rfoot.pos.x*stStateCmd.Ankle.B.Rfoot.pos.y) / (stStateCmd.Ankle.B.Lfoot.pos.x*stStateCmd.Ankle.B.Lfoot.pos.x - 2 * stStateCmd.Ankle.B.Lfoot.pos.x*stStateCmd.Ankle.B.Rfoot.pos.x + stStateCmd.Ankle.B.Rfoot.pos.x*stStateCmd.Ankle.B.Rfoot.pos.x + stStateCmd.Ankle.B.Lfoot.pos.y*stStateCmd.Ankle.B.Lfoot.pos.y - 2 * stStateCmd.Ankle.B.Lfoot.pos.y*stStateCmd.Ankle.B.Rfoot.pos.y + stStateCmd.Ankle.B.Rfoot.pos.y*stStateCmd.Ankle.B.Rfoot.pos.y);
		alpha = fndLimit((xmidpoint - stStateCmd.Ankle.B.Lfoot.pos.x) / (stStateCmd.Ankle.B.Rfoot.pos.x - stStateCmd.Ankle.B.Lfoot.pos.x), Limit_alpha); // force propotion to right foot
		//alpha = fndLimit((stStatePG.ZMP.B.x - stStatePG.Ankle.B.Lfoot.pos.x) / (stStatePG.Ankle.B.Rfoot.pos.x - stStatePG.Ankle.B.Lfoot.pos.x), Limit_alpha);
		stStateRef.FootFT.Rfoot.tx = ((alpha) * 0.0 * stStatePG.FootFT.Rfoot.tx + dAdditrq[0]) + stLimpAddiTrq.Rfoot.tx;
		stStateRef.FootFT.Rfoot.ty = 1.0 * (((alpha) * stStatePG.FootFT.Rfoot.ty) + stLimpAddiTrq.Rfoot.ty);
		stStateRef.FootFT.Rfoot.fz = (alpha) * __MRobot * (__Gravity + 1.0 * stStatePG.Base.pos.ddz);
		stStateRef.FootFT.Lfoot.tx = ((1 - alpha) * 0.0 * stStatePG.FootFT.Lfoot.tx + dAdditrq[0]) + stLimpAddiTrq.Lfoot.tx;
		stStateRef.FootFT.Lfoot.ty = 1.0 * (((1 - alpha) * stStatePG.FootFT.Lfoot.ty) + stLimpAddiTrq.Lfoot.ty);
		stStateRef.FootFT.Lfoot.fz = (1 - alpha) * __MRobot * (__Gravity + 1.0 * stStatePG.Base.pos.ddz);
		//printf("alpha = %f\n", alpha);
	}
	else if (RealSupFlag == Fly) {
		for (int i = 0; i < 3; i++) {
			*(&stStateRef.FootFT.Rfoot.fz + i) = 0.0;
			*(&stStateRef.FootFT.Lfoot.fz + i) = 0.0;
		}
	}
	else {
		printf("Error in fnsLipmAddiTrq !!!\n");
		for (int i = 0; i < 3; i++) {
			*(&stStateRef.FootFT.Rfoot.fz + i) = 0.0;
			*(&stStateRef.FootFT.Lfoot.fz + i) = 0.0;
		}
	}
	// re
	dFootFTRefRe[6] = xmidpoint;
	dFootFTRefRe[7] = alpha;
	nSupSig = stStatePG.SupLeg;
}

/** Ground reaction force control
* InputVal: Paras[d6], Paras[d2], Limits[d9], dLimitAddiTrq[d4], TreshHolds[d6], dVarStiffLat_T, #GrfConVal_old, FootFT_Ref, FootFT_Sens
* OutputVal:
*/
double kf_delF[4] = { 0.0 }; // Rpit, Lpit, Rrol, Lrol
double kp_delF[4] = { 0.0 }; // Rpit, Lpit, Rrol, Lrol
double kz_delF[2] = { 0.0 }; // Rz, Lz
int nInitFlag = 1;
void fnvGrfCon(double dParasGrfC[7], double dParasVarStiff[2], double dLimitsGrfC[7], double dLimitAddiTrq[4], double dThreshGrfC[6], double dVarStiffLat_T) {
	double kp_pitch = dParasGrfC[0]; double kd_pitch = dParasGrfC[1];
	double kp_roll  = dParasGrfC[2]; double kd_roll  = dParasGrfC[3];
	double kf_zctrl = dParasGrfC[4]; double kp_zctrl = dParasGrfC[5]; double kd_zctrl = dParasGrfC[6];
	double limit_pitch[4] = { -dLimitsGrfC[0], dLimitsGrfC[0], -dLimitsGrfC[1], dLimitsGrfC[1] };
	double limit_roll[4] = { -dLimitsGrfC[2], dLimitsGrfC[2], -dLimitsGrfC[3], dLimitsGrfC[3] };
	double limit_zctrl[6] = { dLimitsGrfC[4], dLimitsGrfC[5], -dLimitsGrfC[6], dLimitsGrfC[6], -100.0, 100.0 };
	double thresh_pitch[2] = { dThreshGrfC[0], dThreshGrfC[1] };
	double thresh_roll[2] = { dThreshGrfC[2], dThreshGrfC[3] };
	double thresh_zctrl[2] = { dThreshGrfC[4], dThreshGrfC[5] };
	// var stiff
#ifdef USE_VARSTIFF
	if (nInitFlag == 1) {
		nInitFlag = 0;
		for (int i = 0; i < 2; i++) {
			kf_delF[i] = kp_pitch;
			kp_delF[i] = kd_pitch;
			kf_delF[i + 2] = kp_roll;
			kp_delF[i + 2] = kd_roll;
		}
	}
	double k_kf[2] = { 0.0 }, b_kf[2] = { 0.0 }, k_kp[2] = { 0.0 }, b_kp[2] = { 0.0 }; // pit, rol
	double Zu = dParasVarStiff[0], Zd = dParasVarStiff[1];
	double delFmax[2] = { dLimitAddiTrq[1], dLimitAddiTrq[3] }; // pit, rol
	double LimitPit[2] = { dLimitAddiTrq[0], dLimitAddiTrq[1] }, LimitRol[2] = { dLimitAddiTrq[2], dLimitAddiTrq[3] };
	double delF[4] = { fndLimit(fabs(fndThreshold((stStateSens.FootFT.Rfoot.tx - stStateRef.FootFT.Rfoot.tx), thresh_pitch)), LimitPit), fndLimit(fabs(fndThreshold((stStateSens.FootFT.Lfoot.tx - stStateRef.FootFT.Lfoot.tx), thresh_pitch)), LimitPit), fndLimit(fabs(fndThreshold((stStateSens.FootFT.Rfoot.ty - stStateRef.FootFT.Rfoot.ty), thresh_roll)), LimitRol), fndLimit(fabs(fndThreshold((stStateSens.FootFT.Lfoot.ty - stStateRef.FootFT.Lfoot.ty), thresh_roll)), LimitRol)}; // Rpit, Lpit, Rrol, Lrol
	for (int i = 0; i < 2; i++) {
		k_kf[i] = (dParasGrfC[2 * i] * (Zd * Zu - 1)) / (Zd * delFmax[i]);
		b_kf[i] = dParasGrfC[2 * i] / Zd;
		k_kp[i] = -(dParasGrfC[2 * i + 1] * (Zd * Zu - 1)) / (Zd * delFmax[i]);
		b_kp[i] = Zu * dParasGrfC[2 * i + 1];
	}
	for (int i = 0; i < 2; i++) {
		kf_delF[i] = fndFilterTimeLag(kf_delF[i], k_kf[0] * delF[i] + b_kf[0], __ControlT, dVarStiffLat_T);
		kp_delF[i] = fndFilterTimeLag(kp_delF[i], k_kp[0] * delF[i] + b_kp[0], __ControlT, dVarStiffLat_T);
		kf_delF[i + 2] = fndFilterTimeLag(kf_delF[i + 2], k_kf[1] * delF[i + 2] + b_kf[1], __ControlT, dVarStiffLat_T);
		kp_delF[i + 2] = fndFilterTimeLag(kp_delF[i + 2], k_kp[1] * delF[i + 2] + b_kp[1], __ControlT, dVarStiffLat_T);
	}
#else
	for (int i = 0; i < 2; i++) {
		kf_delF[i] = kp_pitch;
		kp_delF[i] = kd_pitch;
		kf_delF[i + 2] = kp_roll;
		kp_delF[i + 2] = kd_roll;
	}
#endif
#ifdef USE_DCCFUZZY
	double Limit_Fz[2] = { 0.0, 1000.0 };
	double delF_z[2] = { fndLimit(fabs(fndThreshold((stStateSens.FootFT.Rfoot.fz - stStateRef.FootFT.Rfoot.fz), thresh_zctrl)), Limit_Fz), fndLimit(fabs(fndThreshold((stStateSens.FootFT.Lfoot.fz - stStateRef.FootFT.Lfoot.fz), thresh_zctrl)), Limit_Fz) };
	double H_step_z[2] = { stStatePG.Ankle.W.Rfoot.pos.z - __AnkleHeight, stStatePG.Ankle.W.Lfoot.pos.z - __AnkleHeight };
	for (int i = 0; i < 2; i++) {
		kz_delF[i] = fndCalFuzzyConVal(delF_z[i], H_step_z[i], FuzzyDispFlag); // control cycle: delF, H_step
	}
#else
	for (int i = 0; i < 2; i++) {
		kz_delF[i] = kf_zctrl;
	}
#endif
	
	// cal pit rol
	stGrfConVal.Rfoot.rot.dpit = kf_delF[0] * fndThreshold((stStateSens.FootFT.Rfoot.tx - stStateRef.FootFT.Rfoot.tx), thresh_pitch) - kp_delF[0] * stGrfConVal.Rfoot.rot.pit;
	stGrfConVal.Rfoot.rot.drol = kf_delF[2] * fndThreshold((stStateSens.FootFT.Rfoot.ty - stStateRef.FootFT.Rfoot.ty), thresh_roll ) - kp_delF[2] * stGrfConVal.Rfoot.rot.rol;
	stGrfConVal.Lfoot.rot.dpit = kf_delF[1] * fndThreshold((stStateSens.FootFT.Lfoot.tx - stStateRef.FootFT.Lfoot.tx), thresh_pitch) - kp_delF[1] * stGrfConVal.Lfoot.rot.pit;
	stGrfConVal.Lfoot.rot.drol = kf_delF[3] * fndThreshold((stStateSens.FootFT.Lfoot.ty - stStateRef.FootFT.Lfoot.ty), thresh_roll ) - kp_delF[3] * stGrfConVal.Lfoot.rot.rol;
	// cal z
	stGrfConVal.Rfoot.pos.ddz = kz_delF[0] * fndThreshold((stStateSens.FootFT.Rfoot.fz - stStateRef.FootFT.Rfoot.fz), thresh_zctrl) - kp_zctrl * stGrfConVal.Rfoot.pos.z - kd_zctrl * stGrfConVal.Rfoot.pos.dz;
	stGrfConVal.Lfoot.pos.ddz = kz_delF[1] * fndThreshold((stStateSens.FootFT.Lfoot.fz - stStateRef.FootFT.Lfoot.fz), thresh_zctrl) - kp_zctrl * stGrfConVal.Lfoot.pos.z - kd_zctrl * stGrfConVal.Lfoot.pos.dz;
	// update
	fnvVeloLimit(&stGrfConVal.Rfoot.rot.pit, stGrfConVal.Rfoot.rot.dpit, limit_pitch, __ControlT);
	fnvVeloLimit(&stGrfConVal.Rfoot.rot.rol, stGrfConVal.Rfoot.rot.drol, limit_roll , __ControlT);
	fnvVeloLimit(&stGrfConVal.Lfoot.rot.pit, stGrfConVal.Lfoot.rot.dpit, limit_pitch, __ControlT);
	fnvVeloLimit(&stGrfConVal.Lfoot.rot.rol, stGrfConVal.Lfoot.rot.drol, limit_roll , __ControlT);
	fnvIntegLimit(&stGrfConVal.Rfoot.pos.z, &stGrfConVal.Rfoot.pos.dz, stGrfConVal.Rfoot.pos.ddz, limit_zctrl, __ControlT);
	fnvIntegLimit(&stGrfConVal.Lfoot.pos.z, &stGrfConVal.Lfoot.pos.dz, stGrfConVal.Lfoot.pos.ddz, limit_zctrl, __ControlT);
#ifdef USE_GRFC
	stStateConVal.Ankle.B.Rfoot.rot.pit = 1.0 * stGrfConVal.Rfoot.rot.pit;
	stStateConVal.Ankle.B.Rfoot.rot.rol = 1.0 * stGrfConVal.Rfoot.rot.rol;
	stStateConVal.Ankle.B.Rfoot.pos.z   = 1.0 * stGrfConVal.Rfoot.pos.z;
	stStateConVal.Ankle.B.Lfoot.rot.pit = 1.0 * stGrfConVal.Lfoot.rot.pit;
	stStateConVal.Ankle.B.Lfoot.rot.rol = 1.0 * stGrfConVal.Lfoot.rot.rol;
	stStateConVal.Ankle.B.Lfoot.pos.z   = 1.0 * stGrfConVal.Lfoot.pos.z;
#endif
	// re
	for (int i = 0; i < 3; i++) {
		dFootFTRelRe[i] = *(&stStateSens.FootFT.Rfoot.fz + i);
		dFootFTRelRe[i + 3] = *(&stStateSens.FootFT.Lfoot.fz + i);
		dFootFTRefRe[i] = *(&stStateRef.FootFT.Rfoot.fz + i);
		dFootFTRefRe[i + 3] = *(&stStateRef.FootFT.Lfoot.fz + i);
	}
	dGrfConValRe[0] = stGrfConVal.Rfoot.pos.z; 
	dGrfConValRe[1] = stGrfConVal.Rfoot.rot.pit; 
	dGrfConValRe[2] = stGrfConVal.Rfoot.rot.rol;
	dGrfConValRe[3] = stGrfConVal.Lfoot.pos.z;
	dGrfConValRe[4] = stGrfConVal.Lfoot.rot.pit;
	dGrfConValRe[5] = stGrfConVal.Lfoot.rot.rol;
	for (int i = 0; i < 4; i++) {
		dKfKpRe[i] = kf_delF[i];
		dKfKpRe[i + 4] = kp_delF[i];
	}
}

/** TPC controller
InputVal: Bias[d2], TpcConVal_old[dccPositional], paras[d6], limits[d6], #ZMP_B_Ref, ZMP_B_Sens, FootFT_Sens
OutputVal: TpcConVal
*/
void fnvTpcCon(double dBias[2], double dParasTpc_in[6], double dLimitsTpc_in[6]) {
	double kz_x = dParasTpc_in[0];	double kz_y = dParasTpc_in[0 + 3];
	double kp_x = dParasTpc_in[1];	double kp_y = dParasTpc_in[1 + 3];
	double kd_x = dParasTpc_in[2];  double kd_y = dParasTpc_in[2 + 3];
	double limit_x[6] = { -dLimitsTpc_in[0], dLimitsTpc_in[0], -dLimitsTpc_in[1], dLimitsTpc_in[1], -dLimitsTpc_in[2], dLimitsTpc_in[2] };
	double limit_y[6] = { -dLimitsTpc_in[3], dLimitsTpc_in[3], -dLimitsTpc_in[4], dLimitsTpc_in[4], -dLimitsTpc_in[5], dLimitsTpc_in[5] };
	// fly protection: back to zero point
	if (stStateSens.FootFT.Fsum < dFUpToFly) kz_x = 0.0, kz_y = 0.0; 
	// cal acc
	stTpcConVal.ddx = kz_x * (stStateSens.ZMP.B.x + dBias[0] - stStateRef.ZMP.B.x) - kp_x * stTpcConVal.x - kd_x * stTpcConVal.dx;
	stTpcConVal.ddy = kz_y * (stStateSens.ZMP.B.y + dBias[1] - stStateRef.ZMP.B.y) - kp_y * stTpcConVal.y - kd_y * stTpcConVal.dy;
	// update
	fnvIntegLimit(&stTpcConVal.x, &stTpcConVal.dx, stTpcConVal.ddx, limit_x, __ControlT);
	fnvIntegLimit(&stTpcConVal.y, &stTpcConVal.dy, stTpcConVal.ddy, limit_y, __ControlT);
#ifdef USE_TPC
	stStateConVal.Base.pos.x += stTpcConVal.x;
	stStateConVal.Base.pos.y += stTpcConVal.y;
#endif
	// re
	dTpcRe[0] = stTpcConVal.x;
	dTpcRe[1] = stTpcConVal.y;
}

/** Support phase posture controller
InputVal: Paras[d6], Limits[d4], #Base_rot_Rel, Base_rot_Sens
OutputVal: #Base_rot_Ref, Base_drot_Ref 
*/
void fnvSupPosCon(double dParasSupPos_in[6], double dLimitSupPos_in[4]) {
	double k_pitch  = dParasSupPos_in[0]; double k_roll  = dParasSupPos_in[0 + 3];
	double kp_pitch = dParasSupPos_in[1]; double kp_roll = dParasSupPos_in[1 + 3];
	double kd_pitch = dParasSupPos_in[2]; double kd_roll = dParasSupPos_in[2 + 3];
	double limit_pitch[6] = { -dLimitSupPos_in[0], dLimitSupPos_in[0], -dLimitSupPos_in[1], dLimitSupPos_in[1], -50.0, 50.0};
	double limit_roll[6]  = { -dLimitSupPos_in[2], dLimitSupPos_in[2], -dLimitSupPos_in[3], dLimitSupPos_in[3], -50.0, 50.0};
	if (stStateSens.FootFT.Fsum < dFUpToFly) k_pitch = 0.0, k_roll = 0.0; // fly
	// cal acc
	//stSupPosConVal.ddpit = k_pitch * (stStatePG.Base.rot.pit - stStateSens.Base.rot.pit) - kp_pitch * stSupPosConVal.pit - kd_pitch * stSupPosConVal.dpit;
	//stSupPosConVal.ddrol = k_roll  * (stStatePG.Base.rot.rol - stStateSens.Base.rot.rol) - kp_roll  * stSupPosConVal.rol - kd_roll  * stSupPosConVal.drol;
	stSupPosConVal.ddpit = k_pitch * (stStateRef.Base.rot.pit - stStateSens.Base.rot.pit) - kp_pitch * stSupPosConVal.pit - kd_pitch * stSupPosConVal.dpit;
	stSupPosConVal.ddrol = k_roll  * (stStateRef.Base.rot.rol - stStateSens.Base.rot.rol) - kp_roll  * stSupPosConVal.rol - kd_roll  * stSupPosConVal.drol;
	// update
	fnvIntegLimit(&stSupPosConVal.pit, &stSupPosConVal.dpit, stSupPosConVal.ddpit, limit_pitch, __ControlT);
	fnvIntegLimit(&stSupPosConVal.rol, &stSupPosConVal.drol, stSupPosConVal.ddrol, limit_roll , __ControlT);
#ifdef USE_SUPPOSCON
	for (int i = 0; i < 2; i++) {
		//*(&stStateRef.Base.rot.pit + i) = *(&stStatePG.Base.rot.pit + i) + *(&stSupPosConVal.pit + i);
		*(&stStateRef.Base.rot.pit + i) = *(&stStatePG.Base.rot.pit + i);
		*(&stStateConVal.Base.rot.pit + i) = *(&stSupPosConVal.pit + i);
	}
#endif
	// re
	dSupPosp[0] = stSupPosConVal.pit;
	dSupPosp[1] = stSupPosConVal.rol;

}

void fnvArmSwi(double dParasArmSwi_in[2]) { // armswi
	double K_L = dParasArmSwi_in[0];
	double K_P = 1.0 - K_L;
	double v_M = dParasArmSwi_in[1];
	double dql_arm[3], dqr_arm[3]; // L, P, L + P
	dql_arm[0] = 2 * ANKLE_WIDTH / (SHOUDLER_WIDTH * __LArm * (__MArm + v_M)) * ((0.5 * __MThigh + __MShank + __MFoot) * __LThigh * stJoints.Ldq[2] + (0.5 * __MShank + __MFoot) * __LShank * stJoints.Ldq[3]);
	dqr_arm[0] = 2 * ANKLE_WIDTH / (SHOUDLER_WIDTH * __LArm * (__MArm + v_M)) * ((0.5 * __MThigh + __MShank + __MFoot) * __LThigh * stJoints.Rdq[2] + (0.5 * __MShank + __MFoot) * __LShank * stJoints.Rdq[3]);
	dqr_arm[1] = -2 / (__LArm * (__MArm + v_M)) * ((0.5 * __MThigh + __MShank + __MFoot) * __LThigh * stJoints.Ldq[2] + (0.5 * __MShank + __MFoot) * __LShank * stJoints.Ldq[3]);
	dql_arm[1] = -2 / (__LArm * (__MArm + v_M)) * ((0.5 * __MThigh + __MShank + __MFoot) * __LThigh * stJoints.Rdq[2] + (0.5 * __MShank + __MFoot) * __LShank * stJoints.Rdq[3]);
	dql_arm[2] = K_L * dql_arm[0] + K_P * dql_arm[1];
	dqr_arm[2] = K_L * dqr_arm[0] + K_P * dqr_arm[1];
#ifdef USE_ARMSWING
	stJoints.La = stJoints.La - dql_arm[2] * __ControlT;
	stJoints.Ra = stJoints.Ra - dqr_arm[2] * __ControlT;
#endif
	// re
	dArm[0] = stJoints.Ra;
	dArm[1] = stJoints.La;
}

void fnvDccRunCon() {
	// paras FootGeom
	// rec dcc
	double dFootGeom[4] = { /*forw*/0.13, /*back*/0.09, /*iner*/0.05, /*outer*/0.09 };
	// paras Limp
	double dParasLipm[6] = { /*kp_x, kv_x, kz_x*/2.5 * 1.1, 0.8 * 1.1, -0.022917, /*kp_y, kv_y, kz_y*/2.5 * 1.1, 0.8 * 1.1, -0.022917 };
	 // zyx rec
	double dLimitsLipm[4] = { /*x*/-0.0, 0.0, /*y*/-0.0, 0.0 };
	// paras MZmp
	char   cMethodMZmp = 'p';
	double dParasMZmp[6] = { /*micro_x, kp_x, kd_x*/1.0, 60.0, 10.0, /*micro_y, kp_y, kd_y*/1.0, 60.0, 10.0 };
	double dLimitMZmp[2] = { /*x*/DccD2R(10.0), /*y*/DccD2R(30.0) };
	// paras AddiTrq
	double dParasAddiTrq[2] = { /*micro_x*/0.4, /*micro_y*/0.3 };
	double dLimitAddiTrq[4] = { /*trq_x*/-35.0, 50.0, /*trq_y*/-32.0, 32.0 };
	// paras GrfCon
	double dAdditrq[3] = { /*pit*/6.0, /*rol_r*/0.0, /*rol_l*/0.0 };
	double dParasGrfC[7] = { /*pit*/0.00778 * 1.2, 1.00184, /*rol*/0.00778 * 1.2, 1.00184, /*zctrl*/0.042, 88.0, 66.0 };
	double dParasVarStiff[2] = { /*Zu*/2.0, /*Zd*/5.0 };
	double dLimitsGrfC[7] = { /*pit*/DccD2R(30.0), 10.0, /*rol*/DccD2R(20.0), 10.0, /*zctrl*/-0.03, 0.06, 10.0 };
	double dThreshGrfC[6] = { /*pit*/-0.0, 0.0, /*rol*/-0.0, 0.0, /*zctrl*/-5.0, 5.0 };
	double dVarStiffLat_T = 0.04;
	// paras Tpc
	double dBias[2] = { /*x*/0.0, /*y*/-0.01 };
	double dParasTpc[6] = { /*kz_x, kp_x, kv_x*/15.340529, 68.782720, 25.445165, /*kz_y, kp_y, kv_y*/17.98669, 58.31472, 41.17373 };
	double dLimitsTpc[6] = { /*x*/0.04, 10.0, 50.0, /*y*/0.04, 10.0, 50.0 };
	// paras SupPos
	double dParasSupPos[6] = { /*kpit, kp, kd*/550.0, 50.0, 15.0, /*krol, kp, kd*/550.0, 50.0, 15.0 };
	double dLimitSupPos[4] = { /*pit*/DccD2R(20.0), 10.0, /*rol*/DccD2R(15.0), 10.0 };
	// paras ArmSwi
	double dParasArmSwi[2] = { /*K_L*/ 0.8049, /*VritualMassForArm*/ 20.0 };
	// controllers
	fnvGetSupPoly(dFootGeom);								// -> SupPoly rec dcc
	fnvLipmCon(dParasLipm, dLimitsLipm);					// -> ZMP_Ref 
	fnvModelZmpCon(dParasMZmp, dLimitMZmp, cMethodMZmp);	// -> ModelZmp rec dcc
	fnvLipmAddiTrq(dParasAddiTrq, dLimitAddiTrq);			// -> LipmAddiTrq
	fnvCalFootFTRef(dAdditrq);								// -> FootFT_Ref
	fnvGrfCon(dParasGrfC, dParasVarStiff, dLimitsGrfC, dLimitAddiTrq, dThreshGrfC, dVarStiffLat_T);	// -> AnkleConVal
	fnvTpcCon(dBias, dParasTpc, dLimitsTpc);				// -> TpcConVal
	fnvSupPosCon(dParasSupPos, dLimitSupPos);				// -> SupPosConVal
	fnvArmSwi(dParasArmSwi);								// -> ArmSwi
}
 
#endif 