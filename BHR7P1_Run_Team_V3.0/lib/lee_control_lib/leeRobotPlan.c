#include "leeRobotPlan.h"
#include <stdio.h>
#define TOE_ROTATION RAD(-0.0)
#define HEEL_ROTATION RAD(0)
#define RAISE_HEIGHT (FOOT_BACK*sin(fabs(TOE_ROTATION)))
#define BASE_YAW	RAD(0)
#define BASE_ROLL	RAD(0)
//目标值设置
_rf_flg set_goal(BRobot *robot)
{
	int i, j;
	BRobotState *p, *pc;
	BRobotState *mp;

	double temp1, temp2;
	int cstep;
	double com_h;

	cstep = robot->DCM.Current_Step;
	com_h = robot->DCM.COM_Height;
	temp1 = 50.0;
	temp2 = WAIST_HIP_Y;

	p = &(robot->GoalState);
	pc = &(robot->CurrentState);
	mp = &(robot->MidGoalState);
	*p = robot->CurrentState;
	*mp = *p;

	for (i = 0; i < PART_NUM; i++)
	{
		for (j = 0; j < 6; j++)
		{
			mp->Posture[i].MID_IKC_FLAG[j] = NOT_SET;
			p->Posture[i].MID_IKC_FLAG[j] = NOT_SET;
		}
		p->Posture[i].IKC_FLAG = NOT_SET;
		mp->Posture[i].IKC_FLAG = NOT_SET;
	}
	switch (robot->CurrentState.StateFlag)
	{
	case STATE_COM_DOWN://COM down
		robot->MotionFlag = CARTESIAN_MOVE;
		p->Posture[BASE].Posture.p[5] = robot->DCM.COM_Height*1000.0;
		REMAIN_FOOT;

		break;

	case STATE_WALK_1ST_STEP://The 1st DS in 1st step
		robot->MotionFlag = CARTESIAN_MOVE;
		robot->Squat.SpecialMode = DCM_WALKING;

		p->Posture[RIGHT_FOOT].Posture.p[5] = 0.0;
		p->Posture[LEFT_FOOT].Posture.p[5] = 0.0;
		pc->Posture[LEFT_FOOT].Posture.p[5] = 0.0;
		pc->Posture[RIGHT_FOOT].Posture.p[5] = 0.0;
		
		pc->Posture[QBODY].Posture.p[0] = 0;
		p->Posture[QBODY].Posture.p[0] = QBODY_MAX;
		REMAIN_QBODY;
		REMAIN_FOOT;

		robot->DCM.Walking_State = WS_DS;
		break;

	case STATE_WALK_SWL_SS://The SS phase of Swing left foot
		robot->MotionFlag = CARTESIAN_MOVE;
		robot->Squat.SpecialMode = DCM_WALKING;

		mp->Posture[LEFT_FOOT].Posture.p[5] = (robot->DCM.Foot_Height + robot->DCM.VRP_Point[cstep + 1][2] - com_h)*1000.0;
		mp->Posture[LEFT_FOOT].MID_IKC_FLAG[5] = HAD_SET;

		mp->Posture[LEFT_FOOT].Posture.p[1] = 0*HEEL_ROTATION;
		mp->Posture[LEFT_FOOT].MID_IKC_FLAG[1] = HAD_SET;

		if (robot->DCM.Current_Step < robot->DCM.Step_Num + 1)
		{
			p->Posture[LEFT_FOOT].Posture.p[3] = p->Posture[RIGHT_FOOT].Posture.p[3] + robot->DCM.Step_Length*1000.0;
			p->Posture[BASE].Posture.p[2] = -BASE_YAW;
			p->Posture[BASE].Posture.p[0] = BASE_ROLL;
			p->Posture[LEFT_FOOT].Posture.p[1] = TOE_ROTATION;
			p->Posture[LEFT_FOOT].Posture.p[5] = RAISE_HEIGHT;
		}
		else
		{
			p->Posture[LEFT_FOOT].Posture.p[3] = p->Posture[RIGHT_FOOT].Posture.p[3];
			p->Posture[BASE].Posture.p[2] = 0.0;
			p->Posture[BASE].Posture.p[0] = 0.0;
			p->Posture[LEFT_FOOT].Posture.p[1] = 0;
			p->Posture[LEFT_FOOT].Posture.p[5] = 0;
		}

		p->Posture[LEFT_FOOT].Posture.p[5] += (robot->DCM.VRP_Point[cstep + 1][2] - com_h)*1000.0;

		pc->Posture[QBODY].Posture.p[0] = QBODY_MAX;
		p->Posture[QBODY].Posture.p[0] = QBODY_MAX;
		REMAIN_QBODY;
		REMAIN_FOOT;

		robot->DCM.Walking_State = WS_RS;
		break;

	case STATE_WALK_SWL_DS://The DS phase of Swing left foot
		robot->MotionFlag = CARTESIAN_MOVE;
		robot->Squat.SpecialMode = DCM_WALKING;

		p->Posture[LEFT_FOOT].Posture.p[1] = 0.0;
		p->Posture[LEFT_FOOT].Posture.p[5] = (robot->DCM.VRP_Point[cstep + 1][2] - com_h)*1000.0;

		if (robot->DCM.Current_Step < robot->DCM.Step_Num + 1)
		{
			mp->Posture[RIGHT_FOOT].Posture.p[1] = 0;
			//mp->Posture[RIGHT_FOOT].MID_IKC_FLAG[1] = HAD_SET;
			mp->Posture[RIGHT_FOOT].Posture.p[5] = 0;
			//mp->Posture[RIGHT_FOOT].MID_IKC_FLAG[5] = HAD_SET;
			p->Posture[RIGHT_FOOT].Posture.p[1] = HEEL_ROTATION;
			p->Posture[RIGHT_FOOT].Posture.p[5] = FOOT_FONT*_rsin(HEEL_ROTATION);
			
			p->Posture[QBODY].Posture.p[0] = -QBODY_MAX;
		}
		else
		{
			p->Posture[RIGHT_FOOT].Posture.p[1] = 0.0;
			p->Posture[RIGHT_FOOT].Posture.p[5] = 0.0;
			p->Posture[QBODY].Posture.p[0] = 0;
		}

		p->Posture[RIGHT_FOOT].Posture.p[5] += (robot->DCM.VRP_Point[cstep + 1][2] - com_h)*1000.0;
		pc->Posture[QBODY].Posture.p[0] = QBODY_MAX;
		REMAIN_QBODY;
		REMAIN_FOOT;

		robot->DCM.Walking_State = WS_DS;
		break;

	case STATE_WALK_SWR_SS://The SS phase of Swing right foot
		robot->MotionFlag = CARTESIAN_MOVE;
		robot->Squat.SpecialMode = DCM_WALKING;

		mp->Posture[RIGHT_FOOT].Posture.p[5] = (robot->DCM.Foot_Height + robot->DCM.VRP_Point[cstep + 1][2] - com_h)*1000.0;
		mp->Posture[RIGHT_FOOT].MID_IKC_FLAG[5] = HAD_SET;

		mp->Posture[RIGHT_FOOT].Posture.p[1] = 0*HEEL_ROTATION;
		mp->Posture[RIGHT_FOOT].MID_IKC_FLAG[1] = HAD_SET;

		if (robot->DCM.Current_Step < robot->DCM.Step_Num + 1)
		{
			p->Posture[RIGHT_FOOT].Posture.p[3] = p->Posture[LEFT_FOOT].Posture.p[3] + robot->DCM.Step_Length*1000.0;
			p->Posture[BASE].Posture.p[2] = BASE_YAW;
			p->Posture[BASE].Posture.p[0] = -BASE_ROLL;
			p->Posture[RIGHT_FOOT].Posture.p[1] = TOE_ROTATION;
			p->Posture[RIGHT_FOOT].Posture.p[5] = RAISE_HEIGHT;
		}
		else
		{
			p->Posture[RIGHT_FOOT].Posture.p[3] = p->Posture[LEFT_FOOT].Posture.p[3];
			p->Posture[BASE].Posture.p[2] = 0;
			p->Posture[BASE].Posture.p[0] = 0;
			p->Posture[RIGHT_FOOT].Posture.p[1] = 0;
			p->Posture[RIGHT_FOOT].Posture.p[5] = 0;
		}

		p->Posture[RIGHT_FOOT].Posture.p[5] += (robot->DCM.VRP_Point[cstep + 1][2] - com_h)*1000.0;

		pc->Posture[QBODY].Posture.p[0] = -QBODY_MAX;
		p->Posture[QBODY].Posture.p[0] = -QBODY_MAX;
		REMAIN_QBODY;
		REMAIN_FOOT;

		robot->DCM.Walking_State = WS_LS;
		break;

	case STATE_WALK_SWR_DS://The DS phase of Swing right foot
		robot->MotionFlag = CARTESIAN_MOVE;
		robot->Squat.SpecialMode = DCM_WALKING;

		p->Posture[RIGHT_FOOT].Posture.p[1] = 0.0;
		p->Posture[RIGHT_FOOT].Posture.p[5] = (robot->DCM.VRP_Point[cstep + 1][2] - com_h)*1000.0;

		if (robot->DCM.Current_Step < robot->DCM.Step_Num + 1)
		{
			mp->Posture[LEFT_FOOT].Posture.p[1] = 0;
			//mp->Posture[LEFT_FOOT].MID_IKC_FLAG[1] = HAD_SET;
			mp->Posture[LEFT_FOOT].Posture.p[5] = 0;
			//mp->Posture[LEFT_FOOT].MID_IKC_FLAG[5] = HAD_SET;
			p->Posture[LEFT_FOOT].Posture.p[1] = HEEL_ROTATION;
			p->Posture[LEFT_FOOT].Posture.p[5] = FOOT_FONT*_rsin(HEEL_ROTATION);
			p->Posture[QBODY].Posture.p[0] = QBODY_MAX;
		}
		else
		{
			p->Posture[LEFT_FOOT].Posture.p[1] = 0.0;
			p->Posture[LEFT_FOOT].Posture.p[5] = 0.0;
			p->Posture[QBODY].Posture.p[0] = 0;
		}

		p->Posture[LEFT_FOOT].Posture.p[5] += (robot->DCM.VRP_Point[cstep + 1][2] - com_h)*1000.0;

		pc->Posture[QBODY].Posture.p[0] = -QBODY_MAX;
		REMAIN_QBODY;
		REMAIN_FOOT;

		robot->DCM.Walking_State = WS_DS;
		break;
	case STATE_WALK_DCM_STOP:
		robot->MotionFlag = CARTESIAN_MOVE;
		robot->DCM.Walking_State = WS_DS;
		robot->Squat.SpecialMode = DCM_STOP;
		REMAIN_FOOT;
		break;

	case STATE_DCM_STAND://stand_test
		robot->MotionFlag = CARTESIAN_MOVE;
		robot->DCM.Walking_State = WS_DS;
		REMAIN_FOOT;
		break;

	case STATE_FALL:
		robot->MotionFlag = JOINT_MOVE;
		for (i = 0; i < JOINT_NUMBER; i++)
		{
			p->Joint[i] = 0.0;
		}
		p->Joint[LEFT_LEG_3] = -RAD(70);
		p->Joint[LEFT_LEG_4] = RAD(140);
		p->Joint[LEFT_LEG_5] = -RAD(90);

		p->Joint[RIGHT_LEG_3] = -RAD(70);
		p->Joint[RIGHT_LEG_4] = RAD(140);
		p->Joint[RIGHT_LEG_5] = -RAD(90);

		p->Joint[WAIST_2] = RAD(10);

	case STATE_NUM:
		break;

	default:
		break;
	}
	p->Time += robot->Squat.StepTime[robot->CurrentState.StateFlag - 1] - robot->Squat.inter_time;
	mp->Time += 0.5*robot->Squat.StepTime[robot->CurrentState.StateFlag - 1] - robot->Squat.inter_time;
	return 0;
}

//姿态调整
_rf_flg posture_adjust(BRobot * robot)
{
	BRobotState * pc;
	BRobotState * pp;
	BRobotState * pg;
	TStruct wt;
	TStruct kt;
	PostureStruct adjust_base_posture;
	TStruct adjust_base_t;
	_rv_t temp;

	// Ellipse Move
	double b;
	// DCM Walking
	int i;
	static double real_vrp[3] = { 0,0,0 };
	static double real_dcm[3] = { 0,0,0 };	
	static double dcm[3] = { 0,0,0 };
	static double ddcm[3] = { 0,0,0.0 };
	static double cog[3] = { 0,0,0 };
	static double dcog[3] = {0};
	static double vrp[3] = { 0.0 };
	int step = robot->DCM.Current_Step;
	double t = robot->DCM.Current_Time;
	double T = robot->Squat.inter_time / 1000.0;
	static double b_real_dcm[3], b_real_vrp[3];
	static double b_dcm[3], b_vrp[3];

#ifdef VREP_SIM
	static FILE * fp_dcm;
	static FILE * fp_vmc6;
#endif


	//vrp[2] = robot->DCM.COM_Height;
	//dcm[2] = robot->DCM.COM_Height;
	//cog[2] = robot->DCM.COM_Height;

	pc = &(robot->CurrentState);
	pp = &(robot->PastState);
	pg = &(robot->GoalState);

	if (pc->StateFlag == 1)
	{
		cog[2] = pc->Posture[BASE].Posture.p[5] / 1000.0;
	}

	//Spetial Move
	switch (robot->Squat.SpecialMode)
	{
	case DCM_WALKING:
		b = robot->DCM.B;		
		
		if (pc->StateFlag == 2)// 1st ds 第一步（只有双脚支撑期）
		{
			get_dcm_p(
				robot->DCM.Step_Time,
				robot->DCM.DCM_DS_Ini[step], robot->DCM.DDCM_DS_Ini[step],
				robot->DCM.DCM_DS_End[step], robot->DCM.DDCM_DS_End[step],
				t, dcm, ddcm);
		}
		
		if (pc->StateFlag == 4 || pc->StateFlag == 6)// ds 双脚支撑期
		{
			get_dcm_p(
				robot->DCM.Step_Time_DS,
				robot->DCM.DCM_DS_Ini[step], robot->DCM.DDCM_DS_Ini[step],
				robot->DCM.DCM_DS_End[step], robot->DCM.DDCM_DS_End[step],
				t - robot->DCM.Step_Time_SS, dcm, ddcm);
		}

		for (i = 0; i < 3;i++)
		{
			if (pc->StateFlag == 3 || pc->StateFlag == 5)// ss //单脚支撑期
			{
				dcm[i] = robot->DCM.VRP_Point[step][i] + exp(1 / b*(t + robot->DCM.Step_Time_DS_End - robot->DCM.Step_Time))*(robot->DCM.DCM_End[step][i] - robot->DCM.VRP_Point[step][i]);
				ddcm[i] = 1 / b*exp(1 / b*(t + robot->DCM.Step_Time_DS_End - robot->DCM.Step_Time))*(robot->DCM.DCM_End[step][i] - robot->DCM.VRP_Point[step][i]);
				vrp[i] = robot->DCM.VRP_Point[step][i];
			}
			else
			{
				vrp[i] = dcm[i] - b*ddcm[i];
			}

			dcog[i] = -1 / b*(cog[i] - dcm[i]);
			cog[i] += dcog[i] * T;
			pc->Posture[BASE].Posture.p[i + 3] = cog[i]*1000.0;
		}
		pc->Posture[BASE].T = get_t(&pc->Posture[BASE].Posture);
		pg->Posture[BASE] = pc->Posture[BASE];
		//printf("step = %d,  x = %lf, y=%lf ,z=%lf\n",step, cog[0], cog[1], cog[2]);
		robot->DCM.Current_Time += T;
		break;
	
	case DCM_STOP:
		b = robot->DCM.B;
		for (i = 0; i < 3; i++)
		{
			dcog[i] = -1 / b*(cog[i] - dcm[i]);
			cog[i] += dcog[i] * T;
			pc->Posture[BASE].Posture.p[i + 3] = cog[i] * 1000.0;
		}
		pc->Posture[BASE].T = get_t(&pc->Posture[BASE].Posture);
		pg->Posture[BASE] = pc->Posture[BASE];
		break;

	default:
#ifdef VREP_SIM
		//if (fp_dcm != NULL)
		//{
		//	fclose(fp_dcm);
		//	fclose(fp_vmc6);
		//}
#endif
		break;
	}

	//根据多连杆模型计算由规划轨迹得到的实际质心位置
	get_body_com(robot);

	//粘弹性控制与平衡控制
	vmc6_update(robot);//更新控制所需要的信息
	zmp_tpc_update(robot, vrp, cog);
	single_foot_zmp_update(robot);
	if ((pc->StateFlag >= 2 && pc->StateFlag <= 7)|| (pc->StateFlag == 8))//行走与站立测试时开启控制
	{
		#ifdef STAND_TEST
		vrp[2] = robot->DCM.COM_Height;
		dcm[2] = robot->DCM.COM_Height;
		#endif
		
		
		posture_control_2_link_model(robot);
		//body_vmc_update(robot);
		single_foot_zmp_control(robot);
		equilibrium_point_control(robot);
		vmc6_control(robot);//控制
		zmp_tpc(robot);
		//平衡控制
		for (i = 0; i < 3; i++)
		{
			//根据力传感信息计算实际VRP点（世界坐标系）
			real_vrp[i] = (robot->VMC6.RealBodyPosture.p[i + 3] - (robot->VMC6.LQR[0][i + 3].real_value + robot->VMC6.LQR[1][i + 3].real_value-((double)(i==2))*ROBOT_WEIGHT*G)*pow(robot->DCM.B, 2.0) / ROBOT_WEIGHT);
			//根据加速度计计算实际VRP点（世界坐标系）
			//real_vrp[i] = robot->VMC6.RealBodyPosture.p[i + 3] - pow(robot->DCM.B, 2.0)*robot->VMC6.RealBodyAcc[i + 3];
			
			//根据加速度计计算实际DCM点（世界坐标系）
			//real_dcm[i] = (robot->VMC6.RealBodyPosture.p[i + 3] + robot->DCM.B*(robot->VMC6.RealBodyVel[i + 3]));
			real_dcm[i] = robot->VMC6.EstDCM[i];
		}
		//跟踪VRP点/DCM点，实现平衡控制
		//vrp_dcm_track(robot, real_vrp, real_dcm, vrp, dcm,cog, b_real_vrp,b_real_dcm,b_vrp,b_dcm);

		//记录VRP点、DCM点信息
		robot->dcmDataTab[DATA_REAL_B_VRP_X] = b_real_vrp[0];
		robot->dcmDataTab[DATA_REAL_B_VRP_Y] = b_real_vrp[1];
		robot->dcmDataTab[DATA_REAL_B_VRP_Z] = b_real_vrp[2];
		robot->dcmDataTab[DATA_REAL_B_DCM_X] = b_real_dcm[0];
		robot->dcmDataTab[DATA_REAL_B_DCM_Y] = b_real_dcm[1];
		robot->dcmDataTab[DATA_REAL_B_DCM_Z] = b_real_dcm[2];
		robot->dcmDataTab[DATA_REF_B_VRP_X] = b_vrp[0];
		robot->dcmDataTab[DATA_REF_B_VRP_Y] = b_vrp[1];
		robot->dcmDataTab[DATA_REF_B_VRP_Z] = b_vrp[2];
		robot->dcmDataTab[DATA_LEFT_FOOT_DELTA_Z] = robot->VMC6.LQR[0][5].delta;
		robot->dcmDataTab[DATA_RIGHT_FOOT_DELTA_Z] = robot->VMC6.LQR[1][5].delta;
		robot->dcmDataTab[DATA_REAL_VRP_X] = real_vrp[0];
		robot->dcmDataTab[DATA_REAL_VRP_Y] = real_vrp[1];
		robot->dcmDataTab[DATA_REAL_VRP_Z] = real_vrp[2];
		robot->dcmDataTab[DATA_REF_VRP_X] = vrp[0];
		robot->dcmDataTab[DATA_REF_VRP_Y] = vrp[1];
		robot->dcmDataTab[DATA_REF_VRP_Z] = vrp[2];
		robot->dcmDataTab[DATA_REF_DCM_X] = dcm[0];
		robot->dcmDataTab[DATA_REF_DCM_Y] = dcm[1];
		robot->dcmDataTab[DATA_REF_DCM_Z] = dcm[2];
		robot->dcmDataTab[DATA_COM_DELTA_X] = robot->ZMP_TPC.LQR[0].delta;
		robot->dcmDataTab[DATA_COM_DELTA_Y] = robot->ZMP_TPC.LQR[1].delta;
	}

#ifdef VREP_SIM
	if (fp_vmc6 == NULL)
	{
		fp_vmc6 = fopen("vmc6.dat", "w");
	}
	else
	{
		fprintf(fp_vmc6, "%f\t%f\t%f\t", robot->VMC6.LQR[0][0].real_value, robot->VMC6.LQR[0][0].ref_value, robot->VMC6.LQR[0][0].delta);	//1
		fprintf(fp_vmc6, "%f\t%f\t%f\t", robot->VMC6.LQR[0][1].real_value, robot->VMC6.LQR[0][1].ref_value, robot->VMC6.LQR[0][1].delta);	//2
		fprintf(fp_vmc6, "%f\t%f\t%f\t", robot->VMC6.LQR[0][2].real_value, robot->VMC6.LQR[0][2].ref_value, robot->VMC6.LQR[0][2].delta);	//3
		fprintf(fp_vmc6, "%f\t%f\t%f\t", robot->VMC6.LQR[0][3].real_value, robot->VMC6.LQR[0][3].ref_value, robot->VMC6.LQR[0][3].delta);	//4
		fprintf(fp_vmc6, "%f\t%f\t%f\t", robot->VMC6.LQR[0][4].real_value, robot->VMC6.LQR[0][4].ref_value, robot->VMC6.LQR[0][4].delta);	//5
		fprintf(fp_vmc6, "%f\t%f\t%f\t", robot->VMC6.LQR[0][5].real_value, robot->VMC6.LQR[0][5].ref_value, robot->VMC6.LQR[0][5].delta);	//6

		fprintf(fp_vmc6, "%f\t%f\t%f\t", robot->VMC6.LQR[1][0].real_value, robot->VMC6.LQR[1][0].ref_value, robot->VMC6.LQR[1][0].delta);	//1
		fprintf(fp_vmc6, "%f\t%f\t%f\t", robot->VMC6.LQR[1][1].real_value, robot->VMC6.LQR[1][1].ref_value, robot->VMC6.LQR[1][1].delta);	//2
		fprintf(fp_vmc6, "%f\t%f\t%f\t", robot->VMC6.LQR[1][2].real_value, robot->VMC6.LQR[1][2].ref_value, robot->VMC6.LQR[1][2].delta);	//3
		fprintf(fp_vmc6, "%f\t%f\t%f\t", robot->VMC6.LQR[1][3].real_value, robot->VMC6.LQR[1][3].ref_value, robot->VMC6.LQR[1][3].delta);	//4
		fprintf(fp_vmc6, "%f\t%f\t%f\t", robot->VMC6.LQR[1][4].real_value, robot->VMC6.LQR[1][4].ref_value, robot->VMC6.LQR[1][4].delta);	//5
		fprintf(fp_vmc6, "%f\t%f\t%f\t", robot->VMC6.LQR[1][5].real_value, robot->VMC6.LQR[1][5].ref_value, robot->VMC6.LQR[1][5].delta);	//6

		fprintf(fp_vmc6, "%f\t%f\t%f\t", real_vrp[0], real_vrp[1], real_vrp[2]); //37,38,39			
		fprintf(fp_vmc6, "%f\t%f\t%f\t", real_dcm[0], real_dcm[1], real_dcm[2]); //40,41,42

		fprintf(fp_vmc6, "%f\t%f\t%f\t", vrp[0], vrp[1], vrp[2]); //43,44,45			
		fprintf(fp_vmc6, "%f\t%f\t%f\t", dcm[0], dcm[1], dcm[2]); //46,47,48

		fprintf(fp_vmc6, "%f\t%f\t%f\t", b_real_vrp[0], b_real_vrp[1], b_real_vrp[2]); //49,50,51			
		fprintf(fp_vmc6, "%f\t%f\t%f\t", b_real_dcm[0], b_real_dcm[1], b_real_dcm[2]); //52,53,54

		fprintf(fp_vmc6, "%f\t%f\t%f\t", b_vrp[0], b_vrp[1], b_vrp[2]); //55,56,57			
		fprintf(fp_vmc6, "%f\t%f\t%f\t", b_dcm[0], b_dcm[1], b_dcm[2]); //58,59,60

		fprintf(fp_vmc6, "%f\t%f\t%f\t", robot->VMC6.PostureAdjust[2].p[3], robot->VMC6.PostureAdjust[2].p[4], robot->VMC6.PostureAdjust[2].p[5]); //61,62,63

		fprintf(fp_vmc6, "%f\t%f\t", robot->VMC6.LQR[0][0].real_value2, robot->VMC6.LQR[0][0].ref_value2); //64,65
		fprintf(fp_vmc6, "%f\t%f\t", robot->VMC6.LQR[0][1].real_value2, robot->VMC6.LQR[0][1].ref_value2); //66,67
		fprintf(fp_vmc6, "%f\t%f\t", robot->VMC6.LQR[0][2].real_value2, robot->VMC6.LQR[0][2].ref_value2); //68,69
		fprintf(fp_vmc6, "%f\t%f\t", robot->VMC6.LQR[0][3].real_value2, robot->VMC6.LQR[0][3].ref_value2); //70,71
		fprintf(fp_vmc6, "%f\t%f\t", robot->VMC6.LQR[0][4].real_value2, robot->VMC6.LQR[0][4].ref_value2); //72,73
		fprintf(fp_vmc6, "%f\t%f\t", robot->VMC6.LQR[0][5].real_value2, robot->VMC6.LQR[0][5].ref_value2); //74,75

		fprintf(fp_vmc6, "%f\t%f\t", robot->VMC6.LQR[1][0].real_value2, robot->VMC6.LQR[1][0].ref_value2); //76,77
		fprintf(fp_vmc6, "%f\t%f\t", robot->VMC6.LQR[1][1].real_value2, robot->VMC6.LQR[1][1].ref_value2); //78,79
		fprintf(fp_vmc6, "%f\t%f\t", robot->VMC6.LQR[1][2].real_value2, robot->VMC6.LQR[1][2].ref_value2); //80,81
		fprintf(fp_vmc6, "%f\t%f\t", robot->VMC6.LQR[1][3].real_value2, robot->VMC6.LQR[1][3].ref_value2); //82,83
		fprintf(fp_vmc6, "%f\t%f\t", robot->VMC6.LQR[1][4].real_value2, robot->VMC6.LQR[1][4].ref_value2); //84,85
		fprintf(fp_vmc6, "%f\t%f\t", robot->VMC6.LQR[1][5].real_value2, robot->VMC6.LQR[1][5].ref_value2); //86,87

		fprintf(fp_vmc6, "%f\t%f\t%f\t", dcog[0], dcog[1], dcog[2]);//88,89,90
		fprintf(fp_vmc6, "%f\t%f\t%f\t", cog[0], cog[1], cog[2]);//91,92,93
		fprintf(fp_vmc6, "%d\t", pc->StateFlag);//94
		fprintf(fp_vmc6, "%f\t", pc->Posture[QBODY].Posture.p[0]);//95
																  //96,97,98,99,100,101
		fprintf(fp_vmc6, "%f\t%f\t%f\t%f\t%f\t%f\t", robot->VMC6.RealBodyPosture.p[0], robot->VMC6.RealBodyPosture.p[1], robot->VMC6.RealBodyPosture.p[2], robot->VMC6.RealBodyPosture.p[3], robot->VMC6.RealBodyPosture.p[4], robot->VMC6.RealBodyPosture.p[5]);
		//102,103,104
		fprintf(fp_vmc6, "%f\t%f\t%f\t", robot->VMC6.InputVRP[0], robot->VMC6.InputVRP[1], robot->VMC6.InputVRP[2]);

		fprintf(fp_vmc6, "%f\t%f\t", robot->ZMP_TPC.LQR[0].ref_value, robot->ZMP_TPC.LQR[1].ref_value);//105,106
		fprintf(fp_vmc6, "%f\t%f\t", robot->ZMP_TPC.LQR[0].real_value, robot->ZMP_TPC.LQR[1].real_value);//107,108
		fprintf(fp_vmc6, "%f\t%f\t", robot->ZMP_TPC.LQR[0].delta, robot->ZMP_TPC.LQR[1].delta);//109,110
		fprintf(fp_vmc6, "%f\t%f\t", robot->VMC6.RealBodyVel[0], robot->VMC6.RealBodyVel[1]);//111,112
		fprintf(fp_vmc6, "\n");
		fflush(fp_vmc6);
	}
#endif

	for (i = 0; i < 6; i++)
	{
		adjust_base_posture.p[i] = pc->Posture[BASE].Posture.p[i] + robot->VMC6.PostureAdjust[2].p[i];
		adjust_base_posture.p[i] = adjust_base_posture.p[i] + robot->VMC6.PostureAdjust[3].p[i];
	}
	adjust_base_t = get_t(&adjust_base_posture);

	//Base Posture Adjust When Hip Change
	if ((robot->GoalState.Posture[LEFT_HIP].IKC_FLAG == HAD_SET) && (robot->GoalState.Posture[RIGHT_HIP].IKC_FLAG == HAD_SET))
	{
		if ((robot->GoalState.Posture[LEFT_KNEE].IKC_FLAG == HAD_SET) && (robot->GoalState.Posture[RIGHT_KNEE].IKC_FLAG == HAD_SET))
		{

			//left
			kt = mt_mul(pc->Posture[LEFT_KNEE].T, get_move(0, 0, KNEE_HEIGHT));
			temp = _rabs(pc->Posture[LEFT_HIP].Posture.p[5] - kt.t[_pz]);
			if (temp > THIGH_LEN)
			{
				temp = THIGH_LEN;
			}
			pc->Posture[LEFT_HIP].Posture.p[3] = kt.t[_px] - _rsqrt(_rpow(THIGH_LEN, 2.0) - _rpow(temp, 2.0));

			//right
			kt = mt_mul(pc->Posture[RIGHT_KNEE].T, get_move(0, 0, KNEE_HEIGHT));
			temp = _rabs(pc->Posture[RIGHT_HIP].Posture.p[5] - kt.t[_pz]);
			if (temp > THIGH_LEN)
			{
				temp = THIGH_LEN;
			}
			pc->Posture[RIGHT_HIP].Posture.p[3] = kt.t[_px] - _rsqrt(_rpow(THIGH_LEN, 2.0) - _rpow(temp, 2.0));


			pc->Posture[LEFT_HIP].T.t[_px] = pc->Posture[LEFT_HIP].Posture.p[3];
			pc->Posture[RIGHT_HIP].T.t[_px] = pc->Posture[RIGHT_HIP].Posture.p[3];
		}
		pc->Posture[BASE].T = mt_mul(pc->Posture[LEFT_HIP].T, get_move(BASE_HIP_X, -BASE_HIP_Y, BASE_HIP_Z));
		ik_posture(pc->Posture[BASE].T.t, pc->Posture[BASE].Posture.p, pc->Posture[BASE].Posture.p);
		robot->GoalState.Posture[BASE].Posture = pc->Posture[BASE].Posture;
		robot->GoalState.Posture[BASE].T = pc->Posture[BASE].T;
	}

	//Hip Posture Adjust
	//if (robot->CurrentState.StateFlag == 1 || robot->CurrentState.StateFlag == 2 || robot->CurrentState.StateFlag == 3)
	{
		pc->Posture[LEFT_HIP].T = mt_mul(adjust_base_t, get_move(-BASE_HIP_X, BASE_HIP_Y, -BASE_HIP_Z));
		pc->Posture[RIGHT_HIP].T = mt_mul(adjust_base_t, get_move(-BASE_HIP_X, -BASE_HIP_Y, -BASE_HIP_Z));
		ik_posture(pc->Posture[LEFT_HIP].T.t, pc->Posture[LEFT_HIP].Posture.p, pc->Posture[LEFT_HIP].Posture.p);
		ik_posture(pc->Posture[RIGHT_HIP].T.t, pc->Posture[RIGHT_HIP].Posture.p, pc->Posture[RIGHT_HIP].Posture.p);
		robot->GoalState.Posture[LEFT_HIP].Posture = pc->Posture[LEFT_HIP].Posture;
		robot->GoalState.Posture[LEFT_HIP].T = pc->Posture[LEFT_HIP].T;
		robot->GoalState.Posture[RIGHT_HIP].Posture = pc->Posture[RIGHT_HIP].Posture;
		robot->GoalState.Posture[RIGHT_HIP].T = pc->Posture[RIGHT_HIP].T;
	}

	//Waist Posture Adjust
	wt = mt_mul(pc->Posture[BASE].T, get_move(0, 0, WAIST_BASE_Z));
	pc->Posture[WAIST].Posture.p[3] = wt.t[_px];
	pc->Posture[WAIST].Posture.p[4] = wt.t[_py];
	pc->Posture[WAIST].Posture.p[5] = wt.t[_pz];
	pc->Posture[WAIST].T.t[_px] = wt.t[_px];
	pc->Posture[WAIST].T.t[_py] = wt.t[_py];
	pc->Posture[WAIST].T.t[_pz] = wt.t[_pz];
	robot->GoalState.Posture[WAIST].Posture = pc->Posture[WAIST].Posture;
	robot->GoalState.Posture[WAIST].T = pc->Posture[WAIST].T;

	//Shoulder Posture Adjust
	pc->Posture[LEFT_SHOULDER].T = mt_mul(pc->Posture[WAIST].T, get_move(-NECK_WAIST_X, SHOULDER_WIDTH / 2.0, NECK_WAIST_Z));
	ik_posture(pc->Posture[LEFT_SHOULDER].T.t, pc->Posture[LEFT_SHOULDER].Posture.p, pc->Posture[LEFT_SHOULDER].Posture.p);
	pc->Posture[RIGHT_SHOULDER].T = mt_mul(pc->Posture[WAIST].T, get_move(-NECK_WAIST_X, -SHOULDER_WIDTH / 2.0, NECK_WAIST_Z));
	ik_posture(pc->Posture[RIGHT_SHOULDER].T.t, pc->Posture[RIGHT_SHOULDER].Posture.p, pc->Posture[RIGHT_SHOULDER].Posture.p);
	robot->GoalState.Posture[LEFT_SHOULDER].Posture = pc->Posture[LEFT_SHOULDER].Posture;
	robot->GoalState.Posture[LEFT_SHOULDER].T = pc->Posture[LEFT_SHOULDER].T;
	robot->GoalState.Posture[RIGHT_SHOULDER].Posture = pc->Posture[RIGHT_SHOULDER].Posture;
	robot->GoalState.Posture[RIGHT_SHOULDER].T = pc->Posture[RIGHT_SHOULDER].T;

	//VRP Posture Adjust
	calulate_vrp_by_com(robot);
	return 0;
}

_rf_flg get_next_state(BRobot * robot)
{
	_rv_flg state;
	static int walking_number = 0;

	printf("Current step %d, Total Step %d\n",walking_number,robot->DCM.Step_Num+1);
	
	switch (robot->CurrentState.StateFlag)
	{
#ifdef STAND_TEST
	case STATE_COM_DOWN:
		state = STATE_DCM_STAND;
		break;

	case STATE_DCM_STAND:
		state = STATE_DCM_STAND;//9;
		break;
#endif
	case STATE_WALK_1ST_STEP://1st ds
		state = STATE_WALK_SWL_SS;
		robot->DCM.Current_Step = 1;
		robot->DCM.Current_Time = 0;
		break;

	case STATE_WALK_SWL_SS://swing left leg ss
		state = STATE_WALK_SWL_DS;
		break;

	case STATE_WALK_SWL_DS://swing left leg ds
		robot->DCM.Current_Step++;
		robot->DCM.Current_Time = 0;
		walking_number++;
		if (walking_number == robot->DCM.Step_Num)
		{
			robot->Squat.StepTime[6] += 1000;
		}
		if (walking_number >= robot->DCM.Step_Num + 1)
		{
			state = STATE_WALK_DCM_STOP;
		}
		else
		{
			state = STATE_WALK_SWR_SS;
		}
		break;

	case STATE_WALK_SWR_SS://swing right leg ss
		state = STATE_WALK_SWR_DS;
		break;

	case STATE_WALK_SWR_DS://swing right leg ds
		robot->DCM.Current_Step++;
		robot->DCM.Current_Time = 0;
		walking_number++;
		if (walking_number == robot->DCM.Step_Num)
		{
			robot->Squat.StepTime[3] += 1000;
		}
		if (walking_number >= robot->DCM.Step_Num+1)
			state = STATE_WALK_DCM_STOP;
		else
			state = STATE_WALK_SWL_SS;
		break;
	case STATE_WALK_DCM_STOP:
		state = STATE_WALK_DCM_STOP;
		break;

	case STATE_NUM:
		state = STATE_NUM;
		break;

	default:
		state = robot->CurrentState.StateFlag + 1;
		break;
	}

	return state;
}

_rf_flg time_init(BRobot * robot)
{
	robot->Squat.StepTime[0] = 2000.0 / TIME_UNIT;	// 1
	robot->Squat.StepTime[1] = 3000.0;// 2
	robot->Squat.StepTime[2] = 2000.0;// 3
	robot->Squat.StepTime[3] = robot->DCM.Step_Time_DS*1000.0;// 4
	robot->Squat.StepTime[4] = robot->DCM.Step_Time_SS*1000.0;// 5
	robot->Squat.StepTime[5] = robot->DCM.Step_Time_DS*1000.0;// 6
	robot->Squat.StepTime[6] = 500.0 / TIME_UNIT;	// 7

	robot->Squat.StepTime[7] = 100000.0 / TIME_UNIT;	// 8

	robot->Squat.StepTime[8] = 2000.0 / TIME_UNIT;	// 9
	robot->Squat.StepTime[9] = 2000.0 / TIME_UNIT;	// 10

	robot->Squat.StepTime[10] = 10000.0 / TIME_UNIT;	// 1

	return 0;
}


_rrm _rf_flg get_dcm_p(double pt, double * dcm_ds_ini, double * ddcm_ds_ini, double * dcm_ds_end, double * ddcm_ds_end, double t, double * dcm, double * ddcm)
{
	MStruct P1, P2, P, Pt;
	MStruct DCM_RES;
	int	i;

	P1 = init_matrix(4, 4);
	P2 = init_matrix(4, 3);
	P = init_matrix(4, 3);
	Pt = init_matrix(2, 4);
	DCM_RES = init_matrix(2, 3);

	P1.m[0][0] = 2.0 / pow(pt, 3.0);	P1.m[0][1] = 1.0 / pow(pt, 2.0);	P1.m[0][2] = -2.0 / pow(pt, 3.0);	P1.m[0][3] = 1.0 / pow(pt, 2.0);
	P1.m[1][0] = -3.0 / pow(pt, 2.0);	P1.m[1][1] = -2.0 / pow(pt, 1.0);	P1.m[1][2] = 3.0 / pow(pt, 2.0);	P1.m[1][3] = -1.0 / pow(pt, 1.0);
	P1.m[2][0] = 0;						P1.m[2][1] = 1;						P1.m[2][2] = 0;						P1.m[2][3] = 0;
	P1.m[3][0] = 1;						P1.m[3][1] = 0;						P1.m[3][2] = 0;						P1.m[3][3] = 0;

	P2.m[0][0] = dcm_ds_ini[0];			P2.m[0][1] = dcm_ds_ini[1];			P2.m[0][2] = dcm_ds_ini[2];
	P2.m[1][0] = ddcm_ds_ini[0];		P2.m[1][1] = ddcm_ds_ini[1];		P2.m[1][2] = ddcm_ds_ini[2];
	P2.m[2][0] = dcm_ds_end[0];			P2.m[2][1] = dcm_ds_end[1];			P2.m[2][2] = dcm_ds_end[2];
	P2.m[3][0] = ddcm_ds_end[0];		P2.m[3][1] = ddcm_ds_end[1];		P2.m[3][2] = ddcm_ds_end[2];

	Pt.m[0][0] = pow(t, 3.0);			Pt.m[0][1] = pow(t, 2.0);			Pt.m[0][2] = pow(t, 1.0);			Pt.m[0][3] = 1;
	Pt.m[1][0] = 3.0*pow(t, 2.0);		Pt.m[1][1] = 2.0*pow(t, 1.0);		Pt.m[1][2] = 1.0;					Pt.m[1][3] = 0;

	P = m_mul(&P1, &P2);
	DCM_RES = m_mul(&Pt, &P);

	for (i = 0; i < 3; i++)
	{
		dcm[i] = DCM_RES.m[0][i];
		ddcm[i] = DCM_RES.m[1][i];
	}

	if (t >= pt)
	{
		for (i = 0; i < 3; i++)
		{
			dcm[i] = dcm_ds_end[i];
			ddcm[i] = ddcm_ds_end[i];
		}
	}
	return 1;
}

void vrp_dcm_track(BRobot * robot, double real_vrp[3], double real_dcm[3], double vrp[3], double dcm[3], double cog[3], double b_real_vrp[3], double b_real_dcm[3], double b_vrp[3], double b_dcm[3])
{
	ViscoelasticModelCompliance6 * vmc6_p = &(robot->VMC6);
	TStruct tempT = UNIT_T;
	PostureStruct tempP = INI_POS;
	TStruct B_P_vrp = UNIT_T;
	TStruct	B_P_dcm = UNIT_T;
	TStruct W_P_vrp = UNIT_T;
	TStruct	W_P_dcm = UNIT_T;
	TStruct W_T_B = UNIT_T;
	TStruct	B_T_W = UNIT_T;
	int i;

	double dk[3] = VRP_DCM_TRACK_TUNE_K;//控制效果系数，越大越猛
	double vdt_k[5];//控制器参数
	static double vdk_x[3] = { 0 };
	static double vdk_dx[3] = { 0 };
	static double vdk_ddx[3] = { 0 };
	static double vdk_dddx[3] = { 0 };
	static double last_vdk_x[3] = { 0 };
	static double last_vdk_dx[3] = { 0 };
	static double last_vdk_ddx[3] = { 0 };
	double limit_max[3] = { 0.03,0.03,0.01 };
	double limit_min[3] = { -0.03,-0.03,-0.01 };
	double T = robot->Squat.inter_time / 1000.0;

	for (i = 0; i < 5; i++)
	{
		vdt_k[i] = robot->dcmSetTab[DCM_SET_VDT_K1 + i];
	}
	if((vmc6_p->SensorFT[0][5]+vmc6_p->SensorFT[1][5])<0.2*ROBOT_WEIGHT)
	{
		//vdt_k[0] = 0;
		//vdt_k[1] = 0;
	}

	// 计算World_T_Body
	W_T_B = vmc6_p->RealBodyT;

	// 计算 Word_P_vrp 与 World_P_dcm
	W_P_vrp.t[0][3] = real_vrp[0]; W_P_vrp.t[1][3] = real_vrp[1]; W_P_vrp.t[2][3] = real_vrp[2];
	W_P_dcm.t[0][3] = real_dcm[0]; W_P_dcm.t[1][3] = real_dcm[1]; W_P_dcm.t[2][3] = real_dcm[2];

	// 计算 B_T_vrp 与 B_T_dcm
	B_T_W = t_inv(&W_T_B);
	B_P_vrp = t_mul(&B_T_W, &W_P_vrp); 
	for (i = 0; i < 3; i++) 
	{ 
		b_real_vrp[i] = B_P_vrp.t[i][3]; 
		//b_real_vrp[i] = real_vrp[i] - vmc6_p->RealBodyPosture.p[i + 3];
		b_vrp[i] = vrp[i] - cog[i]; 
	}
	B_P_dcm = t_mul(&B_T_W, &W_P_dcm); 
	for (i = 0; i < 3; i++) 
	{
		b_real_dcm[i] = B_P_dcm.t[i][3];
		//b_real_dcm[i] = real_dcm[i] - vmc6_p->RealBodyPosture.p[i + 3];
		b_dcm[i] = dcm[i] - cog[i];
	}
	//printf("\n");

	for (i = 0; i < 3; i++)
	{
		last_vdk_x[i] = vdk_x[i];
		last_vdk_dx[i] = vdk_dx[i];
		last_vdk_ddx[i] = vdk_ddx[i];

		vdk_dddx[i] = (-VRP_TRACK_TUNE_K*vdt_k[0] * (b_real_vrp[i] - b_vrp[i]) - DCM_TRACK_TUNE_K* vdt_k[1] * (b_real_dcm[i] - b_dcm[i]))*dk[i] - vdt_k[2] * vdk_x[i] - vdt_k[3] * vdk_dx[i] - vdt_k[4] * vdk_ddx[i];
		vdk_ddx[i] += vdk_dddx[i] * T;
		vdk_dx[i] += vdk_ddx[i] * T;
		vdk_x[i] += vdk_dx[i] * T;

		if (vdk_x[i] > limit_max[i])
		{
			vdk_x[i] = limit_max[i];
			vdk_dx[i] = (vdk_x[i] - last_vdk_x[i]) / T;
			vdk_ddx[i] = (vdk_dx[i] - last_vdk_dx[i]) / T;
			vdk_dddx[i] = (vdk_ddx[i] - last_vdk_ddx[i]) / T;
		}
		else if (vdk_x[i] < limit_min[i])
		{
			vdk_x[i] = limit_min[i];
			vdk_dx[i] = (vdk_x[i] - last_vdk_x[i]) / T;
			vdk_ddx[i] = (vdk_dx[i] - last_vdk_dx[i]) / T;
			vdk_dddx[i] = (vdk_ddx[i] - last_vdk_ddx[i]) / T;
		}
#ifdef USE_VRP_DCM_TRACK
		vmc6_p->PostureAdjust[2].p[i + 3] = (vdk_x[i]+vdk_dx[i]*0.0) * 1000.0;
#endif
	}
	robot->dcmDataTab[DATA_COM_DELTA_X] = vdk_x[0];
	robot->dcmDataTab[DATA_COM_DELTA_Y] = vdk_x[1];
	robot->dcmDataTab[DATA_COM_DELTA_Z] = vdk_x[2];
}

void get_ft_data(BRobot *robot)
{
	ViscoelasticModelCompliance6 * vmc6_p = &(robot->VMC6);
	TStruct tempT = UNIT_T;
	PostureStruct tempP = INI_POS;
	MStruct tempM, tempM1, tempM2;
	int i, j, k;
	double vec_FS[2][3] = { 0.0, 0.0, FOOT_SENSOR / 1000.0 };
	double vec_FfS[2][3];

	// 计算World_T_Body
	tempP = vmc6_p->RealBodyPosture;
	vmc6_p->RealBodyT = get_t(&tempP);

	// 计算 World_T_Foot，左0右1
	vmc6_p->RealFootT[_L_LEFT] = mt_mul(mt_mul(vmc6_p->RealBodyT, get_move(-BASE_HIP_X, BASE_HIP_Y, -BASE_HIP_Z)), get_foot_t(vmc6_p->RealJoint[_L_LEFT]));
	vmc6_p->RealFootPosture[_L_LEFT].p[3] = vmc6_p->RealFootT[_L_LEFT].t[_px];
	vmc6_p->RealFootPosture[_L_LEFT].p[4] = vmc6_p->RealFootT[_L_LEFT].t[_py];
	vmc6_p->RealFootPosture[_L_LEFT].p[5] = vmc6_p->RealFootT[_L_LEFT].t[_pz];

	vmc6_p->RealFootT[_L_RIGHT] = mt_mul(mt_mul(vmc6_p->RealBodyT, get_move(-BASE_HIP_X, -BASE_HIP_Y, -BASE_HIP_Z)), get_foot_t(vmc6_p->RealJoint[_L_RIGHT]));
	vmc6_p->RealFootPosture[_L_RIGHT].p[3] = vmc6_p->RealFootT[_L_RIGHT].t[_px];
	vmc6_p->RealFootPosture[_L_RIGHT].p[4] = vmc6_p->RealFootT[_L_RIGHT].t[_py];
	vmc6_p->RealFootPosture[_L_RIGHT].p[5] = vmc6_p->RealFootT[_L_RIGHT].t[_pz];

	// 计算 F_f_F 与 F_t_F
	for (i = 0; i < 2; i++)
	{
		for (j = 3; j < 6; j++)
		{
			vec_FfS[i][j - 3] = vmc6_p->SensorFT[i][j];
		}
		tempM = cross_3(vec_FS[i], vec_FfS[i]);
		for (j = 0; j < 3; j++)
		{
			vmc6_p->LQR[i][j].real_value = tempM.m[j][0] + vmc6_p->SensorFT[i][j];
			vmc6_p->LQR[i][j + 3].real_value = vmc6_p->SensorFT[i][j + 3];
		}
	}

	// 计算 W_f_F 与 W_t_F
	for (k = 0; k < 2; k++)
	{
		tempM = init_matrix(3, 3);
		tempM1 = init_matrix(3, 1);
		tempM2 = init_matrix(3, 1);
		for (i = 0; i < 3; i++)
		{
			for (j = 0; j < 3; j++)
			{
				tempM.m[i][j] = vmc6_p->RealFootT[k].t[i][j];
			}
			tempM1.m[i][0] = vmc6_p->LQR[k][i].real_value;
			tempM2.m[i][0] = vmc6_p->LQR[k][i + 3].real_value;
		}
		tempM1 = m_mul(&tempM, &tempM1);
		tempM2 = m_mul(&tempM, &tempM2);
		for (j = 0; j < 3; j++)
		{
			vmc6_p->LQR[k][j].real_value = tempM1.m[j][0];
			vmc6_p->LQR[k][j + 3].real_value = tempM2.m[j][0];
		}
	}

	//直接取力传感器数据
#ifdef USE_FT_SENSOR_DATA
	for (i = 0; i < 2; i++)
	{
		for (j = 0; j < 6; j++)
		{
			robot->VMC6.LQR[i][j].real_value = robot->VMC6.SensorFT[i][j];
		}
	}
#endif // USE_FT_SENSOR_DATA

	for (i = 0; i < 6; i++)
	{
		robot->VMC6.LQR_Body_Compliance[i].real_value = robot->VMC6.LQR[0][i].real_value + robot->VMC6.LQR[1][i].real_value;
	}
}

void get_real_ft(BRobot *robot)
{
	int i, j;

	//计算双足受力/力矩变化速度
	for (i = 0; i < 2; i++)
	{
		for (j = 0; j < 6; j++)
		{
			robot->VMC6.LQR[i][j].real_value2 = (robot->VMC6.LQR[i][j].real_value - robot->LAST_VMC6.LQR[i][j].real_value) / robot->VMC6.LQR[i][j].T;
		}
	}

	for (i = 0; i < 6; i++)
	{
		robot->VMC6.LQR_Body_Compliance[i].real_value2 = robot->VMC6.LQR[0][i].real_value2 + robot->VMC6.LQR[1][i].real_value2;
	}
}

void get_ref_ft(BRobot *robot)
{
	int i,j;
	double PLx, PLy, PLz, PRx, PRy, PRz, Cx, Cy, Cz;
	double FRx, FRy, FRz, FLx, FLy, FLz;
	double MRx, MRy, MRz, MLx, MLy, MLz;
	double ddCx, ddCy, ddCz;
	double dHx, dHy, dHz;
	double m, g, J;

	static double delta_ft[6] = {0};
	double d_delta[6];
	double k[6] = {0.5, 0.5, 0, 0, 0, 0};//0.8
	double Tk[3] = { 1772.612427*0.8,532.800633, 10 };
	//double Tk[3] = {15999.1151819855,3970.5772344803,13.6763700288};//R=0.000000
	//double Tk[3] = {39378.1102951763,8176.9555318498,19.6131772956};//R=0.000000

	//double Tk[2] = { 3272.612427,232.800633 };

	static int last_sp_state = WS_NONE;
	
	double fac_n;
	double fac_b;
	double fac;
	double fac_l;
	double fac_r;

	g = G;
	m = ROBOT_WEIGHT;
	J = m*pow(robot->DCM.COM_Height, 2.0);

	PLx = robot->CurrentState.Posture[LEFT_FOOT].Posture.p[3] / 1000.0;
	PLy = robot->CurrentState.Posture[LEFT_FOOT].Posture.p[4] / 1000.0;
	PLz = robot->CurrentState.Posture[LEFT_FOOT].Posture.p[5] / 1000.0;

	PRx = robot->CurrentState.Posture[RIGHT_FOOT].Posture.p[3] / 1000.0;
	PRy = robot->CurrentState.Posture[RIGHT_FOOT].Posture.p[4] / 1000.0;
	PRz = robot->CurrentState.Posture[RIGHT_FOOT].Posture.p[5] / 1000.0;

	Cx = (robot->CurrentState.Posture[BASE].Posture.p[3]) / 1000.0;// +robot->ZMP_TPC.LQR[0].delta;
	Cy = (robot->CurrentState.Posture[BASE].Posture.p[4]) / 1000.0;// +robot->ZMP_TPC.LQR[1].delta;
	Cz = (robot->CurrentState.Posture[BASE].Posture.p[5]) / 1000.0;

	for (i = 0; i < 6; i++)
	{
		robot->CurrentState.Posture[BASE].PostureV.p[i] = (robot->CurrentState.Posture[BASE].Posture.p[i] - robot->PastState.Posture[BASE].Posture.p[i]) / robot->Squat.inter_time;
		robot->CurrentState.Posture[BASE].PostureA.p[i] = (robot->CurrentState.Posture[BASE].PostureV.p[i] - robot->PastState.Posture[BASE].PostureV.p[i]) / robot->Squat.inter_time;
		
		if (fabs(robot->VMC6.RealBodyPosture.p[i]) > RAD(0.0))
		{
			d_delta[i] = -Tk[0] * k[i] * (robot->VMC6.RealBodyPosture.p[i]) - Tk[1] * k[i] * (robot->VMC6.RealBodyVel[i]) - Tk[2]*delta_ft[i];
			delta_ft[i] += d_delta[i] * 0.004;

			delta_ft[i] = -Tk[0] * k[i] * (robot->VMC6.RealBodyPosture.p[i]) - Tk[1] * k[i] * (robot->VMC6.RealBodyVel[i]);
		}
		else
			delta_ft[i] = 0.0;

		if (delta_ft[i] > 500)delta_ft[i] = 500;
		if (delta_ft[i] < -500)delta_ft[i] = -500;
	}

	//角加速度
	dHx = 0.0*J*robot->CurrentState.Posture[BASE].PostureA.p[0] * 1000.0*1000.0;
	dHy = 0.0*J*robot->CurrentState.Posture[BASE].PostureA.p[1] * 1000.0*1000.0;
	dHz = 0.0*J*robot->CurrentState.Posture[BASE].PostureA.p[2] * 1000.0*1000.0;
	//加速度
	ddCx = robot->CurrentState.Posture[BASE].PostureA.p[3] * 1000.0 + robot->ZMP_TPC.LQR[0].dd_delta;
	ddCy = robot->CurrentState.Posture[BASE].PostureA.p[4] * 1000.0 + robot->ZMP_TPC.LQR[1].dd_delta;
	ddCz = robot->CurrentState.Posture[BASE].PostureA.p[5] * 1000.0;

	//Cx = robot->CurrentState.Body_CoM[0];
	//Cy = robot->CurrentState.Body_CoM[1];
	//Cz = robot->CurrentState.Body_CoM[2];
	//ddCx = robot->CurrentState.Body_CoM_A[0];
	//ddCy = robot->CurrentState.Body_CoM_A[1];
	//ddCz = robot->CurrentState.Body_CoM_A[2];

	FRx = (2 * PLy*dHz - 2 * PLz*dHy - 2 * PRy*dHz + 2 * PRz*dHy + 4 * ddCx*m + pow(PLx, 2.0) * ddCx*m + 2 * pow(PLy, 2.0) * ddCx*m + 2 * pow(PLz, 2.0) * ddCx*m + pow(PRx, 2.0) * ddCx*m + 2 * Cx*PLy*ddCy*m + 2 * Cx*PLz*ddCz*m - 2 * Cy*PLy*ddCx*m - 2 * Cz*PLz*ddCx*m - 2 * Cx*PRy*ddCy*m - 2 * Cx*PRz*ddCz*m + 2 * Cy*PRy*ddCx*m + 2 * Cz*PRz*ddCx*m + 2 * Cx*PLz*g*m - 2 * Cx*PRz*g*m - PLx*PLy*ddCy*m - PLx*PLz*ddCz*m - 2 * PLx*PRx*ddCx*m + PLx*PRy*ddCy*m - PLy*PRx*ddCy*m - 2 * PLy*PRy*ddCx*m + PLx*PRz*ddCz*m - PLz*PRx*ddCz*m - 2 * PLz*PRz*ddCx*m + PRx*PRy*ddCy*m + PRx*PRz*ddCz*m - PLx*PLz*g*m + PLx*PRz*g*m - PLz*PRx*g*m + PRx*PRz*g*m) / (2 * (pow(PLx, 2.0) - 2 * PLx*PRx + pow(PLy, 2.0) - 2 * PLy*PRy + pow(PLz, 2.0) - 2 * PLz*PRz + pow(PRx, 2.0) + pow(PRy, 2.0) + pow(PRz, 2.0) + 4));
	FRy = (2 * PLz*dHx - 2 * PLx*dHz + 2 * PRx*dHz - 2 * PRz*dHx + 4 * ddCy*m + 2 * pow(PLx, 2.0) * ddCy*m + pow(PLy, 2.0) * ddCy*m + 2 * pow(PLz, 2.0) * ddCy*m + pow(PRy, 2.0) * ddCy*m - 2 * Cx*PLx*ddCy*m + 2 * Cy*PLx*ddCx*m + 2 * Cy*PLz*ddCz*m - 2 * Cz*PLz*ddCy*m + 2 * Cx*PRx*ddCy*m - 2 * Cy*PRx*ddCx*m - 2 * Cy*PRz*ddCz*m + 2 * Cz*PRz*ddCy*m + 2 * Cy*PLz*g*m - 2 * Cy*PRz*g*m - PLx*PLy*ddCx*m - PLy*PLz*ddCz*m - 2 * PLx*PRx*ddCy*m - PLx*PRy*ddCx*m + PLy*PRx*ddCx*m - 2 * PLy*PRy*ddCy*m + PLy*PRz*ddCz*m - PLz*PRy*ddCz*m - 2 * PLz*PRz*ddCy*m + PRx*PRy*ddCx*m + PRy*PRz*ddCz*m - PLy*PLz*g*m + PLy*PRz*g*m - PLz*PRy*g*m + PRy*PRz*g*m) / (2 * (pow(PLx, 2.0) - 2 * PLx*PRx + pow(PLy, 2.0) - 2 * PLy*PRy + pow(PLz, 2.0) - 2 * PLz*PRz + pow(PRx, 2.0) + pow(PRy, 2.0) + pow(PRz, 2.0) + 4));
	FRz = (2 * PLx*dHy - 2 * PLy*dHx - 2 * PRx*dHy + 2 * PRy*dHx + 4 * ddCz*m + 4 * g*m + 2 * pow(PLx, 2.0) * ddCz*m + 2 * pow(PLy, 2.0) * ddCz*m + pow(PLz, 2.0) * ddCz*m + pow(PRz, 2.0) * ddCz*m + 2 * pow(PLx, 2.0) * g*m + 2 * pow(PLy, 2.0) * g*m + pow(PLz, 2.0) * g*m + pow(PRz, 2.0) * g*m - 2 * Cx*PLx*ddCz*m - 2 * Cy*PLy*ddCz*m + 2 * Cz*PLx*ddCx*m + 2 * Cz*PLy*ddCy*m + 2 * Cx*PRx*ddCz*m + 2 * Cy*PRy*ddCz*m - 2 * Cz*PRx*ddCx*m - 2 * Cz*PRy*ddCy*m - 2 * Cx*PLx*g*m - 2 * Cy*PLy*g*m + 2 * Cx*PRx*g*m + 2 * Cy*PRy*g*m - PLx*PLz*ddCx*m - PLy*PLz*ddCy*m - 2 * PLx*PRx*ddCz*m - PLx*PRz*ddCx*m + PLz*PRx*ddCx*m - 2 * PLy*PRy*ddCz*m - PLy*PRz*ddCy*m + PLz*PRy*ddCy*m - 2 * PLz*PRz*ddCz*m + PRx*PRz*ddCx*m + PRy*PRz*ddCy*m - 2 * PLx*PRx*g*m - 2 * PLy*PRy*g*m - 2 * PLz*PRz*g*m) / (2 * (pow(PLx, 2.0) - 2 * PLx*PRx + pow(PLy, 2.0) - 2 * PLy*PRy + pow(PLz, 2.0) - 2 * PLz*PRz + pow(PRx, 2.0) + pow(PRy, 2.0) + pow(PRz, 2.0) + 4));
	FLx = (2 * PLz*dHy - 2 * PLy*dHz + 2 * PRy*dHz - 2 * PRz*dHy + 4 * ddCx*m + pow(PLx, 2.0) * ddCx*m + pow(PRx, 2.0) * ddCx*m + 2 * pow(PRy, 2.0) * ddCx*m + 2 * pow(PRz, 2.0) * ddCx*m - 2 * Cx*PLy*ddCy*m - 2 * Cx*PLz*ddCz*m + 2 * Cy*PLy*ddCx*m + 2 * Cz*PLz*ddCx*m + 2 * Cx*PRy*ddCy*m + 2 * Cx*PRz*ddCz*m - 2 * Cy*PRy*ddCx*m - 2 * Cz*PRz*ddCx*m - 2 * Cx*PLz*g*m + 2 * Cx*PRz*g*m + PLx*PLy*ddCy*m + PLx*PLz*ddCz*m - 2 * PLx*PRx*ddCx*m - PLx*PRy*ddCy*m + PLy*PRx*ddCy*m - 2 * PLy*PRy*ddCx*m - PLx*PRz*ddCz*m + PLz*PRx*ddCz*m - 2 * PLz*PRz*ddCx*m - PRx*PRy*ddCy*m - PRx*PRz*ddCz*m + PLx*PLz*g*m - PLx*PRz*g*m + PLz*PRx*g*m - PRx*PRz*g*m) / (2 * (pow(PLx, 2.0) - 2 * PLx*PRx + pow(PLy, 2.0) - 2 * PLy*PRy + pow(PLz, 2.0) - 2 * PLz*PRz + pow(PRx, 2.0) + pow(PRy, 2.0) + pow(PRz, 2.0) + 4));
	FLy = (2 * PLx*dHz - 2 * PLz*dHx - 2 * PRx*dHz + 2 * PRz*dHx + 4 * ddCy*m + pow(PLy, 2.0) * ddCy*m + 2 * pow(PRx, 2.0) * ddCy*m + pow(PRy, 2.0) * ddCy*m + 2 * pow(PRz, 2.0) * ddCy*m + 2 * Cx*PLx*ddCy*m - 2 * Cy*PLx*ddCx*m - 2 * Cy*PLz*ddCz*m + 2 * Cz*PLz*ddCy*m - 2 * Cx*PRx*ddCy*m + 2 * Cy*PRx*ddCx*m + 2 * Cy*PRz*ddCz*m - 2 * Cz*PRz*ddCy*m - 2 * Cy*PLz*g*m + 2 * Cy*PRz*g*m + PLx*PLy*ddCx*m + PLy*PLz*ddCz*m - 2 * PLx*PRx*ddCy*m + PLx*PRy*ddCx*m - PLy*PRx*ddCx*m - 2 * PLy*PRy*ddCy*m - PLy*PRz*ddCz*m + PLz*PRy*ddCz*m - 2 * PLz*PRz*ddCy*m - PRx*PRy*ddCx*m - PRy*PRz*ddCz*m + PLy*PLz*g*m - PLy*PRz*g*m + PLz*PRy*g*m - PRy*PRz*g*m) / (2 * (pow(PLx, 2.0) - 2 * PLx*PRx + pow(PLy, 2.0) - 2 * PLy*PRy + pow(PLz, 2.0) - 2 * PLz*PRz + pow(PRx, 2.0) + pow(PRy, 2.0) + pow(PRz, 2.0) + 4));
	FLz = (2 * PLy*dHx - 2 * PLx*dHy + 2 * PRx*dHy - 2 * PRy*dHx + 4 * ddCz*m + 4 * g*m + pow(PLz, 2.0) * ddCz*m + 2 * pow(PRx, 2.0) * ddCz*m + 2 * pow(PRy, 2.0) * ddCz*m + pow(PRz, 2.0) * ddCz*m + pow(PLz, 2.0) * g*m + 2 * pow(PRx, 2.0) * g*m + 2 * pow(PRy, 2.0) * g*m + pow(PRz, 2.0) * g*m + 2 * Cx*PLx*ddCz*m + 2 * Cy*PLy*ddCz*m - 2 * Cz*PLx*ddCx*m - 2 * Cz*PLy*ddCy*m - 2 * Cx*PRx*ddCz*m - 2 * Cy*PRy*ddCz*m + 2 * Cz*PRx*ddCx*m + 2 * Cz*PRy*ddCy*m + 2 * Cx*PLx*g*m + 2 * Cy*PLy*g*m - 2 * Cx*PRx*g*m - 2 * Cy*PRy*g*m + PLx*PLz*ddCx*m + PLy*PLz*ddCy*m - 2 * PLx*PRx*ddCz*m + PLx*PRz*ddCx*m - PLz*PRx*ddCx*m - 2 * PLy*PRy*ddCz*m + PLy*PRz*ddCy*m - PLz*PRy*ddCy*m - 2 * PLz*PRz*ddCz*m - PRx*PRz*ddCx*m - PRy*PRz*ddCy*m - 2 * PLx*PRx*g*m - 2 * PLy*PRy*g*m - 2 * PLz*PRz*g*m) / (2 * (pow(PLx, 2.0) - 2 * PLx*PRx + pow(PLy, 2.0) - 2 * PLy*PRy + pow(PLz, 2.0) - 2 * PLz*PRz + pow(PRx, 2.0) + pow(PRy, 2.0) + pow(PRz, 2.0) + 4));
	MRx = (4 * dHx + pow(PLx, 2.0) * dHx + pow(PRx, 2.0) * dHx + PLx*PLy*dHy + PLx*PLz*dHz - 2 * PLx*PRx*dHx - PLx*PRy*dHy - PLy*PRx*dHy - PLx*PRz*dHz - PLz*PRx*dHz + PRx*PRy*dHy + PRx*PRz*dHz + 4 * Cy*ddCz*m - 4 * Cz*ddCy*m + 4 * Cy*g*m - 2 * PLy*ddCz*m + 2 * PLz*ddCy*m - 2 * PRy*ddCz*m + 2 * PRz*ddCy*m - 2 * PLy*g*m - 2 * PRy*g*m + Cy*pow(PLx, 2.0) * ddCz*m - Cz*pow(PLx, 2.0) * ddCy*m + Cy*pow(PRx, 2.0) * ddCz*m - Cz*pow(PRx, 2.0) * ddCy*m + Cy*pow(PLx, 2.0) * g*m + Cy*pow(PRx, 2.0) * g*m - PLy*pow(PRx, 2.0) * ddCz*m - pow(PLx, 2.0) * PRy*ddCz*m + PLz*pow(PRx, 2.0) * ddCy*m + pow(PLx, 2.0) * PRz*ddCy*m - PLy*pow(PRx, 2.0) * g*m - pow(PLx, 2.0) * PRy*g*m - Cx*PLx*PLy*ddCz*m + Cx*PLx*PLz*ddCy*m - Cy*PLx*PLz*ddCx*m + Cz*PLx*PLy*ddCx*m + Cx*PLx*PRy*ddCz*m + Cx*PLy*PRx*ddCz*m - Cx*PLx*PRz*ddCy*m - Cx*PLz*PRx*ddCy*m - 2 * Cy*PLx*PRx*ddCz*m + Cy*PLx*PRz*ddCx*m + Cy*PLz*PRx*ddCx*m + 2 * Cz*PLx*PRx*ddCy*m - Cz*PLx*PRy*ddCx*m - Cz*PLy*PRx*ddCx*m - Cx*PRx*PRy*ddCz*m + Cx*PRx*PRz*ddCy*m - Cy*PRx*PRz*ddCx*m + Cz*PRx*PRy*ddCx*m - Cx*PLx*PLy*g*m + Cx*PLx*PRy*g*m + Cx*PLy*PRx*g*m - 2 * Cy*PLx*PRx*g*m - Cx*PRx*PRy*g*m + PLx*PLy*PRx*ddCz*m - PLx*PLz*PRx*ddCy*m - PLx*PLy*PRz*ddCx*m + PLx*PLz*PRy*ddCx*m + PLx*PRx*PRy*ddCz*m - PLx*PRx*PRz*ddCy*m + PLy*PRx*PRz*ddCx*m - PLz*PRx*PRy*ddCx*m + PLx*PLy*PRx*g*m + PLx*PRx*PRy*g*m) / (2 * (pow(PLx, 2.0) - 2 * PLx*PRx + pow(PLy, 2.0) - 2 * PLy*PRy + pow(PLz, 2.0) - 2 * PLz*PRz + pow(PRx, 2.0) + pow(PRy, 2.0) + pow(PRz, 2.0) + 4));
	MRy = (4 * dHy + pow(PLy, 2.0) * dHy + pow(PRy, 2.0) * dHy + PLx*PLy*dHx + PLy*PLz*dHz - PLx*PRy*dHx - PLy*PRx*dHx - 2 * PLy*PRy*dHy - PLy*PRz*dHz - PLz*PRy*dHz + PRx*PRy*dHx + PRy*PRz*dHz - 4 * Cx*ddCz*m + 4 * Cz*ddCx*m - 4 * Cx*g*m + 2 * PLx*ddCz*m - 2 * PLz*ddCx*m + 2 * PRx*ddCz*m - 2 * PRz*ddCx*m + 2 * PLx*g*m + 2 * PRx*g*m - Cx*pow(PLy, 2.0) * ddCz*m + Cz*pow(PLy, 2.0) * ddCx*m - Cx*pow(PRy, 2.0) * ddCz*m + Cz*pow(PRy, 2.0) * ddCx*m - Cx*pow(PLy, 2.0) * g*m - Cx*pow(PRy, 2.0) * g*m + PLx*pow(PRy, 2.0) * ddCz*m + pow(PLy, 2.0) * PRx*ddCz*m - PLz*pow(PRy, 2.0) * ddCx*m - pow(PLy, 2.0) * PRz*ddCx*m + PLx*pow(PRy, 2.0) * g*m + pow(PLy, 2.0) * PRx*g*m + Cx*PLy*PLz*ddCy*m + Cy*PLx*PLy*ddCz*m - Cy*PLy*PLz*ddCx*m - Cz*PLx*PLy*ddCy*m + 2 * Cx*PLy*PRy*ddCz*m - Cx*PLy*PRz*ddCy*m - Cx*PLz*PRy*ddCy*m - Cy*PLx*PRy*ddCz*m - Cy*PLy*PRx*ddCz*m + Cy*PLy*PRz*ddCx*m + Cy*PLz*PRy*ddCx*m + Cz*PLx*PRy*ddCy*m + Cz*PLy*PRx*ddCy*m - 2 * Cz*PLy*PRy*ddCx*m + Cx*PRy*PRz*ddCy*m + Cy*PRx*PRy*ddCz*m - Cy*PRy*PRz*ddCx*m - Cz*PRx*PRy*ddCy*m + Cy*PLx*PLy*g*m + 2 * Cx*PLy*PRy*g*m - Cy*PLx*PRy*g*m - Cy*PLy*PRx*g*m + Cy*PRx*PRy*g*m - PLx*PLy*PRy*ddCz*m + PLx*PLy*PRz*ddCy*m - PLy*PLz*PRx*ddCy*m + PLy*PLz*PRy*ddCx*m - PLy*PRx*PRy*ddCz*m - PLx*PRy*PRz*ddCy*m + PLz*PRx*PRy*ddCy*m + PLy*PRy*PRz*ddCx*m - PLx*PLy*PRy*g*m - PLy*PRx*PRy*g*m) / (2 * (pow(PLx, 2.0) - 2 * PLx*PRx + pow(PLy, 2.0) - 2 * PLy*PRy + pow(PLz, 2.0) - 2 * PLz*PRz + pow(PRx, 2.0) + pow(PRy, 2.0) + pow(PRz, 2.0) + 4));
	MRz = (4 * dHz + pow(PLz, 2.0) * dHz + pow(PRz, 2.0) * dHz + PLx*PLz*dHx + PLy*PLz*dHy - PLx*PRz*dHx - PLz*PRx*dHx - PLy*PRz*dHy - PLz*PRy*dHy - 2 * PLz*PRz*dHz + PRx*PRz*dHx + PRy*PRz*dHy + 4 * Cx*ddCy*m - 4 * Cy*ddCx*m - 2 * PLx*ddCy*m + 2 * PLy*ddCx*m - 2 * PRx*ddCy*m + 2 * PRy*ddCx*m + Cx*pow(PLz, 2.0) * ddCy*m - Cy*pow(PLz, 2.0) * ddCx*m + Cx*pow(PRz, 2.0) * ddCy*m - Cy*pow(PRz, 2.0) * ddCx*m - PLx*pow(PRz, 2.0) * ddCy*m - pow(PLz, 2.0) * PRx*ddCy*m + PLy*pow(PRz, 2.0) * ddCx*m + pow(PLz, 2.0) * PRy*ddCx*m - Cx*PLy*PLz*ddCz*m + Cy*PLx*PLz*ddCz*m - Cz*PLx*PLz*ddCy*m + Cz*PLy*PLz*ddCx*m + Cx*PLy*PRz*ddCz*m + Cx*PLz*PRy*ddCz*m - 2 * Cx*PLz*PRz*ddCy*m - Cy*PLx*PRz*ddCz*m - Cy*PLz*PRx*ddCz*m + 2 * Cy*PLz*PRz*ddCx*m + Cz*PLx*PRz*ddCy*m + Cz*PLz*PRx*ddCy*m - Cz*PLy*PRz*ddCx*m - Cz*PLz*PRy*ddCx*m - Cx*PRy*PRz*ddCz*m + Cy*PRx*PRz*ddCz*m - Cz*PRx*PRz*ddCy*m + Cz*PRy*PRz*ddCx*m - Cx*PLy*PLz*g*m + Cy*PLx*PLz*g*m + Cx*PLy*PRz*g*m + Cx*PLz*PRy*g*m - Cy*PLx*PRz*g*m - Cy*PLz*PRx*g*m - Cx*PRy*PRz*g*m + Cy*PRx*PRz*g*m - PLx*PLz*PRy*ddCz*m + PLy*PLz*PRx*ddCz*m + PLx*PLz*PRz*ddCy*m - PLy*PLz*PRz*ddCx*m + PLx*PRy*PRz*ddCz*m - PLy*PRx*PRz*ddCz*m + PLz*PRx*PRz*ddCy*m - PLz*PRy*PRz*ddCx*m - PLx*PLz*PRy*g*m + PLy*PLz*PRx*g*m + PLx*PRy*PRz*g*m - PLy*PRx*PRz*g*m) / (2 * (pow(PLx, 2.0) - 2 * PLx*PRx + pow(PLy, 2.0) - 2 * PLy*PRy + pow(PLz, 2.0) - 2 * PLz*PRz + pow(PRx, 2.0) + pow(PRy, 2.0) + pow(PRz, 2.0) + 4));
	MLx = (4 * dHx + pow(PLx, 2.0) * dHx + pow(PRx, 2.0) * dHx + PLx*PLy*dHy + PLx*PLz*dHz - 2 * PLx*PRx*dHx - PLx*PRy*dHy - PLy*PRx*dHy - PLx*PRz*dHz - PLz*PRx*dHz + PRx*PRy*dHy + PRx*PRz*dHz + 4 * Cy*ddCz*m - 4 * Cz*ddCy*m + 4 * Cy*g*m - 2 * PLy*ddCz*m + 2 * PLz*ddCy*m - 2 * PRy*ddCz*m + 2 * PRz*ddCy*m - 2 * PLy*g*m - 2 * PRy*g*m + Cy*pow(PLx, 2.0) * ddCz*m - Cz*pow(PLx, 2.0) * ddCy*m + Cy*pow(PRx, 2.0) * ddCz*m - Cz*pow(PRx, 2.0) * ddCy*m + Cy*pow(PLx, 2.0) * g*m + Cy*pow(PRx, 2.0) * g*m - PLy*pow(PRx, 2.0) * ddCz*m - pow(PLx, 2.0) * PRy*ddCz*m + PLz*pow(PRx, 2.0) * ddCy*m + pow(PLx, 2.0) * PRz*ddCy*m - PLy*pow(PRx, 2.0) * g*m - pow(PLx, 2.0) * PRy*g*m - Cx*PLx*PLy*ddCz*m + Cx*PLx*PLz*ddCy*m - Cy*PLx*PLz*ddCx*m + Cz*PLx*PLy*ddCx*m + Cx*PLx*PRy*ddCz*m + Cx*PLy*PRx*ddCz*m - Cx*PLx*PRz*ddCy*m - Cx*PLz*PRx*ddCy*m - 2 * Cy*PLx*PRx*ddCz*m + Cy*PLx*PRz*ddCx*m + Cy*PLz*PRx*ddCx*m + 2 * Cz*PLx*PRx*ddCy*m - Cz*PLx*PRy*ddCx*m - Cz*PLy*PRx*ddCx*m - Cx*PRx*PRy*ddCz*m + Cx*PRx*PRz*ddCy*m - Cy*PRx*PRz*ddCx*m + Cz*PRx*PRy*ddCx*m - Cx*PLx*PLy*g*m + Cx*PLx*PRy*g*m + Cx*PLy*PRx*g*m - 2 * Cy*PLx*PRx*g*m - Cx*PRx*PRy*g*m + PLx*PLy*PRx*ddCz*m - PLx*PLz*PRx*ddCy*m - PLx*PLy*PRz*ddCx*m + PLx*PLz*PRy*ddCx*m + PLx*PRx*PRy*ddCz*m - PLx*PRx*PRz*ddCy*m + PLy*PRx*PRz*ddCx*m - PLz*PRx*PRy*ddCx*m + PLx*PLy*PRx*g*m + PLx*PRx*PRy*g*m) / (2 * (pow(PLx, 2.0) - 2 * PLx*PRx + pow(PLy, 2.0) - 2 * PLy*PRy + pow(PLz, 2.0) - 2 * PLz*PRz + pow(PRx, 2.0) + pow(PRy, 2.0) + pow(PRz, 2.0) + 4));
	MLy = (4 * dHy + pow(PLy, 2.0) * dHy + pow(PRy, 2.0) * dHy + PLx*PLy*dHx + PLy*PLz*dHz - PLx*PRy*dHx - PLy*PRx*dHx - 2 * PLy*PRy*dHy - PLy*PRz*dHz - PLz*PRy*dHz + PRx*PRy*dHx + PRy*PRz*dHz - 4 * Cx*ddCz*m + 4 * Cz*ddCx*m - 4 * Cx*g*m + 2 * PLx*ddCz*m - 2 * PLz*ddCx*m + 2 * PRx*ddCz*m - 2 * PRz*ddCx*m + 2 * PLx*g*m + 2 * PRx*g*m - Cx*pow(PLy, 2.0) * ddCz*m + Cz*pow(PLy, 2.0) * ddCx*m - Cx*pow(PRy, 2.0) * ddCz*m + Cz*pow(PRy, 2.0) * ddCx*m - Cx*pow(PLy, 2.0) * g*m - Cx*pow(PRy, 2.0) * g*m + PLx*pow(PRy, 2.0) * ddCz*m + pow(PLy, 2.0) * PRx*ddCz*m - PLz*pow(PRy, 2.0) * ddCx*m - pow(PLy, 2.0) * PRz*ddCx*m + PLx*pow(PRy, 2.0) * g*m + pow(PLy, 2.0) * PRx*g*m + Cx*PLy*PLz*ddCy*m + Cy*PLx*PLy*ddCz*m - Cy*PLy*PLz*ddCx*m - Cz*PLx*PLy*ddCy*m + 2 * Cx*PLy*PRy*ddCz*m - Cx*PLy*PRz*ddCy*m - Cx*PLz*PRy*ddCy*m - Cy*PLx*PRy*ddCz*m - Cy*PLy*PRx*ddCz*m + Cy*PLy*PRz*ddCx*m + Cy*PLz*PRy*ddCx*m + Cz*PLx*PRy*ddCy*m + Cz*PLy*PRx*ddCy*m - 2 * Cz*PLy*PRy*ddCx*m + Cx*PRy*PRz*ddCy*m + Cy*PRx*PRy*ddCz*m - Cy*PRy*PRz*ddCx*m - Cz*PRx*PRy*ddCy*m + Cy*PLx*PLy*g*m + 2 * Cx*PLy*PRy*g*m - Cy*PLx*PRy*g*m - Cy*PLy*PRx*g*m + Cy*PRx*PRy*g*m - PLx*PLy*PRy*ddCz*m + PLx*PLy*PRz*ddCy*m - PLy*PLz*PRx*ddCy*m + PLy*PLz*PRy*ddCx*m - PLy*PRx*PRy*ddCz*m - PLx*PRy*PRz*ddCy*m + PLz*PRx*PRy*ddCy*m + PLy*PRy*PRz*ddCx*m - PLx*PLy*PRy*g*m - PLy*PRx*PRy*g*m) / (2 * (pow(PLx, 2.0) - 2 * PLx*PRx + pow(PLy, 2.0) - 2 * PLy*PRy + pow(PLz, 2.0) - 2 * PLz*PRz + pow(PRx, 2.0) + pow(PRy, 2.0) + pow(PRz, 2.0) + 4));
	MLz = (4 * dHz + pow(PLz, 2.0) * dHz + pow(PRz, 2.0) * dHz + PLx*PLz*dHx + PLy*PLz*dHy - PLx*PRz*dHx - PLz*PRx*dHx - PLy*PRz*dHy - PLz*PRy*dHy - 2 * PLz*PRz*dHz + PRx*PRz*dHx + PRy*PRz*dHy + 4 * Cx*ddCy*m - 4 * Cy*ddCx*m - 2 * PLx*ddCy*m + 2 * PLy*ddCx*m - 2 * PRx*ddCy*m + 2 * PRy*ddCx*m + Cx*pow(PLz, 2.0) * ddCy*m - Cy*pow(PLz, 2.0) * ddCx*m + Cx*pow(PRz, 2.0) * ddCy*m - Cy*pow(PRz, 2.0) * ddCx*m - PLx*pow(PRz, 2.0) * ddCy*m - pow(PLz, 2.0) * PRx*ddCy*m + PLy*pow(PRz, 2.0) * ddCx*m + pow(PLz, 2.0) * PRy*ddCx*m - Cx*PLy*PLz*ddCz*m + Cy*PLx*PLz*ddCz*m - Cz*PLx*PLz*ddCy*m + Cz*PLy*PLz*ddCx*m + Cx*PLy*PRz*ddCz*m + Cx*PLz*PRy*ddCz*m - 2 * Cx*PLz*PRz*ddCy*m - Cy*PLx*PRz*ddCz*m - Cy*PLz*PRx*ddCz*m + 2 * Cy*PLz*PRz*ddCx*m + Cz*PLx*PRz*ddCy*m + Cz*PLz*PRx*ddCy*m - Cz*PLy*PRz*ddCx*m - Cz*PLz*PRy*ddCx*m - Cx*PRy*PRz*ddCz*m + Cy*PRx*PRz*ddCz*m - Cz*PRx*PRz*ddCy*m + Cz*PRy*PRz*ddCx*m - Cx*PLy*PLz*g*m + Cy*PLx*PLz*g*m + Cx*PLy*PRz*g*m + Cx*PLz*PRy*g*m - Cy*PLx*PRz*g*m - Cy*PLz*PRx*g*m - Cx*PRy*PRz*g*m + Cy*PRx*PRz*g*m - PLx*PLz*PRy*ddCz*m + PLy*PLz*PRx*ddCz*m + PLx*PLz*PRz*ddCy*m - PLy*PLz*PRz*ddCx*m + PLx*PRy*PRz*ddCz*m - PLy*PRx*PRz*ddCz*m + PLz*PRx*PRz*ddCy*m - PLz*PRy*PRz*ddCx*m - PLx*PLz*PRy*g*m + PLy*PLz*PRx*g*m + PLx*PRy*PRz*g*m - PLy*PRx*PRz*g*m) / (2 * (pow(PLx, 2.0) - 2 * PLx*PRx + pow(PLy, 2.0) - 2 * PLy*PRy + pow(PLz, 2.0) - 2 * PLz*PRz + pow(PRx, 2.0) + pow(PRy, 2.0) + pow(PRz, 2.0) + 4));

	MRx = 0.0;
	MRy = 0.0;
	MRz = 0.0;
	MLx = 0.0;
	MLy = 0.0;
	MLz = 0.0;

	robot->DCM.Walking_State = robot->CurrentState.SubStateFlag;
	switch (robot->DCM.Walking_State)
	{
	case SR_SUB_STATE_DCM_BIPE_DS:
	//case WS_DS://双脚支撑期
		robot->VMC6.LQR[_L_LEFT][0].ref_value = MLx;
		robot->VMC6.LQR[_L_LEFT][1].ref_value = MLy;
		robot->VMC6.LQR[_L_LEFT][2].ref_value = MLz;
		robot->VMC6.LQR[_L_LEFT][3].ref_value = FLx;
		robot->VMC6.LQR[_L_LEFT][4].ref_value = FLy;
		robot->VMC6.LQR[_L_LEFT][5].ref_value = FLz;

		robot->VMC6.LQR[_L_RIGHT][0].ref_value = MRx;
		robot->VMC6.LQR[_L_RIGHT][1].ref_value = MRy;
		robot->VMC6.LQR[_L_RIGHT][2].ref_value = MRz;
		robot->VMC6.LQR[_L_RIGHT][3].ref_value = FRx;
		robot->VMC6.LQR[_L_RIGHT][4].ref_value = FRy;
		robot->VMC6.LQR[_L_RIGHT][5].ref_value = FRz;

		//for (i = 0; i < 6; i++)
		//{
		//	robot->VMC6.LQR[_L_LEFT][i].ref_value += delta_ft[i] / 2.0;
		//	robot->VMC6.LQR[_L_RIGHT][i].ref_value += delta_ft[i] / 2.0;
		//}
		break;
	case SR_SUB_STATE_DCM_BIPE_LS:
	//case WS_LS://左脚支撑
		robot->VMC6.LQR[_L_LEFT][0].ref_value = MLx + MRx;
		robot->VMC6.LQR[_L_LEFT][1].ref_value = MLy + MRy;
		robot->VMC6.LQR[_L_LEFT][2].ref_value = MLz + MRz;
		robot->VMC6.LQR[_L_LEFT][3].ref_value = FLx + FRx;
		robot->VMC6.LQR[_L_LEFT][4].ref_value = FLy + FRy;
		robot->VMC6.LQR[_L_LEFT][5].ref_value = FLz + FRz;

		robot->VMC6.LQR[_L_RIGHT][0].ref_value = 0;
		robot->VMC6.LQR[_L_RIGHT][1].ref_value = 0;
		robot->VMC6.LQR[_L_RIGHT][2].ref_value = 0;
		robot->VMC6.LQR[_L_RIGHT][3].ref_value = 0;
		robot->VMC6.LQR[_L_RIGHT][4].ref_value = 0;
		robot->VMC6.LQR[_L_RIGHT][5].ref_value = 0;
		
		//for (i = 0; i < 6; i++)
		//{
		//	robot->VMC6.LQR[_L_LEFT][i].ref_value += delta_ft[i];
		//}

		break;
	case SR_SUB_STATE_DCM_BIPE_RS:
	//case WS_RS://右脚支撑
		robot->VMC6.LQR[_L_LEFT][0].ref_value = 0;
		robot->VMC6.LQR[_L_LEFT][1].ref_value = 0;
		robot->VMC6.LQR[_L_LEFT][2].ref_value = 0;
		robot->VMC6.LQR[_L_LEFT][3].ref_value = 0;
		robot->VMC6.LQR[_L_LEFT][4].ref_value = 0;
		robot->VMC6.LQR[_L_LEFT][5].ref_value = 0;

		robot->VMC6.LQR[_L_RIGHT][0].ref_value = MLx + MRx;
		robot->VMC6.LQR[_L_RIGHT][1].ref_value = MLy + MRy;
		robot->VMC6.LQR[_L_RIGHT][2].ref_value = MLz + MRz;
		robot->VMC6.LQR[_L_RIGHT][3].ref_value = FLx + FRx;
		robot->VMC6.LQR[_L_RIGHT][4].ref_value = FLy + FRy;
		robot->VMC6.LQR[_L_RIGHT][5].ref_value = FLz + FRz;

		//for (i = 0; i < 6; i++)
		//{
		//	robot->VMC6.LQR[_L_RIGHT][i].ref_value += delta_ft[i];
		//}

		break;
	default:
		break;
	}

	//姿态控制调整踝关节力矩
	//posture_control_ankle(robot);
	//delta_ft[0] = robot->VMC6.LQR_Posture_Control_Ankle[0].delta;
	//delta_ft[1] = robot->VMC6.LQR_Posture_Control_Ankle[1].delta;

#ifdef TWO_LINK_MODEL_CONTROL	
	delta_ft[0] = robot->TL_Control[0].real_state.Tau.m[0][0];
	delta_ft[1] = robot->TL_Control[1].real_state.Tau.m[0][0];
	//delta_ft[0] = 0.0*delta_ft[0];
	//delta_ft[1] = 0.0*delta_ft[1];;
#endif

	robot->dcmDataTab[DATA_POSTURE_CONTROL_DELTA_X] = delta_ft[0];
	robot->dcmDataTab[DATA_POSTURE_CONTROL_DELTA_Y] = delta_ft[1];

	//Body Compliance
#ifdef USE_BODY_COMPLIANCE_POSTURE_CONTROL
	for (i = 0; i < 3; i++)
	{
		robot->VMC6.LQR_Body_Compliance[i].ref_value = delta_ft[i];
		robot->VMC6.LQR_Body_Compliance[i].ref_value2 = (robot->VMC6.LQR_Body_Compliance[i].ref_value - robot->LAST_VMC6.LQR_Body_Compliance[i].ref_value) / robot->VMC6.LQR_Body_Compliance[i].T;
	}
#endif

	//delta_ft[0] = 0;
	//delta_ft[1] = 0;
	//双足参考力矩调节量分配
	fac_n = 1;
	fac_b = 1.0;
	fac = pow(fabs(robot->VMC6.LQR[_L_LEFT][5].real_value)/ fac_b, fac_n) + pow(fabs(robot->VMC6.LQR[_L_RIGHT][5].real_value) / fac_b, fac_n) + 1;
	fac_l = pow(fabs(robot->VMC6.LQR[_L_LEFT][5].real_value) / fac_b, fac_n);
	fac_r = pow(fabs(robot->VMC6.LQR[_L_RIGHT][5].real_value) / fac_b, fac_n);

#ifdef USE_FOOT_COMPLIANCE_POSTURE_CONTROL
	robot->VMC6.LQR[_L_LEFT][0].ref_value = delta_ft[0]* fac_l/ fac;
	robot->VMC6.LQR[_L_RIGHT][0].ref_value = delta_ft[0] * fac_r / fac;

	robot->VMC6.LQR[_L_LEFT][1].ref_value = delta_ft[1] * fac_l / fac;
	robot->VMC6.LQR[_L_RIGHT][1].ref_value = delta_ft[1] * fac_r / fac;

	//kajita
/* 	if (delta_ft[0] < 0)
	{
		robot->VMC6.LQR[_L_LEFT][0].ref_value = 0;
		robot->VMC6.LQR[_L_RIGHT][0].ref_value = delta_ft[0];
	}
	else
	{
		robot->VMC6.LQR[_L_LEFT][0].ref_value = delta_ft[0];
		robot->VMC6.LQR[_L_RIGHT][0].ref_value = 0;
	} */
#endif

	//双足切换时，力/力矩参考值会有突变，此处防止因为该突变产生一个非常大的力/力矩变化速度
	if (last_sp_state == robot->DCM.Walking_State)
	{
		for (i = 0; i < 2; i++)
		{
			for (j = 0; j < 6; j++)
			{
				robot->VMC6.LQR[i][j].ref_value2 = (robot->VMC6.LQR[i][j].ref_value - robot->LAST_VMC6.LQR[i][j].ref_value) / robot->VMC6.LQR[i][j].T;
			}
		}
	}
	last_sp_state = robot->DCM.Walking_State;
}

void vmc6_update(BRobot *robot)
{
	get_real_ft(robot);//计算实际双足受力
	get_ref_ft(robot);//计算规划双足受力
}

void vmc6_control(BRobot *robot)
{
	ViscoelasticModelCompliance6 *vmc6;
	LQR_Control	*lqr;
	int i, j;
	//double tune_k[6] = { 1, 1.0,0.0, 1,1,1 };
	double tune_k[6] = VMC6_TUNE_K;
	double ex_tune_k = robot->dcmInputTab[DCM_INPUT_VMC6_EX_K];

	double body_tune_k[6] = VMC6_BODY_TUNE_K;
	vmc6 = &(robot->VMC6);

	for (i = 0; i < 2; i++)
	{
#ifdef TWO_LINK_MODEL_FULL_DYNAMICS
		for (j = 3; j < 6; j++)
#else
		for (j = 0; j < 6; j++)
#endif
		{
			lqr = &(vmc6->LQR[i][j]);
			if ((vmc6->SensorFT[0][5] + vmc6->SensorFT[1][5]) < 0.2*ROBOT_WEIGHT)
			{
				//tune_k[j] = 0;
			}
			lqr_control(lqr, ex_tune_k*tune_k[j]);
#ifdef ONLINE_TUNE
			lqr->Td = robot->dcmSetTab[DCM_SET_LEAD_COMP_TIME];
			lqr->Alpha = robot->dcmSetTab[DCM_SET_LEAD_COMP_ALPHA];
#endif
			lead_correction(lqr, &(robot->LAST_VMC6.LQR[i][j]));
#ifdef USE_VMC6_CONTROL
			vmc6->PostureAdjust[i].p[j] = lqr->c_delta;
			if (j >= 3)vmc6->PostureAdjust[i].p[j] = lqr->c_delta*1000.0;
#endif
		}
	}
	
	for (i = 0; i < 6; i++)
	{
		lqr = &(vmc6->LQR_Body_Compliance[i]);
		lqr_control(lqr, body_tune_k[i]);
#ifdef ONLINE_TUNE
		lqr->Td = robot->dcmSetTab[DCM_SET_LEAD_COMP_TIME];
		lqr->Alpha = robot->dcmSetTab[DCM_SET_LEAD_COMP_ALPHA];
#endif
		lead_correction(lqr, &(robot->LAST_VMC6.LQR_Body_Compliance[i]));
#ifdef USE_VMC6_BODY_CONTROL
		vmc6->PostureAdjust[3].p[i] = -lqr->c_delta;
		if (i >= 3)vmc6->PostureAdjust[3].p[i] = -lqr->c_delta*1000.0;
		//printf("%f, ", vmc6->PostureAdjust[3].p[i]);
#endif
	}
	//printf("%f\n",DEG(vmc6->PostureAdjust[3].p[2]));

}


void lqr_control(LQR_Control *lqr,double tune_k)
{
	double ft_err, ft_err2;

	lqr->last_delta = lqr->delta;
	lqr->last_d_delta = lqr->d_delta;

	if (lqr->track_num == 1)
	{
		ft_err = (lqr->real_value - lqr->ref_value);
		lqr->dd_delta = -tune_k * 1 * lqr->k[0] * (lqr->real_value - lqr->ref_value) - lqr->k[1] * (lqr->delta - lqr->ep) - lqr->k[2] * lqr->d_delta;
	}
	if (lqr->track_num == 2)
	{
		ft_err = (lqr->real_value - lqr->ref_value);
		ft_err2 = (lqr->real_value2 - lqr->ref_value2);
		lqr->dd_delta = -tune_k*(lqr->k[0] * ft_err+lqr->k[1]*ft_err2) - lqr->k[2] * (lqr->delta-lqr->ep) - lqr->k[3] * lqr->d_delta;
	}
	lqr->d_delta += lqr->dd_delta*lqr->T;
	lqr->delta += lqr->d_delta*lqr->T;

	if (lqr->delta < lqr->limit[0])
	{
		lqr->delta = lqr->limit[0];
		lqr->d_delta = (lqr->delta - lqr->last_delta) / lqr->T;
		lqr->dd_delta = (lqr->d_delta - lqr->last_d_delta) / lqr->T;
	}
	else if (lqr->delta > lqr->limit[1])
	{
		lqr->delta = lqr->limit[1];
		lqr->d_delta = (lqr->delta - lqr->last_delta) / lqr->T;
		lqr->dd_delta = (lqr->d_delta - lqr->last_d_delta) / lqr->T;
	}
}


void lead_correction(LQR_Control *lqr, LQR_Control * last_lqr)
{
	lqr->c_d_delta = (lqr->c_delta - last_lqr->c_delta) / lqr->T;
	lqr->c_delta = lqr->delta + lqr->Td*lqr->d_delta - lqr->Td*lqr->Alpha*lqr->c_d_delta;
	if (lqr->c_delta < lqr->limit[0])lqr->c_delta = lqr->limit[0];
	if (lqr->c_delta > lqr->limit[1])lqr->c_delta = lqr->limit[1];
}

void calulate_vrp_by_com(BRobot * robot)
{
	int i;
	double com[3];
	double last_com[3];
	double com_v[3];
	double last_com_v[3];
	double com_a[3];

	for (i = 0; i < 3; i++)
	{   
		last_com[i] = (robot->PastState.Posture[BASE].Posture.p[i + 3] + robot->LAST_VMC6.PostureAdjust[2].p[i + 3]) / 1000.0;
		com[i] = (robot->CurrentState.Posture[BASE].Posture.p[i + 3] + robot->VMC6.PostureAdjust[2].p[i + 3])/1000.0;
		//printf("%f\t", com[i]);
		last_com_v[i] = robot->LAST_VMC6.InPutCOMV[i];
		com_v[i] = (com[i] - last_com[i]) / 0.004;
		com_a[i] = (com_v[i] - last_com_v[i]) / 0.004;

		//robot->VMC6.InputVRP[i] = com[i] - (com[2] * com[2])*(com_a[i]);
		robot->VMC6.InputVRP[i] = com[i] - (_rpow(robot->DCM.B,2.0))*(com_a[i]);
		robot->VMC6.InPutCOMV[i] = com_v[i];
	}
	//printf("\n");
}

void dcm_com_est(DCM_COM_Estimator * dce)
{
	MStruct M;
	MStruct A, B, C, D;
	MStruct X, U, Y;
	MStruct temp1, temp2;
	int i, j;
	M = init_matrix(2, 2);
	A = init_matrix(2, 2);
	B = init_matrix(2, 1);
	C = init_matrix(2, 2);
	D = init_matrix(2, 1);
	X = init_matrix(2, 1);
	U = init_matrix(1, 1);
	Y = init_matrix(2, 1);

	U.m[0][0] = dce->input_vrp;
	for (i = 0; i < 2; i++)
	{
		for (j = 0; j < 2; j++)
		{
			M.m[i][j] = dce->M[i][j];
			A.m[i][j] = dce->A[i][j];
			C.m[i][j] = dce->C[i][j];
		}
		B.m[i][0] = dce->B[i];
		D.m[i][0] = dce->D[i];
	}
	X.m[0][0] = dce->state_dcm;
	X.m[1][0] = dce->state_com;
	Y.m[0][0] = dce->output_f;
	Y.m[1][0] = dce->output_ddx;

	// X = AX+BU
	temp1 = m_mul(&A, &X);
	temp2 = m_mul(&B, &U);
	X = m_sum(&temp1, &temp2);

	// X = X + M(Y-CX-DU)
	temp1 = m_mul(&C, &X);
	temp2 = m_mul(&D, &U);
	temp1 = m_sub(&Y, &temp1);
	temp1 = m_sub(&temp1, &temp2);
	temp1 = m_mul(&M, &temp1);
	X = m_sum(&X, &temp1);

	dce->state_dcm = X.m[0][0];
	dce->state_com = X.m[1][0];
}

_rrm void zmp_tpc_update(BRobot *robot, double vrp[3], double cog[3])
{
	TrunkPositionCompliance *tpc = &(robot->ZMP_TPC);
	ViscoelasticModelCompliance6 *vmc6 = &(robot->VMC6);
	TStruct temp_base_foot, temp_foot_zmp, temp_base_zmp;
	int i;	
	double fx, fy, fz, tx, ty, tz;
	double px, py, pz;
	double lfz, rfz;

	//获取参考ZMP
	for (i = 0; i < 2; i++)
	{
		tpc->LQR[i].ref_value = (vrp[i] - cog[i]);
	}

	//获取实际ZMP
	for (i = 0; i < 2; i++)
	{
		//获取单脚坐标系内的ZMP
		tx = vmc6->SensorFT[i][0];
		ty = vmc6->SensorFT[i][1];
		tz = vmc6->SensorFT[i][2];
		fx = vmc6->SensorFT[i][3];
		fy = vmc6->SensorFT[i][4];
		fz = vmc6->SensorFT[i][5];

		px = 0.0;
		py = 0.0;
		pz = FOOT_SENSOR/1000.0;

		if (fz < 0.1*ROBOT_WEIGHT*G / 2.0)
		{
			tpc->FootZMP[i][0] = 0.0;
			tpc->FootZMP[i][1] = 0.0;
		}
		else
		{
			tpc->FootZMP[i][0] = (-ty - pz*fx + px*fz) / fz;
			tpc->FootZMP[i][1] = (tx - pz*fy + py*fz) / fz;
		}

		//单脚坐标系ZMP转化到BASE坐标系下
		temp_base_foot = get_foot_t(vmc6->RealJoint[i]);
		temp_base_foot = mt_mul(get_move(-BASE_HIP_X, ((double)(1 - 2 * i))*BASE_HIP_Y, -BASE_HIP_Z),temp_base_foot);
		temp_foot_zmp = get_move(tpc->FootZMP[i][0] * 1000.0, tpc->FootZMP[i][1] * 1000.0, 0.0);
		temp_base_zmp = t_mul(&temp_base_foot, &temp_foot_zmp);
		tpc->Body_FootZMP[i][0] = temp_base_zmp.t[_px]/1000.0;
		tpc->Body_FootZMP[i][1] = temp_base_zmp.t[_py]/1000.0;
	}
	//获取BASE坐标系下总ZMP坐标
	lfz = vmc6->SensorFT[0][5];
	rfz = vmc6->SensorFT[1][5];
	if (fabs(lfz + rfz)<0.1*ROBOT_WEIGHT*G)
	{
		tpc->LQR[0].real_value = 0.0;
		tpc->LQR[1].real_value = 0.0;
	}
	else
	{
		tpc->LQR[0].real_value = rfz / (lfz + rfz)*tpc->Body_FootZMP[1][0] + lfz / (lfz + rfz)*tpc->Body_FootZMP[0][0];
		tpc->LQR[1].real_value = rfz / (lfz + rfz)*tpc->Body_FootZMP[1][1] + lfz / (lfz + rfz)*tpc->Body_FootZMP[0][1];
	}
}

void zmp_tpc(BRobot *robot)
{
	TrunkPositionCompliance *tpc = &(robot->ZMP_TPC);
	ViscoelasticModelCompliance6 *vmc6 = &(robot->VMC6);
	int i;

	for (i = 0; i < 2; i++)
	{
		lqr_control(&(tpc->LQR[i]), tpc->TuneK[i]);
#ifdef ZMP_BASED_TPC_CONTROL
		vmc6->PostureAdjust[2].p[i+3] = tpc->LQR[i].delta*1000.0;
#endif
	}
	//printf("%f\t%f\t%f\n", tpc->LQR[0].T, tpc->LQR[0].delta, tpc->LQR[1].delta);
}

void posture_control_ankle(BRobot *robot)
{
	int i;
	LQR_Control *p;
	double tune_k[2] = { POSC_ANKLE_ROLL, POSC_ANKLE_PITCH };

	for (i = 0; i < 2; i++)
	{
		p = &(robot->VMC6.LQR_Posture_Control_Ankle[i]);
		p->real_value = robot->VMC6.RealBodyPosture.p[i];
		p->ref_value = 0.0;
		p->real_value2 = robot->VMC6.RealBodyVel[i];
		p->ref_value2 = 0.0;
		lqr_control(p,tune_k[i]);
	}
}

void posture_control_2_link_model(BRobot *robot)
{
	int i,j;
	TwoLinkModelControl *tl;
	MStruct inv_M, inv_Md, temp1, temp2;

	two_link_model_update_Model(robot);

	//导入数据
	for (i = 0; i < 2; i++)
	{
		tl = &(robot->TL_Control[i]);
		tl->real_state.q[1] = tl->adjust_q[1];
		tl->real_state.q[0] = robot->VMC6.RealBodyPosture.p[i] - tl->real_state.q[1];

		tl->real_state.dq[1] = tl->adjust_dq[1];
		tl->real_state.dq[0] = robot->VMC6.RealBodyVel[i] - tl->real_state.dq[1];
	}
	//printf("link 1 roll = %f, pitch = %f, link 2 roll = %f, pitch = %f\n", 
	//	rad2deg(robot->TL_Control[0].real_state.q[0]), rad2deg(robot->TL_Control[1].real_state.q[0]),
	//	rad2deg(robot->TL_Control[0].real_state.q[1]), rad2deg(robot->TL_Control[1].real_state.q[1]));

	//根据反馈控制计算力矩
	two_link_model_get_Tau(robot);


	for (i = 0; i < 2; i++)
	{
		//计算动力学方程系数M，Md, C, Cd
		//two_link_model_get_M
		//two_link_model_get_C
		//robot->TL_Control[i].real_state.
		tl = &(robot->TL_Control[i]);
		tl->real_state.M = two_link_model_get_M(tl->real_state.q[0], tl->real_state.q[1], tl->real_state.dq[0], tl->real_state.dq[1], &tl->Set);
		tl->ref_state.M = two_link_model_get_M(tl->ref_state.q[0], tl->ref_state.q[1], tl->ref_state.dq[0], tl->ref_state.dq[1], &tl->Set);
		tl->real_state.C = two_link_model_get_C(tl->real_state.q[0], tl->real_state.q[1], tl->real_state.dq[0], tl->real_state.dq[1], &tl->Set);
		tl->ref_state.C = two_link_model_get_C(tl->ref_state.q[0], tl->ref_state.q[1], tl->ref_state.dq[0], tl->ref_state.dq[1], &tl->Set);

		//计算角加速度增量
		//delta_ddQ = M^-1*Tau-M^-1*C+Md^-1*Cd
		inv_M = m2_inv(&tl->real_state.M);			//M^-1
		inv_Md = m2_inv(&tl->ref_state.M);			//Md^-1
		temp1 = m_mul(&inv_M, &tl->real_state.Tau);	//M^-1*Tau
		temp2 = m_mul(&inv_M, &tl->real_state.C);	//M^-1 * C
		temp1 = m_sub(&temp1, &temp2);				//M^-1 * Tau - M^-1 * C
		temp2 = m_mul(&inv_Md, &tl->ref_state.C);	//Md^-1 * Cd
		tl->U = m_sum(&temp1, &temp2);				//M^-1 * Tau - M^-1 * C + Md^-1 * Cd

		//计算角度增量
		//Q(k+1) = Ad*Q(k)+Bd*ddQ(k);
		//tl->X = create_zero_matrix(4);
		//tl->X.dim[1] = 1;
		tl->X.m[0][0] = (tl->real_state.q[0] + tl->X.m[0][0])/2;
		tl->X.m[1][0] = (tl->real_state.q[1] + tl->X.m[1][0])/2;
		tl->X.m[2][0] = (tl->real_state.dq[0] + tl->X.m[2][0])/2;
		tl->X.m[3][0] = (tl->real_state.dq[1] + tl->X.m[3][0])/2;
		temp1 = m_mul(&tl->A, &tl->X);
		temp2 = m_mul(&tl->B, &tl->U);
		tl->X = m_sum(&temp1, &temp2);

		for (j = 0; j < 2; j++)
		{
			tl->model_state.ddq[j] = tl->U.m[j][0];
			tl->model_state.dq[j] = tl->X.m[j+2][0];
			tl->model_state.q[j] = tl->X.m[j][0];

			//存入姿态修改值中
			tl->adjust_q[j] = tl->model_state.q[j];
			tl->adjust_dq[j] = tl->model_state.dq[j];
			
#ifndef VREP_SIM
			if(tl->adjust_q[j]>RAD(15.0))tl->adjust_q[j] = RAD(15.0);
			if(tl->adjust_q[j]<RAD(-15.0))tl->adjust_q[j] = RAD(-15.0);
#endif
		}
	}
	//printf("pitch = %f\n", DEG(robot->TL_Control[1].adjust_q[1]));
	//printf("\n");
#ifdef TWO_LINK_MODEL_CONTROL
if(robot->dcmInputTab[DCM_INPUT_2LINK_FLAG] == 1.0)
{	//robot->VMC6.PostureAdjust[3].p[0] = robot->TL_Control[0].adjust_q[1];
	robot->VMC6.PostureAdjust[3].p[1] = robot->TL_Control[1].adjust_q[1];	
#ifdef TWO_LINK_MODEL_FULL_DYNAMICS
	//robot->VMC6.PostureAdjust[0].p[0] = robot->TL_Control[0].adjust_q[0];
	//robot->VMC6.PostureAdjust[1].p[0] = robot->TL_Control[0].adjust_q[0];

	robot->VMC6.PostureAdjust[0].p[1] = robot->TL_Control[1].adjust_q[0];
	robot->VMC6.PostureAdjust[1].p[1] = robot->TL_Control[1].adjust_q[0];
#endif
/* 	robot->VMC6.PostureAdjust[0].p[1] = tl->adjust_q[0];
	robot->VMC6.PostureAdjust[1].p[1] = tl->adjust_q[0]; */
}
#endif

	//printf("%f\n", DEG(robot->CurrentState.Posture[BASE].Posture.p[1]));

	//printf("%f,%f\n", tl->U.m[0][0], tl->U.m[1][0]);
	robot->dcmDataTab[DATA_2LINK_ROLL_REAL_Q1]		= robot->TL_Control[0].real_state.q[0];
	robot->dcmDataTab[DATA_2LINK_ROLL_REAL_Q2]		= robot->TL_Control[0].real_state.q[1];
	robot->dcmDataTab[DATA_2LINK_ROLL_REAL_DQ1]		= robot->TL_Control[0].real_state.dq[0];
	robot->dcmDataTab[DATA_2LINK_ROLL_REAL_DQ2]		= robot->TL_Control[0].real_state.dq[1];
	robot->dcmDataTab[DATA_2LINK_ROLL_TAU1]			= robot->TL_Control[0].real_state.Tau.m[0][0];
	robot->dcmDataTab[DATA_2LINK_ROLL_TAU2]			= robot->TL_Control[0].real_state.Tau.m[1][0];
	robot->dcmDataTab[DATA_2LINK_ROLL_DELTA_DDQ1]	= robot->TL_Control[0].model_state.ddq[0];
	robot->dcmDataTab[DATA_2LINK_ROLL_DELTA_DDQ2]	= robot->TL_Control[0].model_state.ddq[1];
	robot->dcmDataTab[DATA_2LINK_ROLL_DELTA_Q1]		= robot->TL_Control[0].adjust_q[0];
	robot->dcmDataTab[DATA_2LINK_ROLL_DELTA_Q2]		= robot->TL_Control[0].adjust_q[1];
	robot->dcmDataTab[DATA_2LINK_PITCH_REAL_Q1]		= robot->TL_Control[0].real_state.q[0];
	robot->dcmDataTab[DATA_2LINK_PITCH_REAL_Q2]		= robot->TL_Control[0].real_state.q[1];
	robot->dcmDataTab[DATA_2LINK_PITCH_REAL_DQ1]	= robot->TL_Control[0].real_state.dq[0];
	robot->dcmDataTab[DATA_2LINK_PITCH_REAL_DQ2]	= robot->TL_Control[0].real_state.dq[1];
	robot->dcmDataTab[DATA_2LINK_PITCH_TAU1]		= robot->TL_Control[0].real_state.Tau.m[0][0];
	robot->dcmDataTab[DATA_2LINK_PITCH_TAU2]		= robot->TL_Control[0].real_state.Tau.m[0][0];
	robot->dcmDataTab[DATA_2LINK_PITCH_DELTA_DDQ1]	= robot->TL_Control[0].model_state.ddq[0];
	robot->dcmDataTab[DATA_2LINK_PITCH_DELTA_DDQ2]	= robot->TL_Control[0].model_state.ddq[1];
	robot->dcmDataTab[DATA_2LINK_PITCH_DELTA_Q1]	= robot->TL_Control[0].adjust_q[0];
	robot->dcmDataTab[DATA_2LINK_PITCH_DELTA_Q2]	= robot->TL_Control[0].adjust_q[1];
}

void two_link_model_get_Tau(BRobot *robot)
{
	int i;
	static double q1_int_err[2] = {0};
	static double body_int_err[2] = { 0 };
	//double joint2_k[2][2] = {{800.0,20.0},{500.0,20.0}};
	//double joint2_k[2][2] = { { 2000.0,100.0 },{ 2000.0,100.0 } };
	double joint2_k[2][2] = { { 2000.0,100.0 },{ 2000.0,100.0 } };//3000

	for (i = 0; i < 2; i++)
	{
		q1_int_err[i] = q1_int_err[i] + robot->TL_Control[i].real_state.q[0];
		body_int_err[i] += robot->VMC6.RealBodyPosture.p[i];

		robot->TL_Control[i].real_state.Tau = init_matrix0(2, 1);
		robot->TL_Control[i].real_state.Tau.m[0][0] = -2*1772.612427 * (robot->TL_Control[i].real_state.q[0] - 0) - 532.800633 * (robot->TL_Control[i].real_state.dq[0] - 0);
		//robot->TL_Control[i].real_state.Tau.m[1][0] = -1000 * (robot->TL_Control[i].real_state.q[1] - 0) - 100 * (robot->TL_Control[i].real_state.dq[1] - 0) - 500.612427 * (robot->TL_Control[i].real_state.q[0] - 0) - 100.800633 * (robot->TL_Control[i].real_state.dq[0] - 0);

		//robot->TL_Control[i].real_state.Tau.m[0][0] = -2000 * (robot->TL_Control[i].real_state.q[0]) - 100 * (robot->TL_Control[i].real_state.dq[0])-0* q1_int_err[i]  - 0.0 * (robot->ZMP_TPC.LQR[i].real_value - robot->ZMP_TPC.LQR[i].ref_value);
		robot->TL_Control[i].real_state.Tau.m[1][0] = -joint2_k[i][0] * (robot->VMC6.RealBodyPosture.p[i]) - joint2_k[i][1] * (robot->VMC6.RealBodyVel[i]) - 0.000*body_int_err[i] - 00.0 * (robot->ZMP_TPC.LQR[i].real_value - robot->ZMP_TPC.LQR[i].ref_value);

		//printf("%f\n", robot->TL_Control[i].real_state.Tau.m[1][0]);
		robot->TL_Control[i].real_state.Tau.m[0][0] = fmax(robot->TL_Control[i].real_state.Tau.m[0][0], -1000.0);
		robot->TL_Control[i].real_state.Tau.m[0][0] = fmin(robot->TL_Control[i].real_state.Tau.m[0][0], 1000.0);

		robot->TL_Control[i].real_state.Tau.m[1][0] = fmax(robot->TL_Control[i].real_state.Tau.m[1][0], -2000.0);
		robot->TL_Control[i].real_state.Tau.m[1][0] = fmin(robot->TL_Control[i].real_state.Tau.m[1][0], 2000.0);
	}
	//printf("%f\n",robot->TL_Control[1].real_state.Tau.m[1][0]);
}

MStruct two_link_model_get_M(double q1, double q2, double dq1, double dq2, TwoLinkModelSet *set)
{
	MStruct M;
	double I1, I2, L1, L2, m1, m2, s1, s2;

	I1 = set->I[0];		I2 = set->I[1];
	L1 = set->Len[0];		L2 = set->Len[1];
	m1 = set->m[0];		m2 = set->m[1];
	s1 = set->s[0];		s2 = set->s[1];
	//-M = [-I1_2 - I2_2 - L1 ^ 2 * m2 - m1*s1 ^ 2 - m2*s2 ^ 2 - 2 * L1*m2*s2*cos(q2), -I2_2 - m2*s2 ^ 2 - L1*m2*s2*cos(q2);
	//-I2_2 - m2*s2 ^ 2 - L1*m2*s2*cos(q2), -I2_2 - m2*s2 ^ 2];
	M = init_matrix(2, 2);
	M.m[0][0] = -(-I1 - I2 - pow(L1,2) * m2 - m1*s1*s1 - m2*s2*s2 - 2 * L1*m2*s2*cos(q2));
	M.m[0][1] = -(-I2 - m2*s2*s2 - L1*m2*s2*cos(q2));
	M.m[1][0] = -(-I2 - m2*s2*s2 - L1*m2*s2*cos(q2));
	M.m[1][1] = -(-I2 - m2*s2*s2);

	//show_matrix(&M);

	return M;
}

MStruct two_link_model_get_C(double q1, double q2, double dq1, double dq2, TwoLinkModelSet *set)
{
	MStruct C;
	double I1, I2, L1, L2, m1, m2, s1, s2;

	I1 = set->I[0];		I2 = set->I[1];
	L1 = set->Len[0];		L2 = set->Len[1];
	m1 = set->m[0];		m2 = set->m[1];
	s1 = set->s[0];		s2 = set->s[1];
	//C = -g*m2*s2*sin(q1 + q2) - L1*g*m2*sin(q1) - g*m1*s1*sin(q1) - L1*dq2*m2*s2*sin(q2)*(2 * dq1 + dq2)
	//	L1*m2*s2*sin(q2)*dq1 ^ 2 - g*m2*s2*sin(q1 + q2)

	C = init_matrix(2, 1);
	C.m[0][0] = -G*m2*s2*sin(q1 + q2) - L1*G*m2*sin(q1) - G*m1*s1*sin(q1) - L1*dq2*m2*s2*sin(q2)*(2 * dq1 + dq2);
	C.m[1][0] = L1*m2*s2*sin(q2)*dq1*dq1 - G*m2*s2*sin(q1 + q2);
	return C;
}


void two_link_model_update_Model(BRobot *robot)
{
	double p1[3];
	double p2[3];
	TwoLinkModelSet * set1 = &(robot->TL_Control[0].Set);
	TwoLinkModelSet * set2 = &(robot->TL_Control[1].Set);

	p1[0] = 0.5*(robot->CurrentState.Posture[LEFT_FOOT].Posture.p[3] + robot->CurrentState.Posture[LEFT_FOOT].Posture.p[3]) / 1000.0;
	p1[1] = 0.5*(robot->CurrentState.Posture[LEFT_FOOT].Posture.p[4] + robot->CurrentState.Posture[LEFT_FOOT].Posture.p[4]) / 1000.0;
	p1[2] = 0.5*(robot->CurrentState.Posture[LEFT_FOOT].Posture.p[5] + robot->CurrentState.Posture[LEFT_FOOT].Posture.p[5]) / 1000.0;

	p2[0] = robot->CurrentState.Posture[BASE].Posture.p[3] / 1000.0;
	p2[1] = robot->CurrentState.Posture[BASE].Posture.p[4] / 1000.0;
	p2[2] = robot->CurrentState.Posture[BASE].Posture.p[5] / 1000.0;

	set2->Len[0] = sqrt(pow(p1[0] - p2[0], 2.0) + pow(p1[2] - p2[2], 2.0))*0.8;
	set1->Len[0] = sqrt(pow(p1[1] - p2[1], 2.0) + pow(p1[2] - p2[2], 2.0))*0.8;

	set1->s[0] = 0.5*set1->Len[0];			set1->I[0] = 1.0 / 12.0 * set1->m[0] * pow(set1->Len[0], 2.0);
	set2->s[0] = 0.5*set2->Len[0];			set2->I[0] = 1.0 / 12.0 * set2->m[0] * pow(set2->Len[0], 2.0);
}


void body_vmc_update(BRobot *robot)
{
	robot->VMC6.LQR_Body_Compliance[1].ref_value = -20*(robot->ZMP_TPC.LQR[0].real_value - robot->ZMP_TPC.LQR[0].ref_value);
	robot->VMC6.LQR_Body_Compliance[1].ref_value2 = (robot->VMC6.LQR_Body_Compliance[1].ref_value - robot->LAST_VMC6.LQR_Body_Compliance[1].ref_value)/robot->VMC6.LQR_Body_Compliance[1].T;
	
	robot->VMC6.LQR_Body_Compliance[0].ref_value = -20*(robot->ZMP_TPC.LQR[1].real_value - robot->ZMP_TPC.LQR[1].ref_value);
	robot->VMC6.LQR_Body_Compliance[0].ref_value2 = (robot->VMC6.LQR_Body_Compliance[0].ref_value - robot->LAST_VMC6.LQR_Body_Compliance[0].ref_value)/robot->VMC6.LQR_Body_Compliance[0].T;
}


void pid_control(PID_Control *pid, double tune_k)
{
	pid->u = tune_k*(-pid->kp*pid->e - pid->ki*pid->int_e - pid->kd*pid->de);
	pid->u = fmax(pid->u, pid->limit_min); 
	pid->u = fmin(pid->u, pid->limit_max);
}

void single_foot_zmp_update(BRobot *robot)
{
	double k[3] = { -2000,0,0 };//kp,ki,kd
	double limit[2] = { -1000.0,1000.0 };
	ViscoelasticModelCompliance6 * vmc = &(robot->VMC6);
	ViscoelasticModelCompliance6 * last_vmc = &(robot->LAST_VMC6);
	int i, j;

/* 	vmc->PID_Single_Foot_ZMP[0][0].e = fmax(robot->ZMP_TPC.FootZMP[0][0], -FOOT_BACK / 1000.0);
	vmc->PID_Single_Foot_ZMP[0][0].e = fmin(robot->ZMP_TPC.FootZMP[0][0], FOOT_FONT / 1000.0);
	vmc->PID_Single_Foot_ZMP[0][1].e = fmax(robot->ZMP_TPC.FootZMP[0][1], -FOOT_INSIDE / 1000.0);
	vmc->PID_Single_Foot_ZMP[0][1].e = fmin(robot->ZMP_TPC.FootZMP[0][1], FOOT_OUTSIDE / 1000.0);

	vmc->PID_Single_Foot_ZMP[1][0].e = fmax(robot->ZMP_TPC.FootZMP[1][0], -FOOT_BACK / 1000.0);
	vmc->PID_Single_Foot_ZMP[1][0].e = fmin(robot->ZMP_TPC.FootZMP[1][0], FOOT_FONT / 1000.0);
	vmc->PID_Single_Foot_ZMP[1][1].e = fmax(robot->ZMP_TPC.FootZMP[1][1], -FOOT_OUTSIDE / 1000.0);
	vmc->PID_Single_Foot_ZMP[1][1].e = fmin(robot->ZMP_TPC.FootZMP[1][1], FOOT_INSIDE / 1000.0); */


	for (i = 0; i < 2; i++)
	{
		for (j = 0; j < 2; j++)
		{
			vmc->PID_Single_Foot_ZMP[i][j].kp = k[0];
			vmc->PID_Single_Foot_ZMP[i][j].ki = k[1];
			vmc->PID_Single_Foot_ZMP[i][j].kd = k[2];
			vmc->PID_Single_Foot_ZMP[i][j].limit_min = limit[0];
			vmc->PID_Single_Foot_ZMP[i][j].limit_max = limit[1];
			vmc->PID_Single_Foot_ZMP[i][j].e = robot->ZMP_TPC.FootZMP[i][j],
			vmc->PID_Single_Foot_ZMP[i][j].de = (vmc->PID_Single_Foot_ZMP[i][j].e - last_vmc->PID_Single_Foot_ZMP[i][j].e) / vmc->LQR[i][1-j].T;
			vmc->PID_Single_Foot_ZMP[i][j].int_e += vmc->PID_Single_Foot_ZMP[i][j].e;
		}
	}
}

void single_foot_zmp_control(BRobot *robot)
{
	double tune_k[2][2] = { { 1.0,-1.0 },{ 1.0,-1.0 } };
	ViscoelasticModelCompliance6 * vmc = &(robot->VMC6);
	ViscoelasticModelCompliance6 * last_vmc = &(robot->LAST_VMC6);
	int i, j;

	for (i = 0; i < 2; i++)
	{
		for (j = 0; j < 2; j++)
		{
			pid_control(&vmc->PID_Single_Foot_ZMP[i][j], tune_k[i][j]);
#ifdef USE_SINGLE_FOOT_ZMP_CONTROL
			vmc->LQR[i][1-j].ref_value += vmc->PID_Single_Foot_ZMP[i][j].u;
			vmc->LQR[i][1-j].ref_value2 += 0*(vmc->LQR[i][1-j].ref_value - last_vmc->LQR[i][1-j].ref_value) / vmc->LQR[i][1-j].T;
#endif
		}
	}
}

void equilibrium_point_control(BRobot *robot)
{
	static double ep[2][6] = {0.0};
	double epk[6] = EPC_K;
	int i, j;
	double delta_f;
	
	double foot_v[2][6] = { 0.0 };

	for (i = 0; i < 2; i++)
	{
		for (j = 0; j < 6; j++)
		{
			foot_v[i][j] = (robot->VMC6.RealFootPosture[i].p[j] - robot->LAST_VMC6.RealFootPosture[i].p[j]) / robot->VMC6.LQR[i][j].T;
		}
		delta_f = robot->VMC6.LQR[i][5].real_value - robot->VMC6.LQR[i][5].ref_value;
		if (fabs(delta_f) > 0.5*ROBOT_WEIGHT*G)
			ep[i][5] = _lsign(delta_f)*0.09-0.0000015*robot->VMC6.LQR[i][5].real_value*foot_v[i][5];
		else
			ep[i][5] += -0.1*ep[i][5];
	}
	//printf("%f,%f\n", ep[0][5], ep[1][5]);

#ifdef USE_EQUILIBRIUM_POINT_CONTROL
	for (i = 0; i < 2; i++)
	{
		for (j = 0; j < 6; j++)
		{
			robot->VMC6.LQR[i][j].ep = epk[j]*ep[i][j];
		}
	}
#endif
}