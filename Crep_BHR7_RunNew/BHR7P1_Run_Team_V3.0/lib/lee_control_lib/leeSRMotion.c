#include "leeSRMotion.h"
#include <stdio.h>

void sr_motion(BRobot *robot, DCMWalkingState *dcm)
{
	double ini_base[6] = { 0 };
	double ini_joint[21] = { 0 };
	int i1;
	//初始化设置
	if (robot->Init_Flag != HAD_SET)
	{
		#ifndef VREP_SIM
		ini_joint[LEFT_LEG_3] = RAD(-10);
		ini_joint[LEFT_LEG_4] = RAD(20);
		ini_joint[LEFT_LEG_5] = RAD(-10);
		ini_joint[RIGHT_LEG_3] = RAD(-10);
		ini_joint[RIGHT_LEG_4] = RAD(20);
		ini_joint[RIGHT_LEG_5] = RAD(-10);
		ini_base[5] = FOOT_HEIGHT + CRUS_LEN*_rcos(RAD(10)) + THIGH_LEN*_rcos(RAD(10)) + BASE_HIP_Z;
		#else
		ini_base[5] = FOOT_HEIGHT + CRUS_LEN + THIGH_LEN + BASE_HIP_Z;
		#endif
		ini_joint[LEFT_ARM_3] = RAD(-50.0);
		ini_joint[RIGHT_ARM_3] = RAD(-50.0);
		dcm_walk_init(robot);
		two_link_model_init(robot);
		dcm_init(robot);
		vmc6_init(robot);
		zmp_tpc_init(robot);
		robot->VMC6.RealBodyPosture.p[5] = ini_base[5] / 1000.0;//设置初始Body/Base位置
		robot_init(robot);
		robot->CurrentState.StateFlag = SR_STATE_COM_DOWN;
		robot->CurrentState.SubStateFlag = SR_SUB_STATE_NONE;
		robot->Squat.inter_time = robot->dcmSetTab[DCM_SET_SYS_CIRCLE] * 1000.0;
		for (i1 = 0; i1 < 6; i1++)
		{
			robot->CurrentState.Posture[BASE].Posture.p[i1] = ini_base[i1];
		}
		for (i1 = 0; i1 < JOINT_NUMBER; i1++)
		{
			robot->CurrentState.Joint[i1] = ini_joint[i1];
			robot->CurrentState.O_Joint[i1] = ini_joint[i1];
		}
		robot->CurrentState.Posture[BASE].T = get_t(&(robot->CurrentState.Posture[BASE].Posture));
		update_current(robot);
		odw_init_set(dcm, robot);
		dcm_com_est_init(robot);
		init_filters(robot);
		init_estimators(robot);
		sr_arm_vmc_init(robot);
		BW_IMU_init(robot);
	}

	//执行
	load_data(robot);
	//规划与控制更新
	robot->PastState = robot->CurrentState;
	update_current(robot);
	//时间累加
	robot->CurrentState.Time += robot->Squat.inter_time;
	sr_get_next(robot, dcm);

	for (i1 = 0; i1 < 3; i1++)
	{
		//printf("[%f, %f] ",robot->CurrentState.Posture[BASE].Posture.p[i1+3], dcm->cog[i1]);
		//printf("[%f, %f] ", robot->CurrentState.Posture[LEFT_FOOT].Posture.p[i1 + 3], dcm->foot[0][i1]);
	}
	//printf("%d\n",dcm->Walking_State);

	//输出关节角
	for (i1 = 0; i1 < 6; i1++)
	{
		robot->dcmOutputTab[DCM_OUTPUT_JOINT_L1 + i1] = robot->CurrentState.Joint[LEFT_LEG_1 + i1];
		robot->dcmOutputTab[DCM_OUTPUT_JOINT_R1 + i1] = robot->CurrentState.Joint[RIGHT_LEG_1 + i1];
	}
	for (i1 = 0; i1 < 3; i1++)
	{
		robot->dcmOutputTab[DCM_OUTPUT_ARM_L1 + i1] = robot->CurrentState.Joint[LEFT_ARM_1 + i1];
		robot->dcmOutputTab[DCM_OUTPUT_ARM_R1 + i1] = robot->CurrentState.Joint[RIGHT_ARM_1 + i1];
		robot->dcmOutputTab[DCM_OUTPUT_WAIST_1 + i1] = robot->CurrentState.Joint[WAIST_1 + i1];
	}

	//腿2,6关节加上QBody补偿
	// robot->dcmOutputTab[DCM_OUTPUT_JOINT_L2] -= QBODY_LEFT_2*dcm->qbody;
	// robot->dcmOutputTab[DCM_OUTPUT_JOINT_L6] -= QBODY_LEFT_6*dcm->qbody;
	// robot->dcmOutputTab[DCM_OUTPUT_JOINT_R2] -= QBODY_RIGHT_2*dcm->qbody;
	// robot->dcmOutputTab[DCM_OUTPUT_JOINT_R6] -= QBODY_RIGHT_6*dcm->qbody;
	robot->dcmOutputTab[DCM_OUTPUT_JOINT_L2] -= QBODY_LEFT_2*robot->CurrentState.Posture[QBODY].Posture.p[0];
	robot->dcmOutputTab[DCM_OUTPUT_JOINT_L6] -= QBODY_LEFT_6*robot->CurrentState.Posture[QBODY].Posture.p[0];
	robot->dcmOutputTab[DCM_OUTPUT_JOINT_R2] -= QBODY_RIGHT_2*robot->CurrentState.Posture[QBODY].Posture.p[0];
	robot->dcmOutputTab[DCM_OUTPUT_JOINT_R6] -= QBODY_RIGHT_6*robot->CurrentState.Posture[QBODY].Posture.p[0];
	
	//printf("%f,\n",DEG(robot->CurrentState.Posture[QBODY].Posture.p[0]));
	//摔倒手臂控制
	//if (robot->CurrentState == SR_STATE_FALL)
	{
		robot->dcmOutputTab[DCM_OUTPUT_ARM_L1] += robot->SR_Arm_Control[0].delta_joint[0];
		robot->dcmOutputTab[DCM_OUTPUT_ARM_L2] += robot->SR_Arm_Control[0].delta_joint[1];
		robot->dcmOutputTab[DCM_OUTPUT_ARM_L3] += robot->SR_Arm_Control[0].delta_joint[2];

		robot->dcmOutputTab[DCM_OUTPUT_ARM_R1] += robot->SR_Arm_Control[1].delta_joint[0];
		robot->dcmOutputTab[DCM_OUTPUT_ARM_R2] += robot->SR_Arm_Control[1].delta_joint[1];
		robot->dcmOutputTab[DCM_OUTPUT_ARM_R3] += robot->SR_Arm_Control[1].delta_joint[2];
	}

	//输出结束判断标志（系统当前状态）
	robot->dcmOutputTab[DCM_OUTPUT_STATE_FLAG] = (double)(robot->CurrentState.StateFlag);

	if (robot->CurrentState.StateFlag == 0)
	{
		robot->Init_Flag = NOT_SET;
	}
}

void sr_get_next(BRobot *robot, DCMWalkingState *dcm)
{	
	BRobotState *pc;

	pc = &(robot->CurrentState);
	pc->Time2 += robot->Squat.inter_time;
	//对应状态规划与控制
	switch (pc->StateFlag)
	{
	case SR_STATE_COM_DOWN:
		sr_com_down(robot);
		break;

	case SR_STATE_DCM_BIPE:
		sr_dcm_bipe(robot, dcm);
		break;

	case SR_STATE_FALL:
		sr_fall(robot);
		break;

	case SR_STATE_RECOVER:
		sr_recover(robot);
		break;

	case SR_STATE_CONTROL:
		lee_control(robot, dcm);
		break;

	default:
		break;
	}
	//生成运动轨迹
	sr_adjust(robot,dcm);
	get_joint(robot);
	//状态切换
	sr_state_trans(robot,dcm);
}

//大状态切换
void sr_state_trans(BRobot *robot, DCMWalkingState *dcm)
{
	BRobotState *pc;

	pc = &(robot->CurrentState);
	switch (pc->StateFlag)
	{
	case SR_STATE_COM_DOWN:
		//时间触发切换
		if (pc->Time2 >= robot->Squat.StepTime[SR_STATE_COM_DOWN - 1])
		{
			ikc_flag_reset(robot);
			pc->Time2 = 0.0;
			pc->StateFlag = SR_STATE_DCM_BIPE;
			pc->SubStateFlag = SR_SUB_STATE_DCM_BIPE_DS;
			odw_init_set(dcm, robot);
			dcm->DCM_Set.StepSet[0].Walking_State = ODW_STATE_STAND; //ODW_STATE_1ST_STEP;//
		}
		//对外部命令无响应
		robot->Command = CMD_NONE;
		break;

	case SR_STATE_DCM_BIPE:
		//外部事件触发切换
		switch(robot->Command)
		{
		case CMD_FALL:
			ikc_flag_reset(robot);
			pc->Time2 = 0.0;
			pc->StateFlag = SR_STATE_FALL;
			robot->Command = CMD_NONE;
			break;
		case CMD_DCM_STAND:
		case CMD_DCM_WALK:
			break;
		default://对其他命令无响应
			robot->Command = CMD_NONE;
			break;
		}
		break;

	case SR_STATE_FALL:
		//外部事件触发切换
		if ((robot->Command == CMD_RECOVER) && (pc->SubStateFlag == SR_SUB_STATE_FALLEN))
		{
			sr_arm_vmc_end(robot);
			ikc_flag_reset(robot);
			pc->Time2 = 0.0;
			pc->StateFlag = SR_STATE_RECOVER;
			pc->SubStateFlag = SR_SUB_STATE_RECOVERING;
			robot->Command = CMD_NONE;
		}
		else//其他命令无响应
		{
			robot->Command = CMD_NONE;
		}
		if(pc->SubStateFlag == SR_SUB_STATE_FALLEN)
		{
			//pc->StateFlag = 0;
		}
		break;

	case SR_STATE_RECOVER:
		//外部事件触发切换
		if (pc->SubStateFlag == SR_SUB_STATE_RECOVERED)
		{
			/*if (robot->Command == CMD_DCM_STAND)
			{
				pc->Time2 = 0.0;
				pc->StateFlag = SR_STATE_DCM_BIPE;
				pc->SubStateFlag = SR_SUB_STATE_DCM_BIPE_DS;
				odw_init_set(dcm, robot);
				dcm->DCM_Set.StepSet[0].Walking_State = ODW_STATE_STAND;
				robot->Command = CMD_NONE;
			}
			else if (robot->Command == CMD_DCM_WALK)
			{
				pc->Time2 = 0.0;
				pc->StateFlag = SR_STATE_DCM_BIPE;
				pc->SubStateFlag = SR_SUB_STATE_DCM_BIPE_DS;
				odw_init_set(dcm, robot);
				dcm->DCM_Set.StepSet[0].Walking_State = ODW_STATE_1ST_STEP;
				robot->Command = CMD_NONE;
			}
			else
			{
				robot->Command = CMD_NONE;
			}*/
			ikc_flag_reset(robot);
			pc->Time2 = 0.0;
			pc->StateFlag = SR_STATE_COM_DOWN;
			pc->SubStateFlag = SR_SUB_STATE_NONE;
			odw_init_set(dcm, robot);
			vmc6_init(robot);
			zmp_tpc_init(robot);
			two_link_model_init(robot);
			robot->Command = CMD_NONE;
		}
		else//其他命令无响应
		{
			robot->Command = CMD_NONE;
		}
		break;
	case SR_STATE_CONTROL:
		//无状态切换，对任何命令也不响应
		robot->Command = CMD_NONE;
		break;
	default:
		pc->StateFlag = 0;
		break;
	}
}

//下蹲
void sr_com_down(BRobot *robot)
{
	BRobotState *p, *pc;
	static double dcog_z = 0.0;
	static double ldfoot_z = 0.0;
	static double rdfoot_z = 0.0;

	p = &(robot->GoalState);
	pc = &(robot->CurrentState);
	//插值计算
	robot->MotionFlag = CARTESIAN_MOVE;
	realtime_1D_interpolation(&(pc->Posture[BASE].Posture.p[5]), &dcog_z, robot->dcmSetTab[DCM_SET_WALK_HEIGHT] * 1000.0, 0.0, pc->Time2, robot->Squat.StepTime[SR_STATE_COM_DOWN - 1], robot->Squat.inter_time);
	realtime_1D_interpolation(&(pc->Posture[LEFT_FOOT].Posture.p[5]), &(ldfoot_z), 0.0, 0.0, pc->Time2, robot->Squat.StepTime[SR_STATE_COM_DOWN - 1], robot->Squat.inter_time);
	realtime_1D_interpolation(&(pc->Posture[RIGHT_FOOT].Posture.p[5]), &(rdfoot_z), 0.0, 0.0, pc->Time2, robot->Squat.StepTime[SR_STATE_COM_DOWN - 1], robot->Squat.inter_time);
	REMAIN_FOOT;
}

//行走
void sr_dcm_bipe(BRobot *robot, DCMWalkingState *dcm)
{
	int i;
	double last_cog[3];
	double last_dcog[3];
	BRobotState *p, *pc;
	p = &(robot->GoalState);
	pc = &(robot->CurrentState);
	REMAIN_FOOT;

	if (dcm->init_flag != HAD_SET)odw_init_set(dcm,robot);
	if (dcm->DCM_Set.StepSet[0].Walking_State != ODW_STATE_STAND)//行走
	{
		if (robot->Command == CMD_DCM_STAND)
		{
			dcm->DCM_Set.Plan_Step_Num = dcm->Current_Step 
				+ 3 * (dcm->Current_Step == 0)
				+ 2 * ((dcm->Current_Step > 0) && (dcm->Current_Step < (dcm->DCM_Set.Plan_Step_Num - 1)))
				+ 1 * (dcm->Current_Step == (dcm->DCM_Set.Plan_Step_Num - 1));
		}
		dcm->DCM_Set.StepSet[0].Step_Width = robot->dcmSetTab[DCM_SET_STEP_WIDTH];
		dcm->DCM_Set.StepSet[0].Step_Length = robot->dcmSetTab[DCM_SET_STEP_LEN];
		realtime_dcm_walk(dcm);
		robot->CurrentState.Posture[QBODY].Posture.p[0] = dcm->qbody;
		//判断支撑脚
		if (dcm->Current_Time < dcm->DCM_Set.StepSet[0].Step_Time_SS)
		{
			if(dcm->DCM_Set.StepSet[0].Walking_State == ODW_STATE_SWL)
				pc->SubStateFlag = SR_SUB_STATE_DCM_BIPE_RS;
			else
				pc->SubStateFlag = SR_SUB_STATE_DCM_BIPE_LS;
		}
		else
		{
			pc->SubStateFlag = SR_SUB_STATE_DCM_BIPE_DS;
		}
		//自动切换到站立 
		if (dcm->Current_Step >= dcm->DCM_Set.Plan_Step_Num)
		{
			dcm->DCM_Set.StepSet[0].Walking_State = ODW_STATE_STAND;
			pc->SubStateFlag = SR_SUB_STATE_DCM_BIPE_DS;
		}
	}
	else//站立
	{
		//计算质心速度，积分计算质心位置
		for (i = 0; i < 3; i++)
		{
			last_dcog[i] = dcm->dcog[i];
			dcm->dcog[i] = -1.0 / dcm->DCM_Set.StepSet[0].B*(dcm->cog[i] - dcm->dcm[i]);
			dcm->cog[i] += 0.5*(last_dcog[i] + dcm->dcog[i])*dcm->DCM_Set.inter_time;//梯形积分公式
		}
		//外部事件触发站立到行走的转换
		if (robot->Command == CMD_DCM_WALK)
		{
			pc->SubStateFlag = SR_SUB_STATE_DCM_BIPE_DS;
			for (i = 0; i < 3; i++)
			{
				last_cog[i] = dcm->cog[i];
				last_dcog[i] = dcm->dcog[i];
			}
			odw_init_set(dcm, robot);
			dcm->DCM_Set.StepSet[0].Walking_State = ODW_STATE_1ST_STEP;
			for (i = 0; i < 3; i++)
			{
				dcm->cog[i] = last_cog[i];
				dcm->dcog[i] = last_dcog[i];
			}
		}
	}
	for (i = 0; i < 3; i++)
	{
		pc->Posture[BASE].Posture.p[i + 3] = dcm->cog[i] * 1000.0;
		pc->Posture[LEFT_FOOT].Posture.p[i + 3] = dcm->foot[_L_LEFT][i] * 1000.0;
		pc->Posture[RIGHT_FOOT].Posture.p[i + 3] = dcm->foot[_L_RIGHT][i] * 1000.0;
	}
	pc->Posture[BASE].T = get_t(&(pc->Posture[BASE].Posture));
	pc->Posture[LEFT_FOOT].T = get_t(&(pc->Posture[LEFT_FOOT].Posture));
	pc->Posture[RIGHT_FOOT].T = get_t(&(pc->Posture[RIGHT_FOOT].Posture));

	if (dcm->Current_Time >= dcm->DCM_Set.StepSet[0].Step_Time)
	{
		dcm->Current_Time = 0.0;
		dcm->Current_Step += 1;
		dcm->DCM_Set.LastStepSet = dcm->DCM_Set.StepSet[0];
	}
}

//摔倒
void sr_fall(BRobot *robot)
{
	int i;
	double goal_joint[JOINT_NUMBER] = { 0.0 };
	double Fall_Time = 1.5; //s
	double Arm_Time = 0.8; //s

	robot->MotionFlag = THIRD_MOVE;
	goal_joint[LEFT_LEG_3] = -RAD(70);
	goal_joint[LEFT_LEG_4] = RAD(140);
	goal_joint[LEFT_LEG_5] = -RAD(90);

	goal_joint[RIGHT_LEG_3] = -RAD(70);
	goal_joint[RIGHT_LEG_4] = RAD(140);
	goal_joint[RIGHT_LEG_5] = -RAD(90);

	goal_joint[WAIST_2] = RAD(10);//10

	sr_arm_vmc_update(robot);

	//腿腰规划
	if (robot->CurrentState.Time2 < (Fall_Time*1000.0))
	{
		robot->CurrentState.SubStateFlag = SR_SUB_STATE_FALLING;
		for (i = 0; i < JOINT_NUMBER;i++)
		{
			#ifdef USE_FALL_ARM_CONTROL
			if ((i >= LEFT_ARM_1) && (i <= LEFT_ARM_3))continue;
			if ((i >= RIGHT_ARM_1) && (i <= RIGHT_ARM_3))continue;
			#endif
			 realtime_1D_interpolation(
				 &(robot->CurrentState.Joint[i]), &(robot->CurrentState.JointV[i]),
				 goal_joint[i], 0.0, robot->CurrentState.Time2/1000.0, Fall_Time, robot->Squat.inter_time/1000.0
			 );
			//realtime_1D_interpolation_5(
			//	&(robot->CurrentState.Joint[i]), &(robot->CurrentState.JointV[i]), &(robot->CurrentState.JointA[i]),
			//	goal_joint[i], 0.0, 0.0, robot->CurrentState.Time2/1000.0, Fall_Time, robot->Squat.inter_time/1000.0
			//);
		}
		realtime_1D_interpolation(
				 &(robot->CurrentState.Posture[QBODY].Posture.p[0]), &(robot->CurrentState.Posture[QBODY].PostureV.p[0]),
				 0.0, 0.0, robot->CurrentState.Time2/1000.0, Fall_Time, robot->Squat.inter_time/1000.0);
	}
	else
	{
		robot->CurrentState.SubStateFlag = SR_SUB_STATE_FALLEN;
	}

	#ifdef USE_FALL_ARM_CONTROL
	//手臂规划
	goal_joint[LEFT_ARM_1] = -RAD(40.0);	//40.0
	goal_joint[LEFT_ARM_2] = RAD(20.0);		//20.0
	goal_joint[LEFT_ARM_3] = -RAD(60.0);	//60.0
	goal_joint[RIGHT_ARM_1] = -RAD(40.0);	//40.0
	goal_joint[RIGHT_ARM_2] = -RAD(20.0);	//20.0
	goal_joint[RIGHT_ARM_3] = -RAD(60.0);	//60.0
	if (robot->CurrentState.Time2 < (Arm_Time*1000.0))
	{
		robot->GoalState.Posture[LEFT_HAND].IKC_FLAG = NOT_SET;
		robot->GoalState.Posture[RIGHT_HAND].IKC_FLAG = NOT_SET;
		for (i = 0; i < 3; i++)
		{
			 realtime_1D_interpolation(
				&(robot->CurrentState.Joint[LEFT_ARM_1+i]), &(robot->CurrentState.JointV[LEFT_ARM_1 + i]),
				goal_joint[LEFT_ARM_1 + i], 0.0, robot->CurrentState.Time2 / 1000.0, Arm_Time, robot->Squat.inter_time / 1000.0
			);
			realtime_1D_interpolation(
				&(robot->CurrentState.Joint[RIGHT_ARM_1 + i]), &(robot->CurrentState.JointV[RIGHT_ARM_1 + i]),
				goal_joint[RIGHT_ARM_1 + i], 0.0, robot->CurrentState.Time2 / 1000.0, Arm_Time, robot->Squat.inter_time / 1000.0
			); 
			//realtime_1D_interpolation_5(
			//	&(robot->CurrentState.Joint[LEFT_ARM_1+i]), &(robot->CurrentState.JointV[LEFT_ARM_1 + i]),&(robot->CurrentState.JointA[LEFT_ARM_1 + i]),
			//	goal_joint[LEFT_ARM_1 + i], 0.0,0.0, robot->CurrentState.Time2 / 1000.0, Arm_Time, robot->Squat.inter_time / 1000.0
			//);
			//realtime_1D_interpolation_5(
			//	&(robot->CurrentState.Joint[RIGHT_ARM_1 + i]), &(robot->CurrentState.JointV[RIGHT_ARM_1 + i]),&(robot->CurrentState.JointA[RIGHT_ARM_1 + i]),
			//	goal_joint[RIGHT_ARM_1 + i], 0.0, 0.0, robot->CurrentState.Time2 / 1000.0, Arm_Time, robot->Squat.inter_time / 1000.0
			//);
		}
	}
	else
	{
		goal_joint[LEFT_ARM_1] = -RAD(100.0);
		goal_joint[LEFT_ARM_2] = RAD(60.0);
		goal_joint[LEFT_ARM_3] = -RAD(60.0);
		goal_joint[RIGHT_ARM_1] = -RAD(100.0);
		goal_joint[RIGHT_ARM_2] = -RAD(60.0);
		goal_joint[RIGHT_ARM_3] = -RAD(60.0);
		/*if (robot->CurrentState.Time2 < (2*1000.0))
		{
			for (i = 0; i < 3; i++)
			{
				realtime_1D_interpolation(
					&(robot->CurrentState.Joint[LEFT_ARM_1 + i]), &(robot->CurrentState.JointV[LEFT_ARM_1 + i]),
					goal_joint[LEFT_ARM_1 + i], 0.0, (robot->CurrentState.Time2 / 1000.0-Arm_Time), 2-Arm_Time, robot->Squat.inter_time / 1000.0
				);
				realtime_1D_interpolation(
					&(robot->CurrentState.Joint[RIGHT_ARM_1 + i]), &(robot->CurrentState.JointV[RIGHT_ARM_1 + i]),
					goal_joint[RIGHT_ARM_1 + i], 0.0, (robot->CurrentState.Time2 / 1000.0-Arm_Time), 2-Arm_Time, robot->Squat.inter_time / 1000.0
				);
			}
		}*/
		//笛卡尔空间位置保持-简单控制，效果不太好
		//sr_hand_falling_control(robot);
		//robot->GoalState.Posture[LEFT_HAND].IKC_FLAG = HAD_SET;
		//robot->GoalState.Posture[RIGHT_HAND].IKC_FLAG = HAD_SET;

		//采用基于笛卡尔空间虚拟弹簧阻尼模型与关节空间粘弹性模型的手臂控制策略
		sr_arm_vmc_control(robot);
	}
	#endif
}

//恢复
void sr_recover(BRobot *robot)
{
	int step_number = 4;
	double goal_joint[4][JOINT_NUMBER] = {
		//la1   la2   la3   ll1   ll2   ll3   ll4   ll5   ll6   ra1   ra2   ra3   rl1   rl2   rl3   rl4   rl5   rl6   wy   wp   wr    
		//{ 10,   0,    -135, 0,    0,    -70,  150,  -90,  0,    10,   0,    -135, 0,    0,    -70,  150,  -90,  0,    0,   44,  0 },
		//{ -15,  0,    -135, 0,    0,    -70,  150,  -90,  0,    -15,  0,    -135, 0,    0,    -70,  150,  -90,  0,    0,   44,  0 },
		//{ -70,  0,    -5,   0,    0,    -70,  150,  -90,  0,    -70,  0,    -5,   0,    0,    -70,  150,  -90,  0,    0,   44,  0 },
		//{ -30,  0,    -5,   0,    0,    -50,  150,  -90,  0,    -30,  0,    -5,   0,    0,    -50,  150,  -90,  0,    0,   0,   0 },
		//{ 0,    0,    -50,  0,    0,    -40,  80,   -40,  0,     0,   0,    -50,  0,    0,    -40,  80,   -40,  0,    0,   0,   0 }

		//{ -20,   0,    -135, 0,    0,    -70,  150,  -90,  0,    -20,   0,    -135, 0,    0,    -70,  150,  -90,  0,    0,   0,  0 },
		//{ -20,  0,    -135, 0,    0,    -70,  100,  -90,  0,    -20,  0,    -135, 0,    0,    -70,  100,  -90,  0,    0,   44,  0 },
		//{ -70,  0,    -5,   0,    0,    -70,  150,  -90,  0,    -70,  0,    -5,   0,    0,    -70,  150,  -90,  0,    0,   44,  0 },
		//{ -30,  0,    -5,   0,    0,    -50,  150,  -90,  0,    -30,  0,    -5,   0,    0,    -50,  150,  -90,  0,    0,   0,   0 },
		//{ 0,    0,    -50,  0,    0,    -10,  20,   -10,  0,     0,   0,    -50,  0,    0,    -10,  20,   -10,  0,    0,   0,   0 }

		{ -20,   0,    -135, 0,    0,    -70,  110,  -90,  0,    -20,   0,    -125, 0,    0,    -70,  110,  -90,  0,    0,   0,  0 },
		//{ -20,  0,    -135, 0,    0,    -70,  100,  -90,  0,    -20,  0,    -135, 0,    0,    -70,  100,  -90,  0,    0,   0,  0 },
		{ -70,  0,    -5,   0,    0,    -70,  150,  -90,  0,    -70,  0,    -5,   0,    0,    -70,  150,  -90,  0,    0,   44,  0 },
		{ -30,  0,    -5,   0,    0,    -50,  150,  -90,  0,    -30,  0,    -5,   0,    0,    -50,  150,  -90,  0,    0,   0,   0 },
		{ 0,    0,    -50,  0,    0,    -10,  20,   -10,  0,     0,   0,    -50,  0,    0,    -10,  20,   -10,  0,    0,   0,   0 }
	};
	//double goal_time[6] = {0,5,9,14,19,22};
	double goal_time[5] = { 5-5,9-5,14-5,19-5,22-5 };
	int i, j;
	double now_time = robot->CurrentState.Time2 / 1000.0;
	robot->MotionFlag = THIRD_MOVE;
	for (i = 0; i < step_number; i++)
	{
		for (j = 0; j < JOINT_NUMBER; j++)
		{
			goal_joint[i][j] = RAD(goal_joint[i][j]);
			if ((now_time > goal_time[i]) && (now_time< goal_time[i + 1]))
			{
				realtime_1D_interpolation(&robot->CurrentState.Joint[j], &robot->CurrentState.JointV[j], goal_joint[i][j], 0.0, now_time - goal_time[i], goal_time[i+1]- goal_time[i], robot->Squat.inter_time / 1000.0);
			}
			if (now_time == goal_time[i + 1])
			{
				robot->CurrentState.Joint[j] = goal_joint[i][j];
			}
		}
	}
	if (now_time >= goal_time[step_number])
	{
		robot->CurrentState.SubStateFlag = SR_SUB_STATE_RECOVERED;
	}
	else
	{
		robot->CurrentState.SubStateFlag = SR_SUB_STATE_RECOVERING;
	}
}

void lee_control(BRobot * robot, DCMWalkingState * dcm)
{
}

//运动轨迹调整
void sr_adjust(BRobot *robot, DCMWalkingState *dcm)
{
	BRobotState * pc;
	BRobotState * pp;
	BRobotState * pg;
	TStruct wt;
	PostureStruct adjust_base_posture;
	TStruct adjust_base_t;
	double real_vrp[3];
	double real_dcm[3];

	int i;
	
	double T = robot->Squat.inter_time / 1000.0;

#ifdef VREP_SIM
	static FILE * fp_dcm;
	static FILE * fp_vmc6;
#endif
	pc = &(robot->CurrentState);
	pp = &(robot->PastState);
	pg = &(robot->GoalState);

	//根据多连杆模型计算由规划轨迹得到的实际质心位置
	get_body_com(robot);

	//粘弹性控制与平衡控制
	vmc6_update(robot);//更新控制所需要的信息
	zmp_tpc_update(robot, dcm->vrp, dcm->cog);
	single_foot_zmp_update(robot);
	if (pc->StateFlag == SR_STATE_DCM_BIPE || pc->StateFlag == SR_STATE_CONTROL)//行走与站立测试时开启控制;控制器模式时开启控制
	{
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
			real_vrp[i] = (robot->VMC6.RealBodyPosture.p[i + 3] - (robot->VMC6.LQR[0][i + 3].real_value + robot->VMC6.LQR[1][i + 3].real_value - ((double)(i == 2))*ROBOT_WEIGHT*G)*pow(dcm->DCM_Set.StepSet[0].B, 2.0) / ROBOT_WEIGHT);
			//根据加速度计计算实际VRP点（世界坐标系）
			//real_vrp[i] = robot->VMC6.RealBodyPosture.p[i + 3] - pow(dcm->DCM_Set.StepSet[0].B, 2.0)*robot->VMC6.RealBodyAcc[i + 3];

			//根据加速度计计算实际DCM点（世界坐标系）
			//real_dcm[i] = (robot->VMC6.RealBodyPosture.p[i + 3] + dcm->DCM_Set.StepSet[0].B*(robot->VMC6.RealBodyVel[i + 3]));
			real_dcm[i] = robot->VMC6.EstDCM[i];
		}

		//记录VRP点、DCM点信息
		//robot->dcmDataTab[DATA_REAL_B_VRP_X] = b_real_vrp[0];
		//robot->dcmDataTab[DATA_REAL_B_VRP_Y] = b_real_vrp[1];
		//robot->dcmDataTab[DATA_REAL_B_VRP_Z] = b_real_vrp[2];
		//robot->dcmDataTab[DATA_REAL_B_DCM_X] = b_real_dcm[0];
		//robot->dcmDataTab[DATA_REAL_B_DCM_Y] = b_real_dcm[1];
		//robot->dcmDataTab[DATA_REAL_B_DCM_Z] = b_real_dcm[2];
		//robot->dcmDataTab[DATA_REF_B_VRP_X] = b_vrp[0];
		//robot->dcmDataTab[DATA_REF_B_VRP_Y] = b_vrp[1];
		//robot->dcmDataTab[DATA_REF_B_VRP_Z] = b_vrp[2];
		robot->dcmDataTab[DATA_LEFT_FOOT_DELTA_Z] = robot->VMC6.LQR[0][5].delta;
		robot->dcmDataTab[DATA_RIGHT_FOOT_DELTA_Z] = robot->VMC6.LQR[1][5].delta;
		robot->dcmDataTab[DATA_REAL_VRP_X] = real_vrp[0];
		robot->dcmDataTab[DATA_REAL_VRP_Y] = real_vrp[1];
		robot->dcmDataTab[DATA_REAL_VRP_Z] = real_vrp[2];
		robot->dcmDataTab[DATA_REF_VRP_X] = dcm->vrp[0];
		robot->dcmDataTab[DATA_REF_VRP_Y] = dcm->vrp[1];
		robot->dcmDataTab[DATA_REF_VRP_Z] = dcm->vrp[2];
		robot->dcmDataTab[DATA_REF_DCM_X] = dcm->dcm[0];
		robot->dcmDataTab[DATA_REF_DCM_Y] = dcm->dcm[1];
		robot->dcmDataTab[DATA_REF_DCM_Z] = dcm->dcm[2];
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

		fprintf(fp_vmc6, "%f\t%f\t%f\t", dcm->vrp[0], dcm->vrp[1], dcm->vrp[2]); //43,44,45			
		fprintf(fp_vmc6, "%f\t%f\t%f\t", dcm->dcm[0], dcm->dcm[1], dcm->dcm[2]); //46,47,48

		fprintf(fp_vmc6, "%f\t%f\t%f\t", dcm->vrp[0], dcm->vrp[1], dcm->vrp[2]); //49,50,51			
		fprintf(fp_vmc6, "%f\t%f\t%f\t", dcm->dcm[0], dcm->dcm[1], dcm->dcm[2]); //52,53,54

		fprintf(fp_vmc6, "%f\t%f\t%f\t", dcm->vrp[0], dcm->vrp[1], dcm->vrp[2]); //55,56,57			
		fprintf(fp_vmc6, "%f\t%f\t%f\t", dcm->dcm[0], dcm->dcm[1], dcm->dcm[2]); //58,59,60

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

		fprintf(fp_vmc6, "%f\t%f\t%f\t", dcm->dcog[0], dcm->dcog[1], dcm->dcog[2]);//88,89,90
		fprintf(fp_vmc6, "%f\t%f\t%f\t", dcm->cog[0], dcm->cog[1], dcm->cog[2]);//91,92,93
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

		fprintf(fp_vmc6, "%f\t%f\t", robot->VMC6.RawBodyPosture.p[0], robot->VMC6.RawBodyPosture.p[1]);//113,114
		fprintf(fp_vmc6, "%f\t%f\t", robot->VMC6.RawBodyVel[0], robot->VMC6.RawBodyVel[1]);//115,116

		fprintf(fp_vmc6, "%f\t%f\t", robot->SR_Arm_Control[0].h, robot->SR_Arm_Control[1].h);//117,118
		fprintf(fp_vmc6, "%f\t%f\t", robot->SR_Arm_Control[0].Fw.m[2][0], robot->SR_Arm_Control[1].Fw.m[2][0]);//119,120
		fprintf(fp_vmc6, "%f\t%f\t%f\t", robot->SR_Arm_Control[0].Fa.m[0][0], robot->SR_Arm_Control[0].Fa.m[1][0], robot->SR_Arm_Control[0].Fa.m[2][0]);//121,122,123
		fprintf(fp_vmc6, "%f\t%f\t%f\t", robot->SR_Arm_Control[1].Fa.m[0][0], robot->SR_Arm_Control[1].Fa.m[1][0], robot->SR_Arm_Control[1].Fa.m[2][0]);//124,125,126
		fprintf(fp_vmc6, "%f\t%f\t%f\t", robot->SR_Arm_Control[0].Torque[0], robot->SR_Arm_Control[0].Torque[1], robot->SR_Arm_Control[0].Torque[2]);//127,128,129
		fprintf(fp_vmc6, "%f\t%f\t%f\t", robot->SR_Arm_Control[1].Torque[0], robot->SR_Arm_Control[1].Torque[1], robot->SR_Arm_Control[1].Torque[2]);//130,131,132


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
}

void sr_hand_falling_control(BRobot *robot)
{
	double Ground;
	double Hand_Height[2];
	double Safe_Distance = 0.05;//5cm
	PostureStruct body_pos;
	TStruct body_t, knee_t[2], hand_t[2];

	//获取髋姿态
	body_pos = robot->CurrentState.Posture[BASE].Posture;
	body_pos.p[0] = 0.0*robot->VMC6.RealBodyPosture.p[0];
	body_pos.p[1] = robot->VMC6.RealBodyPosture.p[1];
	body_pos.p[2] = 0.0;
	body_t = get_t(&body_pos);

	//1. 获取世界坐标系下手相对于膝盖（假设地面）的位置
	//1.1 获取世界坐标系下膝盖位置
	knee_t[0] = mt_mul(body_t, mt_mul(t_inv(&robot->CurrentState.Posture[BASE].T), robot->CurrentState.Posture[LEFT_KNEE].T));
	knee_t[1] = mt_mul(body_t, mt_mul(t_inv(&robot->CurrentState.Posture[BASE].T), robot->CurrentState.Posture[RIGHT_KNEE].T));
	//1.2 获取世界坐标系下手的位置
	hand_t[0] = mt_mul(body_t, mt_mul(t_inv(&robot->CurrentState.Posture[BASE].T), robot->CurrentState.Posture[LEFT_HAND].T));
	hand_t[1] = mt_mul(body_t, mt_mul(t_inv(&robot->CurrentState.Posture[BASE].T), robot->CurrentState.Posture[RIGHT_HAND].T));
	//1.3 获取假定地面高度
	Ground = fmin(knee_t[0].t[_pz], knee_t[1].t[_pz]);
	Hand_Height[0] = (hand_t[0].t[_pz] - Ground) / 1000.0 - 0.0*Safe_Distance;
	Hand_Height[1] = (hand_t[1].t[_pz] - Ground) / 1000.0 - 0.0*Safe_Distance;

	//2. 计算关节运动轨迹
	//2.1 位置滤波与保护计算
	//Hand_Height[0] = fmax(Hand_Height[0], 0.0);
	//Hand_Height[1] = fmax(Hand_Height[1], 0.0);
	//Hand_Height[0] = filt_data(&robot->FilterHandPos[0], Hand_Height[0]);
	//Hand_Height[1] = filt_data(&robot->FilterHandPos[1], Hand_Height[1]);
	printf("%f\n", Hand_Height[0]);
	//2.2 规划值转换到机器人坐标系下
	//if (Hand_Height[0] < Safe_Distance)
	{
		Hand_Height[0] += -0.01*(Hand_Height[0]- Safe_Distance);
	}
	hand_t[0].t[_pz] = (Hand_Height[0] + 0.0*Safe_Distance)*1000.0 + Ground;
	robot->CurrentState.Posture[LEFT_HAND].T = mt_mul(robot->CurrentState.Posture[BASE].T, mt_mul(t_inv(&body_t), hand_t[0]));
	ik_posture(robot->CurrentState.Posture[LEFT_HAND].T.t, robot->CurrentState.Posture[LEFT_HAND].Posture.p, robot->CurrentState.Posture[LEFT_HAND].Posture.p);

	//if (Hand_Height[1] < Safe_Distance)
	{
		Hand_Height[1] += -0.01*(Hand_Height[1]- Safe_Distance);
	}
	hand_t[1].t[_pz] = (Hand_Height[1] + 0.0*Safe_Distance)*1000.0 + Ground;
	robot->CurrentState.Posture[RIGHT_HAND].T = mt_mul(robot->CurrentState.Posture[BASE].T, mt_mul(t_inv(&body_t), hand_t[1]));
	ik_posture(robot->CurrentState.Posture[RIGHT_HAND].T.t, robot->CurrentState.Posture[LEFT_HAND].Posture.p, robot->CurrentState.Posture[RIGHT_HAND].Posture.p);
	//printf("%f,%f,%f, ", t_inv(&body_t).t[_px], t_inv(&body_t).t[_py], t_inv(&body_t).t[_pz]);
}


void sr_arm_vmc_init(BRobot *robot)
{
	ArmJointVMC * pa;
	int i, j;
	double vmc_k[4] = { 
		//-0.3732416040, -0.0005596609, 190.3103471362, 27.6102124236 // 1.0000000000
		//- 0.0397983739,-0.0000596761,59.3846019032,12.6150690796	// 20.0000000000
		- 0.0103245031,-0.0000154812,29.3370871056,8.3153604745	// 100
		//- 0.0024744180,-0.0000037103,13.9180826994,5.5087911830 // 500
	};	
	double delta_limit_min[3] = { -RAD(70.0),-RAD(50.0), -RAD(50.0) };
	double delta_limit_max[3] = { RAD(50.0), RAD(50.0),  RAD(50.0) };
	for (i = 0; i < 2; i++)
	{
		pa = &robot->SR_Arm_Control[i];
		pa->h = 0.0;
		pa->dh = 0.0;
		pa->H = 0.06;//距地面5cm,开始作用//0.15
		pa->K = 200000.0;
		pa->B = 100000.0;
		for (j = 0; j < 3; j++)
		{
			pa->delta_joint[j] = 0;
			pa->Torque[j] = 0;
			pa->LQR[j].track_num = 2;
			pa->LQR[j].T = robot->dcmSetTab[DCM_SET_SYS_CIRCLE];
			pa->LQR[j].ref_value = 0.0;
			pa->LQR[j].real_value = 0.0;
			pa->LQR[j].ref_value2 = 0.0;
			pa->LQR[j].real_value2 = 0.0;
			pa->LQR[j].delta = 0.0;
			pa->LQR[j].d_delta = 0.0;
			pa->LQR[j].dd_delta = 0.0;
			pa->LQR[j].last_delta = 0.0;
			pa->LQR[j].last_d_delta = 0.0;
			pa->LQR[j].limit[0] = delta_limit_min[j];
			pa->LQR[j].limit[1] = delta_limit_max[j];
			pa->LQR[j].k[0] = vmc_k[0];
			pa->LQR[j].k[1] = vmc_k[1];
			pa->LQR[j].k[2] = vmc_k[2];
			pa->LQR[j].k[3] = vmc_k[3];
			pa->LQR[j].Alpha = robot->dcmSetTab[DCM_SET_LEAD_COMP_ALPHA];//LEAD_CORRECTION_RATE;
			pa->LQR[j].Td = robot->dcmSetTab[DCM_SET_LEAD_COMP_TIME];//LEAD_CORRECTION_TIME;
			pa->LQR[j].c_delta = 0;
			pa->LQR[j].c_d_delta = 0;
			pa->LQR[j].ep = 0;
		}
		pa->J = init_matrix0(3, 3);
		pa->Fa = init_matrix0(3, 1); 
		pa->Fw = init_matrix0(3, 1);
	}
}

void sr_arm_vmc_update(BRobot *robot)
{
	ArmJointVMC * pa;
	PostureStruct body_pos;
	TStruct body_t, knee_t[2], hand_t[2];
	TStruct shoulder_t[2];
	MStruct temp, temp1;
	int i, j;
	double Ground = 0.0;
	double q1, q2, q3, L2_1z, c1z;
	double last_torque[3];
	double arm_joint[2][3];

	//--获取body姿态
	body_pos = robot->CurrentState.Posture[BASE].Posture;
	body_pos.p[0] = 0.0*robot->VMC6.RealBodyPosture.p[0];
	body_pos.p[1] = robot->VMC6.RealBodyPosture.p[1];
	body_pos.p[2] = 0.0;
	body_t = get_t(&body_pos);
	//--1. 获取世界坐标系下手相对于膝盖（假设地面）的位置
	//----1.1 获取世界坐标系下膝盖位置
	knee_t[0] = mt_mul(body_t, mt_mul(t_inv(&robot->CurrentState.Posture[BASE].T), robot->CurrentState.Posture[LEFT_KNEE].T));
	knee_t[1] = mt_mul(body_t, mt_mul(t_inv(&robot->CurrentState.Posture[BASE].T), robot->CurrentState.Posture[RIGHT_KNEE].T));
	//----1.2 获取世界坐标系下肩膀的位置
	shoulder_t[0] = mt_mul(body_t, mt_mul(t_inv(&robot->CurrentState.Posture[BASE].T), robot->CurrentState.Posture[LEFT_SHOULDER].T));
	shoulder_t[1] = mt_mul(body_t, mt_mul(t_inv(&robot->CurrentState.Posture[BASE].T), robot->CurrentState.Posture[RIGHT_SHOULDER].T));
	//----1.25 获取世界坐标系下手的位置
	//hand_t[0] = mt_mul(body_t, mt_mul(t_inv(&robot->CurrentState.Posture[BASE].T), robot->CurrentState.Posture[LEFT_HAND].T));
	//hand_t[1] = mt_mul(body_t, mt_mul(t_inv(&robot->CurrentState.Posture[BASE].T), robot->CurrentState.Posture[RIGHT_HAND].T));
	//----1.3 获取假定地面高度
	Ground = fmin(knee_t[0].t[_pz], knee_t[1].t[_pz]);
	for (i = 0; i < 2; i++)
	{
		pa = &robot->SR_Arm_Control[i];

		for (j = 0; j < 3; j++)
		{
			arm_joint[i][j] = robot->CurrentState.Joint[LEFT_ARM_1 + i*(RIGHT_ARM_1 - LEFT_ARM_1)+j] + pa->delta_joint[j];
		}
		hand_t[i] = mt_mul(shoulder_t[i], get_arm_t(&(arm_joint[i][0])));

		//获取手部位置h
		pa->dh = ((hand_t[i].t[_pz] - Ground) / 1000.0-pa->h)/ pa->LQR[0].T;
		pa->h = (hand_t[i].t[_pz] - Ground) / 1000.0;

		//计算世界坐标系下手部受力Fw = -K*(h-H)-B*dh
		if (pa->h > pa->H)
		{
			pa->Fw.m[2][0] = 0.0;
		}
		else
		{
			pa->Fw.m[2][0] = -pa->K*(pa->h - pa->H) - pa->B*fmin(pa->dh,0.0);
		}
		//printf("%f,%f -- ", pa->h, pa->dh);

		//将Fw转换到手臂坐标系下：Fa
		temp = get_rot_from_t(t_inv(&shoulder_t[i]));
		pa->Fa = m_mul(&temp, &pa->Fw);

		//计算雅可比矩阵
		q1 = robot->CurrentState.Joint[LEFT_ARM_1 + i*(RIGHT_ARM_1 - LEFT_ARM_1)] + pa->delta_joint[0];
		q2 = robot->CurrentState.Joint[LEFT_ARM_2 + i*(RIGHT_ARM_2 - LEFT_ARM_2)] + pa->delta_joint[1];
		q3 = robot->CurrentState.Joint[LEFT_ARM_3 + i*(RIGHT_ARM_3 - LEFT_ARM_3)] + pa->delta_joint[2];
		L2_1z = -UPPER_ARM_LEN / 1000.0;
		c1z = -(LOWER_ARM_LEN + HAND_R) / 1000.0;
		pa->J.m[0][0] = L2_1z*cos(q1)*cos(q2) - c1z*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3));
		pa->J.m[0][1] = -sin(q1)*(L2_1z*sin(q2) + c1z*cos(q3)*sin(q2));
		pa->J.m[0][2] = c1z*cos(q1)*cos(q3)*pow(sin(q2), 2.0) - c1z*cos(q2)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3));
		pa->J.m[1][0] = 0;
		pa->J.m[1][1] = cos(q1)*(c1z*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - L2_1z*cos(q1)*cos(q2)) - sin(q1)*(c1z*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + L2_1z*cos(q2)*sin(q1));
		pa->J.m[1][2] = c1z*cos(q1)*sin(q2)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + c1z*sin(q1)*sin(q2)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3));
		pa->J.m[2][0] = -c1z*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - L2_1z*cos(q2)*sin(q1);
		pa->J.m[2][1] = -cos(q1)*(L2_1z*sin(q2) + c1z*cos(q3)*sin(q2));
		pa->J.m[2][2] = -c1z*cos(q2)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - c1z*cos(q3)*sin(q1)*pow(sin(q2), 2.0);

		//计算各关节力矩 Tau = J'*Fa
		temp = m_T(&pa->J);
		temp1 = m_mul(&temp, &pa->Fa);

		for (j = 0; j < 3; j++)
		{
			last_torque[j] = pa->Torque[j];
			pa->Torque[j] = temp1.m[j][0];
		}
		//VMC信息更新
		for (j = 0; j < 3; j++)
		{
			pa->LQR[j].ref_value = 0.0;
			pa->LQR[j].ref_value2 = 0.0;
			pa->LQR[j].real_value = pa->Torque[j];
			pa->LQR[j].real_value2 = (pa->Torque[j] - last_torque[j]) / pa->LQR[j].T;
		}
	}
	//printf("\n");
}


void sr_arm_vmc_control(BRobot *robot)
{
	ArmJointVMC * pa;
	int i,j;
	double tune_k[3] = { 1,1,1 };

	for (i = 0; i < 2; i++)
	{
		pa = &robot->SR_Arm_Control[i];
		//VMC计算各关节角度增量
		for (j = 0; j < 3; j++)
		{
			lqr_control(&(pa->LQR[j]), tune_k[j]);
			pa->delta_joint[j] = pa->LQR[j].delta;
			//robot->CurrentState.Joint[LEFT_ARM_1 + i*(RIGHT_ARM_1 - LEFT_ARM_1) + j] += pa->delta_joint[j];
		}
		//在主程序输出的地方叠加关节角增量
	}
}

//手臂运动规划结束时将调整量叠加到机器人CurrentState与PastState上，避免角度突变
void sr_arm_vmc_end(BRobot * robot)
{
	int i, j;
	ArmJointVMC *pa;

	//角度增量叠加
	for (i = 0; i < 2; i++)
	{
		pa = &(robot->SR_Arm_Control[i]);
		for (j = 0; j < 3; j++)
		{
			robot->CurrentState.Joint[LEFT_ARM_1 + (RIGHT_ARM_1 - LEFT_ARM_1)*i + j] += pa->delta_joint[j];
			robot->PastState.Joint[LEFT_ARM_1 + (RIGHT_ARM_1 - LEFT_ARM_1)*i + j] += pa->delta_joint[j];// pa->LQR[j].last_delta;
		}
	}

	//结构体初始化清零
	sr_arm_vmc_init(robot);
}