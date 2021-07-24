#include "leeOnlineDCMWalking.h"

//DCM行走实时规划程序入口 
void realtime_dcm_walk(DCMWalkingState * dcm_state)
{
	dcm_state->Current_Time += dcm_state->DCM_Set.inter_time;
	dcm_state->Walking_State = dcm_state->DCM_Set.StepSet[0].Walking_State;
	odw_get_steps(dcm_state);
	odw_get_points(dcm_state);
	odw_get_trajectory(dcm_state);
}

//初始化(先初始化robot，再初始化dcm_state)
void odw_init_set(DCMWalkingState * dcm_state, BRobot * robot)
{
	//DCMWalkingState ini_state;
	int i,j;
	//重置
//	*dcm_state = ini_state;
	dcm_state->init_flag = HAD_SET;
	dcm_state->Current_Step = 0;
	dcm_state->Current_Time = 0.0;
	dcm_state->Remain_Step_Num = PLAN_STEP_NUM;
	dcm_state->Walking_State = ODW_STATE_STAND;
	dcm_state->qbody = 0.0;
	dcm_state->qbody_v = 0.0;
	dcm_state->qbody_a = 0.0;
	for (i = 0; i < 3; i++)
	{
		dcm_state->cog[i] = robot->CurrentState.Posture[BASE].Posture.p[3 + i] / 1000.0;
		dcm_state->dcog[i] = 0.0;
		dcm_state->dcm[i] = dcm_state->cog[i];
		dcm_state->ddcm[i] = 0.0;
		dcm_state->vrp[i] = dcm_state->cog[i];
		dcm_state->foot[0][i] = robot->CurrentState.Posture[LEFT_FOOT].Posture.p[3 + i] / 1000.0;
		dcm_state->foot[1][i] = robot->CurrentState.Posture[RIGHT_FOOT].Posture.p[3 + i] / 1000.0;
		dcm_state->dfoot[0][i] = 0.0;
		dcm_state->dfoot[1][i] = 0.0; 
		dcm_state->ddfoot[0][i] = 0.0;
		dcm_state->ddfoot[1][i] = 0.0;
	}
	dcm_state->DCM_Set.Step_Num = (int)robot->dcmSetTab[DCM_SET_STEP_NUM];//总的行走步数
	dcm_state->DCM_Set.Plan_Step_Num = dcm_state->DCM_Set.Step_Num + 2;
	dcm_state->DCM_Set.inter_time = robot->dcmSetTab[DCM_SET_SYS_CIRCLE];
	for (i = 0; i < PLAN_STEP_NUM; i++)
	{
		dcm_state->DCM_Set.StepSet[i].Walking_State = ODW_STATE_STAND;
		dcm_state->DCM_Set.StepSet[i].Walking_State_Next = ODW_STATE_STAND;
		dcm_state->DCM_Set.StepSet[i].SS_Rate = robot->dcmSetTab[DCM_SET_SS_RATE];
		dcm_state->DCM_Set.StepSet[i].Step_Time = robot->dcmSetTab[DCM_SET_STEP_TIME];
		dcm_state->DCM_Set.StepSet[i].Step_Time_SS = robot->dcmSetTab[DCM_SET_STEP_TIME]*robot->dcmSetTab[DCM_SET_SS_RATE];
		dcm_state->DCM_Set.StepSet[i].Step_Time_DS = dcm_state->DCM_Set.StepSet[i].Step_Time - dcm_state->DCM_Set.StepSet[i].Step_Time_SS;
		dcm_state->DCM_Set.StepSet[i].Step_Time_DS_Ini = 0.5*dcm_state->DCM_Set.StepSet[i].Step_Time_DS;
		dcm_state->DCM_Set.StepSet[i].Step_Time_DS_End = 0.5*dcm_state->DCM_Set.StepSet[i].Step_Time_DS;
		dcm_state->DCM_Set.StepSet[i].Step_Width = robot->dcmSetTab[DCM_SET_STEP_WIDTH];
		dcm_state->DCM_Set.StepSet[i].VRP_Offset = (1.0-0.9)*BASE_HIP_Y/1000.0;
		dcm_state->DCM_Set.StepSet[i].Step_Length = robot->dcmSetTab[DCM_SET_STEP_LEN];
		dcm_state->DCM_Set.StepSet[i].COM_Height = robot->dcmSetTab[DCM_SET_WALK_HEIGHT];
		dcm_state->DCM_Set.StepSet[i].Foot_Height = robot->dcmSetTab[DCM_SET_FOOT_HEIGHT];
		dcm_state->DCM_Set.StepSet[i].B = sqrt(dcm_state->DCM_Set.StepSet[i].COM_Height / 9.8);
		dcm_state->DCM_Set.StepSet[i].Delta_Step_Length = 0.0;
		dcm_state->DCM_Set.StepSet[i].Delta_Step_Width = 0.0;
		for (j = 0; j < 3; j++)
		{
			dcm_state->DCM_Set.StepSet[i].VRP_Now[j]	=	dcm_state->vrp[j];
			dcm_state->DCM_Set.StepSet[i].VRP_Next[j]	=	dcm_state->vrp[j];
			dcm_state->DCM_Set.StepSet[i].DCM_End[j]	=	dcm_state->dcm[j];
			dcm_state->DCM_Set.StepSet[i].DCM_Ini[j]	=	dcm_state->dcm[j];
			dcm_state->DCM_Set.StepSet[i].DCM_DS_Ini[j] =	dcm_state->dcm[j];
			dcm_state->DCM_Set.StepSet[i].DCM_DS_End[j] =	dcm_state->dcm[j];
			dcm_state->DCM_Set.StepSet[i].DDCM_DS_Ini[j] =	0;
			dcm_state->DCM_Set.StepSet[i].DDCM_DS_End[j] =	0;
			dcm_state->DCM_Set.StepSet[i].FOOT[0][j]	=	dcm_state->foot[0][j];
			dcm_state->DCM_Set.StepSet[i].FOOT[1][j]	=	dcm_state->foot[1][j];
		}
	}
	dcm_state->DCM_Set.LastStepSet = dcm_state->DCM_Set.StepSet[0];
}

//实时落脚点规划
void odw_get_steps(DCMWalkingState * dcm_state)
{
	DCMWalkingSet *p = &(dcm_state->DCM_Set);
	const int START_STEP = 0;
	const int FINAL_STEP = p->Plan_Step_Num - 1;
	
	int i = 0;
	int j = 0;
	//行走期望值:SR 直线行走
	
	//可在此处调整当前步步长、时间等参数

	//保存上一步步行参数
	//p->LastStepSet = p->StepSet[0];

	//获取当前步及接下来N-1步（共N步）的步行参数：落脚点位置、质心高度（B）、抬脚高度、步行周期
	//获取第一步
	if (dcm_state->Current_Step == START_STEP)//当前步为起始步
	{
		if (p->StepSet[0].Step_Width >= 0.0)//迈左脚，下一步VRP移到右脚
		{
			p->StepSet[0].Walking_State_Next = ODW_STATE_SWL;
		}
		else//迈右脚，下一步VRP移到左脚
		{
			p->StepSet[0].Walking_State_Next = ODW_STATE_SWR;
		}
		p->StepSet[0].Walking_State = ODW_STATE_1ST_STEP;

		if (p->StepSet[0].Walking_State_Next == ODW_STATE_SWL)
		{
			p->StepSet[0].VRP_Next[1] = p->LastStepSet.FOOT[_L_RIGHT][1] + p->StepSet[0].VRP_Offset;
			p->StepSet[0].QBODY_SS = 0.0;
			p->StepSet[0].QBODY_DS = -QBODY_MAX;
		}
		else
		{
			p->StepSet[0].VRP_Next[1] = p->LastStepSet.FOOT[_L_LEFT][1] - p->StepSet[0].VRP_Offset;
			p->StepSet[0].QBODY_SS = 0.0;
			p->StepSet[0].QBODY_DS = QBODY_MAX;
		}
	}
	else if (dcm_state->Current_Step == FINAL_STEP)//当前步为结束步
	{
		//p->StepSet[0].Walking_State = ODW_STATE_SWL * ((int)(p->LastStepSet.Walking_State == ODW_STATE_SWR)) + ODW_STATE_SWR * ((int)(p->LastStepSet.Walking_State == ODW_STATE_SWL));
		p->StepSet[0].Walking_State = p->LastStepSet.Walking_State_Next;
		for (i = 0; i < 3; i++)p->StepSet[0].VRP_Now[i] = p->LastStepSet.VRP_Next[i];
		if (p->StepSet[0].Walking_State == ODW_STATE_SWL)//摆动左腿，右脚位置保持不变，左脚位置改变
		{
			p->StepSet[0].Walking_State_Next = ODW_STATE_STAND;
			p->StepSet[0].FOOT[_L_LEFT][0] = p->StepSet[0].FOOT[_L_RIGHT][0];
			p->StepSet[0].FOOT[_L_LEFT][1] = p->StepSet[0].FOOT[_L_RIGHT][1] + 2.0*BASE_HIP_Y / 1000.0;
			//p->StepSet[0].FOOT[_L_LEFT][2];//高度保持不变
			p->StepSet[0].VRP_Next[0] = p->StepSet[0].FOOT[_L_LEFT][0] + BASE_HIP_X / 1000.0;
			p->StepSet[0].VRP_Next[1] = p->StepSet[0].FOOT[_L_LEFT][1] - BASE_HIP_Y / 1000.0;
			//行走高度目前不变
			p->StepSet[0].QBODY_SS = QBODY_MAX;
			p->StepSet[0].QBODY_DS = 0.0;
		}
		else//右腿摆动，左脚位置保持不变，右脚位置改变
		{
			p->StepSet[0].Walking_State_Next = ODW_STATE_STAND;
			p->StepSet[0].FOOT[_L_RIGHT][0] = p->StepSet[0].FOOT[_L_LEFT][0];
			p->StepSet[0].FOOT[_L_RIGHT][1] = p->StepSet[0].FOOT[_L_LEFT][1] - 2.0*BASE_HIP_Y / 1000.0;
			//p->StepSet[0].FOOT[_L_LEFT][2];//高度保持不变
			p->StepSet[0].VRP_Next[0] = p->StepSet[0].FOOT[_L_RIGHT][0] + BASE_HIP_X / 1000.0;
			p->StepSet[0].VRP_Next[1] = p->StepSet[0].FOOT[_L_RIGHT][1] + BASE_HIP_Y / 1000.0;
			//行走高度目前不变
			p->StepSet[0].QBODY_SS = -QBODY_MAX;
			p->StepSet[0].QBODY_DS = 0.0;
		}
	}
	else//当前步为中间步
	{
		//p->StepSet[0].Walking_State = ODW_STATE_SWL * ((int)(p->LastStepSet.Walking_State == ODW_STATE_1ST_STEP)) + ODW_STATE_SWL * ((int)(p->LastStepSet.Walking_State == ODW_STATE_SWR)) + ODW_STATE_SWR * ((int)(p->LastStepSet.Walking_State == ODW_STATE_SWL));
		p->StepSet[0].Walking_State = p->LastStepSet.Walking_State_Next;
		for (i = 0; i < 3; i++)p->StepSet[0].VRP_Now[i] = p->LastStepSet.VRP_Next[i];
		if (p->StepSet[0].Walking_State == ODW_STATE_SWL)//摆动左腿，右脚位置保持不变，左脚位置改变
		{
			p->StepSet[0].Walking_State_Next = ODW_STATE_SWR;
			p->StepSet[0].FOOT[_L_LEFT][0] = p->StepSet[0].FOOT[_L_RIGHT][0] + p->StepSet[0].Step_Length + p->StepSet[0].Delta_Step_Length;
			p->StepSet[0].FOOT[_L_LEFT][1] = p->StepSet[0].FOOT[_L_RIGHT][1] + 2.0*BASE_HIP_Y/1000.0 + fmax(p->StepSet[0].Step_Width,0.0) + p->StepSet[0].Delta_Step_Width;
			//p->StepSet[0].FOOT[_L_LEFT][2];//高度保持不变
			p->StepSet[0].VRP_Next[0] = p->StepSet[0].FOOT[_L_LEFT][0] + BASE_HIP_X / 1000.0;
			p->StepSet[0].VRP_Next[1] = p->StepSet[0].FOOT[_L_LEFT][1] - p->StepSet[0].VRP_Offset;
			//行走高度目前不变
			p->StepSet[0].QBODY_SS = QBODY_MAX;
			p->StepSet[0].QBODY_DS = -QBODY_MAX;
		}
		else//右腿摆动，左脚位置保持不变，右脚位置改变
		{
			p->StepSet[0].Walking_State_Next = ODW_STATE_SWL;
			p->StepSet[0].FOOT[_L_RIGHT][0] = p->StepSet[0].FOOT[_L_LEFT][0] + p->StepSet[0].Step_Length + p->StepSet[0].Delta_Step_Length;
			p->StepSet[0].FOOT[_L_RIGHT][1] = p->StepSet[0].FOOT[_L_LEFT][1] - 2.0*BASE_HIP_Y / 1000.0 + fmin(p->StepSet[0].Step_Width,0.0) + p->StepSet[0].Delta_Step_Width;
			//p->StepSet[0].FOOT[_L_LEFT][2];//高度保持不变
			p->StepSet[0].VRP_Next[0] = p->StepSet[0].FOOT[_L_RIGHT][0] + BASE_HIP_X / 1000.0;
			p->StepSet[0].VRP_Next[1] = p->StepSet[0].FOOT[_L_RIGHT][1] + p->StepSet[0].VRP_Offset;
			//行走高度目前不变
			p->StepSet[0].QBODY_SS = -QBODY_MAX;
			p->StepSet[0].QBODY_DS = QBODY_MAX;
		}
	}
	
	//获取中间步
	dcm_state->Remain_Step_Num = (int)fmin((double)(dcm_state->DCM_Set.Plan_Step_Num - dcm_state->Current_Step), (double)PLAN_STEP_NUM);
	for(i=1;i<dcm_state->Remain_Step_Num-1;i++)
	{
		//VRP与FOOT
		p->StepSet[i].Walking_State = p->StepSet[i - 1].Walking_State_Next;
		for (j = 0; j < 3; j++)p->StepSet[i].VRP_Now[j] = p->StepSet[i - 1].VRP_Next[j];
		if (p->StepSet[i].Walking_State == ODW_STATE_SWL)//摆动左腿，右脚位置保持不变，左脚位置改变
		{
			p->StepSet[i].Walking_State_Next = ODW_STATE_SWR;
			p->StepSet[i].FOOT[_L_LEFT][0] = p->StepSet[i].FOOT[_L_RIGHT][0] + p->StepSet[i].Step_Length + p->StepSet[i].Delta_Step_Length;
			p->StepSet[i].FOOT[_L_LEFT][1] = p->StepSet[i].FOOT[_L_RIGHT][1] + 2.0*BASE_HIP_Y / 1000.0 + fmax(p->StepSet[i].Step_Width, 0.0) + p->StepSet[i].Delta_Step_Width;
			//p->StepSet[0].FOOT[_L_LEFT][2];//高度保持不变
			p->StepSet[i].VRP_Next[0] = p->StepSet[i].FOOT[_L_LEFT][0] + BASE_HIP_X / 1000.0;
			p->StepSet[i].VRP_Next[1] = p->StepSet[i].FOOT[_L_LEFT][1] - p->StepSet[i].VRP_Offset;
			//行走高度目前不变
			p->StepSet[i].QBODY_SS = QBODY_MAX;
			p->StepSet[i].QBODY_DS = -QBODY_MAX;
		}
		else//右腿摆动，左脚位置保持不变，右脚位置改变
		{
			p->StepSet[i].Walking_State_Next = ODW_STATE_SWL;
			p->StepSet[i].FOOT[_L_RIGHT][0] = p->StepSet[i].FOOT[_L_LEFT][0] + p->StepSet[i].Step_Length + p->StepSet[i].Delta_Step_Length;
			p->StepSet[i].FOOT[_L_RIGHT][1] = p->StepSet[i].FOOT[_L_LEFT][1] - 2.0*BASE_HIP_Y / 1000.0 + fmin(p->StepSet[i].Step_Width, 0.0) + p->StepSet[i].Delta_Step_Width;
			//p->StepSet[0].FOOT[_L_LEFT][2];//高度保持不变
			p->StepSet[i].VRP_Next[0] = p->StepSet[i].FOOT[_L_RIGHT][0] + BASE_HIP_X / 1000.0;
			p->StepSet[i].VRP_Next[1] = p->StepSet[i].FOOT[_L_RIGHT][1] + p->StepSet[i].VRP_Offset;
			//行走高度目前不变
			p->StepSet[i].QBODY_SS = -QBODY_MAX;
			p->StepSet[i].QBODY_DS = QBODY_MAX;
		}
	}
	//获取最后一步
	p->StepSet[i].Walking_State = p->StepSet[i - 1].Walking_State_Next;
	for (j = 0; j < 3; j++)p->StepSet[i].VRP_Now[j] = p->StepSet[i - 1].VRP_Next[j];
	if (p->StepSet[i].Walking_State == ODW_STATE_SWL)//摆动左腿，右脚位置保持不变，左脚位置改变
	{
		p->StepSet[i].Walking_State_Next = ODW_STATE_STAND;
		p->StepSet[i].FOOT[_L_LEFT][0] = p->StepSet[i].FOOT[_L_RIGHT][0];
		p->StepSet[i].FOOT[_L_LEFT][1] = p->StepSet[i].FOOT[_L_RIGHT][1] + 2.0*BASE_HIP_Y / 1000.0;
		//p->StepSet[0].FOOT[_L_LEFT][2];//高度保持不变
		p->StepSet[i].VRP_Next[0] = p->StepSet[i].FOOT[_L_LEFT][0] + BASE_HIP_X / 1000.0;
		p->StepSet[i].VRP_Next[1] = p->StepSet[i].FOOT[_L_LEFT][1] - BASE_HIP_Y / 1000.0;
		//行走高度目前不变
		p->StepSet[i].QBODY_SS = QBODY_MAX;
		p->StepSet[i].QBODY_DS = 0.0;
	}
	else//右腿摆动，左脚位置保持不变，右脚位置改变
	{
		p->StepSet[i].Walking_State_Next = ODW_STATE_STAND;
		p->StepSet[i].FOOT[_L_RIGHT][0] = p->StepSet[i].FOOT[_L_LEFT][0];
		p->StepSet[i].FOOT[_L_RIGHT][1] = p->StepSet[i].FOOT[_L_LEFT][1] - 2.0*BASE_HIP_Y / 1000.0;
		//p->StepSet[0].FOOT[_L_LEFT][2];//高度保持不变
		p->StepSet[i].VRP_Next[0] = p->StepSet[i].FOOT[_L_RIGHT][0] + BASE_HIP_X / 1000.0;
		p->StepSet[i].VRP_Next[1] = p->StepSet[i].FOOT[_L_RIGHT][1] + BASE_HIP_Y / 1000.0;
		//行走高度目前不变
		p->StepSet[i].QBODY_SS = -QBODY_MAX;
		p->StepSet[i].QBODY_DS = 0.0;
	}

}

//离散DCM、DS_DCM计算
void odw_get_points(DCMWalkingState * dcm_state)
{
	int i, j;
	DCMWalkingSet *p = &(dcm_state->DCM_Set);
	const int START_STEP = 0;
	const int FINAL_STEP = p->Plan_Step_Num - 1;

	//计算离散DCM点
	for (i = 0; i < 3; i++)
	{
		p->StepSet[dcm_state->Remain_Step_Num - 1].DCM_End[i] = p->StepSet[dcm_state->Remain_Step_Num - 1].VRP_Next[i];
		for (j = dcm_state->Remain_Step_Num - 1; j >= 0; j--)
		{
			p->StepSet[j].DCM_Ini[i] = p->StepSet[j].VRP_Now[i] + exp(-1.0 / p->StepSet[j].B*p->StepSet[j].Step_Time)*(p->StepSet[j].DCM_End[i] - p->StepSet[j].VRP_Now[i]);
			if (j < dcm_state->Remain_Step_Num - 1)
			{
				p->StepSet[j].DCM_End[i] = p->StepSet[j + 1].DCM_Ini[i];
			}
		}
	}

	//引入双脚支撑期，计算当前步离散DS_DCM(计算当前步的离散DS_DCM点与对应速度)
	for (i = 0; i < 3; i++)
	{
		if (dcm_state->Current_Step == START_STEP)
		{
			p->StepSet[0].Step_Time_DS = p->StepSet[0].Step_Time;
			p->StepSet[0].Step_Time_DS_End = 0.5*(p->StepSet[0].Step_Time*(1.0- p->StepSet[0].SS_Rate));
			p->StepSet[0].Step_Time_DS_Ini = p->StepSet[0].Step_Time_DS_End;//p->StepSet[0].Step_Time_DS - p->StepSet[0].Step_Time_DS_End;
			p->StepSet[0].Step_Time_SS = 0.0;
			p->StepSet[0].DCM_DS_Ini[i] = p->StepSet[0].VRP_Now[i];
			p->StepSet[0].DDCM_DS_Ini[i] = 0.0;
			p->StepSet[0].DCM_DS_End[i] = p->StepSet[1].VRP_Now[i] + exp(1.0 / p->StepSet[1].B*p->StepSet[0].Step_Time_DS_End)*(p->StepSet[0].DCM_End[i] - p->StepSet[1].VRP_Now[i]);
			p->StepSet[0].DDCM_DS_End[i] = 1.0 / p->StepSet[1].B*exp(1.0 / p->StepSet[1].B*p->StepSet[0].Step_Time_DS_End)*(p->StepSet[0].DCM_End[i] - p->StepSet[1].VRP_Now[i]);
		}
		else if (dcm_state->Current_Step == FINAL_STEP)
		{
			p->StepSet[0].Step_Time_SS = p->StepSet[0].Step_Time * p->StepSet[0].SS_Rate;
			p->StepSet[0].Step_Time_DS = p->StepSet[0].Step_Time - p->StepSet[0].Step_Time_SS;
			p->StepSet[0].Step_Time_DS_End = 0.5*p->StepSet[0].Step_Time_DS;
			p->StepSet[0].Step_Time_DS_Ini = p->StepSet[0].Step_Time_DS - p->StepSet[0].Step_Time_DS_End;
			p->StepSet[0].DCM_DS_Ini[i] = p->StepSet[0].VRP_Now[i] + exp(-1.0 / p->StepSet[0].B*p->StepSet[0].Step_Time_DS_Ini)*(p->StepSet[0].DCM_End[i] - p->StepSet[0].VRP_Now[i]);
			p->StepSet[0].DDCM_DS_Ini[i] = 1.0 / p->StepSet[0].B*exp(-1.0 / p->StepSet[0].B*p->StepSet[0].Step_Time_DS_Ini)*(p->StepSet[0].DCM_End[i] - p->StepSet[0].VRP_Now[i]);
			p->StepSet[0].DCM_DS_End[i] = p->StepSet[0].DCM_End[i];
			p->StepSet[0].DDCM_DS_End[i] = 0.0;

		}
		else
		{
			p->StepSet[0].Step_Time_SS = p->StepSet[0].Step_Time * p->StepSet[0].SS_Rate;
			p->StepSet[0].Step_Time_DS = p->StepSet[0].Step_Time - p->StepSet[0].Step_Time_SS;
			p->StepSet[0].Step_Time_DS_End = 0.5*p->StepSet[0].Step_Time_DS;
			p->StepSet[0].Step_Time_DS_Ini = p->StepSet[0].Step_Time_DS - p->StepSet[0].Step_Time_DS_End;
			p->StepSet[0].DCM_DS_Ini[i] = p->StepSet[0].VRP_Now[i] + exp(-1.0 / p->StepSet[0].B*p->StepSet[0].Step_Time_DS_Ini)*(p->StepSet[0].DCM_End[i] - p->StepSet[0].VRP_Now[i]);
			p->StepSet[0].DDCM_DS_Ini[i] = 1.0 / p->StepSet[0].B*exp(-1.0 / p->StepSet[0].B*p->StepSet[0].Step_Time_DS_Ini)*(p->StepSet[0].DCM_End[i] - p->StepSet[0].VRP_Now[i]);
			p->StepSet[0].DCM_DS_End[i] = p->StepSet[1].VRP_Now[i] + exp(1.0 / p->StepSet[1].B*p->StepSet[0].Step_Time_DS_End)*(p->StepSet[0].DCM_End[i] - p->StepSet[1].VRP_Now[i]);
			p->StepSet[0].DDCM_DS_End[i] = 1.0 / p->StepSet[1].B*exp(1.0 / p->StepSet[1].B*p->StepSet[0].Step_Time_DS_End)*(p->StepSet[0].DCM_End[i] - p->StepSet[1].VRP_Now[i]);
		}
	}

}

//计算连续VRP、DCM、FOOT轨迹
void odw_get_trajectory(DCMWalkingState * dcm_state)
{
	DCMWalkingSet *p = &(dcm_state->DCM_Set);
	StepWalkingSet *ps = &(dcm_state->DCM_Set.StepSet[0]);
	StepWalkingSet *lps = &(dcm_state->DCM_Set.LastStepSet);
	int i;
	int FOOT_ID;
	double t = dcm_state->Current_Time;
	double b = ps->B;
	double last_dcog;
	//计算DCM轨迹与FOOT轨迹。根据单步时间判断是双脚支撑期还是单脚支撑期
	if (dcm_state->Current_Time < ps->Step_Time_SS)//单脚支撑期
	{
		//单脚支撑期根据固定VRP计算DCM轨迹
		for (i = 0; i < 3; i++)
		{
			dcm_state->dcm[i] = ps->VRP_Now[i] + exp(1.0 / b*(t + ps->Step_Time_DS_End - ps->Step_Time))*(ps->DCM_End[i] - ps->VRP_Now[i]);
			dcm_state->ddcm[i] = 1.0 / b*exp(1.0 / ps->B*(t + ps->Step_Time_DS_End - ps->Step_Time))*(ps->DCM_End[i] - ps->VRP_Now[i]);
			dcm_state->vrp[i] = ps->VRP_Now[i];
		}
		//插值计算FOOT轨迹
		if (ps->Walking_State == ODW_STATE_SWL)
		{
			FOOT_ID = _L_LEFT;//摆动左脚
		}
		else
		{
			FOOT_ID = _L_RIGHT;//摆动右脚
		}
		for (i = 0; i < 2; i++)//首先插值计算XY方向轨迹
		{
			realtime_1D_interpolation_5(
				&(dcm_state->foot[FOOT_ID][i]), &(dcm_state->dfoot[FOOT_ID][i]), &(dcm_state->ddfoot[FOOT_ID][i]),
				ps->FOOT[FOOT_ID][i], 0.0, 0.0, t, ps->Step_Time_SS, p->inter_time);
			//realtime_1D_interpolation(
			//	&(dcm_state->foot[FOOT_ID][i]), &(dcm_state->dfoot[FOOT_ID][i]),
			//	ps->FOOT[FOOT_ID][i], 0.0, t, ps->Step_Time_SS, p->inter_time
			//);
		}
		if (t <= 0.5*ps->Step_Time_SS)//再插值计算Z方向轨迹，因为Z方向需要抬起再落下
		{
			realtime_1D_interpolation_5(
				&(dcm_state->foot[FOOT_ID][2]), &(dcm_state->dfoot[FOOT_ID][2]), &(dcm_state->ddfoot[FOOT_ID][2]),
				ps->Foot_Height, 0.0, 0.0, t, ps->Step_Time_SS*0.5, p->inter_time
			);
		}
		else
		{
			realtime_1D_interpolation_5(
				&(dcm_state->foot[FOOT_ID][2]), &(dcm_state->dfoot[FOOT_ID][2]), &(dcm_state->ddfoot[FOOT_ID][2]),
				ps->FOOT[FOOT_ID][2], 0.0, 0.0, t- ps->Step_Time_SS*0.5, ps->Step_Time_SS*0.5, p->inter_time
			);
		}
		//插值计算Qbody轨迹
		realtime_1D_interpolation_5(
			&dcm_state->qbody, &dcm_state->qbody_v, &dcm_state->qbody_a,
			ps->QBODY_SS, 0.0, 0.0,
			t, ps->Step_Time_SS, p->inter_time
		);
	}
	else//双脚支撑期
	{
		//双支撑期插值计算DCM，再计算VRP
		t = t - ps->Step_Time_SS;
		realtime_3D_interpolation(dcm_state->dcm, dcm_state->ddcm, ps->DCM_DS_End, ps->DDCM_DS_End, t, ps->Step_Time_DS, p->inter_time);
		for (i = 0; i < 3; i++)
		{
			dcm_state->vrp[i] = dcm_state->dcm[i] - b*dcm_state->ddcm[i];
		}
		//插值计算Qbody轨迹
		realtime_1D_interpolation_5(
			&dcm_state->qbody, &dcm_state->qbody_v, &dcm_state->qbody_a,
			ps->QBODY_DS, 0.0, 0.0,
			t, ps->Step_Time_DS, p->inter_time
		);
	}
	
	//计算质心速度，积分计算质心位置
	for (i = 0; i < 3; i++)
	{
		last_dcog = dcm_state->dcog[i];
		dcm_state->dcog[i] = -1.0 / b*(dcm_state->cog[i] - dcm_state->dcm[i]);
		//dcm_state->cog[i] += 0.5*(last_dcog + dcm_state->dcog[i])*p->inter_time;//梯形积分公式
		dcm_state->cog[i] += dcm_state->dcog[i]*p->inter_time;//矩形积分公式
	}
}
