/*-------------------------------------/
�� �ִ����⣺
	��1��ȱ��QBody��˫����̬�Ĺ滮��Ӧ��ֻ�ù滮��ǰ��
�� 2019-0407, �Ѳ���QBody�Ĺ滮
�� 2019-0408, �޸Ĳ�����ŵ�滮����Ӻ����ƶ�
--------------------------------------*/
#ifndef LEE_ON_LINE_DCM_WALKING
#define LEE_ON_LINE_DCM_WALKING
#include "leeMatrix.h"
#include "leeRobotConfig.h"
#include "leeRobotTool.h"

#define PLAN_STEP_NUM 5

#define STATE_COM_DOWN			1
#define STATE_WALK_1ST_STEP		2
#define STATE_WALK_SWL_SS		3
#define STATE_WALK_SWL_DS		4
#define STATE_WALK_SWR_SS		5
#define STATE_WALK_SWR_DS		6
#define STATE_WALK_DCM_STOP		7
#define STATE_DCM_STAND			8
#define STATE_FALL				9
#define STATE_NUM				10

#define ODW_STATE_STAND		0
#define ODW_STATE_1ST_STEP	1
#define ODW_STATE_SWL		2
#define ODW_STATE_SWR		3

typedef struct StepWalkingSet
{
	int		Walking_State;
	int		Walking_State_Next;

	double  SS_Rate;
	double	Step_Time;
	double	Step_Time_SS;
	double	Step_Time_DS;
	double	Step_Time_DS_Ini;
	double	Step_Time_DS_End;
	double	Step_Width;
	double	Step_Length;
	double	VRP_Offset;
	double	COM_Height;
	double	Foot_Height;
	double  B;

	double	Delta_Step_Width;
	double	Delta_Step_Length;

	double	VRP_Now[3];
	double	VRP_Next[3];
	double	DCM_End[3];
	double	DCM_Ini[3];
	double	DCM_DS_Ini[3];
	double	DCM_DS_End[3];
	double	DDCM_DS_Ini[3];
	double	DDCM_DS_End[3];
	double	FOOT[2][3];

	double	QBODY_SS;
	double	QBODY_DS;
}StepWalkingSet;

typedef struct DCMWalkingSet
{
	int	Step_Num;
	int	Plan_Step_Num;//ʵ�ʹ滮ʱ�����������սţ�һ����Step_Num+2��
	double inter_time;//��������[s]
	StepWalkingSet LastStepSet;
	StepWalkingSet StepSet[PLAN_STEP_NUM];
}DCMWalkingSet;

typedef struct DCMWalkingState
{
	int		init_flag;
	int		Current_Step;
	int		Remain_Step_Num;	//��ʱ�滮��ʣ�ಽ��
	double	Current_Time;
	int		Walking_State;
	double	vrp[3];
	double	dcm[3];
	double	ddcm[3];
	double	cog[3];
	double	dcog[3];
	double	foot[2][3];
	double	dfoot[2][3];
	double	ddfoot[2][3];
	double	qbody;
	double	qbody_v;
	double	qbody_a;
	DCMWalkingSet DCM_Set;
}DCMWalkingState;

void realtime_dcm_walk(DCMWalkingState * dcm_state);

//��ʼ��
void odw_init_set(DCMWalkingState * dcm_state, BRobot *  robot);

//ʵʱ��ŵ�滮
void odw_get_steps(DCMWalkingState * dcm_state);

//��ɢVRP��DCM��DS_DCM��FOOT����
void odw_get_points(DCMWalkingState * dcm_state);

//��������VRP��DCM��FOOT�켣
void odw_get_trajectory(DCMWalkingState * dcm_state); 
#endif