/*--------------------------------------------------------------/
■ Lee, 2019-01-18, 添加滤波器结构体 LEE_Filter 与 滤波函数 filt_data
■ Lee, 2019-01-19, 添加状态预估结构体 LEE_Estimator 与 状态估计函数 estimate_state
■ Lee, 2019-02-24, 添加二连杆动力学结构体 TwoLinkModelControl
■ Lee, 2019-04-08, 修正滤波函数 filt_data
■ Lee, 2019-04-11, 添加 IMU_DATA IMU数据处理结构体与初始化函数
■ Lee, 2019-04-13, 优化 update_current 函数，修改为：计算逆运动学情况下，不进行正运动学计算；否则，根据关节角进行正运动学运算得到新的姿态矩阵
/*-------------------------------------------------------------*/

#ifndef LEE_ROBOT_TOOL
#define LEE_ROBOT_TOOL
#include <string.h>
#include "leeRobotConfig.h"
#include "leeMatrix.h"

#define	DCM_SET_SYS_CIRCLE	0	//system circle[s]
#define	DCM_SET_STEP_TIME	1	//walking step time[s]
#define	DCM_SET_SS_RATE		2	//sinsgle support phase rate[-]
#define	DCM_SET_STEP_LEN	3	//walking step length[m]
#define	DCM_SET_WALK_HEIGHT	4	//walking BASE height[m]
#define	DCM_SET_VMC_K_RX_1	5	//vmc gain k - rx 1[-]
#define	DCM_SET_VMC_K_RX_2	6	//vmc gain k - rx 2[-]
#define	DCM_SET_VMC_K_RX_3	7	//vmc gain k - rx 3[-]
#define	DCM_SET_VMC_K_RX_4	8	//vmc gain k - rx 4[-]
#define	DCM_SET_VMC_K_RY_1	9	//vmc gain k - ry 1[-]
#define	DCM_SET_VMC_K_RY_2	10	//vmc gain k - ry 2[-]
#define	DCM_SET_VMC_K_RY_3	11	//vmc gain k - ry 3[-]
#define	DCM_SET_VMC_K_RY_4	12	//vmc gain k - ry 4[-]
#define	DCM_SET_VMC_K_RZ_1	13	//vmc gain k - ry 1[-]
#define	DCM_SET_VMC_K_RZ_2	14	//vmc gain k - ry 2[-]
#define	DCM_SET_VMC_K_RZ_3	15	//vmc gain k - ry 3[-]
#define	DCM_SET_VMC_K_RZ_4	16	//vmc gain k - ry 4[-]
#define	DCM_SET_VMC_K_X_1	17	//vmc gain k - x 1[-]
#define	DCM_SET_VMC_K_X_2	18	//vmc gain k - x 2[-]
#define	DCM_SET_VMC_K_X_3	19	//vmc gain k - x 3[-]
#define	DCM_SET_VMC_K_X_4	20	//vmc gain k - x 4[-]
#define	DCM_SET_VMC_K_Y_1	21	//vmc gain k - y 1[-]
#define	DCM_SET_VMC_K_Y_2	22	//vmc gain k - y 2[-]
#define	DCM_SET_VMC_K_Y_3	23	//vmc gain k - y 3[-]
#define	DCM_SET_VMC_K_Y_4	24	//vmc gain k - y 4[-]
#define	DCM_SET_VMC_K_Z_1	25	//vmc gain k - z 1[-]
#define	DCM_SET_VMC_K_Z_2	26	//vmc gain k - z 2[-]
#define	DCM_SET_VMC_K_Z_3	27	//vmc gain k - z 3[-]
#define	DCM_SET_VMC_K_Z_4	28	//vmc gain k - z 4[-]
#define DCM_SET_STEP_NUM	29
#define DCM_SET_FOOT_HEIGHT	30	//foot height when walking[m]
#define DCM_SET_VDT_K1	31	//平衡控制器系数k1
#define DCM_SET_VDT_K2	32	//平衡控制器系数k2
#define DCM_SET_VDT_K3	33	//平衡控制器系数k3
#define DCM_SET_VDT_K4	34	//平衡控制器系数k4
#define DCM_SET_VDT_K5	35	//平衡控制器系数k5
#define DCM_SET_LEAD_COMP_TIME	36//超前校正时间系数
#define DCM_SET_LEAD_COMP_ALPHA	37//超前校正比例系数
#define DCM_SET_TPC_K1 38//TPC系数k1
#define DCM_SET_TPC_K2 39//TPC系数k2
#define DCM_SET_TPC_K3 40//TPC系数k3
#define DCM_SET_STEP_WIDTH	41//步行横向步长
#define DCM_TABLE_SIZE_SET	42

#define	DCM_INPUT_BODY_RX	1	//body posture - rx[rad]
#define	DCM_INPUT_BODY_RY	2	//body posture - ry[rad]
#define	DCM_INPUT_BODY_RZ	3	//body posture - rz[rad]
#define	DCM_INPUT_BODY_X	4	//body position - x[m]
#define	DCM_INPUT_BODY_Y	5	//body position - y[m]
#define	DCM_INPUT_BODY_Z	6	//body position - z[m]
#define	DCM_INPUT_BODY_V_RX	7	//body velocity - drx[rad/s]
#define	DCM_INPUT_BODY_V_RY	8	//body velocity - dry[rad/s]
#define	DCM_INPUT_BODY_V_RZ	9	//body velocity - drz[rad/s]
#define	DCM_INPUT_BODY_V_X	10	//body velocity - vx[m/s]
#define	DCM_INPUT_BODY_V_Y	11	//body velocity - vy[m/s]
#define	DCM_INPUT_BODY_V_Z	12	//body velocity - vz[m/s]
#define	DCM_INPUT_LEFT_TORQUE_X		13	//x torque of left ft sensor[N/m]
#define	DCM_INPUT_LEFT_TORQUE_Y		14	//y torque of left ft sensor[N/m]
#define	DCM_INPUT_LEFT_TORQUE_Z		15	//z torque of left ft sensor[N/m]
#define	DCM_INPUT_LEFT_FORCE_X		16	//x force of left ft sensor[N]
#define	DCM_INPUT_LEFT_FORCE_Y		17	//y force of left ft sensor[N]
#define	DCM_INPUT_LEFT_FORCE_Z		18	//z force of left ft sensor[N]
#define	DCM_INPUT_RIGHT_TORQUE_X	19	//x torque of right ft sensor[N/m]
#define	DCM_INPUT_RIGHT_TORQUE_Y	20	//y torque of right ft sensor[N/m]
#define	DCM_INPUT_RIGHT_TORQUE_Z	21	//z torque of right ft sensor[N/m]
#define	DCM_INPUT_RIGHT_FORCE_X		22	//x force of right ft sensor[N]
#define	DCM_INPUT_RIGHT_FORCE_Y		23	//y force of right ft sensor[N]
#define	DCM_INPUT_RIGHT_FORCE_Z		24	//z force of right ft sensor[N]
#define DCM_INPUT_REAL_JOINT_L1		25	//real joint of left leg 1[rad]
#define DCM_INPUT_REAL_JOINT_L2		26	//real joint of left leg 2[rad]
#define DCM_INPUT_REAL_JOINT_L3		27	//real joint of left leg 3[rad]
#define DCM_INPUT_REAL_JOINT_L4		28	//real joint of left leg 4[rad]
#define DCM_INPUT_REAL_JOINT_L5		29	//real joint of left leg 5[rad]
#define DCM_INPUT_REAL_JOINT_L6		30	//real joint of left leg 6[rad]
#define DCM_INPUT_REAL_JOINT_R1		31	//real joint of right leg 1[rad]
#define DCM_INPUT_REAL_JOINT_R2		32	//real joint of right leg 2[rad]
#define DCM_INPUT_REAL_JOINT_R3		33	//real joint of right leg 3[rad]
#define DCM_INPUT_REAL_JOINT_R4		34	//real joint of right leg 4[rad]
#define DCM_INPUT_REAL_JOINT_R5		35	//real joint of right leg 5[rad]
#define DCM_INPUT_REAL_JOINT_R6		36	//real joint of right leg 6[rad]
#define	DCM_INPUT_BODY_ACC_X		37	//body position - x[m/s^2]
#define	DCM_INPUT_BODY_ACC_Y		38	//body position - y[m/s^2]
#define	DCM_INPUT_BODY_ACC_Z		39	//body position - z[m/s^2]
#define DCM_INPUT_BODY_ROT_V_X		40	//body rotation velocity x - [rad/s]
#define DCM_INPUT_BODY_ROT_V_Y		41	//body rotation velocity y - [rad/s]
#define DCM_INPUT_BODY_ROT_V_Z		42	//body rotation velocity z - [rad/s]
#define DCM_INPUT_WAIST_1			43  //real joint of waist yaw	[rad]
#define DCM_INPUT_WAIST_2			44  //real joint of waist pitch	[rad]
#define DCM_INPUT_WAIST_3			45  //real joint of waist roll	[rad]
#define DCM_INPUT_ARM_L1			46  //real joint of left arm 1	[rad]
#define DCM_INPUT_ARM_L2			47  //real joint of left arm 2	[rad]
#define DCM_INPUT_ARM_L3			48  //real joint of left arm 3	[rad]
#define DCM_INPUT_ARM_R1			49  //real joint of right arm 1	[rad]
#define DCM_INPUT_ARM_R2			50  //real joint of right arm 2	[rad]
#define DCM_INPUT_ARM_R3			51  //real joint of right arm 3	[rad]
#define DCM_INPUT_2LINK_FLAG		52	//二连杆控制开关
#define DCM_INPUT_VMC6_EX_K			53	//
#define DCM_TABLE_SIZE_INPUT		54

#define DCM_OUTPUT_STATE_FLAG	0	//system state flag[-]
#define DCM_OUTPUT_JOINT_L1		1	//ref joint of left leg 1[rad]
#define DCM_OUTPUT_JOINT_L2		2	//ref joint of left leg 2[rad]
#define DCM_OUTPUT_JOINT_L3		3	//ref joint of left leg 3[rad]
#define DCM_OUTPUT_JOINT_L4		4	//ref joint of left leg 4[rad]
#define DCM_OUTPUT_JOINT_L5		5	//ref joint of left leg 5[rad]
#define DCM_OUTPUT_JOINT_L6		6	//ref joint of left leg 6[rad]
#define DCM_OUTPUT_JOINT_R1		7	//ref joint of right leg 1[rad]
#define DCM_OUTPUT_JOINT_R2		8	//ref joint of right leg 2[rad]
#define DCM_OUTPUT_JOINT_R3		9	//ref joint of right leg 3[rad]
#define DCM_OUTPUT_JOINT_R4		10	//ref joint of right leg 4[rad]
#define DCM_OUTPUT_JOINT_R5		11	//ref joint of right leg 5[rad]
#define DCM_OUTPUT_JOINT_R6		12	//ref joint of right leg 6[rad]
#define DCM_OUTPUT_WAIST_1		13  //ref joint of waist yaw	[rad]
#define DCM_OUTPUT_WAIST_2		14  //ref joint of waist pitch	[rad]
#define DCM_OUTPUT_WAIST_3		15  //ref joint of waist roll	[rad]
#define DCM_OUTPUT_ARM_L1		16  //ref joint of left arm 1	[rad]
#define DCM_OUTPUT_ARM_L2		17  //ref joint of left arm 2	[rad]
#define DCM_OUTPUT_ARM_L3		18  //ref joint of left arm 3	[rad]
#define DCM_OUTPUT_ARM_R1		19  //ref joint of right arm 1	[rad]
#define DCM_OUTPUT_ARM_R2		20  //ref joint of right arm 2	[rad]
#define DCM_OUTPUT_ARM_R3		21  //ref joint of right arm 3	[rad]
#define DCM_TABLE_SIZE_OUTPUT	22

#define DATA_REAL_B_VRP_X	0
#define DATA_REAL_B_VRP_Y	1
#define DATA_REAL_B_VRP_Z	2
#define DATA_REF_B_VRP_X	3
#define DATA_REF_B_VRP_Y	4
#define DATA_REF_B_VRP_Z	5
#define DATA_COM_DELTA_X	6
#define DATA_COM_DELTA_Y	7
#define DATA_COM_DELTA_Z	8
#define DATA_LEFT_FOOT_DELTA_Z	9
#define DATA_RIGHT_FOOT_DELTA_Z	10
#define	DATA_REAL_VRP_X		11
#define	DATA_REAL_VRP_Y		12
#define	DATA_REAL_VRP_Z		13
#define	DATA_REF_VRP_X		14
#define	DATA_REF_VRP_Y		15
#define	DATA_REF_VRP_Z		16
#define	DATA_REF_DCM_X		17
#define	DATA_REF_DCM_Y		18
#define	DATA_REF_DCM_Z		19
#define DATA_REAL_B_DCM_X	20
#define DATA_REAL_B_DCM_Y	21
#define DATA_REAL_B_DCM_Z	22
#define DATA_POSTURE_CONTROL_DELTA_X	23
#define DATA_POSTURE_CONTROL_DELTA_Y	24
#define	DATA_2LINK_ROLL_REAL_Q1	25
#define	DATA_2LINK_ROLL_REAL_Q2	26
#define	DATA_2LINK_ROLL_REAL_DQ1	27
#define	DATA_2LINK_ROLL_REAL_DQ2	28
#define	DATA_2LINK_ROLL_TAU1		29
#define	DATA_2LINK_ROLL_TAU2		30
#define	DATA_2LINK_ROLL_DELTA_DDQ1	31
#define	DATA_2LINK_ROLL_DELTA_DDQ2	32
#define	DATA_2LINK_ROLL_DELTA_Q1	33
#define	DATA_2LINK_ROLL_DELTA_Q2	34
#define	DATA_2LINK_PITCH_REAL_Q1	35
#define	DATA_2LINK_PITCH_REAL_Q2	36
#define	DATA_2LINK_PITCH_REAL_DQ1	37
#define	DATA_2LINK_PITCH_REAL_DQ2	38
#define	DATA_2LINK_PITCH_TAU1		39
#define	DATA_2LINK_PITCH_TAU2		40
#define	DATA_2LINK_PITCH_DELTA_DDQ1	41
#define	DATA_2LINK_PITCH_DELTA_DDQ2	42
#define	DATA_2LINK_PITCH_DELTA_Q1	43
#define	DATA_2LINK_PITCH_DELTA_Q2	44
#define DATA_TAB_SIZE		45

/*------------------机器人参数-------------------*/
#ifndef ROBOT_PARAMETERS
#define ROBOT_PARAMETERS

//#define		STEP_NUM		15//8//18
#define		SLOW_FAC		1

//length of robot bodys
#define		LEN_UNIT_MM
#define		TIME_UNIT_MS

#define		RAD		deg2rad
#define		DEG		rad2deg

#define QBODY_MAX	RAD(1.0)

#ifdef LEN_UNIT_M
#define		LEN_UNIT			1000.0
#else
#define		LEN_UNIT			1.0
#endif

#ifdef TIME_UNIT_S
#define		TIME_UNIT			1000.0;
#else
#define		TIME_UNIT			(1.0/SLOW_FAC)
#endif

#ifdef _BHR_5_ 
#define		BASE_HIP_X			(0.0/LEN_UNIT)
#define		BASE_HIP_Y			(85.0/LEN_UNIT)//
#define		BASE_HIP_Z			(0.0/LEN_UNIT)//(127/LEN_UNIT)
#define		THIGH_LEN			(312.0/LEN_UNIT)
#define		CRUS_LEN			(312.0/LEN_UNIT)
#define		FOOT_HEIGHT			((112.0)/LEN_UNIT)
#define		FOOT_FONT			((128.0)/LEN_UNIT)//(172.0/LEN_UNIT)
#define		FOOT_BACK			((102.0)/LEN_UNIT)//(129.5/LEN_UNIT)
#define		FOOT_INSIDE			((67.5)/LEN_UNIT)//(68.0/LEN_UNIT)
#define		FOOT_OUTSIDE		((82.5)/LEN_UNIT)//(96.0/LEN_UNIT)
#endif

#ifdef _BHR_6P_ 
#define		BASE_HIP_X			(32.0/LEN_UNIT)
#define		BASE_HIP_Y			(75.0/LEN_UNIT)//
#define		BASE_HIP_Z			(127/LEN_UNIT)//(127/LEN_UNIT)
#define		THIGH_LEN			(330.0/LEN_UNIT)
#define		CRUS_LEN			(330.0/LEN_UNIT)
#define		FOOT_HEIGHT			((112.0)/LEN_UNIT)
#define		FOOT_FONT			((130.0+25.0)/LEN_UNIT)//(172.0/LEN_UNIT)
#define		FOOT_BACK			((90.0+25.0)/LEN_UNIT)//(129.5/LEN_UNIT)
#define		FOOT_INSIDE			((50.0+10.0)/LEN_UNIT)//(68.0/LEN_UNIT)
#define		FOOT_OUTSIDE		((90.0+10.0)/LEN_UNIT)//(96.0/LEN_UNIT)
#endif

#ifdef _BHR_6_ 
#define		BASE_HIP_X			(32.0/LEN_UNIT)
#define		BASE_HIP_Y			(80.0/LEN_UNIT)//
#define		BASE_HIP_Z			(127/LEN_UNIT)//(127/LEN_UNIT)
#define		THIGH_LEN			(320.0/LEN_UNIT)
#define		CRUS_LEN			(320.0/LEN_UNIT)
#define		FOOT_HEIGHT			((112.0)/LEN_UNIT)
#define		FOOT_FONT			((130.0+25.0)/LEN_UNIT)//(172.0/LEN_UNIT)
#define		FOOT_BACK			((90.0+25.0)/LEN_UNIT)//(129.5/LEN_UNIT)
#define		FOOT_INSIDE			((50.0+10.0)/LEN_UNIT)//(68.0/LEN_UNIT)
#define		FOOT_OUTSIDE		((90.0+10.0)/LEN_UNIT)//(96.0/LEN_UNIT)
#endif

#define		UPPER_ARM_LEN		(325.0/LEN_UNIT)//(340.0/LEN_UNIT)//(370.8/LEN_UNIT)
#define		LOWER_ARM_LEN		(360.0/LEN_UNIT)//(378.0/LEN_UNIT)
#define		HAND_R				(30.0/LEN_UNIT)//(30.0/LEN_UNIT)
#define		SHOULDER_WIDTH		(298.5*2.0/LEN_UNIT)//(434.0/LEN_UNIT)	//Len_Shoulder2Neck_Y = 217
#define		NECK_WAIST_Z		((333.0+20.5061)/LEN_UNIT)//(313.0+20.5061/LEN_UNIT)
#define		NECK_WAIST_X		((32.0-20.5061)/LEN_UNIT)
#define		WAIST_Hip_X			(32.0/LEN_UNIT)
#define		WAIST_HIP_Y			(80.0/LEN_UNIT)//75
#define		WAIST_HIP_Z			(214.5/LEN_UNIT)//(199.0/LEN_UNIT)
#define		WAIST_BASE_Z		(87.5/LEN_UNIT)
#define		KNEE_ROTATION		(-103.0)//(-99.13)//(-108.0)
#define		KNEE_WIDTH			(75.0/LEN_UNIT)
#define		KNEE_UP_LEN			100.0//(27.92/LEN_UNIT)
#define		KNEE_DOWN_LEN		0.0//(52.08/LEN_UNIT)
#define		KNEE_HEIGHT			80.0//((62.68+10.0+20.0)/LEN_UNIT)//(64.1372/LEN_UNIT)//(52.1588/LEN_UNIT)//(76.0/LEN_UNIT)
#define		FOOT_SENSOR			(30.0/LEN_UNIT)

//limits of joint angles 
//待添加

//COMMAND
#define		REMAIN_HAND			p->Posture[LEFT_HAND].IKC_FLAG = HAD_SET;p->Posture[RIGHT_HAND].IKC_FLAG = HAD_SET;
#define		REMAIN_FOOT			p->Posture[LEFT_FOOT].IKC_FLAG = HAD_SET;p->Posture[RIGHT_FOOT].IKC_FLAG = HAD_SET;
#define		REMAIN_QBODY		p->Posture[QBODY].IKC_FLAG = HAD_SET;

//number of joints
#define		JOINT_NUMBER		21 
#define		_L_LEFT				0
#define		_L_RIGHT			1


//posture matrix index
#define	_nx		0][0
#define	_ox		0][1
#define	_ax		0][2
#define	_px		0][3
#define	_ny		1][0
#define	_oy		1][1
#define	_ay		1][2
#define	_py		1][3
#define	_nz		2][0
#define	_oz		2][1
#define	_az		2][2
#define	_pz		2][3

//命令
#define CMD_NONE		0
#define	CMD_DCM_WALK	1
#define CMD_DCM_STAND	2
#define CMD_FALL		3
#define	CMD_RECOVER		4

//flag defination
#define IK_SUCCEEDED	 1
#define IK_FAILED		-1	
#define NOT_SET			-1
#define HAD_SET			 1
#define CARTESIAN_MOVE	 1
#define JOINT_MOVE		-1
#define THIRD_MOVE		2

#define NORMAL_MOVE		0
#define ELLIPSE_MOVE	1 
#define DCM_WALKING		2
#define DCM_STOP		3

//Part num
#define PART_NUM		20
#define	BASE			0
#define WAIST			1
#define	LEFT_SHOULDER	2
#define LEFT_HAND		3
#define LEFT_HIP		4
#define LEFT_KNEE		5
//#define LEFT_ANKLE		6
#define LEFT_FOOT		6//7
#define RIGHT_SHOULDER	7//8
#define RIGHT_HAND		8//9
#define RIGHT_HIP		9//10
#define RIGHT_KNEE		10//11
//#define	RIGHT_ANKLE		12
#define RIGHT_FOOT		11//13
#define SPECIAL_MOTION  12
//#define ZMP				13
#define COG				14
#define VRP				15
#define PART_DCM				16
#define LEFT_SENSOR		17
#define RIGHT_SENSOR	18
#define QBODY			19

//Joint num
#define LEFT_ARM_1		0
#define LEFT_ARM_2		1
#define LEFT_ARM_3		2
#define LEFT_LEG_1		3
#define LEFT_LEG_2		4
#define LEFT_LEG_3		5
#define LEFT_LEG_4		6
#define LEFT_LEG_5		7
#define LEFT_LEG_6		8
#define RIGHT_ARM_1		9
#define RIGHT_ARM_2		10
#define RIGHT_ARM_3		11
#define RIGHT_LEG_1		12
#define RIGHT_LEG_2		13
#define RIGHT_LEG_3		14
#define RIGHT_LEG_4		15
#define RIGHT_LEG_5		16
#define RIGHT_LEG_6		17
#define WAIST_1			18
#define WAIST_2			19
#define WAIST_3			20
//Axis
#define AXIS_X			1
#define AXIS_Y			2
#define AXIS_Z			3

//Knee XZ
#define KNEE_X			1
#define KNEE_Z			2

//Walking state
#define WS_NONE			0
#define WS_DS			1
#define WS_LS			2
#define	WS_RS			3

#define MAX_STEP		100

#define		G		9.8

//Init Matrix and Posture
#define UNIT_T { 1.0,0,0,0,0,1.0,0,0,0,0,1.0,0,0,0,0,1.0 }
#define INI_POS {0,0,0,0,0,0}

#endif
/*<-----------------机器人参数------------------>*/

/*-----------------数值计算类型-------------------*/
#ifndef ROBOT_CALCULATE_TYPE
#define ROBOT_CALCULATE_TYPE

#include <math.h>

#define _rrm	  //reserved macro for special needs
#define _rex	
#define _rf_t	double //robot calculate type for functions
#define _rv_t	double //robot calculate type for variables
#define _rv_flg int
#define _rf_flg	int    //robot flag type for return value of function

//math functions
#define _rcos	cos
#define _rsin	sin
#define _rtan	tan
#define _rasin	asin
#define _racos	acos
#define _ratan	atan
#define _ratan2	atan2
#define _rpow	pow
#define _rsqrt	sqrt
#define _rabs	fabs

//the minimum allowed error
#define MIN_ERROR	(1e-6)

#define _pi	3.141592657

#define _ppt(pt,m,n) ((&(pt[m]))[n])
#endif
/*<----------------数值计算类型------------------>*/

/*-----------------机器人结构体-------------------*/

#ifndef ROBOT_STRUCT
#define ROBOT_STRUCT

typedef struct PostureStruct
{
	_rv_t	p[6];
}PostureStruct;

typedef struct TStruct
{
	_rv_t	t[4][4];
}TStruct;


typedef struct BPosture
{
	char			Name[20];
	int				ID;
	int				IKC_FLAG;
	int				MID_IKC_FLAG[6];
	_rv_t			IntPolFac[6][6];
	PostureStruct	Posture;
	PostureStruct	PostureV;
	PostureStruct	PostureA;
	TStruct			T;
}BPosture;

//State struct
typedef struct BRobotState
{
	_rv_t		Time;
	_rv_t		Time2;//切换状态时清零
	_rv_flg		StateFlag;
	_rv_flg		SubStateFlag;
	_rv_flg		SetFlag;
	_rv_t		Joint[JOINT_NUMBER];
	_rv_t		JointV[JOINT_NUMBER];
	_rv_t		JointA[JOINT_NUMBER];
	_rv_t		ForceSensor[2][6];
	BPosture	Posture[PART_NUM];
	double		Body_CoM[3];//基于多连杆模型，根据规划角度算出的质心位置
	double		Body_CoM_V[3];//同上，质心速度
	double		Body_CoM_A[3];//同上，质心加速度
	double		O_Joint[JOINT_NUMBER];//纯规划角度，未加调节的角度
}BRobotState;

//Motion Parametes;
typedef struct MotionSet
{
	int		StepNum;
	int		KneeXZ;
	_rv_t	StepTime[100];
	_rv_t	inter_time;
	_rv_flg	SpecialMode;
	_rv_t	SpecialParameters[10];
	//待续
}MotionSet; 

typedef struct WalkingSet
{
	double	Step_Width;
	int		Step_Num;
	int		Walking_State;
	double	Step_Time;
	double	Step_Time_SS;
	double	Step_Time_DS;
	double	Step_Time_DS_Ini;
	double	Step_Time_DS_End;
	double	Step_Length;
	double	COM_Height;
	double	Foot_Height;
	double  B;

	double	VRP_Point[MAX_STEP][3];
	double	DCM_End[MAX_STEP][3];
	double	DCM_Ini[MAX_STEP][3];
	double	DCM_DS_Ini[MAX_STEP][3];
	double	DCM_DS_End[MAX_STEP][3];
	double	DDCM_DS_Ini[MAX_STEP][3];
	double	DDCM_DS_End[MAX_STEP][3];

	int		Current_Step;
	double	Current_Time;
}WalkingSet;

typedef struct LQR_Control
{
	int track_num;
	double ref_value;
	double real_value;//Left and Right Force Torque

	double ref_value2;
	double real_value2;//Left and Right Force Torque

	double delta;
	double d_delta;
	double dd_delta;

	double last_delta;
	double last_d_delta;

	double limit[2];

	double k[4];

	double T;

	double Td;//lead correction time
	double Alpha;//lead correction ratio
	double c_delta;
	double c_d_delta;

	double ep; //平衡点
}LQR_Control;

typedef struct PID_Control 
{
	double kp;
	double ki;
	double kd;
	double e;
	double de;
	double int_e;
	double limit_min;
	double limit_max;
	double u;
}PID_Control;

typedef struct IMU_DATA
{
	int		init_flag;
	int		init_count;
	double	Omega[3];
	double	last_Omega[3];
	double	lastlast_Omega[3];
	double	d_Omega[3];
	double	last_d_Omega[3];
	double	new_Omega[3];
}IMU_DATA;

typedef struct ViscoelasticModelCompliance6
{
	double			RealJoint[2][6];
	double			SensorFT[2][6];
	double			RawBodyVel[6];
	double			RealBodyVel[6];
	double			RealBodyAcc[6];
	double			InPutCOMV[3];
	double			InputVRP[3];
	double			EstDCM[3];

	double			IMU_Posture_Data[3];
	double			IMU_Posture_Data_d[3];
	double			IMU_Posture_Data_dd[3];
	double			IMU_Posture[3];
	double			IMU_Posture_d[3];
	double			IMU_Posture_dd[3];

	PostureStruct	PostureAdjust[4];
	PostureStruct	RealBodyPosture;
	PostureStruct	RawBodyPosture;

	TStruct			RealBodyT;
	PostureStruct	RealFootPosture[2];
	TStruct			RealFootT[2];

	PID_Control		PID_Single_Foot_ZMP[2][2];

	LQR_Control		LQR[2][6];//Left rx ry rz fx fy fz; Right rx ry rz fx fy fz 
	LQR_Control		LQR_Posture_Control_Ankle[2];
	LQR_Control		LQR_Body_Compliance[6];
	
}ViscoelasticModelCompliance6;

typedef struct TrunkPositionCompliance
{
	double			TuneK[2];
	double			FootZMP[2][2];
	double			Body_FootZMP[2][2];
	LQR_Control		LQR[2];
}TrunkPositionCompliance;

typedef struct DCM_COM_Estimator
{
	double M[2][2];
	double A[2][2];
	double B[2];
	double C[2][2];
	double D[2];
	double T;
	double state_dcm;
	double state_com;
	double input_vrp;
	double output_f;
	double output_ddx;
}DCM_COM_Estimator;

typedef struct Lee_Filter
{
	int first_data_flag;
	int order;
	int data_num;
	double X[20];
	double Y[20];
	double A[11];
	double B[11];
}Lee_Filter;

typedef struct Lee_Estimator
{
	int x_num;
	int u_num;
	int y_num;
	MStruct M;
	MStruct A;
	MStruct B;
	MStruct C;
	MStruct D;
	MStruct X;
	MStruct U;
	MStruct Y;
}Lee_Estimator;

typedef struct TwoLinkModelSet
{
	double I[2];
	double m[2];
	double s[2];
	double Len[2];
}TwoLinkModelSet;

typedef struct TwoLinkModelState
{
	double q[2];
	double dq[2];
	double ddq[2];
	MStruct Tau;
	MStruct M;
	MStruct C;
}TwoLinkModelState;

typedef struct TwoLinkModelControl
{
	double adjust_q[2];
	double adjust_dq[2];
	TwoLinkModelSet Set;
	TwoLinkModelState real_state;	//实际状态，由传感器测得
	TwoLinkModelState model_state;	//模型状态，由理论计算得到
	TwoLinkModelState ref_state;	//参考状态，由规划得到
	MStruct A;
	MStruct X;
	MStruct B;
	MStruct U;
}TwoLinkModelControl;

typedef struct ArmJointVMC
{
	double h;	//手到地的距离
	double dh;	//h差分
	double H;	//设定弹簧阻尼模型开始作用的高度
	double K;	//手-地弹簧阻尼模型刚度
	double B;	//手-地弹簧阻尼模型阻尼
	double delta_joint[3];	//关节角度调节量
	double Torque[3];	//虚拟关节力矩
	MStruct	J;	//手到肩的雅可比矩阵
	MStruct Fw;	//世界坐标系下虚拟力向量
	MStruct Fa;	//手臂坐标系下虚拟力向量
	LQR_Control LQR[3]; //关节粘弹性控制器
}ArmJointVMC;

//Main struct
typedef struct BRobot
{
	_rv_flg		Init_Flag;
	_rv_flg		MotionFlag;
	_rv_flg		Command;
	_rv_t		JointIntPolFac[JOINT_NUMBER][6];
	MotionSet	Squat;
	WalkingSet	DCM;
	BRobotState PastState;
	BRobotState CurrentState;
	BRobotState GoalState;
	BRobotState	MidGoalState;

	IMU_DATA	BW_IMU_Data;
	ArmJointVMC SR_Arm_Control[2];

	ViscoelasticModelCompliance6 VMC6;
	ViscoelasticModelCompliance6 LAST_VMC6;

	TwoLinkModelControl TL_Control[2];

	DCM_COM_Estimator DC_Est[3];
	TrunkPositionCompliance ZMP_TPC;

	Lee_Filter FilterIMU_Acc[3];
	Lee_Filter FilterIMU_Posture[3];
	Lee_Filter FilterIMU_Ang_Vel[3];
	Lee_Filter FilterFT[2][6];

	Lee_Filter FilterHandPos[2];

	Lee_Estimator EstimatorPosture[3];

	double dcmSetTab[DCM_TABLE_SIZE_SET];
	double dcmInputTab[DCM_TABLE_SIZE_INPUT];
	double dcmOutputTab[DCM_TABLE_SIZE_OUTPUT];
	double dcmDataTab[DATA_TAB_SIZE];
}BRobot;



#endif
/*<----------------机器人结构体------------------>*/

/*-----------------基本计算函数-------------------*/
#ifndef ROBOT_BASIC_FUNCTION
#define ROBOT_BASIC_FUNCTION
_rrm _rf_t		deg2rad(const _rv_t deg);
_rrm _rf_t		rad2deg(const _rv_t rad);
_rrm _rf_t		aprox(const _rv_t value, const _rv_t standard);
_rrm _rf_t		aprox_all(const _rv_t value);
_rrm _rf_t		solve_sc(const _rv_t a, const _rv_t b, const _rv_t c, const int num);
_rrm _rf_t		solve_sc_2(const _rv_t factor[6]);
_rrm _rf_flg	equal_check(const _rv_t v1, const _rv_t v2);
_rrm _rf_flg	res_choose(const _rv_t *ang, const _rv_t *ref, const int joint_num, const int res_num);
_rrm _rf_flg	get5_ipf_va0(const _rv_t t, const _rv_t x1, const _rv_t x2, _rv_t f[6]);

#endif
/*<----------------基本计算函数------------------>*/

/*-----------------逆运动学计算函数-------------------*/
#ifndef ROBOT_IK_FUNCTION
#define ROBOT_IK_FUNCTION
_rrm _rf_flg ik_arm(const _rv_t T[4][4], _rv_t ang[3], const _rv_t ref[3]);
_rrm _rf_flg ik_waist(const _rv_t T[4][4], _rv_t ang[3], const _rv_t ref[3]);
_rrm _rf_flg ik_leg(const _rv_t T[4][4], _rv_t ang[6], const _rv_t ref[6]);
_rrm _rf_flg ik_knee(const _rv_flg xz_flg, const _rv_t hip_pos[6], const _rv_t patella_pos[6], const _rv_t foot_pos[6], _rv_t ang[6], const _rv_t ref[6]);
_rrm _rf_flg ik_posture(const _rv_t T[4][4], _rv_t pos[6], const _rv_t ref[6]);
#endif
/*<----------------逆运动学计算函数------------------>*/


/*-----------------运动学计算函数-------------------*/
#ifndef ROBOT_K_FUNCTION
#define ROBOT_K_FUNCTION
//根据姿态向量获取姿态矩阵
_rrm TStruct get_t(const PostureStruct *posture);
//平移矩阵
_rrm TStruct get_move(_rv_t dx, _rv_t dy, _rv_t dz);
//旋转矩阵
_rrm TStruct get_rot(_rv_flg axis, _rv_t angle);
//腰部运动学 基：浮动基
_rrm TStruct get_waist_t(_rv_t *waist_joint);
//手部运动学 基：肩
_rrm TStruct get_arm_t(_rv_t *arm_joint);
//膝部运动学 基：髋
_rrm TStruct get_knee_t(_rv_t *leg_joint);
//脚部运动学 基：髋
_rrm TStruct get_foot_t(_rv_t *leg_joint);
#endif
/*<----------------运动学计算函数------------------>*/

/*-------------------运动规划函数-------------------*/
#ifndef ROBOT_PLAN_FUNCTION
#define ROBOT_PLAN_FUNCTION
_rrm _rf_flg robot_update(BRobot * robot);
_rrm _rf_flg update_current(BRobot *robot);
_rrm _rf_flg get_next(BRobot *robot);
_rrm _rf_flg get_int_pol_fac(BRobot * robot);
_rrm _rf_flg get_next_postrure(BRobot * robot);
_rrm _rf_flg get_joint(BRobot * robot);
_rrm _rf_flg state_trans(BRobot * robot);
_rrm _rf_flg get_body_com(BRobot * robot);
#endif
/*<------------------运动规划函数------------------>*/


/*------------------结构体处理函数------------------*/
#ifndef ROBOT_STRUCT_DISPOSE_FUNCTION
#define ROBOT_STRUCT_DISPOSE_FUNCTION

//机器人结构体初始化函数
_rrm _rf_flg robot_init(BRobot * robot);
_rrm void ikc_flag_reset(BRobot * robot);
_rrm MStruct get_rot_from_t(TStruct T);
_rrm MStruct get_ts_from_t(TStruct T);
_rrm TStruct t_mul(const TStruct * t1, const TStruct * t2);
_rrm TStruct mt_mul(TStruct t1, TStruct t2);
_rrm TStruct t_inv(const TStruct * t);
_rrm void init_filter(Lee_Filter *f, int order, int data_num, double A[], double B[]);
_rrm double filt_data(Lee_Filter * f, double new_data);
_rrm void init_estimator(Lee_Estimator * e, int num[3], double sys_matrix[8][2][2]);
_rrm void estimate_state(Lee_Estimator * e);
#endif
/*<-----------------结构体处理函数----------------->*/

#endif