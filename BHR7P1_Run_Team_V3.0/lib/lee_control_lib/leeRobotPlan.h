/*--------------------------------------------------------------/
■ Lee, 2019-02-24, 添加基于二连杆动力学模型的姿态控制
	□ posture_control_2_link_model
	□ two_link_model_get_Tau
	□ two_link_model_get_M
	□ two_link_model_get_C
■ Lee, 2019-03-05, 添加二连杆动力学模型实时更新代码（杆长、惯量）
	□ two_link_model_update_Model
■ Lee, 2019-03-09, 添加单足坐标系内ZMP点位置的控制，以计算各脚的参考力矩
	□ single_foot_zmp_update
	□ single_foot_zmp_control
■ Lee, 2019-03-26, 柔顺控制中增加平衡点控制
/*-------------------------------------------------------------*/

#ifndef LEE_ROBOT_PLAN
#define LEE_ROBOT_PLAN
#include "leeMatrix.h"
#include "leeRobotConfig.h"
#include "leeRobotTool.h"
#include "leeOnlineDCMWalking.h"

#define _BHR_6P_BW_ //BHR-6P, 北微（BW）IMU 单元

#define ONLINE_TUNE
//#define STAND_TEST
#define USE_VMC6_CONTROL
//#define USE_FOOT_COMPLIANCE_POSTURE_CONTROL
//#define USE_VMC6_BODY_CONTROL
//#define USE_BODY_COMPLIANCE_POSTURE_CONTROL
//#define USE_VRP_DCM_TRACK
//#define ZMP_BASED_TPC_CONTROL
#define USE_FT_SENSOR_DATA
//#define TWO_LINK_MODEL_CONTROL
//#define TWO_LINK_MODEL_FULL_DYNAMICS
//#define USE_FT_FILTER
//#define USE_SINGLE_FOOT_ZMP_CONTROL
//#define USE_EQUILIBRIUM_POINT_CONTROL

#define STAGE_HEIGHT 0.0

#define VMC6_TUNE_K	{0.0, 0.0, 0.0, 0.0, 0.0, 0.5}
#define TPC_TUNE_K_X	1
#define TPC_TUNE_K_Y	1
#define POSC_ANKLE_ROLL		1
#define POSC_ANKLE_PITCH	1
#define VMC6_BODY_TUNE_K {0.8,0.8,1,0,0,0}
#define EPC_K {0,0,0,0,0,1}
#define VRP_DCM_TRACK_TUNE_K {0,0,0}
#define VRP_TRACK_TUNE_K	0
#define DCM_TRACK_TUNE_K	0

#ifdef STAND_TEST
#define STEP_NUM	STATE_DCM_STAND
#else
#define	STEP_NUM	STATE_NUM
#endif

//机器人SR运动状态定义
//大状态
#define SR_STATE_COM_DOWN	1
#define SR_STATE_DCM_BIPE	2
#define SR_STATE_FALL		3
#define SR_STATE_RECOVER	4
#define SR_STATE_CONTROL	5
//小状态
#define SR_SUB_STATE_NONE			0
#define SR_SUB_STATE_DCM_BIPE_DS	1
#define SR_SUB_STATE_DCM_BIPE_LS	2
#define SR_SUB_STATE_DCM_BIPE_RS	3
#define SR_SUB_STATE_FALLING		4
#define SR_SUB_STATE_FALLEN			5
#define SR_SUB_STATE_RECOVERING		6
#define SR_SUB_STATE_RECOVERED		7



_rrm _rf_flg set_goal(BRobot *robot);
_rrm _rf_flg posture_adjust(BRobot * robot);
_rrm _rf_flg get_next_state(BRobot * robot);
_rrm _rf_flg time_init(BRobot * robot);
_rrm _rf_flg get_dcm_p(double pt, double * dcm_ds_ini, double * ddcm_ds_ini, double * dcm_ds_end, double * ddcm_ds_end, double t, double * dcm, double * ddcm);

_rrm void get_ft_data(BRobot *robot);
_rrm void get_real_ft(BRobot *robot);
_rrm void get_ref_ft(BRobot *robot);
_rrm void vmc6_update(BRobot *robot);
_rrm void vmc6_control(BRobot *robot);
_rrm void lqr_control(LQR_Control *lqr, double tune_k);
_rrm void lead_correction(LQR_Control *lqr, LQR_Control * last_lqr);
_rrm void vrp_dcm_track(BRobot * robot, double real_vrp[3], double real_dcm[3], double vrp[3], double dcm[3], double cog[3], double b_real_vrp[3], double b_real_dcm[3], double b_vrp[3], double b_dcm[3]);
_rrm void calulate_vrp_by_com(BRobot * robot);
_rrm void dcm_com_est(DCM_COM_Estimator * dce);
_rrm void zmp_tpc_update(BRobot *robot, double vrp[3], double cog[3]);
_rrm void zmp_tpc(BRobot *robot);
_rrm void posture_control_ankle(BRobot *robot);

/*------------------------------------/
2019-02-24 基于二连杆模型的姿态控制:
	Tau = -Kp*(Q-Qd)-Kd*(dQ-dQd);
	delta_ddQ = M^-1*Tau-M^-1*C+Md^-1*Cd
/------------------------------------*/
_rrm void posture_control_2_link_model(BRobot *robot);
_rrm void two_link_model_get_Tau(BRobot *robot);
_rrm MStruct two_link_model_get_M(double q1, double q2, double dq1, double dq2, TwoLinkModelSet *set);
_rrm MStruct two_link_model_get_C(double q1, double q2, double dq1, double dq2, TwoLinkModelSet *set);
_rrm void two_link_model_update_Model(BRobot *robot);
_rrm void body_vmc_update(BRobot *robot);

_rrm void pid_control(PID_Control *pid, double tune_k);
_rrm void single_foot_zmp_update(BRobot *robot);
_rrm void single_foot_zmp_control(BRobot *robot); 
//_rrm void get_real_zmp(BRobot * robot);
//_rrm void get_ref_zmp(BRobot * robot);

/*------平衡点控制-------*/
_rrm void equilibrium_point_control(BRobot *robot);
#endif