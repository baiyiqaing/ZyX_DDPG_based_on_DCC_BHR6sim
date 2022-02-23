/*---------------------------------------------------/
	DCM library for simulation, BHR-6 and BHR-6P
	Qingqing Li
	DCM-UnevenWalker-1 - 20190106
	DCM-UnevenWalker-1.1 - 20190106
		Add estimator for DCM and COM by MatLab function "dlqe"
	DCM-UnevenWalker-1.2 - 20190110
		Online tuning for lead compensation	
	DCM-UnevenWalker-1.3 - 20190111
		ZMP based Trunk Position Compliance	
	DCM-UnevenWalker-1.4 - 20190118
		Sensor Data Filter
		
	■ Lee, 2019-1-19, 添加IMU姿态角微分的限幅滤波, 解决角度跳变的问题
	■ Lee, 2019-1-20, 
		改进IMU姿态角滤波，改为二阶微分的限幅滤波, 解决角度跳变的问题；
		添加角度、角速度变换与估计；
		增加机器人伴随矩阵的计算；
		改进姿态控制器，二阶力矩不太好，一阶力矩还行。
	■ Lee, 2019-02-24, 添加基于二连杆模型的姿态控制
	■ Lee, 2019-03-03, 移植到BHR5平台上
	■ Lee, 2019-03-05, 二连杆姿态控制改进
	■ Lee, 2019-03-08, 增加机器人配置头文件（leeRobotConfig.h），将于机器人类型有关的参数放在该头文件中统一定义
						增加基于粘弹性模型的Body柔顺控制
	■ Lee, 2019-03-09, BHR5修改姿态传感器数据处理代码，取消角加速度限幅滤波
	■ Lee, 2019-03-26, 粘弹性控制中增加平衡点控制
	■ Lee, 2019-03-28, 版本：SR，为 Science Robotics 作出修改
		增加腰与手臂的关节角度输出

------------------------------------------------*/
#ifndef DCM_WALKING_UPDATE
#define DCM_WALKING_UPDATE
#include "leeRobotTool.h"
#include "leeRobotPlan.h"
#include "leeMatrix.h"

int dcmWalkState;
double DCM_JOINT_TRANS[6];

_rrm  void two_link_model_init(BRobot *bhr6);
_rrm  void dcm_walk_init(BRobot *bhr6);
_rrm  void vmc6_init(BRobot *bhr6);
_rrm  void load_data(BRobot *bhr6);
_rrm  void dcm_init(BRobot *bhr6);
_rrm  void dcm_com_est_init(BRobot *bhr6);
_rrm  void zmp_tpc_init(BRobot *bhr6);
_rrm  void init_filters(BRobot *bhr6);
_rrm  void init_estimators(BRobot *bhr6);
_rrm  void get_imu_data(BRobot *bhr6);
_rrm  void dcm_walking(BRobot *bhr6);
_rrm  void BW_IMU_init(BRobot *robot);

#endif