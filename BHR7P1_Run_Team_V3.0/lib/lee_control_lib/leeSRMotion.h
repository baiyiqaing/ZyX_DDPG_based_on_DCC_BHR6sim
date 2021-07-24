/*---------------------------------------------------/
	Library for Science Robotics Papaer on Faling During Walking Applied on BHR-6P
	Qingqing Li
	■ Lee, 2019 - 04 - 06, Version <SR - 1.0>
	（1）重构运动控制框架	function : sr_motion
	（2）在线DCM双足运动轨迹规划	leeOnlineDCMWalking
	■ Lee, 2019 - 04 - 11, Version <SR - 2.0>
	（1）北微IMU角速度与角度噪声处理
	（2）基于关节粘弹性模型与末端弹簧阻尼模型的摔倒手臂运动控制
	■ Lee, 2019 - 04 - 13
	（1）北微IMU角速度与角度噪声处理
	（2）基于关节粘弹性模型与末端弹簧阻尼模型的摔倒手臂运动控制
	■ Lee, 2019 - 11 - 29
	添加控制接口，以便用于其他规划程序调用本程序中的控制代码
---------------------------------------------------*/
#ifndef LEE_SR_MOTION
#define LEE_SR_MOTION
#include "DCMWalkingUpdate.h"

#define USE_FALL_ARM_CONTROL

//机器人控制程序入口
void sr_motion(BRobot *robot, DCMWalkingState *dcm);
//运动控制程序入口
void sr_get_next(BRobot *robot, DCMWalkingState *dcm);
//大状态切换
void sr_state_trans(BRobot *robot, DCMWalkingState *dcm);
//运动轨迹调整
void sr_adjust(BRobot *robot, DCMWalkingState *dcm);

//各个状态对应运动规划与控制函数
//质心下移
void sr_com_down(BRobot *robot);
//行走
void sr_dcm_bipe(BRobot *robot, DCMWalkingState *dcm);
//摔倒
void sr_fall(BRobot *robot);
//恢复
void sr_recover(BRobot *robot);
//控制接口
void lee_control(BRobot *robot, DCMWalkingState *dcm);

//摔倒过程中手部位置控制
void sr_hand_falling_control(BRobot *robot);

//关节空间控制（粘弹性模型）
void sr_arm_vmc_init(BRobot *robot);
void sr_arm_vmc_update(BRobot *robot);
void sr_arm_vmc_control(BRobot *robot);
void sr_arm_vmc_end(BRobot * robot);
#endif // !LEE_SR_MOTION