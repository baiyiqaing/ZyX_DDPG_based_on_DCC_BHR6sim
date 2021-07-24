/*---------------------------------------------------/
	Library for Science Robotics Papaer on Faling During Walking Applied on BHR-6P
	Qingqing Li
	�� Lee, 2019 - 04 - 06, Version <SR - 1.0>
	��1���ع��˶����ƿ��	function : sr_motion
	��2������DCM˫���˶��켣�滮	leeOnlineDCMWalking
	�� Lee, 2019 - 04 - 11, Version <SR - 2.0>
	��1����΢IMU���ٶ���Ƕ���������
	��2�����ڹؽ�ճ����ģ����ĩ�˵�������ģ�͵�ˤ���ֱ��˶�����
	�� Lee, 2019 - 04 - 13
	��1����΢IMU���ٶ���Ƕ���������
	��2�����ڹؽ�ճ����ģ����ĩ�˵�������ģ�͵�ˤ���ֱ��˶�����
	�� Lee, 2019 - 11 - 29
	��ӿ��ƽӿڣ��Ա����������滮������ñ������еĿ��ƴ���
---------------------------------------------------*/
#ifndef LEE_SR_MOTION
#define LEE_SR_MOTION
#include "DCMWalkingUpdate.h"

#define USE_FALL_ARM_CONTROL

//�����˿��Ƴ������
void sr_motion(BRobot *robot, DCMWalkingState *dcm);
//�˶����Ƴ������
void sr_get_next(BRobot *robot, DCMWalkingState *dcm);
//��״̬�л�
void sr_state_trans(BRobot *robot, DCMWalkingState *dcm);
//�˶��켣����
void sr_adjust(BRobot *robot, DCMWalkingState *dcm);

//����״̬��Ӧ�˶��滮����ƺ���
//��������
void sr_com_down(BRobot *robot);
//����
void sr_dcm_bipe(BRobot *robot, DCMWalkingState *dcm);
//ˤ��
void sr_fall(BRobot *robot);
//�ָ�
void sr_recover(BRobot *robot);
//���ƽӿ�
void lee_control(BRobot *robot, DCMWalkingState *dcm);

//ˤ���������ֲ�λ�ÿ���
void sr_hand_falling_control(BRobot *robot);

//�ؽڿռ���ƣ�ճ����ģ�ͣ�
void sr_arm_vmc_init(BRobot *robot);
void sr_arm_vmc_update(BRobot *robot);
void sr_arm_vmc_control(BRobot *robot);
void sr_arm_vmc_end(BRobot * robot);
#endif // !LEE_SR_MOTION