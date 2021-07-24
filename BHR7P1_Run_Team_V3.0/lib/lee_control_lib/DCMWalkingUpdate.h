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
		
	�� Lee, 2019-1-19, ���IMU��̬��΢�ֵ��޷��˲�, ����Ƕ����������
	�� Lee, 2019-1-20, 
		�Ľ�IMU��̬���˲�����Ϊ����΢�ֵ��޷��˲�, ����Ƕ���������⣻
		��ӽǶȡ����ٶȱ任����ƣ�
		���ӻ����˰������ļ��㣻
		�Ľ���̬���������������ز�̫�ã�һ�����ػ��С�
	�� Lee, 2019-02-24, ��ӻ��ڶ�����ģ�͵���̬����
	�� Lee, 2019-03-03, ��ֲ��BHR5ƽ̨��
	�� Lee, 2019-03-05, ��������̬���ƸĽ�
	�� Lee, 2019-03-08, ���ӻ���������ͷ�ļ���leeRobotConfig.h�������ڻ����������йصĲ������ڸ�ͷ�ļ���ͳһ����
						���ӻ���ճ����ģ�͵�Body��˳����
	�� Lee, 2019-03-09, BHR5�޸���̬���������ݴ�����룬ȡ���Ǽ��ٶ��޷��˲�
	�� Lee, 2019-03-26, ճ���Կ���������ƽ������
	�� Lee, 2019-03-28, �汾��SR��Ϊ Science Robotics �����޸�
		���������ֱ۵ĹؽڽǶ����

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