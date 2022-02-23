#include "ReadRos.h"

RosEstmData rosData;
ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::TransformStamped> startposeSub("/Tcw2", &chattercCallback);
FILE *ros_fp;

void chattercCallback(const geometry_msgs::TransformStamped& goal_pose)
{
	//printf("[t: %ld x:%f , y:%f , z:%f ,X:%f , Y:%f , Z:%f ,W:%f]\n", goal_pose.header.stamp.sec, goal_pose.transform.translation.x, goal_pose.transform.translation.y, goal_pose.transform.translation.z, goal_pose.transform.rotation.x, goal_pose.transform.rotation.y, goal_pose.transform.rotation.z, goal_pose.transform.rotation.w);
	rosData.dTime = goal_pose.header.stamp.sec;
	rosData.dPosX = goal_pose.transform.translation.x;
	rosData.dPosY = goal_pose.transform.translation.y;
	rosData.dPosZ = goal_pose.transform.translation.z;
	rosData.dAngX = goal_pose.transform.rotation.x;
	rosData.dAngY = goal_pose.transform.rotation.y;
	rosData.dAngZ = goal_pose.transform.rotation.z;
	rosData.flag++;
	if (ros_fp != NULL)
	{
		fprintf(ros_fp, "%ld\t%ld\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n",
			goal_pose.header.stamp.sec, goal_pose.header.stamp.nsec,
			rosData.dPosX, rosData.dPosY, rosData.dPosZ,
			rosData.dAngX, rosData.dAngY, rosData.dAngZ
		);
		fflush(ros_fp);
	}
}

void NRT_InitRosEstm(LPSTR strRosMasterIP)
{
	ros_fp = fopen("ros_data.dat", "w");
	printf("\nConnecting to ros server for STATE ESTIMATION at %s\n", strRosMasterIP);
	nh.initNode(strRosMasterIP);
	rosData.flag = 1;
	nh.subscribe(startposeSub);
}


void NRT_TestRosEstm()
{
	printf("[Recieve ROS Infor]: %.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\t%.3f\n",
		rosData.dTime,
		rosData.dPosX, rosData.dPosY, rosData.dPosZ,
		rosData.dAngX, rosData.dAngY, rosData.dAngZ
	);
}

void NRT_ReadRosEstm(RosEstmData *pRosData)
{
	//printf("\nRead from ros\n");
	nh.spinOnce();
}

bhrrtx::RtxNRTComputingEngine<RosEstmData, double> ceWTEst(L"WuTongEst");

inline void funcReadRosData(RosEstmData * pRos, double * unuse)
{
	NRT_ReadRosEstm(pRos);
	//pRos->dTime += 0.004;
}
// RosEstmData rosData; // This has been defined in "readros.lib"
double dUnusedTemp;

int NRT_InitWTEstEngine(LPSTR strRosMasterIP)
{
	ceWTEst.SetProcessPriority();
	ceWTEst.SetComputaion(&dUnusedTemp, funcReadRosData);
	ceWTEst.SetInterface(&rosData);
	ceWTEst.SetWaitTime(1000);
	ceWTEst.InitNRT();
	NRT_InitRosEstm(strRosMasterIP);
	for (int i = 0; i < 10; i++) {
		printf("Ros Data Test: ");
		NRT_ReadRosEstm(&rosData); SleepEx(100, FALSE);
		NRT_TestRosEstm();
	}
	SleepEx(100, FALSE);
	return 0;
}

int NRT_StartWTEstEngine()
{
	ceWTEst.StartCompThread();
	return 0;
}

int NRT_EndWTEstEngine()
{
	ceWTEst.SetRunFlag(false);
	SleepEx(200, FALSE);
	return 0;
}