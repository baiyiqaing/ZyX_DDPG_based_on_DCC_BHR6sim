// ReadRos.h Lee <hexb66@bit.edu.cn>
#pragma once
#include "RosDataType.h"

#ifdef __cplusplus
#include <iostream>
#include <tchar.h>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <cmath>
#include <vector>
#include <memory>
#include <windows.h>
#include <fstream>

#undef DELETE
#undef ERROR

#include "ros_lib\ros.h"
#include "ros_lib\nav_msgs\Path.h"
#include "ros_lib\visualization_msgs\Marker.h"
#include "ros_lib\visualization_msgs\MarkerArray.h"
#include "ros_lib\geometry_msgs\Pose.h"
#include "ros_lib\geometry_msgs\TransformStamped.h"
#include "ros_lib\geometry_msgs\Quaternion.h"
#include "ros_lib\geometry_msgs\Point.h"
#include "ros_lib\std_msgs\String.h"
#include "ros_lib\std_msgs\Header.h"

#include "..\BhrRtxClass\RtxCNRTComputingEngine.hpp"
//#include <BhrRtxClass\RtxCNRTComputingEngine.hpp>
using namespace bhrrtx;

extern "C"{
#endif
	extern RosEstmData rosData;
	void NRT_InitRosEstm(LPSTR strRosMasterIP);
	void NRT_TestRosEstm();
	void NRT_ReadRosEstm(RosEstmData *pRosData);

	int NRT_InitWTEstEngine(LPSTR strRosMasterIP);
	int NRT_StartWTEstEngine();
	int NRT_EndWTEstEngine();
#ifdef __cplusplus
}
void chattercCallback(const geometry_msgs::TransformStamped& goal_pose);
#endif