#pragma once
#include "../ReadRosLib/RosDataType.h"
#ifdef __cplusplus
#include "../BhrRtxClass/RtxCNRTComputingEngine.hpp"
using namespace bhrrtx;
extern "C" {
#endif
	int RT_InitWTEst(RosEstmData *pRosData, DWORD dwWaitTimeMs);
	int RT_GetWTEst();
#ifdef __cplusplus
}
#endif