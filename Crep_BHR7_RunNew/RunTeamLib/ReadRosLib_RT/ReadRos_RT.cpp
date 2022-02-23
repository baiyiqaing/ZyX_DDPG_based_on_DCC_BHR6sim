#include "ReadRos_RT.h"

bhrrtx::RtxNRTComputingEngine<RosEstmData> ceWTEst(L"WuTongEst");

int RT_InitWTEst(RosEstmData * pRosData, DWORD dwWaitTimeMs)
{
	ceWTEst.SetInterface(pRosData);
	ceWTEst.SetWaitTime(dwWaitTimeMs);
	return ceWTEst.InitRT();
}

int RT_GetWTEst()
{
	return ceWTEst.RecvResult();
}
