// DCC_RUN_ReadConfig.h, Lee <hexb66@bit.edu.cn>
#pragma once
#ifdef __cplusplus
#include <BhrRtxClass\RtxCConfigJson.hpp>
#include <BhrRtxClass\RtxCEvent.hpp>
extern "C" {
#endif
	#include "DCC_RunCon.h"
#ifndef UNDER_RTSS 
	BOOL NRT_ReadDCCRunParms(DCCRunParms *p, LPWSTR filename);
#else
	BOOL RT_InitDCCRunParms();
	BOOL RT_LoadDCCRunParms(DCCRunParms *p);
	void RT_LoadDefalutDCCRunParms(DCCRunParms *p);
	void RT_PrintfDCCRunParms(DCCRunParms *p);
#endif
#ifdef __cplusplus
}
#endif