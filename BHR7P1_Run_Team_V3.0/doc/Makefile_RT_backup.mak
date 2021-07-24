#!include <rtx.mak>
!include <MakeFile_ReadRos_Lib.mak>
#Note: There should be no on other sign after the line feed sign,"\".
CC		=	cl


CFLAGS_RT =	/c   /nologo /W3 \
	/WX- /Ox /D _AMD64_ /D UNDER_RTSS /D NDEBUG /D _UNICODE /D UNICODE /Gm- /EHsc \
	/MT /GS- /fp:precise /Zc:wchar_t /Zc:forScope /Zc:inline /GR- /Gd /TC /openmp-  \
	/Gd /TC /analyze- /errorReport:prompt /openmp-
	
CFLAGS_RT_CPP =	/c   /nologo /W3 \
	/WX- /Ox /D _AMD64_ /D UNDER_RTSS /D NDEBUG /D _UNICODE /D UNICODE /Gm- /EHsc \
	/MT /GS- /fp:precise /Zc:wchar_t /Zc:forScope /Zc:inline /GR- /Gd /openmp-  \
	/Gd /analyze- /errorReport:prompt /openmp-

CLINK	=	link
#/align:0x20	
CLINK_RT = /OUT:rtx_process.rtss /INCREMENTAL:NO /NOLOGO  STARTUPCRT.LIB LIBCMT.LIB LIBUCRT.LIB LIBVCRUNTIME.LIB  RTX_RTSS.LIB /NODEFAULTLIB:KERNEL32.LIB /NODEFAULTLIB:USER32.LIB /NODEFAULTLIB:LIBC /NODEFAULTLIB:LIBCD /MANIFEST:NO /SUBSYSTEM:NATIVE /Driver /TLBID:1 /ENTRY:"_RtapiProcessEntryCRT" /MACHINE:X64 \
    .\objs-RTX64\can_G_ds402_RTX64.lib  .\objs-RTX64\PCM3680I_RTX64.lib
#-----------------------------------------------	
all: rtx_process.rtss RT_Over

#rtx_process.rtss: rtx_process.obj 
rtx_process.rtss: rtx_process.obj hardware_conf.obj  SPP_Cntl_RTX.obj rtdata.obj \
 Tra_Generate.obj  DemoControl.obj \
 leeCpp2C.obj leeLegLengthControl.obj \
 Resistant_Compliance.obj Uneven_Trailblazer.obj DCC_motion.obj CoM_Stabilizer.obj QP_balance.obj DCC_RunCon.obj DCC_filters.obj\
 leeSRMotion.obj  DCMWalkingUpdate.obj leeMatrix.obj leeRobotTool.obj leeRobotPlan.obj leeOnLineDCMWalking.obj \
 ChzCpp2C.obj Chz_Base.obj Chz_Calculator.obj Chz_Filter.obj Chz_Footstep.obj Chz_FootDown.obj Chz_Include.obj Chz_Kinematics.obj Chz_LandingForce.obj Chz_LowLevelFootStep.obj Chz_PerspectiveMapping.obj Chz_RobotParam.obj Chz_Spline.obj Chz_SwingFootControl.obj Chz_WaistComp.obj \
 DCC_Run_ReadConfig.obj
	$(CLINK) $(CLINK_RT) rtx_process.obj hardware_conf.obj SPP_Cntl_RTX.obj rtdata.obj \
	Tra_Generate.obj DemoControl.obj \
	leeCpp2C.obj leeLegLengthControl.obj \
	Resistant_Compliance.obj Uneven_Trailblazer.obj DCC_motion.obj CoM_Stabilizer.obj QP_balance.obj DCC_RunCon.obj DCC_filters.obj \
	leeSRMotion.obj  DCMWalkingUpdate.obj leeMatrix.obj leeRobotTool.obj leeRobotPlan.obj  leeOnLineDCMWalking.obj \
	ChzCpp2C.obj  Chz_Base.obj  Chz_Calculator.obj  Chz_Filter.obj  Chz_Footstep.obj  Chz_FootDown.obj  Chz_Include.obj  Chz_Kinematics.obj  Chz_LandingForce.obj  Chz_LowLevelFootStep.obj  Chz_PerspectiveMapping.obj  Chz_RobotParam.obj  Chz_Spline.obj  Chz_SwingFootControl.obj  Chz_WaistComp.obj \
	DCC_Run_ReadConfig.obj
#	StampTool.exe rtx_process.rtss	
#----------------------------------------------------------
rtx_process.obj:	
	$(CC) $(CFLAGS_RT) rtx_process.c	
hardware_conf.obj:	
	$(CC) $(CFLAGS_RT)  lib\hardware_conf.c 
rtdata.obj:
	$(CC) $(CFLAGS_RT)  lib\rtdata.c 
Tra_Generate.obj:
	$(CC) $(CFLAGS_RT)  lib\Tra_Generate.c 
SPP_Cntl_RTX.obj:
	$(CC) $(CFLAGS_RT)  lib\SPP_Cntl_RTX.c
DemoControl.obj:
	$(CC) $(CFLAGS_RT) lib\DemoControl.c
	
leeCpp2C.obj:
	$(CC) $(CFLAGS_RT_CPP) lib\leeCpp2C.cpp
leeLegLengthControl.obj:
	$(CC) $(CFLAGS_RT_CPP) lib\leeLegLengthControl.cpp
leeSRMotion.obj:
	$(CC) $(CFLAGS_RT) lib\lee_control_lib\leeSRMotion.c
DCMWalkingUpdate.obj:
	$(CC) $(CFLAGS_RT) lib\lee_control_lib\DCMWalkingUpdate.c
leeMatrix.obj:	
	$(CC) $(CFLAGS_RT) lib\lee_control_lib\leeMatrix.c
leeRobotTool.obj:	
	$(CC) $(CFLAGS_RT) lib\lee_control_lib\leeRobotTool.c
leeRobotPlan.obj:	
	$(CC) $(CFLAGS_RT) lib\lee_control_lib\leeRobotPlan.c
leeOnLineDCMWalking.obj:
	$(CC) $(CFLAGS_RT) lib\lee_control_lib\leeOnLineDCMWalking.c

Resistant_Compliance.obj:
	$(CC) $(CFLAGS_RT_CPP) lib\Resistant_Compliance.c	
Uneven_Trailblazer.obj:
	$(CC) $(CFLAGS_RT_CPP) lib\Uneven_Trailblazer.c	
DCC_motion.obj:
	$(CC) $(CFLAGS_RT_CPP) lib\DCC_motion.c	
CoM_Stabilizer.obj:
	$(CC) $(CFLAGS_RT_CPP) lib\CoM_Stabilizer.c	
QP_balance.obj:
	$(CC) $(CFLAGS_RT_CPP) lib\QP_balance.c	
DCC_RunCon.obj:
	$(CC) $(CFLAGS_RT_CPP) lib\DCC_RunCon.c	
DCC_filters.obj:
	$(CC) $(CFLAGS_RT_CPP) lib\DCC_filters.c	
	
ChzCpp2C.obj:
	$(CC) $(CFLAGS_RT_CPP) lib\ChzCpp2C.cpp
Chz_Base.obj:
	$(CC) $(CFLAGS_RT_CPP) lib\Chz_control_lib\Chz_Base.cpp
Chz_Calculator.obj:
	$(CC) $(CFLAGS_RT_CPP) lib\Chz_control_lib\Chz_Calculator.cpp
Chz_Filter.obj:
	$(CC) $(CFLAGS_RT_CPP) lib\Chz_control_lib\Chz_Filter.cpp
Chz_Footstep.obj:
	$(CC) $(CFLAGS_RT_CPP) lib\Chz_control_lib\Chz_Footstep.cpp
Chz_FootDown.obj:
	$(CC) $(CFLAGS_RT_CPP) lib\Chz_control_lib\Chz_FootDown.cpp
Chz_Include.obj:
	$(CC) $(CFLAGS_RT_CPP) lib\Chz_control_lib\Chz_Include.cpp
Chz_Kinematics.obj:
	$(CC) $(CFLAGS_RT_CPP) lib\Chz_control_lib\Chz_Kinematics.cpp
Chz_LandingForce.obj:
	$(CC) $(CFLAGS_RT_CPP) lib\Chz_control_lib\Chz_LandingForce.cpp
Chz_LowLevelFootStep.obj:
	$(CC) $(CFLAGS_RT_CPP) lib\Chz_control_lib\Chz_LowLevelFootStep.cpp
Chz_PerspectiveMapping.obj:
	$(CC) $(CFLAGS_RT_CPP) lib\Chz_control_lib\Chz_PerspectiveMapping.cpp
Chz_RobotParam.obj:
	$(CC) $(CFLAGS_RT_CPP) lib\Chz_control_lib\Chz_RobotParam.cpp
Chz_Spline.obj:
	$(CC) $(CFLAGS_RT_CPP) lib\Chz_control_lib\Chz_Spline.cpp
Chz_SwingFootControl.obj:
	$(CC) $(CFLAGS_RT_CPP) lib\Chz_control_lib\Chz_SwingFootControl.cpp
Chz_WaistComp.obj:
	$(CC) $(CFLAGS_RT_CPP) lib\Chz_control_lib\Chz_WaistComp.cpp
	
DCC_Run_ReadConfig.obj:
	$(CC) $(CFLAGS_RT_CPP) lib\DCC_Run_ReadConfig.cpp
#----------------------------------------------------------	
#clean:
#	-del *.obj
RT_Over:
	@del *.obj
	copy *.rtss build\