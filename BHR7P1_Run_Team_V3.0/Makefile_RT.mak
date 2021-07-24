#!include <rtx.mak>
# !include <MakeFile_ReadRos_Lib.mak>
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
 Resistant_Compliance.obj Uneven_Trailblazer.obj DCC_motion.obj CoM_Stabilizer.obj QP_balance.obj DCC_RunCon.obj DCC_filters.obj TPCMPC.obj dcc_con_base.obj\
 leeSRMotion.obj  DCMWalkingUpdate.obj leeMatrix.obj leeRobotTool.obj leeRobotPlan.obj leeOnLineDCMWalking.obj \
 ChzCpp2C.obj DCC_Run_ReadConfig.obj
	$(CLINK) $(CLINK_RT) rtx_process.obj hardware_conf.obj SPP_Cntl_RTX.obj rtdata.obj \
	Tra_Generate.obj DemoControl.obj \
	leeCpp2C.obj leeLegLengthControl.obj \
	Resistant_Compliance.obj Uneven_Trailblazer.obj DCC_motion.obj CoM_Stabilizer.obj QP_balance.obj DCC_RunCon.obj DCC_filters.obj TPCMPC.obj dcc_con_base.obj\
	leeSRMotion.obj  DCMWalkingUpdate.obj leeMatrix.obj leeRobotTool.obj leeRobotPlan.obj  leeOnLineDCMWalking.obj \
	ChzCpp2C.obj DCC_Run_ReadConfig.obj \
	ChzControlLib\ChzControl.lib
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
	
TPCMPC.obj:
	
	$(CC) $(CFLAGS_RT_CPP) lib\TPCMPC.c	
	
dcc_con_base.obj:
	
	$(CC) $(CFLAGS_RT_CPP) lib\Dcc_lib\dcc_con_base.c
	
	
ChzCpp2C.obj:
	
	$(CC) $(CFLAGS_RT_CPP) lib\ChzCpp2C.cpp
	
	
DCC_Run_ReadConfig.obj:
	
	$(CC) $(CFLAGS_RT_CPP) lib\DCC_Run_ReadConfig.cpp
	
	
#----------------------------------------------------------	
#clean:
#	-del *.obj
RT_Over:
	@del *.obj
	move *.rtss build\ 