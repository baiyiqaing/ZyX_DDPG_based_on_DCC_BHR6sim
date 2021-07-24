#!include <rtx.mak>
#Note: There should be no on other sign after the line feed sign,"\".
CC		=	@cl
#/align:0x20	
CFLAGS_RT =	/c   /nologo /W3 \
	/WX- /O2 /Ot /D _AMD64_ /D UNDER_RTSS /D NDEBUG /D _UNICODE /D UNICODE /Gm- /EHsc \
	/MDd /GS- /fp:precise /Zc:wchar_t /Zc:forScope /Zc:inline /GR- /Gd /TC /openmp-  \
	/Gd /TC /analyze- /errorReport:prompt /openmp-
CFLAGS_RT_CPP =	/c   /nologo /W3 \
	/WX- /O2 /Ot /D _AMD64_ /D UNDER_RTSS /D NDEBUG /D _UNICODE /D UNICODE /Gm- /EHsc \
	/MDd /GS- /fp:precise /Zc:wchar_t /Zc:forScope /Zc:inline /GR- /Gd /openmp-  \
	/Gd /analyze- /errorReport:prompt /openmp-
	
CLIB = @lib
CLIB_RT = /OUT:ChzControl.lib /NOLOGO	/MACHINE:X64
#-----------------------------------------------	
all: ChzControl.lib ChzControl_Clean

ChzControl.lib: Chz_Base.obj Chz_Calculator.obj Chz_Eigen.obj Chz_Filter.obj Chz_Footstep.obj Chz_FootDown.obj Chz_Include.obj Chz_Kinematics.obj Chz_LandingForce.obj Chz_LowLevelFootStep.obj Chz_PerspectiveMapping.obj Chz_RobotParam.obj Chz_Spline.obj Chz_SwingFootControl.obj Chz_WaistComp.obj
	$(CLIB) $(CLIB_RT) Chz_Base.obj Chz_Calculator.obj Chz_Eigen.obj Chz_Filter.obj Chz_Footstep.obj Chz_FootDown.obj Chz_Include.obj Chz_Kinematics.obj Chz_LandingForce.obj Chz_LowLevelFootStep.obj Chz_PerspectiveMapping.obj Chz_RobotParam.obj Chz_Spline.obj Chz_SwingFootControl.obj Chz_WaistComp.obj
#----------------------------------------------------------
Chz_Base.obj:
	$(CC) $(CFLAGS_RT_CPP) Chz_Base.cpp
Chz_Calculator.obj:
	$(CC) $(CFLAGS_RT_CPP) Chz_Calculator.cpp
Chz_Eigen.obj:
	$(CC) $(CFLAGS_RT_CPP) Chz_Eigen.cpp
Chz_Filter.obj:
	$(CC) $(CFLAGS_RT_CPP) Chz_Filter.cpp
Chz_Footstep.obj:
	$(CC) $(CFLAGS_RT_CPP) Chz_Footstep.cpp
Chz_FootDown.obj:
	$(CC) $(CFLAGS_RT_CPP) Chz_FootDown.cpp
Chz_Include.obj:
	$(CC) $(CFLAGS_RT_CPP) Chz_Include.cpp
Chz_Kinematics.obj:
	$(CC) $(CFLAGS_RT_CPP) Chz_Kinematics.cpp
Chz_LandingForce.obj:
	$(CC) $(CFLAGS_RT_CPP) Chz_LandingForce.cpp
Chz_LowLevelFootStep.obj:
	$(CC) $(CFLAGS_RT_CPP) Chz_LowLevelFootStep.cpp
Chz_PerspectiveMapping.obj:
	$(CC) $(CFLAGS_RT_CPP) Chz_PerspectiveMapping.cpp
Chz_RobotParam.obj:
	$(CC) $(CFLAGS_RT_CPP) Chz_RobotParam.cpp
Chz_Spline.obj:
	$(CC) $(CFLAGS_RT_CPP) Chz_Spline.cpp
Chz_SwingFootControl.obj:
	$(CC) $(CFLAGS_RT_CPP) Chz_SwingFootControl.cpp
Chz_WaistComp.obj:
	$(CC) $(CFLAGS_RT_CPP) Chz_WaistComp.cpp

#----------------------------------------------------------	
ChzControl_Clean:
	@del *.obj