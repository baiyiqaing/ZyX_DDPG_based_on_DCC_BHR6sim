#!include <rtx.mak>
#Note: There should be no on other sign after the line feed sign,"\".
CC		=	cl
	
CFLAGS_NONRT = /c /nologo /W3 /WX- /O2 /GL /D NDEBUG /D _CONSOLE /D _UNICODE /D UNICODE /Gm- /EHsc /MT /GS /fp:precise /Zc:wchar_t /Zc:forScope /Zc:inline /GR- /Gd /TC		
CFLAGS_NONRT_CPP = /c /nologo /W3 /WX- /O2 /GL /D NDEBUG /D _CONSOLE /D _UNICODE /D UNICODE /Gm- /EHsc /MT /GS /fp:precise /Zc:wchar_t /Zc:forScope /Zc:inline /GR- /Gd

CLINK	=	link
#/align:0x20	

CLINK_NONRT	=  /OUT:Hmi_Process.exe  /INCREMENTAL:NO /NOLOGO /LTCG  RTAPI.LIB KERNEL32.LIB USER32.LIB GDI32.LIB WINSPOOL.LIB COMDLG32.LIB ADVAPI32.LIB SHELL32.LIB OLE32.LIB OLEAUT32.LIB UUID.LIB ODBC32.LIB ODBCCP32.LIB /SUBSYSTEM:CONSOLE /TLBID:1 /DYNAMICBASE /NXCOMPAT  /MACHINE:X64  	
#-----------------------------------------------	
all: Hmi_Process.exe NonRT_Over
	
Hmi_Process.exe: robot_hmi.obj log.obj \
DCC_Run_ReadConfig.obj
	$(CLINK) $(CLINK_NONRT) robot_hmi.obj  log.obj .\objs-RTX64\send_pattern_RTX64.lib \
	DCC_Run_ReadConfig.obj ReadRosLib\ReadRos.lib
 
#----------------------------------------------------------	 
#non-realtime obj		 
robot_hmi.obj:
	$(CC) $(CFLAGS_NONRT) robot_hmi.c
	
log.obj:
	$(CC) $(CFLAGS_NONRT)  .\lib\log.c
	
DCC_Run_ReadConfig.obj:
	$(CC) $(CFLAGS_NONRT_CPP) lib\DCC_Run_ReadConfig.cpp
#----------------------------------------------------------	
#clean:
#	-del *.obj
NonRT_Over:
	move Hmi_Process.exe build\Hmi_Process.exe
	@del *.obj