CC		=	cl
	
CFLAGS_NONRT = /c /nologo /W3 /WX- /O2 /GL /D NDEBUG /D _CONSOLE /D _UNICODE /D UNICODE /Gm- /EHsc /MT /GS /fp:precise /Zc:wchar_t /Zc:forScope /Zc:inline /GR- /Gd /TC		
CFLAGS_NONRT_CPP = /c /nologo /W3 /WX- /O2 /GL /D NDEBUG /D _CONSOLE /D _UNICODE /D UNICODE /Gm- /EHsc /MT /GS /fp:precise /Zc:wchar_t /Zc:forScope /Zc:inline /GR- /Gd

CLINK	=	link
CLIB = lib
#/align:0x20	

CLINK_NONRT_LIB	=  /OUT:ReadRos.lib /NOLOGO /LTCG
#RTAPI.LIB KERNEL32.LIB USER32.LIB GDI32.LIB WINSPOOL.LIB COMDLG32.LIB ADVAPI32.LIB SHELL32.LIB OLE32.LIB OLEAUT32.LIB UUID.LIB ODBC32.LIB ODBCCP32.LIB /SUBSYSTEM:CONSOLE  /MACHINE:X64  

all: ReadRos.lib clean

ReadRos.lib: ReadRos.obj duration.obj time.obj WindowsSocket.obj
	$(CLIB) $(CLINK_NONRT_LIB) ReadRos.obj duration.obj time.obj WindowsSocket.obj
	
ReadRos.obj:
	$(CC) $(CFLAGS_NONRT_CPP) ReadRos.cpp
duration.obj:
	$(CC) $(CFLAGS_NONRT_CPP) ros_lib\duration.cpp
time.obj:
	$(CC) $(CFLAGS_NONRT_CPP) ros_lib\time.cpp
WindowsSocket.obj:
	$(CC) $(CFLAGS_NONRT_CPP) ros_lib\WindowsSocket.cpp

clean:
	@del *.obj