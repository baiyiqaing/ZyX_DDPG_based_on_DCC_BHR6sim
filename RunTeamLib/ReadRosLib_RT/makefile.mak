CC		=	cl
	
CFLAGS_RT_CPP =	/c   /nologo /W3 \
	/WX- /O2 /Ot /D _AMD64_ /D UNDER_RTSS /D NDEBUG /D _UNICODE /D UNICODE /Gm- /EHsc \
	/MT /GS- /fp:precise /Zc:wchar_t /Zc:forScope /Zc:inline /GR- /Gd /openmp-  \
	/Gd /analyze- /errorReport:prompt /openmp-

CLINK	=	link
CLIB = lib
#/align:0x20	

CLIB_RT = /OUT:ReadRos_RT.lib /NOLOGO	/MACHINE:X64
#RTAPI.LIB KERNEL32.LIB USER32.LIB GDI32.LIB WINSPOOL.LIB COMDLG32.LIB ADVAPI32.LIB SHELL32.LIB OLE32.LIB OLEAUT32.LIB UUID.LIB ODBC32.LIB ODBCCP32.LIB /SUBSYSTEM:CONSOLE  /MACHINE:X64  

all: ReadRos_RT.lib clean

ReadRos_RT.lib: ReadRos_RT.obj
	$(CLIB) $(CLIB_RT) ReadRos_RT.obj 
	
ReadRos_RT.obj:
	$(CC) $(CFLAGS_RT_CPP) ReadRos_RT.cpp

clean:
	@del *.obj