@echo off
@cls
@color 17
echo .............delete obsolete.obj files.............
@del *.obj
REM SearchCurDirectory.exe
@del build\Hmi_Process.exe
@del build\rtx_process.rtss

echo .............Start to compile and link.............
REM @nmake -s -f Makefile_rtx64.mak
@nmake -s -f Makefile_RT.mak
echo .............ʵʱ�������..........................
REM @del *.obj
REM @nmake -f MakeFile_ReadRos_Lib.mak
@nmake -s -f Makefile_NonRT.mak
echo .............��ʵʱ�������..........................
echo .............delete temporary files.............
REM @del *.obj
@color 06
@date /t 
@time /t
