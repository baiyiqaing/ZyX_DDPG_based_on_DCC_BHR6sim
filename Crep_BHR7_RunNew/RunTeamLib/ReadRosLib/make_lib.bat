@set CurrentLibPath=%cd%
@cd ../BasicEnv
@call runteamlib_setenv.bat
@cd %CurrentLibPath%
@set INCLUDE=%INCLUDE%;.\ros_lib

REM @set INCLUDE=%INCLUDE%;..\

@color f1
@nmake -f makefile.mak
@color 06
@pause