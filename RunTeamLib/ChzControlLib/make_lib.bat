@set CurrentLibPath=%cd%
@cd ../BasicEnv
@call runteamlib_setenv.bat
@cd %CurrentLibPath%

REM @set INCLUDE=%INCLUDE%;..\

@color f1
@nmake -f makefile.mak
@color 06
@pause