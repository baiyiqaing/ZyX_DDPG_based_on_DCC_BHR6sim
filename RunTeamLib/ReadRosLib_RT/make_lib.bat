@set CurrentLibPath=%cd%
@cd ../BasicEnv
@call runteamlib_setenv.bat
@cd %CurrentLibPath%

@color f1
@nmake -f makefile.mak
@color 06
@pause