@echo off
@color 17
@doskey ls=dir $1 $2 
@doskey rm = del $1
@doskey cp = copy $1 $2
@doskey vi = Notepad++ $1
@doskey edit = devenv $1 
@set CURRENT_PATH=%cd%
@set CURRENT_DISK=%~d0
echo -----set compiler and linker for RTX64 3.2 combined with MS VS in Win10---- 
echo .............waiting.............

if "%COMPUTERNAME%"=="LEELAB" (
rem 不用call的话，会进到SetMacro_LeeLab.bat中出不来（reminded by 朱西硕）
	call SetMacro_LeeLab.bat
) else if "%COMPUTERNAME%"=="LEE" (
	call SetMacro_Lee.bat
) else if "%COMPUTERNAME%"=="BHR-RUNTEAM-NUC" (
	call SetMacro_BHRRunTeamNUC.bat
) else if "%COMPUTERNAME%"=="DESKTOP-94N41UI" (
	call SetMacro_LKDCC.bat
) else (
	echo Computer Name Not Found, current name is %COMPUTERNAME%
	echo .............break.............
	goto EXIT_SET
)

@%VS_DISK%
@cd %VS_BIN_PATH%
@call vcvars64.bat
@set PATH=%PATH%;%NOTEPAD_PATH%
@set PATH=%PATH%;%VS_IDE_PATH%
@set INCLUDE=%INCLUDE%;%RTX_INCLUDE%
@set LIB=%LIB%;%RTX_LIB_PATH%
@%CURRENT_DISK%
@cd %CURRENT_PATH%
@set INCLUDE=%INCLUDE%;%BHR_LIB_PATH%
@set PATH=%PATH%;%BHR_LIB_PATH%
@set LIB=%LIB%;%BHR_LIB_PATH%

@color 06

echo .............over.............

:EXIT_SET