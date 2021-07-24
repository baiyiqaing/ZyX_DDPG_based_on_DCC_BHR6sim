@REM call run_rtx_process.bat
@REM 2 seconds
@REM ping -n 2 127.0.0.1 >null
@REM call run_hmi_process.bat

@color F1
@REM rtsskill 1 
@REM ping -n 5 127.0.0.1 >null 
SearchRTPID.exe 
rtssrun rtx_process.rtss
@ping -n 5 127.0.0.1 >null  
@Hmi_Process.exe
