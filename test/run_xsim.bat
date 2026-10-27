@echo off
rem run_xsim.bat -- compile + run a Pinwheel testbench with Vivado xsim.
rem   test\run_xsim.bat tb_demo              behavioural flop memory
rem   test\run_xsim.bat tb_demo sram         IHP RM_IHPSG13_1P_512x16 model (macro\RM_IHPSG13_1P_512x16_c2_bm_bist)
rem   test\run_xsim.bat tb_rand PW_TRACE     extra define
setlocal
set TOP=%~1
if "%TOP%"=="" set TOP=tb_demo
if "%VIVADO%"=="" set VIVADO=C:\AMDDesignTools\2026.1\Vivado\bin
set ROOT=%~dp0..
set SRAMDIR=%ROOT%\macro\RM_IHPSG13_1P_512x16_c2_bm_bist
set DEF=
set SRAM=
if /i "%~2"=="sram" (
  if not exist "%SRAMDIR%\RM_IHPSG13_1P_core_behavioral_bm_bist.v" (
    echo IHP SRAM model missing in %SRAMDIR%
    exit /b 1
  )
  rem FUNCTIONAL = IHP's zero-delay model (the timing model clocks via $setuphold-delayed nets)
  set DEF=-d PW_MEM_IHP_SRAM -d FUNCTIONAL
  set SRAM="%SRAMDIR%\RM_IHPSG13_1P_core_behavioral_bm_bist.v" "%SRAMDIR%\RM_IHPSG13_1P_512x16_c2_bm_bist.v"
) else if not "%~2"=="" (
  set DEF=-d %~2
)
if not exist "%~dp0xsim_work" mkdir "%~dp0xsim_work"
pushd "%~dp0xsim_work"
call "%VIVADO%\xvlog.bat" --sv -d SIM %DEF% -i "%ROOT%\test" %SRAM% ^
  "%ROOT%\src\pw_fifo.v" "%ROOT%\src\pw_mem.v" "%ROOT%\src\pw_lane.v" "%ROOT%\src\pw_core.v" ^
  "%ROOT%\src\pw_host_spi.v" "%ROOT%\src\pw_top.v" "%ROOT%\src\tt_um_pinwheel.v" ^
  "%ROOT%\test\%TOP%.sv" > xvlog.out || (findstr ERROR xvlog.out & popd & exit /b 1)
call "%VIVADO%\xelab.bat" %TOP% -s %TOP%_snap --timescale 1ns/1ps --debug off > xelab.out || (findstr ERROR xelab.out & popd & exit /b 1)
call "%VIVADO%\xsim.bat" %TOP%_snap -R > %TOP%.log
findstr /C:"FAIL" /C:"ok:" /C:"====" /C:"ERROR" /C:"Fatal" %TOP%.log
popd
endlocal
