@echo off
REM ---------------------------------------------------------------------------
REM RoboMaster Mission Control - standalone launcher (Windows)
REM
REM Run this from your OWN terminal / by double-clicking it.  It needs no
REM internet connection: join the robot's Wi-Fi (RMEP-xxxxxx) and start it.
REM
REM Do NOT start the panel from inside another tool's session.  If that session
REM is killed while the robot is driving, the process dies with a non-zero
REM velocity still latched in the chassis and the robot keeps rolling.
REM
REM   run_panel.bat                       simulation mode
REM   run_panel.bat --mode real           physical robot over Wi-Fi AP
REM ---------------------------------------------------------------------------
cd /d "%~dp0"
python main.py panel %*
echo.
echo Panel closed. Chassis stop was sent on exit.
pause
