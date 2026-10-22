@echo off
title AeroSim-Control Simulation Launcher
echo ========================================================
echo   AEROSIM-CONTROL: FLIGHT CONTROL SYSTEMS LAB
echo ========================================================
echo.
echo Starting local web server and launching your browser...
echo.

python run_simulation.py

if %ERRORLEVEL% NEQ 0 (
    echo Python server failed to start, launching index.html directly...
    start index.html
)
pause
