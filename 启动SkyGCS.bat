@echo off
rem ============================================================
rem  SkyGCS  one-click launcher
rem  Starts the prebuilt Release build (fastest) if present,
rem  otherwise falls back to the Debug build.
rem ============================================================
setlocal
cd /d "%~dp0"

set "EXE=build-release\skygcs.exe"
if exist "%EXE%" goto :run

set "EXE=build\skygcs.exe"
if exist "%EXE%" goto :run

echo [ERROR] skygcs.exe not found.
echo         Run the build first, e.g.:
echo         cmake --build build-release   (or:  cmake --build build)
pause
exit /b 1

:run
echo Starting SkyGCS: %EXE%
start "" "%EXE%"
exit /b 0
