@echo off
set ADB=adb
where adb >nul 2>nul || set ADB=%LOCALAPPDATA%\Android\Sdk\platform-tools\adb.exe
"%ADB%" forward tcp:8000 tcp:8000
if errorlevel 1 (
    echo Could not connect. Is the robot plugged in by USB?
) else (
    echo Connected. Open http://localhost:8000
)
timeout /t 5
