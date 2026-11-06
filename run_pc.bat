@echo off
setlocal enabledelayedexpansion
chcp 65001 >nul
cd /d "%~dp0"

echo ============================================
echo   AI Smart Greenhouse - PC Monitor v3
echo ============================================
echo.

set "PY="

rem --- Step 1: python resolvable via PATH? ---
where python >nul 2>nul
if not errorlevel 1 (
    python -c "import serial" >nul 2>nul
    if not errorlevel 1 set "PY=python"
)

rem --- Step 2: probe known locations ---
if "!PY!"=="" (
    for %%P in (
        "%USERPROFILE%\.workbuddy\binaries\python\versions\3.13.12\python.exe"
        "%USERPROFILE%\.workbuddy\binaries\python\versions\3.12.0\python.exe"
        "%LOCALAPPDATA%\Programs\Python\Python313\python.exe"
        "%LOCALAPPDATA%\Programs\Python\Python312\python.exe"
        "%LOCALAPPDATA%\Programs\Python\Python311\python.exe"
        "C:\Python313\python.exe"
        "C:\Python312\python.exe"
        "C:\Python311\python.exe"
    ) do (
        if "!PY!"=="" (
            if exist "%%~P" (
                "%%~P" -c "import serial" >nul 2>nul
                if not errorlevel 1 set "PY=%%~P"
            )
        )
    )
)

rem --- Step 3: python found but deps missing ---
if "!PY!"=="" (
    for %%P in (
        "%USERPROFILE%\.workbuddy\binaries\python\versions\3.13.12\python.exe"
        "%LOCALAPPDATA%\Programs\Python\Python313\python.exe"
        "%LOCALAPPDATA%\Programs\Python\Python312\python.exe"
        "C:\Python313\python.exe"
    ) do (
        if "!PY!"=="" (
            if exist "%%~P" set "PY=%%~P"
        )
    )
    if "!PY!"=="" (
        echo [ERROR] No python interpreter found.
        echo Install Python 3 and tick "Add python.exe to PATH",
        echo then run:  python -m pip install -r requirements.txt
        echo.
        pause
        exit /b 1
    )
    echo Dependencies missing on: !PY!
    echo Installing dependencies, please wait 10-60 seconds...
    echo.
    "!PY!" -m pip install -r "requirements.txt"
    if errorlevel 1 (
        echo.
        echo [ERROR] pip install failed. Check your network, then retry.
        pause
        exit /b 1
    )
)

if not exist "pc\AI_farm_v3.py" (
    echo [ERROR] pc\AI_farm_v3.py not found next to this script.
    echo Do not move run_pc.bat away from the project folder.
    echo.
    pause
    exit /b 1
)

echo Using: !PY!
echo Starting GUI...
echo.
"!PY!" pc\AI_farm_v3.py

echo.
echo --------------------------------------------
echo Program closed. If it crashed, scroll up and
echo copy everything in red, then send it to B.
echo --------------------------------------------
pause
