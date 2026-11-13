@echo off
setlocal
set "WING_CODE_DIR=%~dp0"
set "WING_PYTHON=%USERPROFILE%\.airfoil-wing-venv\Scripts\python.exe"
if not exist "%WING_PYTHON%" (
    py -3.11 -m venv "%USERPROFILE%\.airfoil-wing-venv" || exit /b 1
)
"%WING_PYTHON%" -c "import cadquery" >nul 2>&1
if errorlevel 1 (
    "%WING_PYTHON%" -m pip install cadquery || exit /b 1
)
if "%~1"=="" (
    "%WING_PYTHON%" -B "%WING_CODE_DIR%airfoil_wing_to_step.py"
) else (
    "%WING_PYTHON%" -B "%WING_CODE_DIR%airfoil_wing_to_step.py" "%~1"
)
exit /b %errorlevel%
