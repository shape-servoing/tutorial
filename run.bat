@echo off
rem Windows launcher: serves the demos (docs/) and opens them in the browser; --tests runs the tests.
cd /d "%~dp0"
where py >nul 2>nul
if %errorlevel%==0 (
    py -3 run.py %*
) else (
    python run.py %*
)
