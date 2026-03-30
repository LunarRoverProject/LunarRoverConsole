@echo off
echo ==================================================
echo   Lunar Rover Console - One-Click Start (Windows)
echo ==================================================
echo.
echo Starting Frontend (React) in a new window...
start "rover-frontend" cmd /c "cd frontend && npm start"

echo.
echo Starting Backend (FastAPI + XBee) in a new window...
start "rover-backend" cmd /k "cd backend && call venv\Scripts\Activate.ps1 && uvicorn main:app --reload --host 0.0.0.0"

echo.
echo Launch sequence initiated! 
echo Keep both black terminal windows open.
echo You can safely close this window.
timeout /t 5 >nul
