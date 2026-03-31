@echo off
echo ==================================================
echo   Lunar Rover Console - XBee Mode (Windows)
echo ==================================================
echo   USE_XBEE = True  /  rosbridge 不要
echo ==================================================

echo.
echo [1/2] Starting Backend (FastAPI + XBee Serial)...
start "rover-backend" cmd /k "cd backend && uvicorn main:app --reload --host 0.0.0.0"

echo.
echo Waiting 3 seconds for backend to initialize...
timeout /t 3 >nul

echo.
echo [2/2] Starting Frontend (React)...
start "rover-frontend" cmd /k "cd frontend && npm start"

echo.
echo ==================================================
echo   Launch complete!
echo   XBee ポートの確認: backend\.env の XBEE_PORT
echo   main.py の USE_XBEE = True になっているか確認
echo ==================================================
timeout /t 5 >nul
