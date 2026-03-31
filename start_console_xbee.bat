@echo off
echo ==================================================
echo   基地局ローンチ --- XBee あり (AT モード)
echo ==================================================
echo   起動: Backend (FastAPI) + Frontend (React)
echo   不要: rosbridge (Wi-Fi 通信は使わない)
echo ==================================================

echo.
echo [確認] main.py の USE_XBEE = True になっていること
echo [確認] backend\.env の XBEE_PORT が正しいこと (例: COM3)
echo.

echo [1/2] Starting Backend (FastAPI + XBee Serial)...
start "backend-xbee" cmd /k "cd backend && uvicorn main:app --reload --host 0.0.0.0"

echo.
echo Waiting 3 seconds for backend to initialize...
timeout /t 3 >nul

echo.
echo [2/2] Starting Frontend (React)...
start "frontend" cmd /k "cd frontend && npm start"

echo.
echo ==================================================
echo   起動完了！
echo   ブラウザ: http://localhost:3000
echo ==================================================
timeout /t 5 >nul
