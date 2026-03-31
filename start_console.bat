@echo off
echo ==================================================
echo   基地局ローンチ --- XBee なし (Wi-Fi モード)
echo ==================================================
echo   起動: ROS Bridge + Web Video Server (WSL)
echo          Backend (FastAPI) + Frontend (React)
echo ==================================================

echo.
echo [確認] main.py の USE_XBEE = False になっていること
echo [確認] backend\.env の ROSBRIDGE_IP がローバーの IP であること
echo.

echo [1/4] Starting ROS Bridge (WebSocket) in WSL...
start "rosbridge" wsl bash -c "source /opt/ros/humble/setup.bash && ros2 launch rosbridge_server rosbridge_websocket_launch.xml; exec bash"

echo.
echo [2/4] Starting Web Video Server in WSL...
start "web-video" wsl bash -c "source /opt/ros/humble/setup.bash && ros2 run web_video_server web_video_server; exec bash"

echo.
echo [3/4] Starting QoS Bridge in WSL...
start "qos-bridge" wsl bash -c "source /opt/ros/humble/setup.bash && cd ros_code_ws/src && python3 qos_bridge.py; exec bash"

echo.
echo Waiting 5 seconds for ROS to initialize...
timeout /t 5 >nul

echo.
echo [4/4] Starting Backend (FastAPI)...
start "backend-wifi" cmd /k "cd backend && uvicorn main:app --reload --host 0.0.0.0"

echo.
echo Waiting 3 seconds for backend to initialize...
timeout /t 3 >nul

echo.
echo Starting Frontend (React)...
start "frontend" cmd /k "cd frontend && npm start"

echo.
echo ==================================================
echo   起動完了！
echo   ブラウザ: http://localhost:3000
echo ==================================================
timeout /t 5 >nul
