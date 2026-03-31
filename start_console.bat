@echo off
echo ==================================================
echo   Lunar Rover Console - One-Click Start (Windows)
echo ==================================================

echo.
echo Starting ROS Bridge (WebSocket) in WSL...
start "ros-bridge" wsl bash -c "source /opt/ros/humble/setup.bash && ros2 launch rosbridge_server rosbridge_websocket_launch.xml; exec bash"

echo.
echo Starting Web Video Server (Camera Socket) in WSL...
start "web-video" wsl bash -c "source /opt/ros/humble/setup.bash && ros2 run web_video_server web_video_server; exec bash"

echo.
echo Starting QoS Bridge in WSL...
start "qos-bridge" wsl bash -c "source /opt/ros/humble/setup.bash && cd ros_code_ws/src && python3 qos_bridge.py; exec bash"

echo.
echo Starting Backend (FastAPI + XBee) in a new window...
start "rover-backend" cmd /k "cd backend && uvicorn main:app --reload --host 0.0.0.0"

echo.
echo Waiting 5 seconds for systems to initialize...
timeout /t 5 >nul

echo.
echo Starting Frontend (React) in a new window...
start "rover-frontend" cmd /c "cd frontend && npm start"

echo.
echo Launch sequence initiated! 
echo Keep the terminal windows open.
echo You can safely close this orchestrator window.
timeout /t 5 >nul
