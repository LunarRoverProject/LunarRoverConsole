#!/bin/bash

echo "=================================================="
echo "  Lunar Rover Console - One-Click Start (Ubuntu)"
echo "=================================================="

# 終了時にバックグラウンドプロセスも一緒にキルするための設定
trap 'echo "Stopping all servers..."; kill $FRONTEND_PID $BACKEND_PID $ROSBRIDGE_PID $WEBVIDEO_PID; exit' SIGINT SIGTERM

echo -e "\n[1/4] Starting ROS Bridge (WebSocket)..."
cd ros_bridge_ws
# ROS 2 環境を読み込んで起動
source install/setup.bash || true
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
ROSBRIDGE_PID=$!
cd ..

echo -e "\n[2/4] Starting Web Video Server (Camera Socket)..."
ros2 run web_video_server web_video_server &
WEBVIDEO_PID=$!

echo -e "\n[3/4] Starting Backend (FastAPI)..."
cd backend
uvicorn main:app --reload --host 0.0.0.0 &
BACKEND_PID=$!
cd ..

echo -e "\nWaiting 5 seconds for backend and ROS nodes to initialize..."
sleep 5

echo -e "\n[4/4] Starting Frontend (React)..."
cd frontend
npm start &
FRONTEND_PID=$!
cd ..

echo -e "\n🚀 Launch sequence initiated!"
echo "Press [Ctrl + C] to stop all servers at any time."

# バックグラウンドプロセスの完了を待つ（Ctrl+Cが押されるまでループ）
wait $FRONTEND_PID $BACKEND_PID $ROSBRIDGE_PID $WEBVIDEO_PID
