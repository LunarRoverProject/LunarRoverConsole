#!/bin/bash

echo "=================================================="
echo "  Lunar Rover Console - One-Click Start (Ubuntu)"
echo "=================================================="

# 終了時にバックグラウンドプロセスも一緒にキルするための設定
trap 'echo "Stopping both servers..."; kill $FRONTEND_PID $BACKEND_PID; exit' SIGINT SIGTERM

echo -e "\n[1/2] Starting Backend (FastAPI)..."
cd backend
uvicorn main:app --reload --host 0.0.0.0 &
BACKEND_PID=$!
cd ..

echo -e "\nWaiting 5 seconds for Backend to initialize..."
sleep 5

echo -e "\n[2/2] Starting Frontend (React)..."
cd frontend
npm start &
FRONTEND_PID=$!
cd ..

echo -e "\n🚀 Launch sequence initiated!"
echo "Press [Ctrl + C] to stop both servers at any time."

# バックグラウンドプロセスの完了を待つ（Ctrl+Cが押されるまでループ）
wait $FRONTEND_PID $BACKEND_PID
