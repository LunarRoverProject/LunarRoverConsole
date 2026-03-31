#!/bin/bash

echo "=================================================="
echo "  基地局ローンチ --- XBee あり (AT モード)"
echo "=================================================="
echo "  起動: Backend (FastAPI) + Frontend (React)"
echo "  不要: rosbridge (Wi-Fi 通信は使わない)"
echo "=================================================="

# tmux がインストールされているか確認
if ! command -v tmux &> /dev/null; then
    echo "tmux is not installed. Installing..."
    sudo apt update && sudo apt install -y tmux
fi

SESSION="lunar_console_xbee"

# 既存セッションを破棄して再作成
tmux kill-session -t $SESSION 2>/dev/null

echo ""
echo "[確認] main.py の USE_XBEE = True になっていること"
echo "[確認] backend/.env の XBEE_PORT が正しいこと (例: /dev/ttyUSB0 または COM3)"
echo ""
echo "Starting in tmux session: $SESSION ..."

# [Pane 0] 左上: Backend (FastAPI)
tmux new-session -d -s $SESSION -n "Console-XBee" \
    "cd backend && uvicorn main:app --reload --host 0.0.0.0; exec bash"

# [Pane 1] 右上: Frontend (React)
tmux split-window -h \
    "sleep 5 && cd frontend && npm start; exec bash"

# タイル状に整える
tmux select-layout even-horizontal
tmux set-option -g mouse on

echo ""
echo "=================================================="
echo "  起動完了！"
echo "  ブラウザ: http://localhost:3000"
echo "  終了: Ctrl+b → d (デタッチ) または"
echo "        tmux kill-session -t $SESSION"
echo "=================================================="

tmux attach-session -t $SESSION
