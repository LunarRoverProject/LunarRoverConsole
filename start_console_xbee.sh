#!/bin/bash

echo "=================================================="
echo "  基地局ローンチ --- XBee あり (AT モード)"
echo "=================================================="
echo "  起動: QoS Bridge + Web Video Server"
echo "         Backend (FastAPI) + Frontend (React)"
echo "  不要: rosbridge"
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

# --- ポート 8000 (Backend) の残留プロセスを解放 ---
echo "Checking port 8000..."
if fuser 8000/tcp &>/dev/null; then
    echo "Port 8000 is in use. Killing existing process..."
    fuser -k 8000/tcp
    sleep 1
fi

echo "Starting in tmux session: $SESSION ..."

# [Pane 0] 左上: QoS Bridge
tmux new-session -d -s $SESSION -n "Console-XBee" \
    "source /opt/ros/humble/setup.bash \
     && cd $(dirname "$(realpath "$0")")/ros_code_ws/src \
     && echo '[QoS Bridge] Starting...' \
     && python3 qos_bridge.py; exec bash"

# [Pane 1] 右上: Web Video Server
tmux split-window -h \
    "source /opt/ros/humble/setup.bash \
     && echo '[web_video_server] Starting...' \
     && ros2 run web_video_server web_video_server; exec bash"

# [Pane 2] 左下: Backend (FastAPI)
tmux select-pane -t 0
tmux split-window -v \
    "sleep 5 \
     && cd $(dirname "$(realpath "$0")")/backend \
     && echo '[Backend] Starting...' \
     && uvicorn main:app --reload --host 0.0.0.0; exec bash"

# [Pane 3] 右下: Frontend (React)
tmux select-pane -t 1
tmux split-window -v \
    "sleep 5 \
     && cd $(dirname "$(realpath "$0")")/frontend \
     && echo '[Frontend] Starting...' \
     && npm start; exec bash"

# タイル状に整える
tmux select-layout tiled
tmux set-option -g mouse on

echo ""
echo "=================================================="
echo "  起動完了！"
echo "  ブラウザ: http://localhost:3000"
echo "  終了: Ctrl+b → d (デタッチ) または"
echo "        tmux kill-session -t $SESSION"
echo "=================================================="

tmux attach-session -t $SESSION
