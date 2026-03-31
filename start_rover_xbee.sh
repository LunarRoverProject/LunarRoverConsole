#!/bin/bash

echo "=================================================="
echo "  Lunar Rover - XBee Mode Startup (Ubuntu)"
echo "=================================================="
echo "  起動するプロセス:"
echo "   [1] xbee_rover_bridge  (XBee ↔ ROS 2)"
echo "   [2] qos_bridge         (カメラ QoS 変換)"
echo "   [3] web_video_server   (カメラ映像配信)"
echo "=================================================="

# tmux がインストールされているか確認
if ! command -v tmux &> /dev/null; then
    echo "tmux is not installed. Installing..."
    sudo apt update && sudo apt install -y tmux
fi

SESSION="lunar_rover_xbee"

# 既存セッションを破棄して再作成
tmux kill-session -t $SESSION 2>/dev/null

# XBEE_PORT が未設定なら /dev/ttyUSB0 をデフォルトに
XBEE_PORT="${XBEE_PORT:-/dev/ttyUSB0}"
XBEE_BAUD_RATE="${XBEE_BAUD_RATE:-115200}"

echo "Using XBEE_PORT=$XBEE_PORT  XBEE_BAUD_RATE=$XBEE_BAUD_RATE"
echo "Starting in tmux session: $SESSION"

# ---- ペイン構成 ----
# [Pane 0] 左上: XBee ブリッジ
tmux new-session -d -s $SESSION -n "Rover-XBee" \
    "source /opt/ros/humble/setup.bash \
     && export XBEE_PORT=$XBEE_PORT \
     && export XBEE_BAUD_RATE=$XBEE_BAUD_RATE \
     && cd $(dirname "$0")/ros_code_ws/src \
     && echo '[XBee Bridge] Starting...' \
     && python3 xbee_rover_bridge.py; exec bash"

# [Pane 1] 右上: QoS ブリッジ
tmux split-window -h \
    "source /opt/ros/humble/setup.bash \
     && cd $(dirname "$0")/ros_code_ws/src \
     && echo '[QoS Bridge] Starting...' \
     && python3 qos_bridge.py; exec bash"

# [Pane 2] 左下: web_video_server
tmux select-pane -t 0
tmux split-window -v \
    "source /opt/ros/humble/setup.bash \
     && echo '[web_video_server] Starting...' \
     && ros2 run web_video_server web_video_server; exec bash"

# タイル状に整える
tmux select-layout tiled
tmux set-option -g mouse on

echo ""
echo "All processes started!"
echo "Tip: Ctrl+b + d でデタッチ / tmux kill-session -t $SESSION で終了"
echo ""

tmux attach-session -t $SESSION
