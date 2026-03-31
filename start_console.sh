#!/bin/bash

echo "=================================================="
echo "  Lunar Rover Console - tmux 4 Splits (Ubuntu)"
echo "=================================================="

# tmux がインストールされているか確認
if ! command -v tmux &> /dev/null
then
    echo "tmux is not installed. Please install it with: sudo apt install tmux"
    echo "Attempting to install tmux..."
    sudo apt update && sudo apt install -y tmux
fi

SESSION="lunar_rover_session"

# すでに同じ名前のセッションが裏で動いていれば強制終了して新しく作り直す
tmux kill-session -t $SESSION 2>/dev/null

echo "Starting servers in a 4-pane layout..."

# [Pane 1] 左上: ROS Bridge
tmux new-session -d -s $SESSION -n "Console" "cd ros_bridge_ws && ros2 launch rosbridge_server rosbridge_websocket_launch.xml; exec bash"

# [Pane 2] 右上: Web Video Server (左右に2分割)
tmux split-window -h "ros2 run web_video_server web_video_server; exec bash"

# [Pane 3] 左下: Backend (FastAPI) (pane 0=左上 を選んで上下に分割)
tmux select-pane -t 0
tmux split-window -v "cd backend && uvicorn main:app --reload --host 0.0.0.0; exec bash"

# [Pane 4] 右下: Frontend (React) (pane 2=右上 を選んで上下に分割)
tmux select-pane -t 2
tmux split-window -v "sleep 5 && cd frontend && npm start; exec bash"

# [Pane 5] 追加: QoS Bridge
tmux split-window -v "cd ros_code && python3 qos_bridge.py; exec bash"

# すべてのペインが同じ大きさになるようにタイル状に整える
tmux select-layout tiled

# マウス操作でペインのサイズを変えられるように設定（おまけ）
tmux set-option -g mouse on

# セッションを画面に表示！
# 注: 終了するときはこのターミナル画面で「Ctrl+b を押したあと d」を押すか、
#    tmux kill-session -t lunar_rover_session と入力します。
tmux attach-session -t $SESSION
