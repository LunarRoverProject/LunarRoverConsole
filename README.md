# 月面ローバー制御コンソール

## 概要
このプロジェクトは、月面ローバーを制御するためのコンソールアプリケーションです。フロントエンド、バックエンド、およびROS (Robot Operating System) ノードで構成されています。

## 特徴
- **フロントエンド**: ローバーの状態表示、コマンド送信のためのユーザーインターフェース。
- **バックエンド**: フロントエンドとROSノード間の通信を仲介し、データ処理を行います。
- **ROSノード**: ローバーのセンサーデータ（GPS、IMUなど）の取得、モーター制御コマンドの送信など、ローバーとの直接的なインタラクションを担当します。

## プロジェクト構成
```
.
├── backend/      # バックエンド (FastAPI, WebSocketサーバー)
├── frontend/     # フロントエンド (Reactアプリケーション)
├── ros_bridge/   # ROSブロック (ROS 2ワークスペース)
└── ros_code/     # ROSノードスクリプト 
```

## 手順

### 1. 全体のセットアップ

1.  **リポジトリのクローン**
    ```bash
    git clone [リポジトリのURL]
    cd LunarRoverConsole
    ```

2.  **フロントエンドのセットアップ**
    ```bash
    cd frontend
    npm install
    cd ..
    ```

3.  **ROS Bridgeのビルド**
    `ros_bridge` ディレクトリで、ワークスペースをビルドします。
    ```bash
    cd ros_bridge
    colcon build
    cd ..
    ```

---

### 2. フロントエンドの起動

**新しいターミナル**で、フロントエンドを起動します。
```bash
cd frontend
npm start
```

---

### 3. バックエンドの起動

**新しいターミナル**で、バックエンドを起動します。
```bash
cd backend
# 仮想環境をアクティベート
source venv/bin/activate 
# または .\venv\Scripts\Activate.ps1

uvicorn main:app --reload --host 0.0.0.0
```

---

### 4. ROS Bridgeの起動

**新しいターミナル**で、ROS Bridgeを起動します。
```bash
cd ros_bridge
# 環境設定ファイルを読み込む
source install/setup.bash

ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

---

### 5. カメラ映像サーバーの起動 (ローバー側PC)

カメラの映像を超低遅延で配信するために、ローバーに搭載されたUbuntuマシンで以下の2つのプロセスを動かしてください。

**A. QoSブリッジの起動 (新しいターミナル)**
カメラが送信する`BEST_EFFORT`（高頻度）なデータを、ビデオサーバーが受け取れる`RELIABLE`へと変換します。
```bash
cd ros_code
python3 qos_bridge.py
```

**B. web_video_serverの起動 (新しいターミナル)**
ブラウザ向けに映像をストリーミング配信するサーバーを起動します。
```bash
ros2 run web_video_server web_video_server
```