# 月面ローバー制御コンソール

## 概要
このプロジェクトは、月面ローバーを制御するためのコンソールアプリケーションです。
`backend/main.py` 内の `USE_XBEE` 変数を変更することで、通信方式を以下の2つから切り替えられます。

1. **`USE_XBEE = True` の場合（ハイブリッド構成）**
   映像データはWi-Fi（`web_video_server`）経由で、GPSや操作コマンドなどのテレメトリデータは **XBee（シリアル通信）** 経由で送受信します。

2. **`USE_XBEE = False` の場合（Wi-Fi構成）**
   すべてWi-Fi経由で送受信します。

## プロジェクト構成
```
.
├── backend/            # バックエンド (FastAPI, pyserial: XBee通信の仲介)
├── frontend/           # フロントエンド (Reactアプリケーション)
├── ros_code/           # Ubuntuローバー側で動かすROS中継スクリプト群
├── start_console.bat   # Windows用の一斉起動ファイル
├── start_console.sh    # Ubuntu用の一斉起動ファイル
└── ros_bridge/         # ROS_bridge (WebとROSの仲介)
```

---

## 🚀 起動方法 (一斉起動)

初期設定（後述）が完了していれば、以下の手順で一発でシステムを立ち上げられます。

### 💻 Windowsでの起動方法
LunarRoverConsoleフォルダ直下にある **`start_console.bat`** をダブルクリックしてください。

### 🐧 Ubuntu（ローバー）側の起動
Ubuntuのターミナルで、プロジェクトフォルダを開き、以下のコマンドを実行します。
```bash
./start_console.sh
```
---

## ⚙️ 初回セットアップ手順

### 1. リポジトリのクローン
```bash
git clone [リポジトリのURL]
cd LunarRoverConsole
```

### 2. 環境変数の設定 (重要)
基地局PCで、それぞれ設定ファイルを作ります。

**【基地局PC側】 `backend/.env` を作成**
接続したXBeeのCOMポート番号を指定します。※Xbee使用時（例: COM4）
```env
XBEE_PORT=COM4
XBEE_BAUD_RATE=115200
```

### 3. フロントエンドの起動
```bash
cd frontend
npm install
npm start
```

### 4. バックエンドの起動

**新しいターミナル**で、バックエンドを起動します。
```bash
cd backend
uvicorn main:app --reload --host 0.0.0.0
```

### 5. ROS Bridgeの起動

**新しいターミナル**で、ROS Bridgeを起動します。
```bash
cd ros_bridge
colcon build
# 環境設定ファイルを読み込む
source install/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

### 6. カメラ映像サーバーの起動 (ローバー側PC)

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