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
├── ros_code_ws/        # Ubuntuローバー側で動かすROS中継スクリプト群
├── start_console.bat   # Windows用の一斉起動ファイル
├── start_console.sh    # Ubuntu用の一斉起動ファイル
```

---

## 🚀 起動方法 (一斉起動)

2回目以降は、以下の手順で一発でシステムを立ち上げられます。

### 💻 Windowsでの起動方法
LunarRoverConsoleフォルダ直下にある **`start_console.bat`** をダブルクリックしてください。

### 🐧 Ubuntuでの起動方法
Ubuntuのターミナルで、プロジェクトフォルダを開き、以下のコマンドを実行します。
```bash
chmod +x start_console.sh
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
`backend/.env.example` を複製して `.env` を作成し、自分の環境に合わせた値を入力します。

```bash
cd backend
cp .env.example .env   # Windows (PowerShell) なら: Copy-Item .env.example .env
```

後は `.env` をエディタで開き、以下の値を自分の環境に合わせて変更してください。
| 変数 | 設定例 | 説明 |
|---|---|---|
| `XBEE_PORT` | `COM3` (Windows) / `/dev/ttyUSB0` (Ubuntu) | XBeeのシリアルポート |
| `XBEE_BAUD_RATE` | `115200` | XBeeのボーレート（大乃そのまま） |
| `ROSBRIDGE_IP` | ローバーのIPアドレス | Wi-Fiモード時のみ使用 |

> [!NOTE]
> 通信モードの切り替えは `backend/main.py` の先頭の `USE_XBEE` で行います。
> `True` = XBee ATモード / `False` = Wi-Fi (rosbridge) モード

### 3. フロントエンドの起動
```bash
cd frontend
npm install
npm start
```

### 4. バックエンドの起動

**新しいターミナル**で、バックエンドを起動します。
初回は依存パッケージのインストールが必要です。
```bash
cd backend
pip install -r requirements.txt
uvicorn main:app --reload --host 0.0.0.0
```

### 5. ROS Bridgeの起動

ROS Bridgeが未インストールの場合は、初回のみ以下を実行してインストールしてください。
```bash
sudo apt install ros-humble-rosbridge-suite
```

**新しいターミナル**で、ROS Bridgeを起動します。
```bash
source /opt/ros/humble/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

### 6. カメラ映像サーバーの起動 (ローバー側PC)

カメラの映像を超低遅延で配信するために、ローバーに搭載されたUbuntuマシンで以下の2つのプロセスを動かしてください。

**A. QoSブリッジの起動 (新しいターミナル)**
カメラが送信する`BEST_EFFORT`（高頻度）なデータを、ビデオサーバーが受け取れる`RELIABLE`へと変換します。
```bash
cd ros_code_ws/src
source /opt/ros/humble/setup.bash
python3 qos_bridge.py
```

**B. web_video_serverの起動 (新しいターミナル)**
ブラウザ向けに映像をストリーミング配信するサーバーを起動します。
```bash
ros2 run web_video_server web_video_server
```

### 7. XBeeブリッジの起動 (ローバー側PC) — XBeeモード時のみ

`backend/main.py` の `USE_XBEE = True` に設定している場合のみ必要です。
XBee3モジュールをローバー側のシリアルポート（`/dev/ttyUSB0`等）に接続し、以下を実行します。

```bash
cd ros_code_ws/src
source /opt/ros/humble/setup.bash
export XBEE_PORT=/dev/ttyUSB0    # 実際のポートに合わせて変更
export XBEE_BAUD_RATE=115200
python3 xbee_rover_bridge.py
```