# 月面ローバー制御コンソール

## 概要
このプロジェクトは、月面ローバーを制御するためのコンソールアプリケーションです。
main.pyでROSとXBeeの通信を切り替えられます。
1.映像データはWi-Fi（`web_video_server`）経由で、GPSや操作コマンドなどのテレメトリデータは **XBee（シリアル通信）** 経由で送受信するハイブリッド構成
2.すべてWi-Fiで送受信する構成

## プロジェクト構成
```
.
├── backend/      # バックエンド (FastAPI, pyserial: XBee通信の仲介)
├── frontend/     # フロントエンド (Reactアプリケーション)
├── ros_code/     # Ubuntuローバー側で動かすROS中継スクリプト群
├── start_console.bat # Windows用の一斉起動ファイル
├── start_console.sh # Ubuntu用の一斉起動ファイル
└── ros_bridge/   # (※旧ネットワーク通信用の名残。現在は使用していません)
```

---

## 🚀 起動方法 (一斉起動)

初期設定（後述）が完了していれば、以下の手順で一発でシステムを立ち上げられます。

### 💻 Windows（コンソール）側の起動
プロジェクトフォルダの中にある **`start_console.bat`** をダブルクリックしてください。
自動的にフロント画面（React）と裏側（FastAPI）の2つの黒い画面が立ち上がり、ローバーとの接続を待機します。

### 🐧 Ubuntu（ローバー）側の起動
XBeeを挿したUbuntuのターミナルで、プロジェクトフォルダを開き、以下のコマンドを実行します。
```bash
./start_console.sh
```
これにより「映像配信」「QoS変換」「XBee通信」の3つのプログラムが同時に起動し、Windows側へデータが飛び始めます！

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

### 3. バックエンドの起動

**新しいターミナル**で、バックエンドを起動します。
```bash
cd backend
uvicorn main:app --reload --host 0.0.0.0
```

---

### 4. ROS Bridgeの起動

**新しいターミナル**で、ROS Bridgeを起動します。
```bash
cd ros_bridge
colcon build
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