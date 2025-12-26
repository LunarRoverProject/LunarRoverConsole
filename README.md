# 月面ローバー制御コンソール

## 概要
このプロジェクトは、月面ローバーを制御するためのコンソールアプリケーションです。フロントエンド、バックエンド、およびROS (Robot Operating System) ノードで構成されています。

## 特徴
- **フロントエンド**: ローバーの状態表示、コマンド送信のためのユーザーインターフェース。
- **バックエンド**: フロントエンドとROSノード間の通信を仲介し、データ処理を行います。
- **ROSノード**: ローバーのセンサーデータ（GPS、IMUなど）の取得、モーター制御コマンドの送信など、ローバーとの直接的なインタラクションを担当します。

## 手順

### 1. 全体のセットアップ

1.  **リポジトリのクローン**
    ```bash
    git clone [リポジトリのURL]
    cd rover-console
    ```

2.  **フロントエンドのセットアップ**
    ```bash
    cd frontend
    npm install
    cd ..
    ```

3.  **バックエンドのセットアップ**
    `backend` ディレクトリで、お使いのシェルに合わせて仮想環境をアクティベートし、依存関係をインストールします。

    **Windows (PowerShell):**
    ```powershell
    cd backend
    .\venv\Scripts\Activate.ps1
    pip install -r requirements.txt
    cd ..
    ```
    *（他のOS向けのコマンドは省略）*

4.  **ROS Bridgeのビルド**
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