from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import HTMLResponse
import asyncio
import json
import roslibpy
import time
import threading
from google.oauth2 import service_account
import gspread
from twisted.internet import reactor
import os
import math # Added for coordinate conversion
import base64
import numpy as np
import cv2
from dotenv import load_dotenv

load_dotenv()

app = FastAPI()

# --- CORS 設定 ---
origins = [
    "http://localhost",
    "http://localhost:3000",
]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# --- グローバル変数 ---
main_event_loop = None # メインの asyncio イベントループを保存
latest_rover_gps = None # ローバーの最新GPSデータを保存

# --- ROS 設定 ---
ROSBRIDGE_IP = 'localhost'
ROSBRIDGE_PORT = 9090

ros_client = roslibpy.Ros(host=ROSBRIDGE_IP, port=ROSBRIDGE_PORT)

# --- Google Sheets 設定 ---
SPREADSHEET_ID = '1-lNnJv-WoQuhm-Tfg297eg-JENLAWAjZ7oc5hBWsSlw' # 例: '1_aBcDeFgHiJkLmNoPqRsTuVwXyZ-1234567890'
SHEET_NAME = 'RawData' # 例: 'RawData'

# Google Sheets クライアント (グローバルに初期化)
gs_client = None

# ROS トピックの定義 (Node.js バックエンドから移植)
CMD_VEL_TOPIC = 'cmd_vel'
CMD_VEL_MSG_TYPE = 'geometry_msgs/Twist'
GPS_FIX_TOPIC = 'gps/fix'
GPS_FIX_MSG_TYPE = 'sensor_msgs/NavSatFix'
IMU_TOPIC = '/imu/data'
IMU_MSG_TYPE = 'std_msgs/Float32MultiArray'
GOAL_FIX_TOPIC = 'goal/fix'
GOAL_FIX_MSG_TYPE = 'sensor_msgs/NavSatFix'
SPEED_TOPIC = '/rover/speed'
SPEED_MSG_TYPE = 'std_msgs/Float32'
RPM_TOPIC = '/rover/rpm'
RPM_MSG_TYPE = 'std_msgs/Float32'
ACTUAL_RAD_TOPIC = '/C620/actual_rad'
ACTUAL_RAD_MSG_TYPE = 'std_msgs/Float64MultiArray'
TARGETS_TOPIC = '/rover/targets'
TARGETS_MSG_TYPE = 'std_msgs/Float64MultiArray'
GOAL_REACHED_TOPIC = '/goal_reached'
GOAL_REACHED_MSG_TYPE = 'std_msgs/Bool'
CAMERA_IMAGE_TOPIC = '/camera/image_raw'
CAMERA_IMAGE_MSG_TYPE = 'sensor_msgs/Image'


# ROS パブリッシャーとサブスクライバー
cmd_vel_publisher = roslibpy.Topic(ros_client, CMD_VEL_TOPIC, CMD_VEL_MSG_TYPE)
goal_publisher = roslibpy.Topic(ros_client, GOAL_FIX_TOPIC, GOAL_FIX_MSG_TYPE)

# WebSocket クライアント管理
class ConnectionManager:
    def __init__(self):
        self.active_connections: list[WebSocket] = []

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)

    def disconnect(self, websocket: WebSocket):
        self.active_connections.remove(websocket)

    async def send_personal_message(self, message: str, websocket: WebSocket):
        await websocket.send_text(message)

    async def broadcast(self, message: str):
        for connection in self.active_connections:
            await connection.send_text(message)

manager = ConnectionManager()

# --- ROS データ転送レート制御 ---
THROTTLE_MS = 100 # Default to 10Hz (100ms)
active_listeners = [] # List to keep track of active topic listeners

last_sheet_update_time = 0
SHEET_UPDATE_INTERVAL = 5 # seconds

# --- Google Sheets 連携関数 ---
async def append_to_sheet(values: list):
    global gs_client, last_sheet_update_time

    current_time = time.time()
    if current_time - last_sheet_update_time < SHEET_UPDATE_INTERVAL:
        print(f"[GSheets] Skipping update due to rate limit. Next update in {SHEET_UPDATE_INTERVAL - (current_time - last_sheet_update_time):.2f}s")
        return
    last_sheet_update_time = current_time
    print("[GSheets] Attempting to append data...")

    if gs_client is None:
        try:
            creds_json_str = os.getenv("GOOGLE_CREDENTIALS_JSON")
            if not creds_json_str:
                print("[GSheets] GOOGLE_CREDENTIALS_JSON environment variable not found.")
                return
            
            creds_info = json.loads(creds_json_str)
            
            creds = service_account.Credentials.from_service_account_info(
                creds_info,
                scopes=['https://www.googleapis.com/auth/spreadsheets']
            )
            gs_client = gspread.authorize(creds)
            print("[GSheets] Google Sheets client authorized.")
        except Exception as e:
            print(f"[GSheets] Error authorizing Google Sheets client: {e}")
            print("[GSheets] Make sure GOOGLE_CREDENTIALS_JSON is set correctly and has necessary permissions.")
            return

    try:
        if SPREADSHEET_ID == 'YOUR_SPREADSHEET_ID_HERE':
            print("[GSheets] SPREADSHEET_ID is not set. Skipping Google Sheets logging.")
            return
        
        spreadsheet = gs_client.open_by_key(SPREADSHEET_ID)
        worksheet = spreadsheet.worksheet(SHEET_NAME)
        worksheet.append_row(values)
        # print('[GSheets] Appended data successfully.');
    except Exception as e:
        print(f"[GSheets] Error appending data: {e}")
        print("[GSheets] Make sure SPREADSHEET_ID is correct and the sheet is shared with the service account email.")

# --- ROS 接続とデータ転送 ---
async def connect_to_ros():
    print(f"[ROS] Attempting to connect to ROS Bridge at ws://{ROSBRIDGE_IP}:{ROSBRIDGE_PORT}")

    def _handle_ros_disconnect(ws_client):
        print("[ROS] Disconnected from ROS Bridge.")
        coro = manager.broadcast(json.dumps({"type": "ros_status", "data": {"connected": False}}))
        asyncio.run_coroutine_threadsafe(coro, main_event_loop)
        # TODO: Consider adding a reconnection strategy here.

    ros_client.on('closing', _handle_ros_disconnect)
    
    # roslibpyのTwistedリアクターをシグナルハンドラなしで別スレッドで実行
    ros_thread = threading.Thread(target=lambda: reactor.run(installSignalHandlers=False))
    ros_thread.daemon = True # メインプログラム終了時にスレッドも終了
    ros_thread.start()

    while not ros_client.is_connected:
        print(f"[ROS] Waiting for ROS Bridge connection... Retrying in 1 second...")
        await asyncio.sleep(1)
    print("[ROS] Connected to ROS Bridge.")
    coro = manager.broadcast(json.dumps({"type": "ros_status", "data": {"connected": True}}))
    asyncio.run_coroutine_threadsafe(coro, main_event_loop)

    # 初回購読
    await resubscribe_topics()


async def resubscribe_topics():
    global active_listeners, THROTTLE_MS
    print(f"[ROS] Unsubscribing from all topics to re-subscribe with new rate ({THROTTLE_MS}ms)...")
    
    # 既存のリスナーをすべて購読解除
    for listener in active_listeners:
        listener.unsubscribe()
    active_listeners = []
    print("[ROS] All topics unsubscribed.")

    # ROS トピック購読と WebSocket へのブロードキャスト
    def _gps_callback(message):
        global latest_rover_gps
        latest_rover_gps = {"latitude": message["latitude"], "longitude": message["longitude"]}
        print(f"[ROS] Received GPS: {message}")
        coro = manager.broadcast(json.dumps({"type": "gps", "data": message}))
        asyncio.run_coroutine_threadsafe(coro, main_event_loop)

    def _imu_callback(message):
        received_timestamp = time.time() # Capture the current time
        # The message from roslibpy for Float32MultiArray is a dict with a 'data' key
        imu_data = message.get('data', [])
        if len(imu_data) != 3:
            print(f"[ROS] Received IMU data with incorrect length: {len(imu_data)}")
            return

        heading = imu_data[0]
        roll = imu_data[1]
        pitch = imu_data[2]

        print(
            f'[ROS] Received IMU → Heading: {heading:.2f}, Roll: {roll:.2f}, Pitch: {pitch:.2f}'
        )

        # Structure the data for the frontend
        formatted_data = {
            "timestamp": received_timestamp,
            "heading": heading,
            "roll": roll,
            "pitch": pitch
        }
        
        coro = manager.broadcast(json.dumps({"type": "imu", "data": formatted_data}))
        asyncio.run_coroutine_threadsafe(coro, main_event_loop)

    def _pose_callback(message):
        coro = handle_pose_message(message)
        asyncio.run_coroutine_threadsafe(coro, main_event_loop)

    gps_listener = roslibpy.Topic(ros_client, GPS_FIX_TOPIC, GPS_FIX_MSG_TYPE, throttle_rate=THROTTLE_MS)
    gps_listener.subscribe(_gps_callback)
    active_listeners.append(gps_listener)
    print(f"[ROS] Subscribed to {GPS_FIX_TOPIC} with throttle_rate={THROTTLE_MS}ms")

    imu_listener = roslibpy.Topic(ros_client, IMU_TOPIC, IMU_MSG_TYPE, throttle_rate=THROTTLE_MS)
    imu_listener.subscribe(_imu_callback)
    active_listeners.append(imu_listener)
    print(f"[ROS] Subscribed to {IMU_TOPIC} with throttle_rate={THROTTLE_MS}ms")

    pose_listener = roslibpy.Topic(ros_client, '/rover/pose', 'geometry_msgs/PoseStamped', throttle_rate=THROTTLE_MS)
    pose_listener.subscribe(_pose_callback)
    active_listeners.append(pose_listener)
    print(f"[ROS] Subscribed to /rover/pose with throttle_rate={THROTTLE_MS}ms")

    def _speed_callback(message):
        print(f"[ROS] Received Speed: {message['data']}")
        coro = manager.broadcast(json.dumps({"type": "speed", "data": message['data']}))
        asyncio.run_coroutine_threadsafe(coro, main_event_loop)

    def _rpm_callback(message):
        print(f"[ROS] Received RPM: {message['data']}")
        coro = manager.broadcast(json.dumps({"type": "rpm", "data": message['data']}))
        asyncio.run_coroutine_threadsafe(coro, main_event_loop)

    speed_listener = roslibpy.Topic(ros_client, SPEED_TOPIC, SPEED_MSG_TYPE, throttle_rate=THROTTLE_MS)
    speed_listener.subscribe(_speed_callback)
    active_listeners.append(speed_listener)
    print(f"[ROS] Subscribed to {SPEED_TOPIC} with throttle_rate={THROTTLE_MS}ms")

    rpm_listener = roslibpy.Topic(ros_client, RPM_TOPIC, RPM_MSG_TYPE, throttle_rate=THROTTLE_MS)
    rpm_listener.subscribe(_rpm_callback)
    active_listeners.append(rpm_listener)
    print(f"[ROS] Subscribed to {RPM_TOPIC} with throttle_rate={THROTTLE_MS}ms")

    def _actual_rad_callback(message):
        print(f"[ROS] Received Actual Rad: {message['data']}")
        coro = manager.broadcast(json.dumps({"type": "actual_rad", "data": message['data']}))
        asyncio.run_coroutine_threadsafe(coro, main_event_loop)

    def _targets_callback(message):
        print(f"[ROS] Received Targets: {message['data']}")
        coro = manager.broadcast(json.dumps({"type": "targets", "data": message['data']}))
        asyncio.run_coroutine_threadsafe(coro, main_event_loop)

    def _goal_reached_callback(message):
        goal_reached = message['data']
        log_message = 'Goal reached!' if goal_reached else 'Goal not reached yet.'
        print(f"[ROS] {log_message}")
        coro = manager.broadcast(json.dumps({"type": "goal_status", "data": goal_reached}))
        asyncio.run_coroutine_threadsafe(coro, main_event_loop)

    def _image_callback(message):
        try:
            # Check if encoding is 'bgr8', which is common.
            # You might need to handle other encodings like 'rgb8', 'mono8', etc.
            if message['encoding'] != 'bgr8':
                print(f"[ROS] Received image with unsupported encoding: {message['encoding']}")
                return

            # Create a numpy array from the raw image data
            np_image = np.frombuffer(bytes(message['data']), dtype=np.uint8).reshape(
                message['height'], message['width'], 3 # 3 channels for bgr8
            )

            # Encode the image to JPEG format
            ret, jpeg_buffer = cv2.imencode('.jpg', np_image)
            if not ret:
                print("[ROS] Failed to encode image to JPEG.")
                return

            # Convert JPEG buffer to Base64 string
            jpeg_base64 = base64.b64encode(jpeg_buffer).decode('utf-8')
            
            # Create the data URI for easier use on the frontend
            image_data_uri = f"data:image/jpeg;base64,{jpeg_base64}"

            # Broadcast the image data
            coro = manager.broadcast(json.dumps({"type": "camera_image", "data": image_data_uri}))
            asyncio.run_coroutine_threadsafe(coro, main_event_loop)
        
        except Exception as e:
            print(f"[ROS] Error processing image message: {e}")

    actual_rad_listener = roslibpy.Topic(ros_client, ACTUAL_RAD_TOPIC, ACTUAL_RAD_MSG_TYPE, throttle_rate=THROTTLE_MS)
    actual_rad_listener.subscribe(_actual_rad_callback)
    active_listeners.append(actual_rad_listener)
    print(f"[ROS] Subscribed to {ACTUAL_RAD_TOPIC} with throttle_rate={THROTTLE_MS}ms")

    targets_listener = roslibpy.Topic(ros_client, TARGETS_TOPIC, TARGETS_MSG_TYPE, throttle_rate=THROTTLE_MS)
    targets_listener.subscribe(_targets_callback)
    active_listeners.append(targets_listener)
    print(f"[ROS] Subscribed to {TARGETS_TOPIC} with throttle_rate={THROTTLE_MS}ms")

    goal_reached_listener = roslibpy.Topic(ros_client, GOAL_REACHED_TOPIC, GOAL_REACHED_MSG_TYPE, throttle_rate=THROTTLE_MS)
    goal_reached_listener.subscribe(_goal_reached_callback)
    active_listeners.append(goal_reached_listener)
    print(f"[ROS] Subscribed to {GOAL_REACHED_TOPIC} with throttle_rate={THROTTLE_MS}ms")

    camera_image_listener = roslibpy.Topic(ros_client, CAMERA_IMAGE_TOPIC, CAMERA_IMAGE_MSG_TYPE, throttle_rate=THROTTLE_MS)
    camera_image_listener.subscribe(_image_callback)
    active_listeners.append(camera_image_listener)
    print(f"[ROS] Subscribed to {CAMERA_IMAGE_TOPIC} with throttle_rate={THROTTLE_MS}ms")



async def handle_pose_message(message):
    print(f"[ROS] Received Pose: {message}")
    # WebSocket へのブロードキャスト
    await manager.broadcast(json.dumps({"type": "pose", "data": message}))

    # Google Sheets への記録
    timestamp = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(message['header']['stamp']['sec']))
    position = message['pose']['position']
    orientation = message['pose']['orientation']

    row_data = [
        timestamp,
        position['x'], position['y'], position['z'],
        orientation['x'], orientation['y'], orientation['z'], orientation['w']
    ]
    await append_to_sheet(row_data)

@app.on_event("startup")
async def startup_event():
    global main_event_loop
    main_event_loop = asyncio.get_event_loop()
    asyncio.create_task(connect_to_ros())

@app.on_event("shutdown")
async def shutdown_event():
    if ros_client.is_connected:
        print("[ROS] Disconnecting from ROS Bridge.")
        # ros_client.terminate() # This causes AttributeError

    if reactor.running:
        print("[Twisted] Stopping reactor.")
        reactor.callFromThread(reactor.stop)

# --- FastAPI エンドポイント ---
@app.get("/")
async def read_root():
    return {"message": "Hello from FastAPI! ROS integration is running."}

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await manager.connect(websocket)
    # 新規クライアントに現在のROS接続ステータスを送信
    await manager.send_personal_message(
        json.dumps({"type": "ros_status", "data": {"connected": ros_client.is_connected}}),
        websocket
    )
    try:
        while True:
            data = await websocket.receive_text()
            message = json.loads(data)
            msg_type = message.get("type")
            msg_data = message.get("data")

            if msg_type == "rover-command":
                command = msg_data.get("command")
                await publish_twist_command(command)
            elif msg_type == "publish-goal":
                goal_data = msg_data.get("goalData")
                await publish_goal(goal_data)
            elif msg_type == "set-refresh-rate":
                interval_seconds = msg_data.get("interval_seconds", 0.1) # Default to 0.1s (10Hz)
                global THROTTLE_MS
                THROTTLE_MS = int(interval_seconds * 1000)
                print(f"[WebSocket] Setting update interval to {interval_seconds}s ({THROTTLE_MS}ms)")
                await resubscribe_topics()
            else:
                print(f"[WebSocket] Unknown message type: {msg_type}")

    except WebSocketDisconnect:
        manager.disconnect(websocket)
        print(f"[WebSocket] Client disconnected.")
    except Exception as e:
        print(f"[WebSocket] Error: {e}")
        manager.disconnect(websocket)

# --- ROS コマンド発行関数 ---
async def publish_twist_command(command: str):
    if not ros_client.is_connected:
        print("[ROS] Not connected to ROS Bridge. Cannot publish twist command.")
        return

    twist = roslibpy.Message({
        'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},
        'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}
    })

    linear_speed = 0.2
    angular_speed = 0.5

    if command == 'forward':
        twist['linear']['x'] = linear_speed
    elif command == 'backward':
        twist['linear']['x'] = -linear_speed
    elif command == 'left':
        twist['angular']['z'] = angular_speed
    elif command == 'right':
        twist['angular']['z'] = -angular_speed
    elif command == 'stop':
        pass # すべて0なので何もしない
    else:
        print(f"[ROS] Unknown command: {command}")
        return

    print(f"[ROS] Publishing to {CMD_VEL_TOPIC}: {twist}")
    cmd_vel_publisher.publish(twist)

async def publish_goal(goal_data: dict):
    if not ros_client.is_connected:
        print("[ROS] Not connected to ROS Bridge. Cannot publish goal.")
        return

    latitude = goal_data.get('latitude', 0.0)
    longitude = goal_data.get('longitude', 0.0)
    altitude = goal_data.get('altitude', 0.0)

    goal_msg = roslibpy.Message({
        'header': {
            'stamp': {'sec': int(time.time()), 'nanosec': int((time.time() - int(time.time())) * 1e9)},
            'frame_id': 'gps',
        },
        'latitude': latitude,
        'longitude': longitude,
        'altitude': altitude,
        'position_covariance': [0.0] * 9, # Can be zero-filled if not used
        'position_covariance_type': 0 # COVARIANCE_TYPE_UNKNOWN
    })
    
    print(f"[ROS] Publishing to {GOAL_FIX_TOPIC}: {goal_msg}")
    goal_publisher.publish(goal_msg)

    # Broadcast the original Lat/Lng goal back to the frontend for display
    coro = manager.broadcast(json.dumps({"type": "goal_set", "data": {"lat": latitude, "lng": longitude}}))
    asyncio.run_coroutine_threadsafe(coro, main_event_loop)