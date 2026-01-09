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
import sys
import re
import numpy as np
import cv2
from datetime import datetime
from dotenv import load_dotenv


# --- Logging Setup ---
log_queue = []
data_queue = [] # Queue for topic data: [sheet_name, row_data]
LOG_SHEET_NAME = 'ConsoleLog'


def log(message, upload=True):
    timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
    formatted_message = f"[{timestamp}] {message}"
    print(formatted_message)
    
    if not upload:
        return

    # Parse type from message (e.g., "[ROS] Connected..." -> type="ROS", content="Connected...")
    log_type = "General"
    content = message
    
    # Match pattern: start of string, optional whitespace, [TYPE], space, rest of message
    match = re.match(r'^\s*\[(.*?)\]\s?(.*)', message)
    if match:
        log_type = match.group(1)
        content = match.group(2)

    # Queue for Google Sheets: Timestamp, Type, Message
    log_queue.append([timestamp, log_type, content])

def get_gs_client():
    global gs_client
    if gs_client:
        return gs_client
    
    try:
        creds_json_str = os.getenv("GOOGLE_CREDENTIALS_JSON")
        if not creds_json_str:
            log("[GSheets] GOOGLE_CREDENTIALS_JSON environment variable not found.", upload=False)
            return None
        
        creds_info = json.loads(creds_json_str)
        
        creds = service_account.Credentials.from_service_account_info(
            creds_info,
            scopes=['https://www.googleapis.com/auth/spreadsheets']
        )
        gs_client = gspread.authorize(creds)
        log("[GSheets] Google Sheets client authorized.", upload=True) # One-time success can be uploaded
        return gs_client
    except Exception as e:
        log(f"[GSheets] Error authorizing Google Sheets client: {e}", upload=False)
        return None

async def process_log_queue():
    while True:
        await asyncio.sleep(5) # Upload every 5 seconds
        if not log_queue:
            continue

        # Drain queue up to a limit or all
        batch = []
        # Simple drain
        while log_queue:
            batch.append(log_queue.pop(0))
        
        if not batch:
            continue
            
        client = get_gs_client()
        if not client:
            continue

        try:
            if SPREADSHEET_ID == 'YOUR_SPREADSHEET_ID_HERE':
                continue
            
            spreadsheet = client.open_by_key(SPREADSHEET_ID)
            # Ensure sheet exists or just append? Assuming it exists per user request.
            try:
                worksheet = spreadsheet.worksheet(LOG_SHEET_NAME)
            except gspread.exceptions.WorksheetNotFound:
                # Optionally create it? For now, print error.
                # worksheet = spreadsheet.add_worksheet(title=LOG_SHEET_NAME, rows=1000, cols=2)
                # print(f"[GSheets] Created sheet {LOG_SHEET_NAME}")
                 log(f"[GSheets] Sheet {LOG_SHEET_NAME} not found. Skipping log upload.", upload=False)
                 continue

            worksheet.append_rows(batch)
            # log(f"[GSheets] Uploaded {len(batch)} log entries.", upload=False)
        except Exception as e:
            log(f"[GSheets] Error uploading logs: {e}", upload=False)

async def process_data_queue():
    while True:
        await asyncio.sleep(5) # Upload every 5 seconds
        if not data_queue:
            continue

        # Drain queue
        batch_map = {} # sheet_name -> list of rows
        while data_queue:
            item = data_queue.pop(0)
            sheet_name = item[0]
            row_data = item[1]
            if sheet_name not in batch_map:
                batch_map[sheet_name] = []
            batch_map[sheet_name].append(row_data)
        
        if not batch_map:
            continue
            
        client = get_gs_client()
        if not client:
            continue

        if SPREADSHEET_ID == 'YOUR_SPREADSHEET_ID_HERE':
            continue

        try:
            spreadsheet = client.open_by_key(SPREADSHEET_ID)
            
            for sheet_name, rows in batch_map.items():
                try:
                    worksheet = spreadsheet.worksheet(sheet_name)
                except gspread.exceptions.WorksheetNotFound:
                    try:
                        worksheet = spreadsheet.add_worksheet(title=sheet_name, rows=1000, cols=10)
                        log(f"[GSheets] Created new sheet: {sheet_name}")
                    except Exception as create_err:
                        log(f"[GSheets] Error creating sheet {sheet_name}: {create_err}", upload=False)
                        continue

                worksheet.append_rows(rows)
                # log(f"[GSheets] Uploaded {len(rows)} rows to {sheet_name}", upload=False)
                
        except Exception as e:
            log(f"[GSheets] Error uploading data batch: {e}", upload=False)


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
IMU_TOPIC = 'bno055/imu' # Changed from /imu/data to match subscriber check? defaulting to imu/data
IMU_MSG_TYPE = 'sensor_msgs/Imu'
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
GOAL_REACHED_MSG_TYPE = 'std_msgs/msg/Bool'
CAMERA_FRONT_TOPIC = 'camera_front/image/compressed'
CAMERA_BACK_TOPIC = 'camera_back/image/compressed'
CAMERA_MSG_TYPE = 'sensor_msgs/CompressedImage'


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

def quaternion_to_euler(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z # in radians


# --- Google Sheets 連携関数 ---
async def append_to_sheet(values: list):
    global gs_client, last_sheet_update_time

    current_time = time.time()
    if current_time - last_sheet_update_time < SHEET_UPDATE_INTERVAL:
        log(f"[GSheets] Skipping update due to rate limit. Next update in {SHEET_UPDATE_INTERVAL - (current_time - last_sheet_update_time):.2f}s")
        return
    last_sheet_update_time = current_time
    log("[GSheets] Attempting to append data...")

    if gs_client is None:
        if not get_gs_client():
            return

    try:
        if SPREADSHEET_ID == 'YOUR_SPREADSHEET_ID_HERE':
            log("[GSheets] SPREADSHEET_ID is not set. Skipping Google Sheets logging.")
            return
        
        spreadsheet = gs_client.open_by_key(SPREADSHEET_ID)
        worksheet = spreadsheet.worksheet(SHEET_NAME)
        worksheet.append_row(values)
        # print('[GSheets] Appended data successfully.');
    except Exception as e:
        log(f"[GSheets] Error appending data: {e}")
        log("[GSheets] Make sure SPREADSHEET_ID is correct and the sheet is shared with the service account email.")

# --- Google Sheets Queue Wrapper ---
def queue_data_for_sheet(sheet_name, row_data):
    data_queue.append([sheet_name, row_data])

# --- ROS 接続とデータ転送 ---
async def connect_to_ros():
    print(f"[ROS] Attempting to connect to ROS Bridge at ws://{ROSBRIDGE_IP}:{ROSBRIDGE_PORT}")

    def _handle_ros_disconnect(*args):
        log("[ROS] Disconnected from ROS Bridge.")
        coro = manager.broadcast(json.dumps({"type": "ros_status", "data": {"connected": False}}))
        asyncio.run_coroutine_threadsafe(coro, main_event_loop)
        # TODO: Consider adding a reconnection strategy here.

    ros_client.on('closing', _handle_ros_disconnect)
    
    # roslibpyのTwistedリアクターをシグナルハンドラなしで別スレッドで実行
    ros_thread = threading.Thread(target=lambda: reactor.run(installSignalHandlers=False))
    ros_thread.daemon = True # メインプログラム終了時にスレッドも終了
    ros_thread.start()

    while not ros_client.is_connected:
        log(f"[ROS] Waiting for ROS Bridge connection... Retrying in 1 second...")
        await asyncio.sleep(1)
    log("[ROS] Connected to ROS Bridge.")
    coro = manager.broadcast(json.dumps({"type": "ros_status", "data": {"connected": True}}))
    asyncio.run_coroutine_threadsafe(coro, main_event_loop)

    # 初回購読
    await resubscribe_topics()


async def resubscribe_topics():
    global active_listeners, THROTTLE_MS
    log(f"[ROS] Unsubscribing from all topics to re-subscribe with new rate ({THROTTLE_MS}ms)...")
    
    # 既存のリスナーをすべて購読解除
    for listener in active_listeners:
        listener.unsubscribe()
    active_listeners = []
    log("[ROS] All topics unsubscribed.")

    # ROS トピック購読と WebSocket へのブロードキャスト
    def _gps_callback(message):
        global latest_rover_gps
        latest_rover_gps = {"latitude": message["latitude"], "longitude": message["longitude"]}
        log(f"[ROS] Received GPS: {message}")
        
        # Queue for Sheets
        ts_str = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        queue_data_for_sheet('GPS', [
            ts_str, 
            message.get('latitude'), 
            message.get('longitude'), 
            message.get('altitude'),
            str(message.get('position_covariance')) # Convert list to string
        ])

        coro = manager.broadcast(json.dumps({"type": "gps", "data": message}))
        asyncio.run_coroutine_threadsafe(coro, main_event_loop)

    def _imu_callback(message):
        received_timestamp = time.time()
        
        # message is sensor_msgs/Imu
        # 'orientation': {'x': ..., 'y': ..., 'z': ..., 'w': ...}
        orientation = message.get('orientation', {})
        ox = orientation.get('x', 0.0)
        oy = orientation.get('y', 0.0)
        oz = orientation.get('z', 0.0)
        ow = orientation.get('w', 1.0)
        
        roll, pitch, yaw = quaternion_to_euler(ox, oy, oz, ow)
        
        # Convert to degrees for display if needed, or keep radians?
        # Frontend likely expects degrees or radians. The previous code was direct passing.
        # Assuming typical IMU usage for Compass/Map: Yaw is Heading.
        # Map.js might expect degrees or radians?
        # Let's check Map.js later. For now, let's provide degrees as commonly used in UI.
        # Wait, if previous code was Float32MultiArray, it might have been pre-processed.
        # Let's assume degrees for now as it's 'Heading'.
        
        # Convert Yaw (ENU: East=0, CCW) to Heading (Map: North=0, CW)
        # ROS 0 (East) -> Map 90
        # ROS 90 (North) -> Map 0
        # Formula: (90 - Yaw_deg) % 360
        
        yaw_deg = math.degrees(yaw)
        heading_deg = (90 - yaw_deg) % 360
        
        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch)
        
        log(f'[ROS] IMU (Map Heading): {heading_deg:.2f} (Yaw: {yaw_deg:.2f}, R: {roll_deg:.2f}, P: {pitch_deg:.2f})')

        # Structure the data for the frontend
        formatted_data = {
            "timestamp": received_timestamp,
            "heading": heading_deg,
            "roll": roll_deg,
            "pitch": pitch_deg
        }
        
        # Queue for Google Sheets
        ts_str = datetime.fromtimestamp(received_timestamp).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        queue_data_for_sheet('IMU', [
            ts_str, 
            heading_deg, roll_deg, pitch_deg,
            ox, oy, oz, ow # Save raw quaternion too just in case
        ])

        coro = manager.broadcast(json.dumps({"type": "imu", "data": formatted_data}))
        asyncio.run_coroutine_threadsafe(coro, main_event_loop)

    def _pose_callback(message):
        coro = handle_pose_message(message)
        asyncio.run_coroutine_threadsafe(coro, main_event_loop)

    gps_listener = roslibpy.Topic(ros_client, GPS_FIX_TOPIC, GPS_FIX_MSG_TYPE, throttle_rate=THROTTLE_MS)
    gps_listener.subscribe(_gps_callback)
    active_listeners.append(gps_listener)
    log(f"[ROS] Subscribed to {GPS_FIX_TOPIC} with throttle_rate={THROTTLE_MS}ms")

    imu_listener = roslibpy.Topic(ros_client, IMU_TOPIC, IMU_MSG_TYPE, throttle_rate=THROTTLE_MS)
    imu_listener.subscribe(_imu_callback)
    active_listeners.append(imu_listener)
    log(f"[ROS] Subscribed to {IMU_TOPIC} with throttle_rate={THROTTLE_MS}ms")

    pose_listener = roslibpy.Topic(ros_client, '/rover/pose', 'geometry_msgs/PoseStamped', throttle_rate=THROTTLE_MS)
    pose_listener.subscribe(_pose_callback)
    active_listeners.append(pose_listener)
    log(f"[ROS] Subscribed to /rover/pose with throttle_rate={THROTTLE_MS}ms")

    def _speed_callback(message):
        log(f"[ROS] Received Speed: {message['data']}")
        coro = manager.broadcast(json.dumps({"type": "speed", "data": message['data']}))
        asyncio.run_coroutine_threadsafe(coro, main_event_loop)

    def _rpm_callback(message):
        log(f"[ROS] Received RPM: {message['data']}")
        coro = manager.broadcast(json.dumps({"type": "rpm", "data": message['data']}))
        asyncio.run_coroutine_threadsafe(coro, main_event_loop)

    speed_listener = roslibpy.Topic(ros_client, SPEED_TOPIC, SPEED_MSG_TYPE, throttle_rate=THROTTLE_MS)
    speed_listener.subscribe(_speed_callback)
    active_listeners.append(speed_listener)
    log(f"[ROS] Subscribed to {SPEED_TOPIC} with throttle_rate={THROTTLE_MS}ms")

    rpm_listener = roslibpy.Topic(ros_client, RPM_TOPIC, RPM_MSG_TYPE, throttle_rate=THROTTLE_MS)
    rpm_listener.subscribe(_rpm_callback)
    active_listeners.append(rpm_listener)
    log(f"[ROS] Subscribed to {RPM_TOPIC} with throttle_rate={THROTTLE_MS}ms")

    def _actual_rad_callback(message):
        log(f"[ROS] Received Actual Rad: {message['data']}")
        coro = manager.broadcast(json.dumps({"type": "actual_rad", "data": message['data']}))
        asyncio.run_coroutine_threadsafe(coro, main_event_loop)

    def _targets_callback(message):
        log(f"[ROS] Received Targets: {message['data']}")
        coro = manager.broadcast(json.dumps({"type": "targets", "data": message['data']}))
        asyncio.run_coroutine_threadsafe(coro, main_event_loop)

    def _goal_reached_callback(message):
        log(f"[ROS] Received raw goal_reached message: {message}") # Added debug log
        goal_reached = message['data']
        log_message = 'Goal reached!' if goal_reached else 'Goal not reached yet.'
        log(f"[ROS] {log_message}")
        
        # Queue for Sheets
        ts_str = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        queue_data_for_sheet('Status', [ts_str, 'Goal Reached', str(goal_reached)])

        coro = manager.broadcast(json.dumps({"type": "goal_reached", "data": goal_reached}))
        log(f"[ROS] Broadcasting goal_reached WebSocket message: {json.dumps({'type': 'goal_reached', 'data': goal_reached})}")
        asyncio.run_coroutine_threadsafe(coro, main_event_loop)



    def _process_compressed_image(message, camera_type):
        try:
            # message['data'] is typically a base64 encoded string of the compressed image (jpeg/png)
            # We just need to prepend the data URI header.
            # Assuming JPEG for 'compressed' transport usually.
            # format could be 'jpeg', 'png' found in message['format']
            
            # Note: roslibpy decodes the base64 field automatically in some versions?
            # Let's verify roslibpy behavior.
            # roslibpy Message uses json only. The bridge sends base64 for uint8[].
            # So message['data'] is base64 string.
            
            img_format = 'jpeg'
            if 'png' in message.get('format', '').lower():
                img_format = 'png'
            
            base64_data = message['data']
            image_data_uri = f"data:image/{img_format};base64,{base64_data}"
            
            coro = manager.broadcast(json.dumps({"type": camera_type, "data": image_data_uri}))
            asyncio.run_coroutine_threadsafe(coro, main_event_loop)
        
        except Exception as e:
            log(f"[ROS] Error processing {camera_type} image message: {e}")

    def _front_camera_callback(message):
        _process_compressed_image(message, 'camera_front')

    def _back_camera_callback(message):
        _process_compressed_image(message, 'camera_back')


    actual_rad_listener = roslibpy.Topic(ros_client, ACTUAL_RAD_TOPIC, ACTUAL_RAD_MSG_TYPE, throttle_rate=THROTTLE_MS)
    actual_rad_listener.subscribe(_actual_rad_callback)
    active_listeners.append(actual_rad_listener)
    log(f"[ROS] Subscribed to {ACTUAL_RAD_TOPIC} with throttle_rate={THROTTLE_MS}ms")

    targets_listener = roslibpy.Topic(ros_client, TARGETS_TOPIC, TARGETS_MSG_TYPE, throttle_rate=THROTTLE_MS)
    targets_listener.subscribe(_targets_callback)
    active_listeners.append(targets_listener)
    log(f"[ROS] Subscribed to {TARGETS_TOPIC} with throttle_rate={THROTTLE_MS}ms")

    goal_reached_listener = roslibpy.Topic(ros_client, GOAL_REACHED_TOPIC, GOAL_REACHED_MSG_TYPE, throttle_rate=THROTTLE_MS)
    goal_reached_listener.subscribe(_goal_reached_callback)
    active_listeners.append(goal_reached_listener)
    log(f"[ROS] Subscribed to {GOAL_REACHED_TOPIC} with throttle_rate={THROTTLE_MS}ms")

    camera_front_listener = roslibpy.Topic(ros_client, CAMERA_FRONT_TOPIC, CAMERA_MSG_TYPE, throttle_rate=THROTTLE_MS)
    camera_front_listener.subscribe(_front_camera_callback)
    active_listeners.append(camera_front_listener)
    log(f"[ROS] Subscribed to {CAMERA_FRONT_TOPIC} with throttle_rate={THROTTLE_MS}ms")

    camera_back_listener = roslibpy.Topic(ros_client, CAMERA_BACK_TOPIC, CAMERA_MSG_TYPE, throttle_rate=THROTTLE_MS)
    camera_back_listener.subscribe(_back_camera_callback)
    active_listeners.append(camera_back_listener)
    log(f"[ROS] Subscribed to {CAMERA_BACK_TOPIC} with throttle_rate={THROTTLE_MS}ms")



async def handle_pose_message(message):
    log(f"[ROS] Received Pose: {message}")
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
    queue_data_for_sheet('Pose', row_data)

@app.on_event("startup")
async def startup_event():
    global main_event_loop
    main_event_loop = asyncio.get_event_loop()
    main_event_loop = asyncio.get_event_loop()
    asyncio.create_task(connect_to_ros())
    asyncio.create_task(process_log_queue())
    asyncio.create_task(process_data_queue())

@app.on_event("shutdown")
async def shutdown_event():
    if ros_client.is_connected:
        log("[ROS] Disconnecting from ROS Bridge.")
        ros_client.close()

    if reactor.running:
        log("[Twisted] Stopping reactor.")
        reactor.callFromThread(reactor.stop)
    
    # Force exit to ensure no hanging tasks
    # Give a small delay for logs to flush if needed, but here we just want it dead.
    # But uvicorn might complain if we kill it too early. 
    # Actually, uvicorn catches signals. Typically, we just return.
    # If tasks are hanging, we should cancel them.
    # Cancel all running tasks except current one
    tasks = [t for t in asyncio.all_tasks() if t is not asyncio.current_task()]
    [task.cancel() for task in tasks]
    log(f"[Shutdown] Cancelled {len(tasks)} background tasks.")

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
            elif msg_type == "twist-command":
                linear = msg_data.get("linear", 0.0)
                angular = msg_data.get("angular", 0.0)
                await publish_analog_twist(linear, angular)
            elif msg_type == "publish-goal":
                goal_data = msg_data.get("goalData")
                await publish_goal(goal_data)
            elif msg_type == "set-refresh-rate":
                interval_seconds = msg_data.get("interval_seconds", 0.1) # Default to 0.1s (10Hz)
                global THROTTLE_MS
                THROTTLE_MS = int(interval_seconds * 1000)
                log(f"[WebSocket] Setting update interval to {interval_seconds}s ({THROTTLE_MS}ms)")
                await resubscribe_topics()
            else:
                log(f"[WebSocket] Unknown message type: {msg_type}")

    except WebSocketDisconnect:
        manager.disconnect(websocket)
        log(f"[WebSocket] Client disconnected.")
    except asyncio.CancelledError:
        manager.disconnect(websocket)
        log(f"[WebSocket] Client connection cancelled (asyncio).")
    except Exception as e:
        log(f"[WebSocket] Error: {e}")
        manager.disconnect(websocket)

# --- ROS コマンド発行関数 ---
async def publish_twist_command(command: str):
    if not ros_client.is_connected:
        log("[ROS] Not connected to ROS Bridge. Cannot publish twist command.")
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

    log(f"[ROS] Publishing to {CMD_VEL_TOPIC}: {twist}")
    cmd_vel_publisher.publish(twist)

async def publish_analog_twist(linear: float, angular: float):
    if not ros_client.is_connected:
        log("[ROS] Not connected to ROS Bridge. Cannot publish analog twist command.")
        return

    twist = roslibpy.Message({
        'linear': {'x': linear, 'y': 0.0, 'z': 0.0},
        'angular': {'x': 0.0, 'y': 0.0, 'z': angular}
    })

    # Optional: Log only occasionally to avoid flooding? 
    # For now, let's trust the throttling on the frontend side or just log it.
    if linear != 0.0 or angular != 0.0:
        log(f"[ROS] Publishing to {CMD_VEL_TOPIC}: {twist}")
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
    
    log(f"[ROS] Publishing to {GOAL_FIX_TOPIC}: {goal_msg}")
    goal_publisher.publish(goal_msg)

    # Broadcast the original Lat/Lng goal back to the frontend for display
    coro = manager.broadcast(json.dumps({"type": "goal_set", "data": {"lat": latitude, "lng": longitude}}))
    asyncio.run_coroutine_threadsafe(coro, main_event_loop)