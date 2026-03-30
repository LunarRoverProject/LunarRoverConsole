from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
import asyncio
import json
import time
import threading
from google.oauth2 import service_account
import gspread
import os
import math
import re
from datetime import datetime
from dotenv import load_dotenv

load_dotenv()

# --- 実行モードの設定 ---
# True なら XBee (シリアル通信) モード。
# False なら 従来の rosbridge (Wi-Fi通信) モード。
USE_XBEE = False

if USE_XBEE:
    import serial
else:
    import roslibpy
    from twisted.internet import reactor

# --- Logging Setup ---
log_queue = []
data_queue = [] # Queue for topic data: [sheet_name, row_data]
LOG_SHEET_NAME = 'ConsoleLog'

def log(message, upload=True):
    timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
    formatted_message = f"[{timestamp}] {message}"
    print(formatted_message)
    
    if not upload: return

    log_type = "General"
    content = message
    match = re.match(r'^\s*\[(.*?)\]\s?(.*)', message)
    if match:
        log_type = match.group(1)
        content = match.group(2)

    log_queue.append([timestamp, log_type, content])

# --- Google Sheets Setup ---
SPREADSHEET_ID = '1-lNnJv-WoQuhm-Tfg297eg-JENLAWAjZ7oc5hBWsSlw'
SHEET_NAME = 'RawData'
gs_client = None
last_sheet_update_time = 0
SHEET_UPDATE_INTERVAL = 5 # seconds

def get_gs_client():
    global gs_client
    if gs_client: return gs_client
    try:
        creds_json_str = os.getenv("GOOGLE_CREDENTIALS_JSON")
        if not creds_json_str:
            log("[GSheets] GOOGLE_CREDENTIALS_JSON not found.", upload=False)
            return None
        creds_info = json.loads(creds_json_str)
        creds = service_account.Credentials.from_service_account_info(
            creds_info, scopes=['https://www.googleapis.com/auth/spreadsheets']
        )
        gs_client = gspread.authorize(creds)
        log("[GSheets] Google Sheets authorized.", upload=True)
        return gs_client
    except Exception as e:
        log(f"[GSheets] Error authorizing: {e}", upload=False)
        return None

def queue_data_for_sheet(sheet_name, row_data):
    data_queue.append([sheet_name, row_data])

async def process_log_queue():
    while True:
        await asyncio.sleep(5)
        if not log_queue: continue
        batch = []
        while log_queue:
            batch.append(log_queue.pop(0))
        if not batch: continue
            
        client = get_gs_client()
        if not client: continue
        try:
            if SPREADSHEET_ID == 'YOUR_SPREADSHEET_ID_HERE': continue
            spreadsheet = client.open_by_key(SPREADSHEET_ID)
            try:
                worksheet = spreadsheet.worksheet(LOG_SHEET_NAME)
                worksheet.append_rows(batch)
            except gspread.exceptions.WorksheetNotFound:
                 pass
        except Exception:
            pass

async def process_data_queue():
    while True:
        await asyncio.sleep(5)
        if not data_queue: continue
        batch_map = {}
        while data_queue:
            item = data_queue.pop(0)
            sheet_name, row_data = item[0], item[1]
            if sheet_name not in batch_map:
                batch_map[sheet_name] = []
            batch_map[sheet_name].append(row_data)
        
        if not batch_map: continue
        client = get_gs_client()
        if not client or SPREADSHEET_ID == 'YOUR_SPREADSHEET_ID_HERE': continue

        try:
            spreadsheet = client.open_by_key(SPREADSHEET_ID)
            for sheet_name, rows in batch_map.items():
                try:
                    worksheet = spreadsheet.worksheet(sheet_name)
                except gspread.exceptions.WorksheetNotFound:
                    try:
                        worksheet = spreadsheet.add_worksheet(title=sheet_name, rows=1000, cols=10)
                        log(f"[GSheets] Created sheet: {sheet_name}")
                    except Exception:
                        continue
                worksheet.append_rows(rows)
        except Exception:
            pass

# --- FastAPI & WebSocket Setup ---
app = FastAPI()
app.add_middleware(CORSMiddleware, allow_origins=["*"], allow_credentials=True, allow_methods=["*"], allow_headers=["*"])

main_event_loop = None
latest_rover_gps = None

class ConnectionManager:
    def __init__(self):
        self.active_connections: list[WebSocket] = []
    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)
    def disconnect(self, websocket: WebSocket):
        self.active_connections.remove(websocket)
    async def broadcast(self, message: str):
        for connection in self.active_connections:
            try: await connection.send_text(message)
            except Exception: pass

manager = ConnectionManager()

def quaternion_to_euler(x, y, z, w):
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
    return roll_x, pitch_y, yaw_z 

# ==========================================
# 共通データハンドラー
# ==========================================
def broadcast_connection_status():
    is_connected = False
    if USE_XBEE:
        is_connected = xbee_connected
    else:
        is_connected = ros_client.is_connected if ros_client else False
    
    # どちらのモードでもフロントエンドには "ros_status" として送る
    coro = manager.broadcast(json.dumps({"type": "ros_status", "data": {"connected": is_connected}}))
    if main_event_loop:
        asyncio.run_coroutine_threadsafe(coro, main_event_loop)

def handle_gps_data(lat, lng, alt, cov):
    global latest_rover_gps
    latest_rover_gps = {"latitude": lat, "longitude": lng}
    ts_str = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
    queue_data_for_sheet('GPS', [ts_str, lat, lng, alt, str(cov)])
    if main_event_loop:
        coro = manager.broadcast(json.dumps({"type": "gps", "data": {"latitude": lat, "longitude": lng, "altitude": alt}}))
        asyncio.run_coroutine_threadsafe(coro, main_event_loop)

def handle_imu_data(ox, oy, oz, ow):
    received_timestamp = time.time()
    roll, pitch, yaw = quaternion_to_euler(ox, oy, oz, ow)
    yaw_deg = math.degrees(yaw)
    heading_deg = (90 - yaw_deg) % 360
    roll_deg = math.degrees(roll)
    pitch_deg = math.degrees(pitch)
    
    formatted_data = {"timestamp": received_timestamp, "heading": heading_deg, "roll": roll_deg, "pitch": pitch_deg}
    ts_str = datetime.fromtimestamp(received_timestamp).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
    queue_data_for_sheet('IMU', [ts_str, heading_deg, roll_deg, pitch_deg, ox, oy, oz, ow])

    if main_event_loop:
        coro = manager.broadcast(json.dumps({"type": "imu", "data": formatted_data}))
        asyncio.run_coroutine_threadsafe(coro, main_event_loop)

def handle_generic_topic(msg_type, data):
    if msg_type == "goal_reached":
        ts_str = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
        queue_data_for_sheet('Status', [ts_str, 'Goal Reached', str(data)])
    if main_event_loop:
        coro = manager.broadcast(json.dumps({"type": msg_type, "data": data}))
        asyncio.run_coroutine_threadsafe(coro, main_event_loop)

# ==========================================
# XBEE MODE LOGIC
# ==========================================
XBEE_PORT = os.getenv('XBEE_PORT', 'COM3')
XBEE_BAUD_RATE = int(os.getenv('XBEE_BAUD_RATE', '115200'))
xbee_serial = None
xbee_connected = False

def xbee_read_loop():
    global xbee_connected, xbee_serial
    while True:
        try:
            if xbee_serial is None or not xbee_serial.is_open:
                try:
                    xbee_serial = serial.Serial(XBEE_PORT, XBEE_BAUD_RATE, timeout=2)
                    xbee_connected = True
                    log(f"[XBee] Connected on {XBEE_PORT}")
                    broadcast_connection_status()
                except Exception as e:
                    xbee_connected = False
                    log(f"[XBee] Connect failed: {e}", upload=False)
                    time.sleep(2)
                    continue

            if xbee_serial.in_waiting > 0:
                line = xbee_serial.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    try:
                        msg = json.loads(line)
                        m_type = msg.get("type", "")
                        data = msg.get("data", {})
                        
                        if m_type == "gps":
                            handle_gps_data(data.get("latitude"), data.get("longitude"), data.get("altitude"), data.get("position_covariance"))
                        elif m_type == "imu":
                            o = data.get('orientation', {})
                            handle_imu_data(o.get('x'), o.get('y'), o.get('z'), o.get('w'))
                        elif m_type == "pose":
                            handle_generic_topic("pose", data)
                        elif m_type in ["speed", "rpm", "actual_rad", "targets", "goal_reached"]:
                            handle_generic_topic(m_type, data)
                    except json.JSONDecodeError:
                        pass
            else:
                time.sleep(0.01)
                
        except serial.SerialException as e:
            log(f"[XBee] Disconnected: {e}")
            xbee_connected = False
            if xbee_serial: xbee_serial.close()
            time.sleep(2)
        except Exception:
            time.sleep(1)

def xbee_write(msg_dict):
    global xbee_serial
    if xbee_serial and xbee_serial.is_open:
        try:
            xbee_serial.write((json.dumps(msg_dict) + "\n").encode('utf-8'))
        except Exception as e:
            log(f"[XBee] Write error: {e}")

# ==========================================
# ROS WI-FI MODE LOGIC (roslibpy)
# ==========================================
ROSBRIDGE_IP = os.getenv('ROSBRIDGE_IP', 'localhost')
ROSBRIDGE_PORT = int(os.getenv('ROSBRIDGE_PORT', '9090'))
ros_client = None
active_listeners = []
THROTTLE_MS = 100

cmd_vel_publisher = None
goal_publisher = None

async def connect_to_ros():
    global ros_client, cmd_vel_publisher, goal_publisher
    print(f"[ROS-WiFi] Attempting connection to ws://{ROSBRIDGE_IP}:{ROSBRIDGE_PORT}")

    def _on_close(*args):
        log("[ROS-WiFi] Disconnected.")
        broadcast_connection_status()

    ros_client.on('closing', _on_close)
    
    ros_thread = threading.Thread(target=lambda: reactor.run(installSignalHandlers=False), daemon=True)
    ros_thread.start()

    while not ros_client.is_connected:
        log("[ROS-WiFi] Waiting for ROS Bridge connection... Retrying in 1 second...")
        await asyncio.sleep(1)
        
    log("[ROS-WiFi] Connected!")
    broadcast_connection_status()
    
    cmd_vel_publisher = roslibpy.Topic(ros_client, 'cmd_vel', 'geometry_msgs/Twist')
    goal_publisher = roslibpy.Topic(ros_client, 'goal/fix', 'sensor_msgs/NavSatFix')

    await resubscribe_topics()

async def resubscribe_topics():
    global active_listeners
    for listener in active_listeners: listener.unsubscribe()
    active_listeners = []
    
    def _gps_cb(msg): handle_gps_data(msg.get("latitude"), msg.get("longitude"), msg.get("altitude"), msg.get("position_covariance"))
    def _imu_cb(msg): 
        o = msg.get('orientation', {})
        handle_imu_data(o.get('x',0), o.get('y',0), o.get('z',0), o.get('w',1))
    def _speed_cb(msg): handle_generic_topic("speed", msg.get("data"))
    def _rpm_cb(msg): handle_generic_topic("rpm", msg.get("data"))
    def _rad_cb(msg): handle_generic_topic("actual_rad", msg.get("data"))
    def _targ_cb(msg): handle_generic_topic("targets", msg.get("data"))
    def _goal_cb(msg): handle_generic_topic("goal_reached", msg.get("data"))
    def _pose_cb(msg): handle_generic_topic("pose", msg) # passes entire message

    listeners_def = [
        ('gps/fix', 'sensor_msgs/NavSatFix', _gps_cb),
        ('bno055/imu', 'sensor_msgs/Imu', _imu_cb),
        ('/rover/speed', 'std_msgs/Float32', _speed_cb),
        ('/rover/rpm', 'std_msgs/Float32', _rpm_cb),
        ('/C620/actual_rad', 'std_msgs/Float64MultiArray', _rad_cb),
        ('/rover/targets', 'std_msgs/Float64MultiArray', _targ_cb),
        ('/goal_reached', 'std_msgs/msg/Bool', _goal_cb),
        ('/rover/pose', 'geometry_msgs/PoseStamped', _pose_cb)
    ]
    
    for topic, msg_type, cb in listeners_def:
        listener = roslibpy.Topic(ros_client, topic, msg_type, throttle_rate=THROTTLE_MS)
        listener.subscribe(cb)
        active_listeners.append(listener)


# ==========================================
# FastAPI Lifecycle & Endpoints
# ==========================================
@app.on_event("startup")
async def startup_event():
    global main_event_loop
    main_event_loop = asyncio.get_event_loop()
    
    if USE_XBEE:
        log("[System] Mode: XBee Serial")
        threading.Thread(target=xbee_read_loop, daemon=True).start()
    else:
        log("[System] Mode: ROS Wi-Fi (roslibpy)")
        asyncio.create_task(connect_to_ros())
        
    asyncio.create_task(process_log_queue())
    asyncio.create_task(process_data_queue())

@app.on_event("shutdown")
async def shutdown_event():
    if USE_XBEE and xbee_serial and xbee_serial.is_open:
        xbee_serial.close()
    elif not USE_XBEE and ros_client and ros_client.is_connected:
        ros_client.close()
        reactor.callFromThread(reactor.stop)
        
    tasks = [t for t in asyncio.all_tasks() if t is not asyncio.current_task()]
    [t.cancel() for t in tasks]

@app.get("/")
async def read_root(): return {"message": f"Backend is running! Mode: {'XBee' if USE_XBEE else 'Wi-Fi'}"}

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await manager.connect(websocket)
    broadcast_connection_status() # send initial status
    try:
        while True:
            data = await websocket.receive_text()
            message = json.loads(data)
            msg_type = message.get("type")
            msg_data = message.get("data")

            if msg_type == "rover-command":
                await publish_twist_command(msg_data.get("command"))
            elif msg_type == "twist-command":
                await publish_analog_twist(msg_data.get("linear", 0.0), msg_data.get("angular", 0.0))
            elif msg_type == "publish-goal":
                await publish_goal(msg_data.get("goalData"))
            elif msg_type == "set-refresh-rate":
                if not USE_XBEE:
                    global THROTTLE_MS
                    THROTTLE_MS = int(msg_data.get("interval_seconds", 0.1) * 1000)
                    await resubscribe_topics()
    except Exception:
        manager.disconnect(websocket)

async def publish_twist_command(command: str):
    linear_speed, angular_speed = 0.2, 0.5
    l, a = 0.0, 0.0
    if command == 'forward': l = linear_speed
    elif command == 'backward': l = -linear_speed
    elif command == 'left': a = angular_speed
    elif command == 'right': a = -angular_speed
    elif command == 'stop': pass
    else: return

    await publish_analog_twist(l, a)

async def publish_analog_twist(linear: float, angular: float):
    if USE_XBEE:
        xbee_write({"type": "cmd_vel", "linear": linear, "angular": angular})
    else:
        if ros_client and ros_client.is_connected:
            twist = roslibpy.Message({'linear': {'x': linear, 'y': 0.0, 'z': 0.0}, 'angular': {'x': 0.0, 'y': 0.0, 'z': angular}})
            cmd_vel_publisher.publish(twist)

async def publish_goal(goal_data: dict):
    lat = goal_data.get('latitude', 0.0)
    lng = goal_data.get('longitude', 0.0)
    alt = goal_data.get('altitude', 0.0)
    
    if USE_XBEE:
        xbee_write({"type": "goal_fix", "latitude": lat, "longitude": lng, "altitude": alt})
    else:
        if ros_client and ros_client.is_connected:
            goal_msg = roslibpy.Message({
                'header': {'stamp': {'sec': int(time.time()), 'nanosec': 0}, 'frame_id': 'gps'},
                'latitude': lat, 'longitude': lng, 'altitude': alt,
                'position_covariance': [0.0]*9, 'position_covariance_type': 0
            })
            goal_publisher.publish(goal_msg)

    # Broadcast to frontend
    coro = manager.broadcast(json.dumps({"type": "goal_set", "data": {"lat": lat, "lng": lng}}))
    asyncio.run_coroutine_threadsafe(coro, main_event_loop)