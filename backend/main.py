from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import HTMLResponse
import asyncio
import json
import time
import threading
from google.oauth2 import service_account
import gspread
import os
import math # Added for coordinate conversion
import base64
import sys
import re
import numpy as np
import cv2
from datetime import datetime
from dotenv import load_dotenv
import serial

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
                 log(f"[GSheets] Sheet {LOG_SHEET_NAME} not found. Skipping log upload.", upload=False)
                 continue

            worksheet.append_rows(batch)
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

# --- XBee 設定 ---
XBEE_PORT = os.getenv('XBEE_PORT', 'COM3')
XBEE_BAUD_RATE = int(os.getenv('XBEE_BAUD_RATE', '115200'))
xbee_serial = None
xbee_connected = False

# --- Google Sheets 設定 ---
SPREADSHEET_ID = '1-lNnJv-WoQuhm-Tfg297eg-JENLAWAjZ7oc5hBWsSlw' # 例: '1_aBcDeFgHiJkLmNoPqRsTuVwXyZ-1234567890'
SHEET_NAME = 'RawData' # 例: 'RawData'

gs_client = None

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
            try:
                await connection.send_text(message)
            except Exception:
                pass

manager = ConnectionManager()

last_sheet_update_time = 0
SHEET_UPDATE_INTERVAL = 5 # seconds

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
    except Exception as e:
        log(f"[GSheets] Error appending data: {e}")
        log("[GSheets] Make sure SPREADSHEET_ID is correct and the sheet is shared with the service account email.")

def queue_data_for_sheet(sheet_name, row_data):
    data_queue.append([sheet_name, row_data])

# --- XBee 接続とデータ転送 ---
def xbee_read_loop():
    global xbee_connected, main_event_loop, xbee_serial
    
    while True:
        try:
            if xbee_serial is None or not xbee_serial.is_open:
                try:
                    xbee_serial = serial.Serial(XBEE_PORT, XBEE_BAUD_RATE, timeout=2)
                    xbee_connected = True
                    log(f"[XBee] Connected to {XBEE_PORT} at {XBEE_BAUD_RATE} baud.")
                    if main_event_loop:
                        coro = manager.broadcast(json.dumps({"type": "xbee_status", "data": {"connected": True}}))
                        asyncio.run_coroutine_threadsafe(coro, main_event_loop)
                except Exception as e:
                    xbee_connected = False
                    log(f"[XBee] Failed to connect on {XBEE_PORT}: {e}", upload=False)
                    time.sleep(2)
                    continue

            if xbee_serial.in_waiting > 0:
                line = xbee_serial.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    process_incoming_xbee_data(line)
            else:
                time.sleep(0.01)
                
        except serial.SerialException as e:
            log(f"[XBee] Serial disconnected or error: {e}")
            xbee_connected = False
            if xbee_serial:
                xbee_serial.close()
            time.sleep(2)
        except Exception as e:
            log(f"[XBee] Unexpected error in read loop: {e}", upload=False)
            time.sleep(1)

def process_incoming_xbee_data(line):
    global latest_rover_gps, main_event_loop
    try:
        message = json.loads(line)
        msg_type = message.get("type", "")
        data = message.get("data", {})

        if msg_type == "gps":
            latest_rover_gps = {"latitude": data.get("latitude"), "longitude": data.get("longitude")}
            ts_str = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
            queue_data_for_sheet('GPS', [
                ts_str, data.get('latitude'), data.get('longitude'), data.get('altitude'), str(data.get('position_covariance'))
            ])
            if main_event_loop:
                coro = manager.broadcast(json.dumps({"type": "gps", "data": data}))
                asyncio.run_coroutine_threadsafe(coro, main_event_loop)

        elif msg_type == "imu":
            received_timestamp = time.time()
            orientation = data.get('orientation', {})
            ox = orientation.get('x', 0.0)
            oy = orientation.get('y', 0.0)
            oz = orientation.get('z', 0.0)
            ow = orientation.get('w', 1.0)
            
            roll, pitch, yaw = quaternion_to_euler(ox, oy, oz, ow)
            yaw_deg = math.degrees(yaw)
            heading_deg = (90 - yaw_deg) % 360
            roll_deg = math.degrees(roll)
            pitch_deg = math.degrees(pitch)

            formatted_data = {
                "timestamp": received_timestamp,
                "heading": heading_deg,
                "roll": roll_deg,
                "pitch": pitch_deg
            }
            
            ts_str = datetime.fromtimestamp(received_timestamp).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
            queue_data_for_sheet('IMU', [
                ts_str, heading_deg, roll_deg, pitch_deg, ox, oy, oz, ow
            ])

            if main_event_loop:
                coro = manager.broadcast(json.dumps({"type": "imu", "data": formatted_data}))
                asyncio.run_coroutine_threadsafe(coro, main_event_loop)

        elif msg_type == "pose":
            if main_event_loop:
                coro = manager.broadcast(json.dumps({"type": "pose", "data": data}))
                asyncio.run_coroutine_threadsafe(coro, main_event_loop)
            
            # Gsheet logic for pose
            try:
                timestamp = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(data['header']['stamp']['sec']))
                position = data['pose']['position']
                orientation = data['pose']['orientation']
                row_data = [
                    timestamp,
                    position['x'], position['y'], position['z'],
                    orientation['x'], orientation['y'], orientation['z'], orientation['w']
                ]
                queue_data_for_sheet('Pose', row_data)
            except Exception:
                pass
            
        elif msg_type in ["speed", "rpm", "actual_rad", "targets", "goal_reached"]:
            if msg_type == "goal_reached":
                ts_str = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
                queue_data_for_sheet('Status', [ts_str, 'Goal Reached', str(data)])

            if main_event_loop:
                coro = manager.broadcast(json.dumps({"type": msg_type, "data": data}))
                asyncio.run_coroutine_threadsafe(coro, main_event_loop)
                
    except json.JSONDecodeError:
        pass # Ignore corrupt serial lines
    except Exception as e:
        log(f"[XBee] Error processing line {line}: {e}", upload=False)

def write_to_xbee(msg_dict):
    global xbee_serial
    if xbee_serial and xbee_serial.is_open:
        try:
            msg_str = json.dumps(msg_dict) + "\n"
            xbee_serial.write(msg_str.encode('utf-8'))
        except Exception as e:
            log(f"[XBee] Error writing to serial: {e}")

@app.on_event("startup")
async def startup_event():
    global main_event_loop
    main_event_loop = asyncio.get_event_loop()
    
    # Start background tasks
    xbee_thread = threading.Thread(target=xbee_read_loop, daemon=True)
    xbee_thread.start()
    
    asyncio.create_task(process_log_queue())
    asyncio.create_task(process_data_queue())

@app.on_event("shutdown")
async def shutdown_event():
    global xbee_serial
    if xbee_serial and xbee_serial.is_open:
        log("[XBee] Closing XBee serial port.")
        xbee_serial.close()
    
    tasks = [t for t in asyncio.all_tasks() if t is not asyncio.current_task()]
    [task.cancel() for task in tasks]
    log(f"[Shutdown] Cancelled {len(tasks)} background tasks.")


# --- FastAPI エンドポイント ---
@app.get("/")
async def read_root():
    return {"message": "Hello from FastAPI! XBee integration is running."}

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await manager.connect(websocket)
    # 新規クライアントに現在の接続ステータスを送信
    await manager.send_personal_message(
        json.dumps({"type": "xbee_status", "data": {"connected": xbee_connected}}),
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
                pass # This is now managed at the rover XBee publishing side if needed.
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

# --- XBee コマンド発行関数 ---
async def publish_twist_command(command: str):
    linear_speed = 0.2
    angular_speed = 0.5
    twist = {"linear": 0.0, "angular": 0.0}

    if command == 'forward':
        twist["linear"] = linear_speed
    elif command == 'backward':
        twist["linear"] = -linear_speed
    elif command == 'left':
        twist["angular"] = angular_speed
    elif command == 'right':
        twist["angular"] = -angular_speed
    elif command == 'stop':
        pass 
    else:
        print(f"[XBee] Unknown command: {command}")
        return

    # log(f"[XBee] Publishing twist_command: {twist}")
    write_to_xbee({"type": "cmd_vel", "linear": twist["linear"], "angular": twist["angular"]})

async def publish_analog_twist(linear: float, angular: float):
    if linear != 0.0 or angular != 0.0:
        pass # Optional log
    write_to_xbee({"type": "cmd_vel", "linear": linear, "angular": angular})

async def publish_goal(goal_data: dict):
    latitude = goal_data.get('latitude', 0.0)
    longitude = goal_data.get('longitude', 0.0)
    altitude = goal_data.get('altitude', 0.0)
    
    log(f"[XBee] Publishing goal_fix: {latitude}, {longitude}")
    write_to_xbee({
        "type": "goal_fix",
        "latitude": latitude,
        "longitude": longitude,
        "altitude": altitude
    })

    # Broadcast back to frontend
    coro = manager.broadcast(json.dumps({"type": "goal_set", "data": {"lat": latitude, "lng": longitude}}))
    asyncio.run_coroutine_threadsafe(coro, main_event_loop)