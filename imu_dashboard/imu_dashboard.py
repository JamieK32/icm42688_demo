import asyncio
import json
import time
import threading
import webbrowser
import serial
import numpy as np
from collections import deque
from http.server import HTTPServer, SimpleHTTPRequestHandler
import websockets
from websockets.exceptions import ConnectionClosedOK, ConnectionClosedError

# ==============================
# 配置
# ==============================
SERIAL_PORT = "COM4"     # Linux: "/dev/ttyUSB0"
BAUDRATE = 115200
WS_PORT = 8765
HTTP_PORT = 8000

WINDOW_SIZE = 30
SEND_INTERVAL = 0.02     # WS 推送间隔（秒）
SERIAL_RETRY_BASE = 1.0  # 串口重连初始等待
SERIAL_RETRY_MAX = 10.0  # 串口重连最大等待

# ==============================
# 全局状态（线程安全）
# ==============================
latest_data = None
latest_lock = threading.Lock()
latest_seq = 0

serial_lines = deque(maxlen=2000)
serial_lock = threading.Lock()
serial_seq = 0

angle_window = deque(maxlen=WINDOW_SIZE)
fps = 0
counter = 0
last_time = time.time()

shutdown_evt = threading.Event()

# ==============================
# 计算函数
# ==============================
def compute_fps():
    global fps, counter, last_time
    counter += 1
    now = time.time()
    dt = now - last_time
    if dt >= 1:
        fps = counter / dt
        counter = 0
        last_time = now
    return fps

def compute_stationary(r, p, y):
    angle_window.append([r, p, y])
    if len(angle_window) < WINDOW_SIZE:
        return 0
    std = np.mean(np.std(np.array(angle_window), axis=0))
    return 1 if std < 0.05 else 0

def compute_trust():
    if len(angle_window) < WINDOW_SIZE:
        return 0.5
    std = np.mean(np.std(np.array(angle_window), axis=0))
    return round(float(1 - min(std / 5, 1)), 3)

def euler_to_q(r, p, y):
    r, p, y = np.radians([r, p, y])
    cy, sy = np.cos(y/2), np.sin(y/2)
    cp, sp = np.cos(p/2), np.sin(p/2)
    cr, sr = np.cos(r/2), np.sin(r/2)
    return [
        float(cr*cp*cy + sr*sp*sy),
        float(sr*cp*cy - cr*sp*sy),
        float(cr*sp*cy + sr*cp*sy),
        float(cr*cp*sy - sr*sp*cy)
    ]

# ==============================
# 串口：连接 + 读循环（支持断线重连）
# ==============================
def open_serial_with_retry():
    backoff = SERIAL_RETRY_BASE
    while not shutdown_evt.is_set():
        try:
            ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
            # 有些设备重连后需要清输入缓存，避免读到半包
            try:
                ser.reset_input_buffer()
            except Exception:
                pass
            print(f"[SERIAL] connected: {SERIAL_PORT} @ {BAUDRATE}")
            return ser
        except Exception as e:
            print(f"[SERIAL] connect failed: {e} | retry in {backoff:.1f}s")
            time.sleep(backoff)
            backoff = min(backoff * 1.5, SERIAL_RETRY_MAX)
    return None

def serial_thread():
    global latest_data, latest_seq, serial_seq
    ser = None
    while not shutdown_evt.is_set():
        if ser is None:
            ser = open_serial_with_retry()
            if ser is None:
                break

        try:
            line = ser.readline()  # bytes
            if not line:
                continue
            text = line.decode(errors="ignore").strip()
            if not text:
                continue

            now_ts = time.time()
            parsed_ok = True
            try:
                r, p, y = map(float, text.split(","))
            except Exception:
                parsed_ok = False

            with serial_lock:
                serial_seq += 1
                serial_lines.append((
                    serial_seq,
                    {
                        "type": "serial_output",
                        "payload": {
                            "ts": now_ts,
                            "line": text,
                            "parsed": parsed_ok
                        }
                    }
                ))

            if not parsed_ok:
                # 解析失败的数据仅作为串口终端输出，不进入姿态计算
                continue

            fps_val = compute_fps()
            stationary = compute_stationary(r, p, y)
            trust = compute_trust()
            q = euler_to_q(r, p, y)

            msg = {
                "type": "imu_update",
                "payload": {
                    "fps": fps_val,
                    "trust": trust,
                    "stationary": stationary,
                    "euler": [r, p, y],
                    "q": q
                }
            }

            with latest_lock:
                latest_data = msg
                latest_seq += 1

        except (serial.SerialException, OSError, PermissionError) as e:
            # Windows 的 ClearCommError / 设备拔插等会到这里
            print(f"[SERIAL] read error: {e} | reconnecting...")
            try:
                ser.close()
            except Exception:
                pass
            ser = None
            # 读错后让出一点时间，避免疯狂循环
            time.sleep(0.5)
        except Exception as e:
            # 兜底：不让线程死
            print(f"[SERIAL] unexpected error: {e}")
            time.sleep(0.2)

    # 退出清理
    try:
        if ser:
            ser.close()
    except Exception:
        pass
    print("[SERIAL] stopped")

# ==============================
# WebSocket：支持多客户端 + 正常断开不报错
# ==============================
connected = set()
connected_lock = asyncio.Lock()

async def ws_handler(websocket):
    peer = getattr(websocket, "remote_address", None)
    print(f"[WS] client connected: {peer}")
    last_imu_seq = 0
    last_serial_seq = 0

    async with connected_lock:
        connected.add(websocket)

    try:
        # 每个连接独立循环推送
        while True:
            imu_msg = None
            imu_seq = 0
            with latest_lock:
                imu_seq = latest_seq
                if imu_seq > last_imu_seq and latest_data is not None:
                    imu_msg = latest_data

            if imu_msg is not None:
                try:
                    await websocket.send(json.dumps(imu_msg))
                    last_imu_seq = imu_seq
                except (ConnectionClosedOK, ConnectionClosedError):
                    break

            pending_serial = []
            with serial_lock:
                if serial_lines and serial_lines[-1][0] > last_serial_seq:
                    pending_serial = [item for item in serial_lines if item[0] > last_serial_seq]

            for seq, msg in pending_serial:
                try:
                    await websocket.send(json.dumps(msg))
                    last_serial_seq = seq
                except (ConnectionClosedOK, ConnectionClosedError):
                    return
            await asyncio.sleep(SEND_INTERVAL)

    except ConnectionClosedOK:
        # 浏览器关闭/刷新：正常现象，不要当错误
        pass
    except ConnectionClosedError:
        pass
    except Exception as e:
        print(f"[WS] handler unexpected error: {e}")
    finally:
        async with connected_lock:
            connected.discard(websocket)
        print(f"[WS] client disconnected: {peer}")

async def start_ws(stop_event: asyncio.Event):
    async with websockets.serve(
        ws_handler,
        "0.0.0.0",
        WS_PORT,
        ping_interval=20,
        ping_timeout=20,
        max_queue=1,
    ):
        print(f"[WS] started on :{WS_PORT}")
        await stop_event.wait()
    print("[WS] stopped")

# ==============================
# HTTP 服务（可选：增强退出能力）
# ==============================
class NoCacheHTTPRequestHandler(SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
        self.send_header("Pragma", "no-cache")
        self.send_header("Expires", "0")
        super().end_headers()


def start_http():
    httpd = HTTPServer(("0.0.0.0", HTTP_PORT), NoCacheHTTPRequestHandler)
    print(f"[HTTP] started on :{HTTP_PORT}")
    # 没有优雅 stop 的话就让它跟着主进程结束（daemon thread）
    httpd.serve_forever()

# ==============================
# 启动与优雅退出
# ==============================
async def main():
    # 串口线程
    threading.Thread(target=serial_thread, daemon=True).start()

    # HTTP
    threading.Thread(target=start_http, daemon=True).start()

    # 打开浏览器
    time.sleep(1)
    webbrowser.open(
        f"http://localhost:{HTTP_PORT}/imu_dashboard.html"
        f"?ws=ws://localhost:{WS_PORT}&v={int(time.time())}"
    )

    # WS
    stop_event = asyncio.Event()
    try:
        await start_ws(stop_event)
    except asyncio.CancelledError:
        # asyncio 取消属于正常退出路径
        pass

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[MAIN] KeyboardInterrupt, shutting down...")
        shutdown_evt.set()
        # 让后台线程有机会看到 shutdown_evt
        time.sleep(0.2)
