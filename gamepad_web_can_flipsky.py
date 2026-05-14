#!/usr/bin/env python3
import os
import re
import json
import time
import math
import threading
import subprocess
import socket
import struct
import glob
import xml.etree.ElementTree as ET
from typing import Any, Dict, List, Optional

from flask import Flask, request, jsonify, render_template_string
from flask_socketio import SocketIO

try:
    from evdev import InputDevice, list_devices, ecodes
    EVDEV_AVAILABLE = True
    EVDEV_IMPORT_ERROR = ""
except Exception as import_error:
    InputDevice = None
    list_devices = None
    ecodes = None
    EVDEV_AVAILABLE = False
    EVDEV_IMPORT_ERROR = str(import_error)

try:
    import serial
    SERIAL_AVAILABLE = True
    SERIAL_IMPORT_ERROR = ""
except Exception as import_error:
    serial = None
    SERIAL_AVAILABLE = False
    SERIAL_IMPORT_ERROR = str(import_error)

try:
    from ob_depth import DepthCamera
    DEPTH_AVAILABLE = True
    DEPTH_IMPORT_ERROR = ""
except Exception as depth_import_error:
    DepthCamera = None
    DEPTH_AVAILABLE = False
    DEPTH_IMPORT_ERROR = str(depth_import_error)

import base64


APP_DIR = os.path.dirname(os.path.abspath(__file__))
CONFIG_FILE = os.path.join(APP_DIR, "gamepad_config.json")

DEFAULT_CONFIG = {
    "device_path": "",
    "device_name_contains": "",
    "deadzone": 0.05,
    "mappings": {},
    "can": {
        "interface": "can0",
        "bitrate": 500000,
        "left_id": 1,
        "right_id": 2,
        "max_duty": 0.25,
        "steering_gain": 0.65,
        "send_interval": 0.05,
        "throttle_axis": "ABS_Y",
        "steering_axis": "ABS_X",
        "invert_throttle": True,
        "invert_steering": False,
        "invert_left": False,
        "invert_right": True,
        "require_deadman": True,
        "deadman_button": "BTN_TR",
        "brake_button": "BTN_SOUTH",
        "brake_current": 8.0
    },
    "lidar": {
        "port": "/dev/ttyUSB0",
        "baudrate": 230400,
        "min_distance": 150,
        "max_distance": 12000,
        "emit_interval": 0.08
    },
    "gps": {
        "at_port": "/dev/ttyUSB1",
        "at_baudrate": 115200,
        "emit_interval": 1.0
    },
    "depth_camera": {
        "enabled": True,
        "min_depth_mm": 500,
        "max_depth_mm": 8000,
        "emit_fps": 10
    }
}

CODE_FALLBACK_NAMES = {
    304: "BTN_SOUTH",
    305: "BTN_EAST",
    306: "BTN_C",
    307: "BTN_NORTH",
    308: "BTN_WEST",
    309: "BTN_Z",
    310: "BTN_TL",
    311: "BTN_TR",
    312: "BTN_TL2",
    313: "BTN_TR2",
    314: "BTN_SELECT",
    315: "BTN_START",
    316: "BTN_MODE",
    317: "BTN_THUMBL",
    318: "BTN_THUMBR",
    0: "ABS_X",
    1: "ABS_Y",
    2: "ABS_Z",
    3: "ABS_RX",
    4: "ABS_RY",
    5: "ABS_RZ",
    16: "ABS_HAT0X",
    17: "ABS_HAT0Y"
}

FRIENDLY_NAMES = {
    "BTN_SOUTH": "B",
    "BTN_EAST": "A",
    "BTN_NORTH": "X",
    "BTN_WEST": "Y",
    "BTN_C": "C",
    "BTN_Z": "Z",
    "BTN_TL": "L",
    "BTN_TR": "R",
    "BTN_TL2": "ZL",
    "BTN_TR2": "ZR",
    "BTN_SELECT": "-",
    "BTN_START": "+",
    "BTN_MODE": "HOME",
    "BTN_THUMBL": "L3",
    "BTN_THUMBR": "R3",
    "ABS_X": "Analógico esquerdo X",
    "ABS_Y": "Analógico esquerdo Y",
    "ABS_RX": "Analógico direito X",
    "ABS_RY": "Analógico direito Y",
    "ABS_Z": "Gatilho esquerdo",
    "ABS_RZ": "Gatilho direito",
    "ABS_HAT0X": "D-Pad horizontal",
    "ABS_HAT0Y": "D-Pad vertical"
}


MAC_RE = re.compile(r"([0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2}")

app = Flask(__name__)
app.config["SECRET_KEY"] = "gamepad-web-secret"

socketio = SocketIO(
    app,
    cors_allowed_origins="*",
    async_mode="threading"
)

state_lock = threading.Lock()
reader_restart_event = threading.Event()

current_state = {
    "connected": False,
    "device_path": None,
    "device_name": None,
    "last_event": None,
    "buttons": {},
    "axes": {},
    "error": None,
    "evdev_available": EVDEV_AVAILABLE,
    "evdev_import_error": EVDEV_IMPORT_ERROR
}

CAN_EFF_FLAG = 0x80000000
CAN_PACKET_SET_DUTY = 0
CAN_PACKET_SET_CURRENT = 1
CAN_PACKET_SET_CURRENT_BRAKE = 2
CAN_PACKET_SET_RPM = 3

robot_lock = threading.Lock()

robot_state = {
    "armed": False,
    "can_ready": False,
    "interface": "can0",
    "last_left": 0.0,
    "last_right": 0.0,
    "last_send_time": 0.0,
    "last_error": None,
    "last_tx": None,
    "deadman_ok": False,
    "brake_active": False
}

lidar_lock = threading.Lock()
lidar_restart_event = threading.Event()

lidar_state: Dict[str, Any] = {
    "connected": False,
    "port": "/dev/ttyUSB0",
    "scanning": False,
    "rotation_speed": 0.0,
    "points": [],
    "last_count": 0,
    "timestamp": 0.0,
    "error": None,
    "serial_available": SERIAL_AVAILABLE,
    "serial_import_error": SERIAL_IMPORT_ERROR
}

depth_lock = threading.Lock()
depth_restart_event = threading.Event()

depth_state: Dict[str, Any] = {
    "connected": False,
    "fps": 0.0,
    "min_depth_mm": 500,
    "max_depth_mm": 8000,
    "width": 640,
    "height": 480,
    "error": None,
    "depth_available": DEPTH_AVAILABLE,
    "depth_import_error": DEPTH_IMPORT_ERROR
}


gps_lock = threading.Lock()
gps_restart_event = threading.Event()

gps_state: Dict[str, Any] = {
    "connected": False,
    "gps_powered": False,
    "fix": 0,
    "latitude": None,
    "longitude": None,
    "altitude": None,
    "speed_kmh": None,
    "heading": None,
    "hdop": None,
    "satellites_used": 0,
    "satellites_in_view": 0,
    "satellites": [],
    "utc_time": None,
    "error": None,
    "serial_available": SERIAL_AVAILABLE,
    "serial_import_error": SERIAL_IMPORT_ERROR,
    "at_port": "/dev/ttyUSB1",
    "emit_interval": 1.0,
    "last_emit": 0.0
}

trajectory_state: Dict[str, Any] = {
    "recording": False,
    "paused": False,
    "point_count": 0,
    "start_time": None,
    "points": []
}

gps_log_lines: List[Dict[str, Any]] = []
MAX_GPS_LOG = 50

trajectory_lock = threading.Lock()

uploaded_trajectory: Dict[str, Any] = {
    "filename": "",
    "points": [],
    "point_count": 0,
    "loaded": False
}

follow_lock = threading.Lock()
follow_state: Dict[str, Any] = {
    "active": False,
    "wp_index": 0,
    "wp_total": 0,
    "distance": 0.0,
    "bearing": 0.0,
    "throttle": 0.0,
    "steering": 0.0,
    "error": None
}

follow_config: Dict[str, Any] = {
    "waypoint_threshold": 2.0,
    "max_auto_speed": 0.15,
    "steering_kp": 0.5,
    "avoidance_enabled": True,
    "avoidance_weight": 0.5,
    "safe_distance_mm": 500,
    "critical_distance_mm": 300
}

lidar_obstacle_lock = threading.Lock()
lidar_obstacle_data: Dict[str, Any] = {
    "timestamp": 0.0,
    "emergency_stop": False,
    "avoidance_steering": 0.0,
    "min_front_dist": None,
    "active": True
}


def _gps_log(level: str, message: str) -> None:
    entry = {
        "time": time.strftime("%H:%M:%S"),
        "level": level,
        "message": message
    }
    gps_log_lines.append(entry)
    if len(gps_log_lines) > MAX_GPS_LOG:
        gps_log_lines.pop(0)
    socketio.emit("gps_log", entry)


def parse_cgnssinfo(line: str) -> Optional[Dict[str, Any]]:
    prefix = "+CGNSSINFO:"
    idx = line.find(prefix)
    if idx == -1:
        return None
    data = line[idx + len(prefix):].strip()
    if not data:
        return None
    parts = data.split(",")
    if len(parts) < 13:
        return None
    try:
        fix = int(parts[0]) if parts[0].strip() else 0
    except ValueError:
        fix = 0
    try:
        gps_sats = int(parts[1]) if parts[1].strip() else 0
    except ValueError:
        gps_sats = 0
    try:
        glo_sats = int(parts[2]) if parts[2].strip() else 0
    except ValueError:
        glo_sats = 0
    try:
        bd_sats = int(parts[3]) if parts[3].strip() else 0
    except ValueError:
        bd_sats = 0
    try:
        ga_sats = int(parts[4]) if parts[4].strip() else 0
    except ValueError:
        ga_sats = 0
    lat_val: Optional[float] = None
    lng_val: Optional[float] = None
    lat_raw = parts[5].strip()
    ns = parts[6].strip() if len(parts) > 6 else ""
    lng_raw = parts[7].strip() if len(parts) > 7 else ""
    ew = parts[8].strip() if len(parts) > 8 else ""
    if lat_raw:
        try:
            lat_val = float(lat_raw)
            if ns == "S":
                lat_val = -lat_val
        except ValueError:
            lat_val = None
    if lng_raw:
        try:
            lng_val = float(lng_raw)
            if ew == "W":
                lng_val = -lng_val
        except ValueError:
            lng_val = None
    utc_time = parts[10].strip() if len(parts) > 10 and parts[10].strip() else None
    alt = None
    if len(parts) > 11 and parts[11].strip():
        try:
            alt = float(parts[11])
        except ValueError:
            alt = None
    speed_kmh = None
    if len(parts) > 12 and parts[12].strip():
        try:
            speed_kmh = float(parts[12])
        except ValueError:
            speed_kmh = None
    heading = None
    if len(parts) > 13 and parts[13].strip():
        try:
            heading = float(parts[13])
        except ValueError:
            heading = None
    hdop = None
    if len(parts) > 15 and parts[15].strip():
        try:
            hdop = float(parts[15])
        except ValueError:
            hdop = None
    sats_used = 0
    if len(parts) > 18 and parts[18].strip():
        try:
            sats_used = int(parts[18])
        except ValueError:
            sats_used = 0
    sats_view = gps_sats + glo_sats + bd_sats + ga_sats
    satellites = []
    if gps_sats > 0:
        satellites.append({"prn": 0, "system": "GPS", "count": gps_sats})
    if glo_sats > 0:
        satellites.append({"prn": 0, "system": "GLONASS", "count": glo_sats})
    if bd_sats > 0:
        satellites.append({"prn": 0, "system": "BeiDou", "count": bd_sats})
    if ga_sats > 0:
        satellites.append({"prn": 0, "system": "Galileo", "count": ga_sats})
    satellites.sort(key=lambda s: s["count"], reverse=True)
    return {
        "fix": fix,
        "latitude": lat_val,
        "longitude": lng_val,
        "altitude": alt,
        "speed_kmh": speed_kmh,
        "heading": heading,
        "hdop": hdop,
        "satellites_used": sats_used or sats_view,
        "satellites_in_view": sats_view,
        "satellites": satellites,
        "utc_time": utc_time
    }



HTML_PAGE = r"""
<!doctype html>
<html lang="pt-BR">
<head>
    <meta charset="utf-8">
    <title>Gamepad HID Bluetooth - Raspberry Pi</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">

    <style>
        :root {
            --bg: #0f172a;
            --panel: #111827;
            --border: #334155;
            --text: #e5e7eb;
            --muted: #94a3b8;
            --blue: #2563eb;
            --green: #22c55e;
            --red: #ef4444;
            --yellow: #eab308;
        }

        * {
            box-sizing: border-box;
        }

        body {
            margin: 0;
            font-family: Arial, Helvetica, sans-serif;
            background: var(--bg);
            color: var(--text);
        }

        header {
            padding: 16px;
            background: #020617;
            border-bottom: 1px solid var(--border);
            position: sticky;
            top: 0;
            z-index: 10;
        }

        header h1 {
            margin: 0;
            font-size: 22px;
        }

        header small {
            color: var(--muted);
        }

        main {
            display: grid;
            grid-template-columns: 430px 1fr;
            gap: 16px;
            padding: 16px;
        }

        .card {
            background: var(--panel);
            border: 1px solid var(--border);
            border-radius: 14px;
            padding: 16px;
            margin-bottom: 16px;
            box-shadow: 0 10px 24px rgba(0, 0, 0, 0.18);
        }

        h2, h3 {
            margin-top: 0;
        }

        label {
            display: block;
            margin-top: 12px;
            margin-bottom: 5px;
            color: var(--muted);
            font-size: 14px;
        }

        input, select, button {
            width: 100%;
            padding: 10px;
            border-radius: 9px;
            border: 1px solid #475569;
            background: #020617;
            color: var(--text);
            outline: none;
            font-size: 14px;
        }

        input:focus, select:focus {
            border-color: var(--blue);
        }

        button {
            margin-top: 10px;
            background: var(--blue);
            border: none;
            cursor: pointer;
            font-weight: bold;
        }

        button:hover {
            filter: brightness(1.15);
        }

        .button-secondary {
            background: #475569;
        }

        .button-danger {
            background: #b91c1c;
        }

        .button-green {
            background: #15803d;
        }

        .button-yellow {
            background: #a16207;
        }

        .status-ok {
            color: var(--green);
            font-weight: bold;
        }

        .status-off {
            color: var(--red);
            font-weight: bold;
        }

        .status-warn {
            color: var(--yellow);
            font-weight: bold;
        }

        .muted {
            color: var(--muted);
        }

        .row {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 10px;
        }

        .device-item {
            padding: 10px;
            border: 1px solid var(--border);
            border-radius: 10px;
            margin-top: 8px;
            background: #020617;
        }

        .device-item strong {
            display: block;
            margin-bottom: 5px;
        }

        .device-item small {
            display: block;
            color: var(--muted);
            word-break: break-all;
            margin-bottom: 3px;
        }

        table {
            width: 100%;
            border-collapse: collapse;
        }

        td, th {
            padding: 9px;
            border-bottom: 1px solid var(--border);
            text-align: left;
            font-size: 14px;
        }

        th {
            color: var(--muted);
            font-weight: normal;
        }

        pre {
            margin: 0;
            white-space: pre-wrap;
            word-break: break-word;
            background: #020617;
            border: 1px solid var(--border);
            border-radius: 10px;
            padding: 12px;
            color: #cbd5e1;
            max-height: 420px;
            overflow-y: auto;
            font-size: 12px;
        }

        .log {
            height: 300px;
            overflow-y: auto;
            background: #020617;
            border: 1px solid var(--border);
            border-radius: 10px;
            padding: 10px;
            font-family: monospace;
            font-size: 13px;
        }

        .log-line {
            margin-bottom: 4px;
        }

        .axis-bar {
            height: 18px;
            width: 100%;
            background: #020617;
            border: 1px solid #475569;
            border-radius: 20px;
            overflow: hidden;
            position: relative;
            min-width: 160px;
        }

        .axis-zero {
            position: absolute;
            left: 50%;
            top: 0;
            width: 1px;
            height: 100%;
            background: #64748b;
        }

        .axis-fill-positive {
            position: absolute;
            left: 50%;
            top: 0;
            height: 100%;
            background: var(--green);
        }

        .axis-fill-negative {
            position: absolute;
            right: 50%;
            top: 0;
            height: 100%;
            background: var(--yellow);
        }

        .pill {
            display: inline-block;
            padding: 4px 8px;
            border-radius: 20px;
            background: #020617;
            border: 1px solid var(--border);
            color: var(--muted);
            font-size: 12px;
        }

        .grid-3 {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            gap: 12px;
        }

        .metric {
            background: #020617;
            border: 1px solid var(--border);
            border-radius: 10px;
            padding: 12px;
        }

        .metric strong {
            display: block;
            font-size: 22px;
        }

        .metric span {
            color: var(--muted);
            font-size: 13px;
        }

        .mini-help {
            font-size: 13px;
            color: var(--muted);
            line-height: 1.45;
        }

        .gamepad-panel {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 10px;
            align-items: stretch;
        }

        .gamepad-section {
            background: radial-gradient(circle at top, #1e293b 0%, #020617 72%);
            border: 1px solid #334155;
            border-radius: 14px;
            padding: 10px;
            min-height: 130px;
        }

        .gamepad-section h3 {
            margin: 0 0 6px 0;
            color: #cbd5e1;
            font-size: 12px;
            font-weight: normal;
        }

        .face-buttons {
            width: 176px;
            height: 176px;
            position: relative;
            margin: 0 auto;
        }

        .gp-button {
            position: absolute;
            width: 52px;
            height: 52px;
            border-radius: 999px;
            border: 2px solid #475569;
            background: linear-gradient(145deg, #111827, #020617);
            color: #e5e7eb;
            display: flex;
            align-items: center;
            justify-content: center;
            font-size: 18px;
            font-weight: bold;
            box-shadow: inset 0 0 18px rgba(255,255,255,0.03), 0 10px 20px rgba(0,0,0,0.28);
            transition: transform 0.08s ease, box-shadow 0.08s ease, border-color 0.08s ease, background 0.08s ease;
        }

        .gp-button small {
            display: block;
            font-size: 8px;
            color: #94a3b8;
            font-weight: normal;
            margin-top: 1px;
        }

        .gp-button .btn-label {
            display: flex;
            flex-direction: column;
            align-items: center;
            line-height: 1;
        }

        .gp-button.active {
            transform: scale(0.92);
            border-color: #22c55e;
            background: radial-gradient(circle, #22c55e 0%, #15803d 45%, #052e16 100%);
            box-shadow: 0 0 22px rgba(34,197,94,0.72), inset 0 0 14px rgba(255,255,255,0.18);
        }

        .gp-button-a { right: 3px; top: 62px; }
        .gp-button-b { right: 62px; bottom: 3px; }
        .gp-button-x { right: 62px; top: 3px; }
        .gp-button-y { left: 3px; top: 62px; }

        .center-buttons {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            gap: 6px;
            margin-top: 6px;
        }

        .center-button {
            border: 1px solid #475569;
            border-radius: 10px;
            min-height: 36px;
            background: #020617;
            color: #e5e7eb;
            display: flex;
            align-items: center;
            justify-content: center;
            flex-direction: column;
            transition: 0.08s ease;
        }

        .center-button span {
            font-size: 14px;
            font-weight: bold;
        }

        .center-button small {
            font-size: 8px;
            color: #94a3b8;
        }

        .center-button.active {
            border-color: #38bdf8;
            background: #075985;
            box-shadow: 0 0 18px rgba(56,189,248,0.55);
            transform: translateY(1px);
        }

        .shoulder-grid {
            display: grid;
            grid-template-columns: repeat(4, 1fr);
            gap: 6px;
            margin-top: 6px;
        }

        .shoulder-button {
            border: 1px solid #475569;
            border-radius: 10px;
            min-height: 40px;
            background: #020617;
            display: flex;
            align-items: center;
            justify-content: center;
            flex-direction: column;
            transition: 0.08s ease;
        }

        .shoulder-button span {
            font-size: 13px;
            font-weight: bold;
        }

        .shoulder-button small {
            color: #94a3b8;
            font-size: 8px;
        }

        .shoulder-button.active {
            border-color: #facc15;
            background: #713f12;
            box-shadow: 0 0 18px rgba(250,204,21,0.48);
            transform: translateY(1px);
        }

        .stick-grid {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 10px;
        }

        .stick-widget {
            display: flex;
            flex-direction: column;
            align-items: center;
            gap: 5px;
        }

        .stick-area {
            width: 130px;
            height: 130px;
            border-radius: 999px;
            position: relative;
            background:
                linear-gradient(90deg, transparent 49%, rgba(148,163,184,0.28) 50%, transparent 51%),
                linear-gradient(0deg, transparent 49%, rgba(148,163,184,0.28) 50%, transparent 51%),
                radial-gradient(circle, #0f172a 0%, #020617 68%);
            border: 2px solid #475569;
            box-shadow: inset 0 0 28px rgba(0,0,0,0.55), 0 10px 22px rgba(0,0,0,0.25);
        }

        .stick-dot {
            position: absolute;
            width: 32px;
            height: 32px;
            border-radius: 999px;
            left: 50%;
            top: 50%;
            transform: translate(-50%, -50%);
            background: radial-gradient(circle at top left, #f8fafc 0%, #38bdf8 30%, #075985 100%);
            border: 2px solid #bae6fd;
            box-shadow: 0 0 18px rgba(56,189,248,0.7);
            transition: left 0.06s linear, top 0.06s linear;
        }

        .stick-readout {
            font-size: 10px;
            color: #94a3b8;
            text-align: center;
        }

        .dpad-widget {
            width: 150px;
            height: 150px;
            position: relative;
            margin: 0 auto;
        }

        .dpad-key {
            position: absolute;
            width: 48px;
            height: 48px;
            border-radius: 12px;
            background: #020617;
            border: 1px solid #475569;
            display: flex;
            align-items: center;
            justify-content: center;
            color: #e5e7eb;
            font-size: 18px;
            transition: 0.08s ease;
        }

        .dpad-key.active {
            background: #581c87;
            border-color: #c084fc;
            box-shadow: 0 0 18px rgba(192,132,252,0.55);
            transform: scale(0.95);
        }

        .dpad-up { left: 51px; top: 0; }
        .dpad-down { left: 51px; bottom: 0; }
        .dpad-left { left: 0; top: 51px; }
        .dpad-right { right: 0; top: 51px; }
        .dpad-center {
            left: 51px;
            top: 51px;
            background: #111827;
            color: #64748b;
        }

        .trigger-bars {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 6px;
            margin-top: 6px;
        }

        .trigger-card {
            background: #020617;
            border: 1px solid #334155;
            border-radius: 10px;
            padding: 8px;
        }

        .trigger-card strong {
            display: block;
            margin-bottom: 4px;
            font-size: 12px;
        }

        .vertical-meter {
            height: 70px;
            border: 1px solid #475569;
            border-radius: 10px;
            background: #0f172a;
            position: relative;
            overflow: hidden;
        }

        .vertical-fill {
            position: absolute;
            left: 0;
            right: 0;
            bottom: 0;
            height: 0%;
            background: linear-gradient(0deg, #22c55e, #eab308);
            box-shadow: 0 0 16px rgba(34,197,94,0.45);
            transition: height 0.06s linear;
        }

        .compact-table {
            margin-top: 16px;
            overflow-x: auto;
        }

        .compact-table table {
            min-width: 720px;
        }

        .hidden-technical {
            display: none;
        }

        .technical-toggle {
            margin-top: 14px;
        }

        .can-grid {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 10px;
        }

        .can-status {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            gap: 10px;
            margin-top: 12px;
        }

        .can-status-box {
            background: #020617;
            border: 1px solid #334155;
            border-radius: 12px;
            padding: 10px;
        }

        .can-status-box strong {
            display: block;
            font-size: 18px;
        }

        .can-status-box span {
            color: #94a3b8;
            font-size: 12px;
        }

        .can-warning {
            border: 1px solid #7f1d1d;
            background: rgba(127, 29, 29, 0.22);
            color: #fecaca;
            padding: 10px;
            border-radius: 10px;
            font-size: 13px;
            line-height: 1.45;
            margin-bottom: 10px;
        }

        @media (max-width: 980px) {
            main {
                grid-template-columns: 1fr;
            }

            .row, .grid-3, .gamepad-panel, .stick-grid, .trigger-bars {
                grid-template-columns: 1fr;
            }
        }

        .lidar-container {
            text-align: center;
        }

        .lidar-canvas-wrapper {
            position: relative;
            display: inline-block;
        }

        #lidarCanvas {
            background: #020617;
            border: 1px solid #475569;
            border-radius: 14px;
            display: block;
        }

        .lidar-metrics-row {
            display: grid;
            grid-template-columns: repeat(5, 1fr);
            gap: 8px;
            margin-top: 8px;
        }

        .lidar-metric {
            background: #020617;
            border: 1px solid #334155;
            border-radius: 10px;
            padding: 8px;
        }

        .lidar-metric strong {
            display: block;
            font-size: 16px;
        }

        .lidar-metric span {
            color: #94a3b8;
            font-size: 11px;
        }

        .lidar-legend {
            display: flex;
            justify-content: center;
            gap: 16px;
            margin-top: 8px;
            font-size: 12px;
            color: #94a3b8;
        }

        .lidar-legend-item {
            display: flex;
            align-items: center;
            gap: 5px;
        }

        .lidar-legend-dot {
            width: 10px;
            height: 10px;
            border-radius: 999px;
            display: inline-block;
        }

        .gps-metrics-row {
            display: grid;
            grid-template-columns: repeat(4, 1fr);
            gap: 8px;
            margin-top: 8px;
        }

        .gps-metric {
            background: #020617;
            border: 1px solid #334155;
            border-radius: 10px;
            padding: 8px;
        }

        .gps-metric strong {
            display: block;
            font-size: 16px;
        }

        .gps-metric span {
            color: #94a3b8;
            font-size: 11px;
        }

        .gps-metric-row-2 {
            display: grid;
            grid-template-columns: repeat(2, 1fr);
            gap: 8px;
            margin-top: 8px;
        }

        .gps-sats-canvas-wrapper {
            position: relative;
            display: inline-block;
            margin-top: 8px;
        }

        #gpsSatsCanvas {
            background: #020617;
            border: 1px solid #475569;
            border-radius: 14px;
            display: block;
        }

        .gps-sats-legend {
            display: flex;
            justify-content: center;
            gap: 16px;
            margin-top: 4px;
            font-size: 11px;
            color: #94a3b8;
        }

        .gps-sats-legend-item {
            display: flex;
            align-items: center;
            gap: 4px;
        }

        .gps-sats-legend-dot {
            width: 9px;
            height: 9px;
            border-radius: 999px;
            display: inline-block;
        }

        #gpsMap {
            height: 350px;
            border-radius: 10px;
            border: 1px solid #475569;
            margin-top: 8px;
            z-index: 1;
        }

        .gps-fix-badge {
            display: inline-block;
            padding: 2px 8px;
            border-radius: 10px;
            font-size: 11px;
            font-weight: bold;
        }

        .gps-fix-none { background: #b91c1c; color: #fff; }
        .gps-fix-2d { background: #a16207; color: #fff; }
        .gps-fix-3d { background: #15803d; color: #fff; }

        .gps-coords {
            font-family: monospace;
            font-size: 14px;
            margin: 6px 0;
            word-break: break-all;
        }

        .gps-controls-row {
            display: grid;
            grid-template-columns: repeat(4, 1fr);
            gap: 6px;
            margin-top: 10px;
        }

        .gps-controls-row button {
            padding: 8px 4px;
            font-size: 12px;
            margin-top: 0;
        }

        .gps-config-row {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 8px;
            margin-top: 10px;
        }

        .gps-config-row input {
            margin-top: 4px;
            padding: 6px;
            font-size: 13px;
        }

        .gps-log {
            height: 150px;
            overflow-y: auto;
            background: #020617;
            border: 1px solid var(--border);
            border-radius: 10px;
            padding: 8px;
            font-family: monospace;
            font-size: 12px;
            margin-top: 8px;
        }

        .gps-log-line {
            margin-bottom: 3px;
        }

        .gps-log-info { color: #94a3b8; }
        .gps-log-warn { color: #eab308; }
        .gps-log-error { color: #ef4444; }
    </style>
    <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" />
</head>

<body>
<header>
    <h1>Gamepad HID Bluetooth - Raspberry Pi</h1>
    <small>Bluetoothctl + HID /dev/input/eventX + Python evdev + Flask Socket.IO</small>
</header>

<main>
    <section>
        <div class="card">
            <h2>Status</h2>

            <p>Conexão HID: <span id="connection" class="status-off">desconectado</span></p>
            <p>Dispositivo: <span id="deviceName">-</span></p>
            <p>Path: <span id="devicePath">-</span></p>
            <p>Erro: <span id="errorText" class="status-warn">-</span></p>

            <div class="grid-3">
                <div class="metric">
                    <strong id="buttonCount">0</strong>
                    <span>Botões</span>
                </div>
                <div class="metric">
                    <strong id="axisCount">0</strong>
                    <span>Eixos</span>
                </div>
                <div class="metric">
                    <strong id="eventCount">0</strong>
                    <span>Eventos</span>
                </div>
            </div>
        </div>

        <div class="card">
            <h2>Bluetooth</h2>

            <p class="mini-help">
                Para Pro Controller: clique em <b>Power ON</b>, depois <b>Preparar Agent</b>,
                segure o botão pequeno de sync no controle, clique em <b>Scan</b> e depois
                <b>Parear + Trust + Conectar</b>.
            </p>

            <div class="row">
                <button onclick="btPowerOn()">Power ON</button>
                <button class="button-danger" onclick="btPowerOff()">Power OFF</button>
            </div>

            <button class="button-secondary" onclick="btPrepare()">Preparar Agent</button>

            <div class="row">
                <button onclick="btScan()">Scan 8 segundos</button>
                <button class="button-secondary" onclick="btDevices()">Listar salvos</button>
            </div>

            <div class="row">
                <button class="button-green" onclick="btScanOn()">Scan ON</button>
                <button class="button-yellow" onclick="btScanOff()">Scan OFF</button>
            </div>

            <label>MAC manual</label>
            <input id="btMac" placeholder="Ex: 98:B6:E9:72:58:7E">

            <button onclick="btPairConnectManual()">Parear + Trust + Conectar</button>

            <div class="row">
                <button class="button-secondary" onclick="btConnectManual()">Conectar</button>
                <button class="button-secondary" onclick="btDisconnectManual()">Desconectar</button>
            </div>

            <button class="button-danger" onclick="btRemoveManual()">Remover pareamento</button>

            <h3>Dispositivos Bluetooth</h3>
            <div id="btDevicesList">
                <p class="muted">Clique em Scan ou Listar salvos.</p>
            </div>

            <h3>Log Bluetooth</h3>
            <pre id="btLog">-</pre>
        </div>

        <div class="card">
            <h2>CAN / Robô Flipsky 75100</h2>

            <div class="can-warning">
                Segurança: mantenha o robô suspenso no primeiro teste. O sistema só envia potência quando está <b>ARMADO</b>
                e, por padrão, exige segurar <b>R / BTN_TR</b> como botão homem-morto. O botão de emergência envia duty 0 para os dois motores.
            </div>

            <div class="can-grid">
                <button onclick="canScan()">Escanear CANable</button>
                <button class="button-secondary" onclick="canSetup()">Ativar CAN</button>
            </div>

            <label>Interface SocketCAN</label>
            <select id="canInterface">
                <option value="can0">can0</option>
            </select>

            <div class="row">
                <div>
                    <label>Bitrate</label>
                    <input id="canBitrate" type="number" value="500000">
                </div>
                <div>
                    <label>Duty máximo</label>
                    <input id="canMaxDuty" type="number" min="0.01" max="0.95" step="0.01" value="0.25">
                </div>
            </div>

            <div class="row">
                <div>
                    <label>ID VESC esquerdo</label>
                    <input id="canLeftId" type="number" min="0" max="255" value="1">
                </div>
                <div>
                    <label>ID VESC direito</label>
                    <input id="canRightId" type="number" min="0" max="255" value="2">
                </div>
            </div>

            <div class="row">
                <div>
                    <label>Ganho direção</label>
                    <input id="canSteeringGain" type="number" min="0" max="1" step="0.01" value="0.65">
                </div>
                <div>
                    <label>Botão homem-morto</label>
                    <input id="canDeadmanButton" value="BTN_TR">
                </div>
            </div>

            <div class="row">
                <button class="button-green" onclick="canArm()">ARMAR robô</button>
                <button class="button-secondary" onclick="canDisarm()">Desarmar</button>
            </div>

            <button class="button-danger" onclick="canEmergencyStop()">PARADA DE EMERGÊNCIA</button>

            <div class="can-status">
                <div class="can-status-box">
                    <strong id="canArmedText">OFF</strong>
                    <span>Armado</span>
                </div>
                <div class="can-status-box">
                    <strong id="canLeftDuty">0.000</strong>
                    <span>Duty esquerdo</span>
                </div>
                <div class="can-status-box">
                    <strong id="canRightDuty">0.000</strong>
                    <span>Duty direito</span>
                </div>
            </div>

            <h3>Log CAN</h3>
            <pre id="canLog">-</pre>
        </div>

        <div class="card">
            <h2>Dispositivos HID</h2>

            <button onclick="loadDevices()">Atualizar lista HID</button>

            <div class="row">
                <button class="button-secondary" onclick="hidDiagnostics()">Diagnóstico HID</button>
                <button class="button-secondary" onclick="hidLoadModules()">Carregar módulos HID</button>
            </div>

            <pre id="hidLog">-</pre>

            <label>Selecionar path fixo</label>
            <select id="devicePathSelect"></select>

            <button onclick="saveSelectedDevice()">Usar dispositivo selecionado</button>
            <button class="button-secondary" onclick="clearSelectedDevice()">Limpar seleção fixa</button>

            <label>Ou filtrar por nome parcial</label>
            <input id="deviceNameContains" placeholder="Ex: Gamepad, Pro Controller, Wireless, Xbox">

            <label>Deadzone dos analógicos</label>
            <input id="deadzone" type="number" min="0" max="0.5" step="0.01">

            <button onclick="saveGeneralConfig()">Salvar configuração HID</button>

            <div id="devicesList"></div>
        </div>

        <div class="card">
            <h2>Mapeamento rápido</h2>

            <p class="muted">
                Aperte um botão ou mova um eixo. O último código aparece abaixo.
                Dê um nome de ação e salve.
            </p>

            <label>Último código recebido</label>
            <input id="mapCode" placeholder="Ex: BTN_SOUTH ou ABS_X">

            <label>Nome da ação</label>
            <input id="mapAction" placeholder="Ex: Frente, Ré, Freio, Servo esquerda">

            <button onclick="saveMapping()">Salvar mapeamento</button>
            <button class="button-danger" onclick="removeMapping()">Remover mapeamento deste código</button>
        </div>

        <div class="card">
            <h2>Mapeamentos salvos</h2>
            <table>
                <thead>
                    <tr>
                        <th>Código</th>
                        <th>Ação</th>
                    </tr>
                </thead>
                <tbody id="mappingsTable"></tbody>
            </table>
        </div>
    </section>

    <section>
        <div style="display:flex; gap:16px; flex-wrap:wrap;">
        <div class="card" style="flex:1; min-width:340px;">
            <h2>LiDAR LDROBOT STL-06P</h2>

            <div class="lidar-container">
                <div class="lidar-canvas-wrapper">
                    <canvas id="lidarCanvas" width="420" height="420"></canvas>
                </div>

                <div class="lidar-metrics-row">
                    <div class="lidar-metric">
                        <strong id="lidarStatus">OFF</strong>
                        <span>Conexao</span>
                    </div>
                    <div class="lidar-metric">
                        <strong id="lidarPointCount">0</strong>
                        <span>Pontos</span>
                    </div>
                    <div class="lidar-metric">
                        <strong id="lidarClosest">-</strong>
                        <span>Mais proximo</span>
                    </div>
                    <div class="lidar-metric">
                        <strong id="lidarSpeed">0</strong>
                        <span>RPM</span>
                    </div>
                    <div class="lidar-metric">
                        <strong id="lidarHz">0</strong>
                        <span>FPS</span>
                    </div>
                </div>

                <div class="lidar-legend">
                    <div class="lidar-legend-item">
                        <span class="lidar-legend-dot" style="background:#ef4444;"></span> &lt; 1m
                    </div>
                    <div class="lidar-legend-item">
                        <span class="lidar-legend-dot" style="background:#f97316;"></span> &lt; 2m
                    </div>
                    <div class="lidar-legend-item">
                        <span class="lidar-legend-dot" style="background:#eab308;"></span> &lt; 4m
                    </div>
                    <div class="lidar-legend-item">
                        <span class="lidar-legend-dot" style="background:#22c55e;"></span> &gt; 4m
                    </div>
                </div>

                <p class="muted" id="lidarError" style="margin-top:8px;font-size:12px;"></p>
            </div>

            <label>Porta serial</label>
            <input id="lidarPort" placeholder="/dev/ttyUSB0" value="/dev/ttyUSB0">

            <div class="row">
                <button onclick="lidarLoadConfig()">Carregar config</button>
                <button class="button-secondary" onclick="lidarSaveConfig()">Salvar config</button>
            </div>
        </div>

        <div class="card" style="flex:1; min-width:340px;">
            <h2>Depth Camera Orbbec Astra Pro</h2>

            <div style="text-align:center;">
                <img id="depthImage" src=""
                     style="width:100%;max-width:420px;border-radius:10px;border:1px solid #475569;
                            background:#020617;min-height:220px;display:block;margin:0 auto;"
                     alt="Depth stream">
            </div>

            <div class="lidar-metrics-row" style="margin-top:8px;">
                <div class="lidar-metric">
                    <strong id="depthStatusCard">OFF</strong>
                    <span>Conexao</span>
                </div>
                <div class="lidar-metric">
                    <strong id="depthFpsCard">0</strong>
                    <span>FPS</span>
                </div>
                <div class="lidar-metric">
                    <strong id="depthMinCard">-</strong>
                    <span>Min (m)</span>
                </div>
                <div class="lidar-metric">
                    <strong id="depthMaxCard">-</strong>
                    <span>Max (m)</span>
                </div>
            </div>

            <div class="row" style="margin-top:6px;">
                <div>
                    <label style="font-size:11px;">Dist. minima (mm)</label>
                    <input id="depthMinMm" type="number" min="100" max="5000" value="500"
                           style="padding:4px;font-size:11px;" onchange="depthSaveConfig()">
                </div>
                <div>
                    <label style="font-size:11px;">Dist. maxima (mm)</label>
                    <input id="depthMaxMm" type="number" min="1000" max="20000" value="8000"
                           style="padding:4px;font-size:11px;" onchange="depthSaveConfig()">
                </div>
            </div>

            <div style="display:flex;justify-content:center;gap:16px;margin-top:6px;">
                <span style="font-size:10px;color:#ef4444;">&#9632; perto</span>
                <span style="font-size:10px;color:#eab308;">&#9632; medio</span>
                <span style="font-size:10px;color:#22c55e;">&#9632; longe</span>
                <span style="font-size:10px;color:#2563eb;">&#9632; muito longe</span>
            </div>

            <p class="muted" id="depthError" style="margin-top:6px;font-size:12px;"></p>
        </div>
        </div>

        <div class="card">
            <h2>GPS A76XX + Trajeto</h2>

            <div class="gps-metrics-row">
                <div class="gps-metric">
                    <strong id="gpsStatus">OFF</strong>
                    <span>Conexao</span>
                </div>
                <div class="gps-metric">
                    <strong><span id="gpsFixBadge" class="gps-fix-badge gps-fix-none">NONE</span></strong>
                    <span>Fix</span>
                </div>
                <div class="gps-metric">
                    <strong id="gpsSats">0/0</strong>
                    <span>Satelites Us./Vis.</span>
                </div>
                <div class="gps-metric">
                    <strong id="gpsHdop">-</strong>
                    <span>HDOP</span>
                </div>
            </div>

            <div class="gps-coords">
                <div>Lat: <span id="gpsLat">-</span> &nbsp; Lng: <span id="gpsLng">-</span></div>
                <div style="margin-top:4px;">Alt: <span id="gpsAlt">-</span> &nbsp; Vel: <span id="gpsSpeed">-</span> km/h &nbsp; Rumo: <span id="gpsHeading">-</span>&deg;</div>
            </div>

            <div class="gps-metric-row-2">
                <div>
                    <div class="gps-sats-canvas-wrapper">
                        <canvas id="gpsSatsCanvas" width="200" height="200"></canvas>
                    </div>
                    <div class="gps-sats-legend">
                        <div class="gps-sats-legend-item"><span class="gps-sats-legend-dot" style="background:#ef4444;"></span> SNR &lt; 20</div>
                        <div class="gps-sats-legend-item"><span class="gps-sats-legend-dot" style="background:#eab308;"></span> SNR &lt; 30</div>
                        <div class="gps-sats-legend-item"><span class="gps-sats-legend-dot" style="background:#22c55e;"></span> SNR &ge; 30</div>
                    </div>
                </div>
                <div class="gps-metric" style="display:flex;flex-direction:column;align-items:center;justify-content:center;">
                    <div style="text-align:center;">
                        <strong id="gpsUtcTime">--:--:--</strong>
                        <span>UTC</span>
                    </div>
                    <div style="text-align:center;margin-top:6px;">
                        <strong id="gpsTrajectoryPoints">0</strong>
                        <span>Pontos trajeto</span>
                    </div>
                </div>
            </div>

            <div id="gpsMap"></div>
            <label style="margin-top:6px;font-size:12px;"><input type="checkbox" id="gpsAutoCenter" checked onchange="gpsToggleAutoCenter()"> Seguir posicao (auto-center)</label>

            <div class="gps-controls-row">
                <button class="button-green" onclick="gpsTrajectoryStart()" id="btnGpsStart">&#9654; Iniciar</button>
                <button class="button-yellow" onclick="gpsTrajectoryPause()" id="btnGpsPause" disabled>&#9208; Pausar</button>
                <button class="button-secondary" onclick="gpsTrajectoryResume()" id="btnGpsResume" disabled>&#9654; Retomar</button>
                <button class="button-danger" onclick="gpsTrajectoryStop()" id="btnGpsStop" disabled>&#9209; Parar</button>
            </div>

            <div class="row" style="margin-top:6px;">
                <button class="button-green" onclick="gpsDownloadGPX()">&#11015; Baixar GPX</button>
                <button class="button-secondary button-yellow" onclick="gpsClearMap()">&#128465; Limpar mapa</button>
            </div>
            <div class="row" style="margin-top:4px;">
                <button class="button-secondary" onclick="gpsUploadGPX()">&#128194; Upload GPX</button>
                <input type="file" id="gpsUploadInput" accept=".gpx" style="display:none" onchange="gpsHandleUpload(this.files[0])">
                <button class="button-secondary" onclick="gpsTogglePower()" id="btnGpsPower">&#9889; Ligar GPS</button>
            </div>

            <div id="gpsFollowSection" style="display:none;margin-top:10px;padding:8px;background:#020617;border:1px solid #334155;border-radius:10px;">
                <div style="font-size:12px;color:#94a3b8;margin-bottom:6px;" id="gpsFollowInfo">Trajeto carregado: 0 pontos</div>
                <div class="row">
                    <button class="button-green" onclick="gpsFollowStart()" id="btnFollowStart">&#9654; Seguir trajeto</button>
                    <button class="button-danger" onclick="gpsFollowStop()" id="btnFollowStop" disabled>&#9209; Parar</button>
                </div>
                <div id="gpsFollowProgress" style="display:none;margin-top:6px;font-size:13px;color:#22c55e;">
                    WP <span id="followWpIndex">0</span>/<span id="followWpTotal">0</span> &middot; <span id="followDist">0</span>m &middot; <span id="followSpeed">0.0</span> km/h
                </div>
                <div style="margin-top:6px;display:grid;grid-template-columns:1fr 1fr;gap:6px;">
                    <div>
                        <label style="font-size:11px;">Dist. segura (cm)</label>
                        <input id="gpsSafeDist" type="number" min="10" max="200" value="50"
                               style="padding:4px;font-size:11px;margin-top:2px;" onchange="gpsSaveFollowConfig()">
                    </div>
                    <div>
                        <label style="font-size:11px;">Dist. critica (cm)</label>
                        <input id="gpsCritDist" type="number" min="5" max="100" value="30"
                               style="padding:4px;font-size:11px;margin-top:2px;" onchange="gpsSaveFollowConfig()">
                    </div>
                </div>
                <label style="font-size:12px;margin-top:4px;display:block;">
                    <input type="checkbox" id="gpsAvoidance" checked onchange="gpsToggleAvoidance()">
                    &#128737; Desviar de obstaculos (LiDAR)
                </label>
            </div>

            <div class="gps-config-row" style="margin-top:6px;">
                <div>
                    <label style="font-size:11px;margin-top:4px;">Porta AT</label>
                    <input id="gpsAtPort" placeholder="/dev/ttyUSB1" value="/dev/ttyUSB1">
                </div>
                <div style="display:flex;align-items:flex-end;gap:4px;">
                    <button onclick="gpsLoadConfig()" style="margin-top:4px;padding:6px 4px;font-size:11px;">Carregar</button>
                    <button onclick="gpsSaveConfig()" class="button-secondary" style="margin-top:4px;padding:6px 4px;font-size:11px;">Salvar</button>
                </div>
            </div>

            <h3 style="margin-top:12px;font-size:14px;">Log GPS</h3>
            <div id="gpsLog" class="gps-log">
                <div class="gps-log-line gps-log-info">Aguardando dados...</div>
            </div>

            <p class="muted" id="gpsError" style="margin-top:6px;font-size:12px;"></p>
        </div>

        <div class="card">
            <h2>Controle visual</h2>

            <div class="gamepad-panel">
                <div class="gamepad-section">
                    <h3>Botões principais</h3>

                    <div class="face-buttons">
                        <div class="gp-button gp-button-x" id="visual_BTN_NORTH">
                            <div class="btn-label"><span>X</span><small>BTN_NORTH</small></div>
                        </div>
                        <div class="gp-button gp-button-y" id="visual_BTN_WEST">
                            <div class="btn-label"><span>Y</span><small>BTN_WEST</small></div>
                        </div>
                        <div class="gp-button gp-button-a" id="visual_BTN_EAST">
                            <div class="btn-label"><span>A</span><small>BTN_EAST</small></div>
                        </div>
                        <div class="gp-button gp-button-b" id="visual_BTN_SOUTH">
                            <div class="btn-label"><span>B</span><small>BTN_SOUTH</small></div>
                        </div>
                    </div>

                    <div class="center-buttons">
                        <div class="center-button" id="visual_BTN_SELECT"><span>-</span><small>SELECT</small></div>
                        <div class="center-button" id="visual_BTN_MODE"><span>⌂</span><small>HOME</small></div>
                        <div class="center-button" id="visual_BTN_START"><span>+</span><small>START</small></div>
                    </div>

                    <div class="shoulder-grid">
                        <div class="shoulder-button" id="visual_BTN_TL"><span>L</span><small>BTN_TL</small></div>
                        <div class="shoulder-button" id="visual_BTN_TL2"><span>ZL</span><small>BTN_TL2</small></div>
                        <div class="shoulder-button" id="visual_BTN_TR2"><span>ZR</span><small>BTN_TR2</small></div>
                        <div class="shoulder-button" id="visual_BTN_TR"><span>R</span><small>BTN_TR</small></div>
                    </div>
                </div>

                <div class="gamepad-section">
                    <h3>D-Pad</h3>

                    <div class="dpad-widget">
                        <div class="dpad-key dpad-up" id="dpad_up">▲</div>
                        <div class="dpad-key dpad-left" id="dpad_left">◀</div>
                        <div class="dpad-key dpad-center">●</div>
                        <div class="dpad-key dpad-right" id="dpad_right">▶</div>
                        <div class="dpad-key dpad-down" id="dpad_down">▼</div>
                    </div>
                </div>

                <div class="gamepad-section">
                    <h3>Analógicos</h3>

                    <div class="stick-grid">
                        <div class="stick-widget">
                            <div class="stick-area">
                                <div class="stick-dot" id="leftStickDot"></div>
                            </div>
                            <div class="stick-readout" id="leftStickText">L: X 0.00 | Y 0.00</div>
                        </div>

                        <div class="stick-widget">
                            <div class="stick-area">
                                <div class="stick-dot" id="rightStickDot"></div>
                            </div>
                            <div class="stick-readout" id="rightStickText">R: X 0.00 | Y 0.00</div>
                        </div>
                    </div>
                </div>

                <div class="gamepad-section">
                    <h3>Gatilhos analógicos</h3>

                    <div class="trigger-bars">
                        <div class="trigger-card">
                            <strong>ZL / ABS_Z</strong>
                            <div class="vertical-meter">
                                <div class="vertical-fill" id="triggerLeftFill"></div>
                            </div>
                        </div>

                        <div class="trigger-card">
                            <strong>ZR / ABS_RZ</strong>
                            <div class="vertical-meter">
                                <div class="vertical-fill" id="triggerRightFill"></div>
                            </div>
                        </div>
                    </div>
                </div>
            </div>

            <button class="button-secondary technical-toggle" onclick="toggleTechnicalTables()">Mostrar/Ocultar tabelas técnicas</button>

            <div id="technicalTables" class="hidden-technical">
                <div class="compact-table">
                    <h3>Botões técnicos</h3>
                    <table>
                        <thead>
                            <tr>
                                <th>Código</th>
                                <th>Ação</th>
                                <th>Valor</th>
                                <th>Estado</th>
                            </tr>
                        </thead>
                        <tbody id="buttonsTable"></tbody>
                    </table>
                </div>

                <div class="compact-table">
                    <h3>Eixos técnicos</h3>
                    <table>
                        <thead>
                            <tr>
                                <th>Código</th>
                                <th>Ação</th>
                                <th>Valor bruto</th>
                                <th>Normalizado</th>
                                <th>Barra</th>
                            </tr>
                        </thead>
                        <tbody id="axesTable"></tbody>
                    </table>
                </div>
            </div>
        </div>

        <div class="card">
            <h2>Último evento</h2>
            <pre id="lastEvent">-</pre>
        </div>

        <div class="card">
            <h2>Log em tempo real</h2>
            <div id="log" class="log"></div>
        </div>
    </section>
</main>

<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<script src="https://cdn.socket.io/4.7.5/socket.io.min.js"></script>

<script>
    const socket = io();

    let config = {
        device_path: "",
        device_name_contains: "",
        deadzone: 0.05,
        mappings: {}
    };

    let buttons = {};
    let axes = {};
    let eventCounter = 0;

    const friendlyNames = {
        "BTN_SOUTH": "B",
        "BTN_EAST": "A",
        "BTN_NORTH": "X",
        "BTN_WEST": "Y",
        "BTN_C": "C",
        "BTN_Z": "Z",
        "BTN_TL": "L",
        "BTN_TR": "R",
        "BTN_TL2": "ZL",
        "BTN_TR2": "ZR",
        "BTN_SELECT": "-",
        "BTN_START": "+",
        "BTN_MODE": "HOME",
        "BTN_THUMBL": "L3",
        "BTN_THUMBR": "R3",
        "ABS_X": "Analógico esquerdo X",
        "ABS_Y": "Analógico esquerdo Y",
        "ABS_RX": "Analógico direito X",
        "ABS_RY": "Analógico direito Y",
        "ABS_Z": "Gatilho esquerdo",
        "ABS_RZ": "Gatilho direito",
        "ABS_HAT0X": "D-Pad horizontal",
        "ABS_HAT0Y": "D-Pad vertical"
    };


    const visualAxes = {
        ABS_X: 0,
        ABS_Y: 0,
        ABS_RX: 0,
        ABS_RY: 0,
        ABS_Z: 0,
        ABS_RZ: 0,
        ABS_HAT0X: 0,
        ABS_HAT0Y: 0
    };

    function toggleTechnicalTables() {
        const element = byId("technicalTables");
        element.classList.toggle("hidden-technical");
    }

    function setVisualButton(code, value) {
        const element = byId("visual_" + code);

        if (!element) {
            return;
        }

        if (value === 1 || value === 2) {
            element.classList.add("active");
        } else {
            element.classList.remove("active");
        }
    }

    function updateStick(dotId, textId, x, y, label) {
        const dot = byId(dotId);
        const text = byId(textId);

        if (!dot || !text) {
            return;
        }

        const radius = 58;
        const px = 50 + (x * radius / 1.7);
        const py = 50 + (y * radius / 1.7);

        dot.style.left = px + "%";
        dot.style.top = py + "%";

        text.textContent = label + ": X " + Number(x).toFixed(2) + " | Y " + Number(y).toFixed(2);
    }

    function updateDpad() {
        const x = Number(visualAxes.ABS_HAT0X || 0);
        const y = Number(visualAxes.ABS_HAT0Y || 0);

        byId("dpad_left")?.classList.toggle("active", x < -0.2);
        byId("dpad_right")?.classList.toggle("active", x > 0.2);
        byId("dpad_up")?.classList.toggle("active", y < -0.2);
        byId("dpad_down")?.classList.toggle("active", y > 0.2);
    }

    function updateTrigger(fillId, value) {
        const fill = byId(fillId);

        if (!fill) {
            return;
        }

        let percent = ((Number(value) + 1) / 2) * 100;

        if (Number(value) >= 0 && Number(value) <= 1) {
            percent = Number(value) * 100;
        }

        if (percent < 0) percent = 0;
        if (percent > 100) percent = 100;

        fill.style.height = percent + "%";
    }

    function updateVisualAxes() {
        updateStick("leftStickDot", "leftStickText", Number(visualAxes.ABS_X || 0), Number(visualAxes.ABS_Y || 0), "L");
        updateStick("rightStickDot", "rightStickText", Number(visualAxes.ABS_RX || 0), Number(visualAxes.ABS_RY || 0), "R");

        updateDpad();
        updateTrigger("triggerLeftFill", Number(visualAxes.ABS_Z || 0));
        updateTrigger("triggerRightFill", Number(visualAxes.ABS_RZ || 0));
    }


    function byId(id) {
        return document.getElementById(id);
    }

    function escapeHtml(value) {
        return String(value)
            .replaceAll("&", "&amp;")
            .replaceAll("<", "&lt;")
            .replaceAll(">", "&gt;")
            .replaceAll('"', "&quot;")
            .replaceAll("'", "&#039;");
    }

    function addLog(message) {
        const log = byId("log");
        const line = document.createElement("div");
        line.className = "log-line";

        const now = new Date().toLocaleTimeString();
        line.textContent = "[" + now + "] " + message;

        log.appendChild(line);
        log.scrollTop = log.scrollHeight;

        while (log.children.length > 300) {
            log.removeChild(log.firstChild);
        }
    }

    async function loadConfig() {
        const response = await fetch("/api/config");
        config = await response.json();

        byId("deviceNameContains").value = config.device_name_contains || "";
        byId("deadzone").value = config.deadzone ?? 0.05;

        renderMappings();
    }

    async function saveConfig() {
        const response = await fetch("/api/config", {
            method: "POST",
            headers: {
                "Content-Type": "application/json"
            },
            body: JSON.stringify(config)
        });

        config = await response.json();
        renderMappings();

        return config;
    }

    async function saveGeneralConfig() {
        config.device_name_contains = byId("deviceNameContains").value.trim();
        config.deadzone = parseFloat(byId("deadzone").value || "0.05");

        await saveConfig();
        addLog("Configuração geral HID salva.");
    }

    async function saveSelectedDevice() {
        const selectedPath = byId("devicePathSelect").value;

        if (!selectedPath) {
            alert("Selecione um dispositivo HID.");
            return;
        }

        config.device_path = selectedPath;
        await saveConfig();

        addLog("Dispositivo HID fixo salvo: " + selectedPath);
    }

    async function clearSelectedDevice() {
        config.device_path = "";
        await saveConfig();

        addLog("Seleção fixa HID removida.");
    }

    async function saveMapping() {
        const code = byId("mapCode").value.trim();
        const action = byId("mapAction").value.trim();

        if (!code || !action) {
            alert("Informe o código e o nome da ação.");
            return;
        }

        config.mappings[code] = action;
        await saveConfig();

        addLog("Mapeamento salvo: " + code + " => " + action);
    }

    async function removeMapping() {
        const code = byId("mapCode").value.trim();

        if (!code) {
            alert("Informe o código.");
            return;
        }

        delete config.mappings[code];
        await saveConfig();

        addLog("Mapeamento removido: " + code);
    }

    async function loadDevices() {
        const response = await fetch("/api/devices");
        const devices = await response.json();

        const select = byId("devicePathSelect");
        const list = byId("devicesList");

        select.innerHTML = "";
        list.innerHTML = "";

        const emptyOption = document.createElement("option");
        emptyOption.value = "";
        emptyOption.textContent = "Selecione...";
        select.appendChild(emptyOption);

        devices.forEach(device => {
            const option = document.createElement("option");
            option.value = device.path;
            option.textContent = device.name + " - " + device.path;

            if (config.device_path === device.path) {
                option.selected = true;
            }

            select.appendChild(option);

            const div = document.createElement("div");
            div.className = "device-item";

            div.innerHTML = `
                <strong>${escapeHtml(device.name || "-")}</strong>
                <small>${escapeHtml(device.path)}</small>
                <small>Botões: ${device.has_keys ? "sim" : "não"} | Eixos: ${device.has_axes ? "sim" : "não"}</small>
                <small>Provável gamepad: ${device.is_gamepad ? "sim" : "não"}</small>
                <button onclick="selectHidPath('${escapeHtml(device.path)}')">Selecionar este HID</button>
            `;

            list.appendChild(div);
        });

        if (devices.length === 0) {
            list.innerHTML = `<p class="muted">Nenhum dispositivo /dev/input/event* encontrado.</p>`;
        }
    }

    function selectHidPath(path) {
        byId("devicePathSelect").value = path;
        addLog("HID selecionado: " + path);
    }

    function renderMappings() {
        const tbody = byId("mappingsTable");
        tbody.innerHTML = "";

        const keys = Object.keys(config.mappings || {}).sort();

        if (keys.length === 0) {
            tbody.innerHTML = `
                <tr>
                    <td colspan="2" class="muted">Nenhum mapeamento salvo.</td>
                </tr>
            `;
            return;
        }

        keys.forEach(code => {
            const tr = document.createElement("tr");

            tr.innerHTML = `
                <td>${escapeHtml(code)}</td>
                <td>${escapeHtml(config.mappings[code])}</td>
            `;

            tr.onclick = () => {
                byId("mapCode").value = code;
                byId("mapAction").value = config.mappings[code];
            };

            tbody.appendChild(tr);
        });
    }

    function renderButtons() {
        const tbody = byId("buttonsTable");
        tbody.innerHTML = "";

        const keys = Object.keys(buttons).sort();
        byId("buttonCount").textContent = keys.length;

        if (keys.length === 0) {
            tbody.innerHTML = `
                <tr>
                    <td colspan="4" class="muted">Nenhum botão recebido ainda.</td>
                </tr>
            `;
            return;
        }

        keys.forEach(code => {
            const value = buttons[code];
            const action = config.mappings[code] || friendlyNames[code] || code;

            let state = "solto";
            if (value === 1) state = "pressionado";
            if (value === 2) state = "segurando";

            const tr = document.createElement("tr");

            tr.innerHTML = `
                <td>${escapeHtml(code)}</td>
                <td>${escapeHtml(action)}</td>
                <td>${value}</td>
                <td><span class="pill">${state}</span></td>
            `;

            tr.onclick = () => {
                byId("mapCode").value = code;
                byId("mapAction").value = action;
            };

            tbody.appendChild(tr);
        });
    }

    function axisBar(normalized) {
        let value = Number(normalized);

        if (value > 1) value = 1;
        if (value < -1) value = -1;

        const percent = Math.abs(value) * 50;

        if (value >= 0) {
            return `
                <div class="axis-bar">
                    <div class="axis-zero"></div>
                    <div class="axis-fill-positive" style="width:${percent}%"></div>
                </div>
            `;
        }

        return `
            <div class="axis-bar">
                <div class="axis-zero"></div>
                <div class="axis-fill-negative" style="width:${percent}%"></div>
            </div>
        `;
    }

    function renderAxes() {
        const tbody = byId("axesTable");
        tbody.innerHTML = "";

        const keys = Object.keys(axes).sort();
        byId("axisCount").textContent = keys.length;

        if (keys.length === 0) {
            tbody.innerHTML = `
                <tr>
                    <td colspan="5" class="muted">Nenhum eixo recebido ainda.</td>
                </tr>
            `;
            return;
        }

        keys.forEach(code => {
            const item = axes[code];
            const action = config.mappings[code] || friendlyNames[code] || code;

            const tr = document.createElement("tr");

            tr.innerHTML = `
                <td>${escapeHtml(code)}</td>
                <td>${escapeHtml(action)}</td>
                <td>${item.raw}</td>
                <td>${item.normalized}</td>
                <td>${axisBar(item.normalized)}</td>
            `;

            tr.onclick = () => {
                byId("mapCode").value = code;
                byId("mapAction").value = action;
            };

            tbody.appendChild(tr);
        });
    }

    async function btPost(url, data = {}) {
        const response = await fetch(url, {
            method: "POST",
            headers: {
                "Content-Type": "application/json"
            },
            body: JSON.stringify(data)
        });

        const result = await response.json();

        byId("btLog").textContent = JSON.stringify(result, null, 2);

        return result;
    }

    async function btGet(url) {
        const response = await fetch(url);
        const result = await response.json();

        byId("btLog").textContent = JSON.stringify(result, null, 2);

        return result;
    }

    function getBtMac() {
        return byId("btMac").value.trim().toUpperCase();
    }

    async function btPowerOn() {
        await btPost("/api/bluetooth/power-on");
        addLog("Bluetooth power on executado.");
    }

    async function btPowerOff() {
        await btPost("/api/bluetooth/power-off");
        addLog("Bluetooth power off executado.");
    }

    async function btPrepare() {
        await btPost("/api/bluetooth/prepare");
        addLog("Bluetooth preparado.");
    }

    async function btScan() {
        byId("btLog").textContent = "Escaneando Bluetooth por 8 segundos...";

        const result = await btPost("/api/bluetooth/scan", {
            duration: 8
        });

        renderBtDevices(result.devices || []);

        addLog("Scan Bluetooth finalizado.");
    }

    async function btScanOn() {
        await btPost("/api/bluetooth/scan-on");
        addLog("Bluetooth scan ON executado.");
    }

    async function btScanOff() {
        await btPost("/api/bluetooth/scan-off");
        addLog("Bluetooth scan OFF executado.");
    }

    async function btDevices() {
        const result = await btGet("/api/bluetooth/devices");
        renderBtDevices(result.devices || []);
        addLog("Lista de dispositivos Bluetooth atualizada.");
    }

    function renderBtDevices(devices) {
        const container = byId("btDevicesList");
        container.innerHTML = "";

        if (!devices.length) {
            container.innerHTML = `<p class="muted">Nenhum dispositivo Bluetooth encontrado.</p>`;
            return;
        }

        devices.forEach(device => {
            const div = document.createElement("div");
            div.className = "device-item";

            div.innerHTML = `
                <strong>${escapeHtml(device.name || "-")}</strong>
                <small>${escapeHtml(device.mac)}</small>
                <button onclick="btUseMac('${device.mac}')">Usar este MAC</button>
                <button onclick="btPairConnect('${device.mac}')">Parear + Trust + Conectar</button>
                <button class="button-secondary" onclick="btConnect('${device.mac}')">Conectar</button>
                <button class="button-secondary" onclick="btDisconnect('${device.mac}')">Desconectar</button>
                <button class="button-danger" onclick="btRemove('${device.mac}')">Remover</button>
            `;

            container.appendChild(div);
        });
    }

    function btUseMac(mac) {
        byId("btMac").value = mac;
        addLog("MAC Bluetooth selecionado: " + mac);
    }

    async function btPairConnect(mac) {
        byId("btLog").textContent = "Pareando e conectando " + mac + "...";

        await btPost("/api/bluetooth/pair-connect", {
            mac: mac
        });

        addLog("Pair/trust/connect executado para " + mac);

        setTimeout(() => {
            loadDevices();
            hidDiagnostics();
        }, 2000);
    }

    async function btConnect(mac) {
        await btPost("/api/bluetooth/connect", {
            mac: mac
        });

        addLog("Connect executado para " + mac);

        setTimeout(() => {
            loadDevices();
            hidDiagnostics();
        }, 2000);
    }

    async function btDisconnect(mac) {
        await btPost("/api/bluetooth/disconnect", {
            mac: mac
        });

        addLog("Disconnect executado para " + mac);

        setTimeout(() => {
            loadDevices();
        }, 1800);
    }

    async function btRemove(mac) {
        await btPost("/api/bluetooth/remove", {
            mac: mac
        });

        addLog("Remove executado para " + mac);

        setTimeout(() => {
            loadDevices();
            btDevices();
            hidDiagnostics();
        }, 1800);
    }

    async function btPairConnectManual() {
        const mac = getBtMac();

        if (!mac) {
            alert("Informe o MAC.");
            return;
        }

        await btPairConnect(mac);
    }

    async function btConnectManual() {
        const mac = getBtMac();

        if (!mac) {
            alert("Informe o MAC.");
            return;
        }

        await btConnect(mac);
    }

    async function btDisconnectManual() {
        const mac = getBtMac();

        if (!mac) {
            alert("Informe o MAC.");
            return;
        }

        await btDisconnect(mac);
    }

    async function btRemoveManual() {
        const mac = getBtMac();

        if (!mac) {
            alert("Informe o MAC.");
            return;
        }

        await btRemove(mac);
    }

    async function hidDiagnostics() {
        const response = await fetch("/api/hid/diagnostics");
        const result = await response.json();

        byId("hidLog").textContent = JSON.stringify(result, null, 2);

        addLog("Diagnóstico HID executado.");
    }

    async function hidLoadModules() {
        const response = await fetch("/api/hid/load-modules", {
            method: "POST",
            headers: {
                "Content-Type": "application/json"
            },
            body: JSON.stringify({})
        });

        const result = await response.json();

        byId("hidLog").textContent = JSON.stringify(result, null, 2);

        addLog("Tentativa de carregar módulos HID executada.");
    }

    function readCanForm() {
        return {
            interface: byId("canInterface").value || "can0",
            bitrate: parseInt(byId("canBitrate").value || "500000"),
            left_id: parseInt(byId("canLeftId").value || "1"),
            right_id: parseInt(byId("canRightId").value || "2"),
            max_duty: parseFloat(byId("canMaxDuty").value || "0.25"),
            steering_gain: parseFloat(byId("canSteeringGain").value || "0.65"),
            deadman_button: byId("canDeadmanButton").value.trim() || "BTN_TR"
        };
    }

    async function canSaveConfig() {
        const payload = readCanForm();

        const response = await fetch("/api/can/config", {
            method: "POST",
            headers: {
                "Content-Type": "application/json"
            },
            body: JSON.stringify(payload)
        });

        return await response.json();
    }

    async function canLoadConfig() {
        const response = await fetch("/api/can/config");
        const result = await response.json();

        if (result.interface) byId("canInterface").value = result.interface;
        if (result.bitrate) byId("canBitrate").value = result.bitrate;
        if (result.left_id !== undefined) byId("canLeftId").value = result.left_id;
        if (result.right_id !== undefined) byId("canRightId").value = result.right_id;
        if (result.max_duty !== undefined) byId("canMaxDuty").value = result.max_duty;
        if (result.steering_gain !== undefined) byId("canSteeringGain").value = result.steering_gain;
        if (result.deadman_button) byId("canDeadmanButton").value = result.deadman_button;
    }

    async function canScan() {
        const response = await fetch("/api/can/scan");
        const result = await response.json();

        const select = byId("canInterface");
        select.innerHTML = "";

        const interfaces = result.interfaces || [];

        if (interfaces.length === 0) {
            const option = document.createElement("option");
            option.value = "can0";
            option.textContent = "can0";
            select.appendChild(option);
        }

        interfaces.forEach(item => {
            const option = document.createElement("option");
            option.value = item.name;
            option.textContent = item.name + " - " + item.state;
            select.appendChild(option);
        });

        byId("canLog").textContent = JSON.stringify(result, null, 2);
        addLog("Scan CANable/SocketCAN executado.");
    }

    async function canSetup() {
        await canSaveConfig();

        const response = await fetch("/api/can/setup", {
            method: "POST",
            headers: {
                "Content-Type": "application/json"
            },
            body: JSON.stringify(readCanForm())
        });

        const result = await response.json();
        byId("canLog").textContent = JSON.stringify(result, null, 2);

        addLog("Setup CAN executado.");
    }

    async function canArm() {
        await canSaveConfig();

        const response = await fetch("/api/can/arm", {
            method: "POST",
            headers: {
                "Content-Type": "application/json"
            },
            body: JSON.stringify({})
        });

        const result = await response.json();
        byId("canLog").textContent = JSON.stringify(result, null, 2);

        addLog("Robô armado via CAN.");
    }

    async function canDisarm() {
        const response = await fetch("/api/can/disarm", {
            method: "POST",
            headers: {
                "Content-Type": "application/json"
            },
            body: JSON.stringify({})
        });

        const result = await response.json();
        byId("canLog").textContent = JSON.stringify(result, null, 2);

        addLog("Robô desarmado via CAN.");
    }

    async function canEmergencyStop() {
        const response = await fetch("/api/can/emergency-stop", {
            method: "POST",
            headers: {
                "Content-Type": "application/json"
            },
            body: JSON.stringify({})
        });

        const result = await response.json();
        byId("canLog").textContent = JSON.stringify(result, null, 2);

        addLog("PARADA DE EMERGÊNCIA enviada via CAN.");
    }

    function updateCanStatus(data) {
        if (!data) return;

        byId("canArmedText").textContent = data.armed ? "ON" : "OFF";
        byId("canArmedText").className = data.armed ? "status-ok" : "status-off";
        byId("canLeftDuty").textContent = Number(data.last_left || 0).toFixed(3);
        byId("canRightDuty").textContent = Number(data.last_right || 0).toFixed(3);
    }

    socket.on("can_status", data => {
        updateCanStatus(data);
    });

    socket.on("gamepad_status", data => {
        const connection = byId("connection");

        if (data.connected) {
            connection.textContent = "conectado";
            connection.className = "status-ok";
        } else {
            connection.textContent = "desconectado";
            connection.className = "status-off";
        }

        byId("deviceName").textContent = data.device_name || "-";
        byId("devicePath").textContent = data.device_path || "-";

        let errorText = data.error || "-";

        if (!data.evdev_available) {
            errorText = "evdev não carregado: " + data.evdev_import_error;
        }

        byId("errorText").textContent = errorText;
    });

    socket.on("gamepad_event", event => {
        eventCounter++;
        byId("eventCount").textContent = eventCounter;

        byId("lastEvent").textContent = JSON.stringify(event, null, 2);

        if (event.code) {
            byId("mapCode").value = event.code;

            if (!byId("mapAction").value) {
                byId("mapAction").value = event.action || event.code;
            }
        }

        if (event.kind === "button") {
            buttons[event.code] = event.value;
            setVisualButton(event.code, event.value);
            renderButtons();

            if (event.value === 1) {
                addLog("BOTÃO pressionado: " + event.code + " => " + event.action);
            }

            if (event.value === 0) {
                addLog("BOTÃO solto: " + event.code + " => " + event.action);
            }
        }

        if (event.kind === "axis") {
            axes[event.code] = {
                raw: event.value,
                normalized: event.normalized
            };

            visualAxes[event.code] = Number(event.normalized || 0);
            updateVisualAxes();
            renderAxes();
        }

        if (event.kind === "error") {
            addLog("ERRO: " + event.message);
        }
    });

    loadConfig().then(() => {
        loadDevices();
        renderButtons();
        renderAxes();
        updateVisualAxes();
        canLoadConfig();
        canScan();
        lidarLoadConfig();
    });

    let lidarPoints = [];
    let allLidarPoints = [];
    const MAX_LIDAR_AGE = 3000;
    let lidarConnected = false;
    let lidarSpeedRpm = 0;
    let lastLidarFrame = 0;

    async function lidarLoadConfig() {
        try {
            const response = await fetch("/api/lidar/config");
            const result = await response.json();

            byId("lidarPort").value = result.port || "/dev/ttyUSB0";
        } catch (e) {
            addLog("Erro ao carregar config LiDAR.");
        }
    }

    async function lidarSaveConfig() {
        const port = byId("lidarPort").value.trim() || "/dev/ttyUSB0";

        try {
            const response = await fetch("/api/lidar/config", {
                method: "POST",
                headers: { "Content-Type": "application/json" },
                body: JSON.stringify({ port: port })
            });

            const result = await response.json();

            byId("lidarPort").value = result.port || port;
            addLog("Configuracao LiDAR salva. Reiniciando leitura...");
        } catch (e) {
            addLog("Erro ao salvar config LiDAR.");
        }
    }

    socket.on("lidar_frame", (data) => {
        if (!data || !data.points) return;

        const now = Date.now();

        for (const pt of data.points) {
            allLidarPoints.push({
                x: pt.x || 0,
                y: pt.y || 0,
                distance: pt.distance || 0,
                angle: pt.angle || 0,
                addedAt: now
            });
        }

        allLidarPoints = allLidarPoints.filter(p => (now - p.addedAt) < MAX_LIDAR_AGE);

        let minDist = Infinity;
        for (const p of allLidarPoints) {
            if (p.distance > 0 && p.distance < minDist) minDist = p.distance;
        }

        if (minDist < Infinity) {
            if (minDist >= 1000) {
                byId("lidarClosest").textContent = (minDist / 1000).toFixed(1) + "m";
            } else {
                byId("lidarClosest").textContent = Math.round(minDist) + "mm";
            }
        }

        lidarPoints = allLidarPoints;
        lidarConnected = true;

        if (lidarSpeedRpm > 0) {
            byId("lidarSpeed").textContent = Math.round(lidarSpeedRpm);
        }

        byId("lidarPointCount").textContent = allLidarPoints.length;

        if (lastLidarFrame > 0) {
            const fps = 1000 / (now - lastLidarFrame);
            byId("lidarHz").textContent = Math.round(fps);
        }

        lastLidarFrame = now;

        byId("lidarStatus").textContent = "ON";
        byId("lidarStatus").className = "status-ok";
        byId("lidarError").textContent = "";

        drawLidar();
    });

    socket.on("lidar_status", (data) => {
        if (!data) return;

        lidarConnected = data.connected || false;
        lidarSpeedRpm = data.rotation_speed || 0;

        if (data.connected) {
            byId("lidarStatus").textContent = "ON";
            byId("lidarStatus").className = "status-ok";
        } else {
            byId("lidarStatus").textContent = "OFF";
            byId("lidarStatus").className = "status-off";
        }

        if (data.error) {
            byId("lidarError").textContent = data.error;
        } else {
            byId("lidarError").textContent = "";
        }

        if (data.speed > 0) {
            const rpm = (data.speed / 360.0) * 60.0;
            byId("lidarSpeed").textContent = Math.round(rpm);
        }

        if (!data.serial_available && data.serial_import_error) {
            byId("lidarError").textContent = "pyserial nao disponivel: " + data.serial_import_error;
        }
    });

    function drawLidar() {
        const canvas = byId("lidarCanvas");

        if (!canvas) return;

        const ctx = canvas.getContext("2d");
        const w = canvas.width;
        const h = canvas.height;
        const cx = w / 2;
        const cy = h / 2;

        ctx.clearRect(0, 0, w, h);

        const scale = 0.025;
        const maxDrawMm = 12000;
        const now = Date.now();

        for (let rMm = 1000; rMm <= maxDrawMm; rMm += 1000) {
            const rPx = rMm * scale;

            ctx.beginPath();
            ctx.arc(cx, cy, rPx, 0, Math.PI * 2);

            if (rMm <= 4000) {
                ctx.strokeStyle = "rgba(71, 85, 105, 0.25)";
            } else {
                ctx.strokeStyle = "rgba(71, 85, 105, 0.10)";
            }

            ctx.lineWidth = 1;
            ctx.stroke();

            ctx.fillStyle = "#475569";
            ctx.font = "10px monospace";
            ctx.fillText((rMm / 1000) + "m", cx + 3, cy - rPx - 4);
        }

        ctx.beginPath();
        ctx.arc(cx, cy, 500 * scale, 0, Math.PI * 2);
        ctx.strokeStyle = "rgba(239, 68, 68, 0.18)";
        ctx.lineWidth = 1.5;
        ctx.stroke();
        ctx.setLineDash([4, 6]);
        ctx.stroke();
        ctx.setLineDash([]);

        ctx.beginPath();
        ctx.arc(cx, cy, 1000 * scale, 0, Math.PI * 2);
        ctx.strokeStyle = "rgba(249, 115, 22, 0.14)";
        ctx.lineWidth = 1.5;
        ctx.stroke();
        ctx.setLineDash([4, 6]);
        ctx.stroke();
        ctx.setLineDash([]);

        ctx.beginPath();
        ctx.moveTo(cx, 0);
        ctx.lineTo(cx, h);
        ctx.strokeStyle = "rgba(71, 85, 105, 0.15)";
        ctx.lineWidth = 1;
        ctx.stroke();

        ctx.beginPath();
        ctx.moveTo(0, cy);
        ctx.lineTo(w, cy);
        ctx.strokeStyle = "rgba(71, 85, 105, 0.15)";
        ctx.lineWidth = 1;
        ctx.stroke();

        const robotW = 30 * scale;
        const robotH = 50 * scale;

        ctx.fillStyle = "#38bdf8";
        ctx.strokeStyle = "#075985";
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.roundRect(cx - robotW / 2, cy - robotH / 2, robotW, robotH, 4);
        ctx.fill();
        ctx.stroke();

        ctx.fillStyle = "#e5e7eb";
        ctx.beginPath();
        ctx.arc(cx, cy - robotH / 2 - 3, 4, 0, Math.PI * 2);
        ctx.fill();

        if (!allLidarPoints || allLidarPoints.length === 0) return;

        for (const pt of allLidarPoints) {
            const age = now - (pt.addedAt || 0);

            if (age >= MAX_LIDAR_AGE) continue;

            const xMm = pt.x || 0;
            const yMm = pt.y || 0;
            const distMm = pt.distance || Math.sqrt(xMm * xMm + yMm * yMm);

            if (distMm < 150 || distMm > maxDrawMm) continue;

            const px = cx + xMm * scale;
            const py = cy - yMm * scale;

            if (px < 0 || px > w || py < 0 || py > h) continue;

            const ageAlpha = 1.0 - (age / MAX_LIDAR_AGE);

            let baseAlpha;

            if (distMm < 1000) {
                baseAlpha = 0.75;
            } else if (distMm < 2000) {
                baseAlpha = 0.60;
            } else if (distMm < 4000) {
                baseAlpha = 0.45;
            } else {
                const fade = Math.min(1, (distMm - 4000) / 8000);
                baseAlpha = 0.40 - fade * 0.25;
            }

            const finalAlpha = (baseAlpha * ageAlpha).toFixed(3);

            let r, g, b;

            if (distMm < 1000) {
                r = 239; g = 68; b = 68;
            } else if (distMm < 2000) {
                r = 249; g = 115; b = 22;
            } else if (distMm < 4000) {
                r = 234; g = 179; b = 8;
            } else {
                r = 34; g = 197; b = 94;
            }

            ctx.fillStyle = "rgba(" + r + "," + g + "," + b + "," + finalAlpha + ")";
            ctx.beginPath();
            ctx.arc(px, py, 1.8, 0, Math.PI * 2);
            ctx.fill();
        }
    }


    let gpsMap = null;
    let gpsMarker = null;
    let gpsTrajectoryLine = null;
    let gpsTrajectoryCoords = [];
    let gpsAutoCenter = true;
    let gpsSatellites = [];
    let gpsPowered = false;
    let gpsConnected = false;
    let gpsLastFix = 0;
    let gpsUploadedLine = null;
    let gpsWaypointMarker = null;
    let gpsLastSpeed = "0.0";

    function gpsInitMap() {
        if (gpsMap) return;
        const mapEl = byId("gpsMap");
        if (!mapEl) return;
        gpsMap = L.map("gpsMap", { attributionControl: false, zoomControl: true }).setView([-23.55, -46.63], 15);
        L.tileLayer("https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png", {
            maxZoom: 19
        }).addTo(gpsMap);
        gpsTrajectoryLine = L.polyline([], { color: "#22c55e", weight: 3, opacity: 0.8 }).addTo(gpsMap);
    }

    function gpsToggleAutoCenter() {
        gpsAutoCenter = byId("gpsAutoCenter").checked;
    }

    function gpsUpdateMap(lat, lng) {
        if (!gpsMap) gpsInitMap();
        if (!gpsMap) return;
        const latlng = L.latLng(lat, lng);
        if (!gpsMarker) {
            const icon = L.divIcon({
                className: "",
                html: '<div style="width:14px;height:14px;background:#2563eb;border:2px solid #fff;border-radius:50%;box-shadow:0 0 8px rgba(37,99,235,0.8);"></div>',
                iconSize: [14, 14],
                iconAnchor: [7, 7]
            });
            gpsMarker = L.marker(latlng, { icon: icon }).addTo(gpsMap);
        } else {
            gpsMarker.setLatLng(latlng);
        }
        if (gpsAutoCenter) {
            gpsMap.panTo(latlng, { animate: true, duration: 0.5 });
        }
    }

    function gpsAddTrajectoryPoint(lat, lng) {
        if (!gpsMap) gpsInitMap();
        if (!gpsMap) return;
        const ll = L.latLng(lat, lng);
        gpsTrajectoryCoords.push(ll);
        if (gpsTrajectoryLine) {
            gpsTrajectoryLine.setLatLngs(gpsTrajectoryCoords);
        }
    }

    function gpsLoadTrajectoryOnMap(points) {
        if (!gpsMap) gpsInitMap();
        if (!gpsMap) return;
        gpsTrajectoryCoords = [];
        for (const pt of points) {
            if (pt.lat != null && pt.lng != null) {
                gpsTrajectoryCoords.push(L.latLng(pt.lat, pt.lng));
            }
        }
        if (gpsTrajectoryLine) {
            gpsTrajectoryLine.setLatLngs(gpsTrajectoryCoords);
        }
        if (gpsTrajectoryCoords.length > 0) {
            gpsMap.fitBounds(gpsTrajectoryLine.getBounds().pad(0.1));
        }
    }

    function drawGpsSatellites() {
        const canvas = byId("gpsSatsCanvas");
        if (!canvas) return;
        const ctx = canvas.getContext("2d");
        const w = canvas.width;
        const h = canvas.height;
        const cx = w / 2;
        const cy = h / 2;
        ctx.clearRect(0, 0, w, h);

        for (let r = 30; r <= 90; r += 30) {
            ctx.beginPath();
            ctx.arc(cx, cy, r, 0, Math.PI * 2);
            ctx.strokeStyle = "rgba(71,85,105,0.2)";
            ctx.lineWidth = 1;
            ctx.stroke();
        }

        ctx.beginPath();
        ctx.arc(cx, cy, 90, 0, Math.PI * 2);
        ctx.strokeStyle = "#475569";
        ctx.lineWidth = 1;
        ctx.stroke();

        ctx.beginPath();
        ctx.moveTo(cx, cy - 93);
        ctx.lineTo(cx, cy + 93);
        ctx.strokeStyle = "rgba(71,85,105,0.15)";
        ctx.stroke();
        ctx.beginPath();
        ctx.moveTo(cx - 93, cy);
        ctx.lineTo(cx + 93, cy);
        ctx.stroke();

        ctx.fillStyle = "#64748b";
        ctx.font = "10px monospace";
        ctx.fillText("N", cx - 6, cy - 78);
        ctx.fillText("S", cx - 6, cy + 84);
        ctx.fillText("E", cx + 76, cy + 4);
        ctx.fillText("W", cx - 84, cy + 4);

        for (const sat of gpsSatellites) {
            const el = sat.elevation != null ? sat.elevation : 0;
            const az = sat.azimuth != null ? sat.azimuth : 0;
            const snr = sat.snr != null ? sat.snr : 0;
            const r = 90 * (1 - el / 90);
            const angleRad = ((az - 90) * Math.PI) / 180;
            const px = cx + r * Math.cos(angleRad);
            const py = cy + r * Math.sin(angleRad);

            let color;
            if (snr < 20) color = "#ef4444";
            else if (snr < 30) color = "#eab308";
            else color = "#22c55e";

            ctx.fillStyle = color;
            ctx.beginPath();
            ctx.arc(px, py, 3.5, 0, Math.PI * 2);
            ctx.fill();

            ctx.fillStyle = "#fff";
            ctx.font = "8px monospace";
            ctx.fillText(sat.prn, px + 5, py - 4);
        }
    }

    function gpsLogLine(entry) {
        const logDiv = byId("gpsLog");
        if (!logDiv) return;
        const line = document.createElement("div");
        line.className = "gps-log-line gps-log-" + (entry.level || "info");
        line.textContent = "[" + (entry.time || "") + "] " + (entry.message || "");
        logDiv.appendChild(line);
        logDiv.scrollTop = logDiv.scrollHeight;
        while (logDiv.children.length > 100) {
            logDiv.removeChild(logDiv.firstChild);
        }
    }

    socket.on("gps_status", data => {
        if (!data) return;
        gpsConnected = data.connected || false;
        gpsPowered = data.gps_powered || false;

        const statusEl = byId("gpsStatus");
        if (gpsConnected) {
            statusEl.textContent = "ON";
            statusEl.className = "status-ok";
        } else {
            statusEl.textContent = "OFF";
            statusEl.className = "status-off";
        }

        const fix = data.fix || 0;
        gpsLastFix = fix;
        const badge = byId("gpsFixBadge");
        if (fix >= 2) { badge.textContent = "3D"; badge.className = "gps-fix-badge gps-fix-3d"; }
        else if (fix === 1) { badge.textContent = "2D"; badge.className = "gps-fix-badge gps-fix-2d"; }
        else { badge.textContent = "NONE"; badge.className = "gps-fix-badge gps-fix-none"; }

        byId("gpsSats").textContent = (data.satellites_used || 0) + "/" + (data.satellites_in_view || 0);
        byId("gpsHdop").textContent = data.hdop != null ? Number(data.hdop).toFixed(1) : "-";

        if (data.latitude != null) byId("gpsLat").textContent = Number(data.latitude).toFixed(7);
        if (data.longitude != null) byId("gpsLng").textContent = Number(data.longitude).toFixed(7);
        if (data.altitude != null) byId("gpsAlt").textContent = Number(data.altitude).toFixed(1) + "m";
        else byId("gpsAlt").textContent = "-";

        gpsLastSpeed = data.speed_kmh != null ? Number(data.speed_kmh).toFixed(1) : "0.0";
        byId("gpsSpeed").textContent = gpsLastSpeed !== "0.0" ? gpsLastSpeed : "-";
        byId("gpsHeading").textContent = data.heading != null ? Math.round(Number(data.heading)) : "-";
        if (data.utc_time) {
            const t = String(data.utc_time).replace(/\.\d+$/, "");
            if (t.length >= 6) byId("gpsUtcTime").textContent = t.slice(0,2) + ":" + t.slice(2,4) + ":" + t.slice(4,6);
            else byId("gpsUtcTime").textContent = t;
        } else {
            byId("gpsUtcTime").textContent = "--:--:--";
        }

        if (data.satellites && data.satellites.length > 0) {
            gpsSatellites = data.satellites;
        } else if (data.satellites_in_view === 0) {
            gpsSatellites = [];
        }
        drawGpsSatellites();

        if (data.latitude != null && data.longitude != null && data.latitude !== 0 && data.longitude !== 0) {
            gpsInitMap();
            gpsUpdateMap(data.latitude, data.longitude);
        }

        if (data.error) {
            byId("gpsError").textContent = data.error;
        } else {
            byId("gpsError").textContent = "";
        }

        const btnPwr = byId("btnGpsPower");
        if (gpsPowered) {
            btnPwr.textContent = "⚡ Desligar GPS";
            btnPwr.className = "button-danger";
        } else {
            btnPwr.textContent = "⚡ Ligar GPS";
            btnPwr.className = "button-secondary";
        }
    });

    socket.on("gps_trajectory_status", data => {
        if (!data) return;
        byId("gpsTrajectoryPoints").textContent = data.point_count || 0;

        const recording = data.recording || false;
        const paused = data.paused || false;

        byId("btnGpsStart").disabled = recording;
        byId("btnGpsPause").disabled = !recording || paused;
        byId("btnGpsResume").disabled = !recording || !paused;
        byId("btnGpsStop").disabled = !recording;

        if (recording && paused) {
            byId("gpsTrajectoryPoints").textContent = (data.point_count || 0) + " PAUSADO";
        }
    });

    socket.on("gps_trajectory_point", point => {
        if (!point || point.lat == null || point.lng == null) return;
        gpsAddTrajectoryPoint(point.lat, point.lng);
    });

    socket.on("gps_log", gpsLogLine);

    socket.on("gps_follow_status", data => {
        if (!data) return;
        const active = data.active || false;
        byId("btnFollowStart").disabled = active;
        byId("btnFollowStop").disabled = !active;
        if (active) {
            byId("gpsFollowProgress").style.display = "block";
            byId("followWpIndex").textContent = data.wp_index || 0;
            byId("followWpTotal").textContent = data.wp_total || 0;
            byId("followDist").textContent = data.distance || 0;
            if (gpsLastSpeed) byId("followSpeed").textContent = gpsLastSpeed;
        } else {
            byId("gpsFollowProgress").style.display = "none";
            byId("followWpIndex").textContent = "0";
            byId("followWpTotal").textContent = "0";
            byId("followDist").textContent = "0";
            if (gpsWaypointMarker && gpsMap) gpsMap.removeLayer(gpsWaypointMarker);
            gpsWaypointMarker = null;
        }
    });

    socket.on("depth_frame", data => {
        if (!data || !data.image) return;
        byId("depthImage").src = data.image;
        byId("depthStatusCard").textContent = "ON";
        byId("depthStatusCard").className = "status-ok";
        if (data.min_mm) byId("depthMinCard").textContent = (data.min_mm / 1000).toFixed(1) + "m";
        if (data.max_mm) byId("depthMaxCard").textContent = (data.max_mm / 1000).toFixed(1) + "m";
        byId("depthError").textContent = "";
    });

    socket.on("depth_status", data => {
        if (!data) return;
        if (data.connected) {
            byId("depthStatusCard").textContent = "ON";
            byId("depthStatusCard").className = "status-ok";
        } else {
            byId("depthStatusCard").textContent = "OFF";
            byId("depthStatusCard").className = "status-off";
        }
        if (data.fps) byId("depthFpsCard").textContent = Math.round(data.fps);
        if (data.error) {
            byId("depthError").textContent = data.error;
        } else {
            byId("depthError").textContent = "";
        }
        if (!data.depth_available && data.depth_import_error) {
            byId("depthError").textContent = data.depth_import_error;
        }
    });

    async function gpsLoadConfig() {
        try {
            const resp = await fetch("/api/gps/config");
            const cfg = await resp.json();
            byId("gpsAtPort").value = cfg.at_port || "/dev/ttyUSB1";
        } catch (e) {
            addLog("Erro ao carregar config GPS.");
        }
    }

    async function gpsSaveConfig() {
        const atPort = byId("gpsAtPort").value.trim() || "/dev/ttyUSB1";
        try {
            await fetch("/api/gps/config", {
                method: "POST",
                headers: { "Content-Type": "application/json" },
                body: JSON.stringify({ at_port: atPort })
            });
            addLog("Config GPS salva. Reiniciando...");
        } catch (e) {
            addLog("Erro ao salvar config GPS.");
        }
    }

    async function gpsTogglePower() {
        try {
            if (gpsPowered) {
                const resp = await fetch("/api/gps/power/off", { method: "POST" });
                const result = await resp.json();
                addLog(result.message || result.error || "GPS desligado");
            } else {
                const resp = await fetch("/api/gps/power/on", { method: "POST" });
                const result = await resp.json();
                addLog(result.message || result.error || "GPS ligado");
            }
        } catch (e) {
            addLog("Erro ao alternar GPS.");
        }
    }

    async function gpsTrajectoryStart() {
        try {
            const resp = await fetch("/api/gps/trajectory/start", { method: "POST" });
            const result = await resp.json();
            gpsTrajectoryCoords = [];
            if (gpsTrajectoryLine) gpsTrajectoryLine.setLatLngs([]);
            addLog(result.message || result.error || "");
        } catch (e) {
            addLog("Erro ao iniciar gravacao.");
        }
    }

    async function gpsTrajectoryPause() {
        try {
            const resp = await fetch("/api/gps/trajectory/pause", { method: "POST" });
            const result = await resp.json();
            addLog(result.message || result.error || "");
        } catch (e) {
            addLog("Erro ao pausar gravacao.");
        }
    }

    async function gpsTrajectoryResume() {
        try {
            const resp = await fetch("/api/gps/trajectory/resume", { method: "POST" });
            const result = await resp.json();
            addLog(result.message || result.error || "");
        } catch (e) {
            addLog("Erro ao retomar gravacao.");
        }
    }

    async function gpsTrajectoryStop() {
        try {
            const resp = await fetch("/api/gps/trajectory/stop", { method: "POST" });
            const result = await resp.json();
            addLog(result.message || result.error || "");
        } catch (e) {
            addLog("Erro ao finalizar gravacao.");
        }
    }

    function gpsClearMap() {
        if (gpsUploadedLine && gpsMap) gpsMap.removeLayer(gpsUploadedLine);
        gpsUploadedLine = null;
        if (gpsTrajectoryLine) gpsTrajectoryLine.setLatLngs([]);
        gpsTrajectoryCoords = [];
        if (gpsWaypointMarker && gpsMap) gpsMap.removeLayer(gpsWaypointMarker);
        gpsWaypointMarker = null;
        fetch("/api/gps/trajectory/uploaded", { method: "DELETE" });
        byId("gpsFollowSection").style.display = "none";
        byId("gpsTrajectoryPoints").textContent = "0";
        byId("gpsUploadInput").value = "";
        addLog("Mapa limpo.");
    }

    async function gpsDownloadGPX() {
        try {
            const resp = await fetch("/api/gps/trajectory/download-gpx");
            if (!resp.ok) { addLog("Nenhum trajeto para baixar."); return; }
            const blob = await resp.blob();
            const url = URL.createObjectURL(blob);
            const a = document.createElement("a");
            a.href = url;
            const ts = new Date().toISOString().replace(/[:.]/g, "-");
            a.download = "trajeto_pandorapi_" + ts + ".gpx";
            a.click();
            URL.revokeObjectURL(url);
            addLog("GPX baixado com sucesso.");
        } catch (e) {
            addLog("Erro ao baixar GPX.");
        }
    }

    function gpsUploadGPX() {
        byId("gpsUploadInput").click();
    }

    async function gpsHandleUpload(file) {
        if (!file) return;
        if (!file.name.endsWith(".gpx")) { addLog("Apenas arquivos .gpx sao aceitos."); return; }
        const formData = new FormData();
        formData.append("file", file);
        try {
            const resp = await fetch("/api/gps/trajectory/upload", { method: "POST", body: formData });
            const result = await resp.json();
            if (result.ok) {
                addLog("GPX carregado: " + result.point_count + " pontos.");
                byId("gpsFollowInfo").textContent = "Trajeto carregado: " + result.point_count + " pontos";
                byId("gpsFollowSection").style.display = "block";
                const ptsResp = await fetch("/api/gps/trajectory/uploaded");
                const ptsData = await ptsResp.json();
                if (gpsMap && ptsData.points.length > 0) {
                    const coords = ptsData.points.filter(p => p.lat != null && p.lng != null).map(p => L.latLng(p.lat, p.lng));
                    if (gpsUploadedLine) gpsMap.removeLayer(gpsUploadedLine);
                    gpsUploadedLine = L.polyline(coords, { color: "#ef4444", weight: 3, opacity: 0.7 }).addTo(gpsMap);
                    if (coords.length > 0) gpsMap.fitBounds(gpsUploadedLine.getBounds().pad(0.1));
                }
            } else {
                addLog("Erro: " + (result.error || "Falha no upload"));
            }
            byId("gpsUploadInput").value = "";
        } catch (e) {
            addLog("Erro ao enviar GPX.");
        }
    }

    async function gpsSaveFollowConfig() {
        const safe = parseInt(byId("gpsSafeDist").value) || 50;
        const crit = parseInt(byId("gpsCritDist").value) || 30;
        try {
            await fetch("/api/gps/follow/config", {
                method: "PUT",
                headers: { "Content-Type": "application/json" },
                body: JSON.stringify({ safe_distance_mm: safe * 10, critical_distance_mm: crit * 10 })
            });
        } catch (e) {}
    }

    async function gpsToggleAvoidance() {
        const enabled = byId("gpsAvoidance").checked;
        try {
            await fetch("/api/gps/follow/config", {
                method: "PUT",
                headers: { "Content-Type": "application/json" },
                body: JSON.stringify({ avoidance_enabled: enabled })
            });
        } catch (e) {}
    }

    async function gpsLoadFollowConfig() {
        try {
            const resp = await fetch("/api/gps/follow/config");
            const cfg = await resp.json();
            byId("gpsSafeDist").value = Math.round((cfg.safe_distance_mm || 500) / 10);
            byId("gpsCritDist").value = Math.round((cfg.critical_distance_mm || 300) / 10);
            byId("gpsAvoidance").checked = cfg.avoidance_enabled !== false;
        } catch (e) {}
    }

    async function gpsFollowStart() {
        try {
            const resp = await fetch("/api/gps/follow/start", { method: "POST" });
            const result = await resp.json();
            addLog(result.message || result.error || "");
        } catch (e) {
            addLog("Erro ao iniciar follow.");
        }
    }

    async function gpsFollowStop() {
        try {
            const resp = await fetch("/api/gps/follow/stop", { method: "POST" });
            const result = await resp.json();
            addLog(result.message || result.error || "");
        } catch (e) {
            addLog("Erro ao parar follow.");
        }
    }

    async function gpsRestoreUploadedTrajectory() {
        try {
            const resp = await fetch("/api/gps/trajectory/uploaded");
            const data = await resp.json();
            if (data.loaded && data.points && data.points.length > 0) {
                byId("gpsFollowInfo").textContent = "Trajeto carregado: " + data.point_count + " pontos";
                byId("gpsFollowSection").style.display = "block";
                if (gpsMap) {
                    const coords = data.points.filter(p => p.lat != null && p.lng != null).map(p => L.latLng(p.lat, p.lng));
                    if (gpsUploadedLine) gpsMap.removeLayer(gpsUploadedLine);
                    gpsUploadedLine = L.polyline(coords, { color: "#ef4444", weight: 3, opacity: 0.7 }).addTo(gpsMap);
                }
            }
        } catch (e) {}
    }

    async function depthLoadConfig() {
        try {
            const resp = await fetch("/api/depth/config");
            const cfg = await resp.json();
            byId("depthMinMm").value = cfg.min_depth_mm || 500;
            byId("depthMaxMm").value = cfg.max_depth_mm || 8000;
        } catch (e) {}
    }

    async function depthSaveConfig() {
        const minMm = parseInt(byId("depthMinMm").value) || 500;
        const maxMm = parseInt(byId("depthMaxMm").value) || 8000;
        try {
            await fetch("/api/depth/config", {
                method: "POST",
                headers: { "Content-Type": "application/json" },
                body: JSON.stringify({ min_depth_mm: minMm, max_depth_mm: maxMm })
            });
        } catch (e) {}
    }

    gpsLoadConfig();
    gpsInitMap();
    gpsRestoreUploadedTrajectory();
    gpsLoadFollowConfig();
    depthLoadConfig();
</script>
</body>
</html>
"""


def load_config() -> Dict[str, Any]:
    if not os.path.exists(CONFIG_FILE):
        save_config(DEFAULT_CONFIG)
        return dict(DEFAULT_CONFIG)

    try:
        with open(CONFIG_FILE, "r", encoding="utf-8") as file:
            data = json.load(file)

        config = dict(DEFAULT_CONFIG)
        config.update(data)

        if not isinstance(config.get("mappings"), dict):
            config["mappings"] = {}

        return config

    except Exception:
        save_config(DEFAULT_CONFIG)
        return dict(DEFAULT_CONFIG)


def save_config(config: Dict[str, Any]) -> None:
    clean_config = dict(DEFAULT_CONFIG)
    clean_config.update(config)

    with open(CONFIG_FILE, "w", encoding="utf-8") as file:
        json.dump(clean_config, file, indent=2, ensure_ascii=False)


def ev_name(mapping: Dict[int, Any], code: int) -> str:
    try:
        if isinstance(mapping, dict):
            name = mapping.get(code)

            if isinstance(name, list):
                preferred = [
                    "BTN_SOUTH",
                    "BTN_EAST",
                    "BTN_NORTH",
                    "BTN_WEST",
                    "BTN_TL",
                    "BTN_TR",
                    "BTN_TL2",
                    "BTN_TR2",
                    "BTN_SELECT",
                    "BTN_START",
                    "BTN_MODE",
                    "BTN_THUMBL",
                    "BTN_THUMBR",
                    "ABS_X",
                    "ABS_Y",
                    "ABS_RX",
                    "ABS_RY",
                    "ABS_Z",
                    "ABS_RZ",
                    "ABS_HAT0X",
                    "ABS_HAT0Y"
                ]

                for item in preferred:
                    if item in name:
                        return item

                return str(name[0])

            if name is not None:
                return str(name)

        if isinstance(code, int) and code in CODE_FALLBACK_NAMES:
            return CODE_FALLBACK_NAMES[code]

        if EVDEV_AVAILABLE and ecodes is not None:
            for group_name in ["KEY", "BTN", "ABS", "REL", "SW"]:
                group = getattr(ecodes, group_name, {})

                if isinstance(group, dict):
                    name = group.get(code)

                    if isinstance(name, list) and name:
                        return str(name[0])

                    if name is not None:
                        return str(name)

    except Exception:
        pass

    return str(code)


def run_system_command(args: List[str], timeout: int = 10) -> Dict[str, Any]:
    try:
        result = subprocess.run(
            args,
            capture_output=True,
            text=True,
            timeout=timeout
        )

        return {
            "ok": result.returncode == 0,
            "cmd": " ".join(args),
            "stdout": result.stdout,
            "stderr": result.stderr,
            "returncode": result.returncode
        }

    except subprocess.TimeoutExpired as error:
        stdout = error.stdout or ""
        stderr = error.stderr or ""

        if isinstance(stdout, bytes):
            stdout = stdout.decode(errors="ignore")

        if isinstance(stderr, bytes):
            stderr = stderr.decode(errors="ignore")

        return {
            "ok": False,
            "cmd": " ".join(args),
            "stdout": stdout,
            "stderr": stderr + "\nTimeout executando comando",
            "returncode": -1
        }

    except Exception as error:
        return {
            "ok": False,
            "cmd": " ".join(args),
            "stdout": "",
            "stderr": str(error),
            "returncode": -1
        }


def is_module_loaded(module_name: str) -> bool:
    try:
        with open("/proc/modules", "r", encoding="utf-8") as file:
            modules = file.read()

        return re.search(rf"^{re.escape(module_name)}\s+", modules, re.MULTILINE) is not None

    except Exception:
        return False


def module_exists(module_name: str) -> bool:
    result = run_system_command(["modinfo", module_name], timeout=5)
    return result["ok"]


def try_modprobe(module_name: str) -> Dict[str, Any]:
    if is_module_loaded(module_name):
        return {
            "ok": True,
            "module": module_name,
            "loaded": True,
            "already_loaded": True,
            "result": None
        }

    direct_result = run_system_command(["modprobe", module_name], timeout=10)

    if direct_result["ok"]:
        return {
            "ok": True,
            "module": module_name,
            "loaded": is_module_loaded(module_name),
            "already_loaded": False,
            "result": direct_result
        }

    sudo_result = run_system_command(["sudo", "-n", "modprobe", module_name], timeout=10)

    return {
        "ok": sudo_result["ok"],
        "module": module_name,
        "loaded": is_module_loaded(module_name),
        "already_loaded": False,
        "result": sudo_result,
        "direct_result": direct_result,
        "hint": "Se falhar, rode manualmente: sudo modprobe " + module_name
    }


def load_hid_modules() -> Dict[str, Any]:
    hidp_result = try_modprobe("hidp")

    hid_nintendo_exists = module_exists("hid-nintendo")

    if hid_nintendo_exists:
        hid_nintendo_result = try_modprobe("hid-nintendo")
    else:
        hid_nintendo_result = {
            "ok": False,
            "module": "hid-nintendo",
            "exists": False,
            "loaded": False,
            "hint": "Módulo hid-nintendo não existe neste kernel. Atualize o kernel ou teste outro controle HID genérico."
        }

    return {
        "ok": hidp_result.get("loaded", False),
        "hidp": hidp_result,
        "hid_nintendo": hid_nintendo_result,
        "modules_loaded_now": {
            "hidp": is_module_loaded("hidp"),
            "hid_nintendo": is_module_loaded("hid-nintendo")
        }
    }


def read_proc_input_devices() -> str:
    try:
        with open("/proc/bus/input/devices", "r", encoding="utf-8", errors="ignore") as file:
            return file.read()
    except Exception as error:
        return f"Erro lendo /proc/bus/input/devices: {error}"


def list_dev_input_files() -> List[Dict[str, Any]]:
    items = []

    try:
        base = "/dev/input"

        if not os.path.exists(base):
            return items

        for name in sorted(os.listdir(base)):
            path = os.path.join(base, name)

            try:
                stat_info = os.stat(path)

                items.append({
                    "name": name,
                    "path": path,
                    "mode": oct(stat_info.st_mode),
                    "uid": stat_info.st_uid,
                    "gid": stat_info.st_gid
                })

            except Exception as error:
                items.append({
                    "name": name,
                    "path": path,
                    "error": str(error)
                })

    except Exception:
        pass

    return items


def is_probably_gamepad(device: Any) -> bool:
    if not EVDEV_AVAILABLE:
        return False

    try:
        name = (device.name or "").lower()

        if "controller" in name:
            return True

        if "gamepad" in name:
            return True

        if "joystick" in name:
            return True

        if "pro controller" in name:
            return True

        capabilities = device.capabilities()

        has_keys = ecodes.EV_KEY in capabilities
        has_abs = ecodes.EV_ABS in capabilities

        if not has_keys:
            return False

        keys = capabilities.get(ecodes.EV_KEY, [])
        axes = capabilities.get(ecodes.EV_ABS, [])

        gamepad_buttons = {
            ecodes.BTN_SOUTH,
            ecodes.BTN_EAST,
            ecodes.BTN_NORTH,
            ecodes.BTN_WEST,
            ecodes.BTN_TL,
            ecodes.BTN_TR,
            ecodes.BTN_SELECT,
            ecodes.BTN_START,
            ecodes.BTN_MODE,
            ecodes.BTN_THUMBL,
            ecodes.BTN_THUMBR,
            ecodes.BTN_A,
            ecodes.BTN_B,
            ecodes.BTN_X,
            ecodes.BTN_Y
        }

        gamepad_axes = {
            ecodes.ABS_X,
            ecodes.ABS_Y,
            ecodes.ABS_RX,
            ecodes.ABS_RY,
            ecodes.ABS_Z,
            ecodes.ABS_RZ,
            ecodes.ABS_HAT0X,
            ecodes.ABS_HAT0Y
        }

        has_gamepad_button = any(code in keys for code in gamepad_buttons)
        has_gamepad_axis = any(code in axes for code in gamepad_axes)

        if has_abs:
            return has_gamepad_button or has_gamepad_axis

        return has_gamepad_button

    except Exception:
        return False


def list_input_devices() -> List[Dict[str, Any]]:
    if not EVDEV_AVAILABLE:
        return []

    devices = []

    for path in list_devices():
        try:
            device = InputDevice(path)
            capabilities = device.capabilities()

            has_keys = ecodes.EV_KEY in capabilities
            has_axes = ecodes.EV_ABS in capabilities

            devices.append({
                "path": path,
                "name": device.name or "Sem nome",
                "physical": device.phys,
                "uniq": device.uniq,
                "has_keys": has_keys,
                "has_axes": has_axes,
                "is_gamepad": is_probably_gamepad(device)
            })

        except Exception as error:
            devices.append({
                "path": path,
                "name": "Erro ao abrir",
                "physical": "",
                "uniq": "",
                "has_keys": False,
                "has_axes": False,
                "is_gamepad": False,
                "error": str(error)
            })

    devices.sort(key=lambda item: (not item["is_gamepad"], item["name"], item["path"]))
    return devices


def input_diagnostics() -> Dict[str, Any]:
    proc_devices = read_proc_input_devices()
    evdev_devices = list_input_devices() if EVDEV_AVAILABLE else []

    return {
        "evdev_available": EVDEV_AVAILABLE,
        "evdev_import_error": EVDEV_IMPORT_ERROR,
        "dev_input_files": list_dev_input_files(),
        "evdev_devices": evdev_devices,
        "proc_bus_input_devices": proc_devices,
        "pro_controller_seen_in_proc": "Pro Controller" in proc_devices,
        "modules": {
            "hidp_loaded": is_module_loaded("hidp"),
            "hid_nintendo_loaded": is_module_loaded("hid-nintendo"),
            "hid_nintendo_exists": module_exists("hid-nintendo")
        },
        "hint": "Se Bluetooth está conectado mas não há Pro Controller em /proc/bus/input/devices, o kernel não criou o HID input."
    }


def wait_for_hid_device(name_hint: str = "Pro Controller", timeout_seconds: int = 12) -> Dict[str, Any]:
    started = time.time()

    while time.time() - started < timeout_seconds:
        devices = list_input_devices() if EVDEV_AVAILABLE else []
        proc_devices = read_proc_input_devices()

        for device in devices:
            device_name = str(device.get("name", ""))

            if name_hint.lower() in device_name.lower():
                return {
                    "ok": True,
                    "found": True,
                    "device": device,
                    "devices": devices,
                    "elapsed": round(time.time() - started, 2)
                }

        if name_hint.lower() in proc_devices.lower():
            return {
                "ok": True,
                "found": True,
                "device": None,
                "devices": devices,
                "proc_contains_name": True,
                "elapsed": round(time.time() - started, 2)
            }

        time.sleep(1)

    return {
        "ok": False,
        "found": False,
        "devices": list_input_devices() if EVDEV_AVAILABLE else [],
        "dev_input_files": list_dev_input_files(),
        "proc_bus_input_devices": read_proc_input_devices(),
        "elapsed": round(time.time() - started, 2),
        "hint": "Bluetooth conectou, mas o Linux não criou /dev/input/eventX para o controle."
    }


def open_device_by_path(path: str) -> Optional[Any]:
    if not EVDEV_AVAILABLE:
        return None

    if not path:
        return None

    if not os.path.exists(path):
        return None

    try:
        device = InputDevice(path)

        if is_probably_gamepad(device):
            return device

        capabilities = device.capabilities()

        if ecodes.EV_KEY in capabilities or ecodes.EV_ABS in capabilities:
            return device

    except Exception:
        return None

    return None


def find_gamepad() -> Optional[Any]:
    if not EVDEV_AVAILABLE:
        return None

    config = load_config()

    fixed_path = str(config.get("device_path", "")).strip()
    name_filter = str(config.get("device_name_contains", "")).strip().lower()

    fixed_device = open_device_by_path(fixed_path)

    if fixed_device is not None:
        return fixed_device

    candidates = []

    for path in list_devices():
        try:
            device = InputDevice(path)

            if name_filter:
                if name_filter in (device.name or "").lower():
                    candidates.append(device)
            else:
                if is_probably_gamepad(device):
                    candidates.append(device)

        except Exception:
            pass

    if candidates:
        candidates.sort(key=lambda dev: (not is_probably_gamepad(dev), dev.name or "", dev.path))
        return candidates[0]

    return None


def get_abs_infos(device: Any) -> Dict[str, Any]:
    abs_infos = {}

    if not EVDEV_AVAILABLE:
        return abs_infos

    try:
        capabilities = device.capabilities(absinfo=True)
        abs_list = capabilities.get(ecodes.EV_ABS, [])

        for code, abs_info in abs_list:
            name = ev_name(ecodes.ABS, code)
            abs_infos[name] = abs_info

    except Exception:
        pass

    return abs_infos


def normalize_axis(value: int, abs_info: Any, deadzone: float) -> float:
    if abs_info is None:
        return float(value)

    minimum = abs_info.min
    maximum = abs_info.max

    if maximum == minimum:
        return 0.0

    normalized = ((value - minimum) / (maximum - minimum)) * 2.0 - 1.0

    if abs(normalized) < deadzone:
        normalized = 0.0

    if normalized > 1.0:
        normalized = 1.0

    if normalized < -1.0:
        normalized = -1.0

    return round(normalized, 4)


def set_state(**kwargs: Any) -> None:
    with state_lock:
        for key, value in kwargs.items():
            current_state[key] = value


def emit_status() -> None:
    with state_lock:
        payload = dict(current_state)

    socketio.emit("gamepad_status", payload)


def emit_event(payload: Dict[str, Any]) -> None:
    socketio.emit("gamepad_event", payload)


def clear_runtime_state() -> None:
    with state_lock:
        current_state["buttons"] = {}
        current_state["axes"] = {}
        current_state["last_event"] = None


def handle_key_event(event: Any, mappings: Dict[str, str]) -> None:
    code = ev_name(ecodes.KEY, event.code)
    action = mappings.get(code, FRIENDLY_NAMES.get(code, code))

    payload = {
        "kind": "button",
        "code": code,
        "action": action,
        "value": event.value,
        "pressed": event.value == 1,
        "released": event.value == 0,
        "held": event.value == 2,
        "timestamp": time.time()
    }

    with state_lock:
        current_state["buttons"][code] = event.value
        current_state["last_event"] = payload

    emit_event(payload)

    if code == get_can_config().get("deadman_button", "BTN_TR"):
        robot_update_from_gamepad(force=True)

    brake_button = get_can_config().get("brake_button", "BTN_SOUTH")
    if code == brake_button:
        if event.value in [1, 2]:
            brake_current = float(get_can_config().get("brake_current", 8.0))
            with robot_lock:
                robot_state["brake_active"] = True
            robot_brake(brake_current, force=True)
        elif event.value == 0:
            with robot_lock:
                robot_state["brake_active"] = False
            robot_brake(0.0, force=True)


def handle_abs_event(event: Any, mappings: Dict[str, str], abs_infos: Dict[str, Any], deadzone: float) -> None:
    code = ev_name(ecodes.ABS, event.code)
    action = mappings.get(code, FRIENDLY_NAMES.get(code, code))
    normalized = normalize_axis(event.value, abs_infos.get(code), deadzone)

    payload = {
        "kind": "axis",
        "code": code,
        "action": action,
        "value": event.value,
        "normalized": normalized,
        "timestamp": time.time()
    }

    with state_lock:
        current_state["axes"][code] = normalized
        current_state["last_event"] = payload

    emit_event(payload)

    if code in [
        get_can_config().get("throttle_axis", "ABS_Y"),
        get_can_config().get("steering_axis", "ABS_X"),
        "ABS_Z",
        "ABS_RZ"
    ]:
        robot_update_from_gamepad(force=False)


def gamepad_reader_loop() -> None:
    while True:
        reader_restart_event.clear()

        if not EVDEV_AVAILABLE:
            set_state(
                connected=False,
                device_path=None,
                device_name=None,
                error="Biblioteca evdev não disponível. Instale com: sudo apt install python3-evdev ou pip install evdev",
                evdev_available=False,
                evdev_import_error=EVDEV_IMPORT_ERROR
            )

            emit_status()
            time.sleep(3)
            continue

        device = find_gamepad()

        if device is None:
            set_state(
                connected=False,
                device_path=None,
                device_name=None,
                error="Nenhum gamepad HID encontrado. Bluetooth pode estar conectado, mas falta /dev/input/eventX."
            )

            emit_status()
            time.sleep(2)
            continue

        clear_runtime_state()

        set_state(
            connected=True,
            device_path=device.path,
            device_name=device.name,
            error=None
        )

        emit_status()

        abs_infos = get_abs_infos(device)

        try:
            for event in device.read_loop():
                if reader_restart_event.is_set():
                    break

                config = load_config()
                mappings = config.get("mappings", {})
                deadzone = float(config.get("deadzone", 0.05))

                if event.type == ecodes.EV_KEY:
                    handle_key_event(event, mappings)

                elif event.type == ecodes.EV_ABS:
                    handle_abs_event(event, mappings, abs_infos, deadzone)

        except PermissionError as error:
            set_state(
                connected=False,
                error=f"Sem permissão para ler {getattr(device, 'path', '/dev/input/eventX')}: {error}"
            )

            emit_status()
            time.sleep(3)

        except OSError as error:
            set_state(
                connected=False,
                error=f"Dispositivo desconectado ou inacessível: {error}"
            )

            emit_status()
            time.sleep(1)

        except Exception as error:
            payload = {
                "kind": "error",
                "message": str(error),
                "timestamp": time.time()
            }

            with state_lock:
                current_state["connected"] = False
                current_state["error"] = str(error)
                current_state["last_event"] = payload

            emit_event(payload)
            emit_status()
            time.sleep(2)


def is_valid_mac(mac: str) -> bool:
    return bool(MAC_RE.fullmatch(str(mac).strip()))


def run_bluetoothctl(args: List[str], timeout: int = 20) -> Dict[str, Any]:
    try:
        result = subprocess.run(
            ["bluetoothctl"] + args,
            capture_output=True,
            text=True,
            timeout=timeout
        )

        return {
            "ok": result.returncode == 0,
            "cmd": "bluetoothctl " + " ".join(args),
            "stdout": result.stdout,
            "stderr": result.stderr,
            "returncode": result.returncode
        }

    except subprocess.TimeoutExpired as error:
        stdout = error.stdout or ""
        stderr = error.stderr or ""

        if isinstance(stdout, bytes):
            stdout = stdout.decode(errors="ignore")

        if isinstance(stderr, bytes):
            stderr = stderr.decode(errors="ignore")

        return {
            "ok": False,
            "cmd": "bluetoothctl " + " ".join(args),
            "stdout": stdout,
            "stderr": stderr + "\nTimeout executando bluetoothctl",
            "returncode": -1
        }

    except Exception as error:
        return {
            "ok": False,
            "cmd": "bluetoothctl " + " ".join(args),
            "stdout": "",
            "stderr": str(error),
            "returncode": -1
        }


def analyze_bluetooth_output(output: str) -> Dict[str, Any]:
    text = output or ""

    connected = False
    paired = False
    trusted = False

    if "Connection successful" in text:
        connected = True

    if "Pairing successful" in text:
        paired = True

    if "trust succeeded" in text:
        trusted = True

    if re.search(r"Connected:\s+yes", text, re.IGNORECASE):
        connected = True

    if re.search(r"Paired:\s+yes", text, re.IGNORECASE):
        paired = True

    if re.search(r"Trusted:\s+yes", text, re.IGNORECASE):
        trusted = True

    no_controller = "No default controller available" in text

    auth_failed = (
        "AuthenticationFailed" in text
        or "AuthenticationCanceled" in text
        or "AuthenticationRejected" in text
        or "Failed to pair" in text
    )

    failed_to_pair = "Failed to pair" in text
    failed_to_connect = "Failed to connect" in text

    already_exists = (
        "AlreadyExists" in text
        or "Already Exists" in text
        or "already exists" in text
    )

    return {
        "connected": connected,
        "paired": paired,
        "trusted": trusted,
        "no_controller": no_controller,
        "auth_failed": auth_failed,
        "failed_to_pair": failed_to_pair,
        "failed_to_connect": failed_to_connect,
        "already_exists": already_exists,
        "raw_error": ""
    }


def run_bluetoothctl_interactive(steps: List[Any], timeout: int = 80) -> Dict[str, Any]:
    started_at = time.time()
    sent_commands = []

    try:
        process = subprocess.Popen(
            ["bluetoothctl"],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1
        )

        for item in steps:
            if isinstance(item, tuple):
                command, delay = item
            else:
                command, delay = item, 0.8

            if process.poll() is not None:
                break

            if process.stdin:
                process.stdin.write(command + "\n")
                process.stdin.flush()

            sent_commands.append(command)
            time.sleep(float(delay))

            if time.time() - started_at > timeout:
                try:
                    process.kill()
                except Exception:
                    pass

                stdout, stderr = process.communicate(timeout=5)
                output = (stdout or "") + "\n" + (stderr or "")

                return {
                    "ok": False,
                    "cmd": "bluetoothctl interactive",
                    "sent_commands": sent_commands,
                    "stdout": stdout or "",
                    "stderr": (stderr or "") + "\nTimeout executando bluetoothctl interativo",
                    "returncode": -1,
                    "analysis": analyze_bluetooth_output(output)
                }

        if process.stdin:
            process.stdin.write("exit\n")
            process.stdin.flush()

        stdout, stderr = process.communicate(timeout=10)

        output = (stdout or "") + "\n" + (stderr or "")
        analysis = analyze_bluetooth_output(output)

        return {
            "ok": process.returncode == 0 and not analysis["no_controller"],
            "cmd": "bluetoothctl interactive",
            "sent_commands": sent_commands,
            "stdout": stdout or "",
            "stderr": stderr or "",
            "returncode": process.returncode,
            "analysis": analysis
        }

    except Exception as error:
        return {
            "ok": False,
            "cmd": "bluetoothctl interactive",
            "sent_commands": sent_commands,
            "stdout": "",
            "stderr": str(error),
            "returncode": -1,
            "analysis": {
                "connected": False,
                "paired": False,
                "trusted": False,
                "no_controller": False,
                "auth_failed": False,
                "failed_to_pair": False,
                "failed_to_connect": False,
                "already_exists": False,
                "raw_error": str(error)
            }
        }


def parse_bluetooth_devices(text: str) -> List[Dict[str, str]]:
    devices = {}
    lines = text.splitlines()

    for line in lines:
        line = line.strip()

        match = re.search(
            r"(?:Device|\[NEW\]\s+Device|\[CHG\]\s+Device)\s+(([0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2})\s+(.+)",
            line
        )

        if match:
            mac = match.group(1).upper()
            name = match.group(3).strip()

            if name:
                devices[mac] = {
                    "mac": mac,
                    "name": name
                }

    return list(devices.values())


def bluetooth_prepare() -> Dict[str, Any]:
    module_result = load_hid_modules()

    result = run_bluetoothctl_interactive([
        ("power on", 1.0),
        ("agent off", 0.5),
        ("agent NoInputNoOutput", 1.0),
        ("default-agent", 1.0),
        ("pairable on", 0.8),
        ("discoverable on", 0.8),
        ("show", 1.0)
    ], timeout=25)

    return {
        "ok": result.get("ok", False),
        "modules": module_result,
        "result": result
    }


def bluetooth_scan(duration: int = 8) -> Dict[str, Any]:
    try:
        duration = int(duration)
    except Exception:
        duration = 8

    if duration < 3:
        duration = 3

    if duration > 30:
        duration = 30

    module_result = load_hid_modules()

    result = run_bluetoothctl_interactive([
        ("power on", 1.0),
        ("agent off", 0.5),
        ("agent NoInputNoOutput", 1.0),
        ("default-agent", 1.0),
        ("pairable on", 0.8),
        ("discoverable on", 0.8),
        ("scan on", duration),
        ("devices", 1.0),
        ("scan off", 1.0)
    ], timeout=duration + 25)

    combined_output = ""
    combined_output += result.get("stdout", "")
    combined_output += "\n"
    combined_output += result.get("stderr", "")

    devices_result = run_bluetoothctl(["devices"], timeout=10)

    combined_output += "\n"
    combined_output += devices_result.get("stdout", "")

    devices = parse_bluetooth_devices(combined_output)

    return {
        "ok": result.get("ok", False),
        "duration": duration,
        "modules": module_result,
        "scan_result": result,
        "devices_command": devices_result,
        "devices": devices
    }


def bluetooth_scan_on() -> Dict[str, Any]:
    module_result = load_hid_modules()

    result = run_bluetoothctl_interactive([
        ("power on", 1.0),
        ("agent off", 0.5),
        ("agent NoInputNoOutput", 1.0),
        ("default-agent", 1.0),
        ("pairable on", 0.8),
        ("discoverable on", 0.8),
        ("scan on", 1.0)
    ], timeout=15)

    return {
        "ok": result.get("ok", False),
        "modules": module_result,
        "result": result
    }


def bluetooth_scan_off() -> Dict[str, Any]:
    result = run_bluetoothctl_interactive([
        ("scan off", 1.0)
    ], timeout=10)

    return {
        "ok": result.get("ok", False),
        "result": result
    }


def bluetooth_pair_trust_connect(mac: str) -> Dict[str, Any]:
    mac = str(mac).strip().upper()

    if not is_valid_mac(mac):
        return {
            "ok": False,
            "error": "MAC inválido"
        }

    module_result = load_hid_modules()

    result = run_bluetoothctl_interactive([
        ("power on", 1.0),
        ("agent off", 0.5),
        ("agent NoInputNoOutput", 1.0),
        ("default-agent", 1.0),
        ("pairable on", 1.0),
        ("discoverable on", 1.0),
        ("scan on", 8.0),
        (f"pair {mac}", 10.0),
        (f"trust {mac}", 2.0),
        (f"connect {mac}", 10.0),
        (f"info {mac}", 2.0),
        ("scan off", 1.0)
    ], timeout=85)

    analysis = result.get("analysis", {})
    bluetooth_ok = bool(analysis.get("connected")) and not bool(analysis.get("no_controller"))
    hid_wait = wait_for_hid_device("Pro Controller", timeout_seconds=12)
    final_ok = bluetooth_ok and hid_wait.get("found", False)

    return {
        "ok": final_ok,
        "bluetooth_ok": bluetooth_ok,
        "hid_ok": hid_wait.get("found", False),
        "mac": mac,
        "connected": analysis.get("connected", False),
        "paired": analysis.get("paired", False),
        "trusted": analysis.get("trusted", False),
        "analysis": analysis,
        "modules": module_result,
        "hid_wait": hid_wait,
        "diagnostics": input_diagnostics(),
        "result": result
    }


def bluetooth_connect(mac: str) -> Dict[str, Any]:
    mac = str(mac).strip().upper()

    if not is_valid_mac(mac):
        return {
            "ok": False,
            "error": "MAC inválido"
        }

    module_result = load_hid_modules()

    result = run_bluetoothctl_interactive([
        ("power on", 1.0),
        ("agent off", 0.5),
        ("agent NoInputNoOutput", 1.0),
        ("default-agent", 1.0),
        (f"trust {mac}", 1.5),
        (f"connect {mac}", 10.0),
        (f"info {mac}", 2.0)
    ], timeout=40)

    analysis = result.get("analysis", {})
    bluetooth_ok = bool(analysis.get("connected")) and not bool(analysis.get("no_controller"))
    hid_wait = wait_for_hid_device("Pro Controller", timeout_seconds=12)
    final_ok = bluetooth_ok and hid_wait.get("found", False)

    return {
        "ok": final_ok,
        "bluetooth_ok": bluetooth_ok,
        "hid_ok": hid_wait.get("found", False),
        "mac": mac,
        "connected": analysis.get("connected", False),
        "paired": analysis.get("paired", False),
        "trusted": analysis.get("trusted", False),
        "analysis": analysis,
        "modules": module_result,
        "hid_wait": hid_wait,
        "diagnostics": input_diagnostics(),
        "result": result
    }


def bluetooth_disconnect(mac: str) -> Dict[str, Any]:
    mac = str(mac).strip().upper()

    if not is_valid_mac(mac):
        return {
            "ok": False,
            "error": "MAC inválido"
        }

    result = run_bluetoothctl_interactive([
        (f"disconnect {mac}", 2.0),
        (f"info {mac}", 1.0)
    ], timeout=15)

    return {
        "ok": result.get("ok", False),
        "mac": mac,
        "result": result,
        "diagnostics": input_diagnostics()
    }


def bluetooth_remove(mac: str) -> Dict[str, Any]:
    mac = str(mac).strip().upper()

    if not is_valid_mac(mac):
        return {
            "ok": False,
            "error": "MAC inválido"
        }

    result = run_bluetoothctl_interactive([
        (f"disconnect {mac}", 1.5),
        (f"remove {mac}", 2.0),
        ("devices", 1.0)
    ], timeout=15)

    return {
        "ok": result.get("ok", False),
        "mac": mac,
        "result": result,
        "diagnostics": input_diagnostics()
    }



def get_can_config() -> Dict[str, Any]:
    config = load_config()
    can_config = dict(DEFAULT_CONFIG["can"])

    existing = config.get("can", {})
    if isinstance(existing, dict):
        can_config.update(existing)

    try:
        can_config["bitrate"] = int(can_config.get("bitrate", 500000))
    except Exception:
        can_config["bitrate"] = 500000

    try:
        can_config["left_id"] = max(0, min(255, int(can_config.get("left_id", 1))))
    except Exception:
        can_config["left_id"] = 1

    try:
        can_config["right_id"] = max(0, min(255, int(can_config.get("right_id", 2))))
    except Exception:
        can_config["right_id"] = 2

    try:
        can_config["max_duty"] = max(0.01, min(0.95, float(can_config.get("max_duty", 0.25))))
    except Exception:
        can_config["max_duty"] = 0.25

    try:
        can_config["steering_gain"] = max(0.0, min(1.0, float(can_config.get("steering_gain", 0.65))))
    except Exception:
        can_config["steering_gain"] = 0.65

    try:
        can_config["send_interval"] = max(0.02, min(0.5, float(can_config.get("send_interval", 0.05))))
    except Exception:
        can_config["send_interval"] = 0.05

    can_config["interface"] = str(can_config.get("interface", "can0")).strip() or "can0"
    can_config["deadman_button"] = str(can_config.get("deadman_button", "BTN_TR")).strip() or "BTN_TR"
    can_config["brake_button"] = str(can_config.get("brake_button", "BTN_SOUTH")).strip() or "BTN_SOUTH"
    try:
        can_config["brake_current"] = max(0.0, min(200.0, float(can_config.get("brake_current", 8.0))))
    except Exception:
        can_config["brake_current"] = 8.0

    return can_config


def save_can_config(update: Dict[str, Any]) -> Dict[str, Any]:
    config = load_config()
    can_config = get_can_config()

    allowed = set(DEFAULT_CONFIG["can"].keys())

    for key, value in update.items():
        if key in allowed:
            can_config[key] = value

    config["can"] = can_config
    save_config(config)

    with robot_lock:
        robot_state["interface"] = can_config["interface"]

    return get_can_config()


def validate_can_interface_name(name: str) -> str:
    name = str(name or "").strip()

    if not re.fullmatch(r"[a-zA-Z0-9_.:-]{1,32}", name):
        raise ValueError("Interface CAN inválida")

    return name


def list_socketcan_interfaces() -> List[Dict[str, Any]]:
    interfaces = []

    base = "/sys/class/net"

    if not os.path.exists(base):
        return interfaces

    for name in sorted(os.listdir(base)):
        if not (name.startswith("can") or name.startswith("slcan") or name.startswith("vcan")):
            continue

        iface_path = os.path.join(base, name)

        state = "unknown"
        try:
            with open(os.path.join(iface_path, "operstate"), "r", encoding="utf-8") as file:
                state = file.read().strip()
        except Exception:
            pass

        details = run_system_command(["ip", "-details", "link", "show", name], timeout=5)

        interfaces.append({
            "name": name,
            "state": state,
            "details": details
        })

    return interfaces


def list_canable_serial_candidates() -> List[Dict[str, Any]]:
    candidates = []

    for pattern in ["/dev/ttyACM*", "/dev/ttyUSB*"]:
        for path in sorted(glob.glob(pattern)):
            info = run_system_command(["udevadm", "info", "-q", "property", "-n", path], timeout=5)

            text = (info.get("stdout", "") + "\n" + info.get("stderr", "")).lower()

            likely = (
                "canable" in text
                or "candlelight" in text
                or "can" in text
                or "stm32" in text
                or "mks" in text
            )

            candidates.append({
                "path": path,
                "likely_canable": likely,
                "udev": info
            })

    return candidates


def can_scan() -> Dict[str, Any]:
    return {
        "ok": True,
        "interfaces": list_socketcan_interfaces(),
        "serial_candidates": list_canable_serial_candidates(),
        "hint": "CANable com firmware candleLight geralmente aparece direto como can0. Com firmware slcan, configure slcand manualmente ou troque para candleLight."
    }


def can_setup_interface(interface_name: str, bitrate: int) -> Dict[str, Any]:
    try:
        interface_name = validate_can_interface_name(interface_name)
        bitrate = int(bitrate)
    except Exception as error:
        return {
            "ok": False,
            "error": str(error)
        }

    if bitrate not in [125000, 250000, 500000, 1000000]:
        return {
            "ok": False,
            "error": "Bitrate inválido. Use 125000, 250000, 500000 ou 1000000."
        }

    down_result = run_system_command(["ip", "link", "set", interface_name, "down"], timeout=8)

    setup_result = run_system_command(
        ["ip", "link", "set", interface_name, "up", "type", "can", "bitrate", str(bitrate)],
        timeout=8
    )

    if not setup_result["ok"]:
        setup_result_sudo = run_system_command(
            ["sudo", "-n", "ip", "link", "set", interface_name, "up", "type", "can", "bitrate", str(bitrate)],
            timeout=8
        )
    else:
        setup_result_sudo = None

    final_details = run_system_command(["ip", "-details", "link", "show", interface_name], timeout=5)

    ok = setup_result["ok"] or (setup_result_sudo is not None and setup_result_sudo["ok"])

    with robot_lock:
        robot_state["can_ready"] = ok
        robot_state["interface"] = interface_name
        robot_state["last_error"] = None if ok else "Falha ao ativar interface CAN"

    emit_can_status()

    return {
        "ok": ok,
        "interface": interface_name,
        "bitrate": bitrate,
        "down_result": down_result,
        "setup_result": setup_result,
        "setup_result_sudo": setup_result_sudo,
        "details": final_details,
        "hint": "Se sudo -n falhar, rode o Python com sudo ou configure permissão sudo para ip link."
    }


def build_vesc_ext_id(command_id: int, vesc_id: int) -> int:
    command_id = int(command_id) & 0xFF
    vesc_id = int(vesc_id) & 0xFF
    return (command_id << 8) | vesc_id


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def socketcan_send_extended(interface_name: str, arbitration_id: int, data: bytes) -> Dict[str, Any]:
    try:
        interface_name = validate_can_interface_name(interface_name)
        data = bytes(data[:8])
        can_id = CAN_EFF_FLAG | (arbitration_id & 0x1FFFFFFF)
        can_dlc = len(data)
        frame = struct.pack("=IB3x8s", can_id, can_dlc, data.ljust(8, b"\x00"))

        with socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as sock:
            sock.bind((interface_name,))
            sock.send(frame)

        return {
            "ok": True,
            "interface": interface_name,
            "arbitration_id": hex(arbitration_id),
            "data_hex": data.hex(" ")
        }

    except Exception as error:
        return {
            "ok": False,
            "interface": interface_name,
            "arbitration_id": hex(arbitration_id),
            "data_hex": bytes(data).hex(" ") if data is not None else "",
            "error": str(error)
        }


def vesc_send_duty(interface_name: str, vesc_id: int, duty: float) -> Dict[str, Any]:
    duty = clamp(float(duty), -1.0, 1.0)
    scaled = int(duty * 100000.0)
    data = struct.pack(">i", scaled)
    arbitration_id = build_vesc_ext_id(CAN_PACKET_SET_DUTY, int(vesc_id))

    return socketcan_send_extended(interface_name, arbitration_id, data)


def vesc_send_current_brake(interface_name: str, vesc_id: int, current_a: float) -> Dict[str, Any]:
    current_a = max(0.0, min(200.0, float(current_a)))
    scaled = int(current_a * 1000.0)
    data = struct.pack(">i", scaled)
    arbitration_id = build_vesc_ext_id(CAN_PACKET_SET_CURRENT_BRAKE, int(vesc_id))
    return socketcan_send_extended(interface_name, arbitration_id, data)


def robot_brake(current_a: float, force: bool = False) -> Dict[str, Any]:
    can_config = get_can_config()
    now = time.time()

    with robot_lock:
        if not force:
            elapsed = now - float(robot_state.get("last_send_time", 0.0))
            if elapsed < float(can_config.get("send_interval", 0.05)):
                return {"ok": True, "skipped": True, "reason": "rate_limit"}
        robot_state["last_send_time"] = now

    interface_name = can_config["interface"]
    left_id = can_config["left_id"]
    right_id = can_config["right_id"]

    left_result = vesc_send_current_brake(interface_name, left_id, current_a)
    right_result = vesc_send_current_brake(interface_name, right_id, current_a)

    ok = bool(left_result.get("ok")) and bool(right_result.get("ok"))

    with robot_lock:
        robot_state["last_tx"] = {
            "left": left_result,
            "right": right_result,
            "timestamp": now
        }
        robot_state["last_error"] = None if ok else json.dumps(robot_state["last_tx"], ensure_ascii=False)

    emit_can_status()

    return {"ok": ok, "current_a": current_a, "left": left_result, "right": right_result}


def robot_send_duty(left_duty: float, right_duty: float, force: bool = False) -> Dict[str, Any]:
    can_config = get_can_config()
    now = time.time()

    with robot_lock:
        elapsed = now - float(robot_state.get("last_send_time", 0.0))

        if not force and elapsed < float(can_config.get("send_interval", 0.05)):
            return {
                "ok": True,
                "skipped": True,
                "reason": "rate_limit"
            }

        robot_state["last_send_time"] = now

    interface_name = can_config["interface"]
    left_id = can_config["left_id"]
    right_id = can_config["right_id"]

    left_result = vesc_send_duty(interface_name, left_id, left_duty)
    right_result = vesc_send_duty(interface_name, right_id, right_duty)

    ok = bool(left_result.get("ok")) and bool(right_result.get("ok"))

    with robot_lock:
        robot_state["last_left"] = float(left_duty)
        robot_state["last_right"] = float(right_duty)
        robot_state["last_tx"] = {
            "left": left_result,
            "right": right_result,
            "timestamp": now
        }
        robot_state["last_error"] = None if ok else json.dumps(robot_state["last_tx"], ensure_ascii=False)

    emit_can_status()

    return {
        "ok": ok,
        "left_duty": left_duty,
        "right_duty": right_duty,
        "left_result": left_result,
        "right_result": right_result
    }


def robot_stop(force: bool = True) -> Dict[str, Any]:
    return robot_send_duty(0.0, 0.0, force=force)


def emit_can_status() -> None:
    with robot_lock:
        payload = dict(robot_state)

    socketio.emit("can_status", payload)


def robot_update_from_gamepad(force: bool = False) -> Dict[str, Any]:
    can_config = get_can_config()

    with robot_lock:
        armed = bool(robot_state.get("armed", False))

    if not armed:
        if force:
            return robot_stop(force=True)

        return {
            "ok": False,
            "skipped": True,
            "reason": "robot_disarmed"
        }

    with state_lock:
        axes_snapshot = dict(current_state.get("axes", {}))
        buttons_snapshot = dict(current_state.get("buttons", {}))

    deadman_button = can_config.get("deadman_button", "BTN_TR")
    require_deadman = bool(can_config.get("require_deadman", True))
    deadman_ok = True

    if require_deadman:
        deadman_ok = int(buttons_snapshot.get(deadman_button, 0)) in [1, 2]

    with robot_lock:
        robot_state["deadman_ok"] = deadman_ok

    if not deadman_ok:
        return robot_stop(force=True)

    if robot_state.get("brake_active"):
        robot_send_duty(0.0, 0.0, force=True)
        robot_brake(float(can_config.get("brake_current", 8.0)), force=True)
        return {"ok": True, "braking": True}

    throttle_axis = can_config.get("throttle_axis", "ABS_Y")
    steering_axis = can_config.get("steering_axis", "ABS_X")

    throttle = float(axes_snapshot.get(throttle_axis, 0.0))
    steering = float(axes_snapshot.get(steering_axis, 0.0))

    if bool(can_config.get("invert_throttle", True)):
        throttle *= -1.0

    if bool(can_config.get("invert_steering", False)):
        steering *= -1.0

    steering_gain = float(can_config.get("steering_gain", 0.65))
    max_duty = float(can_config.get("max_duty", 0.25))

    left = throttle + (steering * steering_gain)
    right = throttle - (steering * steering_gain)

    left = clamp(left, -1.0, 1.0) * max_duty
    right = clamp(right, -1.0, 1.0) * max_duty

    if bool(can_config.get("invert_left", False)):
        left *= -1.0

    if bool(can_config.get("invert_right", True)):
        right *= -1.0

    if abs(left) < 0.002:
        left = 0.0

    if abs(right) < 0.002:
        right = 0.0

    return robot_send_duty(left, right, force=force)


def get_lidar_config() -> Dict[str, Any]:
    config = load_config()
    lidar_config = dict(DEFAULT_CONFIG["lidar"])

    existing = config.get("lidar", {})
    if isinstance(existing, dict):
        lidar_config.update(existing)

    try:
        lidar_config["baudrate"] = int(lidar_config.get("baudrate", 230400))
    except Exception:
        lidar_config["baudrate"] = 230400

    try:
        lidar_config["min_distance"] = max(0, int(lidar_config.get("min_distance", 150)))
    except Exception:
        lidar_config["min_distance"] = 150

    try:
        lidar_config["max_distance"] = max(1000, int(lidar_config.get("max_distance", 12000)))
    except Exception:
        lidar_config["max_distance"] = 12000

    try:
        lidar_config["emit_interval"] = max(0.03, min(0.5, float(lidar_config.get("emit_interval", 0.08))))
    except Exception:
        lidar_config["emit_interval"] = 0.08

    lidar_config["port"] = str(lidar_config.get("port", "/dev/ttyUSB0")).strip() or "/dev/ttyUSB0"

    return lidar_config


def save_lidar_config(update: Dict[str, Any]) -> Dict[str, Any]:
    config = load_config()
    lidar_config = get_lidar_config()

    allowed = set(DEFAULT_CONFIG["lidar"].keys())

    for key, value in update.items():
        if key in allowed:
            lidar_config[key] = value

    config["lidar"] = lidar_config
    save_config(config)

    with lidar_lock:
        lidar_state["port"] = lidar_config["port"]

    lidar_restart_event.set()

    return get_lidar_config()


def emit_lidar_status() -> None:
    with lidar_lock:
        payload = {
            "connected": lidar_state["connected"],
            "port": lidar_state["port"],
            "scanning": lidar_state["scanning"],
            "rotation_speed": lidar_state["rotation_speed"],
            "last_count": lidar_state["last_count"],
            "timestamp": lidar_state["timestamp"],
            "error": lidar_state["error"],
            "serial_available": lidar_state.get("serial_available", False),
            "serial_import_error": lidar_state.get("serial_import_error", "")
        }

    socketio.emit("lidar_status", payload)


def emit_lidar_frame(points: List[Dict[str, Any]]) -> None:
    payload = {
        "points": points,
        "count": len(points),
        "timestamp": time.time()
    }

    socketio.emit("lidar_frame", payload)


def parse_lidar_packet(data: bytes) -> Optional[Dict[str, Any]]:
    if len(data) < 7:
        return None

    if data[0] != 0x54:
        return None

    ver_len = data[1]
    n_points = ver_len & 0x1F

    expected_len = 7 + (3 * n_points) + 2 + 2 + 1

    if len(data) < expected_len:
        return None

    packet = data[:expected_len]

    crc = 0
    for b in packet[:-1]:
        crc ^= b

    if crc != packet[-1]:
        return None

    speed = struct.unpack_from("<H", packet, 2)[0]
    start_angle_raw = struct.unpack_from("<H", packet, 4)[0]
    start_angle = start_angle_raw / 100.0

    measurements = []

    for i in range(n_points):
        offset = 6 + i * 3
        distance = struct.unpack_from("<H", packet, offset)[0]
        confidence = packet[offset + 2]
        measurements.append((distance, confidence))

    end_angle_raw = struct.unpack_from("<H", packet, 6 + 3 * n_points)[0]
    end_angle = end_angle_raw / 100.0
    timestamp_raw = struct.unpack_from("<H", packet, 6 + 3 * n_points + 2)[0]

    if n_points > 1:
        if end_angle < start_angle:
            end_angle += 360.0
        angle_step = (end_angle - start_angle) / (n_points - 1)
    else:
        angle_step = 0.0

    points = []

    for i, (distance, conf) in enumerate(measurements):
        if distance == 0:
            continue

        angle_deg = (start_angle + i * angle_step) % 360.0
        angle_rad = angle_deg * (3.141592653589793 / 180.0)

        x_mm = distance * (-1.0) * math.sin(angle_rad)
        y_mm = distance * math.cos(angle_rad)

        points.append({
            "x": round(x_mm, 1),
            "y": round(y_mm, 1),
            "distance": distance,
            "angle": round(angle_deg, 2),
            "confidence": conf
        })

    return {
        "packet_len": expected_len,
        "speed": speed,
        "start_angle": start_angle,
        "end_angle": end_angle,
        "timestamp": timestamp_raw,
        "n_points": n_points,
        "points": points
    }


def lidar_reader_loop() -> None:
    accumulated_points: Dict[int, Dict[str, Any]] = {}
    last_emit_time = 0.0

    while True:
        lidar_restart_event.clear()
        accumulated_points.clear()

        if not SERIAL_AVAILABLE:
            with lidar_lock:
                lidar_state["connected"] = False
                lidar_state["scanning"] = False
                lidar_state["error"] = "Biblioteca pyserial nao disponivel. Instale com: pip install pyserial"
                lidar_state["serial_available"] = False
                lidar_state["serial_import_error"] = SERIAL_IMPORT_ERROR

            emit_lidar_status()
            time.sleep(3)
            continue

        lidar_config = get_lidar_config()
        port = lidar_config["port"]
        baudrate = lidar_config["baudrate"]

        if not os.path.exists(port):
            with lidar_lock:
                lidar_state["connected"] = False
                lidar_state["scanning"] = False
                lidar_state["error"] = f"Porta serial {port} nao encontrada. Conecte o LiDAR USB."

            emit_lidar_status()
            time.sleep(2)
            continue

        ser = None

        try:
            ser = serial.Serial(port=port, baudrate=baudrate, timeout=0.1)
            ser.reset_input_buffer()

            with lidar_lock:
                lidar_state["connected"] = True
                lidar_state["scanning"] = True
                lidar_state["error"] = None
                lidar_state["serial_available"] = True
                lidar_state["serial_import_error"] = ""

            emit_lidar_status()

            buffer = b""

            while not lidar_restart_event.is_set():
                try:
                    chunk = ser.read(512)

                    if not chunk:
                        continue

                    buffer += chunk

                    while len(buffer) >= 47:
                        if buffer[0] != 0x54:
                            buffer = buffer[1:]
                            continue

                        ver_len_index = 1
                        if len(buffer) <= ver_len_index:
                            break

                        ver_len = buffer[ver_len_index]
                        n_points = ver_len & 0x1F

                        expected_len = 7 + (3 * n_points) + 2 + 2 + 1

                        if len(buffer) < expected_len:
                            break

                        result = parse_lidar_packet(buffer[:expected_len])

                        if result is None:
                            buffer = buffer[1:]
                            continue

                        buffer = buffer[expected_len:]

                        lidar_config = get_lidar_config()
                        min_dist = lidar_config["min_distance"]
                        max_dist = lidar_config["max_distance"]

                        rotation_speed = result["speed"]

                        for pt in result["points"]:
                            if pt["distance"] < min_dist or pt["distance"] > max_dist:
                                continue

                            angle_idx = int(pt["angle"] * 2.0 + 0.5) % 720

                            if angle_idx not in accumulated_points or pt["confidence"] > accumulated_points[angle_idx].get("confidence", 0):
                                accumulated_points[angle_idx] = pt

                        with lidar_lock:
                            lidar_state["rotation_speed"] = rotation_speed
                            lidar_state["last_count"] = len(accumulated_points)
                            lidar_state["timestamp"] = time.time()

                        now = time.time()
                        emit_interval = lidar_config.get("emit_interval", 0.08)

                        if now - last_emit_time >= emit_interval and accumulated_points:
                            pts = sorted(accumulated_points.values(), key=lambda p: p["angle"])
                            emit_lidar_frame(pts)
                            if follow_state.get("active"):
                                update_lidar_obstacle_map(pts)
                            last_emit_time = now

                            accumulated_points.clear()

                except serial.SerialException as error:
                    with lidar_lock:
                        lidar_state["connected"] = False
                        lidar_state["scanning"] = False
                        lidar_state["error"] = f"Erro serial: {error}"

                    emit_lidar_status()
                    break

                except Exception as error:
                    with lidar_lock:
                        lidar_state["error"] = str(error)

                    emit_lidar_status()

        except serial.SerialException as error:
            with lidar_lock:
                lidar_state["connected"] = False
                lidar_state["scanning"] = False
                lidar_state["error"] = f"Nao foi possivel abrir {port}: {error}"

            emit_lidar_status()
            time.sleep(3)

        except PermissionError as error:
            with lidar_lock:
                lidar_state["connected"] = False
                lidar_state["scanning"] = False
                lidar_state["error"] = f"Sem permissao para acessar {port}: {error}. Use sudo ou adicione usuario ao grupo dialout."

            emit_lidar_status()
            time.sleep(5)

        except Exception as error:
            with lidar_lock:
                lidar_state["connected"] = False
                lidar_state["scanning"] = False
                lidar_state["error"] = str(error)

            emit_lidar_status()
            time.sleep(3)

        finally:
            if ser is not None and ser.is_open:
                try:
                    ser.close()
                except Exception:
                    pass

            with lidar_lock:
                lidar_state["connected"] = False
                lidar_state["scanning"] = False

            emit_lidar_status()


def get_depth_config() -> Dict[str, Any]:
    config = load_config()
    depth_config = dict(DEFAULT_CONFIG["depth_camera"])
    existing = config.get("depth_camera", {})
    if isinstance(existing, dict):
        depth_config.update(existing)
    try:
        depth_config["min_depth_mm"] = max(100, min(10000, int(depth_config.get("min_depth_mm", 500))))
    except Exception:
        depth_config["min_depth_mm"] = 500
    try:
        depth_config["max_depth_mm"] = max(500, min(20000, int(depth_config.get("max_depth_mm", 8000))))
    except Exception:
        depth_config["max_depth_mm"] = 8000
    try:
        depth_config["emit_fps"] = max(3, min(30, int(depth_config.get("emit_fps", 10))))
    except Exception:
        depth_config["emit_fps"] = 10
    return depth_config


def save_depth_config(update: Dict[str, Any]) -> Dict[str, Any]:
    config = load_config()
    depth_config = get_depth_config()
    allowed = set(DEFAULT_CONFIG["depth_camera"].keys())
    for key, value in update.items():
        if key in allowed:
            depth_config[key] = value
    config["depth_camera"] = depth_config
    save_config(config)
    with depth_lock:
        depth_state["min_depth_mm"] = depth_config["min_depth_mm"]
        depth_state["max_depth_mm"] = depth_config["max_depth_mm"]
    depth_restart_event.set()
    return get_depth_config()


def emit_depth_status() -> None:
    with depth_lock:
        payload = {
            "connected": depth_state["connected"],
            "fps": depth_state["fps"],
            "min_depth_mm": depth_state["min_depth_mm"],
            "max_depth_mm": depth_state["max_depth_mm"],
            "width": depth_state["width"],
            "height": depth_state["height"],
            "error": depth_state["error"],
            "depth_available": depth_state.get("depth_available", False),
            "depth_import_error": depth_state.get("depth_import_error", "")
        }
    socketio.emit("depth_status", payload)


def depth_camera_loop() -> None:
    if not DEPTH_AVAILABLE:
        with depth_lock:
            depth_state["connected"] = False
            depth_state["error"] = "ob_depth nao disponivel: " + DEPTH_IMPORT_ERROR
        emit_depth_status()
        return

    while True:
        depth_restart_event.clear()

        depth_config = get_depth_config()
        min_mm = depth_config["min_depth_mm"]
        max_mm = depth_config["max_depth_mm"]
        emit_fps = depth_config["emit_fps"]

        with depth_lock:
            depth_state["min_depth_mm"] = min_mm
            depth_state["max_depth_mm"] = max_mm

        cam = None
        try:
            import numpy as np
            import cv2

            cam = DepthCamera()
            cam.start(width=640, height=480, fps=30)

            with depth_lock:
                depth_state["connected"] = True
                depth_state["error"] = None
                depth_state["width"] = cam.width
                depth_state["height"] = cam.height

            emit_depth_status()

            frame_count = 0
            fps_timer = time.time()

            while not depth_restart_event.is_set():
                frame = cam.get_frame(timeout_ms=1000)
                if frame is None:
                    continue

                clipped = np.clip(frame, min_mm, max_mm)
                norm = ((clipped - min_mm) / max(1, (max_mm - min_mm)) * 255).astype(np.uint8)
                colored = cv2.applyColorMap(norm, cv2.COLORMAP_JET)
                _, jpeg = cv2.imencode('.jpg', colored, [cv2.IMWRITE_JPEG_QUALITY, 60])
                b64 = base64.b64encode(jpeg).decode('ascii')

                socketio.emit("depth_frame", {
                    "image": "data:image/jpeg;base64," + b64,
                    "width": cam.width,
                    "height": cam.height,
                    "min_mm": min_mm,
                    "max_mm": max_mm,
                    "timestamp": time.time()
                })

                frame_count += 1
                now = time.time()
                if now - fps_timer >= 2.0:
                    with depth_lock:
                        depth_state["fps"] = round(frame_count / (now - fps_timer), 1)
                    frame_count = 0
                    fps_timer = now

                elapsed = time.time() - now
                target = 1.0 / emit_fps
                if elapsed < target:
                    time.sleep(target - elapsed)

        except Exception as e:
            with depth_lock:
                depth_state["connected"] = False
                depth_state["error"] = str(e)
            emit_depth_status()
            time.sleep(3)

        finally:
            if cam is not None:
                try:
                    cam.close()
                except Exception:
                    pass
            with depth_lock:
                depth_state["connected"] = False
            emit_depth_status()
            if not depth_restart_event.is_set():
                time.sleep(2)


def get_gps_config() -> Dict[str, Any]:
    config = load_config()
    gps_config = dict(DEFAULT_CONFIG["gps"])
    existing = config.get("gps", {})
    if isinstance(existing, dict):
        gps_config.update(existing)
    gps_config["at_port"] = str(gps_config.get("at_port", "/dev/ttyUSB1")).strip() or "/dev/ttyUSB1"
    try:
        gps_config["at_baudrate"] = int(gps_config.get("at_baudrate", 115200))
    except Exception:
        gps_config["at_baudrate"] = 115200
    try:
        gps_config["emit_interval"] = max(0.2, min(5.0, float(gps_config.get("emit_interval", 1.0))))
    except Exception:
        gps_config["emit_interval"] = 1.0
    return gps_config


def save_gps_config(update: Dict[str, Any]) -> Dict[str, Any]:
    config = load_config()
    gps_config = get_gps_config()
    allowed = set(DEFAULT_CONFIG["gps"].keys())
    for key, value in update.items():
        if key in allowed:
            gps_config[key] = value
    config["gps"] = gps_config
    save_config(config)
    with gps_lock:
        gps_state["at_port"] = gps_config["at_port"]
        gps_state["emit_interval"] = gps_config["emit_interval"]
    gps_restart_event.set()
    _gps_log("info", "Config GPS salva. Reiniciando leitura...")
    return get_gps_config()


def gps_power_on() -> Dict[str, Any]:
    if not SERIAL_AVAILABLE:
        return {"ok": False, "error": "pyserial nao disponivel"}
    gps_config = get_gps_config()
    at_port = gps_config["at_port"]
    at_baudrate = gps_config["at_baudrate"]
    if not os.path.exists(at_port):
        return {"ok": False, "error": f"Porta AT {at_port} nao encontrada"}
    try:
        ser = serial.Serial(port=at_port, baudrate=at_baudrate, timeout=1.0)
        ser.write(b"AT\r\n")
        time.sleep(0.3)
        resp = ser.read(512)
        if b"OK" not in resp:
            ser.close()
            return {"ok": False, "error": "Modem nao respondeu AT na porta " + at_port}
        ser.write(b"AT+CGNSSPWR=1\r\n")
        time.sleep(0.5)
        resp2 = ser.read(512)
        ser.close()
        if b"OK" in resp2 or b"READY" in resp2:
            with gps_lock:
                gps_state["gps_powered"] = True
            _gps_log("info", "GPS ligado: AT+CGNSSPWR=1 -> OK")
            gps_restart_event.set()
            return {"ok": True, "message": "GPS ligado com sucesso"}
        else:
            _gps_log("error", f"AT+CGNSSPWR=1 falhou: {resp2.decode(errors='ignore')}")
            return {"ok": False, "error": "Falha ao ligar GPS: " + resp2.decode(errors="ignore")}
    except Exception as e:
        _gps_log("error", f"Erro ao ligar GPS: {e}")
        return {"ok": False, "error": str(e)}


def gps_power_off() -> Dict[str, Any]:
    if not SERIAL_AVAILABLE:
        return {"ok": False, "error": "pyserial nao disponivel"}
    gps_config = get_gps_config()
    at_port = gps_config["at_port"]
    at_baudrate = gps_config["at_baudrate"]
    if not os.path.exists(at_port):
        return {"ok": False, "error": f"Porta AT {at_port} nao encontrada"}
    try:
        ser = serial.Serial(port=at_port, baudrate=at_baudrate, timeout=1.0)
        ser.write(b"AT+CGNSSPWR=0\r\n")
        time.sleep(0.5)
        resp = ser.read(512)
        ser.close()
        with gps_lock:
            gps_state["gps_powered"] = False
        _gps_log("info", "GPS desligado: AT+CGNSSPWR=0")
        return {"ok": True, "message": "GPS desligado"}
    except Exception as e:
        _gps_log("error", f"Erro ao desligar GPS: {e}")
        return {"ok": False, "error": str(e)}


def trajectory_start() -> Dict[str, Any]:
    with trajectory_lock:
        if trajectory_state["recording"]:
            return {"ok": False, "error": "Ja esta gravando"}
        trajectory_state["recording"] = True
        trajectory_state["paused"] = False
        trajectory_state["point_count"] = 0
        trajectory_state["start_time"] = time.time()
        trajectory_state["points"] = []
    _gps_log("info", "Gravacao de trajeto iniciada")
    emit_gps_trajectory_status()
    return {"ok": True, "message": "Gravacao iniciada"}


def trajectory_pause() -> Dict[str, Any]:
    with trajectory_lock:
        if not trajectory_state["recording"]:
            return {"ok": False, "error": "Nao esta gravando"}
        if trajectory_state["paused"]:
            return {"ok": False, "error": "Ja esta pausado"}
        trajectory_state["paused"] = True
    _gps_log("info", f"Gravacao pausada ({trajectory_state['point_count']} pontos)")
    emit_gps_trajectory_status()
    return {"ok": True, "message": f"Gravacao pausada ({trajectory_state['point_count']} pontos)"}


def trajectory_resume() -> Dict[str, Any]:
    with trajectory_lock:
        if not trajectory_state["recording"]:
            return {"ok": False, "error": "Nao esta gravando"}
        if not trajectory_state["paused"]:
            return {"ok": False, "error": "Nao esta pausado"}
        trajectory_state["paused"] = False
    _gps_log("info", "Gravacao retomada")
    emit_gps_trajectory_status()
    return {"ok": True, "message": "Gravacao retomada"}


def trajectory_stop() -> Dict[str, Any]:
    with trajectory_lock:
        if not trajectory_state["recording"]:
            return {"ok": False, "error": "Nao esta gravando"}
        trajectory_state["recording"] = False
        trajectory_state["paused"] = False
        count = trajectory_state["point_count"]
    _gps_log("info", f"Gravacao finalizada ({count} pontos)")
    emit_gps_trajectory_status()
    return {"ok": True, "message": f"Gravacao finalizada ({count} pontos)", "point_count": count}


def trajectory_to_json() -> Dict[str, Any]:
    with trajectory_lock:
        points = list(trajectory_state["points"])
        start_time = trajectory_state["start_time"]
        count = trajectory_state["point_count"]
    total_dist = 0.0
    for i in range(1, len(points)):
        lat1 = points[i - 1].get("lat")
        lng1 = points[i - 1].get("lng")
        lat2 = points[i].get("lat")
        lng2 = points[i].get("lng")
        if all(v is not None for v in [lat1, lng1, lat2, lng2]):
            try:
                dlat = (lat2 - lat1) * 111320.0
                dlng = (lng2 - lng1) * 111320.0 * math.cos(math.radians((lat1 + lat2) / 2))
                total_dist += math.sqrt(dlat * dlat + dlng * dlng)
            except Exception:
                pass
    total_dist_km = round(total_dist / 1000.0, 3)
    return {
        "metadata": {
            "start_time": time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime(start_time)) if start_time else None,
            "end_time": time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime()),
            "total_points": count,
            "total_distance_km": total_dist_km,
            "device": "SIMCom A7670E-MASA"
        },
        "points": points
    }


def haversine_distance(lat1: float, lng1: float, lat2: float, lng2: float) -> float:
    R = 6371000.0
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lng2 - lng1)
    a_val = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2) ** 2
    return R * 2 * math.atan2(math.sqrt(a_val), math.sqrt(1 - a_val))


def bearing_to(lat1: float, lng1: float, lat2: float, lng2: float) -> float:
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dlam = math.radians(lng2 - lng1)
    x = math.sin(dlam) * math.cos(phi2)
    y = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlam)
    bearing = math.degrees(math.atan2(x, y))
    return (bearing + 360) % 360


def angle_diff(a: float, b: float) -> float:
    diff = (a - b + 180) % 360 - 180
    if diff > 180:
        diff -= 360
    elif diff < -180:
        diff += 360
    return diff


def trajectory_to_gpx() -> str:
    with trajectory_lock:
        points = list(trajectory_state["points"])
        start_time = trajectory_state["start_time"]
    if not points:
        return ""
    start_iso = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.localtime(start_time)) if start_time else time.strftime("%Y-%m-%dT%H:%M:%SZ", time.localtime())
    lines = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        '<gpx version="1.1" creator="PandoraPi" xmlns="http://www.topografix.com/GPX/1/1">',
        '  <metadata><time>' + start_iso + '</time></metadata>',
        '  <trk>',
        '    <name>Trajeto PandoraPi</name>',
        '    <trkseg>'
    ]
    for pt in points:
        lat = pt.get("lat")
        lng = pt.get("lng")
        if lat is None or lng is None:
            continue
        lines.append('      <trkpt lat="' + str(round(lat, 7)) + '" lon="' + str(round(lng, 7)) + '">')
        alt = pt.get("alt")
        if alt is not None:
            lines.append('        <ele>' + str(round(alt, 1)) + '</ele>')
        epoch = pt.get("epoch")
        if epoch:
            pt_iso = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.localtime(epoch))
            lines.append('        <time>' + pt_iso + '</time>')
        spd = pt.get("speed_kmh")
        if spd is not None:
            lines.append('        <speed>' + str(round(spd, 2)) + '</speed>')
        hdg = pt.get("heading")
        if hdg is not None:
            lines.append('        <course>' + str(round(hdg, 1)) + '</course>')
        lines.append('      </trkpt>')
    lines.append('    </trkseg>')
    lines.append('  </trk>')
    lines.append('</gpx>')
    return "\n".join(lines)


ns = {"gpx": "http://www.topografix.com/GPX/1/1"}


def parse_gpx_xml(xml_string: str) -> Dict[str, Any]:
    try:
        root = ET.fromstring(xml_string)
    except ET.ParseError as e:
        return {"ok": False, "error": f"Erro no XML: {e}", "points": [], "point_count": 0, "filename": ""}
    tag = root.tag
    is_gpx = tag == "{http://www.topografix.com/GPX/1/1}gpx" or tag == "gpx"
    if not is_gpx:
        return {"ok": False, "error": "Arquivo nao parece ser GPX valido", "points": [], "point_count": 0, "filename": ""}
    name = ""
    name_el = root.find(".//gpx:trk/gpx:name", ns) or root.find(".//{http://www.topografix.com/GPX/1/1}trk/{http://www.topografix.com/GPX/1/1}name")
    if name_el is None:
        name_el = root.find(".//trk/name")
    if name_el is not None and name_el.text:
        name = name_el.text.strip()
    points: List[Dict[str, Any]] = []
    trkpts = root.findall(".//gpx:trkpt", ns)
    if not trkpts:
        trkpts = root.findall(".//{http://www.topografix.com/GPX/1/1}trkpt")
    if not trkpts:
        trkpts = root.findall(".//trkpt")
    for trkpt in trkpts:
        lat_s = trkpt.get("lat")
        lon_s = trkpt.get("lon")
        if lat_s is None or lon_s is None:
            continue
        try:
            lat = float(lat_s)
            lng = float(lon_s)
        except (ValueError, TypeError):
            continue
        ele_val = None
        ele_el = trkpt.find("gpx:ele", ns) or trkpt.find("{http://www.topografix.com/GPX/1/1}ele") or trkpt.find("ele")
        if ele_el is not None and ele_el.text:
            try:
                ele_val = float(ele_el.text)
            except ValueError:
                pass
        time_str = None
        time_el = trkpt.find("gpx:time", ns) or trkpt.find("{http://www.topografix.com/GPX/1/1}time") or trkpt.find("time")
        if time_el is not None and time_el.text:
            time_str = time_el.text.strip()
        points.append({
            "lat": round(lat, 7),
            "lng": round(lng, 7),
            "alt": ele_val,
            "time": time_str
        })
    return {
        "ok": True,
        "filename": name,
        "points": points,
        "point_count": len(points)
    }


def emit_gps_status() -> None:
    with gps_lock:
        payload = {
            "connected": gps_state["connected"],
            "gps_powered": gps_state["gps_powered"],
            "fix": gps_state["fix"],
            "latitude": gps_state["latitude"],
            "longitude": gps_state["longitude"],
            "altitude": gps_state["altitude"],
            "speed_kmh": gps_state["speed_kmh"],
            "heading": gps_state["heading"],
            "hdop": gps_state["hdop"],
            "satellites_used": gps_state["satellites_used"],
            "satellites_in_view": gps_state["satellites_in_view"],
            "satellites": list(gps_state["satellites"]),
            "utc_time": gps_state["utc_time"],
            "error": gps_state["error"],
            "serial_available": gps_state.get("serial_available", False),
            "serial_import_error": gps_state.get("serial_import_error", ""),
            "at_port": gps_state["at_port"],
            "timestamp": time.time()
        }
    socketio.emit("gps_status", payload)


def emit_gps_trajectory_status() -> None:
    with trajectory_lock:
        payload = {
            "recording": trajectory_state["recording"],
            "paused": trajectory_state["paused"],
            "point_count": trajectory_state["point_count"],
            "start_time": trajectory_state["start_time"]
        }
    socketio.emit("gps_trajectory_status", payload)


def emit_gps_follow_status() -> None:
    with follow_lock:
        payload = {
            "active": follow_state["active"],
            "wp_index": follow_state["wp_index"],
            "wp_total": follow_state["wp_total"],
            "distance": follow_state["distance"],
            "bearing": follow_state["bearing"],
            "throttle": follow_state["throttle"],
            "steering": follow_state["steering"],
            "error": follow_state["error"]
        }
    socketio.emit("gps_follow_status", payload)


def update_lidar_obstacle_map(points: List[Dict[str, Any]]) -> None:
    if not lidar_obstacle_data.get("active", True):
        return
    num_sectors = 18
    safe_mm = follow_config.get("safe_distance_mm", 500)
    critical_mm = follow_config.get("critical_distance_mm", 300)
    sectors = [safe_mm + 1.0] * num_sectors
    for pt in points:
        angle = pt.get("angle", 0)
        dist = pt.get("distance", 0)
        if dist <= 0:
            continue
        if angle > 180:
            continue
        sector = int((angle + 90) / (180 // num_sectors))
        sector = max(0, min(num_sectors - 1, sector))
        if dist < sectors[sector]:
            sectors[sector] = dist
    front_left = num_sectors // 2 - 1
    front_right = num_sectors // 2
    min_front = min(sectors[front_left], sectors[front_right])
    emergency = min_front < critical_mm
    steer_sum = 0.0
    total_weight = 0.0
    for s in range(num_sectors):
        if sectors[s] < safe_mm:
            force = (safe_mm - sectors[s]) / safe_mm
            center_angle = (s + 0.5) * (180.0 / num_sectors) - 90.0
            steer_sum += force * (-center_angle / 90.0)
            total_weight += force
    avoidance = 0.0
    if total_weight > 0:
        avoidance = max(-1.0, min(1.0, steer_sum / total_weight))
    with lidar_obstacle_lock:
        lidar_obstacle_data["timestamp"] = time.time()
        lidar_obstacle_data["emergency_stop"] = emergency
        lidar_obstacle_data["avoidance_steering"] = round(avoidance, 4)
        lidar_obstacle_data["min_front_dist"] = min_front if min_front <= safe_mm else None


def gps_follow_step() -> None:
    if not follow_state["active"]:
        return
    with follow_lock:
        ws = dict(follow_state)
    points = uploaded_trajectory.get("points", [])
    wp = ws["wp_index"]
    if wp >= len(points):
        follow_stop()
        _gps_log("info", "Trajeto concluido. Ultimo waypoint atingido.")
        return
    with gps_lock:
        lat = gps_state["latitude"]
        lng = gps_state["longitude"]
        fix = gps_state["fix"]
        heading = gps_state["heading"] or 0.0
        spd = gps_state["speed_kmh"] or 0.0
    if fix < 2 or lat is None or lng is None:
        robot_stop()
        with follow_lock:
            follow_state["error"] = "Sem fix GPS 3D. Pausado aguardando sinal."
        emit_gps_follow_status()
        return
    if not robot_state.get("armed", False):
        follow_stop()
        _gps_log("warn", "Robo desarmado. Follow cancelado.")
        return
    target = points[wp]
    t_lat = target.get("lat")
    t_lng = target.get("lng")
    if t_lat is None or t_lng is None:
        with follow_lock:
            follow_state["wp_index"] = wp + 1
            follow_state["error"] = None
        emit_gps_follow_status()
        return
    distance = haversine_distance(lat, lng, t_lat, t_lng)
    bearing = bearing_to(lat, lng, t_lat, t_lng)
    hdg_error = angle_diff(bearing, heading)
    kp = follow_config["steering_kp"]
    steering = max(-1.0, min(1.0, hdg_error * kp))
    throttle = min(follow_config["max_auto_speed"], max(0.01, distance * 0.04))

    if lidar_obstacle_data.get("active", True):
        with lidar_obstacle_lock:
            emergency = lidar_obstacle_data.get("emergency_stop", False)
            avoidance = lidar_obstacle_data.get("avoidance_steering", 0.0)
            min_front = lidar_obstacle_data.get("min_front_dist")
        if emergency:
            robot_stop()
            with follow_lock:
                front_mm = min_front if min_front else 0
                follow_state["error"] = f"OBSTACULO! Parada emergencia ({front_mm:.0f}mm)"
            emit_gps_follow_status()
            return
        weight = follow_config.get("avoidance_weight", 0.5)
        if abs(avoidance) > 0.01:
            steering = steering * (1.0 - weight) + avoidance * weight
        safe_mm = follow_config.get("safe_distance_mm", 500)
        if min_front is not None and min_front < safe_mm and min_front > 0:
            factor = min_front / safe_mm
            throttle *= max(0.0, factor)
            if throttle < 0.01:
                throttle = 0.0

    threshold = follow_config["waypoint_threshold"]
    if distance < threshold:
        wp += 1
        with follow_lock:
            follow_state["wp_index"] = wp
            follow_state["error"] = None
        _gps_log("info", f"Waypoint {wp}/{follow_state['wp_total']} atingido ({round(distance,1)}m)")
        emit_gps_follow_status()
        if wp >= len(points):
            follow_stop()
            _gps_log("info", "Trajeto concluido.")
            return
        emit_gps_follow_status()
        return
    left_raw = throttle + steering
    right_raw = throttle - steering
    left = max(-1.0, min(1.0, left_raw))
    right = max(-1.0, min(1.0, right_raw))
    robot_send_duty(left, right, force=True)
    with follow_lock:
        follow_state["distance"] = round(distance, 1)
        follow_state["bearing"] = round(bearing, 1)
        follow_state["throttle"] = round(throttle, 3)
        follow_state["steering"] = round(steering, 3)
        follow_state["error"] = None
    emit_gps_follow_status()


def follow_start() -> Dict[str, Any]:
    if not uploaded_trajectory.get("loaded"):
        return {"ok": False, "error": "Nenhum trajeto carregado. Faca upload de um GPX primeiro."}
    with gps_lock:
        fix = gps_state["fix"]
    if fix < 1:
        return {"ok": False, "error": "Sem fix GPS. Aguarde sinal antes de iniciar."}
    if not robot_state.get("armed", False):
        return {"ok": False, "error": "Robo nao esta armado. Arme primeiro."}
    points = uploaded_trajectory["points"]
    with follow_lock:
        follow_state["active"] = True
        follow_state["wp_index"] = 0
        follow_state["wp_total"] = len(points)
        follow_state["distance"] = 0.0
        follow_state["bearing"] = 0.0
        follow_state["throttle"] = 0.0
        follow_state["steering"] = 0.0
        follow_state["error"] = None
    _gps_log("info", f"Follow iniciado: {len(points)} waypoints")
    emit_gps_follow_status()
    return {"ok": True, "message": f"Seguindo trajeto com {len(points)} waypoints"}


def follow_stop() -> None:
    with follow_lock:
        follow_state["active"] = False
        follow_state["error"] = None
    robot_stop()
    _gps_log("info", "Follow parado")
    emit_gps_follow_status()


def gps_reader_loop() -> None:
    gs = gps_state

    while True:
        gps_restart_event.clear()

        if not SERIAL_AVAILABLE:
            with gps_lock:
                gs["connected"] = False
                gs["gps_powered"] = False
                gs["error"] = "Biblioteca pyserial nao disponivel. Instale com: pip install pyserial"
                gs["serial_available"] = False
                gs["serial_import_error"] = SERIAL_IMPORT_ERROR
            emit_gps_status()
            time.sleep(3)
            continue

        gps_config = get_gps_config()
        at_port = gps_config["at_port"]
        at_baudrate = gps_config["at_baudrate"]

        with gps_lock:
            gs["at_port"] = at_port
            gs["emit_interval"] = gps_config["emit_interval"]

        if not os.path.exists(at_port):
            with gps_lock:
                gs["connected"] = False
                gs["error"] = f"Porta AT {at_port} nao encontrada"
            emit_gps_status()
            _gps_log("error", f"Porta AT {at_port} nao encontrada")
            time.sleep(2)
            continue

        _gps_log("info", f"Ligando GPS via AT em {at_port}...")
        ser = None
        try:
            ser = serial.Serial(port=at_port, baudrate=at_baudrate, timeout=0.3)

            ser.write(b"AT\r\n")
            time.sleep(0.3)
            resp = ser.read(512)
            if b"OK" not in resp:
                ser.close()
                with gps_lock:
                    gs["connected"] = False
                    gs["error"] = f"Modem nao respondeu AT em {at_port}"
                emit_gps_status()
                _gps_log("error", f"Modem nao respondeu AT em {at_port}")
                time.sleep(2)
                continue

            ser.write(b"AT+CGNSSPWR=1\r\n")
            time.sleep(0.5)
            resp2 = ser.read(512)

            if b"OK" not in resp2 and b"READY" not in resp2:
                _gps_log("warn", f"AT+CGNSSPWR=1: {resp2.decode(errors='ignore')[:100]}")

            with gps_lock:
                gs["gps_powered"] = True
            _gps_log("info", "GPS ligado, iniciando streaming CGNSSINFO...")

            ser.write(b"AT+CGNSSINFO=1\r\n")
            time.sleep(0.5)
            resp3 = ser.read(512)
            if b"OK" not in resp3:
                _gps_log("warn", f"AT+CGNSSINFO=1: {resp3.decode(errors='ignore')[:100]}")

            with gps_lock:
                gs["connected"] = True
                gs["error"] = None
                gs["serial_available"] = True
                gs["serial_import_error"] = ""
                gs["satellites"].clear()

            emit_gps_status()
            _gps_log("info", f"Streaming GPS ativo em {at_port}. Aguardando fix...")

            buffer = ""
            last_emit = 0.0

            while not gps_restart_event.is_set():
                try:
                    chunk = ser.read(512)
                    if not chunk:
                        now = time.time()
                        gps_config = get_gps_config()
                        emit_interval = gps_config.get("emit_interval", 1.0)
                        if now - last_emit >= emit_interval:
                            emit_gps_status()
                            last_emit = now
                        continue

                    buffer += chunk.decode("ascii", errors="ignore")

                    while "\n" in buffer:
                        line_end = buffer.index("\n")
                        line = buffer[:line_end].strip()
                        buffer = buffer[line_end + 1:]

                        if not line:
                            continue

                        if line.startswith("+CGNSSINFO:"):
                            info = parse_cgnssinfo(line)
                            if info is None:
                                continue

                            with gps_lock:
                                if info.get("fix") is not None:
                                    gs["fix"] = info["fix"]
                                if info.get("latitude") is not None:
                                    gs["latitude"] = info["latitude"]
                                if info.get("longitude") is not None:
                                    gs["longitude"] = info["longitude"]
                                if info.get("altitude") is not None:
                                    gs["altitude"] = info["altitude"]
                                if info.get("speed_kmh") is not None:
                                    gs["speed_kmh"] = info["speed_kmh"]
                                if info.get("heading") is not None:
                                    gs["heading"] = info["heading"]
                                if info.get("hdop") is not None:
                                    gs["hdop"] = info["hdop"]
                                if info.get("satellites_used") is not None:
                                    gs["satellites_used"] = info["satellites_used"]
                                if info.get("satellites_in_view") is not None:
                                    gs["satellites_in_view"] = info["satellites_in_view"]
                                if info.get("satellites"):
                                    gs["satellites"] = info["satellites"]
                                if info.get("utc_time"):
                                    gs["utc_time"] = info["utc_time"]

                            now = time.time()
                            gps_config = get_gps_config()
                            emit_interval = gps_config.get("emit_interval", 1.0)

                            if now - last_emit >= emit_interval:
                                with gps_lock:
                                    lat = gs["latitude"]
                                    lng = gs["longitude"]
                                    fix = gs["fix"]
                                    alt = gs["altitude"]
                                    spd = gs["speed_kmh"]
                                    hdg = gs["heading"]
                                    utc = gs["utc_time"]
                                emit_gps_status()
                                last_emit = now

                                with trajectory_lock:
                                    recording = trajectory_state["recording"]
                                    paused = trajectory_state["paused"]

                                if recording and not paused and fix > 0 and lat is not None and lng is not None:
                                    point = {
                                        "lat": lat,
                                        "lng": lng,
                                        "alt": alt,
                                        "speed_kmh": spd,
                                        "heading": hdg,
                                        "utc_time": utc,
                                        "epoch": time.time()
                                    }
                                    with trajectory_lock:
                                        if trajectory_state["recording"] and not trajectory_state["paused"]:
                                            point["index"] = trajectory_state["point_count"]
                                            trajectory_state["points"].append(point)
                                            trajectory_state["point_count"] = len(trajectory_state["points"])
                                            socketio.emit("gps_trajectory_point", point)

                                if follow_state.get("active"):
                                    gps_follow_step()

                except serial.SerialException as e:
                    _gps_log("error", f"Erro serial: {e}")
                    with gps_lock:
                        gs["connected"] = False
                        gs["error"] = f"Erro serial: {e}"
                    emit_gps_status()
                    break

                except Exception as e:
                    _gps_log("error", str(e))
                    with gps_lock:
                        gs["error"] = str(e)
                    emit_gps_status()

        except serial.SerialException as e:
            _gps_log("error", f"Nao foi possivel abrir {at_port}: {e}")
            with gps_lock:
                gs["connected"] = False
                gs["error"] = f"Nao foi possivel abrir {at_port}: {e}"
            emit_gps_status()
            time.sleep(3)

        except PermissionError as e:
            _gps_log("error", f"Sem permissao para {at_port}: {e}")
            with gps_lock:
                gs["connected"] = False
                gs["error"] = f"Sem permissao: {e}"
            emit_gps_status()
            time.sleep(5)

        except Exception as e:
            _gps_log("error", str(e))
            with gps_lock:
                gs["connected"] = False
                gs["error"] = str(e)
            emit_gps_status()
            time.sleep(3)

        finally:
            if ser is not None and ser.is_open:
                try:
                    ser.write(b"AT+CGNSSINFO=0\r\n")
                    time.sleep(0.2)
                    ser.read(256)
                except Exception:
                    pass
                try:
                    ser.close()
                except Exception:
                    pass
            with gps_lock:
                gs["connected"] = False
                gs["satellites"].clear()
                gs["satellites_in_view"] = 0
            emit_gps_status()


@app.route("/api/gps/config", methods=["GET"])
def api_gps_get_config():
    return jsonify(get_gps_config())


@app.route("/api/gps/config", methods=["POST"])
def api_gps_set_config():
    data = request.get_json(force=True, silent=True)
    if not isinstance(data, dict):
        data = {}
    return jsonify(save_gps_config(data))


@app.route("/api/gps/status", methods=["GET"])
def api_gps_get_status():
    with gps_lock:
        gps_payload = {
            "connected": gps_state["connected"],
            "gps_powered": gps_state["gps_powered"],
            "fix": gps_state["fix"],
            "latitude": gps_state["latitude"],
            "longitude": gps_state["longitude"],
            "altitude": gps_state["altitude"],
            "speed_kmh": gps_state["speed_kmh"],
            "heading": gps_state["heading"],
            "hdop": gps_state["hdop"],
            "satellites_used": gps_state["satellites_used"],
            "satellites_in_view": gps_state["satellites_in_view"],
            "satellites": list(gps_state["satellites"]),
            "utc_time": gps_state["utc_time"],
            "error": gps_state["error"],
            "at_port": gps_state["at_port"]
        }
    with trajectory_lock:
        traj_payload = {
            "recording": trajectory_state["recording"],
            "paused": trajectory_state["paused"],
            "point_count": trajectory_state["point_count"],
            "start_time": trajectory_state["start_time"]
        }
    return jsonify({"gps": gps_payload, "trajectory": traj_payload})


@app.route("/api/gps/power/on", methods=["POST"])
def api_gps_power_on():
    return jsonify(gps_power_on())


@app.route("/api/gps/power/off", methods=["POST"])
def api_gps_power_off():
    return jsonify(gps_power_off())


@app.route("/api/gps/stream/start", methods=["POST"])
def api_gps_stream_start():
    if not SERIAL_AVAILABLE:
        return jsonify({"ok": False, "error": "pyserial nao disponivel"})
    gps_config = get_gps_config()
    at_port = gps_config["at_port"]
    at_baudrate = gps_config["at_baudrate"]
    if not os.path.exists(at_port):
        return jsonify({"ok": False, "error": f"Porta {at_port} nao encontrada"})
    try:
        ser = serial.Serial(port=at_port, baudrate=at_baudrate, timeout=1.0)
        ser.write(b"AT+CGNSSINFO=1\r\n")
        time.sleep(0.5)
        resp = ser.read(512)
        ser.close()
        if b"OK" in resp:
            _gps_log("info", "Stream CGNSSINFO iniciado")
            gps_restart_event.set()
            return jsonify({"ok": True, "message": "Stream iniciado"})
        return jsonify({"ok": False, "error": "Resposta: " + resp.decode(errors="ignore")[:100]})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)})


@app.route("/api/gps/stream/stop", methods=["POST"])
def api_gps_stream_stop():
    if not SERIAL_AVAILABLE:
        return jsonify({"ok": False, "error": "pyserial nao disponivel"})
    gps_config = get_gps_config()
    at_port = gps_config["at_port"]
    at_baudrate = gps_config["at_baudrate"]
    if not os.path.exists(at_port):
        return jsonify({"ok": False, "error": f"Porta {at_port} nao encontrada"})
    try:
        ser = serial.Serial(port=at_port, baudrate=at_baudrate, timeout=1.0)
        ser.write(b"AT+CGNSSINFO=0\r\n")
        time.sleep(0.5)
        resp = ser.read(512)
        ser.close()
        _gps_log("info", "Stream CGNSSINFO parado")
        gps_restart_event.set()
        return jsonify({"ok": True, "message": "Stream parado"})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)})


@app.route("/api/gps/poll", methods=["POST"])
def api_gps_poll_once():
    if not SERIAL_AVAILABLE:
        return jsonify({"ok": False, "error": "pyserial nao disponivel"})
    gps_config = get_gps_config()
    at_port = gps_config["at_port"]
    at_baudrate = gps_config["at_baudrate"]
    if not os.path.exists(at_port):
        return jsonify({"ok": False, "error": f"Porta {at_port} nao encontrada"})
    try:
        ser = serial.Serial(port=at_port, baudrate=at_baudrate, timeout=1.0)
        ser.write(b"AT+CGNSSINFO\r\n")
        time.sleep(0.5)
        resp = ser.read(512)
        ser.close()
        text = resp.decode(errors="ignore")
        for line in text.split("\n"):
            line = line.strip()
            if line.startswith("+CGNSSINFO:"):
                info = parse_cgnssinfo(line)
                if info:
                    _gps_log("info", f"Poll: fix={info.get('fix')}, sats={info.get('satellites_used')}")
                    return jsonify({"ok": True, "data": info})
        return jsonify({"ok": False, "error": "Nenhum CGNSSINFO recebido"})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)})


@app.route("/api/gps/coldboot", methods=["POST"])
def api_gps_cold_boot():
    if not SERIAL_AVAILABLE:
        return jsonify({"ok": False, "error": "pyserial nao disponivel"})
    gps_config = get_gps_config()
    at_port = gps_config["at_port"]
    at_baudrate = gps_config["at_baudrate"]
    if not os.path.exists(at_port):
        return jsonify({"ok": False, "error": f"Porta {at_port} nao encontrada"})
    try:
        ser = serial.Serial(port=at_port, baudrate=at_baudrate, timeout=1.0)
        ser.write(b"AT+CGNSSCOLD\r\n")
        time.sleep(0.5)
        resp = ser.read(512)
        ser.close()
        _gps_log("info", "Cold boot GPS executado")
        gps_restart_event.set()
        return jsonify({"ok": True, "message": "Cold boot executado. Aguardando reaquisicao..."})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)})


@app.route("/api/gps/hotboot", methods=["POST"])
def api_gps_hot_boot():
    if not SERIAL_AVAILABLE:
        return jsonify({"ok": False, "error": "pyserial nao disponivel"})
    gps_config = get_gps_config()
    at_port = gps_config["at_port"]
    at_baudrate = gps_config["at_baudrate"]
    if not os.path.exists(at_port):
        return jsonify({"ok": False, "error": f"Porta {at_port} nao encontrada"})
    try:
        ser = serial.Serial(port=at_port, baudrate=at_baudrate, timeout=1.0)
        ser.write(b"AT+CGNSSHOT\r\n")
        time.sleep(0.5)
        resp = ser.read(512)
        ser.close()
        _gps_log("info", "Hot boot GPS executado")
        gps_restart_event.set()
        return jsonify({"ok": True, "message": "Hot boot executado"})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)})


@app.route("/api/gps/trajectory/start", methods=["POST"])
def api_trajectory_start():
    return jsonify(trajectory_start())


@app.route("/api/gps/trajectory/pause", methods=["POST"])
def api_trajectory_pause():
    return jsonify(trajectory_pause())


@app.route("/api/gps/trajectory/resume", methods=["POST"])
def api_trajectory_resume():
    return jsonify(trajectory_resume())


@app.route("/api/gps/trajectory/stop", methods=["POST"])
def api_trajectory_stop():
    return jsonify(trajectory_stop())


@app.route("/api/gps/trajectory/download", methods=["GET"])
def api_trajectory_download():
    data = trajectory_to_json()
    data.pop("points", None)
    return jsonify(data)


@app.route("/api/gps/trajectory/download-full", methods=["GET"])
def api_trajectory_download_full():
    data = trajectory_to_json()
    return jsonify(data)


@app.route("/api/gps/log", methods=["GET"])
def api_gps_log():
    return jsonify(list(gps_log_lines))


@app.route("/api/gps/trajectory/download-gpx", methods=["GET"])
def api_trajectory_download_gpx():
    gpx = trajectory_to_gpx()
    if not gpx:
        return jsonify({"ok": False, "error": "Nenhum trajeto gravado"}), 404
    from flask import Response
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    filename = f"trajeto_pandorapi_{timestamp}.gpx"
    return Response(
        gpx,
        mimetype="application/gpx+xml",
        headers={"Content-Disposition": f'attachment; filename="{filename}"'}
    )


@app.route("/api/gps/trajectory/upload", methods=["POST"])
def api_trajectory_upload():
    if "file" not in request.files:
        return jsonify({"ok": False, "error": "Nenhum arquivo enviado"})
    file = request.files["file"]
    if file.filename == "":
        return jsonify({"ok": False, "error": "Nome de arquivo vazio"})
    try:
        xml_string = file.read().decode("utf-8", errors="ignore")
    except Exception as e:
        return jsonify({"ok": False, "error": f"Erro ao ler arquivo: {e}"})
    result = parse_gpx_xml(xml_string)
    if not result.get("ok"):
        return jsonify(result)
    uploaded_trajectory["filename"] = file.filename or result.get("filename", "")
    uploaded_trajectory["points"] = result["points"]
    uploaded_trajectory["point_count"] = result["point_count"]
    uploaded_trajectory["loaded"] = True
    _gps_log("info", f"GPX carregado: {uploaded_trajectory['filename']} — {result['point_count']} pontos")
    return jsonify({
        "ok": True,
        "message": f"GPX carregado: {result['point_count']} pontos",
        "point_count": result["point_count"],
        "filename": uploaded_trajectory["filename"]
    })


@app.route("/api/gps/trajectory/uploaded", methods=["GET"])
def api_trajectory_get_uploaded():
    return jsonify({
        "loaded": uploaded_trajectory["loaded"],
        "filename": uploaded_trajectory["filename"],
        "point_count": uploaded_trajectory["point_count"],
        "points": uploaded_trajectory["points"]
    })


@app.route("/api/gps/trajectory/uploaded", methods=["DELETE"])
def api_trajectory_clear_uploaded():
    uploaded_trajectory["loaded"] = False
    uploaded_trajectory["filename"] = ""
    uploaded_trajectory["points"] = []
    uploaded_trajectory["point_count"] = 0
    _gps_log("info", "Trajeto carregado removido")
    return jsonify({"ok": True, "message": "Trajeto removido"})


@app.route("/api/gps/follow/start", methods=["POST"])
def api_follow_start():
    result = follow_start()
    return jsonify(result)


@app.route("/api/gps/follow/stop", methods=["POST"])
def api_follow_stop():
    follow_stop()
    return jsonify({"ok": True, "message": "Follow parado"})


@app.route("/api/gps/follow/status", methods=["GET"])
def api_follow_get_status():
    with follow_lock:
        return jsonify(dict(follow_state))


@app.route("/api/gps/follow/config", methods=["GET"])
def api_follow_config_get():
    return jsonify({
        "safe_distance_mm": follow_config.get("safe_distance_mm", 500),
        "critical_distance_mm": follow_config.get("critical_distance_mm", 300),
        "avoidance_weight": follow_config.get("avoidance_weight", 0.5),
        "avoidance_enabled": lidar_obstacle_data.get("active", True)
    })


@app.route("/api/gps/follow/config", methods=["PUT"])
def api_follow_config_update():
    data = request.get_json(force=True, silent=True) or {}
    updated = {}
    if "safe_distance_mm" in data:
        val = max(100, min(2000, int(data.get("safe_distance_mm", 500))))
        follow_config["safe_distance_mm"] = val
        updated["safe_distance_mm"] = val
    if "critical_distance_mm" in data:
        val = max(50, min(1000, int(data.get("critical_distance_mm", 300))))
        follow_config["critical_distance_mm"] = val
        updated["critical_distance_mm"] = val
    if "avoidance_weight" in data:
        val = max(0.0, min(1.0, float(data.get("avoidance_weight", 0.5))))
        follow_config["avoidance_weight"] = val
        updated["avoidance_weight"] = val
    if "avoidance_enabled" in data:
        lidar_obstacle_data["active"] = bool(data["avoidance_enabled"])
        updated["avoidance_enabled"] = lidar_obstacle_data["active"]
    _gps_log("info", f"Config avoidance: {json.dumps(updated)}")
    return jsonify(updated)


@app.route("/")
def index():
    return render_template_string(HTML_PAGE)


@app.route("/api/config", methods=["GET"])
def api_get_config():
    return jsonify(load_config())


@app.route("/api/config", methods=["POST"])
def api_set_config():
    data = request.get_json(force=True, silent=True)

    if not isinstance(data, dict):
        data = {}

    config = load_config()

    if "device_path" in data:
        config["device_path"] = str(data.get("device_path") or "")

    if "device_name_contains" in data:
        config["device_name_contains"] = str(data.get("device_name_contains") or "")

    if "deadzone" in data:
        try:
            deadzone = float(data.get("deadzone", 0.05))
            deadzone = max(0.0, min(0.5, deadzone))
            config["deadzone"] = deadzone
        except Exception:
            config["deadzone"] = 0.05

    if "mappings" in data and isinstance(data["mappings"], dict):
        clean_mappings = {}

        for key, value in data["mappings"].items():
            clean_key = str(key).strip()
            clean_value = str(value).strip()

            if clean_key and clean_value:
                clean_mappings[clean_key] = clean_value

        config["mappings"] = clean_mappings

    save_config(config)

    reader_restart_event.set()

    return jsonify(config)


@app.route("/api/state", methods=["GET"])
def api_get_state():
    with state_lock:
        return jsonify(current_state)


@app.route("/api/devices", methods=["GET"])
def api_get_devices():
    return jsonify(list_input_devices())


@app.route("/api/hid/diagnostics", methods=["GET"])
def api_hid_diagnostics():
    return jsonify(input_diagnostics())


@app.route("/api/hid/load-modules", methods=["POST"])
def api_hid_load_modules():
    return jsonify(load_hid_modules())



@app.route("/api/can/config", methods=["GET"])
def api_can_get_config():
    return jsonify(get_can_config())


@app.route("/api/can/config", methods=["POST"])
def api_can_set_config():
    data = request.get_json(force=True, silent=True)

    if not isinstance(data, dict):
        data = {}

    can_config = save_can_config(data)

    return jsonify(can_config)


@app.route("/api/can/scan", methods=["GET"])
def api_can_scan():
    return jsonify(can_scan())


@app.route("/api/can/setup", methods=["POST"])
def api_can_setup():
    data = request.get_json(force=True, silent=True) or {}
    can_config = save_can_config(data)

    return jsonify(can_setup_interface(can_config["interface"], can_config["bitrate"]))


@app.route("/api/can/arm", methods=["POST"])
def api_can_arm():
    can_config = get_can_config()

    with robot_lock:
        robot_state["armed"] = True
        robot_state["interface"] = can_config["interface"]
        robot_state["last_error"] = None

    stop_result = robot_stop(force=True)
    emit_can_status()

    return jsonify({
        "ok": True,
        "armed": True,
        "config": can_config,
        "stop_result": stop_result,
        "message": "Robô armado. Segure o botão homem-morto para enviar movimento."
    })


@app.route("/api/can/disarm", methods=["POST"])
def api_can_disarm():
    stop_result = robot_stop(force=True)

    with robot_lock:
        robot_state["armed"] = False
        robot_state["last_left"] = 0.0
        robot_state["last_right"] = 0.0

    emit_can_status()

    return jsonify({
        "ok": True,
        "armed": False,
        "stop_result": stop_result
    })


@app.route("/api/can/emergency-stop", methods=["POST"])
def api_can_emergency_stop():
    stop_result = robot_stop(force=True)

    with robot_lock:
        robot_state["armed"] = False
        robot_state["last_left"] = 0.0
        robot_state["last_right"] = 0.0
        robot_state["last_error"] = "Parada de emergência acionada"

    emit_can_status()

    return jsonify({
        "ok": True,
        "armed": False,
        "emergency_stop": True,
        "stop_result": stop_result
    })


@app.route("/api/can/test-zero", methods=["POST"])
def api_can_test_zero():
    return jsonify(robot_stop(force=True))


@app.route("/api/bluetooth/prepare", methods=["POST"])
def api_bluetooth_prepare():
    return jsonify(bluetooth_prepare())


@app.route("/api/bluetooth/scan", methods=["POST"])
def api_bluetooth_scan():
    data = request.get_json(force=True, silent=True) or {}
    duration = data.get("duration", 8)

    return jsonify(bluetooth_scan(duration))


@app.route("/api/bluetooth/scan-on", methods=["POST"])
def api_bluetooth_scan_on():
    return jsonify(bluetooth_scan_on())


@app.route("/api/bluetooth/scan-off", methods=["POST"])
def api_bluetooth_scan_off():
    return jsonify(bluetooth_scan_off())


@app.route("/api/bluetooth/devices", methods=["GET"])
def api_bluetooth_devices():
    result = run_bluetoothctl(["devices"], timeout=10)

    return jsonify({
        "ok": result["ok"],
        "devices": parse_bluetooth_devices(result.get("stdout", "")),
        "result": result
    })


@app.route("/api/bluetooth/pair-connect", methods=["POST"])
def api_bluetooth_pair_connect():
    data = request.get_json(force=True, silent=True) or {}
    mac = data.get("mac", "")

    return jsonify(bluetooth_pair_trust_connect(mac))


@app.route("/api/bluetooth/connect", methods=["POST"])
def api_bluetooth_connect():
    data = request.get_json(force=True, silent=True) or {}
    mac = data.get("mac", "")

    return jsonify(bluetooth_connect(mac))


@app.route("/api/bluetooth/disconnect", methods=["POST"])
def api_bluetooth_disconnect():
    data = request.get_json(force=True, silent=True) or {}
    mac = data.get("mac", "")

    return jsonify(bluetooth_disconnect(mac))


@app.route("/api/bluetooth/remove", methods=["POST"])
def api_bluetooth_remove():
    data = request.get_json(force=True, silent=True) or {}
    mac = data.get("mac", "")

    return jsonify(bluetooth_remove(mac))


@app.route("/api/bluetooth/power-on", methods=["POST"])
def api_bluetooth_power_on():
    return jsonify(run_bluetoothctl(["power", "on"], timeout=10))


@app.route("/api/bluetooth/power-off", methods=["POST"])
def api_bluetooth_power_off():
    return jsonify(run_bluetoothctl(["power", "off"], timeout=10))


@app.route("/api/lidar/config", methods=["GET"])
def api_lidar_get_config():
    return jsonify(get_lidar_config())


@app.route("/api/lidar/config", methods=["POST"])
def api_lidar_set_config():
    data = request.get_json(force=True, silent=True)

    if not isinstance(data, dict):
        data = {}

    return jsonify(save_lidar_config(data))


@app.route("/api/lidar/status", methods=["GET"])
def api_lidar_get_status():
    with lidar_lock:
        return jsonify({
            "connected": lidar_state["connected"],
            "port": lidar_state["port"],
            "scanning": lidar_state["scanning"],
            "rotation_speed": lidar_state["rotation_speed"],
            "last_count": lidar_state["last_count"],
            "timestamp": lidar_state["timestamp"],
            "error": lidar_state["error"],
            "serial_available": lidar_state.get("serial_available", False),
            "serial_import_error": lidar_state.get("serial_import_error", "")
        })


@app.route("/api/depth/config", methods=["GET"])
def api_depth_get_config():
    return jsonify(get_depth_config())


@app.route("/api/depth/config", methods=["POST"])
def api_depth_set_config():
    data = request.get_json(force=True, silent=True)
    if not isinstance(data, dict):
        data = {}
    return jsonify(save_depth_config(data))


@app.route("/api/depth/status", methods=["GET"])
def api_depth_get_status():
    with depth_lock:
        return jsonify({
            "connected": depth_state["connected"],
            "fps": depth_state["fps"],
            "min_depth_mm": depth_state["min_depth_mm"],
            "max_depth_mm": depth_state["max_depth_mm"],
            "width": depth_state["width"],
            "height": depth_state["height"],
            "error": depth_state["error"],
            "depth_available": depth_state.get("depth_available", False),
            "depth_import_error": depth_state.get("depth_import_error", "")
        })


@socketio.on("connect")
def socket_connect():
    emit_status()
    emit_gps_status()
    emit_gps_trajectory_status()
    emit_gps_follow_status()
    emit_depth_status()
    for entry in gps_log_lines:
        socketio.emit("gps_log", entry)


def print_startup_message() -> None:
    print("")
    print("============================================================")
    print(" Gamepad HID Bluetooth Web - Raspberry Pi / Debian")
    print("============================================================")
    print(" Abra no navegador:")
    print(" http://IP_DO_RASPBERRY:5005")
    print("")
    print(" Local:")
    print(" http://127.0.0.1:5005")
    print("")
    print(" Arquivo de configuracao:")
    print(f" {CONFIG_FILE}")
    print("")
    print(" Dependencias:")
    print(" sudo apt install -y bluetooth bluez evtest joystick can-utils iproute2")
    print(" pip install flask flask-socketio evdev")
    print("")
    print(" Se der erro de permissao no /dev/input/eventX:")
    print(" sudo usermod -aG input $USER")
    print(" sudo reboot")
    print("")
    print(" Diagnostico se Bluetooth conecta mas nao aparece HID:")
    print(" cat /proc/bus/input/devices")
    print(" ls -l /dev/input/event*")
    print(" modinfo hid-nintendo")
    print(" sudo modprobe hidp")
    print(" sudo modprobe hid-nintendo")
    print("")
    print(" CANable / Flipsky:")
    print(" ip -details link show can0")
    print(" sudo ip link set can0 up type can bitrate 500000")
    print(" candump can0")
    print("============================================================")
    print("")


if __name__ == "__main__":
    load_config()
    print_startup_message()

    reader_thread = threading.Thread(
        target=gamepad_reader_loop,
        daemon=True
    )

    reader_thread.start()

    lidar_thread = threading.Thread(
        target=lidar_reader_loop,
        daemon=True
    )
    lidar_thread.start()

    gps_thread = threading.Thread(
        target=gps_reader_loop,
        daemon=True
    )
    gps_thread.start()

    if get_depth_config().get("enabled", True):
        depth_thread = threading.Thread(
            target=depth_camera_loop,
            daemon=True
        )
        depth_thread.start()

    socketio.run(
        app,
        host="0.0.0.0",
        port=5005,
        allow_unsafe_werkzeug=True
    )
