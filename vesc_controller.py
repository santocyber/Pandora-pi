#!/usr/bin/env python3
"""
vesc_controller.py — Controle de robô diferencial com gamepad Bluetooth HID
Modos: serial (1 VESC USB) ou CAN (2 VESCs via CANable)
Telemetria em tempo real (serial) + envio de duty/freio (serial ou CAN)
"""

import os
import re
import json
import time
import math
import glob
import socket
import struct
import subprocess
import threading
from datetime import datetime
from typing import Any, Dict, Optional, List

from flask import Flask, request, jsonify, render_template_string
from flask_socketio import SocketIO

# ---------------------------------------------------------------------------
# evdev (gamepad)
# ---------------------------------------------------------------------------
try:
    from evdev import InputDevice, list_devices, ecodes
    EVDEV_AVAILABLE = True
    EVDEV_IMPORT_ERROR = ""
except Exception as e:
    InputDevice = None
    list_devices = None
    ecodes = None
    EVDEV_AVAILABLE = False
    EVDEV_IMPORT_ERROR = str(e)

# ---------------------------------------------------------------------------
# pyvesc (serial VESC)
# ---------------------------------------------------------------------------
try:
    import pyvesc
    from pyvesc import VESC
    from pyvesc.VESC.messages import SetCurrentBrake
    PYVESC_AVAILABLE = True
    PYVESC_IMPORT_ERROR = ""
except Exception as e:
    VESC = None
    SetCurrentBrake = None
    pyvesc = None
    PYVESC_AVAILABLE = False
    PYVESC_IMPORT_ERROR = str(e)

# ---------------------------------------------------------------------------
# OpenCV (camera)
# ---------------------------------------------------------------------------
try:
    import cv2
    import base64
    CAMERA_AVAILABLE = True
    CAMERA_IMPORT_ERROR = ""
except Exception as e:
    cv2 = None
    base64 = None
    CAMERA_AVAILABLE = False
    CAMERA_IMPORT_ERROR = str(e)

# ---------------------------------------------------------------------------
# Flask + Socket.IO
# ---------------------------------------------------------------------------
app = Flask(__name__)
app.config["SECRET_KEY"] = "vesc-controller-secret"
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
CAN_EFF_FLAG = 0x80000000
CAN_PACKET_SET_DUTY = 0
CAN_PACKET_SET_CURRENT = 1
CAN_PACKET_SET_CURRENT_BRAKE = 2
CAN_PACKET_SET_RPM = 3

CODE_FALLBACK_NAMES = {
    304: "BTN_SOUTH", 305: "BTN_EAST", 306: "BTN_C", 307: "BTN_NORTH",
    308: "BTN_WEST", 309: "BTN_Z", 310: "BTN_TL", 311: "BTN_TR",
    312: "BTN_TL2", 313: "BTN_TR2", 314: "BTN_SELECT", 315: "BTN_START",
    316: "BTN_MODE", 317: "BTN_THUMBL", 318: "BTN_THUMBR",
    0: "ABS_X", 1: "ABS_Y", 2: "ABS_Z", 3: "ABS_RX",
    4: "ABS_RY", 5: "ABS_RZ", 16: "ABS_HAT0X", 17: "ABS_HAT0Y",
}

FRIENDLY_NAMES = {
    "BTN_SOUTH": "B", "BTN_EAST": "A", "BTN_NORTH": "X", "BTN_WEST": "Y",
    "BTN_TL": "L", "BTN_TR": "R", "BTN_TL2": "ZL", "BTN_TR2": "ZR",
    "BTN_SELECT": "-", "BTN_START": "+", "BTN_MODE": "HOME",
    "BTN_THUMBL": "L3", "BTN_THUMBR": "R3",
    "ABS_X": "Eixo X", "ABS_Y": "Eixo Y",
    "ABS_RX": "Eixo RX", "ABS_RY": "Eixo RY",
    "ABS_Z": "Gatilho L", "ABS_RZ": "Gatilho R",
}

FAULTS = {
    0: "NONE", 1: "OVER_VOLTAGE", 2: "UNDER_VOLTAGE", 3: "DRV",
    4: "ABS_OVER_CURRENT", 5: "OVER_TEMP_FET", 6: "OVER_TEMP_MOTOR",
    7: "GATE_DRIVER_OVER_VOLTAGE", 8: "GATE_DRIVER_UNDER_VOLTAGE",
    9: "MCU_UNDER_VOLTAGE", 10: "BOOTING_FROM_WATCHDOG_RESET",
    11: "ENCODER_SPI", 12: "ENCODER_SINCOS_BELOW_MIN_AMPLITUDE",
    13: "ENCODER_SINCOS_ABOVE_MAX_AMPLITUDE", 14: "FLASH_CORRUPTION",
    15: "HIGH_OFFSET_CURRENT_SENSOR_1", 16: "HIGH_OFFSET_CURRENT_SENSOR_2",
    17: "HIGH_OFFSET_CURRENT_SENSOR_3", 18: "UNBALANCED_CURRENTS",
}

# ---------------------------------------------------------------------------
# Configuration (env vars + defaults)
# ---------------------------------------------------------------------------
VESC_MODE = os.getenv("VESC_MODE", "serial").strip().lower()
if VESC_MODE not in ("serial", "can"):
    VESC_MODE = "serial"

# Serial config
VESC_SERIAL_PORT = os.getenv("VESC_SERIAL_PORT", "/dev/ttyACM0")
VESC_SERIAL_BAUD = int(os.getenv("VESC_SERIAL_BAUD", "115200"))

# CAN config
VESC_CAN_INTERFACE = os.getenv("VESC_CAN_INTERFACE", "can0")
VESC_CAN_BITRATE = int(os.getenv("VESC_CAN_BITRATE", "500000"))
VESC_CAN_LEFT_ID = int(os.getenv("VESC_CAN_LEFT_ID", "1"))
VESC_CAN_RIGHT_ID = int(os.getenv("VESC_CAN_RIGHT_ID", "2"))
VESC_CAN_ID_3 = int(os.getenv("VESC_CAN_ID_3", "3"))
VESC_CAN_ID_4 = int(os.getenv("VESC_CAN_ID_4", "4"))
VESC_MOTOR_COUNT = int(os.getenv("VESC_MOTOR_COUNT", "2"))  # 2 or 4

# Control params
MAX_DUTY = float(os.getenv("VESC_MAX_DUTY", "0.25"))
STEERING_GAIN = float(os.getenv("VESC_STEERING_GAIN", "0.65"))
DEADMAN_BUTTON = os.getenv("VESC_DEADMAN_BUTTON", "BTN_TR")
BRAKE_BUTTON = os.getenv("VESC_BRAKE_BUTTON", "BTN_SOUTH")
BRAKE_CURRENT = float(os.getenv("VESC_BRAKE_CURRENT", "8.0"))
SEND_INTERVAL = float(os.getenv("VESC_SEND_INTERVAL", "0.05"))
CONTROL_TIMEOUT = float(os.getenv("VESC_CONTROL_TIMEOUT", "0.5"))
INVERT_THROTTLE = os.getenv("VESC_INVERT_THROTTLE", "1") == "1"
INVERT_STEERING = os.getenv("VESC_INVERT_STEERING", "0") == "1"
INVERT_LEFT = os.getenv("VESC_INVERT_LEFT", "0") == "1"
INVERT_RIGHT = os.getenv("VESC_INVERT_RIGHT", "1") == "1"
THROTTLE_AXIS = os.getenv("VESC_THROTTLE_AXIS", "ABS_Y")
STEERING_AXIS = os.getenv("VESC_STEERING_AXIS", "ABS_RX")
DEADZONE = float(os.getenv("VESC_DEADZONE", "0.05"))

# ---------------------------------------------------------------------------
# Locks + shared state
# ---------------------------------------------------------------------------
gamepad_lock = threading.Lock()
vesc_lock = threading.Lock()
control_lock = threading.Lock()

gamepad_state: Dict[str, Any] = {
    "connected": False,
    "device_path": None,
    "device_name": None,
    "buttons": {},
    "axes": {},
    "evdev_available": EVDEV_AVAILABLE,
    "error": None,
}

vesc_state: Dict[str, Any] = {
    "connected": False,
    "mode": VESC_MODE,
    "firmware": None,
    "data": {},
    "last_update": None,
    "error": None,
}

control_state: Dict[str, Any] = {
    "armed": False,
    "deadman_ok": False,
    "brake_active": False,
    "throttle": 0.0,
    "steering": 0.0,
    "left_duty": 0.0,
    "right_duty": 0.0,
    "last_send_time": 0.0,
    "can_ready": False,
}

control_mode: str = "gamepad"  # "gamepad" or "keyboard"
keyboard_throttle: float = 0.0
keyboard_steering: float = 0.0
keyboard_brake: bool = False
keyboard_last_update: float = 0.0

selected_gamepad_path: Optional[str] = None

camera_lock = threading.Lock()
RECORDINGS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "recordings")
os.makedirs(RECORDINGS_DIR, exist_ok=True)

camera_state: Dict[str, Any] = {
    "active": False,
    "recording": False,
    "fps": 0.0,
    "width": 640,
    "height": 480,
    "current_file": None,
    "error": None,
    "camera_available": CAMERA_AVAILABLE,
    "camera_import_error": CAMERA_IMPORT_ERROR,
}

can_interface_ready = False
gamepad_restart_event = threading.Event()

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))

def safe_float(value, default=0.0):
    try:
        if value is None or value == "":
            return default
        return float(str(value).replace(",", "."))
    except Exception:
        return default

def now_iso():
    return datetime.now().isoformat(timespec="seconds")

def build_vesc_ext_id(command_id: int, vesc_id: int) -> int:
    return ((int(command_id) & 0xFF) << 8) | (int(vesc_id) & 0xFF)

# ---------------------------------------------------------------------------
# Gamepad (evdev)
# ---------------------------------------------------------------------------
def ev_name(mapping, code):
    try:
        name = mapping.get(code)
        if name:
            return name if isinstance(name, str) else str(name)
    except Exception:
        pass
    return CODE_FALLBACK_NAMES.get(code, f"CODE_{code}")

def is_probably_gamepad(device):
    name = (getattr(device, "name", "") or "").lower()
    ev = getattr(device, "capabilities", lambda: {})().get(ecodes.EV_ABS, []) if ecodes else []
    if "gamepad" in name or "joy" in name or "pro controller" in name:
        return True
    if "nintendo" in name or "dualshock" in name or "xbox" in name:
        return True
    if ecodes is not None:
        abs_codes = {c[0] if isinstance(c, tuple) else c for c in ev}
        if {0, 1}.issubset(abs_codes) or {16, 17}.issubset(abs_codes):
            return True
    return False

def find_gamepad():
    if not EVDEV_AVAILABLE or list_devices is None:
        return None
    if selected_gamepad_path:
        try:
            dev = InputDevice(selected_gamepad_path)
            if is_probably_gamepad(dev):
                return dev
        except Exception:
            pass
    paths = list_devices()
    for p in paths:
        try:
            dev = InputDevice(p)
            if is_probably_gamepad(dev):
                return dev
        except Exception:
            continue
    return None

def get_abs_infos(device):
    infos = {}
    if ecodes is None:
        return infos
    caps = device.capabilities().get(ecodes.EV_ABS, [])
    for cap in caps:
        code, abs_info = cap if isinstance(cap, tuple) else (cap, None)
        name = ev_name(ecodes.ABS, code)
        infos[name] = abs_info
    return infos

def normalize_axis(value, abs_info, deadzone):
    try:
        low = abs_info.min if abs_info and hasattr(abs_info, "min") else 0
        high = abs_info.max if abs_info and hasattr(abs_info, "max") else 255
        mid = (low + high) / 2.0
        span = (high - low) / 2.0
        if span == 0:
            return 0.0
        raw = (value - mid) / span
        if abs(raw) < deadzone:
            return 0.0
        rng = 1.0 - deadzone
        return (raw - math.copysign(deadzone, raw)) / rng
    except Exception:
        return 0.0

def emit_gamepad_status():
    with gamepad_lock:
        payload = dict(gamepad_state)
    payload["selected_path"] = selected_gamepad_path
    socketio.emit("gamepad_status", payload)

def emit_vesc_telemetry():
    with vesc_lock:
        payload = dict(vesc_state)
    socketio.emit("vesc_telemetry", payload)

def emit_control_status():
    with control_lock:
        payload = dict(control_state)
    payload["can_interface_ready"] = can_interface_ready
    payload["mode"] = VESC_MODE
    socketio.emit("control_status", payload)

def emit_camera_status():
    with camera_lock:
        payload = dict(camera_state)
    socketio.emit("camera_status", payload)

# ---------------------------------------------------------------------------
# VESC Serial Telemetry
# ---------------------------------------------------------------------------
def convert_measurements(values):
    raw = vars(values).copy()
    data = {}
    for key, value in raw.items():
        if isinstance(value, bytes):
            if len(value) == 1:
                data[key] = value[0]
            else:
                try:
                    data[key] = value.decode("utf-8", errors="ignore")
                except Exception:
                    data[key] = value.hex()
        elif isinstance(value, float):
            data[key] = None if math.isnan(value) or math.isinf(value) else value
        else:
            data[key] = value

    fault_val = raw.get("mc_fault_code")
    if isinstance(fault_val, bytes) and len(fault_val) > 0:
        fault_val = fault_val[0]
    fault_num = fault_val if isinstance(fault_val, int) else None
    data["fault_number"] = fault_num
    data["fault_name"] = FAULTS.get(fault_num, "UNKNOWN") if fault_num else "NONE"

    v_in = safe_float(data.get("v_in"))
    i_in = safe_float(data.get("avg_input_current"))
    i_motor = safe_float(data.get("avg_motor_current"))
    duty = safe_float(data.get("duty_cycle_now"))
    rpm = safe_float(data.get("rpm"))

    data["input_power_w"] = round(v_in * i_in, 1)
    data["duty_percent"] = round(duty * 100.0, 1)
    data["rpm_abs"] = abs(rpm)
    return data

def send_current_brake_serial(motor, current_a):
    current_a = clamp(abs(float(current_a)), 0.1, 200.0)
    if hasattr(motor, "set_current_brake"):
        motor.set_current_brake(current_a)
    else:
        motor.write(pyvesc.encode(SetCurrentBrake(current_a)))

# ---------------------------------------------------------------------------
# CAN Protocol
# ---------------------------------------------------------------------------
def validate_can_interface_name(name):
    return re.sub(r"[^a-zA-Z0-9_.-]", "", str(name).strip())[:15]

def socketcan_send_extended(interface_name, arbitration_id, data):
    try:
        interface_name = validate_can_interface_name(interface_name)
        data = bytes(data[:8])
        can_id = CAN_EFF_FLAG | (arbitration_id & 0x1FFFFFFF)
        can_dlc = len(data)
        frame = struct.pack("=IB3x8s", can_id, can_dlc, data.ljust(8, b"\x00"))
        with socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as sock:
            sock.bind((interface_name,))
            sock.send(frame)
        return {"ok": True}
    except Exception as e:
        return {"ok": False, "error": str(e)}

def vesc_send_duty(interface_name, vesc_id, duty):
    duty = clamp(float(duty), -1.0, 1.0)
    scaled = int(duty * 100000.0)
    data = struct.pack(">i", scaled)
    arb_id = build_vesc_ext_id(CAN_PACKET_SET_DUTY, int(vesc_id))
    return socketcan_send_extended(interface_name, arb_id, data)

def vesc_send_current_brake_can(interface_name, vesc_id, current_a):
    current_a = clamp(float(current_a), 0.0, 200.0)
    scaled = int(current_a * 1000.0)
    data = struct.pack(">i", scaled)
    arb_id = build_vesc_ext_id(CAN_PACKET_SET_CURRENT_BRAKE, int(vesc_id))
    return socketcan_send_extended(interface_name, arb_id, data)

def can_setup_interface():
    global can_interface_ready
    iface = validate_can_interface_name(VESC_CAN_INTERFACE)
    try:
        subprocess.run(
            ["ip", "link", "set", iface, "down"],
            capture_output=True, timeout=5
        )
        result = subprocess.run(
            ["ip", "link", "set", iface, "up", "type", "can",
             "bitrate", str(VESC_CAN_BITRATE)],
            capture_output=True, text=True, timeout=10
        )
        if result.returncode != 0:
            return {"ok": False, "error": result.stderr.strip() or "Falha ao ativar CAN"}
        time.sleep(0.2)
        can_interface_ready = True
        return {"ok": True, "interface": iface, "bitrate": VESC_CAN_BITRATE}
    except FileNotFoundError:
        return {"ok": False, "error": "Comando 'ip' nao encontrado. Precisa de iproute2."}
    except Exception as e:
        return {"ok": False, "error": str(e)}

# ---------------------------------------------------------------------------
# Motor Math
# ---------------------------------------------------------------------------
def compute_motor_duty():
    throttle = 0.0
    steering = 0.0
    deadman = False
    if control_mode == "keyboard":
        throttle = keyboard_throttle
        steering = keyboard_steering
        deadman = True
    else:
        with gamepad_lock:
            axes = dict(gamepad_state.get("axes", {}))
            buttons = dict(gamepad_state.get("buttons", {}))
        throttle = float(axes.get(THROTTLE_AXIS, 0.0))
        steering = float(axes.get(STEERING_AXIS, 0.0))
        deadman = int(buttons.get(DEADMAN_BUTTON, 0)) in [1, 2]
    if control_mode != "keyboard":
        if INVERT_THROTTLE:
            throttle *= -1.0
        if INVERT_STEERING:
            steering *= -1.0
    left = throttle + (steering * STEERING_GAIN)
    right = throttle - (steering * STEERING_GAIN)
    left = clamp(left, -1.0, 1.0) * MAX_DUTY
    right = clamp(right, -1.0, 1.0) * MAX_DUTY
    if INVERT_LEFT:
        left *= -1.0
    if INVERT_RIGHT:
        right *= -1.0
    if abs(left) < 0.002:
        left = 0.0
    if abs(right) < 0.002:
        right = 0.0
    brake = False
    if control_mode == "keyboard":
        brake = keyboard_brake
        if time.time() - keyboard_last_update > 0.3:
            left, right, throttle, steering = 0.0, 0.0, 0.0, 0.0
            brake = False
            deadman = False
    else:
        with gamepad_lock:
            deadman = int(buttons.get(DEADMAN_BUTTON, 0)) in [1, 2]
            brake = int(buttons.get(BRAKE_BUTTON, 0)) in [1, 2]
    return left, right, throttle, steering, deadman, brake

# ---------------------------------------------------------------------------
# Control logic (CAN mode)
# ---------------------------------------------------------------------------
def apply_can_control(left, right, brake):
    iface = validate_can_interface_name(VESC_CAN_INTERFACE)
    ids = [VESC_CAN_LEFT_ID, VESC_CAN_RIGHT_ID] + ([VESC_CAN_ID_3, VESC_CAN_ID_4] if VESC_MOTOR_COUNT == 4 else [])
    duties = [left, right] + ([left, right] if VESC_MOTOR_COUNT == 4 else [])
    if brake:
        results = [vesc_send_current_brake_can(iface, vid, BRAKE_CURRENT) for vid in ids]
    else:
        if left == 0.0 and right == 0.0:
            for vid in ids:
                vesc_send_current_brake_can(iface, vid, 0.0)
        results = [vesc_send_duty(iface, vid, duty) for vid, duty in zip(ids, duties)]
    ok = all(r.get("ok") for r in results)
    return ok, results[0] if results else {}, results[1] if len(results) > 1 else {}

# ---------------------------------------------------------------------------
# Control loop step
# ---------------------------------------------------------------------------
def control_loop_step():
    left, right, throttle, steering, deadman, brake = compute_motor_duty()
    now = time.time()
    with control_lock:
        armed = control_state["armed"]
        control_state["throttle"] = throttle
        control_state["steering"] = steering
        control_state["deadman_ok"] = deadman
        control_state["brake_active"] = brake
    if not armed:
        left, right = 0.0, 0.0
        brake = False
    if not deadman and armed:
        left, right = 0.0, 0.0
        brake_active = False
        with control_lock:
            control_state["brake_active"] = False
    with control_lock:
        elapsed = now - control_state.get("last_send_time", 0.0)
        if elapsed < SEND_INTERVAL and not brake:
            return
        control_state["left_duty"] = left
        control_state["right_duty"] = right
        control_state["last_send_time"] = now
    if VESC_MODE == "can":
        ok, rl, rr = apply_can_control(left, right, brake)
        with control_lock:
            control_state["can_ready"] = can_interface_ready
    else:
        ok, rl, rr = True, {}, {}
        with vesc_lock:
            vesc_state["_pending_left"] = left
            vesc_state["_pending_right"] = right
            vesc_state["_pending_brake"] = brake
            vesc_state["_control_updated"] = now
    emit_control_status()

# ---------------------------------------------------------------------------
# Gamepad event handlers
# ---------------------------------------------------------------------------
def handle_key_event(event):
    if ecodes is None:
        return
    code = ev_name(ecodes.KEY, event.code)
    action = FRIENDLY_NAMES.get(code, code)
    with gamepad_lock:
        gamepad_state["buttons"][code] = event.value
    socketio.emit("gamepad_event", {
        "kind": "button", "code": code, "action": action,
        "value": event.value, "pressed": event.value == 1,
        "released": event.value == 0, "held": event.value == 2,
        "timestamp": time.time()
    })
    print(f"[gamepad] BTN {code} val={event.value}")
    if code in (DEADMAN_BUTTON, BRAKE_BUTTON, "ABS_Z", "ABS_RZ"):
        control_loop_step()

def handle_abs_event(event, abs_infos):
    if ecodes is None:
        return
    code = ev_name(ecodes.ABS, event.code)
    action = FRIENDLY_NAMES.get(code, code)
    normalized = normalize_axis(event.value, abs_infos.get(code), DEADZONE)
    with gamepad_lock:
        gamepad_state["axes"][code] = normalized
    socketio.emit("gamepad_event", {
        "kind": "axis", "code": code, "action": action,
        "value": event.value, "normalized": round(normalized, 3),
        "timestamp": time.time()
    })
    print(f"[gamepad] AXIS {code} raw={event.value} norm={normalized:.3f}")
    if code in (THROTTLE_AXIS, STEERING_AXIS, "ABS_Z", "ABS_RZ"):
        control_loop_step()

# ---------------------------------------------------------------------------
# Gamepad reader thread
# ---------------------------------------------------------------------------
def gamepad_reader_loop():
    while True:
        gamepad_restart_event.clear()
        if not EVDEV_AVAILABLE:
            with gamepad_lock:
                gamepad_state["connected"] = False
                gamepad_state["error"] = "evdev nao disponivel"
            emit_gamepad_status()
            time.sleep(3)
            continue
        device = find_gamepad()
        if device is None:
            with gamepad_lock:
                gamepad_state["connected"] = False
                gamepad_state["error"] = "Nenhum gamepad HID encontrado"
            emit_gamepad_status()
            time.sleep(2)
            continue
        with gamepad_lock:
            gamepad_state["connected"] = True
            gamepad_state["device_path"] = device.path
            gamepad_state["device_name"] = device.name
            gamepad_state["buttons"] = {}
            gamepad_state["axes"] = {}
            gamepad_state["error"] = None
        emit_gamepad_status()
        abs_infos = get_abs_infos(device)
        print(f"[gamepad] Conectado: {device.name} @ {device.path}")
        print(f"[gamepad] Eixos detectados: {list(abs_infos.keys())}")
        try:
            for event in device.read_loop():
                if gamepad_restart_event.is_set():
                    break
                if event.type == ecodes.EV_KEY:
                    handle_key_event(event)
                elif event.type == ecodes.EV_ABS:
                    handle_abs_event(event, abs_infos)
        except PermissionError as e:
            with gamepad_lock:
                gamepad_state["connected"] = False
                gamepad_state["error"] = f"Permissao negada: {e}"
            emit_gamepad_status()
            time.sleep(3)
        except OSError as e:
            with gamepad_lock:
                gamepad_state["connected"] = False
                gamepad_state["error"] = f"Dispositivo desconectado: {e}"
            emit_gamepad_status()
            time.sleep(1)
        except Exception as e:
            with gamepad_lock:
                gamepad_state["connected"] = False
                gamepad_state["error"] = str(e)
            emit_gamepad_status()
            time.sleep(2)

# ---------------------------------------------------------------------------
# VESC reader thread (serial mode only)
# ---------------------------------------------------------------------------
def vesc_reader_loop():
    while True:
        if VESC_MODE != "serial" or not PYVESC_AVAILABLE:
            time.sleep(1)
            continue
        try:
            with VESC(serial_port=VESC_SERIAL_PORT, start_heartbeat=False) as motor:
                try:
                    fw = motor.get_firmware_version()
                except Exception:
                    fw = None
                with vesc_lock:
                    vesc_state["connected"] = True
                    vesc_state["firmware"] = str(fw)
                    vesc_state["error"] = None
                emit_vesc_telemetry()
                while True:
                    try:
                        values = motor.get_measurements()
                        data = convert_measurements(values)
                        fault = data.get("fault_number")
                        if fault and fault != 0:
                            try:
                                motor.set_duty_cycle(0.0)
                                motor.set_current(0.0)
                            except Exception:
                                pass
                            with control_lock:
                                control_state["armed"] = False
                            emit_control_status()
                    except Exception:
                        data = {}
                    data["timestamp"] = time.time()
                    with vesc_lock:
                        vesc_state["data"] = data
                        vesc_state["last_update"] = now_iso()
                    emit_vesc_telemetry()
                    pending_left = vesc_state.pop("_pending_left", 0.0)
                    pending_right = vesc_state.pop("_pending_right", 0.0)
                    pending_brake = vesc_state.pop("_pending_brake", False)
                    updated = vesc_state.pop("_control_updated", 0.0)
                    with control_lock:
                        armed = control_state["armed"]
                        deadman = control_state["deadman_ok"]
                    if armed and deadman and (time.time() - updated) < CONTROL_TIMEOUT:
                        if pending_brake:
                            try:
                                motor.set_duty_cycle(0.0)
                            except Exception:
                                pass
                            send_current_brake_serial(motor, BRAKE_CURRENT)
                        elif pending_left != 0.0 or pending_right != 0.0:
                            duty = pending_left if abs(pending_left) >= abs(pending_right) else pending_right
                            try:
                                motor.set_current(0.0)
                            except Exception:
                                pass
                            motor.set_duty_cycle(clamp(duty, -MAX_DUTY, MAX_DUTY))
                        else:
                            try:
                                motor.set_duty_cycle(0.0)
                            except Exception:
                                pass
                    else:
                        try:
                            motor.set_duty_cycle(0.0)
                            motor.set_current(0.0)
                        except Exception:
                            pass
                    emit_control_status()
                    time.sleep(0.05)
        except Exception as e:
            with vesc_lock:
                vesc_state["connected"] = False
                vesc_state["error"] = str(e)
            emit_vesc_telemetry()
            time.sleep(2)

# ---------------------------------------------------------------------------
# Flask routes
# ---------------------------------------------------------------------------
@app.route("/")
def index():
    return render_template_string(HTML_TEMPLATE,
        vesc_mode=VESC_MODE,
        vesc_serial_port=VESC_SERIAL_PORT,
        vesc_can_interface=VESC_CAN_INTERFACE,
        vesc_can_bitrate=VESC_CAN_BITRATE,
        vesc_can_left_id=VESC_CAN_LEFT_ID,
        vesc_can_right_id=VESC_CAN_RIGHT_ID,
        max_duty=MAX_DUTY,
        throttle_axis=THROTTLE_AXIS,
        steering_axis=STEERING_AXIS)

@app.route("/api/state")
def api_state():
    with gamepad_lock:
        gp = dict(gamepad_state)
    with vesc_lock:
        vs = dict(vesc_state)
        vs_data = vs.pop("data", {})
        vs.pop("_pending_left", None)
        vs.pop("_pending_right", None)
        vs.pop("_pending_brake", None)
        vs.pop("_control_updated", None)
    with control_lock:
        cs = dict(control_state)
    return jsonify({
        "gamepad": gp,
        "vesc": vs,
        "vesc_data": vs_data,
        "control": cs,
        "can_interface_ready": can_interface_ready,
        "mode": VESC_MODE,
    })

@app.route("/api/arm", methods=["POST"])
def api_arm():
    with control_lock:
        control_state["armed"] = True
    if VESC_MODE == "can" and not can_interface_ready:
        can_setup_interface()
    emit_control_status()
    return jsonify({"ok": True, "armed": True})

@app.route("/api/disarm", methods=["POST"])
def api_disarm():
    with control_lock:
        control_state["armed"] = False
        control_state["left_duty"] = 0.0
        control_state["right_duty"] = 0.0
    if VESC_MODE == "can":
        apply_can_control(0.0, 0.0, False)
    with vesc_lock:
        vesc_state["_pending_left"] = 0.0
        vesc_state["_pending_right"] = 0.0
        vesc_state["_pending_brake"] = False
    emit_control_status()
    return jsonify({"ok": True, "armed": False})

@app.route("/api/emergency-stop", methods=["POST"])
def api_emergency_stop():
    with control_lock:
        control_state["armed"] = False
        control_state["left_duty"] = 0.0
        control_state["right_duty"] = 0.0
    if VESC_MODE == "can":
        apply_can_control(0.0, 0.0, False)
    with vesc_lock:
        vesc_state["_pending_left"] = 0.0
        vesc_state["_pending_right"] = 0.0
        vesc_state["_pending_brake"] = False
    emit_control_status()
    return jsonify({"ok": True, "emergency_stop": True})

@app.route("/api/can/setup", methods=["POST"])
def api_can_setup():
    result = can_setup_interface()
    emit_control_status()
    return jsonify(result)

@app.route("/api/can/status", methods=["GET"])
def api_can_status():
    return jsonify({
        "can_interface_ready": can_interface_ready,
        "interface": VESC_CAN_INTERFACE,
        "bitrate": VESC_CAN_BITRATE,
        "left_id": VESC_CAN_LEFT_ID,
        "right_id": VESC_CAN_RIGHT_ID,
    })

@app.route("/api/ports", methods=["GET"])
def api_ports():
    ports = []
    patterns = ["/dev/ttyACM*", "/dev/ttyUSB*", "/dev/ttyAMA*"]
    for pat in patterns:
        ports.extend(glob.glob(pat))
    ports = sorted(set(ports))
    input_devs = []
    if EVDEV_AVAILABLE and list_devices:
        try:
            for p in list_devices():
                try:
                    dev = InputDevice(p)
                    input_devs.append({"path": p, "name": dev.name})
                except Exception:
                    input_devs.append({"path": p, "name": "?"})
        except Exception:
            pass
    return jsonify({
        "serial_ports": ports,
        "current_serial_port": VESC_SERIAL_PORT,
        "input_devices": input_devs,
        "current_device_path": gamepad_state.get("device_path", ""),
        "selected_gamepad_path": selected_gamepad_path,
        "camera_devices": list_camera_devices(),
        "current_camera_device": CAMERA_DEVICE,
    })

@app.route("/api/gamepad/select", methods=["POST"])
def api_gamepad_select():
    global selected_gamepad_path
    data = request.get_json(silent=True) or {}
    path = data.get("path", None)
    if path is not None:
        path = str(path).strip()
        if not path or not os.path.exists(path):
            return jsonify({"ok": False, "error": f"Caminho invalido: {path}"})
    selected_gamepad_path = path if path else None
    gamepad_restart_event.set()
    print(f"[gamepad] Selecionado: {selected_gamepad_path or 'auto'}")
    emit_gamepad_status()
    return jsonify({"ok": True, "selected_path": selected_gamepad_path})

@app.route("/api/control/mode", methods=["POST"])
def api_control_mode():
    global control_mode
    data = request.get_json(silent=True) or {}
    mode = str(data.get("mode", control_mode)).strip().lower()
    if mode not in ("gamepad", "keyboard"):
        return jsonify({"ok": False, "error": "Modo invalido. Use 'gamepad' ou 'keyboard'."})
    control_mode = mode
    print(f"[control] Modo: {control_mode}")
    emit_control_status()
    return jsonify({"ok": True, "control_mode": control_mode})

@app.route("/api/control/keyboard", methods=["POST"])
def api_control_keyboard():
    global keyboard_throttle, keyboard_steering, keyboard_brake, keyboard_last_update
    data = request.get_json(silent=True) or {}
    if control_mode != "keyboard":
        return jsonify({"ok": False, "error": "Modo teclado nao ativo"})
    keyboard_throttle = clamp(safe_float(data.get("throttle", 0.0)), -1.0, 1.0)
    keyboard_steering = clamp(safe_float(data.get("steering", 0.0)), -1.0, 1.0)
    keyboard_brake = bool(data.get("brake", False))
    keyboard_last_update = time.time()
    control_loop_step()
    return jsonify({"ok": True, "throttle": keyboard_throttle, "steering": keyboard_steering})

@app.route("/api/mode", methods=["POST"])
def api_set_mode():
    global VESC_MODE, can_interface_ready
    data = request.get_json(silent=True) or {}
    new_mode = str(data.get("mode", VESC_MODE)).strip().lower()
    if new_mode not in ("serial", "can"):
        return jsonify({"ok": False, "error": "Modo invalido. Use 'serial' ou 'can'."})
    old_mode = VESC_MODE
    VESC_MODE = new_mode
    with vesc_lock:
        vesc_state["mode"] = VESC_MODE
    with control_lock:
        control_state["can_ready"] = can_interface_ready
    if VESC_MODE == "serial" and old_mode != "serial":
        if PYVESC_AVAILABLE:
            t = threading.Thread(target=vesc_reader_loop, daemon=True)
            t.start()
            print("[mode] VESC reader thread iniciada (serial)")
    elif VESC_MODE == "can":
        print("[mode] Modo CAN ativado — use 'ATIVAR CAN' ou /api/can/setup")
    emit_control_status()
    emit_vesc_telemetry()
    return jsonify({"ok": True, "mode": VESC_MODE, "old_mode": old_mode})

@app.route("/api/config", methods=["POST"])
def api_set_config():
    global VESC_SERIAL_PORT, VESC_CAN_INTERFACE, VESC_CAN_BITRATE
    global VESC_CAN_LEFT_ID, VESC_CAN_RIGHT_ID, MAX_DUTY, STEERING_GAIN
    data = request.get_json(silent=True) or {}
    updated = {}
    if "serial_port" in data:
        VESC_SERIAL_PORT = str(data["serial_port"]).strip()
        updated["serial_port"] = VESC_SERIAL_PORT
    if "can_interface" in data:
        VESC_CAN_INTERFACE = validate_can_interface_name(str(data["can_interface"]))
        updated["can_interface"] = VESC_CAN_INTERFACE
    if "can_bitrate" in data:
        try:
            VESC_CAN_BITRATE = int(data["can_bitrate"])
            updated["can_bitrate"] = VESC_CAN_BITRATE
        except Exception:
            pass
    if "can_left_id" in data:
        try:
            VESC_CAN_LEFT_ID = max(0, min(255, int(data["can_left_id"])))
            updated["can_left_id"] = VESC_CAN_LEFT_ID
        except Exception:
            pass
    if "can_right_id" in data:
        try:
            VESC_CAN_RIGHT_ID = max(0, min(255, int(data["can_right_id"])))
            updated["can_right_id"] = VESC_CAN_RIGHT_ID
        except Exception:
            pass
    if "max_duty" in data:
        try:
            MAX_DUTY = clamp(float(data["max_duty"]), 0.01, 0.95)
            updated["max_duty"] = MAX_DUTY
        except Exception:
            pass
    if "steering_gain" in data:
        try:
            STEERING_GAIN = clamp(float(data["steering_gain"]), 0.0, 1.0)
            updated["steering_gain"] = STEERING_GAIN
        except Exception:
            pass
    if "motor_count" in data:
        try:
            VESC_MOTOR_COUNT = int(data["motor_count"])
            if VESC_MOTOR_COUNT not in (2, 4):
                VESC_MOTOR_COUNT = 2
            updated["motor_count"] = VESC_MOTOR_COUNT
        except Exception:
            pass
    if "can_id_3" in data:
        try:
            VESC_CAN_ID_3 = max(0, min(255, int(data["can_id_3"])))
            updated["can_id_3"] = VESC_CAN_ID_3
        except Exception:
            pass
    if "can_id_4" in data:
        try:
            VESC_CAN_ID_4 = max(0, min(255, int(data["can_id_4"])))
            updated["can_id_4"] = VESC_CAN_ID_4
        except Exception:
            pass
    print(f"[config] Atualizado: {updated}")
    return jsonify({"ok": True, "updated": updated})

# ---------------------------------------------------------------------------
# Camera
# ---------------------------------------------------------------------------
CAMERA_DEVICE = int(os.getenv("CAMERA_DEVICE", "0"))
CAMERA_WIDTH = int(os.getenv("CAMERA_WIDTH", "640"))
CAMERA_HEIGHT = int(os.getenv("CAMERA_HEIGHT", "480"))
CAMERA_FPS_TARGET = int(os.getenv("CAMERA_FPS", "15"))

def list_camera_devices():
    devices = []
    for i in range(10):
        path = f"/dev/video{i}"
        if os.path.exists(path):
            label = path
            if CAMERA_AVAILABLE and cv2 is not None:
                try:
                    cap = cv2.VideoCapture(path, cv2.CAP_V4L2)
                    if cap.isOpened():
                        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                        h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                        cap.release()
                        if w > 0 and h > 0:
                            label = f"{path} ({w}x{h})"
                            devices.append({"index": i, "path": path, "label": label})
                    else:
                        cap.release()
                except Exception:
                    pass
            if not any(d["index"] == i for d in devices):
                devices.append({"index": i, "path": path, "label": f"{path} (nao captura)"})
    return devices

def camera_loop():
    while True:
        if not CAMERA_AVAILABLE:
            with camera_lock:
                camera_state["error"] = "OpenCV nao disponivel. pip install opencv-python"
            emit_camera_status()
            time.sleep(10)
            continue
        with camera_lock:
            active = camera_state.get("active")
        if not active:
            time.sleep(1)
            continue
        cap = None
        writer = None
        current_device = CAMERA_DEVICE
        try:
            with camera_lock:
                current_device = CAMERA_DEVICE
            print(f"[camera] Abrindo /dev/video{current_device}...")
            cap = cv2.VideoCapture(f"/dev/video{current_device}", cv2.CAP_V4L2)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
            cap.set(cv2.CAP_PROP_FPS, CAMERA_FPS_TARGET)
            if not cap.isOpened():
                with camera_lock:
                    camera_state["active"] = False
                    camera_state["error"] = f"Nao foi possivel abrir /dev/video{current_device}"
                emit_camera_status()
                print(f"[camera] ERRO: nao abriu /dev/video{current_device}")
                time.sleep(3)
                continue
            w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            real_fps = cap.get(cv2.CAP_PROP_FPS)
            with camera_lock:
                camera_state["width"] = w
                camera_state["height"] = h
                camera_state["error"] = None
            emit_camera_status()
            print(f"[camera] Aberto /dev/video{current_device} {w}x{h} @ {real_fps:.1f}fps")
            fps_frame_count = 0
            fps_t0 = time.time()
            read_failures = 0
            while True:
                with camera_lock:
                    if not camera_state.get("active"):
                        print(f"[camera] Desligando...")
                        break
                    if CAMERA_DEVICE != current_device:
                        print(f"[camera] Device trocado de {current_device} para {CAMERA_DEVICE}, reiniciando...")
                        break
                    recording = camera_state.get("recording")
                    should_record = recording and writer is None
                    should_stop_record = not recording and writer is not None
                ret, frame = cap.read()
                if not ret or frame is None:
                    read_failures += 1
                    if read_failures == 1:
                        print(f"[camera] cap.read() falhou (dispositivo pode nao ser camera)")
                    if read_failures >= 50:
                        with camera_lock:
                            camera_state["active"] = False
                            camera_state["error"] = f"/dev/video{current_device} nao retorna frames (codec?)"
                        emit_camera_status()
                        print(f"[camera] ERRO: {camera_state['error']}")
                        break
                    time.sleep(0.1)
                    continue
                read_failures = 0
                if should_record:
                    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
                    fname = f"{ts}.mp4"
                    fpath = os.path.join(RECORDINGS_DIR, fname)
                    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
                    writer = cv2.VideoWriter(fpath, fourcc, CAMERA_FPS_TARGET, (w, h))
                    if writer.isOpened():
                        with camera_lock:
                            camera_state["current_file"] = fname
                        emit_camera_status()
                        print(f"[camera] Gravando: {fname}")
                    else:
                        with camera_lock:
                            camera_state["recording"] = False
                            camera_state["error"] = "Falha ao criar VideoWriter"
                        emit_camera_status()
                if should_stop_record and writer is not None:
                    writer.release()
                    writer = None
                    with camera_lock:
                        camera_state["current_file"] = None
                    emit_camera_status()
                    print(f"[camera] Gravacao parada")
                if writer is not None:
                    writer.write(frame)
                _, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 60])
                b64 = base64.b64encode(jpeg).decode("ascii")
                socketio.emit("camera_frame", {
                    "data": b64,
                    "recording": recording,
                    "width": w, "height": h,
                })
                fps_frame_count += 1
                now = time.time()
                elapsed = now - fps_t0
                if elapsed >= 2.0:
                    with camera_lock:
                        camera_state["fps"] = round(fps_frame_count / elapsed, 1)
                    emit_camera_status()
                    fps_frame_count = 0
                    fps_t0 = now
                time.sleep(1.0 / max(CAMERA_FPS_TARGET, 5))
        except Exception as e:
            with camera_lock:
                camera_state["active"] = False
                camera_state["error"] = str(e)
            emit_camera_status()
            print(f"[camera] EXCECAO: {e}")
        finally:
            if writer is not None:
                try:
                    writer.release()
                except Exception:
                    pass
            if cap is not None:
                try:
                    cap.release()
                except Exception:
                    pass
            with camera_lock:
                camera_state["active"] = False
            time.sleep(2)

@app.route("/api/camera/device", methods=["POST"])
def api_camera_set_device():
    global CAMERA_DEVICE
    data = request.get_json(silent=True) or {}
    try:
        dev = int(data.get("device", CAMERA_DEVICE))
    except (ValueError, TypeError):
        return jsonify({"ok": False, "error": "device deve ser numero inteiro"})
    if not os.path.exists(f"/dev/video{dev}"):
        return jsonify({"ok": False, "error": f"/dev/video{dev} nao existe"})
    CAMERA_DEVICE = dev
    with camera_lock:
        camera_state["active"] = False
    emit_camera_status()
    print(f"[camera] Device selecionado: /dev/video{dev}")
    return jsonify({"ok": True, "device": dev, "path": f"/dev/video{dev}"})

@app.route("/api/camera/on", methods=["POST"])
def api_camera_on():
    with camera_lock:
        if not CAMERA_AVAILABLE:
            return jsonify({"ok": False, "error": camera_state.get("camera_import_error", "OpenCV indisponivel")})
        camera_state["active"] = True
    emit_camera_status()
    return jsonify({"ok": True, "active": True})

@app.route("/api/camera/off", methods=["POST"])
def api_camera_off():
    with camera_lock:
        camera_state["active"] = False
        camera_state["recording"] = False
    emit_camera_status()
    return jsonify({"ok": True, "active": False})

@app.route("/api/camera/record/toggle", methods=["POST"])
def api_camera_record_toggle():
    with camera_lock:
        current = camera_state.get("recording", False)
        camera_state["recording"] = not current
        new_state = camera_state["recording"]
    emit_camera_status()
    return jsonify({"ok": True, "recording": new_state})

@app.route("/api/camera/recordings", methods=["GET"])
def api_camera_recordings():
    files = []
    try:
        for fname in sorted(os.listdir(RECORDINGS_DIR), reverse=True):
            if fname.endswith(".mp4"):
                fpath = os.path.join(RECORDINGS_DIR, fname)
                st = os.stat(fpath)
                files.append({
                    "name": fname,
                    "size_mb": round(st.st_size / (1024 * 1024), 2),
                    "date": datetime.fromtimestamp(st.st_mtime).strftime("%Y-%m-%d %H:%M:%S"),
                })
    except Exception:
        pass
    return jsonify({"ok": True, "files": files})

@app.route("/api/camera/recording/<path:filename>")
def api_camera_serve_recording(filename):
    from flask import send_file
    fpath = os.path.join(RECORDINGS_DIR, os.path.basename(filename))
    if not os.path.exists(fpath):
        return jsonify({"ok": False, "error": "Arquivo nao encontrado"}), 404
    return send_file(fpath, mimetype="video/mp4")

# ---------------------------------------------------------------------------
# Socket.IO
# ---------------------------------------------------------------------------
@socketio.on("connect")
def socket_connect():
    emit_gamepad_status()
    emit_vesc_telemetry()
    emit_control_status()
    emit_camera_status()

# ---------------------------------------------------------------------------
# HTML Template
# ---------------------------------------------------------------------------
HTML_TEMPLATE = r"""
<!doctype html>
<html lang="pt-BR">
<head>
    <meta charset="utf-8">
    <title>VESC Controller</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <script src="https://cdn.socket.io/4.7.5/socket.io.min.js"></script>
    <style>
        :root {
            --bg: #0f172a; --panel: #111827; --border: #334155;
            --text: #e5e7eb; --muted: #94a3b8;
            --green: #22c55e; --red: #ef4444; --yellow: #eab308; --blue: #3b82f6;
        }
        * { box-sizing:border-box; margin:0; padding:0 }
        body {
            font-family: Arial, Helvetica, sans-serif;
            background: radial-gradient(circle at top,#182235 0,#0f172a 50%,#05070a 100%);
            color: var(--text); min-height:100vh;
        }
        header {
            padding: 12px 20px; background: rgba(8,11,16,0.9);
            backdrop-filter: blur(10px); border-bottom:1px solid var(--border);
            display:flex; align-items:center; justify-content:space-between;
            position:sticky; top:0; z-index:10;
        }
        header h1 { font-size:20px; }
        .dot { width:10px; height:10px; border-radius:50%; display:inline-block; margin-right:6px; }
        .dot.ok { background:var(--green); box-shadow:0 0 12px var(--green); }
        .dot.err { background:var(--red); box-shadow:0 0 12px var(--red); }
        .dot.warn { background:var(--yellow); box-shadow:0 0 12px var(--yellow); }
        .badge {
            padding:4px 10px; border-radius:20px; font-size:12px; font-weight:bold;
            border:1px solid var(--border); background:#020617;
        }
        .badge.can { border-color:var(--blue); color:var(--blue); }
        .badge.serial { border-color:var(--green); color:var(--green); }
        main {
            display:grid; grid-template-columns:1fr 1fr; gap:16px;
            padding:16px; max-width:1200px; margin:0 auto;
        }
        .card {
            background: var(--panel); border:1px solid var(--border);
            border-radius:14px; padding:16px; box-shadow:0 8px 24px rgba(0,0,0,0.2);
        }
        .card h2 { font-size:16px; margin-bottom:12px; color:var(--muted); text-transform:uppercase; letter-spacing:0.05em; }
        .telemetry-grid { display:grid; grid-template-columns:repeat(3,1fr); gap:10px; }
        .telemetry-item {
            background:#020617; border:1px solid var(--border); border-radius:10px;
            padding:12px; text-align:center;
        }
        .telemetry-item .value { font-size:24px; font-weight:800; }
        .telemetry-item .label { font-size:11px; color:var(--muted); text-transform:uppercase; letter-spacing:0.08em; }
        .motor-bars { margin-top:12px; }
        .motor-row { display:grid; grid-template-columns:80px 1fr 60px; gap:8px; align-items:center; margin-bottom:8px; }
        .motor-row .name { font-size:13px; font-weight:bold; }
        .motor-row .duty-val { font-size:13px; text-align:right; font-family:monospace; }
        .bar-track {
            height:22px; background:#020617; border:1px solid var(--border);
            border-radius:12px; overflow:hidden; position:relative;
        }
        .bar-zero { position:absolute; left:50%; top:0; width:1px; height:100%; background:var(--muted); }
        .bar-fill { position:absolute; top:0; height:100%; border-radius:2px; transition:width 0.08s linear; }
        .bar-fill.pos { left:50%; background:var(--green); }
        .bar-fill.neg { right:50%; background:var(--yellow); }
        .btn {
            padding:12px; border-radius:10px; border:none; cursor:pointer;
            font-weight:bold; font-size:14px; width:100%; margin-top:8px; transition:filter 0.15s;
        }
        .btn:hover { filter:brightness(1.15); }
        .btn-arm { background:var(--green); color:#000; }
        .btn-disarm { background:#475569; }
        .btn-emergency { background:#b91c1c; font-size:16px; }
        .btn-can { background:var(--blue); }
        .btn-save { background:#a16207; }
        .status-row { display:flex; gap:10px; align-items:center; margin-bottom:8px; font-size:14px; }
        .config-row { display:grid; grid-template-columns:120px 1fr; gap:6px; align-items:center; margin-bottom:6px; }
        .config-row label { font-size:12px; color:var(--muted); }
        .config-row input, .config-row select {
            padding:8px; border-radius:6px; border:1px solid var(--border);
            background:#020617; color:var(--text); font-size:13px; width:100%;
        }
        .config-section { margin-top:10px; padding-top:10px; border-top:1px solid var(--border); }
        .config-section h3 { font-size:13px; color:var(--muted); margin-bottom:8px; }
        .gamepad-panel {
            display:grid; grid-template-columns:1fr 1fr; gap:8px;
        }
        .gp-section {
            background:radial-gradient(circle at top,#1e293b 0,#020617 72%);
            border:1px solid #334155; border-radius:10px; padding:8px;
        }
        .gp-section h3 { font-size:11px; color:var(--muted); margin-bottom:4px; font-weight:normal; }
        .stick-area {
            width:100px; height:100px; border-radius:999px; position:relative; margin:0 auto;
            background: linear-gradient(90deg,transparent 49%,rgba(148,163,184,0.2) 50%,transparent 51%),
                        linear-gradient(0deg, transparent 49%,rgba(148,163,184,0.2) 50%,transparent 51%),
                        radial-gradient(circle,#0f172a 0,#020617 68%);
            border:2px solid #475569; box-shadow:inset 0 0 18px rgba(0,0,0,0.3);
        }
        .stick-dot {
            position:absolute; width:22px; height:22px; border-radius:999px;
            left:50%; top:50%; transform:translate(-50%,-50%);
            background: radial-gradient(circle at top left,#f8fafc 0,#3b82f6 30%,#075985 100%);
            border:2px solid #bae6fd; box-shadow:0 0 14px rgba(59,130,246,0.6);
            transition: left 0.06s linear, top 0.06s linear;
        }
        .axis-row { display:flex; justify-content:space-between; align-items:center; padding:4px 0; font-size:12px; }
        .axis-row .axis-name { color:var(--muted); width:80px; }
        .axis-row .axis-bar {
            flex:1; height:14px; background:#020617; border:1px solid var(--border);
            border-radius:8px; overflow:hidden; position:relative; margin:0 8px;
        }
        .axis-fill {
            position:absolute; top:0; height:100%; transition:width 0.06s linear;
            background:var(--blue); border-radius:2px;
        }
        .deadman-indicator {
            padding:10px; border-radius:10px; text-align:center; font-weight:bold; font-size:13px;
        }
        .deadman-indicator.ok { background:rgba(34,197,94,0.2); border:1px solid var(--green); color:var(--green); }
        .deadman-indicator.off { background:rgba(239,68,68,0.2); border:1px solid var(--red); color:var(--red); }
        .info-row { font-size:12px; color:var(--muted); margin-top:4px; }
        .camera-feed { width:100%; border-radius:10px; border:1px solid var(--border); background:#020617; }
        .camera-controls { display:flex; gap:8px; margin-top:8px; }
        .camera-controls .btn { flex:1; font-size:12px; padding:8px; }
        .btn-record { background:var(--red); }
        .btn-record.active { background:var(--yellow); color:#000; }
        .recordings-list { margin-top:8px; max-height:120px; overflow-y:auto; }
        .recordings-list .file-item {
            display:flex; justify-content:space-between; align-items:center;
            padding:6px 8px; background:#020617; border:1px solid var(--border);
            border-radius:6px; margin-bottom:4px; font-size:12px;
        }
        .file-item .file-name { font-family:monospace; color:var(--blue); }
        .file-item .file-info { color:var(--muted); }
        @media(max-width:800px) {
            main { grid-template-columns:1fr; }
            .telemetry-grid { grid-template-columns:repeat(2,1fr); }
        }
    </style>
</head>
<body>
<header>
    <div>
        <h1>VESC Controller</h1>
        <span class="badge {{ 'can' if vesc_mode == 'can' else 'serial' }}">
            MODO: {{ vesc_mode.upper() }}
        </span>
    </div>
    <div style="display:flex;gap:16px;align-items:center;font-size:13px;">
        <span><span id="dotGamepad" class="dot err"></span> Gamepad</span>
        <span><span id="dotVesc" class="dot err"></span> VESC</span>
        <span><span id="dotArmed" class="dot err"></span> Armado</span>
    </div>
</header>
<main>
    <!-- COLUNA ESQUERDA: Camera + Telemetria + Motor -->
    <div>
        <div class="card" id="cameraCard" style="display:none">
            <h2>Camera</h2>
            <div class="config-row" style="margin-bottom:8px;">
                <label>Dispositivo:</label>
                <select id="cameraDeviceSelect" onchange="selectCameraDevice()">
                    <option value="0">/dev/video0</option>
                </select>
            </div>
            <img id="cameraFeed" class="camera-feed" src="" alt="Camera feed" style="display:none">
            <div class="camera-controls">
                <button id="btnCameraOn" class="btn btn-arm" onclick="cameraOn()" style="flex:1">LIGAR CAMERA</button>
                <button id="btnCameraOff" class="btn btn-disarm" onclick="cameraOff()" style="flex:1;display:none">DESLIGAR</button>
                <button id="btnRecord" class="btn btn-record" onclick="toggleRecord()" style="flex:1;display:none">GRAVAR</button>
            </div>
            <div class="info-row" id="cameraInfo"></div>
            <div class="recordings-list" id="recordingsList"></div>
        </div>
        <div class="card">
            <h2>Telemetria VESC</h2>
            <div class="telemetry-grid">
                <div class="telemetry-item"><span class="value" id="valVoltage">--</span><span class="label">Tensao (V)</span></div>
                <div class="telemetry-item"><span class="value" id="valCurrentIn">--</span><span class="label">Corr. Entrada (A)</span></div>
                <div class="telemetry-item"><span class="value" id="valCurrentMotor">--</span><span class="label">Corr. Motor (A)</span></div>
                <div class="telemetry-item"><span class="value" id="valRPM">--</span><span class="label">RPM</span></div>
                <div class="telemetry-item"><span class="value" id="valDuty">--</span><span class="label">Duty (%)</span></div>
                <div class="telemetry-item"><span class="value" id="valPower">--</span><span class="label">Potencia (W)</span></div>
                <div class="telemetry-item"><span class="value" id="valTempFET">--</span><span class="label">Temp FET (°C)</span></div>
                <div class="telemetry-item"><span class="value" id="valTempMotor">--</span><span class="label">Temp Motor (°C)</span></div>
                <div class="telemetry-item"><span class="value" id="valFault">--</span><span class="label">Fault</span></div>
            </div>
            <div class="motor-bars">
                <div class="motor-row" id="motorRow1">
                    <span class="name">M1 (E)</span>
                    <div class="bar-track"><div class="bar-zero"></div>
                        <div class="bar-fill pos" id="barM1Pos" style="width:0%"></div>
                        <div class="bar-fill neg" id="barM1Neg" style="width:0%"></div>
                    </div><span class="duty-val" id="dutyM1">0%</span>
                </div>
                <div class="motor-row" id="motorRow2">
                    <span class="name">M2 (D)</span>
                    <div class="bar-track"><div class="bar-zero"></div>
                        <div class="bar-fill pos" id="barM2Pos" style="width:0%"></div>
                        <div class="bar-fill neg" id="barM2Neg" style="width:0%"></div>
                    </div><span class="duty-val" id="dutyM2">0%</span>
                </div>
                <div class="motor-row" id="motorRow3" style="display:none">
                    <span class="name">M3 (TE)</span>
                    <div class="bar-track"><div class="bar-zero"></div>
                        <div class="bar-fill pos" id="barM3Pos" style="width:0%"></div>
                        <div class="bar-fill neg" id="barM3Neg" style="width:0%"></div>
                    </div><span class="duty-val" id="dutyM3">0%</span>
                </div>
                <div class="motor-row" id="motorRow4" style="display:none">
                    <span class="name">M4 (TD)</span>
                    <div class="bar-track"><div class="bar-zero"></div>
                        <div class="bar-fill pos" id="barM4Pos" style="width:0%"></div>
                        <div class="bar-fill neg" id="barM4Neg" style="width:0%"></div>
                    </div><span class="duty-val" id="dutyM4">0%</span>
                </div>
            </div>
            <div class="info-row" id="vescInfo"></div>
        </div>
    </div>
    <!-- COLUNA DIREITA: Gamepad + Controle -->
    <div>
        <div class="card">
            <h2>Gamepad</h2>
            <div style="margin-bottom:10px;">
                <select id="gamepadSelect" onchange="selectGamepad()" style="width:100%;padding:8px;border-radius:6px;border:1px solid var(--border);background:#020617;color:var(--text);font-size:13px;">
                    <option value="">Auto (primeiro gamepad)</option>
                </select>
            </div>
            <div style="margin-bottom:10px;">
                <select id="controlModeSelect" onchange="switchControlMode()" style="width:100%;padding:8px;border-radius:6px;border:1px solid var(--border);background:#020617;color:var(--text);font-size:13px;">
                    <option value="gamepad">Gamepad (evdev)</option>
                    <option value="keyboard">Teclado (setinhas)</option>
                </select>
            </div>
            <div id="keyboardHelp" style="display:none;font-size:11px;color:var(--muted);text-align:center;margin-bottom:8px;">
                Setas/WASD = mover | Espaco = freio
            </div>
            <div class="gamepad-panel">
                <div class="gp-section">
                    <h3>Stick Esquerdo</h3>
                    <div class="stick-area">
                        <div class="stick-dot" id="stickLeft"></div>
                    </div>
                    <div style="text-align:center;font-size:10px;color:var(--muted);margin-top:4px;">
                        <span id="stickLeftVal">X: 0.00 Y: 0.00</span>
                    </div>
                </div>
                <div class="gp-section">
                    <h3>Stick Direito</h3>
                    <div class="stick-area">
                        <div class="stick-dot" id="stickRight"></div>
                    </div>
                    <div style="text-align:center;font-size:10px;color:var(--muted);margin-top:4px;">
                        <span id="stickRightVal">X: 0.00 Y: 0.00</span>
                    </div>
                </div>
            </div>
            <div style="margin-top:8px;">
                <div class="axis-row">
                    <span class="axis-name">Acelerador</span>
                    <div class="axis-bar"><div class="axis-fill" id="throttleBar" style="width:50%;left:50%"></div></div>
                </div>
                <div class="axis-row">
                    <span class="axis-name">Direcao</span>
                    <div class="axis-bar"><div class="axis-fill" id="steeringBar" style="width:50%;left:50%"></div></div>
                </div>
            </div>
            <div class="deadman-indicator off" id="deadmanIndicator">HOMEM-MORTO: SOLTO</div>
            <div class="info-row" id="gamepadInfo"></div>
        </div>
        <div class="card">
            <h2>Controle</h2>
            <div class="status-row">
                <span>Modo:</span>
                <select id="modeSelect" onchange="switchMode()" style="padding:6px;border-radius:6px;border:1px solid var(--border);background:#020617;color:var(--text);font-size:13px;">
                    <option value="serial" {{ 'selected' if vesc_mode == 'serial' else '' }}>Serial (USB) - 1 VESC</option>
                    <option value="can" {{ 'selected' if vesc_mode == 'can' else '' }}>CAN - 2 VESCs</option>
                </select>
            </div>
            <div id="configSerial" style="{{ 'display:none' if vesc_mode != 'serial' else '' }}">
                <div class="config-section"><h3>Config Serial</h3>
                    <div class="config-row">
                        <label>Porta USB:</label>
                        <select id="serialPortSelect" onchange="saveSerialPort()">
                            <option value="{{ vesc_serial_port }}">{{ vesc_serial_port }} (atual)</option>
                        </select>
                    </div>
                </div>
            </div>
            <div id="configCAN" style="{{ 'display:none' if vesc_mode != 'can' else '' }}">
                <div class="config-section"><h3>Config CAN</h3>
                    <div class="config-row">
                        <label>Motores:</label>
                        <select id="motorCount" onchange="saveCANConfig()">
                            <option value="2">2 motores</option>
                            <option value="4">4 motores</option>
                        </select>
                    </div>
                    <div class="config-row">
                        <label>Interface:</label>
                        <input id="canInterface" value="{{ vesc_can_interface }}" onchange="saveCANConfig()">
                    </div>
                    <div class="config-row">
                        <label>Bitrate:</label>
                        <select id="canBitrate" onchange="saveCANConfig()">
                            {% for br in [125000, 250000, 500000, 1000000] %}
                            <option value="{{ br }}" {{ 'selected' if br == vesc_can_bitrate else '' }}>{{ br // 1000 }}k</option>
                            {% endfor %}
                        </select>
                    </div>
                    <div class="config-row">
                        <label>ID M1 (F.Esq):</label>
                        <input id="canLeftId" type="number" min="0" max="255" value="{{ vesc_can_left_id }}" onchange="saveCANConfig()">
                    </div>
                    <div class="config-row">
                        <label>ID M2 (F.Dir):</label>
                        <input id="canRightId" type="number" min="0" max="255" value="{{ vesc_can_right_id }}" onchange="saveCANConfig()">
                    </div>
                    <div class="config-row" id="id3row" style="display:none">
                        <label>ID M3 (T.Esq):</label>
                        <input id="canId3" type="number" min="0" max="255" value="3" onchange="saveCANConfig()">
                    </div>
                    <div class="config-row" id="id4row" style="display:none">
                        <label>ID M4 (T.Dir):</label>
                        <input id="canId4" type="number" min="0" max="255" value="4" onchange="saveCANConfig()">
                    </div>
                    <button class="btn btn-can" onclick="setupCAN()">ATIVAR INTERFACE CAN</button>
                </div>
            </div>
            <button class="btn btn-arm" onclick="arm()">ARMAR ROBO</button>
            <button class="btn btn-disarm" onclick="disarm()">DESARMAR</button>
            <button class="btn btn-emergency" onclick="emergencyStop()">PARADA DE EMERGENCIA</button>
            <div class="info-row" style="margin-top:8px;" id="controlInfo"></div>
        </div>
    </div>
</main>
<script>
const MAX_DUTY = {{ max_duty }};
const socket = io();

// --- Socket handlers ---
socket.on("gamepad_status", function(data) {
    document.getElementById("dotGamepad").className = "dot " + (data.connected ? "ok" : "err");
    document.getElementById("gamepadInfo").textContent =
        data.connected ? (data.device_name || data.device_path) : (data.error || "Desconectado");
    var sel = document.getElementById("gamepadSelect");
    if (data.selected_path) {
        for (var i = 0; i < sel.options.length; i++) {
            sel.options[i].selected = (sel.options[i].value === data.selected_path);
        }
    } else {
        sel.value = "";
    }
});

socket.on("vesc_telemetry", function(data) {
    document.getElementById("dotVesc").className = "dot " + (data.connected ? "ok" : "err");
    document.getElementById("vescInfo").textContent =
        data.connected ? ("Firmware: " + (data.firmware || "?")) : (data.error || "Desconectado");

    var d = data.data || {};
    document.getElementById("valVoltage").textContent = fmt(d.v_in, 1);
    document.getElementById("valCurrentIn").textContent = fmt(d.avg_input_current, 2);
    document.getElementById("valCurrentMotor").textContent = fmt(d.avg_motor_current, 2);
    document.getElementById("valRPM").textContent = fmt(d.rpm_abs || d.rpm, 0);
    document.getElementById("valDuty").textContent = fmt(d.duty_percent, 1) + "%";
    document.getElementById("valPower").textContent = fmt(d.input_power_w, 1);
    document.getElementById("valTempFET").textContent = fmt(d.temp_fet, 1) + "°";
    document.getElementById("valTempMotor").textContent = fmt(d.temp_motor, 1) + "°";
    var faultName = d.fault_name || "NONE";
    var faultNum = d.fault_number;
    var faultEl = document.getElementById("valFault");
    faultEl.textContent = faultName;
    faultEl.style.color = (faultNum && faultNum != 0) ? "var(--red)" : "var(--green)";
});

socket.on("control_status", function(data) {
    document.getElementById("dotArmed").className = "dot " + (data.armed ? "ok" : "err");
    var maxD = MAX_DUTY || 0.25;
    updateMotorBar("M1", data.left_duty || 0, maxD);
    updateMotorBar("M2", data.right_duty || 0, maxD);
    updateMotorBar("M3", data.left_duty || 0, maxD);
    updateMotorBar("M4", data.right_duty || 0, maxD);
    var deadman = document.getElementById("deadmanIndicator");
    if (data.deadman_ok) {
        deadman.className = "deadman-indicator ok";
        deadman.textContent = "HOMEM-MORTO: SEGURO";
    } else {
        deadman.className = "deadman-indicator off";
        deadman.textContent = "HOMEM-MORTO: SOLTO";
    }
    document.getElementById("controlInfo").textContent =
        "Throttle: " + fmt(data.throttle, 2) + " | Steering: " + fmt(data.steering, 2) +
        " | Armed: " + (data.armed ? "SIM" : "NAO") +
        " | Motores: " + (document.getElementById("motorCount") ? document.getElementById("motorCount").value : "2");
});

socket.on("gamepad_event", function(data) {
    console.log("gamepad_event", data.kind, data.code, data.normalized);
    if (data.kind === "axis") {
        if (data.code === "ABS_X") updateStick("stickLeft", data.normalized, 0);
        if (data.code === "ABS_Y") updateStick("stickLeft", null, -data.normalized);
        if (data.code === "ABS_RX") updateStick("stickRight", data.normalized, 0);
        if (data.code === "ABS_RY") updateStick("stickRight", null, -data.normalized);
        if (data.code === "{{ throttle_axis }}") updateThrottleBar(data.normalized);
        if (data.code === "{{ steering_axis }}") updateSteeringBar(data.normalized);
        if (["ABS_HAT0X","ABS_HAT0Y","ABS_Z","ABS_RZ"].indexOf(data.code) >= 0) {
            document.getElementById("gamepadInfo").textContent = data.code + ": " + data.normalized.toFixed(2);
        }
    }
});

// --- UI helpers ---
function fmt(v, d) { if (v == null || v == undefined) return "--"; return Number(v).toFixed(d); }

function updateMotorBar(side, duty, maxD) {
    var pct = Math.abs(duty) / (maxD || 0.01) * 50;
    pct = Math.min(pct, 50);
    var posId = "bar" + side + "Pos", negId = "bar" + side + "Neg";
    var valId = "duty" + side;
    document.getElementById(posId).style.width = (duty > 0 ? pct : 0) + "%";
    document.getElementById(negId).style.width = (duty < 0 ? pct : 0) + "%";
    document.getElementById(valId).textContent = (duty * 100).toFixed(1) + "%";
}

var stickState = { stickLeft: {x:0,y:0}, stickRight: {x:0,y:0} };

function updateStick(id, x, y) {
    if (x != null) stickState[id].x = x;
    if (y != null) stickState[id].y = y;
    var s = stickState[id];
    var el = document.getElementById(id);
    el.style.left = (50 + s.x * 40) + "%";
    el.style.top = (50 + s.y * 40) + "%";
    var labelId = id + "Val";
    var label = document.getElementById(labelId);
    if (label) label.textContent = "X: " + s.x.toFixed(2) + " Y: " + s.y.toFixed(2);
}

var lastThrottle = 0;
function updateThrottleBar(v) {
    lastThrottle = v;
    var bar = document.getElementById("throttleBar");
    var pct = Math.abs(v) * 50;
    bar.style.width = pct + "%";
    bar.style.left = v >= 0 ? "50%" : (50 - pct) + "%";
    bar.style.background = v >= 0 ? "var(--green)" : "var(--yellow)";
}

var lastSteering = 0;
function updateSteeringBar(v) {
    lastSteering = v;
    var bar = document.getElementById("steeringBar");
    var pct = Math.abs(v) * 50;
    bar.style.width = pct + "%";
    bar.style.left = v >= 0 ? "50%" : (50 - pct) + "%";
    bar.style.background = v >= 0 ? "var(--blue)" : "#a855f7";
}

// --- Control actions ---
function arm() {
    fetch("/api/arm", {method:"POST"}).then(r=>r.json()).then(d=>console.log("arm:",d));
}
function disarm() {
    fetch("/api/disarm", {method:"POST"}).then(r=>r.json()).then(d=>console.log("disarm:",d));
}
function emergencyStop() {
    fetch("/api/emergency-stop", {method:"POST"}).then(r=>r.json()).then(d=>console.log("emergency:",d));
}
function setupCAN() {
    saveCANConfig();
    fetch("/api/can/setup", {method:"POST"}).then(r=>r.json()).then(d=>{
        document.getElementById("controlInfo").textContent =
            d.ok ? ("CAN " + d.interface + " @ " + d.bitrate + " ativado") : ("Erro CAN: " + (d.error || "?"));
    });
}
function switchMode() {
    var mode = document.getElementById("modeSelect").value;
    fetch("/api/mode", {method:"POST", headers:{"Content-Type":"application/json"}, body:JSON.stringify({mode:mode})})
    .then(r=>r.json()).then(d=>{
        console.log("mode switch:", d);
        document.getElementById("configSerial").style.display = mode === "serial" ? "" : "none";
        document.getElementById("configCAN").style.display = mode === "can" ? "" : "none";
        if (mode === "serial") loadSerialPorts();
        if (d.ok) document.getElementById("controlInfo").textContent = "Modo alterado para " + mode.toUpperCase();
    });
}
function saveSerialPort() {
    var port = document.getElementById("serialPortSelect").value;
    fetch("/api/config", {method:"POST", headers:{"Content-Type":"application/json"}, body:JSON.stringify({serial_port:port})})
    .then(r=>r.json()).then(d=>console.log("config:", d));
}
function saveCANConfig() {
    var data = {
        motor_count: parseInt(document.getElementById("motorCount").value),
        can_interface: document.getElementById("canInterface").value,
        can_bitrate: parseInt(document.getElementById("canBitrate").value),
        can_left_id: parseInt(document.getElementById("canLeftId").value),
        can_right_id: parseInt(document.getElementById("canRightId").value),
        can_id_3: parseInt(document.getElementById("canId3").value),
        can_id_4: parseInt(document.getElementById("canId4").value),
    };
    var mc = data.motor_count;
    document.getElementById("motorRow3").style.display = mc >= 4 ? "" : "none";
    document.getElementById("motorRow4").style.display = mc >= 4 ? "" : "none";
    document.getElementById("id3row").style.display = mc >= 4 ? "" : "none";
    document.getElementById("id4row").style.display = mc >= 4 ? "" : "none";
    fetch("/api/config", {method:"POST", headers:{"Content-Type":"application/json"}, body:JSON.stringify(data)})
    .then(r=>r.json()).then(d=>console.log("can config:", d));
}
async function loadSerialPorts() {
    try {
        var res = await fetch("/api/ports", {cache:"no-store"});
        var data = await res.json();
        var ports = data.serial_ports || [];
        var sel = document.getElementById("serialPortSelect");
        var current = data.current_serial_port || "";
        sel.innerHTML = "";
        if (ports.length === 0) {
            sel.innerHTML = '<option value="">Nenhuma porta encontrada</option>';
            if (current) sel.innerHTML += '<option value="' + current + '" selected>' + current + '</option>';
        } else {
            ports.forEach(function(p) {
                sel.innerHTML += '<option value="' + p + '" ' + (p === current ? "selected" : "") + '>' + p + '</option>';
            });
        }
    } catch(e) {
        console.log("loadSerialPorts error:", e);
    }
}
// Carrega portas ao iniciar
loadSerialPorts();
setInterval(loadSerialPorts, 10000);

// --- Gamepad selector ---
function selectGamepad() {
    var path = document.getElementById("gamepadSelect").value;
    fetch("/api/gamepad/select", {
        method: "POST",
        headers: {"Content-Type": "application/json"},
        body: JSON.stringify({path: path || null})
    }).then(r => r.json()).then(d => console.log("gamepad select:", d));
}
async function loadGamepadDevices() {
    try {
        var res = await fetch("/api/ports", {cache: "no-store"});
        var data = await res.json();
        var devices = data.input_devices || [];
        var selected = data.selected_gamepad_path || "";
        var sel = document.getElementById("gamepadSelect");
        var currentValue = sel.value;
        sel.innerHTML = '<option value="">Auto (primeiro gamepad)</option>';
        devices.forEach(function(dev) {
            var selAttr = (dev.path === selected || dev.path === currentValue) ? " selected" : "";
            sel.innerHTML += '<option value="' + dev.path + '"' + selAttr + '>' + dev.name + ' (' + dev.path + ')</option>';
        });
    } catch(e) { console.log("loadGamepadDevices:", e); }
}
loadGamepadDevices();
setInterval(loadGamepadDevices, 5000);

// --- Keyboard control ---
var keyState = { w: false, a: false, s: false, d: false, up: false, down: false, left: false, right: false, space: false };
var keyboardInterval = null;

function switchControlMode() {
    var mode = document.getElementById("controlModeSelect").value;
    fetch("/api/control/mode", {method:"POST", headers:{"Content-Type":"application/json"}, body:JSON.stringify({mode:mode})})
    .then(r=>r.json()).then(d=>{
        console.log("control mode:", d);
        document.getElementById("keyboardHelp").style.display = mode === "keyboard" ? "" : "none";
        if (mode === "keyboard") startKeyboard();
        else stopKeyboard();
    });
}

function sendKeyboardState() {
    var t = (keyState.w || keyState.up) ? 1.0 : ((keyState.s || keyState.down) ? -1.0 : 0.0);
    var s = (keyState.d || keyState.right) ? 1.0 : ((keyState.a || keyState.left) ? -1.0 : 0.0);
    fetch("/api/control/keyboard", {method:"POST", headers:{"Content-Type":"application/json"},
        body:JSON.stringify({throttle:t, steering:s, brake:keyState.space})})
    .then(r=>r.json()).then(d=>console.log("key:",d));
}

function startKeyboard() {
    stopKeyboard();
    keyboardInterval = setInterval(sendKeyboardState, 50);
}

function stopKeyboard() {
    if (keyboardInterval) { clearInterval(keyboardInterval); keyboardInterval = null; }
    Object.keys(keyState).forEach(function(k){ keyState[k] = false; });
}

document.addEventListener("keydown", function(e) {
    if (document.getElementById("controlModeSelect").value !== "keyboard") return;
    if (["ArrowUp","ArrowDown","ArrowLeft","ArrowRight","Space","KeyW","KeyA","KeyS","KeyD"].indexOf(e.code) >= 0) {
        e.preventDefault();
    }
    if (e.code === "ArrowUp" || e.code === "KeyW") keyState.up = keyState.w = true;
    if (e.code === "ArrowDown" || e.code === "KeyS") keyState.down = keyState.s = true;
    if (e.code === "ArrowLeft" || e.code === "KeyA") keyState.left = keyState.a = true;
    if (e.code === "ArrowRight" || e.code === "KeyD") keyState.right = keyState.d = true;
    if (e.code === "Space") keyState.space = true;
});

document.addEventListener("keyup", function(e) {
    if (document.getElementById("controlModeSelect").value !== "keyboard") return;
    if (e.code === "ArrowUp" || e.code === "KeyW") keyState.up = keyState.w = false;
    if (e.code === "ArrowDown" || e.code === "KeyS") keyState.down = keyState.s = false;
    if (e.code === "ArrowLeft" || e.code === "KeyA") keyState.left = keyState.a = false;
    if (e.code === "ArrowRight" || e.code === "KeyD") keyState.right = keyState.d = false;
    if (e.code === "Space") keyState.space = false;
    if (!keyState.w && !keyState.s && !keyState.up && !keyState.down &&
        !keyState.a && !keyState.d && !keyState.left && !keyState.right && !keyState.space) {
        sendKeyboardState();
    }
});

// --- Camera ---
document.getElementById("cameraCard").style.display = "";

socket.on("camera_frame", function(data) {
    console.log("camera_frame received, size:", data.data ? data.data.length : 0);
    var feed = document.getElementById("cameraFeed");
    feed.src = "data:image/jpeg;base64," + data.data;
    feed.style.display = "";
    document.getElementById("cameraInfo").textContent =
        (data.width || "?") + "x" + (data.height || "?") +
        " | Gravando: " + (data.recording ? "SIM" : "NAO");
});

socket.on("camera_status", function(data) {
    var onBtn = document.getElementById("btnCameraOn");
    var offBtn = document.getElementById("btnCameraOff");
    var recBtn = document.getElementById("btnRecord");
    if (data.active) {
        onBtn.style.display = "none";
        offBtn.style.display = "";
        recBtn.style.display = "";
        recBtn.textContent = data.recording ? "PARAR GRAVACAO" : "GRAVAR";
        recBtn.className = "btn btn-record" + (data.recording ? " active" : "");
    } else {
        onBtn.style.display = "";
        offBtn.style.display = "none";
        recBtn.style.display = "none";
        document.getElementById("cameraFeed").style.display = "none";
    }
    if (data.error) {
        document.getElementById("cameraInfo").textContent = "Erro: " + data.error;
    }
});

function cameraOn() {
    fetch("/api/camera/on", {method:"POST"}).then(r=>r.json()).then(d=>{
        if (!d.ok) document.getElementById("cameraInfo").textContent = "Erro: " + (d.error || "?");
    });
}
function cameraOff() {
    fetch("/api/camera/off", {method:"POST"}).then(r=>r.json());
}
function toggleRecord() {
    fetch("/api/camera/record/toggle", {method:"POST"}).then(r=>r.json()).then(d=>{
        if (d.ok) loadRecordings();
    });
}
async function loadRecordings() {
    try {
        var res = await fetch("/api/camera/recordings", {cache:"no-store"});
        var data = await res.json();
        var files = data.files || [];
        var list = document.getElementById("recordingsList");
        if (files.length === 0) {
            list.innerHTML = '<div class="file-item"><span class="file-name">Nenhuma gravacao</span></div>';
        } else {
            list.innerHTML = files.map(function(f) {
                return '<a href="/api/camera/recording/' + f.name + '" target="_blank" class="file-item" style="text-decoration:none;color:inherit;display:flex;justify-content:space-between;align-items:center;padding:6px 8px;background:#020617;border:1px solid var(--border);border-radius:6px;margin-bottom:4px;font-size:12px;">' +
                    '<span class="file-name" style="font-family:monospace;color:var(--blue);">' + f.name + '</span>' +
                    '<span class="file-info" style="color:var(--muted);">' + f.size_mb + ' MB | ' + f.date + '</span></a>';
            }).join("");
        }
    } catch(e) { console.log("loadRecordings:", e); }
}
loadRecordings();

function selectCameraDevice() {
    var dev = document.getElementById("cameraDeviceSelect").value;
    fetch("/api/camera/device", {
        method: "POST",
        headers: {"Content-Type": "application/json"},
        body: JSON.stringify({device: parseInt(dev)})
    }).then(r => r.json()).then(d => {
        console.log("camera device:", d);
        if (!d.ok) document.getElementById("cameraInfo").textContent = "Erro: " + (d.error || "?");
    });
}
async function loadCameraDevices() {
    try {
        var res = await fetch("/api/ports", {cache: "no-store"});
        var data = await res.json();
        var devices = data.camera_devices || [];
        var current = data.current_camera_device;
        var sel = document.getElementById("cameraDeviceSelect");
        if (devices.length === 0) {
            sel.innerHTML = '<option value="0">/dev/video0 (nao encontrado)</option>';
        } else {
            sel.innerHTML = devices.map(function(d) {
                return '<option value="' + d.index + '" ' + (d.index === current ? "selected" : "") + '>' + d.label + '</option>';
            }).join("");
        }
    } catch(e) { console.log("loadCameraDevices:", e); }
}
loadCameraDevices();
setInterval(loadCameraDevices, 10000);
</script>
</body>
</html>
"""

# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def print_startup():
    print("")
    print("=" * 60)
    print("  VESC Controller — Gamepad HID + VESC (Serial / CAN)")
    print("=" * 60)
    print(f"  Modo:           {VESC_MODE.upper()}")
    if VESC_MODE == "serial":
        print(f"  Porta serial:   {VESC_SERIAL_PORT}")
        print(f"  pyvesc:         {'OK' if PYVESC_AVAILABLE else 'INDISPONIVEL'}")
    else:
        print(f"  Interface CAN:  {VESC_CAN_INTERFACE}")
        print(f"  Bitrate:        {VESC_CAN_BITRATE}")
        print(f"  Left ID:        {VESC_CAN_LEFT_ID}")
        print(f"  Right ID:       {VESC_CAN_RIGHT_ID}")
    print(f"  max_duty:       {MAX_DUTY}")
    print(f"  steering_gain:  {STEERING_GAIN}")
    print(f"  deadman:        {DEADMAN_BUTTON}")
    print(f"  brake:          {BRAKE_BUTTON} ({BRAKE_CURRENT}A)")
    print(f"  evdev:          {'OK' if EVDEV_AVAILABLE else 'INDISPONIVEL'}")
    print("")
    print(f"  Acessar:        http://0.0.0.0:5009")
    print("=" * 60)
    print("")

if __name__ == "__main__":
    print_startup()
    gp_thread = threading.Thread(target=gamepad_reader_loop, daemon=True)
    gp_thread.start()
    vesc_thread = threading.Thread(target=vesc_reader_loop, daemon=True)
    vesc_thread.start()
    cam_thread = threading.Thread(target=camera_loop, daemon=True)
    cam_thread.start()
    socketio.run(app, host="0.0.0.0", port=5009, allow_unsafe_werkzeug=True)
