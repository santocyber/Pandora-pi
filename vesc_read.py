import os
import time
import glob
import math
import json
import shutil
import socket
import serial
import threading
import queue
import xml.etree.ElementTree as ET
from pathlib import Path
from collections import deque
from datetime import datetime

from flask import Flask, jsonify, render_template_string, request
from werkzeug.utils import secure_filename
import pyvesc
from pyvesc import VESC

try:
    from pyvesc.VESC.messages import SetCurrentBrake
except Exception:
    from pyvesc.VESC.messages.setters import SetCurrentBrake


APP_HOST = "0.0.0.0"
APP_PORT = 5008

DEFAULT_VESC_PORT = os.getenv("VESC_PORT", "/dev/ttyACM0")
VESC_PORT = DEFAULT_VESC_PORT

selected_serial_port = {
    "port": DEFAULT_VESC_PORT,
    "auto": True,
    "last_scan": None,
    "last_error": None,
}
READ_INTERVAL = float(os.getenv("VESC_INTERVAL", "0.25"))
HISTORY_LIMIT = int(os.getenv("VESC_HISTORY_LIMIT", "600"))

REAL_CONFIG_DIR = Path(os.getenv("VESC_REAL_CONFIG_DIR", "vesc_real_configs"))
REAL_CONFIG_UPLOAD_DIR = REAL_CONFIG_DIR / "uploads"
REAL_APP_CONFIG_FILE = REAL_CONFIG_DIR / "app_config.xml"
REAL_MOTOR_CONFIG_FILE = REAL_CONFIG_DIR / "motor_config.xml"
REAL_CONFIG_DIR.mkdir(parents=True, exist_ok=True)
REAL_CONFIG_UPLOAD_DIR.mkdir(parents=True, exist_ok=True)

TCP_BRIDGE_HOST = os.getenv("VESC_TCP_HOST", "0.0.0.0")
TCP_BRIDGE_DEFAULT_PORT = int(os.getenv("VESC_TCP_PORT", "65102"))
TCP_BRIDGE_BAUD = int(os.getenv("VESC_BAUD", "115200"))
TCP_BUFFER_SIZE = int(os.getenv("VESC_TCP_BUFFER", "4096"))

MOTOR_TEST_MAX_DUTY_PERCENT = float(os.getenv("VESC_MOTOR_TEST_MAX_DUTY_PERCENT", "8.0"))
MOTOR_TEST_DEFAULT_DUTY_PERCENT = float(os.getenv("VESC_MOTOR_TEST_DEFAULT_DUTY_PERCENT", "1.0"))
MOTOR_TEST_STEP_PERCENT = float(os.getenv("VESC_MOTOR_TEST_STEP_PERCENT", "1.0"))
MOTOR_TEST_MAX_DURATION_S = float(os.getenv("VESC_MOTOR_TEST_MAX_DURATION_S", "10.0"))
MOTOR_TEST_REGEN_BRAKE_CURRENT_A = float(os.getenv("VESC_MOTOR_TEST_REGEN_BRAKE_CURRENT_A", "2.0"))
MOTOR_TEST_REGEN_BRAKE_DURATION_S = float(os.getenv("VESC_MOTOR_TEST_REGEN_BRAKE_DURATION_S", "2.0"))

app = Flask(__name__)

lock = threading.Lock()

state = {
    "connected": False,
    "mode": "monitor",
    "port": VESC_PORT,
    "firmware": None,
    "last_error": None,
    "last_update": None,
    "started_at": datetime.now().isoformat(timespec="seconds"),
    "read_interval": READ_INTERVAL,
}

latest_data = {}
history = deque(maxlen=HISTORY_LIMIT)

bridge_thread = None
bridge_server_socket = None
bridge_client_socket = None
bridge_stop_event = threading.Event()

tcp_bridge_state = {
    "enabled": False,
    "running": False,
    "host": TCP_BRIDGE_HOST,
    "port": TCP_BRIDGE_DEFAULT_PORT,
    "serial_port": VESC_PORT,
    "baud": TCP_BRIDGE_BAUD,
    "client_addr": None,
    "bytes_to_serial": 0,
    "bytes_to_tcp": 0,
    "last_error": None,
    "last_event": None,
    "started_at": None,
}

motor_command_queue = queue.Queue()

motor_test_state = {
    "running": False,
    "brake_running": False,
    "direction": "forward",
    "direction_label": "Frente",
    "duty_percent": 0.0,
    "duty_magnitude_percent": 0.0,
    "max_duty_percent": MOTOR_TEST_MAX_DUTY_PERCENT,
    "default_duty_percent": MOTOR_TEST_DEFAULT_DUTY_PERCENT,
    "step_percent": MOTOR_TEST_STEP_PERCENT,
    "max_duration_s": MOTOR_TEST_MAX_DURATION_S,
    "regen_brake_current_a": MOTOR_TEST_REGEN_BRAKE_CURRENT_A,
    "regen_brake_duration_s": MOTOR_TEST_REGEN_BRAKE_DURATION_S,
    "started_at": None,
    "brake_started_at": None,
    "updated_at": None,
    "last_error": None,
    "last_event": "Teste do motor parado.",
}

FAULTS = {
    0: "FAULT_CODE_NONE",
    1: "FAULT_CODE_OVER_VOLTAGE",
    2: "FAULT_CODE_UNDER_VOLTAGE",
    3: "FAULT_CODE_DRV",
    4: "FAULT_CODE_ABS_OVER_CURRENT",
    5: "FAULT_CODE_OVER_TEMP_FET",
    6: "FAULT_CODE_OVER_TEMP_MOTOR",
    7: "FAULT_CODE_GATE_DRIVER_OVER_VOLTAGE",
    8: "FAULT_CODE_GATE_DRIVER_UNDER_VOLTAGE",
    9: "FAULT_CODE_MCU_UNDER_VOLTAGE",
    10: "FAULT_CODE_BOOTING_FROM_WATCHDOG_RESET",
    11: "FAULT_CODE_ENCODER_SPI",
    12: "FAULT_CODE_ENCODER_SINCOS_BELOW_MIN_AMPLITUDE",
    13: "FAULT_CODE_ENCODER_SINCOS_ABOVE_MAX_AMPLITUDE",
    14: "FAULT_CODE_FLASH_CORRUPTION",
    15: "FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_1",
    16: "FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_2",
    17: "FAULT_CODE_HIGH_OFFSET_CURRENT_SENSOR_3",
    18: "FAULT_CODE_UNBALANCED_CURRENTS",
}


def now_iso():
    return datetime.now().isoformat(timespec="seconds")


def clamp_value(value, min_value, max_value):
    return max(min_value, min(max_value, value))


def safe_float(value, default=None):
    try:
        if value is None or value == "":
            return default
        return float(str(value).replace(",", "."))
    except Exception:
        return default


def safe_value(value):
    if isinstance(value, bytes):
        if len(value) == 1:
            return value[0]

        try:
            decoded = value.decode("utf-8", errors="ignore")
            return decoded if decoded else value.hex()
        except Exception:
            return value.hex()

    if isinstance(value, float):
        if math.isnan(value) or math.isinf(value):
            return None
        return value

    return value


def get_fault_number(value):
    if value is None:
        return None

    if isinstance(value, bytes):
        if len(value) == 0:
            return None
        return value[0]

    if isinstance(value, int):
        return value

    if isinstance(value, str):
        try:
            return int(value)
        except Exception:
            return None

    return None


def xml_text_to_value(text):
    if text is None:
        return ""

    value = str(text).strip()

    if value == "":
        return ""

    low = value.lower()

    if low == "true":
        return True

    if low == "false":
        return False

    try:
        if "." not in value and "," not in value:
            return int(value)
    except Exception:
        pass

    try:
        return float(value.replace(",", "."))
    except Exception:
        return value


def normalize_xml_key(key):
    key = str(key or "").strip()
    if not key:
        return key
    return key.split("}")[-1]


def flatten_xml_element(element, prefix="", out=None):
    if out is None:
        out = {}

    tag = normalize_xml_key(element.tag)
    key = f"{prefix}.{tag}" if prefix else tag
    children = list(element)

    if not children:
        out[key] = xml_text_to_value(element.text)
        return out

    name_child = None
    value_child = None

    for child in children:
        child_tag = normalize_xml_key(child.tag).lower()
        if child_tag in ["name", "key", "param", "parameter"]:
            name_child = child
        elif child_tag in ["value", "val"]:
            value_child = child

    if name_child is not None and value_child is not None:
        param_name = str(name_child.text or "").strip()
        if param_name:
            out[param_name] = xml_text_to_value(value_child.text)

    for child in children:
        flatten_xml_element(child, key, out)

    return out


def compact_flat_config(flat):
    compact = {}
    for key, value in (flat or {}).items():
        parts = str(key).split(".")
        leaf = parts[-1] if parts else key
        if leaf and leaf not in compact:
            compact[leaf] = value
    return compact


def load_xml_config(path):
    if not path.exists():
        return {
            "exists": False,
            "path": str(path),
            "updated_at": None,
            "root": None,
            "flat": {},
            "compact": {},
            "error": None,
        }

    try:
        tree = ET.parse(path)
        root = tree.getroot()
        flat = flatten_xml_element(root)
        compact = compact_flat_config(flat)

        return {
            "exists": True,
            "path": str(path),
            "updated_at": datetime.fromtimestamp(path.stat().st_mtime).isoformat(timespec="seconds"),
            "root": normalize_xml_key(root.tag),
            "flat": flat,
            "compact": compact,
            "error": None,
        }

    except Exception as e:
        return {
            "exists": True,
            "path": str(path),
            "updated_at": None,
            "root": None,
            "flat": {},
            "compact": {},
            "error": repr(e),
        }


def find_config_value(flat, names):
    if not flat:
        return None

    lowered = {str(k).lower(): v for k, v in flat.items()}

    for name in names:
        name_low = str(name).lower()

        if name_low in lowered:
            return lowered[name_low]

        for key, value in lowered.items():
            leaf = key.split(".")[-1]
            if leaf == name_low or key.endswith("." + name_low):
                return value

    return None


def enum_motor_type(value):
    mapping = {
        0: "BLDC",
        1: "DC",
        2: "FOC",
        3: "GPD",
    }

    if isinstance(value, str):
        if value.upper() in mapping.values():
            return value.upper()
        try:
            value = int(value)
        except Exception:
            return value

    return mapping.get(value, value)


def enum_app_to_use(value):
    mapping = {
        0: "APP_NONE",
        1: "PPM",
        2: "ADC",
        3: "UART",
        4: "PPM_UART",
        5: "ADC_UART",
        6: "NUNCHUK",
        7: "NRF",
        8: "CUSTOM",
        9: "BALANCE",
        10: "PAS",
        11: "ADC_PAS",
    }

    if isinstance(value, str):
        if value.upper() in ["UART", "PPM", "ADC", "NRF", "CUSTOM", "BALANCE"]:
            return value.upper()
        try:
            value = int(value)
        except Exception:
            return value

    return mapping.get(value, value)


def enum_sensor_port_mode(value):
    mapping = {
        0: "HALL",
        1: "ABI",
        2: "AS5047_SPI",
        3: "AD2S1205",
        4: "TS5700N8501",
        5: "TS5700N8501_MULTITURN",
        6: "MT6816_SPI",
        7: "MT6816_SPI_MULTITURN",
    }

    if isinstance(value, str):
        if value.upper() in ["HALL", "ABI", "SENSORLESS", "HALL SENSORS"]:
            return value
        try:
            value = int(value)
        except Exception:
            return value

    return mapping.get(value, value)


def summarize_real_app_config(app_flat):
    app_to_use = find_config_value(app_flat, [
        "app_to_use",
        "app_to_use_old",
        "app_usage",
        "app",
    ])

    return {
        "app_to_use": enum_app_to_use(app_to_use),
        "app_to_use_raw": app_to_use,
        "vesc_id": find_config_value(app_flat, [
            "controller_id",
            "vesc_id",
            "can_id",
        ]),
        "timeout_msec": find_config_value(app_flat, [
            "timeout_msec",
        ]),
        "timeout_brake_current": find_config_value(app_flat, [
            "timeout_brake_current",
        ]),
        "can_status_mode": find_config_value(app_flat, [
            "send_can_status",
            "can_status_msg",
            "can_status_message_mode",
        ]),
        "can_status_rate_hz": find_config_value(app_flat, [
            "send_can_status_rate_hz",
            "can_status_rate_hz",
        ]),
        "can_baud_rate": find_config_value(app_flat, [
            "can_baud_rate",
        ]),
        "can_mode": find_config_value(app_flat, [
            "can_mode",
        ]),
        "permanent_uart_enabled": find_config_value(app_flat, [
            "permanent_uart_enabled",
        ]),
        "shutdown_mode": find_config_value(app_flat, [
            "shutdown_mode",
        ]),
    }


def summarize_real_motor_config(motor_flat):
    motor_type = find_config_value(motor_flat, [
        "motor_type",
    ])

    sensor_port_mode = find_config_value(motor_flat, [
        "m_sensor_port_mode",
        "sensor_port_mode",
    ])

    return {
        "motor_type": enum_motor_type(motor_type),
        "motor_type_raw": motor_type,
        "sensor_port_mode": enum_sensor_port_mode(sensor_port_mode),
        "sensor_port_mode_raw": sensor_port_mode,
        "foc_sensor_mode": find_config_value(motor_flat, [
            "foc_sensor_mode",
        ]),
        "motor_current_max": find_config_value(motor_flat, [
            "l_current_max",
        ]),
        "motor_current_min_brake": find_config_value(motor_flat, [
            "l_current_min",
        ]),
        "battery_current_max": find_config_value(motor_flat, [
            "l_in_current_max",
        ]),
        "battery_current_regen": find_config_value(motor_flat, [
            "l_in_current_min",
        ]),
        "absolute_max_current": find_config_value(motor_flat, [
            "l_abs_current_max",
        ]),
        "battery_cutoff_start": find_config_value(motor_flat, [
            "l_battery_cut_start",
        ]),
        "battery_cutoff_end": find_config_value(motor_flat, [
            "l_battery_cut_end",
        ]),
        "max_duty": find_config_value(motor_flat, [
            "l_max_duty",
        ]),
        "min_duty": find_config_value(motor_flat, [
            "l_min_duty",
        ]),
        "max_wattage": find_config_value(motor_flat, [
            "l_watt_max",
        ]),
        "max_braking_wattage": find_config_value(motor_flat, [
            "l_watt_min",
        ]),
        "max_erpm": find_config_value(motor_flat, [
            "l_max_erpm",
        ]),
        "min_erpm": find_config_value(motor_flat, [
            "l_min_erpm",
        ]),
        "temp_fet_start": find_config_value(motor_flat, [
            "l_temp_fet_start",
        ]),
        "temp_fet_end": find_config_value(motor_flat, [
            "l_temp_fet_end",
        ]),
        "temp_motor_start": find_config_value(motor_flat, [
            "l_temp_motor_start",
        ]),
        "temp_motor_end": find_config_value(motor_flat, [
            "l_temp_motor_end",
        ]),
        "min_input_voltage": find_config_value(motor_flat, [
            "l_min_vin",
            "l_min_voltage",
        ]),
        "max_input_voltage": find_config_value(motor_flat, [
            "l_max_vin",
            "l_max_voltage",
        ]),
    }


def load_real_vesc_configs():
    app_cfg = load_xml_config(REAL_APP_CONFIG_FILE)
    motor_cfg = load_xml_config(REAL_MOTOR_CONFIG_FILE)

    app_flat = app_cfg.get("flat", {})
    motor_flat = motor_cfg.get("flat", {})

    app_summary = summarize_real_app_config(app_flat)
    motor_summary = summarize_real_motor_config(motor_flat)

    return {
        "app": app_cfg,
        "motor": motor_cfg,
        "app_summary": app_summary,
        "motor_summary": motor_summary,
        "dir": str(REAL_CONFIG_DIR),
        "upload_dir": str(REAL_CONFIG_UPLOAD_DIR),
    }


def estimate_battery_percent_from_real_config(v_in):
    real_config = load_real_vesc_configs()
    motor_summary = real_config.get("motor_summary", {})

    cutoff_end = safe_float(motor_summary.get("battery_cutoff_end"))
    cutoff_start = safe_float(motor_summary.get("battery_cutoff_start"))
    vin = safe_float(v_in)

    if vin is None or cutoff_end is None or cutoff_start is None:
        return None

    if cutoff_start <= cutoff_end:
        return None

    pct = ((vin - cutoff_end) / (cutoff_start - cutoff_end)) * 100.0
    return max(0.0, min(100.0, round(pct, 1)))


def convert_measurements(values):
    raw = vars(values).copy()
    data = {}

    for key, value in raw.items():
        data[key] = safe_value(value)

    fault_number = get_fault_number(raw.get("mc_fault_code"))
    data["fault_number"] = fault_number
    data["fault_name"] = FAULTS.get(fault_number, "UNKNOWN")

    v_in = float(data.get("v_in") or 0)
    i_in = float(data.get("avg_input_current") or 0)
    i_motor = float(data.get("avg_motor_current") or 0)
    duty = float(data.get("duty_cycle_now") or 0)
    rpm = float(data.get("rpm") or 0)

    data["input_power_w"] = v_in * i_in
    data["motor_power_est_w"] = v_in * abs(duty) * i_motor
    data["duty_percent"] = duty * 100.0
    data["rpm_abs"] = abs(rpm)

    real_config = load_real_vesc_configs()
    app_summary = real_config.get("app_summary", {})

    app_xml_vesc_id = app_summary.get("vesc_id")
    live_app_controller_id = data.get("app_controller_id")

    data["app_xml_vesc_id"] = app_xml_vesc_id
    data["live_app_controller_id"] = live_app_controller_id

    if live_app_controller_id is not None and live_app_controller_id != "":
        data["display_can_id"] = live_app_controller_id
    else:
        data["display_can_id"] = app_xml_vesc_id

    data["configured_can_id"] = data.get("display_can_id")
    data["battery_percent_est"] = estimate_battery_percent_from_real_config(v_in)

    return data


def list_serial_ports():
    ports = []
    patterns = [
        "/dev/serial/by-id/*",
        "/dev/ttyACM*",
        "/dev/ttyUSB*",
    ]

    for pattern in patterns:
        ports.extend(glob.glob(pattern))

    ports = sorted(set(ports))

    def score(port):
        p = str(port).lower()

        if "/dev/serial/by-id/" in p and (
            "stmicroelectronics" in p or
            "chibios" in p or
            "vesc" in p
        ):
            return 0

        if "/dev/serial/by-id/" in p:
            return 1

        if "/dev/ttyacm" in p:
            return 2

        if "/dev/ttyusb" in p:
            return 3

        return 9

    return sorted(ports, key=score)


def port_exists(port):
    if not port:
        return False

    return os.path.exists(port)


def auto_detect_vesc_port():
    ports = list_serial_ports()

    for port in ports:
        p = str(port).lower()

        if "/dev/serial/by-id/" in p and (
            "stmicroelectronics" in p or
            "chibios" in p or
            "vesc" in p
        ):
            return port

    for port in ports:
        if str(port).startswith("/dev/serial/by-id/"):
            return port

    for port in ports:
        if str(port).startswith("/dev/ttyACM"):
            return port

    for port in ports:
        if str(port).startswith("/dev/ttyUSB"):
            return port

    return None


def set_active_vesc_port(port, auto=False):
    global VESC_PORT

    if not port:
        return False, "Porta inválida."

    if not port_exists(port):
        return False, f"Porta não encontrada: {port}"

    with lock:
        VESC_PORT = port

        selected_serial_port["port"] = port
        selected_serial_port["auto"] = bool(auto)
        selected_serial_port["last_scan"] = now_iso()
        selected_serial_port["last_error"] = None

        state["port"] = port
        tcp_bridge_state["serial_port"] = port

    return True, f"Porta VESC selecionada: {port}"


def auto_select_active_vesc_port():
    detected = auto_detect_vesc_port()

    if not detected:
        with lock:
            selected_serial_port["last_scan"] = now_iso()
            selected_serial_port["last_error"] = "Nenhuma porta VESC detectada."

        return False, "Nenhuma porta VESC detectada."

    return set_active_vesc_port(detected, auto=True)


def get_active_vesc_port():
    with lock:
        current = selected_serial_port.get("port")
        auto_enabled = bool(selected_serial_port.get("auto", True))

    if current and port_exists(current):
        return current

    if auto_enabled:
        detected = auto_detect_vesc_port()

        if detected:
            set_active_vesc_port(detected, auto=True)
            return detected

    return current or DEFAULT_VESC_PORT


def get_serial_port_state():
    ports = list_serial_ports()

    active = get_active_vesc_port()

    with lock:
        selected = selected_serial_port.copy()

    return {
        "ports": ports,
        "selected": active,
        "exists": port_exists(active),
        "auto": selected.get("auto", True),
        "last_scan": selected.get("last_scan"),
        "last_error": selected.get("last_error"),
    }

def get_tcp_bridge_state():
    with lock:
        return tcp_bridge_state.copy()


def tcp_bridge_requested():
    with lock:
        return bool(tcp_bridge_state.get("enabled") or tcp_bridge_state.get("running"))


def close_bridge_sockets():
    global bridge_server_socket, bridge_client_socket

    try:
        if bridge_client_socket:
            try:
                bridge_client_socket.shutdown(socket.SHUT_RDWR)
            except Exception:
                pass
            bridge_client_socket.close()
    except Exception:
        pass

    try:
        if bridge_server_socket:
            try:
                bridge_server_socket.shutdown(socket.SHUT_RDWR)
            except Exception:
                pass
            bridge_server_socket.close()
    except Exception:
        pass

    bridge_client_socket = None
    bridge_server_socket = None


def bridge_socket_to_serial(client, ser):
    while not bridge_stop_event.is_set():
        try:
            data = client.recv(TCP_BUFFER_SIZE)

            if not data:
                break

            ser.write(data)
            ser.flush()

            with lock:
                tcp_bridge_state["bytes_to_serial"] += len(data)

        except socket.timeout:
            continue
        except Exception:
            break


def bridge_serial_to_socket(ser, client):
    while not bridge_stop_event.is_set():
        try:
            data = ser.read(TCP_BUFFER_SIZE)

            if data:
                client.sendall(data)

                with lock:
                    tcp_bridge_state["bytes_to_tcp"] += len(data)

        except Exception:
            break


def handle_tcp_bridge_client(client, addr):
    global bridge_client_socket

    client_addr = f"{addr[0]}:{addr[1]}"

    with lock:
        bridge_client_socket = client
        tcp_bridge_state["client_addr"] = client_addr
        tcp_bridge_state["last_event"] = f"Cliente conectado: {client_addr}"
        tcp_bridge_state["last_error"] = None

    try:
        client.settimeout(0.2)

        active_port = get_active_vesc_port()

        if not port_exists(active_port):
            auto_select_active_vesc_port()
            active_port = get_active_vesc_port()

        if not port_exists(active_port):
            raise serial.SerialException(f"Porta serial não encontrada: {active_port}")

        with serial.Serial(
            active_port,
            baudrate=TCP_BRIDGE_BAUD,
            timeout=0.05,
            write_timeout=0.5,
        ) as ser:
            try:
                ser.reset_input_buffer()
                ser.reset_output_buffer()
            except Exception:
                pass

            t1 = threading.Thread(
                target=bridge_socket_to_serial,
                args=(client, ser),
                daemon=True,
            )

            t2 = threading.Thread(
                target=bridge_serial_to_socket,
                args=(ser, client),
                daemon=True,
            )

            t1.start()
            t2.start()

            while (
                not bridge_stop_event.is_set()
                and t1.is_alive()
                and t2.is_alive()
            ):
                time.sleep(0.1)

    except Exception as e:
        with lock:
            tcp_bridge_state["last_error"] = repr(e)
            tcp_bridge_state["last_event"] = "Erro no cliente TCP Bridge"

    try:
        client.close()
    except Exception:
        pass

    with lock:
        bridge_client_socket = None
        tcp_bridge_state["client_addr"] = None
        tcp_bridge_state["last_event"] = f"Cliente desconectado: {client_addr}"


def tcp_bridge_loop(port):
    global bridge_server_socket

    try:
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((TCP_BRIDGE_HOST, port))
        server.listen(1)
        server.settimeout(0.5)

        with lock:
            bridge_server_socket = server
            tcp_bridge_state["running"] = True
            tcp_bridge_state["enabled"] = True
            tcp_bridge_state["port"] = port
            tcp_bridge_state["host"] = TCP_BRIDGE_HOST
            tcp_bridge_state["serial_port"] = get_active_vesc_port()
            tcp_bridge_state["baud"] = TCP_BRIDGE_BAUD
            tcp_bridge_state["started_at"] = now_iso()
            tcp_bridge_state["last_error"] = None
            tcp_bridge_state["last_event"] = f"TCP Bridge ativo em {TCP_BRIDGE_HOST}:{port}"

        while not bridge_stop_event.is_set():
            try:
                client, addr = server.accept()
            except socket.timeout:
                continue
            except OSError:
                break

            handle_tcp_bridge_client(client, addr)

    except Exception as e:
        with lock:
            tcp_bridge_state["last_error"] = repr(e)
            tcp_bridge_state["last_event"] = "Falha ao iniciar TCP Bridge"

    finally:
        close_bridge_sockets()

        with lock:
            tcp_bridge_state["running"] = False
            tcp_bridge_state["enabled"] = False
            tcp_bridge_state["client_addr"] = None

            if not tcp_bridge_state.get("last_event"):
                tcp_bridge_state["last_event"] = "TCP Bridge parado"


def start_tcp_bridge(port=None):
    global bridge_thread

    try:
        port = int(port or TCP_BRIDGE_DEFAULT_PORT)
    except Exception:
        return {
            "ok": False,
            "message": "Porta TCP inválida.",
            "bridge": get_tcp_bridge_state(),
        }

    if port < 1 or port > 65535:
        return {
            "ok": False,
            "message": "A porta TCP deve estar entre 1 e 65535.",
            "bridge": get_tcp_bridge_state(),
        }

    with lock:
        if tcp_bridge_state.get("running") or tcp_bridge_state.get("enabled"):
            return {
                "ok": True,
                "message": "TCP Bridge já está ligado.",
                "bridge": tcp_bridge_state.copy(),
            }

        tcp_bridge_state["enabled"] = True
        tcp_bridge_state["running"] = False
        tcp_bridge_state["port"] = port
        tcp_bridge_state["bytes_to_serial"] = 0
        tcp_bridge_state["bytes_to_tcp"] = 0
        tcp_bridge_state["client_addr"] = None
        tcp_bridge_state["last_error"] = None
        tcp_bridge_state["last_event"] = "Inicializando TCP Bridge..."

    bridge_stop_event.clear()

    time.sleep(0.8)

    bridge_thread = threading.Thread(
        target=tcp_bridge_loop,
        args=(port,),
        daemon=True,
    )
    bridge_thread.start()

    return {
        "ok": True,
        "message": f"TCP Bridge ligando na porta {port}.",
        "bridge": get_tcp_bridge_state(),
    }


def stop_tcp_bridge():
    with lock:
        tcp_bridge_state["enabled"] = False
        tcp_bridge_state["last_event"] = "Parando TCP Bridge..."

    bridge_stop_event.set()
    close_bridge_sockets()

    time.sleep(0.3)

    return {
        "ok": True,
        "message": "TCP Bridge desligado. O monitor VESC voltará a usar a serial.",
        "bridge": get_tcp_bridge_state(),
    }


def get_motor_test_state():
    with lock:
        return motor_test_state.copy()


def safe_direction(direction):
    direction = str(direction or "").strip().lower()

    if direction in ["reverse", "backward", "tras", "trás", "re", "ré", "back"]:
        return "reverse"

    return "forward"


def direction_label(direction):
    return "Trás" if safe_direction(direction) == "reverse" else "Frente"


def signed_duty_from_direction(magnitude_percent, direction):
    magnitude = safe_test_duty_magnitude_percent(magnitude_percent)

    if safe_direction(direction) == "reverse":
        return -magnitude

    return magnitude


def safe_test_duty_magnitude_percent(value):
    try:
        duty = abs(float(value))
    except Exception:
        duty = 0.0

    max_abs = abs(float(MOTOR_TEST_MAX_DUTY_PERCENT))
    return round(clamp_value(duty, 0.0, max_abs), 2)


def safe_test_duty_percent(value):
    try:
        duty = float(value)
    except Exception:
        duty = 0.0

    max_abs = abs(float(MOTOR_TEST_MAX_DUTY_PERCENT))
    return round(clamp_value(duty, -max_abs, max_abs), 2)


def safe_regen_brake_current(value=None):
    current = safe_float(
        MOTOR_TEST_REGEN_BRAKE_CURRENT_A if value is None else value,
        MOTOR_TEST_REGEN_BRAKE_CURRENT_A,
    )

    return round(clamp_value(abs(current), 0.1, 10.0), 2)


def send_current_brake(motor, current_a):
    current_a = safe_regen_brake_current(current_a)

    if hasattr(motor, "set_current_brake"):
        motor.set_current_brake(current_a)
        return

    motor.write(pyvesc.encode(SetCurrentBrake(current_a)))


def enqueue_motor_command(command):
    motor_command_queue.put(command)


def start_motor_test(duty_percent=None, direction=None):
    if tcp_bridge_requested():
        return {
            "ok": False,
            "message": "Não é possível testar o motor com o TCP Bridge ativo.",
            "motor_test": get_motor_test_state(),
        }

    with lock:
        current_direction = safe_direction(direction or motor_test_state.get("direction", "forward"))

    magnitude = safe_test_duty_magnitude_percent(
        MOTOR_TEST_DEFAULT_DUTY_PERCENT if duty_percent is None else duty_percent
    )

    if magnitude == 0:
        magnitude = safe_test_duty_magnitude_percent(MOTOR_TEST_DEFAULT_DUTY_PERCENT)

    signed_duty = signed_duty_from_direction(magnitude, current_direction)

    with lock:
        motor_test_state["running"] = True
        motor_test_state["brake_running"] = False
        motor_test_state["direction"] = current_direction
        motor_test_state["direction_label"] = direction_label(current_direction)
        motor_test_state["duty_percent"] = signed_duty
        motor_test_state["duty_magnitude_percent"] = magnitude
        motor_test_state["started_at"] = now_iso()
        motor_test_state["brake_started_at"] = None
        motor_test_state["updated_at"] = now_iso()
        motor_test_state["last_error"] = None
        motor_test_state["last_event"] = (
            f"Teste iniciado: {direction_label(current_direction)} com duty {magnitude:.2f}%."
        )

    enqueue_motor_command({
        "type": "set_duty",
        "duty_percent": signed_duty,
        "source": "start_motor_test",
    })

    return {
        "ok": True,
        "message": f"Teste iniciado: {direction_label(current_direction)} com duty {magnitude:.2f}%.",
        "motor_test": get_motor_test_state(),
    }


def stop_motor_test():
    with lock:
        motor_test_state["running"] = False
        motor_test_state["brake_running"] = False
        motor_test_state["duty_percent"] = 0.0
        motor_test_state["duty_magnitude_percent"] = 0.0
        motor_test_state["updated_at"] = now_iso()
        motor_test_state["last_event"] = "Parada solicitada."

    enqueue_motor_command({
        "type": "stop",
        "source": "stop_motor_test",
    })

    return {
        "ok": True,
        "message": "Motor parado.",
        "motor_test": get_motor_test_state(),
    }


def set_motor_test_direction(direction):
    if tcp_bridge_requested():
        return {
            "ok": False,
            "message": "Não é possível alterar direção com o TCP Bridge ativo.",
            "motor_test": get_motor_test_state(),
        }

    selected_direction = safe_direction(direction)

    with lock:
        motor_test_state["direction"] = selected_direction
        motor_test_state["direction_label"] = direction_label(selected_direction)
        magnitude = float(motor_test_state.get("duty_magnitude_percent") or 0.0)
        running = bool(motor_test_state.get("running"))
        motor_test_state["updated_at"] = now_iso()
        motor_test_state["last_error"] = None
        motor_test_state["last_event"] = f"Direção selecionada: {direction_label(selected_direction)}."

    if running and magnitude > 0:
        signed_duty = signed_duty_from_direction(magnitude, selected_direction)

        with lock:
            motor_test_state["duty_percent"] = signed_duty
            motor_test_state["last_event"] = (
                f"Direção alterada para {direction_label(selected_direction)} com duty {magnitude:.2f}%."
            )

        enqueue_motor_command({
            "type": "set_duty",
            "duty_percent": signed_duty,
            "source": "set_motor_test_direction",
        })

    return {
        "ok": True,
        "message": f"Direção selecionada: {direction_label(selected_direction)}.",
        "motor_test": get_motor_test_state(),
    }


def set_motor_test_duty(duty_percent):
    if tcp_bridge_requested():
        return {
            "ok": False,
            "message": "Não é possível ajustar duty com o TCP Bridge ativo.",
            "motor_test": get_motor_test_state(),
        }

    magnitude = safe_test_duty_magnitude_percent(duty_percent)

    with lock:
        selected_direction = safe_direction(motor_test_state.get("direction", "forward"))
        signed_duty = signed_duty_from_direction(magnitude, selected_direction)

        motor_test_state["running"] = magnitude != 0
        motor_test_state["brake_running"] = False
        motor_test_state["duty_percent"] = signed_duty
        motor_test_state["duty_magnitude_percent"] = magnitude
        motor_test_state["updated_at"] = now_iso()
        motor_test_state["last_error"] = None
        motor_test_state["last_event"] = (
            f"Duty ajustado para {magnitude:.2f}% em {direction_label(selected_direction)}."
        )

        if magnitude != 0 and not motor_test_state.get("started_at"):
            motor_test_state["started_at"] = now_iso()

    enqueue_motor_command({
        "type": "set_duty",
        "duty_percent": signed_duty,
        "source": "set_motor_test_duty",
    })

    return {
        "ok": True,
        "message": f"Duty ajustado para {magnitude:.2f}% em {direction_label(selected_direction)}.",
        "motor_test": get_motor_test_state(),
    }


def step_motor_test_duty(delta_percent):
    with lock:
        current = float(motor_test_state.get("duty_magnitude_percent") or 0.0)

    delta = safe_float(delta_percent, 0.0) or 0.0
    return set_motor_test_duty(current + delta)


def regen_brake_motor_test(current_a=None):
    if tcp_bridge_requested():
        return {
            "ok": False,
            "message": "Não é possível frear com o TCP Bridge ativo.",
            "motor_test": get_motor_test_state(),
        }

    current = safe_regen_brake_current(current_a)

    with lock:
        motor_test_state["running"] = False
        motor_test_state["brake_running"] = True
        motor_test_state["duty_percent"] = 0.0
        motor_test_state["duty_magnitude_percent"] = 0.0
        motor_test_state["regen_brake_current_a"] = current
        motor_test_state["brake_started_at"] = now_iso()
        motor_test_state["updated_at"] = now_iso()
        motor_test_state["last_error"] = None
        motor_test_state["last_event"] = f"Freio regenerativo ativo: {current:.2f}A."

    enqueue_motor_command({
        "type": "regen_brake",
        "brake_current_a": current,
        "source": "regen_brake_motor_test",
    })

    return {
        "ok": True,
        "message": f"Freio regenerativo aplicado com {current:.2f}A.",
        "motor_test": get_motor_test_state(),
    }


def process_motor_commands(motor):
    processed = 0

    while True:
        try:
            command = motor_command_queue.get_nowait()
        except queue.Empty:
            break

        processed += 1
        cmd_type = command.get("type")

        try:
            if cmd_type == "set_duty":
                duty_percent = safe_test_duty_percent(command.get("duty_percent", 0.0))
                duty = duty_percent / 100.0
                magnitude = abs(duty_percent)

                if duty_percent == 0:
                    motor.set_duty_cycle(0.0)
                    try:
                        motor.set_current(0.0)
                    except Exception:
                        pass

                    with lock:
                        motor_test_state["running"] = False
                        motor_test_state["brake_running"] = False
                        motor_test_state["duty_percent"] = 0.0
                        motor_test_state["duty_magnitude_percent"] = 0.0
                        motor_test_state["last_event"] = "Duty zerado."
                        motor_test_state["updated_at"] = now_iso()
                else:
                    try:
                        motor.set_current(0.0)
                    except Exception:
                        pass

                    motor.set_duty_cycle(duty)

                    with lock:
                        direction = "reverse" if duty_percent < 0 else "forward"
                        motor_test_state["running"] = True
                        motor_test_state["brake_running"] = False
                        motor_test_state["direction"] = direction
                        motor_test_state["direction_label"] = direction_label(direction)
                        motor_test_state["duty_percent"] = duty_percent
                        motor_test_state["duty_magnitude_percent"] = magnitude
                        motor_test_state["last_event"] = (
                            f"Duty enviado: {magnitude:.2f}% em {direction_label(direction)}."
                        )
                        motor_test_state["updated_at"] = now_iso()

            elif cmd_type == "regen_brake":
                current = safe_regen_brake_current(command.get("brake_current_a"))

                try:
                    motor.set_duty_cycle(0.0)
                except Exception:
                    pass

                send_current_brake(motor, current)

                with lock:
                    motor_test_state["running"] = False
                    motor_test_state["brake_running"] = True
                    motor_test_state["duty_percent"] = 0.0
                    motor_test_state["duty_magnitude_percent"] = 0.0
                    motor_test_state["regen_brake_current_a"] = current
                    motor_test_state["brake_started_at"] = now_iso()
                    motor_test_state["last_event"] = f"Freio regenerativo enviado: {current:.2f}A."
                    motor_test_state["updated_at"] = now_iso()

            elif cmd_type == "stop":
                try:
                    motor.set_duty_cycle(0.0)
                except Exception:
                    pass

                try:
                    motor.set_current(0.0)
                except Exception:
                    pass

                with lock:
                    motor_test_state["running"] = False
                    motor_test_state["brake_running"] = False
                    motor_test_state["duty_percent"] = 0.0
                    motor_test_state["duty_magnitude_percent"] = 0.0
                    motor_test_state["last_event"] = "Motor parado."
                    motor_test_state["updated_at"] = now_iso()

        except Exception as e:
            with lock:
                motor_test_state["last_error"] = repr(e)
                motor_test_state["last_event"] = "Erro ao enviar comando para o VESC."
                motor_test_state["running"] = False
                motor_test_state["brake_running"] = False
                motor_test_state["duty_percent"] = 0.0
                motor_test_state["duty_magnitude_percent"] = 0.0

    return processed


def maintain_motor_test(motor):
    with lock:
        running = bool(motor_test_state.get("running"))
        brake_running = bool(motor_test_state.get("brake_running"))
        duty_percent = float(motor_test_state.get("duty_percent") or 0.0)
        brake_current = float(motor_test_state.get("regen_brake_current_a") or MOTOR_TEST_REGEN_BRAKE_CURRENT_A)
        started_at = motor_test_state.get("started_at")
        brake_started_at = motor_test_state.get("brake_started_at")

    if brake_running:
        if brake_started_at:
            try:
                brake_dt = datetime.fromisoformat(brake_started_at)
                brake_elapsed = (datetime.now() - brake_dt).total_seconds()
            except Exception:
                brake_elapsed = 0

            if brake_elapsed > MOTOR_TEST_REGEN_BRAKE_DURATION_S:
                try:
                    motor.set_current(0.0)
                except Exception:
                    pass

                try:
                    motor.set_duty_cycle(0.0)
                except Exception:
                    pass

                with lock:
                    motor_test_state["brake_running"] = False
                    motor_test_state["running"] = False
                    motor_test_state["duty_percent"] = 0.0
                    motor_test_state["duty_magnitude_percent"] = 0.0
                    motor_test_state["last_event"] = (
                        f"Freio regenerativo liberado após {MOTOR_TEST_REGEN_BRAKE_DURATION_S:.1f}s."
                    )
                    motor_test_state["updated_at"] = now_iso()
                return

        try:
            send_current_brake(motor, brake_current)
        except Exception as e:
            with lock:
                motor_test_state["brake_running"] = False
                motor_test_state["last_error"] = repr(e)
                motor_test_state["last_event"] = "Erro no freio regenerativo."
        return

    if not running:
        return

    if started_at:
        try:
            start_dt = datetime.fromisoformat(started_at)
            elapsed = (datetime.now() - start_dt).total_seconds()
        except Exception:
            elapsed = 0

        if elapsed > MOTOR_TEST_MAX_DURATION_S:
            try:
                motor.set_duty_cycle(0.0)
            except Exception:
                pass

            try:
                motor.set_current(0.0)
            except Exception:
                pass

            with lock:
                motor_test_state["running"] = False
                motor_test_state["brake_running"] = False
                motor_test_state["duty_percent"] = 0.0
                motor_test_state["duty_magnitude_percent"] = 0.0
                motor_test_state["last_event"] = f"Auto-stop: limite de {MOTOR_TEST_MAX_DURATION_S:.1f}s atingido."
                motor_test_state["updated_at"] = now_iso()
            return

    duty_percent = safe_test_duty_percent(duty_percent)

    try:
        motor.set_duty_cycle(duty_percent / 100.0)
    except Exception as e:
        with lock:
            motor_test_state["running"] = False
            motor_test_state["brake_running"] = False
            motor_test_state["duty_percent"] = 0.0
            motor_test_state["duty_magnitude_percent"] = 0.0
            motor_test_state["last_error"] = repr(e)
            motor_test_state["last_event"] = "Erro no envio contínuo do duty."


def vesc_reader_loop():
    global latest_data

    while True:
        if tcp_bridge_requested():
            with lock:
                state["connected"] = False
                state["mode"] = "tcp_bridge"
                state["last_error"] = "Monitor pausado: TCP Bridge ativo para VESC Tool."
                state["last_update"] = datetime.now().isoformat(timespec="seconds")
                motor_test_state["running"] = False
                motor_test_state["brake_running"] = False
                motor_test_state["duty_percent"] = 0.0
                motor_test_state["duty_magnitude_percent"] = 0.0
                motor_test_state["last_event"] = "Teste do motor bloqueado: TCP Bridge ativo."

            time.sleep(0.5)
            continue

        try:
            active_port = get_active_vesc_port()

            if not port_exists(active_port):
                auto_select_active_vesc_port()
                active_port = get_active_vesc_port()

            if not port_exists(active_port):
                with lock:
                    state["connected"] = False
                    state["mode"] = "monitor"
                    state["last_error"] = f"Porta serial não encontrada: {active_port}"
                    state["port"] = active_port
                    state["last_update"] = now_iso()

                time.sleep(2)
                continue

            with lock:
                state["connected"] = False
                state["mode"] = "monitor"
                state["last_error"] = None
                state["port"] = active_port

            with VESC(serial_port=active_port, start_heartbeat=False) as motor:
                try:
                    firmware = motor.get_firmware_version()
                except Exception:
                    firmware = None

                with lock:
                    state["connected"] = True
                    state["mode"] = "monitor"
                    state["firmware"] = str(firmware)
                    state["last_error"] = None

                while True:
                    if tcp_bridge_requested():
                        with lock:
                            state["connected"] = False
                            state["mode"] = "tcp_bridge"
                            state["last_error"] = "Monitor pausado: TCP Bridge ativo para VESC Tool."
                            state["last_update"] = datetime.now().isoformat(timespec="seconds")
                        break

                    process_motor_commands(motor)
                    maintain_motor_test(motor)

                    values = motor.get_measurements()
                    data = convert_measurements(values)

                    fault_number = data.get("fault_number")
                    if fault_number not in [None, 0]:
                        try:
                            motor.set_duty_cycle(0.0)
                        except Exception:
                            pass

                        try:
                            motor.set_current(0.0)
                        except Exception:
                            pass

                        with lock:
                            motor_test_state["running"] = False
                            motor_test_state["brake_running"] = False
                            motor_test_state["duty_percent"] = 0.0
                            motor_test_state["duty_magnitude_percent"] = 0.0
                            motor_test_state["last_error"] = data.get("fault_name")
                            motor_test_state["last_event"] = "Auto-stop por fault do VESC."

                    now = datetime.now()
                    data["timestamp"] = now.isoformat(timespec="milliseconds")
                    data["time_label"] = now.strftime("%H:%M:%S")

                    with lock:
                        latest_data = data
                        history.append(data.copy())
                        state["connected"] = True
                        state["mode"] = "monitor"
                        state["last_update"] = data["timestamp"]
                        state["last_error"] = None

                    time.sleep(READ_INTERVAL)

        except Exception as e:
            with lock:
                state["connected"] = False
                state["last_error"] = repr(e)
                state["last_update"] = datetime.now().isoformat(timespec="seconds")

            time.sleep(2)


@app.route("/")
def index():
    return render_template_string(TEMPLATE, app_port=APP_PORT, vesc_port=VESC_PORT)


@app.route("/api/data")
def api_data():
    with lock:
        data = latest_data.copy()

        return jsonify({
            "state": state.copy(),
            "data": data,
            "ports": list_serial_ports(),
            "serial": get_serial_port_state(),
            "real_config": load_real_vesc_configs(),
            "tcp_bridge": tcp_bridge_state.copy(),
            "motor_test": motor_test_state.copy(),
        })


@app.route("/api/history")
def api_history():
    with lock:
        return jsonify({
            "state": state.copy(),
            "history": list(history),
        })


@app.route("/api/ports")
def api_ports():
    return jsonify({
        "ok": True,
        "serial": get_serial_port_state(),
    })


@app.route("/api/serial-port", methods=["GET"])
def api_serial_port_get():
    return jsonify({
        "ok": True,
        "serial": get_serial_port_state(),
    })


@app.route("/api/serial-port", methods=["POST"])
def api_serial_port_set():
    payload = request.get_json(silent=True) or {}
    port = payload.get("port", "")

    ok, message = set_active_vesc_port(port, auto=False)

    return jsonify({
        "ok": ok,
        "message": message,
        "serial": get_serial_port_state(),
    }), 200 if ok else 400


@app.route("/api/serial-port/auto", methods=["POST"])
def api_serial_port_auto():
    ok, message = auto_select_active_vesc_port()

    return jsonify({
        "ok": ok,
        "message": message,
        "serial": get_serial_port_state(),
    }), 200 if ok else 400


@app.route("/api/real-config", methods=["GET"])
def api_real_config():
    return jsonify({
        "ok": True,
        "real_config": load_real_vesc_configs(),
    })


@app.route("/api/real-config/upload", methods=["POST"])
def api_real_config_upload():
    cfg_type = request.form.get("type", "").strip().lower()

    if cfg_type not in ["app", "motor"]:
        return jsonify({
            "ok": False,
            "message": "Tipo inválido. Use type=app ou type=motor.",
        }), 400

    if "file" not in request.files:
        return jsonify({
            "ok": False,
            "message": "Nenhum arquivo enviado.",
        }), 400

    uploaded = request.files["file"]

    if not uploaded.filename.lower().endswith(".xml"):
        return jsonify({
            "ok": False,
            "message": "Envie um arquivo .xml exportado pelo VESC Tool.",
        }), 400

    target = REAL_APP_CONFIG_FILE if cfg_type == "app" else REAL_MOTOR_CONFIG_FILE

    original_name = secure_filename(uploaded.filename or f"{cfg_type}_config.xml")
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    archive_name = f"{timestamp}_{cfg_type}_{original_name}"
    archive_target = REAL_CONFIG_UPLOAD_DIR / archive_name

    uploaded.save(archive_target)
    shutil.copyfile(archive_target, target)

    return jsonify({
        "ok": True,
        "message": (
            f"Configuração {cfg_type} carregada com sucesso. "
            f"Arquivo salvo em {archive_target} e aplicado como {target}."
        ),
        "saved_file": str(archive_target),
        "active_file": str(target),
        "real_config": load_real_vesc_configs(),
    })


@app.route("/api/motor-test", methods=["GET"])
def api_motor_test_status():
    return jsonify({
        "ok": True,
        "motor_test": get_motor_test_state(),
    })


@app.route("/api/motor-test/start", methods=["POST"])
def api_motor_test_start():
    payload = request.get_json(silent=True) or {}
    duty = payload.get("duty_percent", MOTOR_TEST_DEFAULT_DUTY_PERCENT)

    result = start_motor_test(duty)
    status_code = 200 if result.get("ok") else 400

    return jsonify(result), status_code


@app.route("/api/motor-test/stop", methods=["POST"])
def api_motor_test_stop():
    result = stop_motor_test()
    return jsonify(result)


@app.route("/api/motor-test/set-duty", methods=["POST"])
def api_motor_test_set_duty():
    payload = request.get_json(silent=True) or {}
    duty = payload.get("duty_percent", 0.0)

    result = set_motor_test_duty(duty)
    status_code = 200 if result.get("ok") else 400

    return jsonify(result), status_code


@app.route("/api/motor-test/step", methods=["POST"])
def api_motor_test_step():
    payload = request.get_json(silent=True) or {}
    delta = payload.get("delta_percent", MOTOR_TEST_STEP_PERCENT)

    result = step_motor_test_duty(delta)
    status_code = 200 if result.get("ok") else 400

    return jsonify(result), status_code


@app.route("/api/motor-test/direction", methods=["POST"])
def api_motor_test_direction():
    payload = request.get_json(silent=True) or {}
    direction = payload.get("direction", "forward")

    result = set_motor_test_direction(direction)
    status_code = 200 if result.get("ok") else 400

    return jsonify(result), status_code


@app.route("/api/motor-test/regen-brake", methods=["POST"])
def api_motor_test_regen_brake():
    payload = request.get_json(silent=True) or {}
    current = payload.get("current_a", MOTOR_TEST_REGEN_BRAKE_CURRENT_A)

    result = regen_brake_motor_test(current)
    status_code = 200 if result.get("ok") else 400

    return jsonify(result), status_code


@app.route("/api/tcp-bridge", methods=["GET"])
def api_tcp_bridge_status():
    return jsonify({
        "ok": True,
        "bridge": get_tcp_bridge_state(),
    })


@app.route("/api/tcp-bridge/start", methods=["POST"])
def api_tcp_bridge_start():
    payload = request.get_json(silent=True) or {}
    port = payload.get("port", TCP_BRIDGE_DEFAULT_PORT)

    result = start_tcp_bridge(port)
    status_code = 200 if result.get("ok") else 400

    return jsonify(result), status_code


@app.route("/api/tcp-bridge/stop", methods=["POST"])
def api_tcp_bridge_stop():
    result = stop_tcp_bridge()
    return jsonify(result)


TEMPLATE = r"""
<!doctype html>
<html lang="pt-BR">
<head>
    <meta charset="utf-8">
    <title>Monitor VESC / Flipsky 75100</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">

    <link
        href="https://cdn.jsdelivr.net/npm/bootstrap@5.3.3/dist/css/bootstrap.min.css"
        rel="stylesheet"
    >

    <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>

    <style>
        :root {
            --bg: #080b10;
            --panel: #111827;
            --panel2: #172033;
            --text: #e5e7eb;
            --muted: #9ca3af;
            --border: rgba(255,255,255,0.08);
        }

        body {
            background: radial-gradient(circle at top, #182235 0, #080b10 45%, #05070a 100%);
            color: var(--text);
            min-height: 100vh;
        }

        .navbar {
            background: rgba(8, 11, 16, 0.85);
            backdrop-filter: blur(12px);
            border-bottom: 1px solid var(--border);
        }

        .status-dot {
            width: 12px;
            height: 12px;
            border-radius: 50%;
            display: inline-block;
            margin-right: 8px;
            background: #6b7280;
            box-shadow: 0 0 12px rgba(255,255,255,0.15);
        }

        .status-dot.ok {
            background: #22c55e;
            box-shadow: 0 0 18px rgba(34,197,94,0.75);
        }

        .status-dot.err {
            background: #ef4444;
            box-shadow: 0 0 18px rgba(239,68,68,0.75);
        }

        .card-vesc {
            background: linear-gradient(145deg, rgba(17,24,39,0.96), rgba(23,32,51,0.92));
            border: 1px solid var(--border);
            border-radius: 18px;
            box-shadow: 0 15px 50px rgba(0,0,0,0.28);
        }

        .metric-label {
            color: var(--muted);
            font-size: 0.82rem;
            text-transform: uppercase;
            letter-spacing: 0.08em;
        }

        .metric-value {
            font-size: 2rem;
            font-weight: 800;
            line-height: 1.1;
        }

        .metric-unit {
            color: var(--muted);
            font-size: 0.95rem;
        }

        .small-muted {
            color: var(--muted);
            font-size: 0.88rem;
        }

        .chart-box {
            height: 260px;
        }

        .raw-table {
            color: var(--text);
        }

        .raw-table td,
        .raw-table th {
            background: transparent !important;
            color: var(--text);
            border-color: var(--border);
        }

        .raw-key {
            color: #93c5fd;
            font-family: monospace;
        }

        .raw-value {
            font-family: monospace;
            word-break: break-all;
        }

        .progress {
            background: rgba(255,255,255,0.08);
            height: 8px;
        }

        .progress-bar {
            transition: width 0.25s ease;
        }

        .badge-soft {
            background: rgba(255,255,255,0.08);
            color: var(--text);
            border: 1px solid var(--border);
        }

        .footer-space {
            height: 40px;
        }

        .form-control,
        .form-select {
            background: rgba(0,0,0,0.25);
            color: var(--text);
            border-color: var(--border);
        }

        .form-control:focus,
        .form-select:focus {
            background: rgba(0,0,0,0.35);
            color: var(--text);
            border-color: #60a5fa;
            box-shadow: 0 0 0 0.2rem rgba(96,165,250,0.15);
        }

        .form-select option {
            background: #111827;
            color: #e5e7eb;
        }
    </style>
</head>

<body>
<nav class="navbar navbar-dark sticky-top">
    <div class="container-fluid">
        <div>
            <span class="navbar-brand mb-0 h1">Monitor VESC / Flipsky 75100</span>
            <span class="badge badge-soft">porta Flask {{ app_port }}</span>
            <span class="badge badge-soft">serial {{ vesc_port }}</span>
        </div>

        <div class="d-flex align-items-center">
            <span id="statusDot" class="status-dot"></span>
            <span id="statusText">Conectando...</span>
        </div>
    </div>
</nav>

<div class="container-fluid py-4">

    <div class="row g-3 mb-3">
        <div class="col-12">
            <div class="card-vesc p-3">
                <div class="d-flex justify-content-between align-items-center mb-3">
                    <div>
                        <h5 class="m-0">Teste seguro do motor</h5>
                        <div class="small-muted">
                            Teste por duty cycle baixo. Use com a roda/motor suspenso, sem carga e com mão longe do eixo.
                            Auto-stop em poucos segundos e limite máximo de duty aplicado no backend.
                        </div>
                    </div>
                    <span class="badge badge-soft" id="motorTestBadge">--</span>
                </div>

                <div class="row g-3 align-items-end">
                    <div class="col-12 col-md-2">
                        <label class="form-label small-muted">Direção</label>
                        <div class="d-flex gap-2">
                            <button id="btnMotorForward" class="btn btn-outline-success w-50" onclick="setMotorDirection('forward')">
                                Frente
                            </button>
                            <button id="btnMotorReverse" class="btn btn-outline-warning w-50" onclick="setMotorDirection('reverse')">
                                Trás
                            </button>
                        </div>
                    </div>

                    <div class="col-12 col-md-2">
                        <label class="form-label small-muted">Duty alvo (%)</label>
                        <input id="motor_test_duty" type="number" step="0.1" min="0" max="8" class="form-control" value="1.0">
                    </div>

                    <div class="col-12 col-md-2">
                        <button id="btnMotorStart" class="btn btn-success w-100" onclick="startMotorTest()">
                            Iniciar teste
                        </button>
                    </div>

                    <div class="col-6 col-md-1">
                        <button id="btnMotorDutyDown" class="btn btn-outline-light w-100" onclick="stepMotorTest(-1)">
                            -1%
                        </button>
                    </div>

                    <div class="col-6 col-md-1">
                        <button id="btnMotorDutyUp" class="btn btn-outline-light w-100" onclick="stepMotorTest(1)">
                            +1%
                        </button>
                    </div>

                    <div class="col-12 col-md-2">
                        <button id="btnMotorApplyDuty" class="btn btn-primary w-100" onclick="setMotorTestDuty()">
                            Aplicar duty
                        </button>
                    </div>

                    <div class="col-12 col-md-1">
                        <button id="btnMotorRegenBrake" class="btn btn-warning w-100" onclick="regenBrakeMotorTest()">
                            Frear
                        </button>
                    </div>

                    <div class="col-12 col-md-2">
                        <button id="btnMotorStop" class="btn btn-danger w-100" onclick="stopMotorTest()">
                            Parar motor
                        </button>
                    </div>
                </div>

                <div class="row g-3 mt-2">
                    <div class="col-12 col-md-2">
                        <div class="metric-label">Status</div>
                        <div id="motorTestStatus">--</div>
                    </div>

                    <div class="col-12 col-md-2">
                        <div class="metric-label">Direção atual</div>
                        <div id="motorTestDirection">--</div>
                    </div>

                    <div class="col-12 col-md-2">
                        <div class="metric-label">Duty atual</div>
                        <div><span id="motorTestDutyNow" class="metric-value fs-5">--</span> <span class="metric-unit">%</span></div>
                    </div>

                    <div class="col-12 col-md-2">
                        <div class="metric-label">Freio regen</div>
                        <div><span id="motorTestBrakeCurrent">--</span>A / <span id="motorTestBrakeDuration">--</span>s</div>
                    </div>

                    <div class="col-12 col-md-2">
                        <div class="metric-label">Limite seguro</div>
                        <div><span id="motorTestMaxDuty">--</span>% duty máx.</div>
                    </div>

                    <div class="col-12 col-md-2">
                        <div class="metric-label">Auto-stop</div>
                        <div><span id="motorTestMaxDuration">--</span>s</div>
                    </div>

                    <div class="col-12">
                        <div class="metric-label">Evento/erro</div>
                        <div id="motorTestEvent">--</div>
                    </div>
                </div>
            </div>
        </div>
    </div>


    <div class="row g-3 mb-3">
        <div class="col-12 col-md-6 col-xl-3">
            <div class="card-vesc p-3">
                <div class="metric-label">Tensão de entrada</div>
                <div>
                    <span id="v_in" class="metric-value">--</span>
                    <span class="metric-unit">V</span>
                </div>
                <div class="progress mt-3">
                    <div id="bar_vin" class="progress-bar" style="width:0%"></div>
                </div>
                <div class="small-muted mt-2">Bateria / barramento DC</div>
            </div>
        </div>

        <div class="col-12 col-md-6 col-xl-3">
            <div class="card-vesc p-3">
                <div class="metric-label">Bateria estimada</div>
                <div>
                    <span id="battery_percent_est" class="metric-value">--</span>
                    <span class="metric-unit">%</span>
                </div>
                <div class="progress mt-3">
                    <div id="bar_battery" class="progress-bar" style="width:0%"></div>
                </div>
                <div class="small-muted mt-2">Pelo cutoff real do XML</div>
            </div>
        </div>

        <div class="col-12 col-md-6 col-xl-3">
            <div class="card-vesc p-3">
                <div class="metric-label">RPM</div>
                <div>
                    <span id="rpm" class="metric-value">--</span>
                    <span class="metric-unit">rpm</span>
                </div>
                <div class="progress mt-3">
                    <div id="bar_rpm" class="progress-bar" style="width:0%"></div>
                </div>
                <div class="small-muted mt-2">Rotação elétrica/mecânica conforme VESC</div>
            </div>
        </div>

        <div class="col-12 col-md-6 col-xl-3">
            <div class="card-vesc p-3">
                <div class="metric-label">ID CAN em tempo real</div>
                <div>
                    <span id="display_can_id" class="metric-value">--</span>
                </div>
                <div class="small-muted mt-3">
                    app_controller_id da telemetria
                </div>
            </div>
        </div>
    </div>

    <div class="row g-3 mb-3">
        <div class="col-12 col-md-6 col-xl-3">
            <div class="card-vesc p-3">
                <div class="metric-label">Corrente motor</div>
                <div>
                    <span id="avg_motor_current" class="metric-value">--</span>
                    <span class="metric-unit">A</span>
                </div>
                <div class="progress mt-3">
                    <div id="bar_imotor" class="progress-bar" style="width:0%"></div>
                </div>
                <div class="small-muted mt-2">avg_motor_current</div>
            </div>
        </div>

        <div class="col-12 col-md-6 col-xl-3">
            <div class="card-vesc p-3">
                <div class="metric-label">Corrente entrada</div>
                <div>
                    <span id="avg_input_current" class="metric-value">--</span>
                    <span class="metric-unit">A</span>
                </div>
                <div class="progress mt-3">
                    <div id="bar_iin" class="progress-bar" style="width:0%"></div>
                </div>
                <div class="small-muted mt-2">avg_input_current</div>
            </div>
        </div>

        <div class="col-12 col-md-6 col-xl-3">
            <div class="card-vesc p-3">
                <div class="metric-label">Duty Cycle</div>
                <div>
                    <span id="duty_percent" class="metric-value">--</span>
                    <span class="metric-unit">%</span>
                </div>
                <div class="progress mt-3">
                    <div id="bar_duty" class="progress-bar" style="width:0%"></div>
                </div>
                <div class="small-muted mt-2">duty_cycle_now</div>
            </div>
        </div>

        <div class="col-12 col-md-6 col-xl-3">
            <div class="card-vesc p-3">
                <div class="metric-label">Potência entrada</div>
                <div>
                    <span id="input_power_w" class="metric-value">--</span>
                    <span class="metric-unit">W</span>
                </div>
                <div class="small-muted mt-3">Vin × corrente entrada</div>
            </div>
        </div>
    </div>

    <div class="row g-3 mb-3">
        <div class="col-12 col-xl-6">
            <div class="card-vesc p-3">
                <strong>RPM</strong>
                <div class="chart-box">
                    <canvas id="chartRpm"></canvas>
                </div>
            </div>
        </div>

        <div class="col-12 col-xl-6">
            <div class="card-vesc p-3">
                <strong>Tensão de entrada</strong>
                <div class="chart-box">
                    <canvas id="chartVin"></canvas>
                </div>
            </div>
        </div>

        <div class="col-12 col-xl-6">
            <div class="card-vesc p-3">
                <strong>Correntes</strong>
                <div class="chart-box">
                    <canvas id="chartCurrent"></canvas>
                </div>
            </div>
        </div>

        <div class="col-12 col-xl-6">
            <div class="card-vesc p-3">
                <strong>Temperaturas</strong>
                <div class="chart-box">
                    <canvas id="chartTemp"></canvas>
                </div>
            </div>
        </div>
    </div>

    <div class="row g-3">
        <div class="col-12 col-xl-4">
            <div class="card-vesc p-3 h-100">
                <h5>Status</h5>

                <div class="small-muted">Conexão</div>
                <div id="statusDetail" class="mb-2">--</div>

                <div class="small-muted">Firmware</div>
                <div id="firmware" class="mb-2">--</div>

                <div class="small-muted">Última atualização</div>
                <div id="lastUpdate" class="mb-2">--</div>

                <div class="small-muted">Erro</div>
                <div id="lastError" class="mb-2 text-break">--</div>

                <div class="small-muted">Portas detectadas</div>
                <div id="portsList" class="text-break">--</div>

                <hr style="border-color: rgba(255,255,255,0.12);">

                <div class="small-muted">Selecionar porta VESC</div>

                <div class="d-flex gap-2 mt-2">
                    <select id="serialPortSelect" class="form-select"></select>

                    <button class="btn btn-primary" onclick="applySerialPort()">
                        Aplicar
                    </button>

                    <button class="btn btn-outline-light" onclick="autoSelectSerialPort()">
                        Auto
                    </button>
                </div>

                <div class="small-muted mt-2" id="serialPortMessage">--</div>
            </div>
        </div>

        <div class="col-12 col-xl-8">
            <div class="card-vesc p-3">
                <div class="d-flex justify-content-between align-items-center mb-2">
                    <h5 class="m-0">Todos os dados recebidos do VESC</h5>
                    <span class="small-muted">raw measurements</span>
                </div>

                <div class="table-responsive" style="max-height: 520px; overflow:auto;">
                    <table class="table table-sm raw-table align-middle">
                        <thead>
                            <tr>
                                <th>Campo</th>
                                <th>Valor</th>
                            </tr>
                        </thead>
                        <tbody id="rawTable">
                            <tr>
                                <td colspan="2">Aguardando dados...</td>
                            </tr>
                        </tbody>
                    </table>
                </div>
            </div>
        </div>
    </div>
    <div class="row g-3 mb-3">
        <div class="col-12">
            <div class="card-vesc p-3">
                <div class="d-flex justify-content-between align-items-center mb-3">
                    <div>
                        <h5 class="m-0">Configuração real lida do VESC</h5>
                        <div class="small-muted">
                            Exporte no VESC Tool após A↓ Read App Configuration e M↓ Read Motor Configuration.
                        </div>
                    </div>
                    <span class="badge badge-soft" id="realConfigBadge">--</span>
                </div>

                <div class="row g-3 mb-3">
                    <div class="col-12 col-md-6">
                        <label class="form-label small-muted">Upload App Configuration XML</label>
                        <input id="app_config_xml" type="file" accept=".xml" class="form-control">
                        <button class="btn btn-outline-light mt-2" onclick="uploadRealConfig('app')">
                            Enviar App XML
                        </button>
                        <div class="small-muted mt-2" id="appConfigPath">--</div>
                    </div>

                    <div class="col-12 col-md-6">
                        <label class="form-label small-muted">Upload Motor Configuration XML</label>
                        <input id="motor_config_xml" type="file" accept=".xml" class="form-control">
                        <button class="btn btn-outline-light mt-2" onclick="uploadRealConfig('motor')">
                            Enviar Motor XML
                        </button>
                        <div class="small-muted mt-2" id="motorConfigPath">--</div>
                    </div>
                </div>

                <div class="row g-3 mb-3">
                    <div class="col-12 col-md-3">
                        <div class="metric-label">VESC ID / CAN ID</div>
                        <div><span id="real_vesc_id" class="metric-value fs-4">--</span></div>
                    </div>

                    <div class="col-12 col-md-3">
                        <div class="metric-label">App to Use</div>
                        <div><span id="real_app_to_use" class="metric-value fs-5">--</span></div>
                    </div>

                    <div class="col-12 col-md-3">
                        <div class="metric-label">CAN Baud Rate</div>
                        <div><span id="real_can_baud_rate" class="metric-value fs-5">--</span></div>
                    </div>

                    <div class="col-12 col-md-3">
                        <div class="metric-label">CAN Mode</div>
                        <div><span id="real_can_mode" class="metric-value fs-5">--</span></div>
                    </div>
                </div>

                <div class="row g-3 mb-3">
                    <div class="col-12 col-md-3">
                        <div class="metric-label">Motor Type</div>
                        <div><span id="real_motor_type" class="metric-value fs-5">--</span></div>
                    </div>

                    <div class="col-12 col-md-3">
                        <div class="metric-label">Sensor Mode</div>
                        <div><span id="real_sensor_mode" class="metric-value fs-5">--</span></div>
                    </div>

                    <div class="col-12 col-md-3">
                        <div class="metric-label">Motor Current Max</div>
                        <div><span id="real_motor_current_max" class="metric-value fs-5">--</span> <span class="metric-unit">A</span></div>
                    </div>

                    <div class="col-12 col-md-3">
                        <div class="metric-label">Battery Current Max</div>
                        <div><span id="real_battery_current_max" class="metric-value fs-5">--</span> <span class="metric-unit">A</span></div>
                    </div>
                </div>

                <div class="row g-3 mb-3">
                    <div class="col-12 col-md-3">
                        <div class="metric-label">Cutoff Start</div>
                        <div><span id="real_cutoff_start" class="metric-value fs-5">--</span> <span class="metric-unit">V</span></div>
                    </div>

                    <div class="col-12 col-md-3">
                        <div class="metric-label">Cutoff End</div>
                        <div><span id="real_cutoff_end" class="metric-value fs-5">--</span> <span class="metric-unit">V</span></div>
                    </div>

                    <div class="col-12 col-md-3">
                        <div class="metric-label">Max Duty</div>
                        <div><span id="real_max_duty" class="metric-value fs-5">--</span></div>
                    </div>

                    <div class="col-12 col-md-3">
                        <div class="metric-label">Max Wattage</div>
                        <div><span id="real_max_wattage" class="metric-value fs-5">--</span> <span class="metric-unit">W</span></div>
                    </div>
                </div>

                <div class="small-muted mb-2" id="realConfigMessage">--</div>

                <div class="row g-3">
                    <div class="col-12 col-xl-6">
                        <h6>App Configuration XML</h6>
                        <div class="table-responsive" style="max-height: 420px; overflow:auto;">
                            <table class="table table-sm raw-table align-middle">
                                <thead>
                                    <tr>
                                        <th>Campo</th>
                                        <th>Valor</th>
                                    </tr>
                                </thead>
                                <tbody id="appConfigTable">
                                    <tr><td colspan="2">Aguardando XML...</td></tr>
                                </tbody>
                            </table>
                        </div>
                    </div>

                    <div class="col-12 col-xl-6">
                        <h6>Motor Configuration XML</h6>
                        <div class="table-responsive" style="max-height: 420px; overflow:auto;">
                            <table class="table table-sm raw-table align-middle">
                                <thead>
                                    <tr>
                                        <th>Campo</th>
                                        <th>Valor</th>
                                    </tr>
                                </thead>
                                <tbody id="motorConfigTable">
                                    <tr><td colspan="2">Aguardando XML...</td></tr>
                                </tbody>
                            </table>
                        </div>
                    </div>
                </div>

            </div>
        </div>
    </div>

    <div class="row g-3 mb-3">
        <div class="col-12">
            <div class="card-vesc p-3">
                <div class="d-flex justify-content-between align-items-center mb-3">
                    <div>
                        <h5 class="m-0">TCP Bridge para VESC Tool</h5>
                        <div class="small-muted">
                            Liga uma ponte TCP para acessar o VESC Tool pela rede. Enquanto ligado, o monitor serial fica pausado.
                        </div>
                    </div>
                    <span class="badge badge-soft" id="tcpBridgeBadge">--</span>
                </div>

                <div class="row g-3 align-items-end">
                    <div class="col-12 col-md-2">
                        <label class="form-label small-muted">Porta TCP</label>
                        <input id="tcp_bridge_port" type="number" min="1" max="65535" class="form-control" value="65102">
                    </div>

                    <div class="col-12 col-md-2">
                        <button id="btnStartBridge" class="btn btn-success w-100" onclick="startTcpBridge()">
                            Ligar TCP Bridge
                        </button>
                    </div>

                    <div class="col-12 col-md-2">
                        <button id="btnStopBridge" class="btn btn-danger w-100" onclick="stopTcpBridge()">
                            Desligar
                        </button>
                    </div>

                    <div class="col-12 col-md-6">
                        <div class="small-muted">Conectar no VESC Tool usando:</div>
                        <div class="fs-5">
                            TCP →
                            <span id="tcpBridgeEndpoint">--</span>
                        </div>
                    </div>
                </div>

                <div class="row g-3 mt-2">
                    <div class="col-12 col-md-3">
                        <div class="metric-label">Status</div>
                        <div id="tcpBridgeStatus">--</div>
                    </div>

                    <div class="col-12 col-md-3">
                        <div class="metric-label">Cliente conectado</div>
                        <div id="tcpBridgeClient">--</div>
                    </div>

                    <div class="col-12 col-md-3">
                        <div class="metric-label">Bytes TCP → Serial</div>
                        <div id="tcpBridgeBytesToSerial">--</div>
                    </div>

                    <div class="col-12 col-md-3">
                        <div class="metric-label">Bytes Serial → TCP</div>
                        <div id="tcpBridgeBytesToTcp">--</div>
                    </div>
                </div>

                <div class="small-muted mt-3">
                    Erro/evento:
                    <span id="tcpBridgeError">--</span>
                </div>
            </div>
        </div>
    </div>



</div>

<div class="footer-space"></div>

<script>
const maxPoints = 160;

function numberValue(v) {
    const n = Number(v);
    return Number.isFinite(n) ? n : null;
}

function fmt(v, decimals = 2) {
    const n = Number(v);
    if (!Number.isFinite(n)) {
        if (v === null || v === undefined || v === "") return "--";
        return String(v);
    }
    return n.toFixed(decimals);
}

function fmtInt(v) {
    const n = Number(v);
    if (!Number.isFinite(n)) return "--";
    return Math.round(n).toString();
}

function setText(id, value) {
    const el = document.getElementById(id);
    if (el) el.textContent = value;
}

function updateSerialPortBox(serial) {
    if (!serial) return;

    const select = document.getElementById("serialPortSelect");

    if (select) {
        const selectedBefore = select.value;
        const ports = serial.ports || [];

        select.innerHTML = "";

        if (ports.length === 0) {
            const option = document.createElement("option");
            option.value = "";
            option.textContent = "Nenhuma porta detectada";
            select.appendChild(option);
        } else {
            for (const port of ports) {
                const option = document.createElement("option");
                option.value = port;
                option.textContent = port;

                if (port === serial.selected) {
                    option.selected = true;
                }

                select.appendChild(option);
            }
        }

        if (
            selectedBefore &&
            ports.includes(selectedBefore) &&
            document.activeElement === select
        ) {
            select.value = selectedBefore;
        }
    }

    const status = serial.exists ? "OK" : "não encontrada";
    const mode = serial.auto ? "auto" : "manual";

    setText(
        "serialPortMessage",
        `Porta atual: ${serial.selected || "--"} | ${status} | modo ${mode}`
    );
}


async function loadSerialPorts() {
    try {
        const res = await fetch("/api/serial-port", { cache: "no-store" });
        const payload = await res.json();

        if (payload.ok) {
            updateSerialPortBox(payload.serial);
        }
    } catch (e) {
        setText("serialPortMessage", "Erro ao listar portas: " + String(e));
    }
}


async function applySerialPort() {
    const select = document.getElementById("serialPortSelect");
    const port = select ? select.value : "";

    if (!port) {
        setText("serialPortMessage", "Nenhuma porta selecionada.");
        return;
    }

    try {
        const res = await fetch("/api/serial-port", {
            method: "POST",
            headers: {
                "Content-Type": "application/json"
            },
            body: JSON.stringify({ port: port })
        });

        const payload = await res.json();

        updateSerialPortBox(payload.serial);
        setText("serialPortMessage", payload.message || "--");

    } catch (e) {
        setText("serialPortMessage", "Erro ao aplicar porta: " + String(e));
    }
}


async function autoSelectSerialPort() {
    try {
        const res = await fetch("/api/serial-port/auto", {
            method: "POST",
            headers: {
                "Content-Type": "application/json"
            }
        });

        const payload = await res.json();

        updateSerialPortBox(payload.serial);
        setText("serialPortMessage", payload.message || "--");

    } catch (e) {
        setText("serialPortMessage", "Erro no auto-detect: " + String(e));
    }
}


function setBar(id, value, maxAbs) {
    const el = document.getElementById(id);
    if (!el) return;

    const n = Math.abs(Number(value));
    if (!Number.isFinite(n) || maxAbs <= 0) {
        el.style.width = "0%";
        return;
    }

    const pct = Math.max(0, Math.min(100, (n / maxAbs) * 100));
    el.style.width = pct + "%";
}

function setBarPercent(id, value) {
    const el = document.getElementById(id);
    if (!el) return;

    const n = Number(value);
    if (!Number.isFinite(n)) {
        el.style.width = "0%";
        return;
    }

    const pct = Math.max(0, Math.min(100, n));
    el.style.width = pct + "%";
}

function updateMotorTestBox(test) {
    if (!test) return;

    const dutyInput = document.getElementById("motor_test_duty");

    if (dutyInput && document.activeElement !== dutyInput) {
        dutyInput.value = Number(test.duty_magnitude_percent || Math.abs(test.duty_percent || 0)).toFixed(1);
        dutyInput.max = Number(test.max_duty_percent || 8).toFixed(1);
        dutyInput.min = "0";
    }

    const running = Boolean(test.running);
    const brakeRunning = Boolean(test.brake_running);
    const direction = test.direction || "forward";

    setText("motorTestBadge", running ? "MOTOR TEST ON" : (brakeRunning ? "REGEN BRAKE" : "MOTOR TEST OFF"));
    setText("motorTestDutyNow", fmt(test.duty_percent, 2));
    setText("motorTestStatus", running ? "Rodando" : (brakeRunning ? "Freando" : "Parado"));
    setText("motorTestDirection", test.direction_label || (direction === "reverse" ? "Trás" : "Frente"));
    setText("motorTestBrakeCurrent", fmt(test.regen_brake_current_a, 2));
    setText("motorTestBrakeDuration", fmt(test.regen_brake_duration_s, 1));
    setText("motorTestMaxDuty", fmt(test.max_duty_percent, 1));
    setText("motorTestMaxDuration", fmt(test.max_duration_s, 1));
    setText("motorTestEvent", test.last_error || test.last_event || "--");

    const startBtn = document.getElementById("btnMotorStart");
    const stopBtn = document.getElementById("btnMotorStop");
    const dutyUpBtn = document.getElementById("btnMotorDutyUp");
    const dutyDownBtn = document.getElementById("btnMotorDutyDown");
    const applyBtn = document.getElementById("btnMotorApplyDuty");
    const brakeBtn = document.getElementById("btnMotorRegenBrake");
    const forwardBtn = document.getElementById("btnMotorForward");
    const reverseBtn = document.getElementById("btnMotorReverse");

    if (startBtn) startBtn.disabled = running || brakeRunning;
    if (stopBtn) stopBtn.disabled = !(running || brakeRunning);
    if (dutyUpBtn) dutyUpBtn.disabled = brakeRunning;
    if (dutyDownBtn) dutyDownBtn.disabled = brakeRunning;
    if (applyBtn) applyBtn.disabled = brakeRunning;
    if (brakeBtn) brakeBtn.disabled = brakeRunning;

    if (forwardBtn) {
        forwardBtn.classList.toggle("btn-success", direction === "forward");
        forwardBtn.classList.toggle("btn-outline-success", direction !== "forward");
    }

    if (reverseBtn) {
        reverseBtn.classList.toggle("btn-warning", direction === "reverse");
        reverseBtn.classList.toggle("btn-outline-warning", direction !== "reverse");
    }
}

async function loadMotorTest() {
    try {
        const res = await fetch("/api/motor-test", { cache: "no-store" });
        const payload = await res.json();

        if (payload.ok) {
            updateMotorTestBox(payload.motor_test);
        }
    } catch (e) {
        setText("motorTestEvent", "Erro ao consultar teste do motor: " + String(e));
    }
}


async function startMotorTest() {
    const duty = Number(document.getElementById("motor_test_duty").value || 1.0);

    try {
        const res = await fetch("/api/motor-test/start", {
            method: "POST",
            headers: {
                "Content-Type": "application/json"
            },
            body: JSON.stringify({ duty_percent: duty })
        });

        const payload = await res.json();

        updateMotorTestBox(payload.motor_test);
        setText("motorTestEvent", payload.message || "--");

    } catch (e) {
        setText("motorTestEvent", "Erro ao iniciar teste: " + String(e));
    }
}


async function stopMotorTest() {
    try {
        const res = await fetch("/api/motor-test/stop", {
            method: "POST",
            headers: {
                "Content-Type": "application/json"
            }
        });

        const payload = await res.json();

        updateMotorTestBox(payload.motor_test);
        setText("motorTestEvent", payload.message || "--");

    } catch (e) {
        setText("motorTestEvent", "Erro ao parar motor: " + String(e));
    }
}


async function setMotorDirection(direction) {
    try {
        const res = await fetch("/api/motor-test/direction", {
            method: "POST",
            headers: {
                "Content-Type": "application/json"
            },
            body: JSON.stringify({ direction: direction })
        });

        const payload = await res.json();

        updateMotorTestBox(payload.motor_test);
        setText("motorTestEvent", payload.message || "--");

    } catch (e) {
        setText("motorTestEvent", "Erro ao alterar direção: " + String(e));
    }
}


async function regenBrakeMotorTest() {
    try {
        const res = await fetch("/api/motor-test/regen-brake", {
            method: "POST",
            headers: {
                "Content-Type": "application/json"
            },
            body: JSON.stringify({})
        });

        const payload = await res.json();

        updateMotorTestBox(payload.motor_test);
        setText("motorTestEvent", payload.message || "--");

    } catch (e) {
        setText("motorTestEvent", "Erro ao frear: " + String(e));
    }
}


async function setMotorTestDuty() {
    const duty = Number(document.getElementById("motor_test_duty").value || 0);

    try {
        const res = await fetch("/api/motor-test/set-duty", {
            method: "POST",
            headers: {
                "Content-Type": "application/json"
            },
            body: JSON.stringify({ duty_percent: duty })
        });

        const payload = await res.json();

        updateMotorTestBox(payload.motor_test);
        setText("motorTestEvent", payload.message || "--");

    } catch (e) {
        setText("motorTestEvent", "Erro ao aplicar duty: " + String(e));
    }
}


async function stepMotorTest(delta) {
    try {
        const res = await fetch("/api/motor-test/step", {
            method: "POST",
            headers: {
                "Content-Type": "application/json"
            },
            body: JSON.stringify({ delta_percent: delta })
        });

        const payload = await res.json();

        updateMotorTestBox(payload.motor_test);
        setText("motorTestEvent", payload.message || "--");

    } catch (e) {
        setText("motorTestEvent", "Erro ao ajustar duty: " + String(e));
    }
}


function displayValue(value, decimals = null) {
    if (value === null || value === undefined || value === "") return "--";

    const n = Number(value);

    if (Number.isFinite(n) && decimals !== null) {
        return n.toFixed(decimals);
    }

    return String(value);
}

function tableFromFlatConfig(tableId, flat) {
    const tbody = document.getElementById(tableId);

    if (!tbody) return;

    if (!flat || Object.keys(flat).length === 0) {
        tbody.innerHTML = "<tr><td colspan='2'>Nenhum XML carregado.</td></tr>";
        return;
    }

    const keys = Object.keys(flat).sort();
    let html = "";

    for (const key of keys) {
        let value = flat[key];

        if (typeof value === "object" && value !== null) {
            value = JSON.stringify(value);
        }

        html += `
            <tr>
                <td class="raw-key">${key}</td>
                <td class="raw-value">${value}</td>
            </tr>
        `;
    }

    tbody.innerHTML = html;
}

function updateRealConfigBox(realConfig) {
    if (!realConfig) return;

    const app = realConfig.app || {};
    const motor = realConfig.motor || {};
    const appSummary = realConfig.app_summary || {};
    const motorSummary = realConfig.motor_summary || {};

    const hasApp = Boolean(app.exists);
    const hasMotor = Boolean(motor.exists);

    let badge = "Sem XML";
    if (hasApp && hasMotor) {
        badge = "APP + MOTOR carregados";
    } else if (hasApp) {
        badge = "APP carregado";
    } else if (hasMotor) {
        badge = "MOTOR carregado";
    }

    setText("realConfigBadge", badge);

    setText("appConfigPath", hasApp ? `${app.path} | ${app.updated_at || "--"}` : "App XML não carregado.");
    setText("motorConfigPath", hasMotor ? `${motor.path} | ${motor.updated_at || "--"}` : "Motor XML não carregado.");

    setText("real_vesc_id", displayValue(appSummary.vesc_id));
    setText("real_app_to_use", displayValue(appSummary.app_to_use));
    setText("real_can_baud_rate", displayValue(appSummary.can_baud_rate));
    setText("real_can_mode", displayValue(appSummary.can_mode));

    setText("real_motor_type", displayValue(motorSummary.motor_type));
    setText("real_sensor_mode", displayValue(motorSummary.sensor_port_mode || motorSummary.foc_sensor_mode));
    setText("real_motor_current_max", displayValue(motorSummary.motor_current_max, 2));
    setText("real_battery_current_max", displayValue(motorSummary.battery_current_max, 2));
    setText("real_cutoff_start", displayValue(motorSummary.battery_cutoff_start, 2));
    setText("real_cutoff_end", displayValue(motorSummary.battery_cutoff_end, 2));
    setText("real_max_duty", displayValue(motorSummary.max_duty));
    setText("real_max_wattage", displayValue(motorSummary.max_wattage, 1));

    tableFromFlatConfig("appConfigTable", app.compact || app.flat || {});
    tableFromFlatConfig("motorConfigTable", motor.compact || motor.flat || {});

    const errors = [];

    if (app.error) errors.push("APP XML: " + app.error);
    if (motor.error) errors.push("MOTOR XML: " + motor.error);

    if (errors.length) {
        setText("realConfigMessage", errors.join(" | "));
    } else {
        setText("realConfigMessage", "Configurações reais carregadas dos XMLs exportados pelo VESC Tool.");
    }
}

async function loadRealConfig() {
    try {
        const res = await fetch("/api/real-config", { cache: "no-store" });
        const payload = await res.json();

        if (payload.ok) {
            updateRealConfigBox(payload.real_config);
        }
    } catch (e) {
        setText("realConfigMessage", "Erro ao carregar configuração real: " + String(e));
    }
}

async function uploadRealConfig(type) {
    const inputId = type === "app" ? "app_config_xml" : "motor_config_xml";
    const input = document.getElementById(inputId);

    if (!input || !input.files || input.files.length === 0) {
        setText("realConfigMessage", "Selecione um arquivo XML primeiro.");
        return;
    }

    const formData = new FormData();
    formData.append("type", type);
    formData.append("file", input.files[0]);

    try {
        const res = await fetch("/api/real-config/upload", {
            method: "POST",
            body: formData
        });

        const payload = await res.json();

        let msg = payload.message || "--";
        if (payload.saved_file) {
            msg += " | salvo: " + payload.saved_file;
        }

        setText("realConfigMessage", msg);

        if (payload.ok) {
            updateRealConfigBox(payload.real_config);
        }

    } catch (e) {
        setText("realConfigMessage", "Erro no upload: " + String(e));
    }
}

function updateTcpBridgeBox(bridge) {
    if (!bridge) return;

    const portInput = document.getElementById("tcp_bridge_port");

    if (portInput && document.activeElement !== portInput) {
        portInput.value = bridge.port || 65102;
    }

    let statusText = "Desligado";
    let badgeText = "TCP Bridge OFF";

    if (bridge.running) {
        statusText = "Ativo";
        badgeText = "TCP Bridge ON";
    } else if (bridge.enabled) {
        statusText = "Inicializando";
        badgeText = "Ligando...";
    }

    setText("tcpBridgeBadge", badgeText);
    setText("tcpBridgeStatus", statusText);
    setText("tcpBridgeEndpoint", `${window.location.hostname}:${bridge.port || 65102}`);
    setText("tcpBridgeClient", bridge.client_addr || "--");
    setText("tcpBridgeBytesToSerial", bridge.bytes_to_serial ?? 0);
    setText("tcpBridgeBytesToTcp", bridge.bytes_to_tcp ?? 0);
    setText("tcpBridgeError", bridge.last_error || bridge.last_event || "--");

    const startBtn = document.getElementById("btnStartBridge");
    const stopBtn = document.getElementById("btnStopBridge");

    if (startBtn) {
        startBtn.disabled = Boolean(bridge.enabled || bridge.running);
    }

    if (stopBtn) {
        stopBtn.disabled = !Boolean(bridge.enabled || bridge.running);
    }
}

async function loadTcpBridge() {
    try {
        const res = await fetch("/api/tcp-bridge", { cache: "no-store" });
        const payload = await res.json();

        if (payload.ok) {
            updateTcpBridgeBox(payload.bridge);
        }
    } catch (e) {
        setText("tcpBridgeError", "Erro ao consultar TCP Bridge: " + String(e));
    }
}

async function startTcpBridge() {
    const port = Number(document.getElementById("tcp_bridge_port").value || 65102);

    try {
        const res = await fetch("/api/tcp-bridge/start", {
            method: "POST",
            headers: {
                "Content-Type": "application/json"
            },
            body: JSON.stringify({ port: port })
        });

        const payload = await res.json();

        updateTcpBridgeBox(payload.bridge);
        setText("tcpBridgeError", payload.message || "--");

    } catch (e) {
        setText("tcpBridgeError", "Erro ao ligar TCP Bridge: " + String(e));
    }
}

async function stopTcpBridge() {
    try {
        const res = await fetch("/api/tcp-bridge/stop", {
            method: "POST",
            headers: {
                "Content-Type": "application/json"
            }
        });

        const payload = await res.json();

        updateTcpBridgeBox(payload.bridge);
        setText("tcpBridgeError", payload.message || "--");

    } catch (e) {
        setText("tcpBridgeError", "Erro ao desligar TCP Bridge: " + String(e));
    }
}

function createLineChart(canvasId, labels) {
    const ctx = document.getElementById(canvasId);

    return new Chart(ctx, {
        type: "line",
        data: {
            labels: [],
            datasets: labels.map(label => ({
                label: label,
                data: [],
                tension: 0.25,
                pointRadius: 0,
                borderWidth: 2
            }))
        },
        options: {
            responsive: true,
            maintainAspectRatio: false,
            animation: false,
            interaction: {
                intersect: false,
                mode: "index"
            },
            plugins: {
                legend: {
                    labels: {
                        color: "#e5e7eb"
                    }
                }
            },
            scales: {
                x: {
                    ticks: {
                        color: "#9ca3af",
                        maxTicksLimit: 8
                    },
                    grid: {
                        color: "rgba(255,255,255,0.06)"
                    }
                },
                y: {
                    ticks: {
                        color: "#9ca3af"
                    },
                    grid: {
                        color: "rgba(255,255,255,0.06)"
                    }
                }
            }
        }
    });
}

const chartRpm = createLineChart("chartRpm", ["RPM"]);
const chartVin = createLineChart("chartVin", ["Vin"]);
const chartCurrent = createLineChart("chartCurrent", ["Corrente motor", "Corrente entrada"]);
const chartTemp = createLineChart("chartTemp", ["Temp FET", "Temp motor"]);

function pushChart(chart, label, values) {
    chart.data.labels.push(label);

    values.forEach((value, idx) => {
        chart.data.datasets[idx].data.push(value);
    });

    while (chart.data.labels.length > maxPoints) {
        chart.data.labels.shift();
        chart.data.datasets.forEach(ds => ds.data.shift());
    }

    chart.update("none");
}

function updateStatus(state, ports) {
    const dot = document.getElementById("statusDot");

    if (state.connected) {
        dot.classList.remove("err");
        dot.classList.add("ok");
        setText("statusText", "Conectado");
        setText("statusDetail", "Conectado em " + state.port);
    } else {
        dot.classList.remove("ok");
        dot.classList.add("err");

        if (state.mode === "tcp_bridge") {
            setText("statusText", "TCP Bridge ativo");
            setText("statusDetail", "Monitor pausado para uso do VESC Tool via TCP");
        } else {
            setText("statusText", "Desconectado");
            setText("statusDetail", "Desconectado de " + state.port);
        }
    }

    setText("firmware", state.firmware || "--");
    setText("lastUpdate", state.last_update || "--");
    setText("lastError", state.last_error || "--");
    setText("portsList", ports && ports.length ? ports.join(" | ") : "--");
}

function updateCards(data) {
    setText("v_in", fmt(data.v_in, 2));
    setText("battery_percent_est", fmt(data.battery_percent_est, 1));
    setText("display_can_id", data.display_can_id ?? data.live_app_controller_id ?? data.app_controller_id ?? "--");

    setText("rpm", fmtInt(data.rpm));
    setText("avg_motor_current", fmt(data.avg_motor_current, 2));
    setText("avg_input_current", fmt(data.avg_input_current, 2));
    setText("duty_percent", fmt(data.duty_percent, 2));
    setText("input_power_w", fmt(data.input_power_w, 1));

    setBar("bar_vin", data.v_in, 84);
    setBarPercent("bar_battery", data.battery_percent_est);
    setBar("bar_rpm", data.rpm, 60000);
    setBar("bar_imotor", data.avg_motor_current, 100);
    setBar("bar_iin", data.avg_input_current, 100);
    setBar("bar_duty", data.duty_percent, 100);
}

function updateRawTable(data) {
    const tbody = document.getElementById("rawTable");
    const keys = Object.keys(data).sort();

    if (!keys.length) {
        tbody.innerHTML = "<tr><td colspan='2'>Aguardando dados...</td></tr>";
        return;
    }

    let html = "";

    for (const key of keys) {
        let value = data[key];

        if (typeof value === "object" && value !== null) {
            value = JSON.stringify(value);
        }

        html += `
            <tr>
                <td class="raw-key">${key}</td>
                <td class="raw-value">${value}</td>
            </tr>
        `;
    }

    tbody.innerHTML = html;
}

function updateCharts(data) {
    const label = data.time_label || new Date().toLocaleTimeString();

    pushChart(chartRpm, label, [
        numberValue(data.rpm)
    ]);

    pushChart(chartVin, label, [
        numberValue(data.v_in)
    ]);

    pushChart(chartCurrent, label, [
        numberValue(data.avg_motor_current),
        numberValue(data.avg_input_current)
    ]);

    pushChart(chartTemp, label, [
        numberValue(data.temp_fet),
        numberValue(data.temp_motor)
    ]);
}

async function loadData() {
    try {
        const res = await fetch("/api/data", { cache: "no-store" });
        const payload = await res.json();

        updateStatus(payload.state, payload.ports);

        if (payload.serial) {
            updateSerialPortBox(payload.serial);
        }

        if (payload.real_config) {
            updateRealConfigBox(payload.real_config);
        }

        if (payload.tcp_bridge) {
            updateTcpBridgeBox(payload.tcp_bridge);
        }

        if (payload.motor_test) {
            updateMotorTestBox(payload.motor_test);
        }

        if (payload.data && Object.keys(payload.data).length > 0) {
            updateCards(payload.data);
            updateRawTable(payload.data);
            updateCharts(payload.data);
        }
    } catch (e) {
        const dot = document.getElementById("statusDot");
        dot.classList.remove("ok");
        dot.classList.add("err");
        setText("statusText", "Erro na API Flask");
        setText("lastError", String(e));
    }
}

async function loadHistory() {
    try {
        const res = await fetch("/api/history", { cache: "no-store" });
        const payload = await res.json();

        if (!payload.history) return;

        for (const item of payload.history) {
            updateCharts(item);
        }
    } catch (e) {
        console.log(e);
    }
}

loadRealConfig();
loadTcpBridge();
loadMotorTest();
loadSerialPorts();
loadHistory();
loadData();

setInterval(loadData, 500);
setInterval(loadTcpBridge, 1000);
setInterval(loadMotorTest, 1000);
setInterval(loadSerialPorts, 2000);
setInterval(loadRealConfig, 3000);
</script>

</body>
</html>
"""


auto_select_active_vesc_port()


if __name__ == "__main__":
    thread = threading.Thread(target=vesc_reader_loop, daemon=True)
    thread.start()

    print("Monitor VESC iniciado")
    print(f"Porta serial: {get_active_vesc_port()}")
    print(f"Interface: http://0.0.0.0:{APP_PORT}")
    print(f"Intervalo leitura: {READ_INTERVAL}s")
    print(f"Diretório XML real: {REAL_CONFIG_DIR.resolve()}")
    print(f"TCP Bridge padrão: {TCP_BRIDGE_HOST}:{TCP_BRIDGE_DEFAULT_PORT}")
    print(f"Teste motor: duty max {MOTOR_TEST_MAX_DUTY_PERCENT}% | auto-stop {MOTOR_TEST_MAX_DURATION_S}s")
    print(f"Freio regenerativo teste: {MOTOR_TEST_REGEN_BRAKE_CURRENT_A}A por {MOTOR_TEST_REGEN_BRAKE_DURATION_S}s")

    app.run(host=APP_HOST, port=APP_PORT, debug=False, threaded=True)
