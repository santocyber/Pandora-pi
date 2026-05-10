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
        "deadman_button": "BTN_TR"
    },
    "lidar": {
        "port": "/dev/ttyUSB0",
        "baudrate": 230400,
        "min_distance": 150,
        "max_distance": 12000,
        "emit_interval": 0.08
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
    "deadman_ok": False
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
    </style>
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
        <div class="card">
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


@socketio.on("connect")
def socket_connect():
    emit_status()


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

    socketio.run(
        app,
        host="0.0.0.0",
        port=5005,
        allow_unsafe_werkzeug=True
    )
