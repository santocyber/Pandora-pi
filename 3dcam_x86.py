#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import glob
import time
import json
import signal
import subprocess
import threading
import traceback
import pwd

import cv2
import numpy as np
from flask import Flask, Response, jsonify, request, render_template_string


app = Flask(__name__)

camera_lock = threading.Lock()
depth_lock = threading.Lock()
bridge_lock = threading.Lock()

rgb_cap = None
rgb_thread = None
depth_thread = None
depth_bridge_proc = None
depth_bridge_log_handle = None
depth_bridge_started_at = 0.0

running = False

latest_rgb = None
latest_depth_raw = None
latest_depth_colored = None
latest_points = []

last_rgb_error = ""
last_depth_error = ""
last_usb_fix_result = {}
last_depth_meta = {}

RAW_PATH = "/tmp/orbbec_depth.raw"
META_PATH = "/tmp/orbbec_depth_meta.json"
LOG_PATH = "/tmp/orbbec_depth_bridge.log"

BRIDGE_FIRST_FRAME_GRACE_SEC = 90.0
BRIDGE_RESTART_COOLDOWN_SEC = 25.0
POINTCLOUD_INTERVAL_SEC = 0.25


def get_real_home():
    sudo_user = os.environ.get("SUDO_USER", "").strip()
    if sudo_user:
        try:
            return pwd.getpwnam(sudo_user).pw_dir
        except Exception:
            return os.path.join("/home", sudo_user)
    return os.path.expanduser("~")


REAL_HOME = get_real_home()

PROJECT_ROOT = os.environ.get(
    "PANDORAPI_ROOT",
    os.path.join(REAL_HOME, "PROJETOTESTE", "pandorapi")
)

ORBBEC_SDK_ROOT = os.environ.get(
    "ORBBEC_SDK_ROOT",
    os.path.join(PROJECT_ROOT, "OrbbecSDK")
)

BRIDGE_PATH = os.environ.get(
    "ORBBEC_BRIDGE",
    os.path.join(ORBBEC_SDK_ROOT, "build", "bin", "orbbec_depth_bridge")
)

ORBBEC_LIB_DIR = os.path.join(ORBBEC_SDK_ROOT, "lib", "linux_x64")

config = {
    "rgb_device": "/dev/video2",
    "width": 640,
    "height": 480,
    "fps": 30,

    "near_mm": 200,
    "far_mm": 4000,

    "jpeg_quality": 85,

    "fx": 575.0,
    "fy": 575.0,

    "mirror_rgb": False,
    "mirror_depth": False,
    "start_rgb_immediately": True,

    "depth_display_scale": 1.6,
    "depth_colormap": "TURBO",
    "depth_color_direction": "NEAR_HOT",
    "depth_labels": True,
    "depth_grid": 4,
    "depth_sample_radius": 7,
    "depth_median_blur": True,
    "depth_show_legend": True,
    "depth_invalid_gray": False,
    "depth_invalid_contours": True,
}


HTML = r"""
<!doctype html>
<html lang="pt-BR">
<head>
    <meta charset="utf-8">
    <title>Orbbec Astra Pro - OrbbecSDK v1 + Flask</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">

    <link
        href="https://cdn.jsdelivr.net/npm/bootstrap@5.3.3/dist/css/bootstrap.min.css"
        rel="stylesheet"
    >
    <style>
        body {
            background: #0d1117;
            color: #e6edf3;
        }

        h1, h2, h3, h4, h5, h6,
        label,
        .form-label,
        .form-check-label,
        .card h4,
        .card h5 {
            color: #e6edf3 !important;
            opacity: 1 !important;
        }

        .text-muted,
        .small-muted {
            color: #a9b4c2 !important;
            opacity: 1 !important;
        }

        .card {
            background: #161b22;
            border: 1px solid #30363d;
            border-radius: 16px;
        }

        .form-control,
        .form-select {
            background: #0d1117;
            color: #f0f6fc !important;
            border-color: #30363d;
        }

        .form-control::placeholder {
            color: #8b949e !important;
        }

        .form-control:disabled,
        .form-select:disabled {
            background: #111820 !important;
            color: #c9d1d9 !important;
            opacity: 1 !important;
        }

        option {
            background: #0d1117;
            color: #f0f6fc;
        }

        .form-control:focus,
        .form-select:focus {
            background: #0d1117;
            color: #e6edf3;
            border-color: #58a6ff;
            box-shadow: none;
        }

        .form-check-input {
            background-color: #0d1117;
            border-color: #30363d;
        }

        .preview-img {
            width: 100%;
            background: #010409;
            border: 1px solid #30363d;
            border-radius: 12px;
            min-height: 260px;
            object-fit: contain;
        }

        #depthPreview {
            min-height: 480px;
            image-rendering: auto;
        }

        .depth-help {
            color: #a9b4c2;
            font-size: 0.85rem;
            margin-top: 0.5rem;
        }

        pre {
            background: #010409;
            color: #7ee787;
            border: 1px solid #30363d;
            border-radius: 12px;
            padding: 1rem;
            height: 330px;
            overflow-y: auto;
            font-size: 0.80rem;
        }

        .small-muted {
            color: #8b949e;
            font-size: 0.9rem;
        }

        .btn {
            border-radius: 10px;
        }

        code {
            color: #ffa657;
        }

        .badge-status {
            font-size: 0.95rem;
            padding: 0.6rem 0.8rem;
        }
    </style>
</head>

<body>
<div class="container-fluid py-4">

    <div class="d-flex flex-wrap justify-content-between align-items-center mb-4">
        <div>
            <h1 class="mb-1">Orbbec Astra Pro</h1>
            <div class="small-muted">
                RGB via OpenCV + Depth colorido via OrbbecSDK v1 com distâncias
            </div>
        </div>

        <div class="mt-3 mt-md-0">
            <span id="statusBadge" class="badge bg-secondary badge-status">parado</span>
        </div>
    </div>

    <div class="row g-4">

        <div class="col-xl-3 col-lg-4">
            <div class="card p-3 mb-4">
                <h4>Controles</h4>

                <div class="mb-3">
                    <label class="form-label">Câmera RGB</label>
                    <div class="input-group">
                        <select id="rgbDevice" class="form-select"></select>
                        <button class="btn btn-outline-info" onclick="loadDevices()">Atualizar</button>
                    </div>
                    <div class="small-muted mt-1">
                        Preferencial: <code>/dev/video2</code> quando for Astra Pro HD Camera.
                    </div>
                </div>

                <div class="row g-2">
                    <div class="col-6">
                        <label class="form-label">RGB largura</label>
                        <input id="width" class="form-control" type="number" value="640">
                    </div>

                    <div class="col-6">
                        <label class="form-label">RGB altura</label>
                        <input id="height" class="form-control" type="number" value="480">
                    </div>

                    <div class="col-6">
                        <label class="form-label">RGB FPS</label>
                        <input id="fps" class="form-control" type="number" value="30">
                    </div>

                    <div class="col-6">
                        <label class="form-label">JPEG</label>
                        <input id="jpegQuality" class="form-control" type="number" value="85">
                    </div>
                </div>

                <hr>

                <div class="row g-2">
                    <div class="col-6">
                        <label class="form-label">Perto mm</label>
                        <input id="nearMm" class="form-control" type="number" value="200">
                    </div>

                    <div class="col-6">
                        <label class="form-label">Longe mm</label>
                        <input id="farMm" class="form-control" type="number" value="4000">
                    </div>
                </div>

                <hr>

                <h5 class="mt-2 mb-2">Depth / distância</h5>

                <div class="row g-2">
                    <div class="col-6">
                        <label class="form-label">Escala preview</label>
                        <input id="depthDisplayScale" class="form-control" type="text" value="1.6">
                    </div>

                    <div class="col-6">
                        <label class="form-label">Grade dist.</label>
                        <input id="depthGrid" class="form-control" type="number" value="4">
                    </div>

                    <div class="col-6">
                        <label class="form-label">Raio amostra</label>
                        <input id="depthSampleRadius" class="form-control" type="number" value="7">
                    </div>

                    <div class="col-6">
                        <label class="form-label">Mapa cor</label>
                        <select id="depthColormap" class="form-select">
                            <option value="TURBO" selected>TURBO</option>
                            <option value="JET">JET</option>
                            <option value="VIRIDIS">VIRIDIS</option>
                            <option value="PLASMA">PLASMA</option>
                            <option value="MAGMA">MAGMA</option>
                            <option value="BONE">BONE</option>
                        </select>
                    </div>

                    <div class="col-12">
                        <label class="form-label">Direção da cor</label>
                        <select id="depthColorDirection" class="form-select">
                            <option value="NEAR_HOT" selected>Perto quente / longe frio</option>
                            <option value="FAR_HOT">Perto frio / longe quente</option>
                        </select>
                    </div>
                </div>

                <hr>

                <div class="form-check mb-2">
                    <input id="mirrorRgb" class="form-check-input" type="checkbox">
                    <label class="form-check-label" for="mirrorRgb">Espelhar RGB</label>
                </div>

                <div class="form-check mb-2">
                    <input id="mirrorDepth" class="form-check-input" type="checkbox">
                    <label class="form-check-label" for="mirrorDepth">Espelhar depth</label>
                </div>

                <div class="form-check mb-2">
                    <input id="depthLabels" class="form-check-input" type="checkbox" checked>
                    <label class="form-check-label" for="depthLabels">Mostrar distâncias no depth</label>
                </div>

                <div class="form-check mb-2">
                    <input id="depthMedianBlur" class="form-check-input" type="checkbox" checked>
                    <label class="form-check-label" for="depthMedianBlur">Suavizar ruído do depth</label>
                </div>

                <div class="form-check mb-2">
                    <input id="depthShowLegend" class="form-check-input" type="checkbox" checked>
                    <label class="form-check-label" for="depthShowLegend">Mostrar legenda de cores</label>
                </div>

                <div class="form-check mb-2">
                    <input id="depthInvalidContours" class="form-check-input" type="checkbox" checked>
                    <label class="form-check-label" for="depthInvalidContours">Marcar áreas sem leitura</label>
                </div>

                <div class="form-check mb-2">
                    <input id="depthInvalidGray" class="form-check-input" type="checkbox">
                    <label class="form-check-label" for="depthInvalidGray">Sem leitura em cinza</label>
                </div>

                <div class="form-check mb-3">
                    <input id="startRgbImmediately" class="form-check-input" type="checkbox" checked>
                    <label class="form-check-label" for="startRgbImmediately">Abrir RGB imediatamente</label>
                </div>

                <div class="d-grid gap-2">
                    <button class="btn btn-success" onclick="startCamera()">Iniciar câmera</button>
                    <button class="btn btn-warning" onclick="stopCamera()">Parar câmera</button>
                    <button class="btn btn-info" onclick="applyUsbFix()">Aplicar correção USB</button>
                    <button class="btn btn-danger" onclick="restartDepthClean()">Reiniciar depth limpo</button>
                    <button class="btn btn-primary" onclick="refreshStatus()">Atualizar status</button>
                </div>
            </div>

            <div class="card p-3">
                <h4>Status</h4>
                <pre id="statusBox">Carregando...</pre>
            </div>
        </div>

        <div class="col-xl-9 col-lg-8">
            <div class="row g-4">
                <div class="col-xl-6 col-12">
                    <div class="card p-3">
                        <h4>Imagem normal RGB</h4>
                        <img id="rgbPreview" class="preview-img" src="/video/rgb">
                    </div>
                </div>

                <div class="col-xl-6 col-12">
                    <div class="card p-3">
                        <h4>Depth colorido</h4>
                        <img id="depthPreview" class="preview-img" src="/video/depth">
                        <div class="depth-help">
                            Preto = sem leitura real do sensor. As distâncias são calculadas pela mediana ao redor de cada ponto.
                        </div>
                    </div>
                </div>
</div>
        </div>

    </div>
</div>

<script>

async function apiGet(url) {
    const response = await fetch(url);
    return await response.json();
}

async function apiPost(url, data) {
    const response = await fetch(url, {
        method: "POST",
        headers: {"Content-Type": "application/json"},
        body: JSON.stringify(data)
    });

    return await response.json();
}

function setBadge(text, cls) {
    const badge = document.getElementById("statusBadge");
    badge.textContent = text;
    badge.className = "badge badge-status " + cls;
}

function parseFloatBR(value, fallback) {
    if (value === null || value === undefined) {
        return fallback;
    }

    const cleaned = String(value).replace(",", ".").trim();
    const parsed = parseFloat(cleaned);

    if (Number.isNaN(parsed)) {
        return fallback;
    }

    return parsed;
}

async function loadDevices() {
    const result = await apiGet("/api/devices");
    const select = document.getElementById("rgbDevice");

    select.innerHTML = "";

    if (!result.ok || result.devices.length === 0) {
        const opt = document.createElement("option");
        opt.value = "/dev/video2";
        opt.textContent = "/dev/video2";
        select.appendChild(opt);
        return;
    }

    for (const dev of result.devices) {
        const opt = document.createElement("option");
        opt.value = dev.device || dev;
        opt.textContent = dev.label || dev.device || dev;
        select.appendChild(opt);
    }

    if (result.default_device) {
        select.value = result.default_device;
    }
}

function collectConfig() {
    return {
        rgb_device: document.getElementById("rgbDevice").value,
        width: parseInt(document.getElementById("width").value || "640"),
        height: parseInt(document.getElementById("height").value || "480"),
        fps: parseInt(document.getElementById("fps").value || "30"),

        near_mm: parseInt(document.getElementById("nearMm").value || "200"),
        far_mm: parseInt(document.getElementById("farMm").value || "4000"),

        jpeg_quality: parseInt(document.getElementById("jpegQuality").value || "85"),

        depth_display_scale: parseFloatBR(document.getElementById("depthDisplayScale").value, 1.6),
        depth_grid: parseInt(document.getElementById("depthGrid").value || "4"),
        depth_sample_radius: parseInt(document.getElementById("depthSampleRadius").value || "7"),
        depth_colormap: document.getElementById("depthColormap").value,
        depth_color_direction: document.getElementById("depthColorDirection").value,
        depth_labels: document.getElementById("depthLabels").checked,
        depth_median_blur: document.getElementById("depthMedianBlur").checked,
        depth_show_legend: document.getElementById("depthShowLegend").checked,
        depth_invalid_contours: document.getElementById("depthInvalidContours").checked,
        depth_invalid_gray: document.getElementById("depthInvalidGray").checked,

        mirror_rgb: document.getElementById("mirrorRgb").checked,
        mirror_depth: document.getElementById("mirrorDepth").checked,
        start_rgb_immediately: document.getElementById("startRgbImmediately").checked
    };
}

async function startCamera() {
    const result = await apiPost("/api/start", collectConfig());

    if (result.ok) {
        setBadge("rodando", "bg-success");
        refreshStreams();
    } else {
        alert(result.error || "Erro ao iniciar câmera");
        setBadge("erro", "bg-danger");
    }

    await refreshStatus();
}

async function stopCamera() {
    const result = await apiPost("/api/stop", {});

    if (result.ok) {
        setBadge("parado", "bg-secondary");
    }

    await refreshStatus();
}

async function applyUsbFix() {
    const result = await apiPost("/api/usb_fix", {});
    document.getElementById("statusBox").textContent = JSON.stringify(result, null, 2);
}

async function restartDepthClean() {
    setBadge("reiniciando depth", "bg-warning");

    const result = await apiPost("/api/restart_depth_clean", {});
    document.getElementById("statusBox").textContent = JSON.stringify(result, null, 2);

    if (result.ok) {
        setBadge("rodando", "bg-success");
        refreshStreams();
    } else {
        setBadge("depth falhou", "bg-danger");
    }

    await refreshStatus();
}

async function refreshStatus() {
    const result = await apiGet("/api/status");
    document.getElementById("statusBox").textContent = JSON.stringify(result, null, 2);

    if (result.running) {
        setBadge("rodando", "bg-success");
    } else {
        setBadge("parado", "bg-secondary");
    }
}

function refreshStreams() {
    const ts = Date.now();
    document.getElementById("rgbPreview").src = "/video/rgb?t=" + ts;
    document.getElementById("depthPreview").src = "/video/depth?t=" + ts;
}



window.addEventListener("load", async function() {
    await loadDevices();
    await refreshStatus();

    setInterval(refreshStatus, 3000);
});
</script>
</body>
</html>
"""


def safe_int(value, default, min_value=None, max_value=None):
    try:
        value = int(value)
    except Exception:
        return default

    if min_value is not None:
        value = max(min_value, value)

    if max_value is not None:
        value = min(max_value, value)

    return value


def safe_float(value, default, min_value=None, max_value=None):
    try:
        if isinstance(value, str):
            value = value.replace(",", ".")
        value = float(value)
    except Exception:
        return default

    if min_value is not None:
        value = max(min_value, value)

    if max_value is not None:
        value = min(max_value, value)

    return value


def tail_file(path, max_chars=8000):
    try:
        if not os.path.exists(path):
            return ""

        with open(path, "rb") as f:
            f.seek(0, os.SEEK_END)
            size = f.tell()
            f.seek(max(0, size - max_chars), os.SEEK_SET)
            return f.read().decode("utf-8", errors="replace")
    except Exception as exc:
        return str(exc)


def shell_output(cmd, timeout=3):
    try:
        out = subprocess.check_output(
            cmd,
            stderr=subprocess.STDOUT,
            timeout=timeout,
            text=True
        )
        return out
    except Exception:
        return ""


def parse_v4l2_devices():
    out = shell_output(["v4l2-ctl", "--list-devices"], timeout=3)
    items = []
    current_name = ""

    for raw_line in out.splitlines():
        line = raw_line.rstrip()

        if not line:
            current_name = ""
            continue

        if not line.startswith("\t") and not line.startswith(" "):
            current_name = line.strip()
            continue

        dev = line.strip()
        if dev.startswith("/dev/video"):
            items.append({
                "device": dev,
                "name": current_name,
                "label": f"{dev} - {current_name}" if current_name else dev
            })

    if not items:
        for dev in sorted(glob.glob("/dev/video*")):
            items.append({
                "device": dev,
                "name": "",
                "label": dev
            })

    return items


def pick_orbbec_rgb_device():
    items = parse_v4l2_devices()

    for item in items:
        name = item.get("name", "").lower()
        dev = item.get("device", "")

        if "astra pro hd camera" in name or ("astra" in name and "camera" in name):
            return dev

    if "/dev/video2" in [i["device"] for i in items]:
        return "/dev/video2"

    return items[0]["device"] if items else "/dev/video2"


def apply_usb_fixes():
    result = {
        "autosuspend": None,
        "usbfs_memory_mb": None,
        "orbbec_devices": [],
        "errors": []
    }

    try:
        path = "/sys/module/usbcore/parameters/autosuspend"
        if os.path.exists(path):
            with open(path, "w") as f:
                f.write("-1\n")
            with open(path, "r") as f:
                result["autosuspend"] = f.read().strip()
    except Exception as exc:
        result["errors"].append("autosuspend: " + str(exc))

    try:
        path = "/sys/module/usbcore/parameters/usbfs_memory_mb"
        if os.path.exists(path):
            with open(path, "w") as f:
                f.write("1000\n")
            with open(path, "r") as f:
                result["usbfs_memory_mb"] = f.read().strip()
    except Exception as exc:
        result["errors"].append("usbfs_memory_mb: " + str(exc))

    for dev in glob.glob("/sys/bus/usb/devices/*"):
        try:
            vid_path = os.path.join(dev, "idVendor")
            pid_path = os.path.join(dev, "idProduct")

            if not os.path.isfile(vid_path) or not os.path.isfile(pid_path):
                continue

            with open(vid_path, "r") as f:
                vid = f.read().strip().lower()

            if vid != "2bc5":
                continue

            with open(pid_path, "r") as f:
                pid = f.read().strip().lower()

            item = {
                "device": os.path.basename(dev),
                "vid": vid,
                "pid": pid,
                "power_control": None,
                "autosuspend_delay_ms": None
            }

            power_control = os.path.join(dev, "power", "control")
            autosuspend_delay = os.path.join(dev, "power", "autosuspend_delay_ms")

            if os.path.exists(power_control):
                with open(power_control, "w") as f:
                    f.write("on\n")
                with open(power_control, "r") as f:
                    item["power_control"] = f.read().strip()

            if os.path.exists(autosuspend_delay):
                with open(autosuspend_delay, "w") as f:
                    f.write("0\n")
                with open(autosuspend_delay, "r") as f:
                    item["autosuspend_delay_ms"] = f.read().strip()

            result["orbbec_devices"].append(item)

        except Exception as exc:
            result["errors"].append("usb device: " + str(exc))

    return result



def reset_orbbec_depth_usb():
    result = {
        "ok": False,
        "device": None,
        "message": ""
    }

    try:
        target = None

        for dev in glob.glob("/sys/bus/usb/devices/*"):
            vid_path = os.path.join(dev, "idVendor")
            pid_path = os.path.join(dev, "idProduct")

            if not os.path.isfile(vid_path) or not os.path.isfile(pid_path):
                continue

            with open(vid_path, "r") as f:
                vid = f.read().strip().lower()

            with open(pid_path, "r") as f:
                pid = f.read().strip().lower()

            if vid == "2bc5" and pid == "0403":
                target = os.path.basename(dev)
                break

        if not target:
            result["message"] = "Dispositivo depth 2bc5:0403 não encontrado"
            return result

        result["device"] = target

        with open("/sys/bus/usb/drivers/usb/unbind", "w") as f:
            f.write(target)

        time.sleep(3.0)

        with open("/sys/bus/usb/drivers/usb/bind", "w") as f:
            f.write(target)

        time.sleep(5.0)

        result["ok"] = True
        result["message"] = "Depth USB 2bc5:0403 resetado com sucesso"
        return result

    except Exception as exc:
        result["message"] = str(exc)
        return result


def make_error_image(text, width=640, height=480):
    img = np.zeros((height, width, 3), dtype=np.uint8)
    img[:, :] = (10, 10, 10)

    words = str(text).split(" ")
    lines = []
    current = ""

    for word in words:
        if len(current + " " + word) > 48:
            lines.append(current)
            current = word
        else:
            current = (current + " " + word).strip()

    if current:
        lines.append(current)

    y = 55

    for line in lines[:11]:
        cv2.putText(
            img,
            line,
            (25, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (0, 190, 255),
            2,
            cv2.LINE_AA
        )
        y += 34

    return img


def encode_jpeg(frame):
    quality = safe_int(config.get("jpeg_quality", 85), 85, 20, 100)

    ok, buffer = cv2.imencode(
        ".jpg",
        frame,
        [int(cv2.IMWRITE_JPEG_QUALITY), quality]
    )

    if not ok:
        return None

    return buffer.tobytes()


def frame_to_mjpeg(frame):
    jpg = encode_jpeg(frame)

    if jpg is None:
        return b""

    return (
        b"--frame\r\n"
        b"Content-Type: image/jpeg\r\n\r\n" +
        jpg +
        b"\r\n"
    )


def mjpeg_response(generator):
    return Response(
        generator,
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )


def list_video_devices():
    return parse_v4l2_devices()


def find_default_rgb_device():
    current = config.get("rgb_device", "")

    if current and os.path.exists(current):
        return current

    return pick_orbbec_rgb_device()


def try_open_video_device(device):
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(config["width"]))
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(config["height"]))
    cap.set(cv2.CAP_PROP_FPS, int(config["fps"]))
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    if not cap.isOpened():
        cap.release()
        return None

    ret, frame = cap.read()

    if not ret or frame is None:
        cap.release()
        return None

    return cap


def open_rgb_camera():
    global rgb_cap
    global last_rgb_error

    requested = config.get("rgb_device", "/dev/video2")
    preferred = pick_orbbec_rgb_device()

    candidates = []

    for dev in [requested, preferred, "/dev/video2", "/dev/video1", "/dev/video0"]:
        if dev and dev not in candidates and os.path.exists(dev):
            candidates.append(dev)

    errors = []

    for dev in candidates:
        cap = try_open_video_device(dev)

        if cap is not None:
            rgb_cap = cap
            config["rgb_device"] = dev
            last_rgb_error = ""
            return

        errors.append(dev)

    raise RuntimeError(
        "Não foi possível abrir RGB. Tentados: " + ", ".join(errors)
    )



def close_rgb_camera():
    global rgb_cap
    global latest_rgb

    if rgb_cap is not None:
        try:
            rgb_cap.release()
        except Exception:
            pass

    rgb_cap = None
    latest_rgb = None


def rgb_worker():
    global latest_rgb
    global running
    global rgb_cap
    global last_rgb_error

    while running:
        try:
            if rgb_cap is None:
                time.sleep(0.05)
                continue

            ret, frame = rgb_cap.read()

            if not ret or frame is None:
                time.sleep(0.05)
                continue

            if config.get("mirror_rgb", False):
                frame = cv2.flip(frame, 1)

            with camera_lock:
                latest_rgb = frame.copy()

            last_rgb_error = ""

        except Exception:
            last_rgb_error = traceback.format_exc()
            time.sleep(0.1)


def start_rgb_now():
    global rgb_thread
    global last_rgb_error

    try:
        if rgb_cap is None:
            open_rgb_camera()

        if rgb_thread is None or not rgb_thread.is_alive():
            rgb_thread = threading.Thread(target=rgb_worker, daemon=True)
            rgb_thread.start()

        last_rgb_error = ""
    except Exception:
        last_rgb_error = traceback.format_exc()


def is_depth_bridge_alive():
    return depth_bridge_proc is not None and depth_bridge_proc.poll() is None


def bridge_runtime_sec():
    if depth_bridge_started_at <= 0:
        return 0.0

    return time.time() - depth_bridge_started_at


def start_depth_bridge():
    global depth_bridge_proc
    global depth_bridge_log_handle
    global depth_bridge_started_at
    global last_depth_error

    with bridge_lock:
        if is_depth_bridge_alive():
            return

        if not os.path.exists(BRIDGE_PATH):
            raise RuntimeError(
                "Bridge OrbbecSDK não encontrado: "
                + BRIDGE_PATH
                + ". Compile orbbec_depth_bridge.cpp primeiro."
            )

        for path in [RAW_PATH, META_PATH]:
            try:
                if os.path.exists(path):
                    os.remove(path)
            except Exception:
                pass

        try:
            if os.path.exists(LOG_PATH) and os.path.getsize(LOG_PATH) > 2 * 1024 * 1024:
                os.remove(LOG_PATH)
        except Exception:
            pass

        env = os.environ.copy()
        old_ld = env.get("LD_LIBRARY_PATH", "")
        env["LD_LIBRARY_PATH"] = ORBBEC_LIB_DIR + ":" + old_ld

        depth_bridge_log_handle = open(LOG_PATH, "ab", buffering=0)

        last_depth_error = ""
        depth_bridge_started_at = time.time()

        depth_bridge_proc = subprocess.Popen(
            [BRIDGE_PATH, RAW_PATH, META_PATH],
            cwd=os.path.dirname(BRIDGE_PATH),
            stdout=depth_bridge_log_handle,
            stderr=subprocess.STDOUT,
            start_new_session=True,
            env=env
        )


def stop_depth_bridge():
    global depth_bridge_proc
    global depth_bridge_log_handle
    global depth_bridge_started_at

    with bridge_lock:
        if depth_bridge_proc is not None:
            try:
                if depth_bridge_proc.poll() is None:
                    try:
                        os.killpg(os.getpgid(depth_bridge_proc.pid), signal.SIGTERM)
                    except Exception:
                        depth_bridge_proc.terminate()

                    try:
                        depth_bridge_proc.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        try:
                            os.killpg(os.getpgid(depth_bridge_proc.pid), signal.SIGKILL)
                        except Exception:
                            depth_bridge_proc.kill()
            except Exception:
                pass

        depth_bridge_proc = None
        depth_bridge_started_at = 0.0

        if depth_bridge_log_handle is not None:
            try:
                depth_bridge_log_handle.close()
            except Exception:
                pass

        depth_bridge_log_handle = None


def depth_files_are_fresh(max_age_sec=3.0):
    if not os.path.exists(RAW_PATH) or not os.path.exists(META_PATH):
        return False

    try:
        now = time.time()
        raw_age = now - os.path.getmtime(RAW_PATH)
        meta_age = now - os.path.getmtime(META_PATH)

        return raw_age <= max_age_sec and meta_age <= max_age_sec
    except Exception:
        return False


def read_depth_raw():
    global last_depth_meta
    global last_depth_error

    if not os.path.exists(META_PATH):
        last_depth_error = "Aguardando meta do bridge: " + META_PATH
        return None

    if not os.path.exists(RAW_PATH):
        last_depth_error = "Aguardando raw do bridge: " + RAW_PATH
        return None

    try:
        with open(META_PATH, "r") as f:
            meta = json.load(f)

        width = int(meta.get("width", 0))
        height = int(meta.get("height", 0))
        data_size = int(meta.get("data_size", 0))

        if width <= 0 or height <= 0:
            last_depth_error = "Meta inválido do bridge: width/height zerado"
            return None

        expected = width * height * 2

        if data_size and data_size < expected:
            last_depth_error = f"Meta data_size inválido: {data_size}, esperado {expected}"
            return None

        size = os.path.getsize(RAW_PATH)

        if size < expected:
            last_depth_error = f"RAW incompleto: {size} bytes, esperado {expected}"
            return None

        with open(RAW_PATH, "rb") as f:
            raw = f.read(expected)

        depth = np.frombuffer(raw, dtype=np.uint16)
        depth = depth.reshape((height, width)).copy()

        last_depth_meta = meta
        return depth

    except Exception:
        last_depth_error = traceback.format_exc()
        return None



def wait_for_first_depth_frame(timeout_sec=70.0):
    start = time.time()

    while time.time() - start < timeout_sec:
        if depth_files_are_fresh(max_age_sec=3.0):
            depth = read_depth_raw()

            if depth is not None:
                return True

        time.sleep(0.25)

    return False



def get_colormap_id(name):
    name = str(name or "TURBO").upper()

    table = {
        "JET": cv2.COLORMAP_JET,
        "VIRIDIS": cv2.COLORMAP_VIRIDIS,
        "PLASMA": cv2.COLORMAP_PLASMA,
        "MAGMA": cv2.COLORMAP_MAGMA,
        "BONE": cv2.COLORMAP_BONE,
    }

    if name == "TURBO" and hasattr(cv2, "COLORMAP_TURBO"):
        return cv2.COLORMAP_TURBO

    return table.get(name, cv2.COLORMAP_JET)


def get_real_depth_mask(depth):
    return (depth > 0) & (depth < 10000)


def get_visible_depth_mask(depth):
    near_mm = safe_int(config.get("near_mm", 200), 200, 0, 20000)
    far_mm = safe_int(config.get("far_mm", 4000), 4000, 1, 20000)

    if far_mm <= near_mm:
        far_mm = near_mm + 1

    return (depth > 0) & (depth >= near_mm) & (depth <= far_mm)


def valid_depth_values(depth, use_near_far=True):
    if use_near_far:
        mask = get_visible_depth_mask(depth)
    else:
        mask = get_real_depth_mask(depth)

    if not np.any(mask):
        return np.array([], dtype=np.uint16)

    return depth[mask]


def format_distance(mm):
    try:
        mm = int(mm)
    except Exception:
        return "--"

    if mm <= 0:
        return "--"

    if mm >= 1000:
        return f"{mm / 1000.0:.2f} m"

    return f"{mm} mm"


def sample_depth_mm(depth, x, y, radius=7):
    h, w = depth.shape
    radius = safe_int(radius, 7, 1, 40)

    x0 = max(0, int(x) - radius)
    x1 = min(w, int(x) + radius + 1)
    y0 = max(0, int(y) - radius)
    y1 = min(h, int(y) + radius + 1)

    roi = depth[y0:y1, x0:x1]
    values = valid_depth_values(roi, use_near_far=False)

    if values.size == 0:
        return 0

    return int(np.median(values))


def depth_stats(depth):
    values = valid_depth_values(depth, use_near_far=False)
    h, w = depth.shape

    if values.size == 0:
        return {
            "valid": False,
            "min_mm": 0,
            "max_mm": 0,
            "mean_mm": 0,
            "median_mm": 0,
            "center_mm": 0,
            "valid_pixels": 0,
            "width": int(w),
            "height": int(h),
        }

    center_mm = sample_depth_mm(
        depth,
        w // 2,
        h // 2,
        safe_int(config.get("depth_sample_radius", 7), 7, 1, 40)
    )

    return {
        "valid": True,
        "min_mm": int(np.min(values)),
        "max_mm": int(np.max(values)),
        "mean_mm": int(np.mean(values)),
        "median_mm": int(np.median(values)),
        "center_mm": int(center_mm),
        "valid_pixels": int(values.size),
        "width": int(w),
        "height": int(h),
    }


def put_text_box(img, text, org, scale=0.55, fg=(255, 255, 255), bg=(0, 0, 0), thickness=1):
    x, y = org

    (tw, th), baseline = cv2.getTextSize(
        text,
        cv2.FONT_HERSHEY_SIMPLEX,
        scale,
        thickness
    )

    pad = 5

    x0 = max(0, x - pad)
    y0 = max(0, y - th - baseline - pad)
    x1 = min(img.shape[1] - 1, x + tw + pad)
    y1 = min(img.shape[0] - 1, y + baseline + pad)

    overlay = img.copy()
    cv2.rectangle(overlay, (x0, y0), (x1, y1), bg, -1)
    cv2.addWeighted(overlay, 0.65, img, 0.35, 0, img)

    cv2.putText(
        img,
        text,
        (x, y),
        cv2.FONT_HERSHEY_SIMPLEX,
        scale,
        (0, 0, 0),
        thickness + 3,
        cv2.LINE_AA
    )
    cv2.putText(
        img,
        text,
        (x, y),
        cv2.FONT_HERSHEY_SIMPLEX,
        scale,
        fg,
        thickness,
        cv2.LINE_AA
    )


def draw_depth_legend(img, near_mm, far_mm):
    if not bool(config.get("depth_show_legend", True)):
        return img

    h, w = img.shape[:2]
    x0 = max(8, w - 78)
    y0 = 58
    bar_w = 18
    bar_h = min(220, h - 110)

    if bar_h < 80:
        return img

    gradient = np.linspace(255, 0, bar_h, dtype=np.uint8).reshape(bar_h, 1)
    gradient = np.repeat(gradient, bar_w, axis=1)

    cmap = get_colormap_id(config.get("depth_colormap", "TURBO"))

    if str(config.get("depth_color_direction", "NEAR_HOT")).upper() == "FAR_HOT":
        gradient = 255 - gradient

    bar = cv2.applyColorMap(gradient, cmap)

    img[y0:y0 + bar_h, x0:x0 + bar_w] = bar

    cv2.rectangle(img, (x0 - 1, y0 - 1), (x0 + bar_w + 1, y0 + bar_h + 1), (255, 255, 255), 1)

    put_text_box(img, "perto", (x0 - 45, y0 + 12), scale=0.42, fg=(255, 255, 255))
    put_text_box(img, format_distance(near_mm), (x0 - 58, y0 + 34), scale=0.42, fg=(255, 255, 255))

    put_text_box(img, "longe", (x0 - 45, y0 + bar_h - 18), scale=0.42, fg=(255, 255, 255))
    put_text_box(img, format_distance(far_mm), (x0 - 58, y0 + bar_h + 4), scale=0.42, fg=(255, 255, 255))

    return img



def draw_invalid_depth_contours(img, depth_original):
    if not bool(config.get("depth_invalid_contours", True)):
        return img

    invalid = (depth_original == 0).astype(np.uint8) * 255

    if invalid.size == 0:
        return img

    h, w = depth_original.shape
    ih, iw = img.shape[:2]

    invalid_vis = cv2.resize(
        invalid,
        (iw, ih),
        interpolation=cv2.INTER_NEAREST
    )

    try:
        contours, _ = cv2.findContours(invalid_vis, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    except Exception:
        return img

    min_area = max(300, int(0.004 * iw * ih))

    for c in contours:
        area = cv2.contourArea(c)
        if area < min_area:
            continue

        cv2.drawContours(img, [c], -1, (80, 80, 80), 2, lineType=cv2.LINE_AA)

        x, y, ww, hh = cv2.boundingRect(c)
        if ww > 45 and hh > 25:
            put_text_box(
                img,
                "sem leitura",
                (x + 6, max(20, y + 22)),
                scale=0.45,
                fg=(220, 220, 220),
                bg=(0, 0, 0),
                thickness=1
            )

    return img


def draw_depth_overlay(colored, depth_original):
    if not bool(config.get("depth_labels", True)):
        return colored

    out = colored.copy()
    h, w = depth_original.shape
    ih, iw = out.shape[:2]

    sx = iw / max(1, w)
    sy = ih / max(1, h)

    stats = depth_stats(depth_original)
    radius = safe_int(config.get("depth_sample_radius", 7), 7, 1, 40)
    grid = safe_int(config.get("depth_grid", 4), 4, 1, 8)

    near_mm = safe_int(config.get("near_mm", 200), 200, 0, 20000)
    far_mm = safe_int(config.get("far_mm", 4000), 4000, 1, 20000)

    if far_mm <= near_mm:
        far_mm = near_mm + 1

    center_mm = stats.get("center_mm", 0)

    header_1 = (
        f"Centro {format_distance(center_mm)} | "
        f"Mediana {format_distance(stats.get('median_mm', 0))}"
    )

    header_2 = (
        f"Min {format_distance(stats.get('min_mm', 0))} | "
        f"Max {format_distance(stats.get('max_mm', 0))} | "
        f"Depth {w}x{h}"
    )

    put_text_box(out, header_1, (16, 28), scale=0.62, fg=(255, 255, 255))
    put_text_box(out, header_2, (16, 58), scale=0.52, fg=(220, 255, 220))

    cx = w // 2
    cy = h // 2
    cxo = int(cx * sx)
    cyo = int(cy * sy)

    cv2.drawMarker(
        out,
        (cxo, cyo),
        (255, 255, 255),
        markerType=cv2.MARKER_CROSS,
        markerSize=26,
        thickness=2,
        line_type=cv2.LINE_AA
    )
    put_text_box(out, format_distance(center_mm), (cxo + 12, cyo - 10), scale=0.62, fg=(0, 255, 255))

    if grid > 1:
        xs = np.linspace(0.14 * w, 0.84 * w, grid).astype(int)
        ys = np.linspace(0.20 * h, 0.84 * h, grid).astype(int)

        for yy in ys:
            for xx in xs:
                mm = sample_depth_mm(depth_original, xx, yy, radius)
                if mm <= 0:
                    continue

                xo = int(xx * sx)
                yo = int(yy * sy)

                cv2.circle(out, (xo, yo), 4, (255, 255, 255), -1, lineType=cv2.LINE_AA)
                put_text_box(out, format_distance(mm), (xo + 7, yo - 7), scale=0.48, fg=(255, 255, 255))

    draw_depth_legend(out, near_mm, far_mm)

    return out


def prepare_depth_for_visualization(depth):
    if bool(config.get("depth_median_blur", True)):
        try:
            return cv2.medianBlur(depth, 3)
        except Exception:
            return depth

    return depth


def depth_to_colormap(depth):
    depth_vis = prepare_depth_for_visualization(depth)

    near_mm = safe_int(config.get("near_mm", 200), 200, 0, 20000)
    far_mm = safe_int(config.get("far_mm", 4000), 4000, 1, 20000)

    if far_mm <= near_mm:
        far_mm = near_mm + 1

    # Regra importante:
    # depth == 0 significa "sem leitura do sensor", nao uma distancia curta.
    # Objetos muito perto do limite fisico da Astra podem virar 0, por isso ficam pretos.
    real_mask = get_real_depth_mask(depth_vis)

    norm = np.zeros(depth_vis.shape, dtype=np.uint8)

    if np.any(real_mask):
        depth_float = depth_vis.astype(np.float32)

        clipped = np.clip(depth_float, near_mm, far_mm)
        normalized = ((clipped - near_mm) / max(1.0, float(far_mm - near_mm)) * 255.0)
        normalized = np.clip(normalized, 0, 255).astype(np.uint8)

        direction = str(config.get("depth_color_direction", "NEAR_HOT")).upper()

        if direction == "NEAR_HOT":
            # Perto = vermelho/amarelo no TURBO/JET; longe = azul.
            normalized = 255 - normalized

        norm[real_mask] = normalized[real_mask]

    cmap = get_colormap_id(config.get("depth_colormap", "TURBO"))
    colored = cv2.applyColorMap(norm, cmap)

    if bool(config.get("depth_invalid_gray", False)):
        colored[~real_mask] = (35, 35, 35)
    else:
        colored[~real_mask] = (0, 0, 0)

    scale = safe_float(config.get("depth_display_scale", 1.6), 1.6, 0.5, 4.0)

    if abs(scale - 1.0) > 0.01:
        colored = cv2.resize(
            colored,
            None,
            fx=scale,
            fy=scale,
            interpolation=cv2.INTER_CUBIC
        )

    colored = draw_depth_overlay(colored, depth_vis)
    colored = draw_invalid_depth_contours(colored, depth_vis)

    return colored



def depth_worker():
    global latest_depth_raw
    global latest_depth_colored
    global latest_points
    global running
    global last_depth_error

    last_pointcloud_time = 0.0
    last_restart_time = 0.0
    first_valid_frame_received = False

    try:
        start_depth_bridge()
    except Exception:
        last_depth_error = traceback.format_exc()

    while running:
        try:
            now = time.time()

            if not is_depth_bridge_alive():
                if now - last_restart_time > BRIDGE_RESTART_COOLDOWN_SEC:
                    last_restart_time = now
                    last_depth_error = "Bridge não está vivo. Reiniciando bridge."
                    start_depth_bridge()

                time.sleep(0.25)
                continue

            runtime = bridge_runtime_sec()

            if not depth_files_are_fresh(max_age_sec=5.0):
                if not first_valid_frame_received and runtime < BRIDGE_FIRST_FRAME_GRACE_SEC:
                    last_depth_error = (
                        "Bridge inicializando OrbbecSDK. "
                        f"Aguardando primeiro frame... runtime={runtime:.1f}s"
                    )
                    time.sleep(0.25)
                    continue

                if now - last_restart_time > BRIDGE_RESTART_COOLDOWN_SEC:
                    last_restart_time = now
                    last_depth_error = (
                        "Bridge vivo, mas sem frames frescos. "
                        f"runtime={runtime:.1f}s. Reiniciando bridge."
                    )

                    stop_depth_bridge()
                    time.sleep(3.0)
                    start_depth_bridge()

                time.sleep(0.25)
                continue

            depth = read_depth_raw()

            if depth is None:
                time.sleep(0.05)
                continue

            first_valid_frame_received = True

            if config.get("mirror_depth", False):
                depth = np.fliplr(depth)

            colored = depth_to_colormap(depth)
            with depth_lock:
                latest_depth_raw = depth.copy()
                latest_depth_colored = colored.copy()

            last_depth_error = ""

        except Exception:
            last_depth_error = traceback.format_exc()

            err = make_error_image(
                "Erro lendo depth bridge: " + last_depth_error.splitlines()[-1],
                width=640,
                height=480
            )

            with depth_lock:
                latest_depth_colored = err

            time.sleep(0.2)


def stop_all():
    global running
    global rgb_cap
    global rgb_thread
    global depth_thread

    running = False

    time.sleep(0.5)

    close_rgb_camera()

    rgb_thread = None
    depth_thread = None

    stop_depth_bridge()


@app.route("/")
def index():
    return render_template_string(HTML)


@app.route("/api/devices")
def api_devices():
    devices = list_video_devices()

    return jsonify({
        "ok": True,
        "devices": devices,
        "default_device": find_default_rgb_device()
    })


@app.route("/api/usb_fix", methods=["POST"])
def api_usb_fix():
    global last_usb_fix_result

    last_usb_fix_result = apply_usb_fixes()

    return jsonify({
        "ok": True,
        "usb_fix": last_usb_fix_result
    })


@app.route("/api/start", methods=["POST"])
def api_start():
    global running
    global rgb_thread
    global depth_thread
    global latest_rgb
    global latest_depth_colored
    global latest_depth_raw
    global latest_points
    global last_rgb_error
    global last_depth_error
    global last_usb_fix_result

    data = request.get_json(force=True)

    stop_all()

    last_usb_fix_result = apply_usb_fixes()

    config["rgb_device"] = str(data.get("rgb_device", config["rgb_device"]))
    config["width"] = safe_int(data.get("width"), config["width"], 160, 1920)
    config["height"] = safe_int(data.get("height"), config["height"], 120, 1080)
    config["fps"] = safe_int(data.get("fps"), config["fps"], 1, 60)

    config["near_mm"] = safe_int(data.get("near_mm"), config["near_mm"], 0, 20000)
    config["far_mm"] = safe_int(data.get("far_mm"), config["far_mm"], 1, 20000)

    config["jpeg_quality"] = safe_int(data.get("jpeg_quality"), config["jpeg_quality"], 20, 100)

    config["depth_display_scale"] = safe_float(data.get("depth_display_scale"), config["depth_display_scale"], 0.5, 4.0)
    config["depth_grid"] = safe_int(data.get("depth_grid"), config["depth_grid"], 1, 8)
    config["depth_sample_radius"] = safe_int(data.get("depth_sample_radius"), config["depth_sample_radius"], 1, 40)
    config["depth_colormap"] = str(data.get("depth_colormap", config["depth_colormap"])).upper()
    config["depth_color_direction"] = str(data.get("depth_color_direction", config["depth_color_direction"])).upper()
    config["depth_labels"] = bool(data.get("depth_labels", True))
    config["depth_median_blur"] = bool(data.get("depth_median_blur", True))
    config["depth_show_legend"] = bool(data.get("depth_show_legend", True))
    config["depth_invalid_contours"] = bool(data.get("depth_invalid_contours", True))
    config["depth_invalid_gray"] = bool(data.get("depth_invalid_gray", False))

    config["mirror_rgb"] = bool(data.get("mirror_rgb", False))
    config["mirror_depth"] = bool(data.get("mirror_depth", False))
    config["start_rgb_immediately"] = bool(data.get("start_rgb_immediately", True))

    latest_rgb = None
    latest_depth_colored = None
    latest_depth_raw = None
    last_rgb_error = ""
    last_depth_error = ""

    running = True

    if config.get("start_rgb_immediately", True):
        start_rgb_now()

    depth_thread = threading.Thread(target=depth_worker, daemon=True)
    depth_thread.start()

    return jsonify({
        "ok": True,
        "message": "Câmera iniciada. RGB roda independente; depth roda via OrbbecSDK bridge.",
        "config": config,
        "last_rgb_error": last_rgb_error,
        "last_depth_error": last_depth_error,
        "usb_fix": last_usb_fix_result
    })



@app.route("/api/restart_depth_clean", methods=["POST"])
def api_restart_depth_clean():
    global running
    global rgb_thread
    global depth_thread
    global latest_depth_raw
    global latest_depth_colored
    global latest_points
    global last_depth_error
    global last_rgb_error
    global last_usb_fix_result

    running = False
    time.sleep(0.5)

    last_depth_error = ""
    last_rgb_error = ""

    close_rgb_camera()
    stop_depth_bridge()

    for path in [RAW_PATH, META_PATH, LOG_PATH]:
        try:
            if os.path.exists(path):
                os.remove(path)
        except Exception:
            pass

    last_usb_fix_result = apply_usb_fixes()
    reset_result = reset_orbbec_depth_usb()
    last_usb_fix_result["depth_usb_reset"] = reset_result

    running = True

    try:
        start_depth_bridge()
    except Exception:
        last_depth_error = traceback.format_exc()
        if config.get("start_rgb_immediately", True):
            start_rgb_now()
        return jsonify({
            "ok": False,
            "message": "Falha ao iniciar bridge depth.",
            "last_depth_error": last_depth_error,
            "last_rgb_error": last_rgb_error,
            "usb_fix": last_usb_fix_result,
            "bridge_log_tail": tail_file(LOG_PATH, 8000)
        })

    depth_ok = wait_for_first_depth_frame(timeout_sec=70.0)

    depth_thread = threading.Thread(target=depth_worker, daemon=True)
    depth_thread.start()

    if depth_ok:
        depth = read_depth_raw()

        if depth is not None:
            colored = depth_to_colormap(depth)

            with depth_lock:
                latest_depth_raw = depth.copy()
                latest_depth_colored = colored.copy()

        if config.get("start_rgb_immediately", True):
            start_rgb_now()

        return jsonify({
            "ok": True,
            "message": "Depth reiniciado com sucesso. RGB reaberto.",
            "last_depth_meta": last_depth_meta,
            "last_rgb_error": last_rgb_error,
            "usb_fix": last_usb_fix_result,
            "bridge_log_tail": tail_file(LOG_PATH, 8000)
        })

    if config.get("start_rgb_immediately", True):
        start_rgb_now()

    last_depth_error = (
        "Depth não entregou frame em 70s. RGB foi reaberto. "
        "Veja bridge_log_tail."
    )

    return jsonify({
        "ok": False,
        "message": last_depth_error,
        "last_rgb_error": last_rgb_error,
        "usb_fix": last_usb_fix_result,
        "bridge_log_tail": tail_file(LOG_PATH, 8000)
    })


@app.route("/api/stop", methods=["POST"])
def api_stop():
    stop_all()

    return jsonify({
        "ok": True,
        "message": "Câmera parada"
    })


@app.route("/api/status")
def api_status():
    rgb_ready = latest_rgb is not None
    depth_ready = latest_depth_colored is not None
    depth_valid = latest_depth_raw is not None
    bridge_alive = is_depth_bridge_alive()

    raw_age = None
    meta_age = None
    raw_size = None
    meta_size = None

    try:
        if os.path.exists(RAW_PATH):
            raw_age = round(time.time() - os.path.getmtime(RAW_PATH), 3)
            raw_size = os.path.getsize(RAW_PATH)

        if os.path.exists(META_PATH):
            meta_age = round(time.time() - os.path.getmtime(META_PATH), 3)
            meta_size = os.path.getsize(META_PATH)
    except Exception:
        pass

    return jsonify({
        "ok": True,
        "running": running,
        "rgb_ready": rgb_ready,
        "depth_ready": depth_ready,
        "depth_valid": depth_valid,
        "project_root": PROJECT_ROOT,
        "orbbec_sdk_root": ORBBEC_SDK_ROOT,
        "orbbec_lib_dir": ORBBEC_LIB_DIR,

        "bridge_path": BRIDGE_PATH,
        "bridge_exists": os.path.exists(BRIDGE_PATH),
        "bridge_alive": bridge_alive,
        "bridge_pid": None if depth_bridge_proc is None else depth_bridge_proc.pid,
        "bridge_runtime_sec": round(bridge_runtime_sec(), 2),

        "raw_path": RAW_PATH,
        "meta_path": META_PATH,
        "raw_age_sec": raw_age,
        "meta_age_sec": meta_age,
        "raw_size": raw_size,
        "meta_size": meta_size,

        "last_depth_meta": last_depth_meta,
        "depth_stats": None if latest_depth_raw is None else depth_stats(latest_depth_raw),

        "last_rgb_error": last_rgb_error,
        "last_depth_error": last_depth_error,
        "bridge_log_tail": tail_file(LOG_PATH, 8000),
        "last_usb_fix_result": last_usb_fix_result,
        "config": config,
        "video_devices": list_video_devices()
    })



@app.route("/api/depth_stats")
def api_depth_stats():
    with depth_lock:
        depth = None if latest_depth_raw is None else latest_depth_raw.copy()

    if depth is None:
        return jsonify({
            "ok": False,
            "message": "Sem frame depth ainda"
        })

    return jsonify({
        "ok": True,
        "stats": depth_stats(depth),
        "last_depth_meta": last_depth_meta
    })


@app.route("/video/rgb")
def video_rgb():
    def generate():
        while True:
            with camera_lock:
                frame = None if latest_rgb is None else latest_rgb.copy()

            if frame is None:
                msg = "RGB indisponível. Selecione Astra Pro HD Camera ou /dev/video2 e clique Iniciar."
                frame = make_error_image(msg)

            yield frame_to_mjpeg(frame)
            time.sleep(0.03)

    return mjpeg_response(generate())


@app.route("/video/depth")
def video_depth():
    def generate():
        while True:
            with depth_lock:
                frame = None if latest_depth_colored is None else latest_depth_colored.copy()

            if frame is None:
                if not is_depth_bridge_alive():
                    msg = "Depth aguardando bridge OrbbecSDK. Clique em Iniciar câmera."
                elif last_depth_error:
                    msg = last_depth_error.splitlines()[-1]
                else:
                    msg = "Depth aguardando frames do OrbbecSDK bridge."

                frame = make_error_image(msg)

            yield frame_to_mjpeg(frame)
            time.sleep(0.03)

    return mjpeg_response(generate())


def handle_exit(signum, frame):
    stop_all()
    raise SystemExit(0)


if __name__ == "__main__":
    signal.signal(signal.SIGINT, handle_exit)
    signal.signal(signal.SIGTERM, handle_exit)

    detected_rgb = pick_orbbec_rgb_device()
    if detected_rgb:
        config["rgb_device"] = detected_rgb

    last_usb_fix_result = apply_usb_fixes()

    print("Painel Orbbec Astra Pro iniciado em http://127.0.0.1:5007")
    print("Backend depth: OrbbecSDK v1 bridge com overlay de distancias")
    print("Project root:", PROJECT_ROOT)
    print("SDK root:", ORBBEC_SDK_ROOT)
    print("Bridge:", BRIDGE_PATH)
    print("Lib dir:", ORBBEC_LIB_DIR)
    print("RGB detectado:", config["rgb_device"])

    app.run(host="0.0.0.0", port=5007, debug=False, threaded=True)
