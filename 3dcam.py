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
from collections import deque, Counter

import cv2
import numpy as np
from flask import Flask, Response, jsonify, request, render_template_string


app = Flask(__name__)

@app.after_request
def add_no_cache_headers(response):
    response.headers["Cache-Control"] = "no-store, no-cache, must-revalidate, max-age=0"
    response.headers["Pragma"] = "no-cache"
    response.headers["Expires"] = "0"
    return response

camera_lock = threading.Lock()
depth_lock = threading.Lock()
bridge_lock = threading.Lock()
detection_lock = threading.Lock()
mediapipe_lock = threading.Lock()

rgb_cap = None
rgb_thread = None
rgb_inference_thread = None
depth_thread = None
depth_bridge_proc = None
depth_bridge_log_handle = None
depth_bridge_started_at = 0.0

running = False

latest_rgb = None
latest_rgb_raw = None
latest_rgb_source_shape = None
latest_depth_raw = None
latest_depth_colored = None
latest_points = []
latest_rgb_detections = []
latest_depth_detections = []

last_rgb_error = ""
last_depth_error = ""
last_detection_error = ""
last_usb_fix_result = {}
last_depth_meta = {}

RAW_PATH = "/dev/shm/orbbec_depth.raw"
META_PATH = "/dev/shm/orbbec_depth_meta.json"
ALT_RAW_PATH = "/tmp/orbbec_depth.raw"
ALT_META_PATH = "/tmp/orbbec_depth_meta.json"
ACTIVE_RAW_PATH = RAW_PATH
ACTIVE_META_PATH = META_PATH
LOG_PATH = "/tmp/orbbec_depth_bridge.log"

BRIDGE_FIRST_FRAME_GRACE_SEC = 120.0
BRIDGE_RESTART_COOLDOWN_SEC = 60.0
POINTCLOUD_INTERVAL_SEC = 0.25
DETECTION_INTERVAL_SEC = 0.45

object_detector_mediapipe_pose = None
object_detector_mediapipe_face = None
object_detector_mediapipe_hands = None
object_detector_mediapipe_error = ""
object_detector_mediapipe_available = None

last_rgb_detection_time = 0.0
last_rgb_frame_time = 0.0
last_rgb_capture_fps = 0.0
last_rgb_inference_fps = 0.0
last_rgb_detection_duration_ms = 0.0
simple_bg_subtractor = None
simple_detection_last_reset = 0.0
depth_motion_subtractor = None
depth_motion_last_reset = 0.0

TEMPORAL_ALPHA = 0.65
GESTURE_HISTORY_LEN = 6

temporal_tracker = {
    "rgb": {"bbox": None, "pose_points": [], "hand_points": [], "gesture_history": deque(maxlen=GESTURE_HISTORY_LEN), "last_seen": 0.0},
    "depth": {"bbox": None, "pose_points": [], "hand_points": [], "gesture_history": deque(maxlen=GESTURE_HISTORY_LEN), "last_seen": 0.0},
}


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
    os.path.join(REAL_HOME, "", "pandorapi")
)

ORBBEC_SDK_ROOT = os.environ.get(
    "ORBBEC_SDK_ROOT",
    os.path.join(PROJECT_ROOT, "OrbbecSDK")
)

BRIDGE_PATH = os.environ.get(
    "ORBBEC_BRIDGE",
    os.path.join(ORBBEC_SDK_ROOT, "", "", "orbbec_depth_bridge_arm")
)

ORBBEC_LIB_DIR = os.path.join(ORBBEC_SDK_ROOT, "lib", "linux_x64")

config = {
    "rgb_device": "/dev/video2",
    "width": 640,
    "height": 480,
    "fps": 30,

    "near_mm": 200,
    "far_mm": 4000,

    "jpeg_quality": 70,

    "fx": 575.0,
    "fy": 575.0,

    "mirror_rgb": False,
    "mirror_depth": False,
    "start_rgb_immediately": True,
    "rgb_pose_hands_enabled": False,

    "depth_display_scale": 1.6,
    "depth_colormap": "TURBO",
    "depth_color_direction": "NEAR_HOT",
    "depth_labels": False,
    "depth_grid": 4,
    "depth_sample_radius": 7,
    "depth_median_blur": False,
    "depth_show_legend": False,
    "depth_invalid_gray": False,
    "depth_invalid_contours": False,
    "rgb_detection_enabled": False,
    "rgb_simple_detection_enabled": False,
    "depth_detection_enabled": False,
    "exoskeleton_enabled": True,
    "mediapipe_pose_enabled": True,
    "mediapipe_hands_enabled": True,
    "gesture_enabled": True,

    "detection_interval_sec": 0.08,
    "mediapipe_input_width": 224,
    "mediapipe_pose_input_width": 224,
    "mediapipe_hands_input_width": 320,

    "preview_enabled": True,
    "rgb_preview_enabled": True,
    "preview_size": "medio",

    "mediapipe_face_enabled": False,
    "mediapipe_min_confidence": 0.35,
    "depth_mediapipe_enabled": False,
    "depth_detection_enabled": False,
    "depth_silhouette_enabled": False,
    "depth_use_rgb_projection": False,
    "depth_foreground_margin_mm": 250,
    "depth_min_person_area_ratio": 0.018,

    "depth_bridge_width": 640,
    "depth_bridge_height": 480,
    "depth_bridge_fps": 30,
    "depth_bridge_timeout_ms": 500,
    "depth_bridge_publish_fps": 10,
    "depth_bridge_restart_after_ms": 30000,
    "depth_bridge_stale_republish_ms": 12000,
    "depth_bridge_max_attempts": 0,

    "python_depth_fresh_timeout_sec": 30.0,
    "python_depth_read_interval_sec": 0.05,
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

        .preview-area {
            transition: all 0.2s ease;
        }

        .preview-card-col {
            flex: 0 0 auto;
            width: 50%;
        }

        .preview-img {
            width: 100%;
            background: #010409;
            border: 1px solid #30363d;
            border-radius: 12px;
            min-height: 260px;
            max-height: 78vh;
            object-fit: contain;
            image-rendering: auto;
        }

        .preview-area.preview-miniatura .preview-card-col {
            width: 25%;
        }

        .preview-area.preview-miniatura .preview-img {
            min-height: 170px;
            max-height: 260px;
        }

        .preview-area.preview-pequeno .preview-card-col {
            width: 33.333333%;
        }

        .preview-area.preview-pequeno .preview-img {
            min-height: 240px;
            max-height: 360px;
        }

        .preview-area.preview-medio .preview-card-col {
            width: 50%;
        }

        .preview-area.preview-medio .preview-img {
            min-height: 420px;
            max-height: 560px;
        }

        .preview-area.preview-grande .preview-card-col {
            width: 100%;
        }

        .preview-area.preview-grande .preview-img {
            min-height: 620px;
            max-height: 780px;
        }

        .preview-area.preview-muito-grande .preview-card-col {
            width: 100%;
        }

        .preview-area.preview-muito-grande .preview-img {
            min-height: 820px;
            max-height: none;
        }

        @media (max-width: 991.98px) {
            .preview-card-col,
            .preview-area.preview-miniatura .preview-card-col,
            .preview-area.preview-pequeno .preview-card-col,
            .preview-area.preview-medio .preview-card-col,
            .preview-area.preview-grande .preview-card-col,
            .preview-area.preview-muito-grande .preview-card-col {
                width: 100%;
            }

            .preview-img,
            .preview-area.preview-miniatura .preview-img,
            .preview-area.preview-pequeno .preview-img,
            .preview-area.preview-medio .preview-img,
            .preview-area.preview-grande .preview-img,
            .preview-area.preview-muito-grande .preview-img {
                min-height: 260px;
                max-height: none;
            }
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
                RGB em baixa latência com reconhecimento opcional
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
                        <input id="jpegQuality" class="form-control" type="number" value="70">
                    </div>

                    <div class="col-6">
                        <label class="form-label">Detecção s</label>
                        <input id="detectionInterval" class="form-control" type="text" value="0.08">
                    </div>

                    <div class="col-6">
                        <label class="form-label">Pose px</label>
                        <input id="mediapipeInputWidth" class="form-control" type="number" value="224">
                    </div>

                    <div class="col-6">
                        <label class="form-label">Mãos px</label>
                        <input id="mediapipeHandsInputWidth" class="form-control" type="number" value="320">
                    </div>
                </div>

                <div class="row g-2 mt-2">
                    <div class="col-12">
                        <label class="form-label">Tamanho dos cards de preview</label>
                        <select id="previewSize" class="form-select" onchange="applyPreviewSize()">
                            <option value="miniatura">Miniatura</option>
                            <option value="pequeno">Pequeno</option>
                            <option value="medio" selected>Médio</option>
                            <option value="grande">Grande</option>
                            <option value="muito-grande">Muito grande</option>
                        </select>
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

                    <div class="col-6">
                        <label class="form-label">Margem silhueta mm</label>
                        <input id="depthForegroundMarginMm" class="form-control" type="number" value="250">
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
                    <input id="rgbMediapipeEnabled" class="form-check-input" type="checkbox">
                    <label class="form-check-label" for="rgbMediapipeEnabled">Ativar MediaPipe Pose/Hands</label>
                </div>

                <div class="form-check mb-2">
                    <input id="rgbSimpleDetectionEnabled" class="form-check-input" type="checkbox">
                    <label class="form-check-label" for="rgbSimpleDetectionEnabled">Ativar reconhecimento leve com retângulo verde no RGB</label>
                </div>

                <div class="form-check mb-2">
                    <input id="depthContourDetectionEnabled" class="form-check-input" type="checkbox">
                    <label class="form-check-label" for="depthContourDetectionEnabled">Ativar reconhecimento no depth por contorno</label>
                </div>

                <div class="small-muted mb-3">
                    Para menor atraso, deixe o MediaPipe desligado. O modo leve RGB usa OpenCV/MOG2. O depth por contorno usa apenas o mapa de profundidade, segmentando o objeto mais próximo por distância.
                </div>

                <div class="d-grid gap-2">
                    <button class="btn btn-success" onclick="startCamera()">Iniciar câmera</button>
                    <button class="btn btn-warning" onclick="stopCamera()">Parar câmera</button>
                    <button class="btn btn-primary" onclick="refreshStatus()">Atualizar status</button>
                </div>
            </div>

            <div class="card p-3 mb-4">
                <h4>Reconhecimento RGB</h4>
                <div class="small-muted mb-2">Gestos aparecem somente quando o MediaPipe estiver ativado.</div>
                <div id="rgbGesturePanel" class="small-muted">Nenhum gesto detectado.</div>
            </div>

            <div class="card p-3">
                <h4>Status</h4>
                <pre id="statusBox">Carregando...</pre>
            </div>
        </div>

        <div id="previewArea" class="col-xl-9 col-lg-8 preview-area preview-medio">
            <div class="row g-4">
                <div id="rgbCard" class="preview-card-col col-12">
                    <div class="card p-3">
                        <h4>RGB em tempo real</h4>
                        <img id="rgbPreview" class="preview-img" src="/video/rgb">
                    </div>
                </div>

                <div id="depthCard" class="preview-card-col col-12">
                    <div class="card p-3">
                        <h4>Depth colorido</h4>
                        <img id="depthPreview" class="preview-img" src="/video/depth">
                        <div class="depth-help">
                            Depth colorido. Opcionalmente detecta objeto/pessoa por contorno usando apenas profundidade, sem RGB e sem IA.
                        </div>
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

function renderGesturePanel(elementId, detections) {
    const el = document.getElementById(elementId);
    if (!el) {
        return;
    }

    const gestures = [];

    for (const det of (detections || [])) {
        for (const gesture of (det.gestures || [])) {
            if (gesture && !gestures.includes(gesture)) {
                gestures.push(gesture);
            }
        }
    }

    if (gestures.length === 0) {
        el.textContent = "Nenhum gesto detectado.";
        return;
    }

    el.innerHTML = gestures.map(g => `<span class="badge bg-info text-dark me-1 mb-1">${g}</span>`).join(" ");
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
        depth_foreground_margin_mm: parseInt(document.getElementById("depthForegroundMarginMm").value || "250"),

        jpeg_quality: parseInt(document.getElementById("jpegQuality").value || "85"),

        depth_display_scale: parseFloatBR(document.getElementById("depthDisplayScale").value, 1.6),
        depth_grid: parseInt(document.getElementById("depthGrid").value || "4"),
        depth_sample_radius: parseInt(document.getElementById("depthSampleRadius").value || "7"),
        depth_colormap: document.getElementById("depthColormap").value,
        depth_color_direction: document.getElementById("depthColorDirection").value,
        depth_labels: false,
        depth_median_blur: false,
        depth_show_legend: false,
        depth_invalid_contours: false,
        depth_invalid_gray: false,

        mirror_rgb: document.getElementById("mirrorRgb").checked,
        mirror_depth: document.getElementById("mirrorDepth").checked,

        preview_enabled: true,
        rgb_preview_enabled: true,
        preview_size: document.getElementById("previewSize").value,

        start_rgb_immediately: true,
        rgb_pose_hands_enabled: document.getElementById("rgbMediapipeEnabled").checked,
        rgb_simple_detection_enabled: document.getElementById("rgbSimpleDetectionEnabled").checked,
        rgb_detection_enabled: document.getElementById("rgbMediapipeEnabled").checked || document.getElementById("rgbSimpleDetectionEnabled").checked,

        depth_detection_enabled: document.getElementById("depthContourDetectionEnabled").checked,
        depth_silhouette_enabled: document.getElementById("depthContourDetectionEnabled").checked,
        depth_mediapipe_enabled: false,
        depth_use_rgb_projection: false,

        exoskeleton_enabled: true,
        mediapipe_pose_enabled: true,
        mediapipe_hands_enabled: document.getElementById("rgbMediapipeEnabled").checked,
        mediapipe_face_enabled: false,
        gesture_enabled: true,

        detection_interval_sec: parseFloatBR(document.getElementById("detectionInterval").value, 0.08),
        mediapipe_input_width: parseInt(document.getElementById("mediapipeInputWidth").value || "256"),
        mediapipe_pose_input_width: parseInt(document.getElementById("mediapipeInputWidth").value || "256"),
        mediapipe_hands_input_width: parseInt(document.getElementById("mediapipeHandsInputWidth").value || "480")
    };
}

function applyPreviewSize() {
    const area = document.getElementById("previewArea");
    const select = document.getElementById("previewSize");

    if (!area || !select) {
        return;
    }

    area.classList.remove(
        "preview-miniatura",
        "preview-pequeno",
        "preview-medio",
        "preview-grande",
        "preview-muito-grande"
    );

    area.classList.add("preview-" + select.value);
}

function applyPreviewVisibility() {
    const area = document.getElementById("previewArea");
    const rgbCard = document.getElementById("rgbCard");
    const depthCard = document.getElementById("depthCard");

    if (!area) {
        return;
    }

    area.style.display = "";
    applyPreviewSize();

    if (rgbCard) {
        rgbCard.style.display = "";
    }

    if (depthCard) {
        depthCard.style.width = "";
    }

    refreshStreams();
}

async function startCamera() {
    const result = await apiPost("/api/start", collectConfig());

    if (result.ok) {
        setBadge("rodando", "bg-success");
        applyPreviewVisibility();
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


async function refreshStatus() {
    const result = await apiGet("/api/status");
    document.getElementById("statusBox").textContent = JSON.stringify(result, null, 2);
    renderGesturePanel("rgbGesturePanel", result.rgb_detections || []);

    if (result.running) {
        setBadge("rodando", "bg-success");
    } else {
        setBadge("parado", "bg-secondary");
    }
}

function refreshStreams() {
    const ts = Date.now();
    const depthPreview = document.getElementById("depthPreview");
    const rgbPreview = document.getElementById("rgbPreview");

    if (depthPreview) {
        depthPreview.src = "/video/depth?t=" + ts;
    }

    if (rgbPreview) {
        rgbPreview.src = "/video/rgb?t=" + ts;
    }
}




window.addEventListener("load", async function() {
    applyPreviewSize();
    applyPreviewVisibility();

    await loadDevices();
    await refreshStatus();

    setInterval(refreshStatus, 1000);
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
    response = Response(
        generator,
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )
    response.headers["Cache-Control"] = "no-store, no-cache, must-revalidate, max-age=0"
    response.headers["Pragma"] = "no-cache"
    response.headers["Expires"] = "0"
    response.headers["X-Accel-Buffering"] = "no"
    return response


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
    global latest_rgb_raw
    global latest_rgb_source_shape
    global latest_rgb_detections

    if rgb_cap is not None:
        try:
            rgb_cap.release()
        except Exception:
            pass

    rgb_cap = None
    latest_rgb = None
    latest_rgb_raw = None
    latest_rgb_source_shape = None
    latest_rgb_detections = []


def rgb_worker():
    global latest_rgb
    global latest_rgb_raw
    global latest_rgb_source_shape
    global running
    global rgb_cap
    global last_rgb_error
    global last_rgb_frame_time
    global last_rgb_capture_fps

    fps_last_time = time.time()
    fps_count = 0

    while running:
        try:
            if rgb_cap is None:
                time.sleep(0.005)
                continue

            # Baixa latencia: descarta frames antigos do buffer sempre que possivel.
            grabbed = False
            for _ in range(2):
                try:
                    grabbed = rgb_cap.grab()
                except Exception:
                    grabbed = False
                    break

            if grabbed:
                ret, frame = rgb_cap.retrieve()
            else:
                ret, frame = rgb_cap.read()

            if not ret or frame is None:
                time.sleep(0.003)
                continue

            if config.get("mirror_rgb", False):
                frame = cv2.flip(frame, 1)

            with camera_lock:
                latest_rgb_raw = frame.copy()
                latest_rgb_source_shape = frame.shape
                last_rgb_frame_time = time.time()

            with detection_lock:
                detections = list(latest_rgb_detections)

            if detections:
                output_frame = draw_detections(
                    frame,
                    detections,
                    source_shape=frame.shape,
                    title=None
                )
            else:
                output_frame = frame

            with camera_lock:
                latest_rgb = output_frame.copy()

            fps_count += 1
            now = time.time()
            if now - fps_last_time >= 1.0:
                last_rgb_capture_fps = round(fps_count / max(0.001, now - fps_last_time), 2)
                fps_last_time = now
                fps_count = 0

            last_rgb_error = ""

        except Exception:
            last_rgb_error = traceback.format_exc()
            time.sleep(0.02)


def rgb_inference_worker():
    global latest_rgb_detections
    global last_rgb_detection_time
    global last_rgb_inference_fps
    global last_rgb_detection_duration_ms
    global last_detection_error

    fps_last_time = time.time()
    fps_count = 0

    while running:
        try:
            enabled = bool(config.get("rgb_pose_hands_enabled", False)) or bool(config.get("rgb_simple_detection_enabled", False))

            if not enabled:
                with detection_lock:
                    latest_rgb_detections = []
                time.sleep(0.05)
                continue

            interval = safe_float(
                config.get("detection_interval_sec", DETECTION_INTERVAL_SEC),
                DETECTION_INTERVAL_SEC,
                0.02,
                2.0
            )

            now = time.time()
            if now - last_rgb_detection_time < interval:
                time.sleep(0.004)
                continue

            with camera_lock:
                frame = None if latest_rgb_raw is None else latest_rgb_raw.copy()

            if frame is None:
                time.sleep(0.01)
                continue

            started = time.time()
            detections = detect_rgb_selected_mode(frame)
            last_rgb_detection_duration_ms = round((time.time() - started) * 1000.0, 2)
            last_rgb_detection_time = time.time()

            with detection_lock:
                latest_rgb_detections = detections

            fps_count += 1
            fps_now = time.time()
            if fps_now - fps_last_time >= 1.0:
                last_rgb_inference_fps = round(fps_count / max(0.001, fps_now - fps_last_time), 2)
                fps_last_time = fps_now
                fps_count = 0

        except Exception:
            last_detection_error = traceback.format_exc()
            time.sleep(0.03)


def start_rgb_now():
    global rgb_thread
    global rgb_inference_thread
    global last_rgb_error

    try:
        if rgb_cap is None:
            open_rgb_camera()

        if rgb_thread is None or not rgb_thread.is_alive():
            rgb_thread = threading.Thread(target=rgb_worker, daemon=True)
            rgb_thread.start()

        if rgb_inference_thread is None or not rgb_inference_thread.is_alive():
            rgb_inference_thread = threading.Thread(target=rgb_inference_worker, daemon=True)
            rgb_inference_thread.start()

        last_rgb_error = ""
    except Exception:
        last_rgb_error = traceback.format_exc()


def is_depth_bridge_alive():
    return depth_bridge_proc is not None and depth_bridge_proc.poll() is None


def bridge_runtime_sec():
    if depth_bridge_started_at <= 0:
        return 0.0

    return time.time() - depth_bridge_started_at




def build_depth_bridge_cmd():
    return [
        BRIDGE_PATH,
        RAW_PATH,
        META_PATH,
        str(safe_int(config.get("depth_bridge_width", 640), 640, 1, 4096)),
        str(safe_int(config.get("depth_bridge_height", 480), 480, 1, 4096)),
        str(safe_int(config.get("depth_bridge_fps", 30), 30, 1, 60)),
        str(safe_int(config.get("depth_bridge_timeout_ms", 500), 500, 20, 5000)),
        str(safe_int(config.get("depth_bridge_publish_fps", 10), 10, 0, 60)),
        str(safe_int(config.get("depth_bridge_restart_after_ms", 30000), 30000, 0, 120000)),
        str(safe_int(config.get("depth_bridge_stale_republish_ms", 12000), 12000, 0, 120000)),
        str(safe_int(config.get("depth_bridge_max_attempts", 0), 0, 0, 1000)),
    ]


def meta_file_age_sec():
    try:
        if not os.path.exists(ACTIVE_META_PATH):
            return None
        return time.time() - os.path.getmtime(ACTIVE_META_PATH)
    except Exception:
        return None


def bridge_meta_status():
    try:
        if not last_depth_meta:
            return ""
        return str(last_depth_meta.get("status", ""))
    except Exception:
        return ""


def bridge_frame_age_ms():
    try:
        if not last_depth_meta:
            return None
        value = last_depth_meta.get("frame_age_ms", None)
        return None if value is None else int(value)
    except Exception:
        return None


def depth_path_age(raw_path, meta_path):
    try:
        if not os.path.exists(raw_path) or not os.path.exists(meta_path):
            return None, None
        now = time.time()
        return now - os.path.getmtime(raw_path), now - os.path.getmtime(meta_path)
    except Exception:
        return None, None


def choose_active_depth_paths(max_age_sec=None):
    global ACTIVE_RAW_PATH
    global ACTIVE_META_PATH

    if max_age_sec is None:
        max_age_sec = safe_float(config.get("python_depth_fresh_timeout_sec", 30.0), 30.0, 2.0, 120.0)

    candidates = [
        (RAW_PATH, META_PATH, "devshm"),
        (ALT_RAW_PATH, ALT_META_PATH, "tmp_fallback"),
    ]

    for raw_path, meta_path, mode in candidates:
        raw_age, meta_age = depth_path_age(raw_path, meta_path)
        if raw_age is not None and meta_age is not None and raw_age <= max_age_sec and meta_age <= max_age_sec:
            ACTIVE_RAW_PATH = raw_path
            ACTIVE_META_PATH = meta_path
            return raw_path, meta_path, mode

    return ACTIVE_RAW_PATH, ACTIVE_META_PATH, "last_active"

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

        for path in [RAW_PATH, META_PATH, ALT_RAW_PATH, ALT_META_PATH]:
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

        cmd = build_depth_bridge_cmd()

        depth_bridge_proc = subprocess.Popen(
            cmd,
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


def depth_files_are_fresh(max_age_sec=None):
    if max_age_sec is None:
        max_age_sec = safe_float(config.get("python_depth_fresh_timeout_sec", 30.0), 30.0, 2.0, 120.0)

    raw_path, meta_path, _mode = choose_active_depth_paths(max_age_sec=max_age_sec)
    raw_age, meta_age = depth_path_age(raw_path, meta_path)

    if raw_age is None or meta_age is None:
        return False

    return raw_age <= max_age_sec and meta_age <= max_age_sec


def read_depth_raw():
    global last_depth_meta
    global last_depth_error

    raw_path, meta_path, path_mode = choose_active_depth_paths()

    if not os.path.exists(meta_path):
        last_depth_error = "Aguardando meta do bridge: " + meta_path
        return None

    if not os.path.exists(raw_path):
        last_depth_error = "Aguardando raw do bridge: " + raw_path
        return None

    for attempt in range(4):
        try:
            with open(meta_path, "r") as f:
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

            size = os.path.getsize(raw_path)
            if size < expected:
                if attempt < 3:
                    time.sleep(0.015)
                    continue
                last_depth_error = f"RAW incompleto em {raw_path}: {size} bytes, esperado {expected}"
                return None

            with open(raw_path, "rb") as f:
                raw = f.read(expected)

            if len(raw) < expected:
                if attempt < 3:
                    time.sleep(0.015)
                    continue
                last_depth_error = f"RAW lido incompleto em {raw_path}: {len(raw)} bytes, esperado {expected}"
                return None

            depth = np.frombuffer(raw, dtype=np.uint16)
            depth = depth.reshape((height, width)).copy()

            meta["_active_raw_path"] = raw_path
            meta["_active_meta_path"] = meta_path
            meta["_path_mode"] = path_mode
            last_depth_meta = meta

            status = str(meta.get("status", "ok"))
            frame_age_ms = int(meta.get("frame_age_ms", 0) or 0)

            if status == "stale_republish":
                last_depth_error = f"Bridge vivo, republicando último frame. frame_age_ms={frame_age_ms}"
            else:
                last_depth_error = ""

            return depth

        except Exception:
            if attempt < 3:
                time.sleep(0.015)
                continue
            last_depth_error = traceback.format_exc()
            return None

    return None



def wait_for_first_depth_frame(timeout_sec=70.0):
    start = time.time()

    while time.time() - start < timeout_sec:
        if depth_files_are_fresh():
            depth = read_depth_raw()
            if depth is not None:
                return True
        time.sleep(0.10)

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






def get_mediapipe_detectors():
    global object_detector_mediapipe_pose
    global object_detector_mediapipe_face
    global object_detector_mediapipe_hands
    global object_detector_mediapipe_error
    global object_detector_mediapipe_available

    if object_detector_mediapipe_available is False:
        return None, None, None

    try:
        import mediapipe as mp

        min_conf = safe_float(config.get("mediapipe_min_confidence", 0.35), 0.35, 0.05, 0.95)

        if object_detector_mediapipe_pose is None and bool(config.get("mediapipe_pose_enabled", True)):
            object_detector_mediapipe_pose = mp.solutions.pose.Pose(
                static_image_mode=False,
                model_complexity=0,
                smooth_landmarks=True,
                enable_segmentation=False,
                min_detection_confidence=min_conf,
                min_tracking_confidence=min_conf
            )

        if object_detector_mediapipe_face is None and bool(config.get("mediapipe_face_enabled", True)):
            object_detector_mediapipe_face = mp.solutions.face_detection.FaceDetection(
                model_selection=0,
                min_detection_confidence=min_conf
            )

        if object_detector_mediapipe_hands is None and bool(config.get("mediapipe_hands_enabled", True)):
            object_detector_mediapipe_hands = mp.solutions.hands.Hands(
                static_image_mode=False,
                max_num_hands=2,
                model_complexity=0,
                min_detection_confidence=min_conf,
                min_tracking_confidence=min_conf
            )

        object_detector_mediapipe_error = ""
        object_detector_mediapipe_available = True
        return object_detector_mediapipe_pose, object_detector_mediapipe_face, object_detector_mediapipe_hands

    except Exception:
        object_detector_mediapipe_pose = None
        object_detector_mediapipe_face = None
        object_detector_mediapipe_hands = None
        object_detector_mediapipe_available = False
        object_detector_mediapipe_error = traceback.format_exc()
        return None, None, None



def expand_bbox(bbox, width, height, pad_x_ratio=0.10, pad_y_ratio=0.12):
    x1, y1, x2, y2 = bbox
    bw = max(1, x2 - x1)
    bh = max(1, y2 - y1)

    px = int(bw * pad_x_ratio)
    py = int(bh * pad_y_ratio)

    return clip_bbox(x1 - px, y1 - py, x2 + px, y2 + py, width, height)



def scale_bbox_to_shape(bbox, from_shape, to_shape):
    if bbox is None:
        return None

    from_h, from_w = from_shape[:2]
    to_h, to_w = to_shape[:2]

    sx = to_w / max(1.0, float(from_w))
    sy = to_h / max(1.0, float(from_h))

    x1, y1, x2, y2 = bbox

    return list(clip_bbox(
        int(x1 * sx),
        int(y1 * sy),
        int(x2 * sx),
        int(y2 * sy),
        to_w,
        to_h
    ))


def resize_frame_to_width(frame, target_w):
    if frame is None:
        return None

    target_w = safe_int(target_w, 320, 160, 1280)
    h, w = frame.shape[:2]

    if target_w <= 0 or w <= target_w:
        return frame

    target_h = max(1, int(h * (target_w / float(w))))
    return cv2.resize(frame, (target_w, target_h), interpolation=cv2.INTER_AREA)


def resize_for_mediapipe(frame):
    target_w = safe_int(config.get("mediapipe_input_width", 256), 256, 160, 1280)
    return resize_frame_to_width(frame, target_w)


def mediapipe_pose_bbox(results, frame_shape):
    if results is None or not getattr(results, "pose_landmarks", None):
        return None, 0.0

    h, w = frame_shape[:2]
    points = []
    visibilities = []

    for lm in results.pose_landmarks.landmark:
        visibility = float(getattr(lm, "visibility", 1.0))

        if visibility < 0.35:
            continue

        x = int(lm.x * w)
        y = int(lm.y * h)

        if x < -w * 0.15 or x > w * 1.15 or y < -h * 0.15 or y > h * 1.15:
            continue

        points.append((x, y))
        visibilities.append(visibility)

    if len(points) < 5:
        return None, 0.0

    xs = [p[0] for p in points]
    ys = [p[1] for p in points]

    x1, y1, x2, y2 = min(xs), min(ys), max(xs), max(ys)
    x1, y1, x2, y2 = expand_bbox((x1, y1, x2, y2), w, h, 0.24, 0.30)

    area_ratio = bbox_area((x1, y1, x2, y2)) / max(1.0, float(w * h))

    if area_ratio < 0.004 or area_ratio > 0.92:
        return None, 0.0

    confidence = float(np.mean(visibilities)) if visibilities else 0.50
    return [x1, y1, x2, y2], confidence


def mediapipe_face_to_person_bbox(face_detection, frame_shape):
    h, w = frame_shape[:2]

    try:
        box = face_detection.location_data.relative_bounding_box
        score = float(face_detection.score[0]) if face_detection.score else 0.50

        fx1 = int(box.xmin * w)
        fy1 = int(box.ymin * h)
        fw = int(box.width * w)
        fh = int(box.height * h)

        fx2 = fx1 + fw
        fy2 = fy1 + fh

        x1 = int(fx1 - fw * 1.15)
        y1 = int(fy1 - fh * 1.25)
        x2 = int(fx2 + fw * 1.15)
        y2 = int(fy2 + fh * 6.50)

        x1, y1, x2, y2 = clip_bbox(x1, y1, x2, y2, w, h)

        if bbox_area((x1, y1, x2, y2)) < 500:
            return None, 0.0

        return [x1, y1, x2, y2], score

    except Exception:
        return None, 0.0


POSE_CONNECTIONS_FAST = [
    (11, 12), (11, 13), (13, 15), (12, 14), (14, 16),
    (11, 23), (12, 24), (23, 24),
    (23, 25), (25, 27), (24, 26), (26, 28)
]

HAND_CONNECTIONS_FAST = [
    (0, 1), (1, 2), (2, 3), (3, 4),
    (0, 5), (5, 6), (6, 7), (7, 8),
    (5, 9), (9, 10), (10, 11), (11, 12),
    (9, 13), (13, 14), (14, 15), (15, 16),
    (13, 17), (17, 18), (18, 19), (19, 20),
    (0, 17)
]


def landmarks_to_points(landmark_list, frame_shape, visibility_threshold=0.0):
    if landmark_list is None:
        return []

    h, w = frame_shape[:2]
    points = []

    for idx, lm in enumerate(landmark_list.landmark):
        visibility = float(getattr(lm, "visibility", 1.0))

        if visibility < visibility_threshold:
            points.append(None)
            continue

        x = int(float(lm.x) * w)
        y = int(float(lm.y) * h)
        z = float(getattr(lm, "z", 0.0))

        if x < -w * 0.25 or x > w * 1.25 or y < -h * 0.25 or y > h * 1.25:
            points.append(None)
            continue

        points.append({
            "i": int(idx),
            "x": int(x),
            "y": int(y),
            "z": float(z),
            "visibility": float(visibility)
        })

    return points


def scale_points_to_shape(points, from_shape, to_shape):
    if not points:
        return []

    from_h, from_w = from_shape[:2]
    to_h, to_w = to_shape[:2]

    sx = to_w / max(1.0, float(from_w))
    sy = to_h / max(1.0, float(from_h))

    scaled = []

    for item in points:
        if item is None:
            scaled.append(None)
            continue

        scaled.append({
            "i": int(item.get("i", len(scaled))),
            "x": int(item.get("x", 0) * sx),
            "y": int(item.get("y", 0) * sy),
            "z": float(item.get("z", 0.0)),
            "visibility": float(item.get("visibility", 1.0))
        })

    return scaled


def bbox_from_points(points, frame_shape, pad_ratio=0.12):
    valid = [p for p in points if p is not None]

    if not valid:
        return None

    h, w = frame_shape[:2]
    xs = [p["x"] for p in valid]
    ys = [p["y"] for p in valid]

    x1, y1, x2, y2 = min(xs), min(ys), max(xs), max(ys)
    return list(expand_bbox((x1, y1, x2, y2), w, h, pad_ratio, pad_ratio))


def point_at(points, idx):
    if not points or idx < 0 or idx >= len(points):
        return None

    return points[idx]


def angle_degrees(a, b, c):
    if a is None or b is None or c is None:
        return None

    v1 = np.array([a["x"] - b["x"], a["y"] - b["y"]], dtype=np.float32)
    v2 = np.array([c["x"] - b["x"], c["y"] - b["y"]], dtype=np.float32)

    n1 = float(np.linalg.norm(v1))
    n2 = float(np.linalg.norm(v2))

    if n1 <= 1e-5 or n2 <= 1e-5:
        return None

    cosang = float(np.dot(v1, v2) / (n1 * n2))
    cosang = max(-1.0, min(1.0, cosang))

    return float(np.degrees(np.arccos(cosang)))


def classify_hand_gesture(points, handedness=""):
    if not points or len(points) < 21:
        return ""

    wrist = point_at(points, 0)
    if wrist is None:
        return ""

    def dist(a, b):
        if a is None or b is None:
            return 0.0
        return float(np.hypot(a["x"] - b["x"], a["y"] - b["y"]))

    def finger_extended(tip_idx, pip_idx, mcp_idx):
        tip = point_at(points, tip_idx)
        pip = point_at(points, pip_idx)
        mcp = point_at(points, mcp_idx)
        if tip is None or pip is None or mcp is None:
            return False

        # Combina distância do pulso com direção vertical. Isso é mais robusto para mão inclinada.
        d_tip = dist(tip, wrist)
        d_pip = dist(pip, wrist)
        d_mcp = dist(mcp, wrist)
        return (d_tip > max(d_pip * 1.10, d_mcp * 1.18)) or (tip["y"] < pip["y"] - 5)

    index_open = finger_extended(8, 6, 5)
    middle_open = finger_extended(12, 10, 9)
    ring_open = finger_extended(16, 14, 13)
    pinky_open = finger_extended(20, 18, 17)

    thumb_tip = point_at(points, 4)
    thumb_ip = point_at(points, 3)
    thumb_mcp = point_at(points, 2)

    thumb_open = False
    if thumb_tip is not None and thumb_ip is not None and thumb_mcp is not None:
        d_tip = dist(thumb_tip, wrist)
        d_ip = dist(thumb_ip, wrist)
        d_mcp = dist(thumb_mcp, wrist)
        thumb_open = d_tip > max(d_ip * 1.08, d_mcp * 1.15)

    opened = [thumb_open, index_open, middle_open, ring_open, pinky_open]
    count = sum(1 for v in opened if v)

    if count >= 4:
        return "mao aberta"

    if thumb_open and not index_open and not middle_open and not ring_open and not pinky_open:
        return "joinha"

    if index_open and not middle_open and not ring_open and not pinky_open:
        return "apontando"

    if index_open and middle_open and not ring_open and not pinky_open:
        return "vitoria"

    if count <= 1:
        return "punho fechado"

    return ""


def classify_body_gestures(pose_points, hand_gestures):
    gestures = []

    ls = point_at(pose_points, 11)
    rs = point_at(pose_points, 12)
    lw = point_at(pose_points, 15)
    rw = point_at(pose_points, 16)
    lh = point_at(pose_points, 23)
    rh = point_at(pose_points, 24)
    lk = point_at(pose_points, 25)
    rk = point_at(pose_points, 26)
    la = point_at(pose_points, 27)
    ra = point_at(pose_points, 28)

    if ls and lw and lw["y"] < ls["y"] - 20:
        gestures.append("braco esquerdo levantado")

    if rs and rw and rw["y"] < rs["y"] - 20:
        gestures.append("braco direito levantado")

    left_knee = angle_degrees(lh, lk, la)
    right_knee = angle_degrees(rh, rk, ra)

    if left_knee is not None and right_knee is not None and left_knee < 135 and right_knee < 135:
        gestures.append("agachamento")

    if lh and lk and lk["y"] < lh["y"] - 10:
        gestures.append("perna esquerda levantada")

    if rh and rk and rk["y"] < rh["y"] - 10:
        gestures.append("perna direita levantada")

    for hand in hand_gestures:
        label = hand.get("gesture", "")
        if label and label not in gestures:
            gestures.append(label)

    return gestures[:10]


def draw_limb(img, p1, p2, color, thickness=2):
    if p1 is None or p2 is None:
        return

    cv2.line(
        img,
        (int(p1["x"]), int(p1["y"])),
        (int(p2["x"]), int(p2["y"])),
        color,
        thickness,
        cv2.LINE_AA
    )


def draw_joint(img, p, label=None, color=(180, 255, 220), radius=2):
    if p is None:
        return

    x, y = int(p["x"]), int(p["y"])
    cv2.circle(img, (x, y), radius, color, -1, lineType=cv2.LINE_AA)


def draw_exoskeleton_overlay(img, detection, source_shape=None):
    if img is None or detection is None:
        return img

    out = img.copy()
    overlay = out.copy()
    out_h, out_w = out.shape[:2]

    if source_shape is None:
        source_shape = out.shape

    source_h, source_w = source_shape[:2]
    sx = out_w / max(1.0, float(source_w))
    sy = out_h / max(1.0, float(source_h))

    def scale_p(p):
        if p is None:
            return None
        return {
            "x": int(p["x"] * sx),
            "y": int(p["y"] * sy),
            "z": float(p.get("z", 0.0)),
            "visibility": float(p.get("visibility", 1.0))
        }

    pose_points = [scale_p(p) for p in detection.get("pose_points", [])]

    is_depth = detection.get("source") == "depth"

    if pose_points:
        if is_depth:
            arm_color = (255, 255, 255)
            trunk_color = (0, 255, 255)
            leg_color = (255, 255, 255)
            joint_color = (0, 255, 255)
            body_thickness = 3
            joint_radius = 3
        else:
            arm_color = (90, 220, 120)
            trunk_color = (120, 200, 255)
            leg_color = (220, 120, 220)
            joint_color = (220, 240, 255)
            body_thickness = 2
            joint_radius = 2

        for a, b in [(11, 13), (13, 15), (12, 14), (14, 16)]:
            draw_limb(overlay, point_at(pose_points, a), point_at(pose_points, b), arm_color, body_thickness)

        for a, b in [(11, 12), (11, 23), (12, 24), (23, 24)]:
            draw_limb(overlay, point_at(pose_points, a), point_at(pose_points, b), trunk_color, body_thickness)

        for a, b in [(23, 25), (25, 27), (24, 26), (26, 28)]:
            draw_limb(overlay, point_at(pose_points, a), point_at(pose_points, b), leg_color, body_thickness)

        for idx in [11, 12, 13, 14, 15, 16, 23, 24, 25, 26, 27, 28]:
            draw_joint(overlay, point_at(pose_points, idx), color=joint_color, radius=joint_radius)

    for hand in detection.get("hand_points", []):
        points = [scale_p(p) for p in hand.get("points", [])]
        hand_line_color = (255, 255, 255) if is_depth else (0, 210, 255)
        hand_joint_color = (0, 255, 255) if is_depth else (120, 240, 255)
        hand_thickness = 2 if is_depth else 1
        hand_radius = 2 if is_depth else 1
        for a, b in HAND_CONNECTIONS_FAST:
            draw_limb(overlay, point_at(points, a), point_at(points, b), hand_line_color, hand_thickness)
        for idx in range(21):
            draw_joint(overlay, point_at(points, idx), color=hand_joint_color, radius=hand_radius)

    alpha = 0.88 if is_depth else 0.58
    cv2.addWeighted(overlay, alpha, out, 1.0 - alpha, 0, out)
    return out


def smooth_point(prev, curr, alpha=TEMPORAL_ALPHA):
    if prev is None:
        return curr
    if curr is None:
        return prev
    return {
        "i": int(curr.get("i", prev.get("i", 0))),
        "x": int(prev.get("x", 0) * (1.0 - alpha) + curr.get("x", 0) * alpha),
        "y": int(prev.get("y", 0) * (1.0 - alpha) + curr.get("y", 0) * alpha),
        "z": float(prev.get("z", 0.0) * (1.0 - alpha) + curr.get("z", 0.0) * alpha),
        "visibility": float(prev.get("visibility", 1.0) * (1.0 - alpha) + curr.get("visibility", 1.0) * alpha),
    }


def smooth_points(prev_points, curr_points, alpha=TEMPORAL_ALPHA):
    if not curr_points:
        return []
    if not prev_points:
        return curr_points

    out = []
    prev_by_i = {int(p.get("i", idx)): p for idx, p in enumerate(prev_points) if p is not None}

    for idx, curr in enumerate(curr_points):
        if curr is None:
            out.append(None)
            continue
        key = int(curr.get("i", idx))
        out.append(smooth_point(prev_by_i.get(key), curr, alpha))

    return out


def smooth_bbox(prev_bbox, curr_bbox, alpha=TEMPORAL_ALPHA):
    if curr_bbox is None:
        return prev_bbox
    if prev_bbox is None:
        return curr_bbox
    return [
        int(prev_bbox[0] * (1.0 - alpha) + curr_bbox[0] * alpha),
        int(prev_bbox[1] * (1.0 - alpha) + curr_bbox[1] * alpha),
        int(prev_bbox[2] * (1.0 - alpha) + curr_bbox[2] * alpha),
        int(prev_bbox[3] * (1.0 - alpha) + curr_bbox[3] * alpha),
    ]


def stabilize_gestures(source, gestures):
    state = temporal_tracker.setdefault(source, {"bbox": None, "pose_points": [], "hand_points": [], "gesture_history": deque(maxlen=GESTURE_HISTORY_LEN), "last_seen": 0.0})
    hist = state["gesture_history"]

    current = [g for g in list(gestures or []) if g]
    if current:
        hist.append(current)

    if not hist:
        return []

    counts = Counter()
    for row in hist:
        for g in row:
            counts[g] += 1

    # Dois frames já confirmam; se for o primeiro frame, mostra imediatamente.
    threshold = 1 if len(hist) <= 2 else 2
    stable = [g for g, c in counts.most_common() if c >= threshold]

    if stable:
        return stable[:10]

    return current[:10]


def apply_temporal_filter(source, bbox, pose_points, hand_items):
    state = temporal_tracker.setdefault(source, {"bbox": None, "pose_points": [], "hand_points": [], "gesture_history": deque(maxlen=GESTURE_HISTORY_LEN), "last_seen": 0.0})

    if state.get("last_seen") and (time.time() - float(state.get("last_seen", 0.0)) > 1.2):
        state["bbox"] = None
        state["pose_points"] = []
        state["hand_points"] = []
        state["gesture_history"].clear()

    bbox = smooth_bbox(state.get("bbox"), bbox)
    pose_points = smooth_points(state.get("pose_points", []), pose_points)

    prev_hands = state.get("hand_points", []) or []
    smoothed_hands = []
    for idx, hand in enumerate(hand_items or []):
        prev_hand = prev_hands[idx] if idx < len(prev_hands) else None
        hand_points = smooth_points(prev_hand.get("points", []) if prev_hand else [], hand.get("points", []), alpha=0.72)
        smoothed_hands.append({
            "handedness": hand.get("handedness", ""),
            "score": hand.get("score", 0.0),
            "gesture": hand.get("gesture", ""),
            "points": hand_points
        })

    state["bbox"] = bbox
    state["pose_points"] = pose_points
    state["hand_points"] = smoothed_hands
    state["last_seen"] = time.time()

    return bbox, pose_points, smoothed_hands


def detect_mediapipe_people_on_bgr(frame, depth=None, source="rgb", distance_source_shape=None):
    global last_detection_error
    global object_detector_mediapipe_error

    detections = []

    try:
        if frame is None:
            return detections

        pose_detector, face_detector, hands_detector = get_mediapipe_detectors()

        if pose_detector is None and face_detector is None and hands_detector is None:
            last_detection_error = object_detector_mediapipe_error
            return detections

        original_shape = frame.shape

        pose_input_w = safe_int(
            config.get("mediapipe_pose_input_width", config.get("mediapipe_input_width", 256)),
            256,
            160,
            1280
        )
        hands_input_w = safe_int(
            config.get("mediapipe_hands_input_width", 480),
            480,
            224,
            1280
        )

        work_frame = resize_frame_to_width(frame, pose_input_w)
        work_shape = work_frame.shape

        rgb = cv2.cvtColor(work_frame, cv2.COLOR_BGR2RGB)
        rgb.flags.writeable = False

        pose_points = []
        hand_items = []
        bbox = None
        confidence = 0.0
        model_name = "mediapipe_exoskeleton"

        if pose_detector is not None:
            with mediapipe_lock:
                pose_results = pose_detector.process(rgb)

            if pose_results is not None and getattr(pose_results, "pose_landmarks", None):
                work_pose_points = landmarks_to_points(
                    pose_results.pose_landmarks,
                    work_shape,
                    visibility_threshold=0.20
                )
                pose_points = scale_points_to_shape(work_pose_points, work_shape, original_shape)
                bbox, confidence = mediapipe_pose_bbox(pose_results, work_shape)
                bbox = scale_bbox_to_shape(bbox, work_shape, original_shape)
                model_name = "mediapipe_pose_exoskeleton"

        if hands_detector is not None and bool(config.get("mediapipe_hands_enabled", True)):
            hands_frame = resize_frame_to_width(frame, hands_input_w)
            hands_shape = hands_frame.shape
            hands_rgb = cv2.cvtColor(hands_frame, cv2.COLOR_BGR2RGB)
            hands_rgb.flags.writeable = False

            with mediapipe_lock:
                hands_results = hands_detector.process(hands_rgb)

            multi_landmarks = getattr(hands_results, "multi_hand_landmarks", None)
            multi_handedness = getattr(hands_results, "multi_handedness", None)

            if multi_landmarks:
                for i, hand_lms in enumerate(multi_landmarks):
                    handedness = ""
                    score = 0.0

                    try:
                        cls = multi_handedness[i].classification[0]
                        handedness = str(cls.label)
                        score = float(cls.score)
                    except Exception:
                        pass

                    work_points = landmarks_to_points(hand_lms, hands_shape, visibility_threshold=0.0)
                    points = scale_points_to_shape(work_points, hands_shape, original_shape)
                    gesture = classify_hand_gesture(points, handedness)

                    hand_items.append({
                        "handedness": handedness,
                        "score": round(score, 3),
                        "gesture": gesture,
                        "points": points
                    })

        if bbox is None and hand_items:
            all_points = []
            for hand in hand_items:
                all_points.extend(hand.get("points", []))
            bbox = bbox_from_points(all_points, original_shape, pad_ratio=0.35)
            confidence = max([float(h.get("score", 0.45)) for h in hand_items] + [0.45])
            model_name = "mediapipe_hands_only"

        if bbox is None and face_detector is not None:
            with mediapipe_lock:
                face_results = face_detector.process(rgb)

            if getattr(face_results, "detections", None):
                face = face_results.detections[0]
                bbox, confidence = mediapipe_face_to_person_bbox(face, work_shape)
                bbox = scale_bbox_to_shape(bbox, work_shape, original_shape)
                model_name = "mediapipe_face_exoskeleton"

        if bbox is None:
            last_detection_error = object_detector_mediapipe_error
            return []

        bbox, pose_points, hand_items = apply_temporal_filter(source, bbox, pose_points, hand_items)

        if source == "depth":
            distance_mm = bbox_depth_distance_mm(depth, bbox, original_shape) if depth is not None else 0
            dtype = "depth_person"
        else:
            distance_mm = bbox_depth_distance_mm(depth, bbox, distance_source_shape or original_shape)
            dtype = "person"

        gestures = classify_body_gestures(pose_points, hand_items) if bool(config.get("gesture_enabled", True)) else []
        gestures = stabilize_gestures(source, gestures)

        detections.append({
            "type": dtype,
            "label": "exoesqueleto",
            "bbox": bbox,
            "confidence": round(max(0.10, min(0.99, confidence)), 3),
            "distance_mm": int(distance_mm),
            "source": source,
            "model": model_name,
            "pose_points": pose_points,
            "hand_points": hand_items,
            "gestures": gestures
        })

        detections = nms_detections(detections, iou_threshold=0.30, limit=3)
        last_detection_error = object_detector_mediapipe_error
        return detections

    except Exception:
        object_detector_mediapipe_error = traceback.format_exc()
        last_detection_error = object_detector_mediapipe_error
        return detections



def detect_mediapipe_rgb_people(frame, depth=None):
    return detect_mediapipe_people_on_bgr(
        frame,
        depth=depth,
        source="rgb",
        distance_source_shape=frame.shape if frame is not None else None
    )


def depth_to_mediapipe_bgr(depth):
    return None


def detect_mediapipe_depth_people(depth):
    return []


def project_rgb_detections_to_depth(rgb_detections, depth, rgb_shape):
    return []


def bbox_touches_border(bbox, width, height, margin=4):
    x1, y1, x2, y2 = bbox

    return (
        x1 <= margin or
        y1 <= margin or
        x2 >= width - 1 - margin or
        y2 >= height - 1 - margin
    )


def surrounding_depth_distance_mm(depth, bbox, pad=14):
    try:
        if depth is None:
            return 0

        h, w = depth.shape[:2]
        x1, y1, x2, y2 = bbox
        x1, y1, x2, y2 = clip_bbox(x1, y1, x2, y2, w, h)

        px1 = max(0, x1 - pad)
        py1 = max(0, y1 - pad)
        px2 = min(w - 1, x2 + pad)
        py2 = min(h - 1, y2 + pad)

        outer = depth[py1:py2, px1:px2].copy()

        if outer.size == 0:
            return 0

        ix1 = x1 - px1
        iy1 = y1 - py1
        ix2 = x2 - px1
        iy2 = y2 - py1

        try:
            outer[iy1:iy2, ix1:ix2] = 0
        except Exception:
            pass

        values = valid_depth_values(outer, use_near_far=False)

        if values.size == 0:
            return sample_depth_mm(depth, (x1 + x2) // 2, (y1 + y2) // 2, radius=20)

        return int(np.median(values))

    except Exception:
        return 0


def remove_border_connected(mask):
    try:
        h, w = mask.shape[:2]
        work = mask.astype(np.uint8).copy()
        ff_mask = np.zeros((h + 2, w + 2), dtype=np.uint8)

        for x in range(w):
            if work[0, x]:
                cv2.floodFill(work, ff_mask, (x, 0), 0)
            if work[h - 1, x]:
                cv2.floodFill(work, ff_mask, (x, h - 1), 0)

        for y in range(h):
            if work[y, 0]:
                cv2.floodFill(work, ff_mask, (0, y), 0)
            if work[y, w - 1]:
                cv2.floodFill(work, ff_mask, (w - 1, y), 0)

        return work > 0

    except Exception:
        return mask


def clip_bbox(x1, y1, x2, y2, width, height):
    x1 = max(0, min(int(x1), width - 1))
    y1 = max(0, min(int(y1), height - 1))
    x2 = max(0, min(int(x2), width - 1))
    y2 = max(0, min(int(y2), height - 1))

    if x2 <= x1:
        x2 = min(width - 1, x1 + 1)

    if y2 <= y1:
        y2 = min(height - 1, y1 + 1)

    return x1, y1, x2, y2


def bbox_area(bbox):
    x1, y1, x2, y2 = bbox
    return max(0, int(x2) - int(x1)) * max(0, int(y2) - int(y1))


def bbox_iou(a, b):
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b

    ix1 = max(ax1, bx1)
    iy1 = max(ay1, by1)
    ix2 = min(ax2, bx2)
    iy2 = min(ay2, by2)

    inter = bbox_area((ix1, iy1, ix2, iy2))

    if inter <= 0:
        return 0.0

    union = bbox_area(a) + bbox_area(b) - inter

    if union <= 0:
        return 0.0

    return float(inter) / float(union)


def nms_detections(detections, iou_threshold=0.35, limit=12):
    if not detections:
        return []

    ordered = sorted(
        detections,
        key=lambda item: float(item.get("confidence", 0.0)),
        reverse=True
    )

    kept = []

    for det in ordered:
        bbox = det.get("bbox", [0, 0, 1, 1])

        should_keep = True

        for prev in kept:
            prev_bbox = prev.get("bbox", [0, 0, 1, 1])
            if bbox_iou(bbox, prev_bbox) > iou_threshold:
                should_keep = False
                break

        if should_keep:
            kept.append(det)

        if len(kept) >= limit:
            break

    return kept


def bbox_depth_distance_mm(depth, bbox, source_shape):
    if depth is None:
        return 0

    try:
        source_h, source_w = source_shape[:2]
        depth_h, depth_w = depth.shape[:2]
        x1, y1, x2, y2 = bbox

        dx1 = int((x1 / max(1, source_w)) * depth_w)
        dx2 = int((x2 / max(1, source_w)) * depth_w)
        dy1 = int((y1 / max(1, source_h)) * depth_h)
        dy2 = int((y2 / max(1, source_h)) * depth_h)

        dx1, dy1, dx2, dy2 = clip_bbox(dx1, dy1, dx2, dy2, depth_w, depth_h)

        margin_x = max(1, int((dx2 - dx1) * 0.18))
        margin_y = max(1, int((dy2 - dy1) * 0.18))

        rx1 = min(dx2, dx1 + margin_x)
        rx2 = max(dx1 + 1, dx2 - margin_x)
        ry1 = min(dy2, dy1 + margin_y)
        ry2 = max(dy1 + 1, dy2 - margin_y)

        roi = depth[ry1:ry2, rx1:rx2]
        values = valid_depth_values(roi, use_near_far=False)

        if values.size == 0:
            return 0

        return int(np.median(values))
    except Exception:
        return 0



def detect_rgb_objects(frame, depth=None):
    return detect_rgb_selected_mode(frame)

def reset_simple_rgb_detector():
    global simple_bg_subtractor
    global simple_detection_last_reset

    simple_bg_subtractor = cv2.createBackgroundSubtractorMOG2(
        history=80,
        varThreshold=32,
        detectShadows=False
    )
    simple_detection_last_reset = time.time()


def detect_simple_rgb_objects(frame):
    global simple_bg_subtractor
    global last_detection_error

    detections = []

    try:
        if frame is None:
            return detections

        if simple_bg_subtractor is None:
            reset_simple_rgb_detector()

        h, w = frame.shape[:2]
        target_w = 320

        if w > target_w:
            scale = target_w / float(w)
            work = cv2.resize(frame, (target_w, max(1, int(h * scale))), interpolation=cv2.INTER_AREA)
        else:
            scale = 1.0
            work = frame

        gray = cv2.cvtColor(work, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)

        mask = simple_bg_subtractor.apply(gray, learningRate=0.015)
        _, mask = cv2.threshold(mask, 200, 255, cv2.THRESH_BINARY)

        kernel3 = np.ones((3, 3), np.uint8)
        kernel7 = np.ones((7, 7), np.uint8)

        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel3, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel7, iterations=2)
        mask = cv2.dilate(mask, kernel3, iterations=1)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        candidates = []
        wh, ww = work.shape[:2]
        frame_area = float(wh * ww)

        for contour in contours:
            area = float(cv2.contourArea(contour))

            if area < max(250.0, frame_area * 0.006):
                continue

            x, y, bw, bh = cv2.boundingRect(contour)

            if bw < 12 or bh < 18:
                continue

            if area > frame_area * 0.85:
                continue

            aspect = bh / max(1.0, float(bw))

            if aspect < 0.28:
                continue

            candidates.append((area, x, y, bw, bh))

        if not candidates:
            return detections

        candidates.sort(reverse=True, key=lambda item: item[0])
        area, x, y, bw, bh = candidates[0]

        inv = 1.0 / max(0.0001, scale)
        x1 = int(x * inv)
        y1 = int(y * inv)
        x2 = int((x + bw) * inv)
        y2 = int((y + bh) * inv)
        x1, y1, x2, y2 = clip_bbox(x1, y1, x2, y2, w, h)

        confidence = min(0.99, max(0.20, area / max(1.0, frame_area * 0.20)))

        detections.append({
            "type": "simple_object",
            "label": "objeto/pessoa",
            "bbox": [x1, y1, x2, y2],
            "confidence": round(float(confidence), 3),
            "distance_mm": 0,
            "source": "rgb",
            "model": "opencv_mog2_light",
            "draw_mode": "green_box",
            "pose_points": [],
            "hand_points": [],
            "gestures": []
        })

        last_detection_error = ""
        return detections

    except Exception:
        last_detection_error = traceback.format_exc()
        return detections


def detect_rgb_selected_mode(frame):
    if bool(config.get("rgb_pose_hands_enabled", False)):
        return detect_mediapipe_rgb_people(frame, None)

    if bool(config.get("rgb_simple_detection_enabled", False)):
        return detect_simple_rgb_objects(frame)

    return []




def reset_depth_motion_detector():
    global depth_motion_subtractor
    global depth_motion_last_reset

    depth_motion_subtractor = cv2.createBackgroundSubtractorMOG2(
        history=90,
        varThreshold=18,
        detectShadows=False
    )
    depth_motion_last_reset = time.time()


def build_depth_motion_mask(depth, visible):
    global depth_motion_subtractor

    if depth is None or visible is None:
        return None

    if depth_motion_subtractor is None:
        reset_depth_motion_detector()

    near_mm = safe_int(config.get("near_mm", 200), 200, 0, 20000)
    far_mm = safe_int(config.get("far_mm", 4000), 4000, 1, 20000)
    if far_mm <= near_mm:
        far_mm = near_mm + 1

    depth_float = depth.astype(np.float32)
    clipped = np.clip(depth_float, near_mm, far_mm)

    gray = np.zeros(depth.shape, dtype=np.uint8)
    if np.any(visible):
        normalized = ((clipped - near_mm) / max(1.0, float(far_mm - near_mm)) * 255.0)
        normalized = np.clip(normalized, 0, 255).astype(np.uint8)
        # Mais perto mais claro ajuda a destacar o primeiro plano.
        gray[visible] = 255 - normalized[visible]

    try:
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
    except Exception:
        pass

    motion = depth_motion_subtractor.apply(gray, learningRate=0.02)
    _, motion = cv2.threshold(motion, 200, 255, cv2.THRESH_BINARY)
    motion[~visible] = 0

    kernel3 = np.ones((3, 3), np.uint8)
    kernel5 = np.ones((5, 5), np.uint8)
    motion = cv2.morphologyEx(motion, cv2.MORPH_OPEN, kernel3, iterations=1)
    motion = cv2.morphologyEx(motion, cv2.MORPH_CLOSE, kernel5, iterations=2)
    motion = cv2.dilate(motion, kernel3, iterations=1)
    return motion


def contour_to_pseudo_pose_points(contour, bbox, frame_shape):
    # Mantido para compatibilidade; o modo atual usa retângulo verde no depth.
    return []


def classify_depth_blob_as_person_like(bbox, area, frame_shape):
    h, w = frame_shape[:2]
    x1, y1, x2, y2 = bbox
    bw = max(1, x2 - x1)
    bh = max(1, y2 - y1)

    aspect = bh / float(bw)
    height_ratio = bh / max(1.0, float(h))
    area_ratio = area / max(1.0, float(w * h))

    # Heurística simples: objeto alto/vertical é mais parecido com pessoa.
    if aspect >= 1.15 and height_ratio >= 0.28 and area_ratio >= 0.015:
        return "pessoa/objeto"

    return "objeto"


def detect_depth_objects(depth):
    global last_detection_error

    detections = []

    try:
        if depth is None:
            return detections

        h, w = depth.shape[:2]
        visible = get_visible_depth_mask(depth)

        if np.count_nonzero(visible) < 80:
            return detections

        motion_mask = build_depth_motion_mask(depth, visible)
        if motion_mask is None or np.count_nonzero(motion_mask) < 40:
            return detections

        values = depth[visible].astype(np.uint16)

        if values.size < 80:
            return detections

        near_mm = safe_int(config.get("near_mm", 200), 200, 0, 20000)
        far_mm = safe_int(config.get("far_mm", 4000), 4000, 1, 20000)
        margin = safe_int(config.get("depth_foreground_margin_mm", 250), 250, 50, 2500)

        # O depth nao tem textura; a melhor leitura e separar o primeiro plano.
        # Usamos percentis para pegar o "grupo mais perto" e nao a parede inteira.
        p10 = int(np.percentile(values, 10))
        p20 = int(np.percentile(values, 20))
        p35 = int(np.percentile(values, 35))
        median_mm = int(np.median(values))

        limits = [
            p10 + margin,
            p20 + margin,
            p35,
            max(near_mm + 1, median_mm - margin),
        ]

        all_candidates = []
        min_area = max(80.0, float(w * h) * 0.006)
        max_area = float(w * h) * 0.70

        for limit in limits:
            limit = int(max(near_mm + 1, min(far_mm, limit)))

            mask_bool = visible & (depth <= limit) & (motion_mask > 0)

            if np.count_nonzero(mask_bool) < min_area:
                continue

            mask = (mask_bool.astype(np.uint8) * 255)

            # Limpeza de contorno: remove ruido e fecha buracos pequenos.
            try:
                mask = cv2.medianBlur(mask, 5)
                kernel3 = np.ones((3, 3), np.uint8)
                kernel7 = np.ones((7, 7), np.uint8)
                mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel3, iterations=1)
                mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel7, iterations=2)
                mask = cv2.dilate(mask, kernel3, iterations=1)
            except Exception:
                pass

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours:
                area = float(cv2.contourArea(contour))

                if area < min_area or area > max_area:
                    continue

                x, y, bw, bh = cv2.boundingRect(contour)

                if bw < 8 or bh < 8:
                    continue

                bbox = list(clip_bbox(x, y, x + bw, y + bh, w, h))
                x1, y1, x2, y2 = bbox
                bw = max(1, x2 - x1)
                bh = max(1, y2 - y1)

                width_ratio = bw / max(1.0, float(w))
                height_ratio = bh / max(1.0, float(h))
                aspect = bh / float(bw)

                # Evita pegar parede/tela inteira ou faixas horizontais.
                if width_ratio > 0.92 or height_ratio > 0.95:
                    continue

                if aspect < 0.20:
                    continue

                roi = depth[y1:y2, x1:x2]
                roi_values = valid_depth_values(roi, use_near_far=True)

                if roi_values.size < 30:
                    continue

                distance_mm = int(np.median(roi_values))
                center_bias = 1.0 - min(1.0, abs((x1 + x2) * 0.5 - w * 0.5) / max(1.0, w * 0.5))
                closeness = 1.0 - min(1.0, max(0.0, (distance_mm - p10) / max(1.0, float(far_mm - p10))))

                score = area * (0.65 + 0.35 * center_bias) * (0.65 + 0.35 * closeness)
                label = classify_depth_blob_as_person_like(bbox, area, depth.shape)

                all_candidates.append({
                    "score": float(score),
                    "type": "depth_contour_object",
                    "label": label,
                    "bbox": bbox,
                    "confidence": round(float(min(0.99, max(0.20, score / max(1.0, float(w * h) * 0.18)))), 3),
                    "distance_mm": int(distance_mm),
                    "source": "depth",
                    "model": "depth_nearest_contour",
                    "draw_mode": "green_box",
                    "pose_points": [],
                    "hand_points": [],
                    "gestures": []
                })

        if not all_candidates:
            return []

        all_candidates.sort(key=lambda item: item["score"], reverse=True)

        kept = []
        for item in all_candidates:
            if all(bbox_iou(item["bbox"], prev["bbox"]) < 0.35 for prev in kept):
                kept.append(item)
            if len(kept) >= 3:
                break

        for item in kept:
            item.pop("score", None)

        last_detection_error = ""
        return kept

    except Exception:
        last_detection_error = traceback.format_exc()
        return detections



def draw_detections(img, detections, source_shape=None, title=None):
    if img is None:
        return img

    out = img.copy()

    if not detections:
        return out

    for item in detections:
        bbox = item.get("bbox")
        if item.get("draw_mode") == "green_box" and bbox:
            x1, y1, x2, y2 = bbox

            if source_shape is not None:
                sh, sw = source_shape[:2]
                oh, ow = out.shape[:2]
                sx = ow / max(1.0, float(sw))
                sy = oh / max(1.0, float(sh))
                x1 = int(x1 * sx)
                x2 = int(x2 * sx)
                y1 = int(y1 * sy)
                y2 = int(y2 * sy)

            x1, y1, x2, y2 = clip_bbox(x1, y1, x2, y2, out.shape[1], out.shape[0])
            cv2.rectangle(out, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2, cv2.LINE_AA)

            if str(item.get("source", "")).lower() != "depth":
                label = str(item.get("label", "objeto"))
                distance = int(item.get("distance_mm", 0) or 0)
                text_label = label if distance <= 0 else f"{label} {distance}mm"
                put_text_box(out, text_label, (int(x1) + 5, max(20, int(y1) + 22)), scale=0.48, fg=(0, 255, 0), bg=(0, 0, 0), thickness=1)
            continue

        if bool(config.get("exoskeleton_enabled", True)):
            out = draw_exoskeleton_overlay(out, item, source_shape=source_shape)

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

    return colored



def depth_worker():
    global latest_depth_raw
    global latest_depth_colored
    global latest_depth_detections
    global latest_points
    global running
    global last_depth_error

    last_restart_time = 0.0
    first_valid_frame_received = False
    last_good_depth = None
    last_good_colored = None

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

                time.sleep(0.50)
                continue

            runtime = bridge_runtime_sec()

            if not depth_files_are_fresh():
                if not first_valid_frame_received and runtime < BRIDGE_FIRST_FRAME_GRACE_SEC:
                    last_depth_error = (
                        "Bridge inicializando OrbbecSDK em /dev/shm com fallback /tmp. "
                        f"Aguardando primeiro frame... runtime={runtime:.1f}s"
                    )
                    time.sleep(0.25)
                    continue

                if not first_valid_frame_received and now - last_restart_time > BRIDGE_RESTART_COOLDOWN_SEC:
                    last_restart_time = now
                    last_depth_error = (
                        "Bridge vivo, mas sem primeiro frame dentro da janela. "
                        f"runtime={runtime:.1f}s. Reiniciando bridge."
                    )
                    stop_depth_bridge()
                    time.sleep(2.0)
                    start_depth_bridge()
                    continue

                if last_good_colored is not None:
                    with depth_lock:
                        latest_depth_colored = last_good_colored.copy()
                        if last_good_depth is not None:
                            latest_depth_raw = last_good_depth.copy()

                last_depth_error = (
                    "Bridge vivo, aguardando novo frame fresco. "
                    f"active={ACTIVE_RAW_PATH} meta_age={meta_file_age_sec()}"
                )
                time.sleep(0.25)
                continue

            depth = read_depth_raw()

            if depth is None:
                if last_good_colored is not None:
                    with depth_lock:
                        latest_depth_colored = last_good_colored.copy()
                        if last_good_depth is not None:
                            latest_depth_raw = last_good_depth.copy()

                time.sleep(0.05)
                continue

            first_valid_frame_received = True

            if config.get("mirror_depth", False):
                depth = np.fliplr(depth)

            colored = depth_to_colormap(depth)

            last_good_depth = depth.copy()
            last_good_colored = colored.copy()

            if bool(config.get("depth_detection_enabled", False)):
                depth_detections = detect_depth_objects(depth)
                colored = draw_detections(
                    colored,
                    depth_detections,
                    source_shape=depth.shape,
                    title=None
                )
            else:
                depth_detections = []

            with detection_lock:
                latest_depth_detections = depth_detections

            with depth_lock:
                latest_depth_raw = depth.copy()
                latest_depth_colored = colored.copy()

            status = bridge_meta_status()
            if status == "stale_republish":
                last_depth_error = f"Bridge vivo em stale_republish. frame_age_ms={bridge_frame_age_ms()}"
            else:
                last_depth_error = ""

            time.sleep(safe_float(config.get("python_depth_read_interval_sec", 0.05), 0.05, 0.01, 1.0))

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
    global rgb_inference_thread
    global depth_thread

    running = False

    time.sleep(0.5)

    close_rgb_camera()

    rgb_thread = None
    rgb_inference_thread = None
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


@app.route("/api/start", methods=["POST"])
def api_start():
    global running
    global rgb_thread
    global rgb_inference_thread
    global depth_thread
    global latest_rgb
    global latest_rgb_raw
    global latest_rgb_source_shape
    global latest_depth_colored
    global latest_depth_raw
    global latest_points
    global latest_rgb_detections
    global latest_depth_detections
    global last_rgb_error
    global last_depth_error
    global last_detection_error

    data = request.get_json(force=True)

    stop_all()

    config["rgb_device"] = str(data.get("rgb_device", config["rgb_device"]))
    config["width"] = safe_int(data.get("width"), config["width"], 160, 1920)
    config["height"] = safe_int(data.get("height"), config["height"], 120, 1080)
    config["fps"] = safe_int(data.get("fps"), config["fps"], 1, 60)

    config["near_mm"] = safe_int(data.get("near_mm"), config["near_mm"], 0, 20000)
    config["far_mm"] = safe_int(data.get("far_mm"), config["far_mm"], 1, 20000)
    config["depth_foreground_margin_mm"] = safe_int(data.get("depth_foreground_margin_mm"), config.get("depth_foreground_margin_mm", 250), 50, 2000)

    config["jpeg_quality"] = safe_int(data.get("jpeg_quality"), config["jpeg_quality"], 20, 100)

    config["depth_display_scale"] = safe_float(data.get("depth_display_scale"), config["depth_display_scale"], 0.5, 4.0)
    config["depth_grid"] = safe_int(data.get("depth_grid"), config["depth_grid"], 1, 8)
    config["depth_sample_radius"] = safe_int(data.get("depth_sample_radius"), config["depth_sample_radius"], 1, 40)
    config["depth_colormap"] = str(data.get("depth_colormap", config["depth_colormap"])).upper()
    config["depth_color_direction"] = str(data.get("depth_color_direction", config["depth_color_direction"])).upper()
    config["depth_labels"] = False
    config["depth_median_blur"] = False
    config["depth_show_legend"] = False
    config["depth_invalid_contours"] = False
    config["depth_invalid_gray"] = False

    config["mirror_rgb"] = bool(data.get("mirror_rgb", False))
    config["mirror_depth"] = bool(data.get("mirror_depth", False))
    config["preview_enabled"] = True
    config["rgb_preview_enabled"] = True
    config["preview_size"] = str(data.get("preview_size", "medio"))
    config["detection_interval_sec"] = safe_float(data.get("detection_interval_sec"), config.get("detection_interval_sec", 0.08), 0.02, 2.0)
    config["mediapipe_input_width"] = safe_int(data.get("mediapipe_input_width"), config.get("mediapipe_input_width", 256), 160, 1280)
    config["mediapipe_pose_input_width"] = safe_int(data.get("mediapipe_pose_input_width"), config.get("mediapipe_pose_input_width", config.get("mediapipe_input_width", 256)), 160, 1280)
    config["mediapipe_hands_input_width"] = safe_int(data.get("mediapipe_hands_input_width"), config.get("mediapipe_hands_input_width", 320), 160, 1280)
    config["start_rgb_immediately"] = True
    config["rgb_pose_hands_enabled"] = bool(data.get("rgb_pose_hands_enabled", False))
    config["rgb_simple_detection_enabled"] = bool(data.get("rgb_simple_detection_enabled", False))

    legacy_detection = bool(data.get("object_detection_enabled", False))
    config["rgb_detection_enabled"] = bool(config["rgb_pose_hands_enabled"] or config["rgb_simple_detection_enabled"])
    config["depth_detection_enabled"] = bool(data.get("depth_detection_enabled", False))
    config["depth_silhouette_enabled"] = bool(data.get("depth_silhouette_enabled", config["depth_detection_enabled"]))
    config["depth_mediapipe_enabled"] = False
    config["depth_use_rgb_projection"] = False
    config["exoskeleton_enabled"] = True
    config["mediapipe_hands_enabled"] = bool(config["rgb_pose_hands_enabled"])
    config["mediapipe_face_enabled"] = False
    config["gesture_enabled"] = bool(config["rgb_pose_hands_enabled"])

    latest_rgb = None
    latest_rgb_raw = None
    latest_rgb_source_shape = None
    latest_depth_colored = None
    latest_depth_raw = None
    latest_rgb_detections = []
    latest_depth_detections = []
    last_rgb_error = ""
    last_depth_error = ""
    last_detection_error = ""
    reset_simple_rgb_detector()

    running = True

    start_rgb_now()

    depth_thread = threading.Thread(target=depth_worker, daemon=True)
    depth_thread.start()

    return jsonify({
        "ok": True,
        "message": "Câmera iniciada. RGB em baixa latência. MediaPipe e modo leve são opcionais por checkbox.",
        "config": config,
        "last_rgb_error": last_rgb_error,
        "last_depth_error": last_depth_error,
        "last_detection_error": last_detection_error,
        "rgb_detections": latest_rgb_detections,
        "depth_detections": latest_depth_detections
    })



@app.route("/api/stop", methods=["POST"])
def api_stop():
    stop_all()

    return jsonify({
        "ok": True,
        "message": "Câmera parada"
    })


def collect_gestures_from_detections(detections):
    gestures = []
    for det in detections or []:
        for gesture in det.get("gestures", []) or []:
            if gesture and gesture not in gestures:
                gestures.append(gesture)
    return gestures


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
        choose_active_depth_paths()
        if os.path.exists(ACTIVE_RAW_PATH):
            raw_age = round(time.time() - os.path.getmtime(ACTIVE_RAW_PATH), 3)
            raw_size = os.path.getsize(ACTIVE_RAW_PATH)

        if os.path.exists(ACTIVE_META_PATH):
            meta_age = round(time.time() - os.path.getmtime(ACTIVE_META_PATH), 3)
            meta_size = os.path.getsize(ACTIVE_META_PATH)
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
        "bridge_cmd": build_depth_bridge_cmd(),

        "raw_path": RAW_PATH,
        "meta_path": META_PATH,
        "alt_raw_path": ALT_RAW_PATH,
        "alt_meta_path": ALT_META_PATH,
        "active_raw_path": ACTIVE_RAW_PATH,
        "active_meta_path": ACTIVE_META_PATH,
        "raw_age_sec": raw_age,
        "meta_age_sec": meta_age,
        "bridge_meta_status": bridge_meta_status(),
        "bridge_frame_age_ms": bridge_frame_age_ms(),
        "python_depth_fresh_timeout_sec": config.get("python_depth_fresh_timeout_sec", 30.0),
        "raw_size": raw_size,
        "meta_size": meta_size,

        "last_depth_meta": last_depth_meta,
        "depth_stats": None if latest_depth_raw is None else depth_stats(latest_depth_raw),

        "last_rgb_error": last_rgb_error,
        "last_depth_error": last_depth_error,
        "last_detection_error": last_detection_error,
        "mediapipe_available": object_detector_mediapipe_available,
        "mediapipe_error": object_detector_mediapipe_error,
        "rgb_detections": latest_rgb_detections,
        "depth_detections": latest_depth_detections,
        "rgb_detection_count": len(latest_rgb_detections),
        "depth_detection_count": len(latest_depth_detections),
        "rgb_gestures": collect_gestures_from_detections(latest_rgb_detections),
        "depth_gestures": [],
        "latest_rgb_source_shape": None if latest_rgb_source_shape is None else list(latest_rgb_source_shape),
        "rgb_preview_enabled": config.get("rgb_preview_enabled", False),
        "rgb_pose_hands_enabled": config.get("rgb_pose_hands_enabled", False),
        "rgb_simple_detection_enabled": config.get("rgb_simple_detection_enabled", False),
        "rgb_capture_fps": last_rgb_capture_fps,
        "rgb_inference_fps": last_rgb_inference_fps,
        "rgb_detection_duration_ms": last_rgb_detection_duration_ms,
        "rgb_pipeline": "capture_thread_plus_optional_inference_thread",
        "depth_silhouette_enabled": config.get("depth_silhouette_enabled", False),
        "depth_mediapipe_enabled": False,
        "depth_detection_enabled": config.get("depth_detection_enabled", False),
        "depth_use_rgb_projection": False,
        "recognition_output": "rgb_preview",
        "recognition_input": "checkbox_mediapipe_or_simple_green_box",
        "pose_hands_draw_target": "rgb_preview",
        "depth_draw_target": "depth_colored_optional_contour",
        "usb_actions_removed": True,
        "bridge_log_tail": tail_file(LOG_PATH, 8000),
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
                msg = "RGB indisponível. O RGB pode estar apenas como sensor interno para reconhecimento."
                frame = make_error_image(msg)

            yield frame_to_mjpeg(frame)
            time.sleep(0.01)

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

    print("Painel Orbbec Astra Pro iniciado em http://127.0.0.1:5007")
    print("Backend depth: OrbbecSDK bridge com /dev/shm + fallback /tmp. RGB em baixa latência com MediaPipe opcional.")
    print("Project root:", PROJECT_ROOT)
    print("SDK root:", ORBBEC_SDK_ROOT)
    print("Bridge:", BRIDGE_PATH)
    print("Lib dir:", ORBBEC_LIB_DIR)
    print("RGB detectado:", config["rgb_device"])

    app.run(host="0.0.0.0", port=5007, debug=False, threaded=True)
