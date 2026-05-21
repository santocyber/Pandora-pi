import argparse
import math
import threading
import time
from flask import Flask, jsonify, render_template_string
import serial


HTML = """
<!doctype html>
<html lang="pt-br">
<head>
<meta charset="utf-8">
<title>STL-06P LiDAR Radar</title>
<meta name="viewport" content="width=device-width, initial-scale=1">

<style>
html, body {
    margin: 0;
    padding: 0;
    background: #020617;
    color: #e5e7eb;
    font-family: Arial, sans-serif;
    overflow: hidden;
}

#top {
    height: 62px;
    background: linear-gradient(90deg, #020617, #0f172a, #111827);
    border-bottom: 1px solid #334155;
    display: flex;
    align-items: center;
    gap: 18px;
    padding: 0 20px;
    box-sizing: border-box;
    box-shadow: 0 0 25px rgba(56, 189, 248, 0.15);
}

#top strong {
    color: #38bdf8;
    font-size: 18px;
}

#top span {
    font-size: 14px;
}

#canvas {
    display: block;
    width: 100vw;
    height: calc(100vh - 62px);
}

#panel {
    position: fixed;
    top: 82px;
    left: 16px;
    width: 405px;
    max-height: calc(100vh - 105px);
    overflow-y: auto;
    background: rgba(15, 23, 42, 0.94);
    border: 1px solid rgba(148, 163, 184, 0.28);
    border-radius: 16px;
    padding: 16px;
    box-sizing: border-box;
    z-index: 10;
    font-size: 14px;
    backdrop-filter: blur(12px);
    box-shadow: 0 0 30px rgba(0, 0, 0, 0.45);
}

#panel::-webkit-scrollbar {
    width: 7px;
}

#panel::-webkit-scrollbar-track {
    background: #020617;
}

#panel::-webkit-scrollbar-thumb {
    background: #334155;
    border-radius: 10px;
}

#panel h3 {
    margin: 0 0 12px 0;
    color: #f8fafc;
    font-size: 16px;
}

.row {
    margin-bottom: 7px;
    display: flex;
    justify-content: space-between;
    gap: 12px;
}

.label {
    color: #94a3b8;
}

.value {
    color: #f8fafc;
    font-weight: bold;
}

.ok {
    color: #22c55e;
}

.bad {
    color: #ef4444;
}

.warn {
    color: #facc15;
}

input[type="range"] {
    width: 100%;
    accent-color: #38bdf8;
}

input[type="checkbox"] {
    accent-color: #38bdf8;
}

.control-block {
    margin-top: 14px;
}

.control-block label {
    display: block;
    color: #cbd5e1;
    font-size: 13px;
    margin-bottom: 5px;
}

.check-label {
    display: flex !important;
    align-items: center;
    gap: 8px;
}

.badge {
    padding: 3px 8px;
    border-radius: 999px;
    background: rgba(56, 189, 248, 0.14);
    border: 1px solid rgba(56, 189, 248, 0.35);
    color: #7dd3fc;
    font-size: 12px;
}

#extremePanel {
    margin-top: 14px;
    border-top: 1px solid #334155;
    padding-top: 12px;
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 10px;
}

.extreme-card {
    border-radius: 14px;
    padding: 11px;
    background: rgba(2, 6, 23, 0.70);
    border: 1px solid rgba(148, 163, 184, 0.22);
}

.extreme-card.near {
    border-color: rgba(239, 68, 68, 0.65);
    box-shadow: 0 0 18px rgba(239, 68, 68, 0.16);
}

.extreme-card.far {
    border-color: rgba(168, 85, 247, 0.65);
    box-shadow: 0 0 18px rgba(168, 85, 247, 0.16);
}

.extreme-title {
    font-size: 12px;
    color: #cbd5e1;
    margin-bottom: 7px;
}

.extreme-distance {
    font-size: 19px;
    font-weight: bold;
    margin-bottom: 4px;
}

.extreme-card.near .extreme-distance {
    color: #f87171;
}

.extreme-card.far .extreme-distance {
    color: #c084fc;
}

.extreme-detail {
    font-size: 11px;
    color: #94a3b8;
    line-height: 1.45;
}

#stats {
    margin-top: 12px;
    border-top: 1px solid #334155;
    padding-top: 12px;
}

.export-box {
    margin-top: 14px;
    border-top: 1px solid #334155;
    padding-top: 12px;
}

.export-grid {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 8px;
}

.export-btn {
    width: 100%;
    border: 1px solid rgba(56, 189, 248, 0.45);
    background: rgba(14, 165, 233, 0.12);
    color: #e0f2fe;
    border-radius: 10px;
    padding: 9px 10px;
    font-size: 13px;
    cursor: pointer;
    transition: 0.2s;
}

.export-btn:hover {
    background: rgba(14, 165, 233, 0.26);
    box-shadow: 0 0 16px rgba(56, 189, 248, 0.22);
}

.export-btn.purple {
    border-color: rgba(168, 85, 247, 0.45);
    background: rgba(168, 85, 247, 0.12);
    color: #f3e8ff;
}

.export-btn.purple:hover {
    background: rgba(168, 85, 247, 0.25);
    box-shadow: 0 0 16px rgba(168, 85, 247, 0.22);
}

.export-btn.green {
    border-color: rgba(34, 197, 94, 0.45);
    background: rgba(34, 197, 94, 0.12);
    color: #dcfce7;
}

.export-btn.green:hover {
    background: rgba(34, 197, 94, 0.25);
    box-shadow: 0 0 16px rgba(34, 197, 94, 0.22);
}

.export-btn.orange {
    border-color: rgba(249, 115, 22, 0.45);
    background: rgba(249, 115, 22, 0.12);
    color: #ffedd5;
}

.export-btn.orange:hover {
    background: rgba(249, 115, 22, 0.25);
    box-shadow: 0 0 16px rgba(249, 115, 22, 0.22);
}

.export-note {
    margin-top: 8px;
    color: #94a3b8;
    font-size: 11px;
    line-height: 1.45;
}

.legend {
    margin-top: 14px;
    border-top: 1px solid #334155;
    padding-top: 12px;
}

.legend-title {
    font-weight: bold;
    color: #f8fafc;
    margin-bottom: 8px;
}

.legend-row {
    display: flex;
    align-items: center;
    gap: 9px;
    margin-bottom: 7px;
    color: #cbd5e1;
    font-size: 13px;
}

.color-box {
    width: 18px;
    height: 12px;
    border-radius: 4px;
    box-shadow: 0 0 10px currentColor;
}

.c1 { background: #ef4444; color: #ef4444; }
.c2 { background: #f97316; color: #f97316; }
.c3 { background: #facc15; color: #facc15; }
.c4 { background: #22c55e; color: #22c55e; }
.c5 { background: #06b6d4; color: #06b6d4; }
.c6 { background: #3b82f6; color: #3b82f6; }
.c7 { background: #a855f7; color: #a855f7; }

#lastHexBox {
    margin-top: 12px;
    border-top: 1px solid #334155;
    padding-top: 12px;
}

pre {
    white-space: pre-wrap;
    word-break: break-all;
    background: #020617;
    border: 1px solid #1e293b;
    border-radius: 10px;
    padding: 8px;
    max-height: 86px;
    overflow: auto;
    font-size: 11px;
    color: #a7f3d0;
}

#miniHelp {
    position: fixed;
    right: 18px;
    bottom: 18px;
    z-index: 10;
    background: rgba(15, 23, 42, 0.88);
    border: 1px solid rgba(148, 163, 184, 0.25);
    border-radius: 14px;
    padding: 12px 14px;
    color: #cbd5e1;
    font-size: 13px;
    backdrop-filter: blur(10px);
    line-height: 1.55;
}

#miniHelp strong {
    color: #f8fafc;
}

#preview3dCard {
    position: fixed;
    top: 82px;
    right: 16px;
    width: 430px;
    background: rgba(15, 23, 42, 0.94);
    border: 1px solid rgba(148, 163, 184, 0.28);
    border-radius: 16px;
    padding: 12px;
    box-sizing: border-box;
    z-index: 11;
    font-size: 13px;
    backdrop-filter: blur(12px);
    box-shadow: 0 0 30px rgba(0, 0, 0, 0.50);
}

#preview3dHeader {
    display: flex;
    align-items: center;
    justify-content: space-between;
    color: #f8fafc;
    margin-bottom: 9px;
}

#preview3dHeader strong {
    color: #e0f2fe;
}

#toggle3dBtn {
    border: 1px solid rgba(148, 163, 184, 0.35);
    background: rgba(2, 6, 23, 0.65);
    color: #cbd5e1;
    border-radius: 8px;
    padding: 4px 8px;
    cursor: pointer;
}

#preview3dViewport {
    width: 100%;
    height: 290px;
    background: #020617;
    border: 1px solid #1e293b;
    border-radius: 12px;
    overflow: hidden;
}

#preview3dControls {
    display: grid;
    grid-template-columns: repeat(3, 1fr);
    gap: 6px;
    margin-top: 9px;
}

#preview3dControls button {
    border: 1px solid rgba(56, 189, 248, 0.35);
    background: rgba(14, 165, 233, 0.12);
    color: #e0f2fe;
    border-radius: 8px;
    padding: 7px 4px;
    font-size: 12px;
    cursor: pointer;
}

#preview3dControls button:hover {
    background: rgba(14, 165, 233, 0.26);
}

#preview3dExtremesPanel {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 8px;
    margin-top: 9px;
}

.preview3d-badge {
    border-radius: 10px;
    padding: 8px 9px;
    font-size: 12px;
    line-height: 1.35;
    background: rgba(2, 6, 23, 0.72);
}

.preview3d-badge.near {
    border: 1px solid rgba(239, 68, 68, 0.55);
    color: #fecaca;
    box-shadow: 0 0 14px rgba(239, 68, 68, 0.12);
}

.preview3d-badge.far {
    border: 1px solid rgba(168, 85, 247, 0.55);
    color: #e9d5ff;
    box-shadow: 0 0 14px rgba(168, 85, 247, 0.12);
}

.preview3d-badge.near strong {
    color: #f87171;
}

.preview3d-badge.far strong {
    color: #c084fc;
}


.preview3d-note {
    margin-top: 8px;
    color: #94a3b8;
    font-size: 11px;
    line-height: 1.4;
}

#preview3dCard.hidden3d #preview3dViewport,
#preview3dCard.hidden3d #preview3dExtremesPanel,
#preview3dCard.hidden3d #preview3dControls,
#preview3dCard.hidden3d .control-block,
#preview3dCard.hidden3d .preview3d-note {
    display: none;
}

#preview3dUnavailable {
    display: none;
    padding: 12px;
    color: #fecaca;
    background: rgba(127, 29, 29, 0.35);
    border: 1px solid rgba(248, 113, 113, 0.35);
    border-radius: 10px;
    line-height: 1.45;
    font-size: 12px;
}

#preview3dUnavailable.visible {
    display: block;
}


@media (max-width: 760px) {
    #panel {
        width: calc(100vw - 32px);
        max-height: 58vh;
        overflow-y: auto;
    }

    #top {
        gap: 8px;
        padding: 0 10px;
    }

    #top span.badge {
        display: none;
    }

    #miniHelp {
        display: none;
    }

    #preview3dCard {
        left: 16px;
        right: 16px;
        top: auto;
        bottom: 16px;
        width: auto;
    }

    #preview3dViewport {
        height: 220px;
    }

}
</style>
</head>

<body>
<div id="top">
    <strong>LDROBOT STL-06P</strong>
    <span class="badge">Radar colorido</span>
    <span class="badge">Objetos identificados</span>
    <span class="badge">Exportação PNG/JSON/CSV/PLY</span>
    <span class="badge">Preview 3D com extremos</span>
    <span id="conn">conectando...</span>
</div>

<div id="panel">
    <h3>Status do LiDAR</h3>

    <div class="row"><span class="label">Serial</span> <span id="serialStatus" class="value">-</span></div>
    <div class="row"><span class="label">Bytes recebidos</span> <span id="bytesReceived" class="value">0</span></div>
    <div class="row"><span class="label">Pacotes válidos</span> <span id="framesOk" class="value">0</span></div>
    <div class="row"><span class="label">Pacotes inválidos</span> <span id="framesBad" class="value">0</span></div>
    <div class="row"><span class="label">Pontos atuais</span> <span id="pointsCount" class="value">0</span></div>
    <div class="row"><span class="label">Objetos detectados</span> <span id="objectsCount" class="value">0</span></div>
    <div class="row"><span class="label">Rotação</span> <span><span id="scanHz" class="value">0</span> Hz</span></div>
    <div class="row"><span class="label">Último pacote</span> <span><span id="lastAge" class="value">-</span> ms</span></div>

    <div id="extremePanel">
        <div class="extreme-card near">
            <div class="extreme-title">Ponto mais próximo</div>
            <div id="nearestDistance" class="extreme-distance">-</div>
            <div class="extreme-detail">
                Ângulo: <span id="nearestAngle">-</span>°<br>
                Confiança: <span id="nearestConf">-</span>
            </div>
        </div>

        <div class="extreme-card far">
            <div class="extreme-title">Ponto mais longe</div>
            <div id="farthestDistance" class="extreme-distance">-</div>
            <div class="extreme-detail">
                Ângulo: <span id="farthestAngle">-</span>°<br>
                Confiança: <span id="farthestConf">-</span>
            </div>
        </div>
    </div>

    <div id="stats">
        <div class="control-block">
            <label>Alcance máximo: <span id="maxRangeText">8000</span> mm</label>
            <input id="maxRange" type="range" min="500" max="12000" step="100" value="8000">
        </div>

        <div class="control-block">
            <label>Offset angular: <span id="angleOffsetText">-90</span>°</label>
            <input id="angleOffset" type="range" min="-180" max="180" step="1" value="-90">
        </div>

        <div class="control-block">
            <label>Tamanho dos pontos: <span id="pointSizeText">3</span> px</label>
            <input id="pointSize" type="range" min="1" max="9" step="1" value="3">
        </div>

        <div class="control-block">
            <label>Intensidade visual: <span id="trailText">0.22</span></label>
            <input id="trailAlpha" type="range" min="0.04" max="0.50" step="0.01" value="0.22">
        </div>

        <div class="control-block">
            <label class="check-label">
                <input id="showRuler" type="checkbox" checked>
                Mostrar régua de escala
            </label>
        </div>

        <div class="control-block">
            <label class="check-label">
                <input id="showObjects" type="checkbox" checked>
                Identificar objetos na tela
            </label>
        </div>

        <div class="control-block">
            <label class="check-label">
                <input id="showObjectLines" type="checkbox" checked>
                Mostrar linha até cada objeto
            </label>
        </div>

        <div class="control-block">
            <label>Máximo de objetos marcados: <span id="maxLabelsText">25</span></label>
            <input id="maxLabels" type="range" min="5" max="80" step="1" value="25">
        </div>

        <div class="control-block">
            <label>Sensibilidade de agrupamento: <span id="clusterText">350</span> mm</label>
            <input id="clusterDistance" type="range" min="100" max="1000" step="50" value="350">
        </div>
    </div>

    <div class="export-box">
        <div class="legend-title">Exportar mapa</div>
        <div class="export-grid">
            <button id="exportPngBtn" class="export-btn">PNG visual</button>
            <button id="exportJsonBtn" class="export-btn purple">JSON completo</button>
            <button id="exportCsvBtn" class="export-btn green">CSV pontos</button>
            <button id="exportPlyBtn" class="export-btn orange">PLY nuvem 3D</button>
        </div>
        <div class="export-note">
            JSON/CSV/PLY usam os pontos visíveis dentro do alcance atual.  
            PLY exporta X/Y em metros e Z = 0 para abrir como nuvem 2D em visualizadores 3D.
        </div>
    </div>

    <div class="legend">
        <div class="legend-title">Legenda de distância</div>
        <div class="legend-row"><span class="color-box c1"></span> 0 até 0,5 m — muito perto</div>
        <div class="legend-row"><span class="color-box c2"></span> 0,5 até 1 m</div>
        <div class="legend-row"><span class="color-box c3"></span> 1 até 2 m</div>
        <div class="legend-row"><span class="color-box c4"></span> 2 até 4 m</div>
        <div class="legend-row"><span class="color-box c5"></span> 4 até 6 m</div>
        <div class="legend-row"><span class="color-box c6"></span> 6 até 8 m</div>
        <div class="legend-row"><span class="color-box c7"></span> acima de 8 m</div>
    </div>

    <div id="lastHexBox">
        <div class="legend-title">Último pacote HEX</div>
        <pre id="lastHex">-</pre>
    </div>
</div>

<div id="miniHelp">
    <strong>Identificação visual</strong><br>
    Labels #1, #2, #3 = objetos agrupados<br>
    Vermelho = ponto mais próximo<br>
    Roxo = ponto mais longe<br>
    Régua inferior = escala visual<br>
    Exportação salva o mapa atual<br>
    Preview 3D mostra a nuvem como no Blender
</div>


<div id="preview3dCard">
    <div id="preview3dHeader">
        <strong>Preview 3D da nuvem</strong>
        <button id="toggle3dBtn">ocultar</button>
    </div>

    <div id="preview3dUnavailable">
        Three.js não carregou. Verifique a internet ou baixe os arquivos three.min.js e OrbitControls.js localmente.
    </div>

    <div id="preview3dViewport"></div>

    <div id="preview3dExtremesPanel">
        <div class="preview3d-badge near">
            <strong>Mais perto 3D</strong><br>
            <span id="preview3dNearestText">-</span>
        </div>

        <div class="preview3d-badge far">
            <strong>Mais longe 3D</strong><br>
            <span id="preview3dFarthestText">-</span>
        </div>
    </div>

    <div id="preview3dControls">
        <button id="viewTopBtn">Topo</button>
        <button id="viewSideBtn">Lateral</button>
        <button id="viewFrontBtn">Frente</button>
        <button id="viewBackBtn">Traseira</button>
        <button id="viewLeftBtn">Esquerda</button>
        <button id="viewRightBtn">Direita</button>
        <button id="viewIsoBtn">Isométrica</button>
        <button id="viewFirstBtn">1ª pessoa</button>
        <button id="viewNearBtn">Ver perto</button>
        <button id="viewFarBtn">Ver longe</button>
        <button id="viewResetBtn">Reset</button>
    </div>

    <div class="control-block">
        <label>Tamanho 3D dos pontos: <span id="point3dSizeText">0.035</span></label>
        <input id="point3dSize" type="range" min="0.01" max="0.15" step="0.005" value="0.035">
    </div>

    <div class="control-block">
        <label>Altura visual Z: <span id="zLiftText">0.00</span> m</label>
        <input id="zLift" type="range" min="0" max="2" step="0.05" value="0">
    </div>

    <div class="preview3d-note">
        Arraste para girar, use scroll para zoom. Esfera vermelha = mais perto; roxa = mais longe. Use Ver perto/Ver longe para focar os extremos.
    </div>
</div>

<canvas id="canvas"></canvas>

<script src="https://cdn.jsdelivr.net/npm/three@0.128.0/build/three.min.js"></script>
<script src="https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/controls/OrbitControls.js"></script>

<script>
const canvas = document.getElementById("canvas");
const ctx = canvas.getContext("2d");

const maxRange = document.getElementById("maxRange");
const angleOffset = document.getElementById("angleOffset");
const pointSize = document.getElementById("pointSize");
const trailAlpha = document.getElementById("trailAlpha");

const showRuler = document.getElementById("showRuler");
const showObjects = document.getElementById("showObjects");
const showObjectLines = document.getElementById("showObjectLines");
const maxLabels = document.getElementById("maxLabels");
const clusterDistance = document.getElementById("clusterDistance");

const maxRangeText = document.getElementById("maxRangeText");
const angleOffsetText = document.getElementById("angleOffsetText");
const pointSizeText = document.getElementById("pointSizeText");
const trailText = document.getElementById("trailText");
const maxLabelsText = document.getElementById("maxLabelsText");
const clusterText = document.getElementById("clusterText");

const point3dSize = document.getElementById("point3dSize");
const point3dSizeText = document.getElementById("point3dSizeText");
const zLift = document.getElementById("zLift");
const zLiftText = document.getElementById("zLiftText");

let scene3d = null;
let camera3d = null;
let renderer3d = null;
let controls3d = null;
let pointCloud3d = null;
let lidarMarker3d = null;
let nearestMarker3d = null;
let farthestMarker3d = null;
let nearestLine3d = null;
let farthestLine3d = null;
let grid3d = null;
let axes3d = null;
let preview3dVisible = true;
let last3dUpdate = 0;

let currentPoints = [];
let sweepAngle = 0;
let lastDetectedObjects = [];

function resize() {
    canvas.width = window.innerWidth;
    canvas.height = window.innerHeight - 62;
    resize3DPreview();
}

window.addEventListener("resize", resize);
resize();

maxRange.oninput = () => maxRangeText.textContent = maxRange.value;
angleOffset.oninput = () => angleOffsetText.textContent = angleOffset.value;
pointSize.oninput = () => pointSizeText.textContent = pointSize.value;
trailAlpha.oninput = () => trailText.textContent = trailAlpha.value;
maxLabels.oninput = () => maxLabelsText.textContent = maxLabels.value;
clusterDistance.oninput = () => clusterText.textContent = clusterDistance.value;
point3dSize.oninput = () => point3dSizeText.textContent = point3dSize.value;
zLift.oninput = () => zLiftText.textContent = Number(zLift.value).toFixed(2);

function formatDistance(distance) {
    if (!distance || distance <= 0) return "-";

    if (distance < 1000) {
        return Math.round(distance) + " mm";
    }

    return (distance / 1000).toFixed(2) + " m";
}

function getTimestampName() {
    const now = new Date();

    const y = now.getFullYear();
    const m = String(now.getMonth() + 1).padStart(2, "0");
    const d = String(now.getDate()).padStart(2, "0");
    const h = String(now.getHours()).padStart(2, "0");
    const min = String(now.getMinutes()).padStart(2, "0");
    const s = String(now.getSeconds()).padStart(2, "0");

    return y + "-" + m + "-" + d + "_" + h + "-" + min + "-" + s;
}

function downloadTextFile(filename, content, mimeType) {
    const blob = new Blob([content], { type: mimeType });
    const url = URL.createObjectURL(blob);

    const a = document.createElement("a");
    a.href = url;
    a.download = filename;
    document.body.appendChild(a);
    a.click();

    document.body.removeChild(a);
    URL.revokeObjectURL(url);
}

function getDistanceColor(distance) {
    if (distance < 500) return "239, 68, 68";
    if (distance < 1000) return "249, 115, 22";
    if (distance < 2000) return "250, 204, 21";
    if (distance < 4000) return "34, 197, 94";
    if (distance < 6000) return "6, 182, 212";
    if (distance < 8000) return "59, 130, 246";
    return "168, 85, 247";
}

function getDistanceSolidColor(distance) {
    if (distance < 500) return "#ef4444";
    if (distance < 1000) return "#f97316";
    if (distance < 2000) return "#facc15";
    if (distance < 4000) return "#22c55e";
    if (distance < 6000) return "#06b6d4";
    if (distance < 8000) return "#3b82f6";
    return "#a855f7";
}

function colorHexToRgbArray(hexColor) {
    const clean = hexColor.replace("#", "");
    const r = parseInt(clean.substring(0, 2), 16);
    const g = parseInt(clean.substring(2, 4), 16);
    const b = parseInt(clean.substring(4, 6), 16);
    return [r, g, b];
}

function getVisiblePoints(range) {
    return currentPoints.filter(function(p) {
        return p && p.distance > 0 && p.distance <= range;
    });
}

function getAngleDiff(a, b) {
    let d = Math.abs(a - b);
    if (d > 180) d = 360 - d;
    return d;
}

function findExtremePoints(range) {
    const points = getVisiblePoints(range);

    let nearest = null;
    let farthest = null;

    for (const p of points) {
        if (!nearest || p.distance < nearest.distance) {
            nearest = p;
        }

        if (!farthest || p.distance > farthest.distance) {
            farthest = p;
        }
    }

    return {
        nearest: nearest,
        farthest: farthest
    };
}

function pointToCanvas(p, cx, cy, scale, offset) {
    const a = (p.angle + offset) * Math.PI / 180.0;

    const x = Math.cos(a) * p.distance;
    const y = Math.sin(a) * p.distance;

    return {
        x: cx + x * scale,
        y: cy + y * scale,
        rad: a
    };
}

function pointToMeters(p) {
    const rad = p.angle * Math.PI / 180.0;
    const distanceM = p.distance / 1000.0;

    return {
        x_m: Math.cos(rad) * distanceM,
        y_m: Math.sin(rad) * distanceM,
        z_m: 0
    };
}

function detectObjects(range) {
    const points = getVisiblePoints(range)
        .filter(function(p) {
            return p.distance > 80;
        })
        .sort(function(a, b) {
            return a.angle - b.angle;
        });

    const maxDistanceGap = Number(clusterDistance.value);
    const maxAngleGap = 3.0;
    const minPointsPerObject = 2;

    let groups = [];
    let current = [];

    for (const p of points) {
        if (current.length === 0) {
            current.push(p);
            continue;
        }

        const last = current[current.length - 1];
        const angleGap = getAngleDiff(p.angle, last.angle);
        const distanceGap = Math.abs(p.distance - last.distance);

        if (angleGap <= maxAngleGap && distanceGap <= maxDistanceGap) {
            current.push(p);
        } else {
            if (current.length >= minPointsPerObject) {
                groups.push(current);
            }

            current = [p];
        }
    }

    if (current.length >= minPointsPerObject) {
        groups.push(current);
    }

    if (groups.length > 1) {
        const first = groups[0];
        const last = groups[groups.length - 1];

        const firstPoint = first[0];
        const lastPoint = last[last.length - 1];

        const wrapAngleGap = getAngleDiff(firstPoint.angle, lastPoint.angle);
        const wrapDistanceGap = Math.abs(firstPoint.distance - lastPoint.distance);

        if (wrapAngleGap <= maxAngleGap && wrapDistanceGap <= maxDistanceGap) {
            groups[0] = last.concat(first);
            groups.pop();
        }
    }

    let objects = [];

    for (const group of groups) {
        let sumX = 0;
        let sumY = 0;
        let sumDistance = 0;
        let minDistance = Infinity;
        let maxDistance = 0;
        let sumConfidence = 0;

        for (const p of group) {
            const rad = p.angle * Math.PI / 180.0;

            sumX += Math.cos(rad) * p.distance;
            sumY += Math.sin(rad) * p.distance;
            sumDistance += p.distance;
            sumConfidence += p.confidence || 0;

            if (p.distance < minDistance) minDistance = p.distance;
            if (p.distance > maxDistance) maxDistance = p.distance;
        }

        const avgX = sumX / group.length;
        const avgY = sumY / group.length;

        let angle = Math.atan2(avgY, avgX) * 180.0 / Math.PI;
        if (angle < 0) angle += 360.0;

        const avgDistance = Math.sqrt(avgX * avgX + avgY * avgY);

        objects.push({
            angle: Number(angle.toFixed(3)),
            distance: Number(avgDistance.toFixed(3)),
            minDistance: Number(minDistance.toFixed(3)),
            maxDistance: Number(maxDistance.toFixed(3)),
            avgRawDistance: Number((sumDistance / group.length).toFixed(3)),
            confidence: Number((sumConfidence / group.length).toFixed(3)),
            points: group.length
        });
    }

    objects.sort(function(a, b) {
        return a.distance - b.distance;
    });

    return objects.slice(0, Number(maxLabels.value));
}

function updateExtremePanel(range) {
    const extremes = findExtremePoints(range);
    const nearest = extremes.nearest;
    const farthest = extremes.farthest;

    if (nearest) {
        document.getElementById("nearestDistance").textContent = formatDistance(nearest.distance);
        document.getElementById("nearestAngle").textContent = Number(nearest.angle).toFixed(1);
        document.getElementById("nearestConf").textContent = nearest.confidence !== undefined ? nearest.confidence : "-";
    } else {
        document.getElementById("nearestDistance").textContent = "-";
        document.getElementById("nearestAngle").textContent = "-";
        document.getElementById("nearestConf").textContent = "-";
    }

    if (farthest) {
        document.getElementById("farthestDistance").textContent = formatDistance(farthest.distance);
        document.getElementById("farthestAngle").textContent = Number(farthest.angle).toFixed(1);
        document.getElementById("farthestConf").textContent = farthest.confidence !== undefined ? farthest.confidence : "-";
    } else {
        document.getElementById("farthestDistance").textContent = "-";
        document.getElementById("farthestAngle").textContent = "-";
        document.getElementById("farthestConf").textContent = "-";
    }
}

function buildExportData() {
    const range = Number(maxRange.value);
    const offset = Number(angleOffset.value);

    const points = getVisiblePoints(range);
    const objects = detectObjects(range);
    const extremes = findExtremePoints(range);

    const exportedPoints = points.map(function(p, index) {
        const meter = pointToMeters(p);
        const visualRad = (p.angle + offset) * Math.PI / 180.0;
        const distanceM = p.distance / 1000.0;
        const rgb = colorHexToRgbArray(getDistanceSolidColor(p.distance));

        return {
            index: index,
            angle_deg: Number(Number(p.angle).toFixed(3)),
            visual_angle_deg: Number(Number(p.angle + offset).toFixed(3)),
            distance_mm: Number(p.distance),
            distance_m: Number(distanceM.toFixed(6)),
            confidence: Number(p.confidence || 0),
            x_m: Number(meter.x_m.toFixed(6)),
            y_m: Number(meter.y_m.toFixed(6)),
            z_m: 0,
            visual_x_m: Number((Math.cos(visualRad) * distanceM).toFixed(6)),
            visual_y_m: Number((Math.sin(visualRad) * distanceM).toFixed(6)),
            color_r: rgb[0],
            color_g: rgb[1],
            color_b: rgb[2]
        };
    });

    return {
        exported_at: new Date().toISOString(),
        sensor: "LDROBOT STL-06P",
        export_type: "2D_lidar_map_snapshot",
        coordinate_system: {
            x_m: "cos(angle_deg) * distance_m",
            y_m: "sin(angle_deg) * distance_m",
            z_m: "0",
            angle_origin: "0 degrees from sensor protocol",
            visual_angle_deg: "angle_deg + current UI offset"
        },
        units: {
            angle: "degrees",
            distance_mm: "millimeters",
            distance_m: "meters",
            x_y_z: "meters"
        },
        ui_settings: {
            max_range_mm: range,
            angle_offset_deg: offset,
            point_size_px: Number(pointSize.value),
            intensity: Number(trailAlpha.value),
            object_cluster_distance_mm: Number(clusterDistance.value),
            max_object_labels: Number(maxLabels.value)
        },
        summary: {
            point_count: exportedPoints.length,
            object_count: objects.length,
            nearest_point: extremes.nearest,
            farthest_point: extremes.farthest
        },
        objects: objects,
        points: exportedPoints
    };
}

function exportPng() {
    const filename = "lidar_mapa_visual_" + getTimestampName() + ".png";
    const a = document.createElement("a");
    a.download = filename;
    a.href = canvas.toDataURL("image/png");
    a.click();
}

function exportJson() {
    const data = buildExportData();
    const filename = "lidar_mapa_completo_" + getTimestampName() + ".json";
    const content = JSON.stringify(data, null, 2);

    downloadTextFile(filename, content, "application/json;charset=utf-8");
}

function exportCsv() {
    const data = buildExportData();

    let csv = "";
    csv += "index,angle_deg,visual_angle_deg,distance_mm,distance_m,confidence,x_m,y_m,z_m,visual_x_m,visual_y_m,color_r,color_g,color_b\\n";

    for (const p of data.points) {
        csv += [
            p.index,
            p.angle_deg,
            p.visual_angle_deg,
            p.distance_mm,
            p.distance_m,
            p.confidence,
            p.x_m,
            p.y_m,
            p.z_m,
            p.visual_x_m,
            p.visual_y_m,
            p.color_r,
            p.color_g,
            p.color_b
        ].join(",") + "\\n";
    }

    const filename = "lidar_pontos_" + getTimestampName() + ".csv";

    downloadTextFile(filename, csv, "text/csv;charset=utf-8");
}

function exportPly() {
    const data = buildExportData();
    const points = data.points;

    let ply = "";
    ply += "ply\\n";
    ply += "format ascii 1.0\\n";
    ply += "comment Exportado do visualizador Flask STL-06P\\n";
    ply += "comment Coordenadas em metros; z = 0\\n";
    ply += "element vertex " + points.length + "\\n";
    ply += "property float x\\n";
    ply += "property float y\\n";
    ply += "property float z\\n";
    ply += "property uchar red\\n";
    ply += "property uchar green\\n";
    ply += "property uchar blue\\n";
    ply += "property float distance_m\\n";
    ply += "property float angle_deg\\n";
    ply += "property uchar confidence\\n";
    ply += "end_header\\n";

    for (const p of points) {
        ply += [
            p.x_m,
            p.y_m,
            p.z_m,
            p.color_r,
            p.color_g,
            p.color_b,
            p.distance_m,
            p.angle_deg,
            p.confidence
        ].join(" ") + "\\n";
    }

    const filename = "lidar_nuvem_2d_" + getTimestampName() + ".ply";

    downloadTextFile(filename, ply, "application/octet-stream");
}

function setupExportButtons() {
    document.getElementById("exportPngBtn").onclick = exportPng;
    document.getElementById("exportJsonBtn").onclick = exportJson;
    document.getElementById("exportCsvBtn").onclick = exportCsv;
    document.getElementById("exportPlyBtn").onclick = exportPly;
}

function init3DPreview() {
    const unavailable = document.getElementById("preview3dUnavailable");

    if (typeof THREE === "undefined") {
        unavailable.classList.add("visible");
        return;
    }

    const container = document.getElementById("preview3dViewport");

    scene3d = new THREE.Scene();
    scene3d.background = new THREE.Color(0x020617);

    const width = container.clientWidth || 430;
    const height = container.clientHeight || 290;

    camera3d = new THREE.PerspectiveCamera(60, width / height, 0.01, 1000);
    camera3d.up.set(0, 0, 1);
    camera3d.position.set(0, -7, 4.5);
    camera3d.lookAt(0, 0, 0);

    renderer3d = new THREE.WebGLRenderer({
        antialias: true,
        alpha: false
    });

    renderer3d.setPixelRatio(window.devicePixelRatio || 1);
    renderer3d.setSize(width, height);

    container.innerHTML = "";
    container.appendChild(renderer3d.domElement);

    if (typeof THREE.OrbitControls === "function") {
        controls3d = new THREE.OrbitControls(camera3d, renderer3d.domElement);
        controls3d.enableDamping = true;
        controls3d.dampingFactor = 0.08;
        controls3d.screenSpacePanning = false;
        controls3d.minDistance = 0.3;
        controls3d.maxDistance = 80;
        controls3d.target.set(0, 0, 0);
    }

    const ambient = new THREE.AmbientLight(0xffffff, 0.70);
    scene3d.add(ambient);

    const light = new THREE.DirectionalLight(0xffffff, 0.90);
    light.position.set(5, -5, 8);
    scene3d.add(light);

    grid3d = new THREE.GridHelper(16, 32, 0x334155, 0x1e293b);
    grid3d.rotation.x = Math.PI / 2;
    scene3d.add(grid3d);

    axes3d = new THREE.AxesHelper(1.2);
    scene3d.add(axes3d);

    const lidarGeometry = new THREE.SphereGeometry(0.035, 16, 12);
    const lidarMaterial = new THREE.MeshBasicMaterial({ color: 0xffffff });
    lidarMarker3d = new THREE.Mesh(lidarGeometry, lidarMaterial);
    lidarMarker3d.position.set(0, 0, 0.025);
    scene3d.add(lidarMarker3d);

    const nearestGeometry = new THREE.SphereGeometry(0.075, 24, 16);
    const nearestMaterial = new THREE.MeshBasicMaterial({ color: 0xef4444 });
    nearestMarker3d = new THREE.Mesh(nearestGeometry, nearestMaterial);
    nearestMarker3d.visible = false;
    scene3d.add(nearestMarker3d);

    const farthestGeometry = new THREE.SphereGeometry(0.075, 24, 16);
    const farthestMaterial = new THREE.MeshBasicMaterial({ color: 0xa855f7 });
    farthestMarker3d = new THREE.Mesh(farthestGeometry, farthestMaterial);
    farthestMarker3d.visible = false;
    scene3d.add(farthestMarker3d);

    const nearestLineGeometry = new THREE.BufferGeometry();
    nearestLineGeometry.setAttribute("position", new THREE.Float32BufferAttribute([0, 0, 0, 0, 0, 0], 3));
    const nearestLineMaterial = new THREE.LineBasicMaterial({
        color: 0xef4444,
        transparent: true,
        opacity: 0.90
    });
    nearestLine3d = new THREE.Line(nearestLineGeometry, nearestLineMaterial);
    nearestLine3d.visible = false;
    scene3d.add(nearestLine3d);

    const farthestLineGeometry = new THREE.BufferGeometry();
    farthestLineGeometry.setAttribute("position", new THREE.Float32BufferAttribute([0, 0, 0, 0, 0, 0], 3));
    const farthestLineMaterial = new THREE.LineBasicMaterial({
        color: 0xa855f7,
        transparent: true,
        opacity: 0.90
    });
    farthestLine3d = new THREE.Line(farthestLineGeometry, farthestLineMaterial);
    farthestLine3d.visible = false;
    scene3d.add(farthestLine3d);

    createEmptyPointCloud3D();

    document.getElementById("viewTopBtn").onclick = set3DTopView;
    document.getElementById("viewSideBtn").onclick = set3DSideView;
    document.getElementById("viewFrontBtn").onclick = set3DFrontView;
    document.getElementById("viewBackBtn").onclick = set3DBackView;
    document.getElementById("viewLeftBtn").onclick = set3DLeftView;
    document.getElementById("viewRightBtn").onclick = set3DRightView;
    document.getElementById("viewIsoBtn").onclick = set3DIsoView;
    document.getElementById("viewFirstBtn").onclick = set3DFirstPersonView;
    document.getElementById("viewNearBtn").onclick = set3DNearestView;
    document.getElementById("viewFarBtn").onclick = set3DFarthestView;
    document.getElementById("viewResetBtn").onclick = set3DResetView;

    document.getElementById("toggle3dBtn").onclick = function() {
        const card = document.getElementById("preview3dCard");
        preview3dVisible = !preview3dVisible;

        if (preview3dVisible) {
            card.classList.remove("hidden3d");
            this.textContent = "ocultar";
            resize3DPreview();
            update3DPointCloud(true);
        } else {
            card.classList.add("hidden3d");
            this.textContent = "mostrar";
        }
    };

    window.addEventListener("resize", resize3DPreview);
}

function resize3DPreview() {
    if (!renderer3d || !camera3d || !preview3dVisible) return;

    const container = document.getElementById("preview3dViewport");
    if (!container) return;

    const width = container.clientWidth;
    const height = container.clientHeight;

    if (width <= 0 || height <= 0) return;

    camera3d.aspect = width / height;
    camera3d.updateProjectionMatrix();
    renderer3d.setSize(width, height);
}

function createEmptyPointCloud3D() {
    const geometry = new THREE.BufferGeometry();

    geometry.setAttribute("position", new THREE.Float32BufferAttribute([], 3));
    geometry.setAttribute("color", new THREE.Float32BufferAttribute([], 3));

    const material = new THREE.PointsMaterial({
        size: Number(point3dSize.value),
        vertexColors: true,
        transparent: true,
        opacity: 0.95,
        sizeAttenuation: true
    });

    pointCloud3d = new THREE.Points(geometry, material);
    scene3d.add(pointCloud3d);
}


function getPoint3DPosition(p, zBase) {
    const rad = p.angle * Math.PI / 180.0;
    const distanceM = p.distance / 1000.0;

    const x = Math.cos(rad) * distanceM;
    const y = Math.sin(rad) * distanceM;

    let z = 0;

    if (zBase > 0) {
        const confidence = Math.max(0, Math.min(255, p.confidence || 0)) / 255.0;
        z = confidence * zBase;
    }

    return {
        x: x,
        y: y,
        z: z,
        distanceM: distanceM
    };
}

function setLine3D(line, p, zBase) {
    if (!line || !p) return;

    const pos = getPoint3DPosition(p, zBase);

    const linePositions = new Float32Array([
        0, 0, 0.03,
        pos.x, pos.y, pos.z + 0.055
    ]);

    line.geometry.setAttribute("position", new THREE.BufferAttribute(linePositions, 3));
    line.geometry.attributes.position.needsUpdate = true;
    line.visible = true;
}

function update3DExtremeMarkers(range, zBase) {
    const extremes = findExtremePoints(range);
    const nearest = extremes.nearest;
    const farthest = extremes.farthest;

    const nearestBadge = document.getElementById("preview3dNearestText");
    const farthestBadge = document.getElementById("preview3dFarthestText");

    if (nearest && nearestMarker3d && nearestLine3d) {
        const nearPos = getPoint3DPosition(nearest, zBase);

        nearestMarker3d.position.set(nearPos.x, nearPos.y, nearPos.z + 0.055);
        nearestMarker3d.visible = true;
        setLine3D(nearestLine3d, nearest, zBase);

        if (nearestBadge) {
            nearestBadge.innerHTML =
                formatDistance(nearest.distance) +
                " | ângulo " + Number(nearest.angle).toFixed(1) +
                "° | conf. " + (nearest.confidence !== undefined ? nearest.confidence : "-");
        }
    } else {
        if (nearestMarker3d) nearestMarker3d.visible = false;
        if (nearestLine3d) nearestLine3d.visible = false;
        if (nearestBadge) nearestBadge.textContent = "-";
    }

    if (farthest && farthestMarker3d && farthestLine3d) {
        const farPos = getPoint3DPosition(farthest, zBase);

        farthestMarker3d.position.set(farPos.x, farPos.y, farPos.z + 0.055);
        farthestMarker3d.visible = true;
        setLine3D(farthestLine3d, farthest, zBase);

        if (farthestBadge) {
            farthestBadge.innerHTML =
                formatDistance(farthest.distance) +
                " | ângulo " + Number(farthest.angle).toFixed(1) +
                "° | conf. " + (farthest.confidence !== undefined ? farthest.confidence : "-");
        }
    } else {
        if (farthestMarker3d) farthestMarker3d.visible = false;
        if (farthestLine3d) farthestLine3d.visible = false;
        if (farthestBadge) farthestBadge.textContent = "-";
    }
}

function lookAt3D(x, y, z) {
    if (!camera3d) return;

    if (controls3d) {
        controls3d.target.set(x, y, z);
        controls3d.update();
    }

    camera3d.lookAt(x, y, z);
}

function update3DPointCloud(forceUpdate) {
    if (!pointCloud3d || !preview3dVisible) return;

    const now = performance.now();

    if (!forceUpdate && now - last3dUpdate < 120) {
        return;
    }

    last3dUpdate = now;

    const range = Number(maxRange.value);
    const points = getVisiblePoints(range);
    const zBase = Number(zLift.value);

    const positions = [];
    const colors = [];

    for (const p of points) {
        const pos = getPoint3DPosition(p, zBase);
        const color = new THREE.Color(getDistanceSolidColor(p.distance));

        positions.push(pos.x, pos.y, pos.z);
        colors.push(color.r, color.g, color.b);
    }

    pointCloud3d.geometry.dispose();

    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute("position", new THREE.Float32BufferAttribute(positions, 3));
    geometry.setAttribute("color", new THREE.Float32BufferAttribute(colors, 3));

    if (positions.length > 0) {
        geometry.computeBoundingSphere();
    }

    pointCloud3d.geometry = geometry;
    pointCloud3d.material.size = Number(point3dSize.value);
    pointCloud3d.material.needsUpdate = true;

    update3DExtremeMarkers(range, zBase);
}

function animate3DPreview() {
    requestAnimationFrame(animate3DPreview);

    if (!renderer3d || !scene3d || !camera3d || !preview3dVisible) return;

    update3DPointCloud(false);

    if (controls3d) {
        controls3d.update();
    }

    renderer3d.render(scene3d, camera3d);
}

function set3DTopView() {
    if (!camera3d) return;

    camera3d.up.set(0, 1, 0);
    camera3d.position.set(0, 0, 10);
    lookAt3D(0, 0, 0);
}

function set3DSideView() {
    if (!camera3d) return;

    camera3d.up.set(0, 0, 1);
    camera3d.position.set(0, -8, 2.2);
    lookAt3D(0, 0, 0.2);
}

function set3DFrontView() {
    if (!camera3d) return;

    camera3d.up.set(0, 0, 1);
    camera3d.position.set(0, -8, 1.2);
    lookAt3D(0, 0, 0.2);
}

function set3DBackView() {
    if (!camera3d) return;

    camera3d.up.set(0, 0, 1);
    camera3d.position.set(0, 8, 1.2);
    lookAt3D(0, 0, 0.2);
}

function set3DLeftView() {
    if (!camera3d) return;

    camera3d.up.set(0, 0, 1);
    camera3d.position.set(-8, 0, 1.2);
    lookAt3D(0, 0, 0.2);
}

function set3DRightView() {
    if (!camera3d) return;

    camera3d.up.set(0, 0, 1);
    camera3d.position.set(8, 0, 1.2);
    lookAt3D(0, 0, 0.2);
}

function set3DIsoView() {
    if (!camera3d) return;

    camera3d.up.set(0, 0, 1);
    camera3d.position.set(6, -6, 4.5);
    lookAt3D(0, 0, 0.2);
}

function set3DFirstPersonView() {
    if (!camera3d) return;

    const range = Number(maxRange.value);
    const farthest = findExtremePoints(range).farthest;

    camera3d.up.set(0, 0, 1);
    camera3d.position.set(0, -0.35, 0.28);

    if (farthest) {
        const pos = getPoint3DPosition(farthest, Number(zLift.value));
        lookAt3D(pos.x, pos.y, Math.max(0.12, pos.z));
    } else {
        lookAt3D(0, 5, 0.15);
    }
}

function set3DNearestView() {
    if (!camera3d) return;

    const range = Number(maxRange.value);
    const nearest = findExtremePoints(range).nearest;

    if (!nearest) {
        set3DResetView();
        return;
    }

    const pos = getPoint3DPosition(nearest, Number(zLift.value));
    const angle = Math.atan2(pos.y, pos.x);

    const backDistance = 1.25;
    const camX = pos.x - Math.cos(angle) * backDistance;
    const camY = pos.y - Math.sin(angle) * backDistance;
    const camZ = Math.max(0.45, pos.z + 0.65);

    camera3d.up.set(0, 0, 1);
    camera3d.position.set(camX, camY, camZ);
    lookAt3D(pos.x, pos.y, pos.z + 0.08);
}

function set3DFarthestView() {
    if (!camera3d) return;

    const range = Number(maxRange.value);
    const farthest = findExtremePoints(range).farthest;

    if (!farthest) {
        set3DResetView();
        return;
    }

    const pos = getPoint3DPosition(farthest, Number(zLift.value));
    const angle = Math.atan2(pos.y, pos.x);

    const backDistance = 2.2;
    const camX = pos.x - Math.cos(angle) * backDistance;
    const camY = pos.y - Math.sin(angle) * backDistance;
    const camZ = Math.max(0.8, pos.z + 0.9);

    camera3d.up.set(0, 0, 1);
    camera3d.position.set(camX, camY, camZ);
    lookAt3D(pos.x, pos.y, pos.z + 0.1);
}

function set3DResetView() {
    if (!camera3d) return;

    camera3d.up.set(0, 0, 1);
    camera3d.position.set(0, -7, 4.5);
    lookAt3D(0, 0, 0);
}

function drawRoundedRect(x, y, width, height, radius) {
    ctx.beginPath();
    ctx.moveTo(x + radius, y);
    ctx.lineTo(x + width - radius, y);
    ctx.quadraticCurveTo(x + width, y, x + width, y + radius);
    ctx.lineTo(x + width, y + height - radius);
    ctx.quadraticCurveTo(x + width, y + height, x + width - radius, y + height);
    ctx.lineTo(x + radius, y + height);
    ctx.quadraticCurveTo(x, y + height, x, y + height - radius);
    ctx.lineTo(x, y + radius);
    ctx.quadraticCurveTo(x, y, x + radius, y);
    ctx.closePath();
}

function drawBackground(w, h) {
    const cx = w / 2;
    const cy = h / 2;

    const grad = ctx.createRadialGradient(cx, cy, 20, cx, cy, Math.max(w, h));
    grad.addColorStop(0, "#111827");
    grad.addColorStop(0.42, "#07111f");
    grad.addColorStop(1, "#020617");

    ctx.fillStyle = grad;
    ctx.fillRect(0, 0, w, h);
}

function drawRangeBands(cx, cy, scale, range) {
    const bands = [
        { max: 500, color: "rgba(239, 68, 68, 0.040)" },
        { max: 1000, color: "rgba(249, 115, 22, 0.035)" },
        { max: 2000, color: "rgba(250, 204, 21, 0.030)" },
        { max: 4000, color: "rgba(34, 197, 94, 0.024)" },
        { max: 6000, color: "rgba(6, 182, 212, 0.021)" },
        { max: 8000, color: "rgba(59, 130, 246, 0.018)" },
        { max: 12000, color: "rgba(168, 85, 247, 0.016)" }
    ];

    ctx.save();

    let previous = 0;

    for (const band of bands) {
        const outer = Math.min(band.max, range);
        if (outer <= previous) continue;

        ctx.beginPath();
        ctx.arc(cx, cy, outer * scale, 0, Math.PI * 2);
        ctx.arc(cx, cy, previous * scale, 0, Math.PI * 2, true);
        ctx.closePath();
        ctx.fillStyle = band.color;
        ctx.fill();

        previous = outer;

        if (previous >= range) break;
    }

    ctx.restore();
}

function drawGrid(cx, cy, scale, range) {
    ctx.save();

    ctx.lineWidth = 1;

    for (let r = 500; r <= range; r += 500) {
        const isMeter = r % 1000 === 0;

        ctx.beginPath();
        ctx.arc(cx, cy, r * scale, 0, Math.PI * 2);

        if (isMeter) {
            ctx.strokeStyle = "rgba(148, 163, 184, 0.32)";
            ctx.lineWidth = 1.2;
        } else {
            ctx.strokeStyle = "rgba(148, 163, 184, 0.13)";
            ctx.lineWidth = 1;
        }

        ctx.stroke();

        if (isMeter) {
            ctx.fillStyle = "rgba(226, 232, 240, 0.85)";
            ctx.font = "12px Arial";
            ctx.textAlign = "left";
            ctx.textBaseline = "alphabetic";
            ctx.fillText((r / 1000).toFixed(0) + "m", cx + r * scale + 6, cy - 5);
        }
    }

    for (let a = 0; a < 360; a += 15) {
        const rad = (a - 90) * Math.PI / 180;
        const x = Math.cos(rad) * range * scale;
        const y = Math.sin(rad) * range * scale;

        ctx.beginPath();
        ctx.moveTo(cx, cy);
        ctx.lineTo(cx + x, cy + y);

        if (a % 45 === 0) {
            ctx.strokeStyle = "rgba(148, 163, 184, 0.30)";
        } else {
            ctx.strokeStyle = "rgba(148, 163, 184, 0.10)";
        }

        ctx.stroke();

        if (a % 45 === 0) {
            const tx = cx + Math.cos(rad) * (range * scale + 22);
            const ty = cy + Math.sin(rad) * (range * scale + 22);

            ctx.fillStyle = "rgba(203, 213, 225, 0.82)";
            ctx.font = "12px Arial";
            ctx.textAlign = "center";
            ctx.textBaseline = "middle";
            ctx.fillText(a + "°", tx, ty);
        }
    }

    ctx.strokeStyle = "rgba(56, 189, 248, 0.95)";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.lineTo(cx, cy - range * scale);
    ctx.stroke();

    ctx.fillStyle = "#38bdf8";
    ctx.font = "13px Arial";
    ctx.textAlign = "left";
    ctx.fillText("Frente / 0°", cx + 8, cy - range * scale + 18);

    ctx.restore();
}

function drawSweep(cx, cy, range, scale, intensity) {
    const rad = (sweepAngle - 90) * Math.PI / 180;
    const x = Math.cos(rad) * range * scale;
    const y = Math.sin(rad) * range * scale;

    const gradient = ctx.createRadialGradient(cx, cy, 0, cx, cy, range * scale);
    gradient.addColorStop(0, "rgba(56, 189, 248, " + (0.35 + intensity) + ")");
    gradient.addColorStop(1, "rgba(56, 189, 248, 0.00)");

    ctx.save();

    ctx.strokeStyle = "rgba(56, 189, 248, 0.70)";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.lineTo(cx + x, cy + y);
    ctx.stroke();

    ctx.fillStyle = gradient;
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.arc(cx, cy, range * scale, rad - 0.12, rad);
    ctx.closePath();
    ctx.fill();

    ctx.restore();

    sweepAngle += 1.7;
    if (sweepAngle >= 360) sweepAngle = 0;
}

function drawPoints(cx, cy, scale, range, offset, psize, intensity) {
    for (const p of currentPoints) {
        if (!p) continue;
        if (p.distance <= 0 || p.distance > range) continue;

        const pos = pointToCanvas(p, cx, cy, scale, offset);

        const conf = Math.max(0, Math.min(255, p.confidence || 0));
        const alpha = 0.45 + (conf / 255.0) * 0.55;
        const rgb = getDistanceColor(p.distance);

        ctx.shadowColor = "rgba(" + rgb + ", 0.9)";
        ctx.shadowBlur = 6 + intensity * 24;

        ctx.fillStyle = "rgba(" + rgb + ", " + alpha + ")";
        ctx.beginPath();
        ctx.arc(pos.x, pos.y, psize, 0, Math.PI * 2);
        ctx.fill();

        ctx.shadowBlur = 0;
    }
}

function drawObjectLabels(cx, cy, scale, offset, range) {
    if (!showObjects.checked) {
        lastDetectedObjects = [];
        document.getElementById("objectsCount").textContent = "0";
        return;
    }

    const objects = detectObjects(range);
    lastDetectedObjects = objects;

    document.getElementById("objectsCount").textContent = objects.length;

    ctx.save();

    for (let i = 0; i < objects.length; i++) {
        const obj = objects[i];

        const pos = pointToCanvas(obj, cx, cy, scale, offset);
        const color = getDistanceSolidColor(obj.minDistance);

        const label = "#" + (i + 1) + "  " + formatDistance(obj.minDistance);
        const detail = obj.points + " pts  " + Number(obj.angle).toFixed(1) + "°";

        if (showObjectLines.checked) {
            ctx.strokeStyle = color;
            ctx.globalAlpha = 0.40;
            ctx.lineWidth = 1;
            ctx.setLineDash([4, 7]);

            ctx.beginPath();
            ctx.moveTo(cx, cy);
            ctx.lineTo(pos.x, pos.y);
            ctx.stroke();

            ctx.setLineDash([]);
            ctx.globalAlpha = 1;
        }

        ctx.strokeStyle = color;
        ctx.fillStyle = color;
        ctx.lineWidth = 1.5;

        ctx.shadowColor = color;
        ctx.shadowBlur = 12;

        ctx.beginPath();
        ctx.arc(pos.x, pos.y, 8, 0, Math.PI * 2);
        ctx.stroke();

        ctx.beginPath();
        ctx.arc(pos.x, pos.y, 3, 0, Math.PI * 2);
        ctx.fill();

        ctx.shadowBlur = 0;

        let labelX = pos.x + 12;
        let labelY = pos.y - 12;

        if (labelX > canvas.width - 150) {
            labelX = pos.x - 150;
        }

        if (labelY < 24) {
            labelY = pos.y + 30;
        }

        ctx.font = "bold 11px Arial";
        const w1 = ctx.measureText(label).width;

        ctx.font = "10px Arial";
        const w2 = ctx.measureText(detail).width;

        const boxW = Math.max(w1, w2) + 16;
        const boxH = 34;

        ctx.fillStyle = "rgba(2, 6, 23, 0.84)";
        ctx.strokeStyle = color;
        ctx.lineWidth = 1;

        drawRoundedRect(labelX, labelY - 20, boxW, boxH, 8);
        ctx.fill();
        ctx.stroke();

        ctx.fillStyle = color;
        ctx.font = "bold 11px Arial";
        ctx.fillText(label, labelX + 8, labelY - 6);

        ctx.fillStyle = "#cbd5e1";
        ctx.font = "10px Arial";
        ctx.fillText(detail, labelX + 8, labelY + 8);
    }

    ctx.restore();
}

function drawExtremeMarker(cx, cy, scale, offset, point, type) {
    if (!point) return;

    const pos = pointToCanvas(point, cx, cy, scale, offset);

    let color;
    let label;
    let radius;

    if (type === "near") {
        color = "#ef4444";
        label = "MAIS PRÓXIMO";
        radius = 13;
    } else {
        color = "#a855f7";
        label = "MAIS LONGE";
        radius = 11;
    }

    ctx.save();

    ctx.strokeStyle = color;
    ctx.fillStyle = color;
    ctx.lineWidth = 2;

    ctx.setLineDash([7, 6]);
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.lineTo(pos.x, pos.y);
    ctx.stroke();
    ctx.setLineDash([]);

    ctx.shadowColor = color;
    ctx.shadowBlur = 20;

    ctx.beginPath();
    ctx.arc(pos.x, pos.y, radius, 0, Math.PI * 2);
    ctx.stroke();

    ctx.beginPath();
    ctx.arc(pos.x, pos.y, 4.5, 0, Math.PI * 2);
    ctx.fill();

    ctx.shadowBlur = 0;

    ctx.strokeStyle = color;
    ctx.lineWidth = 1.5;

    ctx.beginPath();
    ctx.arc(pos.x, pos.y, radius + 8, 0, Math.PI * 2);
    ctx.stroke();

    ctx.beginPath();
    ctx.moveTo(pos.x - radius - 12, pos.y);
    ctx.lineTo(pos.x + radius + 12, pos.y);
    ctx.moveTo(pos.x, pos.y - radius - 12);
    ctx.lineTo(pos.x, pos.y + radius + 12);
    ctx.stroke();

    const text = label + "  " + formatDistance(point.distance);
    const detail = "ângulo " + Number(point.angle).toFixed(1) + "°  conf. " + (point.confidence !== undefined ? point.confidence : "-");

    let labelX = pos.x + 16;
    let labelY = pos.y - 20;

    if (labelX > canvas.width - 245) {
        labelX = pos.x - 245;
    }

    if (labelY < 35) {
        labelY = pos.y + 38;
    }

    ctx.font = "bold 12px Arial";
    const width1 = ctx.measureText(text).width;
    ctx.font = "11px Arial";
    const width2 = ctx.measureText(detail).width;
    const boxWidth = Math.max(width1, width2) + 22;

    ctx.fillStyle = "rgba(2, 6, 23, 0.88)";
    ctx.strokeStyle = color;
    ctx.lineWidth = 1;

    drawRoundedRect(labelX, labelY - 22, boxWidth, 42, 9);
    ctx.fill();
    ctx.stroke();

    ctx.fillStyle = color;
    ctx.font = "bold 12px Arial";
    ctx.fillText(text, labelX + 10, labelY - 5);

    ctx.fillStyle = "#cbd5e1";
    ctx.font = "11px Arial";
    ctx.fillText(detail, labelX + 10, labelY + 11);

    ctx.restore();
}

function drawExtremeInfo(cx, cy, scale, offset, range) {
    const extremes = findExtremePoints(range);

    drawExtremeMarker(cx, cy, scale, offset, extremes.farthest, "far");
    drawExtremeMarker(cx, cy, scale, offset, extremes.nearest, "near");
}

function drawCenter(cx, cy) {
    ctx.save();

    ctx.shadowColor = "rgba(248, 250, 252, 0.9)";
    ctx.shadowBlur = 12;

    ctx.fillStyle = "#f8fafc";
    ctx.beginPath();
    ctx.arc(cx, cy, 5, 0, Math.PI * 2);
    ctx.fill();

    ctx.shadowBlur = 0;

    ctx.strokeStyle = "rgba(248, 250, 252, 0.5)";
    ctx.beginPath();
    ctx.arc(cx, cy, 12, 0, Math.PI * 2);
    ctx.stroke();

    ctx.fillStyle = "rgba(248, 250, 252, 0.85)";
    ctx.font = "12px Arial";
    ctx.textAlign = "center";
    ctx.fillText("LiDAR", cx, cy + 28);

    ctx.restore();
}

function drawInfoOverlay(cx, cy, range, scale) {
    ctx.save();

    const text = "Raio exibido: " + formatDistance(range);

    ctx.font = "13px Arial";
    const w = ctx.measureText(text).width + 22;

    ctx.fillStyle = "rgba(2, 6, 23, 0.72)";
    ctx.strokeStyle = "rgba(56, 189, 248, 0.35)";
    ctx.lineWidth = 1;

    drawRoundedRect(cx - w / 2, 18, w, 30, 10);
    ctx.fill();
    ctx.stroke();

    ctx.fillStyle = "#7dd3fc";
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.fillText(text, cx, 33);

    ctx.restore();
}

function drawRuler(cx, cy, scale, range) {
    if (!showRuler.checked) return;

    const candidates = [
        100, 200, 500,
        1000, 2000, 5000,
        10000
    ];

    const targetPx = 160;
    const targetMm = targetPx / scale;

    let selected = candidates[0];

    for (const c of candidates) {
        if (c <= targetMm) {
            selected = c;
        }
    }

    if (selected > range) {
        selected = range;
    }

    const rulerPx = selected * scale;

    const margin = 34;
    const x1 = canvas.width - rulerPx - margin;
    const y = canvas.height - 48;
    const x2 = canvas.width - margin;

    ctx.save();

    ctx.lineWidth = 3;
    ctx.strokeStyle = "rgba(248, 250, 252, 0.95)";
    ctx.beginPath();
    ctx.moveTo(x1, y);
    ctx.lineTo(x2, y);
    ctx.stroke();

    ctx.lineWidth = 2;

    ctx.beginPath();
    ctx.moveTo(x1, y - 10);
    ctx.lineTo(x1, y + 10);
    ctx.moveTo(x2, y - 10);
    ctx.lineTo(x2, y + 10);
    ctx.stroke();

    const parts = 4;

    for (let i = 1; i < parts; i++) {
        const x = x1 + (rulerPx / parts) * i;

        ctx.beginPath();
        ctx.moveTo(x, y - 6);
        ctx.lineTo(x, y + 6);
        ctx.stroke();
    }

    let label;

    if (selected < 1000) {
        label = selected + " mm";
    } else {
        label = (selected / 1000).toFixed(0) + " m";
    }

    ctx.fillStyle = "rgba(2, 6, 23, 0.82)";
    ctx.strokeStyle = "rgba(148, 163, 184, 0.45)";
    ctx.lineWidth = 1;

    const text = "Régua: " + label;
    ctx.font = "bold 13px Arial";
    const boxWidth = ctx.measureText(text).width + 22;

    drawRoundedRect(x1 + rulerPx / 2 - boxWidth / 2, y - 42, boxWidth, 26, 9);
    ctx.fill();
    ctx.stroke();

    ctx.fillStyle = "#f8fafc";
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.fillText(text, x1 + rulerPx / 2, y - 29);

    ctx.fillStyle = "rgba(203, 213, 225, 0.85)";
    ctx.font = "11px Arial";
    ctx.fillText("escala visual", x1 + rulerPx / 2, y + 24);

    ctx.restore();
}

function draw() {
    const w = canvas.width;
    const h = canvas.height;

    const cx = w / 2;
    const cy = h / 2;

    const range = Number(maxRange.value);
    const offset = Number(angleOffset.value);
    const psize = Number(pointSize.value);
    const intensity = Number(trailAlpha.value);

    ctx.clearRect(0, 0, w, h);

    drawBackground(w, h);

    const scale = Math.min(w, h) * 0.43 / range;

    drawRangeBands(cx, cy, scale, range);
    drawGrid(cx, cy, scale, range);
    drawSweep(cx, cy, range, scale, intensity);
    drawPoints(cx, cy, scale, range, offset, psize, intensity);
    drawObjectLabels(cx, cy, scale, offset, range);
    drawExtremeInfo(cx, cy, scale, offset, range);
    drawCenter(cx, cy);
    drawInfoOverlay(cx, cy, range, scale);
    drawRuler(cx, cy, scale, range);
    updateExtremePanel(range);

    requestAnimationFrame(draw);
}

async function updateData() {
    try {
        const res = await fetch("/api/points", { cache: "no-store" });
        const data = await res.json();

        currentPoints = data.points || [];
        update3DPointCloud(true);

        document.getElementById("bytesReceived").textContent = data.status.bytes_received;
        document.getElementById("framesOk").textContent = data.status.frames_ok;
        document.getElementById("framesBad").textContent = data.status.frames_bad;
        document.getElementById("pointsCount").textContent = currentPoints.length;
        document.getElementById("scanHz").textContent = Number(data.status.scan_hz).toFixed(2);
        document.getElementById("lastAge").textContent = data.status.last_age_ms;
        document.getElementById("lastHex").textContent = data.status.last_hex || "-";

        const serialStatus = document.getElementById("serialStatus");
        const conn = document.getElementById("conn");

        if (data.status.connected) {
            serialStatus.innerHTML = '<span class="ok">conectada</span>';
            conn.innerHTML = '<span class="ok">● serial conectada</span>';
        } else {
            serialStatus.innerHTML = '<span class="bad">desconectada</span>';
            conn.innerHTML = '<span class="bad">● sem serial</span>';
        }

    } catch (e) {
        document.getElementById("conn").innerHTML = '<span class="bad">● erro API</span>';
    }

    setTimeout(updateData, 60);
}

setupExportButtons();
init3DPreview();
animate3DPreview();
draw();
updateData();
</script>
</body>
</html>
"""


class LidarState:
    def __init__(self):
        self.lock = threading.Lock()
        self.points_by_bin = [None] * 720
        self.connected = False
        self.bytes_received = 0
        self.frames_ok = 0
        self.frames_bad = 0
        self.last_packet_time = 0
        self.last_hex = ""
        self.scan_hz = 0.0
        self.error = ""

    def add_bytes(self, count):
        with self.lock:
            self.bytes_received += count

    def add_bad(self):
        with self.lock:
            self.frames_bad += 1

    def set_connected(self, value, error=""):
        with self.lock:
            self.connected = value
            self.error = error

    def update_frame(self, frame_data, frame_hex):
        with self.lock:
            for p in frame_data["points"]:
                angle = p["angle"] % 360.0
                idx = int(round(angle * 2.0)) % 720
                self.points_by_bin[idx] = p

            self.frames_ok += 1
            self.last_packet_time = time.time()
            self.last_hex = frame_hex
            self.scan_hz = frame_data["scan_hz"]
            self.connected = True
            self.error = ""

    def snapshot(self):
        with self.lock:
            now = time.time()

            if self.last_packet_time > 0:
                age = int((now - self.last_packet_time) * 1000)
            else:
                age = -1

            connected = self.connected
            if age > 2500:
                connected = False

            points = [p for p in self.points_by_bin if p is not None]

            return {
                "points": points,
                "status": {
                    "connected": connected,
                    "bytes_received": self.bytes_received,
                    "frames_ok": self.frames_ok,
                    "frames_bad": self.frames_bad,
                    "last_age_ms": age,
                    "last_hex": self.last_hex,
                    "scan_hz": self.scan_hz,
                    "error": self.error,
                }
            }


def u16le(data, idx):
    return data[idx] | (data[idx + 1] << 8)


def parse_frame(frame):
    if len(frame) != 47:
        return None

    if frame[0] != 0x54:
        return None

    point_count = frame[1] & 0x1F

    if point_count != 12:
        return None

    speed = u16le(frame, 2)
    start_angle_raw = u16le(frame, 4)
    end_angle_raw = u16le(frame, 42)

    start_angle = start_angle_raw / 100.0
    end_angle = end_angle_raw / 100.0

    diff = end_angle - start_angle
    if diff < 0:
        diff += 360.0

    points = []

    for i in range(point_count):
        base = 6 + i * 3

        distance = u16le(frame, base)
        confidence = frame[base + 2]

        if point_count > 1:
            angle = start_angle + diff * i / (point_count - 1)
        else:
            angle = start_angle

        angle = angle % 360.0

        if 0 < distance < 20000:
            points.append({
                "angle": round(angle, 2),
                "distance": int(distance),
                "confidence": int(confidence),
            })

    return {
        "scan_hz": speed / 360.0 if speed > 0 else 0.0,
        "points": points
    }


def serial_worker(state, port, baud):
    buffer = bytearray()

    while True:
        try:
            print(f"Lendo dados de {port} em {baud} baud")

            ser = serial.Serial(
                port=port,
                baudrate=baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.2
            )

            state.set_connected(True)

            while True:
                chunk = ser.read(512)

                if not chunk:
                    continue

                state.add_bytes(len(chunk))
                buffer.extend(chunk)

                while len(buffer) >= 47:
                    try:
                        start = buffer.index(0x54)
                    except ValueError:
                        buffer.clear()
                        break

                    if start > 0:
                        del buffer[:start]

                    if len(buffer) < 47:
                        break

                    frame = bytes(buffer[:47])
                    del buffer[:47]

                    parsed = parse_frame(frame)

                    if parsed is None or len(parsed["points"]) == 0:
                        state.add_bad()
                        continue

                    state.update_frame(parsed, frame.hex(" "))

        except Exception as e:
            state.set_connected(False, str(e))
            print("Erro serial:", e)
            time.sleep(2)


def create_app(args):
    app = Flask(__name__)
    state = LidarState()

    th = threading.Thread(
        target=serial_worker,
        args=(state, args.port, args.baud),
        daemon=True
    )
    th.start()

    @app.route("/")
    def index():
        return render_template_string(HTML)

    @app.route("/api/points")
    def api_points():
        return jsonify(state.snapshot())

    return app


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=230400)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--flask-port", type=int, default=5010)
    args = parser.parse_args()

    app = create_app(args)

    print("Servidor Flask iniciado")
    print(f"Acesse: http://127.0.0.1:{args.flask_port}")
    print(f"Host: {args.host}")
    print(f"Porta serial: {args.port}")
    print(f"Baud: {args.baud}")

    app.run(
        host=args.host,
        port=args.flask_port,
        debug=False,
        threaded=True,
        use_reloader=False
    )


if __name__ == "__main__":
    main()

