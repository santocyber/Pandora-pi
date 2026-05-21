#!/usr/bin/env python3
import os
import re
import glob
import signal
import subprocess
from flask import Flask, request, jsonify, Response, render_template_string

app = Flask(__name__)

DEFAULT_INTERFACE = "can0"

SLCAN_SPEEDS = {
    "10000": "0",
    "20000": "1",
    "50000": "2",
    "100000": "3",
    "125000": "4",
    "250000": "5",
    "500000": "6",
    "750000": "7",
    "1000000": "8",
}

BITRATE_LABELS = {
    "10000": "10 kbit/s",
    "20000": "20 kbit/s",
    "50000": "50 kbit/s",
    "100000": "100 kbit/s",
    "125000": "125 kbit/s",
    "250000": "250 kbit/s",
    "500000": "500 kbit/s",
    "750000": "750 kbit/s",
    "1000000": "1 Mbit/s",
}

candump_process = None


HTML = r"""
<!doctype html>
<html lang="pt-BR">
<head>
    <meta charset="utf-8">
    <title>Painel CAN - CANable / SocketCAN</title>
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

        .card {
            background: #161b22;
            border: 1px solid #30363d;
            border-radius: 14px;
        }

        .form-control,
        .form-select {
            background: #0d1117;
            color: #e6edf3;
            border: 1px solid #30363d;
        }

        .form-control:focus,
        .form-select:focus {
            background: #0d1117;
            color: #e6edf3;
            border-color: #58a6ff;
            box-shadow: none;
        }

        .badge-status {
            font-size: 0.95rem;
            padding: 0.6rem 0.8rem;
        }

        pre {
            background: #010409;
            color: #7ee787;
            padding: 1rem;
            border-radius: 12px;
            height: 330px;
            overflow-y: auto;
            border: 1px solid #30363d;
            font-size: 0.9rem;
        }

        .small-muted {
            color: #8b949e;
            font-size: 0.9rem;
        }

        .btn {
            border-radius: 10px;
        }

        .table {
            --bs-table-bg: #161b22;
            --bs-table-color: #e6edf3;
            --bs-table-border-color: #30363d;
        }

        code {
            color: #ffa657;
        }
    </style>
</head>
<body>

<div class="container py-4">
    <div class="d-flex flex-wrap align-items-center justify-content-between mb-4">
        <div>
            <h1 class="mb-1">Painel CAN</h1>
            <div class="small-muted">
                Interface visual para CANable / SLCAN / SocketCAN no Ubuntu
            </div>
        </div>

        <div class="mt-3 mt-md-0">
            <span id="canStatusBadge" class="badge bg-secondary badge-status">verificando...</span>
        </div>
    </div>

    <div class="row g-4">

        <div class="col-lg-5">
            <div class="card p-3 h-100">
                <h4>Configuração SLCAN</h4>

                <div class="mb-3">
                    <label class="form-label">Dispositivo serial</label>
                    <div class="input-group">
                        <select id="serialPort" class="form-select"></select>
                        <button class="btn btn-outline-info" onclick="loadDevices()">Atualizar</button>
                    </div>
                    <div class="small-muted mt-1">
                        Para seu CANable deve aparecer algo como <code>/dev/serial/by-id/usb-Openlight...</code> ou <code>/dev/ttyACM0</code>.
                    </div>
                </div>

                <div class="mb-3">
                    <label class="form-label">Interface CAN</label>
                    <input id="interfaceName" class="form-control" value="can0">
                </div>

                <div class="mb-3">
                    <label class="form-label">Bitrate</label>
                    <select id="bitrate" class="form-select">
                        <option value="125000">125 kbit/s</option>
                        <option value="250000">250 kbit/s</option>
                        <option value="500000" selected>500 kbit/s</option>
                        <option value="750000">750 kbit/s</option>
                        <option value="1000000">1 Mbit/s</option>
                    </select>
                </div>

                <div class="d-grid gap-2">
                    <button class="btn btn-success" onclick="startSlcan()">Subir CAN via SLCAN</button>
                    <button class="btn btn-warning" onclick="stopCan()">Parar CAN</button>
                    <button class="btn btn-primary" onclick="refreshStatus()">Atualizar status</button>
                </div>

                <hr>

                <h5>Status detalhado</h5>
                <pre id="statusBox">Carregando...</pre>
            </div>
        </div>

        <div class="col-lg-7">
            <div class="card p-3 mb-4">
                <h4>Enviar frame CAN</h4>

                <div class="row g-3">
                    <div class="col-md-3">
                        <label class="form-label">ID HEX</label>
                        <input id="sendId" class="form-control" value="123" maxlength="8">
                    </div>

                    <div class="col-md-7">
                        <label class="form-label">Dados HEX</label>
                        <input id="sendData" class="form-control" value="DEADBEEF">
                    </div>

                    <div class="col-md-2 d-grid">
                        <label class="form-label">&nbsp;</label>
                        <button class="btn btn-success" onclick="sendFrame()">Enviar</button>
                    </div>
                </div>

                <div class="small-muted mt-2">
                    Exemplo: ID <code>123</code>, dados <code>DEADBEEF</code> envia <code>123#DEADBEEF</code>.
                </div>
            </div>

            <div class="card p-3">
                <div class="d-flex flex-wrap align-items-center justify-content-between mb-2">
                    <h4 class="mb-2">Monitor CAN</h4>

                    <div>
                        <button class="btn btn-outline-success btn-sm" onclick="startMonitor()">Iniciar candump</button>
                        <button class="btn btn-outline-danger btn-sm" onclick="stopMonitor()">Parar</button>
                        <button class="btn btn-outline-secondary btn-sm" onclick="clearMonitor()">Limpar</button>
                    </div>
                </div>

                <pre id="monitorBox"></pre>
            </div>
        </div>

        <div class="col-12">
            <div class="card p-3">
                <h4>Comandos úteis</h4>

                <table class="table table-bordered table-sm align-middle">
                    <thead>
                        <tr>
                            <th>Ação</th>
                            <th>Comando equivalente</th>
                        </tr>
                    </thead>
                    <tbody>
                        <tr>
                            <td>Ver interfaces</td>
                            <td><code>ip -br link</code></td>
                        </tr>
                        <tr>
                            <td>Ver detalhes do CAN</td>
                            <td><code>ip -details -statistics link show can0</code></td>
                        </tr>
                        <tr>
                            <td>Monitorar frames</td>
                            <td><code>candump -tz can0</code></td>
                        </tr>
                        <tr>
                            <td>Enviar frame</td>
                            <td><code>cansend can0 123#DEADBEEF</code></td>
                        </tr>
                        <tr>
                            <td>Parar SLCAN manualmente</td>
                            <td><code>sudo pkill slcand && sudo ip link set can0 down</code></td>
                        </tr>
                    </tbody>
                </table>

                <div class="small-muted">
                    Rode este painel apenas em rede confiável. Ele executa comandos do sistema para configurar CAN.
                </div>
            </div>
        </div>

    </div>
</div>

<script>
let eventSource = null;

function appendMonitor(text) {
    const box = document.getElementById("monitorBox");
    box.textContent += text;
    box.scrollTop = box.scrollHeight;
}

function clearMonitor() {
    document.getElementById("monitorBox").textContent = "";
}

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

async function loadDevices() {
    const result = await apiGet("/api/devices");
    const select = document.getElementById("serialPort");

    select.innerHTML = "";

    if (!result.ok || result.devices.length === 0) {
        const option = document.createElement("option");
        option.value = "";
        option.textContent = "Nenhum dispositivo serial encontrado";
        select.appendChild(option);
        return;
    }

    for (const dev of result.devices) {
        const option = document.createElement("option");
        option.value = dev;
        option.textContent = dev;
        select.appendChild(option);
    }
}

async function refreshStatus() {
    const iface = document.getElementById("interfaceName").value || "can0";
    const result = await apiGet("/api/status?iface=" + encodeURIComponent(iface));

    document.getElementById("statusBox").textContent = result.output || result.error || "Sem saída";

    const badge = document.getElementById("canStatusBadge");

    if (result.up) {
        badge.className = "badge bg-success badge-status";
        badge.textContent = iface + " UP";
    } else if (result.exists) {
        badge.className = "badge bg-warning text-dark badge-status";
        badge.textContent = iface + " existe, mas está DOWN";
    } else {
        badge.className = "badge bg-secondary badge-status";
        badge.textContent = iface + " não existe";
    }
}

async function startSlcan() {
    const port = document.getElementById("serialPort").value;
    const iface = document.getElementById("interfaceName").value || "can0";
    const bitrate = document.getElementById("bitrate").value;

    const result = await apiPost("/api/start_slcan", {
        port: port,
        iface: iface,
        bitrate: bitrate
    });

    alert(result.message || result.error);
    await refreshStatus();
}

async function stopCan() {
    const iface = document.getElementById("interfaceName").value || "can0";

    const result = await apiPost("/api/stop", {
        iface: iface
    });

    alert(result.message || result.error);
    await refreshStatus();
}

async function sendFrame() {
    const iface = document.getElementById("interfaceName").value || "can0";
    const canId = document.getElementById("sendId").value.trim();
    const data = document.getElementById("sendData").value.trim();

    const result = await apiPost("/api/send", {
        iface: iface,
        can_id: canId,
        data: data
    });

    if (result.ok) {
        appendMonitor("[TX] " + result.frame + "\n");
    } else {
        alert(result.error || "Erro ao enviar frame");
    }
}

function startMonitor() {
    const iface = document.getElementById("interfaceName").value || "can0";

    if (eventSource) {
        eventSource.close();
        eventSource = null;
    }

    appendMonitor("=== Iniciando candump em " + iface + " ===\n");

    eventSource = new EventSource("/api/monitor?iface=" + encodeURIComponent(iface));

    eventSource.onmessage = function(event) {
        appendMonitor(event.data + "\n");
    };

    eventSource.onerror = function() {
        appendMonitor("=== conexão do monitor encerrada ou erro no candump ===\n");
        if (eventSource) {
            eventSource.close();
            eventSource = null;
        }
    };
}

async function stopMonitor() {
    if (eventSource) {
        eventSource.close();
        eventSource = null;
    }

    await apiPost("/api/monitor_stop", {});
    appendMonitor("=== candump parado ===\n");
}

window.addEventListener("load", async function() {
    await loadDevices();
    await refreshStatus();
});
</script>

</body>
</html>
"""


def run_command(args, timeout=10):
    try:
        completed = subprocess.run(
            args,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=timeout,
            check=False
        )

        return {
            "returncode": completed.returncode,
            "output": completed.stdout.strip()
        }

    except subprocess.TimeoutExpired:
        return {
            "returncode": 124,
            "output": "Timeout executando: " + " ".join(args)
        }

    except Exception as exc:
        return {
            "returncode": 1,
            "output": str(exc)
        }


def valid_iface(name):
    return bool(re.fullmatch(r"[a-zA-Z0-9_.:-]{1,32}", name or ""))


def valid_can_id(can_id):
    return bool(re.fullmatch(r"[0-9A-Fa-f]{1,8}", can_id or ""))


def valid_can_data(data):
    if data is None:
        return False

    data = data.strip()

    if data == "":
        return True

    if not re.fullmatch(r"[0-9A-Fa-f]+", data):
        return False

    if len(data) % 2 != 0:
        return False

    if len(data) > 16:
        return False

    return True


def list_serial_devices():
    devices = []

    by_id_devices = sorted(glob.glob("/dev/serial/by-id/*"))
    tty_devices = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))

    for dev in by_id_devices + tty_devices:
        if dev not in devices:
            devices.append(dev)

    return devices


@app.route("/")
def index():
    return render_template_string(HTML)


@app.route("/api/devices")
def api_devices():
    return jsonify({
        "ok": True,
        "devices": list_serial_devices()
    })


@app.route("/api/status")
def api_status():
    iface = request.args.get("iface", DEFAULT_INTERFACE)

    if not valid_iface(iface):
        return jsonify({
            "ok": False,
            "error": "Nome de interface inválido"
        })

    brief = run_command(["ip", "-br", "link", "show", iface])
    details = run_command(["ip", "-details", "-statistics", "link", "show", iface])

    exists = brief["returncode"] == 0
    output = details["output"] if exists else brief["output"]

    up = False
    if exists:
        up = " UP " in (" " + brief["output"] + " ") or "<" in details["output"] and "UP" in details["output"]

    return jsonify({
        "ok": True,
        "exists": exists,
        "up": up,
        "output": output
    })


@app.route("/api/start_slcan", methods=["POST"])
def api_start_slcan():
    data = request.get_json(force=True)

    port = data.get("port", "")
    iface = data.get("iface", DEFAULT_INTERFACE)
    bitrate = data.get("bitrate", "500000")

    if not port or not os.path.exists(port):
        return jsonify({
            "ok": False,
            "error": "Porta serial não encontrada"
        })

    if not valid_iface(iface):
        return jsonify({
            "ok": False,
            "error": "Nome de interface inválido"
        })

    if bitrate not in SLCAN_SPEEDS:
        return jsonify({
            "ok": False,
            "error": "Bitrate inválido"
        })

    slcan_speed = SLCAN_SPEEDS[bitrate]

    run_command(["modprobe", "can"])
    run_command(["modprobe", "can_raw"])
    run_command(["modprobe", "slcan"])

    run_command(["pkill", "slcand"])
    run_command(["ip", "link", "set", iface, "down"])

    start = run_command([
        "slcand",
        "-o",
        "-c",
        "-f",
        "-s" + slcan_speed,
        port,
        iface
    ])

    if start["returncode"] != 0:
        return jsonify({
            "ok": False,
            "error": "Erro ao iniciar slcand: " + start["output"]
        })

    up = run_command(["ip", "link", "set", iface, "up"])

    if up["returncode"] != 0:
        return jsonify({
            "ok": False,
            "error": "slcand iniciou, mas não foi possível subir a interface: " + up["output"]
        })

    status = run_command(["ip", "-details", "-statistics", "link", "show", iface])

    return jsonify({
        "ok": True,
        "message": f"{iface} iniciado em {BITRATE_LABELS.get(bitrate, bitrate)} usando {port}",
        "output": status["output"]
    })


@app.route("/api/stop", methods=["POST"])
def api_stop():
    data = request.get_json(force=True)
    iface = data.get("iface", DEFAULT_INTERFACE)

    if not valid_iface(iface):
        return jsonify({
            "ok": False,
            "error": "Nome de interface inválido"
        })

    run_command(["ip", "link", "set", iface, "down"])
    run_command(["pkill", "slcand"])

    return jsonify({
        "ok": True,
        "message": f"{iface} parado"
    })


@app.route("/api/send", methods=["POST"])
def api_send():
    data = request.get_json(force=True)

    iface = data.get("iface", DEFAULT_INTERFACE)
    can_id = data.get("can_id", "").strip().upper()
    payload = data.get("data", "").strip().upper()

    if not valid_iface(iface):
        return jsonify({
            "ok": False,
            "error": "Nome de interface inválido"
        })

    if not valid_can_id(can_id):
        return jsonify({
            "ok": False,
            "error": "ID CAN inválido. Use HEX, exemplo: 123"
        })

    if not valid_can_data(payload):
        return jsonify({
            "ok": False,
            "error": "Dados inválidos. Use HEX com até 8 bytes, exemplo: DEADBEEF"
        })

    frame = can_id + "#" + payload

    result = run_command(["cansend", iface, frame])

    if result["returncode"] != 0:
        return jsonify({
            "ok": False,
            "error": result["output"]
        })

    return jsonify({
        "ok": True,
        "frame": frame
    })


@app.route("/api/monitor")
def api_monitor():
    global candump_process

    iface = request.args.get("iface", DEFAULT_INTERFACE)

    if not valid_iface(iface):
        return Response("data: interface invalida\n\n", mimetype="text/event-stream")

    if candump_process is not None:
        try:
            candump_process.terminate()
        except Exception:
            pass

    candump_process = subprocess.Popen(
        ["candump", "-tz", iface],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1
    )

    def stream():
        global candump_process

        try:
            for line in candump_process.stdout:
                line = line.strip()
                if line:
                    yield "data: " + line + "\n\n"

        except GeneratorExit:
            pass

        except Exception as exc:
            yield "data: ERRO: " + str(exc) + "\n\n"

    return Response(stream(), mimetype="text/event-stream")


@app.route("/api/monitor_stop", methods=["POST"])
def api_monitor_stop():
    global candump_process

    if candump_process is not None:
        try:
            candump_process.terminate()
        except Exception:
            pass

        candump_process = None

    return jsonify({
        "ok": True,
        "message": "Monitor parado"
    })


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5006, debug=False)
