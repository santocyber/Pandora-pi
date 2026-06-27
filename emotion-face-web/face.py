from __future__ import annotations

import glob
import os
import importlib.util
import shutil
import subprocess
import sys
import tempfile
import time
from threading import Lock, Thread

from flask import Flask, jsonify, render_template, request


VALID_EMOTIONS = ("neutro", "feliz", "triste", "pensando", "falando", "erro", "assustado")

EMOTION_LABELS = {
    "neutro": "Neutro",
    "feliz": "Feliz",
    "triste": "Triste",
    "pensando": "Pensando",
    "falando": "Falando",
    "erro": "Erro",
    "assustado": "Assustado",
}

app = Flask(__name__, static_folder="static", template_folder="templates")

_state_lock = Lock()
MODEL_FOLDER = os.path.join(app.static_folder, "models")
os.makedirs(MODEL_FOLDER, exist_ok=True)

_current_state = {
    "emotion": "neutro",
    "speech_text": "",
    "speaking": False,
    "tts_available": None,
    "tts_provider": "auto",
    "tts_voice": "female",
    "speech_id": 0,
    "model": "chibi",
}


EDGE_VOICES = {
    "male": os.environ.get("FACE_EDGE_MALE_VOICE", "pt-BR-AntonioNeural"),
    "female": os.environ.get("FACE_EDGE_FEMALE_VOICE", "pt-BR-FranciscaNeural"),
}

PIPER_MODELS = {
    "male": os.environ.get("FACE_PIPER_MALE_MODEL", "").strip(),
    "female": os.environ.get("FACE_PIPER_FEMALE_MODEL", "").strip(),
}

ESPEAK_VOICES = {
    "male": os.environ.get("FACE_ESPEAK_MALE_VOICE", "pt-br+m3"),
    "female": os.environ.get("FACE_ESPEAK_FEMALE_VOICE", "pt-br+f3"),
}


def command_exists(command: str) -> bool:
    return shutil.which(command) is not None


def player_command_for(path: str) -> list[str] | None:
    if path.endswith(".mp3") and command_exists("mpg123"):
        return [shutil.which("mpg123") or "mpg123", "-q", path]
    if command_exists("aplay"):
        return [shutil.which("aplay") or "aplay", "-q", path]
    if command_exists("ffplay"):
        return [shutil.which("ffplay") or "ffplay", "-nodisp", "-autoexit", "-loglevel", "quiet", path]
    return None


def edge_command() -> list[str] | None:
    command = shutil.which("edge-tts")
    if command:
        return [command]
    if importlib.util.find_spec("edge_tts"):
        return [sys.executable, "-m", "edge_tts"]
    return None


def get_tts_info() -> dict:
    edge_available = edge_command()
    piper_command = shutil.which("piper")
    espeak_command = shutil.which("espeak-ng") or shutil.which("espeak")
    piper_models = {
        gender: bool(model and os.path.exists(model)) for gender, model in PIPER_MODELS.items()
    }

    return {
        "default_provider": os.environ.get("FACE_TTS_PROVIDER", "auto"),
        "default_voice": os.environ.get("FACE_TTS_VOICE", "female"),
        "providers": {
            "edge": {
                "available": bool(edge_available),
                "command": " ".join(edge_available) if edge_available else None,
                "requires_internet": True,
                "voices": EDGE_VOICES,
            },
            "piper": {
                "available": bool(piper_command and any(piper_models.values())),
                "command": piper_command,
                "requires_internet": False,
                "models": PIPER_MODELS,
                "voices": piper_models,
            },
            "espeak": {
                "available": bool(espeak_command),
                "command": espeak_command,
                "requires_internet": False,
                "voices": ESPEAK_VOICES,
            },
        },
    }


def provider_available(provider: str, voice: str) -> bool:
    info = get_tts_info()["providers"]
    if provider == "edge":
        return bool(info["edge"]["available"])
    if provider == "piper":
        return bool(info["piper"]["available"] and info["piper"]["voices"].get(voice))
    if provider == "espeak":
        return bool(info["espeak"]["available"])
    return False


def provider_order(provider: str, voice: str) -> list[str]:
    if provider in ("edge", "piper", "espeak"):
        requested = [provider]
    else:
        requested = ["edge", "piper", "espeak"]

    return [item for item in requested if provider_available(item, voice)]


def speak_with_edge(text: str, voice: str) -> bool:
    command = edge_command()
    if not command:
        return False

    selected_voice = EDGE_VOICES.get(voice, EDGE_VOICES["female"])
    with tempfile.NamedTemporaryFile(suffix=".mp3", delete=False) as tmp:
        output_path = tmp.name

    try:
        result = subprocess.run(
            [*command, "--voice", selected_voice, "--text", text, "--write-media", output_path],
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        if result.returncode != 0:
            return False

        player = player_command_for(output_path)
        if not player:
            return False
        return subprocess.run(player, check=False).returncode == 0
    finally:
        try:
            os.unlink(output_path)
        except OSError:
            pass


def speak_with_piper(text: str, voice: str) -> bool:
    piper_command = shutil.which("piper")
    model = PIPER_MODELS.get(voice) or PIPER_MODELS.get("female")
    if not piper_command or not model or not os.path.exists(model):
        return False

    with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as tmp:
        output_path = tmp.name

    try:
        result = subprocess.run(
            [piper_command, "--model", model, "--output_file", output_path],
            input=text,
            text=True,
            check=False,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        if result.returncode != 0:
            return False

        player = player_command_for(output_path)
        if not player:
            return False
        return subprocess.run(player, check=False).returncode == 0
    finally:
        try:
            os.unlink(output_path)
        except OSError:
            pass


def speak_with_espeak(text: str, voice: str) -> bool:
    espeak_command = shutil.which("espeak-ng") or shutil.which("espeak")
    if not espeak_command:
        return False

    selected_voice = ESPEAK_VOICES.get(voice, ESPEAK_VOICES["female"])
    return subprocess.run(
        [espeak_command, "-v", selected_voice, text],
        check=False,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    ).returncode == 0


def get_current_emotion() -> str:
    with _state_lock:
        return _current_state["emotion"]


def set_current_emotion(emotion: str) -> None:
    with _state_lock:
        _current_state["emotion"] = emotion


def get_speech_state() -> dict:
    with _state_lock:
        return {
            "text": _current_state["speech_text"],
            "speaking": _current_state["speaking"],
            "emotion": _current_state["emotion"],
            "tts_available": _current_state["tts_available"],
            "tts_provider": _current_state["tts_provider"],
            "tts_voice": _current_state["tts_voice"],
        }


def set_speech_state(text: str, speaking: bool, emotion: str | None = None) -> int:
    with _state_lock:
        _current_state["speech_id"] += 1
        _current_state["speech_text"] = text
        _current_state["speaking"] = speaking
        if emotion:
            _current_state["emotion"] = emotion
        return _current_state["speech_id"]


def speak_text_async(
    text: str,
    speech_id: int,
    use_tts: bool = True,
    provider: str = "auto",
    voice: str = "female",
) -> None:
    def worker() -> None:
        selected_providers = provider_order(provider, voice) if use_tts else []
        with _state_lock:
            _current_state["tts_available"] = bool(selected_providers)

        if not selected_providers:
            time.sleep(min(10.0, max(1.5, len(text) / 18)))
            with _state_lock:
                if _current_state["speech_id"] != speech_id:
                    return
                _current_state["speaking"] = False
                _current_state["speech_text"] = ""
                if _current_state["emotion"] == "falando":
                    _current_state["emotion"] = "neutro"
            return

        try:
            spoken = False
            used_provider = selected_providers[0]
            for candidate in selected_providers:
                used_provider = candidate
                if candidate == "edge":
                    spoken = speak_with_edge(text, voice)
                elif candidate == "piper":
                    spoken = speak_with_piper(text, voice)
                elif candidate == "espeak":
                    spoken = speak_with_espeak(text, voice)

                if spoken:
                    break

            with _state_lock:
                if _current_state["speech_id"] == speech_id:
                    _current_state["tts_provider"] = used_provider if spoken else provider
                    _current_state["tts_available"] = spoken

            if not spoken:
                time.sleep(min(10.0, max(1.5, len(text) / 18)))
        finally:
            with _state_lock:
                if _current_state["speech_id"] != speech_id:
                    return
                _current_state["speaking"] = False
                _current_state["speech_text"] = ""
                if _current_state["emotion"] == "falando":
                    _current_state["emotion"] = "neutro"

    Thread(target=worker, daemon=True).start()


@app.get("/")
def index():
    return render_template(
        "index.html",
        initial_state=get_current_emotion(),
        emotions=EMOTION_LABELS,
    )


@app.get("/api/health")
def health():
    return jsonify(
        {
            "ok": True,
            "service": "robot-face",
            "state": get_current_emotion(),
            "speech": get_speech_state(),
        }
    )


@app.get("/api/emotion")
def get_emotion():
    return jsonify(
        {
            "state": get_current_emotion(),
            "emotions": EMOTION_LABELS,
        }
    )


@app.get("/caption")
def caption():
    return render_template("caption.html", speech=get_speech_state())


@app.get("/api/speech")
def get_speech():
    return jsonify(get_speech_state())


@app.get("/api/tts/voices")
def get_tts_voices():
    return jsonify(get_tts_info())


@app.get("/api/models")
def list_models():
    models = []
    for f in sorted(glob.glob(os.path.join(MODEL_FOLDER, "*.glb"))):
        name = os.path.splitext(os.path.basename(f))[0]
        size = os.path.getsize(f)
        display = name.replace("_", " ").title()
        models.append({"id": name, "name": display, "size": size})
    return jsonify({"models": models})


@app.post("/api/speech")
def set_speech():
    payload = request.get_json(silent=True) or request.form or {}
    text = str(payload.get("text", "")).strip()
    emotion = str(payload.get("emotion", "falando")).strip().lower()
    use_tts = str(payload.get("tts", "1")).lower() not in ("0", "false", "no", "nao", "não")
    provider = str(payload.get("provider", os.environ.get("FACE_TTS_PROVIDER", "auto"))).strip().lower()
    voice = str(payload.get("voice", os.environ.get("FACE_TTS_VOICE", "female"))).strip().lower()

    if provider not in ("auto", "edge", "piper", "espeak"):
        return jsonify({"error": "Provider TTS inválido.", "valid": ["auto", "edge", "piper", "espeak"]}), 400

    if voice not in ("male", "female"):
        return jsonify({"error": "Voz inválida.", "valid": ["male", "female"]}), 400

    if not text:
        return jsonify({"error": "Texto vazio."}), 400

    if emotion not in VALID_EMOTIONS:
        return (
            jsonify({"error": "Estado inválido.", "received": emotion, "valid": list(VALID_EMOTIONS)}),
            400,
        )

    with _state_lock:
        _current_state["tts_available"] = bool(provider_order(provider, voice)) if use_tts else False
        _current_state["tts_provider"] = provider
        _current_state["tts_voice"] = voice

    speech_id = set_speech_state(text=text, speaking=True, emotion=emotion)
    speak_text_async(text, speech_id=speech_id, use_tts=use_tts, provider=provider, voice=voice)

    return jsonify(get_speech_state())


@app.post("/api/speech/clear")
def clear_speech():
    set_speech_state(text="", speaking=False, emotion="neutro")
    return jsonify(get_speech_state())


@app.post("/api/emotion")
def set_emotion():
    payload = request.get_json(silent=True) or request.form or {}
    emotion = str(payload.get("state", "")).strip().lower()

    if emotion not in VALID_EMOTIONS:
        return (
            jsonify(
                {
                    "error": "Estado inválido.",
                    "received": emotion,
                    "valid": list(VALID_EMOTIONS),
                }
            ),
            400,
        )

    set_current_emotion(emotion)
    return jsonify({"state": emotion, "label": EMOTION_LABELS[emotion]})


if __name__ == "__main__":
    host = os.environ.get("FACE_HOST", "0.0.0.0")
    port = int(os.environ.get("FACE_PORT", "5011"))
    debug = os.environ.get("FACE_DEBUG", "0") == "1"

    print(f"Face do robô disponível em http://{host}:{port}")
    app.run(host=host, port=port, debug=debug)
