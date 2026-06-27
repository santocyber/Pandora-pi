const scene3d = document.querySelector("#scene3d");
const statusText = document.querySelector("#statusText");
const buttons = [...document.querySelectorAll("[data-set-emotion]")];
const panel = document.querySelector("#controlPanel");
const settingsButton = document.querySelector("#settingsButton");
const panelBackdrop = document.querySelector("#panelBackdrop");
const panelClose = document.querySelector("#panelClose");
const captionOverlay = document.querySelector("#captionOverlay");
const mainCaptionText = document.querySelector("#mainCaptionText");
const speechForm = document.querySelector("#speechForm");
const speechText = document.querySelector("#speechText");
const speechStatus = document.querySelector("#speechStatus");
const clearSpeechButton = document.querySelector("#clearSpeechButton");
const ttsProvider = document.querySelector("#ttsProvider");
const ttsVoice = document.querySelector("#ttsVoice");

const labels = window.ROBOT_FACE_LABELS || {
  neutro: "Neutro",
  feliz: "Feliz",
  triste: "Triste",
  pensando: "Pensando",
  falando: "Falando",
  erro: "Erro",
  assustado: "Assustado",
};

let currentState = window.ROBOT_FACE_INITIAL_STATE || "neutro";
let isPolling = false;
let isSpeechPolling = false;
let currentSpeechText = "";
let currentSpeaking = false;

function openPanel() {
  if (!panel || !panelBackdrop || !settingsButton) return;
  panel.classList.remove("hidden");
  panelBackdrop.classList.remove("hidden");
  settingsButton.setAttribute("aria-expanded", "true");
}

function closePanel() {
  if (!panel || !panelBackdrop || !settingsButton) return;
  panel.classList.add("hidden");
  panelBackdrop.classList.add("hidden");
  settingsButton.setAttribute("aria-expanded", "false");
}

function setStatus(message) {
  if (statusText) {
    statusText.textContent = message;
  }
}

function applyEmotion(emotion) {
  if (!emotion || !labels[emotion]) {
    return;
  }

  currentState = emotion;
  document.body.dataset.emotion = emotion;
  setStatus(`Estado atual: ${labels[emotion]}`);

  for (const button of buttons) {
    button.classList.toggle("active", button.dataset.setEmotion === emotion);
  }

  if (typeof window.updateFace3D === "function") {
    window.updateFace3D(emotion);
  }
}

function applySpeech(data) {
  const text = data?.text || "";
  const speaking = Boolean(data?.speaking);
  currentSpeechText = text;
  currentSpeaking = speaking;

  if (mainCaptionText && captionOverlay) {
    mainCaptionText.textContent = text;
    captionOverlay.classList.toggle("visible", Boolean(text));
  }

  if (data?.emotion && data.emotion !== currentState) {
    applyEmotion(data.emotion);
  } else if (typeof window.updateFace3D === "function") {
    window.updateFace3D(currentState);
  }

  if (speechStatus) {
    if (!text) {
      speechStatus.textContent = "";
    } else if (speaking) {
      speechStatus.textContent = data.tts_available === false ? "Falando sem TTS instalado." : "Falando...";
    } else {
      speechStatus.textContent = "Legenda pronta.";
    }
  }
}

async function sendEmotion(emotion) {
  try {
    setStatus(`Alterando para: ${labels[emotion] || emotion}...`);

    const response = await fetch("/api/emotion", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ state: emotion }),
    });

    const data = await response.json();

    if (!response.ok) {
      throw new Error(data.error || "Não foi possível alterar a emoção.");
    }

    applyEmotion(data.state);
  } catch (error) {
    setStatus(error instanceof Error ? error.message : "Erro ao alterar emoção.");
  }
}

async function pollEmotion() {
  if (isPolling) {
    return;
  }

  isPolling = true;

  try {
    const response = await fetch("/api/emotion", { cache: "no-store" });

    if (!response.ok) {
      throw new Error("Servidor indisponível.");
    }

    const data = await response.json();

    if (data.state !== currentState) {
      applyEmotion(data.state);
    }
  } catch {
    setStatus(`Sem conexão com a API. Último estado: ${labels[currentState]}`);
  } finally {
    isPolling = false;
  }
}

async function pollSpeech() {
  if (isSpeechPolling) {
    return;
  }

  isSpeechPolling = true;

  try {
    const response = await fetch("/api/speech", { cache: "no-store" });
    if (!response.ok) {
      throw new Error("Servidor indisponível.");
    }
    applySpeech(await response.json());
  } catch {
    if (currentSpeechText && speechStatus) {
      speechStatus.textContent = "Sem conexão com a API de fala.";
    }
  } finally {
    isSpeechPolling = false;
  }
}

async function sendSpeech(text) {
  const value = text.trim();
  if (!value) {
    if (speechStatus) speechStatus.textContent = "Digite um texto para falar.";
    return;
  }

  try {
    if (speechStatus) speechStatus.textContent = "Enviando fala...";
    const response = await fetch("/api/speech", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({
        text: value,
        emotion: "falando",
        tts: true,
        provider: ttsProvider?.value || "auto",
        voice: ttsVoice?.value || "female",
      }),
    });
    const data = await response.json();
    if (!response.ok) {
      throw new Error(data.error || "Não foi possível falar o texto.");
    }
    applySpeech(data);
  } catch (error) {
    if (speechStatus) {
      speechStatus.textContent = error instanceof Error ? error.message : "Erro ao enviar fala.";
    }
  }
}

async function loadTtsInfo() {
  try {
    const response = await fetch("/api/tts/voices", { cache: "no-store" });
    if (!response.ok) return;
    const data = await response.json();

    if (ttsProvider && data.default_provider) {
      ttsProvider.value = data.default_provider;
    }
    if (ttsVoice && data.default_voice) {
      ttsVoice.value = data.default_voice;
    }

    const providers = data.providers || {};
    const available = Object.entries(providers)
      .filter(([, info]) => info.available)
      .map(([name]) => name)
      .join(", ");

    if (speechStatus && !available) {
      speechStatus.textContent = "Nenhum TTS instalado: instale edge-tts, piper ou espeak-ng.";
    }
  } catch {}
}

async function clearSpeech() {
  try {
    const response = await fetch("/api/speech/clear", { method: "POST" });
    const data = await response.json();
    if (!response.ok) {
      throw new Error(data.error || "Não foi possível limpar a fala.");
    }
    applySpeech(data);
  } catch (error) {
    if (speechStatus) {
      speechStatus.textContent = error instanceof Error ? error.message : "Erro ao limpar fala.";
    }
  }
}

for (const button of buttons) {
  button.addEventListener("click", () => {
    sendEmotion(button.dataset.setEmotion);
  });
}

if (settingsButton) {
  settingsButton.addEventListener("click", () => {
    if (panel?.classList.contains("hidden")) openPanel();
    else closePanel();
  });
}

if (panelClose) {
  panelClose.addEventListener("click", closePanel);
}

if (panelBackdrop) {
  panelBackdrop.addEventListener("click", closePanel);
}

document.addEventListener("keydown", (event) => {
  if (event.key === "Escape") closePanel();
});

if (speechForm && speechText) {
  speechForm.addEventListener("submit", (event) => {
    event.preventDefault();
    sendSpeech(speechText.value);
  });
}

if (clearSpeechButton) {
  clearSpeechButton.addEventListener("click", clearSpeech);
}

if (scene3d && typeof window.initFace3D === "function") {
  window.initFace3D(scene3d, currentState);
}

/* ── Calibration ──────────────────────────────────── */
var calToggle = document.querySelector("#calToggle");
var calSection = document.querySelector("#calSection");
var calSliders = [...document.querySelectorAll(".cal-slider")];
var calCopyBtn = document.querySelector("#calCopyBtn");

function initCalSliders() {
  for (var s of calSliders) {
    var key = s.dataset.cal;
    var val = typeof window.getCal === "function" ? window.getCal(key) : 0;
    if (val != null) { s.value = val; s.nextElementSibling.textContent = val.toFixed(2); }
    s.addEventListener("input", function () {
      var v = parseFloat(this.value);
      if (typeof window.setCal === "function") window.setCal(this.dataset.cal, v);
      this.nextElementSibling.textContent = v.toFixed(2);
    });
  }
}

if (calToggle && calSection) {
  calToggle.addEventListener("click", function () {
    calSection.classList.toggle("hidden");
    if (!calSection.classList.contains("hidden") && calSliders.length > 0 && !calSliders[0]._init) {
      initCalSliders();
      calSliders.forEach(function (s) { s._init = true; });
    }
  });
}

if (calCopyBtn) {
  calCopyBtn.addEventListener("click", function () {
    var cal = typeof window.getCalAll === "function" ? window.getCalAll() : {};
    var lines = ["var preset = {"];
    var keys = Object.keys(cal);
    for (var i = 0; i < keys.length; i++) {
      var comma = i < keys.length - 1 ? "," : "";
      lines.push("  " + keys[i] + ": " + cal[keys[i]].toFixed(2) + comma);
    }
    lines.push("};");
    var code = lines.join("\n");
    if (navigator.clipboard && navigator.clipboard.writeText) {
      navigator.clipboard.writeText(code).then(function () {
        if (calCopyBtn) calCopyBtn.textContent = "✓ Copiado!";
        setTimeout(function () { if (calCopyBtn) calCopyBtn.textContent = "📋 Copiar código"; }, 2000);
      });
    } else {
      var ta = document.createElement("textarea");
      ta.value = code; ta.style.position = "fixed"; ta.style.opacity = "0";
      document.body.appendChild(ta); ta.select();
      try { document.execCommand("copy"); calCopyBtn.textContent = "✓ Copiado!"; } catch (e) {}
      document.body.removeChild(ta);
      setTimeout(function () { if (calCopyBtn) calCopyBtn.textContent = "📋 Copiar código"; }, 2000);
    }
  });
}

applyEmotion(currentState);
loadTtsInfo();
pollSpeech();
setInterval(pollEmotion, 700);
setInterval(pollSpeech, 500);
