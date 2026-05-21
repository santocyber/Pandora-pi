# PandoraPi — Gamepad Bluetooth + CAN Flipsky + Monitor VESC + LiDAR + GPS A76XX + Depth Camera + Follow Autonomo + Painel CAN + Radar LiDAR + Camera 3D

Seis aplicacoes web que transformam um Raspberry Pi em uma central de controle para robos com tracao diferencial usando controladores **Flipsky 75100 (VESC)** conectados via barramento **CAN** e/ou serial USB.

### `gamepad_web_can_flipsky.py` (porta 5005)
Controle via **gamepad Bluetooth HID** (Nintendo Switch Pro Controller ou qualquer controle compativel com evdev). **LiDAR LDROBOT STL-06P** integrado como radar de navegacao com nuvem de pontos 2D persistente (time-decay de 3 segundos). **GPS via modem SIMCom A76XX LTE** com visualizacao em mapa Leaflet, constelacao de satelites em canvas, gravacao de trajeto com exportacao GPX/JSON, upload de rota GPX e navegacao autonoma (follow) com desvio de obstaculos via LiDAR. **Depth Camera Orbbec Astra Pro** com streaming de profundidade colorizada (colormap JET) via `ob_depth.py` (wrapper ctypes). **Freio regenerativo** no botao B (`CAN_PACKET_SET_CURRENT_BRAKE`, 8A configuravel).

### `can_setup.py` (porta 5006)
Painel visual para configuracao do barramento **CAN via SLCAN/SocketCAN**. Suporte a adaptadores CANable com firmware candleLight ou SLCAN. Lista dispositivos seriais, ativa interface CAN via `slcand`, envia/recebe frames CAN, monitor de trafego com **Server-Sent Events** (candump em tempo real). Interface Bootstrap 5 escura com tabela de comandos uteis.

### `3dcam.py` (porta 5007)
Painel dedicado para a **Orbbec Astra Pro** — streaming MJPEG de **RGB** (via OpenCV V4L2) e **depth colorizado** (via OrbbecSDK v1 bridge em subprocesso). Overlay de distancias (grid configuravel, centro com mediana), legendas de cores (TURBO/JET/VIRIDIS/PLASMA/MAGMA/BONE), correcoes USB automaticas, reinicio limpo do sensor depth via unbind/bind. Escala ajustavel, suavizacao, marcacao de areas sem leitura.

### `vesc_read.py` (porta 5008)
Monitor direto do VESC com pyvesc via **serial USB** — telemetria em tempo real (tensao, corrente, RPM, duty cycle, potencia, temperatura, fault codes), **teste de motor** (duty cycle, forward/reverse, freio regenerativo com auto-stop), **TCP Bridge** serial→TCP para uso simultaneo com VESC Tool, upload/visualizacao de arquivos XML de configuracao do VESC, estimativa de bateria, historico de telemetria e graficos Chart.js.

### `vesc_controller.py` (porta 5009)
Controlador simplificado e modular com suporte a **gamepad Bluetooth HID (evdev)** ou **teclado**. Opera em dois modos: **serial** (1 VESC via USB com telemetria pyvesc) ou **CAN** (2 ou 4 VESCs via CANable com SocketCAN). Streaming de **webcam USB (OpenCV)** com gravacao MP4. Toda configuracao via variaveis de ambiente — sem arquivo JSON de config. Ideal para setups rapidos e depuracao.

### `lidar.py` (porta 5010)
Radar visual para o **LiDAR LDROBOT STL-06P** — nuvem de pontos 2D em canvas com sweep animado, aneis de alcance (bands coloridas), deteccao de objetos agrupados por proximidade (cluster), marcadores de ponto mais proximo/longe, regua de escala, **preview 3D** via Three.js com OrbitControls (girar, zoom, pan). Exportacao completa nos formatos **PNG, JSON, CSV e PLY** com coordenadas em metros, cores por distancia e confianca.

![Interface PandoraPi](assets/screenshot.png)

## Hardware necessario

| Componente | Detalhes |
|---|---|
| Raspberry Pi | Com Bluetooth integrado (ou dongle USB) |
| CANable | Adaptador USB-CAN com firmware **candleLight** (recomendado). Com firmware slcan, e necessario configurar `slcand` manualmente antes de usar |
| 2× Flipsky 75100 VESC | IDs CAN 1 (motor esquerdo) e 2 (motor direito) |
| Gamepad Bluetooth | Nintendo Switch Pro Controller (testado) ou qualquer controle HID reconhecido pelo kernel Linux via evdev |
| LiDAR LDROBOT STL-06P | Conectado via USB serial (aparece como `/dev/ttyUSB0`). Baud rate 230400 |
| SIMCom A76XX LTE Module | Modem com GNSS integrado (GPS, GLONASS, Galileo, BeiDou). Aparece como `/dev/ttyUSB0-3`. GPS via comandos AT em `/dev/ttyUSB1` a 115200 baud |
| Orbbec Astra Pro | Camera 3D depth via USB. Captura 640×480 @ 30fps Y16. Wrapper ctypes em `ob_depth.py` sobre `libOrbbecSDK.so` (SDK incluso em `OrbbecSDK/lib/arm64/`). Colormap JET via OpenCV |

## Como funciona

### gamepad_web_can_flipsky.py (porta 5005)

```
Navegador (http://IP_DO_PI:5005)
        │
        ▼ Socket.IO (WebSocket)
┌───────────────────────────────┐
│         Flask Server          │
│  - Rotas REST de configuracao │
│  - Socket.IO para eventos     │
│    em tempo real              │
├───────────────────────────────┤
│  Thread: gamepad_reader_loop  │
│  Le /dev/input/eventX (evdev) │──► Gamepad Bluetooth (HID)
│  Emite eventos via Socket.IO  │
├───────────────────────────────┤
│  Thread: gps_reader_loop       │
│  Abre /dev/ttyUSB1 (pyserial)  │──► Modem A76XX
│  AT+CGNSSPWR=1, AT+CGNSSINFO=1 │    GNSS (GPS/GLONASS/Galileo/BeiDou)
│  Parse +CGNSSINFO, emite S.IO  │
├───────────────────────────────┤
│  Thread: depth_camera_loop     │
│  ob_depth.py (ctypes)          │──► Orbbec Astra Pro
│  Depth→colormap JET→JPEG→S.IO  │    (640×480 @ 30fps)
├───────────────────────────────┤
│  Thread: lidar_reader_loop    │
│  Le /dev/ttyUSB0 (pyserial)   │──► LiDAR LDROBOT STL-06P
│  Protocolo 0x54, CRC, 230400  │
│  Emite nuvem pontos via S.IO  │
├───────────────────────────────┤
│  CAN / SocketCAN (PF_CAN)     │
│  Envia quadros CAN estendidos │──► CANable ──► Flipsky VESC 1 + 2
│  com duty cycle (-1.0 a 1.0)  │
└───────────────────────────────┘
```

### vesc_read.py (porta 5008)

```
Navegador (http://IP_DO_PI:5008)
        │
        ▼ REST API (JSON)
┌───────────────────────────────┐
│         Flask Server          │
│  - Rotas REST de telemetria   │
│  - Template inline + Chart.js │
├───────────────────────────────┤
│  Thread: vesc_reader_loop     │
│  VESC(serial_port=/dev/ttyACM0)│──► Flipsky VESC (USB Serial)
│  get_measurements() a 4 Hz    │    (pyvesc protocol)
│  Processa fault codes, calcula│
│  potencia, estima bateria     │
├───────────────────────────────┤
│  Motor Test (queue)           │
│  set_duty_cycle / set_current │──► Flipsky VESC (USB Serial)
│  forward/reverse/regen brake  │
│  Auto-stop por tempo/fault    │
├───────────────────────────────┤
│  TCP Bridge (serial ↔ TCP)    │
│  socket server na porta 65102 │──► VESC Tool (TCP)
│  Bridge transparente          │
├───────────────────────────────┤
│  VESC XML Config              │
│  Upload app/motor XML → parse │
│  Visualizacao flat/compact    │
├───────────────────────────────┤
│  Historico (deque 600pts)     │
│  Chart.js graficos historicos │
└───────────────────────────────┘
```

### vesc_controller.py (porta 5009)

```
Navegador (http://IP_DO_PI:5009)
        │
        ▼ Socket.IO (WebSocket) + REST API
┌───────────────────────────────┐
│         Flask Server          │
│  - Rotas REST de config + cam │
│  - Socket.IO para eventos     │
│    em tempo real              │
├───────────────────────────────┤
│  Thread: gamepad_reader_loop  │
│  Le /dev/input/eventX (evdev) │──► Gamepad Bluetooth (HID)
│  Emite eventos via Socket.IO  │
├───────────────────────────────┤
│  Thread: vesc_reader_loop     │
│  Modo serial: pyvesc telemetry│──► Flipsky VESC (USB Serial)
│  Modo CAN: SocketCAN duty     │    ou CANable
├───────────────────────────────┤
│  Thread: camera_loop          │
│  OpenCV /dev/videoX           │──► Webcam USB
│  JPEG encode → base64 → S.IO  │    Grava MP4 (opcional)
├───────────────────────────────┤
│  CAN / SocketCAN (PF_CAN)     │
│  Envia quadros CAN estendidos │──► CANable ──► 2-4 Flipsky VESC
│  com duty cycle (-1.0 a 1.0)  │
└───────────────────────────────┘
```

### can_setup.py (porta 5006)

```
Navegador (http://IP_DO_PI:5006)
        │
        ▼ REST API + SSE
┌───────────────────────────────┐
│         Flask Server          │
│  - Rotas REST de config CAN   │
│  - SSE para candump ao vivo   │
├───────────────────────────────┤
│  subprocess: slcand, cansend  │
│  subprocess: ip link, modprobe│
│  subprocess: candump (SSE)    │──► CANable / SLCAN
├───────────────────────────────┤
│  Lista dispositivos seriais   │
│  /dev/serial/by-id/*          │
│  /dev/ttyACM*, /dev/ttyUSB*   │
└───────────────────────────────┘
```

### 3dcam.py (porta 5007)

```
Navegador (http://IP_DO_PI:5007)
        │
        ▼ MJPEG (multipart/x-mixed-replace) + REST
┌───────────────────────────────┐
│         Flask Server          │
│  - /video/rgb  (MJPEG stream) │
│  - /video/depth (MJPEG stream)│
│  - REST API de controle       │
├───────────────────────────────┤
│  Thread: rgb_worker           │
│  OpenCV V4L2 /dev/video2      │──► Orbbec Astra Pro (RGB)
│  MJPG fourcc, mirror, JPEG    │
├───────────────────────────────┤
│  OrbbecSDK v1 Bridge          │
│  Subprocesso C++ nativo       │──► Orbbec Astra Pro (Depth)
│  Escreve /tmp/orbbec_depth.raw│    Y16 640×480 via USB
│  + /tmp/orbbec_depth_meta.json│
├───────────────────────────────┤
│  Thread: depth_worker         │
│  Le raw → numpy uint16        │
│  Colormap (TURBO/JET/etc)     │
│  Overlay distancias + legendas│
│  MJPEG stream                  │
├───────────────────────────────┤
│  Correcoes USB                │
│  autosuspend, usbfs_memory_mb │
│  unbind/bind 2bc5:0403        │
└───────────────────────────────┘
```

### lidar.py (porta 5010)

```
Navegador (http://IP_DO_PI:5010)
        │
        ▼ REST API + Three.js CDN
┌───────────────────────────────┐
│         Flask Server          │
│  - /api/points (GET JSON)     │
│  - Template inline completo   │
├───────────────────────────────┤
│  Thread: serial_worker        │
│  pyserial /dev/ttyUSB0        │──► LiDAR LDROBOT STL-06P
│  230400 baud, protocolo 0x54  │    (USB serial)
│  Parse 47-byte frames, CRC    │
├───────────────────────────────┤
│  Frontend Canvas 2D           │
│  Nuvem pontos + sweep + grid  │
│  Deteccao objetos (cluster)   │
│  Marcadores perto/longe       │
├───────────────────────────────┤
│  Frontend Three.js 3D         │
│  Preview 3D com OrbitControls │
│  Esferas extremos, grid, eixos│
│  Export PNG/JSON/CSV/PLY      │
└───────────────────────────────┘
```

### vesc_controller.py — funcionamento

O `vesc_controller.py` foi projetado para ser uma alternativa mais simples e modular ao `gamepad_web_can_flipsky.py`. Diferencas principais:

| Caracteristica | gamepad_web_can_flipsky.py | vesc_controller.py |
|---|---|---|
| Linhas | ~7061 | ~1966 |
| Configuracao | JSON (`gamepad_config.json`) | Variaveis de ambiente |
| Perifericos | LiDAR, GPS, Depth Camera (Orbbec) | Webcam USB (OpenCV) |
| Modos VESC | Apenas CAN (2 motores) | Serial (1) ou CAN (2-4) |
| Navegacao autonoma | Follow GPX + obstacle avoidance | Nao |
| Controle por teclado | Nao | Sim (WASD/Setas + Espaco) |
| Gravacao de video | Nao | Sim (MP4 na pasta `recordings/`) |

**Matematica dos motores** (identica entre todas as apps):

```
throttle = Eixo Y  (analogico esquerdo, cima/baixo)
steering = Eixo RX (analogico direito, esquerda/direita)

left  = (throttle + steering × steering_gain) × max_duty
right = (throttle - steering × steering_gain) × max_duty
```

### Modos de operacao (vesc_controller.py)

- **Serial (USB)**: Conecta 1 VESC via USB serial (`/dev/ttyACM0`). Usa pyvesc para telemetria completa (tensao, corrente, RPM, duty, potencia, temperatura FET/motor, fault codes) e controle de duty/freio. Ideal para teste de bancada com 1 motor.
- **CAN**: Conecta 2 ou 4 VESCs via barramento CAN. Envia duty cycle e corrente de freio via SocketCAN. Sem telemetria (o barramento CAN e unidirecional para controle). Suporta 4 motores (2 frontais + 2 traseiros com os mesmos valores de duty).

- **Controle por gamepad**: Modo padrao. Le o dispositivo HID via evdev, normaliza eixos com deadzone configuravel. Homem-morto no botao R (`BTN_TR`), freio regenerativo no botao B (`BTN_SOUTH`).
- **Controle por teclado**: Alternativa quando nao ha gamepad disponivel. Setas ou WASD para mover, Espaco para freio. Inatividade > 0.3s corta a potencia automaticamente.

## Matematica dos motores

```
throttle = ABS_Y  (analogico esquerdo, cima/baixo)
steering = ABS_X  (analogico esquerdo, esquerda/direita)

left  = (throttle + steering × steering_gain) × max_duty
right = (throttle - steering × steering_gain) × max_duty
```

Os valores sao saturados entre `[-1.0, 1.0]` e multiplicados pelo `max_duty` configuravel. O botao homem-morto (`BTN_TR` / botao R) precisa estar pressionado para enviar qualquer potencia aos motores.

### Radar LiDAR — persistencia visual (time-decay)

Os pontos do LiDAR acumulam por **3 segundos** com fading progressivo. Cada ponto recebe um timestamp no recebimento e sua opacidade decai linearmente:

```
alpha = alpha_base × (1 - idade_ms / 3000)

0s: 100% opaco (cor viva)
1s: 66% opaco
2s: 33% opaco
3s: removido
```

Isso cria um **"rastro" visual** ao redor do robo, permitindo ao operador enxergar obstaculos mesmo quando o feixe do LiDAR nao esta apontando diretamente para eles. Aneis de perigo (50cm vermelho tracejado, 1m laranja tracejado) indicam zonas de alerta. O card mostra a distancia do obstaculo mais proximo em tempo real.

O sistema funciona **sem odometria** — assume que o robo se move devagar o suficiente para o historico de 3 segundos ainda ser util. Para maior precisao com o robo em movimento rapido, seria necessario dead reckoning via encoders (nao implementado).

### GPS A76XX — Streaming AT + Parser CGNSSINFO

O modem SIMCom A7670E-MASA usa chipset **ASR** com comandos AT proprietarios:

```
AT+CGNSSPWR=1           # liga o chip GNSS
AT+CGNSSINFO=1          # inicia streaming continuo de posicao
```

A resposta `+CGNSSINFO:` contem 19 campos em formato fixo:

```
+CGNSSINFO: 3,15,,05,07,44.2737427,N,9.5367308,E,100526,152328.00,360.2,4.984,89.12,2.63,1.39,2.23,10
           fix gps glo bd ga  lat    NS lng   EW  date  utc      alt speed hdg pdop hdop vdop tdop sats
```

O parser `parse_cgnssinfo()` extrai: modo do fix (0/2/3D), latitude/longitude em graus decimais, altitude (m), velocidade (km/h), rumo (graus), satelites por constelacao (GPS, GLONASS, BeiDou, Galileo), HDOP/PDOP/VDOP/TDOP.

### Freio regenerativo (botao B / BTN_SOUTH)

O botao **B** (BTN_SOUTH no Pro Controller) envia o comando `CAN_PACKET_SET_CURRENT_BRAKE` (ID 2) com **8A** de corrente de freio para ambos os VESCs. O freio funciona mesmo sem o deadman pressionado — seguranca sobreposta.

```
B pressionado → CAN_PACKET_SET_CURRENT_BRAKE 8A ambos VESCs
B solto       → CAN_PACKET_SET_CURRENT_BRAKE 0A (libera freio)

Payload: int32 big-endian, escala A × 1000
CAN ID: (2 << 8) | vesc_id
```

Configuravel via `can.brake_button` (padrao `BTN_SOUTH`) e `can.brake_current` (padrao `8.0` A, range 0–200A).

### Upload de trajeto GPX

Arquivos **.gpx** (GPS Exchange Format) podem ser carregados via botao **Upload GPX**. O parser `parse_gpx_xml()` extrai os `<trkpt>` de `<trkseg>`, suportando namespace GPX 1.1. O trajeto carregado aparece como polyline **vermelha** no mapa Leaflet, ao lado do trajeto sendo gravado (verde). O botao **Limpar mapa** remove ambos.

Formatos suportados: GPX 1.1 com `<trkpt lat="..." lon="...">`, opcionalmente `<ele>`, `<time>`.

### Seguir trajeto (Follow autonomo)

O robo pode seguir automaticamente um trajeto GPX carregado, navegando por waypoints usando GPS + bussola:

```
A cada ciclo GPS (1 Hz):
  1. current = (gps_state.lat, gps_state.lng)
  2. target  = uploaded_trajectory[next_waypoint]
  3. distance = haversine(current, target)          # metros
  4. bearing  = bearing_to(current, target)          # graus 0-360
  5. hdg_error = angle_diff(bearing, gps.heading)    # -180 a +180
  6. steering = clamp(hdg_error × steering_kp, -1, 1)
  7. throttle = min(max_auto_speed, distance × 0.04)
  8. if distance < waypoint_threshold → proximo waypoint
  9. ultimo waypoint → follow_stop()
```

**Requisitos:** robo armado + fix GPS ≥ 2 (3D) + trajeto GPX carregado. Configuravel via `follow_config`: `waypoint_threshold` (2m), `max_auto_speed` (0.15), `steering_kp` (0.5).

### Desvio de obstaculos (LiDAR durante Follow)

Durante o follow autonomo, o **LiDAR** e usado para detectar e desviar de obstaculos em tempo real:

```
18 setores de 10° no semicirculo frontal (-90° a +90°):
  • Obstaculo < safe_distance_mm (50cm) → forca repulsiva proporcional
  • Obstaculo < critical_distance_mm (30cm) nos setores frontais → PARADA EMERGENCIA
  • Setores esquerdos empurram steering → direita (+)
  • Setores direitos empurram steering → esquerda (-)

steering_final = path_steering × (1 - weight) + avoidance_steering × weight
throttle_final = throttle_path × (dist_front / safe_distance)  [se obstaculo a frente]
```

Configuravel na interface: distancia segura (cm), distancia critica (cm), checkbox toggle. O robo **para imediatamente** se algo entrar a menos de 30cm na frente, e **reduz velocidade** proporcionalmente se obstaculo estiver entre 30cm e 50cm.

### Depth Camera Orbbec Astra Pro

A camera 3D Orbbec Astra Pro e integrada via wrapper ctypes (`ob_depth.py`) sobre `libOrbbecSDK.so`:

```
DepthCamera (ob_depth.py)
  → DepthStream 640×480 Y16 (uint16, mm)
  → numpy clip [min_mm, max_mm] + normalize 0-255
  → OpenCV applyColorMap(JET)
  → JPEG encode (quality 60) → base64
  → Socket.IO "depth_frame" a ~10 fps
```

O card na interface mostra a imagem colorizada em tempo real com escala de cores (perto=vermelho, medio=amarelo, longe=verde, muito longe=azul), metricas de FPS e distancias min/max configuraveis.

### Gravacao de trajeto

O trajeto e armazenado em memoria durante a gravacao. Cada ponto contem:

```json
{"lat": 44.2737, "lng": 9.5367, "alt": 360.2, "speed_kmh": 4.98, "heading": 89.12, "utc_time": "152328.00", "epoch": 1746891130.123}
```

Funcoes disponiveis: **Iniciar**, **Pausar**, **Retomar**, **Parar**. O download esta disponivel em dois formatos:
- **GPX 1.1** (`trajectory_to_gpx()`) — padrao universal, compativel com Google Earth, Strava, Garmin, OSMAnd, QGIS
- **JSON** (`trajectory_to_json()`) — formato nativo com metadados (tempo de inicio/fim, total de pontos, distancia total calculada via Haversine)

## Dependencias

### Sistema (pacotes apt)

| Pacote | Uso | Obrigatorio |
|---|---|---|
| `python3 python3-pip` | Runtime Python | Sim |
| `bluetooth bluez` | Stack Bluetooth + `bluetoothctl` | Sim (gamepad) |
| `kmod` | `modprobe` / `modinfo` (modulos HID) | Sim |
| `iproute2` | `ip link` (ativar interface CAN) | Sim |
| `sudo` | Escalacao de privilegios (CAN, serial) | Sim |
| `can-utils` | `candump`, `cansend` (diagnostico CAN) | Recomendado |
| `evtest` | Teste de dispositivos de entrada | Recomendado |
| `joystick` | Utilitarios de joystick | Recomendado |
| `usbutils` | `lsusb` (depuracao USB) | Recomendado |
| `libusb-1.0-0` | Dependencia nativa do Orbbec SDK (`libob_usb.so`) | Somente Depth Camera |
| `v4l-utils` | `v4l2-ctl --list-devices` (deteccao de cameras RGB) | Somente 3dcam.py |

```bash
# Instalacao completa (todos os componentes)
sudo apt install -y python3 python3-pip bluetooth bluez kmod iproute2 sudo \
  can-utils evtest joystick usbutils libusb-1.0-0 v4l-utils

# Instalacao minima (gamepad + CAN, sem perifericos)
sudo apt install -y python3 python3-pip bluetooth bluez kmod iproute2 sudo
```

### Modulos do kernel

| Modulo | Funcao | Quando carregar |
|---|---|---|
| `hidp` | HID sobre Bluetooth | Obrigatorio para gamepad |
| `hid-nintendo` | Driver Nintendo Switch Pro Controller | Obrigatorio para Pro Controller |
| `can` / `can_raw` | Subsistema SocketCAN | Carregado automaticamente pelo `ip link` |

```bash
sudo modprobe hidp
sudo modprobe hid-nintendo
# Os modulos CAN (can, can_raw, can_dev) sao carregados automaticamente
# ao executar "sudo ip link set can0 up type can bitrate 500000"
```

> Se `hid-nintendo` nao existir no seu kernel, atualize o kernel ou use outro gamepad HID generico.

### Python (pip)

| Pacote | Uso | Obrigatorio |
|---|---|---|
| `flask` | Framework web (todas as apps) | Sim |
| `flask-socketio` | WebSocket / eventos em tempo real (gamepad + controller) | Sim (gamepad/controller) |
| `evdev` | Leitura de gamepad (`/dev/input/eventX`) | Sim (gamepad) |
| `pyserial` | Comunicacao serial (LiDAR + GPS A76XX) | Sim (LiDAR/GPS) |
| `pyvesc` | Comunicacao serial com VESC (vesc_read.py + controller serial) | Sim (monitor VESC) |
| `numpy` | Processamento de arrays (colormap depth) | Somente Depth Camera |
| `opencv-python` | Colormap JET + compressao JPEG + webcam (vesc_controller) | Depth Camera e webcam |

```bash
# Instalacao completa (todos os componentes)
pip install flask flask-socketio evdev pyserial pyvesc numpy opencv-python

# Instalacao minima (gamepad + CAN, sem LiDAR/GPS/Depth/VESC monitor)
pip install flask flask-socketio evdev

# Instalacao minima (monitor VESC standalone)
pip install flask pyvesc

# Instalacao minima (controller simplificado)
pip install flask flask-socketio evdev
```

> Cada componente e importado sob `try/except` — se faltar, o subsistema correspondente e desabilitado sem quebrar o resto. `pyserial` e necessario para LiDAR e GPS. `numpy` e `opencv-python` sao necessarios para a camera depth (colormap + JPEG) e webcam USB no `vesc_controller.py`.

### Depth Camera (Orbbec Astra Pro)

A camera requer o SDK Orbbec (clonado no projeto) e `ob_depth.py` (ja incluso):

```bash
# O SDK ja esta em OrbbecSDK/ (clonado do GitHub)
# As bibliotecas nativas estao em lib/arm64/
# O wrapper Python esta em ob_depth.py

# Verificar se a camera e reconhecida:
python3 -c "
from ob_depth import DepthCamera
cam = DepthCamera()
cam.start()
frame = cam.get_frame()
print(f'OK: {frame.shape}')
cam.close()
"
```

> A camera depth so funciona em **Raspberry Pi (aarch64/arm64)**. Em x86_64, `ob_depth.py` exibe um erro claro de arquitetura.

## VESC Controller (`vesc_controller.py`) — porta 5009

Aplicacao web simplificada para controle essencial do robo. Suporta gamepad HID (evdev) ou teclado, modo serial (1 VESC) ou CAN (2-4 VESCs), webcam USB com gravacao MP4.

### Funcionalidades

| Funcionalidade | Descricao |
|---|---|
| **Gamepad HID (evdev)** | Leitura de gamepad Bluetooth com sticks visuais, barras de throttle/steering, indicador homem-morto |
| **Controle por teclado** | Modo alternativo com setas/WASD + Espaco (freio). Timeout de inatividade 0.3s |
| **Modo Serial (USB)** | 1 VESC com telemetria completa via pyvesc (9 metricas em tempo real) |
| **Modo CAN** | 2 ou 4 VESCs via SocketCAN. Suporte a 4 motores (2 frontais + 2 traseiros) |
| **Webcam USB** | Streaming JPEG via OpenCV + gravacao MP4. Lista e download de gravacoes |
| **Parada de emergencia** | Botao dedicado que desarma e zera duty de todos os motores |
| **Auto-stop por fault** | Qualquer fault code ≠ 0 desarma o robo automaticamente (modo serial) |

### Como rodar

```bash
# Modo padrao (serial, 1 VESC via USB)
python vesc_controller.py

# Modo CAN (2 VESCs via CANable)
VESC_MODE=can python vesc_controller.py

# Com webcam
python vesc_controller.py
# Clique em "LIGAR CAMERA" na interface

# Com todas as opcoes
VESC_MODE=can VESC_CAN_INTERFACE=can0 VESC_MAX_DUTY=0.25 VESC_STEERING_GAIN=0.65 \
  python vesc_controller.py
```

Acesse no navegador: **http://<IP_DO_RASPBERRY>:5009**

### Variaveis de ambiente

| Variavel | Padrao | Descricao |
|---|---|---|
| `VESC_MODE` | `serial` | Modo de operacao: `serial` (USB) ou `can` (CANable) |
| `VESC_SERIAL_PORT` | `/dev/ttyACM0` | Porta serial do VESC (modo serial) |
| `VESC_SERIAL_BAUD` | `115200` | Baud rate serial |
| `VESC_CAN_INTERFACE` | `can0` | Nome da interface SocketCAN |
| `VESC_CAN_BITRATE` | `500000` | Taxa do barramento CAN |
| `VESC_CAN_LEFT_ID` | `1` | ID CAN do VESC esquerdo (0-255) |
| `VESC_CAN_RIGHT_ID` | `2` | ID CAN do VESC direito (0-255) |
| `VESC_CAN_ID_3` | `3` | ID CAN do 3º VESC (modo 4 motores) |
| `VESC_CAN_ID_4` | `4` | ID CAN do 4º VESC (modo 4 motores) |
| `VESC_MOTOR_COUNT` | `2` | Numero de motores: `2` ou `4` |
| `VESC_MAX_DUTY` | `0.25` | Duty cycle maximo (0.01 a 0.95) |
| `VESC_STEERING_GAIN` | `0.65` | Ganho de direcao (0 a 1) |
| `VESC_DEADMAN_BUTTON` | `BTN_TR` | Botao homem-morto (R no Pro Controller) |
| `VESC_BRAKE_BUTTON` | `BTN_SOUTH` | Botao de freio regenerativo (B) |
| `VESC_BRAKE_CURRENT` | `8.0` | Corrente de freio (A) |
| `VESC_SEND_INTERVAL` | `0.05` | Intervalo minimo entre envios CAN (s) |
| `VESC_CONTROL_TIMEOUT` | `0.5` | Timeout do controle (s) |
| `VESC_INVERT_THROTTLE` | `1` | Inverte acelerador (1=sim, 0=nao) |
| `VESC_INVERT_STEERING` | `0` | Inverte direcao |
| `VESC_INVERT_LEFT` | `0` | Inverte motor esquerdo |
| `VESC_INVERT_RIGHT` | `1` | Inverte motor direito |
| `VESC_THROTTLE_AXIS` | `ABS_Y` | Eixo do acelerador |
| `VESC_STEERING_AXIS` | `ABS_RX` | Eixo da direcao |
| `VESC_DEADZONE` | `0.05` | Zona morta dos analogicos |
| `CAMERA_DEVICE` | `0` | Dispositivo de camera (`/dev/videoX`) |
| `CAMERA_WIDTH` | `640` | Largura da captura |
| `CAMERA_HEIGHT` | `480` | Altura da captura |
| `CAMERA_FPS` | `15` | FPS alvo da camera |

### Endpoints REST

| Metodo | Rota | Descricao |
|---|---|---|
| `GET` | `/` | Interface web com telemetria, sticks e controle |
| `GET` | `/api/state` | Estado completo (gamepad + VESC + controle + camera) |
| `POST` | `/api/arm` | Armar robo |
| `POST` | `/api/disarm` | Desarmar robo |
| `POST` | `/api/emergency-stop` | Parada de emergencia |
| `POST` | `/api/can/setup` | Ativar interface CAN (`ip link set up`) |
| `GET` | `/api/can/status` | Status da interface CAN |
| `GET` | `/api/ports` | Listar portas seriais, dispositivos de input e cameras |
| `POST` | `/api/gamepad/select` | Selecionar gamepad por caminho (`/dev/input/eventX`) |
| `POST` | `/api/control/mode` | Alternar modo: `gamepad` ou `keyboard` |
| `POST` | `/api/control/keyboard` | Enviar comando de teclado (`throttle`, `steering`, `brake`) |
| `POST` | `/api/mode` | Alternar modo VESC: `serial` ou `can` |
| `POST` | `/api/config` | Atualizar config em runtime |
| `POST` | `/api/camera/device` | Selecionar dispositivo de camera |
| `POST` | `/api/camera/on` | Ligar camera |
| `POST` | `/api/camera/off` | Desligar camera |
| `POST` | `/api/camera/record/toggle` | Alternar gravacao |
| `GET` | `/api/camera/recordings` | Listar gravacoes |
| `GET` | `/api/camera/recording/<filename>` | Download de gravacao MP4 |

### Seguranca

- **Auto-stop** por fault: qualquer fault code ≠ 0 desarma o robo (modo serial)
- **Timeout de inatividade**: no modo teclado, 0.3s sem comando zera a potencia
- **Homem-morto**: botao R precisa estar segurado para movimento (modo gamepad)
- **Parada de emergencia**: botao dedicado que zera duty imediatamente
- **Duty maximo**: limitado por `VESC_MAX_DUTY` (padrao 25%)

## Monitor VESC / Teste de Motor (`vesc_read.py`)

Aplicacao web independente que conecta diretamente ao VESC via **serial USB** usando o protocolo `pyvesc`. Permite monitorar e testar o motor sem depender de CAN ou gamepad.

### Funcionalidades

| Funcionalidade | Descricao |
|---|---|
| **Telemetria em tempo real** | Tensao (V), corrente de entrada/motor (A), RPM, duty cycle (%), potencia (W), temperatura FET/motor, consumo (Ah/Wh) |
| **Fault codes** | Deteccao automatica de 19 fault codes com nome e acao de seguranca (auto-stop) |
| **Estimativa de bateria** | Calcula % da bateria baseado nos parametros de cutoff da config real do VESC |
| **Teste de motor** | Controle de duty cycle com forward/reverse, freio regenerativo, auto-stop por tempo limite (10s) ou fault |
| **TCP Bridge** | Ponte serial→TCP (porta 65102) para usar o VESC Tool simultaneamente com o monitor |
| **XML Config** | Upload e visualizacao de arquivos XML de configuracao do VESC (app_config.xml, motor_config.xml) |
| **Historico** | Deque dos ultimos 600 pontos com graficos Chart.js (tensao, corrente, RPM, duty, potencia, temperatura) |

### Como rodar

```bash
# Porta serial padrao: /dev/ttyACM0 (configuravel via VESC_PORT)
python vesc_read.py
```

Acesse no navegador: **http://<IP_DO_RASPBERRY>:5008**

### Variaveis de ambiente

| Variavel | Padrao | Descricao |
|---|---|---|
| `VESC_PORT` | `/dev/ttyACM0` | Porta serial do VESC |
| `VESC_INTERVAL` | `0.25` | Intervalo de leitura (segundos) |
| `VESC_HISTORY_LIMIT` | `600` | Pontos no historico |
| `VESC_TCP_PORT` | `65102` | Porta TCP da bridge |
| `VESC_BAUD` | `115200` | Baud rate da serial (bridge) |
| `VESC_TCP_BUFFER` | `4096` | Tamanho do buffer TCP |
| `VESC_MOTOR_TEST_MAX_DUTY_PERCENT` | `8.0` | Duty maximo no teste de motor |
| `VESC_MOTOR_TEST_DEFAULT_DUTY_PERCENT` | `1.0` | Duty inicial no teste |
| `VESC_MOTOR_TEST_STEP_PERCENT` | `1.0` | Incremento/decremento do duty |
| `VESC_MOTOR_TEST_MAX_DURATION_S` | `10.0` | Tempo maximo de teste continuo |
| `VESC_MOTOR_TEST_REGEN_BRAKE_CURRENT_A` | `2.0` | Corrente do freio regenerativo |
| `VESC_MOTOR_TEST_REGEN_BRAKE_DURATION_S` | `2.0` | Duracao do freio regenerativo |
| `VESC_REAL_CONFIG_DIR` | `vesc_real_configs` | Pasta para XMLs de config |

### Endpoints REST

| Metodo | Rota | Descricao |
|---|---|---|
| `GET` | `/` | Interface web com graficos e controle de motor |
| `GET` | `/api/data` | Telemetria atual + estado + config real + bridge + motor test |
| `GET` | `/api/history` | Historico completo (deque dos ultimos 600 pontos) |
| `GET` | `/api/ports` | Portas seriais disponiveis no sistema |
| `GET` | `/api/real-config` | Configuracao XML carregada (app + motor) |
| `POST` | `/api/real-config/upload` | Upload de XML (`type=app` ou `type=motor`) |
| `GET` | `/api/motor-test` | Estado atual do teste de motor |
| `POST` | `/api/motor-test/start` | Iniciar teste (`duty_percent`, `direction`) |
| `POST` | `/api/motor-test/stop` | Parar teste |
| `POST` | `/api/motor-test/set-duty` | Ajustar duty (`duty_percent`) |
| `POST` | `/api/motor-test/step` | Incrementar/decrementar duty (`delta_percent`) |
| `POST` | `/api/motor-test/direction` | Mudar direcao (`direction`: forward/reverse) |
| `POST` | `/api/motor-test/regen-brake` | Aplicar freio regenerativo (`current_a`) |
| `GET` | `/api/tcp-bridge` | Estado da bridge TCP |
| `POST` | `/api/tcp-bridge/start` | Iniciar bridge (`port`) |
| `POST` | `/api/tcp-bridge/stop` | Parar bridge |

### Seguranca

- **Auto-stop** por tempo: o teste de motor para automaticamente apos `MOTOR_TEST_MAX_DURATION_S` (10s)
- **Auto-stop** por fault: qualquer fault code ≠ 0 interrompe o motor imediatamente
- **TCP Bridge** bloqueia teste de motor enquanto ativa (evita conflito com VESC Tool)
- **Duty maximo** limitado a 8% por padrao (configuravel via env)

## Painel CAN (`can_setup.py`) — porta 5006

Aplicacao web independente para configuracao visual do barramento CAN. Suporta adaptadores CANable com firmware candleLight ou SLCAN via `slcand`. Nao depende de VESC — funciona como ferramenta de diagnostico CAN standalone.

### Funcionalidades

| Funcionalidade | Descricao |
|---|---|
| **Listar dispositivos seriais** | Detecta `/dev/serial/by-id/*`, `/dev/ttyACM*`, `/dev/ttyUSB*` |
| **Subir CAN via SLCAN** | `slcand -o -c -f -sN <port> <iface>` + `ip link set up` |
| **Parar CAN** | `ip link set down` + `pkill slcand` |
| **Enviar frame CAN** | `cansend <iface> <ID>#<HEX_DATA>`, com validacao de ID e dados |
| **Monitor CAN (SSE)** | Server-Sent Events com `candump -tz` em tempo real |
| **Status detalhado** | `ip -details -statistics link show`, estado UP/DOWN/inexistente |
| **Tabela de comandos** | Referencia rapida: `candump`, `cansend`, `slcand`, `ip link` |

### Como rodar

```bash
# Nao requer sudo — usa comandos do sistema para CAN
python can_setup.py
```

Acesse no navegador: **http://<IP_DO_RASPBERRY>:5006**

### Endpoints REST

| Metodo | Rota | Descricao |
|---|---|---|
| `GET` | `/` | Interface web Bootstrap 5 escura |
| `GET` | `/api/devices` | Listar portas seriais disponiveis |
| `GET` | `/api/status?iface=can0` | Status da interface CAN (exists, up, output) |
| `POST` | `/api/start_slcan` | Subir CAN via SLCAN (`port`, `iface`, `bitrate`) |
| `POST` | `/api/stop` | Parar interface CAN (`iface`) |
| `POST` | `/api/send` | Enviar frame CAN (`iface`, `can_id`, `data`) |
| `GET` | `/api/monitor?iface=can0` | SSE stream do candump em tempo real |
| `POST` | `/api/monitor_stop` | Parar monitor SSE |

### Seguranca

- Validacao rigorosa de nome de interface (regex `[a-zA-Z0-9_.:-]{1,32}`)
- Validacao de ID CAN (HEX 1-8 chars) e dados (HEX ate 8 bytes)
- Executa comandos do sistema — rode apenas em rede confiavel

---

## Painel Camera 3D (`3dcam.py`) — porta 5007

Aplicacao web dedicada para a **Orbbec Astra Pro** com streaming MJPEG de RGB e depth colorizado. Usa o OrbbecSDK v1 bridge (subprocesso C++ nativo) para captura de profundidade Y16 com overlay de distancias em tempo real.

### Funcionalidades

| Funcionalidade | Descricao |
|---|---|
| **RGB via OpenCV V4L2** | Streaming MJPEG (`multipart/x-mixed-replace`) com deteccao automatica da Astra Pro HD Camera |
| **Depth via OrbbecSDK v1** | Bridge C++ nativo escreve raw em `/tmp/`. Leitura numpy uint16 com reconexao automatica |
| **Colormap configurável** | TURBO (padrao), JET, VIRIDIS, PLASMA, MAGMA, BONE |
| **Overlay de distancias** | Grid configuravel (1-8), mediana por raio de amostra, centro com crosshair, legenda de cores |
| **Correcoes USB** | autosuspend=-1, usbfs_memory_mb=1000, power/control=on para dispositivos Orbbec (2bc5) |
| **Reinicio limpo do depth** | unbind/bind do dispositivo USB 2bc5:0403, limpeza de `/tmp/`, restart do bridge |
| **Suavizacao** | Median blur 3×3 nas areas sem leitura, contornos "sem leitura" marcados |
| **Interface Bootstrap 5** | Tema escuro GitHub-style, previews lado a lado, todos os controles no painel esquerdo |

### Como rodar

```bash
# Requer sudo para correcoes USB (autosuspend, usbfs_memory_mb, unbind/bind)
sudo python 3dcam.py
```

Acesse no navegador: **http://<IP_DO_RASPBERRY>:5007**

### Endpoints REST

| Metodo | Rota | Descricao |
|---|---|---|
| `GET` | `/` | Interface web com previews lado a lado |
| `GET` | `/api/devices` | Listar dispositivos V4L2 (`v4l2-ctl --list-devices`) |
| `GET` | `/api/status` | Status completo (RGB, bridge, depth, config, logs) |
| `GET` | `/api/depth_stats` | Estatisticas do frame depth atual (min, max, mediana, centro) |
| `POST` | `/api/start` | Iniciar camera (aplica USB fix, inicia bridge, abre RGB) |
| `POST` | `/api/stop` | Parar camera (libera RGB, mata bridge) |
| `POST` | `/api/usb_fix` | Aplicar correcoes USB sem reiniciar camera |
| `POST` | `/api/restart_depth_clean` | Reinicio completo do depth (USB reset + bridge limpo) |
| `GET` | `/video/rgb` | Stream MJPEG do RGB |
| `GET` | `/video/depth` | Stream MJPEG do depth colorizado com overlay |

### Seguranca

- Bridge C++ reiniciado automaticamente se detectado como morto
- Grace period de 90s para primeiro frame do bridge
- Cooldown de 25s entre tentativas de restart
- Fallback para `/dev/video2`, `/dev/video1`, `/dev/video0` se camera principal falhar

---

## Radar LiDAR (`lidar.py`) — porta 5010

Aplicacao web standalone para visualizacao do **LiDAR LDROBOT STL-06P** como radar 2D colorido com preview 3D interativo via Three.js. Nuvem de pontos persistente, deteccao de objetos por cluster, exportacao multi-formato.

### Funcionalidades

| Funcionalidade | Descricao |
|---|---|
| **Radar 2D Canvas** | Nuvem de pontos coloridos por distancia (7 faixas), sweep animado, grid polar com aneis de alcance |
| **Preview 3D Three.js** | Nuvem 3D interativa com OrbitControls (girar, zoom, pan), grid, eixos XYZ, esferas de extremos |
| **Deteccao de objetos** | Agrupamento por cluster (sensibilidade ajustavel 100-1000mm), labels numerados com distancia e angulo |
| **Pontos extremos** | Marcadores visuais e cards para ponto mais proximo (vermelho) e mais longe (roxo) |
| **Controles ajustaveis** | Alcance maximo (500-12000mm), offset angular (-180° a +180°), tamanho dos pontos, intensidade (trail alpha) |
| **Exportacao** | **PNG** (canvas visual), **JSON** (completo com UI settings), **CSV** (coordenadas + cores), **PLY** (nuvem 3D com cores) |
| **Protocolo LDROBOT** | Frames de 47 bytes, CRC XOR, 12 pontos por frame, 230400 baud |
| **Interface responsiva** | Painel esquerdo com todos os controles, preview 3D no canto direito, tema escuro |

### Como rodar

```bash
# Porta serial padrao: /dev/ttyUSB0, baud 230400
python lidar.py

# Com porta e host personalizados
python lidar.py --port /dev/ttyACM0 --baud 230400 --host 0.0.0.0 --flask-port 5010
```

Acesse no navegador: **http://<IP_DO_RASPBERRY>:5010**

### Endpoints REST

| Metodo | Rota | Descricao |
|---|---|---|
| `GET` | `/` | Interface web completa com canvas 2D + preview 3D |
| `GET` | `/api/points` | Nuvem de pontos atual + status (bytes, frames, scan Hz, idade) |

### Exportacao

Todos os formatos usam os pontos visiveis dentro do alcance configurado:

| Formato | Extensao | Conteudo |
|---|---|---|
| **PNG** | `.png` | Screenshot do canvas (imagem visual) |
| **JSON** | `.json` | Completo: pontos (x/y/z em metros, cores RGB, confianca), objetos detectados, UI settings, extremos |
| **CSV** | `.csv` | Tabela: index, angulo, distancia (mm/m), coordenadas (m), cores RGB |
| **PLY** | `.ply` | Nuvem 2D: vertex com x, y, z=0, cores, distancia, angulo, confianca. Abre no MeshLab/CloudCompare/Blender |

---

## Como rodar

O projeto tem **seis aplicacoes independentes**. Rode uma ou mais conforme necessario:

```bash
# Aplicacao principal — gamepad + CAN + LiDAR + GPS + Depth + Follow
sudo python gamepad_web_can_flipsky.py
# Acesse: http://<IP_DO_RASPBERRY>:5005

# Painel CAN — configuracao visual do barramento CAN
python can_setup.py
# Acesse: http://<IP_DO_RASPBERRY>:5006

# Camera 3D — Orbbec Astra Pro RGB + Depth com overlay de distancias
sudo python 3dcam.py
# Acesse: http://<IP_DO_RASPBERRY>:5007

# Monitor VESC + Teste de motor (serial USB, nao requer sudo)
python vesc_read.py
# Acesse: http://<IP_DO_RASPBERRY>:5008

# Controller simplificado — gamepad/teclado + serial/CAN + webcam
python vesc_controller.py
# Acesse: http://<IP_DO_RASPBERRY>:5009

# Radar LiDAR — STL-06P com preview 3D e exportacao
python lidar.py
# Acesse: http://<IP_DO_RASPBERRY>:5010
```

> **Por que sudo?** O `gamepad_web_can_flipsky.py` e o `3dcam.py` precisam executar `ip link set can0 up type can bitrate 500000` para ativar a interface CAN **e** acessar a porta serial `/dev/ttyUSB0` do LiDAR / aplicar correcoes USB (autosuspend, usbfs_memory_mb) para a depth camera. Se for usar apenas o gamepad sem CAN/LiDAR, pode rodar como usuario normal desde que esteja no grupo `input`:
>
> ```bash
> sudo usermod -aG input $USER
> sudo usermod -aG dialout $USER   # para acesso serial (LiDAR)
> sudo reboot
> ```
>
> O `can_setup.py`, `vesc_read.py`, `vesc_controller.py` e `lidar.py` rodam como usuario normal, desde que esteja no grupo `dialout` para acesso a serial `/dev/ttyACM0` ou `/dev/ttyUSB0`. Para modo CAN no `vesc_controller.py`, `sudo` e necessario para `ip link`.

Acesse no navegador: **http://<IP_DO_RASPBERRY>:5005** (gamepad/CAN), **http://<IP_DO_RASPBERRY>:5006** (painel CAN), **http://<IP_DO_RASPBERRY>:5007** (camera 3D), **http://<IP_DO_RASPBERRY>:5008** (monitor VESC), **http://<IP_DO_RASPBERRY>:5009** (controller simplificado) e **http://<IP_DO_RASPBERRY>:5010** (radar LiDAR).

Na inicializacao cada script exibe no terminal as instrucoes de dependencias, diagnostico e comandos uteis.

## Uso passo a passo

### 1. Conectar o gamepad via Bluetooth

1. Abra a pagina no navegador
2. Na secao **Bluetooth**: clique em **Power ON**
3. Clique em **Preparar Agent** (isso carrega os modulos HID, registra o agent NoInputNoOutput e ativa discoverable/pairable)
4. Coloque o controle em modo de pareamento (no Pro Controller, segure o botao pequeno de sync)
5. Clique em **Scan 8 segundos** para encontrar dispositivos
6. Encontre seu controle na lista e clique em **Parear + Trust + Conectar**
7. O controle deve aparecer como **conectado** na secao Status

### 2. Verificar o gamepad

- A secao **Controle visual** mostra em tempo real todos os analogicos, botoes, D-Pad e gatilhos
- Os LEDs do gamepad indicam conexao ativa (no Pro Controller, o LED inferior acende)
- Use a aba **Mapeamento rapido** para associar botoes/eixos a nomes de acao

### 3. Ativar o LiDAR

1. Conecte o LiDAR LDROBOT STL-06P via USB — ele aparece como `/dev/ttyUSB0`
2. Na secao **LiDAR LDROBOT STL-06P**: verifique a porta serial
3. Clique em **Salvar config** para aplicar
4. Se o LiDAR estiver funcionando, o canvas mostrara a nuvem de pontos 2D
5. A metrica **Mais proximo** mostra a distancia do obstaculo mais proximo
6. Aneis vermelho (50cm) e laranja (1m) indicam zonas de perigo ao redor do robo

### 4. Ativar a Depth Camera (Opcional)

1. Conecte a Orbbec Astra Pro via USB
2. O card **Depth Camera Orbbec Astra Pro** mostra a imagem depth colorizada
3. Ajuste as distancias minima e maxima conforme necessario
4. A escala de cores: vermelho = perto, amarelo = medio, verde = longe, azul = muito longe

### 5. Ativar o GPS A76XX

1. Conecte o modem SIMCom A76XX via USB — ele aparece como `/dev/ttyUSB0` a `/dev/ttyUSB3`
2. O GPS liga automaticamente ao iniciar o servidor (`AT+CGNSSPWR=1` → `AT+CGNSSINFO=1`)
3. A secao **GPS A76XX + Trajeto** mostra fix, coordenadas, satelites, mapa e controles
4. Para gravar um trajeto: clique em **Iniciar**, pilote o robo, **Pausar/Retomar** conforme necessario, **Parar** ao finalizar
5. Clique em **Baixar GPX** para exportar no formato universal, ou use o botao **Limpar mapa** para resetar

### 6. Navegacao autonoma (Seguir trajeto)

1. Clique em **Upload GPX** e selecione um arquivo `.gpx` com a rota desejada
2. A rota aparece como polyline **vermelha** no mapa
3. Ajuste as distancias de seguranca (cm) para o desvio de obstaculos
4. Verifique que o checkbox **Desviar de obstaculos (LiDAR)** esta ativo
5. Com o robo **armado** e GPS com **fix 3D**, clique em **Seguir trajeto**
6. O robo navega automaticamente pelos waypoints, desviando de obstaculos
7. Clique em **Parar** a qualquer momento para retomar controle manual

### 7. Configurar e ativar o CAN

1. Na secao **CAN / Robo Flipsky 75100**: clique em **Escanear CANable**
2. Selecione a interface (geralmente `can0`), confira o bitrate (500000)
3. Clique em **Ativar CAN** — isso executa `ip link set can0 up type can bitrate 500000`
4. Ajuste os parametros:
   - **Duty maximo**: comece com `0.25` (25%) e aumente com cuidado
   - **Ganho direcao**: `0.65` e um bom equilibrio
   - **Botao homem-morto**: por padrao `BTN_TR` (botao R do Pro Controller)
5. Clique em **ARMAR robo**

### 8. Controlar o robo

1. Com o robo **armado**, segure o botao homem-morto (R) e mova o analogico esquerdo
2. Cima/baixo = acelerar/re; esquerda/direita = girar
3. Os valores de duty esquerdo e direito aparecem em tempo real na secao CAN
4. Soltar o botao homem-morto **corta imediatamente** a potencia (envia duty 0)
5. Use o **radar LiDAR** no canto direito para visualizar obstaculos ao redor
6. Acompanhe a posicao GPS e grave o trajeto na secao **GPS A76XX + Trajeto**
7. Pressione **B** para freio regenerativo (funciona mesmo sem deadman)
8. Use a **Depth Camera** para visualizar obstaculos em 3D com o colormap JET

### 9. Parada de emergencia

- O botao **PARADA DE EMERGENCIA** desarma o robo e envia `duty = 0` para ambos os motores imediatamente
- Use isso se o robo se comportar de forma inesperada

### 10. Usar o VESC Controller (vesc_controller.py)

1. Acesse **http://<IP_DO_RASPBERRY>:5009**
2. Selecione o modo VESC: **Serial (USB)** ou **CAN**
3. Se modo CAN: configure interface, bitrate, IDs dos motores e clique em **ATIVAR INTERFACE CAN**
4. Selecione o modo de controle: **Gamepad** ou **Teclado**
5. Para gamepad: selecione o dispositivo no dropdown (ou deixe Auto)
6. Para teclado: use setas/WASD para mover, Espaco para freio
7. Opcional: ligue a webcam em **LIGAR CAMERA** e grave com **GRAVAR**
8. Clique em **ARMAR ROBO** para habilitar o controle
9. Monitore a telemetria na coluna esquerda (modo serial) e as barras de duty dos motores

## Interface web — secoes

### gamepad_web_can_flipsky.py (porta 5005)

#### Coluna esquerda (controles)

| Secao | Funcao |
|---|---|
| **Status** | Conexao HID, nome do dispositivo, contagem de botoes/eixos/eventos |
| **Bluetooth** | Power ON/OFF, preparar agent, scan, listar dispositivos, parear/conectar/remover por MAC |
| **CAN / Robo** | Escanear CANable, ativar interface, armar/desarmar, parada de emergencia, parametros do VESC |
| **Dispositivos HID** | Listar /dev/input/event*, selecionar path fixo, filtrar por nome, deadzone |
| **Mapeamento rapido** | Associar codigos de botao/eixo a nomes de acao personalizados |
| **Mapeamentos salvos** | Tabela com todos os mapeamentos, clique para editar |

#### Coluna direita (visualizacao)

| Secao | Funcao |
|---|---|
| **LiDAR LDROBOT STL-06P** | Canvas 420×420px com nuvem de pontos 2D persistente (3s fading). Aneis de perigo 50cm/1m. Metricas: conexao, nº pontos, obstaculo mais proximo, RPM, FPS. Legendas de cor por distancia |
| **Depth Camera Orbbec Astra Pro** | Imagem depth 640×480 com colormap JET em tempo real (~10 fps). Metricas: conexao, FPS, distancia minima/maxima. Inputs para ajustar range de profundidade. Legenda de cores (perto/medio/longe/muito longe) |
| **GPS A76XX + Trajeto** | Mapa Leaflet com OpenStreetMap, marcador de posicao (azul), polyline do trajeto gravado (verde) e trajeto carregado (vermelho). Metricas: fix (NONE/2D/3D), satelites usados/visiveis, HDOP, lat/lng, altitude, velocidade, rumo, UTC. Canvas de constelacao de satelites. Controles de gravacao: Iniciar/Pausar/Retomar/Parar. Download GPX/JSON, Upload GPX, Limpar mapa, Ligar/Desligar GPS, Seguir trajeto. Log proprio |
| **Controle visual** | Gamepad visual interativo (compacto): botoes A/B/X/Y, D-Pad, analogicos (sticks), gatilhos (triggers), L/R/ZL/ZR, SELECT/HOME/START |
| **Ultimo evento** | JSON do ultimo evento recebido do gamepad |
| **Tabelas tecnicas** | Lista detalhada de todos os botoes e eixos (dentro do Controle visual, toggle) |
| **Log em tempo real** | Ultimos 300 eventos de gamepad + acoes do usuario |

### vesc_controller.py (porta 5009)

| Secao | Funcao |
|---|---|
| **Camera** | Streaming JPEG da webcam USB. Selecao de dispositivo, ligar/desligar, gravar MP4, lista de gravacoes com download |
| **Telemetria VESC** | Grid 3×3 com tensao, corrente entrada, corrente motor, RPM, duty (%), potencia (W), temp FET, temp motor, fault. Barras de duty por motor (2 ou 4 motores) |
| **Gamepad** | Selecao de dispositivo HID. Sticks visuais (esquerdo/direito) com indicador de posicao. Barras de throttle e steering. Indicador homem-morto |
| **Controle** | Seletor de modo (serial/CAN), config de porta serial ou parametros CAN (interface, bitrate, IDs, nº motores). Botoes ARMAR/DESARMAR/PARADA DE EMERGENCIA |

### can_setup.py (porta 5006)

| Secao | Funcao |
|---|---|
| **Configuracao SLCAN** | Dropdown de dispositivos seriais (by-id, ttyACM, ttyUSB), nome da interface CAN, seletor de bitrate (125k–1M). Botoes Subir CAN / Parar CAN |
| **Enviar frame CAN** | Inputs para ID (HEX) e dados (HEX), botao Enviar com feedback no monitor |
| **Monitor CAN** | SSE stream do `candump -tz` em tempo real. Botoes Iniciar/Parar/Limpar |
| **Status detalhado** | Pre com saida de `ip -details -statistics link show`. Badge de status (UP/DOWN/inexistente) |
| **Tabela de comandos** | Referencia rapida: `ip link`, `candump`, `cansend`, `slcand` |

### 3dcam.py (porta 5007)

| Secao | Funcao |
|---|---|
| **Controles** | Dropdown de camera RGB com `v4l2-ctl --list-devices`, resolucao e FPS, JPEG quality, range de profundidade (near/far mm) |
| **Depth / distancia** | Escala do preview, grade de distancias, raio de amostra, mapa de cor (6 opcoes), direcao da cor (perto quente/frio) |
| **Opcoes visuais** | Mirror RGB/Depth, labels de distancia, suavizacao (median blur), legenda de cores, contornos de areas sem leitura, modo cinza |
| **Preview RGB** | Stream MJPEG da camera RGB (lado esquerdo) |
| **Preview Depth** | Stream MJPEG do depth colorizado com overlay de distancias, crosshair no centro, legendas (lado direito) |
| **Status** | Pre com estado completo: bridge PID/runtime, raw/meta age/size, ultimos erros, tail do log do bridge (8KB) |
| **Botoes de acao** | Iniciar camera, Parar camera, Aplicar correcao USB, Reiniciar depth limpo, Atualizar status |

### lidar.py (porta 5010)

| Secao | Funcao |
|---|---|
| **Status do LiDAR** | Serial (conectada/desconectada), bytes recebidos, pacotes validos/invalidos, pontos atuais, objetos detectados, rotacao Hz, idade do ultimo pacote |
| **Pontos extremos** | Cards para ponto mais proximo (vermelho) e mais longe (roxo) com distancia formatada, angulo e confianca |
| **Controles ajustaveis** | Alcance maximo (500-12000mm), offset angular (-180° a +180°), tamanho dos pontos (1-9px), intensidade visual (0.04-0.50), checkboxes para regua/objetos/linhas |
| **Deteccao de objetos** | Maximo de objetos marcados (5-80), sensibilidade de agrupamento (100-1000mm) |
| **Exportacao** | Botoes PNG, JSON, CSV, PLY com download automatico (timestamp no nome do arquivo) |
| **Legenda de distancia** | 7 faixas de cor: 0-0.5m (vermelho), 0.5-1m (laranja), 1-2m (amarelo), 2-4m (verde), 4-6m (ciano), 6-8m (azul), >8m (roxo) |
| **Preview 3D** | Three.js com OrbitControls. Esferas de extremos (vermelha/roxa), grid 16m, eixos XYZ. 10 botoes de camera predefinida (topo/lateral/frente/traseira/esquerda/direita/isometrica/1ª pessoa/ver perto/ver longe/reset). Tamanho 3D dos pontos e altura visual Z ajustaveis |

## Configuracao

### gamepad_web_can_flipsky.py

Toda a configuracao e salva automaticamente em `gamepad_config.json` (mesmo diretorio do script). Ao rodar pela primeira vez, o arquivo e criado com valores padrao.

```json
{
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
    "invert_throttle": true,
    "invert_steering": false,
    "invert_left": false,
    "invert_right": true,
    "require_deadman": true,
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
    "enabled": true,
    "min_depth_mm": 500,
    "max_depth_mm": 8000,
    "emit_fps": 10
  }
}
```

### Parametros CAN

| Parametro | Padrao | Descricao |
|---|---|---|
| `interface` | `can0` | Nome da interface SocketCAN |
| `bitrate` | `500000` | Taxa do barramento (125k, 250k, 500k, 1M) |
| `left_id` | `1` | ID CAN do VESC esquerdo (0-255) |
| `right_id` | `2` | ID CAN do VESC direito (0-255) |
| `max_duty` | `0.25` | Duty cycle maximo (0.01 a 0.95). Aumente com cuidado |
| `steering_gain` | `0.65` | Quanto o steering influencia a diferenca entre motores (0 a 1) |
| `throttle_axis` | `ABS_Y` | Eixo usado para acelerar/re |
| `steering_axis` | `ABS_X` | Eixo usado para direcao |
| `invert_throttle` | `true` | Inverte o acelerador (normalmente ABS_Y vai negativo ao empurrar pra cima) |
| `invert_steering` | `false` | Inverte direcao |
| `invert_left` | `false` | Inverte o motor esquerdo |
| `invert_right` | `true` | Inverte o motor direito |
| `require_deadman` | `true` | Exige botao homem-morto pressionado |
| `deadman_button` | `BTN_TR` | Botao homem-morto (R no Pro Controller) |
| `send_interval` | `0.05` | Intervalo minimo entre envios CAN (em segundos) |
| `brake_button` | `BTN_SOUTH` | Botao do freio regenerativo (B no Pro Controller) |
| `brake_current` | `8.0` | Corrente de freio em Amperes (0–200) |

### Parametros LiDAR

| Parametro | Padrao | Descricao |
|---|---|---|
| `port` | `/dev/ttyUSB0` | Porta serial do LiDAR |
| `baudrate` | `230400` | Baud rate (STL-06P usa 230400) |
| `min_distance` | `150` | Distancia minima de deteccao (mm). Pontos abaixo disso sao ignorados |
| `max_distance` | `12000` | Distancia maxima de deteccao (mm). Pontos acima sao ignorados |
| `emit_interval` | `0.08` | Intervalo minimo entre envios de frame para o frontend (segundos) |

### Parametros GPS

| Parametro | Padrao | Descricao |
|---|---|---|
| `at_port` | `/dev/ttyUSB1` | Porta serial para comandos AT e streaming GPS (ttyUSB0-3 do modem) |
| `at_baudrate` | `115200` | Baud rate da porta AT |
| `emit_interval` | `1.0` | Intervalo minimo entre atualizacoes de status (segundos) |

### Parametros Depth Camera

| Parametro | Padrao | Descricao |
|---|---|---|
| `enabled` | `true` | Habilita/desabilita a camera depth na inicializacao |
| `min_depth_mm` | `500` | Distancia minima do colormap (mm). Abaixo disso = vermelho |
| `max_depth_mm` | `8000` | Distancia maxima do colormap (mm). Acima disso = azul escuro |
| `emit_fps` | `10` | Taxa de envio de frames para o frontend (3–30) |

### Parametros Follow (navegacao autonoma)

| Parametro | Padrao | Descricao |
|---|---|---|
| `waypoint_threshold` | `2.0` | Distancia (m) para considerar waypoint atingido |
| `max_auto_speed` | `0.15` | Duty cycle maximo no modo autonomo (0–1) |
| `steering_kp` | `0.5` | Ganho proporcional do controle de direcao |
| `avoidance_weight` | `0.5` | Blend entre path following e obstacle avoidance (0=so path, 1=so desvio) |
| `safe_distance_mm` | `500` | Distancia (mm) abaixo da qual comeca a desviar de obstaculos |
| `critical_distance_mm` | `300` | Distancia (mm) abaixo da qual ocorre parada de emergencia |
| `avoidance_enabled` | `true` | Habilita/desabilita desvio de obstaculos via LiDAR |

### Filtro de dispositivos HID

- **`device_path`**: caminho fixo para um `/dev/input/eventX` especifico (ex: `/dev/input/event4`)
- **`device_name_contains`**: filtra dispositivos cujo nome contem o termo (ex: `Pro Controller`)
- **`deadzone`**: zona morta dos analogicos (0.0 a 0.5)

> Se ambos estiverem preenchidos, `device_path` tem prioridade.

### Mapeamentos

Os mapeamentos associam codigos de botao/eixo a nomes de acao que aparecem na interface. Exemplos:

```
BTN_SOUTH    → Freio
BTN_NORTH    → Farol
ABS_RZ       → Velocidade maxima
```

## VESC Tool (vesc_tool_free_linux/)

O diretorio `vesc_tool_free_linux/` contem:

| Arquivo | Descricao |
|---|---|
| `vesc_tool_6.06` | Binario do VESC Tool 6.06 para Linux (.gitignored) |
| `vesc_appconf.xml` | Configuracao de exemplo — parametros de aplicacao do VESC (CAN ID, timeout, baud rate, etc.) |
| `vesc_mcconf.xml` | Configuracao de exemplo — parametros do motor (correntes, limites, sensores, etc.) |

O VESC Tool e a ferramenta oficial para configuracao dos controladores Flipsky/VESC. Use-o para:
- Configurar parametros do motor (tipo, corrente maxima, limites de tensao)
- Configurar parametros CAN (ID, baud rate, status rate)
- Atualizar firmware
- Diagnostico em tempo real

Os arquivos XML de exemplo podem ser carregados no VESC Tool ou visualizados via `vesc_read.py` (endpoint `/api/real-config/upload`).

## Diagnostico e troubleshooting

### Bluetooth conecta mas gamepad nao aparece

```bash
cat /proc/bus/input/devices          # O controle aparece?
ls -l /dev/input/event*              # Existem dispositivos de input?
modinfo hid-nintendo                 # Modulo existe?
sudo modprobe hidp                   # Carregar modulo HID
sudo modprobe hid-nintendo           # Carregar driver Nintendo
```

Tambem use o botao **Diagnostico HID** na interface web — ele mostra `/proc/bus/input/devices`, dispositivos evdev, e status dos modulos.

### Permissao negada ao acessar /dev/input/eventX

```bash
sudo usermod -aG input $USER
sudo reboot
```

### LiDAR nao aparece / porta serial nao encontrada

```bash
ls -l /dev/ttyUSB*                   # A porta aparece?
ls -l /dev/ttyACM*                   # Alguns adaptadores usam ttyACM
sudo usermod -aG dialout $USER       # Permissao para porta serial
sudo reboot
dmesg | grep -i usb                  # Verificar se o kernel reconheceu o dispositivo
```

O status do LiDAR aparece no card (OFF → ON quando conectado). Se mostrar `pyserial nao disponivel`:

```bash
pip install pyserial
```

### GPS nao mostra dados

```bash
ls -l /dev/ttyUSB*                   # O modem aparece?
sudo picocom -b 115200 /dev/ttyUSB1  # Testar comandos AT
AT+CGNSSPWR?                         # Verificar se GPS esta ligado
AT+CGNSSINFO                         # Consultar posicao manualmente
sudo cat /dev/ttyUSB3                # Verificar se tem NMEA em outra porta
```

### Depth Camera nao conecta

```bash
lsusb | grep -i orbbec               # Camera reconhecida pelo kernel?
python3 -c "from ob_depth import DepthCamera; DepthCamera()"  # Testar wrapper
```

> A camera so funciona em **Raspberry Pi (aarch64/arm64)**. Em x86_64, `ob_depth.py` exibe "Execute no Raspberry Pi". Verifique tambem que o SDK foi clonado com `git clone` na raiz do projeto.

### Webcam nao funciona (vesc_controller.py)

```bash
ls -l /dev/video*                    # Dispositivos de video existem?
v4l2-ctl --list-devices              # Listar cameras reconhecidas
python3 -c "import cv2; print(cv2.__version__)"  # OpenCV instalado?
```

Se OpenCV nao estiver disponivel, a secao Camera sera desabilitada:

```bash
pip install opencv-python
```

### Freio regenerativo nao funciona

```bash
# Verificar se o VESC responde ao comando de brake (via CAN)
# O botao B (BTN_SOUTH) precisa estar mapeado corretamente
# Verifique no painel visual se o botao B acende ao pressionar
```

### CAN nao funciona

```bash
ip -details link show can0           # Interface existe?
sudo ip link set can0 up type can bitrate 500000   # Ativar manualmente
candump can0                         # Verificar trafego
```

Certifique-se de que o CANable esta com firmware **candleLight** (nao slcan). Com firmware slcan, o adaptador aparece como `/dev/ttyACM*` e precisa do `slcand` para criar a interface CAN.

### Erro "evdev nao disponivel"

```bash
pip install evdev
# ou
sudo apt install python3-evdev
```

## Seguranca

- **Nunca teste com o robo no chao na primeira vez** — mantenha-o suspenso
- Comece com `max_duty` baixo (0.10 a 0.25) e aumente gradualmente
- O botao homem-morto (`BTN_TR` / R) **precisa** estar segurado para qualquer movimento
- A **PARADA DE EMERGENCIA** desarma o robo e zera o duty de ambos os motores
- Sempre tenha um kill switch fisico nos VESCs como redundancia
- O LiDAR e **apenas visualizacao** (no `gamepad_web_can_flipsky.py`, exceto durante follow autonomo com obstacle avoidance)
- O GPS e um modem SIMCom A76XX — certifique-se de que a porta AT configurada corresponde a porta correta do modem
- O **freio regenerativo** (botao B) funciona mesmo sem deadman — e uma camada extra de seguranca
- O **follow autonomo** requer robo armado + fix GPS 3D. O **obstacle avoidance** para o robo automaticamente se detectar obstaculo a menos de 30cm na frente
- No `vesc_controller.py` modo teclado, inatividade > 0.3s corta potencia automaticamente

## Arquitetura do codigo

O projeto consiste em **seis aplicacoes Flask independentes** mais `ob_depth.py` (wrapper ctypes, ~234 linhas) e arquivos de suporte:

```
pandorapi/
├── gamepad_web_can_flipsky.py   ← app principal (~7061 linhas) — porta 5005
├── can_setup.py                  ← painel CAN (~725 linhas) — porta 5006
├── 3dcam.py                      ← camera 3D Astra Pro (~2042 linhas) — porta 5007
├── vesc_read.py                  ← monitor VESC (~3440 linhas) — porta 5008
├── vesc_controller.py            ← controller simplificado (~1966 linhas) — porta 5009
├── lidar.py                      ← radar LiDAR STL-06P (~2482 linhas) — porta 5010
├── ob_depth.py                   ← wrapper ctypes p/ Orbbec Astra Pro
├── OrbbecSDK/                    ← SDK Orbbec (clonado, .gitignored)
│   └── lib/arm64/*.so            ← bibliotecas nativas ARM
├── lib/arm64/                    ← backup das .so
├── vesc_tool_free_linux/         ← VESC Tool 6.06 + XMLs de exemplo
│   ├── vesc_tool_6.06            ← binario VESC Tool (.gitignored)
│   ├── vesc_appconf.xml          ← config app VESC de exemplo
│   └── vesc_mcconf.xml           ← config motor VESC de exemplo
├── recordings/                   ← gravacoes de video (MP4) (.gitignored)
├── vesc_real_configs/            ← XMLs de config do VESC (criado em runtime)
├── assets/screenshot.png
├── gamepad_config.json           ← config gerada automaticamente
├── LICENSE                       ← MIT License
└── README.md

gamepad_web_can_flipsky.py
├── Configuracao (DEFAULT_CONFIG, load/save)
├── Mapeamento de codigos evdev (CODE_FALLBACK_NAMES, FRIENDLY_NAMES)
├── HTML_PAGE (template inline com CSS + JS completo)
├── Funcoes auxiliares de sistema (modulos, /proc, /dev/input)
├── Gamepad reader (evdev, normalizacao de eixos, eventos)
├── GPS reader (pyserial, AT commands, parse CGNSSINFO, trajeto)
├── GPS follow + obstacle avoidance (haversine, bearing, LiDAR sectors)
├── GPX parser/export (parse_gpx_xml, trajectory_to_gpx)
├── Depth Camera reader (ctypes wrapper, colormap JET, JPEG base64)
├── LiDAR reader (pyserial, protocolo LDROBOT 0x54, CRC, parse)
├── Bluetooth (bluetoothctl interativo via subprocess.Popen)
├── CAN / SocketCAN (PF_CAN socket, duty, current brake, quadros estendidos)
├── Logica do robo (deadman, brake, throttle+steering→duty left/right)
├── Rotas Flask (~55 endpoints REST + Socket.IO)
└── Entry point (5 threads + socketio.run na porta 5005)

vesc_controller.py
├── Gamepad reader (evdev, normalizacao, auto-detect, selecao manual)
├── VESC serial telemetry (pyvesc, convert_measurements, fault codes)
├── CAN / SocketCAN (PF_CAN socket, duty, current brake, 2-4 motores)
├── Modo teclado (WASD/setas + timeout inatividade 0.3s)
├── Camera USB (OpenCV, VideoCapture, JPEG base64, gravacao MP4)
├── Logica do robo (deadman, brake, throttle+steering→duty left/right)
├── Rotas Flask (~20 endpoints REST + Socket.IO)
└── Entry point (3 threads + socketio.run na porta 5009)

vesc_read.py
├── Telemetria (pyvesc, VESC.get_measurements(), convert_measurements)
├── Fault codes (19 codigos com nome + auto-stop)
├── Teste de motor (command queue, duty/freio regenerativo, auto-stop)
├── TCP Bridge (serial→TCP para VESC Tool, 2 threads bidirecionais)
├── XML Config (parse de app_config.xml + motor_config.xml, flat/compact)
├── Bateria (estimativa % via cutoff_start/end da config real)
├── Historico (deque maxlen=600, timestamp + time_label)
├── Rotas Flask (~15 endpoints REST)
└── Template inline com Chart.js (graficos de tensao, corrente, RPM, duty, potencia, temperatura)

can_setup.py
├── Listagem de dispositivos seriais (/dev/serial/by-id/*, ttyACM*, ttyUSB*)
├── Comandos de sistema (modprobe, pkill, slcand, ip link, cansend, candump)
├── Validacao (regex para iface, ID CAN, dados CAN)
├── Monitor SSE (candump -tz → EventStream para o navegador)
├── Rotas Flask (~7 endpoints REST + 1 SSE)
└── Template inline Bootstrap 5 escuro (~420 linhas HTML/CSS/JS)

3dcam.py
├── RGB via OpenCV V4L2 (MJPG fourcc, auto-deteccao Astra Pro HD Camera)
├── Depth via OrbbecSDK v1 bridge (subprocesso C++ → /tmp/*.raw + *.json)
├── Colormap + overlay (TURBO/JET/etc, legendas, grid de distancias, contornos)
├── Correcoes USB (autosuspend, usbfs_memory_mb, power/control, unbind/bind)
├── Threads: rgb_worker + depth_worker (bridge manager com restart automatico)
├── Rotas Flask (~10 endpoints REST + 2 streams MJPEG)
└── Template inline Bootstrap 5 escuro (~597 linhas HTML/CSS/JS)

lidar.py
├── Serial reader (pyserial, protocolo LDROBOT 0x54, CRC XOR, 47-byte frames)
├── Canvas 2D (radar com sweep, grid polar, aneis de distancia, pontos coloridos)
├── Deteccao de objetos (cluster por distancia/angulo, labels numerados)
├── Preview 3D (Three.js + OrbitControls, esferas de extremos, grid, eixos)
├── Exportacao (PNG canvas, JSON completo, CSV tabela, PLY nuvem 3D)
├── Rotas Flask (~2 endpoints REST)
└── Template inline completo (~2248 linhas HTML/CSS/JS) com Three.js CDN
```

### Threads — gamepad_web_can_flipsky.py

- **Main thread**: servidor Flask + Socket.IO
- **gamepad_reader_loop** (daemon): loop infinito lendo eventos do `/dev/input/eventX`, reconectando automaticamente se o dispositivo sumir
- **gps_reader_loop** (daemon): abre porta AT, envia `AT+CGNSSPWR=1` + `AT+CGNSSINFO=1`, faz parsing continuo de `+CGNSSINFO:`, emite status, grava trajeto e executa follow autonomo com obstacle avoidance
- **depth_camera_loop** (daemon): abre camera depth via `ob_depth.py` (ctypes), captura frames 640×480 Y16, aplica colormap JET via OpenCV, comprime JPEG e emite via Socket.IO a ~10 fps
- **lidar_reader_loop** (daemon): abre porta serial, faz parsing do protocolo LDROBOT, acumula pontos por angulo (deduplicacao por confianca), emite frames a cada ~80ms e alimenta o obstacle avoidance do follow

### Threads — vesc_controller.py

- **Main thread**: servidor Flask + Socket.IO
- **gamepad_reader_loop** (daemon): loop infinito lendo eventos do `/dev/input/eventX`, reconectando automaticamente se o dispositivo sumir. Suporta selecao manual de dispositivo
- **vesc_reader_loop** (daemon): modo serial — leitura de telemetria + envio de duty/freio via pyvesc com auto-stop por fault. Modo CAN — nao usado (CAN envia duty sob demanda no `control_loop_step`)
- **camera_loop** (daemon): abre webcam via OpenCV (`/dev/videoX`), captura JPEG, emite via Socket.IO, grava MP4 opcionalmente na pasta `recordings/`

### Threads — vesc_read.py

- **Main thread**: servidor Flask
- **vesc_reader_loop** (daemon): loop infinito lendo telemetria do VESC via serial USB com pyvesc, processa comandos de motor test, mantem auto-stop por tempo/fault, escreve no deque de historico
- **TCP Bridge threads** (2 daemon, sob demanda): `bridge_socket_to_serial` e `bridge_serial_to_socket` — ponte bidirecional serial↔TCP para uso simultaneo com VESC Tool

### Threads — can_setup.py

- **Main thread**: servidor Flask
- Sem threads adicionais — opera sob demanda via chamadas REST. O `candump` e lancado como subprocesso com `subprocess.Popen` e transmitido via SSE (Server-Sent Events)

### Threads — 3dcam.py

- **Main thread**: servidor Flask
- **rgb_worker** (daemon): loop infinito lendo frames do OpenCV (`/dev/video2`), aplica mirror se configurado, atualiza `latest_rgb`
- **depth_worker** (daemon): gerencia o bridge OrbbecSDK (subprocesso C++), monitora arquivos `/tmp/orbbec_depth.raw` e `.meta.json`, aplica colormap + overlay de distancias, atualiza `latest_depth_colored`. Auto-restart do bridge com cooldown de 25s

### Threads — lidar.py

- **Main thread**: servidor Flask
- **serial_worker** (daemon): loop infinito lendo frames de 47 bytes do protocolo LDROBOT via pyserial (`/dev/ttyUSB0`, 230400 baud). Parse CRC XOR, atualiza bins de angulo (720 bins, 0.5° cada), reconecta automaticamente em caso de erro serial

### Comunicacao

**gamepad_web_can_flipsky.py:**

- **Browser ↔ Servidor**: Socket.IO para eventos em tempo real (status do gamepad, status CAN, eventos de botao/eixo, status GPS, pontos de trajeto, nuvem de pontos LiDAR, frame depth, follow status)
- **Browser ↔ Servidor**: REST API (~55 endpoints) para configuracao, Bluetooth, CAN, LiDAR, GPS, trajeto, follow, depth camera
- **Servidor ↔ CAN bus**: socket `PF_CAN` raw — `CAN_PACKET_SET_DUTY` (ID 0), `CAN_PACKET_SET_CURRENT_BRAKE` (ID 2), quadros CAN estendidos (29-bit ID)
- **Servidor ↔ Bluetooth**: `subprocess.Popen` com `bluetoothctl` em modo interativo (stdin/stdout)
- **Servidor ↔ LiDAR**: `pyserial` na porta `/dev/ttyUSB0` a 230400 baud
- **Servidor ↔ Modem A76XX**: `pyserial` na porta `/dev/ttyUSB1` a 115200 baud (AT commands + streaming GPS)
- **Servidor ↔ Depth Camera**: `ob_depth.py` (ctypes) → `libOrbbecSDK.so` via USB

**can_setup.py:**

- **Browser ↔ Servidor**: REST API para configuracao CAN, envio de frames, status da interface
- **Browser ↔ Servidor**: SSE (Server-Sent Events) para monitor de trafego CAN em tempo real (`candump -tz`)
- **Servidor ↔ CAN bus**: `subprocess.Popen` com `slcand`, `cansend`, `candump`, `ip link`

**3dcam.py:**

- **Browser ↔ Servidor**: REST API para controle da camera, status, estatisticas de depth
- **Browser ↔ Servidor**: MJPEG (`multipart/x-mixed-replace`) para streams de RGB e depth
- **Servidor ↔ RGB Camera**: OpenCV V4L2 via `/dev/video2` (MJPG fourcc)
- **Servidor ↔ Depth Camera**: Subprocesso C++ (OrbbecSDK v1 bridge) escrevendo em `/tmp/` + leitura via numpy

**lidar.py:**

- **Browser ↔ Servidor**: REST API para nuvem de pontos e status do LiDAR
- **Servidor ↔ LiDAR**: `pyserial` na porta `/dev/ttyUSB0` a 230400 baud (protocolo LDROBOT 0x54)
- **Browser ↔ Three.js CDN**: Carrega `three.min.js` e `OrbitControls.js` do CDN para preview 3D

**vesc_controller.py:**

- **Browser ↔ Servidor**: Socket.IO para eventos em tempo real (status do gamepad, telemetria VESC, status controle, eventos de botao/eixo, frames da camera)
- **Browser ↔ Servidor**: REST API (~20 endpoints) para arm/desarm, config CAN, modo VESC, modo controle, camera, teclado
- **Servidor ↔ CAN bus** (modo CAN): socket `PF_CAN` raw via `socketcan_send_extended()` — duty cycle e current brake para 2-4 VESCs
- **Servidor ↔ VESC Serial** (modo serial): `pyvesc` via USB serial — get_measurements, set_duty_cycle, set_current_brake
- **Servidor ↔ Gamepad**: `evdev` no `/dev/input/eventX` — botoes e eixos normalizados
- **Servidor ↔ Webcam**: `OpenCV VideoCapture` via `/dev/videoX` — captura JPEG + gravacao MP4

**vesc_read.py:**

- **Browser ↔ Servidor**: REST API (~15 endpoints) para telemetria, motor test, TCP bridge, config XML
- **Servidor ↔ VESC**: `pyvesc` via serial USB (`/dev/ttyACM0`) — protocolo VESC (get_measurements, set_duty_cycle, set_current, set_current_brake, get_firmware_version)
- **TCP Bridge**: `socket.socket` (AF_INET, SOCK_STREAM) — ponte transparente serial↔TCP na porta 65102 para VESC Tool

### Formato do quadro CAN

Cada comando de duty cycle e enviado como:
- **ID CAN estendido**: `(CAN_PACKET_SET_DUTY << 8) | vesc_id`
- **Dados (4 bytes big-endian)**: `duty × 100000` como int32
- Via socket `PF_CAN` com flag `CAN_EFF_FLAG`

O `vesc_controller.py` usa o mesmo formato, com funcoes identicas (`socketcan_send_extended`, `vesc_send_duty`, `vesc_send_current_brake_can`).

### Protocolo LiDAR LDROBOT STL-06P

```
Packet: [0x54][VerLen][Speed LE 2B][StartAngle LE 2B][Data N×(Dist LE 2B + Conf 1B)][EndAngle LE 2B][Timestamp LE 2B][CRC 1B]

VerLen   = (data_type << 5) | n_points     (tipicamente 0x2C = 12 pontos)
Speed    = velocidade de rotacao (graus/s)
Angle    = angulo × 100 (0-35999)
Distance = distancia em mm (uint16 little-endian)
Conf     = confianca/intensidade (0-255)
CRC      = XOR de todos os bytes anteriores
```

O parser (`parse_lidar_packet`) busca o header `0x54`, valida CRC, converte coordenadas polares para cartesianas e descarta pontos com `distance = 0`. Pontos sao acumulados no frontend por 3 segundos com fading progressivo.

## Licenca

MIT — veja o arquivo [LICENSE](LICENSE).
