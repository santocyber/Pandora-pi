# PandoraPi — Interface Web para Gamepad Bluetooth + Controle CAN (Flipsky 75100 VESC) + LiDAR

Aplicação web que transforma um Raspberry Pi em uma central de controle para robôs com tração diferencial usando controladores **Flipsky 75100 (VESC)** conectados via barramento **CAN**. O comando é feito por **gamepad Bluetooth HID** (Nintendo Switch Pro Controller ou qualquer controle compatível com evdev). **LiDAR LDROBOT STL-06P** integrado como radar de navegação com nuvem de pontos 2D persistente (time-decay de 3 segundos).

Todo o sistema roda como um único script Python — Flask + Socket.IO no backend, HTML/CSS/JS inline no frontend, leitura de gamepad via `evdev`, envio de quadros CAN raw via SocketCAN e leitura serial do LiDAR via `pyserial`.

## Hardware necessário

| Componente | Detalhes |
|---|---|
| Raspberry Pi | Com Bluetooth integrado (ou dongle USB) |
| CANable | Adaptador USB-CAN com firmware **candleLight** (recomendado). Com firmware slcan, é necessário configurar `slcand` manualmente antes de usar |
| 2× Flipsky 75100 VESC | IDs CAN 1 (motor esquerdo) e 2 (motor direito) |
| Gamepad Bluetooth | Nintendo Switch Pro Controller (testado) ou qualquer controle HID reconhecido pelo kernel Linux via evdev |
| LiDAR LDROBOT STL-06P | Conectado via USB serial (aparece como `/dev/ttyUSB0`). Baud rate 230400 |

## Como funciona

```
Navegador (http://IP_DO_PI:5005)
        │
        ▼ Socket.IO (WebSocket)
┌───────────────────────────────┐
│         Flask Server          │
│  - Rotas REST de configuração │
│  - Socket.IO para eventos     │
│    em tempo real              │
├───────────────────────────────┤
│  Thread: gamepad_reader_loop  │
│  Lê /dev/input/eventX (evdev) │──► Gamepad Bluetooth (HID)
│  Emite eventos via Socket.IO  │
├───────────────────────────────┤
│  Thread: lidar_reader_loop    │
│  Lê /dev/ttyUSB0 (pyserial)   │──► LiDAR LDROBOT STL-06P
│  Protocolo 0x54, CRC, 230400  │
│  Emite nuvem pontos via S.IO  │
├───────────────────────────────┤
│  CAN / SocketCAN (PF_CAN)     │
│  Envia quadros CAN estendidos │──► CANable ──► Flipsky VESC 1 + 2
│  com duty cycle (-1.0 a 1.0)  │
└───────────────────────────────┘
```

### Matemática dos motores

```
throttle = ABS_Y  (analógico esquerdo, cima/baixo)
steering = ABS_X  (analógico esquerdo, esquerda/direita)

left  = (throttle + steering × steering_gain) × max_duty
right = (throttle - steering × steering_gain) × max_duty
```

Os valores são saturados entre `[-1.0, 1.0]` e multiplicados pelo `max_duty` configurável. O botão homem-morto (`BTN_TR` / botão R) precisa estar pressionado para enviar qualquer potência aos motores.

### Radar LiDAR — persistência visual (time-decay)

Os pontos do LiDAR acumulam por **3 segundos** com fading progressivo. Cada ponto recebe um timestamp no recebimento e sua opacidade decai linearmente:

```
alpha = alpha_base × (1 - idade_ms / 3000)

0s: 100% opaco (cor viva)
1s: 66% opaco
2s: 33% opaco
3s: removido
```

Isso cria um **"rastro" visual** ao redor do robô, permitindo ao operador enxergar obstáculos mesmo quando o feixe do LiDAR não está apontando diretamente para eles. Anéis de perigo (50cm vermelho tracejado, 1m laranja tracejado) indicam zonas de alerta. O card mostra a distância do obstáculo mais próximo em tempo real.

O sistema funciona **sem odometria** — assume que o robô se move devagar o suficiente para o histórico de 3 segundos ainda ser útil. Para maior precisão com o robô em movimento rápido, seria necessário dead reckoning via encoders (não implementado).

## Dependências

### Sistema (pacotes apt)

```bash
sudo apt install -y bluetooth bluez evtest joystick can-utils iproute2
```

### Módulos do kernel

```bash
sudo modprobe hidp
sudo modprobe hid-nintendo
```

> Se `hid-nintendo` não existir no seu kernel, atualize o kernel ou use outro gamepad HID genérico.

### Python

```bash
pip install flask flask-socketio evdev pyserial
```

> O `pyserial` é opcional — se ausente, o LiDAR simplesmente não funciona, mas o gamepad e CAN continuam operando normalmente.

## Como rodar

```bash
# Dentro da pasta do projeto
sudo python gamepad_web_can_flipsky.py
```

> **Por que sudo?** O script precisa executar `ip link set can0 up type can bitrate 500000` para ativar a interface CAN **e** acessar a porta serial `/dev/ttyUSB0` do LiDAR. Se for usar apenas o gamepad sem CAN/LiDAR, pode rodar como usuário normal desde que esteja no grupo `input`:
>
> ```bash
> sudo usermod -aG input $USER
> sudo usermod -aG dialout $USER   # para acesso serial (LiDAR)
> sudo reboot
> ```

Acesse no navegador: **http://<IP_DO_RASPBERRY>:5005**

Na inicialização o script exibe no terminal todas as instruções de dependências, diagnóstico e comandos úteis.

## Uso passo a passo

### 1. Conectar o gamepad via Bluetooth

1. Abra a página no navegador
2. Na seção **Bluetooth**: clique em **Power ON**
3. Clique em **Preparar Agent** (isso carrega os módulos HID, registra o agent NoInputNoOutput e ativa discoverable/pairable)
4. Coloque o controle em modo de pareamento (no Pro Controller, segure o botão pequeno de sync)
5. Clique em **Scan 8 segundos** para encontrar dispositivos
6. Encontre seu controle na lista e clique em **Parear + Trust + Conectar**
7. O controle deve aparecer como **conectado** na seção Status

### 2. Verificar o gamepad

- A seção **Controle visual** mostra em tempo real todos os analógicos, botões, D-Pad e gatilhos
- Os LEDs do gamepad indicam conexão ativa (no Pro Controller, o LED inferior acende)
- Use a aba **Mapeamento rápido** para associar botões/eixos a nomes de ação

### 3. Ativar o LiDAR

1. Conecte o LiDAR LDROBOT STL-06P via USB — ele aparece como `/dev/ttyUSB0`
2. Na seção **LiDAR LDROBOT STL-06P**: verifique a porta serial
3. Clique em **Salvar config** para aplicar
4. Se o LiDAR estiver funcionando, o canvas mostrará a nuvem de pontos 2D
5. A métrica **Mais proximo** mostra a distância do obstáculo mais próximo
6. Anéis vermelho (50cm) e laranja (1m) indicam zonas de perigo ao redor do robô

### 4. Configurar e ativar o CAN

1. Na seção **CAN / Robô Flipsky 75100**: clique em **Escanear CANable**
2. Selecione a interface (geralmente `can0`), confira o bitrate (500000)
3. Clique em **Ativar CAN** — isso executa `ip link set can0 up type can bitrate 500000`
4. Ajuste os parâmetros:
   - **Duty máximo**: comece com `0.25` (25%) e aumente com cuidado
   - **Ganho direção**: `0.65` é um bom equilíbrio
   - **Botão homem-morto**: por padrão `BTN_TR` (botão R do Pro Controller)
5. Clique em **ARMAR robô**

### 5. Controlar o robô

1. Com o robô **armado**, segure o botão homem-morto (R) e mova o analógico esquerdo
2. Cima/baixo = acelerar/ré; esquerda/direita = girar
3. Os valores de duty esquerdo e direito aparecem em tempo real na seção CAN
4. Soltar o botão homem-morto **corta imediatamente** a potência (envia duty 0)
5. Use o **radar LiDAR** no canto direito para visualizar obstáculos ao redor

### 6. Parada de emergência

- O botão **PARADA DE EMERGÊNCIA** desarma o robô e envia `duty = 0` para ambos os motores imediatamente
- Use isso se o robô se comportar de forma inesperada

## Interface web — seções

### Coluna esquerda (controles)

| Seção | Função |
|---|---|
| **Status** | Conexão HID, nome do dispositivo, contagem de botões/eixos/eventos |
| **Bluetooth** | Power ON/OFF, preparar agent, scan, listar dispositivos, parear/conectar/remover por MAC |
| **CAN / Robô** | Escanear CANable, ativar interface, armar/desarmar, parada de emergência, parâmetros do VESC |
| **Dispositivos HID** | Listar /dev/input/event*, selecionar path fixo, filtrar por nome, deadzone |
| **Mapeamento rápido** | Associar códigos de botão/eixo a nomes de ação personalizados |
| **Mapeamentos salvos** | Tabela com todos os mapeamentos, clique para editar |

### Coluna direita (visualização)

| Seção | Função |
|---|---|
| **LiDAR LDROBOT STL-06P** | Canvas 420×420px com nuvem de pontos 2D persistente (3s fading). Anéis de perigo 50cm/1m. Métricas: conexão, nº pontos, obstáculo mais próximo, RPM, FPS. Legendas de cor por distância |
| **Controle visual** | Gamepad visual interativo (compacto): botões A/B/X/Y, D-Pad, analógicos (sticks), gatilhos (triggers), L/R/ZL/ZR, SELECT/HOME/START |
| **Último evento** | JSON do último evento recebido do gamepad |
| **Tabelas técnicas** | Lista detalhada de todos os botões e eixos (dentro do Controle visual, toggle) |
| **Log em tempo real** | Últimos 300 eventos de gamepad + ações do usuário |

## Configuração

Toda a configuração é salva automaticamente em `gamepad_config.json` (mesmo diretório do script). Ao rodar pela primeira vez, o arquivo é criado com valores padrão.

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
```

### Parâmetros CAN

| Parâmetro | Padrão | Descrição |
|---|---|---|
| `interface` | `can0` | Nome da interface SocketCAN |
| `bitrate` | `500000` | Taxa do barramento (125k, 250k, 500k, 1M) |
| `left_id` | `1` | ID CAN do VESC esquerdo (0-255) |
| `right_id` | `2` | ID CAN do VESC direito (0-255) |
| `max_duty` | `0.25` | Duty cycle máximo (0.01 a 0.95). Aumente com cuidado |
| `steering_gain` | `0.65` | Quanto o steering influencia a diferença entre motores (0 a 1) |
| `throttle_axis` | `ABS_Y` | Eixo usado para acelerar/ré |
| `steering_axis` | `ABS_X` | Eixo usado para direção |
| `invert_throttle` | `true` | Inverte o acelerador (normalmente ABS_Y vai negativo ao empurrar pra cima) |
| `invert_steering` | `false` | Inverte direção |
| `invert_left` | `false` | Inverte o motor esquerdo |
| `invert_right` | `true` | Inverte o motor direito |
| `require_deadman` | `true` | Exige botão homem-morto pressionado |
| `deadman_button` | `BTN_TR` | Botão homem-morto (R no Pro Controller) |
| `send_interval` | `0.05` | Intervalo mínimo entre envios CAN (em segundos) |

### Parâmetros LiDAR

| Parâmetro | Padrão | Descrição |
|---|---|---|
| `port` | `/dev/ttyUSB0` | Porta serial do LiDAR |
| `baudrate` | `230400` | Baud rate (STL-06P usa 230400) |
| `min_distance` | `150` | Distância mínima de detecção (mm). Pontos abaixo disso são ignorados |
| `max_distance` | `12000` | Distância máxima de detecção (mm). Pontos acima são ignorados |
| `emit_interval` | `0.08` | Intervalo mínimo entre envios de frame para o frontend (segundos) |

### Filtro de dispositivos HID

- **`device_path`**: caminho fixo para um `/dev/input/eventX` específico (ex: `/dev/input/event4`)
- **`device_name_contains`**: filtra dispositivos cujo nome contém o termo (ex: `Pro Controller`)
- **`deadzone`**: zona morta dos analógicos (0.0 a 0.5)

> Se ambos estiverem preenchidos, `device_path` tem prioridade.

### Mapeamentos

Os mapeamentos associam códigos de botão/eixo a nomes de ação que aparecem na interface. Exemplos:

```
BTN_SOUTH    → Freio
BTN_NORTH    → Farol
ABS_RZ       → Velocidade máxima
```

## Diagnóstico e troubleshooting

### Bluetooth conecta mas gamepad não aparece

```bash
cat /proc/bus/input/devices          # O controle aparece?
ls -l /dev/input/event*              # Existem dispositivos de input?
modinfo hid-nintendo                 # Módulo existe?
sudo modprobe hidp                   # Carregar módulo HID
sudo modprobe hid-nintendo           # Carregar driver Nintendo
```

Também use o botão **Diagnóstico HID** na interface web — ele mostra `/proc/bus/input/devices`, dispositivos evdev, e status dos módulos.

### Permissão negada ao acessar /dev/input/eventX

```bash
sudo usermod -aG input $USER
sudo reboot
```

### LiDAR não aparece / porta serial não encontrada

```bash
ls -l /dev/ttyUSB*                   # A porta aparece?
ls -l /dev/ttyACM*                   # Alguns adaptadores usam ttyACM
sudo usermod -aG dialout $USER       # Permissão para porta serial
sudo reboot
dmesg | grep -i usb                  # Verificar se o kernel reconheceu o dispositivo
```

O status do LiDAR aparece no card (OFF → ON quando conectado). Se mostrar `pyserial nao disponivel`:

```bash
pip install pyserial
```

### CAN não funciona

```bash
ip -details link show can0           # Interface existe?
sudo ip link set can0 up type can bitrate 500000   # Ativar manualmente
candump can0                         # Verificar tráfego
```

Certifique-se de que o CANable está com firmware **candleLight** (não slcan). Com firmware slcan, o adaptador aparece como `/dev/ttyACM*` e precisa do `slcand` para criar a interface CAN.

### Erro "evdev não disponível"

```bash
pip install evdev
# ou
sudo apt install python3-evdev
```

## Segurança

- **Nunca teste com o robô no chão na primeira vez** — mantenha-o suspenso
- Comece com `max_duty` baixo (0.10 a 0.25) e aumente gradualmente
- O botão homem-morto (`BTN_TR` / R) **precisa** estar segurado para qualquer movimento
- A **PARADA DE EMERGÊNCIA** desarma o robô e zera o duty de ambos os motores
- Sempre tenha um kill switch físico nos VESCs como redundância
- O LiDAR é **apenas visualização** — não interfere no controle do robô. O operador é responsável por desviar de obstáculos

## Arquitetura do código

O projeto é um **monolito de arquivo único** (~4650 linhas) contendo:

```
gamepad_web_can_flipsky.py
├── Configuração (DEFAULT_CONFIG, load/save)
├── Mapeamento de códigos evdev (CODE_FALLBACK_NAMES, FRIENDLY_NAMES)
├── HTML_PAGE (template inline com CSS + JS completo)
├── Funções auxiliares de sistema (módulos, /proc, /dev/input)
├── Gamepad reader (evdev, normalização de eixos, eventos)
├── LiDAR reader (pyserial, protocolo LDROBOT 0x54, CRC, parse)
├── Bluetooth (bluetoothctl interativo via subprocess.Popen)
├── CAN / SocketCAN (PF_CAN socket, envio de quadros estendidos)
├── Lógica do robô (deadman, throttle+steering→duty left/right)
├── Rotas Flask (REST API + Socket.IO eventos)
└── Entry point (threads do gamepad + LiDAR + socketio.run na porta 5005)
```

### Threads

- **Main thread**: servidor Flask + Socket.IO
- **gamepad_reader_loop** (daemon): loop infinito lendo eventos do `/dev/input/eventX`, reconectando automaticamente se o dispositivo sumir
- **lidar_reader_loop** (daemon): abre porta serial, faz parsing do protocolo LDROBOT, acumula pontos por ângulo (deduplicação por confiança), emite frames a cada ~80ms

### Comunicação

- **Browser ↔ Servidor**: Socket.IO para eventos em tempo real (status do gamepad, status CAN, eventos de botão/eixo, nuvem de pontos LiDAR)
- **Browser ↔ Servidor**: REST API para configuração, Bluetooth, CAN e LiDAR
- **Servidor ↔ CAN bus**: socket `PF_CAN` raw, quadros CAN estendidos (29-bit ID)
- **Servidor ↔ Bluetooth**: `subprocess.Popen` com `bluetoothctl` em modo interativo (stdin/stdout)
- **Servidor ↔ LiDAR**: `pyserial` na porta `/dev/ttyUSB0` a 230400 baud

### Formato do quadro CAN

Cada comando de duty cycle é enviado como:
- **ID CAN estendido**: `(CAN_PACKET_SET_DUTY << 8) | vesc_id`
- **Dados (4 bytes big-endian)**: `duty × 100000` como int32
- Via socket `PF_CAN` com flag `CAN_EFF_FLAG`

### Protocolo LiDAR LDROBOT STL-06P

```
Packet: [0x54][VerLen][Speed LE 2B][StartAngle LE 2B][Data N×(Dist LE 2B + Conf 1B)][EndAngle LE 2B][Timestamp LE 2B][CRC 1B]

VerLen   = (data_type << 5) | n_points     (tipicamente 0x2C = 12 pontos)
Speed    = velocidade de rotação (graus/s)
Angle    = ângulo × 100 (0-35999)
Distance = distância em mm (uint16 little-endian)
Conf     = confiança/intensidade (0-255)
CRC      = XOR de todos os bytes anteriores
```

O parser (`parse_lidar_packet`) busca o header `0x54`, valida CRC, converte coordenadas polares para cartesianas e descarta pontos com `distance = 0`. Pontos são acumulados no frontend por 3 segundos com fading progressivo.

## Licença

Uso livre. Projeto pessoal para robótica educacional.
