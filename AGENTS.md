# AGENTS.md — pandorapi

## Overview

Single-file Flask + Socket.IO web app for controlling a robot via Bluetooth gamepad over CAN bus (Flipsky 75100 VESC). Target platform: Raspberry Pi (Linux).

## Run

```bash
sudo python gamepad_web_can_flipsky.py
```

- Serves on `http://0.0.0.0:5005`
- `sudo` is needed for CAN `ip link` commands; for gamepad-only use, regular user works if in the `input` group
- Config auto-created at `gamepad_config.json` next to the script on first run

## Dependencies

- **Python packages**: `flask`, `flask-socketio`, `evdev` (optional but expected)
- **System**: `bluetoothctl`, `bluez`, `can-utils`, `iproute2`; kernel modules `hidp`, `hid-nintendo`

## Architecture

- **All code is in one file** (~3870 lines): Python backend, inline HTML template, inline JavaScript client
- No build step, no package manager, no tests, no CI
- `gamepad_reader_loop` runs as a daemon thread reading `/dev/input/eventX` via evdev
- CAN frames sent raw over SocketCAN (`PF_CAN` socket)
- Bluetooth operations shell out to `bluetoothctl` via interactive `subprocess.Popen`
- Gamepad-to-motor math: `left = (throttle + steering * gain) * max_duty`, `right = (throttle - steering * gain) * max_duty`

## Hardware expectations

- CANable adapter (candleLight firmware preferred) → `can0`
- Flipsky 75100 VESC with CAN IDs 1 (left) and 2 (right)
- Nintendo Switch Pro Controller (or any evdev-compatible HID gamepad)
