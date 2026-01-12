# The K.O.Z.A. Project 🐐

## Dependencies

Runtime (RPi / production):
- `mpv` (system package)
- `python-mpv` (pip)
- `pyradios`, `PyYAML`

macOS development:
- `brew install mpv`
- `pip install python-mpv pyradios pyyaml`

Example (RPi):
```bash
sudo apt install mpv
pip install python-mpv pyradios pyyaml
```

## Raspberry Pi 2 install

System packages (libmpv + tools):
```bash
sudo apt update
sudo apt install mpv libmpv-dev python3-venv python3-pip
```

Create venv and install Python deps:
```bash
python3 -m venv venv-lw
source venv-lw/bin/activate
pip install python-mpv pyradios pyyaml
```

Run:
```bash
source venv-lw/bin/activate
python3 koza.py
```

## Setup (venv)

Create venv and install deps:
```bash
python3 -m venv venv-lw
source venv-lw/bin/activate
pip install python-mpv pyradios pyyaml
```

Activate venv later:
```bash
source venv-lw/bin/activate
```

## Development

Run interactive mode without GPIO/SMBus:
```bash
python3 koza.py --development
```
