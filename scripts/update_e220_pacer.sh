#!/usr/bin/env bash
# =============================================================================
# Script: update_e220_pacer.sh
# Description: Installs E220 pacer scripts & systemd unit from the directory
#              containing this script, reloads systemd units, restarts
#              e220-pacer, and tails journal logs.
# Usage (from inside the same directory):
#   ./update_e220_pacer.sh              # Deploy + follow logs
#   ./update_e220_pacer.sh --no-follow  # Deploy + print last 40 lines & exit
#   bash update_e220_pacer.sh           # Works without chmod +x
# =============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FOLLOW_LOGS=1

for arg in "$@"; do
    case "$arg" in
        -n|--no-follow)
            FOLLOW_LOGS=0
            ;;
        -h|--help)
            echo "Usage: ./$(basename "$0") [--no-follow]"
            exit 0
            ;;
        *)
            echo "[ERROR] Unknown option: $arg" >&2
            exit 1
            ;;
    esac
done

if [[ ! -f "${SCRIPT_DIR}/e220_pacer.py" ]]; then
    echo "[ERROR] e220_pacer.py not found in ${SCRIPT_DIR}" >&2
    exit 1
fi

echo "[UPDATE] Stopping running e220-pacer.service before replacing scripts..."
sudo systemctl stop e220-pacer || true

echo "[UPDATE] Installing from local directory: ${SCRIPT_DIR}"
sudo install -m 0755 "${SCRIPT_DIR}/e220_pacer.py" /usr/local/bin/e220_pacer.py

if [[ -f "${SCRIPT_DIR}/set_e220_power.py" ]]; then
    echo "[UPDATE] Installing set_e220_power.py -> /usr/local/bin/set_e220_power.py"
    sudo install -m 0755 "${SCRIPT_DIR}/set_e220_power.py" /usr/local/bin/set_e220_power.py
fi

if [[ -f "${SCRIPT_DIR}/scan_e220_channels.py" ]]; then
    echo "[UPDATE] Installing scan_e220_channels.py -> /usr/local/bin/scan_e220_channels.py"
    sudo install -m 0755 "${SCRIPT_DIR}/scan_e220_channels.py" /usr/local/bin/scan_e220_channels.py
fi

if [[ -f "${SCRIPT_DIR}/e220-pacer.service" ]]; then
    echo "[UPDATE] Syncing e220-pacer.service -> /etc/systemd/system/e220-pacer.service"
    sudo install -m 0644 "${SCRIPT_DIR}/e220-pacer.service" /etc/systemd/system/e220-pacer.service
fi

echo "[UPDATE] Reloading systemd unit definitions (daemon-reload)..."
sudo systemctl daemon-reload

echo "[UPDATE] Starting e220-pacer.service..."
sudo systemctl start e220-pacer

STATE="$(sudo systemctl is-active e220-pacer || true)"
echo "[UPDATE] Service state: ${STATE}"

if [[ "${FOLLOW_LOGS}" -eq 1 ]]; then
    echo "[UPDATE] Attaching to journalctl (Ctrl+C to detach)..."
    exec sudo journalctl -u e220-pacer -n 40 -f
else
    sudo journalctl -u e220-pacer -n 40 --no-pager
fi
