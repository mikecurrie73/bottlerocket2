#!/bin/bash

cleanup() {
    echo "Shutting down..."
    pkill -f camera_http_stream.py
    pkill -f rocket_server.py
    echo "Done."
    exit 0
}
trap cleanup SIGINT SIGTERM

# --- Run camera stream using system Python ---
echo "[INFO] Starting camera stream..."
python3 camera_http_stream.py &

sleep 2

# --- Start Python server inside venv ---
echo "[INFO] Starting telemetry server..."
if [ -d "venv" ]; then
    echo "[INFO] Activating Python virtual environment..."
    source venv/bin/activate
fi
python3 rocket_server.py
