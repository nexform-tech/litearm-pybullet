#!/usr/bin/env bash
# Start the virtual display, the VNC server, and the noVNC proxy for this
# container. Idempotent: devcontainer.json runs this on every container start,
# so each step checks for a running instance first.
set -euo pipefail

export DISPLAY="${DISPLAY:-:1}"

wait_for_x() {
    for _ in $(seq 1 50); do
        if xdpyinfo -display "$DISPLAY" >/dev/null 2>&1; then
            return 0
        fi
        sleep 0.1
    done
    echo "Xvfb did not come up on $DISPLAY" >&2
    return 1
}

# 1. Virtual X server (only if the display is not already served).
if ! xdpyinfo -display "$DISPLAY" >/dev/null 2>&1; then
    nohup Xvfb "$DISPLAY" -screen 0 1600x900x24 -nolisten tcp \
        >/tmp/xvfb.log 2>&1 &
    wait_for_x
fi

# 2. VNC server for the display, bound to localhost only.
if ! pgrep -f "x11vnc -display ${DISPLAY}" >/dev/null; then
    x11vnc -display "$DISPLAY" -forever -shared -nopw -localhost -bg \
        -o /tmp/x11vnc.log
fi

# 3. noVNC proxy: http://localhost:6080/vnc.html on the host.
if ! pgrep -f "websockify .*6080" >/dev/null; then
    nohup websockify --web /usr/share/novnc 6080 localhost:5900 \
        >/tmp/novnc.log 2>&1 &
fi

echo "Display ready: open http://localhost:6080/vnc.html in your host browser"
