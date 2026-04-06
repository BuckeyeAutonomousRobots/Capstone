#!/bin/bash
set -euo pipefail

export DISPLAY="${DISPLAY:-:1}"
export VNC_PORT="${VNC_PORT:-5900}"
export NOVNC_PORT="${NOVNC_PORT:-6080}"
export HOME="/home/noah"

# Start a virtual X display and XFCE desktop.
Xvfb "${DISPLAY}" -screen 0 1920x1080x24 >/tmp/xvfb.log 2>&1 &
sleep 1
startxfce4 >/tmp/xfce.log 2>&1 &
sleep 2

# Disable desktop auto-lock/screensaver for this user profile.
mkdir -p "${HOME}/.config/autostart"
cat > "${HOME}/.config/autostart/xfce4-screensaver.desktop" <<'EOF'
[Desktop Entry]
Type=Application
Name=xfce4-screensaver
Hidden=true
EOF
cat > "${HOME}/.config/autostart/light-locker.desktop" <<'EOF'
[Desktop Entry]
Type=Application
Name=light-locker
Hidden=true
EOF

# Disable X11 screen blanking / DPMS and stop any already-running locker.
xset -display "${DISPLAY}" s off -dpms s noblank >/tmp/xset.log 2>&1 || true
xfconf-query -c xfce4-screensaver -p /lock/enabled -n -t bool -s false >/tmp/xfconf.log 2>&1 || true
xfconf-query -c xfce4-screensaver -p /saver/enabled -n -t bool -s false >>/tmp/xfconf.log 2>&1 || true
xfconf-query -c xfce4-screensaver -p /saver/idle-activation/enabled -n -t bool -s false >>/tmp/xfconf.log 2>&1 || true
pkill -f xfce4-screensaver >/dev/null 2>&1 || true
pkill -f light-locker >/dev/null 2>&1 || true

# Expose the desktop over VNC, then bridge it to noVNC on port 6080.
x11vnc -display "${DISPLAY}" -rfbport "${VNC_PORT}" -forever -shared -nopw -xkb >/tmp/x11vnc.log 2>&1 &
websockify --web=/usr/share/novnc/ "${NOVNC_PORT}" "localhost:${VNC_PORT}" >/tmp/novnc.log 2>&1 &

echo "[start_desktop] noVNC ready at http://localhost:${NOVNC_PORT}/vnc.html"

# Keep container alive and stream logs.
tail -F /tmp/xvfb.log /tmp/xfce.log /tmp/x11vnc.log /tmp/novnc.log
