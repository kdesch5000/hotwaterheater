#!/bin/bash
# Flash the water heater monitor over OTA as soon as it is actually reachable.
#
# Why this exists: at the edge of range the board holds a WiFi association it
# cannot use. It answers ARP and can squeeze out a UDP syslog packet while every
# TCP handshake fails, so "it looks connected" is not a signal you can flash on.
# This waits for a real HTTP response and fires immediately when one arrives —
# the reachable windows are short and unpredictable.
#
# Usage:
#   ./ota-flash-when-reachable.sh [device_ip] [bin_path]
#
# Runs until it succeeds or MAX_WAIT_MIN elapses. Safe to leave running.

set -uo pipefail

DEVICE="${1:-192.168.86.95}"
BIN="${2:-$(dirname "$0")/../water_heater_monitor/build/esp32.esp32.lilygo_t_display_s3/water_heater_monitor.ino.bin}"
POLL_SEC="${POLL_SEC:-15}"
MAX_WAIT_MIN="${MAX_WAIT_MIN:-720}"   # 12 hours

[[ -f "$BIN" ]] || { echo "No firmware at $BIN — run arduino-cli compile first."; exit 1; }

MD5=$(md5 -q "$BIN")
SIZE=$(wc -c < "$BIN" | tr -d ' ')
echo "Firmware: $BIN"
echo "  size=$SIZE md5=$MD5"
echo "Waiting for $DEVICE to become reachable (poll ${POLL_SEC}s, give up after ${MAX_WAIT_MIN}m)..."

deadline=$(( $(date +%s) + MAX_WAIT_MIN * 60 ))
attempt=0

while (( $(date +%s) < deadline )); do
    attempt=$((attempt + 1))

    # A real HTTP response is the only trustworthy readiness signal. Keep the
    # timeout short: a reachable window may only last seconds.
    code=$(curl -s -m 5 -o /dev/null -w '%{http_code}' "http://$DEVICE/api/status" 2>/dev/null)

    if [[ "$code" == "200" ]]; then
        echo "[$(date '+%H:%M:%S')] reachable (attempt $attempt) — starting OTA"

        # ElegantOTA is a 2-step API: open a session, then upload.
        start=$(curl -s -m 10 "http://$DEVICE/ota/start?mode=fr&hash=$MD5" 2>/dev/null)
        echo "  /ota/start -> ${start:-<no response>}"

        # MUST be -F multipart. --data-binary returns OK but silently does not
        # apply the firmware; the AsyncWebServer handler requires chunked upload.
        if curl -sf -m 180 -X POST "http://$DEVICE/ota/upload" -F "firmware=@$BIN" >/dev/null 2>&1; then
            echo "[$(date '+%H:%M:%S')] upload accepted — device reboots automatically"
            echo "Verify with: curl -s http://$DEVICE/api/debug | jq .link"
            exit 0
        fi

        echo "  upload failed (link probably dropped mid-transfer) — retrying"
    else
        printf '\r[%s] attempt %d: unreachable (http=%s)   ' "$(date '+%H:%M:%S')" "$attempt" "${code:-000}"
    fi

    sleep "$POLL_SEC"
done

echo
echo "Gave up after ${MAX_WAIT_MIN}m — device never became reachable."
echo "The link is likely too weak to carry an OTA upload. Power-cycle it, or"
echo "move it closer to an AP, then re-run."
exit 1
