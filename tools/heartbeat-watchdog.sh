#!/bin/bash
# Dead-man's switch for the water heater monitor.
#
# The device emits a syslog heartbeat every 15 minutes. Every other syslog
# message it sends is edge-triggered, so a healthy device is otherwise silent
# and a dead one looks identical to a quiet one. This script detects the
# ABSENCE of heartbeats — the failure the syslog mirror exists to catch.
#
# Deploy on netwatch. Cron mails any output, so printing = alerting:
#   */10 * * * * /usr/local/bin/heartbeat-watchdog.sh
#
# Exits 0 when healthy and silent; exits 1 with a message when not.
#
# netwatch's rsyslog splits remote hosts into /var/log/remote/<hostname>.log
# (see $template RemoteFile in /etc/rsyslog.d/) and timestamps them in ISO 8601,
# not the traditional RFC 3164 "Aug 21 09:25:11" format.

set -uo pipefail

LOGFILE="${LOGFILE:-/var/log/remote/waterheater.log}"
MAX_AGE_MIN="${MAX_AGE_MIN:-40}"   # ~2.5 missed 15-min heartbeats

[[ -r "$LOGFILE" ]] || { echo "heartbeat-watchdog: cannot read $LOGFILE"; exit 1; }

last=$(grep -a '\[heartbeat\] alive' "$LOGFILE" 2>/dev/null | tail -1)

if [[ -z "$last" ]]; then
    echo "WATER HEATER: no heartbeat found in $LOGFILE at all."
    echo "Device may be down, or syslog reception has stopped."
    exit 1
fi

# Field 1 is a full ISO 8601 timestamp with offset, e.g.
# 2026-08-21T09:25:11-05:00 — self-contained, so no year inference needed.
ts=$(echo "$last" | awk '{print $1}')
last_epoch=$(date -d "$ts" +%s 2>/dev/null) || {
    echo "heartbeat-watchdog: could not parse timestamp: $ts"; exit 1; }

age_min=$(( ( $(date +%s) - last_epoch ) / 60 ))

if (( age_min > MAX_AGE_MIN )); then
    echo "WATER HEATER: no heartbeat for ${age_min} minutes (threshold ${MAX_AGE_MIN})."
    echo "Last seen: $last"
    exit 1
fi

exit 0
