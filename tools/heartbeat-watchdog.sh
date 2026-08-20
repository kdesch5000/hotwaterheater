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

set -uo pipefail

LOGFILE="${LOGFILE:-/var/log/syslog}"
MAX_AGE_MIN="${MAX_AGE_MIN:-40}"   # ~2.5 missed 15-min heartbeats
TAG="waterheater"

[[ -r "$LOGFILE" ]] || { echo "heartbeat-watchdog: cannot read $LOGFILE"; exit 1; }

last=$(grep -a "$TAG: \[heartbeat\] alive" "$LOGFILE" 2>/dev/null | tail -1)

if [[ -z "$last" ]]; then
    echo "WATER HEATER: no heartbeat found in $LOGFILE at all."
    echo "Device may be down, or syslog reception is not configured."
    exit 1
fi

# Leading field is an RFC 3164 timestamp ("Aug 20 14:05:12"), which omits the
# year; date(1) resolves it against the current year.
ts=$(echo "$last" | awk '{print $1, $2, $3}')
last_epoch=$(date -d "$ts" +%s 2>/dev/null) || {
    echo "heartbeat-watchdog: could not parse timestamp: $ts"; exit 1; }

now_epoch=$(date +%s)
# A December/January rollover can put the parsed date in the future
(( last_epoch > now_epoch )) && last_epoch=$(( last_epoch - 31536000 ))

age_min=$(( (now_epoch - last_epoch) / 60 ))

if (( age_min > MAX_AGE_MIN )); then
    echo "WATER HEATER: no heartbeat for ${age_min} minutes (threshold ${MAX_AGE_MIN})."
    echo "Last seen: $last"
    exit 1
fi

exit 0
