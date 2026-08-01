#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

LAN_IP="$(hostname -I | awk '{print $1}')"
LOCAL_URL="https://${LAN_IP}:8443"

printf 'Quest local URL: %s\n' "$LOCAL_URL"
"$ROOT/scripts/publish_quest_link.sh" "" "$LOCAL_URL" ||
  echo "Could not update the Quest gist; open the local URL directly."

exec telegrip --no-viz "$@"
