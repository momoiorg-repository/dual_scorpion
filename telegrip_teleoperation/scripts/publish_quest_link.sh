#!/usr/bin/env bash
set -euo pipefail

REMOTE_URL="${1:-}"
LOCAL_URL="${2:-}"
GIST_ID="${TELEGRIP_GIST_ID:-a84200f82fd9618213d05f27e1c255ff}"
GIST_FILE="dual_scorpion_teleop_link.md"

command -v gh >/dev/null || {
  echo "GitHub CLI is required: https://cli.github.com/"
  exit 2
}
gh auth status >/dev/null 2>&1 || {
  echo "Run 'gh auth login' before updating the Quest link gist."
  exit 2
}

STAMP="$(date '+%Y-%m-%d %H:%M:%S %Z')"
BODY="# Dual Scorpion — Live Teleoperation Link"

if [ -n "$LOCAL_URL" ]; then
  BODY+="

## [Open local teleoperation]($LOCAL_URL)

Quest and robot host must be on the same LAN/Wi-Fi."
fi

if [ -n "$REMOTE_URL" ]; then
  BODY+="

## [Open remote teleoperation]($REMOTE_URL)"
fi

BODY+="

Updated: $STAMP

Open the applicable link in the Meta Quest Browser, accept any certificate
warning, select **Enter VR**, and connect the robot only after the workspace is
clear."

PAYLOAD="$(mktemp)"
trap 'rm -f "$PAYLOAD"' EXIT
BODY="$BODY" GIST_FILE="$GIST_FILE" python3 - "$PAYLOAD" <<'PY'
import json
import os
import sys

payload = {
    "description": "Dual Scorpion live teleoperation link",
    "files": {os.environ["GIST_FILE"]: {"content": os.environ["BODY"]}},
}
with open(sys.argv[1], "w", encoding="utf-8") as stream:
    json.dump(payload, stream)
PY

gh api -X PATCH "/gists/$GIST_ID" --input "$PAYLOAD" --jq '.html_url'
