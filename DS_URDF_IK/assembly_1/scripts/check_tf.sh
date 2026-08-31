#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'USAGE'
Usage:
  ./scripts/check_tf.sh left
  ./scripts/check_tf.sh right
  ./scripts/check_tf.sh frames

Commands:
  left    Run tf2_echo from base_link to left_gripper_link
  right   Run tf2_echo from base_link to right_gripper_link
  frames  Generate the TF frame graph with tf2_tools view_frames
USAGE
}

run_command() {
  case "$1" in
    left)
      exec ros2 run tf2_ros tf2_echo base_link left_gripper_link
      ;;
    right)
      exec ros2 run tf2_ros tf2_echo base_link right_gripper_link
      ;;
    frames)
      exec ros2 run tf2_tools view_frames
      ;;
    help|-h|--help)
      usage
      exit 0
      ;;
    *)
      usage >&2
      exit 2
      ;;
  esac
}

if [ "$#" -gt 0 ]; then
  run_command "$1"
fi

PS3="Select TF check: "
select target in left right frames quit; do
  case "${target:-}" in
    left|right|frames)
      run_command "$target"
      ;;
    quit)
      exit 0
      ;;
    *)
      usage >&2
      ;;
  esac
done
