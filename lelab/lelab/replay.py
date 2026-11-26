# Copyright 2026 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Replay a recorded episode on the follower arm: an in-process take on
lerobot-replay, so the dataset browser can stop it."""

import logging
import math
import threading
import time
from typing import Any

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
from lerobot.utils.robot_utils import precise_sleep

from . import episode_media
from .hardware import MAX_SCORPION_FPS, HardwareRequest, scorpion_follower
from .scorpion_view import read_view_mapping, scorpion_joint_frame
from .utils.config import setup_follower_calibration_file
from .utils.devices import follower_lock, safe_disconnect_device

logger = logging.getLogger(__name__)

# Only the worker (or a failed start) clears this, so the follower stays
# claimed until it is actually released.
replay_active = False
replay_thread: threading.Thread | None = None
replay_error: str | None = None
_stop = threading.Event()


class ReplayRequest(HardwareRequest):
    follower_port: str = ""
    follower_config: str
    dataset_repo_id: str
    episode_index: int


def _episode_actions(repo_id: str, episode_index: int) -> tuple[list[dict[str, float]], int]:
    """The episode's recorded actions as follower commands, and the dataset fps."""
    dataset_dir = episode_media.resolve_dataset_dir(repo_id)
    info = episode_media.read_info(dataset_dir)
    names = info["features"]["action"]["names"]
    rows = episode_media.load_episode_actions(dataset_dir, episode_index)
    return [dict(zip(names, row, strict=True)) for row in rows], info["fps"]


def handle_start_replay(request: ReplayRequest, websocket_manager=None) -> dict[str, Any]:
    """Connect the follower synchronously, so a failure reaches the caller,
    then send the episode's actions at the recording's rate in the background."""
    global replay_active, replay_thread, replay_error

    from . import record as _record, rollout as _rollout, teleoperate as _teleoperate
    from .calibrate import calibration_manager

    with follower_lock:
        if calibration_manager.get_status().calibration_active:
            return {"success": False, "message": "Calibration is currently active."}
        if replay_active:
            return {"success": False, "message": "A replay is already running"}
        if _record.recording_active:
            return {"success": False, "message": "Recording is currently active. Stop it first."}
        if _teleoperate.teleoperation_busy():
            return {"success": False, "message": "Teleoperation is currently active. Stop it first."}
        if _rollout.inference_active:
            return {"success": False, "message": "Inference is currently active. Stop it first."}
        replay_active = True
        replay_error = None
        _stop.clear()

    robot = None
    view_mapping = {}
    try:
        actions, fps = _episode_actions(request.dataset_repo_id, request.episode_index)

        if request.robot_model == "dual_scorpion":
            from lerobot.robots import make_robot_from_config

            robot = make_robot_from_config(
                scorpion_follower(
                    request.follower_left_port, request.follower_right_port, request.follower_config
                )
            )
            if not actions or any(
                frame.keys() != robot.action_features.keys()
                or not all(math.isfinite(value) for value in frame.values())
                for frame in actions
            ):
                raise ValueError("The episode does not contain valid Dual Scorpion follower actions.")
            if not fps or fps <= 0:
                raise ValueError("The episode has no valid frame rate.")
            fps = min(fps, MAX_SCORPION_FPS)
            view_mapping = read_view_mapping(request.view_mapping, robot)
            robot.connect(calibrate=False)
            current = robot.get_joint_positions()
            if any(
                not math.isfinite(current[key])
                or abs(current[key] - goal) > (5.0 if key.endswith("gripper.pos") else 2.0)
                for key, goal in actions[0].items()
            ):
                raise ValueError(
                    "Follower is too far from the episode's first pose. Align both arms before replay."
                )
        else:
            follower_id = setup_follower_calibration_file(request.follower_config)
            robot = SO101Follower(SO101FollowerConfig(port=request.follower_port, id=follower_id))
            try:
                robot.bus.connect()
            except Exception as e:
                raise RuntimeError(
                    f"Could not connect to the follower arm on {request.follower_port}. "
                    "Make sure it's plugged in and powered on, then try again."
                ) from e
            # Feetech motors only store calibration registers with the torque off.
            robot.bus.disable_torque()
            robot.bus.write_calibration(robot.calibration)
            robot.configure()
    except Exception as e:
        safe_disconnect_device(robot, logger)
        replay_active = False
        logger.error(f"Failed to start replay: {e}")
        return {"success": False, "message": str(e)}

    def replay_worker():
        global replay_active, replay_error

        try:
            for action in actions:
                if _stop.is_set():
                    break
                start = time.perf_counter()
                robot.send_action(action)
                if websocket_manager is not None and request.robot_model == "dual_scorpion":
                    websocket_manager.broadcast_joint_data_sync(
                        {
                            "type": "joint_update",
                            "timestamp": time.time(),
                            **scorpion_joint_frame(robot.get_joint_positions(), view_mapping),
                        }
                    )
                precise_sleep(1 / fps - (time.perf_counter() - start))
        except Exception as e:
            logger.error(f"Replay failed: {e}")
            replay_error = str(e)
        finally:
            safe_disconnect_device(robot, logger)
            replay_active = False
            logger.info("Replay ended")

    replay_thread = threading.Thread(target=replay_worker, name="replay-worker", daemon=True)
    replay_thread.start()

    return {"success": True, "message": "Replay started"}


def handle_stop_replay() -> dict[str, Any]:
    """Signal the worker and wait for it to release the arm."""
    if not replay_active:
        return {"success": False, "message": "No replay is running"}

    _stop.set()
    worker = replay_thread
    if worker is not None:
        worker.join(timeout=5.0)
        if worker.is_alive():
            logger.warning("Replay worker did not exit within 5s")
            return {"success": False, "message": "The replay has not released the arm yet"}

    return {"success": True, "message": "Replay stopped"}


def handle_replay_status() -> dict[str, Any]:
    return {"replay_active": replay_active, "error": replay_error}
