# Dual Scorpion real-robot Agent Harness

This harness connects JOUZU and Gemini 4 Argon to the real Dual Scorpion robot while keeping high-level reasoning and deterministic robot control separate.

- **JOUZU + Gemini 4 Argon:** understand natural language and camera images, select skills, and generate a structured task plan.
- **Harness:** validate the plan, read real state and images, perform FK/IK and trajectory checks, enforce local safety, execute through the existing follower, and return quantitative feedback.
- **Real robot:** the only motion target. MuJoCo is not used as the execution backend.

## Architecture

```text
JOUZU / Gemini 4 Argon
        ↓ structured task plan
Task Plan Receiver and validator
        ↓
WorldState + camera image provider
        ↓
deterministic Cartesian planner (legacy URDF / PyBullet FK-IK)
        ↓
joint-limit, FK, timing, continuity, calibration, and authorization checks
        ↓
existing DualScorpionFollower execution path
        ↓
encoder feedback, execution report, evaluation, replanning context
```

The LLM never supplies a motor command. Direct `joint_deg`, `motor_command`, `goal_position`, or gripper command fields are rejected before planning.

## Local setup

Use the repository environment as described in `AGENTS.md`. The harness does not require JOUZU or Gemini for local tests.

```bash
cd /home/syun/open_pj/dual_scorpion
export PYTHONPATH="$PWD/telegrip_teleoperation"
uv run --no-sync --active python -m telegrip.mocopi.agent --config config/mocopi/agent.example.yaml skills
uv run --no-sync --active python -m telegrip.mocopi.agent --config config/mocopi/agent.example.yaml status
uv run --no-sync --active python -m telegrip.mocopi.agent --config config/mocopi/agent.example.yaml world
```

`config/mocopi/agent.example.yaml` is an example. Replace camera device IDs on the target machine; the harness never infers them.

## Camera and multimodal input

`CameraService` uses the existing OpenCV/UVC adapter. It supports:

- multiple configured cameras,
- camera IDs and device paths from YAML,
- image capture and JPEG encoding,
- monotonic acquisition timestamps,
- image width, height, MIME type, and SHA-256,
- optional coordinate-frame labels,
- stale-frame and capture-error reporting.

The MCP `capture_images` tool returns standard MCP image content:

```json
{
  "type": "image",
  "data": "<base64 JPEG>",
  "mimeType": "image/jpeg"
}
```

The same response contains structured metadata with camera ID, timestamp, dimensions, hash, coordinate label, and source. Image capture has no motion authority.

Semantic labels from Gemini may be stored with `update_object_estimates`, but they are marked `llm_semantic`. A metric 3D position is accepted only when an external validated geometry provider supplies `coordinate_validation` with method, timestamp, and uncertainty. A monocular semantic estimate is never treated as a metric robot-frame coordinate.

## WorldState

WorldState uses schema `dual_scorpion.agent.world_state.v1` and contains:

- left and right measured joint angles,
- TCP position and orientation from FK,
- connection and calibration state,
- camera status and last-frame timestamps,
- object labels, estimates, confidence, and provenance,
- task progress,
- metadata describing unsupported capabilities.

Every value carries a source and timestamp. The current harness reports `vision_object_detection: false` and `metric_pose_estimation: false`; it does not claim perception accuracy that it cannot measure.

## Task plans

Plans use schema `dual_scorpion.agent.task_plan.v2`.

```json
{
  "schema": "dual_scorpion.agent.task_plan.v2",
  "goal": "Move both arms one centimeter forward.",
  "goal_state": {"type": "relative_tcp_motion"},
  "objects": [],
  "environment": {},
  "arm_roles": {"left": "mover", "right": "mover"},
  "steps": [
    {
      "id": "move",
      "skill": "move_arm_to_pose",
      "arms": "both",
      "depends_on": [],
      "parameters": {
        "relative_position_m": [0.01, 0.0, 0.0]
      }
    }
  ],
  "preconditions": ["Both arms are connected and calibrated."],
  "success_conditions": [
    {"type": "tcp_position_error", "step_id": "move", "max_value": 0.025}
  ],
  "failure_conditions": [{"type": "ik_failure"}],
  "safety_constraints": ["Reject stale feedback."],
  "replan_policy": {"max_attempts": 3},
  "provenance": {"planner": "jouzu-gemini"}
}
```

Supported motion targets are:

1. `relative_position_m`, relative to the current validated TCP pose.
2. `target_object_id` plus an explicit offset, where the object has a validated metric pose.
3. `target_position_m` with a `pose_validation` object.

Unknown objects, unavailable skills, cycles, duplicate IDs, stale or unvalidated coordinates, and direct motor fields are rejected with machine-readable errors.

## Robot Skills

The current catalogue is available through the `skills` command and MCP `list_skills`.

| Skill | Status | Backend |
| --- | --- | --- |
| `read_robot_state` | implemented | real follower adapter |
| `get_world_state` | implemented | harness WorldState builder |
| `capture_image` | implemented | OpenCV/UVC camera service |
| `move_arm_to_pose` | implemented software path; physical execution not verified | deterministic planner + existing real replay path |
| `validate_motion` | implemented | existing real-plan validator |
| `stop_motion` | implemented | local safety event |
| `set_gripper` | unavailable | no verified harness integration |

The execution path commands joint0–joint6 only and never sends a gripper command.

## MCP tools

Start the local stdio server:

```bash
uv run --no-sync --active python -m telegrip.mocopi.agent \
  --config config/mocopi/agent.example.yaml mcp
```

JOUZU can connect it as a local MCP server through a project `.pi/mcp.json` file. Use an absolute repository path on the robot host:

```json
{
  "mcpServers": {
    "dual-scorpion-agent": {
      "command": "uv",
      "args": [
        "run",
        "--no-sync",
        "--active",
        "python",
        "-m",
        "telegrip.mocopi.agent",
        "--config",
        "config/mocopi/agent.example.yaml",
        "mcp"
      ],
      "cwd": "/absolute/path/to/dual_scorpion"
    }
  }
}
```

The server implements JSON-RPC `initialize`, `tools/list`, and `tools/call`. JOUZU's Gemini 4 Argon session can then call the tools above. This repository does not start JOUZU, change model settings, or store authentication data.

| Tool | Purpose | Authorization |
| --- | --- | --- |
| `get_status` | robot, camera, safety, execution, and skill status | none |
| `connect_robot` | connect both calibrated follower buses | operator token |
| `get_world_state` | current provenance-labelled state | none |
| `list_skills` | skill contracts and implementation status | none |
| `capture_images` | MCP image content and metadata | none |
| `update_object_estimates` | semantic or validated object estimates | none |
| `submit_task_plan` | validate and store a task plan | none |
| `plan_motion` | generate and validate a joint trajectory | none |
| `execute_motion` | start a real execution job; optionally capture before/after images | operator token |
| `stop_motion` | set the local stop event | none |
| `get_execution_report` | execution metrics and failure state | none |
| `get_replan_context` | failure, current state, and required observations | none |

## Safety and authorization

1. The operator creates a token file outside Git, for example `/run/user/1000/scorpion-agent-motion.token`.
2. The harness reads that path from its YAML config.
3. `connect_robot` and `execute_motion` require the token value.
4. The LLM cannot create, infer, or persist the token.
5. `stop_motion` is local and always available.
6. Execution is rejected unless the buses are connected, calibration is valid, encoder feedback is fresh and finite, and the generated trajectory passes the existing validator.
7. On stop, tracking error, limit violation, or communication failure, the execution loop holds the measured pose and disconnects without commanding the gripper.

Physical robot operation still requires the operator's physical safety procedure and explicit authorization. Software tests do not authorize or verify real motion.

## Execution feedback and evaluation

The existing real execution path records:

- commanded and measured joint angles,
- maximum joint tracking error,
- TCP position and orientation error,
- execution time and measured FPS,
- stop and failure state,
- gripper command count (always zero in this harness),
- model and URDF fingerprints.

Evaluation uses only machine-checkable conditions. Semantic image success is not inferred from motor feedback; JOUZU/Gemini may inspect before/after images separately after execution.

`execute_motion` accepts an optional `capture_before_after` flag. When enabled, the harness saves timestamped JPEGs from configured cameras immediately before and after the motion job and returns their paths and metadata in the execution report. Image capture is not part of the real-time stop path.

## Replanning

`get_replan_context` returns:

- original plan,
- execution status and failure,
- final recorded robot frame,
- evaluation checks,
- current WorldState,
- required observations,
- safety constraints.

JOUZU can use this data to request new images, update object estimates, and submit a new plan. The harness does not run an external LLM loop.

## Local tests

Focused hardware-free tests:

```bash
env -u PYTHONPATH -u LD_LIBRARY_PATH -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH \
  uv run --no-sync --active pytest tests/teleoperators/test_agent_harness.py -q
```

Existing real-replay and mocopi regression tests:

```bash
env -u PYTHONPATH -u LD_LIBRARY_PATH -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH \
  uv run --no-sync --active pytest tests/teleoperators/test_mocopi_dataset.py -q
```

## Current limitations

- Real hardware connection and motion were **not** exercised during this development.
- Gripper commands are **not integrated** and are reported as unavailable.
- Camera-to-robot extrinsics and metric 3D object pose estimation are **not implemented**.
- Image semantic understanding is delegated to JOUZU/Gemini and was **not** tested here.
- Contact and collision checking for arbitrary external environments is not available in the current real execution path.
- The current motion path uses the legacy calibrated URDF/PyBullet planner; the unmeasured full-CAD MuJoCo model is not authorized for hardware replay.
