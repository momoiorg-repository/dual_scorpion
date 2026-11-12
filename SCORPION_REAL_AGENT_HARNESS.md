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

## Real-follower calibration and startup behavior

The `robot` section in the agent config constructs `RealRobotAdapter` for both CLI and MCP startup. Construction does not open serial ports and does not move the robot.

The selected robot ID is:

```text
dual_scorpion_follower_2026_1026
```

LeRobot's standard follower lookup resolves its calibration to:

```text
~/.cache/huggingface/lerobot/calibration/robots/dual_scorpion_follower/dual_scorpion_follower_2026_1026.json
```

The harness validates that file before connection. It checks:

- the file exists and its stem matches the robot ID,
- all 14 `left/right joint0..joint6` entries exist,
- motor IDs match joint0–joint6,
- each `drive_mode` is a supported direction flag,
- `homing_offset`, `range_min`, and `range_max` are present and internally valid,
- the bus calibration exactly matches the loaded file.

The two gripper calibration entries may be present and are reported, but this harness does not command grippers.

No calibration value is rewritten and no recalibration is performed.

### Home pose and current encoder pose

`config/mocopi/tracking.yaml` intentionally distinguishes two presets:

- `start_pose: work` initializes the kinematic model in a table-safe, bent-elbow home posture.
- `retarget_start_pose: neutral` is used by the existing mocopi replay pipeline when human motion is retargeted from a neutral model pose.

These presets initialize existing kinematic components. The real harness does **not** use either preset as a commanded startup pose. `connect_robot` only:

1. validates calibration,
2. connects both follower buses,
3. reads current encoder angles,
4. reports those angles as the initial state.

Before enabling torque, the existing execution path sends the measured present positions as the initial hold command. It then reads the encoders again and records the maximum joint step in `initial_hold`. If enabling the hold moves an arm more than the existing 2° start tolerance, execution aborts before the planned trajectory is sent.

## Measured table and 45 cm support-stick scene

The legacy calibrated URDF has joint frames but no collision meshes. Therefore the harness does not guess link widths. Real motion is rejected until the operator supplies a measured scene with:

- `status: measured`,
- a non-empty coordinate frame, source, and measurement time,
- `robot_to_scene` position and unit quaternion,
- `collision_margin_m`,
- an explicit `arm_collision_radius_m`,
- a `table` box with measured center, half-size, and orientation,
- a `support_stick` cylinder with measured center, radius, orientation, and `length_m: 0.45`,
- any additional measured obstacles.

The scene uses schema `dual_scorpion.agent.scene.v1`. Its YAML is deliberately not committed with guessed values. The operator must measure and record the actual table, stick, and robot-base relationship, then point `scene_geometry` in `config/mocopi/agent.example.yaml` to that local file.

The checker builds capsules between the existing URDF joint frames using the explicit arm radius, transforms the measured scene into the robot frame, and checks:

1. the current encoder pose before IK/execution,
2. every generated trajectory frame,
3. the current pose again immediately before execution.

Any initial contact, penetration, or clearance violation rejects motion. A `template_not_for_execution` scene may be parsed for review but never permits real motion.

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
| `set_gripper` | implemented software path; physical execution not verified | calibrated LeRobot gripper bus |
| `read_gripper_state` | implemented | calibrated LeRobot gripper bus |
| `hold_gripper` | implemented software path; physical execution not verified | calibrated LeRobot gripper bus |
| `release_gripper` | implemented software path; physical execution not verified | calibrated LeRobot gripper bus |
| `grasp` | implemented software path; physical grasp verification depends on configured thresholds | calibrated LeRobot gripper bus |

Arm replay remains joint0–joint6 only. The separate gripper path uses the existing calibrated `gripper` motor on the same LeRobot `DualScorpionFollower` bus.

## Gripper control and generic grasp

Each gripper is motor ID 8 and uses LeRobot's `MotorNormMode.RANGE_0_100`. The existing calibration supplies:

- motor ID,
- `drive_mode`,
- `homing_offset`,
- `range_min`,
- `range_max`.

LeRobot normalizes raw encoder positions into 0–100 and applies `drive_mode`. The harness does not infer open/closed direction from geometry. It reuses the existing teleoperation semantics:

- open: `0.0`
- closed: `45.0`

These values are configurable in `agent.example.yaml`, but the defaults match the existing teleoperation configuration.

Supported operations:

- open,
- close,
- set a normalized position,
- read current position/state,
- hold the current measured position,
- release/open,
- generic monitored `grasp`.

The `grasp` operation is object-agnostic. It closes the selected gripper and reads:

- normalized position,
- raw position,
- `Present_Load`,
- `Present_Current`.

Closed position alone is **not** treated as proof that an object is grasped. By default, `grasp` returns:

```json
{
  "grasp_verified": false,
  "grasp_state": "unverified",
  "verification_basis": ["encoder_position_only"]
}
```

If the operator supplies measured, positive `load_threshold` and `current_threshold` values, the harness marks a grasp as threshold-verified only when:

1. configured load and current thresholds are met, and
2. the encoder position stalls before the closed target.

Even then, the report names the evidence used; it does not claim a universal force-based grasp guarantee.

Gripper commands:

- are rejected outside the configured normalized range,
- are slew-rate limited,
- use a lock shared with arm execution on the same bus,
- honor the local stop event,
- report communication errors,
- never command the gripper during arm-only trajectory replay.

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
| `connect_robot` | connect both calibrated follower buses | local server pre-authorization |
| `get_world_state` | current provenance-labelled state | none |
| `list_skills` | skill contracts and implementation status | none |
| `capture_images` | MCP image content and metadata | none |
| `update_object_estimates` | semantic or validated object estimates | none |
| `submit_task_plan` | validate and store a task plan | none |
| `plan_motion` | generate and validate a joint trajectory | none |
| `execute_motion` | start a real execution job; optionally capture before/after images | local server pre-authorization |
| `read_gripper_state` | read one gripper's calibrated position, load, and current | none |
| `control_gripper` | open, close, set, hold, release, or attempt a generic monitored grasp | local server pre-authorization |
| `execute_task_plan` | execute arm and gripper steps in dependency order | local server pre-authorization |
| `stop_motion` | set the local stop event | none |
| `get_execution_report` | execution metrics and failure state | none |
| `get_replan_context` | failure, current state, and required observations | none |

## Safety and authorization

1. The operator creates a token file outside Git, for example `/run/user/1000/scorpion-agent-motion.token`.
2. The harness reads that path from its YAML config.
3. The local MCP server loads the token at startup.
4. No MCP tool accepts or returns the token. JOUZU and Gemini never see it.
5. The LLM cannot create, infer, or persist the token.
6. `stop_motion` is local and always available.
7. Execution is rejected unless the buses are connected, calibration is valid, encoder feedback is fresh and finite, the measured table/stick scene is loaded, the current pose is clear, and the generated trajectory passes the existing validator.
8. On stop, tracking error, limit violation, initial-hold movement, table/stick clearance violation, or communication failure, the execution loop holds the measured pose and disconnects without commanding the gripper.

Physical robot operation still requires the operator's physical safety procedure and explicit authorization. Software tests do not authorize or verify real motion.

## Execution feedback and evaluation

The existing real execution path records:

- commanded and measured joint angles,
- maximum joint tracking error,
- TCP position and orientation error,
- execution time and measured FPS,
- stop, initial-hold motion, and failure state,
- gripper command count (always zero in this harness),
- model and URDF fingerprints.

Evaluation uses only machine-checkable conditions. Semantic image success is not inferred from motor feedback; JOUZU/Gemini may inspect before/after images separately after execution.

Task plans may combine `move_arm_to_pose`, `set_gripper`, `read_gripper_state`, `hold_gripper`, `release_gripper`, and `grasp`. `execute_task_plan` topologically sorts the steps, checks that each dependency completed, and stops at the first failed step.

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
- The local calibration file was validated, but its physical correspondence to the current motors was **not** re-measured.
- The measured table, 45 cm stick, robot-base transform, and arm collision radius must be supplied by the operator. No guessed scene is committed.
- Gripper software control is integrated and locally tested with mocks, but **physical gripper motion and actual object grasping are not verified**.
- Default `grasp` results are `unverified` unless measured load/current thresholds are explicitly configured.
- Camera-to-robot extrinsics and metric 3D object pose estimation are **not implemented**.
- Image semantic understanding is delegated to JOUZU/Gemini and was **not** tested here.
- Collision checking covers the explicitly measured table, 45 cm stick, operator-declared boxes/cylinders, and URDF joint-frame capsules. Arbitrary unmeasured environments are not inferred.
- The current motion path uses the legacy calibrated URDF/PyBullet planner; the unmeasured full-CAD MuJoCo model is not authorized for hardware replay.
