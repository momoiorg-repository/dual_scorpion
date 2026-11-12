# MCP tool contracts

The local MCP server is started with:

```bash
uv run --no-sync --active python -m telegrip.mocopi.agent --config config/mocopi/agent.example.yaml mcp
```

JOUZU may connect it through a project `.pi/mcp.json` local-server entry using the same command and an absolute `cwd`. The project does not store JOUZU credentials or model configuration.

All arguments are JSON objects. Results include machine-readable `structuredContent`.

| Tool | Purpose | Authorization |
| --- | --- | --- |
| `get_status` | Robot, camera, safety, execution, and skill status | None |
| `connect_robot` | Validate calibration, connect follower buses, and read current encoders without homing | None |
| `get_world_state` | Current provenance-labelled robot/camera/object state | None |
| `list_skills` | Robot Skill contracts and implementation status | None |
| `capture_images` | MCP image content plus camera ID, timestamp, size, hash | None |
| `update_object_estimates` | Store semantic or validated object estimates | None |
| `submit_task_plan` | Validate a `dual_scorpion.agent.task_plan.v2` plan | None |
| `plan_motion` | Generate and validate a joint trajectory | None |
| `execute_motion` | Start the real arm-motion job; optionally capture before/after images | One-shot local arm grant |
| `read_gripper_state` | Read one gripper's calibrated position, state, load, and current | None |
| `control_gripper` | Open, close, set, hold, release, or attempt a generic monitored grasp | One-shot local gripper grant |
| `execute_task_plan` | Execute arm and gripper steps in dependency order | One-shot local task-plan grant |
| `stop_motion` | Set the local stop event | None |
| `get_execution_report` | Return execution metrics and failure state | None |
| `get_replan_context` | Return failure and observation requirements | None |

Motion plans must use Cartesian targets from one of these sources:

- `relative_position_m`, relative to the current validated TCP pose.
- `target_object_id` plus an explicit offset, where the object has a validated metric pose.
- `target_position_m` with a `pose_validation` object containing method, timestamp, and uncertainty.

Direct raw joint and motor goal fields are rejected. Gripper operations must use the supported skills or MCP tools; the harness converts the requested action to the calibrated LeRobot command.

Gripper positions use LeRobot `RANGE_0_100` normalization. The existing teleoperation semantics are open `0.0` and closed `45.0`. A closed position does not prove that an object is grasped. Default `grasp` is `unverified`; configured load/current thresholds plus encoder stall can produce `threshold_verified`.

For a small gripper-only test, the operator creates a one-shot local grant before calling the MCP tool:

```bash
uv run --no-sync --active python -m telegrip.mocopi.agent \
  --config config/mocopi/agent.example.yaml authorize-gripper \
  --arm left --action set_position --max-position-delta 1.0 \
  --expires-in 300 --confirm-fingertip-clearance
```

The tool request must stay within that arm, action, and position range. The grant is deleted when accepted.

Before `plan_motion` or `execute_motion`, `get_status` must report:

- a valid LeRobot calibration for the selected robot ID,
- a `measured` scene,
- the table and 45 cm support stick,
- a measured scene-to-robot transform,
- an explicit arm collision radius.

If any of these are missing, the harness rejects real motion rather than inferring geometry.
