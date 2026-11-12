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
| `connect_robot` | Connect both calibrated follower buses | Operator token |
| `get_world_state` | Current provenance-labelled robot/camera/object state | None |
| `list_skills` | Robot Skill contracts and implementation status | None |
| `capture_images` | MCP image content plus camera ID, timestamp, size, hash | None |
| `update_object_estimates` | Store semantic or validated object estimates | None |
| `submit_task_plan` | Validate a `dual_scorpion.agent.task_plan.v2` plan | None |
| `plan_motion` | Generate and validate a joint trajectory | None |
| `execute_motion` | Start the real follower execution job; optionally capture before/after images | Operator token |
| `stop_motion` | Set the local stop event | None |
| `get_execution_report` | Return execution metrics and failure state | None |
| `get_replan_context` | Return failure and observation requirements | None |

Motion plans must use Cartesian targets from one of these sources:

- `relative_position_m`, relative to the current validated TCP pose.
- `target_object_id` plus an explicit offset, where the object has a validated metric pose.
- `target_position_m` with a `pose_validation` object containing method, timestamp, and uncertainty.

Direct joint, motor, or gripper command fields are rejected.
