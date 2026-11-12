# Safety and authorization

The harness enforces these rules locally:

- The LLM cannot create, infer, or persist motion authorization.
- The operator creates a one-shot local grant outside Git. It specifies operation, arms, actions, position range, plan ID where applicable, and expiry.
- No MCP tool accepts or exposes a secret; JOUZU and Gemini never receive it.
- The harness consumes and deletes the grant when a matching request is accepted. A failed request also consumes it.
- `stop_motion` is always available and does not depend on model reasoning.
- Motion is rejected unless the follower buses are connected and calibrated.
- Arm motion is rejected unless the measured table, 45 cm support stick, scene-to-robot transform, and arm collision radius are present.
- Arm motion is rejected if the current encoder pose or any trajectory frame contacts or violates the measured scene clearance.
- Gripper-only motion does not require the full 3D scene, but it requires a one-shot grant with explicit fingertip-clearance confirmation and a small configured position range.
- Encoder feedback must be fresh and finite.
- Generated trajectories are checked by FK, joint limits, calibrated motor limits, timing, and per-step continuity before execution.
- Arm-only replay never commands the gripper. The separate gripper path uses the same calibrated LeRobot bus and a shared local lock.
- Gripper commands are range-checked, slew-rate limited, stop-aware, and never infer a grasp success from position alone.
- Before enabling torque, the execution loop commands the measured present positions, re-reads the encoders, and rejects unexpected initial-hold motion.

The harness never homes the robot automatically. The `start_pose: work` and `retarget_start_pose: neutral` values initialize existing kinematic/replay components; real harness planning starts from the current encoder pose.

The current software path has not been physically driven during development. Treat a successful mock or local software test as not equivalent to verified real-robot motion.
