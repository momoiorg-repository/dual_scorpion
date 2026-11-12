# Safety and authorization

The harness enforces these rules locally:

- The LLM cannot create, infer, or persist motion authorization.
- The operator supplies a token through a local file outside Git and passes it only to the authorized MCP calls.
- `connect_robot` and `execute_motion` require that token.
- `stop_motion` is always available and does not depend on model reasoning.
- Motion is rejected unless the follower buses are connected and calibrated.
- Motion is rejected unless the measured table, 45 cm support stick, scene-to-robot transform, and arm collision radius are present.
- Motion is rejected if the current encoder pose or any trajectory frame contacts or violates the measured scene clearance.
- Encoder feedback must be fresh and finite.
- Generated trajectories are checked by FK, joint limits, calibrated motor limits, timing, and per-step continuity before execution.
- Arm-only replay never commands the gripper. The separate gripper path uses the same calibrated LeRobot bus and a shared local lock.
- Gripper commands are range-checked, slew-rate limited, stop-aware, and never infer a grasp success from position alone.
- Before enabling torque, the execution loop commands the measured present positions, re-reads the encoders, and rejects unexpected initial-hold motion.

The harness never homes the robot automatically. The `start_pose: work` and `retarget_start_pose: neutral` values initialize existing kinematic/replay components; real harness planning starts from the current encoder pose.

The current software path has not been physically driven during development. Treat a successful mock or local software test as not equivalent to verified real-robot motion.
