# Safety and authorization

The harness enforces these rules locally:

- The LLM cannot create, infer, or persist motion authorization.
- The operator supplies a token through a local file outside Git and passes it only to the authorized MCP calls.
- `connect_robot` and `execute_motion` require that token.
- `stop_motion` is always available and does not depend on model reasoning.
- Motion is rejected unless the follower buses are connected and calibrated.
- Encoder feedback must be fresh and finite.
- Generated trajectories are checked by FK, joint limits, calibrated motor limits, timing, and per-step continuity before execution.
- The real execution loop holds the last measured pose on errors and never commands the gripper.

The current software path has not been physically driven during development. Treat a successful mock or local software test as not equivalent to verified real-robot motion.
