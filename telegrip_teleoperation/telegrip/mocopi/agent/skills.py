"""Robot Skill catalogue for the real Dual Scorpion harness.

The catalogue describes actual harness backends. Unavailable capabilities are
declared as unavailable; they are never represented as working skills.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class SkillSpec:
    name: str
    description: str
    category: str
    input_contract: dict[str, Any]
    output_contract: dict[str, Any]
    preconditions: list[str]
    postconditions: list[str]
    failure_conditions: list[str]
    required_hardware: list[str]
    safety_constraints: list[str]
    backend: str
    implementation_status: str

    def to_dict(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "description": self.description,
            "category": self.category,
            "input_contract": dict(self.input_contract),
            "output_contract": dict(self.output_contract),
            "preconditions": list(self.preconditions),
            "postconditions": list(self.postconditions),
            "failure_conditions": list(self.failure_conditions),
            "required_hardware": list(self.required_hardware),
            "safety_constraints": list(self.safety_constraints),
            "backend": self.backend,
            "implementation_status": self.implementation_status,
        }


class SkillRegistry:
    def __init__(self) -> None:
        self._skills: dict[str, SkillSpec] = {}

    def register(self, spec: SkillSpec) -> None:
        if spec.name in self._skills:
            raise ValueError(f"skill {spec.name!r} is already registered")
        self._skills[spec.name] = spec

    def names(self) -> set[str]:
        return set(self._skills)

    def available_names(self) -> set[str]:
        return {
            name
            for name, spec in self._skills.items()
            if spec.implementation_status != "unavailable"
        }

    def get(self, name: str) -> SkillSpec:
        if name not in self._skills:
            raise KeyError(f"skill {name!r} is not registered")
        return self._skills[name]

    def specs(self) -> list[dict[str, Any]]:
        return [self._skills[name].to_dict() for name in sorted(self._skills)]


def built_in_skill_registry() -> SkillRegistry:
    registry = SkillRegistry()
    for spec in (
        SkillSpec(
            name="read_robot_state",
            description="Read calibrated joint angles and connection status from the real follower buses.",
            category="robot_state",
            input_contract={"required": {}, "optional": {}},
            output_contract={"arms": "left/right joint arrays and timestamps", "connection": "boolean"},
            preconditions=["Both Feetech buses are connected.", "Calibration is loaded and valid."],
            postconditions=["Fresh finite joint readings are returned with a monotonic timestamp."],
            failure_conditions=["A bus is disconnected.", "An encoder value is missing or non-finite."],
            required_hardware=["left follower bus", "right follower bus"],
            safety_constraints=["Read-only; no motor command is sent."],
            backend="telegrip.mocopi.agent.robot.RealRobotAdapter",
            implementation_status="implemented",
        ),
        SkillSpec(
            name="get_world_state",
            description="Return robot, camera, and explicitly labelled object estimates in one WorldState.",
            category="environment_state",
            input_contract={"required": {}, "optional": {}},
            output_contract={"world_state": "dual_scorpion.agent.world_state.v1"},
            preconditions=["Robot and camera providers are configured."],
            postconditions=["Every value reports source, timestamp, and validity."],
            failure_conditions=["A provider raises or returns invalid state."],
            required_hardware=[],
            safety_constraints=["Semantic estimates are never relabelled as measurements."],
            backend="telegrip.mocopi.agent.harness.ScorpionAgentHarness",
            implementation_status="implemented",
        ),
        SkillSpec(
            name="capture_image",
            description="Capture one or more camera frames and return MCP image content with metadata.",
            category="perception",
            input_contract={"required": {}, "optional": {"camera_ids": "string array"}},
            output_contract={"images": "MCP image content", "metadata": "timestamps and camera IDs"},
            preconditions=["A configured camera backend can open the device."],
            postconditions=["Frame bytes, MIME type, timestamp, and source are returned."],
            failure_conditions=["Camera open/read fails.", "Frame timestamp is stale."],
            required_hardware=["one or more UVC cameras"],
            safety_constraints=["Image capture is read-only and has no motion authority."],
            backend="telegrip.mocopi.agent.cameras.CameraService",
            implementation_status="implemented",
        ),
        SkillSpec(
            name="move_arm_to_pose",
            description="Plan and execute a validated Cartesian arm motion on the real robot.",
            category="motion",
            input_contract={
                "required": {"target": "validated object pose, absolute validated pose, or relative pose"},
                "optional": {"arms": "left/right/both", "orientation": "unit quaternion"},
            },
            output_contract={"execution_id": "string", "report": "JSON metrics"},
            preconditions=[
                "A matching one-shot local arm-motion grant is present.",
                "Robot is connected, calibrated, and feedback is fresh.",
                "A measured table, 45 cm support-stick, and arm-radius scene is loaded.",
                "IK, joint limits, and trajectory continuity checks pass.",
                "The current encoder pose and every trajectory frame are clear of the measured scene.",
            ],
            postconditions=["Measured joint and TCP errors are within declared tolerances."],
            failure_conditions=["IK is unreachable.", "A joint limit or rate limit is exceeded.", "Feedback is stale or tracking error grows."],
            required_hardware=["left follower bus", "right follower bus", "calibrated Dual Scorpion arms"],
            safety_constraints=[
                "LLM joint commands are rejected.",
                "Gripper commands use the calibrated LeRobot RANGE_0_100 path.",
                "No automatic homing occurs; planning starts from measured encoders.",
                "Initial table contact or insufficient clearance rejects execution.",
                "Stop requests and local safety checks take precedence over the plan.",
            ],
            backend="telegrip.mocopi.agent.motion + telegrip.mocopi.dataset.real.execute_plan",
            implementation_status="implemented_software_only_physical_execution_not_verified",
        ),
        SkillSpec(
            name="validate_motion",
            description="Validate a generated joint trajectory against FK, calibration limits, and continuity.",
            category="validation",
            input_contract={"required": {"motion_plan_id": "string"}},
            output_contract={"valid": "boolean", "errors": "string array", "metrics": "object"},
            preconditions=["A motion plan has been generated from a validated task plan."],
            postconditions=["Every trajectory frame passes the existing real-replay validator."],
            failure_conditions=["FK mismatch.", "Joint limit or rate violation.", "Invalid trajectory timing."],
            required_hardware=[],
            safety_constraints=["Validation never sends a motor command."],
            backend="telegrip.mocopi.dataset.real.validate_plan",
            implementation_status="implemented",
        ),
        SkillSpec(
            name="stop_motion",
            description="Request a local, immediate stop of the active real-robot execution job.",
            category="safety",
            input_contract={"required": {}, "optional": {}},
            output_contract={"stop_requested": "boolean"},
            preconditions=["An execution job or harness is running."],
            postconditions=["The stop event is set and the execution loop holds the current pose."],
            failure_conditions=["The stop event cannot be set.", "The robot adapter cannot hold the pose."],
            required_hardware=["follower buses"],
            safety_constraints=["This is a local safety action and does not depend on an LLM."],
            backend="telegrip.mocopi.agent.safety.SafetyManager",
            implementation_status="implemented",
        ),
        SkillSpec(
            name="set_gripper",
            description="Open, close, or move one or both calibrated Dual Scorpion grippers to a normalized position.",
            category="gripper",
            input_contract={
                "required": {"action": "open, close, or set_position"},
                "optional": {"position": "normalized 0-100 value required for set_position"},
            },
            output_contract={
                "status": "complete or stopped",
                "final_position": "normalized gripper position",
                "position_error": "absolute normalized error",
            },
            preconditions=[
                "Robot buses are connected and calibrated.",
                "A matching one-shot local gripper grant is present.",
                "The requested position is within the calibrated normalized range.",
            ],
            postconditions=["The gripper reaches the requested position within tolerance or stops safely."],
            failure_conditions=[
                "The gripper bus is disconnected.",
                "The position is outside the calibrated range.",
                "A stop request or communication error occurs.",
            ],
            required_hardware=["left or right gripper motor"],
            safety_constraints=[
                "Commands are slew-rate limited by configuration.",
                "The same bus is protected by a local lock shared with arm execution.",
                "No automatic homing occurs.",
            ],
            backend="telegrip.mocopi.agent.robot.RealRobotAdapter",
            implementation_status="implemented_software_only_physical_execution_not_verified",
        ),
        SkillSpec(
            name="read_gripper_state",
            description="Read the current normalized position, calibration, load, and current of one gripper.",
            category="gripper",
            input_contract={"required": {}, "optional": {}},
            output_contract={"position": "normalized value", "state": "open/closed/intermediate"},
            preconditions=["The selected gripper bus is connected and calibrated."],
            postconditions=["Fresh state is returned with a monotonic timestamp."],
            failure_conditions=["The bus is disconnected.", "The position is non-finite."],
            required_hardware=["left or right gripper motor"],
            safety_constraints=["Read-only; no motor command is sent."],
            backend="telegrip.mocopi.agent.robot.RealRobotAdapter.read_gripper_state",
            implementation_status="implemented",
        ),
        SkillSpec(
            name="hold_gripper",
            description="Hold the selected gripper at its current measured position.",
            category="gripper",
            input_contract={"required": {}, "optional": {}},
            output_contract={"status": "complete or stopped", "final_position": "normalized value"},
            preconditions=["The selected gripper bus is connected and calibrated."],
            postconditions=["The current measured position is commanded as the hold target."],
            failure_conditions=["The bus is disconnected.", "A stop request or communication error occurs."],
            required_hardware=["left or right gripper motor"],
            safety_constraints=["No new open/close target is inferred."],
            backend="telegrip.mocopi.agent.robot.RealRobotAdapter.hold_gripper",
            implementation_status="implemented_software_only_physical_execution_not_verified",
        ),
        SkillSpec(
            name="release_gripper",
            description="Open the selected gripper to its configured release/open position.",
            category="gripper",
            input_contract={"required": {}, "optional": {}},
            output_contract={"status": "complete or stopped", "final_position": "normalized value"},
            preconditions=["The selected gripper bus is connected and calibrated."],
            postconditions=["The gripper reaches the configured open position within tolerance."],
            failure_conditions=["The bus is disconnected.", "A stop request or communication error occurs."],
            required_hardware=["left or right gripper motor"],
            safety_constraints=["Uses the configured open position; no object-specific release sequence is inferred."],
            backend="telegrip.mocopi.agent.robot.RealRobotAdapter.release_gripper",
            implementation_status="implemented_software_only_physical_execution_not_verified",
        ),
        SkillSpec(
            name="grasp",
            description="Close the selected gripper and monitor encoder, load, and current without assuming a particular object.",
            category="gripper",
            input_contract={"required": {}, "optional": {}},
            output_contract={
                "position_reached": "boolean",
                "grasp_verified": "boolean",
                "grasp_state": "threshold_verified or unverified",
            },
            preconditions=[
                "The selected gripper bus is connected and calibrated.",
                "A matching one-shot local gripper grant is present.",
            ],
            postconditions=["The gripper closes or stalls safely and the measured evidence is returned."],
            failure_conditions=["The bus is disconnected.", "A stop request or communication error occurs."],
            required_hardware=["left or right gripper motor"],
            safety_constraints=[
                "Closed position is not treated as proof of a successful grasp.",
                "Grasp is marked verified only when configured load/current thresholds and encoder stall agree.",
                "Otherwise the result remains unverified.",
            ],
            backend="telegrip.mocopi.agent.robot.RealRobotAdapter.grasp",
            implementation_status="implemented_software_only_physical_execution_not_verified",
        ),
    ):
        registry.register(spec)
    return registry
