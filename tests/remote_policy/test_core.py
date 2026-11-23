import time
from dataclasses import replace

import numpy as np
import pytest

from lerobot.remote_policy.adapters.mock import MockPolicy, MockRobot
from lerobot.remote_policy.core import (
    InferenceBackend,
    LocalTransport,
    PolicyClient,
    negotiate,
    resolve_joint_chunk,
)
from lerobot.remote_policy.schema import (
    ABSOLUTE,
    DELTA,
    RELATIVE,
    InferenceRequest,
    JointLimit,
    PolicyCapabilities,
    PolicyError,
    PolicyOutput,
    StaleResponseError,
)


def policy_for(robot, *, horizon=3, representation=ABSOLUTE):
    caps = robot.get_capabilities()
    return PolicyCapabilities(
        "hold",
        (caps.robot_type,),
        caps.images,
        tuple(reversed(caps.state_features)),
        caps.state_units,
        tuple(reversed(caps.action_features)),
        caps.action_units,
        horizon,
        representation,
        30,
        absolute_features=("gripper",)
        if representation != ABSOLUTE and "gripper" in caps.action_features
        else (),
    )


def test_swappable_robot_and_policy_and_dry_run():
    for robot, expected, cameras, mode in (
        (MockRobot(), (3, 2), 1, RELATIVE),
        (
            MockRobot(
                robot_type="other",
                state_features=("a", "b", "c", "d"),
                action_features=("a", "b", "c"),
                images=("left", "right"),
            ),
            (3, 3),
            2,
            ABSOLUTE,
        ),
        (MockRobot(robot_type="no_camera", state_features=("a",), images=()), (3, 1), 0, DELTA),
    ):
        caps = policy_for(robot, representation=mode)
        policy = MockPolicy(caps)
        policy.load(None)
        client = PolicyClient(LocalTransport(InferenceBackend(policy)))
        observation = robot.get_observation()
        assert len(observation.images) == cameras
        assert observation.state.size == len(caps.state_features)
        output, report = client.run(robot)
        assert report.frames == 3
        assert output.actions.shape == expected
        assert robot.executed_chunks == []
        client.run(robot, dry_run=False)
        assert len(robot.executed_chunks) == 1


@pytest.mark.parametrize(
    ("changes", "error"),
    [
        ({"robot_types": ("other",)}, "does not support robot"),
        ({"required_images": ("side",)}, "missing required cameras"),
        ({"state_units": {"joint": "rad", "gripper": "lerobot_0_100"}}, "state units mismatch"),
        ({"action_units": {"joint": "rad", "gripper": "lerobot_0_100"}}, "action units mismatch"),
        ({"action_horizon": 101}, "horizon"),
        ({"control_fps": 60}, "fps"),
        ({"protocol_version": "2.0"}, "Unsupported policy protocol"),
        ({"action_representation": "cartesian_pose"}, "Unsupported action representation"),
    ],
)
def test_capability_negotiation(changes, error):
    robot = MockRobot()
    caps = replace(policy_for(robot), **changes)
    with pytest.raises(PolicyError, match=error):
        negotiate(robot.get_capabilities(), caps)


def test_wrong_model_is_rejected_before_capture(monkeypatch):
    robot = MockRobot()
    client = PolicyClient(LocalTransport(InferenceBackend(MockPolicy(policy_for(robot)))), policy_id="wrong")
    monkeypatch.setattr(robot, "get_observation", lambda *args: pytest.fail("capture was called"))
    with pytest.raises(PolicyError, match="expected policy"):
        client.run(robot)


def test_backend_enforces_contract_fingerprint_and_chunk_length():
    robot = MockRobot()
    policy = MockPolicy(policy_for(robot))
    backend = InferenceBackend(policy)
    observation = robot.get_observation()
    with pytest.raises(PolicyError, match="capabilities changed"):
        backend.infer(InferenceRequest("request", "0" * 64, observation))

    policy.postprocess = lambda actions: actions[:-1]
    request = InferenceRequest("request", backend.get_capabilities().fingerprint, observation)
    with pytest.raises(PolicyError, match="negotiated horizon"):
        backend.infer(request)


def test_client_checks_request_observation_identity_and_staleness():
    robot = MockRobot()
    transport = LocalTransport(InferenceBackend(MockPolicy(policy_for(robot))))
    infer = transport.infer
    client = PolicyClient(transport, timeout_s=1, max_response_age_s=0.02)
    observation = robot.get_observation()

    transport.infer = lambda request, **kw: replace(infer(request, **kw), request_id="previous")
    with pytest.raises(PolicyError, match="request ID"):
        client.predict(observation)
    transport.infer = lambda request, **kw: replace(infer(request, **kw), observation_id="previous")
    with pytest.raises(PolicyError, match="observation ID"):
        client.predict(observation)

    def delayed(request, **kw):
        response = infer(request, **kw)
        time.sleep(0.03)
        return response

    transport.infer = delayed
    with pytest.raises(StaleResponseError, match="age budget"):
        client.predict(observation)

    transport.infer = infer
    camera = replace(observation, image_timestamps_ns={"front": 0})
    robot.get_observation = lambda task: camera
    with pytest.raises(StaleResponseError, match="camera frame"):
        client.run(robot)


def test_action_validator_conversion_order_and_independent_gripper():
    robot = MockRobot()
    observation = replace(robot.get_observation(), state=np.array([5, 50]))
    caps = robot.get_capabilities()
    limits = {"joint": JointLimit(-10, 10, 15, 2), "gripper": JointLimit(0, 100, 60, 3)}
    output = PolicyOutput(
        "request",
        observation.observation_id,
        "policy",
        np.array([[51, 0.25], [52, 0.5]]),
        ("gripper", "joint"),
        RELATIVE,
        caps.action_units,
        30,
        ("gripper",),
    )
    result = resolve_joint_chunk(output, observation, caps, limits)
    np.testing.assert_allclose(result, [[5.25, 51], [5.5, 52]])
    result = resolve_joint_chunk(replace(output, action_representation=DELTA), observation, caps, limits)
    np.testing.assert_allclose(result, [[5.25, 51], [5.75, 52]])

    for changed, error in (
        (replace(output, actions=[[101, 0]]), "gripper.*allowed range"),
        (replace(output, actions=[[50, 3]]), "discontinuity"),
        (replace(output, actions=[[50, 1]]), "velocity limit"),
        (replace(output, action_representation="cartesian"), "Unsupported action representation"),
        (replace(output, observation_id="mismatch"), "does not match the reference"),
    ):
        with pytest.raises(PolicyError, match=error):
            resolve_joint_chunk(changed, observation, caps, limits)


@pytest.mark.parametrize("bad", (np.array([[np.nan, 0]]), [[np.inf, 0]], [[0.0]], [[True, False]], [0, 0]))
def test_schema_rejects_nonfinite_values_and_invalid_shapes(bad):
    with pytest.raises(PolicyError):
        PolicyOutput("id", "obs", "p", bad, ("a", "b"), ABSOLUTE, {"a": "deg", "b": "deg"}, 2)
