import io
import shutil
import subprocess
import time
from contextlib import closing
from dataclasses import replace

import numpy as np
import pytest

from lerobot.remote_policy.adapters.mock import MockPolicy, MockRobot
from lerobot.remote_policy.core import InferenceBackend, LocalTransport, PolicyClient
from lerobot.remote_policy.exchange import (
    decode_json,
    decode_observation,
    decode_output,
    encode_observation,
    encode_output,
)
from lerobot.remote_policy.grpc_transport import GrpcTransport, create_server
from lerobot.remote_policy.schema import ABSOLUTE, InferenceRequest, PolicyCapabilities, PolicyError


def mock_backend():
    robot = MockRobot()
    caps = robot.get_capabilities()
    policy = MockPolicy(
        PolicyCapabilities(
            "mock",
            (caps.robot_type,),
            caps.images,
            caps.state_features,
            caps.state_units,
            caps.action_features,
            caps.action_units,
            4,
            ABSOLUTE,
            30,
        )
    )
    return robot, policy, InferenceBackend(policy)


def test_offline_npz_round_trip_and_invalid_archive():
    robot, _, backend = mock_backend()
    obs = robot.get_observation("hold")
    images = {"untrusted/name": obs.images["front"]}
    obs = replace(obs, images=images, image_timestamps_ns={"untrusted/name": obs.timestamp_ns - 1})
    loaded = decode_observation(encode_observation(obs))
    assert loaded.task == obs.task
    assert loaded.image_timestamps_ns != {"untrusted/name": loaded.timestamp_ns}
    assert loaded.observation_id == obs.observation_id
    np.testing.assert_array_equal(loaded.images["untrusted/name"], obs.images["untrusted/name"])
    np.testing.assert_array_equal(loaded.state, obs.state)

    output = PolicyClient(LocalTransport(backend)).predict(robot.get_observation())
    actual = decode_output(encode_output(output))
    assert actual.request_id == output.request_id
    assert actual.observation_id == output.observation_id
    np.testing.assert_array_equal(actual.actions, output.actions)
    with pytest.raises(PolicyError, match="invalid NPZ"):
        decode_observation(b"arbitrary object bytes")
    with pytest.raises(PolicyError, match="expected a observation"):
        decode_observation(encode_output(output))

    with np.load(io.BytesIO(encode_output(output)), allow_pickle=False) as data:
        buffer = io.BytesIO()
        np.savez(buffer, header=data["header"], actions=np.array([{"untrusted": "object"}], dtype=object))
    with pytest.raises(PolicyError, match="unsafe"):
        decode_output(buffer.getvalue())

    with pytest.raises(PolicyError, match="invalid exchange JSON"):
        decode_json(b'{"robot_id":"first","robot_id":"second"}')


def test_grpc_round_trip_with_auth_and_timeout():
    pytest.importorskip("grpc")
    robot, policy, backend = mock_backend()
    server, port = create_server(backend, "grpc://127.0.0.1:0", "correct-token")
    endpoint = f"grpc://127.0.0.1:{port}"
    server.start()
    try:
        with closing(GrpcTransport(endpoint, "incorrect-token")) as transport:
            with pytest.raises(PolicyError, match="authentication"):
                transport.get_capabilities(timeout=1)
            caps = backend.get_capabilities()
            request = InferenceRequest("request", caps.fingerprint, robot.get_observation())
            with pytest.raises(PolicyError, match="authentication"):
                transport.infer(request, timeout=1)
        with closing(GrpcTransport(endpoint, "correct-token")) as transport:
            client = PolicyClient(transport, policy_id="mock")
            output, validation = client.run(robot)
            assert output.actions.shape == (4, 2)
            assert validation.frames == 4 and not validation.blockers
            assert robot.executed_chunks == []
            original = policy.predict

            def slow(inputs):
                time.sleep(0.15)
                return original(inputs)

            policy.predict = slow
            with pytest.raises(TimeoutError, match="deadline"):
                transport.infer(request, timeout=0.02)
    finally:
        server.stop(grace=0).wait()


def test_remote_plaintext_refused():
    with pytest.raises(PolicyError, match="require TLS"):
        GrpcTransport("grpc://192.0.2.10:50051", "token")


def test_grpc_tls_is_verified(tmp_path):
    pytest.importorskip("grpc")
    if not shutil.which("openssl"):
        pytest.skip("openssl is required for the TLS integration test")
    _, _, backend = mock_backend()
    certificate, key = tmp_path / "cert.pem", tmp_path / "key.pem"
    subprocess.run(
        [
            "openssl",
            "req",
            "-x509",
            "-newkey",
            "rsa:2048",
            "-nodes",
            "-days",
            "1",
            "-subj",
            "/CN=localhost",
            "-addext",
            "subjectAltName=DNS:localhost",
            "-keyout",
            str(key),
            "-out",
            str(certificate),
        ],
        check=True,
        capture_output=True,
    )
    server, port = create_server(
        backend,
        "grpcs://127.0.0.1:0",
        "tls-token",
        tls_certificate=certificate.read_bytes(),
        tls_key=key.read_bytes(),
    )
    server.start()
    try:
        with closing(
            GrpcTransport(
                f"grpcs://localhost:{port}", "tls-token", root_certificates=certificate.read_bytes()
            )
        ) as transport:
            assert transport.get_capabilities(timeout=2).policy_id == "mock"
        with (
            closing(GrpcTransport(f"grpcs://localhost:{port}", "tls-token")) as transport,
            pytest.raises(PolicyError, match="UNAVAILABLE"),
        ):
            transport.get_capabilities(timeout=2)
    finally:
        server.stop(grace=0).wait()
