"""Local YAML profiles and adapter registration."""

from __future__ import annotations

import os
from dataclasses import dataclass
from importlib import import_module
from pathlib import Path
from typing import Any

import yaml

from .adapters.mock import MockPolicy, MockRobot
from .core import InferenceBackend, LocalTransport, PolicyClient, RobotAdapter
from .exchange import MAX_JSON_BYTES, read_file
from .schema import PolicyCapabilities, PolicyError, require_text

ROBOTS = {
    "mock": MockRobot,
    "dual_scorpion": "lerobot.remote_policy.adapters.dual_scorpion:DualScorpionAdapter",
}
POLICIES = {
    "mock": MockPolicy,
    "pi05": "lerobot.remote_policy.adapters.pi05:Pi05Adapter",
    "groot_n17": "lerobot.remote_policy.adapters.groot_n17:GrootN17Adapter",
}


@dataclass(frozen=True)
class Profile:
    path: Path
    data: dict[str, Any]

    @classmethod
    def load(cls, path: Path | None) -> Profile:
        if path is None:
            return cls(Path.cwd(), {})
        path = path.expanduser().resolve()
        raw = yaml.safe_load(read_file(path, MAX_JSON_BYTES))
        if not isinstance(raw, dict) or raw.keys() - {"robot", "policy", "inference", "execution"}:
            raise PolicyError("profile must contain only robot, policy, inference, and execution sections")
        return cls(path.parent, raw)

    def section(self, name: str) -> dict[str, Any]:
        section = self.data.get(name, {})
        if not isinstance(section, dict):
            raise PolicyError(f"{name} must be a mapping")
        return section

    def resolve(self, path: str) -> Path:
        return (self.path / Path(path).expanduser()).resolve()

    def robot(self, override: str | None = None) -> RobotAdapter:
        config = self.section("robot")
        adapter = override or config.get("adapter")
        factory = _factory(ROBOTS, adapter, config)
        options = _options(config)
        if config.get("config") is not None:
            options["config"] = self.resolve(config["config"])
        return factory(**options)

    def backend(self, override: str | None = None) -> InferenceBackend:
        config = self.section("policy")
        adapter = override or config.get("adapter")
        factory = _factory(POLICIES, adapter, config)
        capabilities = PolicyCapabilities.from_dict(config.get("capabilities"))
        options = _options(config)
        policy = factory(capabilities, **options)
        checkpoint = config.get("checkpoint")
        if checkpoint is not None:
            require_text(checkpoint, "policy checkpoint")
            if checkpoint.startswith((".", "/", "~")):
                checkpoint = str(self.resolve(checkpoint))
        policy.load(checkpoint)
        return InferenceBackend(policy)

    def client(self, override: str | None = None) -> PolicyClient:
        inference = self.section("inference")
        transport = inference.get("transport", "local")
        policy_id = self.section("policy").get("id")
        if transport == "local":
            adapter = LocalTransport(self.backend(override))
        elif transport == "grpc":
            from .grpc_transport import GrpcTransport

            endpoint = inference.get("endpoint")
            require_text(endpoint, "inference.endpoint")
            policy_id = override or policy_id
            require_text(policy_id, "policy.id or --policy for remote inference")
            adapter = GrpcTransport(endpoint, self.token(), root_certificates=self.pem("ca_file"))
        else:
            raise PolicyError(f"Unsupported transport: {transport!r}")
        try:
            return PolicyClient(
                adapter,
                policy_id=policy_id,
                timeout_s=inference.get("timeout_s", 30),
                max_response_age_s=inference.get("max_response_age_s", 30),
            )
        except Exception:
            adapter.close()
            raise

    def token(self) -> str:
        key = self.section("inference").get("token_env", "ROBOT_POLICY_TOKEN")
        require_text(key, "inference.token_env")
        token = os.getenv(key)
        if not token:
            raise PolicyError(f"set the local environment variable {key} for gRPC authentication")
        return token

    def pem(self, key: str) -> bytes | None:
        path = self.section("inference").get(key)
        return None if path is None else read_file(self.resolve(path), MAX_JSON_BYTES)

    def dry_run(self, execute: bool) -> bool:
        dry_run = self.section("execution").get("dry_run", True)
        if type(dry_run) is not bool:
            raise PolicyError("execution.dry_run must be a boolean")
        if dry_run and execute:
            raise PolicyError("execution requires execution.dry_run: false and an explicit --execute")
        return not execute


def _factory(registry: dict, name: str | None, config: dict[str, Any]):
    require_text(name, "adapter name")
    plugin = config.get("plugin")
    if plugin is not None:
        if name != config.get("adapter") or name in registry:
            raise PolicyError("plugin must use its own adapter name")
        require_text(plugin, "plugin")
        module, separator, attribute = plugin.partition(":")
        if not separator or not module or not attribute:
            raise PolicyError("plugin must have the form package.module:factory")
        factory = getattr(import_module(module), attribute, None)
        if not callable(factory):
            raise PolicyError(f"plugin factory {plugin!r} is not callable")
        return factory
    if name not in registry:
        raise PolicyError(f"unknown adapter {name!r}; available: {', '.join(sorted(registry))}")
    entry = registry[name]
    if isinstance(entry, str):
        module, attribute = entry.split(":")
        return getattr(import_module(module), attribute)
    return entry


def _options(config: dict[str, Any]) -> dict[str, Any]:
    options = config.get("options", {})
    if not isinstance(options, dict):
        raise PolicyError("adapter options must be a mapping")
    return dict(options)
