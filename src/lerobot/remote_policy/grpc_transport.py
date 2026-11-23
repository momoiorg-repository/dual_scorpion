"""Authenticated gRPC transport for the Universal Policy protocol."""

from __future__ import annotations

import hmac
import ipaddress
import logging
import time
from concurrent.futures import ThreadPoolExecutor
from typing import TYPE_CHECKING
from urllib.parse import urlsplit

from .core import InferenceBackend
from .exchange import (
    MAX_PAYLOAD_BYTES,
    decode_json,
    decode_output,
    decode_request,
    encode_json,
    encode_output,
    encode_request,
)
from .schema import InferenceRequest, PolicyCapabilities, PolicyError, PolicyOutput

if TYPE_CHECKING:
    import grpc

SERVICE = "lerobot.remote_policy.v1.Policy"
logger = logging.getLogger(__name__)


def _grpc():
    try:
        import grpc
    except ImportError as exc:
        raise PolicyError("gRPC transport requires the lerobot[grpcio-dep] extra") from exc
    return grpc


def _options() -> list[tuple[str, int]]:
    return [
        ("grpc.max_receive_message_length", MAX_PAYLOAD_BYTES),
        ("grpc.max_send_message_length", MAX_PAYLOAD_BYTES),
        ("grpc.enable_retries", 0),
        ("grpc.enable_http_proxy", 0),
    ]


def _address(endpoint: str, *, server: bool = False) -> tuple[str, bool]:
    try:
        address = urlsplit(endpoint)
        host, port = address.hostname, address.port
    except ValueError as exc:
        raise PolicyError("invalid gRPC endpoint") from exc
    if (
        address.scheme not in {"grpc", "grpcs"}
        or host is None
        or port is None
        or (port == 0 and not server)
        or address.username is not None
        or address.password is not None
        or address.path
        or address.query
        or address.fragment
    ):
        raise PolicyError("gRPC endpoint must have the form grpc[s]://host:port")
    secure = address.scheme == "grpcs"
    if not secure:
        if host == "localhost":
            host = "127.0.0.1"
        try:
            ip = ipaddress.ip_address(host)
        except ValueError as exc:
            raise PolicyError("gRPC endpoints outside loopback require TLS") from exc
        if not ip.is_loopback:
            raise PolicyError("gRPC endpoints outside loopback require TLS")
    return f"[{host}]:{port}" if ":" in host else f"{host}:{port}", secure


def _bearer(token: str) -> str:
    if (
        not isinstance(token, str)
        or not token
        or len(token) > 1024
        or any(not 33 <= ord(c) <= 126 for c in token)
    ):
        raise PolicyError("a non-empty ASCII bearer token is required")
    return f"Bearer {token}"


class GrpcTransport:
    def __init__(self, endpoint: str, token: str, *, root_certificates: bytes | None = None):
        address, secure = _address(endpoint)
        self._metadata = (("authorization", _bearer(token)),)
        self._grpc = _grpc()
        if secure:
            credentials = self._grpc.ssl_channel_credentials(root_certificates=root_certificates)
            self._channel = self._grpc.secure_channel(address, credentials, options=_options())
        else:
            if root_certificates is not None:
                raise PolicyError("a CA certificate requires a grpcs endpoint")
            self._channel = self._grpc.insecure_channel(address, options=_options())
        self._capabilities = self._channel.unary_unary(f"/{SERVICE}/GetCapabilities")
        self._infer = self._channel.unary_unary(f"/{SERVICE}/Infer")

    def get_capabilities(self, *, timeout: float) -> PolicyCapabilities:
        return PolicyCapabilities.from_dict(decode_json(self._call(self._capabilities, b"", timeout)))

    def infer(self, request: InferenceRequest, *, timeout: float) -> PolicyOutput:
        deadline = time.monotonic() + timeout
        payload = encode_request(request)
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError("gRPC policy deadline expired during request encoding")
        return decode_output(self._call(self._infer, payload, remaining))

    def close(self) -> None:
        self._channel.close()

    def _call(self, rpc, data: bytes, timeout: float) -> bytes:
        try:
            return rpc(data, timeout=timeout, metadata=self._metadata)
        except self._grpc.RpcError as exc:
            code = exc.code()
            if code == self._grpc.StatusCode.DEADLINE_EXCEEDED:
                raise TimeoutError("gRPC policy deadline exceeded") from exc
            if code == self._grpc.StatusCode.UNAUTHENTICATED:
                raise PolicyError("gRPC policy authentication failed") from exc
            details = exc.details() if code == self._grpc.StatusCode.INVALID_ARGUMENT else ""
            raise PolicyError(
                f"gRPC policy request failed ({code.name})" + (f": {details}" if details else "")
            ) from exc


def create_server(
    backend: InferenceBackend,
    endpoint: str,
    token: str,
    *,
    tls_certificate: bytes | None = None,
    tls_key: bytes | None = None,
) -> tuple[grpc.Server, int]:
    """Bind an authenticated server. The caller owns starting and stopping it."""
    address, secure = _address(endpoint, server=True)
    expected = _bearer(token).encode("ascii")
    grpc = _grpc()
    if secure and not (tls_certificate and tls_key):
        raise PolicyError("a grpcs listen endpoint requires both a TLS certificate and key")
    if not secure and (tls_certificate is not None or tls_key is not None):
        raise PolicyError("TLS server credentials require a grpcs listen endpoint")

    def authenticate(context: grpc.ServicerContext) -> None:
        headers = [value for key, value in context.invocation_metadata() if key == "authorization"]
        if len(headers) != 1 or not hmac.compare_digest(headers[0].encode("ascii"), expected):
            context.abort(grpc.StatusCode.UNAUTHENTICATED, "invalid policy credentials")

    def capabilities(payload: bytes, context: grpc.ServicerContext) -> bytes:
        authenticate(context)
        if payload:
            context.abort(grpc.StatusCode.INVALID_ARGUMENT, "GetCapabilities expects an empty request")
        return encode_json(backend.get_capabilities().to_dict())

    def infer(payload: bytes, context: grpc.ServicerContext) -> bytes:
        authenticate(context)
        try:
            request = decode_request(payload)
            output = backend.infer(request, is_active=context.is_active)
            return encode_output(output)
        except PolicyError as exc:
            context.abort(grpc.StatusCode.INVALID_ARGUMENT, str(exc))
        except TimeoutError:
            context.abort(grpc.StatusCode.DEADLINE_EXCEEDED, "inference deadline expired")
        except Exception:
            logger.exception("policy adapter failed")
            context.abort(grpc.StatusCode.INTERNAL, "policy adapter failed")

    server = grpc.server(ThreadPoolExecutor(max_workers=2), options=_options(), maximum_concurrent_rpcs=2)
    server.add_generic_rpc_handlers(
        (
            grpc.method_handlers_generic_handler(
                SERVICE,
                {
                    "GetCapabilities": grpc.unary_unary_rpc_method_handler(capabilities),
                    "Infer": grpc.unary_unary_rpc_method_handler(infer),
                },
            ),
        )
    )
    if secure:
        credentials = grpc.ssl_server_credentials(((tls_key, tls_certificate),))
        port = server.add_secure_port(address, credentials)
    else:
        port = server.add_insecure_port(address)
    if not port:
        raise PolicyError("could not bind the gRPC policy endpoint")
    return server, port
