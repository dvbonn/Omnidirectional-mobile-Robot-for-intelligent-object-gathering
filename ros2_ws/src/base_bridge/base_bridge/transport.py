"""
transport.py - Abstract transport layer for base_bridge (PURE PYTHON, no ROS).
==============================================================================
Separating transport from ROS logic lets us: (1) wire up TCP right away, (2) switch
to CAN later WITHOUT touching node.py, (3) test with a fake transport.

Open Q2: does the base connect over TCP :2004 or a CAN bus?
  -> TcpTransport: FULLY implemented (the firmware is currently a TCP server :2004).
  -> CanTransport: stub - classic CAN payload is max 8B, the 13B velocity struct does NOT
    fit in one frame -> needs multi-frame / CAN-FD / a different encoding (do this once Q2 is settled).

Common interface (node.py depends only on this, not on the concrete TCP/CAN):
  connect()                  - open the connection (blocking, with timeout)
  send(data: bytes)          - send a raw payload (already encoded in protocol.py)
  recv(max_bytes) -> bytes   - read raw bytes (non-fatal when empty -> returns b"")
  close()
"""
import socket
from abc import ABC, abstractmethod


class Transport(ABC):
    """Transport interface to the base. node.py only calls through this."""

    @abstractmethod
    def connect(self) -> None: ...

    @abstractmethod
    def send(self, data: bytes) -> None: ...

    @abstractmethod
    def recv(self, max_bytes: int = 4096) -> bytes:
        """Return received bytes; b"" if none yet (non-blocking/timeout). Raises if the peer closes."""

    @abstractmethod
    def close(self) -> None: ...

    @property
    @abstractmethod
    def connected(self) -> bool: ...


class TcpTransport(Transport):
    """
    TCP client to the ESP32 (server :2004). Mirrors the Multi_platform_app client.
    recv() uses a short timeout -> returns b"" when there is no data (so the node spin does not stall).
    """

    def __init__(self, host: str, port: int = 2004, connect_timeout: float = 5.0,
                 recv_timeout: float = 0.05):
        self.host = host
        self.port = port
        self.connect_timeout = connect_timeout
        self.recv_timeout = recv_timeout
        self._sock = None

    @property
    def connected(self) -> bool:
        return self._sock is not None

    def connect(self) -> None:
        s = socket.create_connection((self.host, self.port), timeout=self.connect_timeout)
        s.settimeout(self.recv_timeout)
        # TCP_NODELAY: small velocity commands (13B) must be sent immediately, no Nagle batching.
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._sock = s

    def send(self, data: bytes) -> None:
        if self._sock is None:
            raise ConnectionError("TcpTransport not connect()ed")
        self._sock.sendall(data)

    def recv(self, max_bytes: int = 4096) -> bytes:
        if self._sock is None:
            raise ConnectionError("TcpTransport not connect()ed")
        try:
            data = self._sock.recv(max_bytes)
        except socket.timeout:
            return b""
        if data == b"":
            # empty recv (not a timeout) = peer closed the connection.
            raise ConnectionError("ESP32 closed the TCP connection")
        return data

    def close(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            finally:
                self._sock = None


class CanTransport(Transport):
    """
    STUB - CAN bus (Jetson AGX mttcan + SN65HVD230 transceiver).
    NOT implemented yet: needs Open Q2 settled (is the transceiver ready + the CAN ID/payload
    format on the ESP32 side). Note: classic CAN 8B < the 13B velocity struct -> needs multi-frame/CAN-FD.
    When implemented: use python-can, encode/decode CAN frames, keep the Transport interface.
    """

    def __init__(self, channel: str = "can0", bitrate: int = 500000):
        self.channel = channel
        self.bitrate = bitrate

    @property
    def connected(self) -> bool:
        return False

    def connect(self) -> None:
        raise NotImplementedError(
            "CanTransport not implemented yet - see Open Q2. "
            "Use TcpTransport (transport='tcp') until the CAN format is settled."
        )

    def send(self, data: bytes) -> None:
        raise NotImplementedError("CanTransport.send not implemented yet")

    def recv(self, max_bytes: int = 4096) -> bytes:
        raise NotImplementedError("CanTransport.recv not implemented yet")

    def close(self) -> None:
        pass


def make_transport(kind: str, **kwargs) -> Transport:
    """Factory: pick the transport from the ROS parameter (transport='tcp'|'can')."""
    kind = (kind or "tcp").lower()
    if kind == "tcp":
        return TcpTransport(
            host=kwargs.get("host", "mecanumbase.local"),
            port=int(kwargs.get("port", 2004)),
        )
    if kind == "can":
        return CanTransport(
            channel=kwargs.get("can_channel", "can0"),
            bitrate=int(kwargs.get("can_bitrate", 500000)),
        )
    raise ValueError("invalid transport: %r (choose 'tcp' or 'can')" % kind)
