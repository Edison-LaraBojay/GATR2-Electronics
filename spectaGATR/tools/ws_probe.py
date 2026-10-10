#!/usr/bin/env python3
"""ws_probe.py: check a running spectagatr receiver's browser feed. Python stdlib only.

    python spectaGATR/tools/ws_probe.py [--url ws://127.0.0.1:8790/ws] [--seconds 3]

Connects like a browser, reads frames for a while and checks the framing of PROTOCOL.md:
every frame is one unmasked, unfragmented text frame holding one JSON object; hello comes
first and once; then pose and status. Prints a summary and one example of each message.
Exit status 0 when every check passed.
"""

import argparse
import base64
import hashlib
import json
import os
import socket
import struct
import sys
import time
from urllib.parse import urlparse

GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"


def handshake(sock, host, port, path):
    key = base64.b64encode(os.urandom(16)).decode()
    request = (
        f"GET {path} HTTP/1.1\r\nHost: {host}:{port}\r\nUpgrade: websocket\r\n"
        f"Connection: Upgrade\r\nSec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n"
    )
    sock.sendall(request.encode())
    head = b""
    while b"\r\n\r\n" not in head:
        chunk = sock.recv(4096)
        if not chunk:
            raise RuntimeError("closed during the handshake")
        head += chunk
    head, rest = head.split(b"\r\n\r\n", 1)
    lines = head.decode("latin-1").split("\r\n")
    if not lines[0].startswith("HTTP/1.1 101"):
        raise RuntimeError("no upgrade: " + lines[0])
    expected = base64.b64encode(hashlib.sha1((key + GUID).encode()).digest()).decode()
    accept = [l.split(":", 1)[1].strip() for l in lines[1:] if l.lower().startswith("sec-websocket-accept:")]
    if accept != [expected]:
        raise RuntimeError("bad Sec-WebSocket-Accept")
    return rest


def read_frames(sock, buffer, seconds):
    """Yields (fin, opcode, masked, payload) until the time is up or the server closes."""
    deadline = time.monotonic() + seconds
    while True:
        while True:
            if len(buffer) >= 2:
                b0, b1 = buffer[0], buffer[1]
                length = b1 & 0x7F
                pos = 2
                if length == 126 and len(buffer) >= 4:
                    length = struct.unpack(">H", buffer[2:4])[0]
                    pos = 4
                elif length == 127 and len(buffer) >= 10:
                    length = struct.unpack(">Q", buffer[2:10])[0]
                    pos = 10
                elif length >= 126:
                    break
                masked = bool(b1 & 0x80)
                if masked:
                    pos += 4
                if len(buffer) >= pos + length:
                    payload = buffer[pos:pos + length]
                    buffer = buffer[pos + length:]
                    yield bool(b0 & 0x80), b0 & 0x0F, masked, payload
                    continue
            break
        left = deadline - time.monotonic()
        if left <= 0:
            return
        sock.settimeout(left)
        try:
            chunk = sock.recv(65536)
        except socket.timeout:
            return
        if not chunk:
            return
        buffer += chunk


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--url", default="ws://127.0.0.1:8790/ws")
    parser.add_argument("--seconds", type=float, default=3.0)
    args = parser.parse_args()
    url = urlparse(args.url)
    host, port, path = url.hostname or "127.0.0.1", url.port or 80, url.path or "/ws"

    sock = socket.create_connection((host, port), timeout=5)
    buffer = handshake(sock, host, port, path)
    problems = []
    counts = {}
    examples = {}
    order = []
    restarts = 0
    last_seq = None
    status_times = []
    stale_flips = []
    for fin, opcode, masked, payload in read_frames(sock, buffer, args.seconds):
        if opcode == 0x8:
            problems.append("server closed the feed")
            break
        if opcode in (0x9, 0xA):
            continue
        if not fin or opcode != 0x1 or masked:
            problems.append(f"bad frame: fin={fin} opcode={opcode} masked={masked}")
            continue
        try:
            message = json.loads(payload.decode("utf-8"))
        except (UnicodeDecodeError, ValueError) as error:
            problems.append(f"not one JSON object: {error}")
            continue
        kind = message.get("type")
        counts[kind] = counts.get(kind, 0) + 1
        examples.setdefault(kind, message)
        order.append(kind)
        if kind == "status":
            status_times.append(time.monotonic())
            stale = message.get("stale")
            if not stale_flips or stale_flips[-1] != stale:
                stale_flips.append(stale)
        if kind == "pose":
            restarts += 1 if message.get("restart") else 0
            if last_seq is not None and message["seq"] == last_seq and not message["restart"]:
                problems.append("the same pose twice")
            last_seq = message["seq"]
            for key in ("x", "y", "yaw", "roll", "pitch", "rx_ms"):
                if not isinstance(message.get(key), (int, float)):
                    problems.append(f"pose {key} is not a number")
    sock.close()

    if not order or order[0] != "hello":
        problems.append("hello was not first")
    if counts.get("hello", 0) != 1:
        problems.append(f"{counts.get('hello', 0)} hello messages")
    for kind in ("pose", "status"):
        if counts.get(kind, 0) == 0:
            problems.append(f"no {kind} message")

    print(f"messages in {args.seconds:g} s: " + ", ".join(f"{k} {v}" for k, v in counts.items()))
    print(f"pose restarts seen: {restarts}")
    print("stale over time: " + " -> ".join("stale" if s else "live" for s in stale_flips))
    if len(status_times) > 1:
        gap = max(b - a for a, b in zip(status_times, status_times[1:])) * 1000.0
        print(f"longest gap between status messages: {gap:.0f} ms")
        if gap > 300.0:
            problems.append(f"status gap {gap:.0f} ms (want at most 250 plus delivery)")
    for kind in ("hello", "pose", "status"):
        if kind in examples:
            print(f"{kind}: {json.dumps(examples[kind])}")
    for problem in problems:
        print("PROBLEM: " + problem)
    print("ok" if not problems else "FAILED")
    return 0 if not problems else 1


if __name__ == "__main__":
    sys.exit(main())
