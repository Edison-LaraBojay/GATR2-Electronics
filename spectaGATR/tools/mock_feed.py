#!/usr/bin/env python3
"""spectaGATR mock receiver: SYNTHETIC poses for the browser viewer.

Serves spectaGATR/web/ at /, spectaGATR/fields/ at /fields/<name>.json,
/api/fields, and a WebSocket at /ws that replays deterministic trajectories
in the receiver-to-browser JSON of spectaGATR/docs/PROTOCOL.md. No Brain, no
VEX device, no serial port. Python stdlib only.

    python spectaGATR/tools/mock_feed.py [--port 8790] [--scenario all]

Scenarios (scenario time t in seconds; every browser connection replays
from --start-s, so a page load sees the same sequence every time):

  figure8   heading-only figure eight, 20 s period
  attitude  fixed spot: roll +20 deg (0-4 s), pitch +20 deg (4-8 s),
            yaw/roll/pitch sweep (8-20 s), heading-only yaw 90 deg (20-24 s)
  restart   pose reset (new epoch) at 6 s, publisher restart (new session)
            at 12 s; 18 s period. The trail must show three pieces
  dropout   figure eight, no poses 6-10 s (stale), then resumes; 14 s period
  searching link "searching", no poses
  all       figure8, attitude, restart, dropout in turn (76 s), repeating

Values are quantized like the wire (1 mm, 0.01 deg) before conversion to SI.
"""

import argparse
import base64
import hashlib
import json
import math
import re
import select
import struct
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
WS_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
SESSIONS = ["5A3C19E2", "7B04D1C9", "1E5F00A7", "C0FFEE42"]
CENTER = 1.7832
ROBOT = {"front_m": 0.1685, "back_m": 0.1685, "left_m": 0.1465, "right_m": 0.1465,
         "height_m": 0.25}
CONTENT_TYPES = {
    ".html": "text/html; charset=utf-8",
    ".js": "text/javascript; charset=utf-8",
    ".css": "text/css; charset=utf-8",
    ".json": "application/json",
    ".md": "text/plain; charset=utf-8",
    ".png": "image/png",
    ".svg": "image/svg+xml",
    "": "text/plain; charset=utf-8",
}
DEG = math.pi / 180.0

STOP = threading.Event()


# --- scenarios ---
#
# Each returns (link, pose, session_index, epoch, phase). pose is None when
# nothing is published; otherwise (x_m, y_m, yaw_rad, roll_rad, pitch_rad,
# attitude). phase is a short text for the status detail.

def pose(x, y, yaw, roll=0.0, pitch=0.0, attitude=False):
    return (x, y, yaw, roll, pitch, attitude)


def figure8_pose(t):
    w = 2.0 * math.pi / 20.0
    x = CENTER + 1.2 * math.sin(w * t)
    y = CENTER + 0.75 * math.sin(2.0 * w * t)
    yaw = math.atan2(1.5 * w * math.cos(2.0 * w * t), 1.2 * w * math.cos(w * t))
    return pose(x, y, yaw)


def sc_figure8(t):
    return "open", figure8_pose(t % 20.0), 0, 0, "figure8 (heading only)"


def sc_attitude(t):
    t %= 24.0
    x, y = CENTER, 1.0
    if t < 4.0:
        return "open", pose(x, y, 0.0, 20 * DEG, 0.0, True), 0, 0, "attitude: roll +20 deg"
    if t < 8.0:
        return "open", pose(x, y, 0.0, 0.0, 20 * DEG, True), 0, 0, "attitude: pitch +20 deg"
    if t < 20.0:
        u = t - 8.0
        yaw = 2.0 * math.pi * u / 12.0
        roll = 15 * DEG * math.sin(2.0 * math.pi * u / 4.0)
        pitch = 10 * DEG * math.sin(2.0 * math.pi * u / 6.0)
        return "open", pose(x, y, yaw, roll, pitch, True), 0, 0, "attitude: sweep"
    return "open", pose(x, y, 90 * DEG), 0, 0, "attitude: heading only, yaw 90 deg"


def sc_restart(t):
    t %= 18.0
    if t < 6.0:
        x = 0.6 + 0.25 * t
        y = 0.9 + 0.3 * math.sin(t)
        return "open", pose(x, y, math.atan2(0.3 * math.cos(t), 0.25)), 0, 0, "restart: session A epoch 0"
    if t < 12.0:
        u = t - 6.0
        x = 3.0 - 0.25 * u
        y = 2.75 + 0.2 * math.sin(u)
        return "open", pose(x, y, math.atan2(0.2 * math.cos(u), -0.25)), 0, 1, "restart: pose reset, epoch 1"
    u = t - 12.0
    a = -math.pi / 2 + 2.0 * math.pi * u / 8.0
    x = CENTER + 0.95 * math.cos(a)
    y = CENTER + 0.95 * math.sin(a)
    return "open", pose(x, y, a + math.pi / 2), 1, 0, "restart: new session B"


def sc_dropout(t):
    t %= 14.0
    if 6.0 <= t < 10.0:
        return "open", None, 0, 0, "dropout: no poses"
    return "open", figure8_pose(t), 0, 0, "dropout: figure8"


def sc_searching(t):
    return "searching", None, 0, 0, "mock: no VEX device (synthetic searching state)"


SEQUENCE = [(sc_figure8, 20.0), (sc_attitude, 24.0), (sc_restart, 18.0), (sc_dropout, 14.0)]
CYCLE_S = sum(d for _, d in SEQUENCE)


def sc_all(t):
    cycle = int(t // CYCLE_S)
    u = t - cycle * CYCLE_S
    for i, (fn, d) in enumerate(SEQUENCE):
        if u < d or i == len(SEQUENCE) - 1:
            link, p, s, e, phase = fn(u)
            # each part is its own pose epoch, so the trail breaks between parts
            return link, p, s, (e + 2 * i + 8 * cycle) % 65536, phase
        u -= d
    raise AssertionError("unreachable")


SCENARIOS = {
    "all": sc_all, "figure8": sc_figure8, "attitude": sc_attitude, "restart": sc_restart,
    "dropout": sc_dropout, "searching": sc_searching,
}


def wire_cdeg(rad):
    c = int(round(rad / DEG * 100.0))
    c = ((c + 18000) % 36000) - 18000
    return 18000 if c == -18000 else c


# --- WebSocket ---

def ws_frame(text):
    data = text.encode("utf-8")
    n = len(data)
    if n < 126:
        head = struct.pack("!BB", 0x81, n)
    elif n < 65536:
        head = struct.pack("!BBH", 0x81, 126, n)
    else:
        head = struct.pack("!BBQ", 0x81, 127, n)
    return head + data


class Stream:
    """One browser connection: hello, then poses and status per PROTOCOL.md."""

    def __init__(self, sock, args, field_names):
        self.sock = sock
        self.args = args
        self.field_names = field_names
        self.scenario = SCENARIOS[args.scenario]
        self.t0 = time.monotonic()
        self.key = None            # (session index, epoch) of the last pose
        self.seq = 0
        self.last_pose_rx = None
        self.poses = 0
        self.sessions = 0
        self.last_status = None

    def send(self, obj):
        self.sock.sendall(ws_frame(json.dumps(obj, separators=(",", ":"))))

    def scenario_time(self, now):
        return self.args.start_s + (now - self.t0) * self.args.speed

    def run(self):
        a = self.args
        self.send({
            "type": "hello", "protocol": 1, "receiver": "mock_feed 1.0 (synthetic)",
            "stale_ms": a.stale_ms, "field": a.field, "fields": self.field_names,
            "robot": ROBOT,
        })
        period = 1.0 / a.rate
        next_pose = time.monotonic()
        next_status = next_pose
        while not STOP.is_set():
            now = time.monotonic()
            link, p, s_idx, epoch, phase = self.scenario(self.scenario_time(now))
            if now >= next_pose:
                if link == "open" and p is not None:
                    self.send_pose(now, p, s_idx, epoch)
                next_pose += period
                if next_pose < now - 1.0:
                    next_pose = now + period
            age = None if self.last_pose_rx is None else (now - self.last_pose_rx) * 1000.0
            stale = link != "open" or age is None or age > a.stale_ms
            sig = (link, stale, phase)
            if now >= next_status or sig != self.last_status:
                self.last_status = sig
                next_status = now + 0.25
                self.send({
                    "type": "status", "rx_ms": round(now * 1000.0, 1), "source": "sim",
                    "port": "mock", "link": link, "detail": "mock feed (synthetic): " + phase,
                    "stale": stale, "pose_age_ms": None if age is None else int(age),
                    "counts": {"poses": self.poses, "bad_crc": 0, "malformed": 0,
                               "out_of_range": 0, "repeated": 0, "out_of_order": 0,
                               "truncated": 0, "too_long": 0, "garbage": 0,
                               "sessions": self.sessions, "bytes": self.poses * 64},
                })
            if self.peer_closed():
                return
            wait = min(next_pose, next_status) - time.monotonic()
            time.sleep(min(0.05, max(0.001, wait)))

    def send_pose(self, now, p, s_idx, epoch):
        x, y, yaw, roll, pitch, attitude = p
        key = (s_idx, epoch)
        restart = key != self.key
        if self.key is None or s_idx != self.key[0]:
            self.seq = 0
            self.sessions += 1
        self.key = key
        self.seq = (self.seq + 1) % (1 << 32)
        t_ms = (100000 + int(self.scenario_time(now) * 1000.0)) % (1 << 32)
        if not attitude:
            roll = pitch = 0.0
        self.last_pose_rx = now
        self.poses += 1
        self.send({
            "type": "pose", "session": SESSIONS[s_idx % len(SESSIONS)], "epoch": epoch,
            "seq": self.seq, "t_ms": t_ms,
            "x": int(round(x * 1000.0)) / 1000.0, "y": int(round(y * 1000.0)) / 1000.0,
            "yaw": wire_cdeg(yaw) / 100.0 * DEG,
            "roll": wire_cdeg(roll) / 100.0 * DEG, "pitch": wire_cdeg(pitch) / 100.0 * DEG,
            "attitude": bool(attitude), "restart": restart, "rx_ms": round(now * 1000.0, 1),
            "source": "sim",
        })

    def peer_closed(self):
        r, _, _ = select.select([self.sock], [], [], 0)
        if not r:
            return False
        try:
            data = self.sock.recv(4096)
        except OSError:
            return True
        # empty read: TCP closed; opcode 8: WebSocket close
        return not data or (data[0] & 0x0F) == 0x8


# --- HTTP ---

class Handler(BaseHTTPRequestHandler):
    server_version = "spectaGATR-mock/1.0"
    args = None

    def log_message(self, fmt, *a):
        if self.args.verbose:
            sys.stderr.write("%s %s\n" % (self.address_string(), fmt % a))

    def field_names(self):
        names = sorted(p.stem for p in Path(self.args.fields_dir).glob("*.json"))
        if self.args.field in names:
            names.remove(self.args.field)
            names.insert(0, self.args.field)
        return names

    def reply(self, code, body, ctype):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        if path == "/ws":
            return self.websocket()
        if path == "/api/fields":
            body = json.dumps({"fields": self.field_names(), "default": self.args.field})
            return self.reply(200, body.encode(), "application/json")
        m = re.fullmatch(r"/fields/([A-Za-z0-9_-]+)\.json", path)
        if m:
            return self.file(Path(self.args.fields_dir) / (m.group(1) + ".json"))
        if path == "/":
            path = "/index.html"
        web = Path(self.args.web_dir).resolve()
        target = (web / path.lstrip("/")).resolve()
        if web not in target.parents:
            return self.reply(404, b"not found\n", "text/plain")
        return self.file(target)

    def file(self, p):
        if not p.is_file():
            return self.reply(404, b"not found\n", "text/plain")
        ctype = CONTENT_TYPES.get(p.suffix, "application/octet-stream")
        return self.reply(200, p.read_bytes(), ctype)

    def websocket(self):
        key = self.headers.get("Sec-WebSocket-Key")
        if self.headers.get("Upgrade", "").lower() != "websocket" or not key:
            return self.reply(400, b"websocket upgrade expected\n", "text/plain")
        accept = base64.b64encode(hashlib.sha1((key + WS_GUID).encode()).digest()).decode()
        self.send_response(101, "Switching Protocols")
        self.send_header("Upgrade", "websocket")
        self.send_header("Connection", "Upgrade")
        self.send_header("Sec-WebSocket-Accept", accept)
        self.end_headers()
        self.close_connection = True
        try:
            Stream(self.connection, self.args, self.field_names()).run()
        except OSError:
            pass   # browser went away


def main(argv):
    ap = argparse.ArgumentParser(description="spectaGATR mock receiver (synthetic poses)")
    ap.add_argument("--bind", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8790)
    ap.add_argument("--scenario", choices=sorted(SCENARIOS), default="all")
    ap.add_argument("--rate", type=float, default=10.0, help="poses per second, 1..50")
    ap.add_argument("--speed", type=float, default=1.0, help="scenario time per real second")
    ap.add_argument("--start-s", type=float, default=0.0, help="scenario time at connect")
    ap.add_argument("--stale-ms", type=int, default=750)
    ap.add_argument("--field", default="override", help="default field name")
    ap.add_argument("--exit-after-s", type=float, default=0.0,
                    help="stop serving after this many seconds (tests the disconnected state)")
    ap.add_argument("--web-dir", default=str(ROOT / "web"))
    ap.add_argument("--fields-dir", default=str(ROOT / "fields"))
    ap.add_argument("--verbose", action="store_true")
    args = ap.parse_args(argv[1:])
    if not 1.0 <= args.rate <= 50.0:
        ap.error("--rate must be within 1..50")
    if args.speed <= 0.0:
        ap.error("--speed must be positive")

    Handler.args = args
    httpd = ThreadingHTTPServer((args.bind, args.port), Handler)
    httpd.daemon_threads = True
    print("spectaGATR mock feed (SYNTHETIC poses): http://%s:%d/  scenario %s, %.0f Hz"
          % (args.bind, args.port, args.scenario, args.rate), flush=True)
    if args.exit_after_s > 0.0:
        def stop():
            STOP.set()
            httpd.shutdown()
        threading.Timer(args.exit_after_s, stop).start()
    try:
        httpd.serve_forever(poll_interval=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        STOP.set()
        httpd.server_close()
    print("mock feed stopped", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
