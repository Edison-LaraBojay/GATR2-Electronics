"""stub_server.py - a stand-in inspect/2 server for the viewer tests.

Serves pi/naviGATR/spectaGATR (and the pinned three.js) and speaks
navigatr.inspect/2 over /ws the way docs/inspection.md describes: hello,
history, replaceable state/diag/telemetry/instrumentation (latest wins per
client, one writer thread per client), reliable event/capture/pong, and the
capture HTTP API with a small recorder that bundles what it actually sent
(capture_fixture.write_bundle). A synthetic robot drives a circle.

It is a test double for the browser: it is not the naviGATR server, and a
test passing against it says nothing about the Pi runtime. Test-only
controls under /stub/: kill (drop all sockets), reset (session reset),
restart (new session id, drop sockets), epoch (new odometry epoch), jump
(teleport the robot 1 m), pause?ms= (stop publishing states; keepalive states
carry a growing age, as the runtime's do), unmeasured?on=1 (valid poses with
no measurement time: a configured placement; &source=1: measured on the Pico
clock but not mapped to the Pi clock, age unknown), ready?on=0 (a localization
function not ready), clock?mapped=0 (diag clock_mapped false),
capfail?mode=file|bundle|none (the next capture's file write fails, or its
bundle fails and it leaves only status.last), stats (what each client sent
us, including its ping ids).

    python stub_server.py [--port 8799] [--state-hz 30]
"""

import argparse
import base64
import collections
import hashlib
import json
import math
import os
import socket
import struct
import threading
import time
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import sys

sys.dont_write_bytecode = True   # no __pycache__ in the source tree
import capture_fixture as cf  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
NAV = os.path.abspath(os.path.join(HERE, '..', '..'))
VIEWER = os.path.join(NAV, 'spectaGATR')
THREE = os.path.join(NAV, 'third_party', 'three')
GUID = '258EAFA5-E914-47DA-95CA-C5AB0DC85B11'
REPLACEABLE = ('state', 'telemetry', 'diag', 'instrumentation')
T_START = time.monotonic()


def now_ms():
    return (time.monotonic() - T_START) * 1000.0


def wrap180(d):
    x = ((d + 180.0) % 360.0) - 180.0
    return 180.0 if x == -180.0 else x


class Client:
    def __init__(self, server, sock, cid):
        self.server = server
        self.sock = sock
        self.id = cid
        self.cv = threading.Condition()
        self.reliable = collections.deque()
        self.reliable_bytes = 0
        self.slots = {}
        self.sent = collections.Counter()
        self.replaced = collections.Counter()
        self.received = collections.Counter()
        self.sub = {'state_hz': 30, 'diag': True, 'telemetry': True, 'instrumentation': False,
                    'raw': False, 'decoded': False}
        self.alive = True
        self.last_state_sent = 0.0
        self.ping_ids = collections.deque(maxlen=200)

    def enqueue(self, kind, text):
        with self.cv:
            if kind in REPLACEABLE:
                if kind in self.slots:
                    self.replaced[kind] += 1
                self.slots[kind] = text
            else:
                self.reliable.append(text)
                self.reliable_bytes += len(text)
                if self.reliable_bytes > 256 * 1024:
                    self.alive = False   # reliable overflow closes the client
            self.cv.notify()

    def writer(self):
        try:
            while True:
                with self.cv:
                    while self.alive and not self.reliable and not self.slots:
                        self.cv.wait(0.5)
                    if not self.alive:
                        break
                    if self.reliable:
                        text = self.reliable.popleft()
                        self.reliable_bytes -= len(text)
                        kind = 'reliable'
                    else:
                        kind = next(k for k in REPLACEABLE if k in self.slots)
                        text = self.slots.pop(kind)
                self.send_frame(text.encode())
                self.sent[kind] += 1
        except OSError:
            pass
        self.close()

    def send_frame(self, data, opcode=1):
        n = len(data)
        if n < 126:
            head = struct.pack('!BB', 0x80 | opcode, n)
        elif n < 65536:
            head = struct.pack('!BBH', 0x80 | opcode, 126, n)
        else:
            head = struct.pack('!BBQ', 0x80 | opcode, 127, n)
        self.sock.sendall(head + data)

    def read_exact(self, n):
        buf = b''
        while len(buf) < n:
            chunk = self.sock.recv(n - len(buf))
            if not chunk:
                raise ConnectionError('closed')
            buf += chunk
        return buf

    def reader(self):
        try:
            while self.alive:
                b0, b1 = self.read_exact(2)
                op = b0 & 0x0F
                n = b1 & 0x7F
                if n == 126:
                    n = struct.unpack('!H', self.read_exact(2))[0]
                elif n == 127:
                    n = struct.unpack('!Q', self.read_exact(8))[0]
                mask = self.read_exact(4)
                data = bytes(b ^ mask[i & 3] for i, b in enumerate(self.read_exact(n)))
                if op == 8:
                    break
                if op != 1:
                    continue
                self.server.on_client_message(self, json.loads(data.decode()))
        except (OSError, ConnectionError, ValueError):
            pass
        self.close()

    def close(self):
        with self.cv:
            self.alive = False
            self.cv.notify()
        try:
            self.sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        try:
            self.sock.close()
        except OSError:
            pass
        self.server.drop(self)


class Recorder:
    """Rolling window of what the stub published, bundled on request."""

    def __init__(self, stub):
        self.stub = stub
        self.lock = threading.Lock()
        self.rows = collections.deque()
        self.events = collections.deque()
        self.telemetry = collections.deque()
        self.active = None
        self.captures = []
        self.version = 1
        self.counter = 0
        self.last = None
        self.fail = 'none'   # next capture: 'file' (write fails) or 'bundle' (build fails)

    def note_robot(self, row):
        with self.lock:
            self.rows.append(row)
            self.trim()

    def note_event(self, row):
        with self.lock:
            self.events.append(row)

    def note_telemetry(self, row):
        with self.lock:
            self.telemetry.append(row)

    def trim(self):
        keep_us = (now_ms() - 12_000) * 1000
        for d in (self.rows, self.events, self.telemetry):
            while d and d[0]['pi_host_us'] < keep_us and not self.active:
                d.popleft()

    def start(self, pre_s, post_s, streams):
        with self.lock:
            if self.active:
                return None, 'busy: a capture is recording'
            self.counter += 1
            cid = f'stub-{self.counter:03d}'
            t = now_ms()
            self.active = {'id': cid, 'state': 'recording', 'reason': 'manual', 'requester': 'http',
                           'pre_s_requested': pre_s, 'post_s_requested': post_s, 'pre_s': min(pre_s, 10.0),
                           'post_s': post_s, 'trigger_pi_host_us': int(t * 1000),
                           'end_pi_host_us': int((t + post_s * 1000) * 1000), 'streams': streams,
                           'elapsed_s': 0.0, 'remaining_s': post_s, 'records': 0}
            self.version += 1
            return cid, None

    def cancel(self, cid):
        with self.lock:
            if not self.active or self.active['id'] != cid:
                return False
            self.active = None
            self.last = {'id': cid, 'outcome': 'cancelled', 'error': ''}
            self.version += 1
            return True

    def tick(self):
        with self.lock:
            a = self.active
            if not a:
                return
            t_us = now_ms() * 1000
            elapsed = (t_us - a['trigger_pi_host_us']) / 1e6
            if int(elapsed) != int(a['elapsed_s']):
                self.version += 1   # progress at most once a second, as the runtime
            a['elapsed_s'] = elapsed
            a['remaining_s'] = max(0.0, a['post_s'] - a['elapsed_s'])
            if t_us < a['end_pi_host_us']:
                return
            lo = a['trigger_pi_host_us'] - a['pre_s'] * 1e6
            hi = a['end_pi_host_us']
            pick = lambda d: [r for r in d if lo <= r['pi_host_us'] <= hi]
            robot, events, tel = pick(self.rows), pick(self.events), pick(self.telemetry)
            events.append({'pi_host_us': a['trigger_pi_host_us'], 'pi_session': self.stub.session,
                           'reset_count': self.stub.reset_count, 'kind': 'capture', 'source': 'capture',
                           'text': 'trigger manual'})
            events.sort(key=lambda r: r['pi_host_us'])
            meta = {'schema': 'gatr2.capture/1', 'id': a['id'], 'created': {'pi_host_us': a['trigger_pi_host_us']},
                    'configuration': {'id': 'stub', 'name': 'viewer stub server (synthetic, not a robot)'},
                    'trigger': {k: a[k] for k in ('reason', 'requester', 'pre_s', 'post_s')},
                    'streams': {'robot_state': {'rows': len(robot)}, 'event': {'rows': len(events)},
                                'brain_telemetry': {'rows': len(tel)}},
                    'drops': {}, 'segments': [], 'markers': []}
            data = cf.write_bundle(meta, {'robot_state.csv': (cf.ROBOT_HEADER, robot),
                                          'events.csv': (cf.EVENTS_HEADER, events),
                                          'brain_telemetry.csv': (cf.TELEMETRY_HEADER, tel)})
            fail, self.fail = self.fail, 'none'
            self.active = None
            self.version += 1
            if fail == 'bundle':
                # as the runtime: no list entry, only the outcome
                self.last = {'id': a['id'], 'outcome': 'failed', 'error': 'bundle: stub failure (test)'}
                return
            ferr = 'stub: cannot open /nonexistent/capture.zip.part (test)' if fail == 'file' else None
            done = dict(a, state='ready', url=f"/api/capture/{a['id']}.zip", size_bytes=len(data),
                        rows={'robot_state': len(robot), 'event': len(events), 'brain_telemetry': len(tel)},
                        truncated=False, dropped={}, file=None, file_error=ferr)
            done['zip'] = data
            self.captures.insert(0, done)
            del self.captures[3:]
            self.last = {'id': a['id'], 'outcome': 'ready', 'error': ferr or ''}

    def status(self):
        with self.lock:
            return {'available': True, 'schema': 'gatr2.capture/1',
                    'state': 'recording' if self.active else 'idle', 'pi_host_us': int(now_ms() * 1000),
                    'active': dict(self.active) if self.active else None,
                    'captures': [{k: v for k, v in c.items() if k != 'zip'} for c in self.captures],
                    'last': dict(self.last) if self.last else None,
                    'limits': {'rolling_s': 10, 'max_pre_s': 10, 'max_post_s': 60, 'default_pre_s': 5,
                               'default_post_s': 10, 'max_records': 400000, 'max_mb': 64, 'keep': 3},
                    'auto': {'triggers': []}}

    def bundle(self, cid):
        with self.lock:
            for c in self.captures:
                if c['id'] == cid:
                    return c['zip']
        return None


class Stub:
    def __init__(self, args):
        self.args = args
        self.lock = threading.Lock()
        self.clients = []
        self.next_cid = 1
        self.session = os.urandom(8).hex()
        self.reset_count = 0
        self.epoch = 1
        self.anchor_rev = 1
        self.publication = 0
        self.jump = 0.0
        self.pause_until = 0.0
        self.seq = collections.Counter()
        self.events = collections.deque(maxlen=32)
        self.history = collections.deque(maxlen=600)
        self.t_epoch = now_ms()
        self.recorder = Recorder(self)
        self.robot = None
        self.telemetry_doc = None
        self.running = True
        self.unmeasured = False    # or 'placement' / 'source'
        self.not_ready = False
        self.clock_mapped = True

    # --- documents ---

    def hello(self, client=None):
        return {'type': 'hello', 'contract': 'navigatr.inspect/2', 'host_ms': int(now_ms()),
                'client_id': client.id if client else None,
                'session': {'id': self.session, 'reset_count': self.reset_count},
                'configuration': {'id': 'stub', 'name': 'viewer stub (synthetic, not a robot)', 'digest': 'stub0001',
                                  'loop_rate_hz': 100},
                'inspection': {'preview_hz': 5, 'preview_quality': 70, 'preview_max_width': 640},
                'features': {'state_hz': self.args.state_hz, 'diag_hz': 4, 'capture': True, 'instrumentation': True,
                             'telemetry': True, 'history_max': 600},
                'robot_body': {'length_m': 0.4, 'width_m': 0.36, 'height_m': 0.25, 'origin_x_m': 0, 'origin_y_m': 0},
                'fields': [{'map_id': 1, 'revision': 1, 'dimensions': {
                    'inside_x_m': 3.6, 'inside_y_m': 3.6, 'tile_m': 0.6, 'wall_thickness_m': 0.05, 'wall_height_m': 0.3,
                    'source': 'stub'}, 'landmarks': [], 'features': [],
                    'boundary': {'min_x_m': 0.1, 'min_y_m': 0.1, 'max_x_m': 3.5, 'max_y_m': 3.5}, 'obstacles': []}],
                'tag_families': {}, 'cameras': [], 'camera_sensors': [], 'localization': {}, 'warnings': []}

    def robot_doc(self, t):
        tt = (t - self.t_epoch) / 1000.0
        a = 2 * math.pi * tt / 8.0
        ox = 0.6 * math.sin(a) + self.jump
        oy = 0.6 * (1 - math.cos(a))
        oh = wrap180(math.degrees(a))
        anchor = (1.2, 1.2, 0.0)
        age = 8.0
        measured = None if self.unmeasured else int(t - age)
        # 'source': measured on the Pico clock, not mapped to the Pi clock
        source_stamp = {'clock': 'pico', 'ms': int(t) + 1000} if self.unmeasured == 'source' else None
        return {
            'valid': True, 'initialized': True,
            'placement_origin': 'configuration' if self.unmeasured == 'placement' else 'command',
            'placement_session': 1,
            'placement_sequence': 1, 'odometry_epoch': self.epoch, 'anchor_revision': self.anchor_rev,
            'odom': {'x_m': ox, 'y_m': oy, 'heading_deg': oh},
            'field': {'x_m': ox + anchor[0], 'y_m': oy + anchor[1], 'heading_deg': oh},
            'field_from_odom': {'x_m': anchor[0], 'y_m': anchor[1], 'heading_deg': anchor[2]},
            'vx_m_s': 0.6 * 2 * math.pi / 8.0, 'vy_m_s': 0.0, 'yaw_rate_deg_s': 45.0, 'confidence': 0.9,
            'has_covariance': False,
            'measured_at': source_stamp or {'clock': 'host' if measured is not None else 'none', 'ms': measured},
            'measured_at_host_ms': measured, 'age_ms': None if measured is None else int(t - measured),
            'attitude': {'valid': True, 'assumed_level': False, 'status': 'measured', 'source': 'stub',
                         'roll_deg': 3 * math.sin(tt), 'pitch_deg': 2 * math.cos(tt), 'age_ms': 10,
                         'measured_at_host_ms': int(t - 10)},
        }

    def state(self, t):
        self.seq['state'] += 1
        robot = self.robot
        if robot and robot.get('measured_at_host_ms') is not None:
            # the age at build time, as the runtime writes it: a keepalive
            # repeating an old pose says how old it is
            robot = dict(robot, age_ms=int(t - robot['measured_at_host_ms']))
        return {'type': 'state', 'seq': self.seq['state'], 'host_ms': int(t), 'host_us': int(t * 1000),
                'session': {'id': self.session, 'reset_count': self.reset_count}, 'publication': self.publication,
                'robot': robot, 'localization': {'all_ready': not self.not_ready, 'stationary': False,
                                                 'continuity_breaks': 0, 'last_break': ''}}

    def diag(self, t, client):
        self.seq['diag'] += 1
        with self.lock:
            # the runtime's per-client shape (docs/inspection.md)
            clients = [{'id': c.id, 'queued_bytes': 0, 'in_flight_bytes': 0, 'reliable_backlog': len(c.reliable),
                        'reliable_bytes': c.reliable_bytes, 'slots_pending': len(c.slots), 'diag_skipped': 0,
                        'channels': {k: {'replaceable': k != 'reliable', 'sent': c.sent.get(k, 0),
                                         'replaced': c.replaced.get(k, 0), 'refused': 0}
                                     for k in REPLACEABLE + ('reliable',)}} for c in self.clients]
        path_pts = [{'x_m': 1.2, 'y_m': 1.2}, {'x_m': 1.8, 'y_m': 1.6}, {'x_m': 2.4, 'y_m': 2.0}, {'x_m': 2.8, 'y_m': 2.6}]
        return {'type': 'diag', 'contract': 'navigatr.inspect/2', 'seq': self.seq['diag'], 'host_ms': int(t),
                'session': {'id': self.session, 'reset_count': self.reset_count}, 'cycle': int(t / 10), 'running': True,
                'robot': self.robot,
                'localization': {'estimator_type': 'stub', 'updates': self.publication, 'history_size': 600,
                                 'clock_mapped': self.clock_mapped, 'publication': self.publication,
                                 'all_ready': not self.not_ready,
                                 'stationary': False, 'continuity_breaks': 0, 'last_break': '',
                                 'functions': [{'id': 'tracking', 'type': 'stub', 'ready': not self.not_ready,
                                                'note': 'waiting for stillness' if self.not_ready else '',
                                                'stillness': {'monitored': True, 'stationary': False, 'calibration': 'done',
                                                              'progress_ms': 1000, 'window_ms': 1000, 'attempts': 1,
                                                              'bias_dps': 0.12 + 0.01 * math.sin(t / 3000)}}]},
                'field_snapshot': {'invocation': int(t / 100), 'status': 'ok', 'age_ms': 20, 'diagnostic': ''},
                'field_objects': [], 'detection_frames': [], 'target': None, 'command': None,
                'brain_link': {'link_open': True, 'session': 0x1234, 'pi_instance': 1, 'takes_profile': True,
                               'last_request_age_ms': 15,
                               'profile': {'state': 'applied', 'applied_id': 'abcd0001', 'id': 'abcd0001',
                                           'running': {'id': 'abcd0001', 'generation': 1, 'topology': 'two perpendicular wheels',
                                                       'imu': {'source': 'brain_vex', 'vex_smart_port': 1},
                                                       'footprint': {'front_m': 0.2, 'back_m': 0.2, 'left_m': 0.18, 'right_m': 0.18}}},
                               'state': {'health': {'encoders_fresh': True, 'gyro_fresh': True}, 'map_id': 1,
                                         'estimate_id': 1, 'calibration': 'none'},
                               'wheels': [{'port': 0, 'counts': int(t * 2), 'travel_m': t / 1000 * 0.4, 'fresh': True,
                                           'valid': True, 'discontinuity': 0, 'counts_per_rev': 2048, 'gear': 1,
                                           'radius_m': 0.03, 'travel_scale': 1, 'reversed': False},
                                          {'port': 1, 'counts': int(300 * math.sin(t / 2000)), 'travel_m': 0.01,
                                           'fresh': True, 'valid': True, 'discontinuity': 0, 'counts_per_rev': 2048,
                                           'gear': 1, 'radius_m': 0.03, 'travel_scale': 1, 'reversed': False}],
                               'path': {'session': 0x1234, 'command_id': 7, 'received_host_ms': 1000, 'mode': 'direct',
                                        'points': path_pts, 'age_ms': int(t - 1000)}},
                'pico': None, 'events': list(self.events),
                'sources': [{'id': 'stub.pose', 'kind': 'resource', 'state': 'valid', 'receipt_age_ms': 8,
                             'sequence': self.publication, 'epoch': self.epoch, 'diagnostic': ''}],
                'workers': {'estimation': {'running': True, 'rate_hz': 100.0, 'cycles': int(t / 10)},
                            'field': {'running': True, 'rate_hz': 30.0, 'cycles': int(t / 33)},
                            'inspection': {'clients': len(clients)}},
                'diagnostics': {'estimation': {'functions': [], 'links': [{'id': 'brain_usb', 'bytes': int(t * 3),
                                                                           'packets': int(t / 10), 'decode_errors': 0,
                                                                           'seq_gaps': 0}]}},
                'inspection': {'clients': clients, 'state_build': {'last_us': 40, 'mean_us': 42, 'last_bytes': 900}},
                'hub': {'posted': self.publication, 'dropped': 0, 'queued': 0}}

    def telemetry(self, t):
        self.seq['telemetry'] += 1
        seg = int(t / 3000) % 3
        return {'type': 'telemetry', 'seq': self.seq['telemetry'], 'host_ms': int(t), 'session': 0x1234,
                'stamp_ms': int(t) + 40000, 'flags': 3, 'attitude': {'roll_deg': 1.0, 'pitch_deg': -0.5},
                'motion': {'command_id': 7, 'state': 2, 'state_name': 'running', 'reason': 0, 'reason_name': 'none',
                           'mode': 1, 'mode_name': 'direct', 'segment': seg, 'segment_count': 3,
                           'target': {'x_m': 2.8, 'y_m': 2.6, 'heading_deg': 45.0},
                           'cmd': {'vx_m_s': 0.35, 'vy_m_s': 0.0, 'omega_deg_s': 40.0},
                           'cross_track_m': 0.02 * math.sin(t / 700), 'distance_error_m': 1.0, 'heading_error_deg': 12.0,
                           'drive_fault': 0, 'drive_fault_name': 'none'},
                'wheels': {'rpm': [110.0, 112.0]}}

    def instrumentation(self, t, client):
        self.seq['instrumentation'] += 1
        raw = [{'ms_ago': 12, 'dir': 'rx', 'hex': 'AA5514000102030405'},
               {'ms_ago': 10, 'dir': 'tx', 'hex': 'AA550401'},
               {'ms_ago': 9, 'dir': 'tx_attempted', 'hex': 'AA550402'}] if client.sub.get('raw') else []
        dec = [{'ms_ago': 12, 'dir': 'rx', 'name': 'sensor v2', 'fields': 'seq=4 enc0=1234'}] if client.sub.get('decoded') else []
        return {'type': 'instrumentation', 'seq': self.seq['instrumentation'], 'host_ms': int(t),
                'links': [{'id': 'pico', 'kind': 'pico_uart', 'rx_bytes': int(t * 1.5), 'tx_attempted': 40,
                           'tx_accepted': 36, 'rx_frames': int(t / 20), 'tx_frames': 10, 'rejected': 2,
                           'reader': {'bytes': int(t * 1.5), 'frames': int(t / 20), 'sync_dropped': 3, 'length_errors': 0,
                                      'check_errors': 1},
                           'rates': {'rx_bytes_s': 1550.0, 'tx_bytes_s': 2.0, 'rx_frames_s': 50.0},
                           'last_rx_ms_ago': 4, 'last_tx_ms_ago': 900, 'last_valid_rx_ms_ago': 4,
                           'raw_on': bool(client.sub.get('raw')), 'raw': raw, 'decoded': dec,
                           'errors': [{'ms_ago': 5000, 'reason': 'crc'}]}],
                'pico': {'available': True, 'reason': '', 'status': {'imu_state': 'ready', 'age_ms': 150},
                         'diag': {'age_ms': 300, 'seq': 9, 'firmware': 'hat2_bno08x',
                                  'pins': [{'name': 'imu_int', 'level': 'HIGH', 'known': True, 'driven': False},
                                           {'name': 'pi_rx', 'level': 'HIGH', 'known': True, 'driven': False}],
                                  'imu': {'rx': 100, 'bad': 0}, 'link_rx_bad': 0, 'ticks_skipped': 0},
                         'encoders': [{'port': 0, 'counts': int(t * 2), 'delta_1s': 2000, 'direction': 'fwd', 'note': ''},
                                      {'port': 1, 'counts': 55, 'delta_1s': 0, 'direction': 'still',
                                       'note': 'no change: stationary or disconnected, cannot tell'}]},
                'brain': {'telemetry_age_ms': 60, 'telemetry_supported': True, 'vex_imu': {'rotation_deg': 12.5}},
                'hub': {'posted': {'robot_state': self.publication}, 'dropped': {'robot_state': 0}, 'queued': 0}}

    # --- plumbing ---

    def broadcast(self, kind, doc, want=None):
        text = json.dumps(doc) if not isinstance(doc, str) else doc
        with self.lock:
            clients = list(self.clients)
        for c in clients:
            if want is None or want(c):
                c.enqueue(kind, text)

    def add_event(self, text):
        t = now_ms()
        self.seq['event'] += 1
        e = {'seq': self.seq['event'], 'host_ms': int(t), 'text': text}
        self.events.append({'host_ms': int(t), 'text': text, 'seq': e['seq']})
        self.recorder.note_event({'pi_host_us': int(t * 1000), 'pi_session': self.session,
                                  'reset_count': self.reset_count, 'kind': 'runtime', 'source': 'stub', 'text': text})
        self.broadcast('event', dict(e, type='event'))

    def greet(self, c):
        c.enqueue('reliable', json.dumps(self.hello(c)))
        c.enqueue('reliable', json.dumps(self.history_doc()))
        c.enqueue('reliable', json.dumps(dict(self.recorder.status(), type='capture')))

    def history_doc(self):
        return {'type': 'history', 'session': {'id': self.session, 'reset_count': self.reset_count},
                'publication': self.publication, 'trail': list(self.history)}

    def on_client_message(self, c, msg):
        kind = msg.get('type')
        c.received[kind] += 1
        if kind == 'ping':
            c.ping_ids.append(msg.get('id'))
            t = now_ms()
            c.enqueue('reliable', json.dumps({'type': 'pong', 'id': msg.get('id'), 'client_ms': msg.get('client_ms'),
                                              'host_ms': int(t), 'host_us': int(t * 1000)}))
        elif kind == 'subscribe':
            for k, v in msg.items():
                if k != 'type':
                    c.sub[k] = v
        elif kind == 'history':
            c.enqueue('reliable', json.dumps(self.history_doc()))

    def add(self, sock):
        with self.lock:
            c = Client(self, sock, self.next_cid)
            self.next_cid += 1
            self.clients.append(c)
        self.greet(c)
        threading.Thread(target=c.writer, daemon=True).start()
        return c

    def drop(self, c):
        with self.lock:
            if c in self.clients:
                self.clients.remove(c)
                self.dropped_stats.append({'id': c.id, 'received': dict(c.received), 'sent': dict(c.sent),
                                           'ping_ids': list(c.ping_ids)})

    dropped_stats = []

    def kill_all(self):
        with self.lock:
            clients = list(self.clients)
        for c in clients:
            c.close()

    def run(self):
        last = {'state': 0.0, 'diag': 0.0, 'telemetry': 0.0, 'instrumentation': 0.0, 'event': now_ms(), 'keep': 0.0}
        last_capture_version = self.recorder.version
        while self.running:
            t = now_ms()
            if t >= self.pause_until:
                self.publication += 1
                self.robot = self.robot_doc(t)
                r = self.robot
                if r['measured_at_host_ms'] is not None:   # the trail holds measured poses only
                    self.history.append({'host_ms': r['measured_at_host_ms'], 'x_m': r['odom']['x_m'], 'y_m': r['odom']['y_m'],
                                         'heading_deg': r['odom']['heading_deg'], 'epoch': self.epoch, 'attitude_valid': True})
                self.recorder.note_robot({
                    'pi_host_us': int(t * 1000), 'pi_session': self.session, 'reset_count': self.reset_count,
                    'source': 'stub', 'publication': self.publication, 'valid': True, 'placed': True,
                    'measured_pi_host_ms': r['measured_at_host_ms'], 'source_age_ms': r['age_ms'],
                    'odometry_epoch': self.epoch, 'anchor_revision': self.anchor_rev,
                    'odom_x_m': r['odom']['x_m'], 'odom_y_m': r['odom']['y_m'], 'odom_heading_deg': r['odom']['heading_deg'],
                    'field_x_m': r['field']['x_m'], 'field_y_m': r['field']['y_m'],
                    'field_heading_deg': r['field']['heading_deg'], 'odom_vx_m_s': r['vx_m_s'], 'odom_vy_m_s': 0.0,
                    'yaw_rate_deg_s': 45.0, 'confidence': 0.9, 'attitude_status': 'measured',
                    'roll_deg': r['attitude']['roll_deg'], 'pitch_deg': r['attitude']['pitch_deg'], 'stationary': False})
                if t - last['state'] >= 1000.0 / self.args.state_hz:
                    last['state'] = t
                    last['keep'] = t   # a keepalive only after 1 s without a state
                    self.broadcast('state', self.state(t))
            elif self.robot and t - last['keep'] >= 1000.0:
                last['keep'] = t
                self.broadcast('state', self.state(t))   # keepalive, same publication
            if self.robot and t - last['diag'] >= 250.0:
                last['diag'] = t
                with self.lock:
                    clients = list(self.clients)
                for c in clients:
                    if c.sub.get('diag', True):
                        c.enqueue('diag', json.dumps(self.diag(t, c)))
            if t - last['telemetry'] >= 100.0:
                last['telemetry'] = t
                doc = self.telemetry(t)
                m = doc['motion']
                self.recorder.note_telemetry({
                    'pi_host_us': int(t * 1000), 'pi_session': self.session, 'reset_count': self.reset_count,
                    'source': 'stub', 'brain_session': 0x1234, 'decoded': True, 'motion_present': True,
                    'attitude_present': True, 'roll_deg': 1.0, 'pitch_deg': -0.5, 'command_id': 7, 'motion_state': 2,
                    'path_segment': m['segment'], 'path_segment_count': 3, 'target_field_x_m': 2.8,
                    'target_field_y_m': 2.6, 'target_field_heading_deg': 45.0, 'cmd_body_vx_m_s': 0.35,
                    'cmd_body_vy_m_s': 0.0, 'cmd_omega_deg_s': 40.0, 'cross_track_m': m['cross_track_m'],
                    'distance_error_m': 1.0, 'heading_error_deg': 12.0, 'drive_fault': 0,
                    'wheels_present': True, 'wheel_count': 2, 'wheel0_rpm': 110.0, 'wheel1_rpm': 112.0})
                self.broadcast('telemetry', doc, lambda c: c.sub.get('telemetry', True))
            if t - last['instrumentation'] >= 250.0:
                last['instrumentation'] = t
                with self.lock:
                    clients = list(self.clients)
                for c in clients:
                    if c.sub.get('instrumentation'):
                        c.enqueue('instrumentation', json.dumps(self.instrumentation(t, c)))
            if t - last['event'] >= 5000.0:
                last['event'] = t
                self.add_event(f'stub event at {t / 1000:.1f} s')
            # as the runtime: a 'capture' message whenever the status version
            # moved (an HTTP start or cancel, progress, the end)
            self.recorder.tick()
            if self.recorder.version != last_capture_version:
                last_capture_version = self.recorder.version
                self.broadcast('capture', dict(self.recorder.status(), type='capture'))
            time.sleep(0.01)

    # --- controls ---

    def control(self, name, q):
        if name == 'kill':
            self.kill_all()
        elif name == 'reset':
            self.reset_count += 1
            self.epoch += 1
            self.history.clear()
            self.add_event('pi reset (stub)')
            with self.lock:
                clients = list(self.clients)
            for c in clients:
                c.enqueue('reliable', json.dumps(self.hello(c)))
                c.enqueue('reliable', json.dumps(self.history_doc()))
        elif name == 'restart':
            self.session = os.urandom(8).hex()
            self.reset_count = 0
            self.history.clear()
            self.events.clear()
            self.kill_all()
        elif name == 'epoch':
            self.epoch += 1
            self.t_epoch = now_ms()
            self.add_event('odometry epoch changed (stub)')
        elif name == 'jump':
            self.jump += 1.0
        elif name == 'pause':
            self.pause_until = now_ms() + float(q.get('ms', ['1000'])[0])
        elif name == 'unmeasured':
            on = q.get('on', ['1'])[0] == '1'
            self.unmeasured = ('source' if q.get('source', ['0'])[0] == '1' else 'placement') if on else False
        elif name == 'ready':
            self.not_ready = q.get('on', ['1'])[0] != '1'
        elif name == 'clock':
            self.clock_mapped = q.get('mapped', ['1'])[0] == '1'
        elif name == 'capfail':
            with self.recorder.lock:
                self.recorder.fail = q.get('mode', ['none'])[0]
        elif name == 'stats':
            with self.lock:
                return {'clients': [{'id': c.id, 'received': dict(c.received), 'sent': dict(c.sent),
                                     'replaced': dict(c.replaced), 'sub': c.sub, 'ping_ids': list(c.ping_ids)}
                                    for c in self.clients],
                        'dropped': self.dropped_stats[-20:], 'session': self.session, 'reset_count': self.reset_count}
        return {'ok': True}


def make_handler(stub):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'

        def log_message(self, *a):
            pass

        def reply(self, code, body, ctype='application/json'):
            data = body if isinstance(body, bytes) else (json.dumps(body) if not isinstance(body, str) else body).encode()
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(data)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(data)

        def do_POST(self):
            self.do_GET()

        def do_GET(self):
            u = urllib.parse.urlparse(self.path)
            q = urllib.parse.parse_qs(u.query)
            path = u.path
            if path == '/ws' and self.headers.get('Upgrade', '').lower() == 'websocket':
                key = self.headers['Sec-WebSocket-Key']
                accept = base64.b64encode(hashlib.sha1((key + GUID).encode()).digest()).decode()
                self.wfile.write(('HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n'
                                  f'Sec-WebSocket-Accept: {accept}\r\n\r\n').encode())
                self.wfile.flush()
                c = stub.add(self.connection)
                c.reader()
                self.close_connection = True
                return
            if path == '/api/health':
                return self.reply(200, {'ok': True, 'running': True, 'stub': True})
            if path == '/api/hello':
                return self.reply(200, stub.hello())
            if path == '/api/snapshot':
                t = now_ms()
                return self.reply(200, {'type': 'snapshot', 'host_ms': int(t), 'robot': stub.robot,
                                        'session': {'id': stub.session, 'reset_count': stub.reset_count}})
            if path == '/api/capture/status':
                return self.reply(200, stub.recorder.status())
            if path == '/api/capture/start':
                pre = float(q.get('pre_s', ['5'])[0])
                post = float(q.get('post_s', ['10'])[0])
                cid, err = stub.recorder.start(pre, post, q.get('streams', ['default'])[0])
                return self.reply(200 if cid else 409, {'ok': True, 'id': cid} if cid else {'ok': False, 'error': err})
            if path == '/api/capture/cancel':
                ok = stub.recorder.cancel(q.get('id', [''])[0])
                return self.reply(200 if ok else 404, {'ok': ok})
            if path.startswith('/api/capture/') and path.endswith('.zip'):
                data = stub.recorder.bundle(path[len('/api/capture/'):-4])
                return self.reply(200, data, 'application/zip') if data else self.reply(404, {'ok': False})
            if path.startswith('/stub/'):
                return self.reply(200, stub.control(path[6:], q))
            rel = 'index.html' if path == '/' else path.lstrip('/')
            base = THREE if rel.startswith('vendor/') else VIEWER
            if rel.startswith('vendor/'):
                rel = rel[len('vendor/'):]
            full = os.path.normpath(os.path.join(base, rel))
            if not full.startswith(base) or not os.path.isfile(full):
                return self.reply(404, {'ok': False, 'error': 'not found'})
            ctype = {'.html': 'text/html', '.js': 'text/javascript', '.css': 'text/css', '.json': 'application/json',
                     '.svg': 'image/svg+xml'}.get(os.path.splitext(full)[1], 'application/octet-stream')
            with open(full, 'rb') as f:
                return self.reply(200, f.read(), ctype)

    return Handler


def serve(port=8799, state_hz=30, block=True):
    args = argparse.Namespace(port=port, state_hz=state_hz)
    stub = Stub(args)
    httpd = ThreadingHTTPServer(('127.0.0.1', port), make_handler(stub))
    httpd.daemon_threads = True
    httpd.handle_error = lambda request, address: None   # browsers drop sockets; not an error here
    threading.Thread(target=stub.run, daemon=True).start()
    if block:
        print(f'stub inspect/2 server on http://127.0.0.1:{httpd.server_address[1]}/')
        httpd.serve_forever()
    else:
        threading.Thread(target=httpd.serve_forever, daemon=True).start()
    return stub, httpd


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', type=int, default=8799)
    ap.add_argument('--state-hz', type=float, default=30.0)
    a = ap.parse_args()
    serve(a.port, a.state_hz)
