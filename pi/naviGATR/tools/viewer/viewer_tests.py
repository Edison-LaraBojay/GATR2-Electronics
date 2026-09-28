"""viewer_tests.py - browser tests of the spectaGATR viewer (headless Chrome/Edge over CDP).

Python standard library only: no pip, no global installs. Runs on Windows (Git
Bash, cmd) and Linux. Every scenario prints its measurements and PASS/FAIL.

Scenarios
    unit       the pure modules in a real browser (tools/viewer/unit.html),
               including replay of a fixture bundle against closed-form truth
    live       the page against a server: live feed, 3D, trail, graphs,
               instrumentation (inspect/2), latency panel, an orbit drag
    nowebgl    the same page with WebGL disabled: documents still flow, and
               a replayed bundle still drives the readout and timeline
    slow       the page behind a bandwidth-limited TCP proxy with CPU
               throttling; staleness is measured on the Pi clock only
               (GET /api/snapshot host_ms minus the page's newest state host_ms)
    reconnect  socket drop with the same process (stub control, or a relay
               cut for --binary), session reset and server restart
    replay     a fixture bundle loaded through the file input: REPLAY banner,
               live-only controls off, pose at seek times, play speed, event
               seek, graphs, nothing sent to the server while replaying, and
               stale spans (a hole in the rows, a frozen pose with growing age)
    record     record on the server (capture UI), then replay that capture; a
               capture started elsewhere shows through pushed status
    capfail    a failed capture file write (stub control, or a blocked capture
               directory for --binary) and a failed bundle (stub only)
    paths      stub only: state pause, teleport, odometry epoch, unmeasured
               poses, not-ready localization, unmapped clock, two viewers

Targets
    --stub               tools/viewer/stub_server.py (inspect/2 test double)
    --binary PATH        a built navigatr; the synthetic demo config is
                         rewritten into a temp dir with static_root pointing at
                         this working tree's spectaGATR, so the page under test
                         is the checked-out one (vendor/ still comes from the
                         binary)
    --config PATH        config for --binary (default config/demo/synthetic_field_demo.xml)

Examples
    py tools/viewer/viewer_tests.py unit
    py tools/viewer/viewer_tests.py all --stub
    py tools/viewer/viewer_tests.py live slow reconnect --binary build/navigatr.exe
    py tools/viewer/viewer_tests.py record --binary build/navigatr.exe

These are host tests of the browser page. They are not hardware validation,
and the stub target is not the naviGATR server.
"""

import argparse
import asyncio
import json
import os
import re
import shutil
import socket
import subprocess
import sys
import tempfile
import threading
import time
import urllib.request
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer

HERE = os.path.dirname(os.path.abspath(__file__))
NAV = os.path.abspath(os.path.join(HERE, '..', '..'))
sys.path.insert(0, HERE)
sys.dont_write_bytecode = True   # no __pycache__ in the source tree

import capture_fixture as cf  # noqa: E402
from cdp import Browser  # noqa: E402


def free_port():
    with socket.socket() as s:
        s.bind(('127.0.0.1', 0))
        return s.getsockname()[1]


def get_json(url, timeout=3.0, method='GET'):
    req = urllib.request.Request(url, method=method)
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read())


class Result:
    def __init__(self, name):
        self.name = name
        self.checks = []
        self.metrics = {}

    def check(self, cond, what):
        self.checks.append((bool(cond), what))
        print(('  ok   ' if cond else '  FAIL ') + what, flush=True)
        return cond

    @property
    def ok(self):
        return all(c for c, _ in self.checks) and self.checks


# --- targets ---

class StubTarget:
    kind = 'stub'

    def __init__(self, out):
        import stub_server
        self.port = free_port()
        self.stub, self.httpd = stub_server.serve(self.port, 30, block=False)
        self.url = f'http://127.0.0.1:{self.port}/'
        self.contract2 = True

    def control(self, name, **q):
        qs = '&'.join(f'{k}={v}' for k, v in q.items())
        return get_json(f'{self.url}stub/{name}' + (f'?{qs}' if qs else ''))

    def kill_connections(self):
        self.control('kill')

    def restart(self):
        self.control('restart')

    def stop(self):
        self.httpd.shutdown()
        self.stub.running = False


class BinaryTarget:
    kind = 'binary'

    def __init__(self, out, binary, config, extra_attrs=''):
        self.binary = os.path.abspath(binary)
        self.out = out
        self.extra_attrs = extra_attrs
        self.port = free_port()
        self.config = self.write_config(config)
        self.proc = None
        self.log = None
        self.start()
        self.url = f'http://127.0.0.1:{self.port}/'
        hello = get_json(self.url + 'api/hello')
        self.contract2 = hello.get('contract') == 'navigatr.inspect/2'

    def write_config(self, config):
        """The same configuration with absolute file= paths and static_root
        pointed at this tree's viewer, in a temp dir."""
        src = os.path.abspath(config)
        base = os.path.dirname(src)
        text = open(src, encoding='utf-8').read()
        text = re.sub(r'file="([^"]+)"', lambda m: 'file="%s"' % os.path.abspath(os.path.join(base, m.group(1))).replace('\\', '/'), text)
        root = os.path.join(NAV, 'spectaGATR').replace('\\', '/')
        if 'static_root=' in text:
            text = re.sub(r'static_root="[^"]*"', f'static_root="{root}"', text)
        else:
            text = re.sub(r'<Inspection\b', f'<Inspection static_root="{root}"', text, count=1)
        if self.extra_attrs:
            text = re.sub(r'<Inspection\b', f'<Inspection {self.extra_attrs}', text, count=1)
        path = os.path.join(self.out, getattr(self, 'config_name', 'viewer_test_config.xml'))
        with open(path, 'w', encoding='utf-8') as f:
            f.write(text)
        return path

    def start(self):
        env = dict(os.environ)
        if os.name == 'nt':
            for d in (os.environ.get('MINGW_BIN'), r'C:\msys64\ucrt64\bin'):
                if d and os.path.exists(os.path.join(d, 'libstdc++-6.dll')):
                    env['PATH'] = d + os.pathsep + env.get('PATH', '')
                    break
        self.log = open(os.path.join(self.out, f'navigatr.{int(time.time())}.log'), 'w')
        self.proc = subprocess.Popen([self.binary, self.config, '--inspect-port', str(self.port), '--cycles', '100000000'],
                                     stdout=self.log, stderr=subprocess.STDOUT, env=env)
        deadline = time.time() + 30
        while time.time() < deadline:
            try:
                if get_json(f'http://127.0.0.1:{self.port}/api/health', 1).get('ok'):
                    return
            except OSError:
                pass
            if self.proc.poll() is not None:
                raise RuntimeError('navigatr exited early; see ' + self.log.name)
            time.sleep(0.3)
        raise RuntimeError('navigatr did not come up')

    def kill_connections(self):
        # no way to drop sockets from outside: restart instead
        self.restart()

    def restart(self):
        self.stop()
        time.sleep(1.0)
        self.start()

    def stop(self):
        if self.proc and self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(10)
            except subprocess.TimeoutExpired:
                self.proc.kill()
        if self.log:
            self.log.close()


# --- a bandwidth-limited TCP proxy (browser <-> server) ---

class ThrottleProxy:
    """Server-to-browser bytes at rate_bps. The upstream socket gets a small
    receive buffer before it connects (so the TCP window stays small) and
    the proxy holds at most a few KB, so backpressure reaches the server
    instead of piling up here. What the server's own kernel send buffer
    holds is still in the path, as it would be on a slow network."""

    def __init__(self, target_port, rate_bps):
        self.target_port = target_port
        self.rate = rate_bps
        self.port = free_port()
        self.read_bytes = 0      # from the server, WebSocket connections only
        self.written_bytes = 0   # to the browser, WebSocket connections only
        self.loop = asyncio.new_event_loop()
        self.thread = threading.Thread(target=self.loop.run_forever, daemon=True)
        self.thread.start()
        asyncio.run_coroutine_threadsafe(self.start(), self.loop).result(5)

    async def start(self):
        self.server = await asyncio.start_server(self.handle, '127.0.0.1', self.port)

    async def handle(self, reader, writer):
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4096)
            sock.setblocking(False)
            await asyncio.get_running_loop().sock_connect(sock, ('127.0.0.1', self.target_port))
            ur, uw = await asyncio.open_connection(sock=sock, limit=2048)
        except OSError:
            writer.close()
            return

        conn = {'ws': False}

        async def up():
            try:
                while True:
                    data = await reader.read(4096)
                    if not data:
                        break
                    if data.startswith(b'GET /ws'):
                        conn['ws'] = True
                    uw.write(data)
                    await uw.drain()
            except (OSError, asyncio.CancelledError):
                pass
            uw.close()

        # a thin pipe: no more than a few KB waits in the proxy either way
        writer.transport.set_write_buffer_limits(high=4096)
        uw.transport.set_write_buffer_limits(high=4096)

        # paced against a schedule, not per chunk: timer granularity (about
        # 15.6 ms on Windows) would otherwise cut the rate well below rate_bps
        chunk = int(min(16384, max(512, self.rate * 0.02)))

        async def down():
            loop = asyncio.get_running_loop()
            start = loop.time()
            sent = 0
            try:
                while True:
                    data = await ur.read(chunk)
                    if not data:
                        break
                    if conn['ws']:
                        self.read_bytes += len(data)
                    sent += len(data)
                    now = loop.time()
                    if start + sent / self.rate < now - 0.1:
                        start = now - sent / self.rate   # idle: no credit for the pause
                    delay = start + sent / self.rate - now
                    if delay > 0:
                        await asyncio.sleep(delay)
                    writer.write(data)
                    if conn['ws']:
                        self.written_bytes += len(data)
                    await writer.drain()
            except (OSError, asyncio.CancelledError):
                pass
            writer.close()

        await asyncio.gather(up(), down())

    def stop(self):
        self.loop.call_soon_threadsafe(self.server.close)


class KillProxy:
    """A plain TCP relay whose connections can be dropped from outside: a
    socket loss with the same server process (same session) behind it."""

    def __init__(self, target_port):
        self.target_port = target_port
        self.port = free_port()
        self.conns = []
        self.loop = asyncio.new_event_loop()
        threading.Thread(target=self.loop.run_forever, daemon=True).start()
        asyncio.run_coroutine_threadsafe(self.start(), self.loop).result(5)

    async def start(self):
        self.server = await asyncio.start_server(self.handle, '127.0.0.1', self.port)

    async def handle(self, reader, writer):
        try:
            ur, uw = await asyncio.open_connection('127.0.0.1', self.target_port)
        except OSError:
            writer.close()
            return
        self.conns.append((writer, uw))

        async def pipe(a, bw):
            try:
                while True:
                    d = await a.read(65536)
                    if not d:
                        break
                    bw.write(d)
                    await bw.drain()
            except (OSError, asyncio.CancelledError):
                pass
            bw.close()

        await asyncio.gather(pipe(reader, uw), pipe(ur, writer))

    def kill(self):
        def k():
            for w, uw in self.conns:
                w.transport.abort()
                uw.transport.abort()
            self.conns.clear()
        self.loop.call_soon_threadsafe(k)

    def stop(self):
        self.kill()
        self.loop.call_soon_threadsafe(self.server.close)


# --- helpers on the page ---

STATUS = 'JSON.stringify(Object.assign({}, document.getElementById("status").dataset))'
BADGES = ('Object.fromEntries([...document.querySelectorAll("#badges .badge")].filter(e => !e.hidden)'
          '.map(e => [e.dataset.badge, e.textContent]))')


def status(b):
    return json.loads(b.eval(STATUS))


def badges(b):
    return b.eval(BADGES) or {}


def poll(b, expr, timeout, step=0.05):
    """Seconds until expr is truthy on the page, or None after timeout."""
    t0 = time.time()
    while time.time() - t0 < timeout:
        if b.eval(expr):
            return time.time() - t0
        b.drain(step)
    return None


def unique_pings(stats):
    """Ping ids the stub saw from each connection: [(client id, repeats)]."""
    out = []
    for c in stats['clients'] + stats['dropped']:
        ids = c.get('ping_ids', [])
        out.append((c['id'], len(ids) - len(set(ids))))
    return out


def metrics(b):
    return b.eval('window.__navigatr.metrics()')


def open_page(out, url, name):
    b = Browser(log_dir=out)
    b.navigate(url, wait_load=True, timeout=40)
    return b


def wait_live(b, timeout=30):
    try:
        b.wait_for('document.getElementById("status").dataset.liveSeen === "1"', timeout=timeout)
    except TimeoutError:
        diag = b.eval('JSON.stringify({status: Object.assign({}, document.getElementById("status").dataset), '
                      'm: window.__navigatr && window.__navigatr.metrics ? window.__navigatr.metrics() : null})')
        raise TimeoutError('feed never reached live: ' + diag[:1500] + ' exceptions ' + str(b.exceptions[:3]) +
                           ' console ' + str(b.console[-5:]))


def page_errors(b):
    errs = b.eval('window.__navigatr ? window.__navigatr.errors : ["no __navigatr"]') or []
    return list(errs) + b.exceptions


def fmt(v):
    return 'n/a' if v is None else (f'{v:.1f}' if isinstance(v, float) else str(v))


# --- scenarios ---

def scenario_unit(args, out):
    r = Result('unit')
    fixture = os.path.join(out, 'fixture_capture.zip')
    with open(fixture, 'wb') as f:
        f.write(cf.synthetic())
    for variant in ('hole', 'frozen'):
        with open(os.path.join(out, f'fixture_{variant}.zip'), 'wb') as f:
            f.write(cf.synthetic(variant))
    expected = {'points': [], 'epoch_t_ms': cf.T0_US / 1000 + cf.EPOCH_S * 1000, 'place_t_ms': cf.T0_US / 1000 + cf.PLACE_S * 1000,
                'stale_probe_ms': cf.T0_US / 1000 + 6500, 'fresh_probe_ms': cf.T0_US / 1000 + 3000}
    for t_s in (0.013, 1.0, 1.99, 2.0, 3.337, 6.99, 7.0, 7.5, 11.97):
        k = int(t_s * cf.RATE_HZ + 1e-9)
        tr = k / cf.RATE_HZ
        x, y, _, placed, _ = cf.robot_at(tr)
        expected['points'].append({'t_ms': cf.T0_US / 1000 + t_s * 1000, 'x': round(x, 6), 'y': round(y, 6), 'placed': placed})
    with open(os.path.join(out, 'fixture_expected.json'), 'w') as f:
        json.dump(expected, f)

    class Handler(SimpleHTTPRequestHandler):
        def __init__(self, *a, **k):
            super().__init__(*a, directory=NAV, **k)

        def log_message(self, *a):
            pass

        def translate_path(self, path):
            if path.startswith('/__out/'):
                return os.path.join(out, path[len('/__out/'):].split('?')[0])
            if path.startswith('/spectaGATR/vendor/'):
                # what the runtime serves at vendor/: the pinned three.js
                return os.path.join(NAV, 'third_party', 'three', path[len('/spectaGATR/vendor/'):].split('?')[0])
            return super().translate_path(path)

        def guess_type(self, path):
            return 'text/javascript' if path.endswith('.js') else super().guess_type(path)

    port = free_port()
    httpd = ThreadingHTTPServer(('127.0.0.1', port), Handler)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    b = Browser(log_dir=out)
    try:
        b.navigate(f'http://127.0.0.1:{port}/tools/viewer/unit.html?fixture=/__out/fixture_capture.zip'
                   f'&expected=/__out/fixture_expected.json&hole=/__out/fixture_hole.zip&frozen=/__out/fixture_frozen.zip')
        b.wait_for('window.__unit && window.__unit.done', timeout=60)
        u = b.eval('window.__unit')
        for t in u['results']:
            r.check(t['ok'], t['name'] + ('' if t['ok'] else '\n         ' + t.get('detail', '').replace('\n', '\n         ')))
        r.metrics = {'passed': u['passed'], 'failed': u['failed']}
        r.check(not b.exceptions, f'no page exceptions {b.exceptions[:3]}')
    finally:
        b.close()
        httpd.shutdown()
    return r


def scenario_live(args, target, out):
    r = Result('live')
    b = open_page(out, target.url, 'live')
    try:
        wait_live(b)
        b.drain(4)
        st = status(b)
        r.check(st['liveSeen'] == '1', 'feed reached live')
        r.check(int(st['snapshots']) > 20, f'state messages received: {st["snapshots"]}')
        r.check(st['webgl'] == '1', '3D view initialized')
        r.check(int(st['trail']) > 20, f'trail points: {st["trail"]}')
        r.check(st['attitude'] in ('valid', 'assumed_level', 'stale', 'unavailable'), f'attitude badge: {st["attitude"]} / {st.get("attitudeStatus")}')
        r.check(st['contract'] == ('navigatr.inspect/2' if target.contract2 else 'navigatr.inspect/1'), f'contract {st["contract"]}')
        r.check(b.eval('!!document.querySelector("#readout b")') and b.eval('document.querySelector("#readout b").textContent') != '',
                'pose readout shows numbers')
        # graphs tab: plots draw with data
        b.eval('document.querySelector("#dock-tabs button[data-tab=graphs]").click()')
        b.drain(2)
        vals = b.eval('[...document.querySelectorAll(".legend-value")].map(e => e.textContent)')
        r.check(any(v not in ('n/a', '') for v in vals), f'graph legend has values ({len(vals)} series)')
        r.check(b.eval('document.querySelectorAll(".plot canvas").length') >= 1, 'plots drawn')
        # instrumentation tab subscribes (inspect/2 only)
        b.eval('document.querySelector("#dock-tabs button[data-tab=inst]").click()')
        if target.contract2:
            try:
                b.wait_for('(window.__navigatr.metrics().counts.instrumentation || 0) > 0', timeout=8)
                r.check(True, 'instrumentation arrives while its tab is visible')
            except TimeoutError:
                r.check(False, 'instrumentation arrives while its tab is visible')
        else:
            b.drain(1)
            r.check('inspect/1' in b.eval('document.getElementById("inst-status").textContent'), 'inspect/1: instrumentation says unsupported')
        b.eval('document.querySelector("#dock-tabs button[data-tab=latency]").click()')
        b.drain(1.5)
        rows = b.eval('document.getElementById("lat-terms").rows.length')
        r.check(rows >= 8, f'latency terms table ({rows} rows)')
        b.eval('document.querySelector("#dock-tabs button[data-tab=diag]").click()')
        # hidden instrumentation tab unsubscribes: the count stops growing
        if target.contract2:
            b.drain(1.5)
            n0 = metrics(b)['counts'].get('instrumentation', 0)
            b.drain(2.0)
            n1 = metrics(b)['counts'].get('instrumentation', 0)
            r.check(n1 - n0 <= 1, f'instrumentation stops after its tab is hidden ({n0} -> {n1})')
        # orbit drag while live; frame stats during interaction
        b.call('Page.bringToFront')
        m0 = metrics(b)
        box = b.eval('(() => { const r = document.querySelector("#view canvas").getBoundingClientRect(); return [r.left, r.top, r.width, r.height]; })()')
        cx, cy = box[0] + box[2] / 2, box[1] + box[3] / 2
        b.drag(cx, cy, cx + 250, cy + 60, steps=40)
        b.drag(cx + 250, cy + 60, cx - 100, cy, steps=40)
        m1 = metrics(b)
        r.check(m1['renders'] > m0['renders'], f'renders during the drag: {m1["renders"] - m0["renders"]}')
        r.metrics = {k: m1[k] for k in ('frameInterval', 'frameWork', 'receiveToRender', 'publishToReceive', 'sourceAge', 'smoothLag', 'rtt')}
        r.metrics['handler'] = m1['handler']
        r.metrics['trailUploadedBytes'] = m1['trailUploadedBytes']
        # camera panel out of view: the page asks for no previews, and back
        # in view for its budget again (configurations with a camera only)
        if b.eval('(window.__navigatr.hello.camera_sensors || []).length') > 0:
            p0 = metrics(b)['sent'].get('preview', 0)
            b.eval('document.getElementById("camera").style.display = "none"')
            b.drain(1.5)
            f0 = int(status(b)['frames'])
            b.drain(2.0)
            f1 = int(status(b)['frames'])
            p1 = metrics(b)['sent'].get('preview', 0)
            r.check(p1 == p0 + 1 and f1 - f0 <= 1, f'camera panel hidden: preview hz 0 asked ({p0} -> {p1}), frames {f0} -> {f1}')
            b.eval('document.getElementById("camera").style.display = ""')
            b.drain(2.0)
            f2 = int(status(b)['frames'])
            p2 = metrics(b)['sent'].get('preview', 0)
            r.check(p2 == p1 + 1 and f2 > f1, f'back in view: budget asked again ({p1} -> {p2}), frames {f1} -> {f2}')
        errs = page_errors(b)
        r.check(not errs, f'no page errors {errs[:3]}')
        b.screenshot(os.path.join(out, 'live.png'))
    finally:
        b.close()
    return r


def scenario_slow(args, target, out):
    r = Result('slow')
    proxy = ThrottleProxy(target.port, args.rate)
    url = f'http://127.0.0.1:{proxy.port}/' + (('?' + args.query) if args.query else '')
    b = Browser(log_dir=out)
    try:
        b.navigate(url, wait_load=True, timeout=90)
        wait_live(b, 60)
        if args.preview_hz is not None:
            # the page's own preview budget control, as a person would set it
            b.eval(f'document.getElementById("preview-hz").value = "{args.preview_hz}"; '
                   'document.getElementById("preview-form").requestSubmit()')
        b.cpu_throttle(args.cpu)
        b.drain(args.warmup)   # start-up under throttling is not steady state
        lags = []
        sample_s = []
        t_end = time.time() + args.seconds
        while time.time() < t_end:
            b.drain(0.5)
            t_a = time.time()
            pi_now = get_json(f'{target.url}api/snapshot', 3)['host_ms']
            newest = b.eval('window.__navigatr.lastSnapshot ? window.__navigatr.lastSnapshot.host_ms : null')
            sample_s.append(time.time() - t_a)
            if newest is not None:
                lags.append(pi_now - newest)
        b.cpu_throttle(1)
        proxy_read, proxy_written = proxy.read_bytes, proxy.written_bytes
        server_view = b.eval('(() => { const f = window.__navigatr.feed; const d = f.diag; const id = f.hello && f.hello.client_id; '
                             'const cs = d && d.inspection && d.inspection.clients ? d.inspection.clients : []; '
                             'return cs.filter(c => c.id === id); })()')
        lags_sorted = sorted(lags)
        n = len(lags_sorted)
        p50 = lags_sorted[n // 2] if n else None
        p95 = lags_sorted[int((n - 1) * 0.95)] if n else None
        last = lags[-5:]
        m = metrics(b)
        r.metrics = {'rate_bytes_s': args.rate, 'cpu_throttle': args.cpu, 'seconds': args.seconds, 'warmup_s': args.warmup,
                     'sample_cost_s_max': max(sample_s) if sample_s else None,
                     'preview_hz': args.preview_hz, 'server_client_stats': server_view,
                     'proxy_read_bytes': proxy_read, 'proxy_written_bytes': proxy_written,
                     'lag_ms_p50': p50, 'lag_ms_p95': p95, 'lag_ms_max': max(lags) if lags else None,
                     'lag_ms_last5': last, 'seq_gaps': m['seqGaps'], 'counts': m['counts'],
                     'frameInterval': m['frameInterval'], 'publishToReceive': m['publishToReceive']}
        print(f'  staleness on the Pi clock (server now - newest state on the page): p50 {fmt(p50)} ms, '
              f'p95 {fmt(p95)} ms, max {fmt(max(lags) if lags else None)} ms; last {last}')
        third = max(1, n // 3)
        early = sum(lags[:third]) / third if n else 0
        late = sum(lags[-third:]) / third if n else 0
        r.metrics['lag_ms_first_third_mean'] = early
        r.metrics['lag_ms_last_third_mean'] = late
        r.check(n >= args.seconds, f'{n} staleness samples (slowest sample took {max(sample_s) if sample_s else 0:.2f} s)')
        r.check(p95 is not None and p95 < args.max_lag, f'p95 staleness {fmt(p95)} ms under {args.max_lag} ms')
        r.check(late <= early + 500, f'no growing backlog (first third {early:.0f} ms, last third {late:.0f} ms)')
        errs = page_errors(b)
        r.check(not errs, f'no page errors {errs[:3]}')
    finally:
        b.close()
        proxy.stop()
    return r


def check_drop(r, b, drop, contract2=True):
    """A socket drop with the same server process behind it: the page
    reconnects, keeps the session, snaps the drawn pose, refills the trail
    from history and gets a clock estimate again."""
    m0 = metrics(b)
    s0 = status(b)
    drop()
    b.wait_for('window.__navigatr.metrics().connectionEpoch > %d' % m0['connectionEpoch'], timeout=15)
    b.wait_for('window.__navigatr.connection === "live"', timeout=15)
    b.drain(1.5)
    s1 = status(b)
    m1 = metrics(b)
    r.check(s1['session'] == s0['session'], 'same session after a socket drop')
    r.check(m1['smoother']['snaps'] > m0['smoother']['snaps'], 'the drawn pose snapped on reconnect')
    r.check(int(s1['trail']) > 20, f'trail rebuilt from history: {s1["trail"]}')
    # an inspect/1 runtime answers no pings: then only the ids are checked
    r.check((m1['clock']['offset'] is not None or not contract2) and m1['clockPings']['next_id'] > m0['clockPings']['next_id'],
            f'clock re-estimated on the new connection, ping ids kept counting ({m0["clockPings"]} -> {m1["clockPings"]})')


def scenario_reconnect(args, target, out):
    r = Result('reconnect')
    proxy = KillProxy(target.port) if target.kind == 'binary' else None
    b = open_page(out, f'http://127.0.0.1:{proxy.port}/' if proxy else target.url, 'reconnect')
    try:
        wait_live(b)
        b.drain(2)
        m0 = metrics(b)
        s0 = status(b)
        if target.kind == 'stub':
            check_drop(r, b, lambda: target.control('kill'))
            m1 = metrics(b)
            # session reset (same process)
            target.control('reset')
            b.wait_for('window.__navigatr.metrics().sessionEpoch > %d' % m1['sessionEpoch'], timeout=10)
            b.drain(1.5)
            r.check(b.eval('window.__navigatr.feed.session.reset_count') >= 1, 'reset_count followed')
            # a new process: new session id, graphs start over
            target.control('restart')
            b.wait_for('document.getElementById("status").dataset.session !== "%s"' % s0['session'], timeout=20)
            b.wait_for('window.__navigatr.connection === "live"', timeout=15)
            b.drain(1.0)
            r.check(True, 'new session id adopted after a server restart')
            stats = target.control('stats')
            sent_types = set()
            for c in stats['clients'] + stats['dropped']:
                sent_types |= set(c['received'])
            r.check(sent_types <= {'ping', 'subscribe', 'history', 'preview'}, f'the page only sent {sorted(sent_types)}')
            reps = unique_pings(stats)
            r.check(all(n == 0 for _, n in reps), f'no ping id repeated on any connection {reps}')
        else:
            # the same process behind a dropped socket (the relay cuts it)
            check_drop(r, b, proxy.kill, target.contract2)
            target.restart()
            b.wait_for('document.getElementById("status").dataset.session !== "%s"' % s0['session'], timeout=40)
            b.wait_for('window.__navigatr.connection === "live"', timeout=20)
            b.drain(2)
            s1 = status(b)
            r.check(s1['session'] != s0['session'], 'new session after a runtime restart')
            r.check(int(s1['trail']) > 0, f'trail restarted: {s1["trail"]}')
        errs = page_errors(b)
        r.check(not errs, f'no page errors {errs[:3]}')
    finally:
        b.close()
        if proxy:
            proxy.stop()
    return r


def scenario_nowebgl(args, target, out):
    r = Result('nowebgl')
    b = Browser(log_dir=out, extra_args=['--disable-3d-apis'])
    try:
        b.navigate(target.url, wait_load=True, timeout=40)
        wait_live(b)
        b.drain(3)
        st = status(b)
        r.check(st['webgl'] == '0', 'WebGL off: the 3D view reports disabled')
        r.check(b.eval('!document.getElementById("view-note").hidden'), 'the page says the 3D view is disabled')
        r.check(int(st['snapshots']) > 10, f'documents still flow ({st["snapshots"]} states)')
        b.eval('document.querySelector("#dock-tabs button[data-tab=graphs]").click()')
        b.drain(1.5)
        vals = b.eval('[...document.querySelectorAll(".legend-value")].map(e => e.textContent)')
        r.check(any(v not in ('n/a', '') for v in vals), 'graphs still work')
        b.eval('document.querySelector("#dock-tabs button[data-tab=diag]").click()')
        # replay without WebGL: the state, readout and timeline still follow the playhead
        fixture = os.path.join(out, 'fixture_capture.zip')
        with open(fixture, 'wb') as f:
            f.write(cf.synthetic())
        b.call('DOM.enable')
        b.set_file('#cap-file', fixture)
        b.wait_for('document.getElementById("status").dataset.mode === "replay"', timeout=15)
        b.eval(f'window.__navigatr.seek({cf.T0_US / 1000 + 3000})')
        b.drain(0.5)
        rs = b.eval('window.__navigatr.replayState()')
        k = int(3.0 * cf.RATE_HZ + 1e-9)
        x, _, _, _, _ = cf.robot_at(k / cf.RATE_HZ)
        r.check(rs['robot'] is not None and abs(rs['robot']['x'] - x) < 1e-5, f'replayed robot state without WebGL: {rs["robot"]}')
        ro = b.eval('document.getElementById("readout").textContent')
        r.check('no robot state' not in ro and 'row at Pi' in ro, f'replay readout: {ro}')
        s0 = b.eval('document.getElementById("replay-scrub").value')
        b.eval('document.getElementById("replay-play").click()')
        b.drain(0.8)
        b.eval('document.getElementById("replay-play").click()')
        s1 = b.eval('document.getElementById("replay-scrub").value')
        r.check(s1 != s0, f'the timeline moves while playing ({s0} -> {s1})')
        b.eval('document.getElementById("replay-exit").click()')
        errs = page_errors(b)
        r.check(not errs, f'no page errors {errs[:3]}')
    finally:
        b.close()
    return r


def check_replay(r, b, expect_fixture):
    r.check(b.eval('document.getElementById("status").dataset.mode') == 'replay', 'page in replay mode')
    r.check(b.eval('!document.getElementById("replay-bar").hidden'), 'REPLAY banner shown')
    r.check(b.eval('document.getElementById("cap-start").disabled'), 'capture start disabled in replay')
    r.check(b.eval('[...document.getElementById("preview-form").elements].every(e => e.disabled)'), 'preview budget disabled in replay')
    rs = b.eval('window.__navigatr.replayState()')
    r.check(rs and not rs['warnings'], f'bundle read without warnings {rs and rs["warnings"]}')
    sent0 = metrics(b)['sent']
    if expect_fixture:
        for t_s in (1.0, 3.337, 7.5):
            t_ms = cf.T0_US / 1000 + t_s * 1000
            b.eval(f'window.__navigatr.seek({t_ms})')
            b.drain(0.3)
            rs = b.eval('window.__navigatr.replayState()')
            k = int(t_s * cf.RATE_HZ + 1e-9)
            x, y, _, _, _ = cf.robot_at(k / cf.RATE_HZ)
            ok = rs['robot'] and abs(rs['robot']['x'] - x) < 1e-5 and abs(rs['robot']['y'] - y) < 1e-5
            r.check(ok, f'pose at {t_s} s matches the recording ({rs["robot"]} vs {x:.6f}, {y:.6f})')
        seg = b.eval('window.__navigatr.replayState().robot.segment')
        r.check(seg == 1, f'after the epoch change the robot is in segment 2 (index {seg})')
        trail = int(status(b)['trail'])
        max_rows = int((7.5 - cf.EPOCH_S) * cf.RATE_HZ) + 1
        r.check(0 < trail <= max_rows, f'trail stays inside the segment ({trail} <= {max_rows})')
    t0 = b.eval('window.__navigatr.replayState().t')
    b.eval('document.getElementById("replay-speed").value = "4"; document.getElementById("replay-speed").dispatchEvent(new Event("change"))')
    b.eval('document.getElementById("replay-play").click()')
    b.drain(1.0)
    t1 = b.eval('window.__navigatr.replayState().t')
    r.check(t1 - t0 > 1500, f'playing at 4x advanced {t1 - t0:.0f} ms in about 1 s')
    b.eval('document.getElementById("replay-play").click()')
    # graphs follow the timeline
    b.eval('document.querySelector("#dock-tabs button[data-tab=graphs]").click()')
    b.drain(1.0)
    vals = b.eval('[...document.querySelectorAll(".legend-value")].map(e => e.textContent)')
    r.check(any(v not in ('n/a', '') for v in vals), 'graphs show recorded values at the playhead')
    b.eval('document.querySelector("#dock-tabs button[data-tab=diag]").click()')
    # event marker click seeks
    rs = b.eval('window.__navigatr.replayState()')
    if rs['events']:
        box = b.eval('(() => { const r = document.getElementById("replay-marks").getBoundingClientRect(); return [r.left, r.top, r.width, r.height]; })()')
        ev_t = b.eval('(() => { const m = window.__navigatr.replayState(); return m; })()')
        b.eval(f'window.__navigatr.seek({ev_t["end"]})')
        b.click(box[0] + 2, box[1] + 4)
        b.drain(0.3)
        r.check(abs(b.eval('window.__navigatr.replayState().t') - ev_t['start']) < 0.2 * (ev_t['end'] - ev_t['start']), 'timeline click seeks')
    sent1 = metrics(b)['sent']
    extra = {k: sent1.get(k, 0) - sent0.get(k, 0) for k in sent1 if sent1.get(k, 0) != sent0.get(k, 0)}
    # pings and subscriptions (what the visible tabs want) are not commands;
    # anything else would be
    r.check(set(extra) <= {'ping', 'subscribe'}, f'only pings and subscriptions sent while replaying ({extra})')


def check_replay_stale(r, b, out):
    """A hole in robot_state rows, and rows repeating an old pose with a
    growing source_age_ms, must replay as stale (faded robot, label, badge,
    row time and age in the readout), and as fresh outside that span."""
    for variant in ('hole', 'frozen'):
        p = os.path.join(out, f'fixture_{variant}.zip')
        with open(p, 'wb') as f:
            f.write(cf.synthetic(variant))
        b.set_file('#cap-file', p)
        b.wait_for(f'/fixture_{variant}/.test(document.getElementById("replay-label").textContent)', timeout=15)
        b.eval(f'window.__navigatr.seek({cf.T0_US / 1000 + 6500})')
        b.drain(0.4)
        rs = b.eval('window.__navigatr.replayState()')['robot']
        bd = badges(b)
        ro = b.eval('document.getElementById("readout").textContent')
        label = metrics(b)['robotLabel']
        span = rs['since_ms'] if variant == 'hole' else rs['age_ms']
        r.check(status(b)['replayPose'] == 'stale' and 'stale in recording' in bd.get('replay-pose', '') and
                label == 'robot (pose stale in recording)' and span > 2000 and 'before the playhead' in ro,
                f'{variant}: replayed as stale (row {rs["row_t"]}, age {rs["age_ms"]}, {rs["since_ms"]} ms before the playhead; '
                f'badge {bd.get("replay-pose")!r}; label {label!r}; readout {ro!r})')
        b.eval(f'window.__navigatr.seek({cf.T0_US / 1000 + 3000})')
        b.drain(0.4)
        bd = badges(b)
        r.check(status(b)['replayPose'] == 'fresh' and 'replay-pose' not in bd and metrics(b)['robotLabel'] == 'robot',
                f'{variant}: fresh outside the span ({status(b)["replayPose"]}, {metrics(b)["robotLabel"]!r})')


def scenario_replay(args, target, out):
    r = Result('replay')
    fixture = os.path.join(out, 'fixture_capture.zip')
    with open(fixture, 'wb') as f:
        f.write(cf.synthetic())
    b = open_page(out, target.url, 'replay')
    try:
        wait_live(b)
        b.call('DOM.enable')
        b.set_file('#cap-file', fixture)
        b.wait_for('document.getElementById("status").dataset.mode === "replay"', timeout=15)
        b.drain(0.5)
        check_replay(r, b, expect_fixture=True)
        b.screenshot(os.path.join(out, 'replay.png'))
        check_replay_stale(r, b, out)
        b.eval('document.getElementById("replay-exit").click()')
        b.wait_for('document.getElementById("status").dataset.mode === "live"', timeout=5)
        b.drain(2.0)
        r.check(int(status(b)['trail']) > 0, 'live trail back after leaving replay')
        errs = page_errors(b)
        r.check(not errs, f'no page errors {errs[:3]}')
    finally:
        b.close()
    return r


def scenario_record(args, target, out):
    r = Result('record')
    b = open_page(out, target.url, 'record')
    try:
        wait_live(b)
        st = get_json(target.url + 'api/capture/status')
        if not r.check(st.get('available'), f'capture available on this runtime ({st})'):
            return r
        b.eval('document.querySelector("#dock-tabs button[data-tab=capture]").click()')
        b.wait_for('!document.getElementById("cap-start").disabled', timeout=10)
        # a capture started elsewhere (another viewer, an auto trigger) must
        # show from the pushed 'capture' messages, not the fallback poll
        b.drain(3.0)   # past the open-tab fetch, so only a push can be quick
        ext = get_json(target.url + 'api/capture/start?pre_s=1&post_s=1', method='POST')
        dt = poll(b, 'document.getElementById("cap-start").disabled', 4.0)
        r.check(dt is not None and dt < 1.5, f'a capture started elsewhere disables start in {fmt(dt)} s (pushed status)')
        b.wait_for(f'!!document.querySelector("button[data-capture=\\"{ext["id"]}\\"]")', timeout=40)
        b.wait_for('!document.getElementById("cap-start").disabled', timeout=10)
        b.drain(0.3)
        last = b.eval('document.getElementById("cap-last").textContent')
        r.check(ext['id'] in last and 'ready' in last, f'the last outcome line names it: {last!r}')
        b.eval('document.getElementById("cap-pre").value = "2"; document.getElementById("cap-post").value = "3"')
        before = {c['id'] for c in st.get('captures', [])} | {ext['id']}
        b.eval('document.getElementById("cap-start").click()')
        b.wait_for('/started/.test(document.getElementById("cap-msg").textContent)', timeout=10)
        cid = re.search(r'capture (\S+) started', b.eval('document.getElementById("cap-msg").textContent')).group(1)
        r.check(cid not in before, f'capture {cid} started from the page')
        # a second start while recording is refused (busy)
        try:
            get_json(target.url + 'api/capture/start?pre_s=1&post_s=1', method='POST')
            r.check(False, 'second start refused while recording')
        except urllib.error.HTTPError as e:
            r.check(e.code == 409, f'second start refused while recording (HTTP {e.code})')
        b.wait_for(f'!!document.querySelector("button[data-capture=\\"{cid}\\"]")', timeout=40)
        b.eval(f'document.querySelector("button[data-capture=\\"{cid}\\"]").click()')
        b.wait_for('document.getElementById("status").dataset.mode === "replay"', timeout=20)
        b.drain(0.5)
        rs = b.eval('window.__navigatr.replayState()')
        r.check(rs['robot'] is not None, 'replayed capture has robot state')
        dur = (rs['end'] - rs['start']) / 1000
        r.check(3.0 <= dur <= 6.5, f'capture spans about pre + post ({dur:.2f} s)')
        check_replay(r, b, expect_fixture=False)
        b.eval('document.getElementById("replay-exit").click()')
        errs = page_errors(b)
        r.check(not errs, f'no page errors {errs[:3]}')
    finally:
        b.close()
    return r


class BlockedCaptureTarget(BinaryTarget):
    """The runtime with <Capture directory> under a plain file: every
    capture's file write fails, and the bundle stays in memory."""

    config_name = 'viewer_test_config_blocked_capture.xml'   # never the shared target's file

    def __init__(self, out, binary, config, blocker):
        self.blocker = blocker
        super().__init__(out, binary, config)

    def write_config(self, config):
        path = super().write_config(config)
        text = open(path, encoding='utf-8').read()
        d = (self.blocker + '/sub').replace('\\', '/')
        text = text.replace('<Robot ', f'<Capture directory="{d}"/>\n    <Robot ', 1)
        with open(path, 'w', encoding='utf-8') as f:
            f.write(text)
        return path


def start_capture_from_page(b):
    b.wait_for('!document.getElementById("cap-start").disabled', timeout=20)
    b.eval('document.getElementById("cap-msg").textContent = ""; document.getElementById("cap-start").click()')
    b.wait_for('/started/.test(document.getElementById("cap-msg").textContent)', timeout=10)
    return re.search(r'capture (\S+) started', b.eval('document.getElementById("cap-msg").textContent')).group(1)


def capture_row(b, cid):
    return b.eval(f'(() => {{ const rows = [...document.getElementById("cap-list").rows]; '
                  f'const r = rows.find(x => x.cells[0] && x.cells[0].textContent === "{cid}"); '
                  f'return r ? [...r.cells].map(c => [c.textContent, c.className]) : null; }})()')


def scenario_capfail(args, target, out):
    """Failed file writes and failed bundles are reported, not hidden."""
    r = Result('capfail')
    own = None
    if target.kind == 'binary':
        blocker = os.path.join(out, 'capture_dir_blocker')
        with open(blocker, 'w') as f:
            f.write('a file where the capture directory should be\n')
        own = BlockedCaptureTarget(out, args.binary, args.config, blocker)
    url = own.url if own else target.url
    b = open_page(out, url, 'capfail')
    try:
        wait_live(b)
        b.eval('document.querySelector("#dock-tabs button[data-tab=capture]").click()')
        b.wait_for('!document.getElementById("cap-start").disabled', timeout=10)
        b.eval('document.getElementById("cap-pre").value = "1"; document.getElementById("cap-post").value = "1"')
        if target.kind == 'stub':
            target.control('capfail', mode='file')
        cid = start_capture_from_page(b)
        b.wait_for(f'!!document.querySelector("button[data-capture=\\"{cid}\\"]")', timeout=40)
        b.drain(0.4)
        row = capture_row(b, cid)
        file_cell = row[6] if row else ['', '']
        r.check('WRITE FAILED' in file_cell[0] and 'state-fault' in file_cell[1],
                f'the list says the file write failed: {file_cell}')
        last = b.eval('[document.getElementById("cap-last").textContent, document.getElementById("cap-last").className]')
        r.check(cid in last[0] and 'file write failed' in last[0] and 'memory only' in last[0] and 'state-fault' in last[1],
                f'the last outcome line says so: {last}')
        msg = b.eval('document.getElementById("cap-msg").textContent')
        r.check('file write failed' in msg, f'announced in the message line: {msg!r}')
        server = [c for c in get_json(url + 'api/capture/status')['captures'] if c['id'] == cid]
        r.check(server and server[0].get('file_error'), f'the server reported file_error: {server and server[0].get("file_error")}')
        if target.kind == 'stub':
            target.control('capfail', mode='bundle')
            cid2 = start_capture_from_page(b)
            b.wait_for(f'document.getElementById("cap-last").textContent.includes("{cid2} FAILED")', timeout=40)
            b.drain(0.4)
            r.check(capture_row(b, cid2) is None, 'a failed bundle leaves no list row')
            msg = b.eval('document.getElementById("cap-msg").textContent')
            r.check(f'{cid2} FAILED' in msg, f'and the failure is announced: {msg!r}')
        else:
            print('  note: a failed bundle cannot be forced on the runtime from outside; the stub run covers it')
        errs = page_errors(b)
        r.check(not errs, f'no page errors {errs[:3]}')
    finally:
        b.close()
        if own:
            own.stop()
    return r


def scenario_paths(args, target, out):
    """Page-level paths driven through the stub's controls: a state pause
    (stale look and badge, kept across keepalives), a teleport (jump snap),
    an odometry epoch (trail restart plus a history request), valid poses
    with no measurement time, a function not ready, an unmapped clock, and
    two viewers each reading their own per-client statistics."""
    r = Result('paths')
    if target.kind != 'stub':
        r.check(True, 'paths needs the stub controls: skipped on --binary')
        return r
    b = open_page(out, target.url, 'paths')
    b2 = None
    try:
        wait_live(b)
        b.drain(2)
        r.check(status(b)['pose'] == 'fresh', f'fresh pose while states flow ({status(b)["pose"]})')
        # pause: stale within 250 ms plus a panel period, and it stays stale
        # through a keepalive that repeats the old pose with its grown age
        target.control('pause', ms=2600)
        dt = poll(b, 'document.getElementById("status").dataset.pose === "stale"', 1.5, 0.02)
        r.check(dt is not None and dt < 0.5, f"stale within {fmt(dt)} s of the pause (rule: 250 ms plus one 100 ms panel pass)")
        bd = badges(b)
        r.check('pose-stale' in bd and metrics(b)['robotLabel'] == 'robot (pose stale)',
                f'stale badge {bd.get("pose-stale")!r} and label {metrics(b)["robotLabel"]!r}')
        k0 = metrics(b)['counts'].get('state', 0)
        b.drain(1.6)
        k1 = metrics(b)['counts'].get('state', 0)
        r.check(k1 > k0 and status(b)['pose'] == 'stale',
                f'still stale after a keepalive arrived ({k1 - k0} states during the pause)')
        dt = poll(b, 'document.getElementById("status").dataset.pose === "fresh"', 3)
        r.check(dt is not None, 'fresh again once states resume')
        # jump: snapped, not blended
        b.drain(1.0)
        s0 = metrics(b)['smoother']
        target.control('jump')
        dt = poll(b, 'window.__navigatr.metrics().smoother.snaps > %d' % s0['snaps'], 2)
        s1 = metrics(b)['smoother']
        r.check(dt is not None and s1['last'] == 'jump', f'a 1 m teleport snaps (reason jump): {s0} -> {s1}')
        # epoch: the trail restarts and history is asked once
        h0 = metrics(b)['sent'].get('history', 0)
        tr0 = int(status(b)['trail'])
        target.control('epoch')
        b.drain(0.2)
        tr1 = int(status(b)['trail'])
        b.drain(1.5)
        h1 = metrics(b)['sent'].get('history', 0)
        tr2 = int(status(b)['trail'])
        r.check(tr1 < tr0 and tr2 >= tr1 and h1 == h0 + 1, f'epoch: trail {tr0} -> {tr1} -> {tr2}, history requests {h0} -> {h1}')
        # valid poses with no measurement time (a configured placement)
        target.control('unmeasured', on=1)
        dt = poll(b, 'document.getElementById("status").dataset.pose === "unmeasured"', 2)
        b.drain(0.2)
        bd = badges(b)
        ro = b.eval('document.getElementById("readout").textContent')
        r.check(dt is not None and 'not measured' in bd.get('pose-unmeasured', '') and 'pose-stale' not in bd and
                metrics(b)['robotLabel'] == 'robot (pose not measured yet)' and 'pose not measured yet' in ro,
                f'unmeasured: badge {bd.get("pose-unmeasured")!r}, label {metrics(b)["robotLabel"]!r}, readout {ro!r}')
        # measured on a source clock not mapped to the Pi clock: age unknown,
        # which is not the same as never measured
        target.control('unmeasured', on=1, source=1)
        dt = poll(b, '(() => { const e = document.querySelector("#badges [data-badge=pose-unmeasured]"); '
                     'return e && !e.hidden && e.textContent.includes("age unknown"); })()', 2)
        b.drain(0.2)
        bd = badges(b)
        r.check(dt is not None and status(b)['pose'] == 'unmeasured' and metrics(b)['robotLabel'] == 'robot (pose age unknown)',
                f'unmapped clock: badge {bd.get("pose-unmeasured")!r}, label {metrics(b)["robotLabel"]!r}')
        target.control('unmeasured', on=0)
        r.check(poll(b, 'document.getElementById("status").dataset.pose === "fresh"', 2) is not None, 'fresh once measured')
        # a localization function not ready, and an unmapped clock
        target.control('ready', on=0)
        target.control('clock', mapped=0)
        dt = poll(b, '(() => { const s = [...document.querySelectorAll("#badges .badge")].filter(e => !e.hidden).map(e => e.dataset.badge); '
                     'return s.includes("loc-not-ready") && s.includes("clock-unmapped"); })()', 3)
        bd = badges(b)
        r.check(dt is not None and 'waiting for stillness' in bd.get('loc-not-ready', ''),
                f'not ready and clock badges: {bd.get("loc-not-ready")!r}, {bd.get("clock-unmapped")!r}')
        target.control('ready', on=1)
        target.control('clock', mapped=1)
        dt = poll(b, '(() => { const s = [...document.querySelectorAll("#badges .badge")].filter(e => !e.hidden).map(e => e.dataset.badge); '
                     'return !s.includes("loc-not-ready") && !s.includes("clock-unmapped"); })()', 3)
        r.check(dt is not None, 'both badges clear when the diag says so')
        # two viewers: each latency panel reads its own client entry
        b2 = open_page(out, target.url, 'paths2')
        wait_live(b2)
        ids = []
        for page in (b, b2):
            page.eval('document.querySelector("#dock-tabs button[data-tab=latency]").click()')
        for page in (b, b2):
            page.drain(1.0)
            got = page.eval('(() => { const f = window.__navigatr.feed; const id = f.hello.client_id; '
                            'const rows = [...document.getElementById("lat-clients").rows].map(x => x.cells[0].textContent); '
                            'return [id, rows.filter(t => t.includes("(this page)"))]; })()')
            ids.append(got)
        r.check(ids[0][0] != ids[1][0] and all(len(x[1]) == 1 and x[1][0].startswith(str(x[0]) + ' ') for x in ids),
                f'each page marks its own client row: {ids}')
        hdr = b.eval('[...document.getElementById("lat-channels").rows[0].cells].map(c => c.textContent)')
        r.check(any('built for other clients' in h for h in hdr) and any('replaced for this page' in h for h in hdr),
                f'channel columns say what a seq gap is: {hdr}')
        errs = page_errors(b) + page_errors(b2)
        r.check(not errs, f'no page errors {errs[:3]}')
    finally:
        b.close()
        if b2:
            b2.close()
    return r


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('scenarios', nargs='*', default=['all'])
    ap.add_argument('--stub', action='store_true', help='test against tools/viewer/stub_server.py')
    ap.add_argument('--binary', help='a built navigatr to test against')
    ap.add_argument('--config', default=os.path.join(NAV, 'config', 'demo', 'synthetic_field_demo.xml'))
    ap.add_argument('--out', help='artifacts directory (default: a temp dir)')
    ap.add_argument('--rate', type=float, default=100000, help='slow: server-to-browser bytes per second')
    ap.add_argument('--send-buffer-kb', type=int, default=None,
                    help='--binary: set <Inspection send_buffer_kb> (the server socket send buffer)')
    ap.add_argument('--preview-hz', type=float, default=None,
                    help='slow: preview budget the page asks for (default: the server default)')
    ap.add_argument('--cpu', type=float, default=4, help='slow: CPU throttling factor')
    ap.add_argument('--seconds', type=float, default=20)
    ap.add_argument('--warmup', type=float, default=5, help='slow: seconds under throttling before sampling')
    ap.add_argument('--max-lag', type=float, default=2000, help='slow: p95 staleness bound, ms')
    ap.add_argument('--query', default='', help='slow: query string for the page, e.g. nobudget or perf')
    ap.add_argument('--json', help='write results as JSON here')
    args = ap.parse_args()
    names = args.scenarios
    if names == ['all']:
        names = ['unit', 'live', 'nowebgl', 'slow', 'reconnect', 'replay', 'record', 'capfail', 'paths']
    out = args.out or tempfile.mkdtemp(prefix='spectagatr-tests-')
    os.makedirs(out, exist_ok=True)
    print(f'artifacts in {out}')
    results = []
    target = None
    try:
        for name in names:
            print(f'== {name}', flush=True)
            if name == 'unit':
                results.append(scenario_unit(args, out))
                continue
            if target is None:
                if args.binary:
                    extra = f'send_buffer_kb="{args.send_buffer_kb}"' if args.send_buffer_kb is not None else ''
                    target = BinaryTarget(out, args.binary, args.config, extra)
                else:
                    target = StubTarget(out)
                print(f'target {target.kind} at {target.url} ({"inspect/2" if target.contract2 else "inspect/1"})')
            fn = {'live': scenario_live, 'nowebgl': scenario_nowebgl, 'slow': scenario_slow, 'reconnect': scenario_reconnect,
                  'replay': scenario_replay, 'record': scenario_record, 'capfail': scenario_capfail,
                  'paths': scenario_paths}[name]
            try:
                results.append(fn(args, target, out))
            except Exception as e:  # a scenario error is a failure, not a crash of the run
                res = Result(name)
                res.check(False, f'scenario raised {type(e).__name__}: {e}')
                results.append(res)
    finally:
        if target:
            target.stop()
    print('== summary')
    for res in results:
        passed = sum(1 for c, _ in res.checks if c)
        print(f'{"PASS" if res.ok else "FAIL"} {res.name}: {passed}/{len(res.checks)} checks')
        if res.metrics:
            print('     ' + json.dumps(res.metrics)[:1200])
    if args.json:
        with open(args.json, 'w') as f:
            json.dump([{'name': x.name, 'ok': bool(x.ok), 'checks': x.checks, 'metrics': x.metrics} for x in results], f, indent=1)
    sys.exit(0 if results and all(x.ok for x in results) else 1)


if __name__ == '__main__':
    main()
