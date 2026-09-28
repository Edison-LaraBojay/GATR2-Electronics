"""viewer_tests.py - browser tests of the spectaGATR viewer (headless Chrome/Edge over CDP).

Python standard library only: no pip, no global installs. Runs on Windows (Git
Bash, cmd) and Linux. Every scenario prints its measurements and PASS/FAIL.

Scenarios
    unit       the pure modules in a real browser (tools/viewer/unit.html),
               including replay of a fixture bundle against closed-form truth
    live       the page against a server: live feed, 3D, trail, graphs,
               instrumentation (inspect/2), latency panel, an orbit drag
    nowebgl    the same page with WebGL disabled: documents still flow
    slow       the page behind a bandwidth-limited TCP proxy with CPU
               throttling; staleness is measured on the Pi clock only
               (GET /api/snapshot host_ms minus the page's newest state host_ms)
    reconnect  socket drop, session reset and server restart
    replay     a fixture bundle loaded through the file input: REPLAY banner,
               live-only controls off, pose at seek times, play speed, event
               seek, graphs, and nothing sent to the server while replaying
    record     record on the server (capture UI), then replay that capture

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
        path = os.path.join(self.out, 'viewer_test_config.xml')
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


# --- helpers on the page ---

STATUS = 'JSON.stringify(Object.assign({}, document.getElementById("status").dataset))'


def status(b):
    return json.loads(b.eval(STATUS))


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
    expected = {'points': [], 'epoch_t_ms': cf.T0_US / 1000 + cf.EPOCH_S * 1000, 'place_t_ms': cf.T0_US / 1000 + cf.PLACE_S * 1000}
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
                   f'&expected=/__out/fixture_expected.json')
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


def scenario_reconnect(args, target, out):
    r = Result('reconnect')
    b = open_page(out, target.url, 'reconnect')
    try:
        wait_live(b)
        b.drain(2)
        m0 = metrics(b)
        s0 = status(b)
        if target.kind == 'stub':
            target.control('kill')
            b.wait_for('window.__navigatr.metrics().connectionEpoch > %d' % m0['connectionEpoch'], timeout=15)
            b.wait_for('window.__navigatr.connection === "live"', timeout=15)
            b.drain(1.5)
            s1 = status(b)
            m1 = metrics(b)
            r.check(s1['session'] == s0['session'], 'same session after a socket drop')
            r.check(m1['smoother']['snaps'] > m0['smoother']['snaps'], 'the drawn pose snapped on reconnect')
            r.check(int(s1['trail']) > 20, f'trail rebuilt from history: {s1["trail"]}')
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
        else:
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
        b.eval('document.getElementById("cap-pre").value = "2"; document.getElementById("cap-post").value = "3"')
        before = {c['id'] for c in st.get('captures', [])}
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
        names = ['unit', 'live', 'nowebgl', 'slow', 'reconnect', 'replay', 'record']
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
                  'replay': scenario_replay, 'record': scenario_record}[name]
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
