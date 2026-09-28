"""cdp.py - a small Chrome DevTools Protocol client, Python standard library only.

Launches headless Chrome or Edge with a throwaway profile, opens one page and
talks CDP over a minimal WebSocket client (RFC 6455 text frames, client side
masking). Enough for the viewer tests: navigate, evaluate, throttle CPU and
network, set a file input, dispatch mouse input, screenshot.

Nothing is installed; runs on Windows (Git Bash or cmd) and Linux.
"""

import base64
import json
import os
import shutil
import socket
import struct
import subprocess
import sys
import tempfile
import time
import urllib.request


def find_browser():
    env = os.environ.get('CHROME_BIN')
    candidates = [env] if env else []
    candidates += [
        r'C:\Program Files\Google\Chrome\Application\chrome.exe',
        r'C:\Program Files (x86)\Google\Chrome\Application\chrome.exe',
        r'C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe',
        r'C:\Program Files\Microsoft\Edge\Application\msedge.exe',
    ]
    for name in ('google-chrome', 'chromium', 'chromium-browser', 'microsoft-edge'):
        p = shutil.which(name)
        if p:
            candidates.append(p)
    for c in candidates:
        if c and os.path.exists(c):
            return c
    return None


class WebSocket:
    """Client side WebSocket over a plain socket (ws:// only)."""

    def __init__(self, url, timeout=10.0):
        assert url.startswith('ws://'), url
        rest = url[5:]
        hostport, _, path = rest.partition('/')
        host, _, port = hostport.partition(':')
        self.sock = socket.create_connection((host, int(port or 80)), timeout=timeout)
        key = base64.b64encode(os.urandom(16)).decode()
        req = (f'GET /{path} HTTP/1.1\r\nHost: {hostport}\r\nUpgrade: websocket\r\n'
               f'Connection: Upgrade\r\nSec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n')
        self.sock.sendall(req.encode())
        buf = b''
        while b'\r\n\r\n' not in buf:
            chunk = self.sock.recv(4096)
            if not chunk:
                raise ConnectionError('websocket handshake: connection closed')
            buf += chunk
        head, _, self.pending = buf.partition(b'\r\n\r\n')
        if b' 101 ' not in head.split(b'\r\n')[0]:
            raise ConnectionError('websocket handshake failed: ' + head.decode(errors='replace'))

    def _read(self, n):
        while len(self.pending) < n:
            chunk = self.sock.recv(65536)
            if not chunk:
                raise ConnectionError('websocket closed')
            self.pending += chunk
        out, self.pending = self.pending[:n], self.pending[n:]
        return out

    def send(self, text, opcode=1):
        data = text.encode() if isinstance(text, str) else text
        head = bytes([0x80 | opcode])
        n = len(data)
        if n < 126:
            head += bytes([0x80 | n])
        elif n < 65536:
            head += bytes([0x80 | 126]) + struct.pack('>H', n)
        else:
            head += bytes([0x80 | 127]) + struct.pack('>Q', n)
        mask = os.urandom(4)
        body = bytes(b ^ mask[i & 3] for i, b in enumerate(data))
        self.sock.sendall(head + mask + body)

    def recv(self, timeout=None):
        """One complete message as str (text) or bytes (binary); None on timeout."""
        self.sock.settimeout(timeout)
        parts = []
        first_op = None
        try:
            while True:
                b0, b1 = self._read(2)
                op = b0 & 0x0F
                n = b1 & 0x7F
                if n == 126:
                    n = struct.unpack('>H', self._read(2))[0]
                elif n == 127:
                    n = struct.unpack('>Q', self._read(8))[0]
                mask = self._read(4) if b1 & 0x80 else None
                data = self._read(n)
                if mask:
                    data = bytes(b ^ mask[i & 3] for i, b in enumerate(data))
                if op == 9:
                    self.send(data, opcode=10)
                    continue
                if op == 10:
                    continue
                if op == 8:
                    raise ConnectionError('websocket closed by peer')
                if op != 0:
                    first_op = op
                parts.append(data)
                if b0 & 0x80:
                    break
        except socket.timeout:
            return None
        data = b''.join(parts)
        return data.decode() if first_op == 1 else data

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass


class Browser:
    """A headless browser process with one page target."""

    def __init__(self, width=1400, height=900, extra_args=None, log_dir=None):
        exe = find_browser()
        if not exe:
            raise RuntimeError('no Chrome/Edge found (set CHROME_BIN)')
        self.exe = exe
        self.profile = tempfile.mkdtemp(prefix='spectagatr-cdp-')
        with socket.socket() as s:
            s.bind(('127.0.0.1', 0))
            self.port = s.getsockname()[1]
        args = [exe, '--headless=new', f'--remote-debugging-port={self.port}', f'--user-data-dir={self.profile}',
                '--no-first-run', '--no-default-browser-check', '--disable-extensions',
                '--disable-background-networking', '--disable-breakpad', '--disable-renderer-backgrounding',
                '--disable-background-timer-throttling', '--disable-backgrounding-occluded-windows',
                '--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--disable-gpu',
                f'--window-size={width},{height}', 'about:blank']
        args += extra_args or []
        err = open(os.path.join(log_dir, 'browser.err.txt'), 'w') if log_dir else subprocess.DEVNULL
        self.proc = subprocess.Popen(args, stdout=subprocess.DEVNULL, stderr=err)
        deadline = time.time() + 20
        targets = None
        while time.time() < deadline:
            try:
                with urllib.request.urlopen(f'http://127.0.0.1:{self.port}/json/list', timeout=1) as r:
                    targets = json.loads(r.read())
                    pages = [t for t in targets if t.get('type') == 'page']
                    if pages:
                        break
            except OSError:
                pass
            time.sleep(0.2)
        else:
            self.close()
            raise RuntimeError('browser did not open its debugging port')
        page = [t for t in targets if t.get('type') == 'page'][0]
        self.ws = WebSocket(page['webSocketDebuggerUrl'])
        self.next_id = 1
        self.events = []
        self.console = []
        self.exceptions = []
        self.call('Page.enable')
        self.call('Runtime.enable')
        self.call('Log.enable')

    def _pump(self, msg):
        m = json.loads(msg)
        if 'id' not in m:
            method = m.get('method')
            if method == 'Runtime.consoleAPICalled':
                args = m['params'].get('args', [])
                self.console.append(' '.join(str(a.get('value', a.get('description', ''))) for a in args))
            elif method == 'Log.entryAdded':
                e = m['params'].get('entry', {})
                if e.get('level') in ('error', 'warning'):
                    self.console.append(f"{e.get('level')}: {e.get('text')} {e.get('url', '')}:{e.get('lineNumber', '')}")
            elif method == 'Runtime.exceptionThrown':
                d = m['params'].get('exceptionDetails', {})
                ex = d.get('exception', {})
                self.exceptions.append(ex.get('description') or d.get('text', 'exception'))
            else:
                self.events.append(m)
                if len(self.events) > 2000:
                    del self.events[:1000]
        return m

    def call(self, method, params=None, timeout=30.0):
        mid = self.next_id
        self.next_id += 1
        self.ws.send(json.dumps({'id': mid, 'method': method, 'params': params or {}}))
        deadline = time.time() + timeout
        while time.time() < deadline:
            msg = self.ws.recv(timeout=max(0.05, deadline - time.time()))
            if msg is None:
                continue
            m = self._pump(msg)
            if m.get('id') == mid:
                if 'error' in m:
                    raise RuntimeError(f'{method}: {m["error"]}')
                return m.get('result', {})
        raise TimeoutError(method)

    def drain(self, seconds):
        """Keeps reading events for a while (lets the page run)."""
        deadline = time.time() + seconds
        while time.time() < deadline:
            msg = self.ws.recv(timeout=max(0.01, deadline - time.time()))
            if msg is not None:
                self._pump(msg)

    def navigate(self, url, wait_load=True, timeout=30.0):
        self.call('Page.navigate', {'url': url})
        if not wait_load:
            return
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.eval('document.readyState') == 'complete':
                return
            self.drain(0.1)
        raise TimeoutError('page load')

    def eval(self, expr, await_promise=False, timeout=30.0):
        r = self.call('Runtime.evaluate', {'expression': expr, 'returnByValue': True,
                                           'awaitPromise': await_promise}, timeout=timeout)
        if 'exceptionDetails' in r:
            d = r['exceptionDetails']
            raise RuntimeError('eval failed: ' + str(d.get('exception', {}).get('description') or d.get('text')))
        return r.get('result', {}).get('value')

    def wait_for(self, expr, timeout=20.0, interval=0.1):
        deadline = time.time() + timeout
        last = None
        while time.time() < deadline:
            try:
                last = self.eval(expr)
            except RuntimeError as e:
                last = str(e)
            if last is True or (last and last is not False and not isinstance(last, str)):
                return last
            self.drain(interval)
        raise TimeoutError(f'waited {timeout} s for: {expr} (last {last!r})')

    def cpu_throttle(self, rate):
        self.call('Emulation.setCPUThrottlingRate', {'rate': rate})

    def set_file(self, selector, path):
        doc = self.call('DOM.getDocument')
        node = self.call('DOM.querySelector', {'nodeId': doc['root']['nodeId'], 'selector': selector})
        self.call('DOM.setFileInputFiles', {'nodeId': node['nodeId'], 'files': [os.path.abspath(path)]})

    def mouse(self, kind, x, y, buttons=0, button='none'):
        p = {'type': kind, 'x': x, 'y': y, 'buttons': buttons, 'button': button}
        if kind in ('mousePressed', 'mouseReleased'):
            p['clickCount'] = 1   # without it no click event fires
        self.call('Input.dispatchMouseEvent', p)

    def click(self, x, y):
        self.mouse('mouseMoved', x, y)
        self.mouse('mousePressed', x, y, buttons=1, button='left')
        self.mouse('mouseReleased', x, y, buttons=0, button='left')

    def drag(self, x0, y0, x1, y1, steps=30, pause=0.016):
        self.mouse('mouseMoved', x0, y0)
        self.mouse('mousePressed', x0, y0, buttons=1, button='left')
        for i in range(1, steps + 1):
            x = x0 + (x1 - x0) * i / steps
            y = y0 + (y1 - y0) * i / steps
            self.mouse('mouseMoved', x, y, buttons=1, button='left')
            self.drain(pause)
        self.mouse('mouseReleased', x1, y1, buttons=0, button='left')

    def screenshot(self, path):
        r = self.call('Page.captureScreenshot', {'format': 'png'})
        with open(path, 'wb') as f:
            f.write(base64.b64decode(r['data']))

    def close(self):
        try:
            self.ws.close()
        except Exception:
            pass
        if self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(5)
            except subprocess.TimeoutExpired:
                self.proc.kill()
        shutil.rmtree(self.profile, ignore_errors=True)


if __name__ == '__main__':
    b = Browser()
    b.navigate(sys.argv[1] if len(sys.argv) > 1 else 'about:blank')
    print(b.eval('navigator.userAgent'))
    b.close()
