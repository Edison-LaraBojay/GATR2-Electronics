// perf_probe.js
// Injected by navigatr_cdp_bench before the viewer's own scripts. Measures
// the page from outside: rAF frame intervals and the cost of the page's rAF
// callbacks, long tasks, WebSocket message handler cost by message type,
// receive-to-next-frame delay, DOM mutation counts, JS heap. Works on any
// viewer version because it wraps the browser APIs, not the app.
// window.__perf.reset() starts a window, window.__perf.report() returns JSON.
(() => {
  if (window.__perf) {
    return;
  }
  const now = () => performance.now();
  const P = {};
  const origRaf = window.requestAnimationFrame.bind(window);
  let frameCost = 0;
  let lastTs = -1;
  let pendingArrivals = [];

  function clear() {
    P.t0 = now();
    P.frames = [];
    P.rafCost = [];
    P.longTasks = [];
    P.msgs = {};
    P.recvToFrame = [];
    P.relLagRaw = [];
    P.mutations = 0;
    P.heap = [];
    frameCost = 0;
    lastTs = -1;
    pendingArrivals = [];
  }
  clear();
  P.sockets = 0;

  // the page's rAF callbacks, timed; a frame's total is booked at the next frame
  window.requestAnimationFrame = function (cb) {
    return origRaf(function (ts) {
      const s = now();
      try {
        cb(ts);
      } finally {
        frameCost += now() - s;
      }
    });
  };
  function loop(ts) {
    const t = now();
    if (lastTs >= 0) {
      P.frames.push(ts - lastTs);
      P.rafCost.push(frameCost);
    }
    frameCost = 0;
    lastTs = ts;
    for (const a of pendingArrivals) {
      P.recvToFrame.push(t - a);
    }
    pendingArrivals = [];
    origRaf(loop);
  }
  origRaf(loop);

  try {
    new PerformanceObserver((list) => {
      for (const e of list.getEntries()) {
        P.longTasks.push(e.duration);
      }
    }).observe({ type: 'longtask', buffered: true });
  } catch (e) {
    P.longTaskError = String(e);
  }

  try {
    new MutationObserver((records) => {
      P.mutations += records.length;
    }).observe(document, { subtree: true, childList: true, attributes: true, characterData: true });
  } catch (e) {
    P.mutationError = String(e);
  }

  setInterval(() => {
    if (performance.memory) {
      P.heap.push(performance.memory.usedJSHeapSize);
    }
  }, 500);

  const typeRe = /"type"\s*:\s*"([A-Za-z_0-9]+)"/;
  const hostRe = /"host_ms"\s*:\s*(-?[0-9.]+)/;
  function record(kind, size, cost) {
    let m = P.msgs[kind];
    if (!m) {
      m = P.msgs[kind] = { n: 0, bytes: 0, cost: [] };
    }
    m.n += 1;
    m.bytes += size;
    m.cost.push(cost);
  }
  function wrap(fn) {
    return function (ev) {
      const s = now();
      let kind = 'binary';
      let size = 0;
      const d = ev.data;
      if (typeof d === 'string') {
        size = d.length;
        const head = d.length > 256 ? d.slice(0, 256) : d;
        const m = typeRe.exec(head);
        kind = m ? m[1] : 'text';
        if (kind === 'snapshot' || kind === 'state') {
          pendingArrivals.push(s);
          const h = hostRe.exec(head);
          if (h) {
            P.relLagRaw.push(s - Number(h[1]));
          }
        }
      } else if (d) {
        size = d.byteLength || d.size || 0;
      }
      try {
        return fn.call(this, ev);
      } finally {
        record(kind, size, now() - s);
      }
    };
  }
  const OrigWS = window.WebSocket;
  const wrapped = new WeakMap();
  class PerfWebSocket extends OrigWS {
    constructor(...args) {
      super(...args);
      P.sockets += 1;
    }
    set onmessage(fn) {
      super.onmessage = typeof fn === 'function' ? wrap(fn) : fn;
    }
    get onmessage() {
      return super.onmessage;
    }
    addEventListener(type, fn, opts) {
      if (type === 'message' && typeof fn === 'function') {
        let w = wrapped.get(fn);
        if (!w) {
          w = wrap(fn);
          wrapped.set(fn, w);
        }
        return super.addEventListener(type, w, opts);
      }
      return super.addEventListener(type, fn, opts);
    }
    removeEventListener(type, fn, opts) {
      if (type === 'message' && wrapped.has(fn)) {
        return super.removeEventListener(type, wrapped.get(fn), opts);
      }
      return super.removeEventListener(type, fn, opts);
    }
  }
  window.WebSocket = PerfWebSocket;

  function summary(values) {
    const v = values.slice().sort((a, b) => a - b);
    const n = v.length;
    if (n === 0) {
      return { n: 0 };
    }
    const q = (p) => v[Math.min(n - 1, Math.max(0, Math.ceil(p * n) - 1))];
    let sum = 0;
    for (const x of v) {
      sum += x;
    }
    const r3 = (x) => Math.round(x * 1000) / 1000;
    return { n, min: r3(v[0]), p50: r3(q(0.5)), p95: r3(q(0.95)), p99: r3(q(0.99)), max: r3(v[n - 1]), mean: r3(sum / n), sum: r3(sum) };
  }

  let renderer = null;
  function webglRenderer() {
    if (renderer !== null) {
      return renderer;
    }
    try {
      const c = document.createElement('canvas');
      const gl = c.getContext('webgl2') || c.getContext('webgl');
      const ext = gl && gl.getExtension('WEBGL_debug_renderer_info');
      renderer = ext ? gl.getParameter(ext.UNMASKED_RENDERER_WEBGL) : gl ? gl.getParameter(gl.RENDERER) : 'no webgl';
    } catch (e) {
      renderer = 'error ' + String(e);
    }
    return renderer;
  }

  P.reset = () => {
    clear();
    return true;
  };

  P.report = () => {
    const elapsed = (now() - P.t0) / 1000;
    const msgs = {};
    let totalCost = 0;
    let totalN = 0;
    for (const k of Object.keys(P.msgs)) {
      const m = P.msgs[k];
      const s = summary(m.cost);
      msgs[k] = { n: m.n, per_s: m.n / elapsed, bytes_s: m.bytes / elapsed, mean_bytes: m.bytes / m.n, handler_ms: s };
      totalCost += s.sum || 0;
      totalN += m.n;
    }
    const minLag = P.relLagRaw.length ? Math.min(...P.relLagRaw) : 0;
    const status = document.querySelector('#status');
    const ds = {};
    if (status) {
      for (const k of Object.keys(status.dataset)) {
        ds[k] = status.dataset[k];
      }
    }
    let nav = null;
    try {
      nav = JSON.stringify(window.__navigatr, (k, v) => (typeof v === 'function' ? undefined : v));
      if (nav && nav.length > 4000) {
        nav = nav.slice(0, 4000) + '...';
      }
    } catch (e) {
      nav = 'unserializable: ' + String(e);
    }
    const frames = summary(P.frames);
    const over = (t) => P.frames.filter((x) => x > t).length;
    return JSON.stringify({
      elapsed_s: elapsed,
      frames: { interval_ms: frames, fps: P.frames.length / elapsed, over_20ms: over(20), over_33ms: over(33.4), over_50ms: over(50), over_100ms: over(100) },
      raf_callback_ms: summary(P.rafCost),
      long_tasks: { count: P.longTasks.length, ms: summary(P.longTasks) },
      messages: msgs,
      message_handler_total_ms_per_s: totalCost / elapsed,
      message_count: totalN,
      receive_to_next_frame_ms: summary(P.recvToFrame),
      relative_lag_ms: summary(P.relLagRaw.map((x) => x - minLag)),
      dom_mutation_records_per_s: P.mutations / elapsed,
      dom_nodes: document.getElementsByTagName('*').length,
      heap_bytes: summary(P.heap),
      sockets: P.sockets,
      webgl_renderer: webglRenderer(),
      device_pixel_ratio: window.devicePixelRatio,
      viewport: [window.innerWidth, window.innerHeight],
      status_dataset: ds,
      navigatr_hook: nav,
      long_task_error: P.longTaskError || null,
    });
  };

  window.__perf = P;
})();
