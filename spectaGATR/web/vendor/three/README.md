# three.js (vendored)

- Upstream: three.js (github.com/mrdoob/three.js)
- Version: r170 (module constant `"170"` in `three.module.min.js`, header
  `Copyright 2010-2024 Three.js Authors`). `OrbitControls.js` is the matching
  `examples/jsm/controls/OrbitControls.js`; it imports `'three'`, which the
  page resolves through its import map.
- License: MIT, see `LICENSE` (kept verbatim).
- Origin in this repo: copied unchanged from
  `deprecated/pi/naviGATR/third_party/three/` (the old Pi viewer's copy).

SHA-256:

```
08fd7545d13d2c7fb65ab691530a802dafefd638596501854f267d0fb13c39e7  three.module.min.js
80efaadea4f8a636a65fb0bd08bfef62f3d93a0bb94e2e7500f23176c5c07f4e  OrbitControls.js
4c40a1ef62450b857c3b2aaf294936304cd552d965fbcd9d32d4c5bcf4ba4454  LICENSE
```

Served locally under `/vendor/three/`; the viewer loads nothing from the network.
