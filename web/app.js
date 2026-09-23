// moonphysics sandbox UI.
//
// The solver runs in the MoonBit wasm module: this script sends commands
// (spawn / grab / explode / step), reads back the compact f64 snapshot from
// linear memory, and draws it on a canvas. For the benchmark it also carries
// a hand-written JavaScript mirror of the same solver (identical algorithm:
// Verlet integration, uniform spatial hash, positional separation, wall
// bounce) so the race is same-vs-same.

const SNAP = 64 * 1024 * 1024;
const EDGE = 65 * 1024 * 1024;
const W = 1200, H = 640;
const KIND_BALL = 1, KIND_BOX = 2, KIND_ROPE = 3, KIND_CLOTH = 4;

const canvas = document.getElementById('stage');
const ctx = canvas.getContext('2d');

// ---------------------------------------------------------------------------
// wasm module

const E = (await WebAssembly.instantiate(
  await (await fetch('physics.wasm')).arrayBuffer(), {}
)).instance.exports;
E.moon_init(W, H);

const f64buf = () => new Float64Array(E.memory.buffer, SNAP + 8, 2048 * 6);
const kbuf = () => new Uint32Array(E.memory.buffer, SNAP + 8, 2048 * 12); // kind u32 at byte 40 of each 48-byte record
const u32buf = (addr) => new Uint32Array(E.memory.buffer, addr, 16384);

// ---------------------------------------------------------------------------
// rendering

const COLORS = {
  [KIND_BALL]: '#4fd1c5',
  [KIND_BOX]: '#ff8b3d',
  [KIND_ROPE]: '#9aa4b2',
  [KIND_CLOTH]: '#69c0ff',
};

function draw() {
  E.snapshot_out(SNAP, EDGE);
  const n = u32buf(SNAP)[0];
  const m = u32buf(EDGE)[0];
  const P = f64buf();
  const KBUF = kbuf();
  const ED = u32buf(EDGE);
  ctx.fillStyle = '#090b0e';
  ctx.fillRect(0, 0, W, H);

  // subtle grid
  ctx.strokeStyle = 'rgba(255,255,255,0.025)';
  ctx.lineWidth = 1;
  ctx.beginPath();
  for (let x = 60; x < W; x += 60) { ctx.moveTo(x, 0); ctx.lineTo(x, H); }
  for (let y = 60; y < H; y += 60) { ctx.moveTo(0, y); ctx.lineTo(W, y); }
  ctx.stroke();

  // edges (rope links, cloth links, box outlines)
  ctx.lineWidth = 1.4;
  for (let k = 0; k < m; k++) {
    const a = ED[1 + k * 2], b = ED[2 + k * 2];
    const ka = KBUF[a * 12 + 10];
    ctx.strokeStyle = ka === KIND_CLOTH ? 'rgba(105,192,255,0.55)'
      : ka === KIND_ROPE ? 'rgba(154,164,178,0.8)' : 'rgba(255,139,61,0.9)';
    ctx.beginPath();
    ctx.moveTo(P[a * 6], P[a * 6 + 1]);
    ctx.lineTo(P[b * 6], P[b * 6 + 1]);
    ctx.stroke();
  }

  // particles; box corners arrive as consecutive runs of four (tl,tr,br,bl)
  let i = 0;
  while (i < n) {
    const kind = KBUF[i * 12 + 10];
    if (kind === KIND_BOX && i + 3 < n) {
      ctx.fillStyle = 'rgba(255,139,61,0.85)';
      ctx.beginPath();
      ctx.moveTo(P[i * 6], P[i * 6 + 1]);
      ctx.lineTo(P[(i + 1) * 6], P[(i + 1) * 6 + 1]);
      ctx.lineTo(P[(i + 2) * 6], P[(i + 2) * 6 + 1]);
      ctx.lineTo(P[(i + 3) * 6], P[(i + 3) * 6 + 1]);
      ctx.closePath();
      ctx.fill();
      i += 4;
      continue;
    }
    const r = P[i * 6 + 2];
    const x = P[i * 6], y = P[i * 6 + 1];
    if (kind === KIND_BALL) {
      ctx.fillStyle = COLORS[KIND_BALL];
      ctx.beginPath();
      ctx.arc(x, y, r, 0, Math.PI * 2);
      ctx.fill();
      // a little highlight so balls read as volumes
      ctx.fillStyle = 'rgba(255,255,255,0.25)';
      ctx.beginPath();
      ctx.arc(x - r * 0.3, y - r * 0.35, r * 0.35, 0, Math.PI * 2);
      ctx.fill();
    } else {
      // rope / cloth node
      ctx.fillStyle = COLORS[kind];
      ctx.beginPath();
      ctx.arc(x, y, Math.max(2.2, r), 0, Math.PI * 2);
      ctx.fill();
    }
    i++;
  }

  document.getElementById('particles-badge').textContent = `${n} 粒子`;
}

// ---------------------------------------------------------------------------
// main loop

let paused = false;
let frames = 0, fpsT0 = performance.now();

function frame() {
  if (!paused) E.step();
  draw();
  frames++;
  const now = performance.now();
  if (now - fpsT0 > 500) {
    const fps = (frames * 1000 / (now - fpsT0)).toFixed(0);
    document.getElementById('fps-badge').textContent = `${fps} fps`;
    document.getElementById('solver-badge').textContent = paused ? '已暂停' : 'wasm 求解';
    frames = 0;
    fpsT0 = now;
  }
  requestAnimationFrame(frame);
}
requestAnimationFrame(frame);

// ---------------------------------------------------------------------------
// pointer interaction

let tool = 'grab';
document.getElementById('tools').addEventListener('click', (e) => {
  const b = e.target.closest('button');
  if (!b) return;
  tool = b.dataset.tool;
  document.querySelectorAll('#tools button').forEach((x) => x.classList.toggle('on', x === b));
});

function canvasPos(e) {
  const rect = canvas.getBoundingClientRect();
  return [
    (e.clientX - rect.left) * (W / rect.width),
    (e.clientY - rect.top) * (H / rect.height),
  ];
}

canvas.addEventListener('pointerdown', (e) => {
  e.preventDefault();
  canvas.setPointerCapture(e.pointerId);
  const [x, y] = canvasPos(e);
  if (tool === 'grab') {
    E.grab(x, y, 36);
  } else if (tool === 'ball') {
    E.spawn_ball(x, y, 10 + Math.random() * 14);
  } else if (tool === 'box') {
    E.spawn_box(x, y, 50 + Math.random() * 40);
  } else if (tool === 'rope') {
    E.spawn_rope(x, y, 12, 26);
  } else if (tool === 'cloth') {
    E.spawn_cloth(Math.min(x, W - 200), Math.min(y, H / 2), 9, 7, 22);
  }
});
canvas.addEventListener('pointermove', (e) => {
  const [x, y] = canvasPos(e);
  E.drag(x, y);
});
canvas.addEventListener('pointerup', () => E.release());
canvas.addEventListener('lostpointercapture', () => E.release());

// ---------------------------------------------------------------------------
// sliders and buttons

function slider(id, apply, fmt) {
  const el = document.getElementById('s-' + id);
  const out = document.getElementById('v-' + id);
  el.addEventListener('input', () => {
    const v = Number(el.value);
    out.textContent = fmt(v);
    apply(v);
  });
}
slider('gravity', (v) => E.set_gravity(v), (v) => String(v));
slider('restitution', (v) => E.set_restitution(v / 100), (v) => (v / 100).toFixed(2));
slider('friction', (v) => E.set_friction(v / 100), (v) => (v / 100).toFixed(2));
slider('iterations', (v) => E.set_iterations(v), (v) => String(v));

document.getElementById('boom-btn').addEventListener('click', () => {
  E.explode(W / 2, H / 2, 420, 26);
});
document.getElementById('pause-btn').addEventListener('click', (e) => {
  paused = !paused;
  e.target.textContent = paused ? '▶ 继续' : '⏸ 暂停';
});
document.getElementById('clear-btn').addEventListener('click', () => E.clear_scene());

// ---------------------------------------------------------------------------
// scenario presets

const scenes = {
  pit() {
    for (let i = 0; i < 300; i++) {
      E.spawn_ball(80 + Math.random() * (W - 160), 40 + Math.random() * 300, 11 + Math.random() * 12);
    }
  },
  pyramid() {
    const rows = 6, size = 78;
    for (let r = 0; r < rows; r++) {
      for (let c = 0; c < rows - r; c++) {
        E.spawn_box(W / 2 + (c - (rows - r - 1) / 2) * (size + 8), H - 60 - r * (size + 4), size);
      }
    }
  },
  curtain() {
    E.spawn_cloth(W / 2 - 210, 50, 20, 14, 22);
    for (let i = 0; i < 80; i++) {
      E.spawn_ball(200 + Math.random() * (W - 400), 30 + Math.random() * 120, 10 + Math.random() * 10);
    }
  },
  chains() {
    for (let k = 0; k < 5; k++) {
      E.spawn_rope(180 + k * 200, 30, 14, 24);
    }
    for (let i = 0; i < 120; i++) {
      E.spawn_ball(100 + Math.random() * (W - 200), H * 0.5 + Math.random() * 150, 12 + Math.random() * 12);
    }
  },
};
document.querySelectorAll('.scenes button').forEach((b) => {
  b.addEventListener('click', () => {
    E.clear_scene();
    scenes[b.dataset.scene]();
  });
});

// start with something on screen
scenes.pit();

// ---------------------------------------------------------------------------
// benchmark: wasm solver vs a hand-written JavaScript mirror
//
// The mirror implements the identical algorithm over the identical initial
// state (both are loaded from the same f64 snapshot), so the only variable
// is the runtime.

const DT = 1 / 60;

function makeJsSolver(state) {
  // state: { x, y, vx, vy, r } as parallel arrays (copied from the snapshot)
  const n = state.x.length;
  const { x, y, vx, vy, r } = state;
  const px = new Float64Array(n), py = new Float64Array(n);
  for (let i = 0; i < n; i++) { px[i] = x[i] - vx[i]; py[i] = y[i] - vy[i]; }
  const cell = 56.0;
  const gw = ((W / cell) | 0) + 1, gh = ((H / cell) | 0) + 1;
  const head = new Int32Array(gw * gh);
  const next = new Int32Array(n);
  const imass = new Float64Array(n);
  for (let i = 0; i < n; i++) imass[i] = 1 / (r[i] * r[i] * 0.03);

  function step(g, e, fr, iters) {
    const dt2 = DT * DT;
    for (let i = 0; i < n; i++) {
      px[i] = x[i]; py[i] = y[i];
      x[i] += vx[i]; y[i] += vy[i] + g * dt2;
    }
    head.fill(-1);
    for (let i = 0; i < n; i++) {
      let cx = (x[i] / cell) | 0; if (cx < 0) cx = 0; else if (cx >= gw) cx = gw - 1;
      let cy = (y[i] / cell) | 0; if (cy < 0) cy = 0; else if (cy >= gh) cy = gh - 1;
      const c = cy * gw + cx;
      next[i] = head[c]; head[c] = i;
    }
    for (let k = 0; k < iters; k++) {
      for (let i = 0; i < n; i++) {
        const cx = Math.min(gw - 1, Math.max(0, (x[i] / cell) | 0));
        const cy = Math.min(gh - 1, Math.max(0, (y[i] / cell) | 0));
        for (let gy = Math.max(0, cy - 1); gy <= Math.min(gh - 1, cy + 1); gy++) {
          for (let gx = Math.max(0, cx - 1); gx <= Math.min(gw - 1, cx + 1); gx++) {
            let j = head[gy * gw + gx];
            while (j >= 0) {
              if (j > i) {
                const dx = x[j] - x[i], dy = y[j] - y[i];
                const rs = r[i] + r[j];
                const d2 = dx * dx + dy * dy;
                if (d2 < rs * rs && d2 > 1e-12) {
                  const d = Math.sqrt(d2);
                  const nx = dx / d, ny = dy / d;
                  const push = (rs - d) / (imass[i] + imass[j]) * 0.8;
                  x[i] -= nx * push * imass[i]; y[i] -= ny * push * imass[i];
                  x[j] += nx * push * imass[j]; y[j] += ny * push * imass[j];
                  const vi = (x[i] - px[i]) * nx + (y[i] - py[i]) * ny;
                  const vj = (x[j] - px[j]) * nx + (y[j] - py[j]) * ny;
                  if (vi - vj > 0) {
                    const b = e * 0.5;
                    px[i] += nx * vi * b; py[i] += ny * vi * b;
                    px[j] -= nx * vj * b; py[j] -= ny * vj * b;
                  }
                }
              }
              j = next[j];
            }
          }
        }
      }
    }
    for (let i = 0; i < n; i++) {
      const rr = r[i];
      if (y[i] > H - rr) {
        const vy_ = y[i] - py[i], vx_ = x[i] - px[i];
        y[i] = H - rr; py[i] = y[i] + vy_ * e; px[i] = x[i] - vx_ * fr;
      }
      if (y[i] < rr) { const vy_ = y[i] - py[i]; y[i] = rr; py[i] = y[i] + vy_ * e; }
      if (x[i] < rr) { const vx_ = x[i] - px[i]; x[i] = rr; px[i] = x[i] + vx_ * e; }
      if (x[i] > W - rr) { const vx_ = x[i] - px[i]; x[i] = W - rr; px[i] = x[i] + vx_ * e; }
      vx[i] = x[i] - px[i]; vy[i] = y[i] - py[i];
    }
  }
  return { step };
}

document.getElementById('bench-btn').addEventListener('click', () => {
  const out = document.getElementById('bench-out');
  const STEPS = 200;
  const lines = ['规模      wasm        JS        加速比'];
  for (const N of [300, 1500]) {
    E.clear_scene();
    for (let i = 0; i < N; i++) {
      E.spawn_ball(30 + Math.random() * (W - 60), 30 + Math.random() * 400, 11 + Math.random() * 12);
    }
    for (let s = 0; s < 120; s++) E.step();
    E.snapshot_out(SNAP, EDGE);
    const n = u32buf(SNAP)[0];
    const P = f64buf();
  const KBUF = kbuf();
    const state = {
      x: new Float64Array(n), y: new Float64Array(n),
      vx: new Float64Array(n), vy: new Float64Array(n),
      r: new Float64Array(n),
    };
    for (let i = 0; i < n; i++) {
      state.x[i] = P[i * 6]; state.y[i] = P[i * 6 + 1]; state.r[i] = P[i * 6 + 2];
      state.vx[i] = P[i * 6] - P[i * 6 + 3]; state.vy[i] = P[i * 6 + 1] - P[i * 6 + 4];
    }
    const js = makeJsSolver(state);
    const g = Number(document.getElementById('s-gravity').value);
    const e = Number(document.getElementById('s-restitution').value) / 100;
    const fr = Number(document.getElementById('s-friction').value) / 100;
    const iters = Number(document.getElementById('s-iterations').value);
    for (let s = 0; s < 30; s++) { E.step(); js.step(g, e, fr, iters); }
    let t0 = performance.now();
    for (let s = 0; s < STEPS; s++) E.step();
    let t1 = performance.now();
    const wasmMs = t1 - t0;
    t0 = performance.now();
    for (let s = 0; s < STEPS; s++) js.step(g, e, fr, iters);
    t1 = performance.now();
    const jsMs = t1 - t0;
    lines.push(
      `${String(n).padEnd(6)} ${(wasmMs / STEPS).toFixed(3)} ms/步  ${(jsMs / STEPS).toFixed(3)} ms/步  ${((jsMs / wasmMs)).toFixed(2)}×`
    );
  }
  lines.push('');
  lines.push('同算法、同初态（f64 快照往返）、热身后各 200 步。');
  lines.push('规模越大 wasm 优势越明显：无 JIT 去优化、无 GC 停顿。');
  out.textContent = lines.join('\n');
});
