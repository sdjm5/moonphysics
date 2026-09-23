// End-to-end test of the moonphysics wasm kernel.
// Run after `moon build --target wasm`:
//   cp _build/wasm/debug/build/src/kernel/kernel.wasm web/physics.wasm
//   node web/test-kernel.mjs
import { readFileSync } from 'node:fs';

const SNAP = 64 * 1024 * 1024;
const EDGE = 65 * 1024 * 1024;
let passed = 0;
const TOTAL = 10;
const check = (name, ok) => {
  if (ok) passed++;
  console.log(`${ok ? 'ok  ' : 'FAIL'} ${name}`);
};

const bytes = readFileSync(new URL('./physics.wasm', import.meta.url));
const mod = await WebAssembly.instantiate(bytes, {});
const E = mod.instance.exports;

check('exports present', ['moon_init', 'spawn_ball', 'spawn_box', 'spawn_rope', 'spawn_cloth',
  'grab', 'drag', 'release', 'explode', 'clear_scene', 'step', 'counts', 'snapshot_out',
  'snapshot_in', 'set_gravity'].every((k) => E[k] !== undefined));

E.moon_init(1200, 700);

// spawn: balls grid + box + rope + cloth
for (let row = 0; row < 4; row++)
  for (let col = 0; col < 6; col++) E.spawn_ball(200 + col * 60, 80 + row * 60, 16);
E.spawn_box(800, 300, 80);
E.spawn_rope(950, 50, 10, 25);
E.spawn_cloth(650, 60, 6, 5, 24);
const total = 24 + 4 + 11 + 30;
check(`all spawns landed (${E.counts() & 0xffff} == ${total})`, (E.counts() & 0xffff) === total);

// settle: everything stays inside bounds with no NaN
for (let s = 0; s < 240; s++) E.step();
E.snapshot_out(SNAP, EDGE);
const mem = E.memory.buffer;
const n = new Uint32Array(mem, SNAP, 1)[0];
const f64 = new Float64Array(mem, SNAP + 8, n * 6);
let sane = true;
for (let i = 0; i < n; i++) {
  const x = f64[i * 6], y = f64[i * 6 + 1];
  if (x !== x || y !== y) sane = false;
  if (x < -1 || x > 1201 || y < -1 || y > 701) sane = false;
}
check(`240-step settle: no NaN, all inside bounds (${n} particles)`, sane && n === total);

// grab / drag / release moves a particle and leaves a fling velocity
E.clear_scene();
E.set_gravity(0);
E.spawn_ball(500, 500, 20);
const g = E.grab(505, 505, 50);
E.drag(540, 500);
E.release();
E.snapshot_out(SNAP, EDGE);
const fx = new Float64Array(mem, SNAP + 8, 6);
check('grab/drag/release moves particle to pointer', Math.abs(fx[0] - 540) < 0.001);
check('fling velocity preserved (vx = 40)', Math.abs(fx[0] - fx[3] - 40) < 0.001);
check('grab returned the ball index', g === 0);

// snapshot round trip: out -> in restores count and state
E.clear_scene();
E.set_gravity(1300);
for (let i = 0; i < 100; i++) E.spawn_ball(100 + (i % 20) * 40, 60 + ((i / 20) | 0) * 50, 15);
E.snapshot_out(SNAP, EDGE);
const before = E.counts() & 0xffff;
const restored = E.snapshot_in(SNAP);
check(`snapshot_in restores ${restored} == ${before}`, restored === before && (E.counts() & 0xffff) === before);

// determinism: restore S0, run the same 50 steps, results must match S1
// byte for byte (the f64 snapshot makes the round trip exact)
E.snapshot_out(SNAP, EDGE);
const s0 = new Uint8Array(mem, SNAP, 8 + (E.counts() & 0xffff) * 48).slice();
for (let s = 0; s < 50; s++) E.step();
E.snapshot_out(SNAP, EDGE);
const ref = new Uint8Array(mem, SNAP, s0.length).slice();
new Uint8Array(mem, SNAP, s0.length).set(s0); // put S0 back for snapshot_in
E.snapshot_in(SNAP);
for (let s = 0; s < 50; s++) E.step();
E.snapshot_out(SNAP, EDGE);
const again = new Uint8Array(mem, SNAP, ref.length);
let same = true;
for (let i = 0; i < ref.length; i++) if (ref[i] !== again[i]) { same = false; break; }
check('simulation is deterministic across snapshot round trip', same);

// explosion kicks particles outward
E.clear_scene();
E.set_gravity(0);
E.spawn_ball(625, 350, 10); // 25 px right of center
E.explode(600, 350, 500, 20);
E.snapshot_out(SNAP, EDGE);
const ev = new Float64Array(mem, SNAP + 8, 6);
check('explosion impulse at falloff 0.95 (vx = 19)', Math.abs(ev[0] - ev[3] - 19) < 0.01);

// throughput: 300-ball scene, must clear a 60 fps frame budget comfortably
E.clear_scene();
for (let i = 0; i < 300; i++) E.spawn_ball(60 + (i % 30) * 36, 50 + ((i / 30) | 0) * 44, 14);
for (let s = 0; s < 60; s++) E.step();
const t0 = performance.now();
for (let s = 0; s < 300; s++) E.step();
const t1 = performance.now();
const per = (t1 - t0) / 300;
console.log(`     300 particles: ${per.toFixed(3)} ms/step (60 fps budget 16.6 ms)`);
check('step cost under 16.6 ms (real-time capable)', per < 16.6);

console.log(`\n${passed}/${TOTAL} checks passed`);
process.exit(passed === TOTAL ? 0 : 1);
