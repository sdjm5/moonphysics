// 2D position-based dynamics solver (Verlet integration).
//
// The scene is a pool of particles connected by distance constraints.
// Rigid bodies are particle clusters (a box is its four corners joined by
// four edge sticks and two hidden diagonals), ropes are pinned chains, cloth
// is a grid whose links can tear when overstretched. Everything is
// preallocated FixedArrays; a uniform spatial hash keeps collision detection
// O(n) so hundreds of particles solve in well under a frame budget.
//
// Verlet keeps the velocity implicit in (p - p_prev), which is what makes
// the solver stable: positional corrections automatically become velocity
// changes next step, bounces fall out of pushing positions back inside
// bounds, and flinging a grabbed particle needs no explicit velocity bookkeeping.

///|
pub const KIND_FREE : Int = 0

///|
pub const KIND_BALL : Int = 1

///|
pub const KIND_BOX : Int = 2

///|
pub const KIND_ROPE : Int = 3

///|
pub const KIND_CLOTH : Int = 4

///|
pub(all) struct World {
  // world bounds in pixels
  mut width : Double
  mut height : Double
  // tunables
  mut gravity : Double // px/s^2
  mut restitution : Double // 0..1
  mut damping : Double // velocity retention per step, 0..1
  mut friction : Double // tangential keep-factor on floor contact, 0..1
  mut iterations : Int // constraint/collision sweeps per step
  // particles
  cap : Int
  mut n : Int
  px : FixedArray[Double]
  py : FixedArray[Double]
  ppx : FixedArray[Double] // previous position (implicit velocity)
  ppy : FixedArray[Double]
  rad : FixedArray[Double]
  imass : FixedArray[Double] // inverse mass; 0 = pinned / kinematic
  kind : FixedArray[Int]
  group : FixedArray[Int] // box/rope/cloth id for rendering
  mut grabbed : Int // particle index currently held by the pointer, -1 none
  // distance constraints
  mut m : Int
  ca : FixedArray[Int]
  cb : FixedArray[Int]
  crest : FixedArray[Double]
  cactive : FixedArray[Bool]
  ctear : FixedArray[Bool] // tearable (cloth) constraints
  cvis : FixedArray[Bool] // drawn as an edge in the UI
  // uniform spatial hash
  cell : Double
  gw : Int
  gh : Int
  head : FixedArray[Int]
  nexti : FixedArray[Int]
  // stats
  mut steps : Int
  mut tears : Int
}

///|
pub const MAX_PARTICLES : Int = 2048

///|
pub const MAX_CONSTRAINTS : Int = 16384

///|
pub const DT : Double = 1.0 / 60.0

///|
pub fn World::new(width : Double, height : Double) -> World {
  let cell = 56.0
  let gw = (width / cell).to_int() + 1
  let gh = (height / cell).to_int() + 1
  {
    width,
    height,
    gravity: 1300.0,
    restitution: 0.35,
    damping: 0.9995,
    friction: 0.75,
    iterations: 8,
    cap: MAX_PARTICLES,
    n: 0,
    px: FixedArray::make(MAX_PARTICLES, 0.0),
    py: FixedArray::make(MAX_PARTICLES, 0.0),
    ppx: FixedArray::make(MAX_PARTICLES, 0.0),
    ppy: FixedArray::make(MAX_PARTICLES, 0.0),
    rad: FixedArray::make(MAX_PARTICLES, 0.0),
    imass: FixedArray::make(MAX_PARTICLES, 0.0),
    kind: FixedArray::make(MAX_PARTICLES, KIND_FREE),
    group: FixedArray::make(MAX_PARTICLES, 0),
    grabbed: -1,
    m: 0,
    ca: FixedArray::make(MAX_CONSTRAINTS, 0),
    cb: FixedArray::make(MAX_CONSTRAINTS, 0),
    crest: FixedArray::make(MAX_CONSTRAINTS, 0.0),
    cactive: FixedArray::make(MAX_CONSTRAINTS, false),
    ctear: FixedArray::make(MAX_CONSTRAINTS, false),
    cvis: FixedArray::make(MAX_CONSTRAINTS, false),
    cell,
    gw,
    gh,
    head: FixedArray::make(gw * gh, -1),
    nexti: FixedArray::make(MAX_PARTICLES, -1),
    steps: 0,
    tears: 0,
  }
}

///|
pub fn World::alloc_particle(
  self : World,
  x : Double,
  y : Double,
  r : Double,
  mass : Double,
  k : Int,
  group : Int,
) -> Int {
  if self.n >= self.cap {
    return -1
  }
  let i = self.n
  self.px[i] = x
  self.py[i] = y
  self.ppx[i] = x
  self.ppy[i] = y
  self.rad[i] = r
  self.imass[i] = if mass <= 0.0 { 0.0 } else { 1.0 / mass }
  self.kind[i] = k
  self.group[i] = group
  self.n = self.n + 1
  i
}

///|
pub fn World::add_constraint(
  self : World,
  a : Int,
  b : Int,
  tearable : Bool,
  visible : Bool,
) -> Int {
  if self.m >= MAX_CONSTRAINTS {
    return -1
  }
  let c = self.m
  let dx = self.px[b] - self.px[a]
  let dy = self.py[b] - self.py[a]
  self.ca[c] = a
  self.cb[c] = b
  self.crest[c] = (dx * dx + dy * dy).sqrt()
  self.cactive[c] = true
  self.ctear[c] = tearable
  self.cvis[c] = visible
  self.m = self.m + 1
  c
}

///|
/// Rebuild the spatial hash: one linked list per grid cell.
fn World::hash_build(self : World) -> Unit {
  FixedArray::fill(self.head, -1)
  let cs = self.cell
  for i = 0; i < self.n; i = i + 1 {
    let cx = (self.px[i] / cs).to_int()
    let cy = (self.py[i] / cs).to_int()
    let cx2 = if cx < 0 { 0 } else if cx >= self.gw { self.gw - 1 } else { cx }
    let cy2 = if cy < 0 { 0 } else if cy >= self.gh { self.gh - 1 } else { cy }
    let c = cy2 * self.gw + cx2
    self.nexti[i] = self.head[c]
    self.head[c] = i
  }
}

///|
/// Integrate one fixed timestep with semi-implicit Verlet.
fn World::integrate(self : World) -> Unit {
  let dt2 = DT * DT
  let g = self.gravity
  let damp = self.damping
  for i = 0; i < self.n; i = i + 1 {
    if i == self.grabbed {
      // kinematic while grabbed: the pointer drives it
      self.ppx[i] = self.px[i]
      self.ppy[i] = self.py[i]
      continue
    }
    if self.imass[i] <= 0.0 {
      // pinned: never moves, keep prev in sync so it carries no velocity
      self.ppx[i] = self.px[i]
      self.ppy[i] = self.py[i]
      continue
    }
    let vx = (self.px[i] - self.ppx[i]) * damp
    let vy = (self.py[i] - self.ppy[i]) * damp
    self.ppx[i] = self.px[i]
    self.ppy[i] = self.py[i]
    self.px[i] = self.px[i] + vx
    self.py[i] = self.py[i] + vy + g * dt2
  }
}

///|
/// Solve one distance constraint (mass-weighted positional correction).
fn World::solve_constraint(self : World, c : Int) -> Unit {
  if !self.cactive[c] {
    return
  }
  let a = self.ca[c]
  let b = self.cb[c]
  let dx = self.px[b] - self.px[a]
  let dy = self.py[b] - self.py[a]
  let d2 = dx * dx + dy * dy
  let rest = self.crest[c]
  if d2 < 0.000000000001 {
    return
  }
  let d = d2.sqrt()
  // cloth tears when overstretched
  if self.ctear[c] && d > rest * 2.4 {
    self.cactive[c] = false
    self.tears = self.tears + 1
    return
  }
  let wa = self.imass[a]
  let wb = self.imass[b]
  let wsum = wa + wb
  if wsum < 0.000000000001 {
    return
  }
  let diff = (d - rest) / d / wsum * 0.85
  self.px[a] = self.px[a] + dx * diff * wa
  self.py[a] = self.py[a] + dy * diff * wa
  self.px[b] = self.px[b] - dx * diff * wb
  self.py[b] = self.py[b] - dy * diff * wb
}

///|
/// Ball-ball positional collision with implicit bounce and floor-style
/// tangential friction. Pairs come from the spatial hash; each pair is
/// handled once (j > i).
fn World::collide_particles(self : World) -> Unit {
  let e = self.restitution
  let cs = self.cell
  for i = 0; i < self.n; i = i + 1 {
    if self.imass[i] <= 0.0 && i != self.grabbed {
      continue
    }
    let cx = (self.px[i] / cs).to_int()
    let cy = (self.py[i] / cs).to_int()
    let y0 = if cy - 1 < 0 { 0 } else { cy - 1 }
    let y1 = if cy + 1 > self.gh - 1 { self.gh - 1 } else { cy + 1 }
    let x0 = if cx - 1 < 0 { 0 } else { cx - 1 }
    let x1 = if cx + 1 > self.gw - 1 { self.gw - 1 } else { cx + 1 }
    for gy = y0; gy <= y1; gy = gy + 1 {
      for gx = x0; gx <= x1; gx = gx + 1 {
        let mut j = self.head[gy * self.gw + gx]
        while j >= 0 {
          if j > i {
            self.separate(i, j, e)
          }
          j = self.nexti[j]
        }
      }
    }
  }
}

///|
fn World::separate(self : World, i : Int, j : Int, e : Double) -> Unit {
  let dx = self.px[j] - self.px[i]
  let dy = self.py[j] - self.py[i]
  let rsum = self.rad[i] + self.rad[j]
  let d2 = dx * dx + dy * dy
  if d2 >= rsum * rsum || d2 < 0.000000000001 {
    return
  }
  let d = d2.sqrt()
  let nx = dx / d
  let ny = dy / d
  let overlap = rsum - d
  let wa = if i == self.grabbed { 0.0 } else { self.imass[i] }
  let wb = if j == self.grabbed { 0.0 } else { self.imass[j] }
  let wsum = wa + wb
  if wsum < 0.000000000001 {
    return
  }
  // positional separation (mass weighted), with a little softness
  let push = overlap / wsum * 0.8
  self.px[i] = self.px[i] - nx * push * wa
  self.py[i] = self.py[i] - ny * push * wa
  self.px[j] = self.px[j] + nx * push * wb
  self.py[j] = self.py[j] + ny * push * wb
  // implicit bounce: pull the prev-position along the normal so next
  // step's velocity reflects with restitution e
  let vi_n = (self.px[i] - self.ppx[i]) * nx + (self.py[i] - self.ppy[i]) * ny
  let vj_n = (self.px[j] - self.ppx[j]) * nx + (self.py[j] - self.ppy[j]) * ny
  if vi_n - vj_n > 0.0 {
    // approaching
    let bounce = e * 0.5
    self.ppx[i] = self.ppx[i] + nx * vi_n * bounce
    self.ppy[i] = self.ppy[i] + ny * vi_n * bounce
    self.ppx[j] = self.ppx[j] - nx * vj_n * bounce
    self.ppy[j] = self.ppy[j] - ny * vj_n * bounce
  }
}

///|
/// Clamp inside the world bounds; bounce with restitution, scrub tangential
/// velocity by the friction factor (piles need floor friction to settle).
fn World::walls(self : World) -> Unit {
  let e = self.restitution
  let fr = self.friction
  for i = 0; i < self.n; i = i + 1 {
    let r = self.rad[i]
    // floor
    if self.py[i] > self.height - r {
      let vy = self.py[i] - self.ppy[i]
      let vx = self.px[i] - self.ppx[i]
      self.py[i] = self.height - r
      self.ppy[i] = self.py[i] + vy * e
      self.ppx[i] = self.px[i] - vx * fr
    }
    // ceiling
    if self.py[i] < r {
      let vy = self.py[i] - self.ppy[i]
      self.py[i] = r
      self.ppy[i] = self.py[i] + vy * e
    }
    // left
    if self.px[i] < r {
      let vx = self.px[i] - self.ppx[i]
      let vy = self.py[i] - self.ppy[i]
      self.px[i] = r
      self.ppx[i] = self.px[i] + vx * e
      self.ppy[i] = self.py[i] - vy * fr
    }
    // right
    if self.px[i] > self.width - r {
      let vx = self.px[i] - self.ppx[i]
      let vy = self.py[i] - self.ppy[i]
      self.px[i] = self.width - r
      self.ppx[i] = self.px[i] + vx * e
      self.ppy[i] = self.py[i] - vy * fr
    }
  }
}

///|
/// Advance the simulation one fixed timestep (1/60 s):
/// integrate, then alternating constraint/collision sweeps, then walls.
pub fn World::step(self : World) -> Unit {
  self.integrate()
  let it = if self.iterations < 1 { 1 } else { self.iterations }
  for k = 0; k < it; k = k + 1 {
    for c = 0; c < self.m; c = c + 1 {
      self.solve_constraint(c)
    }
    if k == 0 {
      self.hash_build()
    }
    self.collide_particles()
  }
  self.walls()
  self.steps = self.steps + 1
}

///|
/// Total kinetic-ish energy proxy (sum of squared speeds) — test helper for
/// "the pile settles" assertions.
pub fn World::energy(self : World) -> Double {
  let mut s = 0.0
  for i = 0; i < self.n; i = i + 1 {
    let vx = self.px[i] - self.ppx[i]
    let vy = self.py[i] - self.ppy[i]
    s = s + vx * vx + vy * vy
  }
  s
}
