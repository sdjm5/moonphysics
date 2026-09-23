// Scene construction and pointer interaction.

///|
/// Spawn a ball; returns its particle index or -1 when the pool is full.
pub fn World::spawn_ball(
  self : World,
  x : Double,
  y : Double,
  r : Double,
) -> Int {
  self.alloc_particle(x, y, r, r * r * 0.03, KIND_BALL, self.n)
}

///|
/// Spawn a rigid box (its four corners joined by four edge sticks and two
/// hidden diagonal braces); corners are consecutive particles in draw order
/// tl, tr, br, bl. Returns the index of the first corner or -1.
pub fn World::spawn_box(
  self : World,
  x : Double,
  y : Double,
  size : Double,
) -> Int {
  let h = size * 0.5
  let g = self.n
  let tl = self.alloc_particle(
    x - h,
    y - h,
    h * 0.42,
    h * h * 0.03,
    KIND_BOX,
    g,
  )
  if tl < 0 {
    return -1
  }
  let tr = self.alloc_particle(
    x + h,
    y - h,
    h * 0.42,
    h * h * 0.03,
    KIND_BOX,
    g,
  )
  let br = self.alloc_particle(
    x + h,
    y + h,
    h * 0.42,
    h * h * 0.03,
    KIND_BOX,
    g,
  )
  let bl = self.alloc_particle(
    x - h,
    y + h,
    h * 0.42,
    h * h * 0.03,
    KIND_BOX,
    g,
  )
  if tr < 0 || br < 0 || bl < 0 {
    return -1
  }
  ignore(self.add_constraint(tl, tr, false, true))
  ignore(self.add_constraint(tr, br, false, true))
  ignore(self.add_constraint(br, bl, false, true))
  ignore(self.add_constraint(bl, tl, false, true))
  // diagonal braces make the quad rigid
  ignore(self.add_constraint(tl, br, false, false))
  ignore(self.add_constraint(tr, bl, false, false))
  tl
}

///|
/// Spawn a rope hanging from a pinned end at (x, y). Returns the head index.
pub fn World::spawn_rope(
  self : World,
  x : Double,
  y : Double,
  segments : Int,
  seg_len : Double,
) -> Int {
  let g = self.n
  let head = self.alloc_particle(x, y, 4.0, 0.0, KIND_ROPE, g) // pinned
  if head < 0 {
    return -1
  }
  let mut prev = head
  for k = 0; k < segments; k = k + 1 {
    let i = self.alloc_particle(
      x,
      y + seg_len * (k + 1).to_double(),
      4.0,
      0.5,
      KIND_ROPE,
      g,
    )
    if i < 0 {
      break
    }
    ignore(self.add_constraint(prev, i, false, true))
    prev = i
  }
  head
}

///|
/// Spawn a cloth grid at (x, y); the two top corners are pinned so it hangs
/// like a curtain. All links are tearable — grab and rip it. Returns the
/// first particle index or -1.
pub fn World::spawn_cloth(
  self : World,
  x : Double,
  y : Double,
  cols : Int,
  rows : Int,
  spacing : Double,
) -> Int {
  let g = self.n
  let mut first = -1
  // index map for this spawn: rows*cols particles in row-major order
  let idx : FixedArray[Int] = FixedArray::make(rows * cols, -1)
  for r = 0; r < rows; r = r + 1 {
    for c = 0; c < cols; c = c + 1 {
      let pinned = r == 0 && (c == 0 || c == cols - 1)
      let i = self.alloc_particle(
        x + c.to_double() * spacing,
        y + r.to_double() * spacing,
        3.0,
        if pinned {
          0.0
        } else {
          0.25
        },
        KIND_CLOTH,
        g,
      )
      if i < 0 {
        return if first < 0 { -1 } else { first }
      }
      idx[r * cols + c] = i
      if first < 0 {
        first = i
      }
    }
  }
  for r = 0; r < rows; r = r + 1 {
    for c = 0; c < cols; c = c + 1 {
      let i = idx[r * cols + c]
      if c < cols - 1 {
        ignore(self.add_constraint(i, idx[r * cols + c + 1], true, true))
      }
      if r < rows - 1 {
        ignore(self.add_constraint(i, idx[(r + 1) * cols + c], true, true))
      }
    }
  }
  first
}

///|
/// Grab the nearest grabbable particle within `radius` of (x, y).
/// Returns the particle index or -1.
pub fn World::grab(
  self : World,
  x : Double,
  y : Double,
  radius : Double,
) -> Int {
  let mut best = -1
  let mut best_d2 = radius * radius
  for i = 0; i < self.n; i = i + 1 {
    if self.kind[i] == KIND_FREE {
      continue
    }
    let dx = self.px[i] - x
    let dy = self.py[i] - y
    let d2 = dx * dx + dy * dy
    if d2 < best_d2 {
      best_d2 = d2
      best = i
    }
  }
  self.grabbed = best
  best
}

///|
/// Move the grabbed particle to (x, y), imparting the pointer's velocity so
/// the fling on release is natural.
pub fn World::drag(self : World, x : Double, y : Double) -> Unit {
  let i = self.grabbed
  if i < 0 {
    return
  }
  self.ppx[i] = self.px[i]
  self.ppy[i] = self.py[i]
  self.px[i] = x
  self.py[i] = y
}

///|
/// Release the grabbed particle.
pub fn World::release(self : World) -> Unit {
  self.grabbed = -1
}

///|
/// Radial impulse: every particle within `radius` of (x, y) gets an outward
/// velocity kick that fades linearly to zero at the rim.
pub fn World::explode(
  self : World,
  x : Double,
  y : Double,
  radius : Double,
  power : Double,
) -> Unit {
  for i = 0; i < self.n; i = i + 1 {
    if self.imass[i] <= 0.0 {
      continue
    }
    let dx = self.px[i] - x
    let dy = self.py[i] - y
    let d = (dx * dx + dy * dy).sqrt()
    if d > radius || d < 0.000001 {
      continue
    }
    let falloff = 1.0 - d / radius
    let kick = power * falloff
    self.ppx[i] = self.ppx[i] - dx / d * kick
    self.ppy[i] = self.ppy[i] - dy / d * kick
  }
}

///|
/// Remove everything from the scene.
pub fn World::clear_scene(self : World) -> Unit {
  self.n = 0
  self.m = 0
  self.grabbed = -1
  self.steps = 0
  self.tears = 0
}

///|
/// Number of live particles (all spawned particles are live; the pool only
/// grows until clear).
pub fn World::particle_count(self : World) -> Int {
  self.n
}

///|
/// Number of active (un-torn) constraints.
pub fn World::constraint_count(self : World) -> Int {
  let mut k = 0
  for c = 0; c < self.m; c = c + 1 {
    if self.cactive[c] {
      k = k + 1
    }
  }
  k
}

///|
/// Highest particle index currently in use + 1 — the snapshot/render span.
pub fn World::span(self : World) -> Int {
  self.n
}
