// Physics unit tests. The engine is fully deterministic, so closed-form
// expectations (free fall, constraint convergence, explosion impulse) are
// asserted exactly; emergent behaviors (piles, stacking) get bounded
// stability assertions.

///|
fn dist(w : World, a : Int, b : Int) -> Double {
  let dx = w.px[b] - w.px[a]
  let dy = w.py[b] - w.py[a]
  (dx * dx + dy * dy).sqrt()
}

///|
test "free fall matches the closed-form Verlet sum" {
  let w = World::new(1000.0, 20000.0)
  w.damping = 1.0
  w.gravity = 1000.0
  w.restitution = 0.0
  let y0 = 100.0
  ignore(w.spawn_ball(500.0, y0, 10.0))
  let n = 60
  for k = 0; k < n; k = k + 1 {
    w.step()
  }
  // after n Verlet steps from rest, the per-step displacement grows 1, 2,
  // 3, ... (velocity is linear in time), so y = y0 + g*dt^2 * n(n+1)/2
  let expected = y0 +
    1000.0 * (DT * DT) * n.to_double() * (n + 1).to_double() / 2.0
  assert_true((w.py[0] - expected).abs() < 0.001)
}

///|
test "distance constraint converges to rest length" {
  let w = World::new(1000.0, 1000.0)
  w.gravity = 0.0
  let a = w.alloc_particle(400.0, 500.0, 1.0, 1.0, KIND_BALL, 0)
  let b = w.alloc_particle(600.0, 500.0, 1.0, 1.0, KIND_BALL, 0)
  ignore(w.add_constraint(a, b, false, true))
  // stretch the pair far past its rest length, then let the solver pull back
  w.px[b] = 800.0
  w.py[b] = 500.0
  for k = 0; k < 30; k = k + 1 {
    w.step()
  }
  assert_true((dist(w, a, b) - 200.0).abs() < 3.0)
}

///|
test "rigid box keeps its diagonals while tumbling" {
  let w = World::new(1200.0, 900.0)
  let tl = w.spawn_box(600.0, 200.0, 80.0)
  let d0 = dist(w, tl, tl + 2) // tl -> br diagonal
  // toss it: kick the whole body
  w.explode(600.0, 200.0, 400.0, 14.0)
  for k = 0; k < 360; k = k + 1 {
    w.step()
  }
  let d1 = dist(w, tl, tl + 2)
  assert_true((d1 - d0).abs() / d0 < 0.05)
  for i = 0; i < 4; i = i + 1 {
    assert_true(w.px[tl + i] == w.px[tl + i]) // no NaN
    assert_true(w.py[tl + i] > 0.0 && w.py[tl + i] < 900.0)
  }
}

///|
test "wall bounce honors restitution" {
  let w = World::new(1000.0, 1000.0)
  w.damping = 1.0
  w.restitution = 1.0
  w.friction = 1.0
  ignore(w.spawn_ball(500.0, 300.0, 10.0))
  // fall ~690 px to the floor
  for k = 0; k < 90; k = k + 1 {
    w.step()
  }
  let mut apex_before = 1000.0
  let mut apex_after = 1000.0
  let mut seen_floor = false
  for k = 0; k < 180; k = k + 1 {
    w.step()
    if !seen_floor && w.py[0] >= 989.0 {
      seen_floor = true
      apex_before = 989.0
    }
    if seen_floor {
      if w.py[0] < apex_after {
        apex_after = w.py[0]
      }
    } else if w.py[0] < apex_before {
      apex_before = w.py[0]
    }
  }
  // with e=1 the rebound apex should come back to roughly the drop height
  assert_true(seen_floor)
  assert_true(apex_after < 500.0)
  assert_true(apex_after > 250.0)
}

///|
test "overlapping balls are pushed apart" {
  let w = World::new(1000.0, 1000.0)
  w.gravity = 0.0
  let a = w.spawn_ball(500.0, 500.0, 20.0)
  let b = w.spawn_ball(530.0, 500.0, 20.0) // 30 apart, radii sum 40
  for k = 0; k < 20; k = k + 1 {
    w.step()
  }
  assert_true(dist(w, a, b) >= 39.0)
}

///|
test "explosion injects the expected outward impulse" {
  let w = World::new(2000.0, 2000.0)
  w.gravity = 0.0
  // 25 px off center: falloff = 1 - 25/500 = 0.95 → kick = 19
  let i = w.spawn_ball(1025.0, 1000.0, 10.0)
  w.explode(1000.0, 1000.0, 500.0, 20.0)
  let vx = w.px[i] - w.ppx[i]
  assert_true((vx - 19.0).abs() < 0.001)
  // at the rim the falloff reaches zero (and beyond it, no kick at all)
  let j = w.spawn_ball(1500.0, 1000.0, 10.0)
  w.explode(1000.0, 1000.0, 500.0, 20.0)
  let vx2 = w.px[j] - w.ppx[j]
  assert_true(vx2.abs() < 0.001)
}

///|
test "grab and drag move the particle; release leaves the fling" {
  let w = World::new(1000.0, 1000.0)
  w.gravity = 0.0
  ignore(w.spawn_ball(500.0, 500.0, 20.0))
  let g = w.grab(505.0, 505.0, 50.0)
  assert_eq(g, 0)
  w.drag(520.0, 500.0)
  assert_true((w.px[0] - 520.0).abs() < 0.001)
  w.release()
  assert_eq(w.grabbed, -1)
  // implied velocity after release is the drag delta
  assert_true((w.px[0] - w.ppx[0] - 20.0).abs() < 0.001)
}

///|
test "cloth tears when overstretched" {
  let w = World::new(1200.0, 900.0)
  let first = w.spawn_cloth(400.0, 100.0, 6, 6, 30.0)
  assert_true(first >= 0)
  let before = w.constraint_count()
  assert_true(before > 0)
  // rip a middle node away
  let mid = first + 3 * 6 + 3
  w.px[mid] = w.px[mid] + 400.0
  w.py[mid] = w.py[mid] + 400.0
  for k = 0; k < 10; k = k + 1 {
    w.step()
  }
  assert_true(w.tears > 0)
  assert_true(w.constraint_count() < before)
}

///|
test "rope hangs from its pinned end" {
  let w = World::new(1000.0, 1200.0)
  let head = w.spawn_rope(500.0, 50.0, 12, 25.0)
  assert_true(w.imass[head].abs() < 0.000000001) // pinned
  for k = 0; k < 300; k = k + 1 {
    w.step()
  }
  let tail = head + 12
  // hangs below the pin, roughly straight down
  assert_true(w.py[tail] > 250.0 && w.py[tail] < 450.0)
  assert_true((w.px[tail] - 500.0).abs() < 40.0)
}

///|
test "three-box stack stays intact and inside bounds" {
  let w = World::new(1200.0, 900.0)
  w.iterations = 10
  let b0 = w.spawn_box(600.0, 650.0, 90.0)
  let b1 = w.spawn_box(600.0, 500.0, 90.0)
  let b2 = w.spawn_box(600.0, 350.0, 90.0)
  for k = 0; k < 600; k = k + 1 {
    w.step()
  }
  // nothing exploded, nothing escaped, boxes stayed rigid
  for i = 0; i < 12; i = i + 1 {
    assert_true(w.px[i] == w.px[i])
    assert_true(w.py[i] > 0.0 && w.py[i] < 900.0)
    assert_true(w.px[i] > 0.0 && w.px[i] < 1200.0)
  }
  let d0 = dist(w, b0, b0 + 2)
  let d1 = dist(w, b1, b1 + 2)
  let d2 = dist(w, b2, b2 + 2)
  let d_target = 90.0 * 1.41421356
  assert_true((d0 - d_target).abs() / d_target < 0.08)
  assert_true((d1 - d_target).abs() / d_target < 0.08)
  assert_true((d2 - d_target).abs() / d_target < 0.08)
  // the stack did not wander across the room
  let cx = (w.px[b0] + w.px[b1] + w.px[b2]) / 3.0
  assert_true((cx - 600.0).abs() < 120.0)
}

///|
test "particle pool capacity is enforced" {
  let w = World::new(4000.0, 4000.0)
  let mut last = 0
  while last >= 0 {
    last = w.spawn_ball(10.0, 10.0, 5.0)
  }
  assert_eq(w.particle_count(), MAX_PARTICLES)
  w.clear_scene()
  assert_eq(w.particle_count(), 0)
  assert_true(w.spawn_ball(10.0, 10.0, 5.0) == 0)
}
