// Wasm boundary for the moonphysics sandbox.
//
// The whole scene lives in the MoonBit (GC) side; the host drives it with
// command-style exported functions and reads a compact snapshot of particle
// positions for canvas rendering — the per-frame host cost is one exported
// call plus one typed-array read, no per-object boundary crossings.
//
// Snapshot layout (little-endian, at SNAP_ADDR):
//   [0]              u32  particle count n
//   [8 + i*48]       f64  x, y, radius, prev-x, prev-y  then u32 kind, u32 pad
// f64 throughout so a snapshot round trip is byte-exact (the benchmark and
// the determinism check both depend on that).
// Edge list (at EDGE_ADDR), for rendering sticks/ropes/cloth links:
//   [0]              u32  edge count m
//   [4 + i*8]        u32  particle index pair (only active visible ones)
// `snapshot_in` restores a balls-only snapshot (benchmark support).

///|

///|
extern "wasm" fn store_u32(addr : Int, v : Int) -> Unit =
  #|(func (param $addr i32) (param $v i32) (i32.store (local.get $addr) (local.get $v)))

///|

///|
extern "wasm" fn load_u32(addr : Int) -> Int =
  #|(func (param $addr i32) (result i32) (i32.load (local.get $addr)))

///|
extern "wasm" fn store_f64(addr : Int, v : Double) -> Unit =
  #|(func (param $addr i32) (param $v f64) (f64.store (local.get $addr) (local.get $v)))

///|
extern "wasm" fn load_f64(addr : Int) -> Double =
  #|(func (param $addr i32) (result f64) (f64.load (local.get $addr)))

///|
let world : @ref.Ref[@lib.World?] = @ref.Ref(None)

///|
/// Create the world for the given viewport size in pixels.
#export_name("moon_init")
pub fn world_init(width : Int, height : Int) -> Unit {
  world.val = Some(@lib.World::new(width.to_double(), height.to_double()))
}

///|
#export_name("set_gravity")
pub fn set_gravity(g : Float) -> Unit {
  match world.val {
    Some(w) => w.gravity = g.to_double()
    None => ()
  }
}

///|
#export_name("set_restitution")
pub fn set_restitution(e : Float) -> Unit {
  match world.val {
    Some(w) => w.restitution = e.to_double()
    None => ()
  }
}

///|
#export_name("set_friction")
pub fn set_friction(f : Float) -> Unit {
  match world.val {
    Some(w) => w.friction = f.to_double()
    None => ()
  }
}

///|
#export_name("set_iterations")
pub fn set_iterations(n : Int) -> Unit {
  match world.val {
    Some(w) => w.iterations = n
    None => ()
  }
}

///|
#export_name("spawn_ball")
pub fn spawn_ball(x : Float, y : Float, r : Float) -> Int {
  match world.val {
    Some(w) => w.spawn_ball(x.to_double(), y.to_double(), r.to_double())
    None => -1
  }
}

///|
#export_name("spawn_box")
pub fn spawn_box(x : Float, y : Float, size : Float) -> Int {
  match world.val {
    Some(w) => w.spawn_box(x.to_double(), y.to_double(), size.to_double())
    None => -1
  }
}

///|
#export_name("spawn_rope")
pub fn spawn_rope(x : Float, y : Float, segments : Int, seg_len : Float) -> Int {
  match world.val {
    Some(w) =>
      w.spawn_rope(x.to_double(), y.to_double(), segments, seg_len.to_double())
    None => -1
  }
}

///|
#export_name("spawn_cloth")
pub fn spawn_cloth(
  x : Float,
  y : Float,
  cols : Int,
  rows : Int,
  spacing : Float,
) -> Int {
  match world.val {
    Some(w) =>
      w.spawn_cloth(
        x.to_double(),
        y.to_double(),
        cols,
        rows,
        spacing.to_double(),
      )
    None => -1
  }
}

///|
/// Grab the particle nearest to (x, y) within `radius`. Returns its index
/// or -1.
#export_name("grab")
pub fn grab(x : Float, y : Float, radius : Float) -> Int {
  match world.val {
    Some(w) => w.grab(x.to_double(), y.to_double(), radius.to_double())
    None => -1
  }
}

///|
#export_name("drag")
pub fn drag(x : Float, y : Float) -> Unit {
  match world.val {
    Some(w) => w.drag(x.to_double(), y.to_double())
    None => ()
  }
}

///|
#export_name("release")
pub fn release() -> Unit {
  match world.val {
    Some(w) => w.release()
    None => ()
  }
}

///|
#export_name("explode")
pub fn explode(x : Float, y : Float, radius : Float, power : Float) -> Unit {
  match world.val {
    Some(w) =>
      w.explode(
        x.to_double(),
        y.to_double(),
        radius.to_double(),
        power.to_double(),
      )
    None => ()
  }
}

///|
#export_name("clear_scene")
pub fn clear_scene() -> Unit {
  match world.val {
    Some(w) => w.clear_scene()
    None => ()
  }
}

///|
/// Advance the simulation one fixed 1/60 s timestep.
#export_name("step")
pub fn step() -> Unit {
  match world.val {
    Some(w) => w.step()
    None => ()
  }
}

///|
/// Returns (edge count << 16) | particle count.
#export_name("counts")
pub fn counts() -> Int {
  match world.val {
    Some(w) => w.constraint_count() * 65536 + w.particle_count()
    None => 0
  }
}

///|
/// Total solver steps taken since the last clear (stats badge).
#export_name("step_count")
pub fn step_count() -> Int {
  match world.val {
    Some(w) => w.steps
    None => 0
  }
}

///|
/// Write the render snapshot (particles at SNAP-style addr, edges at
/// edge_addr). See the layout comment at the top of the file.
#export_name("snapshot_out")
pub fn snapshot_out(addr : Int, edge_addr : Int) -> Unit {
  match world.val {
    Some(w) => {
      let n = w.particle_count()
      store_u32(addr, n)
      let mut a = addr + 8
      for i = 0; i < n; i = i + 1 {
        store_f64(a, w.px[i])
        store_f64(a + 8, w.py[i])
        store_f64(a + 16, w.rad[i])
        store_f64(a + 24, w.ppx[i])
        store_f64(a + 32, w.ppy[i])
        store_u32(a + 40, w.kind[i])
        a = a + 48
      }
      // visible active edges
      let mut m = 0
      let mut b = edge_addr + 4
      for c = 0; c < w.m; c = c + 1 {
        if w.cactive[c] && w.cvis[c] {
          store_u32(b, w.ca[c])
          store_u32(b + 4, w.cb[c])
          b = b + 8
          m = m + 1
        }
      }
      store_u32(edge_addr, m)
    }
    None => ()
  }
}

///|
/// Restore a balls-only snapshot previously written by `snapshot_out` (the
/// benchmark uses this to give wasm and JavaScript the identical initial
/// state). Returns the restored count.
#export_name("snapshot_in")
pub fn snapshot_in(addr : Int) -> Int {
  match world.val {
    Some(w) => {
      let n = load_u32(addr)
      let mut a = addr + 8
      w.clear_scene()
      let mut restored = 0
      for _i = 0; _i < n; _i = _i + 1 {
        let x = load_f64(a)
        let y = load_f64(a + 8)
        let r = load_f64(a + 16)
        let ppx0 = load_f64(a + 24)
        let ppy0 = load_f64(a + 32)
        let k = load_u32(a + 40)
        a = a + 48
        if k == @lib.KIND_BALL {
          let idx = w.spawn_ball(x, y, r)
          if idx >= 0 {
            w.ppx[idx] = ppx0
            w.ppy[idx] = ppy0
            restored = restored + 1
          }
        }
      }
      restored
    }
    None => 0
  }
}
