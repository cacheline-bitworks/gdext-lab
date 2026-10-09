# gdext-lab

A layered reference for vehicle simulation in Godot GDExtension — tire models,
vehicle tiers, scheduler, and the gotchas that come with them.

Each subsystem is built in **tiers**. A simple version exists, it works, and it
teaches a concept. The next tier adds one more physical effect on top. You can
read the tiers in order to understand how a simulation grows from "the car
moves" to "the car feels like a car."

This is a working reference, not a template. Every class in `src/` is designed
to be read. Every design decision is documented, including the ones that were
wrong the first time.

---

## What's Inside

### Tire models

| Version | Adds | Feels like | Status |
|---|---|---|---|
| **V1** | Pacejka Magic Formula, peak + falloff | Arcade | Done |
| **V2** | Load sensitivity + friction ellipse | Simcade | Done |
| **V3** | Relaxation length + transient response | Proper sim | Planned |

### Vehicle bodies

| Tier | Adds | Wheels | Status |
|---|---|---|---|
| **T1** | Rigid body in a plane, analytic load split | 2 virtual | Done |
| **T2a** | Heave + pitch, real suspension, emergent load transfer | 2 | Done |
| **T2b** | Roll, left/right load split | 4 | Planned |
| **T2c** | Full 6-DOF rigid body | 4 | Planned |

### Simulation infrastructure

| Component | Purpose | Status |
|---|---|---|
| **SimulationScheduler** | Priority-driven variable-rate entity stepping | Done |
| **Adaptive scheduler** | Frame-time feedback, quality scaling | Planned |

### Documentation

- [`docs/gotchas.md`](docs/gotchas.md) — every debugging trap hit while building this, and the fix
- [`docs/patterns/01-frequency-lod.md`](docs/patterns/01-frequency-lod.md) — the scheduler's priority model
- [`docs/patterns/02-tire-physics.md`](docs/patterns/02-tire-physics.md) — the physics of a tire
- [`docs/patterns/03-wheel-interface.md`](docs/patterns/03-wheel-interface.md) — the drivetrain/tire boundary
- [`docs/patterns/04-tire-model-v2.md`](docs/patterns/04-tire-model-v2.md) — load sensitivity + combined slip
- [`docs/patterns/05-vehicle-body-t2.md`](docs/patterns/05-vehicle-body-t2.md) — unsprung mass + real suspension
- [`docs/patterns/06-adaptive-scheduler.md`](docs/patterns/06-adaptive-scheduler.md) — frame-time feedback

---

## Requirements

- **Godot** 4.4.x (Mono recommended if you want C# support)
- **Python** 3.8+
- **SCons** 4.5+ (`apt install scons` or `pip install scons`)
- **C++17 compiler** — Clang 14+, GCC 11+, or MSVC 2022+

---

## Quick Start

```bash
# 1. Clone with submodules
git clone --recursive https://github.com/cacheline-bitworks/gdext-lab.git
cd gdext-lab

# 2. Build the extension
scons

# 3. Link the extension into the demo project
cd demo
mkdir -p addons
ln -s ../../addons/gdext_lab addons/gdext_lab
cd ..

# 4. Open the demo in Godot
godot --editor --path demo --rendering-driver opengl3
```

> **Note on `--rendering-driver opengl3`** — Godot 4's Forward+ renderer
> requires Vulkan. On older Intel integrated GPUs (UHD 605 and similar), this
> crashes on launch. The Compatibility renderer uses OpenGL and works
> everywhere. If you have a modern GPU, you can drop the flag.

### Windows Note

Symlinks behave differently on Windows. Either copy `addons/gdext_lab/` into
`demo/addons/` after each build, or modify `SConstruct` to output directly to
`demo/addons/gdext_lab/bin/`.

---

## Demo Scenes

Each scene in `demo/` exercises one subsystem. Open any of them and press F6.

| Scene | What it shows |
|---|---|
| `scheduler_demo.tscn` | 1000 entities at variable frequencies, work-saved percentage |
| `tire_plot.tscn` | The V1 Pacejka curve — rise, peak, falloff |
| `tire_plot_v2.tscn` | V1 vs V2 side by side — load sensitivity visible |
| `tire_v2_debug.tscn` | Numeric verification of every V2 effect |
| `vehicle_demo.tscn` | T1 driving — accelerate, turn, brake, stop |
| `t2a_debug.tscn` | T2a suspension — squat, dive, settle |

---

## Project Layout

```
gdext-lab/
├── addons/gdext_lab/       # The distributable extension package
│   ├── bin/<platform>/     # Compiled .so / .dll / .dylib (gitignored)
│   └── gdext_lab.gdextension
├── demo/                   # Test Godot project
│   ├── assets/             # Car models, textures (gitignored)
│   └── *.gd, *.tscn        # Test scenes
├── doc_classes/            # Godot doctool XML output
├── docs/
│   ├── gotchas.md          # Debugging traps and fixes
│   └── patterns/           # Design documents per subsystem
├── src/                    # C++ source
├── godot-cpp/              # Submodule (Godot C++ bindings, branch 4.4)
└── SConstruct              # Build configuration
```

---

## Design Principles

**1. Data structs are pure numbers.** `SuspensionConfig`, `WheelInput`,
`WheelOutput` hold only values. No behavior, no dependencies.

**2. Behavior lives in classes that read data.** `TireModelV2` reads its
coefficients from members. `VehicleBodyT2a` reads suspension parameters from
its config.

**3. Tiers are siblings, not a chain.** `TireModelV1` and `TireModelV2` share a
common abstract base but neither inherits from the other.

**4. Document the failures.** `docs/gotchas.md` documents what went wrong, not
just what went right.

---

## What This Is Not

- **Not a template** — templates tell you "put your code here." This tells you
  why each piece exists.
- **Not production-ready** — the goal is understanding, not shipping.
- **Not complete** — T2b, T2c, V3, and the adaptive scheduler are planned but
  not built.

---

## Roadmap

- **v0.1.0** — "It drives" tier: V1 tires, T1 vehicle, scheduler — done
- **v0.2.0** — "It feels like a car" tier: V2 tires, T2a vehicle — done
- **v0.3.0** — Roll and four wheels: T2b, V3 tires — planned
- **v0.4.0** — Adaptive scheduler, simplified engine, first C# frontend — planned

See [`CHANGELOG.md`](CHANGELOG.md) for the current state.

---

## License

MIT — see [LICENSE](./LICENSE).

---

## Why This Exists

Most GDExtension repos are either templates ("here's a skeleton, go build") or
single-purpose extensions ("here's a fast noise library"). Very few publish
the reasoning behind the code — the tradeoffs, the wrong turns, the concepts
the tiers are built to teach.

This one does. It is a learning artifact as much as a working library. If
you're building a racing game, a vehicle simulator, or just trying to
understand how C++ extensions work in Godot, this is meant to save you the
sessions it took to figure out.