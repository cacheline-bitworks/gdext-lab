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
| **V1** | Pacejka Magic Formula, peak + falloff | Arcade | ✅ |
| **V2** | Load sensitivity + friction ellipse | Simcade | ✅ |
| **V3** | Relaxation length + transient response | Proper sim | 📋 Planned |

### Vehicle bodies

| Tier | Adds | Wheels | Status |
|---|---|---|---|
| **T1** | Rigid body in a plane, analytic load split | 2 virtual | ✅ |
| **T2a** | Heave + pitch, real suspension, emergent load transfer | 2 | ✅ |
| **T2b** | Roll, left/right load split | 4 | 📋 Planned |
| **T2c** | Full 6-DOF rigid body | 4 | 📋 Planned |

### Simulation infrastructure

| Component | Purpose | Status |
|---|---|---|
| **SimulationScheduler** | Priority-driven variable-rate entity stepping | ✅ |
| **Adaptive scheduler** | Frame-time feedback, quality scaling | 📋 Planned |

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