# Changelog

All notable changes to this project are documented here.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project uses [Semantic Versioning](https://semver.org/spec/v2.0.0.html)
loosely (pre-1.0, so the minor version reflects capability milestones).

---

## [Unreleased]

### Planned
- TireModelV3: relaxation length and transient response
- VehicleBodyT2b: roll and four-wheel model
- Adaptive scheduler: frame-time feedback and quality scaling
- Simplified engine model (torque curve + fake turbo lag)

---

## [0.2.0] - 2026-10-09

The tier where load transfer emerges from the physics.

### Added
- **TireModelV2** - load sensitivity and friction ellipse
  - Friction coefficient drops as vertical load rises
  - Fx and Fy share a single friction budget via the friction ellipse
  - `effective_mu()` exposed for testing and visualization
- **VehicleBodyT2a** - heave and pitch on the bicycle model
  - Two suspension corners with independent spring and damper
  - Tire loads derived from suspension force instead of analytic split
  - `SuspensionConfig` struct holds tunable parameters as pure data
- **Pattern docs** - `04-tire-model-v2.md`, `05-vehicle-body-t2.md`,
  `06-adaptive-scheduler.md`
- **Test scripts** - `tire_plot_v2.gd`, `tire_v2_debug.gd`, `t2a_debug.gd`

### Changed
- `vehicle_body_t1.cpp` - removed dead code, cleaned up comments
- `simulation_scheduler.h` - fixed `get_average_frequency` typo, moved to
  public section, added distance-to-frequency formula documentation
- `vehicle_body_t2a.cpp` - replaced placeholder ASCII comment with proper
  explanations of the yaw damping math

### Fixed
- Yaw damping at low speed - the car no longer spins in place after stopping
- Corrected sign convention in the front/rear velocity_lateral calculations

---

## [0.1.0] - 2026-10-09

First working milestone. The tire model moves a vehicle end-to-end.

### Added
- **TireModelV1** - Pacejka Magic Formula with explicit falloff past peak
- **VehicleBodyT1** - bicycle model with working tire physics
  - Full body-frame force transform
  - Wheel dynamics with drive torque, brake torque, and tire reaction
  - Yaw integration with speed-scaled damping
  - Rolling resistance and aero drag
- **SimulationScheduler** - priority-driven variable-rate entity stepping
  - Distance and importance based priority
  - Accumulator-based timing with spiral-of-death protection
  - Per-tick and cumulative statistics
- **SimulatedEntity** - base class for schedulable entities
- **ExampleClass** and **FrameData** - reference implementations of the
  method/property/signal pattern
- **Full GDExtension build pipeline** - `SConstruct`, `.gdextension` file,
  symlink workflow for the demo project
- **Pattern docs** - `01-frequency-lod.md`, `02-tire-physics.md`,
  `03-wheel-interface.md`
- **`docs/gotchas.md`** - reference of every debugging trap encountered
  (vtable errors, `.gdextension` format, threading rules, GPU compatibility)

### Infrastructure
- `godot-cpp` added as a git submodule pinned to branch 4.4
- Compatibility renderer setup for older Intel GPUs
- `.gitignore` covering build artifacts, Godot cache, .NET output, IDE files

---

## [0.0.1] - 2026-09-22

Initial project skeleton.

### Added
- Repository structure: `addons/`, `demo/`, `docs/`, `src/`, `tests/`
- `README.md`, `LICENSE` (MIT), `.gitignore`
- Initial pattern document structure

---

[Unreleased]: https://github.com/cacheline-bitworks/gdext-lab/compare/v0.2.0...HEAD
[0.2.0]: https://github.com/cacheline-bitworks/gdext-lab/compare/v0.1.0...v0.2.0
[0.1.0]: https://github.com/cacheline-bitworks/gdext-lab/releases/tag/v0.1.0
[0.0.1]: https://github.com/cacheline-bitworks/gdext-lab/commits/main