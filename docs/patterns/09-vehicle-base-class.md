# The VehicleBody Base Class: Consolidating Vehicle Tiers

For the first several vehicle tiers (T1, T2a, T2b, T2c), each class was
written by copying the previous one and modifying it. That worked — the
tiers were already tested, so copying was fast. But by the time T3 landed,
roughly 500 lines of identical code existed across five files.

A code review in late October flagged this as a design flaw (C1). The fix
was a shared base class: `VehicleBody`.

This document explains what the base class owns, what subclasses own, and
the specific design decisions behind the split.

## The Duplication

Every vehicle tier was declaring and implementing the same members:

- `m_mass`, `m_wheel_base`, `m_track_width`, `m_cg_to_front`,
  `m_cg_to_rear`, `m_wheel_radius`, `m_wheel_inertia`
- `m_throttle`, `m_brake_input`, `m_steer_input`
- `m_max_engine_torque`, `m_max_brake_torque`, `m_max_steer_angle`
- `m_omega[4]` and the wheel spin integration
- `set_throttle`, `get_throttle`, `set_brake`, `get_brake`,
  `set_steer`, `get_steer`
- `apply_resistance(dt)` — rolling resistance, aero drag, low-speed
  damping
- The world-to-body and body-to-world rotation blocks
- The yaw integration block with damping
- The tire force summation

That's not a small amount of copy-paste. Every fix to any of those blocks
had to be applied five times, and forgetting one left a tier silently
inconsistent with its siblings — which is exactly what happened with the
yaw inertia (T2b was reverted to a thin-rod formula while T2c still used
3000).

## What Moved Into VehicleBody

The base class owns:

**Configuration and physical constants:**

- Mass and geometry
- Torque limits and steer limits
- Yaw inertia (used by scalar-yaw tiers)
- Wheel omegas array

**Driver input state:**

- Current throttle, brake, steer
- Accessors with clamping

**Shared helpers:**

- `apply_resistance(dt)` — applies rolling resistance, aero drag, and
  low-speed damping to `m_velocity`
- `integrate_wheel_spin(i, force_x, drive_torque, brake_torque, dt, multiplier)`
  — integrates one wheel's angular velocity from torque and reaction

Both helpers are called by every tier. Behavior is identical across them.

## What Stays in Each Tier

Subclasses own their **orientation representation** and their **step()
logic**:

- **T1:** yaw scalar, yaw rate. Two virtual wheels.
- **T2a:** yaw scalar, heave, pitch. Two virtual wheels with suspension.
- **T2b:** yaw scalar, heave, pitch, roll. Four wheels.
- **T2c:** quaternion orientation, angular velocity, inertia tensor,
  per-corner suspension, aero.
- **T3:** same as T2c, plus ARB, Ackermann, camber, toe.

The base class has no orientation state. It has no `step()` — it inherits
`SimulatedEntity::step()`, which does trivial linear motion. Every tier
overrides `step()` completely.

## The Wheel Multiplier

One subtle issue the refactor surfaced: **bicycle tiers and four-wheel
tiers need different effective wheel inertia.**

T1 and T2a use a bicycle model. Each "wheel" represents **two physical
wheels** on an axle. So the effective inertia at the axle is `2 *
m_wheel_inertia`.

T2b, T2c, and T3 use four real wheels. Each wheel is one wheel, with
inertia `m_wheel_inertia`.

The original `integrate_wheel_spin` hardcoded `2.0f * m_wheel_inertia`.
That was correct for T1/T2a but wrong for every 4-wheel tier — those
wheels had 2x the inertia they should have, and spun up at half the
correct rate.

The fix: make the multiplier a parameter.

    void integrate_wheel_spin(int index, float force_x,
            float drive_torque, float brake_torque, double dt,
            float wheel_multiplier);

Call sites:

- T1, T2a: pass `2.0f`
- T2b, T2c, T3: pass `1.0f`

Explicit at the call site rather than hidden in the helper. Future tiers
that use a different wheel count (tandem axles, six-wheel vehicles) can
pass whatever multiplier is correct for their configuration.

## Yaw Convention Unification

The refactor also surfaced a genuine bug: T1/T2a/T2b used a 2D rotation
matrix that mapped yaw=π/2 to world +Z. T2c used Godot's quaternion
convention, which maps the same yaw to world -Z.

Same numeric yaw value meant **opposite orientations** depending on which
tier the vehicle was. Any scene mixing a T1 AI car with a T2c player car
would show them facing opposite directions when they should be aligned.

The fix: all four tiers now use the Godot-compatible convention. T1/T2a/
T2b's body-to-world transforms use the same sign pattern as the
quaternion. The yaw moment sign was flipped to match.

This is documented in `docs/gotchas.md` entry 3.

## Design Principles

**1. Pure data in the base.** Members are plain values with sensible
defaults. No pointers, no references to subclasses.

**2. Helpers that are truly universal.** `apply_resistance` and
`integrate_wheel_spin` are used by every tier without modification. If a
future tier needs different behavior, it overrides the helper — but the
default is correct for the common case.

**3. No orientation state in the base.** Subclasses decide whether they
use scalars (T1/T2a/T2b) or quaternions (T2c/T3). The base class never
assumes.

**4. No `step()` logic in the base.** Every tier's `step()` is different
enough that a shared implementation would be wrong. The base provides
tools; subclasses use them.

## What NOT to Add to the Base

Tempting additions that should be resisted:

- **Suspension state.** T1 has none, T2a has 2 corners, T2b has 4, T2c
  has 4 with different configs. Too divergent to share.
- **Aero parameters.** T1/T2a/T2b have no aero. Only T2c and T3 need it.
- **Wheel arrays of `Ref<TireModel>`.** Different tiers use different
  tire types. T1 uses `TireModelV1`, T2a uses `TireModelV2`, T2c and
  T3 could use `TireModelV3`. The array itself belongs in the subclass.
- **Any helper that has an `if (tier == ...)` branch inside it.** If the
  helper needs to know which subclass it's in, it doesn't belong in the
  base.

## How Future Tiers Extend It

A T4 tier (say, tire thermal modeling) would:

1. Inherit from `VehicleBody`
2. Add its own orientation state (likely quaternion, copying T2c's shape)
3. Add per-tire thermal state
4. Override `step()` with the thermal integration in place
5. Call `apply_resistance(dt)` and `integrate_wheel_spin(...)` at the
   end of its step

The shared mass, geometry, inputs, limits, and omegas come for free. The
tier only implements what makes it distinct.

## Refactor Results

Before:

- T1: ~250 lines
- T2a: ~320 lines
- T2b: ~350 lines
- T2c: ~400 lines
- T3: ~200 lines
- Total: ~1520 lines with ~500 of duplication

After:

- VehicleBody: ~150 lines (shared)
- T1: ~130 lines
- T2a: ~200 lines
- T2b: ~220 lines
- T2c: ~280 lines
- T3: ~120 lines
- Total: ~1100 lines with no duplication

The 420-line reduction is nice. The real win is that any future fix to
mass, geometry, input handling, resistance, or wheel spin only needs to
be applied once.

## References

- `src/vehicle_body.h` / `.cpp` — the base class
- `docs/patterns/05-vehicle-body-t2.md` — tier design overview
- `docs/patterns/07-vehicle-body-t2c.md` — the 6-DOF tier
- `docs/gotchas.md` entry 3 — yaw convention history
