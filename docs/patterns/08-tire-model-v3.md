# TireModelV3: Relaxation Length and Transient Response

V1 and V2 are stateless. Given the same inputs, they return the same
outputs, every time. That's mathematically clean, but it's not how real
tires work.

A real tire carcass deforms over distance. When you steer, the front tire
doesn't produce full lateral force instantly — the rubber builds up shear
over the first few centimeters of travel before the force reaches its
steady-state value. This lag is called **relaxation length**, and it's
what makes a car's response feel like a car.

V3 adds that lag. It's the last public tire tier.

## The Physical Effect

The tire's response to a step change in slip angle looks like an
exponential approach to the new steady-state value:

    slip_angle(t) = target + (initial - target) * exp(-t / tau)

Where tau is a time constant derived from the relaxation length:

    tau = sigma / vx

`sigma` is the relaxation length in meters. `vx` is forward velocity.

At 30 m/s with sigma = 0.5 m, tau = 0.0167 s. The tire reaches 63% of
its target force in about 17 ms, 95% in about 50 ms. That's fast enough
to be invisible in straight-line driving, but slow enough to change how
the car feels at the limit.

## Why It Matters

Three reasons:

**1. Transient handling.** When you turn in fast, the front tires build
lateral force over ~50 ms. The car's rotation begins before the tire
force is fully developed. This produces the small lag between steering
input and yaw response that racing drivers call "feel."

**2. Drift transitions.** At the limit, slip angle changes rapidly. With
instant response (V2), the tire force snaps to a new value every tick —
a hard, binary transition between grip and slide. With relaxation (V3),
force builds over time, giving the driver a window to countersteer.

**3. Combined slip damping.** Combined with V2's friction ellipse, the
relaxation filter smooths the interaction between lateral and
longitudinal force. Fast slip changes produce smooth force changes
rather than sharp spikes.

## The Model

V3 is a first-order lag on the slip values that V2 uses:

    d(slip)/dt = (vx / sigma) * (target_slip - current_slip)

Discretized for a fixed timestep:

    factor = clamp(vx * dt / sigma, 0, 1)
    current_slip += factor * (target_slip - current_slip)

The `factor` is the fraction of the remaining gap closed this tick.
`vx * dt` is the distance traveled this tick. Divided by `sigma`, it's
how many relaxation lengths we've moved. If that's more than 1, we close
the whole gap — clamping at 1.

At low speed the factor becomes very small. The tire barely responds.
That's physically correct — a slow-moving tire is slowly deforming.

## Integration

V3 needs `dt` to advance the filter. This required adding a `dt` field
to `WheelInput`, which V1 and V2 ignore. Any vehicle step that calls a
tire must populate `in.dt` with its own timestep.

The vehicle steps set `in.dt = dt` when building their `WheelInput`
structs. V1/V2 never read it. V3 reads it inside the filter functions.

## Filter Hooks in V2

V2 exposes two virtual methods that V3 overrides:

    virtual float filter_slip_ratio(float target, const WheelInput &);
    virtual float filter_slip_angle(float target, const WheelInput &);

V2's default implementations return the target unchanged. So V2 behaves
exactly as before — the hooks are pure pass-throughs.

V3's overrides apply the relaxation lag:

    float vx_abs = |velocity_longitudinal|;
    factor = clamp(vx_abs * dt / relaxation_length, 0, 1);
    m_relaxed_slip_angle += factor * (target - m_relaxed_slip_angle);
    return m_relaxed_slip_angle;

The state lives in the tire, not in the vehicle. Every tire has its own
relaxation history, independent of the others. That's correct — each
tire deforms based on its own slip history.

## Statefulness

V1 and V2 are stateless. V3 is not. This is the first tire that carries
memory across steps.

Consequences:

- `TireModel::reset()` becomes meaningful. V3 overrides it to clear the
  relaxed slip values. Call it on race restart, respawn, or any reset
  of the vehicle.
- Serializing tire state becomes possible (and required if you want to
  save a session mid-race). Not implemented yet — noted for the future.
- Sharing a V3 tire between two wheels is a bug. Each wheel needs its
  own instance. The `reset()` method is per-tire, not per-vehicle.

## The `step_gd` Override

V2's `step_gd` is a GDScript test helper. It doesn't take a `dt`, so V3's
relaxation filter would be a silent no-op when called this way — the
filter would always return the input unchanged.

V3 overrides `step_gd` to fail loudly:

    ERR_PRINT("TireModelV3::step_gd needs a dt. Use step_gd_with_dt instead.");
    return Dictionary();

And provides `step_gd_with_dt` which does take a `dt`. Callers get an
explicit error pointing them at the right function instead of a
mysteriously inert tire.

## Testing

The tick-by-tick test at `demo/tire_v3_tick_test.gd` shows the effect
directly. Both V2 and V3 start at zero slip angle. At tick 0 the target
jumps to 5 degrees.

V2 returns ~2500 N of lateral force on the next tick. V3 returns ~750 N,
then 1300 N, then 1700 N, converging to 2500 N over ~20 ticks (~100 ms).
Same steady-state result, different transient.

That ~100 ms convergence is the whole point of V3.

## Cost

V3 adds roughly 10% CPU per tire step compared to V2 — one extra
multiply, one comparison, one addition per filter per tick. Negligible
for four wheels at 60-200 Hz.

The real cost is architectural: tire instances must not be shared between
wheels, and the vehicle owns the responsibility of calling `reset()`
when appropriate.

## What V3 Does Not Do

- Thermal effects (V4 concern, faked at the frontend)
- Wear (V5 concern, faked at the frontend)
- Anisotropic relaxation (different sigma for lateral vs longitudinal)
- Surface-dependent relaxation length

V3 is the last public tire tier. V4 and V5 features are faked at the
game layer rather than implemented in the reference library.

## References

- `src/tire_model_v3.h` / `.cpp` — the implementation
- `src/tire_model_v2.h` — the base with filter hooks
- `demo/tire_v3_tick_test.gd` — step response demo
- `docs/patterns/04-tire-model-v2.md` — the base V2 design
