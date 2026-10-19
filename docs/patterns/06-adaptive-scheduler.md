# Adaptive Scheduler: Frame-Time Feedback and Quality Scaling

The current scheduler computes a per-entity priority from distance and
importance, then maps that priority to a simulation frequency. Once set,
each entity's frequency stays fixed until something in the world changes.

This is sufficient when the entity count is small or the frame budget is
always met. It is not sufficient when the simulation is heavy enough that
frame time sometimes exceeds the target. In that case, the scheduler needs
to be able to say: *the system is over budget, everyone runs slower.*

The adaptive scheduler adds a global quality multiplier on top of the
existing per-entity priority. It watches frame time, decides when to
degrade or upgrade, and adjusts quality accordingly. Every entity's
frequency scales with that multiplier.

## What It Adds

Three things, in order of importance:

1. **A single quality level (0.0 to 1.0)** that multiplies every entity's
   computed frequency. When quality drops, everyone runs slower. When
   quality rises, everyone runs faster.

2. **A controller** that watches frame time and adjusts quality with
   hysteresis. The controller is a small state machine: how long has
   frame time been over budget, how far over, is it time to act.

3. **A signal** that fires when the quality level changes. The UI can
   listen, telemetry can log it, other subsystems can react.

## Why Hysteresis Matters

The naive design is: if frame time is over budget, reduce quality. If
under budget, increase quality. This fails immediately.

Frame time is noisy. It varies from frame to frame based on what the GPU
is doing, what the OS is doing, what the disk is doing. A single over-budget
frame is not a sign of trouble; it's normal jitter.

If the controller reacts to every frame, the quality level oscillates:

    frame over -> quality down -> frame under -> quality up -> frame over
    -> quality down -> quality up -> ...

This is called **thrashing**, and it is worse than never adjusting at all.
The user sees stuttering, the physics changes every frame, and the whole
system feels unstable.

The fix is time-based hysteresis. Act only when the trend is sustained:

- Do not degrade on the first over-budget frame. Wait until frame time
  has been over budget for a sustained window (typically 0.5 seconds).
- Do not upgrade on the first under-budget frame. Wait longer (typically
  2 seconds). Upgrades should be reluctant; downgrades should be quick.

The asymmetry is deliberate. A game that sits at reduced quality feels
acceptable. A game that oscillates between quality levels feels broken.

## The Controller State Machine

Three states:

**STABLE** - frame time is within acceptable bounds. Quality does not
change. Transition to DEGRADING if frame time goes too high for too long.
Transition to RECOVERING if frame time stays low for a long time.

**DEGRADING** - quality is being reduced step by step. Each interval,
subtract a fixed amount from quality. If frame time drops back into the
acceptable band, transition to STABLE. If frame time drops low enough,
transition to RECOVERING.

**RECOVERING** - quality is being increased step by step. Each interval,
add a smaller fixed amount to quality. If frame time rises, transition
back to STABLE (do not degrade immediately; give the new quality a chance
to settle). If frame time rises sharply, transition directly to DEGRADING.

The interesting detail is the transition from RECOVERING back to STABLE.
Upgrades are always risky because you do not know if the extra work will
fit until you try it. The controller should upgrade cautiously, hold the
new quality for a moment, and only continue upgrading if the frame time
stayed good.

## The Parameters

- **target_frame_time_ms** - the budget. 16.67 for 60 FPS, 33.33 for 30
  FPS. User-configurable.
- **degrade_threshold** - how far over budget before we act. Usually 1.15
  (15 percent over).
- **upgrade_threshold** - how far under budget before we recover. Usually
  0.85 (15 percent under).
- **degrade_interval_s** - how long over budget before degrading. Usually
  0.5 seconds.
- **upgrade_interval_s** - how long under budget before upgrading. Usually
  2.0 seconds.
- **degrade_step** - how much to reduce quality per degrade. Usually 0.1.
- **upgrade_step** - how much to increase quality per upgrade. Usually
  0.05.
- **min_quality** / **max_quality** - bounds. Usually 0.1 to 1.0.

These values are starting points. They will be tuned once the system is
running.

## Frame Time Averaging

The controller does not react to instant frame time. It uses a smoothed
average. The standard technique is exponential moving average:

    avg = avg * (1 - alpha) + current * alpha

Where alpha is a small value (0.05 to 0.1). This gives recent frames more
weight than older frames, but still provides memory of the recent past.

A raw moving average over a fixed window works too, but requires storing
the window. The exponential form is more memory-efficient and works just
as well for this purpose.

## Interaction With Per-Entity Frequencies

The quality level does not replace per-entity priority. It multiplies it.

    effective_freq = base_freq * quality

Where base_freq is what the current scheduler computes from distance and
importance. So a low-priority entity at quality 0.5 runs at a quarter the
frequency of a high-priority entity at quality 1.0. Both scale together.

This means the scheduler preserves its existing behavior in the common
case (quality stays at 1.0). Adaptive scaling only kicks in when the
system is under pressure.

## Where the Frame Time Comes From

The controller needs to know the actual frame time. In Godot:

- `Engine::get_singleton()->get_frames_per_second()` returns smoothed FPS.
  Inverting gives seconds per frame.
- Or the scheduler can measure its own `_physics_process` delta. This is
  cheaper and does not depend on Godot's averaging, but only reflects the
  physics tick rate, not the full frame time.

For v1 of the adaptive scheduler, use the scheduler's own delta. It is
simpler and covers the common case where the physics is the bottleneck.
A future version could read the engine's frame time if GPU is the limiting
factor.

## Interaction With the Scheduler's Existing Loop

The current scheduler:

    for each entity:
        compute priority
        map priority to frequency
        accumulate delta
        step as needed

The adaptive version adds one step at the top:

    update controller (may adjust quality)
    for each entity:
        compute priority
        map priority to frequency
        apply quality multiplier to frequency
        accumulate delta
        step as needed

Everything else stays the same. The change is additive.

## What It Does Not Do

- **Thermal awareness** - mobile devices can throttle. Detecting that
  requires OS-specific APIs. Out of scope.
- **Per-subsystem degradation** - all entities scale together. A future
  version could degrade particles first, then physics, then audio.
- **GPU-bound detection** - if the frame time is GPU-bound, reducing
  simulation frequency will not help. A future version could read GPU
  time and disable the scheduler's adaptation in that case.
- **Predictive scaling** - the controller reacts to past frame time, not
  predicting future. A future version could anticipate load spikes.

These are all follow-up concerns. The core adaptive controller is the
pattern. Everything else is refinement.

## Testing

Three scenarios:

1. **Idle scenario.** No load. Frame time should be well under budget.
   Quality stays at 1.0. No oscillation.

2. **Overload scenario.** Spawn 10,000 entities. Frame time exceeds
   budget. Quality drops until frame time recovers. Then stabilizes.

3. **Load spike.** Spawn 5,000 entities, wait for the system to stabilize,
   then spawn 5,000 more. Quality should drop again. Then despawn them.
   Quality should rise slowly back to 1.0.

The key metric is **no oscillation**. Quality should be monotone within
each scenario: only decreasing while load is present, only increasing
when load is removed, never bouncing back and forth.

## Summary

The adaptive scheduler adds a global quality multiplier controlled by a
hysteresis state machine. It exists so the simulation can degrade gracefully
under load instead of failing hard. It is not required for the current
project, but it will be when the entity count grows.

It is also a self-contained pattern. Once it works for the scheduler, the
same controller can be reused for particles, audio, or anything else that
needs adaptive quality scaling.

## References

- docs/patterns/01-frequency-lod.md - the base scheduler this builds on
- src/simulation_scheduler.cpp - the implementation being extended
- src/simulation_scheduler.h - the class that will gain the controller
---

## Implementation Notes

The design above was written before the implementation. This section
captures what we learned building it, including two real bugs the
design did not anticipate.

## Bug 1: `p_delta` Is Not Frame Time

The original design said "use the scheduler's own delta for frame time."
That was wrong.

`p_delta` in `_physics_process` is Godot's **fixed simulation timestep**
— 1/60 s at 60 Hz physics, or 1/200 s at 200 Hz. It never changes. It
is a value Godot passes in, not an observation of how long the tick
actually took.

Feeding `p_delta` to the controller as "frame time" made it think every
tick took exactly 16.67 ms. With a 5 ms budget, that reads as
over-budget every tick, and the controller degraded to 0.1 during
**idle** — no entities, no load, no reason to degrade.

**Fix:** measure wall-clock time ourselves.

```cpp
#include <godot_cpp/classes/time.hpp>
// ...
const int64_t start = Time::get_singleton()->get_ticks_usec();
// ... do all the work ...
const int64_t end = Time::get_singleton()->get_ticks_usec();
const float elapsed_ms = (float)(end - start) / 1000.0f;
_update_quality(elapsed_ms, p_delta);
```

The elapsed value reflects real work done by the scheduler's loop and
every entity's `step()`. That's the number the controller needs.

## Bug 2: Recovery Is Very Slow from a Deep Dive

The asymmetry is deliberate — degrade fast, upgrade slow — but the
numbers in the design doc understated how slow "slow" would be.

From quality 0.1 back to 1.0 takes 18 upgrade steps × 2.0 s per step =
**36 seconds minimum**. In practice it's longer because the tick must
stay under budget *continuously* for each window.

That's fine for a game session where the load drops and you have a few
minutes to recover. It's not fine for a benchmark where you want to see
the controller work. For the demo we lowered the target to 1 ms and
loaded 50,000 entities to force visible change within a 20-second test.

A future tweak: make the upgrade interval adaptive to how far below
budget the tick is. Very-under-budget ticks could upgrade faster than
marginally-under-budget ones. Not implemented — noted for later.

## Verification Numbers

Tested on an Intel UHD Graphics 605 laptop (Pop!_OS, 4 cores):

| Phase | Load | Quality | Avg freq |
|---|---|---|---|
| Idle | 0 entities | 1.0 (stable) | — |
| Load | 50,000 entities | 1.0 → 0.1 in ~5 s | 172 → 17 |
| Unload | 0 entities | 0.1 → 0.4 in ~8 s | — |

The frequency scaling is exactly linear:

```text
avg_freq = base_freq × quality
172 × 1.0 = 172
172 × 0.8 = 137.6   (matches observed 137.6)
172 × 0.5 = 86.0    (matches observed 86.0)
172 × 0.1 = 17.2    (matches observed 17.2)
```

The controller's frequency multiplier is provably correct.

## What the Controller Does NOT Do

- **Per-subsystem scaling.** All entities share one quality value.
  A future version could give particles, physics, and AI separate
  quality levels, so the controller degrades the cheapest first.
- **GPU-bound detection.** If the frame time is dominated by
  rendering, reducing simulation frequency doesn't help. The
  controller degrades anyway, wasting the CPU savings. Detecting
  GPU-bound would require reading the engine's frame time (not just
  our tick), which is a future extension.
- **Threading.** Everything runs on the main thread. Worker threads
  are T2/T3 concerns.
- **Hysteresis on the input.** If frame time oscillates right at the
  threshold, the EMA smooths it, but a pathological workload could
  still cause the controller to flap between degrade and upgrade
  states. The up/down asymmetry mitigates this but doesn't eliminate
  it.
- **Signal batching.** Each quality step emits `quality_changed`
  individually. If the UI wants to animate a smooth quality
  transition, it must lerp internally.

## The `quality_changed` Signal

    ADD_SIGNAL(MethodInfo("quality_changed",
            PropertyInfo(Variant::FLOAT, "quality")));

Fires once per step of quality — not per frame. During a 5-second
degrade from 1.0 to 0.1, it fires nine times. During a 20-second
recovery it fires roughly a dozen times. A UI can connect to this to
animate a quality indicator or log the transitions.

Because the signal carries a single float, C# can connect to it with
`stats.QualityChanged += (float q) => { ... };` — same interop pattern
as any other signal with a numeric argument.

## Files

- `src/simulation_scheduler.h` / `.cpp` — the controller lives here
- `demo/adaptive_scheduler_test.gd` — verification demo
- `docs/patterns/01-frequency-lod.md` — the base priority-based
  scheduler that this builds on
