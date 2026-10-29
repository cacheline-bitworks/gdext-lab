# Tire Physics: The Shape of the Problem

A tire model is a mathematical function. It receives the current state of
a tire (slip, load, surface) and returns the forces that tire generates
against the ground.

Everything else — suspension, mass, aerodynamics, engine torque — lives
outside the model. The tire is an isolated system. That isolation is what
makes it testable.

## The Two Inputs That Matter

A tire model takes many inputs, but for a dynamic simulation, two of them
dominate everything else: **Vertical Load** and **Slip**.

**1. Vertical Load (Fz)**

How hard the tire is pressed into the ground. This changes everything.
Tires generate more force under more load, but not linearly.

**2. Slip**

Slip is the difference between the wheel's velocity and the ground's
velocity. It comes in two forms:

- **Slip Ratio (kappa)** — Longitudinal. How much the tire is rotating
  faster or slower than the ground speed would suggest.

      kappa = (omega * R - vx) / |vx|

  Zero means free rolling. Positive means the tire is spinning (wheelspin).
  Negative means the tire is locked (braking lockup).

- **Slip Angle (alpha)** — Lateral. The angle between where the tire is
  pointing and where it is actually going.

      alpha = atan2(vy, |vx|)

  Zero means the tire is moving exactly where it points. Larger means the
  tire is sliding sideways.

## The Magic Formula (Pacejka)

In 1987, Hans Pacejka published an empirical curve fit that matches real
tire data with astonishing accuracy. It is called the "Magic Formula"
because it has no physical derivation — it is purely mathematical, but it
works.

The formula is:

    F(x) = D * sin(C * atan(B*x - E * (B*x - atan(B*x))))

Where `x` is the slip input (kappa or alpha). The coefficients are:

- **B: Stiffness.** How steeply the curve rises from zero.
- **C: Shape.** Controls the overall width and shape of the peak.
- **D: Peak.** The maximum force the tire can produce.
- **E: Curvature.** Controls the shape of the curve near the peak.

In our V1 model, `D = mu * Fz`. The curve peaks exactly at `D`, and falls
off beyond the peak slip value.

## The Peak-and-Fall Shape

This is the most important concept in tire physics:

**Grip rises, peaks, then falls as slip increases.**

If you plot Fx against slip ratio kappa, it looks like a mountain.

- At `kappa = 0`, force is zero (free rolling).
- As `kappa` increases, force rises steeply (the linear region).
- Around `kappa = 0.1` to `0.15`, the force reaches its peak.
- Beyond the peak, force *decreases*. More slip produces less force.

This is why braking lockup is dangerous. You don't just skid; you
*generate less braking force* than you would with a small amount of slip.

## The Three Tiers of Tire Physics

We built three models, each adding one more layer of physical reality.

### V1: The Pacejka Baseline

V1 models pure slip. It assumes friction is constant regardless of load,
and that longitudinal and lateral forces are independent.

- **Shape:** Peak-and-fall curve.
- **Weakness:** It cannot corner and brake at the same time. It gives full
  lateral force even when the tire is using 100% of its grip to brake.

### V2: Load Sensitivity and Combined Slip

Real tires do not scale grip linearly with load. Double the load, and you
get ~1.7x the grip. This is called **load sensitivity**.

More importantly, a tire only has **one friction budget**. If you use it
to brake, you cannot use it to turn. V2 models this with a **friction
ellipse**:

    (Fx / Fx_max)^2 + (Fy / Fy_max)^2 <= 1

If the tire tries to exceed this ellipse, both forces are scaled down
until they fit inside. This is what makes trail-braking possible, and it's
what makes a V2 car understeer when you apply throttle mid-corner.

### V3: Relaxation Length

V1 and V2 are stateless. They assume a tire responds instantly to a change
in slip.

Real tires do not do this. The rubber carcass deforms over distance. When
you turn the wheel, it takes a few centimeters of travel before the tire
produces full lateral force.

V3 adds a first-order lag to the slip values:

    d(slip)/dt = (vx / sigma) * (target_slip - current_slip)

Where `sigma` is the relaxation length (typically 0.5 m).

This lag is what makes a car's response feel like a car instead of a
video game. It gives the driver a window to countersteer during a slide,
and it smooths the transition between grip and slip.

## What We Chose Not to Model

We deliberately stopped at V3. The remaining physics models are:

- **V4 (Thermal):** Temperature changes grip. Cold tires slip, hot tires
  grip, overheated tires blister. (Faked at the frontend layer.)
- **V5 (Wear):** Tread depth changes over time. (Faked at the frontend.)

For our purposes, V3 is the highest fidelity needed for a realistic
driving simulator. The rest are game mechanics layered on top.

### V4: Thermal Modeling

Tires generate heat when they slide. Real racing tires have an optimal temperature window (usually around 80°C to 100°C). 

- If they are too cold, the rubber is hard and grip is low.
- If they are too hot, the rubber blisters and turns "greasy," dropping grip.

V4 adds a temperature state to the tire. It generates heat based on the slip power (`Force * Slip Velocity`) and cools down based on airflow (vehicle speed) and ambient track temperature.

This is what makes burnouts matter, and what forces endurance racers to manage their tires over a long stint.

## References

- `src/tire_model_v1.cpp` — Pacejka + falloff
- `src/tire_model_v2.cpp` — Load sensitivity + friction ellipse
- `src/tire_model_v3.cpp` — Relaxation length
