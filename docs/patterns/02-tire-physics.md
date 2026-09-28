# Tire Physics: The Shape of the Problem

Before writing a single line of C++, we need a map. A tire model is not a
small thing — it is the single most complex subsystem in a vehicle simulator.
But the *minimal* version is small. This document describes what a tire model
actually does, what it takes in, what it produces, and what the smallest
useful implementation looks like.

The goal is not to understand every coefficient. The goal is to know what
the inputs are, what the outputs are, and where the boundary of "the
smallest thing that could work" lies.

## What a Tire Model Does

A tire model is a function. It receives the current state of a tire and
returns the forces and moments that tire generates against the ground.

    inputs  -->  [TIRE MODEL]  -->  outputs

The inputs are physical quantities. The outputs are forces in the tire's
local coordinate frame. That's it. Everything else — suspension, mass,
aerodynamics, driver input — lives outside the model. The tire model is
isolated, and that isolation is what makes it testable.

## Inputs

A tire model takes some subset of these. The minimal model uses four.

**Required by every model:**

- **Vertical load (Fz)** — how hard the tire is pressed into the ground.
  This changes everything. Tires generate more force under more load, but
  not linearly. Getting the load dependence wrong is the most common
  mistake in naive tire models.

- **Slip ratio (kappa)** — how much the tire is rotating faster or slower
  than its ground speed would suggest. Defined as:

      kappa = (omega * R - vx) / |vx|

  Where `omega` is wheel angular velocity, `R` is effective radius, and
  `vx` is longitudinal velocity. Zero means free rolling. Positive means
  the tire is spinning (wheelspin). Negative means the tire is locked
  (braking lockup).

- **Slip angle (alpha)** — the angle between the tire's pointing direction
  and its actual velocity direction. Defined as:

      alpha = atan2(vy, |vx|)

  Where `vy` is lateral velocity. Zero means the tire is moving exactly
  where it points. Larger means the tire is sliding sideways.

- **Surface friction (mu)** — a scalar representing how grippy the ground
  is. Asphalt in dry conditions is ~1.0. Ice is ~0.1. Wet asphalt varies
  from ~0.4 to ~0.9 depending on water depth.

**Used by more advanced models:**

- **Tire temperature** — affects grip. Cold tires grip poorly, optimal
  tires grip best, overheated tires lose grip.
- **Tire wear** — tread depth changes the friction curve over time.
- **Tire pressure** — affects the shape of the contact patch.
- **Camber angle** — the tilt of the wheel relative to vertical.
- **Relaxation length** — a lag term, because tires don't generate force
  instantly. Force builds over distance traveled.

For the first implementation, we ignore all of the advanced ones. Pure slip,
constant friction, no lag, no thermal.

## Outputs

Three forces and moments, expressed in the tire's local frame:

- **Fx** — longitudinal force (forward/backward). This is what accelerates
  and brakes the vehicle.
- **Fy** — lateral force (sideways). This is what turns the vehicle.
- **Mz** — aligning moment. The torque that tries to straighten the tire.
  Often ignored in simple models because its contribution to handling feel
  is subtle.

For a first version, we produce Fx and Fy. Mz comes later.

## The Friction Curve

Here is the fundamental observation that makes tire physics what it is:

**Grip rises, peaks, then falls as slip increases.**

If you plot Fx against slip ratio kappa, the curve looks like this:

    Fx
    ^
    |         ___
    |        /   \
    |       /     \
    |      /       \___
    |     /
    |    /
    |   /
    |  /
    | /
    +------------------> kappa
    0

- At kappa = 0, Fx = 0. Free rolling, no force.
- As kappa increases, Fx rises steeply. This is the "linear region."
- Around kappa = 0.1 to 0.15, Fx reaches its peak. This is peak grip.
- Beyond the peak, Fx *decreases*. More slip produces less force. This is
  the "sliding region."

The same shape applies to Fy versus slip angle alpha, with the peak around
alpha = 6 to 10 degrees depending on the tire.

**This peak-and-fall shape is what makes driving a car hard.** When you're
below the peak, everything feels predictable. When you cross it, the car
suddenly has less grip than it did a moment ago, and the driver has to
respond — this is the edge of control that racing drivers spend years
mastering.

A tire model that gets this shape wrong will feel wrong, no matter how
sophisticated the rest of the vehicle simulation is.

## Pacejka's Magic Formula

The "Magic Formula" is an empirical curve fit developed by Hans Pacejka in
the 1980s. It is not derived from physics. It is a mathematical function
that happens to match real tire data very well.

The formula has this form:

    F = D * sin(C * atan(B * x - E * (B * x - atan(B * x))))

Where x is the slip input (kappa or alpha), and:

- **B** — stiffness factor. Controls how steeply the curve rises.
- **C** — shape factor. Controls the overall shape.
- **D** — peak factor. Controls the maximum force value.
- **E** — curvature factor. Controls the shape near the peak.

For our purposes, we can simplify. The full formula has more parameters
and combined-slip terms, but the *shape* of the output curve is captured
by this four-parameter version. Getting the shape right is more important
than getting the exact coefficients right on the first try.

## The Simplified Version

A minimal tire model looks like this:

    Fx = mu * Fz * f_longitudinal(kappa)
    Fy = mu * Fz * f_lateral(alpha)

Where `f_longitudinal` and `f_lateral` are each a curve of the shape above,
scaled to peak at 1.0.

This is not accurate. Real tires have load sensitivity (grip decreases under
high load), friction ellipse constraints (Fx and Fy interact), and many
more subtleties. But this version captures the essential behavior:

- Grip rises with slip
- Grip peaks and then falls
- Both longitudinal and lateral forces follow the same general curve
- Vertical load scales the whole thing

From this, a car can be made to drive. It can be made to feel like a car
being pushed to its limits. The rest is refinement.

## What We Are Not Doing Yet

- **Load sensitivity**: real tires generate less grip per unit load as
  load increases. Ignored in v1.
- **Combined slip**: Fx and Fy interact. If you brake hard, you lose
  lateral grip. Ignored in v1.
- **Relaxation length**: real tires take time and distance to build
  forces. Ignored in v1.
- **Thermal**: temperature changes grip. Ignored in v1.
- **Wear**: tread wears, grip changes. Ignored in v1.
- **Camber**: wheel angle affects contact patch. Ignored in v1.

Every one of these is a follow-up. Each adds realism and complexity in
that order. We will layer them in as we validate the previous level.

## The Plan

1. **v1**: Pure slip Pacejka, constant friction, one tire, no load
   sensitivity. Test by plotting Fx vs kappa and Fy vs alpha against
   known curves.
2. **v2**: Add load sensitivity. Replot.
3. **v3**: Add combined slip (friction ellipse). This is where the model
   starts to feel like a real tire.
4. **v4**: Add relaxation length. Now transient behavior works.
5. **v5**: Add thermal. Now lap times change as tires warm up.

Each version is testable. Each version is measurable. We do not move to
the next version until the current one is validated.

## References

- `docs/patterns/01-frequency-lod.md` — the scheduler that will run
  these tires at variable frequencies
- `src/simulated_entity.h` — the class we will subclass for the vehicle
- `src/simulation_scheduler.h` — the system that will step the vehicle