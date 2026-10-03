# Vehicle Body T2: Unsprung Mass and Real Suspension

T1 is a bicycle model with two virtual wheels. Load transfer is computed
analytically. The chassis is a single rigid body in the horizontal plane.

T2 is the next tier. Four wheels, each with its own unsprung mass. Real
springs and dampers between the chassis and each wheel. Load transfer
emerges from the physics instead of being computed. The chassis gains
vertical, roll, and pitch degrees of freedom in addition to its existing
position and yaw.

This document describes what T2 is, what it changes, and what it does not
do. It is a design document. The code comes after.

## What T1 Got Wrong

Three things, all connected to the same root cause: T1 assumes the chassis
is a floating point mass with no vertical motion.

### Wrong: Load transfer is instantaneous

In T1, when the car brakes, we compute weight transfer with a placeholder
formula that returns zero. Even if we filled it in, the analytic formula
assumes load moves from rear to front instantly, scaled by deceleration.

Real cars do not do this. When the front brakes grab, the front springs
compress over some period of time. The rear springs extend. The load moves
forward at a rate governed by the spring rate, damper rate, and the mass
of the chassis. If the brakes are released, the load moves back, and the
motion has a natural frequency (bump-and-rebound).

This matters for feel. The rate at which load transfers determines how
the car responds to a sudden brake input. Instant transfer feels like the
car has no suspension. Real transfer has a specific frequency that the
driver can feel.

### Wrong: Unsprung mass does not exist

The wheel, tire, brake rotor, and part of the suspension arm form an
unsprung mass that moves with the wheel. Its weight matters. A car with
heavy wheels responds differently to bumps than one with light wheels.

In T1, the wheel is a point. It has angular velocity but no mass, no
position relative to the chassis, no effect on the vehicle's behavior.

### Wrong: Four wheels are not four points

T1 uses a bicycle model. This is a simplification that works for basic
behavior, but it cannot model roll. When the car corners, the outer
wheels load up and the inner wheels unload. A bicycle model with two
central wheels cannot represent this.

Real cars have four corners, and the difference in load between left and
right is what creates roll and generates the outside-tire grip that keeps
the car from sliding.

## The Two-Mass Model

T2 uses what is called a two-mass model. Two body types:

**Sprung mass** - the chassis, engine, driver, fuel. Everything supported
by the springs. Has mass, center of gravity, and full 6-DOF motion.

**Unsprung mass** - one per corner. The wheel, tire, brake disc, part of
the suspension. Each unsprung mass is constrained to move only vertically
(up and down relative to the chassis). It cannot yaw or roll or pitch on
its own.

The connection between them is the suspension. For each corner:

    [ chassis (sprung) ]
            |
            |  spring + damper
            |
    [ wheel (unsprung) ]
            |
            |  tire contact patch
            |
    [ ground ]

The wheel moves up and down. The spring and damper link it to the
chassis. When the wheel hits a bump, it moves up, compresses the spring,
which pushes back on both the wheel and the chassis.

## New State

T1's state was: position (x, z), velocity (vx, vz), yaw, yaw rate.

T2's state adds, per corner:
- Suspension compression (current displacement from rest, in meters)
- Suspension velocity (rate of compression, in m/s)
- Unsprung mass height (position of the wheel relative to the chassis)

And per chassis:
- Vertical position (heave, in meters)
- Vertical velocity
- Roll angle (rotation around the longitudinal axis)
- Roll rate
- Pitch angle (rotation around the lateral axis)
- Pitch rate

That is six extra chassis values and twelve extra per-corner values. T2
has significantly more state than T1.

## New Parameters

Per corner:

- **Spring rate** (N/mm) - how stiff the spring is. Road cars use 20-40
  N/mm. Race cars use 80-150 N/mm.
- **Damper bump rate** (N/(m/s)) - resistance to compression. Usually
  lower than rebound, allowing bumps to be absorbed.
- **Damper rebound rate** (N/(m/s)) - resistance to extension. Usually
  2-3x the bump rate, to control the chassis returning from bumps.
- **Bump stop engagement point** (mm) - how far into compression the
  bump stop engages. Typically at 70-80% of total travel.
- **Bump stop rate** (N/mm) - much stiffer than the main spring, prevents
  bottoming out.
- **Travel limit** (mm) - total suspension travel. Road cars ~150mm,
  race cars ~50mm.
- **Unsprung mass** (kg) - weight of the wheel assembly. ~30-60 kg per
  corner for road cars, ~20-30 kg for race cars.

## The Integration Loop

T2's step function is significantly more complex than T1's. The conceptual
order:

1. **Update world-frame state** - same as T1, using forces from last step.

2. **Compute suspension state per corner** - for each corner, calculate
   the current compression from the difference between the chassis pose
   and the wheel position.

3. **Compute suspension forces** - spring force, damper force, bump stop
   force. Sum them to get the total force the suspension transmits between
   the chassis and the wheel.

4. **Compute tire loads** - the vertical load on each tire equals the
   suspension force (assuming the tire is on the ground).

5. **Step tires** - same as T1, but now with per-corner inputs.

6. **Sum forces and moments** - four tire forces (not two) act at four
   contact patches. Compute the total force on the chassis and the total
   moment.

7. **Integrate 6-DOF chassis motion** - position, velocity, orientation,
   angular velocity all update.

8. **Integrate unsprung masses** - each wheel's vertical position updates
   based on the net force on it (suspension force minus tire vertical
   force).

The order matters. Suspension forces depend on the current chassis pose.
Tire forces depend on suspension forces. Chassis acceleration depends on
tire forces. Each step feeds the next.

This is a tightly coupled system. The integration timestep must be small
enough that the fastest oscillation (usually the unsprung mass on the
tire, which can oscillate at 15-20 Hz) is resolved correctly. A 100 Hz
physics rate is marginal. 200 Hz is safer.

## What T2 Does Not Do

- **Anti-roll bars** - a torsion spring coupling left and right suspension
  on the same axle. Reduces body roll. Common on all performance cars.
- **Ackermann steering** - inner wheel turns more than outer wheel in a
  corner. Requires per-front-wheel steering geometry.
- **Camber** - the tilt of the wheel relative to vertical. Affects contact
  patch shape and lateral grip.
- **Toe** - static angle of the wheels relative to straight ahead.
- **Caster and kingpin inclination** - steering axis geometry effects.
- **Tire thermal** - V3 territory.
- **Aerodynamic downforce** - separate subsystem.
- **Anti-roll bar coupling** - T3.

T2 is where suspension *exists*. T3 is where it becomes a race car.

## Scope and Sub-Steps

T2 is a multi-session project. The honest plan:

**T2a: 1-DOF vertical per wheel.** Chassis moves vertically only (no roll,
no pitch). Suspension connects chassis to wheels. Wheels move vertically
only. Load transfer is emergent but only in the vertical direction.

**T2b: Add roll.** Chassis gains roll DOF. Left and right suspension
couple through the chassis. Lateral load transfer emerges.

**T2c: Add pitch.** Chassis gains pitch DOF. Front and rear suspension
couple through the chassis. Longitudinal load transfer emerges.

**T2d: Full 6-DOF.** Three rotations plus three translations. Chassis
integrated as a rigid body with full inertia tensor. This is the final
form.

Each sub-step is testable on its own. Each is a natural stopping point.
Skipping ahead to T2d means tuning a complex system without intermediate
reference points.

## Testing

Each sub-step has a clear test.

**T2a:** Drop the chassis from a small height with no horizontal motion.
It should bounce on the springs and settle after a few oscillations. If
it settles too fast, dampers are too stiff. If it never settles, dampers
are too soft.

**T2b:** Apply a steady lateral acceleration (constant yaw rate). The
chassis should roll to a steady angle. The outer wheels should carry more
load than inner wheels. The ratio of load transfer to roll angle should
match the spring rates.

**T2c:** Apply a steady brake input. The chassis should pitch forward.
Front springs compress, rear springs extend. Load transfer should be
proportional to deceleration.

**T2d:** Drive a lap. Compare against T1. T2 should feel more planted,
more predictable, and should corner with more consistent grip. If it does
not, something is wrong.

## Summary

T2 is not "T1 with more parameters." It is a different model. T1 is a
point mass with a bicycle wheel model. T2 is a two-body model with four
wheels and real suspension. The physics of what makes a car feel like a
car - weight transfer, load sensitivity, suspension response - emerge
from T2's structure rather than being approximated by formulas.

This is the tier where the vehicle starts driving like a vehicle.

## References

- docs/patterns/04-tire-model-v2.md - the tire model T2 pairs with
- docs/patterns/03-wheel-interface.md - the interface T2 must preserve
- src/vehicle_body_t1.cpp - the T1 implementation T2 replaces
- src/tire_model.h - the tire interface T2 calls into