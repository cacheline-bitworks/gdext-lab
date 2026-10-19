# Vehicle Body T2: Suspension, Load Transfer, and 6-DOF Motion

T1 is a bicycle model. It represents the car as a single point mass in a
2D plane. It can accelerate, brake, and yaw, but it has no concept of
weight transfer, body roll, or suspension.

T2 is where the car becomes a car. It moves the simulation into 3D space,
introduces four independent corners, and models the springs and dampers
that connect the chassis to the wheels.

This document explains the physics of T2 and the engineering decisions
behind the sub-steps we used to build it.

## The Core Problem: Emergent vs Analytic

In T1, if you brake, the front tires load up. But we didn't actually
simulate this. We just used a static load split, and the physics didn't
care if the car was braking or not. (In fact, the code had a placeholder
that just returned `0` for weight transfer).

Real cars do not work this way. When you brake, the chassis pitches
forward. This compresses the front springs and extends the rear springs.
The tires do not instantly see more load; the load builds up at a rate
governed by the spring rate, the damper rate, and the mass of the car.

**T2 makes load transfer emergent.** We do not tell the tires to load up
under braking. We tell the springs to compress. The tires feel the springs.

## The Degrees of Freedom

A car chassis has 6 degrees of freedom (DOF) in 3D space:
1. Translation X (forward)
2. Translation Y (heave, up/down)
3. Translation Z (lateral)
4. Rotation Pitch (nose up/down)
5. Rotation Roll (body lean left/right)
6. Rotation Yaw (heading)

T1 had 3 (X, Z, Yaw).
T2 adds Heave, Pitch, and Roll.

## The Sub-Steps: T2a, T2b, T2c

Building a full 6-DOF simulation all at once is a recipe for bugs. You
end up with a car that flies into the sky because a sign is flipped. We
built T2 in three distinct phases.

### T2a: Heave and Pitch (2 Wheels)

Before adding left and right, we added up and down, and front and back.

T2a uses the bicycle model (2 virtual wheels) but adds vertical motion.
The chassis can move up and down (heave) and rotate around the lateral
axis (pitch).

**Why this order?** Pitch is the easiest suspension effect to observe.
When you brake, the nose dives. The front suspension compresses. We can
verify the spring/damper math against a simple brake test before
introducing the complexity of left/right split.

### T2b: Add Roll (4 Wheels)

Pitch and heave only require front/rear distinction. Roll requires
left/right distinction. A bicycle model cannot roll.

T2b jumps to 4 wheels. Each wheel has its own spring and damper. The
chassis can now lean outward in a corner.

**Why this order?** Roll introduces lateral load transfer. The outer
wheels compress, the inner wheels extend. If our spring math was wrong
in T2a, this step would compound the error. By isolating heave/pitch
first, we proved the suspension model worked before multiplying it by 4.

### T2c: Full 6-DOF (Quaternion)

T2a and T2b use small-angle approximations. They track yaw, pitch, and
roll as independent scalar angles. That works perfectly as long as the
car stays mostly upright.

T2c replaces the scalar angles with a **quaternion**. It integrates the
chassis using Euler's rigid body equations. This means the car can spin
in any direction, and the gyroscopic coupling between the axes is
mathematically correct.

**Why this order?** A quaternion is a black box. If you build T2c first
and the car doesn't drive straight, is the suspension wrong? The tire
model? The quaternion math? By building the suspension in T2a and T2b
first, we knew the suspension math was correct before we moved to the
complex orientation model.

## The Trade-off: Unsprung Mass

The original design for T2 called for modeling **unsprung mass** — the
weight of the wheels, tires, and brake rotors as separate rigid bodies
connected to the chassis.

We **skipped this**.

True unsprung mass integration requires a two-body model per corner
(the chassis and the wheel moving independently). This introduces very
stiff, high-frequency oscillations (wheels bouncing on tires at 15-20 Hz).
If the physics tick rate isn't high enough, or the integration is slightly
wrong, the car will literally explode in the simulation.

Instead, T2 uses a **sprung mass model**. We assume the tire is perfectly
in contact with the ground, and the suspension compression is calculated
directly from the chassis pose.

This is a deliberate simplification. It captures 95% of the handling feel
(load transfer, roll, pitch) without the numerical instability of
simulating the wheel as a separate mass.

## The Integration Loop

Every tick, T2 performs the following:

1. **Calculate corner positions.** Based on the chassis pose (position +
   orientation) and the geometry (wheelbase, track width), find the world
   position of each suspension corner.
2. **Calculate spring compression.** How far is each corner from its
   equilibrium height?
3. **Calculate suspension force.** `F = (spring_rate * compression) +
   (damper_rate * velocity)`.
4. **Apply to chassis.** The suspension force pushes up on the chassis
   at the corner's position, creating a force and a torque.
5. **Step the tires.** Feed the calculated load into the tire model to
   get Fx and Fy.
6. **Integrate.** Use the forces and torques to update the chassis
   linear and angular velocity.

## What T2 Does Not Do

- **Anti-roll bars.** T3 handles this.
- **Ackermann steering.** T3 handles this.
- **Camber and Toe.** T3 handles this.
- **Aerodynamics.** T2c added drag and downforce, but not complex aero
  maps.

## References

- `src/vehicle_body_t2a.cpp` — Heave + Pitch (2 wheels)
- `src/vehicle_body_t2b.cpp` — Roll (4 wheels)
- `src/vehicle_body_t2c.cpp` — Full 6-DOF + Aero
- `src/vehicle_body_t3.cpp` — ARB, Ackermann, Camber, Toe
