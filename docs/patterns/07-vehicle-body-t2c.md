# VehicleBodyT2c: Full 6-DOF Rigid Body Chassis

T2a and T2b track the chassis as three independent scalar angles — yaw,
pitch, and roll — plus a vertical offset for heave. That works while the
angles stay small. It breaks the moment the body rotates enough that the
axes interact, or when the body rotates past 90 degrees in any axis.

T2c replaces that approximation with a true rigid body. Position,
orientation (as a quaternion), linear velocity, angular velocity, and a
diagonal inertia tensor. Torques integrate through Euler's equations.
The body can rotate in any direction without losing correctness.

This document explains the design. The code is in
`src/vehicle_body_t2c.h/.cpp`.

## Why Quaternions

A rotation in 3D can be represented in several ways. Euler angles (yaw,
pitch, roll) are intuitive but have gimbal lock — at certain orientations
two axes collapse into one and one degree of freedom is lost. Rotation
matrices work but accumulate numerical drift.

Quaternions avoid both problems. They're a four-component representation
of rotation with no singularities and stable integration. Godot's
`Quaternion` class is the natural choice.

Conventions used here:

- Body frame: X forward, Y up, Z right
- Right-handed coordinate system
- Positive yaw rotates the body toward world -Z (matches Godot)
- `m_orientation` rotates from body frame to world frame

## The Inertia Tensor

A rigid body resists rotation differently around different axes. The
distribution of mass determines how much torque is needed for a given
angular acceleration. This is captured by the inertia tensor.

For our purposes the tensor is diagonal in the body frame:

    m_inertia = Vector3(Ixx, Iyy, Izz)

Where:

- Ixx is inertia about the roll axis (roughly m * track² / 4)
- Iyy is inertia about the yaw axis (roughly m * wheelbase² / 4)
- Izz is inertia about the pitch axis (roughly m * wheelbase² / 4)

For a 1200 kg sedan, typical values are approximately (400, 3000, 1800)
kg·m². Yaw inertia is dominant because the car is longest in that axis.

The inverse is computed on demand via `inverse_inertia()`. It's derived
from `m_inertia`, never stored separately, so the two can never go out of
sync.

## Euler's Rigid Body Equations

A point mass obeys `F = ma`. A rigid body obeys a pair of equations: one
for translation, one for rotation.

Translation:

    a = F / m
    v += a * dt
    position += v * dt

Rotation is more subtle because angular momentum lives in the body frame
but forces are computed in world frame:

    torque_body = orientation.inverse() * torque_world
    I_omega = I * omega
    gyro = omega × I_omega
    domega = (torque_body - gyro) * inverse_inertia
    omega += domega * dt

The `gyro` term is what makes the equations work in three dimensions. It
represents the tendency of a rotating body to precess — think of a
spinning top wobbling. For a car doing normal driving it's negligible.
For a car doing a barrel roll it dominates.

Without the gyroscopic term, a body rotating about an axis that isn't a
principal axis of inertia would slowly drift away from physical reality.

## Orientation Integration

After integrating angular velocity, we need to update orientation. The
formula:

    delta_rot = omega * dt
    angle = |delta_rot|
    axis = delta_rot / angle
    dq = Quaternion(axis, angle)
    orientation = (orientation * dq).normalized()

The `dq` is an incremental rotation about the current angular velocity
vector. Right-multiplying by it rotates in the body frame, which is
correct because `omega` is body-frame angular velocity.

Normalization is required every tick because quaternion multiplication
introduces numerical drift. Skip it and orientation eventually becomes
non-unitary, at which point all the rotation math is subtly wrong.

## Force and Torque Accumulation

Each substep of `step()`:

1. Start with gravity on the CG (no torque contribution)
2. Add aerodynamic forces (drag + downforce)
3. For each corner:
   - Compute the corner's world position from body offset and orientation
   - Compute suspension compression and velocity
   - Compute spring and damper force (vertical only)
   - Apply the suspension force at the corner position
   - Compute contact point velocity (world frame)
   - Transform to body frame
   - Build WheelInput and step the tire model
   - Transform the tire force from tire-local back to world
   - Apply at the contact point

Torque is accumulated as `r × F` where `r` is the vector from the CG to
the point of application. Same formula for suspension, tire forces, and
aero.

## The Virtual Hook Pattern

T3 inherits from T2c. To add anti-roll bars, Ackermann steering, and
camber without modifying T2c, T2c exposes three virtual methods:

- `extra_corner_force(corner, compression, opposite_compression)` — return
  extra vertical force at a corner (used by ARB)
- `corner_steer_angle(corner, base_steer)` — override the per-wheel steer
  angle (used by Ackermann and toe)
- `corner_camber_angle(corner)` — return camber for a corner

T2c's default implementations do nothing or return the input unchanged.
T3 overrides all three. T2c's `step()` calls them without knowing anything
about subclass behavior.

This is the correct way to extend a physics tier. Every future feature
that adds per-corner behavior fits this pattern.

## What T2c Does Not Do

- No engine or drivetrain model — torque is applied directly to wheels
- No thermal effects on tires
- No suspension geometry beyond a vertical spring/damper
- No frame-level aerodynamics — downforce is a single force at the CG
- No soil or wet-weather handling

These are all future concerns. T2c is the foundation they'd sit on.

## Relation to Other Tiers

T2c is the direct ancestor of T3, which adds the geometry and ARB hooks
that make a car feel like a race car. T2c is also the highest tier that
remains stateless across frames — T3's ARB computation is per-tick, not
persistent.

For context:

| Tier | DOF | Body | Wheels | Extras |
|---|---|---|---|---|
| T1 | 3 | Scalar | 2 | None |
| T2a | 5 | Scalar | 2 | Suspension |
| T2b | 5 | Scalar | 4 | Roll |
| T2c | 6 | Quaternion | 4 | Aero, hooks |
| T3 | 6 | Quaternion | 4 | ARB, Ackermann, camber, toe |

## References

- `src/vehicle_body_t2c.h` / `.cpp` — the implementation
- `src/vehicle_body_t3.h` / `.cpp` — subclass using the hooks
- `docs/patterns/05-vehicle-body-t2.md` — the tier design overview
- Godot `Quaternion` class documentation
