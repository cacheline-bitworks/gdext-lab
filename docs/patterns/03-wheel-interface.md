# The Wheel Interface: Where Drivetrain Meets Tire

This document defines the boundary between two subsystems that will live in
separate files, evolve at different rates, and be tested independently:

- The **drivetrain** (engine, clutch, gearbox, differential)
- The **tire model** (the math that converts wheel state into forces)

The wheel is where they meet. It is the smallest surface area in the entire
vehicle simulation, and getting it right is the difference between a
simulator that composes cleanly and one that turns into spaghetti as it
grows.

## The Physical Reality

The drivetrain does not push the car. It applies **torque** to the wheel's
axle. The tire does not receive force. It receives **torque**, spins at some
**angular velocity**, and generates **force** against the ground.

The wheel itself stores angular velocity and integrates the net torque:

    wheel_angular_acceleration =
        (drivetrain_torque - tire_reaction_torque - brake_torque) / wheel_inertia

Where `tire_reaction_torque = Fx * wheel_radius`.

Two torques fight for control of the wheel. The drivetrain pushes it one
way. The tire pushes it the other. The wheel's rotation is the negotiation
between them.

## The Interface: Two Structs

Everything that crosses the boundary fits in two small structs. No pointers,
no references, no hidden state. Just data.

### WheelInput

What the drivetrain (and brakes, and ground) tell the tire each step:

    struct WheelInput {
        float angular_velocity;   // rad/s, current wheel spin
        float torque;             // Nm, net drive torque applied this step
        float brake_torque;       // Nm, always opposes rotation
        float vertical_load;      // N, how hard the tire is pressed down
        float friction_coefficient; // dimensionless, surface grip
        float wheel_radius;       // m, for converting torque <-> force
    };

That's six floats. Twenty-four bytes. Everything the tire needs to know
about the outside world.

### WheelOutput

What the tire tells the rest of the world each step:

    struct WheelOutput {
        float force_x;    // N, longitudinal (forward/backward)
        float force_y;    // N, lateral (left/right)
        float moment_z;   // Nm, aligning torque (may be 0 in v1)
        float slip_ratio; // dimensionless, for telemetry
        float slip_angle; // rad, for telemetry
    };

Five floats. Twenty bytes.

Note that `slip_ratio` and `slip_angle` are outputs, not inputs. The tire
computes them from the wheel's angular velocity and the vehicle's ground
velocity. The tire model needs access to both to compute slip.

Wait. We just hit a design problem.

## Where Does Slip Come From?

Slip requires two pieces of information:

1. The wheel's current spin rate (available in WheelInput)
2. The vehicle's velocity at the contact patch (NOT in WheelInput)

Slip ratio is:

    kappa = (omega * R - vx) / |vx|

Slip angle is:

    alpha = atan2(vy, |vx|)

Both need `vx` and `vy`, the longitudinal and lateral velocities at the
contact patch. Those come from the vehicle's rigid body state, not from
the drivetrain.

We have two options:

**Option A: Add velocity to WheelInput.** The vehicle computes velocities
in the tire's local frame and hands them in. This keeps the tire model
pure math with no knowledge of the vehicle.

**Option B: Pass the vehicle state to the tire.** The tire reads position,
rotation, and velocity from the vehicle object directly. Simpler, but
couples the tire to the vehicle's data structure.

For v1 and beyond, we choose **Option A**. The tire model must be testable
without a vehicle. A test should look like:

    WheelInput in;
    in.angular_velocity = 50.0f;
    in.torque = 500.0f;
    in.vertical_load = 4000.0f;
    in.friction_coefficient = 1.0f;
    in.wheel_radius = 0.33f;
    in.velocity_longitudinal = 30.0f;  // <-- new
    in.velocity_lateral = 0.0f;        // <-- new

    WheelOutput out = tire.step(in);

No vehicle needed. The tire model is a pure function of its inputs.

## Revised WheelInput

    struct WheelInput {
        float angular_velocity;        // rad/s
        float torque;                  // Nm
        float brake_torque;            // Nm
        float vertical_load;           // N
        float friction_coefficient;    // dimensionless
        float wheel_radius;            // m
        float velocity_longitudinal;   // m/s, at contact patch
        float velocity_lateral;        // m/s, at contact patch
    };

Eight floats. Thirty-two bytes. Still trivial to pass around.

## The Versioning Strategy

Tire models evolve. v1 is pure Pacejka with constant friction. v2 adds
load sensitivity. v3 adds combined slip. v4 adds relaxation length. v5
adds thermal. Each is a substantial step up in fidelity and cost.

We do not overwrite v1 when writing v2. We keep them as **sibling classes
that share a common interface**:

    class TireModel : public RefCounted {
        GDCLASS(TireModel, RefCounted)
    public:
        virtual WheelOutput step(const WheelInput &input) = 0;
    };

    class TireModelV1 : public TireModel { ... };
    class TireModelV2 : public TireModel { ... };
    class TireModelV3 : public TireModel { ... };

Each version is registered with Godot independently. GDScript chooses
which to instantiate:

    var simple_tire := TireModelV1.new()   # AI traffic
    var hero_tire := TireModelV3.new()     # player car

## Why Sibling Classes Instead of One Class With a "Version" Flag

Three reasons:

**Zero cost for simple paths.** An AI car running `TireModelV1` does not
pay for load sensitivity checks it never uses. Branches exist in a
version-flagged class; they do not exist when versions are separate
classes.

**Clean benchmarking.** Run the same inputs through v1, v2, v3, and
compare. Because they are separate classes, iterating over them and
diffing outputs is straightforward. This is how you validate that v2
is actually better than v1, not just more complicated.

**No regression risk.** v1 keeps working exactly as it did. You never
refactor it. New features go in new classes. Old ones are frozen. If v3
turns out to have a bug, you can swap back to v2 for a release without
touching v2's code.

The cost is small code duplication across versions. That is the correct
trade.

## The Abstract Base

The base class `TireModel` exists only to define the interface. It has
no state and no implementation. Its single job is to guarantee that every
version has a `step()` method that takes a `WheelInput` and returns a
`WheelOutput`.

    class TireModel : public RefCounted {
        GDCLASS(TireModel, RefCounted)

    protected:
        static void _bind_methods();

    public:
        TireModel() = default;
        ~TireModel() override = default;

        virtual WheelOutput step(const WheelInput &input) = 0;
    };

The `= 0` makes `step` **pure virtual**. The base class cannot be
instantiated. Only subclasses can. This is deliberate: there is no such
thing as "a generic tire model" with no algorithm behind it.

## The Wheel Does Not Live Here

`WheelInput` and `WheelOutput` are data. The wheel that produces them is
part of the vehicle, not part of the tire model. The vehicle:

1. Computes `WheelInput` from drivetrain output and body velocity
2. Calls `tire_model->step(input)`
3. Applies `output.force_x` and `output.force_y` to the vehicle body
4. Feeds `output` back to the wheel dynamics (wheel spin integration)

The tire model is stateless between calls. Given the same inputs, it
returns the same outputs. This is what makes it testable and what makes
it safe to compose.

Later versions will break this purity — relaxation length, for instance,
needs to remember the previous state. But for v1, statelessness is a
property we want to preserve.

## What We Are Not Defining Today

- The exact Pacejka coefficient values (they come from published tire data)
- The math inside `step()` (that is v1's job)
- The wheel spin integrator (that belongs to the vehicle, later)
- The brake torque model (drivetrain, later)
- The differential (drivetrain, later)

Today is only the interface. The rest builds on top.

## Summary

- The wheel is a torque-to-force converter.
- `WheelInput` and `WheelOutput` are plain data structs, eight and five
  floats respectively.
- Tire models are versioned as sibling classes with a shared abstract base.
- The tire model is a pure function for v1.
- All future versions extend the same interface without breaking it.

## References

- `docs/patterns/01-frequency-lod.md` — the scheduler that will step
  vehicles at variable rates
- `docs/patterns/02-tire-physics.md` — the physical content of a tire
  model
- `src/simulated_entity.h` — the class vehicles will subclass
