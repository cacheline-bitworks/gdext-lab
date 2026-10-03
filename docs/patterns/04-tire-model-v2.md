# Tire Model V2: Load Sensitivity and Combined Slip

Tire model V2 is the first meaningful upgrade over V1. It adds two physics
effects that separate a believable tire from a curve-fit approximation:

1. Load sensitivity - grip does not scale linearly with vertical load
2. Combined slip - longitudinal and lateral forces share a friction budget

Both effects are essential for realistic handling. Without them, a car cannot
behave correctly under aggressive braking, cornering, or throttle-on exit.

## Interface

V2 inherits from `TireModel`. Same `WheelInput`, same `WheelOutput`. Same
`step()` signature. Callers do not know which version they are talking to.

This is the whole point of the sibling-class pattern. V2 can be swapped in
wherever V1 is used without any code changes elsewhere.

## What V1 Gets Wrong

V1 uses the single-sinusoid Pacejka formula and scales the peak by
`mu * Fz`. Two things are wrong with this.

### Wrong: Grip scales linearly with load

If you double the vertical load on a tire, V1 returns double the force. Real
tires do not. They return somewhere between 1.6x and 1.8x. The extra load
does not produce proportional grip.

This matters because weight transfer is the entire game of racing physics.
When a car brakes, weight shifts forward, loading the front tires. If grip
scaled linearly, more load would always mean more grip, and stiff cars would
always corner faster. Reality disagrees. Softer cars corner faster in the
dry because they transfer less load and lose less grip to load sensitivity.

### Wrong: Fx and Fy are independent

V1 computes longitudinal and lateral force in isolation. A tire with 90
percent of its grip used for braking will still produce 100 percent of its
lateral force in V1. This is physically impossible. A tire has one friction
budget, and all forces spend from it.

This is why V1 cars spin out under trail-braking. The front tires are
simultaneously asked to brake and turn at full capacity, and V1 gives them
both.

## Load Sensitivity

The friction coefficient is a function of vertical load:

    mu_effective = mu_0 * (1 - k_load * (Fz / Fz_nominal - 1))

Where:
- mu_0 is the reference friction at nominal load
- Fz_nominal is the load at which mu_0 was measured
- k_load is the load sensitivity coefficient

Typical values:
- Road tire: k_load = 0.10 to 0.15
- Performance tire: k_load = 0.15 to 0.20
- Racing slick: k_load = 0.20 to 0.30

The effect is symmetric. Lighter loads get slightly more grip per unit load.
Heavier loads get less. This is why unloading a tire helps it maintain grip,
and overloading a tire kills it.

## Combined Slip

The friction ellipse constrains the total force a tire can produce:

    (Fx / Fx_max)^2 + (Fy / Fy_max)^2 <= 1

Where Fx_max and Fy_max are the peak forces the tire could produce in pure
longitudinal and pure lateral conditions respectively.

If the sum exceeds 1, the tire is asking for more grip than it has. Both
forces must be scaled down:

    scale = 1 / sqrt((Fx / Fx_max)^2 + (Fy / Fy_max)^2)
    Fx_out = Fx * scale
    Fy_out = Fy * scale

This is called saturation and it is the single most important effect for
realistic handling feel.

### Why the ellipse and not a circle

Because tires have different limits in different directions. A typical tire
produces more lateral force than longitudinal force at the same slip angle
and slip ratio. The ellipse reflects this anisotropy.

Some sims use a circle with equal limits. It works and is simpler. The
ellipse is more accurate.

## Implementation Notes

Both effects act on the same quantities. The correct order is:

1. Compute mu_effective from load sensitivity
2. Compute Fx and Fy using mu_effective instead of mu_0
3. Apply the friction ellipse constraint to the results

Step 1 modifies the peak. Step 3 clamps the total.

This order matters. If you apply load sensitivity after the ellipse, you
get inconsistent results. If you apply the ellipse before load sensitivity,
you under-constrain.

## What V2 Does Not Do

- Relaxation length (forces still respond instantly to slip changes)
- Thermal effects (grip does not vary with temperature)
- Wear (grip does not change over time)
- Camber (no account for wheel angle)
- Transient response (steady-state only)

These are V3 and beyond.

## Testing

The test is a side-by-side comparison against V1.

1. Plot Fx vs slip ratio at three loads (2000N, 4000N, 6000N)
   - V1 curves are parallel, scaled by load
   - V2 curves peak at progressively lower mu, showing load sensitivity

2. Plot Fy vs slip angle with varying Fx present
   - V1 returns full lateral force regardless of Fx
   - V2 reduces lateral force as Fx rises, showing the friction ellipse

If both plots show the differences above, V2 is correct.

## In the Vehicle

When VehicleBodyT2 lands, V2 tires will be the difference between "the car
spins out under braking" and "the car rotates predictably through the corner."
The combined slip model is what makes trail braking possible, throttle-on
oversteer possible, and understeer at the limit possible.

V1 is a proof of concept. V2 is where the tire model actually behaves like a
tire.

## References

- docs/patterns/02-tire-physics.md - physical background
- docs/patterns/03-wheel-interface.md - the interface V2 inherits
- src/tire_model.h - TireModel base class