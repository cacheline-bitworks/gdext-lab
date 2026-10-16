# Runtime Tire Selection: Swapping Tire Models Per Corner

Every vehicle tier started out with its tire type hardcoded:

    Ref<TireModelV1> m_front_tire;   // T1
    Ref<TireModelV2> m_tires[4];     // T2a, T2b, T2c

The ref type was the type. The compiler enforced it, and that was fine
for a while — no virtual dispatch overhead, no runtime checks.

But it made A/B testing impossible without a rebuild. Testing "what does
T2c feel like with V1 tires?" required editing the header, rebuilding,
running, then editing back. Three cycles for one comparison.

For the frontend phase — where designers will pick tire compounds from
a dropdown and tune relaxation length with a slider — that friction is
unacceptable.

This document explains the runtime tire-selection refactor.

## The Interface

Four methods added to `VehicleBody`:

    virtual int get_corner_count() const = 0;

    virtual void set_corner_tire(int index, const Ref<TireModel> &tire);
    virtual Ref<TireModel> get_corner_tire(int index) const;

    void set_all_tires(const Ref<TireModel> &tire);

`get_corner_count()` is pure virtual. Every concrete tier declares how
many tire slots it has — 2 for bicycle models, 4 for the rest.

`set_corner_tire` and `get_corner_tire` are virtual with default
implementations that print an error. This is deliberate: a subclass that
forgets to override them prints a warning on the first call instead of
silently doing nothing.

`set_all_tires` is a non-virtual convenience in the base. It loops over
`get_corner_count()` calling `set_corner_tire`. Every subclass gets it
for free.

## The Storage Change

Every tier's tire storage changed from a concrete type to the abstract
base:

    // Before:
    Ref<TireModelV2> m_tires[4];

    // After:
    Ref<TireModel> m_tires[4];

Now the tier doesn't know whether it's running V1, V2, V3, or a future
V4. It calls `m_tires[i]->step(in)` and virtual dispatch resolves at
runtime. Cost is 1-3 nanoseconds per call, negligible at physics rates.

## Per-Tier Defaults

The refactor did not change any defaults:

| Tier | Slots | Default tire |
|---|---|---|
| T1 | 2 | V1 |
| T2a | 2 | V2 |
| T2b | 4 | V2 |
| T2c | 4 | V2 |
| T3 | 4 | V2 (inherited) |

T1 keeps V1 because it's the "cheap traffic AI" tier. A background car
doesn't need V3's per-tire relaxation state, and T1 has no `in.dt`
plumbing to feed it. The other tiers default to V2 because that's the
tier where load sensitivity and combined slip actually matter.

## Constructor Wiring

Each tier's constructor instantiates its default tires and stores them
in the abstract-base slots:

    VehicleBodyT2c::VehicleBodyT2c() {
        for (int i = 0; i < 4; i++) {
            Ref<TireModelV2> v2;
            v2.instantiate();
            m_tires[i] = v2;
        }
        m_position.y = m_eq_height;
        m_orientation = Quaternion();
    }

The local `Ref<TireModelV2>` ensures the constructor creates a V2
instance. Assigning to the base-typed slot upcasts through Ref's
inheritance, which handles the reference counting correctly.

## GDScript Usage

From a GDScript script:

    var v = VehicleBodyT2c.new()

    # Same tire everywhere
    v.set_all_tires(TireModelV3.new())

    # Different tires per axle
    v.set_corner_tire(0, TireModelV3.new())  # FL
    v.set_corner_tire(1, TireModelV3.new())  # FR
    v.set_corner_tire(2, TireModelV2.new())  # RL
    v.set_corner_tire(3, TireModelV2.new())  # RR

    # Query what's installed
    var current = v.get_corner_tire(0)

No rebuild. No restarting the editor. Swap the tires and the next
`step()` uses the new ones.

## The Bind Method Gotcha

The `set_all_tires` method lives in the base class but is called from
GDScript through the concrete subclass. This requires the base's
`_bind_methods` to register it:

    void VehicleBody::_bind_methods() {
        // ... throttle/brake/steer bindings ...
        ClassDB::bind_method(D_METHOD("set_all_tires", "tire"),
                &VehicleBody::set_all_tires);
    }

**Writing the C++ function is not enough.** Every method GDScript should
see needs an explicit `ClassDB::bind_method(...)` call. Forgetting it
produces "Nonexistent function 'set_all_tires' in base 'VehicleBodyT2c'"
at runtime — the class is right, the method is right, but Godot has no
idea it exists.

This is the same class of error as forgetting `GDREGISTER_CLASS`. The
code compiles, links, and works from C++ — but is invisible to GDScript.

## The GDScript Type Annotation Gotcha

GDScript has no template types. When a C++ method returns `Ref<T>` or
takes `Ref<T>` as a parameter, GDScript sees a generic `Object`. You
cannot write:

    var check: Ref = vehicle.get_corner_tire(0)  # ERROR

Instead:

    var check = vehicle.get_corner_tire(0)

The variable is untyped. Calling methods on it works (dynamic dispatch
through Godot's Object system), but static type checking doesn't.

For the frontend this means a wrong-typed tire passed to
`set_corner_tire` won't be caught until `step()` is called on it. A
future hardening pass should add a runtime `is_class("TireModel")` check
in the setters.

## Testing the Refactor

`demo/tire_swap_test.tscn` runs the same T2c vehicle three times with
the same drive pattern, swapping the tire between phases:

| Phase | Tire | Yaw at t=4s |
|---|---|---|
| V2 | Load sensitivity + ellipse | -14.3° |
| V3 | + relaxation length | -22.6° |
| V1 | Pure Pacejka | -35.6° |

The three values are physically meaningful:

**V1 turns the hardest** because it has no friction ellipse. It can
produce full lateral force without spending a longitudinal budget, so
combined throttle and steering doesn't constrain it. This is the
"arcade" behavior visible in the numbers.

**V2 is the most controlled** because the ellipse forces lateral and
longitudinal forces to share a budget. Under throttle, lateral grip is
reduced. The car turns less.

**V3 sits between** — the relaxation lag delays turn-in, but once the
tire builds force it produces more yaw than V2 allows. The extra
distance traveled comes from the lag letting the car accelerate for a
longer window before lateral forces engage.

This is exactly what the feature was built for: three tires, one drive
pattern, one comparison, no rebuild.

## What This Enables

**Per-axle tire compounds.** Front V3 (responsive), rear V2 (no lag).
Mimics real race car setups where front and rear use different
compounds for balance.

**Design iteration.** A frontend dropdown can pick tire types per
corner and see the effect instantly.

**Tuning reproducibility.** A vehicle config can serialize its tire
choices as class names and relaxation lengths, then reconstruct the
exact setup on load.

**A/B testing any future tire.** V4 and V5, when they land, plug into
the same interface with no changes to vehicle code.

## What This Does Not Do

- No type checking at the interface. A wrong object passed to
  `set_corner_tire` is stored and fails later.
- No hot-swap during `step()`. Set tires between frames, not inside
  the physics loop.
- No automatic `reset()` when tires are swapped. V3's relaxation state
  carries over if you replace a V3 with another V3. Call
  `tire.reset()` explicitly if you want a fresh start.
- No per-tire parameter persistence. A config system on top of this
  would need to serialize each tire's settings (relaxation length,
  nominal load, etc.) separately.

These are future concerns. The interface is complete.

## References

- `src/vehicle_body.h` / `.cpp` — the base class interface
- `src/vehicle_body_t1.cpp`, `t2a.cpp`, `t2b.cpp`, `t2c.cpp` — per-tier
  overrides
- `demo/tire_swap_test.gd` — the three-phase demo
- `docs/patterns/03-wheel-interface.md` — the `WheelInput` /
  `WheelOutput` boundary
- `docs/patterns/09-vehicle-base-class.md` — the base-class refactor
  that made this possible
