#pragma once

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/variant/vector3.hpp>

#include "simulated_entity.h"
#include "tire_model.h"
#include "tire_model_v2.h"

using namespace godot;

// ----------------------------------------------------------------------------
// CornerConfig - per-corner suspension parameters
// ----------------------------------------------------------------------------
// Four instances exist per vehicle (one per corner). Each is pure data.
// ----------------------------------------------------------------------------
struct CornerConfig {
	float spring_rate = 65000.0f;         // N/m
	float damper_bump = 4200.0f;          // N/(m/s)
	float damper_rebound = 8500.0f;       // N/(m/s)
	float static_load_fraction = 0.25f;   // fraction of mass on this corner at rest
};

// ----------------------------------------------------------------------------
// VehicleBodyT2b - four wheels + roll on the bicycle pitch/heave model
// ----------------------------------------------------------------------------
// Extends T2a with:
//   - Four suspension corners instead of two
//   - Roll degree of freedom (chassis leans in corners)
//   - Emergent lateral load transfer
//
// Still uses a single rigid chassis with heave + pitch + roll. Full 6-DOF
// integration is T2c.
// ----------------------------------------------------------------------------
class VehicleBodyT2b : public SimulatedEntity {
	GDCLASS(VehicleBodyT2b, SimulatedEntity)

	// --- Chassis ---
	float m_mass = 1200.0f;
	float m_cg_height = 0.5f;
	float m_yaw = 0.0f;
	float m_yaw_rate = 0.0f;
	Vector3 m_velocity;

	// Heave / pitch / roll state
	float m_heave = 0.0f;
	float m_heave_velocity = 0.0f;
	float m_pitch = 0.0f;
	float m_pitch_rate = 0.0f;
	float m_roll = 0.0f;
	float m_roll_rate = 0.0f;

	// --- Geometry ---
	float m_wheel_base = 2.6f;
	float m_track_width = 1.6f;
	float m_cg_to_front = 1.3f;
	float m_cg_to_rear = 1.3f;
	float m_wheel_radius = 0.33f;
	float m_wheel_inertia = 1.5f;

	// --- Inertia approximations ---
	float m_pitch_inertia = 1800.0f;
	float m_roll_inertia = 400.0f;

	// --- Suspension ---
	CornerConfig m_front_left;
	CornerConfig m_front_right;
	CornerConfig m_rear_left;
	CornerConfig m_rear_right;

	// --- Wheels ---
	// Order: FL, FR, RL, RR
	float m_omega[4] = {0.0f, 0.0f, 0.0f, 0.0f};
	Ref<TireModelV2> m_tires[4];

	// --- Driver inputs ---
	float m_throttle = 0.0f;
	float m_brake_input = 0.0f;
	float m_steer_input = 0.0f;
	float m_max_engine_torque = 1500.0f;
	float m_max_brake_torque = 1200.0f;
	float m_max_steer_angle = 0.25f;

protected:
	static void _bind_methods();

public:
	VehicleBodyT2b();
	~VehicleBodyT2b() override = default;

	void set_throttle(float p_val);
	float get_throttle() const;
	void set_brake(float p_val);
	float get_brake() const;
	void set_steer(float p_val);
	float get_steer() const;

	float get_yaw() const;
	float get_yaw_rate() const;
	float get_speed() const;
	Vector3 get_velocity() const;

	// Roll state - new in T2b
	float get_roll() const;
	float get_roll_rate() const;

	// Per-corner diagnostics
	// Index: 0 = FL, 1 = FR, 2 = RL, 3 = RR
	float get_corner_load(int p_index) const;
	float get_corner_compression(int p_index) const;

	void step(double p_delta) override;
};