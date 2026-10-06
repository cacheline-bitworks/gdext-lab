#pragma once

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/variant/vector3.hpp>

#include "simulated_entity.h"
#include "tire_model.h"
#include "tire_model_v2.h"


using namespace godot;

// ----------------------------------------------------------------------------
// SuspensionConfig - pure data struct
// ----------------------------------------------------------------------------
// All tunable suspension values live here. No logic, no dependencies.
// When the frontend arrives, this migrates to a Godot Resource with
// mechanical changes only.
// ----------------------------------------------------------------------------
struct SuspensionConfig {
	// Spring rates in N/m (note: real-world spec sheets often use N/mm,
	// multiply by 1000)
	float spring_rate_front = 60000.0f;   // 60 N/mm
	float spring_rate_rear = 70000.0f;    // 70 N/mm

	// Damper rates in N/(m/s). Asymmetric: bump (compression) is usually
	// softer than rebound to allow bumps to be absorbed.
	float damper_bump_front = 4000.0f;
	float damper_rebound_front = 8000.0f;
	float damper_bump_rear = 4500.0f;
	float damper_rebound_rear = 9000.0f;

	// Reference for the tire's nominal load (used for load sensitivity)
	float nominal_load = 4000.0f;
};

// ----------------------------------------------------------------------------
// VehicleBodyT2a - heave + pitch on bicycle model
// ----------------------------------------------------------------------------
// First sub-tier of T2. Adds two chassis degrees of freedom:
//
//   - Heave: chassis moves up and down as a rigid body
//   - Pitch: chassis rotates around the lateral axis (nose up/down)
//
// Two suspension corners (front, rear) with independent spring/damper.
// Wheels are assumed to be at ground level.
//
// What emerges from this:
//   - Front/rear load transfer under braking and acceleration
//   - Nose dive on braking, squat on acceleration
//   - Suspension bounce oscillation at the natural frequency
//
// What does NOT exist yet:
//   - Roll (left/right load split) - that's T2b
//   - Unsprung mass as a separate body - that's T2c/T2d
//   - Bump stops, anti-roll bars
// ----------------------------------------------------------------------------
class VehicleBodyT2a : public SimulatedEntity {
	GDCLASS(VehicleBodyT2a, SimulatedEntity)

	// --- Chassis state ---
	float m_mass = 1200.0f;                // kg
	float m_cg_height = 0.5f;              // m above ground
	float m_yaw = 0.0f;                    // rad
	float m_yaw_rate = 0.0f;               // rad/s
	Vector3 m_velocity;                    // m/s, world frame

	// New in T2a: vertical and pitch state
	float m_heave = 0.0f;                  // m, chassis vertical offset from equilibrium
	float m_heave_velocity = 0.0f;         // m/s
	float m_pitch = 0.0f;                  // rad, positive = nose up
	float m_pitch_rate = 0.0f;             // rad/s

	// --- Geometry ---
	float m_wheel_base = 2.6f;
	float m_cg_to_front = 1.3f;
	float m_cg_to_rear = 1.3f;
	float m_wheel_radius = 0.33f;
	float m_wheel_inertia = 1.5f;

	// --- Pitch inertia (approximated) ---
	float m_pitch_inertia = 1800.0f;       // kg·m², rough for a 1200 kg car

	// --- Suspension config ---
	SuspensionConfig m_suspension;

	// --- Wheels ---
	float m_front_omega = 0.0f;
	float m_rear_omega = 0.0f;
	Ref<TireModelV2> m_front_tire;
	Ref<TireModelV2> m_rear_tire;

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
	VehicleBodyT2a();
	~VehicleBodyT2a() override = default;

	// Driver inputs
	void set_throttle(float p_val);
	float get_throttle() const;
	void set_brake(float p_val);
	float get_brake() const;
	void set_steer(float p_val);
	float get_steer() const;

	// State accessors
	float get_yaw() const;
	float get_yaw_rate() const;
	float get_speed() const;
	Vector3 get_velocity() const;

	// T2a-specific state - for testing and debugging
	float get_heave() const;
	float get_heave_velocity() const;
	float get_pitch() const;
	float get_pitch_rate() const;

	// Suspension diagnostics
	float get_front_compression() const;
	float get_rear_compression() const;
	float get_front_load() const;
	float get_rear_load() const;

	void step(double p_delta) override;
};