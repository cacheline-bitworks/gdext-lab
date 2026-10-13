#pragma once

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/variant/vector3.hpp>

#include "simulated_entity.h"

using namespace godot;

// ----------------------------------------------------------------------------
// VehicleBody - shared base for all vehicle tiers
// ----------------------------------------------------------------------------
// Owns state and helpers that every tier shares:
//   - Mass and geometry
//   - Driver inputs (throttle, brake, steer)
//   - Torque and angle limits
//   - Wheel angular velocities
//   - Resistance forces (rolling + drag + low-speed damping)
//   - Wheel spin integration helper
//
// Subclasses own their orientation state (yaw scalar vs quaternion) and
// their per-tier step() logic.
//
// Registered as abstract — cannot be instantiated from GDScript.
// ----------------------------------------------------------------------------
class VehicleBody : public SimulatedEntity {
	GDCLASS(VehicleBody, SimulatedEntity)

protected:
	// --- Mass and geometry ---
	float m_mass = 1200.0f;
	float m_wheel_base = 2.6f;
	float m_track_width = 1.6f;
	float m_cg_to_front = 1.3f;
	float m_cg_to_rear = 1.3f;
	float m_wheel_radius = 0.33f;
	float m_wheel_inertia = 1.5f;

	// --- Wheels (order: FL, FR, RL, RR). T1/T2a use slots 0 and 1 only. ---
	float m_omega[4] = {0.0f, 0.0f, 0.0f, 0.0f};

	// --- Driver inputs ---
	float m_throttle = 0.0f;
	float m_brake_input = 0.0f;
	float m_steer_input = 0.0f;

	// --- Limits ---
	float m_max_engine_torque = 3000.0f;
	float m_max_brake_torque = 6000.0f;
	float m_max_steer_angle = 0.25f;

	static void _bind_methods();

	// Apply rolling resistance, aero drag, and low-speed damping to
	// m_velocity. Call near the end of step(), after force accumulation.
	void apply_resistance(double p_dt);

	// Integrate one wheel's angular velocity given the tire's longitudinal
	// force and the drive/brake torques applied to that wheel.
	void integrate_wheel_spin(int p_index, float p_force_x,
			float p_drive_torque, float p_brake_torque, double p_dt);

public:
	VehicleBody() = default;
	~VehicleBody() override = default;

	// Driver inputs
	void set_throttle(float p_val);
	float get_throttle() const;
	void set_brake(float p_val);
	float get_brake() const;
	void set_steer(float p_val);
	float get_steer() const;
};