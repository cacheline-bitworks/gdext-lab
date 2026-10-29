#pragma once

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/variant/vector3.hpp>

#include "simulated_entity.h"
#include "tire_model.h"

using namespace godot;

// 
// VehicleBody - shared base for all vehicle tiers
// 
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
// 
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

	// Yaw inertia (kg·m²). Used by scalar-yaw tiers (T1, T2a, T2b).
	// 3000 is realistic for a 1200 kg sedan. The thin-rod formula
	// m * L² / 12 underestimates real cars by 4-5x because mass is
	// concentrated near the center, not at the bumpers.
	float m_yaw_inertia = 3000.0f;

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
	// Integrate one wheel's angular velocity. p_wheel_multiplier is 2.0 for
	// bicycle models (one virtual wheel = two physical wheels) and 1.0 for
	// per-wheel tiers (4-wheel vehicles).
	void integrate_wheel_spin(int p_index, float p_force_x,
		float p_drive_torque, float p_brake_torque, double p_dt,
		float p_wheel_multiplier);

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

	// --- Tire selection ---
	
	virtual int get_corner_count() const = 0;

	virtual void set_corner_tire(int p_index, const Ref<TireModel> &p_tire);
	virtual Ref<TireModel> get_corner_tire(int p_index) const;

	// Convenience: install the same tire at every corner.
	void set_all_tires(const Ref<TireModel> &p_tire);
};