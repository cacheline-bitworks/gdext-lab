#pragma once

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/variant/quaternion.hpp>
#include <godot_cpp/variant/vector3.hpp>

#include "simulated_entity.h"
#include "tire_model.h"
#include "tire_model_v2.h"

using namespace godot;

struct CornerConfigT2c {
	float spring_rate = 65000.0f;
	float damper_bump = 4200.0f;
	float damper_rebound = 8500.0f;
};

// ----------------------------------------------------------------------------
// VehicleBodyT2c - full 6-DOF rigid body chassis
// ----------------------------------------------------------------------------
// See docs/patterns/05-vehicle-body-t2.md for the design.
// ----------------------------------------------------------------------------
class VehicleBodyT2c : public SimulatedEntity {
	GDCLASS(VehicleBodyT2c, SimulatedEntity)

	// --- Chassis ---
	float m_mass = 1200.0f;
	float m_eq_height = 0.5f;
	Vector3 m_velocity;
	Vector3 m_angular_velocity;
	Quaternion m_orientation;

	Vector3 m_inertia = Vector3(400.0f, 3000.0f, 1800.0f);
	Vector3 m_inv_inertia = Vector3(1.0f / 400.0f, 1.0f / 3000.0f, 1.0f / 1800.0f);

protected:
	// --- Geometry (subclasses need these for steering, aero, etc.) ---
	float m_wheel_base = 2.6f;
	float m_track_width = 1.6f;
	float m_cg_to_front = 1.3f;
	float m_cg_to_rear = 1.3f;
	float m_wheel_radius = 0.33f;
	float m_wheel_inertia = 1.5f;

private:
	// --- Suspension (per corner) ---
	CornerConfigT2c m_front_left;
	CornerConfigT2c m_front_right;
	CornerConfigT2c m_rear_left;
	CornerConfigT2c m_rear_right;

	// --- Wheels (order: FL, FR, RL, RR) ---
	float m_omega[4] = {0.0f, 0.0f, 0.0f, 0.0f};
	Ref<TireModelV2> m_tires[4];

	// --- Driver inputs ---
	float m_throttle = 0.0f;
	float m_brake_input = 0.0f;
	float m_steer_input = 0.0f;
	float m_max_engine_torque = 1500.0f;
	float m_max_brake_torque = 6000.0f;
	float m_max_steer_angle = 0.25f;

protected:
	static void _bind_methods();

	// Subclass hook. Return an additional upward force (N) to apply at this
	// corner. Called once per corner per tick. Base returns 0.
	virtual float extra_corner_force(int p_corner_index, float p_compression,
			float p_opposite_compression) const;

	// Subclass hook. Return the actual steer angle (rad) for this corner
	// given the base steering input. Default: front wheels get the base
	// angle, rear wheels get 0.
	virtual float corner_steer_angle(int p_corner_index, float p_base_steer) const;

    // Subclass hook. Return the camber angle (rad) for this corner.
	// Body frame convention: positive = top tilts toward +Z (right),
	// negative = top tilts toward -Z (left). Default: 0.
	virtual float corner_camber_angle(int p_corner_index) const;

public:
	VehicleBodyT2c();
	~VehicleBodyT2c() override = default;

	// Driver inputs
	void set_throttle(float p_val);
	float get_throttle() const;
	void set_brake(float p_val);
	float get_brake() const;
	void set_steer(float p_val);
	float get_steer() const;

	// State accessors
	float get_yaw() const;
	float get_pitch() const;
	float get_roll() const;
	float get_speed() const;
	Vector3 get_velocity() const;
	Quaternion get_orientation() const;
	Vector3 get_angular_velocity() const;

	// Per-corner diagnostics
	float get_corner_load(int p_index) const;
	float get_corner_compression(int p_index) const;
    // Diagnostic: returns the actual steer angle (rad) that will be applied
	// to corner p_index given the current steer input. Useful for verifying
	// Ackermann geometry and toe.
	float get_steer_angle_for_corner(int p_index) const;

	void step(double p_delta) override;
};