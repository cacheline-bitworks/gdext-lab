#pragma once

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/variant/quaternion.hpp>
#include <godot_cpp/variant/vector3.hpp>

#include "simulated_entity.h"
#include "tire_model.h"
#include "tire_model_v2.h"

using namespace godot;

// 
// CornerConfig - per-corner suspension parameters (same as T2b)
// 
struct CornerConfigT2c {
	float spring_rate = 65000.0f;
	float damper_bump = 4200.0f;
	float damper_rebound = 8500.0f;
};

// 
// VehicleBodyT2c - full 6-DOF rigid body chassis
// 
// T2a and T2b use small-angle approximations: yaw, pitch, and roll are
// tracked as independent scalars, and heave is a vertical offset from
// equilibrium. This works until the body tilts enough that the angles
// interact, or until the body rotates more than a few degrees.
//
// T2c tracks the chassis as a true rigid body:
//   - Position (world frame)
//   - Orientation (Quaternion)
//   - Linear velocity (world frame)
//   - Angular velocity (body frame, rad/s)
//   - Diagonal inertia tensor (Ixx, Iyy, Izz in body frame)
//
// Torques computed in world frame are transformed into body frame, then
// integrated via Euler's rigid body equations. Gravity acts on the actual
// CG position regardless of orientation.
//
// Body frame convention (same as T2a/T2b):
//   X = forward
//   Y = up
//   Z = right
//
// With this convention:
//   Rotation about X = roll
//   Rotation about Y = yaw
//   Rotation about Z = pitch
// 
class VehicleBodyT2c : public SimulatedEntity {
	GDCLASS(VehicleBodyT2c, SimulatedEntity)

	// --- Chassis ---
	float m_mass = 1200.0f;
	float m_eq_height = 0.5f;              // CG height above ground at equilibrium
	Vector3 m_velocity;                     // world frame, m/s
	Vector3 m_angular_velocity;             // body frame, rad/s
	Quaternion m_orientation;               // body -> world rotation

	// Diagonal inertia tensor in body frame (kg·m²)
	// Roll (about X), Yaw (about Y), Pitch (about Z)
	Vector3 m_inertia = Vector3(400.0f, 3000.0f, 1800.0f);
	Vector3 m_inv_inertia = Vector3(1.0f / 400.0f, 1.0f / 3000.0f, 1.0f / 1800.0f);

	// --- Geometry ---
	float m_wheel_base = 2.6f;
	float m_track_width = 1.6f;
	float m_cg_to_front = 1.3f;
	float m_cg_to_rear = 1.3f;
	float m_wheel_radius = 0.33f;
	float m_wheel_inertia = 1.5f;

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
	float get_yaw() const;                      // extract yaw from quaternion
	float get_speed() const;
    float get_pitch() const;   // nose up/down
	float get_roll() const;    // side to side
	Vector3 get_velocity() const;
	Quaternion get_orientation() const;
	Vector3 get_angular_velocity() const;

	// Per-corner diagnostics
	float get_corner_load(int p_index) const;
	float get_corner_compression(int p_index) const;

	void step(double p_delta) override;
};