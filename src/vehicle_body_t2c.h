#pragma once

#include <godot_cpp/variant/quaternion.hpp>
#include <godot_cpp/variant/vector3.hpp>

#include "vehicle_body.h"
#include "tire_model.h"
#include "tire_model_v2.h"
#include "tire_model_v3.h"

using namespace godot;

// 
// CornerConfigT2c - per-corner suspension parameters
// 
struct CornerConfigT2c {
	float spring_rate = 65000.0f;
	float damper_bump = 4200.0f;
	float damper_rebound = 8500.0f;
};

// 
// VehicleBodyT2c - full 6-DOF rigid body chassis
// 
// See docs/patterns/05-vehicle-body-t2.md for the tier design.
// 
class VehicleBodyT2c : public VehicleBody {
	GDCLASS(VehicleBodyT2c, VehicleBody)

	// Chassis-specific
	float m_eq_height = 0.5f;
	Vector3 m_angular_velocity;
	Quaternion m_orientation;

	Vector3 m_inertia = Vector3(400.0f, 3000.0f, 1800.0f);
	Vector3 m_inv_inertia = Vector3(1.0f / 400.0f, 1.0f / 3000.0f, 1.0f / 1800.0f);

	// Aerodynamics
	float m_drag_coefficient = 0.30f;
	float m_downforce_coefficient = 0.00f;
	float m_frontal_area = 2.20f;

	// Suspension per corner
	CornerConfigT2c m_front_left;
	CornerConfigT2c m_front_right;
	CornerConfigT2c m_rear_left;
	CornerConfigT2c m_rear_right;

	// Tires (order: FL, FR, RL, RR)
	Ref<TireModelV3> m_tires[4];

protected:
	static void _bind_methods();

	// Subclass hooks
	virtual float extra_corner_force(int p_corner_index, float p_compression,
			float p_opposite_compression) const;
	virtual float corner_steer_angle(int p_corner_index, float p_base_steer) const;
	virtual float corner_camber_angle(int p_corner_index) const;

public:
	VehicleBodyT2c();
	~VehicleBodyT2c() override = default;

	// State accessors
	float get_yaw() const;
	float get_pitch() const;
	float get_roll() const;
	float get_speed() const;
	Vector3 get_velocity() const;
	Quaternion get_orientation() const;
	Vector3 get_angular_velocity() const;

	// Aero accessors
	void set_drag_coefficient(float p_val);
	float get_drag_coefficient() const;
	void set_downforce_coefficient(float p_val);
	float get_downforce_coefficient() const;
	void set_frontal_area(float p_val);
	float get_frontal_area() const;

	// Diagnostics
	float get_corner_load(int p_index) const;
	float get_corner_compression(int p_index) const;
	float get_steer_angle_for_corner(int p_index) const;

	void step(double p_delta) override;
};