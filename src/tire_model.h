#pragma once

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/variant/dictionary.hpp>

using namespace godot;

// ----------------------------------------------------------------------------
// WheelInput - everything the tire model needs to know about the outside world
// ----------------------------------------------------------------------------
struct WheelInput {
	float angular_velocity = 0.0f;       // rad/s
	float torque = 0.0f;                 // Nm
	float brake_torque = 0.0f;           // Nm
	float vertical_load = 0.0f;          // N
	float friction_coefficient = 1.0f;
	float wheel_radius = 0.33f;          // m
	float velocity_longitudinal = 0.0f;  // m/s
	float velocity_lateral = 0.0f;       // m/s
	float steer_angle = 0.0f;            // rad

	// Camber angle in the body frame (X forward, Y up, Z right).
	// Positive = top of wheel tilts toward +Z (right).
	float camber_angle = 0.0f;

	// Integration time step in seconds. V3 uses this for relaxation
	// length; V1/V2 ignore it. Default 0 = no state advancement.
	float dt = 0.0f;
};

// ----------------------------------------------------------------------------
// WheelOutput - what the tire model returns each step
// ----------------------------------------------------------------------------
struct WheelOutput {
	float force_x = 0.0f;
	float force_y = 0.0f;
	float moment_z = 0.0f;
	float slip_ratio = 0.0f;
	float slip_angle = 0.0f;
};

// ----------------------------------------------------------------------------
// TireModel - abstract base for all tire model versions
// ----------------------------------------------------------------------------
class TireModel : public RefCounted {
	GDCLASS(TireModel, RefCounted)

protected:
	static void _bind_methods();

public:
	TireModel() = default;
	~TireModel() override = default;

	virtual WheelOutput step(const WheelInput &p_input) = 0;

	// Clear any per-tire state (V3's relaxation history, thermal, wear).
	// V1 and V2 are stateless — default implementation is a no-op.
	virtual void reset() {}
};

// ----------------------------------------------------------------------------
// TireModelV1 - pure Pacejka with falloff past peak
// ----------------------------------------------------------------------------
class TireModelV1 : public TireModel {
	GDCLASS(TireModelV1, TireModel)

protected:
	// Pacejka coefficients. Protected so V2+ can reuse or override.
	float m_long_stiffness = 10.0f;
	float m_long_shape = 1.65f;
	float m_long_curvature = 0.5f;

	float m_lat_stiffness = 8.0f;
	float m_lat_shape = 1.3f;
	float m_lat_curvature = 0.97f;

	static void _bind_methods();

public:
	TireModelV1() = default;
	~TireModelV1() override = default;

	WheelOutput step(const WheelInput &p_input) override;

	Dictionary step_gd(float p_omega, float p_torque, float p_brake_torque,
			float p_vertical_load, float p_friction, float p_radius,
			float p_vel_long, float p_vel_lat);

	virtual float compute_force_x(float p_slip_ratio, float p_vertical_load, float p_friction) const;
	virtual float compute_force_y(float p_slip_angle, float p_vertical_load, float p_friction) const;
};