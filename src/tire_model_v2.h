#pragma once

#include "tire_model.h"

using namespace godot;

// 
// TireModelV2 - load sensitivity + combined slip
// 
// Adds two physics effects on top of V1:
//
//   1. Load sensitivity - friction coefficient drops as vertical load rises.
//      Real tires do not scale grip linearly with load.
//
//   2. Combined slip - longitudinal and lateral forces share a friction
//      budget. When both are present, they constrain each other.
//
// Same WheelInput / WheelOutput interface as V1. Same step() signature.
// Callers do not know which version they are using.
//
// See docs/patterns/04-tire-model-v2.md for the design.
// 
class TireModelV2 : public TireModel {
	GDCLASS(TireModelV2, TireModel)

	// Pacejka coefficients - same defaults as V1
	float m_long_stiffness = 10.0f;
	float m_long_shape = 1.65f;
	float m_long_curvature = 0.5f;

	float m_lat_stiffness = 8.0f;
	float m_lat_shape = 1.3f;
	float m_lat_curvature = 0.97f;

	// V2 additions
	float m_nominal_load = 4000.0f;       // N, load at which m_friction is defined
	float m_load_sensitivity = 0.15f;     // dimensionless, k_load coefficient

protected:
	static void _bind_methods();

public:
	TireModelV2() = default;
	~TireModelV2() override = default;

	// The real interface - used internally by the vehicle simulation.
	WheelOutput step(const WheelInput &p_input) override;

	// GDScript-friendly test helpers
	Dictionary step_gd(float p_omega, float p_torque, float p_brake_torque,
			float p_vertical_load, float p_friction, float p_radius,
			float p_vel_long, float p_vel_lat, float p_steer_angle);

	float compute_force_x(float p_slip_ratio, float p_vertical_load, float p_friction) const;
	float compute_force_y(float p_slip_angle, float p_vertical_load, float p_friction) const;

	// Property accessors for the V2 parameters
	void set_nominal_load(float p_load);
	float get_nominal_load() const;
	void set_load_sensitivity(float p_k);
	float get_load_sensitivity() const;

	// Compute the friction coefficient after load sensitivity correction.
	// Exposed for testing and visualization.
	float effective_mu(float p_load, float p_friction) const;

};