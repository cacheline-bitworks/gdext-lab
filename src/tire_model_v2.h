#pragma once

#include "tire_model.h"

using namespace godot;

//
// TireModelV2 - load sensitivity + combined slip
//
// Extends V1 with:
//   - Load sensitivity: friction drops as vertical load rises
//   - Combined slip: Fx and Fy share a friction budget (friction ellipse)
//   - Camber thrust: camber angle produces a lateral force component
//
// Pacejka coefficients are inherited from TireModelV1. V2 only overrides
// the force computation to inject the load-sensitivity correction, and
// overrides step() to add the friction ellipse and camber thrust.
//
// See docs/patterns/04-tire-model-v2.md for the design.
//
class TireModelV2 : public TireModelV1 {
	GDCLASS(TireModelV2, TireModelV1)

	// V2-specific state
	float m_nominal_load = 4000.0f;
	float m_load_sensitivity = 0.15f;
	
	// Pneumatic trail (meters). The distance between the contact patch
	// center and the point where lateral force is actually applied.
	// Creates the self-aligning torque (moment_z).
	float m_pneumatic_trail = 0.05f;

protected:
	static void _bind_methods();
	// Filter hooks for slip values before force computation. Default
	// implementations return the input unchanged. V3 overrides these to
	// apply relaxation length (a first-order lag on slip).
	virtual float filter_slip_ratio(float p_target, const WheelInput &p_input) const {
		return p_target;
	}
	virtual float filter_slip_angle(float p_target, const WheelInput &p_input) const {
		return p_target;
	}

public:
	TireModelV2() = default;
	~TireModelV2() override = default;

	WheelOutput step(const WheelInput &p_input) override;	
	void set_pneumatic_trail(float p_val);
	float get_pneumatic_trail() const;
Dictionary step_gd(float p_omega, float p_torque, float p_brake_torque,
			float p_vertical_load, float p_friction, float p_radius,
			float p_vel_long, float p_vel_lat, float p_steer_angle,
			float p_camber_angle);

	float compute_force_x(float p_slip_ratio, float p_vertical_load, float p_friction) const override;
	float compute_force_y(float p_slip_angle, float p_vertical_load, float p_friction) const override;

	void set_nominal_load(float p_load);
	float get_nominal_load() const;
	void set_load_sensitivity(float p_k);
	float get_load_sensitivity() const;

	// Compute effective friction after load sensitivity correction.
	virtual float effective_mu(float p_load, float p_friction) const;
};