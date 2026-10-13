#pragma once

#include "tire_model_v2.h"

using namespace godot;

// ----------------------------------------------------------------------------
// TireModelV3 - relaxation length + transient response
// ----------------------------------------------------------------------------
// Extends V2 with a first-order lag on slip ratio and slip angle. Real tire
// carcasses deform over distance, so forces don't respond instantly to
// kinematic changes. This lag is what makes drift transitions smooth and
// gives countersteer a response window.
//
// Model:
//   d(slip)/dt = (vx / sigma) * (target_slip - current_slip)
//
// Where sigma is the relaxation length (typically 0.3-0.8 m).
//
// V3 is stateful per tire: it remembers m_relaxed_slip_ratio and
// m_relaxed_slip_angle. reset() clears them.
// ----------------------------------------------------------------------------
class TireModelV3 : public TireModelV2 {
	GDCLASS(TireModelV3, TireModelV2)

	// Relaxation length (meters). Larger = slower response.
	float m_relaxation_length = 0.5f;

	// State: filtered slip values, updated each step
	float m_relaxed_slip_ratio = 0.0f;
	float m_relaxed_slip_angle = 0.0f;

protected:
	static void _bind_methods();

	// V2 hooks — apply first-order lag
	float filter_slip_ratio(float p_target, const WheelInput &p_input) const override;
	float filter_slip_angle(float p_target, const WheelInput &p_input) const override;

public:
	TireModelV3() = default;
	~TireModelV3() override = default;

	// Clear relaxation state.
	void reset() override;

	void set_relaxation_length(float p_val);
	float get_relaxation_length() const;

	float get_relaxed_slip_ratio() const { return m_relaxed_slip_ratio; }
	float get_relaxed_slip_angle() const { return m_relaxed_slip_angle; }

	// Test helper: like V2's step_gd but takes an explicit dt so V3's
	// relaxation filter advances correctly from GDScript.
	Dictionary step_gd_with_dt(float p_omega, float p_torque, float p_brake_torque,
			float p_vertical_load, float p_friction, float p_radius,
			float p_vel_long, float p_vel_lat, float p_steer_angle, float p_dt);
};