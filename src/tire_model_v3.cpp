#include "tire_model_v3.h"

#include <cmath>

void TireModelV3::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_relaxation_length", "value"), &TireModelV3::set_relaxation_length);
	ClassDB::bind_method(D_METHOD("get_relaxation_length"), &TireModelV3::get_relaxation_length);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "relaxation_length"), "set_relaxation_length", "get_relaxation_length");

	// Fail-loud override. Inherited V2 step_gd would silently leave dt=0.
	ClassDB::bind_method(D_METHOD("step_gd", "omega", "torque", "brake_torque",
			"vertical_load", "friction", "radius", "vel_long", "vel_lat",
			"steer_angle", "camber_angle"),
			&TireModelV3::step_gd);

	ClassDB::bind_method(D_METHOD("step_gd_with_dt", "omega", "torque", "brake_torque",
		"vertical_load", "friction", "radius", "vel_long", "vel_lat",
		"steer_angle", "camber_angle", "dt"),
		&TireModelV3::step_gd_with_dt);

	ClassDB::bind_method(D_METHOD("get_relaxed_slip_ratio"), &TireModelV3::get_relaxed_slip_ratio);
	ClassDB::bind_method(D_METHOD("get_relaxed_slip_angle"), &TireModelV3::get_relaxed_slip_angle);

	ClassDB::bind_method(D_METHOD("reset"), &TireModelV3::reset);
}

void TireModelV3::set_relaxation_length(float p_val) {
	m_relaxation_length = p_val < 0.01f ? 0.01f : p_val;
}

float TireModelV3::get_relaxation_length() const {
	return m_relaxation_length;
}

void TireModelV3::reset() {
	m_relaxed_slip_ratio = 0.0f;
	m_relaxed_slip_angle = 0.0f;
}

// -
// step_gd override — refuses to run without a dt.
// -
// V3's relaxation filter requires an integration timestep. The inherited
// V2 step_gd does not pass one, so V3 would silently behave like V2.
// Fail loudly instead of returning wrong numbers.
// -
Dictionary TireModelV3::step_gd(float, float, float, float, float, float,
		float, float, float, float) {
	ERR_PRINT("TireModelV3::step_gd needs a dt. Use step_gd_with_dt instead.");
	return Dictionary();
}

// -
// First-order lag helper.
//
// Discrete form derived from d(s)/dt = (vx / sigma) * (target - s):
//   s_new = s + (1 - exp(-vx * dt / sigma)) * (target - s)
//
// For small x, 1 - exp(-x) ~= x, so we use the linear approximation:
//   factor = clamp(vx * dt / sigma, 0, 1)
//   s_new = s + factor * (target - s)
//
// At dt = 0, factor = 0, no update. Matches "no integration without time."
// -
static float relax_toward(float current, float target, float p_speed,
		float p_relaxation_length, float p_dt) {
	if (p_dt <= 0.0f) {
		return current;
	}
	// Below the slip-ratio denominator floor (2 m/s), the tire model already
	// smooths low-speed behavior. Relaxation on top of that causes stale state
	// to produce phantom force when the car stops — the "low-speed creep"
	// that makes the car refuse to settle. Snap to target here; relaxation
	// only applies above this threshold, where it's physically meaningful.
	constexpr float LOW_SPEED_SNAP = 2.0f;
	if (p_speed < LOW_SPEED_SNAP) {
		return target;
	}
	const float raw_factor = p_speed * p_dt / p_relaxation_length;
	const float factor = raw_factor > 1.0f ? 1.0f : raw_factor;
	return current + factor * (target - current);
}

float TireModelV3::filter_slip_ratio(float p_target, const WheelInput &p_input) const {
	// const_cast: the filter needs to update state, but the V2 hook signature
	// is const for stateless V2 implementations. Mutable state lives here.
	TireModelV3 *self = const_cast<TireModelV3 *>(this);
	const float vx_abs = std::fabs(p_input.velocity_longitudinal);
	self->m_relaxed_slip_ratio = relax_toward(
			self->m_relaxed_slip_ratio, p_target, vx_abs,
			self->m_relaxation_length, p_input.dt);
	return self->m_relaxed_slip_ratio;
}

float TireModelV3::filter_slip_angle(float p_target, const WheelInput &p_input) const {
	TireModelV3 *self = const_cast<TireModelV3 *>(this);
	const float vx_abs = std::fabs(p_input.velocity_longitudinal);
	self->m_relaxed_slip_angle = relax_toward(
			self->m_relaxed_slip_angle, p_target, vx_abs,
			self->m_relaxation_length, p_input.dt);
	return self->m_relaxed_slip_angle;
}

Dictionary TireModelV3::step_gd_with_dt(float p_omega, float p_torque, float p_brake_torque,
		float p_vertical_load, float p_friction, float p_radius,
		float p_vel_long, float p_vel_lat, float p_steer_angle,
		float p_camber_angle, float p_dt) {
	WheelInput input;
	input.angular_velocity = p_omega;
	input.torque = p_torque;
	input.brake_torque = p_brake_torque;
	input.vertical_load = p_vertical_load;
	input.friction_coefficient = p_friction;
	input.wheel_radius = p_radius;
	input.velocity_longitudinal = p_vel_long;
	input.velocity_lateral = p_vel_lat;
	input.steer_angle = p_steer_angle;
	input.camber_angle = p_camber_angle;
	input.dt = p_dt;
	const WheelOutput out = step(input);
	


	Dictionary result;
	result["force_x"] = out.force_x;
	result["force_y"] = out.force_y;
	result["moment_z"] = out.moment_z;
	result["slip_ratio"] = out.slip_ratio;
	result["slip_angle"] = out.slip_angle;
	return result;
}