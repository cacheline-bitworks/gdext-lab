#include "tire_model_v2.h"

#include <cmath>

namespace {
constexpr float SLIP_EPSILON = 2.0f;
}

// 
// _bind_methods
// 
void TireModelV2::_bind_methods() {
	ClassDB::bind_method(D_METHOD("step_gd", "omega", "torque", "brake_torque",
			"vertical_load", "friction", "radius", "vel_long", "vel_lat", "steer_angle"),
			&TireModelV2::step_gd);

	ClassDB::bind_method(D_METHOD("compute_force_x", "slip_ratio", "vertical_load", "friction"),
			&TireModelV2::compute_force_x);
	ClassDB::bind_method(D_METHOD("compute_force_y", "slip_angle", "vertical_load", "friction"),
			&TireModelV2::compute_force_y);
    ClassDB::bind_method(D_METHOD("effective_mu", "load", "friction"),
			&TireModelV2::effective_mu);

	ClassDB::bind_method(D_METHOD("set_nominal_load", "load"), &TireModelV2::set_nominal_load);
	ClassDB::bind_method(D_METHOD("get_nominal_load"), &TireModelV2::get_nominal_load);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "nominal_load"), "set_nominal_load", "get_nominal_load");

	ClassDB::bind_method(D_METHOD("set_load_sensitivity", "k"), &TireModelV2::set_load_sensitivity);
	ClassDB::bind_method(D_METHOD("get_load_sensitivity"), &TireModelV2::get_load_sensitivity);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "load_sensitivity"), "set_load_sensitivity", "get_load_sensitivity");
}

//
// Property accessors
//
void TireModelV2::set_nominal_load(float p_load) {
	m_nominal_load = p_load > 1.0f ? p_load : 1.0f;
}

float TireModelV2::get_nominal_load() const {
	return m_nominal_load;
}

void TireModelV2::set_load_sensitivity(float p_k) {
	// k_load typically ranges from 0.05 (very stiff) to 0.35 (very soft)
	m_load_sensitivity = p_k < 0.0f ? 0.0f : (p_k > 0.5f ? 0.5f : p_k);
}

float TireModelV2::get_load_sensitivity() const {
	return m_load_sensitivity;
}

// 
// Load sensitivity - the V2 signature effect
// 
// Friction coefficient is not constant with load. Real tires produce
// slightly less grip per unit load as load increases. This is why weight
// transfer costs total grip, and why soft cars corner better than stiff
// ones in the dry.
//
// Formula:
//   mu_eff = mu_0 * (1 - k_load * (Fz / Fz_nominal - 1))
//
// At nominal load, mu_eff = mu_0. Above nominal, mu_eff drops. Below
// nominal, mu_eff rises (a lightly-loaded tire grips more efficiently
// per unit load).
// 
float TireModelV2::effective_mu(float p_load, float p_friction) const {
	const float load_ratio = p_load / m_nominal_load;
	float mu = p_friction * (1.0f - m_load_sensitivity * (load_ratio - 1.0f));

	// Defensive clamps. Extreme loads could push mu below zero or
	// above what a real tire can do.
	if (mu < 0.1f) {
		mu = 0.1f;
	}
	if (mu > 1.8f) {
		mu = 1.8f;
	}
	return mu;
}

// 
// Longitudinal force (with load sensitivity + falloff)
// 
float TireModelV2::compute_force_x(float p_slip_ratio, float p_vertical_load, float p_friction) const {
	// V2 change: use effective mu instead of raw friction
	const float mu_eff = effective_mu(p_vertical_load, p_friction);
	const float D = mu_eff * p_vertical_load;

	const float B = m_long_stiffness;
	const float C = m_long_shape;
	const float E = m_long_curvature;
	const float x = p_slip_ratio;

	const float Bx = B * x;
	const float inner = Bx - E * (Bx - std::atan(Bx));
	const float raw = D * std::sin(C * std::atan(inner));

	// Same falloff as V1
	const float ax = std::fabs(x);
	float falloff = 1.0f;
	if (ax > 0.2f) {
		falloff = 1.0f / (1.0f + 0.4f * (ax - 0.2f));
	}
	return raw * falloff;
}

// 
// Lateral force (with load sensitivity + falloff)
// 
float TireModelV2::compute_force_y(float p_slip_angle, float p_vertical_load, float p_friction) const {
	const float mu_eff = effective_mu(p_vertical_load, p_friction);
	const float D = mu_eff * p_vertical_load;

	const float B = m_lat_stiffness;
	const float C = m_lat_shape;
	const float E = m_lat_curvature;
	const float x = p_slip_angle;

	const float Bx = B * x;
	const float inner = Bx - E * (Bx - std::atan(Bx));
	const float raw = D * std::sin(C * std::atan(inner));

	const float ax = std::fabs(x);
	float falloff = 1.0f;
	if (ax > 0.15f) {
		falloff = 1.0f / (1.0f + 0.6f * (ax - 0.15f));
	}
	return raw * falloff;
}

// 
// The step function - where the friction ellipse lives
// 
// 
WheelOutput TireModelV2::step(const WheelInput &p_input) {
	WheelOutput output;

	// 1. Slip ratio
	const float vx = p_input.velocity_longitudinal;
	const float omega_R = p_input.angular_velocity * p_input.wheel_radius;
	const float diff = omega_R - vx;
	const float denom = std::fmax(std::fabs(vx), SLIP_EPSILON);

	float slip = diff / denom;
	if (slip > 3.0f) slip = 3.0f;
	if (slip < -3.0f) slip = -3.0f;
	output.slip_ratio = slip;

	// 2. Slip angle
	const float safe_vx = std::fmax(std::fabs(vx), SLIP_EPSILON);
	output.slip_angle = std::atan2(p_input.velocity_lateral, safe_vx) - p_input.steer_angle;

	// 3. Compute raw forces (each already includes load sensitivity + falloff)
	float fx = compute_force_x(output.slip_ratio, p_input.vertical_load, p_input.friction_coefficient);
	float fy = compute_force_y(output.slip_angle, p_input.vertical_load, p_input.friction_coefficient);

	// 4. Friction ellipse constraint
	//    The tire has one friction budget. If Fx and Fy together exceed it,
	//    both must be scaled down. Peak force in either direction is
	//    mu_eff * Fz, so the ellipse is:
	//
	//      (Fx / (mu*Fz))^2 + (Fy / (mu*Fz))^2 <= 1
	//
	//    Which simplifies to:
	//
	//      (Fx^2 + Fy^2) <= (mu*Fz)^2
	//
	const float mu_eff = effective_mu(p_input.vertical_load, p_input.friction_coefficient);
	const float f_peak = mu_eff * p_input.vertical_load;

	if (f_peak > 1.0f) {
		const float total_sq = fx * fx + fy * fy;
		const float peak_sq = f_peak * f_peak;

		if (total_sq > peak_sq) {
			const float scale = f_peak / std::sqrt(total_sq);
			fx *= scale;
			fy *= scale;
		}
	} else {
		// Near-zero load. Tire has essentially no grip. Zero the forces.
		fx = 0.0f;
		fy = 0.0f;
	}

	output.force_x = fx;
	output.force_y = fy;
	output.moment_z = 0.0f;

	return output;
}

// 
// GDScript-friendly wrapper
// 
Dictionary TireModelV2::step_gd(float p_omega, float p_torque, float p_brake_torque,
		float p_vertical_load, float p_friction, float p_radius,
		float p_vel_long, float p_vel_lat, float p_steer_angle) {
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

	const WheelOutput out = step(input);

	Dictionary result;
	result["force_x"] = out.force_x;
	result["force_y"] = out.force_y;
	result["moment_z"] = out.moment_z;
	result["slip_ratio"] = out.slip_ratio;
	result["slip_angle"] = out.slip_angle;
	return result;
}