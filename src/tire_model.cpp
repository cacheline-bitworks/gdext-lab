#include "tire_model.h"

#include <cmath>

namespace {
constexpr float SLIP_EPSILON = 0.1f;  // m/s, minimum speed for slip ratio calculation
}

//
// TireModel (abstract base)
//
// Nothing to bind. The base class exists only to define the interface.
// But we still need to define this, because GDCLASS declares it as a static
// method that the registration machinery expects to exist.
//
void TireModel::_bind_methods() {
	// Intentionally empty.
}

// 
// TireModelV1
// 
void TireModelV1::_bind_methods() {
	ClassDB::bind_method(D_METHOD("step_gd", "omega", "torque", "brake_torque",
			"vertical_load", "friction", "radius", "vel_long", "vel_lat"),
			&TireModelV1::step_gd);

	ClassDB::bind_method(D_METHOD("compute_force_x", "slip_ratio", "vertical_load", "friction"),
			&TireModelV1::compute_force_x);

	ClassDB::bind_method(D_METHOD("compute_force_y", "slip_angle", "vertical_load", "friction"),
			&TireModelV1::compute_force_y);
}

// 
// Pacejka Magic Formula
// 
//   F(x) = D * sin(C * atan(B*x - E * (B*x - atan(B*x))))
//
//   B: stiffness (how steeply the curve rises)
//   C: shape (overall curve shape)
//   D: peak factor (maximum force value)
//   E: curvature (shape near the peak)
//
// In v1, D = mu * Fz. The curve peaks at exactly D when the input is
// near the natural peak slip value, and falls off beyond it.
// 

float TireModelV1::compute_force_x(float p_slip_ratio, float p_vertical_load, float p_friction) const {
	const float D = p_friction * p_vertical_load;
	const float B = m_long_stiffness;
	const float C = m_long_shape;
	const float E = m_long_curvature;
	const float x = p_slip_ratio;

	const float Bx = B * x;
	const float inner = Bx - E * (Bx - std::atan(Bx));
	return D * std::sin(C * std::atan(inner));
}

float TireModelV1::compute_force_y(float p_slip_angle, float p_vertical_load, float p_friction) const {
	const float D = p_friction * p_vertical_load;
	const float B = m_lat_stiffness;
	const float C = m_lat_shape;
	const float E = m_lat_curvature;
	const float x = p_slip_angle;

	const float Bx = B * x;
	const float inner = Bx - E * (Bx - std::atan(Bx));
	return D * std::sin(C * std::atan(inner));
}

// 
// The main step function
// 
WheelOutput TireModelV1::step(const WheelInput &p_input) {
	WheelOutput output;

	// 1. Slip ratio (longitudinal)
	//    kappa = (omega * R - vx) / |vx|
	//    At very low speed the ratio is undefined, so we return 0.
	const float vx = p_input.velocity_longitudinal;
	const float omega_R = p_input.angular_velocity * p_input.wheel_radius;
	const float abs_vx = std::fabs(vx);

	if (abs_vx < SLIP_EPSILON) {
		output.slip_ratio = 0.0f;
	} else {
		output.slip_ratio = (omega_R - vx) / abs_vx;
	}

	// 2. Slip angle (lateral)
	//    alpha = atan2(vy, |vx|)
	//    Using max to keep the divisor away from zero.
	const float safe_vx = (abs_vx < SLIP_EPSILON) ? SLIP_EPSILON : abs_vx;
	output.slip_angle = std::atan2(p_input.velocity_lateral, safe_vx);

	// 3. Forces
	output.force_x = compute_force_x(output.slip_ratio, p_input.vertical_load, p_input.friction_coefficient);
	output.force_y = compute_force_y(output.slip_angle, p_input.vertical_load, p_input.friction_coefficient);

	// 4. Aligning moment: not modeled in v1
	output.moment_z = 0.0f;

	return output;
}

// 
// GDScript-friendly wrapper
// 
Dictionary TireModelV1::step_gd(float p_omega, float p_torque, float p_brake_torque,
		float p_vertical_load, float p_friction, float p_radius,
		float p_vel_long, float p_vel_lat) {
	WheelInput input;
	input.angular_velocity = p_omega;
	input.torque = p_torque;
	input.brake_torque = p_brake_torque;
	input.vertical_load = p_vertical_load;
	input.friction_coefficient = p_friction;
	input.wheel_radius = p_radius;
	input.velocity_longitudinal = p_vel_long;
	input.velocity_lateral = p_vel_lat;

	const WheelOutput out = step(input);

	Dictionary result;
	result["force_x"] = out.force_x;
	result["force_y"] = out.force_y;
	result["moment_z"] = out.moment_z;
	result["slip_ratio"] = out.slip_ratio;
	result["slip_angle"] = out.slip_angle;
	return result;
}