#include "vehicle_body_t3.h"

#include <cmath>

void VehicleBodyT3::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_arb_front", "value"), &VehicleBodyT3::set_arb_front);
	ClassDB::bind_method(D_METHOD("get_arb_front"), &VehicleBodyT3::get_arb_front);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "arb_front"), "set_arb_front", "get_arb_front");

	ClassDB::bind_method(D_METHOD("set_arb_rear", "value"), &VehicleBodyT3::set_arb_rear);
	ClassDB::bind_method(D_METHOD("get_arb_rear"), &VehicleBodyT3::get_arb_rear);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "arb_rear"), "set_arb_rear", "get_arb_rear");

	ClassDB::bind_method(D_METHOD("set_ackermann_factor", "value"), &VehicleBodyT3::set_ackermann_factor);
	ClassDB::bind_method(D_METHOD("get_ackermann_factor"), &VehicleBodyT3::get_ackermann_factor);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "ackermann_factor"), "set_ackermann_factor", "get_ackermann_factor");

	ClassDB::bind_method(D_METHOD("set_toe_front", "value"), &VehicleBodyT3::set_toe_front);
	ClassDB::bind_method(D_METHOD("get_toe_front"), &VehicleBodyT3::get_toe_front);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "toe_front"), "set_toe_front", "get_toe_front");

	ClassDB::bind_method(D_METHOD("set_toe_rear", "value"), &VehicleBodyT3::set_toe_rear);
	ClassDB::bind_method(D_METHOD("get_toe_rear"), &VehicleBodyT3::get_toe_rear);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "toe_rear"), "set_toe_rear", "get_toe_rear");

	ClassDB::bind_method(D_METHOD("set_camber_front", "value"), &VehicleBodyT3::set_camber_front);
	ClassDB::bind_method(D_METHOD("get_camber_front"), &VehicleBodyT3::get_camber_front);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "camber_front"), "set_camber_front", "get_camber_front");

	ClassDB::bind_method(D_METHOD("set_camber_rear", "value"), &VehicleBodyT3::set_camber_rear);
	ClassDB::bind_method(D_METHOD("get_camber_rear"), &VehicleBodyT3::get_camber_rear);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "camber_rear"), "set_camber_rear", "get_camber_rear");
}

// 
// Accessors — all tuning parameters grouped together
// 

// Anti-roll bars
void VehicleBodyT3::set_arb_front(float p_val) {
	m_arb_front = p_val < 0.0f ? 0.0f : p_val;
}
float VehicleBodyT3::get_arb_front() const { return m_arb_front; }

void VehicleBodyT3::set_arb_rear(float p_val) {
	m_arb_rear = p_val < 0.0f ? 0.0f : p_val;
}
float VehicleBodyT3::get_arb_rear() const { return m_arb_rear; }

// Ackermann
void VehicleBodyT3::set_ackermann_factor(float p_val) {
	m_ackermann_factor = p_val < 0.0f ? 0.0f : (p_val > 1.0f ? 1.0f : p_val);
}
float VehicleBodyT3::get_ackermann_factor() const { return m_ackermann_factor; }

// Toe
void VehicleBodyT3::set_toe_front(float p_val) { m_toe_front = p_val; }
float VehicleBodyT3::get_toe_front() const { return m_toe_front; }
void VehicleBodyT3::set_toe_rear(float p_val) { m_toe_rear = p_val; }
float VehicleBodyT3::get_toe_rear() const { return m_toe_rear; }

// Camber
void VehicleBodyT3::set_camber_front(float p_val) { m_camber_front = p_val; }
float VehicleBodyT3::get_camber_front() const { return m_camber_front; }
void VehicleBodyT3::set_camber_rear(float p_val) { m_camber_rear = p_val; }
float VehicleBodyT3::get_camber_rear() const { return m_camber_rear; }

// 
// T2c hook overrides — physical behavior
//

// Anti-roll bar hook. Force is proportional to the difference in
// compression between opposite corners on the same axle. Net vertical
// force on the axle is unchanged; only the left/right distribution shifts.
float VehicleBodyT3::extra_corner_force(int p_corner_index, float p_compression,
		float p_opposite_compression) const {
	const float arb_stiffness = (p_corner_index < 2) ? m_arb_front : m_arb_rear;
	return arb_stiffness * (p_compression - p_opposite_compression);
}

// Camber hook. Physical camber is negative when the top of the wheel
// leans inward. In body frame (X forward, Y up, Z right):
//   Left side (FL, RL): inward = top toward +Z, so body camber = -physical
//   Right side (FR, RR): inward = top toward -Z, so body camber = +physical
float VehicleBodyT3::corner_camber_angle(int p_corner_index) const {
	const float camber = (p_corner_index < 2) ? m_camber_front : m_camber_rear;
	const bool is_left = (p_corner_index == 0 || p_corner_index == 2);
	return is_left ? -camber : camber;
}

// Ackermann + toe hook.
//
// Ackermann geometry:
//   - Given base steer angle delta, turning radius R = wheel_base / tan(|delta|)
//   - Inner wheel traces circle of radius R - track/2
//   - Outer wheel traces circle of radius R + track/2
//   - Inner steer angle = atan(wheel_base / (R - track/2))
//   - Outer steer angle = atan(wheel_base / (R + track/2))
//   - Inner always greater than outer (in magnitude)
//
// Blend with the base angle using m_ackermann_factor:
//   final = base + factor * (ackermann - base)
//
// Toe: small static offsets applied to both front and rear wheels.
//   Positive toe = toe-in (wheels point toward car centerline).
//   Left side gets +toe, right side gets -toe.
float VehicleBodyT3::corner_steer_angle(int p_corner_index, float p_base_steer) const {
	// Rear wheels: toe only, no steering
	if (p_corner_index >= 2) {
		const float sign = (p_corner_index == 2) ? 1.0f : -1.0f;  // RL: +, RR: -
		return sign * m_toe_rear;
	}

	// Front wheels: Ackermann + toe
	float base = p_base_steer;

	const float abs_steer = std::fabs(base);
	if (abs_steer > 0.001f && m_ackermann_factor > 0.001f) {
		const float turn_sign = (base > 0.0f) ? 1.0f : -1.0f;
		const float R = m_wheel_base / std::tan(abs_steer);

		const float half_track = m_track_width * 0.5f;
		const float R_inner = (R > half_track + 0.1f) ? (R - half_track) : 0.1f;
		const float R_outer = R + half_track;

		const float steer_inner = turn_sign * std::atan(m_wheel_base / R_inner);
		const float steer_outer = turn_sign * std::atan(m_wheel_base / R_outer);

		const bool is_left_wheel = (p_corner_index == 0);
		const bool is_inner = (turn_sign > 0.0f && is_left_wheel) ||
							  (turn_sign < 0.0f && !is_left_wheel);

		const float ackermann_angle = is_inner ? steer_inner : steer_outer;
		base = base + m_ackermann_factor * (ackermann_angle - base);
	}

	const float toe_sign = (p_corner_index == 0) ? 1.0f : -1.0f;  // FL: +, FR: -
	return base + toe_sign * m_toe_front;
}