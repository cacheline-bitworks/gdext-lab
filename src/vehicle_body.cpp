#include "vehicle_body.h"

#include <cmath>

void VehicleBody::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_throttle", "value"), &VehicleBody::set_throttle);
	ClassDB::bind_method(D_METHOD("get_throttle"), &VehicleBody::get_throttle);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "throttle"), "set_throttle", "get_throttle");

	ClassDB::bind_method(D_METHOD("set_brake", "value"), &VehicleBody::set_brake);
	ClassDB::bind_method(D_METHOD("get_brake"), &VehicleBody::get_brake);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "brake"), "set_brake", "get_brake");

	ClassDB::bind_method(D_METHOD("set_steer", "value"), &VehicleBody::set_steer);
	ClassDB::bind_method(D_METHOD("get_steer"), &VehicleBody::get_steer);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "steer"), "set_steer", "get_steer");
}

// 
// Driver input accessors
// 
void VehicleBody::set_throttle(float p_val) {
	m_throttle = p_val < 0.0f ? 0.0f : (p_val > 1.0f ? 1.0f : p_val);
}
float VehicleBody::get_throttle() const { return m_throttle; }

void VehicleBody::set_brake(float p_val) {
	m_brake_input = p_val < 0.0f ? 0.0f : (p_val > 1.0f ? 1.0f : p_val);
}
float VehicleBody::get_brake() const { return m_brake_input; }

void VehicleBody::set_steer(float p_val) {
	m_steer_input = p_val < -1.0f ? -1.0f : (p_val > 1.0f ? 1.0f : p_val);
}
float VehicleBody::get_steer() const { return m_steer_input; }

// 
// Resistance: rolling + aero drag + low-speed damping
// 
void VehicleBody::apply_resistance(double p_dt) {
	const float dt = (float)p_dt;
	const float speed = m_velocity.length();

	if (speed > 0.001f) {
		const float rolling = 200.0f;
		const float drag = 0.4f * speed * speed;
		const float total_resist = rolling + drag;
		const float resist_decel = (total_resist / m_mass) * dt;
		if (resist_decel >= speed) {
			m_velocity = Vector3(0.0f, 0.0f, 0.0f);
		} else {
			const Vector3 dir = m_velocity / speed;
			m_velocity -= dir * resist_decel;
		}
	}

	const float speed_after = m_velocity.length();
	if (speed_after < 2.0f && m_throttle < 0.01f && m_brake_input < 0.01f) {
		const float damping = 5.0f;
		m_velocity *= (1.0f - damping * dt);
		if (m_velocity.length() < 0.05f) {
			m_velocity = Vector3(0.0f, 0.0f, 0.0f);
		}
	}
}

// 
// Wheel spin integration
// 
void VehicleBody::integrate_wheel_spin(int p_index, float p_force_x,
		float p_drive_torque, float p_brake_torque, double p_dt,
		float p_wheel_multiplier) {
	if (p_index < 0 || p_index > 3) {
		return;
	}
	const float dt = (float)p_dt;

	// Tire reaction torque opposes wheel spin
	const float reaction = -p_force_x * m_wheel_radius;
	// Bicycle tiers have one virtual wheel per axle (2 physical wheels),
	// so their effective inertia is 2x. Per-wheel tiers pass 1.0.
	const float effective_inertia = p_wheel_multiplier * m_wheel_inertia;
	const float alpha = (p_drive_torque - p_brake_torque + reaction) / effective_inertia;

	m_omega[p_index] += alpha * dt;

	if (m_omega[p_index] < 0.0f) m_omega[p_index] = 0.0f;
	if (m_omega[p_index] > 500.0f) m_omega[p_index] = 500.0f;
}