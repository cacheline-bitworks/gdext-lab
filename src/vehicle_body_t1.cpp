#include "vehicle_body_t1.h"

#include <cmath>

namespace {
constexpr float GRAVITY = 9.81f;
}

VehicleBodyT1::VehicleBodyT1() {
	// Default tires: V1 (cheapest tier, matches T1's "traffic AI" role).
	// Use set_corner_tire() to swap at runtime.
	Ref<TireModelV1> front_v1;
	front_v1.instantiate();
	m_front_tire = front_v1;

	Ref<TireModelV1> rear_v1;
	rear_v1.instantiate();
	m_rear_tire = rear_v1;
}

void VehicleBodyT1::_bind_methods() {
	ClassDB::bind_method(D_METHOD("get_yaw"), &VehicleBodyT1::get_yaw);
	ClassDB::bind_method(D_METHOD("get_yaw_rate"), &VehicleBodyT1::get_yaw_rate);
	ClassDB::bind_method(D_METHOD("get_speed"), &VehicleBodyT1::get_speed);
	ClassDB::bind_method(D_METHOD("get_velocity"), &VehicleBodyT1::get_velocity);
	ClassDB::bind_method(D_METHOD("get_corner_count"), &VehicleBodyT1::get_corner_count);
	ClassDB::bind_method(D_METHOD("set_corner_tire", "index", "tire"), &VehicleBodyT1::set_corner_tire);
	ClassDB::bind_method(D_METHOD("get_corner_tire", "index"), &VehicleBodyT1::get_corner_tire);
}

float VehicleBodyT1::get_yaw() const { return m_yaw; }
float VehicleBodyT1::get_yaw_rate() const { return m_yaw_rate; }
float VehicleBodyT1::get_speed() const { return m_velocity.length(); }
Vector3 VehicleBodyT1::get_velocity() const { return m_velocity; }

void VehicleBodyT1::step(double p_delta) {
	const float dt = (float)p_delta;
	if (dt <= 0.0f) {
		return;
	}
	if (!m_front_tire.is_valid() || !m_rear_tire.is_valid()) {
		return;
	}

	// --- 1. World velocity -> body frame ---
	// Yaw convention: positive yaw rotates the body toward world -Z
	// (matching T2c's quaternion convention).
	// This is the inverse of the body->world transform below.
	const float cy = std::cos(m_yaw);
	const float sy = std::sin(m_yaw);
	const float vx_body = cy * m_velocity.x - sy * m_velocity.z;
	const float vz_body = sy * m_velocity.x + cy * m_velocity.z;

	// --- 2. Steering ---
	const float steer_angle = m_steer_input * m_max_steer_angle;

	// --- 3. Static loads ---
	const float total_load = m_mass * GRAVITY;
	const float front_load = total_load * (m_cg_to_rear / m_wheel_base);
	const float rear_load = total_load * (m_cg_to_front / m_wheel_base);

	// --- 4. Wheel inputs ---
	WheelInput front_input;
	front_input.steer_angle = steer_angle;
	front_input.angular_velocity = m_omega[0];
	front_input.torque = m_throttle * m_max_engine_torque * 0.5f;
	front_input.brake_torque = m_brake_input * m_max_brake_torque * 0.7f;
	front_input.vertical_load = front_load;
	front_input.friction_coefficient = 1.0f;
	front_input.wheel_radius = m_wheel_radius;
	front_input.velocity_longitudinal = vx_body;
	front_input.velocity_lateral = vz_body - m_yaw_rate * m_cg_to_front;
	front_input.dt = dt;

	WheelInput rear_input;
	rear_input.steer_angle = 0.0f;
	rear_input.angular_velocity = m_omega[1];
	rear_input.torque = m_throttle * m_max_engine_torque * 0.5f;
	rear_input.brake_torque = m_brake_input * m_max_brake_torque * 0.3f;
	rear_input.vertical_load = rear_load;
	rear_input.friction_coefficient = 1.0f;
	rear_input.wheel_radius = m_wheel_radius;
	rear_input.velocity_longitudinal = vx_body;
	rear_input.velocity_lateral = vz_body + m_yaw_rate * m_cg_to_rear;
	rear_input.dt = dt;

	// --- 5. Tires ---
	const WheelOutput front_out = m_front_tire->step(front_input);
	const WheelOutput rear_out = m_rear_tire->step(rear_input);

	// --- 6. Sum forces in body frame ---
	// Front wheel is steered: rotate its local force into body frame.
	// Local-to-body:  fx_b = fx*cos - fy*sin
	//                 fz_b = fx*sin + fy*cos
	const float cs = std::cos(steer_angle);
	const float ss = std::sin(steer_angle);

	const float fx_body = front_out.force_x * cs - front_out.force_y * ss
			+ rear_out.force_x;
	const float fy_body = front_out.force_x * ss + front_out.force_y * cs
			+ rear_out.force_y;
	const float mz_body = rear_out.force_y * m_cg_to_rear
			- (front_out.force_y * cs + front_out.force_x * ss) * m_cg_to_front;

	// --- 7. Body forces -> world frame ---
	const float cy_w = std::cos(m_yaw);
	const float sy_w = std::sin(m_yaw);
	const float fx_world = cy_w * fx_body + sy_w * fy_body;
	const float fz_world = -sy_w * fx_body + cy_w * fy_body;

	// --- 8. Resistance (from base) ---
	apply_resistance(dt);

	// --- 9. Linear integration ---
	m_velocity.x += (fx_world / m_mass) * dt;
	m_velocity.z += (fz_world / m_mass) * dt;

	// --- 10. Yaw integration ---
	const float yaw_inertia = m_yaw_inertia;
	const float linear_speed = m_velocity.length();
	const float damping_scale = 1.0f + 15.0f / (1.0f + linear_speed);
	const float yaw_damping = -m_yaw_rate * 200.0f * damping_scale;
	const float yaw_accel = (mz_body + yaw_damping) / yaw_inertia;
	m_yaw_rate += yaw_accel * dt;
	m_yaw += m_yaw_rate * dt;
	if (m_yaw_rate > 20.0f) m_yaw_rate = 20.0f;
	if (m_yaw_rate < -20.0f) m_yaw_rate = -20.0f;
	if (m_velocity.length() < 1.0f) m_yaw_rate = 0.0f;

	// --- 11. Position ---
	m_position.x += m_velocity.x * dt;
	m_position.z += m_velocity.z * dt;

	// --- 12. Wheel spin (from base) ---
	integrate_wheel_spin(0, front_out.force_x,
			m_throttle * m_max_engine_torque * 0.5f,
			m_brake_input * m_max_brake_torque * 0.7f, dt, 2.0f);
	integrate_wheel_spin(1, rear_out.force_x,
			m_throttle * m_max_engine_torque * 0.5f,
			m_brake_input * m_max_brake_torque * 0.3f, dt, 2.0f);
}
	//
// Tire selection
//
void VehicleBodyT1::set_corner_tire(int p_index, const Ref<TireModel> &p_tire) {
	if (p_index == 0) {
		m_front_tire = p_tire;
	} else if (p_index == 1) {
		m_rear_tire = p_tire;
	} else {
		ERR_PRINT("VehicleBodyT1::set_corner_tire: index out of range (0-1)");
	}
}

Ref<TireModel> VehicleBodyT1::get_corner_tire(int p_index) const {
	if (p_index == 0) return m_front_tire;
	if (p_index == 1) return m_rear_tire;
	return Ref<TireModel>();
}