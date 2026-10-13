#include "vehicle_body_t2a.h"

#include <cmath>

namespace {
constexpr float GRAVITY = 9.81f;
}

VehicleBodyT2a::VehicleBodyT2a() {
	m_front_tire.instantiate();
	m_rear_tire.instantiate();
}

void VehicleBodyT2a::_bind_methods() {
	ClassDB::bind_method(D_METHOD("get_yaw"), &VehicleBodyT2a::get_yaw);
	ClassDB::bind_method(D_METHOD("get_yaw_rate"), &VehicleBodyT2a::get_yaw_rate);
	ClassDB::bind_method(D_METHOD("get_speed"), &VehicleBodyT2a::get_speed);
	ClassDB::bind_method(D_METHOD("get_velocity"), &VehicleBodyT2a::get_velocity);

	ClassDB::bind_method(D_METHOD("get_heave"), &VehicleBodyT2a::get_heave);
	ClassDB::bind_method(D_METHOD("get_heave_velocity"), &VehicleBodyT2a::get_heave_velocity);
	ClassDB::bind_method(D_METHOD("get_pitch"), &VehicleBodyT2a::get_pitch);
	ClassDB::bind_method(D_METHOD("get_pitch_rate"), &VehicleBodyT2a::get_pitch_rate);

	ClassDB::bind_method(D_METHOD("get_front_compression"), &VehicleBodyT2a::get_front_compression);
	ClassDB::bind_method(D_METHOD("get_rear_compression"), &VehicleBodyT2a::get_rear_compression);
	ClassDB::bind_method(D_METHOD("get_front_load"), &VehicleBodyT2a::get_front_load);
	ClassDB::bind_method(D_METHOD("get_rear_load"), &VehicleBodyT2a::get_rear_load);
}

float VehicleBodyT2a::get_yaw() const { return m_yaw; }
float VehicleBodyT2a::get_yaw_rate() const { return m_yaw_rate; }
float VehicleBodyT2a::get_speed() const { return m_velocity.length(); }
Vector3 VehicleBodyT2a::get_velocity() const { return m_velocity; }

float VehicleBodyT2a::get_heave() const { return m_heave; }
float VehicleBodyT2a::get_heave_velocity() const { return m_heave_velocity; }
float VehicleBodyT2a::get_pitch() const { return m_pitch; }
float VehicleBodyT2a::get_pitch_rate() const { return m_pitch_rate; }

float VehicleBodyT2a::get_front_compression() const {
	return (-m_heave - m_pitch * m_cg_to_front) * 1000.0f;
}

float VehicleBodyT2a::get_rear_compression() const {
	return (-m_heave + m_pitch * m_cg_to_rear) * 1000.0f;
}

float VehicleBodyT2a::get_front_load() const {
	const float static_load = m_mass * GRAVITY * (m_cg_to_rear / m_wheel_base);
	const float dev = -m_heave - m_pitch * m_cg_to_front;
	float load = static_load + m_suspension.spring_rate_front * dev;
	return load < 0.0f ? 0.0f : load;
}

float VehicleBodyT2a::get_rear_load() const {
	const float static_load = m_mass * GRAVITY * (m_cg_to_front / m_wheel_base);
	const float dev = -m_heave + m_pitch * m_cg_to_rear;
	float load = static_load + m_suspension.spring_rate_rear * dev;
	return load < 0.0f ? 0.0f : load;
}

void VehicleBodyT2a::step(double p_delta) {
	const float dt = (float)p_delta;
	if (dt <= 0.0f) {
		return;
	}
	if (!m_front_tire.is_valid() || !m_rear_tire.is_valid()) {
		return;
	}

	// --- 1. World velocity -> body frame ---
	// Matches Godot's quaternion convention (T2c).
	const float cy = std::cos(m_yaw);
	const float sy = std::sin(m_yaw);
	const float vx_body = cy * m_velocity.x - sy * m_velocity.z;
	const float vz_body = sy * m_velocity.x + cy * m_velocity.z;

	// --- 2. Steering ---
	const float steer_angle = m_steer_input * m_max_steer_angle;

	// --- 3. Suspension state ---
	const float front_dev = -m_heave - m_pitch * m_cg_to_front;
	const float rear_dev = -m_heave + m_pitch * m_cg_to_rear;

	const float front_vel = m_heave_velocity + m_pitch_rate * m_cg_to_front;
	const float rear_vel = m_heave_velocity - m_pitch_rate * m_cg_to_rear;

	// --- 4. Spring + damper forces ---
	const float f_spring_front = m_suspension.spring_rate_front * front_dev;
	const float f_spring_rear = m_suspension.spring_rate_rear * rear_dev;

	const float f_damper_front = (front_vel > 0.0f)
			? -m_suspension.damper_rebound_front * front_vel
			: -m_suspension.damper_bump_front * front_vel;

	const float f_damper_rear = (rear_vel > 0.0f)
			? -m_suspension.damper_rebound_rear * rear_vel
			: -m_suspension.damper_bump_rear * rear_vel;

	const float f_front_total = f_spring_front + f_damper_front;
	const float f_rear_total = f_spring_rear + f_damper_rear;

	// --- 5. Tire loads ---
	const float static_front = m_mass * GRAVITY * (m_cg_to_rear / m_wheel_base);
	const float static_rear = m_mass * GRAVITY * (m_cg_to_front / m_wheel_base);

	float front_load = static_front + f_front_total;
	float rear_load = static_rear + f_rear_total;
	if (front_load < 0.0f) front_load = 0.0f;
	if (rear_load < 0.0f) rear_load = 0.0f;

	// --- 6. Wheel inputs ---
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

	// --- 7. Tires ---
	const WheelOutput front_out = m_front_tire->step(front_input);
	const WheelOutput rear_out = m_rear_tire->step(rear_input);

	// --- 8. Sum forces ---
	const float fx_body = front_out.force_x + rear_out.force_x;
	const float fy_body = front_out.force_y * std::cos(steer_angle) + rear_out.force_y;
	const float mz_body = rear_out.force_y * m_cg_to_rear
			- front_out.force_y * std::cos(steer_angle) * m_cg_to_front;

	// --- 9. Body -> world ---
	const float cy_w = std::cos(m_yaw);
	const float sy_w = std::sin(m_yaw);
	const float fx_world = cy_w * fx_body + sy_w * fy_body;
	const float fz_world = -sy_w * fx_body + cy_w * fy_body;

	// --- 10. Resistance (base) ---
	apply_resistance(dt);

	// --- 11. Linear ---
	m_velocity.x += (fx_world / m_mass) * dt;
	m_velocity.z += (fz_world / m_mass) * dt;

	// --- 12. Yaw ---
	const float yaw_inertia = m_mass * m_wheel_base * m_wheel_base / 12.0f;
	const float linear_speed = m_velocity.length();
	const float damping_scale = 1.0f + 15.0f / (1.0f + linear_speed);
	const float yaw_damping = -m_yaw_rate * 200.0f * damping_scale;
	const float yaw_accel = (mz_body + yaw_damping) / yaw_inertia;
	m_yaw_rate += yaw_accel * dt;
	m_yaw += m_yaw_rate * dt;
	if (m_yaw_rate > 20.0f) m_yaw_rate = 20.0f;
	if (m_yaw_rate < -20.0f) m_yaw_rate = -20.0f;
	if (m_velocity.length() < 1.0f) m_yaw_rate = 0.0f;

	// --- 13. Heave ---
	const float heave_force = f_front_total + f_rear_total;
	const float heave_accel = heave_force / m_mass;
	m_heave_velocity += heave_accel * dt;
	m_heave += m_heave_velocity * dt;

	// --- 14. Pitch ---
	const float pitch_moment_susp = f_front_total * m_cg_to_front
			- f_rear_total * m_cg_to_rear;
	const float pitch_moment_tire = fx_body * m_cg_height;
	const float pitch_moment_total = pitch_moment_susp + pitch_moment_tire;
	const float pitch_accel = pitch_moment_total / m_pitch_inertia;
	m_pitch_rate += pitch_accel * dt;
	m_pitch += m_pitch_rate * dt;
	if (m_pitch > 0.44f) m_pitch = 0.44f;
	if (m_pitch < -0.44f) m_pitch = -0.44f;

	// --- 15. Position ---
	m_position.x += m_velocity.x * dt;
	m_position.z += m_velocity.z * dt;

	// --- 16. Wheel spin (base) ---
	integrate_wheel_spin(0, front_out.force_x,
			m_throttle * m_max_engine_torque * 0.5f,
			m_brake_input * m_max_brake_torque * 0.7f, dt);
	integrate_wheel_spin(1, rear_out.force_x,
			m_throttle * m_max_engine_torque * 0.5f,
			m_brake_input * m_max_brake_torque * 0.3f, dt);
}