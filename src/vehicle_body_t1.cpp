#include "vehicle_body_t1.h"

#include <cmath>

namespace {
constexpr float EPSILON = 0.01f;
constexpr float GRAVITY = 9.81f;
}

//
// Constructor
//
VehicleBodyT1::VehicleBodyT1() {
	m_front_tire.instantiate();
	m_rear_tire.instantiate();

	// Give both tires the same starting coefficients.
	// They can be tuned independently later.
}

// 
// _bind_methods
//
void VehicleBodyT1::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_throttle", "value"), &VehicleBodyT1::set_throttle);
	ClassDB::bind_method(D_METHOD("get_throttle"), &VehicleBodyT1::get_throttle);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "throttle"), "set_throttle", "get_throttle");

	ClassDB::bind_method(D_METHOD("set_brake", "value"), &VehicleBodyT1::set_brake);
	ClassDB::bind_method(D_METHOD("get_brake"), &VehicleBodyT1::get_brake);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "brake"), "set_brake", "get_brake");

	ClassDB::bind_method(D_METHOD("set_steer", "value"), &VehicleBodyT1::set_steer);
	ClassDB::bind_method(D_METHOD("get_steer"), &VehicleBodyT1::get_steer);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "steer"), "set_steer", "get_steer");

	ClassDB::bind_method(D_METHOD("get_yaw"), &VehicleBodyT1::get_yaw);
	ClassDB::bind_method(D_METHOD("get_yaw_rate"), &VehicleBodyT1::get_yaw_rate);
	ClassDB::bind_method(D_METHOD("get_speed"), &VehicleBodyT1::get_speed);
	ClassDB::bind_method(D_METHOD("get_velocity"), &VehicleBodyT1::get_velocity);
}

// 
// Driver input setters with clamping
// 
void VehicleBodyT1::set_throttle(float p_val) {
	m_throttle = p_val < 0.0f ? 0.0f : (p_val > 1.0f ? 1.0f : p_val);
}

float VehicleBodyT1::get_throttle() const {
	return m_throttle;
}

void VehicleBodyT1::set_brake(float p_val) {
	m_brake_input = p_val < 0.0f ? 0.0f : (p_val > 1.0f ? 1.0f : p_val);
}

float VehicleBodyT1::get_brake() const {
	return m_brake_input;
}

void VehicleBodyT1::set_steer(float p_val) {
	m_steer_input = p_val < -1.0f ? -1.0f : (p_val > 1.0f ? 1.0f : p_val);
}

float VehicleBodyT1::get_steer() const {
	return m_steer_input;
}

//
// State accessors
//
float VehicleBodyT1::get_yaw() const {
	return m_yaw;
}

float VehicleBodyT1::get_yaw_rate() const {
	return m_yaw_rate;
}

float VehicleBodyT1::get_speed() const {
	return m_velocity.length();
}

Vector3 VehicleBodyT1::get_velocity() const {
	return m_velocity;
}

// 
// The physics step
// 
void VehicleBodyT1::step(double p_delta) {
	const float dt = (float)p_delta;
	if (dt <= 0.0f) {
		return;
	}

	// --- 1. Transform world velocity into body frame ---
	// Body frame: x = forward, y = up, z = right
	// We rotate world velocity by -yaw to get body-local components.
	const float cy = std::cos(-m_yaw);
	const float sy = std::sin(-m_yaw);

	const float vx_body = cy * m_velocity.x - sy * m_velocity.z;
	const float vz_body = sy * m_velocity.x + cy * m_velocity.z;

	// --- 2. Steering angle in the body frame ---
	const float steer_angle = m_steer_input * m_max_steer_angle;

	// --- 4. Compute vertical load on each axle ---
	// Static load: mass * gravity split front/rear by CG position.
	// Longitudinal weight transfer under acceleration/braking.
	const float total_load = m_mass * GRAVITY;
	const float static_front = total_load * (m_cg_to_rear / m_wheel_base);
	const float static_rear = total_load * (m_cg_to_front / m_wheel_base);

	// Longitudinal weight transfer: braking shifts load forward, accelerating shifts it back.
	// Approximated from body-frame longitudinal acceleration.
	const float long_accel = 0.0f;  // placeholder - we'll refine after the first integration
	const float transfer = long_accel * m_mass * 0.5f;

	const float front_load = static_front - transfer;
	const float rear_load = static_rear + transfer;

	// --- 5. Build WheelInput for each axle ---
	WheelInput front_input;
    front_input.steer_angle = steer_angle;
	front_input.angular_velocity = m_front_omega;
	front_input.torque = m_throttle * m_max_engine_torque * 0.5f;
	front_input.brake_torque = m_brake_input * m_max_brake_torque * 0.7f;
	front_input.vertical_load = front_load;
	front_input.friction_coefficient = 1.0f;
	front_input.wheel_radius = m_wheel_radius;
	front_input.velocity_longitudinal = vx_body;
	front_input.velocity_lateral = vz_body - m_yaw_rate * m_cg_to_front;

	WheelInput rear_input;
    rear_input.steer_angle = 0.0f;
	rear_input.angular_velocity = m_rear_omega;
	rear_input.torque = m_throttle * m_max_engine_torque * 0.5f;
	rear_input.brake_torque = m_brake_input * m_max_brake_torque * 0.3f;
	rear_input.vertical_load = rear_load;
	rear_input.friction_coefficient = 1.0f;
	rear_input.wheel_radius = m_wheel_radius;
	rear_input.velocity_longitudinal = vx_body;
    rear_input.velocity_lateral = vz_body + m_yaw_rate * m_cg_to_rear;

	// --- 6. Step the tire models ---
	const WheelOutput front_out = m_front_tire->step(front_input);
	const WheelOutput rear_out = m_rear_tire->step(rear_input);

	// --- 7. Sum forces in body frame ---
	// Longitudinal: sum of both tire forces
	const float fx_body = front_out.force_x + rear_out.force_x;

	// Lateral: front tire force rotated by steering angle, plus rear
	const float fy_body = front_out.force_y * std::cos(steer_angle) + rear_out.force_y;

	// Yaw moment: front lateral force * lever arm, minus rear lateral force * its lever arm
	const float mz_body = front_out.force_y * std::cos(steer_angle) * m_cg_to_front
			- rear_out.force_y * m_cg_to_rear;

	// --- 8. Convert body-frame forces back to world frame ---
	const float cy_world = std::cos(m_yaw);
	const float sy_world = std::sin(m_yaw);

	const float fx_world = cy_world * fx_body - sy_world * fy_body;
	const float fz_world = sy_world * fx_body + cy_world * fy_body;
    	// --- 8.5. Rolling resistance + air drag + low-speed damping ---
	// Real cars coast to a stop. Our tire model is unstable below ~2 m/s
	// (a known limitation of slip-based models), so we blend in strong
	// damping at low speed to smoothly bring the car to rest.
	{
		const float speed = m_velocity.length();
		if (speed > 0.001f) {
			const float rolling = 200.0f;             // N, rolling resistance
			const float drag = 0.4f * speed * speed;  // N, aero drag
			const float total_resist = rolling + drag;
			const float resist_decel = (total_resist / m_mass) * dt;

			if (resist_decel >= speed) {
				m_velocity = Vector3(0.0f, 0.0f, 0.0f);
			} else {
				const Vector3 dir = m_velocity / speed;
				m_velocity -= dir * resist_decel;
			}
		}

		// Low-speed damping: when coasting below 2 m/s with no inputs,
		// bleed velocity aggressively. Without this, tire force oscillations
		// keep the car in a phantom "creeping" state forever.
		const float speed_after = m_velocity.length();
		if (speed_after < 2.0f && m_throttle < 0.01f && m_brake_input < 0.01f) {
			const float damping = 5.0f; // s^-1
			m_velocity *= (1.0f - damping * dt);
			if (m_velocity.length() < 0.05f) {
				m_velocity = Vector3(0.0f, 0.0f, 0.0f);
				m_yaw_rate = 0.0f;
			}
		}
	}

	// --- 9. Integrate linear motion ---
	const float ax_world = fx_world / m_mass;
	const float az_world = fz_world / m_mass;

	m_velocity.x += ax_world * dt;
	m_velocity.z += az_world * dt;

	// --- 10. Integrate yaw motion ---
		// --- 10. Integrate yaw motion ---
	// Simplified inertia: mass * wheelbase² / 12, a common approximation.
	const float yaw_inertia = m_mass * m_wheel_base * m_wheel_base / 12.0f;
    	// Yaw damping, scaled up at low linear speed.
	// When the car is stopped but still rotating, the tires see huge
	// apparent lateral velocity from rotation alone and keep producing
	// a yaw moment. Real cars stop spinning because rolling resistance
	// and tire scrub kill rotation at low speed.
	const float linear_speed = m_velocity.length();
	const float damping_scale = 1.0f + 15.0f / (1.0f + linear_speed);
	const float yaw_damping = -m_yaw_rate * 200.0f * damping_scale;
	const float mz_body_damped = mz_body + yaw_damping;

	const float yaw_accel = mz_body_damped / yaw_inertia;

	m_yaw_rate += yaw_accel * dt;
	m_yaw += m_yaw_rate * dt;
    	m_yaw_rate += yaw_accel * dt;
	m_yaw += m_yaw_rate * dt;
    	// When the car is essentially stopped, kill any remaining rotation.
	// Real tires stick to the ground and prevent a stationary car from
	// spinning. Without this, the tire model sees phantom lateral velocity
	// from yaw alone and keeps generating force, causing perpetual spin.
		// When the car is essentially stopped, kill any remaining rotation.
	// Real tires stick and prevent a parked car from spinning. Below this
	// speed threshold, we forcibly zero yaw rate — no partial decay.
	if (m_velocity.length() < 1.0f) {
		m_yaw_rate = 0.0f;
	}
	// Safety clamp: real cars never rotate faster than ~3 rev/s.
	if (m_yaw_rate > 20.0f) m_yaw_rate = 20.0f;
	if (m_yaw_rate < -20.0f) m_yaw_rate = -20.0f;

	// --- 11. Integrate world position ---
	m_position.x += m_velocity.x * dt;
	m_position.z += m_velocity.z * dt;

		// --- 12. Integrate wheel spin ---
	const float front_drive = m_throttle * m_max_engine_torque * 0.5f;
	const float rear_drive = m_throttle * m_max_engine_torque * 0.5f;
	const float front_brake = m_brake_input * m_max_brake_torque * 0.7f;
	const float rear_brake = m_brake_input * m_max_brake_torque * 0.3f;

	const float front_reaction = -front_out.force_x * m_wheel_radius;
	const float rear_reaction = -rear_out.force_x * m_wheel_radius;

	const float axle_inertia = 2.0f * m_wheel_inertia;

	const float front_alpha = (front_drive - front_brake + front_reaction) / axle_inertia;
	const float rear_alpha = (rear_drive - rear_brake + rear_reaction) / axle_inertia;

	m_front_omega += front_alpha * dt;
	m_rear_omega += rear_alpha * dt;

	// Clamp to prevent numerical explosion
	if (m_front_omega < 0.0f) m_front_omega = 0.0f;
	if (m_rear_omega < 0.0f) m_rear_omega = 0.0f;
	if (m_front_omega > 500.0f) m_front_omega = 500.0f;
	if (m_rear_omega > 500.0f) m_rear_omega = 500.0f;

	m_last_speed = m_velocity.length();

	// Diagnostic: print internal state once per second
	static float debug_timer = 0.0f;
	debug_timer += dt;
	if (debug_timer >= 1.0f) {
		debug_timer = 0.0f;
		print_line(vformat(
			"[DBG] vx=%.1f vz_body=%.2f yaw_rate=%.3f steer=%.3f | "
			"F_omega=%.1f R_omega=%.1f | "
			"F_Fx=%.0f R_Fx=%.0f | F_Fy=%.0f R_Fy=%.0f | "
			"F_slip=%.3f R_slip=%.3f",
			vx_body, vz_body, m_yaw_rate, steer_angle,
			m_front_omega, m_rear_omega,
			front_out.force_x, rear_out.force_x,
			front_out.force_y, rear_out.force_y,
			front_out.slip_ratio, rear_out.slip_ratio));
	}
}	
