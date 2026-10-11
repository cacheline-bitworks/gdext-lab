#include "vehicle_body_t2b.h"

#include <cmath>

namespace {
constexpr float GRAVITY = 9.81f;
}

// Corner index constants for clarity
// 0 = front-left, 1 = front-right, 2 = rear-left, 3 = rear-right

// ----------------------------------------------------------------------------
// Constructor
// ----------------------------------------------------------------------------
VehicleBodyT2b::VehicleBodyT2b() {
	for (int i = 0; i < 4; i++) {
		m_tires[i].instantiate();
	}
}

// ----------------------------------------------------------------------------
// _bind_methods
// ----------------------------------------------------------------------------
void VehicleBodyT2b::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_throttle", "value"), &VehicleBodyT2b::set_throttle);
	ClassDB::bind_method(D_METHOD("get_throttle"), &VehicleBodyT2b::get_throttle);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "throttle"), "set_throttle", "get_throttle");

	ClassDB::bind_method(D_METHOD("set_brake", "value"), &VehicleBodyT2b::set_brake);
	ClassDB::bind_method(D_METHOD("get_brake"), &VehicleBodyT2b::get_brake);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "brake"), "set_brake", "get_brake");

	ClassDB::bind_method(D_METHOD("set_steer", "value"), &VehicleBodyT2b::set_steer);
	ClassDB::bind_method(D_METHOD("get_steer"), &VehicleBodyT2b::get_steer);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "steer"), "set_steer", "get_steer");

	ClassDB::bind_method(D_METHOD("get_yaw"), &VehicleBodyT2b::get_yaw);
	ClassDB::bind_method(D_METHOD("get_yaw_rate"), &VehicleBodyT2b::get_yaw_rate);
	ClassDB::bind_method(D_METHOD("get_speed"), &VehicleBodyT2b::get_speed);
	ClassDB::bind_method(D_METHOD("get_velocity"), &VehicleBodyT2b::get_velocity);

	ClassDB::bind_method(D_METHOD("get_roll"), &VehicleBodyT2b::get_roll);
	ClassDB::bind_method(D_METHOD("get_roll_rate"), &VehicleBodyT2b::get_roll_rate);

	ClassDB::bind_method(D_METHOD("get_corner_load", "index"), &VehicleBodyT2b::get_corner_load);
	ClassDB::bind_method(D_METHOD("get_corner_compression", "index"), &VehicleBodyT2b::get_corner_compression);
}

// ----------------------------------------------------------------------------
// Driver inputs
// ----------------------------------------------------------------------------
void VehicleBodyT2b::set_throttle(float p_val) {
	m_throttle = p_val < 0.0f ? 0.0f : (p_val > 1.0f ? 1.0f : p_val);
}
float VehicleBodyT2b::get_throttle() const { return m_throttle; }

void VehicleBodyT2b::set_brake(float p_val) {
	m_brake_input = p_val < 0.0f ? 0.0f : (p_val > 1.0f ? 1.0f : p_val);
}
float VehicleBodyT2b::get_brake() const { return m_brake_input; }

void VehicleBodyT2b::set_steer(float p_val) {
	m_steer_input = p_val < -1.0f ? -1.0f : (p_val > 1.0f ? 1.0f : p_val);
}
float VehicleBodyT2b::get_steer() const { return m_steer_input; }

// ----------------------------------------------------------------------------
// State accessors
// ----------------------------------------------------------------------------
float VehicleBodyT2b::get_yaw() const { return m_yaw; }
float VehicleBodyT2b::get_yaw_rate() const { return m_yaw_rate; }
float VehicleBodyT2b::get_speed() const { return m_velocity.length(); }
Vector3 VehicleBodyT2b::get_velocity() const { return m_velocity; }
float VehicleBodyT2b::get_roll() const { return m_roll; }
float VehicleBodyT2b::get_roll_rate() const { return m_roll_rate; }

// Corner positions in body frame (x = forward, z = right)
static void get_corner_position(int idx, float cg_front, float cg_rear,
		float track_width, float &out_x, float &out_z) {
	switch (idx) {
		case 0: out_x = cg_front; out_z = -track_width * 0.5f; break;  // FL
		case 1: out_x = cg_front; out_z = track_width * 0.5f; break;   // FR
		case 2: out_x = -cg_rear; out_z = -track_width * 0.5f; break;  // RL
		default: out_x = -cg_rear; out_z = track_width * 0.5f; break;  // RR
	}
}

float VehicleBodyT2b::get_corner_compression(int p_index) const {
	if (p_index < 0 || p_index > 3) return 0.0f;
	float x, z;
	get_corner_position(p_index, m_cg_to_front, m_cg_to_rear, m_track_width, x, z);
	const float compression = -m_heave - m_pitch * x + m_roll * z;
	return compression * 1000.0f;  // to mm
}

float VehicleBodyT2b::get_corner_load(int p_index) const {
	if (p_index < 0 || p_index > 3) return 0.0f;

	const CornerConfig *cfg = nullptr;
	switch (p_index) {
		case 0: cfg = &m_front_left; break;
		case 1: cfg = &m_front_right; break;
		case 2: cfg = &m_rear_left; break;
		default: cfg = &m_rear_right; break;
	}

	float x, z;
	get_corner_position(p_index, m_cg_to_front, m_cg_to_rear, m_track_width, x, z);

	const float total_weight = m_mass * GRAVITY;
	const float static_front = total_weight * (m_cg_to_rear / m_wheel_base) * 0.5f;
	const float static_rear = total_weight * (m_cg_to_front / m_wheel_base) * 0.5f;
	const float static_load = (p_index < 2) ? static_front : static_rear;

	const float compression = -m_heave - m_pitch * x + m_roll * z;
	float load = static_load + cfg->spring_rate * compression;
	return load < 0.0f ? 0.0f : load;
}

// ----------------------------------------------------------------------------
// The physics step
// ----------------------------------------------------------------------------
void VehicleBodyT2b::step(double p_delta) {
	const float dt = (float)p_delta;
	if (dt <= 0.0f) return;

	// --- 1. Body-frame velocity ---
	const float cy_b = std::cos(-m_yaw);
	const float sy_b = std::sin(-m_yaw);
	const float vx_body = cy_b * m_velocity.x - sy_b * m_velocity.z;
	const float vz_body = sy_b * m_velocity.x + cy_b * m_velocity.z;

	// --- 2. Steering ---
	const float steer_angle = m_steer_input * m_max_steer_angle;

	// --- 3. Corner geometry ---
	float x_pos[4];
	float z_pos[4];
	for (int i = 0; i < 4; i++) {
		get_corner_position(i, m_cg_to_front, m_cg_to_rear, m_track_width,
				x_pos[i], z_pos[i]);
	}

	CornerConfig *configs[4] = { &m_front_left, &m_front_right, &m_rear_left, &m_rear_right };

	// --- 4. Static load per corner ---
	const float total_weight = m_mass * GRAVITY;
	const float static_front_each = total_weight * (m_cg_to_rear / m_wheel_base) * 0.5f;
	const float static_rear_each = total_weight * (m_cg_to_front / m_wheel_base) * 0.5f;

	// --- 5. Per-corner suspension: compression, velocity, forces, load ---
	float compression[4];
	float corner_vel[4];
	float spring_force[4];
	float damper_force[4];
	float corner_load[4];

	for (int i = 0; i < 4; i++) {
		compression[i] = -m_heave - m_pitch * x_pos[i] + m_roll * z_pos[i];
		corner_vel[i] = m_heave_velocity + m_pitch_rate * x_pos[i] - m_roll_rate * z_pos[i];

		spring_force[i] = configs[i]->spring_rate * compression[i];

		if (corner_vel[i] > 0.0f) {
			damper_force[i] = -configs[i]->damper_rebound * corner_vel[i];
		} else {
			damper_force[i] = -configs[i]->damper_bump * corner_vel[i];
		}

		const float static_load = (i < 2) ? static_front_each : static_rear_each;
		corner_load[i] = static_load + spring_force[i] + damper_force[i];
		if (corner_load[i] < 0.0f) corner_load[i] = 0.0f;
	}

	// --- 6. Per-corner tire step, sum body-frame forces ---
	float total_Fx_body = 0.0f;
	float total_Fz_body = 0.0f;
	float M_yaw = 0.0f;
    WheelOutput tire_out[4];

	// Per-wheel drive and brake torques (25% drive per wheel, brakes split 70/30)
	const float drive_per_wheel = m_throttle * m_max_engine_torque * 0.25f;

	for (int i = 0; i < 4; i++) {
		const float corner_steer = (i < 2) ? steer_angle : 0.0f;

		// Lateral velocity at the corner (yaw rate contribution only, per T2a)
		const float vz_corner = vz_body - m_yaw_rate * x_pos[i];

		WheelInput in;
		in.steer_angle = corner_steer;
		in.angular_velocity = m_omega[i];
		in.torque = drive_per_wheel;
		if (i < 2) {
			in.brake_torque = m_brake_input * m_max_brake_torque * 0.35f;
		} else {
			in.brake_torque = m_brake_input * m_max_brake_torque * 0.15f;
		}
		in.vertical_load = corner_load[i];
		in.friction_coefficient = 1.0f;
		in.wheel_radius = m_wheel_radius;
		in.velocity_longitudinal = vx_body;
		in.velocity_lateral = vz_corner;

		const WheelOutput out = m_tires[i]->step(in);
        tire_out[i] = out;

		// Rotate tire-local force into body frame.
		// Tire's forward direction in body frame: (cos s, 0, sin s)
		// Tire's lateral direction in body frame: (-sin s, 0, cos s)
		const float cs = std::cos(corner_steer);
		const float ss = std::sin(corner_steer);
		const float Fx_body_i = out.force_x * cs - out.force_y * ss;
		const float Fz_body_i = out.force_x * ss + out.force_y * cs;

		total_Fx_body += Fx_body_i;
		total_Fz_body += Fz_body_i;

		M_yaw += Fz_body_i * x_pos[i];
	}

	// --- 7. World-frame horizontal forces ---
	const float cy_w = std::cos(m_yaw);
	const float sy_w = std::sin(m_yaw);
	const float fx_world = cy_w * total_Fx_body - sy_w * total_Fz_body;
	const float fz_world = sy_w * total_Fx_body + cy_w * total_Fz_body;

	// --- 8. Moments ---
	// Pitch: spring forces + tire longitudinal force at ground level
	float M_pitch = 0.0f;
	for (int i = 0; i < 4; i++) {
		const float F_up = spring_force[i] + damper_force[i];
		M_pitch += F_up * x_pos[i];
	}
	M_pitch += total_Fx_body * m_cg_height;

	// Roll: spring forces + tire lateral force at ground level
	float M_roll = 0.0f;
	for (int i = 0; i < 4; i++) {
		const float F_up = spring_force[i] + damper_force[i];
		M_roll -= F_up * z_pos[i];
	}
	M_roll += -m_cg_height * total_Fz_body;

	// --- 9. Rolling resistance + low-speed damping ---
	{
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
				m_yaw_rate = 0.0f;
			}
		}
	}

	// --- 10. Integrate horizontal motion ---
	m_velocity.x += (fx_world / m_mass) * dt;
	m_velocity.z += (fz_world / m_mass) * dt;

	// --- 11. Integrate yaw ---
	// Yaw inertia. The uniform-rod formula (/12) underestimates real cars by
// 3-5x because car mass is concentrated near the center, not at the ends.
// 3000 kg·m² is realistic for a 1200 kg sedan.
    const float yaw_inertia = 3000.0f;
	const float linear_speed = m_velocity.length();
	const float damping_scale = 1.0f + 15.0f / (1.0f + linear_speed);
	const float yaw_damping = -m_yaw_rate * 200.0f * damping_scale;
	const float yaw_accel = (M_yaw + yaw_damping) / yaw_inertia;
	m_yaw_rate += yaw_accel * dt;
	m_yaw += m_yaw_rate * dt;
	if (m_yaw_rate > 20.0f) m_yaw_rate = 20.0f;
	if (m_yaw_rate < -20.0f) m_yaw_rate = -20.0f;
	// Kill rotation more aggressively at low speed. Without this, a slow
// spinout can persist indefinitely because the tires keep producing
// lateral force from yaw alone.
if (m_velocity.length() < 3.0f) {
    m_yaw_rate *= 0.7f;
    if (std::fabs(m_yaw_rate) < 0.05f) m_yaw_rate = 0.0f;
}

	// --- 12. Integrate heave ---
	float heave_force = 0.0f;
	for (int i = 0; i < 4; i++) {
		heave_force += spring_force[i] + damper_force[i];
	}
	m_heave_velocity += (heave_force / m_mass) * dt;
	m_heave += m_heave_velocity * dt;

	// --- 13. Integrate pitch ---
	const float pitch_accel = M_pitch / m_pitch_inertia;
	m_pitch_rate += pitch_accel * dt;
	m_pitch += m_pitch_rate * dt;
	if (m_pitch > 0.44f) m_pitch = 0.44f;
	if (m_pitch < -0.44f) m_pitch = -0.44f;

	// --- 14. Integrate roll ---
	const float roll_accel = M_roll / m_roll_inertia;
	m_roll_rate += roll_accel * dt;
	m_roll += m_roll_rate * dt;
	if (m_roll > 0.35f) m_roll = 0.35f;
	if (m_roll < -0.35f) m_roll = -0.35f;

	// --- 15. Integrate position ---
	m_position.x += m_velocity.x * dt;
	m_position.z += m_velocity.z * dt;

		// --- 16. Integrate wheel spin per corner ---
	for (int i = 0; i < 4; i++) {
		const float brake_torque = (i < 2)
				? m_brake_input * m_max_brake_torque * 0.35f
				: m_brake_input * m_max_brake_torque * 0.15f;

		// Tire reaction torque opposes wheel spin
		const float reaction = -tire_out[i].force_x * m_wheel_radius;

		const float alpha = (drive_per_wheel - brake_torque + reaction) / m_wheel_inertia;
		m_omega[i] += alpha * dt;

		if (m_omega[i] < 0.0f) m_omega[i] = 0.0f;
		if (m_omega[i] > 500.0f) m_omega[i] = 500.0f;
	}
}