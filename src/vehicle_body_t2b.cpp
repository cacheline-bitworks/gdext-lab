#include "vehicle_body_t2b.h"

#include <cmath>

namespace {
constexpr float GRAVITY = 9.81f;
}

// Body-frame corner position: (x_offset, z_offset)
// Order: 0=FL, 1=FR, 2=RL, 3=RR
static void get_corner_position(int idx, float cg_front, float cg_rear,
		float track_width, float &out_x, float &out_z) {
	switch (idx) {
		case 0: out_x = cg_front; out_z = -track_width * 0.5f; break;
		case 1: out_x = cg_front; out_z = track_width * 0.5f; break;
		case 2: out_x = -cg_rear; out_z = -track_width * 0.5f; break;
		default: out_x = -cg_rear; out_z = track_width * 0.5f; break;
	}
}

VehicleBodyT2b::VehicleBodyT2b() {
	for (int i = 0; i < 4; i++) {
		m_tires[i].instantiate();
	}
}

void VehicleBodyT2b::_bind_methods() {
	ClassDB::bind_method(D_METHOD("get_yaw"), &VehicleBodyT2b::get_yaw);
	ClassDB::bind_method(D_METHOD("get_yaw_rate"), &VehicleBodyT2b::get_yaw_rate);
	ClassDB::bind_method(D_METHOD("get_speed"), &VehicleBodyT2b::get_speed);
	ClassDB::bind_method(D_METHOD("get_velocity"), &VehicleBodyT2b::get_velocity);
	ClassDB::bind_method(D_METHOD("get_roll"), &VehicleBodyT2b::get_roll);
	ClassDB::bind_method(D_METHOD("get_roll_rate"), &VehicleBodyT2b::get_roll_rate);
	ClassDB::bind_method(D_METHOD("get_corner_load", "index"), &VehicleBodyT2b::get_corner_load);
	ClassDB::bind_method(D_METHOD("get_corner_compression", "index"), &VehicleBodyT2b::get_corner_compression);
}

float VehicleBodyT2b::get_yaw() const { return m_yaw; }
float VehicleBodyT2b::get_yaw_rate() const { return m_yaw_rate; }
float VehicleBodyT2b::get_speed() const { return m_velocity.length(); }
Vector3 VehicleBodyT2b::get_velocity() const { return m_velocity; }
float VehicleBodyT2b::get_roll() const { return m_roll; }
float VehicleBodyT2b::get_roll_rate() const { return m_roll_rate; }

float VehicleBodyT2b::get_corner_compression(int p_index) const {
	if (p_index < 0 || p_index > 3) return 0.0f;
	float x, z;
	get_corner_position(p_index, m_cg_to_front, m_cg_to_rear, m_track_width, x, z);
	const float compression = -m_heave - m_pitch * x + m_roll * z;
	return compression * 1000.0f;
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

void VehicleBodyT2b::step(double p_delta) {
	const float dt = (float)p_delta;
	if (dt <= 0.0f) return;

	for (int i = 0; i < 4; i++) {
		if (!m_tires[i].is_valid()) return;
	}

	// --- 1. World velocity -> body frame ---
	// Matches Godot's quaternion convention (T2c).
	const float cy_b = std::cos(m_yaw);
	const float sy_b = std::sin(m_yaw);
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

	// --- 4. Static loads ---
	const float total_weight = m_mass * GRAVITY;
	const float static_front_each = total_weight * (m_cg_to_rear / m_wheel_base) * 0.5f;
	const float static_rear_each = total_weight * (m_cg_to_front / m_wheel_base) * 0.5f;

	// --- 5. Per-corner suspension ---
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

	// --- 6. Tires ---
	float total_Fx_body = 0.0f;
	float total_Fz_body = 0.0f;
	float M_yaw = 0.0f;
	WheelOutput tire_out[4];

	const float drive_per_wheel = m_throttle * m_max_engine_torque * 0.25f;

	for (int i = 0; i < 4; i++) {
		const float corner_steer = (i < 2) ? steer_angle : 0.0f;
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

		const float cs = std::cos(corner_steer);
		const float ss = std::sin(corner_steer);
		const float Fx_body_i = out.force_x * cs - out.force_y * ss;
		const float Fz_body_i = out.force_x * ss + out.force_y * cs;

		total_Fx_body += Fx_body_i;
		total_Fz_body += Fz_body_i;
		M_yaw -= Fz_body_i * x_pos[i];
	}

	// --- 7. Body forces -> world ---
	const float cy_w = std::cos(m_yaw);
	const float sy_w = std::sin(m_yaw);
	const float fx_world = cy_w * total_Fx_body + sy_w * total_Fz_body;
	const float fz_world = -sy_w * total_Fx_body + cy_w * total_Fz_body;

	// --- 8. Moments ---
	float M_pitch = 0.0f;
	for (int i = 0; i < 4; i++) {
		const float F_up = spring_force[i] + damper_force[i];
		M_pitch += F_up * x_pos[i];
	}
	M_pitch += total_Fx_body * m_cg_height;

	float M_roll = 0.0f;
	for (int i = 0; i < 4; i++) {
		const float F_up = spring_force[i] + damper_force[i];
		M_roll -= F_up * z_pos[i];
	}
	M_roll += -m_cg_height * total_Fz_body;

	// --- 9. Resistance (base) ---
	apply_resistance(dt);

	// --- 10. Linear ---
	m_velocity.x += (fx_world / m_mass) * dt;
	m_velocity.z += (fz_world / m_mass) * dt;

	// --- 11. Yaw ---
	const float yaw_inertia = m_mass * m_wheel_base * m_wheel_base / 12.0f;
	const float linear_speed = m_velocity.length();
	const float damping_scale = 1.0f + 15.0f / (1.0f + linear_speed);
	const float yaw_damping = -m_yaw_rate * 200.0f * damping_scale;
	const float yaw_accel = (M_yaw + yaw_damping) / yaw_inertia;
	m_yaw_rate += yaw_accel * dt;
	m_yaw += m_yaw_rate * dt;
	if (m_yaw_rate > 20.0f) m_yaw_rate = 20.0f;
	if (m_yaw_rate < -20.0f) m_yaw_rate = -20.0f;
	if (m_velocity.length() < 1.0f) m_yaw_rate = 0.0f;

	// --- 12. Heave ---
	float heave_force = 0.0f;
	for (int i = 0; i < 4; i++) {
		heave_force += spring_force[i] + damper_force[i];
	}
	m_heave_velocity += (heave_force / m_mass) * dt;
	m_heave += m_heave_velocity * dt;

	// --- 13. Pitch ---
	const float pitch_accel = M_pitch / m_pitch_inertia;
	m_pitch_rate += pitch_accel * dt;
	m_pitch += m_pitch_rate * dt;
	if (m_pitch > 0.44f) m_pitch = 0.44f;
	if (m_pitch < -0.44f) m_pitch = -0.44f;

	// --- 14. Roll ---
	const float roll_accel = M_roll / m_roll_inertia;
	m_roll_rate += roll_accel * dt;
	m_roll += m_roll_rate * dt;
	if (m_roll > 0.35f) m_roll = 0.35f;
	if (m_roll < -0.35f) m_roll = -0.35f;

	// --- 15. Position ---
	m_position.x += m_velocity.x * dt;
	m_position.z += m_velocity.z * dt;

	// --- 16. Wheel spin (base) ---
	for (int i = 0; i < 4; i++) {
		const float brake_torque = (i < 2)
				? m_brake_input * m_max_brake_torque * 0.35f
				: m_brake_input * m_max_brake_torque * 0.15f;
		integrate_wheel_spin(i, tire_out[i].force_x, drive_per_wheel,
				brake_torque, dt);
	}
}