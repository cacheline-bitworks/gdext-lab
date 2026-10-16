#include "vehicle_body_t2c.h"

#include <cmath>

namespace {
constexpr float GRAVITY = 9.81f;
constexpr float AIR_DENSITY = 1.225f;
}

// Body frame: X forward, Y up, Z right

VehicleBodyT2c::VehicleBodyT2c() {
	// Default tires: V2 at all four corners.
	for (int i = 0; i < 4; i++) {
		Ref<TireModelV2> v2;
		v2.instantiate();
		m_tires[i] = v2;
	}
	m_position.y = m_eq_height;
	m_orientation = Quaternion();
}

void VehicleBodyT2c::_bind_methods() {
	ClassDB::bind_method(D_METHOD("get_yaw"), &VehicleBodyT2c::get_yaw);
	ClassDB::bind_method(D_METHOD("get_pitch"), &VehicleBodyT2c::get_pitch);
	ClassDB::bind_method(D_METHOD("get_roll"), &VehicleBodyT2c::get_roll);
	ClassDB::bind_method(D_METHOD("get_speed"), &VehicleBodyT2c::get_speed);
	ClassDB::bind_method(D_METHOD("get_velocity"), &VehicleBodyT2c::get_velocity);
	ClassDB::bind_method(D_METHOD("get_orientation"), &VehicleBodyT2c::get_orientation);
	ClassDB::bind_method(D_METHOD("get_angular_velocity"), &VehicleBodyT2c::get_angular_velocity);

	ClassDB::bind_method(D_METHOD("set_drag_coefficient", "value"), &VehicleBodyT2c::set_drag_coefficient);
	ClassDB::bind_method(D_METHOD("get_drag_coefficient"), &VehicleBodyT2c::get_drag_coefficient);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "drag_coefficient"), "set_drag_coefficient", "get_drag_coefficient");

	ClassDB::bind_method(D_METHOD("set_downforce_coefficient", "value"), &VehicleBodyT2c::set_downforce_coefficient);
	ClassDB::bind_method(D_METHOD("get_downforce_coefficient"), &VehicleBodyT2c::get_downforce_coefficient);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "downforce_coefficient"), "set_downforce_coefficient", "get_downforce_coefficient");

	ClassDB::bind_method(D_METHOD("set_frontal_area", "value"), &VehicleBodyT2c::set_frontal_area);
	ClassDB::bind_method(D_METHOD("get_frontal_area"), &VehicleBodyT2c::get_frontal_area);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "frontal_area"), "set_frontal_area", "get_frontal_area");

	ClassDB::bind_method(D_METHOD("get_corner_load", "index"), &VehicleBodyT2c::get_corner_load);
	ClassDB::bind_method(D_METHOD("get_corner_compression", "index"), &VehicleBodyT2c::get_corner_compression);
	ClassDB::bind_method(D_METHOD("get_steer_angle_for_corner", "index"), &VehicleBodyT2c::get_steer_angle_for_corner);

		ClassDB::bind_method(D_METHOD("get_corner_count"), &VehicleBodyT2c::get_corner_count);
	ClassDB::bind_method(D_METHOD("set_corner_tire", "index", "tire"), &VehicleBodyT2c::set_corner_tire);
	ClassDB::bind_method(D_METHOD("get_corner_tire", "index"), &VehicleBodyT2c::get_corner_tire);
}

// 
// State accessors
// 
float VehicleBodyT2c::get_speed() const { return m_velocity.length(); }
Vector3 VehicleBodyT2c::get_velocity() const { return m_velocity; }
Quaternion VehicleBodyT2c::get_orientation() const { return m_orientation; }
Vector3 VehicleBodyT2c::get_angular_velocity() const { return m_angular_velocity; }

float VehicleBodyT2c::get_yaw() const {
	Vector3 forward_body(1, 0, 0);
	Vector3 forward_world = m_orientation.xform(forward_body);
	return std::atan2(-forward_world.z, forward_world.x);
}

float VehicleBodyT2c::get_pitch() const {
	Vector3 forward_world = m_orientation.xform(Vector3(1, 0, 0));
	return std::atan2(forward_world.y,
			std::sqrt(forward_world.x * forward_world.x + forward_world.z * forward_world.z));
}

float VehicleBodyT2c::get_roll() const {
	Vector3 up_world = m_orientation.xform(Vector3(0, 1, 0));
	return std::atan2(up_world.z, up_world.y);
}

// 
// Aero accessors
// 
void VehicleBodyT2c::set_drag_coefficient(float p_val) { m_drag_coefficient = p_val; }
float VehicleBodyT2c::get_drag_coefficient() const { return m_drag_coefficient; }
void VehicleBodyT2c::set_downforce_coefficient(float p_val) { m_downforce_coefficient = p_val; }
float VehicleBodyT2c::get_downforce_coefficient() const { return m_downforce_coefficient; }
void VehicleBodyT2c::set_frontal_area(float p_val) { m_frontal_area = p_val; }
float VehicleBodyT2c::get_frontal_area() const { return m_frontal_area; }

// 
// Corner helpers
// 
static Vector3 get_body_corner(int idx, float cg_front, float cg_rear,
		float track_width, float eq_height) {
	switch (idx) {
		case 0: return Vector3(cg_front, -eq_height, -track_width * 0.5f);
		case 1: return Vector3(cg_front, -eq_height, track_width * 0.5f);
		case 2: return Vector3(-cg_rear, -eq_height, -track_width * 0.5f);
		default: return Vector3(-cg_rear, -eq_height, track_width * 0.5f);
	}
}

static float get_static_load(int idx, float mass, float cg_front, float cg_rear,
		float wheel_base) {
	const float weight = mass * GRAVITY;
	if (idx < 2) {
		return weight * (cg_rear / wheel_base) * 0.5f;
	}
	return weight * (cg_front / wheel_base) * 0.5f;
}

float VehicleBodyT2c::get_corner_compression(int p_index) const {
	if (p_index < 0 || p_index > 3) return 0.0f;
	const Vector3 body_c = get_body_corner(p_index, m_cg_to_front, m_cg_to_rear,
			m_track_width, m_eq_height);
	const Vector3 world_c = m_position + m_orientation.xform(body_c);
	return -world_c.y * 1000.0f;
}

float VehicleBodyT2c::get_corner_load(int p_index) const {
	if (p_index < 0 || p_index > 3) return 0.0f;

	const CornerConfigT2c *cfg = nullptr;
	switch (p_index) {
		case 0: cfg = &m_front_left; break;
		case 1: cfg = &m_front_right; break;
		case 2: cfg = &m_rear_left; break;
		default: cfg = &m_rear_right; break;
	}

	const Vector3 body_c = get_body_corner(p_index, m_cg_to_front, m_cg_to_rear,
			m_track_width, m_eq_height);
	const Vector3 world_c = m_position + m_orientation.xform(body_c);
	const float compression = -world_c.y;

	// Spring force
	const float spring_f = cfg->spring_rate * compression;

	// Damper force — depends on vertical velocity at this corner
	const Vector3 omega_world = m_orientation.xform(m_angular_velocity);
	const Vector3 r = world_c - m_position;
	const Vector3 corner_vel_world = m_velocity + omega_world.cross(r);
	const float corner_vel_y = corner_vel_world.y;

	float damper_f;
	if (corner_vel_y > 0.0f) {
		damper_f = -cfg->damper_rebound * corner_vel_y;
	} else {
		damper_f = -cfg->damper_bump * corner_vel_y;
	}

	// Anti-roll bar contribution via subclass hook
	const int opposite = (p_index == 0) ? 1 : (p_index == 1) ? 0 : (p_index == 2) ? 3 : 2;
	const Vector3 opp_body_c = get_body_corner(opposite, m_cg_to_front, m_cg_to_rear,
			m_track_width, m_eq_height);
	const Vector3 opp_world_c = m_position + m_orientation.xform(opp_body_c);
	const float opp_compression = -opp_world_c.y;
	const float extra_force = extra_corner_force(p_index, compression, opp_compression);

	const float static_load = get_static_load(p_index, m_mass, m_cg_to_front,
			m_cg_to_rear, m_wheel_base);

	float load = static_load + spring_f + damper_f + extra_force;
	return load < 0.0f ? 0.0f : load;
}

float VehicleBodyT2c::get_steer_angle_for_corner(int p_index) const {
	const float base = m_steer_input * m_max_steer_angle;
	return corner_steer_angle(p_index, base);
}

// 
// Default hooks (overridden by subclasses)
// 
float VehicleBodyT2c::extra_corner_force(int, float, float) const {
	return 0.0f;
}

float VehicleBodyT2c::corner_steer_angle(int p_corner_index, float p_base_steer) const {
	return (p_corner_index < 2) ? p_base_steer : 0.0f;
}

float VehicleBodyT2c::corner_camber_angle(int) const {
	return 0.0f;
}

// 
// The physics step
// 
void VehicleBodyT2c::step(double p_delta) {
	const float dt = (float)p_delta;
	if (dt <= 0.0f) return;

	for (int i = 0; i < 4; i++) {
		if (!m_tires[i].is_valid()) return;
	}

	// --- 1. Force and torque accumulators ---
	Vector3 total_force_world(0.0f, -m_mass * GRAVITY, 0.0f);
	Vector3 total_torque_world(0.0f, 0.0f, 0.0f);

	// --- 2. Aero forces ---
	const float speed_sq = m_velocity.length_squared();
	if (speed_sq > 0.01f) {
		const float q = 0.5f * AIR_DENSITY * speed_sq;
		const float drag_mag = q * m_drag_coefficient * m_frontal_area;
		const Vector3 vel_dir = m_velocity.normalized();
		total_force_world += -vel_dir * drag_mag;

		const float down_mag = q * m_downforce_coefficient * m_frontal_area;
		const Vector3 local_down = m_orientation.xform(Vector3(0.0f, -1.0f, 0.0f));
		total_force_world += local_down * down_mag;
	}

	// --- 3. World-frame angular velocity ---
	Vector3 omega_world = m_orientation.xform(m_angular_velocity);

	// --- 4. Steering ---
	const float steer_angle = m_steer_input * m_max_steer_angle;

	CornerConfigT2c *configs[4] = {
		&m_front_left, &m_front_right, &m_rear_left, &m_rear_right
	};

	// --- 5. Precompute compressions for ARB hook ---
	float compressions[4];
	for (int i = 0; i < 4; i++) {
		const Vector3 body_c = get_body_corner(i, m_cg_to_front, m_cg_to_rear,
				m_track_width, m_eq_height);
		const Vector3 world_c = m_position + m_orientation.xform(body_c);
		compressions[i] = -world_c.y;
	}

	// --- 6. Per-corner ---
	WheelOutput tire_out[4];

	for (int i = 0; i < 4; i++) {
		const Vector3 body_c = get_body_corner(i, m_cg_to_front, m_cg_to_rear,
				m_track_width, m_eq_height);
		const Vector3 corner_world = m_position + m_orientation.xform(body_c);
		const float compression = -corner_world.y;

		const int opposite = (i == 0) ? 1 : (i == 1) ? 0 : (i == 2) ? 3 : 2;
		const float extra_force = extra_corner_force(i, compressions[i], compressions[opposite]);

		const Vector3 r = corner_world - m_position;
		const Vector3 corner_vel_world = m_velocity + omega_world.cross(r);

		const float corner_vel_y = corner_vel_world.y;
		float damper_f;
		if (corner_vel_y > 0.0f) {
			damper_f = -configs[i]->damper_rebound * corner_vel_y;
		} else {
			damper_f = -configs[i]->damper_bump * corner_vel_y;
		}

		const float spring_f = configs[i]->spring_rate * compression;
		const float static_load = get_static_load(i, m_mass, m_cg_to_front,
				m_cg_to_rear, m_wheel_base);

		float tire_load = static_load + spring_f + damper_f + extra_force;
		if (tire_load < 0.0f) tire_load = 0.0f;

		const Vector3 suspension_force_world(0.0f, static_load + spring_f + damper_f + extra_force, 0.0f);
		total_force_world += suspension_force_world;
		total_torque_world += r.cross(suspension_force_world);

		const Vector3 contact_world(corner_world.x, 0.0f, corner_world.z);
		const Vector3 r_contact = contact_world - m_position;
		const Vector3 contact_vel_world = m_velocity + omega_world.cross(r_contact);
		const Vector3 contact_vel_body = m_orientation.inverse().xform(contact_vel_world);

		const float corner_steer = corner_steer_angle(i, steer_angle);

		WheelInput in;
        in.dt = dt;
		in.steer_angle = corner_steer;
		in.camber_angle = corner_camber_angle(i);
		in.angular_velocity = m_omega[i];
		in.torque = m_throttle * m_max_engine_torque * 0.25f;
		in.brake_torque = (i < 2)
				? m_brake_input * m_max_brake_torque * 0.35f
				: m_brake_input * m_max_brake_torque * 0.15f;
		in.vertical_load = tire_load;
		in.friction_coefficient = 1.0f;
		in.wheel_radius = m_wheel_radius;
		in.velocity_longitudinal = contact_vel_body.x;
		in.velocity_lateral = contact_vel_body.z;

		tire_out[i] = m_tires[i]->step(in);

		const float cs = std::cos(corner_steer);
		const float ss = std::sin(corner_steer);
		const Vector3 tire_force_body(
			tire_out[i].force_x * cs - tire_out[i].force_y * ss,
			0.0f,
			tire_out[i].force_x * ss + tire_out[i].force_y * cs
		);
		const Vector3 tire_force_world = m_orientation.xform(tire_force_body);

		total_force_world += tire_force_world;
		total_torque_world += r_contact.cross(tire_force_world);
	}

	// --- 7. Low-speed damping handled by base apply_resistance ---
	apply_resistance(dt);

	// --- 8. Linear integration ---
	const Vector3 accel_world = total_force_world / m_mass;
	m_velocity += accel_world * dt;
	m_position += m_velocity * dt;

	if (m_position.y < m_eq_height * 0.3f) {
		m_position.y = m_eq_height * 0.3f;
		if (m_velocity.y < 0.0f) m_velocity.y = 0.0f;
	}

	// --- 9. Angular integration ---
	const Vector3 torque_body_raw = m_orientation.inverse().xform(total_torque_world);

	
	// Yaw-only damping. Roll and pitch are damped by the suspension's own
	// damper forces (which act at each corner and produce the correct
	// torques). Applying this damping to all three axes double-damps roll
	// and pitch, killing the natural oscillation that real cars have.
	const float linear_speed = m_velocity.length();
	const float damping_scale = 1.0f + 15.0f / (1.0f + linear_speed);
	const float yaw_damping = -m_angular_velocity.y * 200.0f * damping_scale;
	const Vector3 damping_torque(0.0f, yaw_damping, 0.0f);
	const Vector3 torque_body = torque_body_raw + damping_torque;

	const Vector3 I_omega(
		m_inertia.x * m_angular_velocity.x,
		m_inertia.y * m_angular_velocity.y,
		m_inertia.z * m_angular_velocity.z
	);
	const Vector3 gyro = m_angular_velocity.cross(I_omega);
	const Vector3 inv_I = inverse_inertia();
	const Vector3 domega(
		(torque_body.x - gyro.x) * inv_I.x,
		(torque_body.y - gyro.y) * inv_I.y,
		(torque_body.z - gyro.z) * inv_I.z
	);
	m_angular_velocity += domega * dt;

	// --- 10. Quaternion integration ---
	const Vector3 delta_rot = m_angular_velocity * dt;
	const float angle = delta_rot.length();
	if (angle > 1e-6f) {
		const Vector3 axis = delta_rot / angle;
		const Quaternion dq(axis, angle);
		m_orientation = (m_orientation * dq).normalized();
	}
	// --- 11. Wheel spin (from base) ---
	for (int i = 0; i < 4; i++) {
		const float brake_t = (i < 2)
				? m_brake_input * m_max_brake_torque * 0.35f
				: m_brake_input * m_max_brake_torque * 0.15f;
		integrate_wheel_spin(i, tire_out[i].force_x,
				m_throttle * m_max_engine_torque * 0.25f, brake_t, dt, 1.0f);
	}
}

// -
// Tire selection
// 
void VehicleBodyT2c::set_corner_tire(int p_index, const Ref<TireModel> &p_tire) {
	if (p_index >= 0 && p_index < 4) {
		m_tires[p_index] = p_tire;
	} else {
		ERR_PRINT("VehicleBodyT2c::set_corner_tire: index out of range (0-3)");
	}
}

Ref<TireModel> VehicleBodyT2c::get_corner_tire(int p_index) const {
	if (p_index >= 0 && p_index < 4) return m_tires[p_index];
	return Ref<TireModel>();
}