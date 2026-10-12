#include "vehicle_body_t2c.h"

#include <cmath>

namespace {
constexpr float GRAVITY = 9.81f;
}

// Body frame: X forward, Y up, Z right
// Yaw rotates about Y, roll about X, pitch about Z

// 
// Constructor
// 
VehicleBodyT2c::VehicleBodyT2c() {
	for (int i = 0; i < 4; i++) {
		m_tires[i].instantiate();
	}
	// Start at rest height above ground
	m_position.y = m_eq_height;
	m_orientation = Quaternion();  // identity
}

// 
// _bind_methods
// 
void VehicleBodyT2c::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_throttle", "value"), &VehicleBodyT2c::set_throttle);
	ClassDB::bind_method(D_METHOD("get_throttle"), &VehicleBodyT2c::get_throttle);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "throttle"), "set_throttle", "get_throttle");

	ClassDB::bind_method(D_METHOD("set_brake", "value"), &VehicleBodyT2c::set_brake);
	ClassDB::bind_method(D_METHOD("get_brake"), &VehicleBodyT2c::get_brake);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "brake"), "set_brake", "get_brake");

	ClassDB::bind_method(D_METHOD("set_steer", "value"), &VehicleBodyT2c::set_steer);
	ClassDB::bind_method(D_METHOD("get_steer"), &VehicleBodyT2c::get_steer);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "steer"), "set_steer", "get_steer");

    ClassDB::bind_method(D_METHOD("get_pitch"), &VehicleBodyT2c::get_pitch);
	ClassDB::bind_method(D_METHOD("get_roll"), &VehicleBodyT2c::get_roll);

    ClassDB::bind_method(D_METHOD("get_steer_angle_for_corner", "index"),
			&VehicleBodyT2c::get_steer_angle_for_corner);

	ClassDB::bind_method(D_METHOD("get_yaw"), &VehicleBodyT2c::get_yaw);
	ClassDB::bind_method(D_METHOD("get_speed"), &VehicleBodyT2c::get_speed);
	ClassDB::bind_method(D_METHOD("get_velocity"), &VehicleBodyT2c::get_velocity);
	ClassDB::bind_method(D_METHOD("get_orientation"), &VehicleBodyT2c::get_orientation);
	ClassDB::bind_method(D_METHOD("get_angular_velocity"), &VehicleBodyT2c::get_angular_velocity);

	ClassDB::bind_method(D_METHOD("get_corner_load", "index"), &VehicleBodyT2c::get_corner_load);
	ClassDB::bind_method(D_METHOD("get_corner_compression", "index"), &VehicleBodyT2c::get_corner_compression);
}

// 
// Driver inputs
// 
void VehicleBodyT2c::set_throttle(float p_val) {
	m_throttle = p_val < 0.0f ? 0.0f : (p_val > 1.0f ? 1.0f : p_val);
}
float VehicleBodyT2c::get_throttle() const { return m_throttle; }

void VehicleBodyT2c::set_brake(float p_val) {
	m_brake_input = p_val < 0.0f ? 0.0f : (p_val > 1.0f ? 1.0f : p_val);
}
float VehicleBodyT2c::get_brake() const { return m_brake_input; }

void VehicleBodyT2c::set_steer(float p_val) {
	m_steer_input = p_val < -1.0f ? -1.0f : (p_val > 1.0f ? 1.0f : p_val);
}
float VehicleBodyT2c::get_steer() const { return m_steer_input; }

// 
// State accessors
// 
float VehicleBodyT2c::get_speed() const { return m_velocity.length(); }
Vector3 VehicleBodyT2c::get_velocity() const { return m_velocity; }
Quaternion VehicleBodyT2c::get_orientation() const { return m_orientation; }
Vector3 VehicleBodyT2c::get_angular_velocity() const { return m_angular_velocity; }

float VehicleBodyT2c::get_yaw() const {
	// Extract yaw from the quaternion. With body frame (X=forward, Y=up),
	// yaw is the rotation about world Y axis. Approximate for small pitches.
	Vector3 forward_body(1, 0, 0);
	Vector3 forward_world = m_orientation.xform(forward_body);
	return std::atan2(-forward_world.z, forward_world.x);
}

// Get the body-frame offset for corner index (0=FL, 1=FR, 2=RL, 3=RR)
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
	return -world_c.y * 1000.0f;  // mm, positive = compressed
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

	const float static_load = get_static_load(p_index, m_mass, m_cg_to_front,
			m_cg_to_rear, m_wheel_base);

	float load = static_load + cfg->spring_rate * compression;
	return load < 0.0f ? 0.0f : load;
}
float VehicleBodyT2c::get_pitch() const {
	// Pitch: rotation about the body's Z axis. Extract by measuring
	// how much the forward vector tilts in the vertical plane.
	Vector3 forward_world = m_orientation.xform(Vector3(1, 0, 0));
	return std::atan2(forward_world.y, std::sqrt(forward_world.x * forward_world.x + forward_world.z * forward_world.z));
}

float VehicleBodyT2c::get_roll() const {
	// Roll: rotation about the body's X axis. Extract by measuring
	// how much the up vector tilts in the lateral plane.
	Vector3 up_world = m_orientation.xform(Vector3(0, 1, 0));
	Vector3 right_world = m_orientation.xform(Vector3(0, 0, 1));
	return std::atan2(up_world.z, up_world.y);
}

// 
// The physics step
// 
void VehicleBodyT2c::step(double p_delta) {
	const float dt = (float)p_delta;
	if (dt <= 0.0f) return;

	const Vector3 world_up(0.0f, 1.0f, 0.0f);

	// --- 1. Force and torque accumulators ---
	// Gravity acts at CG, so no torque contribution
	Vector3 total_force_world(0.0f, -m_mass * GRAVITY, 0.0f);
	Vector3 total_torque_world(0.0f, 0.0f, 0.0f);

	// --- 2. Angular velocity in world frame (for corner velocity) ---
	Vector3 omega_world = m_orientation.xform(m_angular_velocity);

	// --- 3. Steering ---
	const float steer_angle = m_steer_input * m_max_steer_angle;

	// --- 4. Corner config lookup ---
	CornerConfigT2c *configs[4] = {
		&m_front_left, &m_front_right, &m_rear_left, &m_rear_right
	};
    	// Precompute compressions so the ARB hook can reference opposite corners
	float compressions[4];
	for (int i = 0; i < 4; i++) {
		const Vector3 body_c = get_body_corner(i, m_cg_to_front, m_cg_to_rear,
				m_track_width, m_eq_height);
		const Vector3 world_c = m_position + m_orientation.xform(body_c);
		compressions[i] = -world_c.y;
	}

	// --- 5. Process each corner ---
	WheelOutput tire_out[4];

	for (int i = 0; i < 4; i++) {
		// World position of the chassis corner
		const Vector3 body_c = get_body_corner(i, m_cg_to_front, m_cg_to_rear,
				m_track_width, m_eq_height);
		const Vector3 corner_world = m_position + m_orientation.xform(body_c);

		// Suspension compression (deviation from rest, positive = compressed)
		const float compression = -corner_world.y;
        		const int opposite = (i == 0) ? 1 : (i == 1) ? 0 : (i == 2) ? 3 : 2;
		const float extra_force = extra_corner_force(i, compressions[i], compressions[opposite]);

		// Corner velocity in world frame: v_cg + ω × r
		const Vector3 r = corner_world - m_position;
		const Vector3 corner_vel_world = m_velocity + omega_world.cross(r);

		// Damper force along world up (project corner velocity onto vertical)
		const float corner_vel_y = corner_vel_world.y;
		float damper_f;
		if (corner_vel_y > 0.0f) {
			damper_f = -configs[i]->damper_rebound * corner_vel_y;
		} else {
			damper_f = -configs[i]->damper_bump * corner_vel_y;
		}

		// Spring force (deviation from equilibrium)
		const float spring_f = configs[i]->spring_rate * compression;

		// Static load (becomes zero if corner lifts off)
		const float static_load = get_static_load(i, m_mass, m_cg_to_front,
				m_cg_to_rear, m_wheel_base);

		// Total vertical load on the tire at this corner

        float tire_load = static_load + spring_f + damper_f + extra_force;
		if (tire_load < 0.0f) tire_load = 0.0f;

		// --- Apply suspension force to chassis at corner position ---
		// Suspension force = static preload + dynamic force.
        // At equilibrium (compression = 0), the spring is preloaded to support
        // the static weight. Without this, gravity sinks the chassis until the
        // springs finally balance it, and everything downstream is wrong.
        		const Vector3 suspension_force_world(0.0f, static_load + spring_f + damper_f + extra_force, 0.0f);
		total_force_world += suspension_force_world;
		total_torque_world += r.cross(suspension_force_world);

		// --- Tire contact point: same XZ as corner, at ground level ---
		const Vector3 contact_world(corner_world.x, 0.0f, corner_world.z);

		// Velocity at the contact point in world
		const Vector3 r_contact = contact_world - m_position;
		const Vector3 contact_vel_world = m_velocity + omega_world.cross(r_contact);

		// Transform to body frame
		const Vector3 contact_vel_body = m_orientation.inverse().xform(contact_vel_world);

		// --- Build WheelInput ---
       	const float corner_steer = corner_steer_angle(i, steer_angle);

		WheelInput in;
		in.steer_angle = corner_steer;
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
        in.camber_angle = corner_camber_angle(i);

		tire_out[i] = m_tires[i]->step(in);

		// --- Transform tire force from tire-local to world ---
		// Tire-local: X along steered forward, Z along steered lateral
		// Rotate by steer_angle within body, then by orientation to world
		const float cs = std::cos(corner_steer);
		const float ss = std::sin(corner_steer);
		const Vector3 tire_force_body(
			tire_out[i].force_x * cs - tire_out[i].force_y * ss,
			0.0f,
			tire_out[i].force_x * ss + tire_out[i].force_y * cs
		);
		const Vector3 tire_force_world = m_orientation.xform(tire_force_body);

		// Apply at contact point
		total_force_world += tire_force_world;
		total_torque_world += r_contact.cross(tire_force_world);
	}

	// --- 6. Low-speed damping ---
	{
		const float speed = m_velocity.length();
		if (speed > 0.001f) {
			const float rolling = 200.0f;
			const float drag = 0.4f * speed * speed;
			const float resist_decel = ((rolling + drag) / m_mass) * dt;
			if (resist_decel >= speed) {
				m_velocity = Vector3(0.0f, 0.0f, 0.0f);
			} else {
				const Vector3 dir = m_velocity / speed;
				m_velocity -= dir * resist_decel;
			}
		}
		if (m_velocity.length() < 2.0f && m_throttle < 0.01f && m_brake_input < 0.01f) {
			m_velocity *= (1.0f - 5.0f * dt);
			if (m_velocity.length() < 0.05f) {
				m_velocity = Vector3(0.0f, 0.0f, 0.0f);
			}
		}
	}

	// --- 7. Linear integration ---
	const Vector3 accel_world = total_force_world / m_mass;
	m_velocity += accel_world * dt;
	m_position += m_velocity * dt;

	// Prevent sinking through ground
	if (m_position.y < m_eq_height * 0.3f) {
		m_position.y = m_eq_height * 0.3f;
		if (m_velocity.y < 0.0f) m_velocity.y = 0.0f;
	}

	// --- 8. Angular integration (Euler's equations) ---
	// Convert torque to body frame
	const Vector3 torque_body = m_orientation.inverse().xform(total_torque_world);

	// Gyroscopic term: ω × (I * ω)
	const Vector3 I_omega(
		m_inertia.x * m_angular_velocity.x,
		m_inertia.y * m_angular_velocity.y,
		m_inertia.z * m_angular_velocity.z
	);
	const Vector3 gyro = m_angular_velocity.cross(I_omega);

	// I * dω/dt = τ - ω × (I*ω)
	const Vector3 domega(
		(torque_body.x - gyro.x) * m_inv_inertia.x,
		(torque_body.y - gyro.y) * m_inv_inertia.y,
		(torque_body.z - gyro.z) * m_inv_inertia.z
	);
	m_angular_velocity += domega * dt;

	// Damping: resist angular velocity at low linear speed (prevents endless spin)
	{
		const float linear_speed = m_velocity.length();
		const float damping_scale = 1.0f + 5.0f / (1.0f + linear_speed);
		m_angular_velocity *= (1.0f - 0.5f * damping_scale * dt);
	}

	// --- 9. Quaternion integration ---
	// dq = 0.5 * q ⊗ (ω, 0), for small dt this is a small rotation by ω*dt
	const Vector3 delta_rot = m_angular_velocity * dt;
	const float angle = delta_rot.length();
	if (angle > 1e-6f) {
		const Vector3 axis = delta_rot / angle;
		const Quaternion dq(axis, angle);
		m_orientation = (m_orientation * dq).normalized();
	}

	// --- 10. Wheel spin integration ---
	for (int i = 0; i < 4; i++) {
		const float drive = m_throttle * m_max_engine_torque * 0.25f;
		const float brake_t = (i < 2)
				? m_brake_input * m_max_brake_torque * 0.35f
				: m_brake_input * m_max_brake_torque * 0.15f;
		const float reaction = -tire_out[i].force_x * m_wheel_radius;

		const float alpha = (drive - brake_t + reaction) / m_wheel_inertia;
		m_omega[i] += alpha * dt;

		if (m_omega[i] < 0.0f) m_omega[i] = 0.0f;
		if (m_omega[i] > 200.0f) m_omega[i] = 200.0f;
	}
    		// Diagnostic
	static float debug_timer = 0.0f;
	debug_timer += dt;
	if (debug_timer >= 1.0f) {
		debug_timer = 0.0f;
		const Vector3 vel_body = m_orientation.inverse().xform(m_velocity);
		const Vector3 fwd_world = m_orientation.xform(Vector3(1, 0, 0));
		print_line(vformat(
			"[DBG] v_body=(%.1f, %.1f) | fwd_world=(%.2f, %.2f, %.2f) | |F| = %.0f N",
			vel_body.x, vel_body.z,
			fwd_world.x, fwd_world.y, fwd_world.z,
						total_force_world.length()));
	}
}

float VehicleBodyT2c::get_steer_angle_for_corner(int p_index) const {
	const float base = m_steer_input * m_max_steer_angle;
	return corner_steer_angle(p_index, base);
}

float VehicleBodyT2c::extra_corner_force(int, float, float) const {
	return 0.0f;
}

float VehicleBodyT2c::corner_steer_angle(int p_corner_index, float p_base_steer) const {
	return (p_corner_index < 2) ? p_base_steer : 0.0f;
}
float VehicleBodyT2c::corner_camber_angle(int) const {
	return 0.0f;
}