#pragma once

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/variant/vector2.hpp>
#include <godot_cpp/variant/vector3.hpp>
#include <godot_cpp/variant/dictionary.hpp>

#include "simulated_entity.h"
#include "tire_model.h"

using namespace godot;

// 
// VehicleBodyT1 - simplest useful vehicle
// 
// A bicycle model: two virtual wheels (front and rear). Integrates the tire
// model's forces into a moving body. No suspension dynamics, no drivetrain
// complexity, no aero. Just enough to drive.
//
// Inherits from SimulatedEntity so it can be stepped by SimulationScheduler.
//
class VehicleBodyT1 : public SimulatedEntity {
	GDCLASS(VehicleBodyT1, SimulatedEntity)

	// --- Chassis state ---
	float m_mass = 1200.0f;                 // kg
	float m_yaw = 0.0f;                     // rad, orientation around Y
	float m_yaw_rate = 0.0f;                // rad/s
	Vector3 m_velocity;                      // m/s, world frame

	// --- Geometry ---
	float m_wheel_base = 2.6f;              // m, front axle to rear axle
	float m_cg_to_front = 1.3f;             // m, CG to front axle
	float m_cg_to_rear = 1.3f;              // m, CG to rear axle
	float m_track_width = 1.6f;             // m, for lateral force application
	float m_wheel_radius = 0.33f;           // m
	float m_wheel_inertia = 1.5f;           // kg·m² per wheel

	// --- Wheels (bicycle model) ---
	float m_front_omega = 0.0f;             // rad/s
	float m_rear_omega = 0.0f;              // rad/s
	Ref<TireModelV1> m_front_tire;
	Ref<TireModelV1> m_rear_tire;

	// --- Driver inputs ---
	float m_throttle = 0.0f;                // 0..1
	float m_brake_input = 0.0f;             // 0..1
	float m_steer_input = 0.0f;             // -1..1
	float m_max_engine_torque = 1500.0f;  // Nm at the wheel per axle (post-gearing)
	float m_max_brake_torque = 1200.0f;     // Nm per axle
	float m_max_steer_angle = 0.25f;         // rad, ~14 degrees

	// --- Diagnostics ---
	float m_last_speed = 0.0f;

protected:
	static void _bind_methods();

public:
	VehicleBodyT1();
	~VehicleBodyT1() override = default;

	// --- Driver inputs ---
	void set_throttle(float p_val);
	float get_throttle() const;
	void set_brake(float p_val);
	float get_brake() const;
	void set_steer(float p_val);
	float get_steer() const;

	// --- State accessors ---
	float get_yaw() const;
	float get_yaw_rate() const;
	float get_speed() const;                 // m/s (magnitude)
	Vector3 get_velocity() const;

	// --- Simulation step ---
	void step(double p_delta) override;
};