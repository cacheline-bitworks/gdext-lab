#pragma once

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/variant/dictionary.hpp>

using namespace godot;

// 
// WheelInput - everything the tire model needs to know about the outside world
// 
// // Plain C++ struct. Not a Godot Object. Zero overhead when passed by reference
// from the vehicle simulation.
// 
struct WheelInput {
	float angular_velocity = 0.0f;       // rad/s, current wheel spin
	float torque = 0.0f;                 // Nm, net drive torque applied this step
	float brake_torque = 0.0f;           // Nm, always opposes rotation
	float vertical_load = 0.0f;          // N, how hard the tire is pressed down
	float friction_coefficient = 1.0f;   // dimensionless, surface grip
	float wheel_radius = 0.33f;          // m, for converting torque <-> force
	float velocity_longitudinal = 0.0f;  // m/s, at contact patch
	float velocity_lateral = 0.0f;    
	float steer_angle = 0.0f;   // m/s, at contact patch
};

//
// WheelOutput - what the tire model returns each step
//
struct WheelOutput {
	float force_x = 0.0f;    // N, longitudinal (forward/backward)
	float force_y = 0.0f;    // N, lateral (left/right)
	float moment_z = 0.0f;   // Nm, aligning torque (0 in v1)
	float slip_ratio = 0.0f; // dimensionless, for telemetry
	float slip_angle = 0.0f; // rad, for telemetry
};

// 
// TireModel - abstract base for all versions
// 
// Cannot be instantiated directly. Defines the single interface that every
// tire model version must implement: a step() method that consumes WheelInput
// and produces WheelOutput.
//
// Versions are sibling classes: TireModelV1, TireModelV2, TireModelV3, ...
// Each is registered with Godot independently and can be chosen at runtime.
//
class TireModel : public RefCounted {
	GDCLASS(TireModel, RefCounted)

protected:
	static void _bind_methods();

public:
	TireModel() = default;
	~TireModel() override = default;

	virtual WheelOutput step(const WheelInput &p_input) = 0;
};

// 
// TireModelV1 - pure Pacejka, constant friction, no load sensitivity
// 
// The simplest useful tire model. Captures the essential shape:
//   - Longitudinal force rises with slip ratio, peaks, then falls
//   - Lateral force rises with slip angle, peaks, then falls
//   - Vertical load scales the entire curve
//
// Does NOT model: load sensitivity, combined slip, relaxation length,
// thermal effects, or wear.
//
// Intended for: AI traffic, background vehicles, tests, and as the baseline
// against which later versions are compared.
// 

class TireModelV1 : public TireModel {
	GDCLASS(TireModelV1, TireModel)

	float m_long_stiffness = 10.0f;
	float m_long_shape = 1.65f;
	float m_long_curvature = 0.5f;

	float m_lat_stiffness = 8.0f;
	float m_lat_shape = 1.3f;
	float m_lat_curvature = 0.97f;

protected:
	static void _bind_methods();

public:
	TireModelV1() = default;
	~TireModelV1() override = default;
	



	// The real interface - used internally by the vehicle simulation.
	WheelOutput step(const WheelInput &p_input) override;

	// GDScript-friendly wrappers for testing without a vehicle.
	// Returns a Dictionary with keys: force_x, force_y, moment_z,
	// slip_ratio, slip_angle.
	Dictionary step_gd(float p_omega, float p_torque, float p_brake_torque,
			float p_vertical_load, float p_friction, float p_radius,
			float p_vel_long, float p_vel_lat);

	// Compute a single longitudinal force from slip ratio, load, and friction.
	float compute_force_x(float p_slip_ratio, float p_vertical_load, float p_friction) const;

	// Compute a single lateral force from slip angle, load, and friction.
	float compute_force_y(float p_slip_angle, float p_vertical_load, float p_friction) const;
};