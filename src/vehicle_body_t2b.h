#pragma once

#include <godot_cpp/variant/vector3.hpp>

#include "vehicle_body.h"
#include "tire_model.h"
#include "tire_model_v2.h"

using namespace godot;

// 
// CornerConfig - per-corner suspension parameters
// 
struct CornerConfig {
	float spring_rate = 65000.0f;
	float damper_bump = 4200.0f;
	float damper_rebound = 8500.0f;
};

// 
// VehicleBodyT2b - four wheels + roll on the bicycle pitch/heave model
// 
class VehicleBodyT2b : public VehicleBody {
	GDCLASS(VehicleBodyT2b, VehicleBody)

	// Chassis-specific
	float m_cg_height = 0.5f;
	float m_yaw = 0.0f;
	float m_yaw_rate = 0.0f;
	float m_heave = 0.0f;
	float m_heave_velocity = 0.0f;
	float m_pitch = 0.0f;
	float m_pitch_rate = 0.0f;
	float m_roll = 0.0f;
	float m_roll_rate = 0.0f;
	float m_pitch_inertia = 1800.0f;
	float m_roll_inertia = 400.0f;

	// Suspension per corner
	CornerConfig m_front_left;
	CornerConfig m_front_right;
	CornerConfig m_rear_left;
	CornerConfig m_rear_right;

	// Tires (order: FL, FR, RL, RR). Abstract base for runtime selection.
	Ref<TireModel> m_tires[4];

protected:
	static void _bind_methods();

public:
	VehicleBodyT2b();
	~VehicleBodyT2b() override = default;

	float get_yaw() const;
	float get_yaw_rate() const;
	float get_speed() const;
	Vector3 get_velocity() const;
	float get_roll() const;
	float get_roll_rate() const;

	float get_corner_load(int p_index) const;
	float get_corner_compression(int p_index) const;
		// Tire selection
	int get_corner_count() const override { return 4; }
	void set_corner_tire(int p_index, const Ref<TireModel> &p_tire) override;
	Ref<TireModel> get_corner_tire(int p_index) const override;

	void step(double p_delta) override;
};