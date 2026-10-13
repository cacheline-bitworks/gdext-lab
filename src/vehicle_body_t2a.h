#pragma once

#include <godot_cpp/variant/vector3.hpp>

#include "vehicle_body.h"
#include "tire_model.h"
#include "tire_model_v2.h"

using namespace godot;

// 
// SuspensionConfig - pure data struct
// 
struct SuspensionConfig {
	float spring_rate_front = 60000.0f;   // 60 N/mm
	float spring_rate_rear = 70000.0f;    // 70 N/mm
	float damper_bump_front = 4000.0f;
	float damper_rebound_front = 8000.0f;
	float damper_bump_rear = 4500.0f;
	float damper_rebound_rear = 9000.0f;
	float nominal_load = 4000.0f;
};

// 
// VehicleBodyT2a - heave + pitch on bicycle model
// 
// See docs/patterns/05-vehicle-body-t2.md for the tier design.
// 
class VehicleBodyT2a : public VehicleBody {
	GDCLASS(VehicleBodyT2a, VehicleBody)

	// T2a-specific state
	float m_cg_height = 0.5f;
	float m_yaw = 0.0f;
	float m_yaw_rate = 0.0f;
	float m_heave = 0.0f;
	float m_heave_velocity = 0.0f;
	float m_pitch = 0.0f;
	float m_pitch_rate = 0.0f;
	float m_pitch_inertia = 1800.0f;

	SuspensionConfig m_suspension;

	// Two V2 tires (bicycle model: front axle, rear axle)
	Ref<TireModelV2> m_front_tire;
	Ref<TireModelV2> m_rear_tire;

protected:
	static void _bind_methods();

public:
	VehicleBodyT2a();
	~VehicleBodyT2a() override = default;

	// State accessors
	float get_yaw() const;
	float get_yaw_rate() const;
	float get_speed() const;
	Vector3 get_velocity() const;

	// T2a-specific
	float get_heave() const;
	float get_heave_velocity() const;
	float get_pitch() const;
	float get_pitch_rate() const;

	float get_front_compression() const;
	float get_rear_compression() const;
	float get_front_load() const;
	float get_rear_load() const;

	void step(double p_delta) override;
};