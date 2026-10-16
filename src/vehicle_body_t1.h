#pragma once

#include <godot_cpp/variant/vector3.hpp>

#include "vehicle_body.h"
#include "tire_model.h"

using namespace godot;

// 
// VehicleBodyT1 - bicycle model with V1 tires
// 
// Simplest tier. No suspension, no roll, no aero. Just yaw + planar motion.
// T1-specific state: yaw and yaw rate as scalars. Two virtual wheels
// (front/rear axle), each representing two physical wheels.
//
// All shared state (mass, geometry, inputs, limits, wheel omegas) lives in
// VehicleBody. See docs/patterns/05-vehicle-body-t2.md for tier design.
// 
class VehicleBodyT1 : public VehicleBody {
	GDCLASS(VehicleBodyT1, VehicleBody)

	// T1-specific orientation state (2D, yaw only)
	float m_yaw = 0.0f;
	float m_yaw_rate = 0.0f;

	
	// Tire slots: 0 = front axle, 1 = rear axle. Storage is the abstract
	// base so any concrete TireModel subclass can be installed at runtime.
	Ref<TireModel> m_front_tire;
	Ref<TireModel> m_rear_tire;

protected:
	static void _bind_methods();

public:
	VehicleBodyT1();
	~VehicleBodyT1() override = default;
	float get_yaw() const;
	float get_yaw_rate() const;
	float get_speed() const;
	Vector3 get_velocity() const;

	// Tire selection
	int get_corner_count() const override { return 2; }
	void set_corner_tire(int p_index, const Ref<TireModel> &p_tire) override;
	Ref<TireModel> get_corner_tire(int p_index) const override;

	void step(double p_delta) override;
};