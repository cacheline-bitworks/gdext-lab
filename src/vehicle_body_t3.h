#pragma once

#include "vehicle_body_t2c.h"

using namespace godot;

// 
// VehicleBodyT3 - T2c plus anti-roll bars, Ackermann, toe, and camber
// 
// Extends T2c with:
//   - Anti-roll bars (T3a): torsion springs coupling left/right on each axle
//   - Ackermann steering (T3b): inner wheel turns more than outer
//   - Static toe (T3b): small offsets in the steering angles
//   - Static camber (T3b): wheel tilt, produces camber thrust
//
// Aero and drivetrain refinement are future T3 sub-steps.
// 
class VehicleBodyT3 : public VehicleBodyT2c {
	GDCLASS(VehicleBodyT3, VehicleBodyT2c)

	// Anti-roll bars (T3a)
	float m_arb_front = 20000.0f;
	float m_arb_rear = 12000.0f;

	// Ackermann factor (T3b). 0.0 = no Ackermann, 1.0 = perfect (100%).
	float m_ackermann_factor = 0.7f;

	// Static toe per axle (T3b). Positive = toe-in.
	float m_toe_front = 0.0f;    // rad
	float m_toe_rear = 0.0f;     // rad

	// Static camber per axle (T3b). Physical convention: negative = top inward.
	float m_camber_front = -0.0262f;   // -1.5 degrees
	float m_camber_rear = -0.0175f;    // -1.0 degrees

protected:
	static void _bind_methods();

	// T2c overrides
	float extra_corner_force(int p_corner_index, float p_compression,
			float p_opposite_compression) const override;
	float corner_steer_angle(int p_corner_index, float p_base_steer) const override;
	float corner_camber_angle(int p_corner_index) const override;

public:
	VehicleBodyT3() = default;
	~VehicleBodyT3() override = default;

	// Anti-roll bar accessors
	void set_arb_front(float p_val);
	float get_arb_front() const;
	void set_arb_rear(float p_val);
	float get_arb_rear() const;

	// Ackermann accessor
	void set_ackermann_factor(float p_val);
	float get_ackermann_factor() const;

	// Toe accessors
	void set_toe_front(float p_val);
	float get_toe_front() const;
	void set_toe_rear(float p_val);
	float get_toe_rear() const;

	// Camber accessors
	void set_camber_front(float p_val);
	float get_camber_front() const;
	void set_camber_rear(float p_val);
	float get_camber_rear() const;
};