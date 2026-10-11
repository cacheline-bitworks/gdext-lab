#pragma once

#include "vehicle_body_t2c.h"

using namespace godot;

// ----------------------------------------------------------------------------
// VehicleBodyT3 - T2c plus anti-roll bars
// ----------------------------------------------------------------------------
// T3a adds anti-roll bars: torsion springs coupling the left and right
// corners of each axle. They resist differential compression, which
// reduces roll angle in corners and shifts load transfer toward the
// stiffer end.
//
// Adjusting front vs rear ARB stiffness is the primary tuning tool for
// understeer/oversteer balance in real cars.
//
// Future T3 sub-steps will add Ackermann steering, camber, toe, and
// aerodynamics.
// ----------------------------------------------------------------------------
class VehicleBodyT3 : public VehicleBodyT2c {
	GDCLASS(VehicleBodyT3, VehicleBodyT2c)
    
float m_arb_front = 20000.0f;   // 30% of spring rate — realistic range
float m_arb_rear = 12000.0f;

protected:
	static void _bind_methods();

	// Override the T2c hook to inject ARB forces.
	float extra_corner_force(int p_corner_index, float p_compression,
			float p_opposite_compression) const override;

public:
	VehicleBodyT3() = default;
	~VehicleBodyT3() override = default;

	void set_arb_front(float p_val);
	float get_arb_front() const;
	void set_arb_rear(float p_val);
	float get_arb_rear() const;
};