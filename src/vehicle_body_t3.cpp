#include "vehicle_body_t3.h"

void VehicleBodyT3::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_arb_front", "value"), &VehicleBodyT3::set_arb_front);
	ClassDB::bind_method(D_METHOD("get_arb_front"), &VehicleBodyT3::get_arb_front);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "arb_front"), "set_arb_front", "get_arb_front");

	ClassDB::bind_method(D_METHOD("set_arb_rear", "value"), &VehicleBodyT3::set_arb_rear);
	ClassDB::bind_method(D_METHOD("get_arb_rear"), &VehicleBodyT3::get_arb_rear);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "arb_rear"), "set_arb_rear", "get_arb_rear");
}

void VehicleBodyT3::set_arb_front(float p_val) {
	m_arb_front = p_val < 0.0f ? 0.0f : p_val;
}

float VehicleBodyT3::get_arb_front() const {
	return m_arb_front;
}

void VehicleBodyT3::set_arb_rear(float p_val) {
	m_arb_rear = p_val < 0.0f ? 0.0f : p_val;
}

float VehicleBodyT3::get_arb_rear() const {
	return m_arb_rear;
}

float VehicleBodyT3::extra_corner_force(int p_corner_index, float p_compression,
		float p_opposite_compression) const {
	// Anti-roll bar: torsion spring between left and right corners on
	// the same axle. Force is proportional to the differential compression.
	//
	// If this corner is MORE compressed than its opposite, the ARB pushes
	// up here (adds load). If LESS compressed, it pulls down here (removes
	// load). The opposite corner receives the negation of this force via
	// the same formula applied at its own index.
	//
	// Net vertical force on the axle is unchanged; only the left/right
	// distribution shifts.
	const float arb_stiffness = (p_corner_index < 2) ? m_arb_front : m_arb_rear;
	return arb_stiffness * (p_compression - p_opposite_compression);
}