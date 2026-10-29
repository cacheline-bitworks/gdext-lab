#include "tire_model_v4.h"
#include <cmath>

void TireModelV4::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_ambient_temperature", "value"), &TireModelV4::set_ambient_temperature);
	ClassDB::bind_method(D_METHOD("get_ambient_temperature"), &TireModelV4::get_ambient_temperature);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "ambient_temperature"), "set_ambient_temperature", "get_ambient_temperature");

	ClassDB::bind_method(D_METHOD("get_temperature"), &TireModelV4::get_temperature);
	ClassDB::bind_method(D_METHOD("set_temperature", "value"), &TireModelV4::set_temperature);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "temperature"), "set_temperature", "get_temperature");

	ClassDB::bind_method(D_METHOD("set_heating_rate", "value"), &TireModelV4::set_heating_rate);
    ClassDB::bind_method(D_METHOD("get_heating_rate"), &TireModelV4::get_heating_rate);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "heating_rate"), "set_heating_rate", "get_heating_rate");

    ClassDB::bind_method(D_METHOD("set_cooling_rate", "value"), &TireModelV4::set_cooling_rate);
    ClassDB::bind_method(D_METHOD("get_cooling_rate"), &TireModelV4::get_cooling_rate);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "cooling_rate"), "set_cooling_rate", "get_cooling_rate");

	ClassDB::bind_method(D_METHOD("get_grip_multiplier"), &TireModelV4::get_grip_multiplier);
}


void TireModelV4::set_ambient_temperature(float p_val) {
	m_ambient_temp = p_val;
}

float TireModelV4::get_ambient_temperature() const {
	return m_ambient_temp;
}

float TireModelV4::get_temperature() const {
	return m_temperature;
}

void TireModelV4::set_temperature(float p_val) {
	m_temperature = p_val;
}

void TireModelV4::set_heating_rate(float p_val) {
	m_heating_rate = p_val > 0.0f ? p_val : 0.0f;
}
float TireModelV4::get_heating_rate() const { return m_heating_rate; }
void TireModelV4::set_cooling_rate(float p_val) {
	m_cooling_rate = p_val > 0.0f ? p_val : 0.0f;
}
float TireModelV4::get_cooling_rate() const { return m_cooling_rate; }

void TireModelV4::reset() {
	TireModelV3::reset(); // Clear relaxation state
	m_temperature = m_ambient_temp; // Tire cools down to ambient
}

// 
// The Grip Multiplier
// 
float TireModelV4::get_grip_multiplier() const {
	// Grip window anchored on m_optimal_temp (default 90 C).
	// Below cold_temp: grip rises from 70% to 100% as temp approaches it.
	// Optimal window: full grip.
	// Above overheat_temp: grip falls off toward 80% at 130 C.
	const float cold_temp      = m_optimal_temp - 40.0f;  // 50 C when optimal = 90
	const float cold_floor     = cold_temp - 30.0f;       // 20 C
	const float overheat_temp  = m_optimal_temp + 20.0f;  // 110 C
	const float overheated_floor = overheat_temp + 20.0f; // 130 C

	if (m_temperature < cold_temp) {
		return 0.7f + 0.3f * ((m_temperature - cold_floor) / (cold_temp - cold_floor));
	} else if (m_temperature > overheat_temp) {
		return 1.0f - 0.2f * ((m_temperature - overheat_temp) / (overheated_floor - overheat_temp));
	}
	return 1.0f;
}
// 
// Override effective_mu to scale grip
// 
float TireModelV4::effective_mu(float p_load, float p_friction) const {
	// Ask V2 for the base mu (with load sensitivity)
	float base_mu = TireModelV2::effective_mu(p_load, p_friction);
	
	// Scale it by the temperature
	return base_mu * get_grip_multiplier();
}

// 
// Override step() to update temperature
// 
WheelOutput TireModelV4::step(const WheelInput &p_input) {
	// 1. Ask V3 (which calls V2) to calculate the forces
	WheelOutput output = TireModelV3::step(p_input);

	// 2. Calculate slip power (how much the tire is sliding)
	// Power = |Force| * |Velocity|
	const float slip_vel_x = std::fabs(p_input.angular_velocity * p_input.wheel_radius - p_input.velocity_longitudinal);
	const float slip_vel_y = std::fabs(p_input.velocity_lateral);
	
	const float power_x = std::fabs(output.force_x) * slip_vel_x;
	const float power_y = std::fabs(output.force_y) * slip_vel_y;
	const float total_slip_power = power_x + power_y;

	// 3. Update temperature
	// Heat up from friction, cool down from airflow (proportional to speed)
	const float cooling_speed_factor = 1.0f + std::fabs(p_input.velocity_longitudinal) * 0.1f;
	const float heat_gained = total_slip_power * m_heating_rate * (float)p_input.dt;
	const float heat_lost = (m_temperature - m_ambient_temp) * m_cooling_rate * cooling_speed_factor * (float)p_input.dt;

	m_temperature += heat_gained - heat_lost;

	// Clamp to realistic bounds
	if (m_temperature < m_ambient_temp) m_temperature = m_ambient_temp;
	if (m_temperature > 150.0f) m_temperature = 150.0f;

	return output;
}