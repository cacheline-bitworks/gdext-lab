#include "simulated_entity.h"

#include <cmath>
void SimulatedEntity::_bind_methods() {
	ClassDB::bind_method(D_METHOD("step", "delta"), &SimulatedEntity::step);
	ClassDB::bind_method(D_METHOD("set_position", "position"), &SimulatedEntity::set_position);
	ClassDB::bind_method(D_METHOD("get_position"), &SimulatedEntity::get_position);
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "position"), "set_position", "get_position");

	ClassDB::bind_method(D_METHOD("set_velocity", "velocity"), &SimulatedEntity::set_velocity);
	ClassDB::bind_method(D_METHOD("get_velocity"), &SimulatedEntity::get_velocity);
	ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "velocity"), "set_velocity", "get_velocity");

	ClassDB::bind_method(D_METHOD("set_importance", "importance"), &SimulatedEntity::set_importance);
	ClassDB::bind_method(D_METHOD("get_importance"), &SimulatedEntity::get_importance);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "importance"), "set_importance", "get_importance");

	ClassDB::bind_method(D_METHOD("get_priority"), &SimulatedEntity::get_priority);
	ClassDB::bind_method(D_METHOD("get_current_frequency"), &SimulatedEntity::get_current_frequency);
	ClassDB::bind_method(D_METHOD("get_accumulator"), &SimulatedEntity::get_accumulator);
}

void SimulatedEntity::set_position(const Vector3 &p_pos) {
	m_position = p_pos;
}

Vector3 SimulatedEntity::get_position() const {
	return m_position;
}

void SimulatedEntity::set_velocity(const Vector3 &p_vel) {
	m_velocity = p_vel;
}

Vector3 SimulatedEntity::get_velocity() const {
	return m_velocity;
}

void SimulatedEntity::set_importance(float p_importance) {
	m_importance = p_importance < 0.0f ? 0.0f : (p_importance > 1.0f ? 1.0f : p_importance);//clamp
	// Just same as: if (p_importance < 0.0f) return 0.0f;
    // else if (p_importance > 1.0f) return 1.0f;
    // else return p_importance;
}

float SimulatedEntity::get_importance() const {
	return m_importance;
}

float SimulatedEntity::get_priority() const {
	return m_priority;
}

void SimulatedEntity::set_priority(float p_priority) {
	// NaN would violate strict weak ordering if the scheduler ever sorts
	// by priority. Clamp to [0, 1] and reject NaN.
	if (std::isnan(p_priority)) {
		p_priority = 1.0f;
	}
	m_priority = p_priority < 0.0f ? 0.0f : (p_priority > 1.0f ? 1.0f : p_priority);
}

float SimulatedEntity::get_current_frequency() const {
	return m_current_frequency;
}

void SimulatedEntity::set_current_frequency(float p_freq) {
	// 0 or negative frequency would produce 1/f = inf downstream.
	// NaN is rejected by the comparison (NaN < x is always false).
	if (std::isnan(p_freq) || p_freq < 0.01f) {
		p_freq = 0.01f;
	}
	m_current_frequency = p_freq;
}

float SimulatedEntity::get_accumulator() const {
	return m_accumulator;
}

void SimulatedEntity::set_accumulator(float p_acc) {
	// Huge accumulators cause catch-up spirals on the next tick.
	// The scheduler clamps this too, but defense in depth is cheap.
	if (std::isnan(p_acc) || p_acc < 0.0f) {
		p_acc = 0.0f;
	}
	// 5 seconds is a generous ceiling — no single frame should bank more.
	if (p_acc > 5.0f) {
		p_acc = 5.0f;
	}
	m_accumulator = p_acc;
}
void SimulatedEntity::step(double p_delta) {
	// Default behavior: linear motion. Subclasses override for real simulation.
	m_position += m_velocity * (float)p_delta;
}