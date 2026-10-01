#pragma once

#include <godot_cpp/classes/ref_counted.hpp>
#include <godot_cpp/variant/vector3.hpp>

using namespace godot;

class SimulatedEntity : public RefCounted {
	GDCLASS(SimulatedEntity, RefCounted)

protected:
	// State - accessible to subclasses (e.g. VehicleBodyT1) so they can
	// integrate motion without going through setters every frame.
	Vector3 m_position;
	Vector3 m_velocity;

	// Scheduler-managed fields. Set by SimulationScheduler each tick.
	float m_importance = 0.5f;
	float m_priority = 1.0f;
	float m_current_frequency = 100.0f;
	float m_accumulator = 0.0f;

	static void _bind_methods();

public:
	SimulatedEntity() = default;
	~SimulatedEntity() override = default;

	// --- Position ---
	void set_position(const Vector3 &p_pos);
	Vector3 get_position() const;

	// --- Velocity ---
	void set_velocity(const Vector3 &p_vel);
	Vector3 get_velocity() const;

	// --- Importance (gameplay significance, 0..1) ---
	void set_importance(float p_importance);
	float get_importance() const;

	// --- Scheduler-managed fields ---
	float get_priority() const;
	void set_priority(float p_priority);

	float get_current_frequency() const;
	void set_current_frequency(float p_freq);

	float get_accumulator() const;
	void set_accumulator(float p_acc);

	// Default behavior: linear motion. Subclasses override for real physics.
	virtual void step(double p_delta);
};