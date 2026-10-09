#pragma once

#include <godot_cpp/classes/node.hpp>
#include <godot_cpp/variant/array.hpp>
#include <godot_cpp/variant/vector3.hpp>
#include <vector>

#include "simulated_entity.h"

using namespace godot;

class SimulationScheduler : public Node {
	GDCLASS(SimulationScheduler, Node)

	// --- Configuration ---
	float m_max_frequency = 100.0f;
	float m_min_frequency = 5.0f;
	float m_max_distance = 200.0f;
	Vector3 m_reference_position;

	// --- Per-tick statistics, readable from GDScript ---
	int m_last_step_count = 0; // this tick: how much work did I just do?
	int m_total_steps = 0;     // since startup: total work
	int m_total_ticks = 0;     // since startup: how many ticks have I run?

	// --- Managed entities ---
	std::vector<Ref<SimulatedEntity>> m_entities;

protected:
	static void _bind_methods();

public:
	SimulationScheduler() = default;
	~SimulationScheduler() override = default;

	// --- Configuration accessors ---
	// Distance -> frequency mapping:
	//   t = clamp(dist / max_distance, 0, 1)
	//   freq = lerp(max_frequency, min_frequency, t)
	// An entity at distance 0 gets max_frequency.
	// An entity at distance >= max_distance gets min_frequency.
	void set_max_frequency(float p_freq);
	float get_max_frequency() const;

	void set_min_frequency(float p_freq);
	float get_min_frequency() const;

	void set_max_distance(float p_dist);
	float get_max_distance() const;

	void set_reference_position(const Vector3 &p_pos);
	Vector3 get_reference_position() const;

	// --- Entity management ---
	void add_entity(const Ref<SimulatedEntity> &p_entity);
	void remove_entity(const Ref<SimulatedEntity> &p_entity);
	void clear_entities();
	int get_entity_count() const;
	Array get_entities() const;

	// --- Statistics ---
	int get_last_step_count() const;
	int get_total_steps() const;
	int get_total_ticks() const;
	float get_average_frequency() const;

	// Each tick, for every entity:
	//   1. recompute target frequency from distance
	//   2. accumulator += p_delta
	//   3. while accumulator >= 1/freq: step(1/freq), accumulator -= 1/freq
	// p_delta is clamped to [0, kMaxFrameDelta] to prevent catch-up spirals.
	void _physics_process(double p_delta) override;
};