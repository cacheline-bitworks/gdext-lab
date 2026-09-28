#pragma once

#include <godot_cpp/classes/node.hpp>
#include <godot_cpp/variant/array.hpp>
#include <godot_cpp/variant/vector3.hpp>
#include <vector>

#include "simulated_entity.h"

using namespace godot;

class SimulationScheduler : public Node {
    GDCLASS(SimulationScheduler, Node)

    float m_max_frequency = 100.0f;
    float m_min_frequency = 5.0f;
    float m_max_distance = 200.0f;
    Vector3 m_reference_position;
    //per-tick stats, readable from gd script
	float get_average_frequency() const;
    int m_last_step_count = 0;
    int m_total_steps = 0;
    int m_total_ticks = 0;

    std::vector<Ref<SimulatedEntity>> m_entities;

    protected:
    static void _bind_methods();

    public:
    SimulationScheduler() = default;
    ~SimulationScheduler() override = default;

    void set_max_frequency( float p_freq);
    float get_max_frequency() const;

    void set_min_frequency(float p_freq);
    float get_min_frequency() const;

    void set_max_distance(float p_dist);
    float get_max_distance() const;

    void set_reference_position(const Vector3 &p_pos);
    Vector3 get_reference_position() const;

    //entity management
    void  add_entity(const Ref<SimulatedEntity> &p_entity);
    void remove_entity(const Ref<SimulatedEntity> &p_entity);
    void clear_entities();
    int get_entity_count() const;
    Array get_entities() const;
    //stats
    int get_last_step_count() const;
    int get_total_steps() const;
    int get_total_ticks() const;
    float get_avarage_frequency() const;

    void _physics_process(double p_delta) override;
    
};