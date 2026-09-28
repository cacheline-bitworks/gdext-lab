#include "simulation_scheduler.h"

#include <godot_cpp/core/math.hpp>

namespace {
    constexpr int MAX_STEPS_PER_TICK = 4;
}

void SimulationScheduler::_bind_methods() {
    ClassDB::bind_method(D_METHOD("set_max_frequency", "freq"), &SimulationScheduler::set_max_frequency);
    ClassDB::bind_method(D_METHOD("get_max_frequency"),        &SimulationScheduler::get_max_frequency);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "max_frequency"),
                 "set_max_frequency", "get_max_frequency");

    ClassDB::bind_method(D_METHOD("set_min_frequency", "freq"), &SimulationScheduler::set_min_frequency);
    ClassDB::bind_method(D_METHOD("get_min_frequency"),        &SimulationScheduler::get_min_frequency);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "min_frequency"),
                 "set_min_frequency", "get_min_frequency");

    ClassDB::bind_method(D_METHOD("set_max_distance", "dist"), &SimulationScheduler::set_max_distance);
    ClassDB::bind_method(D_METHOD("get_max_distance"),         &SimulationScheduler::get_max_distance);
    ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "max_distance"),
                 "set_max_distance", "get_max_distance");

    ClassDB::bind_method(D_METHOD("set_reference_position", "pos"), &SimulationScheduler::set_reference_position);
    ClassDB::bind_method(D_METHOD("get_reference_position"),        &SimulationScheduler::get_reference_position);
    ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "reference_position"),
                 "set_reference_position", "get_reference_position");

    ClassDB::bind_method(D_METHOD("add_entity", "entity"),    &SimulationScheduler::add_entity);
    ClassDB::bind_method(D_METHOD("remove_entity", "entity"), &SimulationScheduler::remove_entity);
    ClassDB::bind_method(D_METHOD("clear_entities"),          &SimulationScheduler::clear_entities);
    ClassDB::bind_method(D_METHOD("get_entity_count"),        &SimulationScheduler::get_entity_count);
    ClassDB::bind_method(D_METHOD("get_entities"),            &SimulationScheduler::get_entities);

    ClassDB::bind_method(D_METHOD("get_last_step_count"),   &SimulationScheduler::get_last_step_count);
    ClassDB::bind_method(D_METHOD("get_total_steps"),       &SimulationScheduler::get_total_steps);
    ClassDB::bind_method(D_METHOD("get_total_ticks"),       &SimulationScheduler::get_total_ticks);
    ClassDB::bind_method(D_METHOD("get_average_frequency"), &SimulationScheduler::get_average_frequency);
}

void SimulationScheduler::set_max_frequency(float p_freq) {
    m_max_frequency = p_freq > 0.0f ? p_freq : 0.0f;
}

float SimulationScheduler::get_max_frequency() const {
    return m_max_frequency;
}

void SimulationScheduler::set_min_frequency(float p_freq) {
    m_min_frequency = p_freq > 0.0f ? p_freq : 0.0f;
}

float SimulationScheduler::get_min_frequency() const {
    return m_min_frequency;
}

void SimulationScheduler::set_max_distance(float p_dist) {
    m_max_distance = p_dist > 0.0f ? p_dist : 0.0f;
}

float SimulationScheduler::get_max_distance() const {
    return m_max_distance;
}

void SimulationScheduler::set_reference_position(const Vector3 &p_pos) {
    m_reference_position = p_pos;
}

Vector3 SimulationScheduler::get_reference_position() const {
    return m_reference_position;
}

void SimulationScheduler::add_entity(const Ref<SimulatedEntity> &p_entity) {
    if (p_entity.is_valid()) {
        m_entities.push_back(p_entity);
    }
}

void SimulationScheduler::remove_entity(const Ref<SimulatedEntity> &p_entity) {
    for (auto it = m_entities.begin(); it != m_entities.end(); ++it) {
        if (*it == p_entity) {
            m_entities.erase(it);
            return;
        }
    }
}

void SimulationScheduler::clear_entities() {
    m_entities.clear();
}

int SimulationScheduler::get_entity_count() const {
    return (int)m_entities.size();
}

Array SimulationScheduler::get_entities() const {
    Array out;
    for (const Ref<SimulatedEntity> &e : m_entities) {
        out.append(e);
    }
    return out;
}

int SimulationScheduler::get_last_step_count() const {
    return m_last_step_count;
}

int SimulationScheduler::get_total_steps() const {
    return m_total_steps;
}

int SimulationScheduler::get_total_ticks() const {
    return m_total_ticks;
}

float SimulationScheduler::get_average_frequency() const {
    if (m_entities.empty()) {
        return 0.0f;
    }
    float sum = 0.0f;
    for (const Ref<SimulatedEntity> &e : m_entities) {
        sum += e->get_current_frequency();
    }
    return sum / (float)m_entities.size();
}

void SimulationScheduler::_physics_process(double p_delta) {
    m_total_ticks++;
    m_last_step_count = 0;

    const float max_dist_sq = m_max_distance * m_max_distance;

    for (const Ref<SimulatedEntity> &entity : m_entities) {
        if (!entity.is_valid()) {
            continue;
        }

        // 1. Compute priority from distance and importance.
        Vector3 to_entity = entity->get_position() - m_reference_position;
        float dist_sq = to_entity.length_squared();

        float distance_factor;
        if (dist_sq < max_dist_sq) {
            distance_factor = 1.0f - Math::sqrt(dist_sq) / m_max_distance;
        } else {
            distance_factor = 0.0f;
        }

        float priority = distance_factor * 0.7f + entity->get_importance() * 0.3f;
        priority = priority < 0.0f ? 0.0f : (priority > 1.0f ? 1.0f : priority);
        entity->set_priority(priority);

        // 2. Map priority to a frequency.
        float freq = m_min_frequency + priority * (m_max_frequency - m_min_frequency);
        if (freq < 0.01f) {
            freq = 0.01f;
        }
        entity->set_current_frequency(freq);

        const float interval = 1.0f / freq;

        // 3. Accumulate time.
        entity->set_accumulator(entity->get_accumulator() + (float)p_delta);

        // 4. Step as needed, capped to prevent spiral of death.
        int steps_this_entity = 0;
        while (entity->get_accumulator() >= interval &&
               steps_this_entity < MAX_STEPS_PER_TICK) {
            entity->step((double)interval);
            entity->set_accumulator(entity->get_accumulator() - interval);
            steps_this_entity++;
            m_total_steps++;
        }
        m_last_step_count += steps_this_entity;

        // If we hit the step cap, clamp the accumulator so it can't
        // grow unbounded and cause a spiral of death on the next tick.
        const float max_accumulated = interval * (float)MAX_STEPS_PER_TICK;
        if (entity->get_accumulator() > max_accumulated) {
            entity->set_accumulator(max_accumulated);
        }
    }
}