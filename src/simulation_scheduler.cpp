#include "simulation_scheduler.h"

#include <algorithm>

#include <godot_cpp/classes/time.hpp>
#include <godot_cpp/core/math.hpp>

namespace {
constexpr float MIN_SAFE_FREQ = 0.01f;   // Hz, avoids 1/f = inf downstream
constexpr double MAX_FRAME_DELTA = 0.25; // s, clamps huge hitches
constexpr int MAX_STEPS_PER_TICK = 4;    // cap to avoid spiral of death
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

    	ClassDB::bind_method(D_METHOD("set_target_frame_time_ms", "ms"), &SimulationScheduler::set_target_frame_time_ms);
	ClassDB::bind_method(D_METHOD("get_target_frame_time_ms"), &SimulationScheduler::get_target_frame_time_ms);
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "target_frame_time_ms"), "set_target_frame_time_ms", "get_target_frame_time_ms");

	ClassDB::bind_method(D_METHOD("get_quality"), &SimulationScheduler::get_quality);
    ADD_SIGNAL(MethodInfo("quality_changed", PropertyInfo(Variant::FLOAT, "quality")));
}

void SimulationScheduler::set_max_frequency(float p_freq) {
	if (p_freq < MIN_SAFE_FREQ) {
		p_freq = MIN_SAFE_FREQ;
	}
	m_max_frequency = p_freq;
	if (m_min_frequency > m_max_frequency) {
		m_min_frequency = m_max_frequency;
	}
}



float SimulationScheduler::get_max_frequency() const {
    return m_max_frequency;
}

void SimulationScheduler::set_min_frequency(float p_freq) {
	if (p_freq < MIN_SAFE_FREQ) {
		p_freq = MIN_SAFE_FREQ;
	}
	if (p_freq > m_max_frequency) {
		p_freq = m_max_frequency;
	}
	m_min_frequency = p_freq;
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
	if (!p_entity.is_valid()) {
		return;
	}
	// Reject duplicates. Adding the same entity twice would step it
	// 2x per tick, giving it an effective 2x frequency.
	for (const Ref<SimulatedEntity> &existing : m_entities) {
		if (existing == p_entity) {
			return;
		}
	}
	m_entities.push_back(p_entity);
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
    return m_cached_avg_freq;
}

void SimulationScheduler::_physics_process(double p_delta) {
	// Clamp huge deltas (scene loads, editor hitches). Without this, a
	// 2-second stall banks 2 seconds of accumulated time per entity.
	if (p_delta > MAX_FRAME_DELTA) {
		p_delta = MAX_FRAME_DELTA;
	}

	// Measure wall-clock time for this tick. p_delta is fixed by Godot
	// (1 / physics_ticks_per_second) and never changes, so it can't
	// measure load. We need our own timer.
	const int64_t start_usec = Time::get_singleton()->get_ticks_usec();

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

	        // 2. Map priority to a frequency, then apply the adaptive quality
        //    multiplier. Quality scales every entity's frequency globally.
        float freq = m_min_frequency + priority * (m_max_frequency - m_min_frequency);
        freq *= m_quality;
        if (freq < 0.01f) {
            freq = 0.01f;
        }
        entity->set_current_frequency(freq);

        const float interval = 1.0f / freq;  //time between steps 

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

    // Cache the average frequency for O(1) reads. Recomputed here because
    // we've already walked every entity — no extra iteration needed.
    if (m_entities.empty()) {
        m_cached_avg_freq = 0.0f;
    } else {
        float sum = 0.0f;
        for (const Ref<SimulatedEntity> &e : m_entities) {
            if (e.is_valid()) {
                sum += e->get_current_frequency();
            }
        }
        m_cached_avg_freq = sum / (float)m_entities.size();
    }
    	// Measure elapsed wall-clock time and feed it to the controller.
	const int64_t end_usec = Time::get_singleton()->get_ticks_usec();
	const float elapsed_ms = (float)(end_usec - start_usec) / 1000.0f;
	_update_quality(elapsed_ms, p_delta);
}
void SimulationScheduler::set_target_frame_time_ms(float p_ms) {
	m_target_frame_time_ms = p_ms > 1.0f ? p_ms : 1.0f;
}

float SimulationScheduler::get_target_frame_time_ms() const {
	return m_target_frame_time_ms;
}

float SimulationScheduler::get_quality() const {
	return m_quality;
}
//	
// Adaptive quality controller
//	
// Watches smoothed frame time and adjusts m_quality with hysteresis:
//
//   - STABLE   -> DEGRADING:  frame time > target * degrade_threshold
//                             sustained for degrade_interval_s
//   - STABLE   -> RECOVERING: frame time < target * upgrade_threshold
//                             sustained for upgrade_interval_s
//   - DEGRADING -> STABLE:    frame time back in the acceptable band
//   - RECOVERING -> STABLE:   frame time back in the acceptable band,
//                             quality stepped up by upgrade_step
//
// Quality changes emit the "quality_changed" signal.
//	
void SimulationScheduler::_update_quality(float p_elapsed_ms, double p_delta) {
	const float dt = (float)p_delta;

	// Smooth the measured wall-clock time with an EMA. Raw values are
	// noisy tick to tick — react to the trend, not the jitter.
	m_avg_frame_time_ms = m_avg_frame_time_ms * 0.9f + p_elapsed_ms * 0.1f;

	const float over_threshold = m_target_frame_time_ms * m_degrade_threshold;
	const float under_threshold = m_target_frame_time_ms * m_upgrade_threshold;

	if (m_avg_frame_time_ms > over_threshold) {
		m_seconds_over_budget += dt;
		m_seconds_under_budget = 0.0f;
	} else if (m_avg_frame_time_ms < under_threshold) {
		m_seconds_under_budget += dt;
		m_seconds_over_budget = 0.0f;
	} else {
		// In the acceptable band — decay both counters toward zero
		m_seconds_over_budget = 0.0f;
		m_seconds_under_budget = 0.0f;
	}

	if (m_seconds_over_budget >= m_degrade_interval_s) {
		m_seconds_over_budget = 0.0f;
		const float old = m_quality;
		m_quality = m_quality - m_degrade_step;
		if (m_quality < m_min_quality) m_quality = m_min_quality;
		if (m_quality != old) {
			emit_signal("quality_changed", m_quality);
		}
	} else if (m_seconds_under_budget >= m_upgrade_interval_s) {
		m_seconds_under_budget = 0.0f;
		const float old = m_quality;
		m_quality = m_quality + m_upgrade_step;
		if (m_quality > m_max_quality) m_quality = m_max_quality;
		if (m_quality != old) {
			emit_signal("quality_changed", m_quality);
		}
	}
}

