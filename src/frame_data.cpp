#include "frame_data.h"

#include <godot_cpp/classes/worker_thread_pool.hpp>
#include <godot_cpp/core/math.hpp>
#include <godot_cpp/variant/callable.hpp>

#include <cmath>

static constexpr double kTau = 6.283185307179586;

void FrameData::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_point_count", "count"), &FrameData::set_point_count);
	ClassDB::bind_method(D_METHOD("get_point_count"), &FrameData::get_point_count);
	ADD_PROPERTY(PropertyInfo(Variant::INT, "point_count"), "set_point_count", "get_point_count");

	ClassDB::bind_method(D_METHOD("get_raw_bytes"), &FrameData::get_raw_bytes);
	ClassDB::bind_method(D_METHOD("get_typed_floats"), &FrameData::get_typed_floats);
	ClassDB::bind_method(D_METHOD("compute_serial"), &FrameData::compute_serial);
	ClassDB::bind_method(D_METHOD("compute_parallel"), &FrameData::compute_parallel);
}

void FrameData::set_point_count(int p_count) {
	m_point_count = p_count < 0 ? 0 : p_count;
}

int FrameData::get_point_count() const {
	return m_point_count;
}

float FrameData::_heavy_compute(int p_index, double p_time) {
	// Angle arithmetic in double. Accumulating hundreds of radians in
	// float loses precision rapidly because sin/cos do range reduction
	// on the argument first.
	double t = (double)p_index * 0.001;
	double sum = 0.0;
	for (int k = 0; k < 50; k++) {
		double angle = t * kTau * (double)(k + 1) + p_time;
		sum += std::sin(angle) * std::cos(angle * 0.5);
	}
	return (float)std::sqrt(std::fabs(sum) + 1.0);
}

PackedByteArray FrameData::get_raw_bytes() const {
	const int float_count = m_point_count * 2;
	const int byte_count = float_count * (int)sizeof(float);

	PackedByteArray out;
	out.resize(byte_count);
	float *w = reinterpret_cast<float *>(out.ptrw());

	for (int i = 0; i < m_point_count; i++) {
		float t = (float)i / (float)m_point_count;
		w[i * 2 + 0] = Math::cos(t * (float)kTau + (float)m_time);
		w[i * 2 + 1] = Math::sin(t * (float)kTau + (float)m_time);
	}
	return out;
}

PackedFloat32Array FrameData::get_typed_floats() const {
	PackedFloat32Array out;
	out.resize(m_point_count * 2);
	for (int i = 0; i < m_point_count; i++) {
		float t = (float)i / (float)m_point_count;
		out[i * 2 + 0] = Math::cos(t * (float)kTau + (float)m_time);
		out[i * 2 + 1] = Math::sin(t * (float)kTau + (float)m_time);
	}
	return out;
}

PackedByteArray FrameData::compute_serial() const {
	PackedByteArray out;
	out.resize(m_point_count * (int)sizeof(float));
	float *w = reinterpret_cast<float *>(out.ptrw());

	for (int i = 0; i < m_point_count; i++) {
		w[i] = _heavy_compute(i, m_time);
	}
	return out;
}

void FrameData::_parallel_worker(int p_index) {
	// Read from members. Each worker writes to a distinct index — no race.
	m_parallel_buffer[p_index] = _heavy_compute(p_index, m_parallel_time);
}

PackedByteArray FrameData::compute_parallel() const {
	PackedByteArray out;
	out.resize(m_point_count * (int)sizeof(float));

	// Set the transient state the workers will read from.
	// const_cast because the method is const but we need to mutate member state.
	FrameData *self = const_cast<FrameData *>(this);
	self->m_parallel_buffer = reinterpret_cast<float *>(out.ptrw());
	self->m_parallel_time = m_time;

	WorkerThreadPool *pool = WorkerThreadPool::get_singleton();
	Callable task = callable_mp(self, &FrameData::_parallel_worker);
	int task_id = pool->add_group_task(task, m_point_count);
	pool->wait_for_group_task_completion(task_id);

	// Clear transient state so no dangling pointers linger.
	self->m_parallel_buffer = nullptr;

	return out;
}
void FrameData::_process(double delta) {
	// Wrap m_time at 2π. sin/cos are 2π-periodic, so the math is identical,
	// but the value stays small enough to keep full double precision
	// forever. Without wrapping, precision degrades after ~months of
	// continuous running.
	m_time = fmod(m_time + delta, 6.283185307179586);
}
void FrameData::_ready() {
	print_line("FrameData ready, point_count = ", m_point_count);
}