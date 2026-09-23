#pragma once

#include <godot_cpp/classes/node.hpp>
#include <godot_cpp/variant/packed_byte_array.hpp>
#include <godot_cpp/variant/packed_float32_array.hpp>

using namespace godot;

class FrameData : public Node {
	GDCLASS(FrameData, Node)

	int m_point_count = 1000;
	double m_time = 0.0;

	// Transient state used only during compute_parallel().
	// The main thread sets these before submitting the group task,
	// waits for completion, then clears them. Workers read/write through them.
	float *m_parallel_buffer = nullptr;
	double m_parallel_time = 0.0;

protected:
	static void _bind_methods();

public:
	FrameData() = default;
	~FrameData() override = default;

	void set_point_count(int p_count);
	int get_point_count() const;

	PackedByteArray get_raw_bytes() const;
	PackedFloat32Array get_typed_floats() const;

	PackedByteArray compute_serial() const;
	PackedByteArray compute_parallel() const;

	void _process(double delta) override;
	void _ready() override;

private:
	static float _heavy_compute(int p_index, double p_time);

	// Worker: called once per index. Reads from members, writes to buffer.
	void _parallel_worker(int p_index);
};