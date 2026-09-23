extends Node

func _ready() -> void:
	print("--- gdext-lab demo starting ---")

	var example := ExampleClass.new()
	example.print_type(example)
	example.print_type(42)
	example.print_type("hello")

	var fd := FrameData.new()
	fd.point_count = 1000
	add_child(fd)

	var bytes := fd.get_raw_bytes()
	print("FrameData raw bytes: %d bytes" % bytes.size())
	print("  First point: (%.4f, %.4f)" % [bytes.decode_float(0), bytes.decode_float(4)])

	var floats := fd.get_typed_floats()
	print("FrameData typed floats: %d elements" % floats.size())
	print("  First point: (%.4f, %.4f)" % [floats[0], floats[1]])

	print("--- gdext-lab demo done ---")
