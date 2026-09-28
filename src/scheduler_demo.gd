extends Node

var scheduler: SimulationScheduler
var frame_times: Array[float] = []
var test_duration := 5.0
var elapsed := 0.0

func _ready() -> void:
	print("--- Scheduler test ---")

	scheduler = SimulationScheduler.new()
	scheduler.max_frequency = 100.0
	scheduler.min_frequency = 5.0
	scheduler.max_distance = 200.0
	add_child(scheduler)

	# Spawn 1000 entities spread across a 400x400 area
	for i in 1000:
		var e := SimulatedEntity.new()
		e.position = Vector3(
			randf_range(-200.0, 200.0),
			0.0,
			randf_range(-200.0, 200.0)
		)
		e.velocity = Vector3(0.0, 0.0, randf_range(-5.0, 5.0))
		e.importance = randf()
		scheduler.add_entity(e)

	print("Entities: %d" % scheduler.get_entity_count())
	print("Max frequency: %.0f Hz" % scheduler.max_frequency)
	print("Min frequency: %.0f Hz" % scheduler.min_frequency)
	print("Max distance:  %.0f m" % scheduler.max_distance)

func _process(delta: float) -> void:
	# Sweep the reference position in a circle (simulating a moving camera)
	var t := Time.get_ticks_msec() / 1000.0
	scheduler.reference_position = Vector3(cos(t) * 100.0, 0.0, sin(t) * 100.0)

	frame_times.append(delta)
	elapsed += delta

	if elapsed >= test_duration:
		_print_results()
		set_process(false)

func _print_results() -> void:
	var total := 0.0
	var worst := 0.0
	for ft in frame_times:
		total += ft
		if ft > worst:
			worst = ft
	var avg_ms := (total / frame_times.size()) * 1000.0

	print("--- Results ---")
	print("Frames: %d over %.2f seconds" % [frame_times.size(), elapsed])
	print("Average frame time: %.3f ms" % avg_ms)
	print("Worst frame time:   %.3f ms" % (worst * 1000.0))
	print("Total ticks:  %d" % scheduler.get_total_ticks())
	print("Total steps:  %d" % scheduler.get_total_steps())
	print("Average freq: %.1f Hz" % scheduler.get_average_frequency())

	var max_possible := scheduler.get_total_ticks() * scheduler.get_entity_count()
	if max_possible > 0:
		var efficiency := 100.0 - (float(scheduler.get_total_steps()) / float(max_possible) * 100.0)
		print("Work saved:   %.1f%%" % efficiency)
		print("  (vs. every entity running at 100 Hz every tick)")

	print("--- Done ---")