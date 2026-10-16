extends Node

var scheduler: SimulationScheduler
var entities: Array = []
var elapsed := 0.0
var phase := "idle"

const LOAD_ENTITY_COUNT := 50000
const PHASE_DURATION := 4.0

func _ready() -> void:
	print("--- Adaptive scheduler test ---")
	print("")
	print("Phase 1 (0-4s):   idle, no entities")
	print("Phase 2 (4-12s):  %d entities added (heavy load)" % LOAD_ENTITY_COUNT)
	print("Phase 3 (12-20s): entities removed, watch recovery")
	print("")

	scheduler = SimulationScheduler.new()
	scheduler.max_frequency = 200.0
	scheduler.min_frequency = 20.0
	scheduler.max_distance = 10000.0
	scheduler.target_frame_time_ms = 1.0   # 1000 FPS budget — forces engage
	add_child(scheduler)

	scheduler.quality_changed.connect(_on_quality_changed)

	print("%6s %10s %10s %10s" % ["t", "entities", "quality", "avg_freq"])

func _process(delta: float) -> void:
	elapsed += delta

	# Phase 1 -> Phase 2 at t=4s: add entities
	if elapsed >= 4.0 and phase == "idle":
		phase = "loading"
		_spawn_entities(LOAD_ENTITY_COUNT)
		print(">> spawned %d entities" % LOAD_ENTITY_COUNT)

	# Phase 2 -> Phase 3 at t=12s: remove entities
	if elapsed >= 12.0 and phase == "loading":
		phase = "unloading"
		_remove_entities()
		print(">> removed all entities")

	# Log once per second
	var sec := int(elapsed)
	if sec > 0 and fmod(elapsed, 1.0) < delta:
		print("%6.2f %10d %10.3f %10.1f" % [
			elapsed,
			scheduler.get_entity_count(),
			scheduler.get_quality(),
			scheduler.get_average_frequency(),
		])

	if elapsed > 20.0:
		print("")
		print("--- Done ---")
		set_process(false)

func _spawn_entities(count: int) -> void:
	for i in count:
		var e := SimulatedEntity.new()
		# Spread entities within 100 m of the reference (all high priority)
		e.position = Vector3(
			randf_range(-100.0, 100.0),
			0.0,
			randf_range(-100.0, 100.0)
		)
		scheduler.add_entity(e)
		entities.append(e)

func _remove_entities() -> void:
	scheduler.clear_entities()
	entities.clear()

func _on_quality_changed(q: float) -> void:
	print("   [quality changed] -> %.2f" % q)
