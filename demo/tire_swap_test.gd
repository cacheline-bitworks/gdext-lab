extends Node

var vehicle: VehicleBodyT2c
var scheduler: SimulationScheduler

var phase := "V2"
var phase_elapsed := 0.0
var phase_duration := 4.0

func _ready() -> void:
	print("--- Tire swap test ---")
	print("Same T2c vehicle, different tires, same drive pattern.")
	print("")

	scheduler = SimulationScheduler.new()
	scheduler.max_frequency = 200.0
	scheduler.min_frequency = 200.0
	scheduler.max_distance = 10000.0
	add_child(scheduler)

	vehicle = VehicleBodyT2c.new()
	scheduler.add_entity(vehicle)

	print("Corner count: %d" % vehicle.get_corner_count())

	_start_phase("V2", TireModelV2.new())

func _start_phase(name: String, tire: TireModel) -> void:
	phase = name
	phase_elapsed = 0.0

	# Reset vehicle position/state between runs
	vehicle.set_position(Vector3(0, 0.5, 0))
	vehicle.set_velocity(Vector3(0, 0, 0))

	vehicle.set_all_tires(tire)

	# Confirm the swap landed
	var check = vehicle.get_corner_tire(0)
	print("")
	print("=== Phase %s: tire installed at FL = %s ===" % [name, check])

func _process(delta: float) -> void:
	phase_elapsed += delta

	# Same drive pattern for every phase: accelerate, then moderate turn
	if phase_elapsed < 2.0:
		vehicle.throttle = 1.0
		vehicle.brake = 0.0
		vehicle.steer = 0.0
	elif phase_elapsed < 3.5:
		vehicle.throttle = 0.5
		vehicle.brake = 0.0
		vehicle.steer = 0.2
	else:
		vehicle.throttle = 0.0
		vehicle.brake = 0.0
		vehicle.steer = 0.0

	# Print a single snapshot at the end of each phase
	if phase_elapsed >= phase_duration:
		var pos: Vector3 = vehicle.get_position()
		print("  At t=%.1f: pos=(%.2f, %.2f), speed=%.2f, yaw=%.1f deg" % [
			phase_elapsed,
			pos.x, pos.z,
			vehicle.get_speed(),
			rad_to_deg(vehicle.get_yaw()),
		])

		# Advance to next phase
		if phase == "V2":
			_start_phase("V3", TireModelV3.new())
		elif phase == "V3":
			_start_phase("V1", TireModelV1.new())
		else:
			print("")
			print("--- Done ---")
			set_process(false)
