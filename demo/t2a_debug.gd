extends Node

var vehicle: VehicleBodyT2a
var scheduler: SimulationScheduler

var elapsed := 0.0
var log_interval := 0.25
var next_log := 0.0

func _ready() -> void:
	print("--- T2a heave + pitch test ---")

	scheduler = SimulationScheduler.new()
	scheduler.max_frequency = 200.0
	scheduler.min_frequency = 200.0
	scheduler.max_distance = 10000.0
	add_child(scheduler)

	vehicle = VehicleBodyT2a.new()
	scheduler.add_entity(vehicle)

	print("Vehicle: T2a, mass 1200 kg, wheelbase 2.6 m")
	print("")
	print("%6s %8s %8s %10s %10s %10s %10s %10s" % [
		"t (s)", "speed", "pitch", "heave", "F_comp", "R_comp", "F_load", "R_load"
	])

func _process(delta: float) -> void:
	elapsed += delta

	# Drive pattern:
	# 0-2s: accelerate
	# 2-4s: cruise at half throttle
	# 4-6s: hard brake (this is where nose dive should appear)
	# 6-8s: coast (watch suspension settle back)
	if elapsed < 2.0:
		vehicle.throttle = 1.0
		vehicle.brake = 0.0
	elif elapsed < 4.0:
		vehicle.throttle = 0.5
		vehicle.brake = 0.0
	elif elapsed < 6.0:
		vehicle.throttle = 0.0
		vehicle.brake = 1.0
	else:
		vehicle.throttle = 0.0
		vehicle.brake = 0.0

	if elapsed >= next_log:
		next_log += log_interval
		print("%6.2f %8.2f %8.3f %10.2f %10.1f %10.1f %10.0f %10.0f" % [
			elapsed,
			vehicle.get_speed(),
			rad_to_deg(vehicle.get_pitch()),
			vehicle.get_heave() * 1000.0,   # convert to mm
			vehicle.get_front_compression(),
			vehicle.get_rear_compression(),
			vehicle.get_front_load(),
			vehicle.get_rear_load(),
		])

	if elapsed > 8.0:
		print("--- Done ---")
		set_process(false)