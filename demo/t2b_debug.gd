extends Node

var vehicle: VehicleBodyT2b
var scheduler: SimulationScheduler

var elapsed := 0.0
var log_interval := 0.3
var next_log := 0.0

func _ready() -> void:
	print("--- T2b four-wheel + roll test ---")

	scheduler = SimulationScheduler.new()
	scheduler.max_frequency = 200.0
	scheduler.min_frequency = 200.0
	scheduler.max_distance = 10000.0
	add_child(scheduler)

	vehicle = VehicleBodyT2b.new()
	scheduler.add_entity(vehicle)

	print("Vehicle: T2b, 4 wheels, mass 1200 kg")
	print("Corners: 0=FL  1=FR  2=RL  3=RR")
	print("")
	print("%5s %7s %7s %8s | %7s %7s %7s %7s" % [
		"t", "speed", "roll", "roll_r", "FL", "FR", "RL", "RR"
	])

func _process(delta: float) -> void:
	elapsed += delta

	# Pattern:
	# 0-2s: accelerate straight
	# 2-3s: transition to turn
	# 3-6s: steady right turn at partial throttle
	# 6-8s: release steering, straighten out
	if elapsed < 2.0:
		vehicle.throttle = 1.0
		vehicle.brake = 0.0
		vehicle.steer = 0.0
	elif elapsed < 3.0:
		vehicle.throttle = 0.7
		vehicle.brake = 0.0
		var t: float = (elapsed - 2.0) / 1.0
		vehicle.steer = 0.2 * t
	elif elapsed < 6.0:
		vehicle.throttle = 0.5
		vehicle.brake = 0.0
		vehicle.steer = 0.2
	elif elapsed < 8.0:
		vehicle.throttle = 0.0
		vehicle.brake = 0.5
		var t: float = (elapsed - 6.0) / 2.0
		vehicle.steer = 0.2 * (1.0 - t)
	else:
		vehicle.throttle = 0.0
		vehicle.brake = 0.0
		vehicle.steer = 0.0

	if elapsed >= next_log:
		next_log += log_interval
		print("%5.2f %7.2f %7.3f %8.4f | %7.0f %7.0f %7.0f %7.0f" % [
			elapsed,
			vehicle.get_speed(),
			rad_to_deg(vehicle.get_roll()),
			vehicle.get_roll_rate(),
			vehicle.get_corner_load(0),
			vehicle.get_corner_load(1),
			vehicle.get_corner_load(2),
			vehicle.get_corner_load(3),
		])

	if elapsed > 10.0:
		print("--- Done ---")
		set_process(false)