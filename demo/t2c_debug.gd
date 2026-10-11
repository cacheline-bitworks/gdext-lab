extends Node

var vehicle: VehicleBodyT2c
var scheduler: SimulationScheduler

var elapsed := 0.0
var log_interval := 0.3
var next_log := 0.0

func _ready() -> void:
	print("--- T2c full 6-DOF rigid body test ---")

	scheduler = SimulationScheduler.new()
	scheduler.max_frequency = 200.0
	scheduler.min_frequency = 200.0
	scheduler.max_distance = 10000.0
	add_child(scheduler)

	vehicle = VehicleBodyT2c.new()
	scheduler.add_entity(vehicle)

	print("Vehicle: T2c, 4 wheels, full quaternion orientation")
	print("Corners: 0=FL  1=FR  2=RL  3=RR")
	print("")
	print("%5s %7s %7s %7s %7s | %7s %7s %7s %7s" % [
		"t", "speed", "yaw", "pitch", "roll", "FL", "FR", "RL", "RR"
	])

func _process(delta: float) -> void:
	elapsed += delta

	# Drive pattern:
	# 0-2s:  accelerate straight
	# 2-4s:  moderate turn right
	# 4-6s:  trail-braking with gentler steering
	# 6-8s:  brake hard, no steering
	# 8-10s: coast to stop
	if elapsed < 2.0:
		vehicle.throttle = 1.0
		vehicle.brake = 0.0
		vehicle.steer = 0.0
	elif elapsed < 4.0:
		vehicle.throttle = 0.6
		vehicle.brake = 0.0
		vehicle.steer = 0.2
	elif elapsed < 6.0:
		vehicle.throttle = 0.0
		vehicle.brake = 0.3
		vehicle.steer = 0.2
	elif elapsed < 8.0:
		vehicle.throttle = 0.0
		vehicle.brake = 1.0
		vehicle.steer = 0.0
	else:
		vehicle.throttle = 0.0
		vehicle.brake = 0.0
		vehicle.steer = 0.0

	if elapsed >= next_log:
		next_log += log_interval
		var pos: Vector3 = vehicle.get_position()
		print("%5.2f %7.2f %7.2f %7.2f %7.2f | %7.0f %7.0f %7.0f %7.0f" % [
			elapsed,
			vehicle.get_speed(),
			rad_to_deg(vehicle.get_yaw()),
			rad_to_deg(vehicle.get_pitch()),
			rad_to_deg(vehicle.get_roll()),
			vehicle.get_corner_load(0),
			vehicle.get_corner_load(1),
			vehicle.get_corner_load(2),
			vehicle.get_corner_load(3),
		])

	if elapsed > 10.0:
		print("--- Done ---")
		set_process(false)