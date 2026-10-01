extends Node

var vehicle: VehicleBodyT1
var scheduler: SimulationScheduler

var elapsed := 0.0
var log_interval := 0.5
var next_log := 0.0

func _ready() -> void:
	print("--- Vehicle T1 test ---")

	scheduler = SimulationScheduler.new()
	scheduler.max_frequency = 100.0
	scheduler.min_frequency = 100.0
	scheduler.max_distance = 10000.0
	add_child(scheduler)

	vehicle = VehicleBodyT1.new()
	scheduler.add_entity(vehicle)

	print("Mass: 1200 kg")
	print("Wheelbase: 2.6 m")
	print("Scheduler entities: %d" % scheduler.get_entity_count())
	print("")
	print("%6s %10s %10s %10s %10s %10s" % ["t (s)", "x (m)", "z (m)", "speed", "yaw(deg)", "steps"])

func _process(delta: float) -> void:
	elapsed += delta

	# Drive pattern:
	# 0-3s: full throttle straight
	# 3-6s: partial throttle, ramped steering to 0.3
	# 6-8s: full brake
	# 8+:   coast
	if elapsed < 3.0:
		vehicle.throttle = 1.0
		vehicle.brake = 0.0
		vehicle.steer = 0.0
	elif elapsed < 6.0:
		vehicle.throttle = 0.6
		vehicle.brake = 0.0
		var t_turn: float = (elapsed - 3.0) / 3.0
		vehicle.steer = 0.3 * t_turn
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
		var pos := vehicle.get_position()
		print("%6.2f %10.2f %10.2f %10.2f %10.2f %10d" % [
			elapsed,
			pos.x,
			pos.z,
			vehicle.get_speed(),
			rad_to_deg(vehicle.get_yaw()),
			scheduler.get_total_steps()
		])

	if elapsed > 12.0:
		print("--- Done ---")
		set_process(false)