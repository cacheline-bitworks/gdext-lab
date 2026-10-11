extends Node

var vehicle: VehicleBodyT3
var scheduler: SimulationScheduler

var elapsed := 0.0
var log_interval := 0.3
var next_log := 0.0

func _ready() -> void:
	print("--- T3 anti-roll bar test ---")

	scheduler = SimulationScheduler.new()
	scheduler.max_frequency = 200.0
	scheduler.min_frequency = 200.0
	scheduler.max_distance = 10000.0
	add_child(scheduler)

	vehicle = VehicleBodyT3.new()
	vehicle.arb_front = 20000.0
	vehicle.arb_rear = 12000.0
	scheduler.add_entity(vehicle)

	print("Vehicle: T3, T2c + anti-roll bars")
	print("ARB front: %.0f N/m, ARB rear: %.0f N/m" % [vehicle.arb_front, vehicle.arb_rear])
	print("Compare roll angles here against t2c_debug output.")
	print("")
	print("%5s %7s %7s %7s %7s | %7s %7s %7s %7s" % [
		"t", "speed", "yaw", "pitch", "roll", "FL", "FR", "RL", "RR"
	])

func _process(delta: float) -> void:
	elapsed += delta

	# Same drive pattern as T2c, so the comparison is direct
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