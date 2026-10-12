extends Node

var vehicle: VehicleBodyT3
var scheduler: SimulationScheduler

var elapsed := 0.0
var run_duration := 12.0

func _ready() -> void:
	print("--- T3c aerodynamics test ---")
	print("Full throttle in a straight line. Watch the top speed plateau.")
	print("")

	scheduler = SimulationScheduler.new()
	scheduler.max_frequency = 200.0
	scheduler.min_frequency = 200.0
	scheduler.max_distance = 10000.0
	add_child(scheduler)

	vehicle = VehicleBodyT3.new()
	# Low drag, high downforce: GT3-style aero
	vehicle.drag_coefficient = 3.0
	vehicle.downforce_coefficient = 2.0
	vehicle.frontal_area = 1.9
	scheduler.add_entity(vehicle)

	print("Cd=%.2f  Cl=%.2f  A=%.2f m^2" % [
		vehicle.drag_coefficient,
		vehicle.downforce_coefficient,
		vehicle.frontal_area
	])
	print("")
	print("%5s %8s %10s %12s" % ["t", "speed", "x_pos", "corner_load"])

func _process(delta: float) -> void:
	elapsed += delta

	vehicle.throttle = 1.0
	vehicle.brake = 0.0
	vehicle.steer = 0.0

	# Log once per second
	var sec := int(elapsed)
	if sec > 0 and fmod(elapsed, 1.0) < delta:
		print("%5.2f %8.2f %10.2f %12.0f" % [
			elapsed,
			vehicle.get_speed(),
			vehicle.get_position().x,
			vehicle.get_corner_load(0),
		])

	if elapsed > run_duration:
		print("--- Done ---")
		set_process(false)