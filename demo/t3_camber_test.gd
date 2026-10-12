extends Node

# Runs the same drive pattern twice:
#   Phase A: camber = 0
#   Phase B: camber = -3 deg front and rear
# Prints yaw at t=5.0s for each, so you can compare directly.

var vehicle: VehicleBodyT3
var scheduler: SimulationScheduler

var elapsed := 0.0
var phase := "A"
var yaw_at_5 := 0.0
var run_duration := 10.0

func _ready() -> void:
	print("--- T3b camber A/B comparison ---")
	print("")
	scheduler = SimulationScheduler.new()
	scheduler.max_frequency = 200.0
	scheduler.min_frequency = 200.0
	scheduler.max_distance = 10000.0
	add_child(scheduler)

	_run_phase("A", 0.0)

func _run_phase(p_phase: String, p_camber_rad: float) -> void:
	# Fresh vehicle each phase
	if vehicle != null:
		scheduler.clear_entities()

	vehicle = VehicleBodyT3.new()
	vehicle.ackermann_factor = 0.7
	vehicle.camber_front = p_camber_rad
	vehicle.camber_rear = p_camber_rad
	scheduler.add_entity(vehicle)

	phase = p_phase
	elapsed = 0.0

	print("=== Phase %s: camber = %.2f deg ===" % [p_phase, rad_to_deg(p_camber_rad)])
	print("%5s %7s %7s %7s %7s" % ["t", "speed", "yaw", "pitch", "roll"])

func _process(delta: float) -> void:
	elapsed += delta

	# Same drive pattern each phase: accelerate, then steady turn
	if elapsed < 1.5:
		vehicle.throttle = 1.0
		vehicle.brake = 0.0
		vehicle.steer = 0.0
	elif elapsed < 8.0:
		vehicle.throttle = 0.4
		vehicle.brake = 0.0
		vehicle.steer = 0.15
	else:
		vehicle.throttle = 0.0
		vehicle.brake = 0.0
		vehicle.steer = 0.0

	# Log every 1 second
	var sec := int(elapsed)
	if sec > 0 and fmod(elapsed, 1.0) < delta:
		print("%5.2f %7.2f %7.2f %7.2f %7.2f" % [
			elapsed,
			vehicle.get_speed(),
			rad_to_deg(vehicle.get_yaw()),
			rad_to_deg(vehicle.get_pitch()),
			rad_to_deg(vehicle.get_roll()),
		])

	# Record yaw at t=5.0 (steady turn, before release)
	if elapsed >= 5.0 and yaw_at_5 == 0.0:
		yaw_at_5 = rad_to_deg(vehicle.get_yaw())

	if elapsed > run_duration:
		if phase == "A":
			print("  yaw at t=5s: %.3f deg" % yaw_at_5)
			print("")
			var baseline := yaw_at_5
			yaw_at_5 = 0.0
			_run_phase("B", deg_to_rad(-3.0))
			# Stash baseline for comparison in a class variable
			set_meta("baseline_yaw", baseline)
		else:
			var baseline: float = get_meta("baseline_yaw")
			print("  yaw at t=5s: %.3f deg" % yaw_at_5)
			print("")
			print("--- Comparison ---")
			print("Camber  0 deg: yaw = %.3f deg" % baseline)
			print("Camber -3 deg: yaw = %.3f deg" % yaw_at_5)
			var diff := yaw_at_5 - baseline
			print("Difference:    %+.3f deg" % diff)
			if abs(diff) > 0.5:
				print("Camber is having an effect.")
			else:
				print("Camber effect is small — increase camber to magnify.")
			print("--- Done ---")
			set_process(false)