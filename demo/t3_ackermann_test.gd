extends Node

var vehicle: VehicleBodyT3
var elapsed := 0.0
var log_interval := 0.5
var next_log := 0.0

func _ready() -> void:
	print("--- T3b: Ackermann + toe demonstration ---")

	vehicle = VehicleBodyT3.new()

	# 100% Ackermann for clarity (real cars use ~60-90%)
	vehicle.ackermann_factor = 1.0

	# Small toe-in on front: 0.5 deg = 0.0087 rad
	vehicle.toe_front = 0.0087
	vehicle.toe_rear = 0.0

	print("Ackermann factor: %.2f" % vehicle.ackermann_factor)
	print("Front toe: %.2f deg" % rad_to_deg(vehicle.toe_front))
	print("Rear toe:  %.2f deg" % rad_to_deg(vehicle.toe_rear))
	print("")
	print("Steer input sweeps between -0.4 and +0.4 over 6 seconds.")
	print("Watch FL vs FR when turning right vs left.")
	print("")
	print("%6s %10s | %8s %8s %8s %8s" % [
		"t", "steer_in", "FL(deg)", "FR(deg)", "RL(deg)", "RR(deg)"
	])

func _process(delta: float) -> void:
	elapsed += delta

	# Sweep steering input to see Ackermann geometry at different angles
	if elapsed < 6.0:
		vehicle.steer = 0.4 * sin(elapsed * 1.0)
	else:
		vehicle.steer = 0.0

	if elapsed >= next_log:
		next_log += log_interval
		print("%6.2f %10.3f | %8.3f %8.3f %8.3f %8.3f" % [
			elapsed,
			vehicle.get_steer(),
			rad_to_deg(vehicle.get_steer_angle_for_corner(0)),
			rad_to_deg(vehicle.get_steer_angle_for_corner(1)),
			rad_to_deg(vehicle.get_steer_angle_for_corner(2)),
			rad_to_deg(vehicle.get_steer_angle_for_corner(3)),
		])

	if elapsed > 8.0:
		print("--- Done ---")
		set_process(false)