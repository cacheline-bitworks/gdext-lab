extends Node

func _ready() -> void:
	print("--- Tire Model V1 test ---")

	var tire := TireModelV1.new()

	var load := 4000.0
	var friction := 1.0
	var radius := 0.33

	print("")
	print("=== Longitudinal force vs slip ratio (Fz=%.0fN, mu=%.2f) ===" % [load, friction])
	print("%8s %12s" % ["slip", "Fx (N)"])

	for i in 21:
		var slip := (float(i) - 10.0) / 50.0
		var fx := tire.compute_force_x(slip, load, friction)
		print("%8.3f %12.1f" % [slip, fx])

	print("")
	print("=== Lateral force vs slip angle (Fz=%.0fN, mu=%.2f) ===" % [load, friction])
	print("%10s %12s" % ["angle(deg)", "Fy (N)"])

	for i in 21:
		var angle_deg := (float(i) - 10.0) * 1.0
		var angle_rad := deg_to_rad(angle_deg)
		var fy := tire.compute_force_y(angle_rad, load, friction)
		print("%10.1f %12.1f" % [angle_deg, fy])

	print("")
	print("=== Full step() with moving vehicle ===")

	# Rolling case: wheel spin matches ground speed
	var rolling := tire.step_gd(
		90.9,      # omega
		0.0,       # drive torque
		0.0,       # brake torque
		load,
		friction,
		radius,
		30.0,      # longitudinal velocity
		0.0,       # lateral velocity
		0.0        # steer angle
	)
	print("Rolling:     ", rolling)

	# Wheelspin case
	var spinning := tire.step_gd(
		150.0,
		0.0,
		0.0,
		load,
		friction,
		radius,
		30.0,
		0.0,
		0.0
	)
	print("Wheelspin:   ", spinning)

	# Cornering case
	var cornering := tire.step_gd(
		90.9,
		0.0,
		0.0,
		load,
		friction,
		radius,
		30.0,
		3.0,
		0.0
	)
	print("Cornering:   ", cornering)

	print("")
	print("--- Done ---")