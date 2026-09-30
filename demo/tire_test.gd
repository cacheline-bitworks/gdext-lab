extends Node

func _ready() -> void:
	print("--- Tire Model V1 test ---")

	var tire := TireModelV1.new()

	# Fixed inputs for the test
	var load := 4000.0        # N, ~400 kg on this tire
	var friction := 1.0       # dry asphalt
	var radius := 0.33        # m, typical racing tire

	# --- Test 1: Longitudinal force vs slip ratio ---
	print("")
	print("=== Longitudinal force vs slip ratio (Fz=%.0fN, mu=%.2f) ===" % [load, friction])
	print("%8s %12s" % ["slip", "Fx (N)"])

	for i in 21:
		var slip := (float(i) - 10.0) / 50.0   # -0.2 to +0.2 in steps of 0.02
		var fx := tire.compute_force_x(slip, load, friction)
		print("%8.3f %12.1f" % [slip, fx])

	# --- Test 2: Lateral force vs slip angle ---
	print("")
	print("=== Lateral force vs slip angle (Fz=%.0fN, mu=%.2f) ===" % [load, friction])
	print("%10s %12s" % ["angle(deg)", "Fy (N)"])

	for i in 21:
		var angle_deg := (float(i) - 10.0) * 1.0    # -10 to +10 degrees
		var angle_rad := deg_to_rad(angle_deg)
		var fy := tire.compute_force_y(angle_rad, load, friction)
		print("%10.1f %12.1f" % [angle_deg, fy])

	# --- Test 3: Full step with slip ratio derived from wheel state ---
	print("")
	print("=== Full step() with moving vehicle ===")

	# Rolling case: wheel spin matches ground speed
	var rolling := tire.step_gd(
		90.9,      # omega (rad/s) - 30 m/s / 0.33m
		0.0,       # drive torque
		0.0,       # brake torque
		load,
		friction,
		radius,
		30.0,      # longitudinal velocity (m/s)
		0.0        # lateral velocity
	)
	print("Rolling:     ", rolling)

	# Wheelspin case: wheel faster than ground
	var spinning := tire.step_gd(
		150.0,     # omega (rad/s) - spinning
		0.0,
		0.0,
		load,
		friction,
		radius,
		30.0,      # longitudinal velocity
		0.0
	)
	print("Wheelspin:   ", spinning)

	# Cornering case: pure lateral velocity
	var cornering := tire.step_gd(
		90.9,
		0.0,
		0.0,
		load,
		friction,
		radius,
		30.0,
		3.0        # lateral velocity (m/s) - sliding sideways
	)
	print("Cornering:   ", cornering)

	print("")
	print("--- Done ---")