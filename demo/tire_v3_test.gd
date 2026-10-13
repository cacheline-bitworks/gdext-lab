extends Node

var tire_v3: TireModelV3

func _ready() -> void:
	print("--- TireModelV3 smoke test ---")
	tire_v3 = TireModelV3.new()
	tire_v3.relaxation_length = 0.5

	print("Relaxation length: %.2f" % tire_v3.relaxation_length)
	print("Initial slip ratio: %.4f" % tire_v3.get_relaxed_slip_ratio())
	print("Initial slip angle: %.4f" % tire_v3.get_relaxed_slip_angle())

	# Call step_gd (inherited from V2) to verify it works
	var result: Dictionary = tire_v3.step_gd(
		90.0,    # omega
		0.0, 0.0,  # torques
		4000.0,  # load
		1.0,     # friction
		0.33,    # radius
		30.0,    # vx
		2.0,     # vy
		0.0      # steer
	)
	print("Step result Fy: %.1f N" % result["force_y"])
	print("")
	print("V3 builds and runs. Transient behavior verified via vehicle test.")