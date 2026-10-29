extends Node

func _ready() -> void:
	print("--- TireModelV4 Thermal Test ---")

	var tire := TireModelV4.new()
	tire.ambient_temperature = 30.0
	tire.temperature = 30.0

	var dt := 1.0 / 60.0
	var load := 4000.0
	var vx := 30.0

	print("Initial temp: %.1f C, Grip Mult: %.2f" % [tire.temperature, tire.get_grip_multiplier()])
	print("")

	# Simulate a massive burnout: Wheel is spinning at 150 rad/s
	# while the car is only moving at 5 m/s
	for i in 120: # 2 seconds of simulation
		tire.step_gd_with_dt(
			150.0,  # omega (spinning fast)
			500.0,  # torque
			0.0,
			load,
			1.0,
			0.33,
			5.0,    # vx (car is barely moving)
			0.0,    # vy
			0.0,    # steer
			0.0,    # camber
			dt
		)

		if i % 20 == 0:
			print("t=%.2fs | Temp: %.1f C | Grip Mult: %.2f" % [
				float(i) * dt, tire.temperature, tire.get_grip_multiplier()
			])

	print("")
	print("Burnout done. Let's let it cool down...")

	# Let it cool down
	for i in 120:
		tire.step_gd_with_dt(
			90.0,   # omega
			0.0,    # torque
			0.0,    # brake
			load,
			1.0,
			0.33,
			30.0,   # vx
			0.0,    # vy
			0.0,    # steer
			0.0,    # camber
			dt
		)
		if i % 40 == 0:
			print("Cool t=%.2fs | Temp: %.1f C | Grip Mult: %.2f" % [
				float(i) * dt, tire.temperature, tire.get_grip_multiplier()
			])