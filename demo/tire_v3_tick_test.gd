extends Node

# Step response test for V3's relaxation length.
# Logs every physics tick so the lag is visible.

var tire_v2: TireModelV2
var tire_v3: TireModelV3

var tick := 0
var max_ticks := 40
var dt := 0.005

func _ready() -> void:
	print("--- TireModelV3 step response ---")
	print("At tick 0 we jump the target slip angle from 0 to 5 deg.")
	print("V2 responds instantly. V3 builds up. Sampled every tick.")
	print("")

	tire_v2 = TireModelV2.new()
	tire_v3 = TireModelV3.new()
	tire_v3.relaxation_length = 0.5

	print("Relaxation length: %.2f m" % tire_v3.relaxation_length)
	print("")
	print("%5s | %10s %10s | %10s %10s" % [
		"tick", "V2 Fy", "V3 Fy", "V2 slip_a", "V3 relaxed"
	])

func _physics_process(delta: float) -> void:
	if tick > max_ticks:
		set_physics_process(false)
		print("--- Done ---")
		return

	var target_angle: float = 0.0 if tick == 0 else deg_to_rad(5.0)

	var load := 4000.0
	var vx := 30.0
	var vy := vx * tan(target_angle)

	var r2: Dictionary = tire_v2.step_gd(
		50.0, 0.0, 0.0, load, 1.0, 0.33, vx, vy, 0.0, 0.0)
	var r3: Dictionary = tire_v3.step_gd_with_dt(
		50.0, 0.0, 0.0, load, 1.0, 0.33, vx, vy, 0.0, dt)

	print("%5d | %10.1f %10.1f | %10.4f %10.4f" % [
		tick,
		r2["force_y"],
		r3["force_y"],
		r2["slip_angle"],
		tire_v3.get_relaxed_slip_angle(),
	])

	tick += 1