extends Node

var tire_v1: TireModelV1
var tire_v2: TireModelV2

func _ready() -> void:
	tire_v1 = TireModelV1.new()
	tire_v2 = TireModelV2.new()

	print("")
	print("=================================================================")
	print(" TireModelV2 numerical verification")
	print("=================================================================")
	print("Nominal load:     %.0f N" % tire_v2.nominal_load)
	print("Load sensitivity: %.3f" % tire_v2.load_sensitivity)
	print("")

	_test_effective_mu()
	_test_longitudinal()
	_test_lateral()
	_test_combined_slip()
	_test_peak_comparison()

	print("")
	print("=================================================================")
	print(" Done")
	print("=================================================================")
	print("")

func _test_effective_mu() -> void:
	print("--- 1. Effective friction coefficient vs load ---")
	print("%10s %12s %12s" % ["load (N)", "V2 mu_eff", "delta from 1.0"])

	var loads: Array[float] = [1000.0, 2000.0, 3000.0, 4000.0, 5000.0, 6000.0, 8000.0]
	for load in loads:
		var mu_eff: float = tire_v2.effective_mu(load, 1.0)
		print("%10.0f %12.4f %12.4f" % [load, mu_eff, mu_eff - 1.0])

	print("")
	print("  At nominal load (4000 N), mu should equal 1.0.")
	print("  Above nominal, mu should drop.")
	print("  Below nominal, mu should rise.")
	print("")

func _test_longitudinal() -> void:
	print("--- 2. Longitudinal force: V1 vs V2 at three loads ---")
	print("    (slip ratio ranges -0.2 to +0.2)")
	print("")
	print("%8s | %10s %10s | %10s %10s" % ["slip", "V1@2k", "V1@4k", "V2@2k", "V2@4k"])
	print("---------|-----------------------|----------------------")

	for i in 11:
		var slip: float = -0.2 + float(i) * 0.04
		var v1_2k: float = tire_v1.compute_force_x(slip, 2000.0, 1.0)
		var v1_4k: float = tire_v1.compute_force_x(slip, 4000.0, 1.0)
		var v2_2k: float = tire_v2.compute_force_x(slip, 2000.0, 1.0)
		var v2_4k: float = tire_v2.compute_force_x(slip, 4000.0, 1.0)
		print("%8.3f | %10.1f %10.1f | %10.1f %10.1f" % [slip, v1_2k, v1_4k, v2_2k, v2_4k])

	print("")
	print("  Watch: at 2000 N, V2 should be higher than V1 (load < nominal).")
	print("         at 4000 N, V2 should equal V1 (load = nominal).")
	print("")

func _test_lateral() -> void:
	print("--- 3. Lateral force: V1 vs V2 at three loads ---")
	print("    (slip angle ranges -15 to +15 degrees)")
	print("")
	print("%8s | %10s %10s | %10s %10s" % ["angle", "V1@2k", "V1@4k", "V2@2k", "V2@4k"])
	print("---------|-----------------------|----------------------")

	for i in 11:
		var angle_deg: float = -15.0 + float(i) * 3.0
		var angle_rad: float = deg_to_rad(angle_deg)
		var v1_2k: float = tire_v1.compute_force_y(angle_rad, 2000.0, 1.0)
		var v1_4k: float = tire_v1.compute_force_y(angle_rad, 4000.0, 1.0)
		var v2_2k: float = tire_v2.compute_force_y(angle_rad, 2000.0, 1.0)
		var v2_4k: float = tire_v2.compute_force_y(angle_rad, 4000.0, 1.0)
		print("%8.1f | %10.1f %10.1f | %10.1f %10.1f" % [angle_deg, v1_2k, v1_4k, v2_2k, v2_4k])

	print("")

func _test_combined_slip() -> void:
	print("--- 4. Combined slip (friction ellipse) demonstration ---")
	print("    Load 4000 N, fixed slip angle = 5 deg, sweep slip ratio")
	print("    Watch how Fy is sacrificed as Fx grows.")
	print("")
	print("%10s %10s %10s %12s %12s" % ["slip", "Fx (N)", "Fy (N)", "F_total", "F_peak"])

	var load: float = 4000.0
	var mu_eff: float = tire_v2.effective_mu(load, 1.0)
	var f_peak: float = mu_eff * load

	for i in 13:
		var slip: float = -0.3 + float(i) * 0.05
		var angle_rad: float = deg_to_rad(5.0)
		# Approximate vx from slip assuming omega*R held constant
		var omega: float = 50.0 + slip * 50.0
		var result: Dictionary = tire_v2.step_gd(
			omega, 0.0, 0.0,
			load, 1.0, 0.33,
			30.0, 30.0 * tan(angle_rad), 0.0
		)
		var fx: float = result["force_x"]
		var fy: float = result["force_y"]
		var f_total: float = sqrt(fx * fx + fy * fy)
		print("%10.3f %10.1f %10.1f %12.1f %12.1f" % [slip, fx, fy, f_total, f_peak])

	print("")
	print("  Watch: F_total should never exceed F_peak.")
	print("         As Fx increases, Fy should decrease to make room.")
	print("")

func _test_peak_comparison() -> void:
	print("--- 5. Peak force scaling: linear vs load sensitive ---")
	print("")
	print("%10s %12s %12s %12s" % ["load (N)", "V1 peak", "V2 peak", "ratio V2/V1"])

	var loads: Array[float] = [1000.0, 2000.0, 3000.0, 4000.0, 6000.0, 8000.0]
	for load in loads:
		# Sweep slip to find peak (rough)
		var v1_peak: float = 0.0
		var v2_peak: float = 0.0
		for i in 100:
			var slip: float = -0.3 + float(i) * 0.006
			var v1_f: float = abs(tire_v1.compute_force_x(slip, load, 1.0))
			var v2_f: float = abs(tire_v2.compute_force_x(slip, load, 1.0))
			if v1_f > v1_peak: v1_peak = v1_f
			if v2_f > v2_peak: v2_peak = v2_f
		var ratio: float = v2_peak / v1_peak if v1_peak > 0.0 else 0.0
		print("%10.0f %12.1f %12.1f %12.4f" % [load, v1_peak, v2_peak, ratio])

	print("")
	print("  V1 ratio should always be 1.0 (no load sensitivity).")
	print("  V2 ratio should drop below 1.0 as load rises above nominal.")
	print("")