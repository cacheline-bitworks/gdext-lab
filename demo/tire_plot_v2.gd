extends Node2D

const SAMPLES := 200
const SLIP_RATIO_MAX := 0.25
const SLIP_ANGLE_MAX_DEG := 15.0

const PLOT_X := 50.0
const PLOT_W := 640.0
const PLOT_H := 180.0
const PLOT_TOP_Y := 60.0
const PLOT_MID_Y := 290.0
const PLOT_BOT_Y := 520.0

var tire_v1: TireModelV1
var tire_v2: TireModelV2

func _ready() -> void:
	tire_v1 = TireModelV1.new()
	tire_v2 = TireModelV2.new()
	queue_redraw()

func _draw() -> void:
	if tire_v1 == null or tire_v2 == null:
		return

	var font: Font = ThemeDB.fallback_font

	_draw_load_sensitivity_plot(
		Vector2(PLOT_X, PLOT_TOP_Y),
		"Load sensitivity: Fx vs slip ratio at 2000 / 4000 / 6000 N",
		font
	)

	_draw_v1_v2_plot(
		Vector2(PLOT_X, PLOT_MID_Y),
		"V1 vs V2: Fy vs slip angle at 4000 N (bright) and 6000 N (dim)",
		font
	)

	_draw_ellipse_plot(
		Vector2(PLOT_X, PLOT_BOT_Y),
		"Friction ellipse: Fy vs Fx at 5 deg slip angle",
		font
	)

# ---------------------------------------------------------------------------
# Plot 1: load sensitivity
# ---------------------------------------------------------------------------
func _draw_load_sensitivity_plot(origin: Vector2, title: String, font: Font) -> void:
	draw_rect(Rect2(origin, Vector2(PLOT_W, PLOT_H)), Color(0.08, 0.08, 0.10), true)

	var center: Vector2 = origin + Vector2(PLOT_W * 0.5, PLOT_H * 0.5)
	var scale_x: float = (PLOT_W * 0.5) / SLIP_RATIO_MAX
	var scale_y: float = (PLOT_H * 0.5) / 6500.0

	draw_line(Vector2(origin.x, center.y), Vector2(origin.x + PLOT_W, center.y),
		Color(0.35, 0.35, 0.40), 1.0)
	draw_line(Vector2(center.x, origin.y), Vector2(center.x, origin.y + PLOT_H),
		Color(0.35, 0.35, 0.40), 1.0)

	var loads: Array[float] = [2000.0, 4000.0, 6000.0]
	var colors: Array[Color] = [
		Color(0.4, 0.7, 1.0),
		Color(0.3, 1.0, 0.5),
		Color(1.0, 0.7, 0.3),
	]

	for idx in range(loads.size()):
		var load: float = loads[idx]
		var color: Color = colors[idx]
		var points := PackedVector2Array()
		for i in SAMPLES + 1:
			var t: float = float(i) / float(SAMPLES)
			var slip: float = lerpf(-SLIP_RATIO_MAX, SLIP_RATIO_MAX, t)
			var fx: float = tire_v2.compute_force_x(slip, load, 1.0)
			points.append(center + Vector2(slip * scale_x, -fx * scale_y))
		draw_polyline(points, color, 2.0, true)

	draw_string(font, origin + Vector2(0, -12), title,
		HORIZONTAL_ALIGNMENT_LEFT, -1, 13, Color.WHITE)
	draw_string(font, Vector2(origin.x + PLOT_W - 80, origin.y + PLOT_H + 16),
		"slip ratio", HORIZONTAL_ALIGNMENT_LEFT, -1, 11, Color(0.7, 0.7, 0.75))
	draw_string(font, Vector2(origin.x - 40, origin.y + 4),
		"Fx (N)", HORIZONTAL_ALIGNMENT_LEFT, -1, 11, Color(0.7, 0.7, 0.75))

# ---------------------------------------------------------------------------
# Plot 2: V1 vs V2
# ---------------------------------------------------------------------------
func _draw_v1_v2_plot(origin: Vector2, title: String, font: Font) -> void:
	draw_rect(Rect2(origin, Vector2(PLOT_W, PLOT_H)), Color(0.08, 0.08, 0.10), true)

	var center: Vector2 = origin + Vector2(PLOT_W * 0.5, PLOT_H * 0.5)
	var scale_x: float = (PLOT_W * 0.5) / SLIP_ANGLE_MAX_DEG
	var scale_y: float = (PLOT_H * 0.5) / 6500.0

	draw_line(Vector2(origin.x, center.y), Vector2(origin.x + PLOT_W, center.y),
		Color(0.35, 0.35, 0.40), 1.0)
	draw_line(Vector2(center.x, origin.y), Vector2(center.x, origin.y + PLOT_H),
		Color(0.35, 0.35, 0.40), 1.0)

	# V1: blue-ish
	var v1_solid := Color(0.4, 0.6, 0.9)
	var v1_dim := Color(0.4, 0.6, 0.9, 0.5)
	_draw_force_y_curve(tire_v1, 4000.0, v1_solid, center, scale_x, scale_y)
	_draw_force_y_curve(tire_v1, 6000.0, v1_dim, center, scale_x, scale_y)

	# V2: green-ish
	var v2_solid := Color(0.4, 1.0, 0.5)
	var v2_dim := Color(0.4, 1.0, 0.5, 0.5)
	_draw_force_y_curve(tire_v2, 4000.0, v2_solid, center, scale_x, scale_y)
	_draw_force_y_curve(tire_v2, 6000.0, v2_dim, center, scale_x, scale_y)

	draw_string(font, origin + Vector2(0, -12), title,
		HORIZONTAL_ALIGNMENT_LEFT, -1, 13, Color.WHITE)
	draw_string(font, origin + Vector2(10, 15), "V1 (blue)   V2 (green)",
		HORIZONTAL_ALIGNMENT_LEFT, -1, 11, Color(0.8, 0.8, 0.8))
	draw_string(font, Vector2(origin.x + PLOT_W - 90, origin.y + PLOT_H + 16),
		"slip angle (deg)", HORIZONTAL_ALIGNMENT_LEFT, -1, 11, Color(0.7, 0.7, 0.75))
	draw_string(font, Vector2(origin.x - 40, origin.y + 4),
		"Fy (N)", HORIZONTAL_ALIGNMENT_LEFT, -1, 11, Color(0.7, 0.7, 0.75))

func _draw_force_y_curve(tire: TireModel, load: float, color: Color,
		center: Vector2, scale_x: float, scale_y: float) -> void:
	var points := PackedVector2Array()
	for i in SAMPLES + 1:
		var t: float = float(i) / float(SAMPLES)
		var angle_deg: float = lerpf(-SLIP_ANGLE_MAX_DEG, SLIP_ANGLE_MAX_DEG, t)
		var angle_rad: float = deg_to_rad(angle_deg)
		var fy: float = tire.compute_force_y(angle_rad, load, 1.0)
		points.append(center + Vector2(angle_deg * scale_x, -fy * scale_y))
	draw_polyline(points, color, 2.0, true)

# ---------------------------------------------------------------------------
# Plot 3: friction ellipse
# ---------------------------------------------------------------------------
func _draw_ellipse_plot(origin: Vector2, title: String, font: Font) -> void:
	draw_rect(Rect2(origin, Vector2(PLOT_W, PLOT_H)), Color(0.08, 0.08, 0.10), true)

	var center: Vector2 = origin + Vector2(PLOT_W * 0.5, PLOT_H * 0.5)
	var scale_x: float = (PLOT_W * 0.5) / 6500.0
	var scale_y: float = (PLOT_H * 0.5) / 6500.0

	draw_line(Vector2(origin.x, center.y), Vector2(origin.x + PLOT_W, center.y),
		Color(0.35, 0.35, 0.40), 1.0)
	draw_line(Vector2(center.x, origin.y), Vector2(center.x, origin.y + PLOT_H),
		Color(0.35, 0.35, 0.40), 1.0)

	var load: float = 4000.0
	var friction: float = 1.0
	var mu_eff: float = tire_v2.effective_mu(load, friction)
	var f_peak: float = mu_eff * load

	# Draw the ellipse itself
	var ellipse_points := PackedVector2Array()
	for i in 64:
		var t: float = float(i) / 64.0 * TAU
		var ex: float = cos(t) * f_peak
		var ey: float = sin(t) * f_peak
		ellipse_points.append(center + Vector2(ex * scale_x, -ey * scale_y))
	draw_polyline(ellipse_points, Color(0.5, 0.5, 0.55), 1.0, true)

	# Trace the actual (Fx, Fy) as slip ratio sweeps
	var fixed_slip_angle: float = deg_to_rad(5.0)
	var points := PackedVector2Array()
	for i in SAMPLES + 1:
		var t: float = float(i) / float(SAMPLES)
		var slip: float = lerpf(-0.3, 0.3, t)
		var result: Dictionary = tire_v2.step_gd(
			50.0 + slip * 50.0, 0.0, 0.0,
			load, friction, 0.33,
			30.0, 30.0 * tan(fixed_slip_angle), 0.0
		)
		var fx: float = result["force_x"]
		var fy: float = result["force_y"]
		points.append(center + Vector2(fx * scale_x, -fy * scale_y))

	draw_polyline(points, Color(1.0, 0.6, 0.3), 2.0, true)

	draw_string(font, origin + Vector2(0, -12), title,
		HORIZONTAL_ALIGNMENT_LEFT, -1, 13, Color.WHITE)
	draw_string(font, origin + Vector2(10, 15), "Ellipse (grey)   Trace (orange)",
		HORIZONTAL_ALIGNMENT_LEFT, -1, 11, Color(0.8, 0.8, 0.8))
	draw_string(font, Vector2(origin.x + PLOT_W - 70, origin.y + PLOT_H + 16),
		"Fx (N)", HORIZONTAL_ALIGNMENT_LEFT, -1, 11, Color(0.7, 0.7, 0.75))
	draw_string(font, Vector2(origin.x - 40, origin.y + 4),
		"Fy (N)", HORIZONTAL_ALIGNMENT_LEFT, -1, 11, Color(0.7, 0.7, 0.75))
