extends Node2D

# --- Configuration ---
const SAMPLES := 200
const SLIP_RATIO_MAX := 0.25        # x-axis range for longitudinal (dimensionless)
const SLIP_ANGLE_MAX_DEG := 15.0    # x-axis range for lateral (degrees)

const PLOT_X := 50.0
const PLOT_W := 640.0
const PLOT_H := 220.0
const PLOT_TOP_Y := 50.0
const PLOT_BOTTOM_Y := 340.0

# --- Test parameters ---
var load := 4000.0     # N per tire (roughly 400 kg)
var friction := 1.0    # dry asphalt

# --- Runtime ---
var tire: TireModelV1

func _ready() -> void:
	tire = TireModelV1.new()
	queue_redraw()

func _draw() -> void:
	if tire == null:
		return

	var font := ThemeDB.fallback_font
	var font_size := 14
	var label_size := 11

	# --- Top plot: Fx vs slip ratio ---
	_draw_plot(
		Vector2(PLOT_X, PLOT_TOP_Y),
		"Longitudinal force vs slip ratio  (Fz=%.0f N, mu=%.2f)" % [load, friction],
		"slip ratio",
		"Fx (N)",
		"Longitudinal peak",
		func(t: float) -> Vector2:
			var slip: float = lerp(-SLIP_RATIO_MAX, SLIP_RATIO_MAX, t)
			return Vector2(slip, tire.compute_force_x(slip, load, friction)),
		font, font_size, label_size
	)

	# --- Bottom plot: Fy vs slip angle ---
	_draw_plot(
		Vector2(PLOT_X, PLOT_BOTTOM_Y),
		"Lateral force vs slip angle  (Fz=%.0f N, mu=%.2f)" % [load, friction],
		"slip angle (deg)",
		"Fy (N)",
		"Lateral peak",
		func(t: float) -> Vector2:
			var angle_deg: float = lerp(-SLIP_ANGLE_MAX_DEG, SLIP_ANGLE_MAX_DEG, t)
			return Vector2(angle_deg, tire.compute_force_y(deg_to_rad(angle_deg), load, friction)),
		font, font_size, label_size
	)

func _draw_plot(origin: Vector2, title: String, x_label: String, y_label: String,
		peak_label: String, sampler: Callable, font: Font, font_size: int,
		label_size: int) -> void:
	# --- Sample the curve ---
	var raw: Array[Vector2] = []
	var max_x_abs := 0.0
	var max_y_abs := 0.0
	for i in SAMPLES + 1:
		var t := float(i) / float(SAMPLES)
		var p: Vector2 = sampler.call(t)
		raw.append(p)
		max_x_abs = max(max_x_abs, abs(p.x))
		max_y_abs = max(max_y_abs, abs(p.y))

	if max_x_abs < 0.0001 or max_y_abs < 0.0001:
		return

	# --- Map to screen coordinates ---
	var center := origin + Vector2(PLOT_W * 0.5, PLOT_H * 0.5)
	var scale_x := (PLOT_W * 0.5) / max_x_abs
	var scale_y := (PLOT_H * 0.5) / max_y_abs

	var points := PackedVector2Array()
	for p in raw:
		points.append(center + Vector2(p.x * scale_x, -p.y * scale_y))

	# --- Background panel ---
	draw_rect(Rect2(origin, Vector2(PLOT_W, PLOT_H)), Color(0.08, 0.08, 0.10), true)

	# --- Axes (crosshair at zero) ---
	var axis_color := Color(0.35, 0.35, 0.40)
	draw_line(Vector2(origin.x, center.y), Vector2(origin.x + PLOT_W, center.y), axis_color, 1.0)
	draw_line(Vector2(center.x, origin.y), Vector2(center.x, origin.y + PLOT_H), axis_color, 1.0)

	# --- Curve ---
	draw_polyline(points, Color(0.45, 0.95, 0.55), 2.0, true)

	# --- Peak marker: scan for max |y| and record its x ---
	var peak_val := 0.0
	var peak_x := 0.0
	for p in raw:
		if abs(p.y) > abs(peak_val):
			peak_val = p.y
			peak_x = p.x

	var peak_screen := center + Vector2(peak_x * scale_x, -peak_val * scale_y)
	draw_circle(peak_screen, 4.0, Color(1.0, 0.85, 0.2))
	draw_string(font, peak_screen + Vector2(8, -8),
		"%s: %.0f N at %.3f" % [peak_label, peak_val, peak_x],
		HORIZONTAL_ALIGNMENT_LEFT, -1, label_size, Color(1.0, 0.85, 0.2))

	# --- Title ---
	draw_string(font, origin + Vector2(0, -12), title,
		HORIZONTAL_ALIGNMENT_LEFT, -1, font_size, Color.WHITE)

	# --- Axis labels ---
	draw_string(font, Vector2(origin.x + PLOT_W - 90, origin.y + PLOT_H + 18),
		x_label, HORIZONTAL_ALIGNMENT_LEFT, -1, label_size, Color(0.7, 0.7, 0.75))
	draw_string(font, Vector2(origin.x - 38, origin.y + 4),
		y_label, HORIZONTAL_ALIGNMENT_LEFT, -1, label_size, Color(0.7, 0.7, 0.75))