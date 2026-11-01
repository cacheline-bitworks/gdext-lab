using Godot;
using GDExtensionBindgen;

public partial class HUD : CanvasLayer
{
    private SimulatedEntity? _car;
    private Label? _label;

    public void SetTarget(SimulatedEntity car) => _car = car;

    public override void _Ready()
    {
        _label = GetNodeOrNull<Label>("SpeedLabel");
        if (_label == null) GD.PushError("SpeedLabel node missing");
    }

    public override void _Process(double delta)
    {
        if (_car == null || _label == null) return;
        var speed = _car.GetVelocity().Length();
        var kmh = speed * 3.6f;
        _label.Text = $"{kmh:F0} km/h";
    }
}