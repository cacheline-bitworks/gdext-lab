using Godot;
using GDExtensionBindgen;

public partial class CameraFollow : Camera3D
{
    [Export] public float Distance = 8.0f;
    [Export] public float Height = 3.0f;
    [Export] public float LookHeight = 1.0f;
    [Export] public float LerpSpeed = 5.0f;

    private SimulatedEntity? _car;

    public void SetTarget(SimulatedEntity car)
    {
        _car = car;
        if (_car != null) SnapToTarget();
    }

    private float GetYaw() => _car != null ? (float)_car.Call("get_yaw") : 0f;

    private Vector3 TargetPosition()
    {
        if (_car == null) return GlobalPosition;
        var pos = _car.GetPosition();
        var yaw = GetYaw();
        var forward = new Vector3(Mathf.Cos(yaw), 0, -Mathf.Sin(yaw));
        return pos - forward * Distance + Vector3.Up * Height;
    }

    private Vector3 LookPosition()
    {
        if (_car == null) return GlobalPosition;
        return _car.GetPosition() + Vector3.Up * LookHeight;
    }

    private void SnapToTarget()
    {
        if (_car == null) return;
        GlobalPosition = TargetPosition();
        LookAt(LookPosition(), Vector3.Up);
    }

    public override void _Process(double delta)
    {
        if (_car == null) return;
        var desired = TargetPosition();
        var t = Mathf.Clamp(LerpSpeed * (float)delta, 0.0f, 1.0f);
        GlobalPosition = GlobalPosition.Lerp(desired, t);
        LookAt(LookPosition(), Vector3.Up);
    }
}