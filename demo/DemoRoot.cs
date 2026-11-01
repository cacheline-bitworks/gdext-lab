using Godot;
using GDExtensionBindgen;

public partial class DemoRoot : Node3D
{
  private VehicleBody? _car;       // ← VehicleBody has SetThrottle/SetBrake/SetSteer
private MeshInstance3D? _mesh;
    private int _currentTier = 1;
            float rawSteer = Input.GetAxis("steer_left", "steer_right");
        // Deadzone: ignore tiny inputs
        

    public override void _Ready()
    {
        _mesh = GetNodeOrNull<MeshInstance3D>("CarMesh");
        if (_mesh == null) GD.PushError("CarMesh node missing");

        SwapTier(1);

        float steer = Mathf.Abs(rawSteer) < 0.05f ? 0.0f : rawSteer;
        _car.SetSteer(steer);
    }

    public override void _Input(InputEvent @event)
    {
        if (@event is InputEventKey { Pressed: true, Echo: false } key)
        {
            switch (key.Keycode)
            {
                case Key.Key1: SwapTier(1); break;
                case Key.Key2: SwapTier(2); break;
                case Key.Key3: SwapTier(3); break;
                case Key.Key4: SwapTier(4); break;
                case Key.Key5: SwapTier(5); break;
            }
        }
    }

    private void SwapTier(int tier)
    {
        var pos = _car?.GetPosition() ?? new Vector3(1, 0.5f, 2);
        var vel = _car?.GetVelocity() ?? Vector3.Zero;

        _car = tier switch
        {
            1 => new VehicleBodyT1(),
            2 => new VehicleBodyT2a(),
            3 => new VehicleBodyT2b(),
            4 => new VehicleBodyT2c(),
            5 => new VehicleBodyT3(),
            _ => new VehicleBodyT1()
        };

        _car.SetPosition(pos);
        _car.SetVelocity(vel);
        _currentTier = tier;

        var camera = GetNodeOrNull<CameraFollow>("Camera");
        if (camera != null) camera.SetTarget(_car);

        var hud = GetNodeOrNull<HUD>("HUD");
        if (hud != null) hud.SetTarget(_car);

        GD.Print($"Tier: {_car.GetType().Name} (press 1-5 to swap)");
    }

    public override void _PhysicsProcess(double delta)
    {
        if (_car == null) return;

        _car.SetThrottle(Input.IsActionPressed("accelerate") ? 1.0f : 0.0f);
        _car.SetBrake(Input.IsActionPressed("brake") ? 1.0f : 0.0f);
        _car.SetSteer(Input.GetAxis("steer_left", "steer_right"));

        _car.Step((float)delta);

        if (_mesh != null)
        {
            _mesh.Position = _car.GetPosition() + Vector3.Up * 0.5f;
            var yaw = (float)_car.Call("get_yaw");
            _mesh.Rotation = new Vector3(0, yaw, 0);
        }
    }
}