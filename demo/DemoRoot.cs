using Godot;
using GDExtensionBindgen;

public partial class DemoRoot : Node3D
{
    private VehicleBody? _car;
    private Node3D? _carVisual;
    
    // Wheel Pivots
    private Node3D? _wheelFLPivot;
    private Node3D? _wheelFRPivot;
    private Node3D? _wheelRLPivot;
    private Node3D? _wheelRRPivot;

    // Visuals
    private GpuParticles3D? _smokeEmitter;
    private PackedScene? _skidMarkScene;
    private float _skidTimer = 0.0f;

    private int _currentTier = 4;

    public override void _Ready()
    {
        // 1. Grab visual nodes safely
        _carVisual = GetNodeOrNull<Node3D>("CarVisual");
        
        // Front Wheels
        _wheelFLPivot = GetNodeOrNull<Node3D>("CarVisual/WheelFL_Pivot");
        _wheelFRPivot = GetNodeOrNull<Node3D>("CarVisual/WheelFR_Pivot");
        
        // Rear Wheels (Make sure these match your Scene Tree names!)
        _wheelRLPivot = GetNodeOrNull<Node3D>("CarVisual/WheelRL_Pivot");
        _wheelRRPivot = GetNodeOrNull<Node3D>("CarVisual/WheelRR_Pivot");

        // Effects
        _smokeEmitter = GetNodeOrNull<GpuParticles3D>("SmokeEmitter");
        _skidMarkScene = GD.Load<PackedScene>("res://SkidMark.tscn");

        if (_carVisual == null) GD.PushError("DemoRoot: CarVisual node not found.");
        if (_skidMarkScene == null) GD.PushWarning("DemoRoot: SkidMark.tscn not found. Skid marks will not spawn.");

        // 2. Start with T2c (Tier 4)
        SwapTier(_currentTier);
    }

    public override void _Input(InputEvent @event)
    {
        // Handle tier swapping (1 through 5)
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
        var pos = _car?.GetPosition() ?? new Vector3(1, 0.5f, 1);
        var vel = _car?.GetVelocity() ?? Vector3.Zero;

        _car = tier switch
        {
            1 => new VehicleBodyT1(),
            2 => new VehicleBodyT2a(),
            3 => new VehicleBodyT2b(),
            4 => new VehicleBodyT2c(),
            5 => new VehicleBodyT3(),
            _ => new VehicleBodyT2c()
        };

        if (_car == null)
        {
            GD.PushError("Failed to instantiate the vehicle tier.");
            return;
        }

        _car.SetPosition(pos);
        _car.SetVelocity(vel);
        _currentTier = tier;

        // Pass the Physics Space RID (Needed for Raycasts)
        var world = GetWorld3D();
        if (world != null) _car.SetSpaceRid(world.Space);

        // Update Camera and HUD
        GetNodeOrNull<CameraFollow>("Camera")?.SetTarget(_car);
        GetNodeOrNull<HUD>("HUD")?.SetTarget(_car);

        GD.Print($"Tier swapped to: {_car.GetType().Name}");
    }

    public override void _PhysicsProcess(double delta)
    {
        if (_car == null) return;

        // --- 1. Read Inputs ---
        float rawSteer = Input.GetAxis("steer_left", "steer_right");
        float steer = Mathf.Abs(rawSteer) < 0.05f ? 0.0f : rawSteer; // Deadzone

        _car.SetThrottle(Input.IsActionPressed("accelerate") ? 1.0f : 0.0f);
        _car.SetBrake(Input.IsActionPressed("brake") ? 1.0f : 0.0f);
        _car.SetSteer(steer);

        // --- 2. Step the Physics ---
        _car.Step((float)delta);

        // --- 3. Visual Telemetry (Smoke & Skid Marks) ---
        if (_car.HasMethod("get_wheel_slip_ratio"))
        {
            // Read Rear Wheel Slip (Index 2 and 3)
            float slipRL = (float)_car.Call("get_wheel_slip_ratio", 2);
            float slipRR = (float)_car.Call("get_wheel_slip_ratio", 3);
            
            // Read Lateral Slip Angle (Drifting)
            float angleRL = (float)_car.Call("get_wheel_slip_angle", 2);
            float angleRR = (float)_car.Call("get_wheel_slip_angle", 3);

            bool isSpinning = Mathf.Abs(slipRL) > 0.4f || Mathf.Abs(slipRR) > 0.4f;
            bool isDrifting = Mathf.Abs(angleRL) > 0.15f || Mathf.Abs(angleRR) > 0.15f; // ~8.5 degrees

            if (isSpinning || isDrifting)
            {
                EmitSmoke();
                if (isDrifting) SpawnSkidMarks(delta);
            }
            else
            {
                if (_smokeEmitter != null) _smokeEmitter.Emitting = false;
            }
        }
    }

    private void EmitSmoke()
    {
        if (_smokeEmitter == null || _car == null) return;
        _smokeEmitter.Emitting = true;
        
        var pos = _car.GetPosition();
        float yaw = (float)_car.Call("get_yaw");
        var forward = new Vector3(Mathf.Cos(yaw), 0, -Mathf.Sin(yaw));
        
        // Position smoke behind the car (CG to rear is ~1.3m)
        _smokeEmitter.GlobalPosition = pos - forward * 1.3f - new Vector3(0, 0.4f, 0);
    }

    private void SpawnSkidMarks(double delta)
    {
        if (_skidMarkScene == null || _car == null) return;
        
        _skidTimer += (float)delta;
        if (_skidTimer < 0.05f) return; // Spawn 20 marks per second max
        _skidTimer = 0.0f;

        var pos = _car.GetPosition();
        float yaw = (float)_car.Call("get_yaw");
        var forward = new Vector3(Mathf.Cos(yaw), 0, -Mathf.Sin(yaw));
        var right = new Vector3(Mathf.Cos(yaw + Mathf.Pi/2), 0, -Mathf.Sin(yaw + Mathf.Pi/2));
        
        // Spawn two decals (Left Rear and Right Rear)
        var leftPos = pos - forward * 1.6f - right * 0.8f;
        var rightPos = pos - forward * 1.0f + right * 0.8f;

        SpawnDecal(leftPos, yaw);
        SpawnDecal(rightPos, yaw);
    }

    private void SpawnDecal(Vector3 pos, float yaw)
    {
        if (_skidMarkScene == null) return;
        
        var decal = _skidMarkScene.Instantiate<Decal>();
        AddChild(decal);
        decal.GlobalPosition = new Vector3(pos.X, 0.01f, pos.Z); // 1cm above ground
        decal.Rotation = new Vector3(0, yaw, 0);
    }

    public override void _Process(double delta)
    {
        if (_car == null || _carVisual == null) return;

        // --- 1. Update Car Body Position & Rotation ---
        _carVisual.Position = _car.GetPosition();

        if (_car.HasMethod("get_orientation"))
        {
            // T2c and T3 use a full Quaternion
            _carVisual.Quaternion = (Quaternion)_car.Call("get_orientation");
        }
        else
        {
            // T1, T2a, T2b use simple Euler angles
            float yaw = (float)_car.Call("get_yaw");
            float pitch = _car.HasMethod("get_pitch") ? (float)_car.Call("get_pitch") : 0.0f;
            float roll = _car.HasMethod("get_roll") ? (float)_car.Call("get_roll") : 0.0f;
            _carVisual.Rotation = new Vector3(pitch, yaw, roll);
        }

        // --- 2. Turn the Front Wheels (Steering) ---
        float steerFL = 0.0f;
        float steerFR = 0.0f;

        if (_car.HasMethod("get_steer_angle_for_corner"))
        {
            steerFL = (float)_car.Call("get_steer_angle_for_corner", 0);
            steerFR = (float)_car.Call("get_steer_angle_for_corner", 1);
        }
        else
        {
            float baseSteer = (float)_car.Call("get_steer") * 0.25f;
            steerFL = baseSteer;
            steerFR = baseSteer;
        }

        if (_wheelFLPivot != null) _wheelFLPivot.Rotation = new Vector3(0, -steerFL, 0);
        if (_wheelFRPivot != null) _wheelFRPivot.Rotation = new Vector3(0, -steerFR, 0);

        // --- 3. Spin all Four Wheels Visually ---
        float speed = _car.GetVelocity().Length();
        float wheelRadius = 0.33f; 
        float spinAmount = (speed / wheelRadius) * (float)delta;

        _wheelFLPivot?.GetNodeOrNull<MeshInstance3D>("WheelFL_Mesh")?.RotateX(spinAmount);
        _wheelFRPivot?.GetNodeOrNull<MeshInstance3D>("WheelFR_Mesh")?.RotateX(spinAmount);
        _wheelRLPivot?.GetNodeOrNull<MeshInstance3D>("WheelRL_Mesh")?.RotateX(spinAmount);
        _wheelRRPivot?.GetNodeOrNull<MeshInstance3D>("WheelRR_Mesh")?.RotateX(spinAmount);
    }
}