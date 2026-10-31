using Godot;
using GDExtensionBindgen;

public partial class FirstCSharpTest : Node
{
	public override void _Ready()
	{
		GD.Print("=== C# GDExtension Test ===");

		// Step 1: create
		var car = new VehicleBodyT1();
		GD.Print($"Created: VehicleBodyT1");

		// Step 2: configure
		car.SetPosition(new Vector3(1, 0, 2));
		car.SetVelocity(new Vector3(10, 0, 0));  // 10 m/s forward (+Z world)
		GD.Print($"Position: {car.GetPosition()}");
		GD.Print($"Velocity: {car.GetVelocity()}");
		GD.Print($"Speed:    {car.GetSpeed():F2} m/s");

		// Step 3: advance 0.1 seconds
		GD.Print("Calling Step(0.1)...");
		car.Step(0.1f);

		// Step 4: read back
		var newPos = car.GetPosition();
		var newSpeed = car.GetSpeed();
		GD.Print($"New position: {newPos}");
		GD.Print($"New speed:    {newSpeed:F2} m/s");
		GD.Print($"Moved {newPos.DistanceTo(new Vector3(1, 0, 2)):F3} m");

		// Verify expectations
		bool moved = newPos.X > 1.9 && newPos.X < 2.1;
bool slowed = newSpeed < 10.0f && newSpeed > 9.0f;

		GD.Print("");
		if (moved && slowed)
			GD.Print("PASS: car moved ~1 m in +Z, decelerated slightly");
		else
			GD.Print($"FAIL: moved={moved}, slowed={slowed}");

		GD.Print("=== Test complete ===");
	}
}
