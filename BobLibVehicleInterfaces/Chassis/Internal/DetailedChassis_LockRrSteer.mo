within BobLibVehicleInterfaces.Chassis.Internal;

model DetailedChassis_LockRrSteer "Chassis with locked rear steering"

  extends BobLibVehicleInterfaces.Chassis.Internal.DetailedChassisBase;

protected
  Modelica.Mechanics.MultiBody.Parts.Mounting1D rearSteerLock annotation(
    Placement(transformation(origin = {30, -90}, extent = {{-10, -10}, {10, 10}}, rotation = -180)));

equation
  connect(rearSteerLock.frame_a, rrAxleDW.axleFrame) annotation(
    Line(points = {{30, -80}, {30, -50}, {0, -50}, {0, -60}}, color = {95, 95, 95}));
  connect(rearSteerLock.flange_b, rrAxleDW.steerFlange) annotation(
    Line(points = {{20, -90}, {0, -90}, {0, -66}}));
  annotation(
    Documentation(info = "<html>
<p>
Model <code>DetailedChassis_LockRrSteer</code> implements the detailed chassis stack with rear steer constrained.
</p>
<p>
It assembles the sprung chassis frame, selected front and rear suspension axles, tires, wheel hubs, aero load path, and locked rear steering fixture behind the public chassis adapter.
</p>
</html>"));
end DetailedChassis_LockRrSteer;
