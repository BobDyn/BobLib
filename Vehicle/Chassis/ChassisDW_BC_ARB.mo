within BobLib.Vehicle.Chassis;

model ChassisDW_BC_ARB "Chassis with locked rear steering"
  
  extends BobLib.Vehicle.Chassis.ChassisBase;

protected
  Modelica.Mechanics.MultiBody.Parts.Mounting1D rearSteerLock annotation(
    Placement(transformation(origin = {30, -90}, extent = {{-10, -10}, {10, 10}}, rotation = -180)));

equation
  connect(rearSteerLock.frame_a, rrAxleDW.axleFrame) annotation(
    Line(points = {{30, -80}, {30, -50}, {0, -50}, {0, -60}}, color = {95, 95, 95}));
  connect(rearSteerLock.flange_b, rrAxleDW.steerFlange) annotation(
    Line(points = {{20, -90}, {0, -90}, {0, -66}}));
end ChassisDW_BC_ARB;
