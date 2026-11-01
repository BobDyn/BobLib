within BobLib.Chassis.Suspension.Tires.Wheels;

partial model PartialWheelCompliance "Calibration and mechanical contract for wheel-stack compliance"

  parameter BobLib.Records.VehicleRecord.Chassis.Suspension.WheelComplianceRecord pCompliance "Local-axis stiffnesses and derived damping calibration" annotation(Dialog(group = "Calibration"));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame_a "Upright and caliper side" annotation(Placement(transformation(origin = {-200, 0}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame_b "Wheel and contact-load side" annotation(Placement(transformation(origin = {200, 0}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {100, 0}, extent = {{-10, -10}, {10, 10}})));

  annotation(Diagram(coordinateSystem(extent = {{-200, -40}, {200, 120}})));
end PartialWheelCompliance;
