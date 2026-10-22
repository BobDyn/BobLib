within BobLib.Records.VehicleRecord.Chassis.Suspension;
record WheelRecord

  parameter Templates.Tire.Templates.PartialWheelRecord geometry "Wheel radius, rim dimensions and static toe and camber alignment";

  parameter Templates.Tire.Wheel1DOF_YRecord rotation(wheelJ = 0.01) "Wheel rotational inertia calibration";

  parameter Templates.Tire.Wheel1DOF_ZRecord vertical(
    wheelC = 0,
    wheelD = 0) "Wheel vertical spring and damping calibration";
  parameter WheelComplianceRecord compliance "Six-axis wheel-stack compliance after the upright/caliper mount";
end WheelRecord;
