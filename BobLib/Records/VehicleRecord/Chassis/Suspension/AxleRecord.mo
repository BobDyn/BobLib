within BobLib.Records.VehicleRecord.Chassis.Suspension;

partial record AxleRecord

  parameter Modelica.Units.SI.Position rideHeightReference[3] = zeros(3) "Left ride-height measurement point in vehicle design coordinates; mirrored for the right side";
  parameter Templates.DoubleWishbone.WishboneUprightLoopRecord geometry "Wishbone and upright hardpoint geometry in vehicle design coordinates";
  parameter Templates.SteeringRack.RackAndPinionRecord steering "Rack and pinion geometry and motion conversion";
  parameter Templates.AxleMassRecord masses "Per-side suspension mass properties; mirrored to construct the opposite side";
  parameter TireRecord tire "Wheel and tire-force calibration shared by the axle";

  parameter Templates.Stabar.StabarRecord stabilizer(
    leftArmEnd = zeros(3),
    leftBarEnd = zeros(3),
    barRate = 0) "Anti-roll-bar pickup geometry and torsional rate";

end AxleRecord;
