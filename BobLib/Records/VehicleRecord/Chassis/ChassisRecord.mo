within BobLib.Records.VehicleRecord.Chassis;

record ChassisRecord

  replaceable record FrontAxleRecord = Suspension.DoubleWishboneBellcrankStabarRecord constrainedby Suspension.AxleRecord;
  replaceable record RearAxleRecord = Suspension.DoubleWishboneBellcrankStabarRecord constrainedby Suspension.AxleRecord;

  parameter FrameRecord frame(
    frontReference = {frontAxle.geometry.wheelCenter[1], 0, frontAxle.geometry.wheelCenter[3]},
    rearReference = {rearAxle.geometry.wheelCenter[1], 0, rearAxle.geometry.wheelCenter[3]}) "Sprung-frame reference geometry, mass properties and torsional compliance";
  parameter FrontAxleRecord frontAxle "Front suspension, steering, tire and stabilizer calibration";
  parameter RearAxleRecord rearAxle "Rear suspension, steering, tire and stabilizer calibration";

end ChassisRecord;
