within BobLib.Records.VehicleRecord.Chassis.Suspension;
record TireRecord

  parameter WheelRecord wheel "Wheel geometry, inertia and vertical compliance calibration";
  parameter Templates.Tire.MF52.MF52Record force "Tire-force law coefficients and operating limits";
end TireRecord;
