within BobLib.Records.VehicleRecord.EnergyStorage;

record BatteryPackRecord

  parameter Integer Ns(min = 1) = 140 "Battery cells in series";
  parameter Integer Np(min = 1) = 4 "Battery cells in parallel";
  parameter Real SOC_start(unit = "1", min = 0, max = 1) = 1
    "Initial battery state of charge";

  annotation(
    Documentation(info = "<html>
<p>
Record <code>BatteryPackRecord</code> contains the vehicle-level parameters
passed to <code>EnergyStorage.BatteryPack</code>.
</p>
</html>"));
end BatteryPackRecord;
