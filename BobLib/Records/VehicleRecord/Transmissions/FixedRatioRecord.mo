within BobLib.Records.VehicleRecord.Transmissions;
record FixedRatioRecord

  parameter Real gearRatio = 3.31
    "Input speed divided by output speed";
  parameter Real efficiency(min = Modelica.Constants.small, max = 1) = 1
    "Mechanical efficiency in either power-flow direction" annotation(Evaluate = false);
  annotation(Documentation(info = "<html><p>Component calibration.</p></html>"));
end FixedRatioRecord;
