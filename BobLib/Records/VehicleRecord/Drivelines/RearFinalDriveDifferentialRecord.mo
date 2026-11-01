within BobLib.Records.VehicleRecord.Drivelines;
record RearFinalDriveDifferentialRecord

  parameter Modelica.Units.SI.Position mountOffset[3] = zeros(3)
    "Rigid mount position relative to the chassis attachment frame, resolved in that frame" annotation(Evaluate = false);

  parameter Real inputRatio(min = Modelica.Constants.small) = 1 "Differential brake/input shaft speed divided by carrier speed";
  parameter Modelica.Units.SI.Inertia inputInertia(min = Modelica.Constants.small) = 0.04 "Rotational inertia at the differential input shaft";
  parameter DifferentialRecord differential "Torque-sharing, preload and friction calibration for the selected differential architecture";
  parameter HalfshaftRecord halfshaft "Calibration copied to both driven halfshafts";
  parameter HalfshaftRecord frontBrakeHalfshaft "Shared calibration of optional undriven front brake shafts";
end RearFinalDriveDifferentialRecord;
