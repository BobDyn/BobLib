within BobLib.Resources.VehicleRecord.Chassis.Suspension.Templates.SteeringRack;

record RackAndPinionRecord
  import SI = Modelica.Units.SI;

  parameter SI.Position leftPickup[3]
    "Left pickup coordinate, expressed in chassis frame";

  parameter Real cFactor
    "Rack C-factor (m rack travel per pinion revolution)";

end RackAndPinionRecord;
