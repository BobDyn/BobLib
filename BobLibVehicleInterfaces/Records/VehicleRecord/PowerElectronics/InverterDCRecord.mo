within BobLibVehicleInterfaces.Records.VehicleRecord.PowerElectronics;

record InverterDCRecord
  import SI = Modelica.Units.SI;

  parameter Real eta_mot = 0.97 "Inverter efficiency (motoring)";
  parameter Real eta_reg = 0.95 "Inverter efficiency (regen)";
  parameter SI.Power P_max_mot = 124e3 "Maximum motoring output power";
  parameter SI.Power P_max_reg = 124e3
    "Maximum regenerative input power as positive magnitude";
  parameter SI.Voltage V_dc_max = 588
    "Maximum DC bus voltage before regen is disabled";

  annotation(
    Documentation(info = "<html>
<p>
Record <code>InverterDCRecord</code> contains the vehicle-level parameters
passed to <code>PowerElectronics.InverterDC</code>.
</p>
<p>
Lower-level current, power, voltage, and efficiency-map defaults remain owned by
the inverter adapter unless a vehicle definition needs to expose them.
</p>
</html>"));
end InverterDCRecord;
