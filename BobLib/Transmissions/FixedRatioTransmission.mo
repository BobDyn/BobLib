within BobLib.Transmissions;
model FixedRatioTransmission

  "Fixed-ratio transmission exposed through the neutral VehicleInterfaces contract"
  extends VehicleInterfaces.Icons.Transmission;
  extends VehicleInterfaces.Transmissions.Interfaces.Base;

  import SI = Modelica.Units.SI;

  parameter Real gearRatio = 3.31
    "Input speed divided by output speed";

  BobLib.Transmissions.Internal.FixedRatioGear gear(
    gearRatio = gearRatio) annotation(
      Placement(transformation(extent = {{-24, -24}, {24, 24}})));

  Modelica.Mechanics.Rotational.Sensors.SpeedSensor outputSpeed annotation(
      Placement(transformation(origin = {58, 48}, extent = {{10, -10}, {-10, 10}})));

  output SI.AngularVelocity w_out "Transmission output speed";

protected
  VehicleInterfaces.Interfaces.TransmissionBus transmissionBus annotation(
    Placement(transformation(extent = {{-60, 50}, {-40, 70}})));

equation
  w_out = outputSpeed.w;

  connect(controlBus.transmissionBus, transmissionBus) annotation(
    Line(points = {{-100, 60}, {-50, 60}}, color = {255, 204, 51}, thickness = 0.5));
  connect(engineFlange.flange, gear.inputFlange) annotation(
    Line(points = {{-100, 0}, {-24, 0}}, color = {135, 135, 135}, thickness = 0.5));
  connect(gear.outputFlange, drivelineFlange.flange) annotation(
    Line(points = {{24, 0}, {100, 0}}, color = {135, 135, 135}, thickness = 0.5));
  connect(outputSpeed.flange, gear.outputFlange) annotation(
    Line(points = {{68, 48}, {78, 48}, {78, 0}, {24, 0}}, color = {0, 0, 0}));
  connect(outputSpeed.w, transmissionBus.outputSpeed) annotation(
    Line(points = {{47, 48}, {-50, 48}, {-50, 60}}, color = {0, 0, 127}));

  annotation(Documentation(info = "<html>
<p>
Model <code>FixedRatioTransmission</code> is the BobLib fixed-ratio
transmission adapter for the neutral VehicleInterfaces transmission contract.
</p>
<p>
It is intentionally small: the public model owns the VI engine/driveline
flanges and control-bus publishing, while
<code>Transmissions.Internal.FixedRatioGear</code> owns the gear relation.
Battery-electric architectures can use this as the single-speed reducer;
internal-combustion references can replace it with an automatic or manual
adapter when gearbox mode, clutch, launch-device, or gear-selection behavior is
actually modeled.
</p>
</html>"));
end FixedRatioTransmission;
