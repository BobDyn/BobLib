within BobLib.Transmissions;
model FixedRatioTransmission

  "Fixed-ratio transmission exposed through the neutral VehicleInterfaces contract"

  import Modelica.Units.SI;

  parameter BobLib.Records.VehicleRecord.Transmissions.FixedRatioRecord pTransmission "Transmission reduction calibration" annotation(Dialog(group = "Calibration"));

  extends VehicleInterfaces.Icons.Transmission;

  extends BobLib.Transmissions.Interfaces.Base(
    includeMount = includeEngineBearing or includeDrivelineBearing or usingMultiBodyEngine or usingMultiBodyDriveline);

  parameter Modelica.Mechanics.MultiBody.Types.Axis shaftAxis = {0, 1, 0}
    "Common input/output rotation axis in transmissionMount" annotation(Dialog(group = "Operating settings"), Evaluate = true);

  BobLib.Transmissions.Internal.FixedRatioGear gear(
    gearRatio = pTransmission.gearRatio, efficiency = pTransmission.efficiency, useSupport = includeMount) annotation(
      Placement(transformation(extent = {{-24, -24}, {24, 24}})));

  Modelica.Mechanics.Rotational.Sensors.SpeedSensor outputSpeed annotation(
      Placement(transformation(origin = {58, 48}, extent = {{10, -10}, {-10, 10}})));

  output SI.AngularVelocity w_out "Transmission output speed";

protected
  outer Modelica.Mechanics.MultiBody.World world;

  Modelica.Mechanics.MultiBody.Parts.Mounting1D housing(n = Modelica.Math.Vectors.normalizeWithAssert(shaftAxis)) if includeMount annotation(
    Placement(visible = includeMount, transformation(origin = {0, -60}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Sources.IntegerConstant currentGear(k = 1)
    "The fixed reducer has one permanently engaged gear" annotation(
    Placement(transformation(origin = {-30, 90}, extent = {{10, -10}, {-10, 10}})));
  BobLib.Transmissions.Interfaces.TransmissionBus transmissionBus annotation(
    Placement(transformation(extent = {{-60, 50}, {-40, 70}})));

  BobLib.Transmissions.Interfaces.TransmissionControlBus transmissionControlBus
    "Typed transmissionControlBus signal contract" annotation(
      Placement(transformation(origin = {-75, 80}, extent = {{-10, -10}, {10, 10}})));

equation
  connect(transmissionControlBus, controlBus.transmissionControlBus) annotation(
    Line(points = {{-75, 80}, {-75, 60}, {-100, 60}}, color = {255, 204, 51}, thickness = 0.5));
  assert(not includeMount or world.driveTrainMechanics3D,
    "Mounted transmission requires world.driveTrainMechanics3D=true");
  connect(gear.support, housing.flange_b) annotation(
    Line(visible = includeMount, points = {{0, -24}, {20, -24}, {20, -60}, {10, -60}}, color = {0, 0, 0}));
  connect(housing.frame_a, transmissionMount) annotation(
    Line(visible = includeMount, points = {{0, -70}, {0, -100}}, color = {95, 95, 95}));
  connect(engineFlange.bearingFrame, transmissionMount) annotation(
    Line(
      visible = includeMount,
      points = {{-100, 0}, {-90, 0}, {-90, -90}, {0, -90}, {0, -100}},
      color = {95, 95, 95}));
  connect(drivelineFlange.bearingFrame, transmissionMount) annotation(
    Line(visible = includeMount, points = {{100, 0}, {90, 0}, {90, -90}, {0, -90}, {0, -100}}, color = {95, 95, 95}));
  connect(currentGear.y, transmissionControlBus.currentGear) annotation(
    Line(points = {{-41, 90}, {-75, 90}, {-75, 80}}, color = {255, 127, 0}));
  w_out = outputSpeed.w;

  connect(controlBus.transmissionBus, transmissionBus) annotation(
    Line(points = {{-100, 60}, {-50, 60}}, color = {255, 204, 51}, thickness = 0.5));
  connect(engineFlange.flange, gear.inputFlange) annotation(
    Line(points = {{-100, 0}, {-24, 0}}, color = {135, 135, 135}, thickness = 0.5));
  connect(gear.outputFlange, drivelineFlange.flange) annotation(
    Line(points = {{24, 0}, {100, 0}}, color = {135, 135, 135}, thickness = 0.5));
  connect(outputSpeed.flange, gear.outputFlange) annotation(
    Line(points = {{68, 48}, {80, 48}, {80, 0}, {24, 0}}, color = {0, 0, 0}));
  connect(outputSpeed.w, transmissionBus.outputSpeed) annotation(
    Line(points = {{47, 48}, {45, 48}, {45, 60}, {-50, 60}}, color = {0, 0, 127}));

  annotation(Documentation(info = "<html>
<p>
Model <code>FixedRatioTransmission</code> is the BobLib fixed-ratio
transmission adapter for the neutral VehicleInterfaces transmission contract.
</p>
<p>Enabling either MultiBody connection exposes the VI transmission mount and
connects shaft bearing frames to its rigid housing. Mounting1D carries gear
reaction torque about shaftAxis; world.driveTrainMechanics3D must be true.
Input and output axes are parallel. Housing mass/inertia remain owned by the
vehicle mass record; this adapter adds no spatial body.</p>
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
