within BobLibTest.Regression;

model WheelAlignmentUnitsSmoke

  import Modelica.Mechanics.MultiBody.Frames;

  parameter
    BobLib.Records.VehicleRecord.Chassis.Suspension.Templates.Tire.Templates.PartialWheelRecord
    pWheel(
      R0 = 0.3,
      rimR0 = 0.2,
      rimWidth = 0.2,
      staticAlpha = 2,
      staticGamma = 3);

  output Real componentX[3] =
    Frames.resolve1(attitude.frame_b.R, {1, 0, 0});
  output Real componentY[3] =
    Frames.resolve1(attitude.frame_b.R, {0, 1, 0});
  output Real componentZ[3] =
    Frames.resolve1(attitude.frame_b.R, {0, 0, 1});
  output Real referenceX[3] = Frames.resolve1(referenceR, {1, 0, 0});
  output Real referenceY[3] = Frames.resolve1(referenceR, {0, 1, 0});
  output Real referenceZ[3] = Frames.resolve1(referenceR, {0, 0, 1});
  output Real maxAxisError = max({
    max(abs(componentX - referenceX)),
    max(abs(componentY - referenceY)),
    max(abs(componentZ - referenceZ))});

protected
  inner Modelica.Mechanics.MultiBody.World world(
    enableAnimation = false,
    gravityType = Modelica.Mechanics.MultiBody.Types.GravityTypes.NoGravity);

  Modelica.Mechanics.MultiBody.Parts.Fixed fixedReference(
    animation = false);

  Modelica.Mechanics.MultiBody.Parts.FixedRotation attitude(
    rotationType = Modelica.Mechanics.MultiBody.Types.RotationTypes.PlanarRotationSequence,
    sequence = {1, 2, 3},
    angles = {pWheel.staticGamma, 0, pWheel.staticAlpha},
    animation = false);

  parameter Frames.Orientation referenceR = Frames.axesRotations(
    {1, 2, 3},
    {
      Modelica.Units.Conversions.from_deg(pWheel.staticGamma),
      0,
      Modelica.Units.Conversions.from_deg(pWheel.staticAlpha)
    },
    {0, 0, 0});

equation
  connect(fixedReference.frame_b, attitude.frame_a);

  assert(
    maxAxisError < 1e-12,
    "Degree-typed wheel alignment must match explicit radian conversion");

  annotation(
    experiment(StartTime = 0, StopTime = 0.001, Tolerance = 1e-6, Interval = 0.001));
end WheelAlignmentUnitsSmoke;
