within BobLib.Experiments.Standards.BaseClasses;

partial model PartialKnC "Shared single-axle kinematics and compliance rig"

  import Modelica.Units.SI;
  import Modelica.Mechanics.MultiBody.Frames;
  import BobLib.Chassis.Suspension.Tires;
  import BobLib.Utilities.Math.Vector;

  extends BobLib.Icons.SimulationIcon;
  parameter Boolean steerable = true "Enable steering input; otherwise retain the linkage at its locked rack position" annotation(choices(checkBox = true), Evaluate = true, Dialog(group = "Architecture", enable = false));
  parameter Boolean useRearAxle = false "Select rear calibration from the vehicle record" annotation(choices(checkBox = true), Evaluate = true, Dialog(group = "Axle"));
  replaceable record VehicleRecord = BobLib.Records.VehicleDefn.EVBatInvMotDiff_DWBCStabar_DWBCStabarRecord constrainedby BobLib.Records.VehicleDefn.EVBatInvMotDiff_DWBCStabar_DWBCStabarRecord annotation(choicesAllMatching = true);
  parameter VehicleRecord pVehicle "Complete vehicle calibration record; subsystem records are passed to the selected models" annotation(Dialog(group = "Calibration"));
  replaceable model AxleModel = BobLib.Chassis.Suspension.DoubleWishboneBellcrankStabar(pAxle(stabilizer(leftArmEnd = if useRearAxle then pVehicle.chassis.rearAxle.stabilizer.leftArmEnd else pVehicle.chassis.frontAxle.stabilizer.leftArmEnd,
      leftBarEnd = if useRearAxle then pVehicle.chassis.rearAxle.stabilizer.leftBarEnd
        else pVehicle.chassis.frontAxle.stabilizer.leftBarEnd))) constrainedby BobLib.Chassis.Suspension.BaseClasses.PartialDoubleWishboneAxle annotation(choicesAllMatching = true, Dialog(group = "Axle"));
  parameter BobLib.Brakes.Types.Placement brakePlacement = BobLib.Brakes.Types.Placement.Outboard "Brake location used to close the axle rig torque and support reactions" annotation(Evaluate = true, Dialog(group = "Brakes"));
  parameter SI.Position lockedRackPosition = 0 "Rack displacement held when steering is disabled, measured from its neutral position" annotation(Evaluate = false, Dialog(group = "Steering", enable = not steerable));
  parameter Integer angleCount(
    min = 1,
    max = 7) = 7 "Number of active steering-angle sweeps, using the first entries of steeringAngles (1 to 7)" annotation(Evaluate = false, Dialog(group = "Steering", enable = steerable));
  parameter SI.Angle steeringAngles[7] = {-90, -60, -30, 0, 30, 60, 90}*Modelica.Constants.pi/180 "Steering pinion angles for the seven available sweep slots, in radians" annotation(Evaluate = false, Dialog(group = "Steering", enable = steerable));
  parameter SI.Length heaveMagnitude = 1.5*0.0254 "Amplitude scaling the normalized vertical-motion protocol" annotation(Evaluate = false, Dialog(group = "Protocol"));
  parameter SI.Angle rollMagnitude = 1.25*Modelica.Constants.pi/180 "Amplitude scaling the normalized roll-motion protocol, in radians" annotation(Evaluate = false, Dialog(group = "Protocol"));
  parameter SI.Force forceMagnitude = 1000 "Force amplitude multiplying the normalized contact-load protocol" annotation(Evaluate = false, Dialog(group = "Protocol"));
  parameter SI.RotationalSpringConstant stabilizerRate = 0 "Zero disconnects the stabilizer during the test" annotation(Evaluate = false, Dialog(group = "Protocol"));
  parameter Real brakeInputRatio(min = Modelica.Constants.small) = 1 "Differential-input speed divided by wheel speed for the rig brake reaction" annotation(Evaluate = false, Dialog(group = "Brakes", enable = brakePlacement==BobLib.Brakes.Types.Placement.DifferentialInput));
  inner parameter Boolean headless = false "Disable MultiBody animation geometry for simulation without visualization" annotation(choices(checkBox = true), Evaluate = true, Dialog(tab = "Animation"));
  inner parameter SI.Length linkDiameter = 0.020 "Diameter of links in the MultiBody animation; does not set structural stiffness" annotation(Dialog(tab = "Animation"));
  inner parameter SI.Length jointDiameter = 0.030 "Diameter of joints in the MultiBody animation; does not set joint geometry" annotation(Dialog(tab = "Animation"));
  final parameter SI.Time duration = if steerable then 119*(angleCount-1)+118
    else 118 "Total protocol duration including transitions between active steering angles";
  final parameter SI.Position cpLeft[3] = axle.pAxle.geometry.wheelCenter+Frames.resolve1(
    Frames.axesRotations(
      {1, 2, 3},
      {Modelica.Units.Conversions.from_deg(axle.pAxle.tire.wheel.geometry.staticGamma), 0, Modelica.Units.Conversions.from_deg(axle.pAxle.tire.wheel.geometry.staticAlpha)},
      {0, 0, 0}),
    {0, 0, -axle.pAxle.tire.wheel.geometry.R0}) "Initial left contact-patch position calculated from wheel geometry and alignment";

  AxleModel axle(pAxle(actuation = if useRearAxle then pVehicle.chassis.rearAxle.actuation else pVehicle.chassis.frontAxle.actuation, steering = if useRearAxle then pVehicle.chassis.rearAxle.steering else pVehicle.chassis.frontAxle.steering, tire = if useRearAxle then pVehicle.chassis.rearAxle.tire else pVehicle.chassis.frontAxle.tire, geometry = if useRearAxle then pVehicle.chassis.rearAxle.geometry else pVehicle.chassis.frontAxle.geometry, masses = if useRearAxle then pVehicle.chassis.rearAxle.masses else pVehicle.chassis.frontAxle.masses, stabilizer(barRate = stabilizerRate)),
final includeRotorEffects = false,
final quasiStatic = true,
final steerable = steerable,
lockedRackPosition = lockedRackPosition,
final exposeWheelBearings = true,
redeclare Tires.BaseTire leftTire(
      redeclare model ComplianceModel = Tires.Wheels.WheelCompliance,
      redeclare Tires.Wheels.Wheel0DOF wheelModel,
      redeclare Tires.MF52.SlipModel.NoSlip slipModel),
redeclare Tires.BaseTire rightTire(
      redeclare model ComplianceModel = Tires.Wheels.WheelCompliance,
      redeclare Tires.Wheels.Wheel0DOF wheelModel,
      redeclare Tires.MF52.SlipModel.NoSlip slipModel)) annotation(Placement(transformation(origin = {0, 40}, extent = {{-72, -32}, {72, 32}})));

  KnCInputs inputs(
    steerable = steerable,
    angleCount = angleCount,
    steeringAngles = steeringAngles,
    heaveMagnitude = heaveMagnitude,
    rollMagnitude = rollMagnitude) annotation(Placement(transformation(origin = {-170, 70}, extent = {{-25, -15}, {25, 15}})));

  Modelica.Mechanics.Rotational.Sources.Position steerPosition(exact = true) if steerable annotation(Placement(visible = steerable, transformation(origin = {-60, 115}, extent = {{-12, -12}, {12, 12}})));

  BobLib.Utilities.Mechanics.MultiBody.Actuators.ChassisActuator chassisActuator(axleRef = axle.effectiveCenter) annotation(Placement(transformation(origin = {0, -65}, extent = {{-20, -20}, {20, 20}})));

  BobLib.Utilities.Mechanics.MultiBody.Actuators.ContactPatchFixture leftFixture(CP_init = cpLeft) annotation(Placement(transformation(origin = {-90, -30}, extent = {{-20, -20}, {20, 20}})));

  BobLib.Utilities.Mechanics.MultiBody.Actuators.ContactPatchFixture rightFixture(CP_init = Vector.mirrorXZ(cpLeft)) annotation(Placement(transformation(origin = {90, -30}, extent = {{-20, -20}, {20, 20}})));

  BobLib.Utilities.Mechanics.MultiBody.Actuators.ContactPatchForceActuator leftLoad(
    fxTable = cat(1, KnCInputs.fxTable, [119, 0]),
    fyTable = cat(1, KnCInputs.fyTable, [119, 0]),
    forceMagnitude = forceMagnitude/2,
    leftFxSource(extrapolation = Modelica.Blocks.Types.Extrapolation.Periodic),
    leftFySource(extrapolation = Modelica.Blocks.Types.Extrapolation.Periodic)) annotation(Placement(transformation(origin = {-110, 16}, extent = {{-18, -18}, {18, 18}})));

  BobLib.Utilities.Mechanics.MultiBody.Actuators.ContactPatchForceActuator rightLoad(
    fxTable = cat(1, KnCInputs.fxTable, [119, 0]),
    fyTable = cat(1, KnCInputs.fyTable, [119, 0]),
    forceMagnitude = forceMagnitude/2,
    leftFxSource(extrapolation = Modelica.Blocks.Types.Extrapolation.Periodic),
    leftFySource(extrapolation = Modelica.Blocks.Types.Extrapolation.Periodic)) annotation(Placement(transformation(origin = {110, 16}, extent = {{18, -18}, {-18, 18}})));

  BobLib.Brakes.Internal.RigAxleBrake brakes(
    useWheelFrames = true,
    placement = brakePlacement,
    inputRatio = brakeInputRatio,
    leftContactTorque = axle.leftContactTorque,
    rightContactTorque = axle.rightContactTorque) annotation(Placement(transformation(origin = {0, 100}, extent = {{-24, -12}, {24, 12}})));

  inner VehicleInterfaces.Roads.FlatRoad road(final animation = false, mu = 1)
    "Flat fixture plane used by tire road queries" annotation(
    Placement(transformation(origin = {-170, -130}, extent = {{-20, -10}, {20, 10}})));

  inner Modelica.Mechanics.MultiBody.World world(
    final driveTrainMechanics3D = true,
    n = {0, 0, -1},
    g = 0,
    enableAnimation = not headless) annotation(Placement(transformation(origin = {-210, -130}, extent = {{-10, -10}, {10, 10}})));
  BobLib.Records.StandardRecord.FourPostEvalRecord knc;

protected

  // Front quantities
  Real frLeftDeltaVec[3];
  Real frLeftKingpinVec[3];
  Real frLeftGroundParam;
  Real frLeftGroundPoint[3];

  Real frRightDeltaVec[3];
  Real frRightKingpinVec[3];
  Real frRightGroundParam;
  Real frRightGroundPoint[3];

equation
  connect(axle.axleFrame, chassisActuator.chassisFrame) annotation(Line(points = {{0, 36}, {0, -49}}, color = {95, 95, 95}));
  connect(leftFixture.frame_a, axle.leftCP) annotation(Line(points = {{-90, -22}, {-90, 16}, {-72, 16}}, color = {95, 95, 95}));
  connect(rightFixture.frame_a, axle.rightCP) annotation(Line(points = {{90, -22}, {90, 16}, {72, 16}}, color = {95, 95, 95}));
  connect(leftLoad.chassisFrame, axle.leftCP) annotation(Line(points = {{-92, 16}, {-72, 16}}, color = {95, 95, 95}));
  connect(rightLoad.chassisFrame, axle.rightCP) annotation(Line(points = {{92, 16}, {72, 16}}, color = {95, 95, 95}));
  connect(
    inputs.steer,
    steerPosition.phi_ref) annotation(Line(visible = steerable, points = {{-145, 80}, {-135, 80}, {-135, 115}, {-74.4, 115}}, color = {0, 0, 127}));
  connect(
    steerPosition.flange,
    axle.steerFlange) annotation(Line(visible = steerable, points = {{-48, 115}, {-40, 115}, {-40, 62}, {0, 62}, {0, 56}}));
  connect(inputs.heave, chassisActuator.heaveInput) annotation(Line(points = {{-145, 70}, {-140, 70}, {-140, -53}, {-7.2, -53}}, color = {0, 0, 127}));
  connect(inputs.roll, chassisActuator.rollInput) annotation(Line(points = {{-145, 60}, {-145, -77}, {-7.2, -77}}, color = {0, 0, 127}));

  connect(brakes.left, axle.leftTorque) annotation(Line(points = {{-24, 100}, {-95, 100}, {-95, 36}, {-72, 36}}));
  connect(brakes.right, axle.rightTorque) annotation(Line(points = {{24, 100}, {95, 100}, {95, 36}, {72, 36}}));
  connect(
    brakes.leftWheelFrame,
    axle.leftBearing) annotation(Line(visible = brakes.useWheelFrames, points = {{-24, 90.4}, {-82, 90.4}, {-82, 52}, {-72, 52}}, color = {95, 95, 95}));
  connect(
    brakes.rightWheelFrame,
    axle.rightBearing) annotation(Line(visible = brakes.useWheelFrames, points = {{24, 90.4}, {82, 90.4}, {82, 52}, {72, 52}}, color = {95, 95, 95}));
  connect(
    brakes.sprungFrame,
    axle.axleFrame) annotation(Line(visible = brakePlacement<>BobLib.Brakes.Types.Placement.Outboard, points = {{0, 88}, {65, 88}, {65, -8}, {0, -8}, {0, 36}}, color = {95, 95, 95}));

  when time>=duration then
    terminate("Kinematics and compliance sweep completed");
  end when;
  knc.leftGamma = axle.leftTire.gamma;

  frLeftDeltaVec = Frames.resolve1(axle.leftCP.R, {1, 0, 0});
  knc.leftToe = atan(frLeftDeltaVec[2]/frLeftDeltaVec[1]);

  frLeftKingpinVec = axle.leftWishboneUprightLoop.upperFrame_o.r_0 - axle.leftWishboneUprightLoop.lowerFrame_o.r_0;
  knc.leftCaster = atan(-1*frLeftKingpinVec[1]/frLeftKingpinVec[3]);
  knc.leftKpi = atan(-1*frLeftKingpinVec[2]/frLeftKingpinVec[3]);

  frLeftGroundParam = (axle.leftCP.r_0[3] - axle.leftWishboneUprightLoop.upperFrame_o.r_0[3])/frLeftKingpinVec[3];
  frLeftGroundPoint = axle.leftWishboneUprightLoop.upperFrame_o.r_0 + frLeftGroundParam*frLeftKingpinVec;
  knc.leftMechTrail = frLeftGroundPoint[1] - axle.leftCP.r_0[1];
  knc.leftMechScrub = axle.leftCP.r_0[2] - frLeftGroundPoint[2];

  knc.rightGamma = axle.rightTire.gamma;

  frRightDeltaVec = Frames.resolve1(axle.rightCP.R, {1, 0, 0});
  knc.rightToe = atan(frRightDeltaVec[2]/frRightDeltaVec[1]);

  frRightKingpinVec = axle.rightWishboneUprightLoop.upperFrame_o.r_0 - axle.rightWishboneUprightLoop.lowerFrame_o.r_0;
  knc.rightCaster = atan(-1*frRightKingpinVec[1]/frRightKingpinVec[3]);
  knc.rightKpi = atan(frRightKingpinVec[2]/frRightKingpinVec[3]);

  frRightGroundParam = (axle.rightCP.r_0[3] - axle.rightWishboneUprightLoop.upperFrame_o.r_0[3])/frRightKingpinVec[3];
  frRightGroundPoint = axle.rightWishboneUprightLoop.upperFrame_o.r_0 + frRightGroundParam*frRightKingpinVec;
  knc.rightMechTrail = frRightGroundPoint[1] - axle.rightCP.r_0[1];
  knc.rightMechScrub = frRightGroundPoint[2] - axle.rightCP.r_0[2];

  knc.leftSpringLength = axle.leftSpringLength;
  knc.leftFz = axle.leftTire.Fz;
  knc.rightSpringLength = axle.rightSpringLength;
  knc.rightFz = axle.rightTire.Fz;
  knc.stabarAngle = axle.stabarAngle;

  knc.jackingForce = chassisActuator.jackingOutput;
  knc.heave = chassisActuator.heaveInput;
  knc.roll = chassisActuator.rollInput;
  knc.fx = leftLoad.worldForce.force[1] + rightLoad.worldForce.force[1];
  knc.fy = leftLoad.worldForce.force[2] + rightLoad.worldForce.force[2];

  annotation(Diagram(coordinateSystem(extent = {{-220, -140}, {220, 140}}, preserveAspectRatio = true, grid = {2, 2})), Documentation(info = "<html><p>Shared one-axle rig. Select AxleModel and a compatible VehicleRecord, then select front or rear calibration with useRearAxle. KnCSteerable and KnCLocked fix the mechanical steering mode. Existing direct, bellcrank and stabilizer double-wishbone architectures are supported; the geometric measurements require their shared double-wishbone interface.</p><p>For vehicle records with different front/rear spring-actuation record types, bind the selected record explicitly: axle(pAxle(actuation=pVehicle.chassis.frontAxle.actuation)) or axle(pAxle(actuation=pVehicle.chassis.rearAxle.actuation)). This avoids a conditional expression between incompatible record types. If redeclaring a stabilizer axle, also supply its pAxle.stabilizer geometry in the redeclaration.</p><p>Each test takes 118 simulated seconds. Steered tests repeat with one-second neutral steering transitions, stopping after angleCount sweeps. Angles are pinion angles, not road-wheel angles. The rack remains installed when locked. Calibration is set before initialization; structural redeclarations require rebuilding.</p><p>Motion uses monotone cubic interpolation with valid acceleration derivatives. Stabilizer stiffness defaults to zero, matching the legacy rig protocol; set stabilizerRate to include the bar. Brake placement selects upright, inboard or differential-input reaction paths.</p><h4>Parameter setup</h4><p><code>pVehicle</code> supplies physical calibration; <code>useRearAxle</code> selects the front or rear record. Choose the replaceable axle separately. The steerable rig uses the first <code>angleCount</code> entries of the fixed seven-element <code>steeringAngles</code> array. Angles are radians. Each angle receives the same 118-second protocol, with one second between cycles. The locked rig runs one cycle at <code>lockedRackPosition</code> in metres. Heave, roll and force amplitudes scale the normalized protocol; forceMagnitude is divided equally between the two contact patches. Brake placement changes load reactions and is an architecture choice. A nonzero stabilizerRate overrides the default disabled rig stabilizer.</p>
</html>"));
end PartialKnC;
