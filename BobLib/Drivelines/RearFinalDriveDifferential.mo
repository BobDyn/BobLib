within BobLib.Drivelines;

model RearFinalDriveDifferential

  "Rear final-drive, differential, and halfshaft driveline"

  import Modelica.Units.SI;

  parameter BobLib.Records.VehicleRecord.Drivelines.RearFinalDriveDifferentialRecord pDriveline(inputRatio = 3.31) "Reduction, differential and shared halfshaft calibration" annotation(Dialog(group = "Calibration"));

  extends VehicleInterfaces.Icons.Driveline;

  extends BobLib.Drivelines.Interfaces.Base(
    final driveLayout = BobLib.Drivelines.Types.DriveLayout.Rear,
    includeMount = usingMultiBodyTransmission or usingMultiBodyChassis);

  parameter Boolean includeRotorEffects = false
    "Include gyroscopic and carrier-acceleration effects when mounted" annotation(Dialog(group = "Architecture"), choices(checkBox = true), Evaluate = true);

  VehicleInterfaces.PowertrainMounts.SingleSystemRigidMount mount(
    r_ChassisToMount1 = pDriveline.mountOffset) if includeMount annotation(
    Placement(visible = includeMount,
      transformation(origin = {-140, -70}, extent = {{-10, -10}, {10, 10}})));

  parameter BobLib.Drivelines.Types.DifferentialArchitecture differentialArchitecture =
    BobLib.Drivelines.Types.DifferentialArchitecture.ClutchLSD
    "Differential architecture" annotation(Dialog(group = "Architecture"));
  parameter Modelica.Mechanics.MultiBody.Types.Axis shaftAxis = {0, 1, 0}
    "Parallel rotor and gear axes resolved in drivelineMount" annotation(Dialog(group = "Operating settings"), Evaluate = true);
  parameter SI.AngularVelocity initialOutputAngularVelocity = 0
    "Initial differential output and rear wheel angular speed" annotation(Dialog(group = "Initialization"));

  Modelica.Mechanics.Rotational.Components.IdealGear finalDrive(
    ratio = pDriveline.inputRatio,
    useSupport = usingMultiBodyTransmission) annotation(
      Placement(transformation(origin = {-70.4, 0}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.Rotational.Components.Inertia diffInputRotor(
    J = pDriveline.inputInertia,
    w(
      start = initialOutputAngularVelocity,
      fixed = true)) if not (usingMultiBodyTransmission and includeRotorEffects) annotation(
      Placement(
        visible = not (usingMultiBodyTransmission and includeRotorEffects),
        transformation(origin = {-28.8, 0}, extent = {{-8, -8}, {8, 8}})));

  Modelica.Mechanics.MultiBody.Parts.Rotor1D diffInputRotor3D(
    n = shaftAxis, animation = false,
    J = pDriveline.inputInertia,
    w(
      start = initialOutputAngularVelocity,
      fixed = true)) if usingMultiBodyTransmission and includeRotorEffects annotation(
      Placement(
        visible = usingMultiBodyTransmission and includeRotorEffects,
        transformation(origin = {-28.8, 0}, extent = {{-8, -8}, {8, 8}})));

  BobLib.Drivelines.Internal.Differential1D differential(
    pDifferential = pDriveline.differential,
    w_l(start = initialOutputAngularVelocity),
    w_r(start = initialOutputAngularVelocity),
    differentialArchitecture = differentialArchitecture) if not diff_lockedKinematics annotation(
      Placement(
        visible = not diff_lockedKinematics,
        transformation(origin = {32, 0}, extent = {{10, -10}, {-10, 10}}, rotation = -180)));

  BobLib.Drivelines.Internal.LockedDifferential1D lockedDifferential
    if diff_lockedKinematics annotation(
      Placement(
        visible = diff_lockedKinematics,
        transformation(origin = {32, 0}, extent = {{10, -10}, {-10, 10}}, rotation = -180)));

  replaceable model HalfshaftModel = Modelica.Mechanics.Rotational.Components.SpringDamper(
    c = pDriveline.halfshaft.stiffness, d = pDriveline.halfshaft.damping, stateSelect = StateSelect.never)
    constrainedby Modelica.Mechanics.Rotational.Interfaces.PartialCompliant
    "Compliance law used on both halfshafts" annotation(choicesAllMatching = true);

  BobLib.Drivelines.Internal.Halfshaft leftHalfshaft(
    final includeRotorEffects = includeRotorEffects,
    pHalfshaft = pDriveline.halfshaft,
    useMount = usingMultiBodyTransmission,
    useWheelFrame = usingMultiBodyChassis,
    useInboardBrake = useRearInboardBrakes,
    shaftAxis = shaftAxis,
    initialAngularVelocity = initialOutputAngularVelocity,
    fixInitialSpeed = not diff_lockedKinematics,
    redeclare model ComplianceModel = HalfshaftModel) annotation(
      Placement(transformation(origin = {96, -55}, extent = {{-12, -12}, {12, 12}}, rotation = -90)));

  BobLib.Drivelines.Internal.Halfshaft rightHalfshaft(
    final includeRotorEffects = includeRotorEffects,
    pHalfshaft = pDriveline.halfshaft,
    useMount = usingMultiBodyTransmission,
    useWheelFrame = usingMultiBodyChassis,
    useInboardBrake = useRearInboardBrakes,
    shaftAxis = shaftAxis,
    initialAngularVelocity = initialOutputAngularVelocity,
    fixInitialSpeed = false,
    redeclare model ComplianceModel = HalfshaftModel) annotation(
      Placement(transformation(origin = {96, 55}, extent = {{-12, -12}, {12, 12}}, rotation = 90)));

  BobLib.Drivelines.Internal.Halfshaft frontLeftBrakeShaft(
    final includeRotorEffects = includeRotorEffects,
    pHalfshaft = pDriveline.frontBrakeHalfshaft,
    useMount = usingMultiBodyTransmission, useWheelFrame = usingMultiBodyChassis,
    useInboardBrake = true, shaftAxis = shaftAxis,
    initialAngularVelocity = initialFrontAngularVelocity, fixInitialSpeed = true) if useFrontInboardBrakes annotation(
    Placement(
      visible = useFrontInboardBrakes,
      transformation(origin = {-96, -55}, extent = {{-12, -12}, {12, 12}}, rotation = -90)));

  BobLib.Drivelines.Internal.Halfshaft frontRightBrakeShaft(
    final includeRotorEffects = includeRotorEffects,
    pHalfshaft = pDriveline.frontBrakeHalfshaft,
    useMount = usingMultiBodyTransmission, useWheelFrame = usingMultiBodyChassis,
    useInboardBrake = true, shaftAxis = shaftAxis,
    initialAngularVelocity = initialFrontAngularVelocity, fixInitialSpeed = true) if useFrontInboardBrakes annotation(
    Placement(
      visible = useFrontInboardBrakes,
      transformation(origin = {-96, 35}, extent = {{-12, -12}, {12, 12}}, rotation = 90)));
  parameter SI.AngularVelocity initialFrontAngularVelocity = 0 "Initial speed of optional undriven front brake shafts" annotation(Dialog(group = "Initialization"));

  output SI.AngularVelocity motorSideSpeed
    "Input shaft speed at the motor side of the final drive";
  output SI.AngularVelocity diffInputSpeed
    "Differential input speed";
  output SI.Torque diffLockTorque
    "Differential locking torque";
  output SI.Torque leftHalfshaftTorque
    "Left halfshaft transmitted torque";
  output SI.Torque rightHalfshaftTorque
    "Right halfshaft transmitted torque";

protected
  outer Modelica.Mechanics.MultiBody.World world;

  Modelica.Mechanics.MultiBody.Parts.Mounting1D housing(n = Modelica.Math.Vectors.normalizeWithAssert(shaftAxis)) if usingMultiBodyTransmission annotation(
    Placement(
      visible = usingMultiBodyTransmission,
      transformation(origin = {-80, -80}, extent = {{-10, -10}, {10, 10}})));
  final parameter Boolean diff_lockedKinematics = differentialArchitecture == BobLib.Drivelines.Types.DifferentialArchitecture.Locked "Use the locked differential kinematic constraint instead of a torque-bias law";
  BobLib.Drivelines.Interfaces.DrivelineBus drivelineBus annotation(
    Placement(
      transformation(origin = {-128, 70}, extent = {{-15, -15}, {15, 15}}),
      iconTransformation(extent = {{-90, 10}, {-70, 30}})));

  Modelica.Blocks.Sources.RealExpression inputSpeedSignal(
    y = motorSideSpeed) "Motor-side speed published to VehicleInterfaces driveline bus" annotation(
      Placement(transformation(origin = {-44.8, 72}, extent = {{17, -17}, {-17, 17}})));

  Modelica.Blocks.Sources.RealExpression diffSpeedSignal(
    y = diffInputSpeed) "Differential input speed published to VehicleInterfaces driveline bus" annotation(
      Placement(transformation(origin = {32, 72}, extent = {{17, -17}, {-17, 17}})));

  Modelica.Blocks.Sources.RealExpression leftTorqueSignal(
    y = leftHalfshaftTorque) "Left halfshaft torque published to VehicleInterfaces driveline bus" annotation(
      Placement(transformation(origin = {-44.8, 32}, extent = {{17, -17}, {-17, 17}})));

  Modelica.Blocks.Sources.RealExpression rightTorqueSignal(
    y = rightHalfshaftTorque) "Right halfshaft torque published to VehicleInterfaces driveline bus" annotation(
      Placement(transformation(origin = {32, 32}, extent = {{17, -17}, {-17, 17}})));

initial equation
  assert(pDriveline.inputRatio > 0, "RearFinalDriveDifferential: pDriveline.inputRatio must be positive");
  assert(pDriveline.inputInertia > 0,
    "RearFinalDriveDifferential: input inertia must be positive");

equation
  connect(chassisFrame, mount.chassisFrame) annotation(
    Line(visible = includeMount,
      points = {{-140, -100}, {-140, -80}}, color = {95, 95, 95}));
  connect(mount.mount_1, drivelineMount) annotation(
    Line(visible = includeMount,
      points = {{-140, -60}, {-140, -40}, {-120, -40}, {-120, -90}, {0, -90}, {0, -100}}, color = {95, 95, 95}));
  connect(differentialBrakeShaft.bearingFrame, drivelineMount) annotation(
    Line(
      visible = useDifferentialInputBrake and usingMultiBodyTransmission,
      points = {{-160, -60}, {-150, -60}, {-150, -40}, {-120, -40}, {-120, -90}, {0, -90}, {0, -100}},
      color = {95, 95, 95}));
  assert(not usingMultiBodyTransmission or world.driveTrainMechanics3D,
    "Mounted driveline requires world.driveTrainMechanics3D=true");
  connect(finalDrive.support, housing.flange_b) annotation(
    Line(
      visible = usingMultiBodyTransmission,
      points = {{-70.4, -10}, {-70.4, -20}, {-70, -20}, {-70, -80}},
      color = {0, 0, 0}));
  connect(housing.frame_a, drivelineMount) annotation(
    Line(visible = usingMultiBodyTransmission, points = {{-80, -90}, {0, -90}, {0, -100}}, color = {95, 95, 95}));
  connect(transmissionFlange.bearingFrame, housing.frame_a) annotation(
    Line(
      visible = usingMultiBodyTransmission,
      points = {{-160, 0}, {-150, 0}, {-150, -40}, {-120, -40}, {-120, -90}, {-80, -90}},
      color = {95, 95, 95}));
  connect(diffInputRotor3D.frame_a, drivelineMount) annotation(
    Line(
      visible = usingMultiBodyTransmission and includeRotorEffects,
      points = {{-28.8, -8}, {-28.8, -90}, {0, -90}, {0, -100}},
      color = {95, 95, 95}));
  connect(
    leftHalfshaft.mount,
    drivelineMount) annotation(Line(visible = usingMultiBodyTransmission or usingMultiBodyChassis, points = {{84, -55}, {84, -90}, {0, -90}, {0, -100}}, color = {95, 95, 95}));
  connect(
    rightHalfshaft.mount,
    leftHalfshaft.mount) annotation(Line(visible = usingMultiBodyTransmission or usingMultiBodyChassis, points = {{108, 55}, {140.8, 55}, {140.8, -85}, {84, -85}, {84, -55}}, color = {95, 95, 95}));
  connect(
    inboardBrake_1.flange,
    frontLeftBrakeShaft.brake) annotation(Line(visible = useFrontInboardBrakes, points = {{-40, -100}, {-40, -75}, {-64, -75}, {-64, -55}, {-84, -55}}, color = {0, 0, 0}));
  connect(
    inboardBrake_1.bearingFrame,
    drivelineMount) annotation(Line(visible = useFrontInboardBrakes and usingMultiBodyTransmission, points = {{-40, -100}, {-121.6, -100}, {-121.6, -94}, {0, -94}, {0, -100}}, color = {95, 95, 95}));
  connect(
    frontLeftBrakeShaft.wheelFrame,
    wheelHub_1.bearingFrame) annotation(Line(visible = useFrontInboardBrakes and usingMultiBodyChassis, points = {{-104.4, -67}, {-120, -67}, {-120, -100}, {-96, -100}}, color = {95, 95, 95}));
  connect(
    frontLeftBrakeShaft.mount,
    drivelineMount) annotation(Line(visible = useFrontInboardBrakes and (usingMultiBodyTransmission or usingMultiBodyChassis), points = {{-108, -55}, {-124.8, -55}, {-124.8, -90}, {0, -90}, {0, -100}}, color = {95, 95, 95}));
  connect(
    frontLeftBrakeShaft.flange_b,
    wheelHub_1.flange) annotation(Line(visible = useFrontInboardBrakes, points = {{-96, -67}, {-96, -100}}, color = {0, 0, 0}));
  connect(
    inboardBrake_2.flange,
    frontRightBrakeShaft.brake) annotation(Line(visible = useFrontInboardBrakes, points = {{-160, 80}, {-140.8, 80}, {-140.8, 35}, {-108, 35}}, color = {0, 0, 0}));
  connect(
    inboardBrake_2.bearingFrame,
    drivelineMount) annotation(Line(visible = useFrontInboardBrakes and usingMultiBodyTransmission, points = {{-160, 80}, {-147.2, 80}, {-147.2, -40}, {-120, -40}, {-120, -90}, {0, -90}, {0, -100}}, color = {95, 95, 95}));
  connect(
    frontRightBrakeShaft.wheelFrame,
    wheelHub_2.bearingFrame) annotation(Line(visible = useFrontInboardBrakes and usingMultiBodyChassis, points = {{-87.6, 47}, {-87.6, 52}, {-80, 52}, {-80, 100}, {-96, 100}}, color = {95, 95, 95}));
  connect(
    frontRightBrakeShaft.mount,
    drivelineMount) annotation(Line(visible = useFrontInboardBrakes and (usingMultiBodyTransmission or usingMultiBodyChassis), points = {{-84, 35}, {-73.6, 35}, {-73.6, 18}, {-3.2, 18}, {-3.2, -90}, {0, -90}, {0, -100}}, color = {95, 95, 95}));
  connect(
    frontRightBrakeShaft.flange_b,
    wheelHub_2.flange) annotation(Line(visible = useFrontInboardBrakes, points = {{-96, 47}, {-96, 100}}, color = {0, 0, 0}));
  connect(
    inboardBrake_3.flange,
    leftHalfshaft.brake) annotation(Line(visible = useRearInboardBrakes, points = {{160, -30}, {128, -30}, {128, -55}, {108, -55}}, color = {0, 0, 0}));
  connect(
    inboardBrake_3.bearingFrame,
    drivelineMount) annotation(Line(visible = useRearInboardBrakes and usingMultiBodyTransmission, points = {{160, -30}, {150.4, -30}, {150.4, -94}, {0, -94}, {0, -100}}, color = {95, 95, 95}));
  connect(
    leftHalfshaft.wheelFrame,
    wheelHub_3.bearingFrame) annotation(Line(visible = usingMultiBodyChassis, points = {{87.6, -67}, {72, -67}, {72, -100}, {96, -100}}, color = {95, 95, 95}));
  connect(
    inboardBrake_4.flange,
    rightHalfshaft.brake) annotation(Line(visible = useRearInboardBrakes, points = {{40, 100}, {40, 95}, {64, 95}, {64, 55}, {84, 55}}, color = {0, 0, 0}));
  connect(
    inboardBrake_4.bearingFrame,
    drivelineMount) annotation(Line(visible = useRearInboardBrakes and usingMultiBodyTransmission, points = {{40, 100}, {150.4, 100}, {150.4, -94}, {0, -94}, {0, -100}}, color = {95, 95, 95}));
  connect(
    rightHalfshaft.wheelFrame,
    wheelHub_4.bearingFrame) annotation(Line(visible = usingMultiBodyChassis, points = {{104.4, 67}, {120, 67}, {120, 100}, {96, 100}}, color = {95, 95, 95}));
  motorSideSpeed = der(transmissionFlange.flange.phi);

  if diff_lockedKinematics then
    diffInputSpeed = lockedDifferential.w_in;
    diffLockTorque = lockedDifferential.T_lock;
  else
    diffInputSpeed = differential.w_in;
    diffLockTorque = differential.T_lock;
  end if;
  leftHalfshaftTorque = leftHalfshaft.tau;
  rightHalfshaftTorque = rightHalfshaft.tau;

  connect(controlBus.drivelineBus, drivelineBus) annotation(
    Line(points = {{-160, 60}, {-160, 70}, {-128, 70}}, color = {255, 204, 51}, thickness = 0.5));
  connect(inputSpeedSignal.y, drivelineBus.motorSideSpeed) annotation(
    Line(points = {{-63.5, 72}, {-88, 72}, {-88, 70}, {-128, 70}}, color = {0, 0, 127}));
  connect(diffSpeedSignal.y, drivelineBus.diffInputSpeed) annotation(
    Line(points = {{13.3, 72}, {-8, 72}, {-8, 54}, {-128, 54}, {-128, 70}}, color = {0, 0, 127}));
  connect(leftTorqueSignal.y, drivelineBus.leftHalfshaftTorque) annotation(
    Line(points = {{-63.5, 32}, {-75.2, 32}, {-75.2, 48}, {-128, 48}, {-128, 70}}, color = {0, 0, 127}));
  connect(rightTorqueSignal.y, drivelineBus.rightHalfshaftTorque) annotation(
    Line(points = {{13.3, 32}, {-8, 32}, {-8, 50}, {-128, 50}, {-128, 70}}, color = {0, 0, 127}));

  connect(transmissionFlange.flange, finalDrive.flange_a) annotation(
    Line(points = {{-160, 0}, {-80.4, 0}}));
  connect(differentialBrakeShaft.flange, finalDrive.flange_a) annotation(
    Line(visible = useDifferentialInputBrake,
      points = {{-160, -60}, {-112, -60}, {-112, 0}, {-80.4, 0}}));
  connect(finalDrive.flange_b, diffInputRotor.flange_a) annotation(
    Line(visible = not (usingMultiBodyTransmission and includeRotorEffects), points = {{-60.4, 0}, {-36.8, 0}}));
  connect(finalDrive.flange_b, diffInputRotor3D.flange_a) annotation(
    Line(visible = usingMultiBodyTransmission and includeRotorEffects, points = {{-60.4, 0}, {-36.8, 0}}));

  if diff_lockedKinematics then
    connect(
      diffInputRotor.flange_b,
      lockedDifferential.shaft_in) annotation(Line(visible = (diff_lockedKinematics) and not (usingMultiBodyTransmission and includeRotorEffects), points = {{-20.8, 0}, {22, 0}}, color = {0, 0, 0}));
  connect(
    diffInputRotor3D.flange_b,
    lockedDifferential.shaft_in) annotation(Line(visible = (diff_lockedKinematics) and usingMultiBodyTransmission and includeRotorEffects, points = {{-20.8, 0}, {22, 0}}, color = {0, 0, 0}));
    connect(
      lockedDifferential.shaft_left,
      leftHalfshaft.flange_a) annotation(Line(visible = diff_lockedKinematics, points = {{42, -4}, {96, -4}, {96, -43}}, color = {0, 0, 0}));
    connect(
      lockedDifferential.shaft_right,
      rightHalfshaft.flange_a) annotation(Line(visible = diff_lockedKinematics, points = {{42, 4}, {96, 4}, {96, 43}}, color = {0, 0, 0}));
  else
    connect(
      diffInputRotor.flange_b,
      differential.shaft_in) annotation(Line(visible = (not diff_lockedKinematics) and not (usingMultiBodyTransmission and includeRotorEffects), points = {{-20.8, 0}, {22, 0}}, color = {0, 0, 0}));
  connect(
    diffInputRotor3D.flange_b,
    differential.shaft_in) annotation(Line(visible = (not diff_lockedKinematics) and usingMultiBodyTransmission and includeRotorEffects, points = {{-20.8, 0}, {22, 0}}, color = {0, 0, 0}));
    connect(
      differential.shaft_left,
      leftHalfshaft.flange_a) annotation(Line(visible = not diff_lockedKinematics, points = {{42, -4}, {96, -4}, {96, -43}}, color = {0, 0, 0}));
    connect(
      differential.shaft_right,
      rightHalfshaft.flange_a) annotation(Line(visible = not diff_lockedKinematics, points = {{42, 4}, {96, 4}, {96, 43}}, color = {0, 0, 0}));
  end if;

  connect(
    leftHalfshaft.flange_b,
    wheelHub_3.flange) annotation(Line(visible = true, points = {{96, -67}, {96, -100}}, color = {0, 0, 0}));
  connect(
    rightHalfshaft.flange_b,
    wheelHub_4.flange) annotation(Line(visible = true, points = {{96, 67}, {96, 100}}, color = {0, 0, 0}));
  annotation(Documentation(info = "<html>
<p>usingMultiBodyTransmission enables the VI mount, gear housing reaction,
independently of rotor fidelity. includeRotorEffects selects Rotor1D
carrier/gyroscopic corrections for the input and halfshaft inertias, replacing
the scalar inertias. Enable world.driveTrainMechanics3D. All modeled axes
are parallel to shaftAxis; no CV-joint geometry or halfshaft bending is implied.
VehicleDefn retains the complete mass and inertia tensor with all rotors
locked; Rotor1D adds relative-spin corrections, not a second body. Wheel outputs retain scalar inboard-shaft
semantics only when usingMultiBodyChassis=false. The internal rigid mount locates drivelineMount relative to chassisFrame using pDriveline.mountOffset. Connect chassisFrame to the sprung chassis; drivelineMount remains the housing frame for attached transmission and caliper reactions. With a MultiBody chassis the halfshafts own carrier motion and reaction transport to the wheel bearing frames and drivelineMount. Inboard brakes attach upstream of shaft compliance; undriven front inboard brakes have independent shafts with free differential-side ends.</p>
<p>
VehicleInterfaces rear driveline containing only the mechanical reduction,
differential, and compliant halfshafts. Battery, inverter, controls, and motor
are intentionally external so vehicle-level experiments can show and replace
each powertrain subsystem independently.
</p>
</html>"));
  annotation(Diagram(coordinateSystem(extent = {{-160, -100}, {160, 100}}, preserveAspectRatio = true)));
end RearFinalDriveDifferential;
