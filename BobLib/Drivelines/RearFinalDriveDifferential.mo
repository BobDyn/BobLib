within BobLib.Drivelines;

model RearFinalDriveDifferential

  "Rear final-drive, differential, and halfshaft driveline"
  extends VehicleInterfaces.Icons.Driveline;

  extends VehicleInterfaces.Drivelines.Interfaces.TwoAxleBase(
    final usingMultiBodyChassis = false,
    final usingMultiBodyTransmission = false,
    final includeMount = false);

  import SI = Modelica.Units.SI;

  parameter Real finalDriveRatio = 3.31
    "Motor speed divided by differential input speed";
  parameter SI.Inertia diffInputRotorJ = 0.04
    "Differential input/ring inertia";
  parameter SI.AngularVelocity initialOutputAngularVelocity = 0
    "Initial differential output and rear wheel angular speed";
  parameter Boolean diff_lockedKinematics = true
    "Use structural spool kinematics instead of open differential kinematics";
  parameter Boolean diff_use_lsd = true
    "Enable differential limited-slip locking torque";
  parameter Real diff_driveSideTorqueSign = 1
    "Sign of differential input torque corresponding to drive-side ramp loading";
  parameter SI.Torque diff_T_preload(min = 0) = 20
    "Differential LSD static preload breakaway torque";
  parameter Real diff_lockFractionAccel(unit = "1", min = 0, max = 1) = 0.35
    "Differential drive-side locking value";
  parameter Real diff_lockFractionDecel(unit = "1", min = 0, max = 1) = 0.15
    "Differential coast-side locking value";
  parameter SI.Torque diff_T_capacity_max(min = 0) = 1000
    "Differential maximum static lock capacity";
  parameter SI.Length diff_clutchEffectiveRadius(min = 1e-6) = 1.0
    "Differential LSD effective clutch torque arm";
  parameter Real diff_kineticFrictionRatio(unit = "1", min = 1e-6, max = 1) = 0.85
    "Differential sliding clutch capacity divided by static capacity";
  parameter SI.AngularVelocity diff_w_transition = 1.0
    "Differential LSD regularized clutch slip transition speed";
  parameter SI.RotationalDampingConstant diff_c_viscous(min = 0) = 0.05
    "Differential small viscous slip damping included in capped lock torque";
  parameter SI.RotationalSpringConstant halfshaftLeftC(min = 0) = 15000
    "Left halfshaft torsional stiffness";
  parameter SI.RotationalDampingConstant halfshaftLeftD(min = 0) = 2*sqrt(halfshaftLeftC*0.02)
    "Left halfshaft torsional damping";
  parameter SI.RotationalSpringConstant halfshaftRightC(min = 0) = 15000
    "Right halfshaft torsional stiffness";
  parameter SI.RotationalDampingConstant halfshaftRightD(min = 0) = 2*sqrt(halfshaftRightC*0.02)
    "Right halfshaft torsional damping";

  Modelica.Mechanics.Rotational.Components.IdealGear finalDrive(
    ratio = finalDriveRatio,
    useSupport = false) annotation(
      Placement(transformation(origin = {-44, 0}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.Rotational.Components.Inertia diffInputRotor(
    J = diffInputRotorJ,
    w(start = initialOutputAngularVelocity, fixed = true)) annotation(
      Placement(transformation(origin = {-18, 0}, extent = {{-8, -8}, {8, 8}})));

  BobLib.Drivelines.Internal.Differential1D differential(
    use_lsd = diff_use_lsd,
    driveSideTorqueSign = diff_driveSideTorqueSign,
    T_preload = diff_T_preload,
    lockFractionAccel = diff_lockFractionAccel,
    lockFractionDecel = diff_lockFractionDecel,
    T_capacity_max = diff_T_capacity_max,
    clutchEffectiveRadius = diff_clutchEffectiveRadius,
    kineticFrictionRatio = diff_kineticFrictionRatio,
    w_transition = diff_w_transition,
    c_viscous = diff_c_viscous) if not diff_lockedKinematics annotation(
      Placement(transformation(origin = {10, 0}, extent = {{10, -10}, {-10, 10}}, rotation = -180)));

  BobLib.Drivelines.Internal.LockedDifferential1D lockedDifferential
    if diff_lockedKinematics annotation(
      Placement(transformation(origin = {10, 0}, extent = {{10, -10}, {-10, 10}}, rotation = -180)));

  Modelica.Mechanics.Rotational.Components.SpringDamper leftHalfshaft(
    c = halfshaftLeftC,
    d = halfshaftLeftD,
    phi_rel(start = 0),
    stateSelect = StateSelect.never) annotation(
      Placement(transformation(origin = {34, -28}, extent = {{-8, -8}, {8, 8}})));

  Modelica.Mechanics.Rotational.Components.SpringDamper rightHalfshaft(
    c = halfshaftRightC,
    d = halfshaftRightD,
    phi_rel(start = 0),
    stateSelect = StateSelect.never) annotation(
      Placement(transformation(origin = {34, 28}, extent = {{-8, -8}, {8, 8}})));

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
  VehicleInterfaces.Interfaces.DrivelineBus drivelineBus annotation(
    Placement(transformation(extent = {{-90, 10}, {-70, 30}}), iconTransformation(extent = {{-90, 10}, {-70, 30}})));

  Modelica.Blocks.Sources.RealExpression motorSideSpeedBusSignal(
    y = motorSideSpeed) "Motor-side speed published to VehicleInterfaces driveline bus" annotation(
      Placement(transformation(origin = {-46, 44}, extent = {{-8, -4}, {8, 4}})));

  Modelica.Blocks.Sources.RealExpression diffInputSpeedBusSignal(
    y = diffInputSpeed) "Differential input speed published to VehicleInterfaces driveline bus" annotation(
      Placement(transformation(origin = {-46, 34}, extent = {{-8, -4}, {8, 4}})));

  Modelica.Blocks.Sources.RealExpression leftHalfshaftTorqueBusSignal(
    y = leftHalfshaftTorque) "Left halfshaft torque published to VehicleInterfaces driveline bus" annotation(
      Placement(transformation(origin = {-46, 24}, extent = {{-8, -4}, {8, 4}})));

  Modelica.Blocks.Sources.RealExpression rightHalfshaftTorqueBusSignal(
    y = rightHalfshaftTorque) "Right halfshaft torque published to VehicleInterfaces driveline bus" annotation(
      Placement(transformation(origin = {-46, 14}, extent = {{-8, -4}, {8, 4}})));

initial equation
  leftHalfshaft.phi_rel = 0;
  rightHalfshaft.phi_rel = 0;

equation
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
    Line(points = {{-100, 60}, {-80, 60}, {-80, 20}}, color = {255, 204, 51}, thickness = 0.5));
  connect(motorSideSpeedBusSignal.y, drivelineBus.motorSideSpeed) annotation(
    Line(points = {{-37.2, 44}, {-80, 44}, {-80, 20}}, color = {0, 0, 127}));
  connect(diffInputSpeedBusSignal.y, drivelineBus.diffInputSpeed) annotation(
    Line(points = {{-37.2, 34}, {-80, 34}, {-80, 20}}, color = {0, 0, 127}));
  connect(leftHalfshaftTorqueBusSignal.y, drivelineBus.leftHalfshaftTorque) annotation(
    Line(points = {{-37.2, 24}, {-80, 24}, {-80, 20}}, color = {0, 0, 127}));
  connect(rightHalfshaftTorqueBusSignal.y, drivelineBus.rightHalfshaftTorque) annotation(
    Line(points = {{-37.2, 14}, {-80, 14}, {-80, 20}}, color = {0, 0, 127}));

  connect(transmissionFlange.flange, finalDrive.flange_a) annotation(
    Line(points = {{-100, 0}, {-54, 0}}));
  connect(finalDrive.flange_b, diffInputRotor.flange_a) annotation(
    Line(points = {{-34, 0}, {-26, 0}}));

  if diff_lockedKinematics then
    connect(diffInputRotor.flange_b, lockedDifferential.shaft_in) annotation(
      Line(points = {{-10, 0}, {0, 0}}));
    connect(lockedDifferential.shaft_left, leftHalfshaft.flange_a) annotation(
      Line(points = {{20, 4}, {20, 14}, {26, 14}, {26, 24}}));
    connect(lockedDifferential.shaft_right, rightHalfshaft.flange_a) annotation(
      Line(points = {{20, -4}, {20, -14}, {26, -14}, {26, -24}}));
  else
    connect(diffInputRotor.flange_b, differential.shaft_in) annotation(
      Line(points = {{-10, 0}, {0, 0}}));
    connect(differential.shaft_left, leftHalfshaft.flange_a) annotation(
      Line(points = {{20, 4}, {20, 14}, {26, 14}, {26, 24}}));
    connect(differential.shaft_right, rightHalfshaft.flange_a) annotation(
      Line(points = {{20, -4}, {20, -14}, {26, -14}, {26, -24}}));
  end if;

  connect(leftHalfshaft.flange_b, wheelHub_3.flange) annotation(
    Line(points = {{42, -28}, {60, -28}, {60, -100}}));
  connect(rightHalfshaft.flange_b, wheelHub_4.flange) annotation(
    Line(points = {{42, 28}, {60, 28}, {60, 100}}));
  annotation(Documentation(info = "<html>
<p>
VehicleInterfaces rear driveline containing only the mechanical reduction,
differential, and compliant halfshafts. Battery, inverter, controls, and motor
are intentionally external so vehicle-level experiments can show and replace
each powertrain subsystem independently.
</p>
</html>"));
end RearFinalDriveDifferential;