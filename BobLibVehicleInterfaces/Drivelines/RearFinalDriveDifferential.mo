within BobLibVehicleInterfaces.Drivelines;

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
    J = diffInputRotorJ) annotation(
      Placement(transformation(origin = {-18, 0}, extent = {{-8, -8}, {8, 8}})));

  BobLibVehicleInterfaces.Drivelines.Internal.Differential1D differential(
    use_lsd = diff_use_lsd,
    driveSideTorqueSign = diff_driveSideTorqueSign,
    T_preload = diff_T_preload,
    lockFractionAccel = diff_lockFractionAccel,
    lockFractionDecel = diff_lockFractionDecel,
    T_capacity_max = diff_T_capacity_max,
    clutchEffectiveRadius = diff_clutchEffectiveRadius,
    kineticFrictionRatio = diff_kineticFrictionRatio,
    w_transition = diff_w_transition,
    c_viscous = diff_c_viscous) annotation(
      Placement(transformation(origin = {12, 0}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.Rotational.Components.SpringDamper leftHalfshaft(
    c = halfshaftLeftC,
    d = halfshaftLeftD,
    phi_rel(start = 0),
    stateSelect = StateSelect.never) annotation(
      Placement(transformation(origin = {34, 24}, extent = {{-8, -8}, {8, 8}})));

  Modelica.Mechanics.Rotational.Components.SpringDamper rightHalfshaft(
    c = halfshaftRightC,
    d = halfshaftRightD,
    phi_rel(start = 0),
    stateSelect = StateSelect.never) annotation(
      Placement(transformation(origin = {34, -24}, extent = {{-8, -8}, {8, 8}})));

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
    Placement(transformation(extent = {{-90, 10}, {-70, 30}})));

  Modelica.Blocks.Interfaces.RealOutput motorSideSpeedBusSignal
    "Motor-side speed published to VehicleInterfaces driveline bus";
  Modelica.Blocks.Interfaces.RealOutput diffInputSpeedBusSignal
    "Differential input speed published to VehicleInterfaces driveline bus";
  Modelica.Blocks.Interfaces.RealOutput leftHalfshaftTorqueBusSignal
    "Left halfshaft torque published to VehicleInterfaces driveline bus";
  Modelica.Blocks.Interfaces.RealOutput rightHalfshaftTorqueBusSignal
    "Right halfshaft torque published to VehicleInterfaces driveline bus";

initial equation
  leftHalfshaft.phi_rel = 0;
  rightHalfshaft.phi_rel = 0;

equation
  motorSideSpeed = der(transmissionFlange.flange.phi);
  diffInputSpeed = differential.w_in;
  diffLockTorque = differential.T_lock;
  leftHalfshaftTorque = leftHalfshaft.tau;
  rightHalfshaftTorque = rightHalfshaft.tau;

  motorSideSpeedBusSignal = motorSideSpeed;
  diffInputSpeedBusSignal = diffInputSpeed;
  leftHalfshaftTorqueBusSignal = leftHalfshaftTorque;
  rightHalfshaftTorqueBusSignal = rightHalfshaftTorque;

  connect(controlBus.drivelineBus, drivelineBus) annotation(
    Line(points = {{-100, 60}, {-80, 60}, {-80, 20}}, color = {255, 204, 51}, thickness = 0.5));
  connect(motorSideSpeedBusSignal, drivelineBus.motorSideSpeed) annotation(
    Line(points = {{0, 0}, {0, 44}, {-80, 44}, {-80, 20}}, color = {0, 0, 127}));
  connect(diffInputSpeedBusSignal, drivelineBus.diffInputSpeed) annotation(
    Line(points = {{0, 0}, {0, 38}, {-80, 38}, {-80, 20}}, color = {0, 0, 127}));
  connect(leftHalfshaftTorqueBusSignal, drivelineBus.leftHalfshaftTorque) annotation(
    Line(points = {{0, 0}, {0, 32}, {-80, 32}, {-80, 20}}, color = {0, 0, 127}));
  connect(rightHalfshaftTorqueBusSignal, drivelineBus.rightHalfshaftTorque) annotation(
    Line(points = {{0, 0}, {0, 26}, {-80, 26}, {-80, 20}}, color = {0, 0, 127}));

  connect(transmissionFlange.flange, finalDrive.flange_a) annotation(
    Line(points = {{-100, 0}, {-54, 0}}));
  connect(finalDrive.flange_b, diffInputRotor.flange_a) annotation(
    Line(points = {{-34, 0}, {-26, 0}}));
  connect(diffInputRotor.flange_b, differential.shaft_in) annotation(
    Line(points = {{-10, 0}, {2, 0}}));
  connect(differential.shaft_left, leftHalfshaft.flange_a) annotation(
    Line(points = {{22, 4}, {22, 24}, {26, 24}}));
  connect(leftHalfshaft.flange_b, wheelHub_3.flange) annotation(
    Line(points = {{42, 24}, {60, 24}, {60, -100}}));
  connect(differential.shaft_right, rightHalfshaft.flange_a) annotation(
    Line(points = {{22, -4}, {22, -24}, {26, -24}}));
  connect(rightHalfshaft.flange_b, wheelHub_4.flange) annotation(
    Line(points = {{42, -24}, {60, -24}, {60, 100}}));

  annotation(Documentation(info = "<html>
<p>
VehicleInterfaces rear driveline containing only the mechanical reduction,
differential, and compliant halfshafts. Battery, inverter, controls, and motor
are intentionally external so vehicle-level experiments can show and replace
each powertrain subsystem independently.
</p>
</html>"));
end RearFinalDriveDifferential;
