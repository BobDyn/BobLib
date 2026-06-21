within BobLib.Vehicle.Powertrain;

model PowertrainBatInvMotDiff

  "Battery-motor-final-drive-differential plant for an external ELC assembly"
  extends BobLib.Resources.Icons.PowertrainBatInvMotDiffIcon;

  import SI = Modelica.Units.SI;

  parameter Integer Ns(min = 1) = 140 "Battery cells in series";
  parameter Integer Np(min = 1) = 4 "Battery cells in parallel";
  parameter Real SOC_start(unit = "1", min = 0, max = 1) = 1
    "Initial battery state of charge";
  parameter Real finalDriveRatio = 3.31
    "Motor speed divided by differential input speed";
  parameter SI.AngularVelocity launch_w_eps = 1.0
    "Motor low-speed regularization for power-to-torque conversion";
  parameter Modelica.Mechanics.MultiBody.Types.Axis drivetrainAxis = {1, 0, 0}
    "Rotor axis resolved in mountFrame";
  parameter SI.Position rMotorRotor[3] = {0.20, 0, 0.10}
    "Vector from mountFrame to motor rotor frame, resolved in mountFrame";
  parameter SI.Position rDiffInputRotor[3] = {0.05, 0, 0.05}
    "Vector from mountFrame to differential input rotor frame, resolved in mountFrame";
  parameter SI.Position rDifferential[3] = {0, 0, 0}
    "Vector from mountFrame to differential case frame, resolved in mountFrame";
  parameter SI.Inertia motorRotorJ = 0.02521
    "Motor rotor inertia used for 3D gyroscopic effects";
  parameter SI.Inertia diffInputRotorJ = 0.04
    "Differential input/ring rotating inertia used for 3D gyroscopic effects";
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
  parameter SI.Inertia halfshaftLeftJEquivalent(min = 0) = 0.02
    "Effective reflected inertia used to estimate left halfshaft critical damping";
  parameter SI.RotationalDampingConstant halfshaftLeftD(min = 0) = 
    2*sqrt(halfshaftLeftC*halfshaftLeftJEquivalent)
    "Left halfshaft torsional damping";
  parameter SI.RotationalSpringConstant halfshaftRightC(min = 0) = 15000
    "Right halfshaft torsional stiffness";
  parameter SI.Inertia halfshaftRightJEquivalent(min = 0) = 0.02
    "Effective reflected inertia used to estimate right halfshaft critical damping";
  parameter SI.RotationalDampingConstant halfshaftRightD(min = 0) = 
    2*sqrt(halfshaftRightC*halfshaftRightJEquivalent)
    "Right halfshaft torsional damping";
  parameter SI.Voltage motorVdcMax = 630
    "Motor maximum DC voltage reference";
  parameter Real motorRpmMaxPeak = 6500
    "Motor peak speed reference";
  parameter SI.Torque motorTPeak = 220
    "Motor peak torque";
  parameter SI.Torque motorTCont = 130
    "Motor continuous torque";
  parameter SI.Current motorIPeak = 360
    "Motor peak current";
  parameter SI.Current motorICont = 180
    "Motor continuous current";
  parameter Real motorKtNmPerA = 0.61
    "Motor torque constant";
  parameter SI.Time motorPeakTime = 120
    "Peak torque/current allowance duration";
  parameter SI.Power motorPMechPeak = 124e3
    "Motor peak mechanical power";
  parameter SI.Power motorPContLow = 75e3
    "Motor continuous power envelope low-speed anchor";
  parameter SI.Power motorPContHigh = 75e3
    "Motor continuous power envelope high-speed anchor";
  parameter Real motorEtaMot(unit = "1") = 0.96
    "Motor motoring efficiency reference";
  parameter Real motorEtaReg(unit = "1") = 0.95
    "Motor regen efficiency reference";
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a mountFrame
    "Powertrain reaction mount frame" annotation(
      Placement(transformation(origin = {0, 100}, extent = {{-16, -16}, {16, 16}}, rotation = -90),
        iconTransformation(origin = {0, 100}, extent = {{-16, -16}, {16, 16}}, rotation = -90)));

  Modelica.Mechanics.Rotational.Interfaces.Flange_b leftFlange
    "Left halfshaft output" annotation(
      Placement(transformation(origin = {128, 24}, extent = {{-10, -10}, {10, 10}}),
        iconTransformation(origin = {-100, -40}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.Rotational.Interfaces.Flange_b rightFlange
    "Right halfshaft output" annotation(
      Placement(transformation(origin = {128, -24}, extent = {{-10, -10}, {10, 10}}),
        iconTransformation(origin = {100, -40}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Electrical.Analog.Interfaces.PositivePin hv_p
    "HV bus positive terminal" annotation(
      Placement(transformation(origin = {-100, 54}, extent = {{-10, -10}, {10, 10}}),
        iconTransformation(origin = {-100, 40}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Electrical.Analog.Interfaces.NegativePin hv_n
    "HV bus negative terminal" annotation(
      Placement(transformation(origin = {-100, 14}, extent = {{-10, -10}, {10, 10}}),
        iconTransformation(origin = {-100, 10}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Interfaces.RealInput P_elec
    "Electrical power delivered to the motor side [W]" annotation(
      Placement(transformation(origin = {-60, -120}, extent = {{-20, -20}, {20, 20}}, rotation = 90),
        iconTransformation(origin = {-60, -120}, extent = {{-20, -20}, {20, 20}}, rotation = 90)));

  Battery.BatteryPack battery(
    Ns = Ns,
    Np = Np,
    SOC_start = SOC_start) annotation(
      Placement(transformation(origin = {-70, 34}, extent = {{-10, -10}, {10, 10}})));

  Drivetrain.Motor motor(
    Vdc_max = motorVdcMax,
    rpm_max_peak = motorRpmMaxPeak,
    T_peak = motorTPeak,
    T_cont = motorTCont,
    I_peak_2min = motorIPeak,
    I_cont = motorICont,
    Kt_Nm_per_A = motorKtNmPerA,
    peakTime = motorPeakTime,
    P_mech_peak = motorPMechPeak,
    P_cont_low = motorPContLow,
    P_cont_high = motorPContHigh,
    eta_mot = motorEtaMot,
    eta_reg = motorEtaReg,
    w_eps = launch_w_eps) annotation(
    Placement(transformation(origin = {10, 0}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.MultiBody.Parts.Rotor1D motorRotor(
    J = motorRotorJ,
    n = drivetrainAxis,
    animation = false) annotation(
      Placement(transformation(origin = {20, 0}, extent = {{-8, -8}, {8, 8}})));

  Modelica.Mechanics.Rotational.Components.IdealGear finalDrive(
    ratio = finalDriveRatio,
    useSupport = false) annotation(
      Placement(transformation(origin = {48, 0}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.MultiBody.Parts.Rotor1D diffInputRotor(
    J = diffInputRotorJ,
    n = drivetrainAxis,
    animation = false) annotation(
      Placement(transformation(origin = {76, 0}, extent = {{-8, -8}, {8, 8}})));

  Drivetrain.Differential differential(
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
      Placement(transformation(origin = {106, 0}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.Rotational.Components.SpringDamper leftHalfshaft(
    c = halfshaftLeftC,
    d = halfshaftLeftD,
    phi_rel(start = 0),
    stateSelect = StateSelect.never) annotation(
      Placement(transformation(origin = {124, 24}, extent = {{-8, -8}, {8, 8}})));

  Modelica.Mechanics.Rotational.Components.SpringDamper rightHalfshaft(
    c = halfshaftRightC,
    d = halfshaftRightD,
    phi_rel(start = 0),
    stateSelect = StateSelect.never) annotation(
      Placement(transformation(origin = {124, -24}, extent = {{-8, -8}, {8, 8}})));

  Modelica.Mechanics.MultiBody.Parts.FixedTranslation toMotorRotor(
    r = rMotorRotor,
    animation = false) annotation(
      Placement(transformation(origin = {20, 74}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));

  Modelica.Mechanics.MultiBody.Parts.FixedTranslation toDiffInputRotor(
    r = rDiffInputRotor,
    animation = false) annotation(
      Placement(transformation(origin = {76, 74}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));

  Modelica.Mechanics.MultiBody.Parts.FixedTranslation toDifferential(
    r = rDifferential,
    animation = false) annotation(
      Placement(transformation(origin = {106, 74}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));

  Modelica.Electrical.Analog.Basic.Ground hvReference annotation(
    Placement(transformation(origin = {-92, 4}, extent = {{-10, -10}, {10, 10}})));

  output Real SOC(unit = "1") "Battery state of charge";
  output Real SOE(unit = "1") "Battery state of energy";
  output SI.Energy E_remaining "Estimated remaining pack energy";
  output SI.Voltage V_dc "HV bus voltage";
  output SI.Current I_dc "HV bus current, positive while discharging";
  output SI.AngularVelocity motorSpeed "Motor shaft speed";
  output SI.AngularVelocity diffInputSpeed "Differential input speed";
  output SI.Torque diffLockTorque "Differential lock torque";
  output SI.Torque diffLockCapacity "Differential static lock capacity";
  output SI.Torque diffLockKineticCapacity "Differential sliding lock capacity";
  output Real diffLockingValue(unit = "1") "Actual differential locking value";
  output Real diffTorqueBiasRatio(unit = "1") "Actual differential torque bias ratio";
  output Boolean diffLocked "True when the LSD is inside the near-stick slip band";
  output SI.Angle leftHalfshaftTwist "Left halfshaft relative twist";
  output SI.Angle rightHalfshaftTwist "Right halfshaft relative twist";
  output SI.Torque leftHalfshaftTorque "Left halfshaft transmitted torque";
  output SI.Torque rightHalfshaftTorque "Right halfshaft transmitted torque";

initial equation
  leftHalfshaft.phi_rel = 0;
  leftHalfshaft.w_rel = 0;
  rightHalfshaft.phi_rel = 0;
  rightHalfshaft.w_rel = 0;

equation
  SOC = battery.SOC;
  SOE = battery.SOE;
  E_remaining = battery.E_remaining;
  V_dc = battery.v;
  I_dc = battery.i;
  motorSpeed = motor.w;
  diffInputSpeed = differential.w_in;
  diffLockTorque = differential.T_lock;
  diffLockCapacity = differential.T_lock_capacity;
  diffLockKineticCapacity = differential.T_lock_kinetic_capacity;
  diffLockingValue = differential.lockingValue;
  diffTorqueBiasRatio = differential.torqueBiasRatio;
  diffLocked = differential.locked;
  leftHalfshaftTwist = leftHalfshaft.phi_rel;
  rightHalfshaftTwist = rightHalfshaft.phi_rel;
  leftHalfshaftTorque = leftHalfshaft.tau;
  rightHalfshaftTorque = rightHalfshaft.tau;

  motor.P_elec = P_elec;

  connect(hvReference.p, battery.p) annotation(
    Line(points = {{-92, 14}, {-92, 34}, {-80, 34}}, color = {0, 0, 255}));
  connect(hv_p, battery.p) annotation(
    Line(points = {{-100, 54}, {-88, 54}, {-88, 34}, {-80, 34}}, color = {0, 0, 255}));
  connect(hv_n, battery.n) annotation(
    Line(points = {{-100, 14}, {-72, 14}, {-72, 34}, {-60, 34}}, color = {0, 0, 255}));

  connect(motor.shaft, motorRotor.flange_a) annotation(
    Line(points = {{20, 0}, {12, 0}}));
  connect(motorRotor.flange_b, finalDrive.flange_a) annotation(
    Line(points = {{28, 0}, {38, 0}}));
  connect(finalDrive.flange_b, diffInputRotor.flange_a) annotation(
    Line(points = {{58, 0}, {68, 0}}));
  connect(diffInputRotor.flange_b, differential.shaft_in) annotation(
    Line(points = {{84, 0}, {96, 0}}));
  connect(differential.shaft_left, leftHalfshaft.flange_a) annotation(
    Line(points = {{116, 4}, {116, 24}}));
  connect(leftHalfshaft.flange_b, leftFlange) annotation(
    Line(points = {{132, 24}, {128, 24}}));
  connect(differential.shaft_right, rightHalfshaft.flange_a) annotation(
    Line(points = {{116, -4}, {116, -24}}));
  connect(rightHalfshaft.flange_b, rightFlange) annotation(
    Line(points = {{132, -24}, {128, -24}}));
  connect(mountFrame, toMotorRotor.frame_a) annotation(
    Line(points = {{0, 100}, {0, 92}, {20, 92}, {20, 84}}, color = {95, 95, 95}));
  connect(toMotorRotor.frame_b, motorRotor.frame_a) annotation(
    Line(points = {{20, 64}, {20, 8}}, color = {95, 95, 95}));
  connect(mountFrame, toDiffInputRotor.frame_a) annotation(
    Line(points = {{0, 100}, {0, 92}, {76, 92}, {76, 84}}, color = {95, 95, 95}));
  connect(toDiffInputRotor.frame_b, diffInputRotor.frame_a) annotation(
    Line(points = {{76, 64}, {76, 8}}, color = {95, 95, 95}));
  connect(mountFrame, toDifferential.frame_a) annotation(
    Line(points = {{0, 100}, {0, 92}, {106, 92}, {106, 84}}, color = {95, 95, 95}));
  connect(toDifferential.frame_b, differential.mountFrame) annotation(
    Line(points = {{106, 64}, {106, 10}}, color = {95, 95, 95}));

  annotation(experiment(StartTime = 0, StopTime = 5, Tolerance = 1e-06, Interval = 0.002),
    __OpenModelica_commandLineOptions = "--matchingAlgorithm=PFPlusExt --indexReductionMethod=dynamicStateSelection -d=initialization,NLSanalyticJacobian",
    __OpenModelica_simulationFlags(lv = "LOG_STDOUT,LOG_ASSERT,LOG_STATS", s = "dassl", variableFilter = ".*"),
    Diagram(coordinateSystem(extent = {{-120, -120}, {140, 120}})),
    Icon(graphics = {
      Line(points = {{58, -34}, {100, -40}}, color = {95, 95, 95}, thickness = 1),
      Line(points = {{36, -34}, {-100, -40}}, color = {95, 95, 95}, thickness = 1),
      Line(points = {{0, 100}, {56, -14}}, color = {95, 95, 95}, thickness = 1)
    }));
end PowertrainBatInvMotDiff;
