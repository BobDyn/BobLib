within BobLib.Resources.VehicleRecord.Powertrain;

record PowertrainBatInvMotDiffRecord
  import SI = Modelica.Units.SI;

  parameter Integer Ns(min = 1) = 140 "Battery cells in series";
  parameter Integer Np(min = 1) = 4 "Battery cells in parallel";
  parameter Real SOC_start(unit = "1", min = 0, max = 1) = 1
    "Initial battery state of charge";
  parameter Real finalDriveRatio = 3.31
    "Motor speed divided by differential input speed";
  parameter SI.AngularVelocity launch_w_eps = 1.0
    "Low-speed regularization for VCU and motor power conversion";
  parameter Real vcuMotorSpeedSign = 1
    "Multiplier mapping sensed motor speed to drive-positive speed in the VCU";
  parameter Modelica.Mechanics.MultiBody.Types.Axis drivetrainAxis = {1, 0, 0}
    "Rotor axis resolved in the powertrain mount frame";

  parameter SI.Position rMotorRotor[3] = {0.20, 0, 0.10}
    "Vector from rear axle mount frame to motor rotor frame";
  parameter SI.Position rDiffInputRotor[3] = {0.05, 0, 0.05}
    "Vector from rear axle mount frame to differential input rotor frame";
  parameter SI.Position rDifferential[3] = {0, 0, 0}
    "Vector from rear axle mount frame to differential case frame";

  parameter SI.Inertia motorRotorJ = 0.02521
    "Motor rotor inertia used for Rotor1D";
  parameter SI.Inertia diffInputRotorJ = 0.04
    "Differential input/ring inertia used for Rotor1D";

  parameter SI.Torque tau_max = 220
    "VCU motoring torque limit";
  parameter SI.Torque regenTorqueLimit = 220
    "Default generated-vehicle regen torque limit magnitude";

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
    "EMRAX 228 MV maximum battery voltage";
  parameter Real motorRpmMaxPeak = 6500
    "EMRAX 228 peak-speed reference";
  parameter SI.Torque motorTPeak = 220
    "EMRAX 228 peak torque";
  parameter SI.Torque motorTCont = 130
    "EMRAX 228 continuous torque";
  parameter SI.Current motorIPeak = 360
    "EMRAX 228 MV peak current";
  parameter SI.Current motorICont = 180
    "EMRAX 228 MV continuous current";
  parameter Real motorKtNmPerA = 0.61
    "EMRAX 228 MV torque constant";
  parameter SI.Time motorPeakTime = 120
    "EMRAX 228 S2 peak-duration reference";
  parameter SI.Power motorPMechPeak = 124e3
    "EMRAX 228 peak mechanical power";
  parameter SI.Power motorPContLow = 75e3
    "EMRAX 228 continuous power envelope low-speed anchor";
  parameter SI.Power motorPContHigh = 75e3
    "EMRAX 228 continuous power envelope high-speed anchor";
  parameter Real motorEtaMot(unit = "1") = 0.96
    "Reference motoring efficiency";
  parameter Real motorEtaReg(unit = "1") = 0.95
    "Reference regen efficiency";

  parameter SI.Power inverterPMaxMot = 124e3
    "Maximum motoring output power";
  parameter SI.Power inverterPMaxReg = 124e3
    "Maximum regenerative input power magnitude";
  parameter SI.Voltage inverterVdcMax = 588
    "Maximum DC bus voltage before regen is disabled";

end PowertrainBatInvMotDiffRecord;
