within BobLibVehicleInterfaces.Records.VehicleRecord.Drivelines;

record RearFinalDriveDifferentialRecord

  import SI = Modelica.Units.SI;

  parameter Real finalDriveRatio = 3.31
    "Motor speed divided by differential input speed";
  parameter SI.Inertia diffInputRotorJ = 0.04
    "Differential input/ring inertia";
  parameter Boolean diff_lockedKinematics = false
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

  annotation(
    Documentation(info = "<html>
<p>
Record <code>RearFinalDriveDifferentialRecord</code> contains the
vehicle-level parameters passed to
<code>Drivelines.RearFinalDriveDifferential</code>.
</p>
<p>
The halfshaft damping defaults are critically damped estimates based on the
declared torsional stiffness and equivalent reflected inertia.
</p>
</html>"));
end RearFinalDriveDifferentialRecord;
