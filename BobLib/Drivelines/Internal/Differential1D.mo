within BobLib.Drivelines.Internal;
model Differential1D "Open, plate, or torque-biasing differential"

  import Modelica.Units.SI;
  import BobLib.Drivelines.Types.DifferentialArchitecture;

  parameter BobLib.Records.VehicleRecord.Drivelines.DifferentialRecord pDifferential "Preload, drive/coast locking and friction calibration" annotation(Dialog(group = "Calibration"));

  extends BobLib.Icons.DifferentialIcon;
  Modelica.Mechanics.Rotational.Interfaces.Flange_a shaft_in
    "Input from final drive" annotation(
      Placement(transformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.Rotational.Interfaces.Flange_b shaft_left
    "Left halfshaft output" annotation(
      Placement(transformation(origin = {100, 40}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.Rotational.Interfaces.Flange_b shaft_right
    "Right halfshaft output" annotation(
      Placement(transformation(origin = {100, -40}, extent = {{-10, -10}, {10, 10}})));

  parameter BobLib.Drivelines.Types.DifferentialArchitecture differentialArchitecture =
    BobLib.Drivelines.Types.DifferentialArchitecture.ClutchLSD
    "Differential architecture" annotation(Dialog(group = "Architecture"));

  SI.AngularVelocity w_in = der(shaft_in.phi);
  SI.AngularVelocity w_l = der(shaft_left.phi);
  SI.AngularVelocity w_r = der(shaft_right.phi);
  SI.AngularVelocity dw = w_l - w_r;
  SI.Torque T_in = shaft_in.tau;
  SI.Torque T_lock "Signed torque added to left output";
  SI.Torque T_left_drive = -shaft_left.tau;
  SI.Torque T_right_drive = -shaft_right.tau;
  SI.Torque capacity "Static transfer torque capacity";
  SI.Power lossPower = -T_lock*dw;
  Boolean locked "Actual static friction state, not a slip-speed threshold";

protected
  final parameter Boolean limitedSlip = differentialArchitecture <> DifferentialArchitecture.Open "Activate the differential locking law for a non-open differential architecture";
  final parameter Real frictionRatio = if differentialArchitecture == DifferentialArchitecture.ClutchLSD then pDifferential.slidingFrictionRatio
    else 1 "Sliding-to-static friction ratio used by the clutch differential; unity for other architectures";
  constant SI.Torque unitTorque = 1;
  OpenDifferential1D gearing annotation(Placement(transformation(origin = {-40, 0}, extent = {{-20, -20}, {20, 20}})));

  Modelica.Blocks.Sources.RealExpression capacitySignal(y = capacity/unitTorque)
    if limitedSlip annotation(Placement(visible = limitedSlip, transformation(origin = {80, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Real biasRatio;
  Real lockingFraction;

  Modelica.Mechanics.Rotational.Components.Clutch clutch(
    fn_max = 1, cgeo = 1, mu_pos = [0, frictionRatio], peak = 1/frictionRatio)
    if limitedSlip annotation(Placement(visible = limitedSlip, transformation(origin = {40, 0}, extent = {{-20, -20}, {20, 20}}, rotation = -90)));

equation
  assert(differentialArchitecture <> DifferentialArchitecture.Locked, "Use LockedDifferential1D for spool kinematics");
  assert(abs(abs(pDifferential.driveSideTorqueSign) - 1) < 1e-12, "driveSideTorqueSign must be +1 or -1");
  assert(pDifferential.preloadTorque >= 0 and pDifferential.driveLockingFraction >= 0 and pDifferential.driveLockingFraction <= 1 and
    pDifferential.coastLockingFraction >= 0 and pDifferential.coastLockingFraction <= 1 and pDifferential.driveTorqueBiasRatio >= 1 and
    pDifferential.coastTorqueBiasRatio >= 1 and pDifferential.slidingFrictionRatio > 0 and pDifferential.slidingFrictionRatio <= 1,
    "Invalid differential calibration");
  connect(shaft_in, gearing.shaft_in) annotation(Line(points = {{-100, 0}, {-60, 0}}));
  connect(gearing.shaft_left, shaft_left) annotation(Line(points = {{-20, 8}, {-20, 40}, {100, 40}}));
  connect(gearing.shaft_right, shaft_right) annotation(Line(points = {{-20, -8}, {-20, -40}, {100, -40}}));
  biasRatio = if noEvent(pDifferential.driveSideTorqueSign*T_in >= 0) then pDifferential.driveTorqueBiasRatio
    else pDifferential.coastTorqueBiasRatio;
  lockingFraction = if noEvent(pDifferential.driveSideTorqueSign*T_in >= 0) then pDifferential.driveLockingFraction
    else pDifferential.coastLockingFraction;

  // shortcut: effective helical friction bound; use measured maps for hardware-specific calibration.
  capacity = if differentialArchitecture == DifferentialArchitecture.ClutchLSD then
    pDifferential.preloadTorque + lockingFraction*abs(T_in)/2
      elseif differentialArchitecture == DifferentialArchitecture.TorqueBiasing then
    (biasRatio - 1)/(biasRatio + 1)*abs(T_in)/2 else 0;

  if limitedSlip then

    // Unit force and geometry convert the effective torque capacity to the MSL friction law.
    connect(
      capacitySignal.y,
      clutch.f_normalized) annotation(Line(visible = limitedSlip, points = {{69, 0}, {62, 0}}, color = {0, 0, 127}));
    connect(
      clutch.flange_a,
      shaft_left) annotation(Line(visible = limitedSlip, points = {{40, 20}, {40, 40}, {100, 40}}));
    connect(
      clutch.flange_b,
      shaft_right) annotation(Line(visible = limitedSlip, points = {{40, -20}, {40, -40}, {100, -40}}));
    T_lock = clutch.tau;
    locked = clutch.locked;
  else
    T_lock = 0;
    locked = false;
  end if;
  annotation(Documentation(info = "<html><p>Plate LSD: static transfer capacity is preloadTorque + lockingFraction*abs(T_in)/2. MSL Coulomb friction provides true sticking and sliding. Preload is transfer torque, not an unspecified bench breakaway measurement.</p><p>TorqueBiasing: effective capacity is abs(T_in)/2*(TBR-1)/(TBR+1), with no preload. This bounds the output torque ratio in same-sign steady transmission. At zero input torque capacity vanishes. Equal output speeds can occur below the friction limit; they are not constrained as a spool.</p><p>Calibration is generic, not a manufacturer preset. Drive/coast selection follows signed input torque; reverse-direction power-based selection is not implemented. Gear losses, temperature and wear are omitted.</p></html>"),
    Diagram(coordinateSystem(extent = {{-100, -60}, {100, 60}})));
end Differential1D;
