within BobLib.ElectricDrives;
model Motor

  "BobLib motor exposed through the VehicleInterfaces electric-drive shaft"

  import Modelica.Units.SI;

  parameter BobLib.Records.VehicleRecord.ElectricDrives.MotorRecord pMotor "Motor torque, power, efficiency and rotor calibration" annotation(Dialog(group = "Calibration"));

  extends VehicleInterfaces.Icons.ElectricMotor;
  extends BobLib.ElectricDrives.Interfaces.Base;

  parameter Boolean includeRotorEffects = false
    "Include gyroscopic and carrier-acceleration effects when mounted" annotation(Dialog(group = "Architecture"), choices(checkBox = true), Evaluate = true);

  parameter Modelica.Mechanics.MultiBody.Types.Axis shaftAxis = {0, 1, 0}
    "Rotor axis resolved in shaft bearing frame" annotation(Dialog(group = "Operating settings"), Evaluate = true);

  VehicleInterfaces.PowertrainMounts.SingleSystemRigidMount mount(
    r_ChassisToMount1 = pMotor.mountOffset) if includeBearing or usingMultiBodySystem annotation(
    Placement(visible = includeBearing or usingMultiBodySystem,
      transformation(origin = {20, 65}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));

  BobLib.ElectricDrives.Internal.PowerLimitedMotor motor(
    pMotor = pMotor,
    useSupport = includeBearing or usingMultiBodySystem) annotation(
      Placement(transformation(origin = {-20, 15}, extent = {{-25, -25}, {25, 25}})));

  // Prefer rotor speed as a state for well-conditioned direction events.
  Modelica.Mechanics.Rotational.Components.Inertia rotor(
    J = pMotor.rotorJ,
    stateSelect = StateSelect.prefer) if not ((includeBearing or usingMultiBodySystem) and includeRotorEffects) annotation(
      Placement(
        visible = not ((includeBearing or usingMultiBodySystem) and includeRotorEffects),
        transformation(origin = {45, 0}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.MultiBody.Parts.Rotor1D spatialRotor(
    J = pMotor.rotorJ, n = shaftAxis, animation = false) if (includeBearing or usingMultiBodySystem) and includeRotorEffects annotation(
    Placement(
      visible = (includeBearing or usingMultiBodySystem) and includeRotorEffects,
      transformation(origin = {45, 0}, extent = {{-10, -10}, {10, 10}})));

  output SI.AngularVelocity w "Shaft speed";
  output SI.Voltage V_dc "Motor-side DC voltage";
  output SI.Current I_dc "Motor-side DC current, positive while motoring";
  output SI.Power P_elec "Electrical power absorbed by motor, positive while motoring";
  output SI.Power P_mech "Actual mechanical shaft power";
  output SI.Torque tau_cmd "Commanded motor torque";
  output SI.Torque tau_lim "Active motor torque limit";

protected
  outer Modelica.Mechanics.MultiBody.World world;

  Modelica.Mechanics.MultiBody.Parts.Mounting1D stator(n = Modelica.Math.Vectors.normalizeWithAssert(shaftAxis))
    if includeBearing or usingMultiBodySystem annotation(
    Placement(
      visible = includeBearing or usingMultiBodySystem,
      transformation(origin = {55, -30}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));

  Modelica.Blocks.Interfaces.RealInput availablePowerBusTap(unit = "W")
    "Power allocated by the inverter" annotation(
    Placement(transformation(origin = {-75, 40}, extent = {{-5, -5}, {5, 5}})));
  SI.Voltage V_dc_eff;

  Modelica.Blocks.Sources.RealExpression powerInput(
    y = P_elec) "Electrical power measurement fed to the internal motor model" annotation(
      Placement(transformation(origin = {-76, 15}, extent = {{-12, -12}, {12, 12}})));

  Modelica.Blocks.Sources.RealExpression speedSignal(
    y = w) "Shaft speed published to electricMotorBus" annotation(
      Placement(transformation(origin = {-55, -60}, extent = {{12, -12}, {-12, 12}})));

  Modelica.Blocks.Sources.RealExpression electricSignal(
    y = P_elec) "Electrical power published to electricMotorBus" annotation(
      Placement(transformation(origin = {0, -60}, extent = {{12, -12}, {-12, 12}})));

  Modelica.Blocks.Sources.RealExpression shaftSignal(
    y = P_mech) "Mechanical shaft power published to electricMotorBus" annotation(
      Placement(transformation(origin = {55, -60}, extent = {{12, -12}, {-12, 12}})));

  Modelica.Blocks.Sources.RealExpression torqueSignal(
    y = tau_cmd) "Commanded torque published to electricMotorBus" annotation(
      Placement(transformation(origin = {-25, -88}, extent = {{12, -12}, {-12, 12}})));

  Modelica.Blocks.Sources.RealExpression limitSignal(
    y = tau_lim) "Active torque limit published to electricMotorBus" annotation(
      Placement(transformation(origin = {35, -88}, extent = {{12, -12}, {-12, 12}})));

  BobLib.ElectricDrives.Interfaces.ElectricMotorBus electricMotorBus
    "Typed electricMotorBus signal contract" annotation(
      Placement(transformation(origin = {-80, -10}, extent = {{-10, -10}, {10, 10}})));
  BobLib.ElectricDrives.Interfaces.ElectricMotorControlBus electricMotorControlBus
    "Typed electricMotorControlBus signal contract" annotation(
      Placement(transformation(origin = {-80, 65}, extent = {{-20, -20}, {20, 20}})));

equation
  connect(chassisFrame, mount.chassisFrame) annotation(
    Line(visible = includeBearing or usingMultiBodySystem,
      points = {{0, 100}, {0, 75}, {20, 75}}, color = {95, 95, 95}));
  connect(mount.mount_1, shaft_b.bearingFrame) annotation(
    Line(visible = includeBearing or usingMultiBodySystem,
      points = {{20, 55}, {35, 55}, {35, 35}, {100, 35}, {100, 0}}, color = {95, 95, 95}));
  connect(electricMotorBus, controlBus.electricMotorBus) annotation(
    Line(points = {{-80, -10}, {-100, -10}, {-100, -60}}, color = {255, 204, 51}, thickness = 0.5));
  connect(electricMotorControlBus, controlBus.electricMotorControlBus) annotation(
    Line(points = {{-80, 65}, {-100, 65}, {-100, -60}}, color = {255, 204, 51}, thickness = 0.5));
  assert(not (includeBearing or usingMultiBodySystem) or world.driveTrainMechanics3D,
    "Spatial motor requires world.driveTrainMechanics3D=true");
  connect(motor.support, stator.flange_b) annotation(
    Line(
      visible = (includeBearing or usingMultiBodySystem),
      points = {{-20, -10}, {-20, -30}, {45, -30}},
      color = {0, 0, 0}));
  connect(stator.frame_a, shaft_b.bearingFrame) annotation(
    Line(
      visible = (includeBearing or usingMultiBodySystem),
      points = {{55, -20}, {55, -15}, {100, -15}, {100, 0}},
      color = {95, 95, 95}));
  connect(spatialRotor.frame_a, shaft_b.bearingFrame) annotation(
    Line(
      visible = (includeBearing or usingMultiBodySystem) and includeRotorEffects,
      points = {{45, -10}, {45, -15}, {100, -15}, {100, 0}},
      color = {95, 95, 95}));
  connect(motor.shaft, spatialRotor.flange_a) annotation(
    Line(
      visible = (includeBearing or usingMultiBodySystem) and includeRotorEffects,
      points = {{5, 15}, {20, 15}, {20, 0}, {35, 0}},
      color = {0, 0, 0}));
  connect(spatialRotor.flange_b, shaft_b.flange) annotation(
    Line(
      visible = (includeBearing or usingMultiBodySystem) and includeRotorEffects,
      points = {{55, 0}, {100, 0}},
      color = {0, 0, 0}));
  assert(pMotor.V_eps > 0, "Motor: V_eps must be positive");
  connect(electricMotorControlBus.availablePower, availablePowerBusTap) annotation(
    Line(points = {{-80, 65}, {-80, 40}, {-75, 40}}, color = {0, 0, 127}));
  0 = pin_p.i + pin_n.i;
  V_dc = pin_p.v - pin_n.v;
  V_dc_eff = if noEvent(V_dc >= 0) then noEvent(max(V_dc, pMotor.V_eps))
    else noEvent(min(V_dc, -pMotor.V_eps));
  I_dc = pin_p.i;
  I_dc = availablePowerBusTap/V_dc_eff;
  P_elec = V_dc*I_dc;

  w = motor.w;
  P_mech = motor.P_mech;
  tau_cmd = motor.tau_cmd;
  tau_lim = motor.tau_lim;

  connect(speedSignal.y, electricMotorBus.speed) annotation(
    Line(points = {{-68.2, -60}, {-80, -60}, {-80, -10}}, color = {0, 0, 127}));
  connect(electricSignal.y, electricMotorBus.electricalPower) annotation(
    Line(points = {{-13.2, -60}, {-28, -60}, {-28, -40}, {-80, -40}, {-80, -10}}, color = {0, 0, 127}));
  connect(shaftSignal.y, electricMotorBus.mechanicalPower) annotation(
    Line(points = {{41.8, -60}, {30, -60}, {30, -38}, {-80, -38}, {-80, -10}}, color = {0, 0, 127}));
  connect(torqueSignal.y, electricMotorBus.torqueCommand) annotation(
    Line(points = {{-38.2, -88}, {-88, -88}, {-88, -10}, {-80, -10}}, color = {0, 0, 127}));
  connect(limitSignal.y, electricMotorBus.torqueLimit) annotation(
    Line(points = {{21.8, -88}, {10, -88}, {10, -75}, {-84, -75}, {-84, -10}, {-80, -10}}, color = {0, 0, 127}));

  connect(powerInput.y, motor.P_elec) annotation(
    Line(points = {{-62.8, 15}, {-50, 15}}, color = {0, 0, 127}));
  connect(motor.shaft, rotor.flange_a) annotation(
    Line(
      visible = not ((includeBearing or usingMultiBodySystem) and includeRotorEffects),
      points = {{5, 15}, {20, 15}, {20, 0}, {35, 0}}));
  connect(rotor.flange_b, shaft_b.flange) annotation(
    Line(
      visible = not ((includeBearing or usingMultiBodySystem) and includeRotorEffects),
      points = {{55, 0}, {100, 0}}));
  annotation(Documentation(info = "<html>
<p>The internal rigid mount locates the motor housing relative to chassisFrame using pMotor.mountOffset. Connect chassisFrame to the sprung chassis; shaft_b.bearingFrame remains at the motor housing. The VI bearing frame carries stator reaction whenever usingMultiBodySystem
or includeBearing is enabled. includeRotorEffects independently selects Rotor1D
for gyroscopic and carrier-acceleration corrections; otherwise the rotor is a
scalar inertia. Enable world.driveTrainMechanics3D for the mounted support.
VehicleDefn owns the complete locked-rotor inertia tensor. Rotor1D supplies the
relative-spin correction; do not subtract it from that tensor.</p>
<p>
Thin adapter around <code>BobLib.ElectricDrives.Internal.PowerLimitedMotor</code>.
The mechanical side follows <code>BobLib.ElectricDrives.Interfaces.Base</code>
and includes the motor rotor inertia so the downstream driveline can remain a
pure final-drive/differential/halfshaft assembly. The adapter publishes motor
speed, absorbed electrical power, mechanical power, commanded torque, and active
torque limit on <code>controlBus.electricMotorBus</code>. Electrical power is
absorbed through the motor-side DC pins. The averaged current law consumes
<code>controlBus.electricMotorControlBus.availablePower</code> from the inverter,
with positive power for motoring and negative power for regeneration.
<code>V_eps</code> regularizes the current law near zero bus voltage.
</p>
</html>"));
end Motor;
