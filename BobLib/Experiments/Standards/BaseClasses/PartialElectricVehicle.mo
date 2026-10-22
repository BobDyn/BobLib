within BobLib.Experiments.Standards.BaseClasses;

partial model PartialElectricVehicle "FSAE EV plant, calibration, brake topology, and measurements"

  import Modelica.Units.SI;

  // Vehicle record and initial state
  replaceable record VehicleRecord = BobLib.Records.VehicleDefn.EVBatInvMotDiff_DWBCStabar_DWBCStabarRecord
    constrainedby BobLib.Records.VehicleDefn.EVBatInvMotDiff_DWBCStabar_DWBCStabarRecord
    annotation(choicesAllMatching = true);

  // OMC 1.27.1 needs calibration before inherited consumers to preserve derived FMI dependencies.
  parameter VehicleRecord pVehicle "Complete vehicle calibration record; subsystem records are passed to the selected models" annotation(
    Dialog(tab = "Setup", group = "Vehicle Definition"));
  parameter Boolean includeRotorEffects = false
    "Include rotor gyroscopic and carrier-acceleration effects; supports remain active" annotation(choices(checkBox = true), Evaluate = true, Dialog(tab = "Setup", group = "Mechanics"));

  parameter BobLib.Brakes.Types.Placement frontBrakePlacement = BobLib.Brakes.Types.Placement.Outboard "Front brake location selecting upright or sprung-frame reaction paths" annotation(Dialog(tab = "Setup", group = "Brakes"), choices(choice = BobLib.Brakes.Types.Placement.Outboard "Outboard", choice = BobLib.Brakes.Types.Placement.Inboard "Inboard"));
  parameter BobLib.Brakes.Types.Placement rearBrakePlacement = BobLib.Brakes.Types.Placement.Outboard "Rear brake location selecting wheel, halfshaft or differential-input braking" annotation(Dialog(tab = "Setup", group = "Brakes"));

  parameter BobLib.Drivelines.Types.DifferentialArchitecture differentialArchitecture = BobLib.Drivelines.Types.DifferentialArchitecture.ClutchLSD "Differential torque-sharing architecture selected at translation time" annotation(Dialog(tab = "Setup", group = "Mechanics"), Evaluate = true);

  parameter SI.Velocity initialVel = 15 "Initial longitudinal vehicle speed used to initialize chassis motion and wheel spin" annotation(
    Evaluate = false,
    Dialog(tab = "Setup", group = "Initial Conditions"));

  // Animation geometry shared by detailed MultiBody subsystems
  inner parameter SI.Length linkDiameter = 0.020 "Diameter of links in the MultiBody animation; does not set structural stiffness" annotation(
    Dialog(tab = "Animation", group = "Visualization"));
  inner parameter SI.Length jointDiameter = 0.030 "Diameter of joints in the MultiBody animation; does not set joint geometry" annotation(
    Dialog(tab = "Animation", group = "Visualization"));
  inner parameter Boolean headless = false "Run without MultiBody animation geometry" annotation(choices(checkBox = true),
    Evaluate = true,
    Dialog(tab = "Animation", group = "Visualization"));

  extends BobLib.Experiments.Standards.BaseClasses.PartialVehicle(
    final showSteeringConnection = true,
    redeclare replaceable BobLib.Chassis.DoubleWishbone chassis(
    redeclare record ChassisRecord = VehicleRecord.ChassisRecord,
    pChassis = pVehicle.chassis) annotation(
    choicesAllMatching = true,
    Dialog(tab = "Subsystems", group = "Plant Models"),
    Placement(transformation(origin = {90, -50}, extent = {{-32, -20}, {32, 20}}))) constrainedby BobLib.Chassis.BaseClasses.PartialChassis(
    final includeRotorEffects = includeRotorEffects,
    final usingMultiBodyDriveline = true,
    headless = headless,
    initialLongitudinalVelocity = initialVel) "Chassis subsystem",
    redeclare replaceable BobLib.Drivelines.RearFinalDriveDifferential driveline(
      pDriveline = pVehicle.driveline,
      final includeRotorEffects = includeRotorEffects,
      initialFrontAngularVelocity = initialVel/pVehicle.chassis.frontAxle.tire.wheel.geometry.R0,
      initialOutputAngularVelocity = initialVel/pVehicle.chassis.rearAxle.tire.wheel.geometry.R0,
      differentialArchitecture = differentialArchitecture) annotation(
    choicesAllMatching = true,
    Dialog(tab = "Subsystems", group = "Plant Models"),
    Placement(transformation(origin = {25, -50}, extent = {{-15, -15}, {15, 15}}))) constrainedby BobLib.Drivelines.Interfaces.Base(
      final usingMultiBodyTransmission = true,
      final usingMultiBodyChassis = true,
      final useFrontInboardBrakes = frontBrakePlacement==BobLib.Brakes.Types.Placement.Inboard,
      final useRearInboardBrakes = rearBrakePlacement==BobLib.Brakes.Types.Placement.Inboard,
      final useDifferentialInputBrake = rearBrakePlacement==BobLib.Brakes.Types.Placement.DifferentialInput) "Driveline subsystem with optional differential braking",
    redeclare replaceable BobLib.Brakes.BasicVCUBrakes brakes(pBrakes = pVehicle.brakes) annotation(
    choicesAllMatching = true,
    Dialog(tab = "Subsystems", group = "Plant Models"),
    Placement(transformation(origin = {155, -49}, extent = {{-15, -15}, {15, 15}}))) constrainedby BobLib.Brakes.Interfaces.Base(
      final usingMultiBodyChassis = true,
      final useDifferentialInputBrake = rearBrakePlacement==BobLib.Brakes.Types.Placement.DifferentialInput) "Brakes subsystem with optional differential braking",
    redeclare replaceable BobLib.DriverEnvironments.CommandedDriveByWire driverEnvironment
    constrainedby BobLib.DriverEnvironments.Interfaces.Base
    "Driver environment publishing driver intent onto the VehicleInterfaces driver bus" annotation(
    choicesAllMatching = true,
    Dialog(tab = "Subsystems", group = "Controller Models"),
    Placement(transformation(origin = {40, 60}, extent = {{-20, -20}, {20, 20}}))),
    redeclare replaceable VehicleInterfaces.Roads.FlatRoad road annotation(
    choicesAllMatching = true,
    Dialog(tab = "Subsystems", group = "Conditions"),
    Placement(transformation(origin = {-60, -150}, extent = {{-20, -10}, {20, 10}}))) constrainedby VehicleInterfaces.Roads.Interfaces.Base "Road model",
    redeclare replaceable BobLib.Atmospheres.ConstantAtmosphere atmosphere annotation(
    choicesAllMatching = true,
    Dialog(tab = "Subsystems", group = "Conditions"),
    Placement(transformation(origin = {-120, -150}, extent = {{-20, -10}, {20, 10}}))) constrainedby VehicleInterfaces.Atmospheres.Interfaces.Base "Atmospheric model",
    redeclare replaceable Modelica.Mechanics.MultiBody.World world(
    enableAnimation = not headless,
    n = {0, 0, -1},
    final driveTrainMechanics3D = true) annotation(
    choicesAllMatching = true,
    Dialog(tab = "Subsystems", group = "Conditions"),
    Placement(transformation(origin = {-190, -170}, extent = {{-10, -10}, {10, 10}}))) constrainedby Modelica.Mechanics.MultiBody.World "Global coordinate system");

  // Standard outputs
  output SI.Acceleration accX;
  output SI.Acceleration accY;
  output SI.Angle handwheelAngle;
  output SI.Angle steerExcess;
  output SI.Torque handwheelTorque;
  output SI.Force Fz_FL;
  output SI.Force Fz_FR;
  output SI.Force Fz_RL;
  output SI.Force Fz_RR;
  output SI.Angle leftSteerAngle;
  output SI.Angle rightSteerAngle;
  output SI.Angle roll;
  output SI.Angle sideslip;
  output SI.Velocity velX;
  output SI.Velocity velY;
  output SI.AngularVelocity yawVel;

  // Vehicle plant and controller subsystem selections

  replaceable BobLib.EnergyStorage.BatteryPack battery(pBattery = pVehicle.battery, includeGround = true) annotation(
    choicesAllMatching = true,
    Dialog(tab = "Subsystems", group = "Plant Models"),
    Placement(transformation(origin = {-150, -10}, extent = {{-10, -10}, {10, 10}}, rotation = -90))) constrainedby BobLib.EnergyStorage.Interfaces.Base "Energy storage subsystem";

  replaceable BobLib.PowerElectronics.InverterDC inverter(pInverter = pVehicle.inverter) constrainedby BobLib.PowerElectronics.Interfaces.Base "Power electronics subsystem" annotation(choicesAllMatching = true,
    Dialog(tab = "Subsystems", group = "Plant Models"),
    Placement(transformation(origin = {-110, -30}, extent = {{-10, -10}, {10, 10}})));

  replaceable BobLib.ElectricDrives.Motor motor(final includeRotorEffects = includeRotorEffects,
    pMotor = pVehicle.motor) annotation(
    choicesAllMatching = true,
    Dialog(tab = "Subsystems", group = "Plant Models"),
    Placement(transformation(origin = {-79, -10}, extent = {{-10, -10}, {10, 10}}, rotation = 90))) constrainedby BobLib.ElectricDrives.Interfaces.Base(
      final usingMultiBodySystem = true) "Traction motor subsystem";

  replaceable BobLib.Transmissions.FixedRatioTransmission transmission(pTransmission = pVehicle.transmission) annotation(
    choicesAllMatching = true,
    Dialog(tab = "Subsystems", group = "Plant Models"),
    Placement(transformation(origin = {-40, -10}, extent = {{-10, -10}, {10, 10}}))) constrainedby BobLib.Transmissions.Interfaces.Base(
      final usingMultiBodyEngine = true, final usingMultiBodyDriveline = true) "Transmission subsystem";

  // Road, atmosphere, and world conditions

  final parameter SI.Length wheelbase = abs(pVehicle.chassis.frontAxle.geometry.wheelCenter[1] - pVehicle.chassis.rearAxle.geometry.wheelCenter[1]) "Longitudinal separation of the front and rear wheel-center reference positions";

protected

  BobLib.Aero.CFDAeroMap aeroModel(
    pAero = pVehicle.aero,
    mountOffset = pVehicle.aero.aeroRef - chassis.chassisReferencePosition,
    headless = headless) annotation(
    Placement(transformation(origin = {-90, -100}, extent = {{32, -20}, {-32, 20}})));

initial equation
  assert(driveline.driveLayout == BobLib.Drivelines.Types.DriveLayout.Rear,
    "PartialElectricVehicle currently requires a rear-drive driveline; front/all-wheel drive requires corresponding assembly connections.");
  assert(pVehicle.chassis.frame.mass.m > 0 and pVehicle.chassis.frame.mass.m < Modelica.Constants.inf,
    "BOBLIB_INVALID_CALIBRATION: pVehicle.chassis.frame.mass.m must be finite and positive; actual=" + String(pVehicle.chassis.frame.mass.m));
  assert(pVehicle.chassis.frontAxle.tire.wheel.geometry.R0 > 0 and pVehicle.chassis.frontAxle.tire.wheel.geometry.R0 < Modelica.Constants.inf and
    pVehicle.chassis.rearAxle.tire.wheel.geometry.R0 > 0 and pVehicle.chassis.rearAxle.tire.wheel.geometry.R0 < Modelica.Constants.inf,
    "BOBLIB_INVALID_CALIBRATION: wheel radii must be finite and positive; front=" + String(pVehicle.chassis.frontAxle.tire.wheel.geometry.R0) +
    ", rear=" + String(pVehicle.chassis.rearAxle.tire.wheel.geometry.R0));

equation
  assert(noEvent(abs(accX) < Modelica.Constants.inf and abs(accY) < Modelica.Constants.inf and
    abs(velX) < Modelica.Constants.inf and abs(velY) < Modelica.Constants.inf and
    abs(yawVel) < Modelica.Constants.inf and abs(sideslip) < Modelica.Constants.inf and
    abs(roll) < Modelica.Constants.inf and abs(Fz_FL) < Modelica.Constants.inf and
    abs(Fz_FR) < Modelica.Constants.inf and abs(Fz_RL) < Modelica.Constants.inf and abs(Fz_RR) < Modelica.Constants.inf),
    "BOBLIB_NONFINITE_STATE: invalid vehicle motion or tire load at t=" + String(time) +
    " s. Check geometry, mass/inertia, tire calibration, initial state, and solver diagnostics.");
  connect(motor.chassisFrame, chassis.chassisFrame) annotation(
    Line(points = {{-69, -10}, {-65, -10}, {-65, -60}, {-50, -60}, {-50, -100}, {45, -100}, {45, -64}, {58, -64}}, color = {95, 95, 95}));
  connect(driveline.chassisFrame, chassis.chassisFrame) annotation(
    Line(points = {{13, -65}, {13, -100}, {45, -100}, {45, -64}, {58, -64}}, color = {95, 95, 95}));
  connect(transmission.transmissionMount, driveline.drivelineMount) annotation(
    Line(points = {{-40, -20}, {-40, -90}, {25, -90}, {25, -65}}, color = {95, 95, 95}));
  assert(
    frontBrakePlacement<>BobLib.Brakes.Types.Placement.DifferentialInput,
    "This rear-drive vehicle has no front differential. Use Outboard or Inboard front brakes.");
  connect(brakes.differentialInput, driveline.differentialBrakeShaft) annotation(
    Line(
      visible = rearBrakePlacement==BobLib.Brakes.Types.Placement.DifferentialInput,
      points = {{170, -49}, {175, -49}, {175, -115}, {0, -115}, {0, -59}, {10, -59}}));
  connect(controlBus, transmission.controlBus) annotation(
    Line(points = {{-200, 30}, {-65, 30}, {-65, -4}, {-50, -4}}, color = {255, 204, 51}, thickness = 0.5));
  connect(controlBus, battery.controlBus) annotation(
    Line(points = {{-200, 30}, {-155.5, 30}, {-155.5, 0}, {-155, 0}}, color = {255, 204, 51}, thickness = 0.5));
  connect(controlBus, inverter.controlBus) annotation(
    Line(points = {{-200, 30}, {-122, 30}, {-116, 30}, {-116, -20}}, color = {255, 204, 51}, thickness = 0.5));
  connect(controlBus, motor.controlBus) annotation(
    Line(points = {{-200, 30}, {-85, 30}, {-85, 0}}, color = {255, 204, 51}, thickness = 0.5));
  connect(controlBus, aeroModel.controlBus) annotation(
    Line(points = {{-200, 30}, {-5, 30}, {-5, -80}, {-66, -80}}, color = {255, 204, 51}, thickness = 0.5));
  leftSteerAngle = chassis.leftSteerAngle;
  rightSteerAngle = chassis.rightSteerAngle;
  handwheelAngle = chassis.steeringWheel.phi;
  steerExcess = chassis.avgSteerAngle - wheelbase*chassis.bodyAngularVelocity[3]/max(chassis.vehicleSpeed, 0.1);
  velX = chassis.bodyVelocity[1];
  velY = chassis.bodyVelocity[2];
  yawVel = chassis.bodyAngularVelocity[3];
  sideslip = Modelica.Math.atan2(velY, velX);
  accX = chassis.bodyAcceleration[1];
  accY = chassis.bodyAcceleration[2];
  Fz_FL = chassis.Fz_1;
  Fz_FR = chassis.Fz_2;
  Fz_RL = chassis.Fz_3;
  Fz_RR = chassis.Fz_4;

  connect(
    driveline.wheelHub_1,
    chassis.wheelHub_1) annotation(Line(visible = frontBrakePlacement==BobLib.Brakes.Types.Placement.Inboard, points = {{16, -65}, {16, -90}, {71.8667, -90}, {71.8667, -70}}));
  connect(
    driveline.wheelHub_2,
    chassis.wheelHub_2) annotation(Line(visible = frontBrakePlacement==BobLib.Brakes.Types.Placement.Inboard, points = {{16, -35}, {16, 20}, {71.8667, 20}, {71.8667, -30}}));

  // Read roll directly from the chassis orientation matrix to avoid Euler branch flips.
  roll = Modelica.Math.atan2(chassis.chassisFrame.R.T[2, 3], chassis.chassisFrame.R.T[3, 3]);

  // Note that .tau is the reaction by Newton's 3rd law. Negate for applied torque.
  handwheelTorque = -1*chassis.steeringWheel.tau;
  connect(battery.pin_p, inverter.p) annotation(
    Line(points = {{-143, -20}, {-143.5, -20}, {-143.5, -23.8}, {-120, -23.8}}, color = {0, 0, 255}));
  connect(inverter.n, battery.pin_n) annotation(
    Line(points = {{-120, -36}, {-155, -36}, {-155, -20}}, color = {0, 0, 255}));
  connect(transmission.drivelineFlange.flange, driveline.transmissionFlange.flange) annotation(
    Line(points = {{-30, -10}, {-10, -10}, {-10, -50}, {10, -50}}, color = {135, 135, 135}, thickness = 0.5));

  if frontBrakePlacement==BobLib.Brakes.Types.Placement.Outboard then
    connect(brakes.wheelHub_2, chassis.wheelHub_2) annotation(
    Line(
      visible = frontBrakePlacement==BobLib.Brakes.Types.Placement.Outboard,
      points = {{146, -34}, {146, 20}, {71.8667, 20}, {71.8667, -30}},
      color = {135, 135, 135}));
  else
    connect(
      brakes.wheelHub_2,
      driveline.inboardBrake_2) annotation(Line(visible = frontBrakePlacement<>BobLib.Brakes.Types.Placement.Outboard, points = {{146, -34}, {146, 25}, {21.25, 25}, {21.25, -35}}));
  end if;
  connect(driveline.wheelHub_4, chassis.wheelHub_4) annotation(
    Line(points = {{34, -35}, {34, 0}, {108.133, 0}, {108.133, -30}}, color = {135, 135, 135}));

  if rearBrakePlacement<>BobLib.Brakes.Types.Placement.Inboard then
    connect(brakes.wheelHub_4, chassis.wheelHub_4) annotation(
    Line(
      visible = rearBrakePlacement<>BobLib.Brakes.Types.Placement.Inboard,
      points = {{164, -34}, {164, 0}, {108.133, 0}, {108.133, -30}},
      color = {135, 135, 135}));
  else
    connect(
      brakes.wheelHub_4,
      driveline.inboardBrake_4) annotation(Line(visible = rearBrakePlacement==BobLib.Brakes.Types.Placement.Inboard, points = {{164, -34}, {164, 10}, {28.75, 10}, {28.75, -35}}));
  end if;

  if frontBrakePlacement==BobLib.Brakes.Types.Placement.Outboard then
    connect(brakes.wheelHub_1, chassis.wheelHub_1) annotation(
    Line(
      visible = frontBrakePlacement==BobLib.Brakes.Types.Placement.Outboard,
      points = {{146, -64}, {146, -90}, {71.8667, -90}, {71.8667, -70}},
      color = {135, 135, 135}));
  else
    connect(
      brakes.wheelHub_1,
      driveline.inboardBrake_1) annotation(Line(visible = frontBrakePlacement<>BobLib.Brakes.Types.Placement.Outboard, points = {{146, -64}, {146, -110}, {21.25, -110}, {21.25, -65}}));
  end if;
  connect(driveline.wheelHub_3, chassis.wheelHub_3) annotation(
    Line(points = {{34, -65}, {34, -80}, {108.133, -80}, {108.133, -70}}, color = {135, 135, 135}));

  if rearBrakePlacement<>BobLib.Brakes.Types.Placement.Inboard then
    connect(brakes.wheelHub_3, chassis.wheelHub_3) annotation(
    Line(
      visible = rearBrakePlacement<>BobLib.Brakes.Types.Placement.Inboard,
      points = {{164, -64}, {164, -80}, {108.133, -80}, {108.133, -70}},
      color = {135, 135, 135}));
  else
    connect(
      brakes.wheelHub_3,
      driveline.inboardBrake_3) annotation(Line(visible = rearBrakePlacement==BobLib.Brakes.Types.Placement.Inboard, points = {{164, -64}, {164, -115}, {28.75, -115}, {28.75, -65}}));
  end if;
  connect(aeroModel.sprungChassisFrame, chassis.chassisFrame) annotation(
    Line(points = {{-58, -100}, {45, -100}, {45, -64}, {58, -64}}, color = {95, 95, 95}));
  connect(inverter.motor_n, motor.pin_n) annotation(
    Line(points = {{-100, -36}, {-92, -36}, {-92, -16}, {-89, -16}}, color = {0, 0, 255}));
  connect(inverter.motor_p, motor.pin_p) annotation(
    Line(points = {{-100, -24}, {-94, -24}, {-94, -4}, {-89, -4}}, color = {0, 0, 255}));
  connect(motor.shaft_b.flange, transmission.engineFlange.flange) annotation(
    Line(points = {{-79, 0}, {-79, 10}, {-50, 10}, {-50, -10}}, color = {135, 135, 135}, thickness = 0.5));
  annotation(Diagram(coordinateSystem(extent = {{-200, -180}, {180, 120}})));
  annotation(Documentation(info = "<html><p>Motor and differential use separate rigid mounts, located by their subsystem mountOffset records. Each subsystem owns its mount and exposes a chassisFrame attachment. The motor shaft connection carries rotation only; its stator reacts through motor.mount. The fixed-ratio transmission reacts at the differential housing through driveline.drivelineMount and driveline.mount. No chain tension or individual fastener loads are inferred from the lumped transmission.</p><h4>Parameter setup</h4><p>Begin with <code>pVehicle</code>: its nested records calibrate the chassis, motor, battery, controller, inverter, transmission, brakes, driveline and aero models. Replaceable subsystem classes and brake/differential architecture choices are separate from calibration. <code>includeRotorEffects</code> adds gyroscopic and carrier-acceleration effects; motor, transmission and brake support reactions remain active either way. <code>initialVel</code> initializes longitudinal motion and wheel spin. Animation controls change visualization only. Geometry and map values can be configured before initialization in supported reusable artifacts; connector switches, array capacities and model redeclarations require rebuilding.</p></html>"));
end PartialElectricVehicle;
