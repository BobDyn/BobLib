within BobLibVehicleInterfaces.Experiments.Standards.Architectures;
model ConventionalRearDrive
  "Complete conventional IC rear-drive vehicle architecture"
  import SI = Modelica.Units.SI;
  import Modelica.Math.Vectors.norm;
  import Modelica.Mechanics.MultiBody.Frames;

  inner parameter SI.Length linkDiameter = 0.020;
  inner parameter SI.Length jointDiameter = 0.030;
  inner parameter Boolean headless = false
    "Run without MultiBody animation geometry"
    annotation(Evaluate = true, Dialog(tab = "Animation"));

  replaceable record VehicleRecord =
    BobLibVehicleInterfaces.Records.VehicleDefn.DWBCStabar_DWBCStabarRecord;

  parameter VehicleRecord pVehicle = VehicleRecord();
  parameter SI.Velocity initialVel = 0
    "Initial longitudinal velocity";
  parameter SI.Angle steeringAngleCommand = 0
    "Autonomous steering-wheel command";
  parameter Real acceleratorPedalCommand(min = 0, max = 1) = 0
    "Autonomous accelerator pedal command";
  parameter Real brakePedalCommand(min = 0, max = 1) = 0
    "Autonomous brake pedal command";
  parameter VehicleInterfaces.Types.Gear requestedGearCommand = 0
    "Requested automatic-transmission gear";
  parameter VehicleInterfaces.Types.GearMode.Temp gearboxModeCommand =
    VehicleInterfaces.Types.GearMode.Drive
    "Requested automatic-transmission mode";
  parameter VehicleInterfaces.Types.IgnitionSetting.Temp ignitionCommand =
    VehicleInterfaces.Types.IgnitionSetting.On
    "Ignition command";

  replaceable VehicleInterfaces.Accessories.MinimalAccessories accessories
    constrainedby VehicleInterfaces.Accessories.Interfaces.Base
    "Accessories subsystem" annotation(
      choicesAllMatching = true,
      Dialog(group = "Plant Models"),
      Placement(transformation(extent = {{-146, -40}, {-126, -20}})));

  replaceable BobLibVehicleInterfaces.Engines.SimpleICEngine engine
    constrainedby VehicleInterfaces.Engines.Interfaces.Base
    "Engine subsystem" annotation(
      choicesAllMatching = true,
      Dialog(group = "Plant Models"),
      Placement(transformation(extent = {{-114, -42}, {-84, -12}})));

  replaceable BobLibVehicleInterfaces.Transmissions.FixedRatioTransmission transmission(
    gearRatio = pVehicle.pDriveline.finalDriveRatio)
    constrainedby VehicleInterfaces.Transmissions.Interfaces.BaseAutomaticTransmission
    "Transmission subsystem" annotation(
      choicesAllMatching = true,
      Dialog(group = "Plant Models"),
      Placement(transformation(extent = {{-64, -42}, {-34, -12}})));

  replaceable BobLibVehicleInterfaces.Drivelines.RearFinalDriveDifferential driveline(
    finalDriveRatio = 1,
    diffInputRotorJ = pVehicle.pDriveline.diffInputRotorJ,
    diff_use_lsd = pVehicle.pDriveline.diff_use_lsd,
    diff_driveSideTorqueSign = pVehicle.pDriveline.diff_driveSideTorqueSign,
    diff_T_preload = pVehicle.pDriveline.diff_T_preload,
    diff_lockFractionAccel = pVehicle.pDriveline.diff_lockFractionAccel,
    diff_lockFractionDecel = pVehicle.pDriveline.diff_lockFractionDecel,
    diff_T_capacity_max = pVehicle.pDriveline.diff_T_capacity_max,
    diff_clutchEffectiveRadius = pVehicle.pDriveline.diff_clutchEffectiveRadius,
    diff_kineticFrictionRatio = pVehicle.pDriveline.diff_kineticFrictionRatio,
    diff_w_transition = pVehicle.pDriveline.diff_w_transition,
    diff_c_viscous = pVehicle.pDriveline.diff_c_viscous,
    halfshaftLeftC = pVehicle.pDriveline.halfshaftLeftC,
    halfshaftLeftD = pVehicle.pDriveline.halfshaftLeftD,
    halfshaftRightC = pVehicle.pDriveline.halfshaftRightC,
    halfshaftRightD = pVehicle.pDriveline.halfshaftRightD)
    constrainedby VehicleInterfaces.Drivelines.Interfaces.TwoAxleBase
    "Rear driveline subsystem" annotation(
      choicesAllMatching = true,
      Dialog(group = "Plant Models"),
      Placement(transformation(extent = {{-12, -42}, {18, -12}})));

  replaceable BobLibVehicleInterfaces.Chassis.Chassis_DWBCStabar_DWBCStabar chassis(
    headless = headless,
    initialLongitudinalVelocity = initialVel,
    pVehicle = pVehicle)
    constrainedby VehicleInterfaces.Chassis.Interfaces.TwoAxleBase
    "Chassis subsystem" annotation(
      choicesAllMatching = true,
      Dialog(group = "Plant Models"),
      Placement(transformation(extent = {{44, -52}, {104, -12}})));

  replaceable VehicleInterfaces.Brakes.MinimalBrakes brakes(
    maxTorque = 1500)
    constrainedby VehicleInterfaces.Brakes.Interfaces.TwoAxleBase
    "Brakes subsystem" annotation(
      choicesAllMatching = true,
      Dialog(group = "Plant Models"),
      Placement(transformation(extent = {{120, -42}, {150, -12}})));

  inner replaceable VehicleInterfaces.Roads.FlatRoad road
    constrainedby VehicleInterfaces.Roads.Interfaces.Base
    "Road model" annotation(
      choicesAllMatching = true,
      Dialog(group = "Conditions"),
      Placement(transformation(extent = {{-120, -140}, {-80, -120}})));

  inner replaceable BobLibVehicleInterfaces.Atmospheres.ConstantAtmosphere atmosphere
    constrainedby VehicleInterfaces.Atmospheres.Interfaces.Base
    "Atmospheric model" annotation(
      choicesAllMatching = true,
      Dialog(group = "Conditions"),
      Placement(transformation(extent = {{-70, -140}, {-30, -120}})));

  inner replaceable Modelica.Mechanics.MultiBody.World world(
    enableAnimation = not headless,
    n = {0, 0, -1},
    driveTrainMechanics3D = false)
    constrainedby Modelica.Mechanics.MultiBody.World
    "Global coordinate system" annotation(
      choicesAllMatching = true,
      Dialog(group = "Conditions"),
      Placement(transformation(extent = {{-150, -140}, {-130, -120}})));

public
  VehicleInterfaces.Interfaces.ControlBus controlBus
    "VehicleInterfaces control bus" annotation(
      Placement(transformation(origin = {-150, 26}, extent = {{-20, -20}, {20, 20}}, rotation = 90)));

  Modelica.Mechanics.MultiBody.Joints.FreeMotion cgFreeMotion(
    animation = false,
    r_rel_a(start = {0, 0, 0}, each fixed = true),
    angles_fixed = true,
    angles_start = {0, 0, 0},
    enforceStates = true,
    useQuaternions = false,
    w_rel_a_fixed = true,
    w_rel_a_start = {0, 0, 0},
    v_rel_a(start = {initialVel, 0, 0}, each fixed = true)) annotation(
      Placement(transformation(origin = {132, -90}, extent = {{10, -10}, {-10, 10}})));

  final parameter SI.Position pVehicleCG[3] = pVehicle.pSprungMass.rCM
    "Approximate sprung chassis CG used for the free-motion reference";

  Real bodyVels[3];
  SI.Velocity windVelocityWorld[3];
  SI.Velocity windVelocityBody[3];
  SI.Velocity relativeAirVelocity[3];
  SI.Velocity relativeAirSpeed;
  SI.Velocity speed;
  SI.Density airDensity;

protected
  VehicleInterfaces.Interfaces.DriverBus driverBus annotation(
    Placement(transformation(extent = {{-136, 74}, {-116, 94}})));

  Modelica.Mechanics.MultiBody.Parts.Fixed cgFixed(
    r = pVehicleCG,
    animation = false) annotation(
      Placement(transformation(origin = {162, -90}, extent = {{10, -10}, {-10, 10}})));

  Modelica.Blocks.Sources.RealExpression steeringAngleSource(
    y = steeringAngleCommand) annotation(
      Placement(transformation(origin = {30, 92}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.Rotational.Sources.Position steerPosition(
    exact = true,
    w(start = 0)) annotation(
      Placement(transformation(origin = {74, 70}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Sources.RealExpression acceleratorPedalSource(
    y = acceleratorPedalCommand) annotation(
      Placement(transformation(origin = {-154, 90}, extent = {{-5, -2}, {5, 2}})));

  Modelica.Blocks.Sources.RealExpression brakePedalSource(
    y = brakePedalCommand) annotation(
      Placement(transformation(origin = {-154, 84}, extent = {{-5, -2}, {5, 2}})));

  Modelica.Blocks.Sources.IntegerConstant requestedGearSource(
    k = requestedGearCommand) annotation(
      Placement(transformation(origin = {-154, 78}, extent = {{-5, -2}, {5, 2}})));

  Modelica.Blocks.Sources.IntegerConstant gearboxModeSource(
    k = gearboxModeCommand) annotation(
      Placement(transformation(origin = {-154, 72}, extent = {{-5, -2}, {5, 2}})));

  Modelica.Blocks.Sources.IntegerConstant ignitionSource(
    k = ignitionCommand) annotation(
      Placement(transformation(origin = {-154, 66}, extent = {{-5, -2}, {5, 2}})));

  Modelica.Blocks.Sources.RealExpression aeroFrontRideHeight(
    y = chassis.frontRideHeight) annotation(
      Placement(transformation(origin = {36, -64}, extent = {{-5, -2}, {5, 2}})));

  Modelica.Blocks.Sources.RealExpression aeroRearRideHeight(
    y = chassis.rearRideHeight) annotation(
      Placement(transformation(origin = {36, -72}, extent = {{-5, -2}, {5, 2}})));

  Modelica.Blocks.Sources.RealExpression aeroRelativeAirSpeed(
    y = relativeAirSpeed) annotation(
      Placement(transformation(origin = {36, -80}, extent = {{-5, -2}, {5, 2}})));

  BobLibVehicleInterfaces.Aero.CFDAeroMap aeroModel(
    pAero = pVehicle.pAero,
    mountOffset = pVehicle.pAero.aeroRef - pVehicleCG,
    headless = headless) annotation(
      Placement(transformation(origin = {76, -84}, extent = {{-10, -10}, {10, 10}})));

equation
  bodyVels =
    Frames.resolve2(cgFreeMotion.frame_b.R, cgFreeMotion.v_rel_a);

  windVelocityWorld =
    atmosphere.windVelocityWorld;

  windVelocityBody =
    Frames.resolve2(cgFreeMotion.frame_b.R, windVelocityWorld);

  relativeAirVelocity =
    bodyVels - windVelocityBody;

  relativeAirSpeed =
    norm(relativeAirVelocity);

  airDensity =
    atmosphere.airDensity;

  speed =
    norm(bodyVels);

  connect(accessories.engineFlange, engine.accessoryFlange) annotation(
    Line(points = {{-126, -30}, {-114, -30}, {-114, -27}}, color = {135, 135, 135}, thickness = 0.5));
  connect(engine.transmissionFlange, transmission.engineFlange) annotation(
    Line(points = {{-84, -27}, {-64, -27}}, color = {135, 135, 135}, thickness = 0.5));
  connect(transmission.drivelineFlange, driveline.transmissionFlange) annotation(
    Line(points = {{-34, -27}, {-12, -27}}, color = {135, 135, 135}, thickness = 0.5));

  connect(chassis.wheelHub_2, driveline.wheelHub_2) annotation(Line(
    points = {{57.125, -12}, {57.125, 4}, {-6, 4}, {-6, -12}},
    color = {135, 135, 135},
    thickness = 0.5));
  connect(driveline.wheelHub_4, chassis.wheelHub_4) annotation(Line(
    points = {{12, -12}, {12, -2}, {90.875, -2}, {90.875, -12}},
    color = {135, 135, 135},
    thickness = 0.5));
  connect(chassis.wheelHub_4, brakes.wheelHub_4) annotation(Line(
    points = {{90.875, -12}, {90.875, -2}, {144, -2}, {144, -12}},
    color = {135, 135, 135},
    thickness = 0.5));
  connect(brakes.wheelHub_2, chassis.wheelHub_2) annotation(Line(
    points = {{126, -12}, {126, 4}, {57.125, 4}, {57.125, -12}},
    color = {135, 135, 135},
    thickness = 0.5));
  connect(driveline.wheelHub_3, chassis.wheelHub_3) annotation(Line(
    points = {{12, -42}, {12, -62}, {90.875, -62}, {90.875, -52}},
    color = {135, 135, 135},
    thickness = 0.5));
  connect(chassis.wheelHub_1, driveline.wheelHub_1) annotation(Line(
    points = {{57.125, -52}, {57.125, -67}, {-6, -67}, {-6, -42}},
    color = {135, 135, 135},
    thickness = 0.5));
  connect(chassis.wheelHub_3, brakes.wheelHub_3) annotation(Line(
    points = {{90.875, -52}, {90.875, -62}, {144, -62}, {144, -42}},
    color = {135, 135, 135},
    thickness = 0.5));
  connect(chassis.wheelHub_1, brakes.wheelHub_1) annotation(Line(
    points = {{57.125, -52}, {57.125, -67}, {126, -67}, {126, -42}},
    color = {135, 135, 135},
    thickness = 0.5));

  connect(controlBus, accessories.controlBus) annotation(Line(
    points = {{-150, 26}, {-154, 26}, {-154, -24}, {-146, -24}},
    color = {255, 204, 51},
    thickness = 0.5));
  connect(controlBus, engine.controlBus) annotation(Line(
    points = {{-150, 26}, {-122, 26}, {-122, -18}, {-114, -18}},
    color = {255, 204, 51},
    thickness = 0.5));
  connect(controlBus, transmission.controlBus) annotation(Line(
    points = {{-150, 26}, {-72, 26}, {-72, -18}, {-64, -18}},
    color = {255, 204, 51},
    thickness = 0.5));
  connect(controlBus, driveline.controlBus) annotation(Line(
    points = {{-150, 26}, {-20, 26}, {-20, -18}, {-12, -18}},
    color = {255, 204, 51},
    thickness = 0.5));
  connect(controlBus, chassis.controlBus) annotation(Line(
    points = {{-150, 26}, {108, 26}, {108, -10}, {44.375, -10}, {44.375, -20}},
    color = {255, 204, 51},
    thickness = 0.5));
  connect(controlBus, brakes.controlBus) annotation(Line(
    points = {{-150, 26}, {115, 26}, {115, -18}, {120, -18}},
    color = {255, 204, 51},
    thickness = 0.5));
  connect(controlBus.driverBus, driverBus) annotation(Line(
    points = {{-150, 26}, {-126, 26}, {-126, 84}},
    color = {255, 204, 51},
    thickness = 0.5));

  connect(acceleratorPedalSource.y, driverBus.acceleratorPedalPosition) annotation(
    Line(points = {{-148.5, 90}, {-126, 90}, {-126, 84}}, color = {0, 0, 127}));
  connect(brakePedalSource.y, driverBus.brakePedalPosition) annotation(
    Line(points = {{-148.5, 84}, {-126, 84}}, color = {0, 0, 127}));
  connect(requestedGearSource.y, driverBus.requestedGear) annotation(
    Line(points = {{-148.5, 78}, {-126, 78}, {-126, 84}}, color = {255, 127, 0}));
  connect(gearboxModeSource.y, driverBus.gearboxMode) annotation(
    Line(points = {{-148.5, 72}, {-122, 72}, {-122, 84}}, color = {255, 127, 0}));
  connect(ignitionSource.y, driverBus.ignition) annotation(
    Line(points = {{-148.5, 66}, {-118, 66}, {-118, 84}}, color = {255, 127, 0}));

  connect(steeringAngleSource.y, steerPosition.phi_ref) annotation(
    Line(points = {{41, 92}, {56, 92}, {56, 70}, {62, 70}}, color = {0, 0, 127}));
  connect(steerPosition.flange, chassis.steeringWheel) annotation(
    Line(points = {{84, 70}, {100, 70}, {100, 2}, {74, 2}, {74, -12}}));

  connect(aeroModel.sprungChassisFrame, chassis.chassisFrame) annotation(
    Line(points = {{66, -84}, {44, -84}, {44, -46}}, color = {95, 95, 95}));
  connect(aeroFrontRideHeight.y, aeroModel.frontRideHeight) annotation(
    Line(points = {{41.5, -64}, {66, -64}, {66, -72}}, color = {0, 0, 127}));
  connect(aeroRearRideHeight.y, aeroModel.rearRideHeight) annotation(
    Line(points = {{41.5, -72}, {70, -72}}, color = {0, 0, 127}));
  connect(aeroRelativeAirSpeed.y, aeroModel.relativeAirSpeed) annotation(
    Line(points = {{41.5, -80}, {74, -80}, {74, -72}}, color = {0, 0, 127}));
  connect(atmosphere.airDensity, aeroModel.airDensity) annotation(
    Line(points = {{-30, -130}, {78, -130}, {78, -72}}, color = {0, 0, 127}));

  connect(cgFixed.frame_b, cgFreeMotion.frame_a) annotation(
    Line(points = {{152, -90}, {142, -90}}, color = {95, 95, 95}));
  connect(cgFreeMotion.frame_b, chassis.chassisFrame) annotation(
    Line(points = {{122, -90}, {112, -90}, {112, -110}, {44, -110}, {44, -46}}, color = {95, 95, 95}));

  annotation(
    Diagram(coordinateSystem(extent = {{-160, -150}, {175, 110}})),
    experiment(StartTime = 0.0, StopTime = 10, Tolerance = 1e-06, Interval = 0.002),
    Documentation(info = "<html>
<p>
Model <code>ConventionalRearDrive</code> is a complete IC rear-drive BobLib
architecture assembled at the VehicleInterfaces subsystem level. It includes
accessories, engine, transmission, driveline, chassis, brakes, road,
atmosphere, aero, and world components, plus autonomous command sources that
populate the standard VehicleInterfaces driver bus.
</p>
<p>
The engine and transmission are deliberately simple sample implementations.
They provide a runnable IC architecture reference while keeping the public
assembly shaped like a VehicleInterfaces conventional vehicle demo.
</p>
</html>"));
end ConventionalRearDrive;
