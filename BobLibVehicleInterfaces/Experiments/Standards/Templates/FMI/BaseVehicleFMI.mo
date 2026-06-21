within BobLibVehicleInterfaces.Experiments.Standards.Templates.FMI;

partial model BaseVehicleFMI

  import SI = Modelica.Units.SI;

  inner parameter SI.Length linkDiameter = 0.020;
  inner parameter SI.Length jointDiameter = 0.030;
  inner parameter Boolean headless = false "Run without MultiBody animation geometry" annotation(
    Evaluate = true,
    Dialog(tab = "Animation"));
  replaceable record VehicleRecord = BobLibVehicleInterfaces.Records.VehicleDefn.EVBatInvMotDiff_DWBCStabar_DWBCStabarRecord;
  parameter VehicleRecord pVehicle = VehicleRecord();
  parameter SI.Velocity initialVel = 0 "Initial vehicle speed" annotation(
    Evaluate = false);

  Modelica.Blocks.Interfaces.RealInput steeringAngleCommand(
    quantity = "Angle",
    unit = "rad",
    start = 0) "Commanded steering-wheel angle" annotation(
      Placement(
        transformation(origin = {-180, 100}, extent = {{-10, -10}, {10, 10}}),
        iconTransformation(origin = {-100, 70}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Interfaces.RealInput acceleratorPedalCommand(
    unit = "1",
    min = 0,
    max = 1,
    start = 0) "Commanded accelerator pedal position" annotation(
      Placement(
        transformation(origin = {-180, 80}, extent = {{-10, -10}, {10, 10}}),
        iconTransformation(origin = {-100, 45}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Interfaces.RealInput brakePedalCommand(
    unit = "1",
    min = 0,
    max = 1,
    start = 0) "Commanded brake pedal position" annotation(
      Placement(
        transformation(origin = {-180, 60}, extent = {{-10, -10}, {10, 10}}),
        iconTransformation(origin = {-100, 20}, extent = {{-10, -10}, {10, 10}})));

  // Standard outputs
  SI.Velocity vehicleSpeed;
  SI.Acceleration accX;
  SI.Acceleration accY;
  SI.Angle handwheelAngle;
  SI.Angle steerExcess;
  SI.Torque handwheelTorque;
  SI.Force Fz_FL;
  SI.Force Fz_FR;
  SI.Force Fz_RL;
  SI.Force Fz_RR;
  SI.Angle leftSteerAngle;
  SI.Angle rightSteerAngle;
  SI.Angle roll;
  SI.Angle sideslip;
  SI.Velocity velX;
  SI.Velocity velY;
  SI.AngularVelocity yawVel;

  replaceable BobLibVehicleInterfaces.Chassis.Chassis_DWBCStabar_DWBCStabar chassis(
    headless = headless,
    initialLongitudinalVelocity = initialVel,
    pVehicle = pVehicle) annotation(
    choicesAllMatching = true,
    Dialog(group = "Plant Models"),
    Placement(transformation(origin = {90, -50}, extent = {{-30, -20}, {30, 20}}))) constrainedby VehicleInterfaces.Chassis.Interfaces.TwoAxleBase "Chassis subsystem";

  replaceable BobLibVehicleInterfaces.EnergyStorage.BatteryPack battery(
    includeGround = true,
    Ns = pVehicle.pBattery.Ns,
    Np = pVehicle.pBattery.Np,
    SOC_start = pVehicle.pBattery.SOC_start) annotation(
    choicesAllMatching = true,
    Dialog(group = "Plant Models"),
    Placement(transformation(origin = {-150, -10}, extent = {{-10, -10}, {10, 10}}, rotation = -90))) constrainedby VehicleInterfaces.EnergyStorage.Interfaces.Base "Energy storage subsystem";

  replaceable BobLibVehicleInterfaces.Controllers.VCU vcu(
    tau_max = pVehicle.pVCU.tau_max,
    w_eps = pVehicle.pVCU.w_eps,
    motorSpeedSign = pVehicle.pVCU.motorSpeedSign,
    finalDriveRatio = pVehicle.pDriveline.finalDriveRatio,
    enablePTNDriveSpeedControl = false,
    enablePTNRegenSpeedControl = false,
    enablePTNMechanicalBrakeSpeedControl = false,
    targetVel = initialVel,
    regenTorqueLimit = pVehicle.pVCU.regenTorqueLimit,
    mechanicalBrakeTorqueLimit = pVehicle.pVCU.mechanicalBrakeTorqueLimit,
    regenBrakeBlend = pVehicle.pVCU.regenBrakeBlend) annotation(
    choicesAllMatching = true,
    Dialog(group = "Controller Models"),
    Placement(transformation(origin = {130, 50}, extent = {{-10, -10}, {10, 10}}))) constrainedby VehicleInterfaces.Controllers.Interfaces.Base "Vehicle control unit";

  replaceable BobLibVehicleInterfaces.PowerElectronics.InverterDC inverter(
    P_max_mot = pVehicle.pInverter.P_max_mot,
    P_max_reg = pVehicle.pInverter.P_max_reg,
    V_dc_max = pVehicle.pInverter.V_dc_max) "Power electronics subsystem" annotation(
    Placement(transformation(origin = {-110, -30}, extent = {{-10, -10}, {10, 10}})));

  replaceable BobLibVehicleInterfaces.ElectricDrives.Motor motor(
    Vdc_max = pVehicle.pMotor.Vdc_max,
    rpm_max_peak = pVehicle.pMotor.rpm_max_peak,
    T_peak = pVehicle.pMotor.T_peak,
    T_cont = pVehicle.pMotor.T_cont,
    I_peak_2min = pVehicle.pMotor.I_peak_2min,
    I_cont = pVehicle.pMotor.I_cont,
    Kt_Nm_per_A = pVehicle.pMotor.Kt_Nm_per_A,
    peakTime = pVehicle.pMotor.peakTime,
    P_mech_peak = pVehicle.pMotor.P_mech_peak,
    P_cont_low = pVehicle.pMotor.P_cont_low,
    P_cont_high = pVehicle.pMotor.P_cont_high,
    eta_mot = pVehicle.pMotor.eta_mot,
    eta_reg = pVehicle.pMotor.eta_reg,
    w_eps = pVehicle.pMotor.w_eps,
    rotorJ = pVehicle.pMotor.rotorJ) annotation(
    choicesAllMatching = true,
    Dialog(group = "Plant Models"),
    Placement(transformation(origin = {-79, -10.3334}, extent = {{-10, -10}, {10, 10}}, rotation = -180))) constrainedby VehicleInterfaces.ElectricDrives.Interfaces.Base "Traction motor subsystem";

  replaceable BobLibVehicleInterfaces.Transmissions.FixedRatioTransmission transmission(
    gearRatio = pVehicle.pDriveline.finalDriveRatio) annotation(
    choicesAllMatching = true,
    Dialog(group = "Plant Models"),
    Placement(transformation(origin = {-40, -10}, extent = {{-10, -10}, {10, 10}}))) constrainedby VehicleInterfaces.Transmissions.Interfaces.Base "Transmission subsystem";

  replaceable BobLibVehicleInterfaces.Drivelines.RearFinalDriveDifferential driveline(
    finalDriveRatio = 1,
    diffInputRotorJ = pVehicle.pDriveline.diffInputRotorJ,
    initialOutputAngularVelocity = initialVel/pVehicle.pRrPartialWheel.R0,
    diff_lockedKinematics = pVehicle.pDriveline.diff_lockedKinematics,
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
    halfshaftRightD = pVehicle.pDriveline.halfshaftRightD) annotation(
    choicesAllMatching = true,
    Dialog(group = "Plant Models"),
    Placement(transformation(origin = {25, -50}, extent = {{-15, -15}, {15, 15}}))) constrainedby VehicleInterfaces.Drivelines.Interfaces.TwoAxleBase "Driveline subsystem";

  replaceable BobLibVehicleInterfaces.Chassis.Brakes.BasicVCUBrakes brakes(
    maxTorque = pVehicle.pVCU.mechanicalBrakeTorqueLimit) annotation(
    choicesAllMatching = true,
    Dialog(group = "Plant Models"),
    Placement(transformation(origin = {155, -49}, extent = {{-15, -15}, {15, 15}}))) constrainedby VehicleInterfaces.Brakes.Interfaces.TwoAxleBase "Brakes subsystem";

  BobLibVehicleInterfaces.DriverEnvironments.Internal.Driver driverEnvironment
    "Driver environment publishing external driver intent onto the VehicleInterfaces driver bus" annotation(
    Placement(transformation(origin = {40, 60}, extent = {{-20, -20}, {20, 20}})));

  inner replaceable VehicleInterfaces.Roads.FlatRoad road annotation(
    choicesAllMatching = true,
    Dialog(group = "Conditions"),
    Placement(transformation(origin = {-60, -150}, extent = {{-20, -10}, {20, 10}}))) constrainedby VehicleInterfaces.Roads.Interfaces.Base "Road model";

  inner replaceable BobLibVehicleInterfaces.Atmospheres.ConstantAtmosphere atmosphere annotation(
    choicesAllMatching = true,
    Dialog(group = "Conditions"),
    Placement(transformation(origin = {-120, -150}, extent = {{-20, -10}, {20, 10}}))) constrainedby VehicleInterfaces.Atmospheres.Interfaces.Base "Atmospheric model";

  inner replaceable Modelica.Mechanics.MultiBody.World world(
    enableAnimation = not headless,
    n = {0, 0, -1},
    driveTrainMechanics3D = false) annotation(
    choicesAllMatching = true,
    Dialog(group = "Conditions"),
    Placement(transformation(origin = {-170, -150}, extent = {{-10, -10}, {10, 10}}))) constrainedby Modelica.Mechanics.MultiBody.World "Global coordinate system";

  final parameter SI.Length wheelbase = abs(pVehicle.pFrDW.wheelCenter[1] - pVehicle.pRrDW.wheelCenter[1]);

protected
  VehicleInterfaces.Interfaces.ControlBus controlBus
    "Internal VehicleInterfaces control bus namespace" annotation(
    Placement(transformation(origin = {-180, 30}, extent = {{-20, -20}, {20, 20}}, rotation = 90)));

  Modelica.Blocks.Sources.Constant driverMotorTorqueCommand(k = 0)
    "Inactive direct driver motor-torque command" annotation(
    Placement(transformation(origin = {-46.5, 116.4}, extent = {{-6.5, -2.6}, {6.5, 2.6}})));

  Modelica.Blocks.Sources.Constant driverRegenTorqueLimitCommand(k = 0)
    "Inactive direct driver regenerative-torque command" annotation(
    Placement(transformation(origin = {-46.5, 107.8}, extent = {{-6.5, -2.6}, {6.5, 2.6}})));

  Modelica.Blocks.Sources.BooleanConstant inverterEnableCommand(k = true)
    "Internal EV ready-to-drive / inverter-enable command" annotation(
    Placement(transformation(origin = {-46.5, 97.4}, extent = {{-6.5, -2.6}, {6.5, 2.6}})));

  BobLibVehicleInterfaces.Aero.CFDAeroMap aeroModel(
    pAero = pVehicle.pAero,
    mountOffset = pVehicle.pAero.aeroRef - chassis.chassisReferencePosition,
    headless = headless) annotation(
    Placement(transformation(origin = {-90, -100}, extent = {{30, -20}, {-30, 20}})));

equation
  vehicleSpeed = chassis.vehicleSpeed;
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

  // Read roll directly from the chassis orientation matrix to avoid Euler branch flips.
  roll = Modelica.Math.atan2(chassis.chassisFrame.R.T[2, 3], chassis.chassisFrame.R.T[3, 3]);

  // Note that .tau is the reaction by Newton's 3rd law. Negate for applied torque.
  handwheelTorque = -1*chassis.steeringWheel.tau;

  connect(controlBus, driveline.controlBus) annotation(
    Line(points = {{-180, 30}, {1, 30}, {1, -41}, {10, -41}}, color = {255, 204, 51}, thickness = 0.5));
  connect(controlBus, transmission.controlBus) annotation(
    Line(points = {{-180, 30}, {-59, 30}, {-59, -4}, {-50, -4}}, color = {255, 204, 51}, thickness = 0.5));
  connect(controlBus, battery.controlBus) annotation(
    Line(points = {{-180, 30}, {-155.5, 30}, {-155.5, 0}, {-155, 0}}, color = {255, 204, 51}, thickness = 0.5));
  connect(controlBus, vcu.controlBus) annotation(
    Line(points = {{-180, 30}, {131, 30}, {131, 40}, {130, 40}}, color = {255, 204, 51}, thickness = 0.5));
  connect(controlBus, inverter.controlBus) annotation(
    Line(points = {{-180, 30}, {-116, 30}, {-116, -20}, {-116, -20}}, color = {255, 204, 51}, thickness = 0.5));
  connect(controlBus, motor.controlBus) annotation(
    Line(points = {{-180, 30}, {-100.25, 30}, {-100.25, -4}, {-89, -4}}, color = {255, 204, 51}, thickness = 0.5));
  connect(controlBus, chassis.controlBus) annotation(
    Line(points = {{-180, 30}, {49, 30}, {49, -38}, {60, -38}}, color = {255, 204, 51}, thickness = 0.5));
  connect(controlBus, aeroModel.controlBus) annotation(
    Line(points = {{-180, 30}, {-20, 30}, {-20, -80}, {-67.5, -80}}, color = {255, 204, 51}, thickness = 0.5));
  connect(controlBus, brakes.controlBus) annotation(
    Line(points = {{-180, 30}, {131, 30}, {131, -40}, {140, -40}}, color = {255, 204, 51}, thickness = 0.5));
  connect(controlBus, driverEnvironment.controlBus) annotation(
    Line(points = {{-180, 30}, {100, 30}, {100, 72}, {60, 72}}, color = {255, 204, 51}, thickness = 0.5));
  connect(steeringAngleCommand, driverEnvironment.steeringAngleCommand) annotation(
    Line(points = {{-180, 100}, {-40, 100}, {-40, 76}, {17.7, 76}}, color = {0, 0, 127}));
  connect(acceleratorPedalCommand, driverEnvironment.acceleratorPedalCommand) annotation(
    Line(points = {{-180, 80}, {-30, 80}, {-30, 69.6}, {17.55, 69.6}}, color = {0, 0, 127}));
  connect(brakePedalCommand, driverEnvironment.brakePedalCommand) annotation(
    Line(points = {{-180, 60}, {18, 60}, {18, 64}}, color = {0, 0, 127}));
  connect(driverMotorTorqueCommand.y, controlBus.electricMotorControlBus.driverTorqueCommand) annotation(
    Line(points = {{-39, 116}, {92, 116}, {92, 30}, {-180, 30}}, color = {0, 0, 127}));
  connect(driverRegenTorqueLimitCommand.y, controlBus.electricMotorControlBus.driverRegenTorqueLimit) annotation(
    Line(points = {{-39, 108}, {94, 108}, {94, 30}, {-180, 30}}, color = {0, 0, 127}));
  connect(inverterEnableCommand.y, controlBus.driverBus.inverterEnable) annotation(
    Line(points = {{-39, 97}, {96, 97}, {96, 30}, {-180, 30}}, color = {255, 0, 255}));
  connect(battery.pin_p, inverter.p) annotation(
    Line(points = {{-143, -20}, {-143.5, -20}, {-143.5, -24}, {-120, -24}}, color = {0, 0, 255}));
  connect(inverter.n, battery.pin_n) annotation(
    Line(points = {{-120, -36}, {-155, -36}, {-155, -20}}, color = {0, 0, 255}));
  connect(transmission.drivelineFlange, driveline.transmissionFlange) annotation(
    Line(points = {{-30, -10}, {-30, -9.75}, {-10, -9.75}, {-10, -49.5}, {10, -49.5}, {10, -50}}, color = {135, 135, 135}, thickness = 0.5));
  connect(atmosphere.atmosphereBus, aeroModel.atmosphereBus) annotation(
    Line(points = {{-135, -140}, {-135, -80}, {-79, -80}}, color = {255, 204, 51}, thickness = 0.5));
  connect(driveline.wheelHub_2, chassis.wheelHub_2) annotation(
    Line(points = {{16, -35}, {16, 20}, {73, 20}, {73, -30}}, color = {135, 135, 135}));
  connect(brakes.wheelHub_2, chassis.wheelHub_2) annotation(
    Line(points = {{146, -34}, {146, 20}, {73, 20}, {73, -30}}, color = {135, 135, 135}));
  connect(driveline.wheelHub_4, chassis.wheelHub_4) annotation(
    Line(points = {{34, -35}, {34, 0}, {107, 0}, {107, -30}}, color = {135, 135, 135}));
  connect(brakes.wheelHub_4, chassis.wheelHub_4) annotation(
    Line(points = {{164, -34}, {164, 0}, {107, 0}, {107, -30}}, color = {135, 135, 135}));
  connect(driveline.wheelHub_1, chassis.wheelHub_1) annotation(
    Line(points = {{16, -65}, {16, -90}, {73, -90}, {73, -70}}, color = {135, 135, 135}));
  connect(brakes.wheelHub_1, chassis.wheelHub_1) annotation(
    Line(points = {{146, -64}, {146, -90}, {73, -90}, {73, -70}}, color = {135, 135, 135}));
  connect(driveline.wheelHub_3, chassis.wheelHub_3) annotation(
    Line(points = {{34, -65}, {34, -80}, {107, -80}, {107, -70}}, color = {135, 135, 135}));
  connect(brakes.wheelHub_3, chassis.wheelHub_3) annotation(
    Line(points = {{164, -64}, {164, -80}, {107, -80}, {107, -70}}, color = {135, 135, 135}));
  connect(driverEnvironment.steeringWheel, chassis.steeringWheel) annotation(
    Line(points = {{60, 60}, {60, 60.5}, {90, 60.5}, {90, -30}}));
  connect(aeroModel.sprungChassisFrame, chassis.chassisFrame) annotation(
    Line(points = {{-60, -100}, {50, -100}, {50, -64}, {60, -64}}, color = {95, 95, 95}));
  connect(inverter.motor_n, motor.pin_n) annotation(
    Line(points = {{-100, -36}, {-85, -36}, {-85, -20}}, color = {0, 0, 255}));
  connect(inverter.motor_p, motor.pin_p) annotation(
    Line(points = {{-100, -24}, {-73, -24}, {-73, -20}}, color = {0, 0, 255}));
  connect(motor.shaft_b, transmission.engineFlange) annotation(
    Line(points = {{-69, -10.3334}, {-51, -10.3334}}, color = {135, 135, 135}, thickness = 0.5));
  annotation(
    Diagram(coordinateSystem(extent = {{-200, -160}, {180, 120}})),
    experiment(StartTime = 0.0, StopTime = 10, Tolerance = 1e-06, Interval = 0.002),
    __OpenModelica_commandLineOptions = "--matchingAlgorithm=PFPlusExt --indexReductionMethod=dynamicStateSelection -d=initialization,NLSanalyticJacobian,disableStartCalc --maxSizeLinearTearing=5000 --generateDynamicJacobian=none",
    __OpenModelica_simulationFlags(
      jacobian = "internalNumerical",
      lv = "LOG_STDOUT,LOG_ASSERT,LOG_STATS",
      noEquidistantTimeGrid = "()",
      noEventEmit = "()",
      s = "dassl",
      variableFilter = "time|.*Command|vehicleSpeed|accX|accY|handwheelAngle|steerExcess|handwheelTorque|Fz_.*|leftSteerAngle|rightSteerAngle|roll|sideslip|velX|velY|yawVel|vcu\\..*|brakes\\..*"),
    Documentation(info = "<html>
<p>
Partial model <code>BaseVehicleFMI</code> is the shared full-vehicle assembly
for FMI export, driver-in-the-loop, and early software-in-the-loop usage.
</p>
<p>
The model exposes only steering wheel angle, accelerator pedal, and brake pedal
at its boundary. Internal vehicle-level signals hold the EV ready-to-drive state
and inactive direct motor/regen requests. Battery, VCU, inverter, motor,
driveline, brakes, chassis, aero, atmosphere, road, and world remain
preconfigured inside the assembly and communicate through the internal
VehicleInterfaces control bus.
</p>
<p>
By default the VCU speed-control paths are disabled so driver pedals produce
rear-axle drive torque and mechanical brake requests through the same bus
publish/subscribe path used by autonomous maneuver simulations. Direct driver
motor-torque and regenerative-torque pins remain inactive in this base vehicle;
those can be opened later as explicit driver-environment capabilities if a DIL
rig needs them.
</p>
</html>"));
end BaseVehicleFMI;
