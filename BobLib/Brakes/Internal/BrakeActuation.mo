within BobLib.Brakes.Internal;
model BrakeActuation

  "Basic VCU-commanded mechanical brakes opposing wheel angular velocity"

  import Modelica.Units.SI;

  parameter BobLib.Records.VehicleRecord.Brakes.BrakeRecord pBrakes "Brake capacity, bias, pedal and input-shaft calibration" annotation(Dialog(group = "Calibration"));

  extends VehicleInterfaces.Icons.Brakes;

  extends BobLib.Brakes.Interfaces.Base(
    final includeWheelBearings = false, final includeBrakePedal = usePedal);

  BobLib.Brakes.Internal.AxleBrakes frontBrakes(
    final wRegularization = pBrakes.wRegularization,
    final usingMultiBodyChassis = usingMultiBodyChassis,
    leftTorqueRequest = brakeTorque_1,
    rightTorqueRequest = brakeTorque_2) annotation(Placement(transformation(origin = {0, 40}, extent = {{-30, 15}, {30, -15}}, rotation = 0)));

  BobLib.Brakes.Internal.AxleBrakes rearBrakes(
    final wRegularization = pBrakes.wRegularization,
    final usingMultiBodyChassis = usingMultiBodyChassis,
    leftTorqueRequest = brakeTorque_3,
    rightTorqueRequest = brakeTorque_4) if not useDifferentialInputBrake annotation(Placement(visible = not useDifferentialInputBrake, transformation(origin = {0, -40}, extent = {{-30, -15}, {30, 15}}, rotation = 0)));

  Modelica.Mechanics.Rotational.Sources.Torque differentialBrake(useSupport = usingMultiBodyChassis) if useDifferentialInputBrake annotation(Placement(visible = useDifferentialInputBrake, transformation(origin = {60, -75}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.Rotational.Sensors.SpeedSensor differentialSpeed if useDifferentialInputBrake annotation(Placement(visible = useDifferentialInputBrake, transformation(origin = {110, -55}, extent = {{-8, -8}, {8, 8}}, rotation = 90)));
  output Modelica.Units.SI.Torque differentialBrakeTorque "Signed brake torque at the differential input";

  parameter Boolean usePedal = false "Read brake demand from the physical pedal travel instead of the control bus" annotation(Dialog(group = "Architecture"), choices(checkBox = true), Evaluate = true);

  output SI.Torque mechanicalBrakeTorqueRequest
    "Limited combined mechanical brake torque request";
  output SI.Torque frontCircuitBrakeTorqueRequest
    "Front hydraulic circuit brake torque request";
  output SI.Torque rearCircuitBrakeTorqueRequest
    "Rear hydraulic circuit brake torque request";
  output SI.Torque brakeTorque_1
    "Front-left brake torque magnitude";
  output SI.Torque brakeTorque_2
    "Front-right brake torque magnitude";
  output SI.Torque brakeTorque_3
    "Rear-left brake torque magnitude";
  output SI.Torque brakeTorque_4
    "Rear-right brake torque magnitude";

  Modelica.Mechanics.Translational.Components.SpringDamper pedalCompliance(
    c = pBrakes.pedalStiffness, d = pBrakes.pedalDamping, s_rel0 = 0) if usePedal
    annotation(Placement(visible = usePedal, transformation(origin = {0, 80}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
  Modelica.Mechanics.Translational.Components.Fixed pedalHousing if usePedal
    annotation(Placement(visible = usePedal, transformation(origin = {20, 70}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));
  Modelica.Mechanics.Translational.Sensors.PositionSensor pedalPosition if usePedal
    annotation(Placement(visible = usePedal, transformation(origin = {40, 80}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));

protected
  outer Modelica.Mechanics.MultiBody.World world;

  Modelica.Mechanics.MultiBody.Parts.Mounting1D differentialCaliper(n = {0, 1, 0})
    if usingMultiBodyChassis and useDifferentialInputBrake annotation(Placement(visible = usingMultiBodyChassis and useDifferentialInputBrake, transformation(origin = {110, -80}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  BobLib.Brakes.Interfaces.BrakesControlBus brakesControlBus if not usePedal
    "Brake-controller command bus tap" annotation(Placement(visible = not usePedal, transformation(origin = {-120, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  BobLib.Brakes.Interfaces.BrakesBus brakesBus
    "Brake measurement bus tap" annotation(Placement(transformation(origin = {-50, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

  Modelica.Blocks.Interfaces.RealInput torqueRequest(
    quantity = "Torque",
    unit = "N.m")
    if not usePedal "Combined mechanical brake torque request from brakesControlBus" annotation(Placement(visible = not usePedal, transformation(origin = {-120, 25}, extent = {{-6, -6}, {6, 6}}, rotation = 0)));

  Modelica.Mechanics.Rotational.Sensors.SpeedSensor wheelSpeed_1 annotation(Placement(transformation(origin = {-100, 18}, extent = {{-6, -6}, {6, 6}}, rotation = -90)));
  Modelica.Mechanics.Rotational.Sensors.SpeedSensor wheelSpeed_2 annotation(Placement(transformation(origin = {100, 18}, extent = {{-6, -6}, {6, 6}}, rotation = -90)));
  Modelica.Mechanics.Rotational.Sensors.SpeedSensor wheelSpeed_3 annotation(Placement(transformation(origin = {-100, -18}, extent = {{-6, -6}, {6, 6}}, rotation = 90)));
  Modelica.Mechanics.Rotational.Sensors.SpeedSensor wheelSpeed_4 annotation(Placement(transformation(origin = {100, -18}, extent = {{-6, -6}, {6, 6}}, rotation = 90)));

equation
  connect(
    frontBrakes.leftRotor,
    wheelHub_1.flange) annotation(Line(points = {{-30, 34}, {-150, 34}, {-150, 40}, {-160, 40}}));
  connect(
    frontBrakes.leftCaliperMount,
    wheelHub_1.bearingFrame) annotation(Line(visible = usingMultiBodyChassis, points = {{-30, 46}, {-146, 46}, {-146, 40}, {-160, 40}}, color = {95, 95, 95}));
  connect(
    frontBrakes.rightRotor,
    wheelHub_2.flange) annotation(Line(points = {{30, 34}, {150, 34}, {150, 40}, {160, 40}}));
  connect(
    frontBrakes.rightCaliperMount,
    wheelHub_2.bearingFrame) annotation(Line(visible = usingMultiBodyChassis, points = {{30, 46}, {146, 46}, {146, 40}, {160, 40}}, color = {95, 95, 95}));
  connect(
    rearBrakes.leftRotor,
    wheelHub_3.flange) annotation(Line(visible = not useDifferentialInputBrake, points = {{-30, -34}, {-150, -34}, {-150, -40}, {-160, -40}}));
  connect(
    rearBrakes.leftCaliperMount,
    wheelHub_3.bearingFrame) annotation(Line(visible = usingMultiBodyChassis and not useDifferentialInputBrake, points = {{-30, -46}, {-146, -46}, {-146, -40}, {-160, -40}}, color = {95, 95, 95}));
  connect(
    rearBrakes.rightRotor,
    wheelHub_4.flange) annotation(Line(visible = not useDifferentialInputBrake, points = {{30, -34}, {150, -34}, {150, -40}, {160, -40}}));
  connect(
    rearBrakes.rightCaliperMount,
    wheelHub_4.bearingFrame) annotation(Line(visible = usingMultiBodyChassis and not useDifferentialInputBrake, points = {{30, -46}, {146, -46}, {146, -40}, {160, -40}}, color = {95, 95, 95}));
  assert(pBrakes.differentialBrakeRatio > 0, "BrakeActuation: pBrakes.differentialBrakeRatio must be positive");
  assert(pBrakes.maxTorque >= 0 and pBrakes.frontBrakeBias >= 0 and pBrakes.frontBrakeBias <= 1,
    "BrakeActuation: maxTorque must be nonnegative and frontBrakeBias must lie in [0, 1]");
  assert(not usingMultiBodyChassis or world.driveTrainMechanics3D,
    "Spatial brakes require world.driveTrainMechanics3D=true");
  connect(differentialBrake.support, differentialCaliper.flange_b) annotation(
    Line(
      visible = usingMultiBodyChassis and useDifferentialInputBrake,
      points = {{60, -85}, {90, -85}, {90, -80}, {100, -80}},
      color = {0, 0, 0}));
  connect(differentialCaliper.frame_a, differentialInput.bearingFrame) annotation(
    Line(
      visible = usingMultiBodyChassis and useDifferentialInputBrake,
      points = {{110, -90}, {110, -95}, {140, -95}, {140, -80}, {160, -80}},
      color = {95, 95, 95}));

  if useDifferentialInputBrake then
    differentialBrakeTorque = -rearCircuitBrakeTorqueRequest/pBrakes.differentialBrakeRatio*noEvent(max(-1, min(1, differentialSpeed.w/max(pBrakes.wRegularization, 1e-6))));
    differentialBrake.tau = differentialBrakeTorque;
  else
    differentialBrakeTorque = 0;
  end if;
  connect(
    differentialBrake.flange,
    differentialInput.flange) annotation(Line(visible = useDifferentialInputBrake, points = {{70, -75}, {150, -75}, {150, -80}, {160, -80}}));
  connect(
    differentialSpeed.flange,
    differentialInput.flange) annotation(Line(visible = useDifferentialInputBrake, points = {{110, -63}, {110, -75}, {150, -75}, {150, -80}, {160, -80}}));

  if usePedal then
    mechanicalBrakeTorqueRequest = noEvent(min(pBrakes.maxTorque, max(0, pBrakes.maxTorque*pedalPosition.s/pBrakes.pedalFullTravel)));
  else
    mechanicalBrakeTorqueRequest = noEvent(min(pBrakes.maxTorque, max(0, torqueRequest)));
  end if;
  assert(
    pBrakes.pedalFullTravel>0 and pBrakes.pedalStiffness>0 and pBrakes.pedalDamping>=0,
    "Invalid pedal calibration");
  connect(
    brakePedal,
    pedalCompliance.flange_a) annotation(Line(visible = usePedal, points = {{0, 100}, {0, 94}, {-6, 94}, {-6, 80}}, color = {0, 127, 0}));
  connect(
    pedalCompliance.flange_b,
    pedalHousing.flange) annotation(Line(visible = usePedal, points = {{6, 80}, {20, 80}, {20, 70}}, color = {0, 127, 0}));
  connect(
    brakePedal,
    pedalPosition.flange) annotation(Line(visible = usePedal, points = {{0, 100}, {30, 100}, {30, 80}, {34, 80}}, color = {0, 127, 0}));

  frontCircuitBrakeTorqueRequest =
    pBrakes.frontBrakeBias*mechanicalBrakeTorqueRequest;
  rearCircuitBrakeTorqueRequest =
    (1 - pBrakes.frontBrakeBias)*mechanicalBrakeTorqueRequest;

  brakeTorque_1 = frontCircuitBrakeTorqueRequest/2;
  brakeTorque_2 = frontCircuitBrakeTorqueRequest/2;
  brakeTorque_3 = if useDifferentialInputBrake then 0 else rearCircuitBrakeTorqueRequest/2;
  brakeTorque_4 = if useDifferentialInputBrake then 0 else rearCircuitBrakeTorqueRequest/2;

  connect(controlBus.brakesControlBus, brakesControlBus) annotation(
    Line(
      visible = not usePedal,
      points = {{-160, 0}, {-120, 0}},
      color = {255, 204, 51},
      thickness = 0.5));
  connect(controlBus.brakesBus, brakesBus) annotation(
    Line(points = {{-160, 0}, {-140, 0}, {-140, -2}, {-50, -2}, {-50, 0}}, color = {255, 204, 51}, thickness = 0.5));
  connect(brakesControlBus.mechanicalBrakeTorqueRequest, torqueRequest) annotation(
    Line(visible = not usePedal, points = {{-120, 0}, {-120, 25}}, color = {0, 0, 127}));

  connect(wheelSpeed_1.flange, wheelHub_1.flange) annotation(
    Line(points = {{-100, 24}, {-100, 34}, {-150, 34}, {-150, 40}, {-160, 40}}));
  connect(wheelSpeed_2.flange, wheelHub_2.flange) annotation(
    Line(points = {{100, 24}, {100, 34}, {150, 34}, {150, 40}, {160, 40}}));
  connect(wheelSpeed_3.flange, wheelHub_3.flange) annotation(
    Line(points = {{-100, -24}, {-100, -34}, {-150, -34}, {-150, -40}, {-160, -40}}));
  connect(wheelSpeed_4.flange, wheelHub_4.flange) annotation(
    Line(points = {{100, -24}, {100, -34}, {150, -34}, {150, -40}, {160, -40}}));

  connect(wheelSpeed_1.w, brakesBus.wheelSpeed_1) annotation(Line(points = {{-100, 11.4}, {-100, 6}, {-54, 6}, {-54, 0}, {-50, 0}}, color = {0, 0, 127}));

  connect(wheelSpeed_2.w, brakesBus.wheelSpeed_2) annotation(Line(points = {{100, 11.4}, {100, 8}, {-46, 8}, {-46, 0}, {-50, 0}}, color = {0, 0, 127}));

  connect(wheelSpeed_3.w, brakesBus.wheelSpeed_3) annotation(Line(points = {{-100, -11.4}, {-100, -6}, {-54, -6}, {-54, 0}, {-50, 0}}, color = {0, 0, 127}));

  connect(wheelSpeed_4.w, brakesBus.wheelSpeed_4) annotation(Line(points = {{100, -11.4}, {100, -8}, {-46, -8}, {-46, 0}, {-50, 0}}, color = {0, 0, 127}));

  annotation(
    Diagram(coordinateSystem(extent = {{-160, -100}, {160, 100}}, preserveAspectRatio = true)),
    Documentation(info = "<html>
<p>
Model <code>BasicVCUBrakes</code> is a minimal BobLib brake model exposed
through the standard VehicleInterfaces two-axle brakes boundary. It subscribes
to <code>controlBus.brakesControlBus.mechanicalBrakeTorqueRequest</code>,
limits that positive combined brake torque request, splits it across the
front and rear hydraulic circuits using a mechanical bias-bar ratio, then
applies torque opposite each wheel's rotational velocity. With useDifferentialInputBrake, the rear wheel sources are zero and a single differentialInput port receives the rear request divided by pBrakes.differentialBrakeRatio. Connect this port upstream of that reduction; never connect it directly to a wheel hub. The frontBrakes and rearBrakes assemblies each expose separate left/right rotor and caliper-mount connections. The existing VI wheelHub ports carry these paired connections at the public subsystem boundary. In differential-input mode the rear axle brake assembly is absent and differentialCaliper mounts the single input brake. The connected bearing frame selects the reaction path.
</p>
<p>
This model intentionally does not implement hydraulic dynamics, ABS, thermal
behavior, or static holding torque. It is a simple VCU-commanded mechanical
brake actuator for early vehicle-level studies. The bias ratio is kept as the
obvious insertion point for future pedal-box modeling, bias-bar compliance, and
hydraulic damping.
</p>
</html>"));
end BrakeActuation;
