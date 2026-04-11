within BobLib.Standards;

model ISO4138
  import Modelica.SIunits;
  import Modelica.Constants.pi;
  
  import Modelica.Math.Vectors.norm;
  import Modelica.Mechanics.MultiBody.Frames;
  import BobLib.Utilities.Math.Vector;
  
  // Import vehicle records
  import BobLib.Resources.VehicleRecord.Chassis.Suspension.Templates.Tire.Templates.PartialWheelRecord;
  import BobLib.Resources.VehicleDefn.OrionRecord;
  
  // Imoport standard record
  import BobLib.Resources.StandardRecord.ISO4138Record;

  inner parameter SIunits.Length linkDiameter = 0.020;
  inner parameter SIunits.Length jointDiameter = 0.030;
  
  parameter OrionRecord pVehicle;
  
  parameter SIunits.Velocity testVel = 15;
  parameter SIunits.Length testRad = 20;
  
  parameter Real curvGain = 3;
  parameter Real curvTi = 0.02;
  
  parameter Real radErrorTol = 0.002;
  parameter Real der_radErrorTol = 0.5;
  
  discrete Real t_hit(start = -1);
  
  Real curvError;
  Real radError;
  
  Real bodyVels[3];
  Real bodyAccels[3];
  Real bodyAngles[3];
  
  Real speedCG;
  
  // Standard record
  ISO4138Record iso;

  inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1}) annotation(
    Placement(transformation(origin = {-130, -110}, extent = {{-10, -10}, {10, 10}})));

  // Vehicle
  BobLib.Vehicle.VehicleDW_RWD_Lock vehicle(pVehicle = pVehicle)  annotation(
    Placement(transformation(origin = {0, 20}, extent = {{-45, -50}, {45, 50}})));

  // Curvature controller
  Modelica.Blocks.Continuous.PI curvPI(T = curvTi, k = curvGain, initType = Modelica.Blocks.Types.Init.InitialOutput)  annotation(
    Placement(transformation(origin = {-70, 110}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.RealExpression curvErrorExpression(y = curvError)  annotation(
    Placement(transformation(origin = {-110, 110}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Sensors.RelativeAngles sprungAngles annotation(
    Placement(transformation(origin = {50, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));

protected
  // Calculated parameters
  final parameter Real cpInitFL[3] = pVehicle.pFrDW.wheelCenter + Frames.resolve1(Frames.axesRotations({1, 2, 3},
                                                                                                       {pVehicle.pFrPartialWheel.staticGamma*pi/180, 0, pVehicle.pFrPartialWheel.staticAlpha*pi/180},
                                                                                                       {0, 0, 0}),
                                                                                  {0, 0, -pVehicle.pFrPartialWheel.R0});
  final parameter Real cpInitFR[3] = Vector.mirrorXZ(cpInitFL);
  final parameter Real cpInitRL[3] = pVehicle.pRrDW.wheelCenter + Frames.resolve1(Frames.axesRotations({1, 2, 3},
                                                                                                       {pVehicle.pRrPartialWheel.staticGamma*pi/180, 0, pVehicle.pRrPartialWheel.staticAlpha*pi/180},
                                                                                                       {0, 0, 0}),
                                                                                  {0, 0, -pVehicle.pRrPartialWheel.R0});
  final parameter Real cpInitRR[3] = Vector.mirrorXZ(cpInitRL);
  
  Real leftWheelVector[3];
  Real rightWheelVector[3];
  
  // Initial geometry
  Modelica.Mechanics.MultiBody.Parts.Fixed fixedFL(r = cpInitFL, animation = false) annotation(
    Placement(transformation(origin = {-130, 10}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Fixed fixedFR(r = cpInitFR, animation = false) annotation(
    Placement(transformation(origin = {130, 10}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Fixed fixedRL(r = cpInitRL, animation = false) annotation(
    Placement(transformation(origin = {-130, -50}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Fixed fixedRR(r = cpInitRR, animation = false) annotation(
    Placement(transformation(origin = {130, -50}, extent = {{10, -10}, {-10, 10}})));
  
  Modelica.Mechanics.MultiBody.Parts.Fixed cgFixed(r = pVehicle.pSprungMass.rCM, animation = false)  annotation(
    Placement(transformation(origin = {130, 90}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Joints.FreeMotion cgFreeMotion(animation = false, r_rel_a(start = {0, 0, 0}, each fixed = true), enforceStates = false, v_rel_a(start = {testVel, 0, 0}, each fixed = true))  annotation(
    Placement(transformation(origin = {100, 90}, extent = {{10, -10}, {-10, 10}})));
  
  // Ground interface
  BobLib.Utilities.Mechanics.Multibody.GroundPhysics groundFL annotation(
    Placement(transformation(origin = {-100, 10}, extent = {{-10, -10}, {10, 10}})));
  BobLib.Utilities.Mechanics.Multibody.GroundPhysics groundFR annotation(
    Placement(transformation(origin = {100, 10}, extent = {{10, -10}, {-10, 10}})));
  BobLib.Utilities.Mechanics.Multibody.GroundPhysics groundRL annotation(
    Placement(transformation(origin = {-100, -50}, extent = {{-10, -10}, {10, 10}})));
  BobLib.Utilities.Mechanics.Multibody.GroundPhysics groundRR annotation(
    Placement(transformation(origin = {100, -50}, extent = {{10, -10}, {-10, 10}})));
  
  // Rear torque input
  Modelica.Blocks.Sources.RealExpression velErrorExpression(y = testVel - speedCG) annotation(
    Placement(transformation(origin = {-70, -70}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Continuous.PI speedPI(T = 1, k = 200) annotation(
    Placement(transformation(origin = {-30, -70}, extent = {{-10, -10}, {10, 10}})));
  
  // Front steer input
  Modelica.Mechanics.Rotational.Sources.Position frSteerPosition(exact = false) annotation(
    Placement(transformation(origin = {-30, 110}, extent = {{-10, -10}, {10, 10}})));
    
initial equation
  vehicle.chassis.frAxleDW.leftTire.wheelModel.hubAxis.w = testVel / pVehicle.pFrPartialWheel.R0;
  vehicle.chassis.frAxleDW.rightTire.wheelModel.hubAxis.w = testVel / pVehicle.pFrPartialWheel.R0;
  vehicle.chassis.rrAxleDW.leftTire.wheelModel.hubAxis.w = testVel / pVehicle.pRrPartialWheel.R0;
  vehicle.chassis.rrAxleDW.rightTire.wheelModel.hubAxis.w = testVel / pVehicle.pRrPartialWheel.R0;

equation
  radError = abs(speedCG / max(abs(vehicle.chassis.spaceFrame.sprungBody.w_a[3]), 0.1) - abs(testRad));
  
  // Steady-state detection
  when abs(radError) < radErrorTol and abs(der(radError)) < der_radErrorTol and pre(t_hit) < 0 then
    t_hit = time;
  end when;
  
  when t_hit > 0 and time > t_hit + 0.1 then
    terminate("Reached steady-state (held 0.1s)");
  end when;
  
  curvError = smooth(1, min(1, max(0, (time - 1)/0.2))) * (1/testRad - vehicle.chassis.spaceFrame.sprungBody.w_a[3] / max(speedCG, 0.1));
  
  // General quantities
  bodyVels = Frames.resolve2(vehicle.chassis.spaceFrame.sprungBody.frame_a.R, vehicle.chassis.spaceFrame.sprungBody.v_0);
  bodyAccels = Frames.resolve2(vehicle.chassis.spaceFrame.sprungBody.frame_a.R, vehicle.chassis.spaceFrame.sprungBody.a_0);
  bodyAngles = Frames.resolve2(vehicle.chassis.spaceFrame.sprungBody.frame_a.R, sprungAngles.angles);
  
  speedCG = norm(bodyVels);
  
  leftWheelVector = Frames.resolve1(vehicle.chassis.frAxleFrame.R, Frames.resolve2(vehicle.frameFL.R, {1, 0, 0}));
  rightWheelVector = Frames.resolve1(vehicle.chassis.frAxleFrame.R, Frames.resolve2(vehicle.frameFR.R, {1, 0, 0}));
  
  // Output record
  iso.leftSteerAngle = -1 * atan(leftWheelVector[2] / leftWheelVector[1]);
  iso.rightSteerAngle = -1 * atan(rightWheelVector[2] / rightWheelVector[1]);
  iso.handwheelAngle = vehicle.steerFlange.phi;
  
  // Kinematics
  iso.velX = bodyVels[1];
  iso.velY = bodyVels[2];
  iso.yawVel = vehicle.chassis.spaceFrame.sprungBody.w_a[3];
  iso.sideslip = atan(iso.velY / iso.velX);
  
  // Accelerations
  iso.accX = bodyAccels[1];
  iso.accY = bodyAccels[2];
  
  // Vehicle response
  iso.roll = bodyAngles[1];
  iso.handwheelTorque = -1 * vehicle.steerFlange.tau; // Note that .tau is the reaction by Newton's 3rd law. Negate for applied torque.
  
  // Derived
  iso.curvature = vehicle.chassis.spaceFrame.sprungBody.w_a[3] / max(speedCG, 0.1);
  
  connect(cgFixed.frame_b, cgFreeMotion.frame_a) annotation(
    Line(points = {{120, 90}, {110, 90}}, color = {95, 95, 95}));
  connect(velErrorExpression.y, speedPI.u) annotation(
    Line(points = {{-59, -70}, {-43, -70}}, color = {0, 0, 127}));
  connect(fixedFL.frame_b, groundFL.frame_a) annotation(
    Line(points = {{-120, 10}, {-110, 10}}, color = {95, 95, 95}));
  connect(fixedFR.frame_b, groundFR.frame_a) annotation(
    Line(points = {{120, 10}, {110, 10}}, color = {95, 95, 95}));
  connect(fixedRL.frame_b, groundRL.frame_a) annotation(
    Line(points = {{-120, -50}, {-110, -50}}, color = {95, 95, 95}));
  connect(fixedRR.frame_b, groundRR.frame_a) annotation(
    Line(points = {{120, -50}, {110, -50}}, color = {95, 95, 95}));
  connect(curvErrorExpression.y, curvPI.u) annotation(
    Line(points = {{-98, 110}, {-82, 110}}, color = {0, 0, 127}));
  connect(curvPI.y, frSteerPosition.phi_ref) annotation(
    Line(points = {{-58, 110}, {-42, 110}}, color = {0, 0, 127}));
  connect(speedPI.y, vehicle.uPTNTorque) annotation(
    Line(points = {{-18, -70}, {0, -70}, {0, -34}}, color = {0, 0, 127}));
  connect(vehicle.frameRL, groundRL.frame_b) annotation(
    Line(points = {{-44, -22}, {-100, -22}, {-100, -40}}, color = {95, 95, 95}));
  connect(groundRR.frame_b, vehicle.frameRR) annotation(
    Line(points = {{100, -40}, {100, -22}, {46, -22}}, color = {95, 95, 95}));
  connect(vehicle.frameFL, groundFL.frame_b) annotation(
    Line(points = {{-44, 38}, {-100, 38}, {-100, 20}}, color = {95, 95, 95}));
  connect(vehicle.frameFR, groundFR.frame_b) annotation(
    Line(points = {{46, 38}, {100, 38}, {100, 20}}, color = {95, 95, 95}));
  connect(frSteerPosition.flange, vehicle.steerFlange) annotation(
    Line(points = {{-20, 110}, {0, 110}, {0, 62}}));
  connect(cgFreeMotion.frame_b, vehicle.cgFrame) annotation(
    Line(points = {{90, 90}, {70, 90}, {70, 20}, {46, 20}}, color = {95, 95, 95}));
  connect(world.frame_b, sprungAngles.frame_a) annotation(
    Line(points = {{-120, -110}, {50, -110}, {50, -80}}, color = {95, 95, 95}));
  connect(sprungAngles.frame_b, vehicle.cgFrame) annotation(
    Line(points = {{50, -60}, {50, 20}, {46, 20}}, color = {95, 95, 95}));
  annotation(
    Diagram(coordinateSystem(extent = {{-140, -120}, {140, 120}})),
    Icon(coordinateSystem(extent = {{-140, -120}, {140, 120}})),
  experiment(StartTime = 0, StopTime = 5, Tolerance = 1e-06, Interval = 0.002),
  __OpenModelica_commandLineOptions = "--matchingAlgorithm=PFPlusExt --indexReductionMethod=dynamicStateSelection -d=initialization,NLSanalyticJacobian --maxSizeLinearTearing=5000");
end ISO4138;
