within BobLib.Tests.TestVehicle.TestChassis.TestSuspension.TestTemplates.TestDoubleWishbone;

model TestDoubleWishbone
  import BobLib.Resources.VehicleDefn.OrionRecord;
  
  parameter OrionRecord pVehicle;
  
  parameter Real linkDiameter = 0.020;
  parameter Real jointDiameter = 0.030;
  
  inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1}) annotation(
    Placement(transformation(origin = {-90, -90}, extent = {{-10, -10}, {10, 10}})));
  
  // Steer input
  Modelica.Blocks.Sources.Ramp steerRamp(duration = 1,
                                         height = 100*Modelica.Constants.pi/180,
                                         startTime = 1) annotation(
    Placement(transformation(origin = {-20, 80}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.Rotational.Sources.Position steerjouncePosition(exact = true) annotation(
    Placement(transformation(origin = {20, 80}, extent = {{-10, -10}, {10, 10}})));
  
  // Jounce input
  Modelica.Mechanics.MultiBody.Parts.Fixed jounceRef(r = pVehicle.pFrDW.lower_o, animation = false) annotation(
    Placement(transformation(origin = {90, -90}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
  Modelica.Blocks.Sources.Ramp jounceRamp(height = 2*0.0254,
                                          duration = 1,
                                          startTime = 1) annotation(
    Placement(transformation(origin = {-90, -50}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.Translational.Sources.Position jouncePosition(useSupport = true, exact = true) annotation(
    Placement(transformation(origin = {-50, -50}, extent = {{-10, -10}, {10, 10}})));
  
  // Inboard fixtures
  Modelica.Mechanics.MultiBody.Parts.Fixed upperFixed_i(r = (pVehicle.pFrDW.upperFore_i + pVehicle.pFrDW.upperAft_i) / 2, animation = false) annotation(
    Placement(transformation(origin = {90, 10}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Fixed lowerFixed_i(r = (pVehicle.pFrDW.lowerFore_i + pVehicle.pFrDW.lowerAft_i) / 2, animation = false) annotation(
    Placement(transformation(origin = {90, -30}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Fixed rackFixed(r = {pVehicle.pRack.leftPickup[1], 0, pVehicle.pRack.leftPickup[3]}, animation = false) annotation(
    Placement(transformation(origin = {90, 50}, extent = {{-10, -10}, {10, 10}}, rotation = -180)));
  
  // Rack
  BobLib.Vehicle.Chassis.Suspension.Templates.SteeringRack.RackAndPinion rackAndPinion(pRack = pVehicle.pRack,
                                                                                       linkDiameter = linkDiameter)  annotation(
    Placement(transformation(origin = {50, 60}, extent = {{-20, -20}, {20, 20}})));
  
  // Kinematic wishbone-upright loop
  BobLib.Vehicle.Chassis.Suspension.Templates.DoubleWishbone.WishboneUprightLoop wishboneUprightLoop(pDW = pVehicle.pFrDW,
                                                                                                     linkDiameter = linkDiameter,
                                                                                                     jointDiameter = jointDiameter) annotation(
    Placement(transformation(origin = {0, 10}, extent = {{30, -30}, {-30, 30}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation tieConnection(r = pVehicle.pFrDW.lower_o - pVehicle.pFrDW.tie_o) annotation(
    Placement(transformation(origin = {-40, 10}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  
  // Tie rod
  BobLib.Vehicle.Chassis.Suspension.Linkages.Rod tieRod(r_a = pVehicle.pRack.leftPickup,
                                                        r_b = pVehicle.pFrDW.tie_o,
                                                        n1_a = {1, 0, 0},
                                                        linkDiameter = linkDiameter,
                                                        jointDiameter = jointDiameter)  annotation(
    Placement(transformation(origin = {-10, 60}, extent = {{20, -20}, {-20, 20}}, rotation = -0)));
  
protected
  // Jounce DOFs
  Modelica.Mechanics.MultiBody.Joints.Prismatic DOF_x(n = {1, 0, 0}, animation = false) annotation(
    Placement(transformation(origin = {50, -90}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
  Modelica.Mechanics.MultiBody.Joints.Prismatic DOF_y(n = {0, 1, 0}, animation = false) annotation(
    Placement(transformation(origin = {20, -90}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));
  Modelica.Mechanics.MultiBody.Joints.Prismatic DOF_z(n = {0, 0, 1}, useAxisFlange = true, animation = false) annotation(
    Placement(transformation(origin = {0, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Joints.Spherical DOF_xyz(animation = false) annotation(
    Placement(transformation(origin = {0, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  
equation
  connect(tieConnection.frame_b, wishboneUprightLoop.steeringFrame) annotation(
    Line(points = {{-40, 0}, {-40, -11.375}, {-30, -11.375}, {-30, -11}}, color = {95, 95, 95}));
  connect(lowerFixed_i.frame_b, wishboneUprightLoop.lowerFrame_i) annotation(
    Line(points = {{80, -30}, {46, -30}, {46, -11}, {30, -11}}, color = {95, 95, 95}));
  connect(upperFixed_i.frame_b, wishboneUprightLoop.upperFrame_i) annotation(
    Line(points = {{80, 10}, {45, 10}, {45, 31}, {30, 31}}, color = {95, 95, 95}));
  connect(DOF_xyz.frame_b, wishboneUprightLoop.lowerFrame_o) annotation(
    Line(points = {{0, -30}, {0, -20}}, color = {95, 95, 95}));
  connect(DOF_z.frame_b, DOF_xyz.frame_a) annotation(
    Line(points = {{0, -60}, {0, -50}}, color = {95, 95, 95}));
  connect(jouncePosition.flange, DOF_z.axis) annotation(
    Line(points = {{-40, -50}, {-30, -50}, {-30, -62}, {-6, -62}}, color = {0, 127, 0}));
  connect(jounceRamp.y, jouncePosition.s_ref) annotation(
    Line(points = {{-79, -50}, {-62, -50}}, color = {0, 0, 127}));
  connect(jounceRef.frame_b, DOF_x.frame_a) annotation(
    Line(points = {{80, -90}, {60, -90}}, color = {95, 95, 95}));
  connect(DOF_x.frame_b, DOF_y.frame_a) annotation(
    Line(points = {{40, -90}, {30, -90}}, color = {95, 95, 95}));
  connect(DOF_y.frame_b, DOF_z.frame_a) annotation(
    Line(points = {{10, -90}, {0, -90}, {0, -80}}, color = {95, 95, 95}));
  connect(jouncePosition.support, DOF_z.support) annotation(
    Line(points = {{-50, -60}, {-50, -74}, {-6, -74}}, color = {0, 127, 0}));
  connect(rackFixed.frame_b, rackAndPinion.mountFrame) annotation(
    Line(points = {{80, 50}, {50, 50}, {50, 56}}, color = {95, 95, 95}));
  connect(steerRamp.y, steerjouncePosition.phi_ref) annotation(
    Line(points = {{-9, 80}, {7, 80}}, color = {0, 0, 127}));
  connect(steerjouncePosition.flange, rackAndPinion.pinionFlange) annotation(
    Line(points = {{30, 80}, {50, 80}, {50, 64}}));
  connect(rackAndPinion.leftFrame, tieRod.frame_a) annotation(
    Line(points = {{30, 60}, {10, 60}}, color = {95, 95, 95}));
  connect(tieRod.frame_b, tieConnection.frame_a) annotation(
    Line(points = {{-30, 60}, {-40, 60}, {-40, 20}}, color = {95, 95, 95}));
  annotation(
    experiment(StartTime = 0, StopTime = 3, Tolerance = 1e-06, Interval = 0.002),
    __OpenModelica_commandLineOptions = "--matchingAlgorithm=PFPlusExt --indexReductionMethod=dynamicStateSelection -d=initialization,NLSanalyticJacobian");
end TestDoubleWishbone;
