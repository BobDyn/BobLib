within BobLibTest.Chassis.Suspension.Components;

model TestDoubleWishbone

  import BobLib.Records.VehicleDefn.EVBatInvMotDiff_DWBCStabar_DWBCStabarRecord;

  parameter EVBatInvMotDiff_DWBCStabar_DWBCStabarRecord pVehicle "Complete vehicle calibration record; subsystem records are passed to the selected models" annotation(Dialog(group = "Calibration"));

  inner parameter Real linkDiameter = 0.020 "Diameter of links in the MultiBody animation; does not set structural stiffness" annotation(Dialog(group = "Animation"));
  inner parameter Real jointDiameter = 0.030 "Diameter of joints in the MultiBody animation; does not set joint geometry" annotation(Dialog(group = "Animation"));
  inner parameter Boolean headless = false "Disable MultiBody animation geometry for simulation without visualization" annotation(Dialog(group = "Animation"), choices(checkBox = true));

  inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1}) annotation(
    Placement(transformation(origin = {-170, -110}, extent = {{-10, -10}, {10, 10}})));

  // Steer input
  Modelica.Blocks.Sources.Ramp steerRamp(duration = 1,
                                         height = 100*Modelica.Constants.pi/180,
                                         startTime = 1) annotation(
    Placement(transformation(origin = {-65, 110}, extent = {{-8, -8}, {8, 8}}, rotation = 0)));

  Modelica.Mechanics.Rotational.Sources.Position steerjouncePosition(exact = true) annotation(
    Placement(transformation(origin = {40, 110}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));

  // Jounce input
  Modelica.Mechanics.MultiBody.Parts.Fixed jounceRef(
    r = pVehicle.pFrDW.lower_o,
    animation = false) annotation(
    Placement(transformation(origin = {90, -90}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));

  Modelica.Blocks.Sources.Ramp jounceRamp(height = 2*0.0254,
                                          duration = 1,
                                          startTime = 1) annotation(
    Placement(transformation(origin = {-90, -50}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.Translational.Sources.Position jouncePosition(useSupport = true, exact = true) annotation(
    Placement(transformation(origin = {-50, -50}, extent = {{-10, -10}, {10, 10}})));

  // Inboard fixtures
  Modelica.Mechanics.MultiBody.Parts.Fixed upperFixed_i(
    r = (pVehicle.pFrDW.upperFore_i + pVehicle.pFrDW.upperAft_i) / 2,
    animation = false) annotation(
    Placement(transformation(origin = {90, 10}, extent = {{10, -10}, {-10, 10}})));

  Modelica.Mechanics.MultiBody.Parts.Fixed lowerFixed_i(
    r = (pVehicle.pFrDW.lowerFore_i + pVehicle.pFrDW.lowerAft_i) / 2,
    animation = false) annotation(
    Placement(transformation(origin = {90, -30}, extent = {{10, -10}, {-10, 10}})));

  Modelica.Mechanics.MultiBody.Parts.Fixed rackFixed(
    r = {pVehicle.pFrRack.leftPickup[1], 0, pVehicle.pFrRack.leftPickup[3]},
    animation = false) annotation(
    Placement(transformation(origin = {90, 50}, extent = {{-10, -10}, {10, 10}}, rotation = -180)));

  // Rack
  BobLib.Chassis.Suspension.Components.RackAndPinion rackAndPinion(pRack = pVehicle.pFrRack,
                                                                                       linkDiameter = linkDiameter) annotation(
    Placement(transformation(origin = {50, 60}, extent = {{-20, -20}, {20, 20}})));

  // Kinematic wishbone-upright loop
  BobLib.Chassis.Suspension.Components.WishboneUprightLoop wishboneUprightLoop(pDW = pVehicle.pFrDW,
                                                                                                     linkDiameter = linkDiameter,
                                                                                                     jointDiameter = jointDiameter) annotation(
    Placement(transformation(origin = {0, 10}, extent = {{30, -30}, {-30, 30}})));

  Modelica.Mechanics.MultiBody.Parts.FixedTranslation tieConnection(r = pVehicle.pFrDW.lower_o - pVehicle.pFrDW.tie_o) annotation(
    Placement(transformation(origin = {-40, 10}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));

  // Tie rod
  BobLib.Chassis.Suspension.Components.Rod tieRod(r_a = pVehicle.pFrRack.leftPickup,
                                                        r_b = pVehicle.pFrDW.tie_o,
                                                        n1_a = {1, 0, 0},
                                                        linkDiameter = linkDiameter,
                                                        jointDiameter = jointDiameter) annotation(
    Placement(transformation(origin = {-10, 60}, extent = {{20, -20}, {-20, 20}}, rotation = -0)));

protected

  // Jounce DOFs
  Modelica.Mechanics.MultiBody.Joints.Prismatic DOF_x(n = {1, 0, 0}, animation = false) annotation(
    Placement(transformation(origin = {50, -90}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));

  Modelica.Mechanics.MultiBody.Joints.Prismatic DOF_y(n = {0, 1, 0}, animation = false) annotation(
    Placement(transformation(origin = {20, -90}, extent = {{10, -10}, {-10, 10}}, rotation = -0)));

  Modelica.Mechanics.MultiBody.Joints.Prismatic DOF_z(
    n = {0, 0, 1},
    useAxisFlange = true,
    animation = false) annotation(
    Placement(transformation(origin = {0, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));

  Modelica.Mechanics.MultiBody.Joints.Spherical DOF_xyz(animation = false) annotation(
    Placement(transformation(origin = {0, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));

equation
  connect(tieConnection.frame_b, wishboneUprightLoop.steeringFrame) annotation(
    Line(points = {{-40, 0}, {-40, -11}, {-30, -11}}, color = {95, 95, 95}));
  connect(lowerFixed_i.frame_b, wishboneUprightLoop.lowerFrame_i) annotation(
    Line(points = {{80, -30}, {77, -30}, {77, -11}, {30, -11}}, color = {95, 95, 95}));
  connect(upperFixed_i.frame_b, wishboneUprightLoop.upperFrame_i) annotation(
    Line(points = {{80, 10}, {77, 10}, {77, 31}, {30, 31}}, color = {95, 95, 95}));
  connect(DOF_xyz.frame_b, wishboneUprightLoop.lowerFrame_o) annotation(
    Line(points = {{0, -30}, {0, -20}}, color = {95, 95, 95}));
  connect(DOF_z.frame_b, DOF_xyz.frame_a) annotation(
    Line(points = {{0, -60}, {-0, -50}}, color = {95, 95, 95}));
  connect(jouncePosition.flange, DOF_z.axis) annotation(
    Line(points = {{-40, -50}, {-37, -50}, {-37, -54}, {-6, -54}, {-6, -62}}, color = {0, 127, 0}));
  connect(jounceRamp.y, jouncePosition.s_ref) annotation(
    Line(points = {{-79, -50}, {-62, -50}}, color = {0, 0, 127}));
  connect(jounceRef.frame_b, DOF_x.frame_a) annotation(
    Line(points = {{80, -90}, {60, -90}}, color = {95, 95, 95}));
  connect(DOF_x.frame_b, DOF_y.frame_a) annotation(
    Line(points = {{40, -90}, {30, -90}}, color = {95, 95, 95}));
  connect(DOF_y.frame_b, DOF_z.frame_a) annotation(
    Line(points = {{10, -90}, {7, -90}, {7, -84}, {0, -84}, {-0, -80}}, color = {95, 95, 95}));
  connect(jouncePosition.support, DOF_z.support) annotation(
    Line(points = {{-50, -60}, {-50, -74}, {-6, -74}}, color = {0, 127, 0}));
  connect(rackFixed.frame_b, rackAndPinion.mountFrame) annotation(
    Line(points = {{80, 50}, {77, 50}, {77, 36}, {50, 36}, {50, 56}}, color = {95, 95, 95}));
  connect(steerRamp.y, steerjouncePosition.phi_ref) annotation(
    Line(points = {{-56.2, 110}, {28, 110}}, color = {0, 0, 127}));
  connect(steerjouncePosition.flange, rackAndPinion.pinionFlange) annotation(
    Line(points = {{50, 110}, {50, 64}}, color = {0, 0, 0}));
  connect(rackAndPinion.leftFrame, tieRod.frame_a) annotation(
    Line(points = {{30, 60}, {10, 60}}, color = {95, 95, 95}));
  connect(tieRod.frame_b, tieConnection.frame_a) annotation(
    Line(points = {{-30, 60}, {-33, 60}, {-33, 26}, {-40, 26}, {-40, 20}}, color = {95, 95, 95}));
  assert(abs(Modelica.Math.Vectors.norm(tieRod.frame_b.r_0 - tieRod.frame_a.r_0) -
    Modelica.Math.Vectors.norm(pVehicle.pFrDW.tie_o -
      pVehicle.pFrRack.leftPickup)) < 1e-7,
    "Steering and jounce must preserve the rigid tie-rod length");
  assert(abs(wishboneUprightLoop.lowerFrame_o.r_0[3] -
    pVehicle.pFrDW.lower_o[3] - jounceRamp.y) < 1e-7,
    "The lower outboard pickup must follow the prescribed vertical jounce");

  annotation(
    experiment(StartTime = 0, StopTime = 3, Tolerance = 1e-06, Interval = 0.002),
    __OpenModelica_commandLineOptions = "--matchingAlgorithm=PFPlusExt --indexReductionMethod=dynamicStateSelection -d=initialization,NLSanalyticJacobian");
  annotation(Diagram(coordinateSystem(extent = {{-180, -120}, {180, 140}}, preserveAspectRatio = true)));
end TestDoubleWishbone;
