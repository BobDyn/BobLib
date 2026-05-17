within BobLib.Standards;

model FourPostSim
  import Modelica.SIunits;
  import Modelica.Constants.pi;
  import Modelica.Math.Vectors.norm;
  import Modelica.Mechanics.MultiBody.Frames;
  import BobLib.Utilities.Math.Vector;

  // Import vehicle records
  import BobLib.Vehicle.Chassis.Suspension.FrAxleDW_BC_Stabar;
  import BobLib.Vehicle.Chassis.Suspension.RrAxleDW_BC_Stabar;
  import BobLib.Vehicle.Chassis.Suspension.Templates.Tire;
  
  import BobLib.Resources.VehicleDefn.DWBCStabar_DWBCStabarRecord;

  inner parameter SIunits.Length linkDiameter = 0.020;
  inner parameter SIunits.Length jointDiameter = 0.030;
  
  parameter DWBCStabar_DWBCStabarRecord pVehicle;
  
  FrAxleDW_BC_Stabar frAxleDW(pAxle = pVehicle.pFrAxleDW,
                              pRack = pVehicle.pFrRack,
                              pStabar = pVehicle.pFrStabar,
                              pLeftPartialWheel = pVehicle.pFrPartialWheel,
                              pLeftDW = pVehicle.pFrDW,
                              pLeftAxleMass = pVehicle.pFrAxleMass,
                              redeclare Tire.BaseTire leftTire(pPartialWheel = pVehicle.pFrPartialWheel,
                                                               redeclare Tire.TirePhysics.Wheel0DOF wheelModel(partialWheelParams = pVehicle.pFrPartialWheel),
                                                               redeclare Tire.MF52.SlipModel.TransientSlip slipModel),
                              redeclare Tire.BaseTire rightTire(pPartialWheel = pVehicle.pFrPartialWheel,
                                                               redeclare Tire.TirePhysics.Wheel0DOF wheelModel(partialWheelParams = pVehicle.pFrPartialWheel),
                                                               redeclare Tire.MF52.SlipModel.TransientSlip slipModel)) annotation(
Placement(transformation(origin = {0.25, 32.4444}, extent = {{-37.25, -16.5556}, {37.25, 16.5556}})));

  RrAxleDW_BC_Stabar rrAxleDW(pAxle = pVehicle.pRrAxleDW,
                              pRack = pVehicle.pRrRack,
                              pStabar = pVehicle.pRrStabar,
                              pLeftPartialWheel = pVehicle.pRrPartialWheel,
                              pLeftDW = pVehicle.pRrDW,
                              pLeftAxleMass = pVehicle.pRrAxleMass,
                              redeclare Tire.BaseTire leftTire(pPartialWheel = pVehicle.pRrPartialWheel,
                                                               redeclare Tire.TirePhysics.Wheel0DOF wheelModel(partialWheelParams = pVehicle.pRrPartialWheel),
                                                               redeclare Tire.MF52.SlipModel.TransientSlip slipModel),
                              redeclare Tire.BaseTire rightTire(pPartialWheel = pVehicle.pRrPartialWheel,
                                                               redeclare Tire.TirePhysics.Wheel0DOF wheelModel(partialWheelParams = pVehicle.pRrPartialWheel),
                                                               redeclare Tire.MF52.SlipModel.TransientSlip slipModel)) annotation(
    Placement(transformation(origin = {-0.143, -29.3333}, extent = {{-36.8571, -14.3333}, {36.8571, 14.3333}})));
  Modelica.Mechanics.Rotational.Sources.Position steerPosition annotation(
    Placement(transformation(origin = {-50, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.RealExpression steerExpression(y = 0)  annotation(
    Placement(transformation(origin = {-80, 0}, extent = {{-10, -10}, {10, 10}})));

  Utilities.Mechanics.Multibody.ContactPatchFixture FL_fixture(CP_init = cpInitFL)  annotation(
    Placement(transformation(origin = {-50, 20}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Utilities.Mechanics.Multibody.ContactPatchFixture FR_fixture(CP_init = cpInitFR)  annotation(
    Placement(transformation(origin = {50, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Utilities.Mechanics.Multibody.ContactPatchFixture RL_fixture(CP_init = cpInitRL)  annotation(
    Placement(transformation(origin = {-50, -40}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Utilities.Mechanics.Multibody.ContactPatchFixture RR_fixture(CP_init = cpInitRR)  annotation(
    Placement(transformation(origin = {50, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  
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

  Modelica.Mechanics.MultiBody.Parts.Fixed frAxleFixed(r = frAxleDW.effectiveCenter) annotation(
    Placement(transformation(origin = {0, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.Fixed rrAxleFixed(r = rrAxleDW.effectiveCenter) annotation(
    Placement(transformation(origin = {0, -10}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));

equation
  connect(FL_fixture.frame_a, frAxleDW.leftCP) annotation(
    Line(points = {{-48, 20}, {-36, 20}}, color = {95, 95, 95}));
  connect(FR_fixture.frame_a, frAxleDW.rightCP) annotation(
    Line(points = {{48, 20}, {38, 20}}, color = {95, 95, 95}));
  connect(RL_fixture.frame_a, rrAxleDW.leftCP) annotation(
    Line(points = {{-48, -40}, {-38, -40}}, color = {95, 95, 95}));
  connect(RR_fixture.frame_a, rrAxleDW.rightCP) annotation(
    Line(points = {{48, -40}, {36, -40}}, color = {95, 95, 95}));
  connect(steerExpression.y, steerPosition.phi_ref) annotation(
    Line(points = {{-68, 0}, {-62, 0}}, color = {0, 0, 127}));
  connect(steerPosition.flange, frAxleDW.steerFlange) annotation(
    Line(points = {{-40, 0}, {-20, 0}, {-20, 40}, {0, 40}}));
  connect(steerPosition.flange, rrAxleDW.steerFlange) annotation(
    Line(points = {{-40, 0}, {-20, 0}, {-20, -34}, {0, -34}}));
  connect(frAxleDW.axleFrame, frAxleFixed.frame_b) annotation(
    Line(points = {{0, 30}, {0, 20}}, color = {95, 95, 95}));
  connect(rrAxleFixed.frame_b, rrAxleDW.axleFrame) annotation(
    Line(points = {{0, -20}, {0, -30}}, color = {95, 95, 95}));
end FourPostSim;
