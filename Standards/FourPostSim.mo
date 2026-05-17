within BobLib.Standards;

model FourPostSim
  import Modelica.SIunits;
  import Modelica.Constants.pi;
  import Modelica.Math.Vectors.norm;
  import Modelica.Mechanics.MultiBody.Frames;
  import BobLib.Utilities.Math.Vector;

  // Import vehicle records
  import BobLib.Vehicle.Vehicle_DWBCStabar_DWBCStabar;
  import BobLib.Vehicle.Chassis.Suspension.FrAxleDW_BC_Stabar;
  import BobLib.Vehicle.Chassis.Suspension.RrAxleDW_BC_Stabar;
  import BobLib.Vehicle.Chassis.Suspension.Templates.Tire;
  import BobLib.Vehicle.Chassis;
  
  import BobLib.Resources.VehicleRecord.Chassis.Suspension.Templates.Tire.Templates.PartialWheelRecord;
  import BobLib.Resources.VehicleDefn.DWBCStabar_DWBCStabarRecord;

  inner parameter SIunits.Length linkDiameter = 0.020;
  inner parameter SIunits.Length jointDiameter = 0.030;
  
  parameter DWBCStabar_DWBCStabarRecord pVehicle;
  
  Vehicle_DWBCStabar_DWBCStabar vehicle(pVehicle = pVehicle,
                                        redeclare Chassis.Chassis_LockRrSteer chassis(pSprungMass = pVehicle.pSprungMass,
                                                                                      pVehicleCG = vehicle.pVehicleCG,
                                                                                      redeclare FrAxleDW_BC_Stabar frAxleDW(pAxle = pVehicle.pFrAxleDW,
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
                                                                                                                                                             redeclare Tire.MF52.SlipModel.TransientSlip slipModel)),
                                                                                      redeclare RrAxleDW_BC_Stabar rrAxleDW(pAxle = pVehicle.pRrAxleDW,
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
                                                                                                                                                             redeclare Tire.MF52.SlipModel.TransientSlip slipModel)),
                                                                                      redeclare Chassis.Body.FrameRigid spaceFrame(frRef = vehicle.chassis.frAxleDW.effectiveCenter,
                                                                                                                                   rrRef = vehicle.chassis.rrAxleDW.effectiveCenter))) annotation(
    Placement(transformation(extent = {{-45, -50}, {45, 50}})));
  Modelica.Blocks.Sources.Constant const(k = 0) annotation(
    Placement(transformation(origin = {-30, -70}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.Fixed fixed annotation(
    Placement(transformation(origin = {-50, 50}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.Rotational.Sources.Position steerPosition annotation(
    Placement(transformation(origin = {-20, 90}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.RealExpression steerExpression(y = 0)  annotation(
    Placement(transformation(origin = {-50, 90}, extent = {{-10, -10}, {10, 10}})));

  Utilities.Mechanics.Multibody.ContactPatchFixture FL_fixture annotation(
    Placement(transformation(origin = {-60, 0}, extent = {{-10, -10}, {10, 10}})));
  Utilities.Mechanics.Multibody.ContactPatchFixture FR_fixture annotation(
    Placement(transformation(origin = {60, 0}, extent = {{-10, -10}, {10, 10}})));
  Utilities.Mechanics.Multibody.ContactPatchFixture RL_fixture annotation(
    Placement(transformation(origin = {-60, -60}, extent = {{-10, -10}, {10, 10}})));
  Utilities.Mechanics.Multibody.ContactPatchFixture RR_fixture annotation(
    Placement(transformation(origin = {60, -60}, extent = {{-10, -10}, {10, 10}})));

equation
  connect(const.y, vehicle.uPTNTorque) annotation(
    Line(points = {{-18, -70}, {0, -70}, {0, -54}}, color = {0, 0, 127}));
  connect(fixed.frame_b, vehicle.frAxleFrame) annotation(
    Line(points = {{-40, 50}, {-20, 50}, {-20, 30}, {0, 30}}, color = {95, 95, 95}));
  connect(steerPosition.flange, vehicle.steerFlange) annotation(
    Line(points = {{-10, 90}, {0, 90}, {0, 42}}));
  connect(steerExpression.y, steerPosition.phi_ref) annotation(
    Line(points = {{-38, 90}, {-32, 90}}, color = {0, 0, 127}));
  connect(vehicle.frameFL, FL_fixture.frame_a) annotation(
    Line(points = {{-44, 18}, {-60, 18}, {-60, 2}}, color = {95, 95, 95}));
  connect(vehicle.frameFR, FR_fixture.frame_a) annotation(
    Line(points = {{46, 18}, {60, 18}, {60, 2}}, color = {95, 95, 95}));
  connect(vehicle.frameRL, RL_fixture.frame_a) annotation(
    Line(points = {{-44, -42}, {-60, -42}, {-60, -58}}, color = {95, 95, 95}));
  connect(vehicle.frameRR, RR_fixture.frame_a) annotation(
    Line(points = {{46, -42}, {60, -42}, {60, -58}}, color = {95, 95, 95}));
end FourPostSim;