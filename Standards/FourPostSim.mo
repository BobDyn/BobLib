within BobLib.Standards;

model FourPostSim
  import Modelica.SIunits;
  import Modelica.Constants.pi;
  import Modelica.Math.Vectors.norm;
  import Modelica.Mechanics.MultiBody.Frames;
  import BobLib.Utilities.Math.Vector;
  
  // KnC record
  import BobLib.Resources.StandardRecord.KnCRecord;
    
  // Suspension axle models
  import BobLib.Vehicle.Chassis.Suspension.FrAxleDW_BC_Stabar;
  import BobLib.Vehicle.Chassis.Suspension.RrAxleDW_BC_Stabar;
  
  // Axle record to override rates
  import BobLib.Resources.VehicleRecord.Chassis.Suspension.AxleDW_BC_StabarRecord;
  
  // Bar record to override rate
  import BobLib.Resources.VehicleRecord.Chassis.Suspension.Templates.Stabar.StabarRecord;
  
  // Tire record
  import BobLib.Vehicle.Chassis.Suspension.Templates.Tire;
  
  // Vehicle record
  import BobLib.Resources.VehicleDefn.DWBCStabar_DWBCStabarRecord;
  
  inner parameter SIunits.Length linkDiameter = 0.020;
  inner parameter SIunits.Length jointDiameter = 0.030;
  
  parameter DWBCStabar_DWBCStabarRecord pVehicle;
  
  parameter SIunits.Angle steerMagnitude = 0 "Maximum pinion angle magnitude" annotation(
    Dialog(group = "Test Parameters"));
  parameter SIunits.Length heaveMagnitude = 1.5*0.0254 "Maximum heave magnitude" annotation(
    Dialog(group = "Test Parameters"));
  parameter SIunits.Angle rollMagnitude = 2*Modelica.Constants.pi/180 "Maximum roll magnitude" annotation(
    Dialog(group = "Test Parameters"));
  parameter SIunits.Force forceMagnitude = 1000 "Maximum contact patch force" annotation(
    Placement(visible = false, transformation(origin = {nan, nan}, extent = {{nan, nan}, {nan, nan}})));
  
  // Time-value tables [time, normalized value]
  final parameter Real rollTable[:, 2] = [0, 0; 1, 0; 2, 0; 3, 0; 4, 0; 5, 0; 6, 0; 7, 0; 8, 0; 9, 0; 10, 0; 11, 0; 12, 0; 13, 0; 14, 0; 15, 0; 16, 0; 17, 0; 18, 0; 19, 0; 20, 0; 21, 0; 22, 0; 23, 0; 24, 0; 25, 0; 26, 0; 27, 0; 28, 0; 29, 0; 30, 0; 31, 0; 32, 0; 33, 0; 34, 0; 35, 0; 36, 0; 37, 0; 38, 0; 39, 0; 40, 0; 41, 0; 42, 0; 43, 0; 44, 0; 45, 0; 46, 0; 47, -1; 48, -1; 49, -1; 50, -1; 51, -0.8; 52, -0.8; 53, -0.8; 54, -0.8; 55, -0.6; 56, -0.6; 57, -0.6; 58, -0.6; 59, -0.4; 60, -0.4; 61, -0.4; 62, -0.4; 63, -0.2; 64, -0.2; 65, -0.2; 66, -0.2; 67, 0; 68, 0; 69, 0; 70, 0; 71, 0.2; 72, 0.2; 73, 0.2; 74, 0.2; 75, 0.4; 76, 0.4; 77, 0.4; 78, 0.4; 79, 0.6; 80, 0.6; 81, 0.6; 82, 0.6; 83, 0.8; 84, 0.8; 85, 0.8; 86, 0.8; 87, 1; 88, 1; 89, 1; 90, 1; 91, 0];
  final parameter Real heaveTable[:, 2] = [0, 0; 1, 0; 2, -1; 3, -1; 4, -1; 5, -1; 6, -0.8; 7, -0.8; 8, -0.8; 9, -0.8; 10, -0.6; 11, -0.6; 12, -0.6; 13, -0.6; 14, -0.4; 15, -0.4; 16, -0.4; 17, -0.4; 18, -0.2; 19, -0.2; 20, -0.2; 21, -0.2; 22, 0; 23, 0; 24, 0; 25, 0; 26, 0.2; 27, 0.2; 28, 0.2; 29, 0.2; 30, 0.4; 31, 0.4; 32, 0.4; 33, 0.4; 34, 0.6; 35, 0.6; 36, 0.6; 37, 0.6; 38, 0.8; 39, 0.8; 40, 0.8; 41, 0.8; 42, 1; 43, 1; 44, 1; 45, 1; 46, 0; 47, 0; 48, 0; 49, 0; 50, 0; 51, 0; 52, 0; 53, 0; 54, 0; 55, 0; 56, 0; 57, 0; 58, 0; 59, 0; 60, 0; 61, 0; 62, 0; 63, 0; 64, 0; 65, 0; 66, 0; 67, 0; 68, 0; 69, 0; 70, 0; 71, 0; 72, 0; 73, 0; 74, 0; 75, 0; 76, 0; 77, 0; 78, 0; 79, 0; 80, 0; 81, 0; 82, 0; 83, 0; 84, 0; 85, 0; 86, 0; 87, 0; 88, 0; 89, 0; 90, 0; 91, 0];
  final parameter Real fxTable[:, 2] = [0, 0; 1, 0; 2, 0; 3, 1; 4, 1; 5, 0; 6, 0; 7, 1; 8, 1; 9, 0; 10, 0; 11, 1; 12, 1; 13, 0; 14, 0; 15, 1; 16, 1; 17, 0; 18, 0; 19, 1; 20, 1; 21, 0; 22, 0; 23, 1; 24, 1; 25, 0; 26, 0; 27, 1; 28, 1; 29, 0; 30, 0; 31, 1; 32, 1; 33, 0; 34, 0; 35, 1; 36, 1; 37, 0; 38, 0; 39, 1; 40, 1; 41, 0; 42, 0; 43, 1; 44, 1; 45, 0; 46, 0; 47, 0; 48, 0; 49, 0; 50, 0; 51, 0; 52, 0; 53, 0; 54, 0; 55, 0; 56, 0; 57, 0; 58, 0; 59, 0; 60, 0; 61, 0; 62, 0; 63, 0; 64, 0; 65, 0; 66, 0; 67, 0; 68, 0; 69, 0; 70, 0; 71, 0; 72, 0; 73, 0; 74, 0; 75, 0; 76, 0; 77, 0; 78, 0; 79, 0; 80, 0; 81, 0; 82, 0; 83, 0; 84, 0; 85, 0; 86, 0; 87, 0; 88, 0; 89, 0; 90, 0; 91, 0];
  final parameter Real fyTable[:, 2] = [0, 0; 1, 0; 2, 0; 3, 0; 4, 0; 5, 0; 6, 0; 7, 0; 8, 0; 9, 0; 10, 0; 11, 0; 12, 0; 13, 0; 14, 0; 15, 0; 16, 0; 17, 0; 18, 0; 19, 0; 20, 0; 21, 0; 22, 0; 23, 0; 24, 0; 25, 0; 26, 0; 27, 0; 28, 0; 29, 0; 30, 0; 31, 0; 32, 0; 33, 0; 34, 0; 35, 0; 36, 0; 37, 0; 38, 0; 39, 0; 40, 0; 41, 0; 42, 0; 43, 0; 44, 0; 45, 0; 46, 0; 47, 0; 48, 1; 49, 1; 50, 0; 51, 0; 52, 1; 53, 1; 54, 0; 55, 0; 56, 1; 57, 1; 58, 0; 59, 0; 60, 1; 61, 1; 62, 0; 63, 0; 64, 1; 65, 1; 66, 0; 67, 0; 68, 1; 69, 1; 70, 0; 71, 0; 72, 1; 73, 1; 74, 0; 75, 0; 76, 1; 77, 1; 78, 0; 79, 0; 80, 1; 81, 1; 82, 0; 83, 0; 84, 1; 85, 1; 86, 0; 87, 0; 88, 1; 89, 1; 90, 0; 91, 0];
  
  inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1}, g = 0) annotation(
    Placement(transformation(origin = {-90, -90}, extent = {{-10, -10}, {10, 10}})));
  
  final parameter AxleDW_BC_StabarRecord fr_pAxle(bellcrankPivot = pVehicle.pFrAxleDW.bellcrankPivot,
                                                                   bellcrankPivotAxis = pVehicle.pFrAxleDW.bellcrankPivotAxis,
                                                                   bellcrankRodPickup = pVehicle.pFrAxleDW.bellcrankRodPickup,
                                                                   bellcrankShockPickup = pVehicle.pFrAxleDW.bellcrankShockPickup,
                                                                   bellcrankStabarPickup = pVehicle.pFrAxleDW.bellcrankStabarPickup,
                                                                   rodPickup = pVehicle.pFrAxleDW.rodPickup,
                                                                   shockPickup = pVehicle.pFrAxleDW.shockPickup,
                                                                   stabarPickup = pVehicle.pFrAxleDW.stabarPickup,
                                                                   shockMount = pVehicle.pFrAxleDW.shockMount,
                                                                   springTable = [0, 0; 1, 0],
                                                                   springFreeLength = pVehicle.pFrAxleDW.springFreeLength,
                                                                   damperTable = [0, 0; 1, 0]);
  final parameter AxleDW_BC_StabarRecord rr_pAxle(bellcrankPivot = pVehicle.pRrAxleDW.bellcrankPivot,
                                                                   bellcrankPivotAxis = pVehicle.pRrAxleDW.bellcrankPivotAxis,
                                                                   bellcrankRodPickup = pVehicle.pRrAxleDW.bellcrankRodPickup,
                                                                   bellcrankShockPickup = pVehicle.pRrAxleDW.bellcrankShockPickup,
                                                                   bellcrankStabarPickup = pVehicle.pRrAxleDW.bellcrankStabarPickup,
                                                                   rodPickup = pVehicle.pRrAxleDW.rodPickup,
                                                                   shockPickup = pVehicle.pRrAxleDW.shockPickup,
                                                                   stabarPickup = pVehicle.pRrAxleDW.stabarPickup,
                                                                   shockMount = pVehicle.pRrAxleDW.shockMount,
                                                                   springTable = [0, 0; 1, 0],
                                                                   springFreeLength = pVehicle.pRrAxleDW.springFreeLength,
                                                                   damperTable = [0, 0; 1, 0]);
  
  // Front axle
  FrAxleDW_BC_Stabar frAxleDW(pAxle = fr_pAxle,
                              pRack = pVehicle.pFrRack,
                              pStabar = StabarRecord(leftBarEnd = pVehicle.pFrStabar.leftBarEnd,
                                                     leftArmEnd = pVehicle.pFrStabar.leftArmEnd,
                                                     barRate = 0),
                              pLeftPartialWheel = pVehicle.pFrPartialWheel,
                              pLeftDW = pVehicle.pFrDW,
                              pLeftAxleMass = pVehicle.pFrAxleMass,
                              redeclare Tire.BaseTire leftTire(pPartialWheel = pVehicle.pFrPartialWheel,
                                                               redeclare Tire.TirePhysics.Wheel0DOF wheelModel(partialWheelParams = pVehicle.pFrPartialWheel),
                                                               redeclare Tire.MF52.SlipModel.NoSlip slipModel),
                              redeclare Tire.BaseTire rightTire(pPartialWheel = pVehicle.pFrPartialWheel,
                                                               redeclare Tire.TirePhysics.Wheel0DOF wheelModel(partialWheelParams = pVehicle.pFrPartialWheel),
                                                               redeclare Tire.MF52.SlipModel.NoSlip slipModel)) annotation(
Placement(transformation(origin = {0.25, 52.4444}, extent = {{-37.25, -16.5556}, {37.25, 16.5556}})));
  
  // Rear axle
  RrAxleDW_BC_Stabar rrAxleDW(pAxle = rr_pAxle,
                              pRack = pVehicle.pRrRack,
                              pStabar = StabarRecord(leftBarEnd = pVehicle.pRrStabar.leftBarEnd,
                                                     leftArmEnd = pVehicle.pRrStabar.leftArmEnd,
                                                     barRate = 0),
                              pLeftPartialWheel = pVehicle.pRrPartialWheel,
                              pLeftDW = pVehicle.pRrDW,
                              pLeftAxleMass = pVehicle.pRrAxleMass,
                              redeclare Tire.BaseTire leftTire(pPartialWheel = pVehicle.pRrPartialWheel,
                                                               redeclare Tire.TirePhysics.Wheel0DOF wheelModel(partialWheelParams = pVehicle.pRrPartialWheel),
                                                               redeclare Tire.MF52.SlipModel.NoSlip slipModel),
                              redeclare Tire.BaseTire rightTire(pPartialWheel = pVehicle.pRrPartialWheel,
                                                               redeclare Tire.TirePhysics.Wheel0DOF wheelModel(partialWheelParams = pVehicle.pRrPartialWheel),
                                                               redeclare Tire.MF52.SlipModel.NoSlip slipModel)) annotation(
    Placement(transformation(origin = {-0.285728, -49.8887}, extent = {{-36.5715, -14.2222}, {36.5715, 14.2222}})));
  
  // Front chassis actuator
  Utilities.Mechanics.Multibody.ChassisActuator frChassisActuator(axleRef = frAxleDW.effectiveCenter)  annotation(
    Placement(transformation(origin = {0, 30}, extent = {{10, -10}, {-10, 10}})));
  
  // Rear chassis actuator
  Utilities.Mechanics.Multibody.ChassisActuator rrChassisActuator(axleRef = rrAxleDW.effectiveCenter)  annotation(
    Placement(transformation(origin = {0, -70}, extent = {{10, -10}, {-10, 10}})));
  
  // Front steer input
  Modelica.Mechanics.Rotational.Sources.Position steerPosition(exact = true)  annotation(
    Placement(transformation(origin = {-30, 80}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.Ramp steerExpression(height = steerMagnitude,
                                               duration = 1,
                                               startTime = 0) annotation(
    Placement(transformation(origin = {-70, 80}, extent = {{-10, -10}, {10, 10}})));

  // Rear steer lock
  Modelica.Mechanics.MultiBody.Parts.Mounting1D rrLock annotation(
    Placement(transformation(origin = {0, -30}, extent = {{10, -10}, {-10, 10}})));
  
  // Contact patch fixtures
  Utilities.Mechanics.Multibody.ContactPatchFixture FL_fixture(CP_init = cpInitFL)  annotation(
    Placement(transformation(origin = {-50, 40}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Utilities.Mechanics.Multibody.ContactPatchFixture FR_fixture(CP_init = cpInitFR)  annotation(
    Placement(transformation(origin = {50, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Utilities.Mechanics.Multibody.ContactPatchFixture RL_fixture(CP_init = cpInitRL)  annotation(
    Placement(transformation(origin = {-50, -60}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));
  Utilities.Mechanics.Multibody.ContactPatchFixture RR_fixture(CP_init = cpInitRR)  annotation(
    Placement(transformation(origin = {50, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  
  // Force actuators
  Utilities.Mechanics.Multibody.CP_ForceActuator FL_ForceActuator(fxTable = fxTable, fyTable = fyTable, forceMagnitude = forceMagnitude / 2)  annotation(
    Placement(transformation(origin = {-70, 30}, extent = {{-10, -10}, {10, 10}})));
  Utilities.Mechanics.Multibody.CP_ForceActuator FR_ForceActuator(fxTable = fxTable, fyTable = fyTable, forceMagnitude = forceMagnitude / 2)  annotation(
    Placement(transformation(origin = {70, 30}, extent = {{10, -10}, {-10, 10}})));
  Utilities.Mechanics.Multibody.CP_ForceActuator RL_ForceActuator(fxTable = fxTable, fyTable = fyTable, forceMagnitude = forceMagnitude / 2)  annotation(
    Placement(transformation(origin = {-70, -70}, extent = {{-10, -10}, {10, 10}})));
  Utilities.Mechanics.Multibody.CP_ForceActuator RR_ForceActuator(fxTable = fxTable, fyTable = fyTable, forceMagnitude = forceMagnitude / 2)  annotation(
    Placement(transformation(origin = {70, -70}, extent = {{10, -10}, {-10, 10}})));

  // Heave input
  Modelica.Blocks.Sources.CombiTimeTable heaveSource(table = heaveTable) annotation(
    Placement(transformation(origin = {90, 10}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Blocks.Math.Gain heaveGain(k = heaveMagnitude) annotation(
    Placement(transformation(origin = {65, 10}, extent = {{5, -5}, {-5, 5}})));

  // Roll input
  Modelica.Blocks.Sources.CombiTimeTable rollSource(table = rollTable) annotation(
    Placement(transformation(origin = {90, -20}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Blocks.Math.Gain rollGain(k = rollMagnitude) annotation(
    Placement(transformation(origin = {65, -20}, extent = {{5, -5}, {-5, 5}})));
  
  // KnC records
  KnCRecord frKnC;
  KnCRecord rrKnC;

protected
  // Front quantities
  Real frLeftDeltaVec[3];
  Real frLeftKingpinVec[3];
  Real frLeftGroundParam;
  Real frLeftGroundPoint[3];
  
  Real frRightDeltaVec[3];
  Real frRightKingpinVec[3];
  Real frRightGroundParam;
  Real frRightGroundPoint[3];
  
  // Rear quantities
  Real rrLeftDeltaVec[3];
  Real rrLeftKingpinVec[3];
  Real rrLeftGroundParam;
  Real rrLeftGroundPoint[3];
  
  Real rrRightDeltaVec[3];
  Real rrRightKingpinVec[3];
  Real rrRightGroundParam;
  Real rrRightGroundPoint[3];
  
protected
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
  
equation
  
  // Front quantities
  frKnC.leftGamma = frAxleDW.leftTire.gamma;

  frLeftDeltaVec = Frames.resolve1(frAxleDW.leftCP.R, {1, 0, 0});
  frKnC.leftToe = atan(frLeftDeltaVec[2]/frLeftDeltaVec[1]);

  frLeftKingpinVec = frAxleDW.leftWishboneUprightLoop.upperFrame_o.r_0 - frAxleDW.leftWishboneUprightLoop.lowerFrame_o.r_0;
  frKnC.leftCaster = atan(-1*frLeftKingpinVec[1]/frLeftKingpinVec[3]);
  frKnC.leftKpi = atan(-1*frLeftKingpinVec[2]/frLeftKingpinVec[3]);

  frLeftGroundParam = (frAxleDW.leftCP.r_0[3] - frAxleDW.leftWishboneUprightLoop.upperFrame_o.r_0[3])/frLeftKingpinVec[3];
  frLeftGroundPoint = frAxleDW.leftWishboneUprightLoop.upperFrame_o.r_0 + frLeftGroundParam*frLeftKingpinVec;
  frKnC.leftMechTrail = frLeftGroundPoint[1] - frAxleDW.leftCP.r_0[1];
  frKnC.leftMechScrub = frAxleDW.leftCP.r_0[2] - frLeftGroundPoint[2];

  frKnC.rightGamma = frAxleDW.rightTire.gamma;

  frRightDeltaVec = Frames.resolve1(frAxleDW.rightCP.R, {1, 0, 0});
  frKnC.rightToe = atan(frRightDeltaVec[2]/frRightDeltaVec[1]);

  frRightKingpinVec = frAxleDW.rightWishboneUprightLoop.upperFrame_o.r_0 - frAxleDW.rightWishboneUprightLoop.lowerFrame_o.r_0;
  frKnC.rightCaster = atan(-1*frRightKingpinVec[1]/frRightKingpinVec[3]);
  frKnC.rightKpi = atan(frRightKingpinVec[2]/frRightKingpinVec[3]);

  frRightGroundParam = (frAxleDW.rightCP.r_0[3] - frAxleDW.rightWishboneUprightLoop.upperFrame_o.r_0[3])/frRightKingpinVec[3];
  frRightGroundPoint = frAxleDW.rightWishboneUprightLoop.upperFrame_o.r_0 + frRightGroundParam*frRightKingpinVec;
  frKnC.rightMechTrail = frRightGroundPoint[1] - frAxleDW.rightCP.r_0[1];
  frKnC.rightMechScrub = frRightGroundPoint[2] - frAxleDW.rightCP.r_0[2];

  frKnC.leftSpringLength = frAxleDW.leftShockLinkage.lineForceWithMass.s;
  frKnC.rightSpringLength = frAxleDW.rightShockLinkage.lineForceWithMass.s;
  frKnC.stabarAngle = frAxleDW.stabar.spring.phi_rel;
  
  frKnC.jackingForce = frChassisActuator.jackingOutput;
  frKnC.heave = frChassisActuator.heaveInput;
  frKnC.roll = frChassisActuator.rollInput;
  frKnC.fx = FL_ForceActuator.worldForce.force[1] + FR_ForceActuator.worldForce.force[1];
  frKnC.fy = FL_ForceActuator.worldForce.force[2] + FR_ForceActuator.worldForce.force[2];
  
  // Rear quantities
  rrKnC.leftGamma = rrAxleDW.leftTire.gamma;

  rrLeftDeltaVec = Frames.resolve1(rrAxleDW.leftCP.R, {1, 0, 0});
  rrKnC.leftToe = atan(rrLeftDeltaVec[2]/rrLeftDeltaVec[1]);

  rrLeftKingpinVec = rrAxleDW.leftWishboneUprightLoop.upperFrame_o.r_0 - rrAxleDW.leftWishboneUprightLoop.lowerFrame_o.r_0;
  rrKnC.leftCaster = atan(-1*rrLeftKingpinVec[1]/rrLeftKingpinVec[3]);
  rrKnC.leftKpi = atan(-1*rrLeftKingpinVec[2]/rrLeftKingpinVec[3]);

  rrLeftGroundParam = (rrAxleDW.leftCP.r_0[3] - rrAxleDW.leftWishboneUprightLoop.upperFrame_o.r_0[3])/rrLeftKingpinVec[3];
  rrLeftGroundPoint = rrAxleDW.leftWishboneUprightLoop.upperFrame_o.r_0 + rrLeftGroundParam*rrLeftKingpinVec;
  rrKnC.leftMechTrail = rrLeftGroundPoint[1] - rrAxleDW.leftCP.r_0[1];
  rrKnC.leftMechScrub = rrAxleDW.leftCP.r_0[2] - rrLeftGroundPoint[2];

  rrKnC.rightGamma = rrAxleDW.rightTire.gamma;

  rrRightDeltaVec = Frames.resolve1(rrAxleDW.rightCP.R, {1, 0, 0});
  rrKnC.rightToe = atan(rrRightDeltaVec[2]/rrRightDeltaVec[1]);

  rrRightKingpinVec = rrAxleDW.rightWishboneUprightLoop.upperFrame_o.r_0 - rrAxleDW.rightWishboneUprightLoop.lowerFrame_o.r_0;
  rrKnC.rightCaster = atan(-1*rrRightKingpinVec[1]/rrRightKingpinVec[3]);
  rrKnC.rightKpi = atan(rrRightKingpinVec[2]/rrRightKingpinVec[3]);

  rrRightGroundParam = (rrAxleDW.rightCP.r_0[3] - rrAxleDW.rightWishboneUprightLoop.upperFrame_o.r_0[3])/rrRightKingpinVec[3];
  rrRightGroundPoint = rrAxleDW.rightWishboneUprightLoop.upperFrame_o.r_0 + rrRightGroundParam*rrRightKingpinVec;
  rrKnC.rightMechTrail = rrRightGroundPoint[1] - rrAxleDW.rightCP.r_0[1];
  rrKnC.rightMechScrub = rrRightGroundPoint[2] - rrAxleDW.rightCP.r_0[2];

  rrKnC.leftSpringLength = rrAxleDW.leftShockLinkage.lineForceWithMass.s;
  rrKnC.rightSpringLength = rrAxleDW.rightShockLinkage.lineForceWithMass.s;
  rrKnC.stabarAngle = rrAxleDW.stabar.spring.phi_rel;
  
  rrKnC.jackingForce = rrChassisActuator.jackingOutput;
  rrKnC.heave = rrChassisActuator.heaveInput;
  rrKnC.roll = rrChassisActuator.rollInput;
  rrKnC.fx = RL_ForceActuator.worldForce.force[1] + RR_ForceActuator.worldForce.force[1];
  rrKnC.fy = RL_ForceActuator.worldForce.force[2] + RR_ForceActuator.worldForce.force[2];
  
  connect(FL_fixture.frame_a, frAxleDW.leftCP) annotation(
    Line(points = {{-48, 40}, {-37, 40}}, color = {95, 95, 95}));
  connect(FR_fixture.frame_a, frAxleDW.rightCP) annotation(
    Line(points = {{48, 40}, {37.5, 40}}, color = {95, 95, 95}));
  connect(steerExpression.y, steerPosition.phi_ref) annotation(
    Line(points = {{-59, 80}, {-42, 80}}, color = {0, 0, 127}));
  connect(frAxleDW.axleFrame, frChassisActuator.chassisFrame) annotation(
    Line(points = {{0, 50}, {0, 38}}, color = {95, 95, 95}));
  connect(heaveSource.y[1], heaveGain.u) annotation(
    Line(points = {{79, 10}, {71, 10}}, color = {0, 0, 127}));
  connect(rollSource.y[1], rollGain.u) annotation(
    Line(points = {{79, -20}, {71, -20}}, color = {0, 0, 127}));
  connect(heaveGain.y, frChassisActuator.heaveInput) annotation(
    Line(points = {{59.5, 10}, {20, 10}, {20, 36}, {4, 36}}, color = {0, 0, 127}));
  connect(rollGain.y, frChassisActuator.rollInput) annotation(
    Line(points = {{59.5, -20}, {10, -20}, {10, 24}, {4, 24}}, color = {0, 0, 127}));
  connect(steerPosition.flange, frAxleDW.steerFlange) annotation(
    Line(points = {{-20, 80}, {0, 80}, {0, 60}}));
  connect(FL_ForceActuator.chassisFrame, frAxleDW.leftCP) annotation(
    Line(points = {{-60, 30}, {-36, 30}, {-36, 40}}, color = {95, 95, 95}));
  connect(FR_ForceActuator.chassisFrame, frAxleDW.rightCP) annotation(
    Line(points = {{60, 30}, {38, 30}, {38, 40}}, color = {95, 95, 95}));
  connect(rrLock.flange_b, rrAxleDW.steerFlange) annotation(
    Line(points = {{-10, -30}, {-16, -30}, {-16, -54}, {0, -54}}));
  connect(rrLock.frame_a, rrAxleDW.axleFrame) annotation(
    Line(points = {{0, -40}, {0, -50}}, color = {95, 95, 95}));
  connect(RL_fixture.frame_a, rrAxleDW.leftCP) annotation(
    Line(points = {{-48, -60}, {-36, -60}}, color = {95, 95, 95}));
  connect(RR_fixture.frame_a, rrAxleDW.rightCP) annotation(
    Line(points = {{48, -60}, {36, -60}}, color = {95, 95, 95}));
  connect(RL_ForceActuator.chassisFrame, rrAxleDW.leftCP) annotation(
    Line(points = {{-60, -70}, {-36, -70}, {-36, -60}}, color = {95, 95, 95}));
  connect(RR_ForceActuator.chassisFrame, rrAxleDW.rightCP) annotation(
    Line(points = {{60, -70}, {36, -70}, {36, -60}}, color = {95, 95, 95}));
  connect(rrChassisActuator.chassisFrame, rrAxleDW.axleFrame) annotation(
    Line(points = {{0, -62}, {0, -50}}, color = {95, 95, 95}));
  connect(heaveGain.y, rrChassisActuator.heaveInput) annotation(
    Line(points = {{60, 10}, {20, 10}, {20, -64}, {4, -64}}, color = {0, 0, 127}));
  connect(rollGain.y, rrChassisActuator.rollInput) annotation(
    Line(points = {{60, -20}, {10, -20}, {10, -76}, {4, -76}}, color = {0, 0, 127}));

  annotation(
    experiment(StartTime = 0, StopTime = 91, Tolerance = 1e-06, Interval = 1),
    __OpenModelica_commandLineOptions = "--matchingAlgorithm=PFPlusExt --indexReductionMethod=dynamicStateSelection -d=initialization,NLSanalyticJacobian --maxSizeLinearTearing=5000",
    __OpenModelica_simulationFlags(lv = "LOG_STDOUT,LOG_ASSERT,LOG_STATS", s = "dassl", variableFilter = ".*"));
end FourPostSim;