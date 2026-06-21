within BobLibVehicleInterfaces.Experiments.Standards.Templates.FourPost;

partial model BaseFourPostSim

  import SI = Modelica.Units.SI;
  import Modelica.Constants.pi;
  import Modelica.Math.Vectors.norm;
  import Modelica.Mechanics.MultiBody.Frames;
  import BobLibVehicleInterfaces.Utilities.Math.Vector;

  // FourPostEval record
  import BobLibVehicleInterfaces.Records.StandardRecord.FourPostEvalRecord;

  // Bar record to override rate
  import BobLibVehicleInterfaces.Records.VehicleRecord.Chassis.Suspension.Templates.Stabar.StabarRecord;

  // Tire record
  import Tire = BobLibVehicleInterfaces.Chassis.Suspension.Tires;

  inner parameter SI.Length linkDiameter = 0.020;
  inner parameter SI.Length jointDiameter = 0.030;
  inner parameter Boolean headless = false
    "Run without MultiBody animation geometry"
    annotation(Evaluate = true, Dialog(tab = "Animation"));

  replaceable record VehicleRecord = BobLibVehicleInterfaces.Records.VehicleDefn.EVBatInvMotDiff_DWBCStabar_DWBCStabarRecord;
  replaceable model FrAxleModel = 
    BobLibVehicleInterfaces.Chassis.Suspension.FrAxleDW_BC_Stabar
    constrainedby BobLibVehicleInterfaces.Chassis.Suspension.AxleDWBase;
  replaceable model RrAxleModel = 
    BobLibVehicleInterfaces.Chassis.Suspension.RrAxleDW_BC_Stabar
    constrainedby BobLibVehicleInterfaces.Chassis.Suspension.AxleDWBase;

  parameter VehicleRecord pVehicle;
  parameter StabarRecord pFrStabar(
    leftArmEnd = {0, 0, 0},
    leftBarEnd = {0, 0, 0},
    barRate = 0);
  parameter StabarRecord pRrStabar(
    leftArmEnd = {0, 0, 0},
    leftBarEnd = {0, 0, 0},
    barRate = 0);

  parameter SI.Angle steerMagnitude = 0 "Maximum pinion angle magnitude" annotation(
    Dialog(group = "Test Parameters"));
  parameter SI.Length heaveMagnitude = 1.5*0.0254 "Maximum heave magnitude" annotation(
    Dialog(group = "Test Parameters"));
  parameter SI.Angle rollMagnitude = 1.25*Modelica.Constants.pi/180 "Maximum roll magnitude" annotation(
    Dialog(group = "Test Parameters"));
  parameter SI.Force forceMagnitude = 1000
    "Maximum contact patch force"
    annotation(Dialog(group = "Test Parameters"));

  // Time-value tables [time, normalized value]
  final parameter Real rollTable[:, 2] = [0, 0.0; 1, 0.0; 2, 0.0; 3, 0.0; 4, 0.0; 5, 0.0; 6, 0.0; 7, 0.0; 8, 0.0; 9, 0.0; 10, 0.0; 11, 0.0; 12, 0.0; 13, 0.0; 14, 0.0; 15, 0.0; 16, 0.0; 17, 0.0; 18, 0.0; 19, 0.0; 20, 0.0; 21, 0.0; 22, 0.0; 23, 0.0; 24, 0.0; 25, 0.0; 26, 0.0; 27, 0.0; 28, 0.0; 29, 0.0; 30, 0.0; 31, 0.0; 32, 0.0; 33, 0.0; 34, 0.0; 35, 0.0; 36, 0.0; 37, 0.0; 38, 0.0; 39, 0.0; 40, 0.0; 41, 0.0; 42, 0.0; 43, 0.0; 44, 0.0; 45, 0.0; 46, 0.0; 47, 0.0; 48, 0.0; 49, 0.0; 50, 0.0; 51, 0.0; 52, 0.0; 53, 0.0; 54, 0.0; 55, 0.0; 56, 0.0; 57, 0.0; 58, 0.0; 59, 0.0; 60, 0.0; 61, 0.0; 62, 0.0; 63, -1.0; 64, -1.0; 65, -1.0; 66, -1.0; 67, -1.0; 68, -0.8; 69, -0.8; 70, -0.8; 71, -0.8; 72, -0.8; 73, -0.6; 74, -0.6; 75, -0.6; 76, -0.6; 77, -0.6; 78, -0.4; 79, -0.4; 80, -0.4; 81, -0.4; 82, -0.4; 83, -0.2; 84, -0.2; 85, -0.2; 86, -0.2; 87, -0.2; 88, 0.0; 89, 0.0; 90, 0.0; 91, 0.0; 92, 0.0; 93, 0.2; 94, 0.2; 95, 0.2; 96, 0.2; 97, 0.2; 98, 0.4; 99, 0.4; 100, 0.4; 101, 0.4; 102, 0.4; 103, 0.6; 104, 0.6; 105, 0.6; 106, 0.6; 107, 0.6; 108, 0.8; 109, 0.8; 110, 0.8; 111, 0.8; 112, 0.8; 113, 1.0; 114, 1.0; 115, 1.0; 116, 1.0; 117, 1.0; 118, 0.0];
  final parameter Real heaveTable[:, 2] = [0, 0.0; 1, 0.0; 2, -1.0; 3, -1.0; 4, -1.0; 5, -1.0; 6, -1.0; 7, -0.8; 8, -0.8; 9, -0.8; 10, -0.8; 11, -0.8; 12, -0.6; 13, -0.6; 14, -0.6; 15, -0.6; 16, -0.6; 17, -0.4; 18, -0.4; 19, -0.4; 20, -0.4; 21, -0.4; 22, -0.2; 23, -0.2; 24, -0.2; 25, -0.2; 26, -0.2; 27, 0.0; 28, 0.0; 29, 0.0; 30, 0.0; 31, 0.0; 32, 0.2; 33, 0.2; 34, 0.2; 35, 0.2; 36, 0.2; 37, 0.4; 38, 0.4; 39, 0.4; 40, 0.4; 41, 0.4; 42, 0.6; 43, 0.6; 44, 0.6; 45, 0.6; 46, 0.6; 47, 0.8; 48, 0.8; 49, 0.8; 50, 0.8; 51, 0.8; 52, 1.0; 53, 1.0; 54, 1.0; 55, 1.0; 56, 1.0; 57, 0.0; 58, 0.0; 59, 0.0; 60, 0.0; 61, 0.0; 62, 0.0];
  final parameter Real fxTable[:, 2] = [0, 0.0; 1, 0.0; 2, 0.0; 3, 1.0; 4, 0.0; 5, 0.0; 6, 0.0; 7, 0.0; 8, 1.0; 9, 0.0; 10, 0.0; 11, 0.0; 12, 0.0; 13, 1.0; 14, 0.0; 15, 0.0; 16, 0.0; 17, 0.0; 18, 1.0; 19, 0.0; 20, 0.0; 21, 0.0; 22, 0.0; 23, 1.0; 24, 0.0; 25, 0.0; 26, 0.0; 27, 0.0; 28, 1.0; 29, 0.0; 30, 0.0; 31, 0.0; 32, 0.0; 33, 1.0; 34, 0.0; 35, 0.0; 36, 0.0; 37, 0.0; 38, 1.0; 39, 0.0; 40, 0.0; 41, 0.0; 42, 0.0; 43, 1.0; 44, 0.0; 45, 0.0; 46, 0.0; 47, 0.0; 48, 1.0; 49, 0.0; 50, 0.0; 51, 0.0; 52, 0.0; 53, 1.0; 54, 0.0; 55, 0.0; 56, 0.0; 57, 0.0; 58, 0.0; 59, 0.0; 60, 0.0; 61, 0.0; 62, 0.0; 63, 0.0; 64, 0.0; 65, 0.0; 66, 0.0; 67, 0.0; 68, 0.0; 69, 0.0; 70, 0.0; 71, 0.0; 72, 0.0; 73, 0.0; 74, 0.0; 75, 0.0; 76, 0.0; 77, 0.0; 78, 0.0; 79, 0.0; 80, 0.0; 81, 0.0; 82, 0.0; 83, 0.0; 84, 0.0; 85, 0.0; 86, 0.0; 87, 0.0; 88, 0.0; 89, 0.0; 90, 0.0; 91, 0.0; 92, 0.0; 93, 0.0; 94, 0.0; 95, 0.0; 96, 0.0; 97, 0.0; 98, 0.0; 99, 0.0; 100, 0.0; 101, 0.0; 102, 0.0; 103, 0.0; 104, 0.0; 105, 0.0; 106, 0.0; 107, 0.0; 108, 0.0; 109, 0.0; 110, 0.0; 111, 0.0; 112, 0.0; 113, 0.0; 114, 0.0; 115, 0.0; 116, 0.0; 117, 0.0; 118, 0.0];
  final parameter Real fyTable[:, 2] = [0, 0.0; 1, 0.0; 2, 0.0; 3, 0.0; 4, 0.0; 5, 0.0; 6, 0.0; 7, 0.0; 8, 0.0; 9, 0.0; 10, 0.0; 11, 0.0; 12, 0.0; 13, 0.0; 14, 0.0; 15, 0.0; 16, 0.0; 17, 0.0; 18, 0.0; 19, 0.0; 20, 0.0; 21, 0.0; 22, 0.0; 23, 0.0; 24, 0.0; 25, 0.0; 26, 0.0; 27, 0.0; 28, 0.0; 29, 0.0; 30, 0.0; 31, 0.0; 32, 0.0; 33, 0.0; 34, 0.0; 35, 0.0; 36, 0.0; 37, 0.0; 38, 0.0; 39, 0.0; 40, 0.0; 41, 0.0; 42, 0.0; 43, 0.0; 44, 0.0; 45, 0.0; 46, 0.0; 47, 0.0; 48, 0.0; 49, 0.0; 50, 0.0; 51, 0.0; 52, 0.0; 53, 0.0; 54, 0.0; 55, 0.0; 56, 0.0; 57, 0.0; 58, 0.0; 59, 0.0; 60, 0.0; 61, 0.0; 62, 0.0; 63, 0.0; 64, 1.0; 65, 0.0; 66, 0.0; 67, 0.0; 68, 0.0; 69, 1.0; 70, 0.0; 71, 0.0; 72, 0.0; 73, 0.0; 74, 1.0; 75, 0.0; 76, 0.0; 77, 0.0; 78, 0.0; 79, 1.0; 80, 0.0; 81, 0.0; 82, 0.0; 83, 0.0; 84, 1.0; 85, 0.0; 86, 0.0; 87, 0.0; 88, 0.0; 89, 1.0; 90, 0.0; 91, 0.0; 92, 0.0; 93, 0.0; 94, 1.0; 95, 0.0; 96, 0.0; 97, 0.0; 98, 0.0; 99, 1.0; 100, 0.0; 101, 0.0; 102, 0.0; 103, 0.0; 104, 1.0; 105, 0.0; 106, 0.0; 107, 0.0; 108, 0.0; 109, 1.0; 110, 0.0; 111, 0.0; 112, 0.0; 113, 0.0; 114, 1.0; 115, 0.0; 116, 0.0; 117, 0.0; 118, 0.0];

  inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1}, g = 0, enableAnimation = not headless) annotation(
    Placement(transformation(origin = {-90, -90}, extent = {{-10, -10}, {10, 10}})));

  // Front axle
  FrAxleModel frAxleDW(pAxle = pVehicle.pFrAxleDW,
                              pRack = pVehicle.pFrRack,
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
  RrAxleModel rrAxleDW(pAxle = pVehicle.pRrAxleDW,
                              pRack = pVehicle.pRrRack,
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
  BobLibVehicleInterfaces.Utilities.Mechanics.MultiBody.Actuators.ChassisActuator frChassisActuator(axleRef = frAxleDW.effectiveCenter) annotation(
    Placement(transformation(origin = {0, 24}, extent = {{10, -10}, {-10, 10}})));

  // Rear chassis actuator
  BobLibVehicleInterfaces.Utilities.Mechanics.MultiBody.Actuators.ChassisActuator rrChassisActuator(axleRef = rrAxleDW.effectiveCenter) annotation(
    Placement(transformation(origin = {0, -76}, extent = {{10, -10}, {-10, 10}})));

  // Front steer input
  Modelica.Mechanics.Rotational.Sources.Position steerPosition(exact = true) annotation(
    Placement(transformation(origin = {-30, 80}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.Ramp steerExpression(height = steerMagnitude,
                                               duration = 1,
                                               startTime = 0) annotation(
    Placement(transformation(origin = {-70, 80}, extent = {{-10, -10}, {10, 10}})));

  // Rear steer lock
  Modelica.Mechanics.MultiBody.Parts.Mounting1D rrLock annotation(
    Placement(transformation(origin = {0, -25}, extent = {{5, -5}, {-5, 5}})));

  // Contact patch fixtures
  BobLibVehicleInterfaces.Utilities.Mechanics.MultiBody.Actuators.ContactPatchFixture FL_fixture(CP_init = cpInitFL) annotation(
    Placement(transformation(origin = {-40, 20}, extent = {{-20, -20}, {20, 20}})));
  BobLibVehicleInterfaces.Utilities.Mechanics.MultiBody.Actuators.ContactPatchFixture FR_fixture(CP_init = cpInitFR) annotation(
    Placement(transformation(origin = {40, 20}, extent = {{-20, -20}, {20, 20}})));
  BobLibVehicleInterfaces.Utilities.Mechanics.MultiBody.Actuators.ContactPatchFixture RL_fixture(CP_init = cpInitRL) annotation(
    Placement(transformation(origin = {-40, -80}, extent = {{-20, -20}, {20, 20}})));
  BobLibVehicleInterfaces.Utilities.Mechanics.MultiBody.Actuators.ContactPatchFixture RR_fixture(CP_init = cpInitRR) annotation(
    Placement(transformation(origin = {40, -80}, extent = {{-20, -20}, {20, 20}})));

  // Force actuators
  BobLibVehicleInterfaces.Utilities.Mechanics.MultiBody.Actuators.ContactPatchForceActuator FL_ForceActuator(
    fxTable = fxTable,
    fyTable = fyTable,
    forceMagnitude = forceMagnitude / 2) annotation(
    Placement(transformation(origin = {-60, 40}, extent = {{-10, -10}, {10, 10}})));
  BobLibVehicleInterfaces.Utilities.Mechanics.MultiBody.Actuators.ContactPatchForceActuator FR_ForceActuator(
    fxTable = fxTable,
    fyTable = fyTable,
    forceMagnitude = forceMagnitude / 2) annotation(
    Placement(transformation(origin = {60, 40}, extent = {{10, -10}, {-10, 10}})));
  BobLibVehicleInterfaces.Utilities.Mechanics.MultiBody.Actuators.ContactPatchForceActuator RL_ForceActuator(
    fxTable = fxTable,
    fyTable = fyTable,
    forceMagnitude = forceMagnitude / 2) annotation(
    Placement(transformation(origin = {-60, -60}, extent = {{-10, -10}, {10, 10}})));
  BobLibVehicleInterfaces.Utilities.Mechanics.MultiBody.Actuators.ContactPatchForceActuator RR_ForceActuator(
    fxTable = fxTable,
    fyTable = fyTable,
    forceMagnitude = forceMagnitude / 2) annotation(
    Placement(transformation(origin = {60, -60}, extent = {{10, -10}, {-10, 10}})));

  // Heave input
  Modelica.Blocks.Sources.CombiTimeTable heaveSource(table = heaveTable) annotation(
    Placement(transformation(origin = {-90, 20}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.RealExpression heaveCommand(y = heaveMagnitude * heaveSource.y[1]) annotation(
    Placement(transformation(origin = {90, 70}, extent = {{10, -10}, {-10, 10}})));

  // Roll input
  Modelica.Blocks.Sources.CombiTimeTable rollSource(table = rollTable) annotation(
    Placement(transformation(origin = {-90, -20}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.RealExpression rollCommand(y = rollMagnitude * rollSource.y[1]) annotation(
    Placement(transformation(origin = {90, 38}, extent = {{10, -10}, {-10, 10}})));

  // FourPostEval records
  FourPostEvalRecord frKnC;
  FourPostEvalRecord rrKnC;

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

  frKnC.leftSpringLength = frAxleDW.leftSpringLength;
  frKnC.leftFz = frAxleDW.leftTire.Fz;
  frKnC.rightSpringLength = frAxleDW.rightSpringLength;
  frKnC.rightFz = frAxleDW.rightTire.Fz;
  frKnC.stabarAngle = frAxleDW.stabarAngle;

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

  rrKnC.leftSpringLength = rrAxleDW.leftSpringLength;
  rrKnC.leftFz = rrAxleDW.leftTire.Fz;
  rrKnC.rightSpringLength = rrAxleDW.rightSpringLength;
  rrKnC.rightFz = rrAxleDW.rightTire.Fz;
  rrKnC.stabarAngle = rrAxleDW.stabarAngle;

  rrKnC.jackingForce = rrChassisActuator.jackingOutput;
  rrKnC.heave = rrChassisActuator.heaveInput;
  rrKnC.roll = rrChassisActuator.rollInput;
  rrKnC.fx = RL_ForceActuator.worldForce.force[1] + RR_ForceActuator.worldForce.force[1];
  rrKnC.fy = RL_ForceActuator.worldForce.force[2] + RR_ForceActuator.worldForce.force[2];

  connect(FL_fixture.frame_a, frAxleDW.leftCP) annotation(
    Line(points = {{-40, 28}, {-40.5, 28}, {-40.5, 40}, {-37, 40}}, color = {95, 95, 95}));
  connect(FR_fixture.frame_a, frAxleDW.rightCP) annotation(
    Line(points = {{40, 28}, {39.75, 28}, {39.75, 40}, {37.5, 40}}, color = {95, 95, 95}));
  connect(steerExpression.y, steerPosition.phi_ref) annotation(
    Line(points = {{-59, 80}, {-42, 80}}, color = {0, 0, 127}));
  connect(frAxleDW.axleFrame, frChassisActuator.chassisFrame) annotation(
    Line(points = {{0, 50}, {0, 32}}, color = {95, 95, 95}));
  connect(steerPosition.flange, frAxleDW.steerFlange) annotation(
    Line(points = {{-20, 80}, {0, 80}, {0, 60}}));
  connect(FL_ForceActuator.chassisFrame, frAxleDW.leftCP) annotation(
    Line(points = {{-50, 40}, {-36, 40}}, color = {95, 95, 95}));
  connect(FR_ForceActuator.chassisFrame, frAxleDW.rightCP) annotation(
    Line(points = {{50, 40}, {38, 40}}, color = {95, 95, 95}));
  connect(rrLock.flange_b, rrAxleDW.steerFlange) annotation(
    Line(points = {{-5, -25}, {-20, -25}, {-20, -54}, {0, -54}}));
  connect(rrLock.frame_a, rrAxleDW.axleFrame) annotation(
    Line(points = {{0, -30}, {0, -50}}, color = {95, 95, 95}));
  connect(RL_fixture.frame_a, rrAxleDW.leftCP) annotation(
    Line(points = {{-40, -72}, {-40, -60}, {-36, -60}}, color = {95, 95, 95}));
  connect(RL_ForceActuator.chassisFrame, rrAxleDW.leftCP) annotation(
    Line(points = {{-50, -60}, {-36, -60}}, color = {95, 95, 95}));
  connect(rrChassisActuator.chassisFrame, rrAxleDW.axleFrame) annotation(
    Line(points = {{0, -68}, {0, -50}}, color = {95, 95, 95}));
  connect(RR_fixture.frame_a, rrAxleDW.rightCP) annotation(
    Line(points = {{40, -72}, {40, -60}, {36, -60}}, color = {95, 95, 95}));
  connect(RR_ForceActuator.chassisFrame, rrAxleDW.rightCP) annotation(
    Line(points = {{50, -60}, {36, -60}}, color = {95, 95, 95}));
  connect(heaveCommand.y, frChassisActuator.heaveInput) annotation(
    Line(points = {{80, 70}, {60, 70}, {60, 30}, {4, 30}}, color = {0, 0, 127}));
  connect(heaveCommand.y, rrChassisActuator.heaveInput) annotation(
    Line(points = {{80, 70}, {60, 70}, {60, -70}, {4, -70}}, color = {0, 0, 127}));
  connect(rollCommand.y, frChassisActuator.rollInput) annotation(
    Line(points = {{80, 38}, {70, 38}, {70, 18}, {4, 18}}, color = {0, 0, 127}));
  connect(rollCommand.y, rrChassisActuator.rollInput) annotation(
    Line(points = {{80, 38}, {70, 38}, {70, -82}, {4, -82}}, color = {0, 0, 127}));
  annotation(
    Diagram,
    experiment(StartTime = 0, StopTime = 118, Tolerance = 1e-06, Interval = 1),
    __OpenModelica_commandLineOptions = "--matchingAlgorithm=PFPlusExt --indexReductionMethod=dynamicStateSelection -d=initialization,NLSanalyticJacobian --maxSizeLinearTearing=5000 --generateDynamicJacobian=none",
    __OpenModelica_simulationFlags(
      jacobian = "internalNumerical",
      lv = "LOG_STDOUT,LOG_ASSERT,LOG_STATS",
      s = "dassl",
      variableFilter = ".*"),
    Documentation(info = "<html>
<p>
Partial model <code>BaseFourPostSim</code> is the shared four-post rig template.
</p>
<p>
It exposes replaceable front and rear axle models, contact-patch fixtures, heave/roll/force tables, steering input, and chassis actuation used by the standard four-post experiments.
</p>
</html>"));
end BaseFourPostSim;