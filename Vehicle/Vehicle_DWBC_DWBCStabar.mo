within BobLib.Vehicle;

model Vehicle_DWBC_DWBCStabar
  import BobLib.Vehicle.Chassis.Suspension.Templates.Tire;
  import BobLib.Resources.VehicleDefn.DWBC_DWBCStabarRecord;
  import BobLib.Resources.VehicleDefn.DWBCStabar_DWBCStabarRecord;
  
  // Record parameters
  parameter DWBC_DWBCStabarRecord fr_pVehicle;
  parameter DWBCStabar_DWBCStabarRecord rr_pVehicle;
  
  extends BobLib.Vehicle.VehicleBase(
    redeclare BobLib.Vehicle.Chassis.Chassis_LockRrSteer chassis(
      redeclare BobLib.Vehicle.Chassis.Suspension.FrAxleDW_BC frAxleDW(pAxle = fr_pVehicle.pFrAxleDW,
                                                                       pRack = fr_pVehicle.pFrRack,
                                                                       pLeftPartialWheel = fr_pVehicle.pFrPartialWheel,
                                                                       pLeftDW = fr_pVehicle.pFrDW,
                                                                       pLeftAxleMass = fr_pVehicle.pFrAxleMass,
                                                                       redeclare Tire.MF52Tire leftTire(pPartialWheel = fr_pVehicle.pFrPartialWheel,
                                                                                                        pTireModel = fr_pVehicle.pFrTireModel,
                                                                                                        redeclare Tire.TirePhysics.Wheel1DOF_Y wheelModel(partialWheelParams = fr_pVehicle.pFrPartialWheel,
                                                                                                                                                           wheel1DOF_YParams = fr_pVehicle.pFrTire1DOF_YParams),
                                                                                                        redeclare Tire.MF52.SlipModel.TransientSlip slipModel),
                                                                       redeclare Tire.MF52Tire rightTire(pPartialWheel = fr_pVehicle.pFrPartialWheel,
                                                                                                         pTireModel = fr_pVehicle.pFrTireModel,
                                                                                                         redeclare Tire.TirePhysics.Wheel1DOF_Y wheelModel(partialWheelParams = fr_pVehicle.pFrPartialWheel,
                                                                                                                                                           wheel1DOF_YParams = fr_pVehicle.pFrTire1DOF_YParams), 
                                                                                                         redeclare Tire.MF52.SlipModel.TransientSlip slipModel)),
    redeclare BobLib.Vehicle.Chassis.Suspension.RrAxleDW_BC_ARB rrAxleDW(pAxle = rr_pVehicle.pRrAxleDW,
                                                                         pRack = rr_pVehicle.pRrRack,
                                                                         pStabar = rr_pVehicle.pRrStabar,
                                                                         pLeftPartialWheel = rr_pVehicle.pRrPartialWheel,
                                                                         pLeftDW = rr_pVehicle.pRrDW,
                                                                         pLeftAxleMass = rr_pVehicle.pRrAxleMass,
                                                                         redeclare Tire.MF52Tire leftTire(pPartialWheel = rr_pVehicle.pRrPartialWheel,
                                                                                                          pTireModel = rr_pVehicle.pRrTireModel,
                                                                                                          redeclare Tire.TirePhysics.Wheel1DOF_Y wheelModel(partialWheelParams = rr_pVehicle.pRrPartialWheel,
                                                                                                                                                            wheel1DOF_YParams = rr_pVehicle.pRrTire1DOF_YParams),
                                                                                                          redeclare Tire.MF52.SlipModel.TransientSlip slipModel),
                                                                         redeclare Tire.MF52Tire rightTire(pPartialWheel = rr_pVehicle.pRrPartialWheel,
                                                                                                           pTireModel = rr_pVehicle.pRrTireModel,
                                                                                                           redeclare Tire.TirePhysics.Wheel1DOF_Y wheelModel(partialWheelParams = rr_pVehicle.pRrPartialWheel,
                                                                                                                                                             wheel1DOF_YParams = rr_pVehicle.pRrTire1DOF_YParams), 
                                                                                                           redeclare Tire.MF52.SlipModel.TransientSlip slipModel)),
   
   redeclare BobLib.Vehicle.Chassis.Body.FrameCompX spaceFrame(frRef = chassis.frAxleDW.effectiveCenter,
                                                               rrRef = chassis.rrAxleDW.effectiveCenter,
                                                               pSprung = fr_pVehicle.pSprungMass)));
  
  Powertrain.PTNPlaceholder ptnPlaceholder annotation(
    Placement(transformation(origin = {0, -70}, extent = {{-20, -4}, {20, 4}})));
  
  Modelica.Mechanics.Rotational.Interfaces.Flange_b flangeFL annotation(
    Placement(transformation(origin = {-100, 60}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {-180, 120}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.Rotational.Interfaces.Flange_b flangeFR annotation(
    Placement(transformation(origin = {100, 60}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {180, 120}, extent = {{-10, -10}, {10, 10}})));
  
  Modelica.Blocks.Interfaces.RealInput uPTNTorque annotation(
    Placement(transformation(origin = {0, -120}, extent = {{-20, -20}, {20, 20}}, rotation = 90), iconTransformation(origin = {0, -220}, extent = {{-20, -20}, {20, 20}}, rotation = 90)));

equation
  connect(chassis.rrAxleFrame, ptnPlaceholder.mountFrame) annotation(
    Line(points = {{0, -40}, {0, -66}}, color = {95, 95, 95}));
  connect(ptnPlaceholder.leftFlange, chassis.flangeRL) annotation(
    Line(points = {{-20, -70}, {-70, -70}, {-70, -40}, {-58, -40}}));
  connect(ptnPlaceholder.rightFlange, chassis.flangeRR) annotation(
    Line(points = {{20, -70}, {70, -70}, {70, -40}, {58, -40}}));
  connect(flangeFL, chassis.flangeFL) annotation(
    Line(points = {{-100, 60}, {-80, 60}, {-80, 38}, {-58, 38}}));
  connect(flangeFR, chassis.flangeFR) annotation(
    Line(points = {{100, 60}, {80, 60}, {80, 38}, {58, 38}}));
  connect(uPTNTorque, ptnPlaceholder.u) annotation(
    Line(points = {{0, -120}, {0, -78}}, color = {0, 0, 127}));
annotation(
    Diagram(graphics),
    Icon(graphics = {Line(origin = {-130, 120}, points = {{-10, 0}, {-50, 0}}, pattern = LinePattern.Dash, thickness = 1), Line(origin = {190, 120}, points = {{-10, 0}, {-50, 0}}, pattern = LinePattern.Dash, thickness = 1), Line(origin = {-130, -120}, points = {{-10, 0}, {-30, 0}}, pattern = LinePattern.Dash, thickness = 1), Line(origin = {190, -120}, points = {{-30, 0}, {-50, 0}}, pattern = LinePattern.Dash, thickness = 1), Line(origin = {-80, -159}, points = {{80, -41}, {80, -31}, {-80, -31}, {-80, 39}}, color = {0, 0, 255}), Line(origin = {71.18, -171.82}, points = {{-71.1799, -18.1799}, {88.8201, -18.1799}, {88.8201, 51.8201}}, color = {0, 0, 255})}));
end Vehicle_DWBC_DWBCStabar;
