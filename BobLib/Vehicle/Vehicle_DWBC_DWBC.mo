within BobLib.Vehicle;

model Vehicle_DWBC_DWBC
  import BobLib.Vehicle.Chassis.Suspension.Templates.Tire;
  import BobLib.Resources.VehicleDefn.DWBC_DWBCRecord;

  // Record parameters
  parameter DWBC_DWBCRecord pVehicle;

  extends BobLib.Vehicle.VehicleBase(
    pAero = pVehicle.pAero,
    pSprungMass = pVehicle.pSprungMass,
    pFrAxleMass = pVehicle.pFrAxleMass,
    pRrAxleMass = pVehicle.pRrAxleMass,
    redeclare BobLib.Vehicle.Chassis.Chassis_LockRrSteer chassis(
      redeclare BobLib.Vehicle.Chassis.Suspension.FrAxleDW_BC frAxleDW(
            pAxle = pVehicle.pFrAxleDW,
            pRack = pVehicle.pFrRack,
            pLeftPartialWheel = pVehicle.pFrPartialWheel,
            pLeftDW = pVehicle.pFrDW,
            pLeftAxleMass = pVehicle.pFrAxleMass,
            redeclare Tire.MF52Tire leftTire(
                  pPartialWheel = pVehicle.pFrPartialWheel,
                  pTireModel = pVehicle.pFrTireModel,
                  redeclare Tire.TirePhysics.Wheel1DOF_Y wheelModel(
                    longitudinalTorqueSign = -1,
                    partialWheelParams = pVehicle.pFrPartialWheel,
                    wheel1DOF_YParams = pVehicle.pFrTire1DOF_YParams),
                  redeclare Tire.MF52.SlipModel.TransientSlip slipModel(
            FNOMIN = pVehicle.pFrTireModel.relaxation.FNOMIN,
            UNLOADED_RADIUS = pVehicle.pFrTireModel.relaxation.UNLOADED_RADIUS,
            LFZO = pVehicle.pFrTireModel.relaxation.LFZO,
            PTX1 = pVehicle.pFrTireModel.relaxation.PTX1,
            PTX2 = pVehicle.pFrTireModel.relaxation.PTX2,
            PTX3 = pVehicle.pFrTireModel.relaxation.PTX3,
            PTY1 = pVehicle.pFrTireModel.relaxation.PTY1,
            PTY2 = pVehicle.pFrTireModel.relaxation.PTY2,
            PKY3 = pVehicle.pFrTireModel.relaxation.PKY3,
            LSGKP = pVehicle.pFrTireModel.relaxation.LSGKP,
            LSGAL = pVehicle.pFrTireModel.relaxation.LSGAL)),
            redeclare Tire.MF52Tire rightTire(
                  pPartialWheel = pVehicle.pFrPartialWheel,
                  pTireModel = pVehicle.pFrTireModel,
                  redeclare Tire.TirePhysics.Wheel1DOF_Y wheelModel(
                    longitudinalTorqueSign = -1,
                    partialWheelParams = pVehicle.pFrPartialWheel,
                    wheel1DOF_YParams = pVehicle.pFrTire1DOF_YParams),
                  redeclare Tire.MF52.SlipModel.TransientSlip slipModel(
            FNOMIN = pVehicle.pFrTireModel.relaxation.FNOMIN,
            UNLOADED_RADIUS = pVehicle.pFrTireModel.relaxation.UNLOADED_RADIUS,
            LFZO = pVehicle.pFrTireModel.relaxation.LFZO,
            PTX1 = pVehicle.pFrTireModel.relaxation.PTX1,
            PTX2 = pVehicle.pFrTireModel.relaxation.PTX2,
            PTX3 = pVehicle.pFrTireModel.relaxation.PTX3,
            PTY1 = pVehicle.pFrTireModel.relaxation.PTY1,
            PTY2 = pVehicle.pFrTireModel.relaxation.PTY2,
            PKY3 = pVehicle.pFrTireModel.relaxation.PKY3,
            LSGKP = pVehicle.pFrTireModel.relaxation.LSGKP,
            LSGAL = pVehicle.pFrTireModel.relaxation.LSGAL))),
      redeclare BobLib.Vehicle.Chassis.Suspension.RrAxleDW_BC rrAxleDW(
            pAxle = pVehicle.pRrAxleDW,
            pRack = pVehicle.pRrRack,
            pLeftPartialWheel = pVehicle.pRrPartialWheel,
            pLeftDW = pVehicle.pRrDW,
            pLeftAxleMass = pVehicle.pRrAxleMass,
            redeclare Tire.MF52Tire leftTire(
                  pPartialWheel = pVehicle.pRrPartialWheel,
                  pTireModel = pVehicle.pRrTireModel,
                  redeclare Tire.TirePhysics.Wheel1DOF_Y wheelModel(
                    longitudinalTorqueSign = -1,
                    partialWheelParams = pVehicle.pRrPartialWheel,
                    wheel1DOF_YParams = pVehicle.pRrTire1DOF_YParams),
                  redeclare Tire.MF52.SlipModel.TransientSlip slipModel(
            FNOMIN = pVehicle.pRrTireModel.relaxation.FNOMIN,
            UNLOADED_RADIUS = pVehicle.pRrTireModel.relaxation.UNLOADED_RADIUS,
            LFZO = pVehicle.pRrTireModel.relaxation.LFZO,
            PTX1 = pVehicle.pRrTireModel.relaxation.PTX1,
            PTX2 = pVehicle.pRrTireModel.relaxation.PTX2,
            PTX3 = pVehicle.pRrTireModel.relaxation.PTX3,
            PTY1 = pVehicle.pRrTireModel.relaxation.PTY1,
            PTY2 = pVehicle.pRrTireModel.relaxation.PTY2,
            PKY3 = pVehicle.pRrTireModel.relaxation.PKY3,
            LSGKP = pVehicle.pRrTireModel.relaxation.LSGKP,
            LSGAL = pVehicle.pRrTireModel.relaxation.LSGAL)),
            redeclare Tire.MF52Tire rightTire(
                  pPartialWheel = pVehicle.pRrPartialWheel,
                  pTireModel = pVehicle.pRrTireModel,
                  redeclare Tire.TirePhysics.Wheel1DOF_Y wheelModel(
                    longitudinalTorqueSign = -1,
                    partialWheelParams = pVehicle.pRrPartialWheel,
                    wheel1DOF_YParams = pVehicle.pRrTire1DOF_YParams),
                  redeclare Tire.MF52.SlipModel.TransientSlip slipModel(
            FNOMIN = pVehicle.pRrTireModel.relaxation.FNOMIN,
            UNLOADED_RADIUS = pVehicle.pRrTireModel.relaxation.UNLOADED_RADIUS,
            LFZO = pVehicle.pRrTireModel.relaxation.LFZO,
            PTX1 = pVehicle.pRrTireModel.relaxation.PTX1,
            PTX2 = pVehicle.pRrTireModel.relaxation.PTX2,
            PTX3 = pVehicle.pRrTireModel.relaxation.PTX3,
            PTY1 = pVehicle.pRrTireModel.relaxation.PTY1,
            PTY2 = pVehicle.pRrTireModel.relaxation.PTY2,
            PKY3 = pVehicle.pRrTireModel.relaxation.PKY3,
            LSGKP = pVehicle.pRrTireModel.relaxation.LSGKP,
            LSGAL = pVehicle.pRrTireModel.relaxation.LSGAL))),
      redeclare BobLib.Vehicle.Chassis.Body.FrameCompX spaceFrame(
        frRef = {pVehicle.pFrDW.wheelCenter[1], 0, pVehicle.pFrDW.wheelCenter[3]},
        rrRef = {pVehicle.pRrDW.wheelCenter[1], 0, pVehicle.pRrDW.wheelCenter[3]},
        pSprung = pVehicle.pSprungMass,
        pSprungMass = pVehicle.pSprungMass,
        torsionalStiff = pVehicle.pTorsionalStiff)
    ));

  Powertrain.PowertrainBatInvMotDiff ptn(
    Ns = pVehicle.pPowertrain.Ns,
    Np = pVehicle.pPowertrain.Np,
    SOC_start = pVehicle.pPowertrain.SOC_start,
    finalDriveRatio = pVehicle.pPowertrain.finalDriveRatio,
    launch_w_eps = pVehicle.pPowertrain.launch_w_eps,
    drivetrainAxis = pVehicle.pPowertrain.drivetrainAxis,
    rMotorRotor = pVehicle.pPowertrain.rMotorRotor,
    rDiffInputRotor = pVehicle.pPowertrain.rDiffInputRotor,
    rDifferential = pVehicle.pPowertrain.rDifferential,
    motorRotorJ = pVehicle.pPowertrain.motorRotorJ,
    diffInputRotorJ = pVehicle.pPowertrain.diffInputRotorJ,
    diff_use_lsd = pVehicle.pPowertrain.diff_use_lsd,
    diff_driveSideTorqueSign = pVehicle.pPowertrain.diff_driveSideTorqueSign,
    diff_T_preload = pVehicle.pPowertrain.diff_T_preload,
    diff_lockFractionAccel = pVehicle.pPowertrain.diff_lockFractionAccel,
    diff_lockFractionDecel = pVehicle.pPowertrain.diff_lockFractionDecel,
    diff_T_capacity_max = pVehicle.pPowertrain.diff_T_capacity_max,
    diff_clutchEffectiveRadius = pVehicle.pPowertrain.diff_clutchEffectiveRadius,
    diff_kineticFrictionRatio = pVehicle.pPowertrain.diff_kineticFrictionRatio,
    diff_w_transition = pVehicle.pPowertrain.diff_w_transition,
    diff_c_viscous = pVehicle.pPowertrain.diff_c_viscous,
    halfshaftLeftC = pVehicle.pPowertrain.halfshaftLeftC,
    halfshaftLeftJEquivalent = pVehicle.pPowertrain.halfshaftLeftJEquivalent,
    halfshaftLeftD = pVehicle.pPowertrain.halfshaftLeftD,
    halfshaftRightC = pVehicle.pPowertrain.halfshaftRightC,
    halfshaftRightJEquivalent = pVehicle.pPowertrain.halfshaftRightJEquivalent,
    halfshaftRightD = pVehicle.pPowertrain.halfshaftRightD,
    motorVdcMax = pVehicle.pPowertrain.motorVdcMax,
    motorRpmMaxPeak = pVehicle.pPowertrain.motorRpmMaxPeak,
    motorTPeak = pVehicle.pPowertrain.motorTPeak,
    motorTCont = pVehicle.pPowertrain.motorTCont,
    motorIPeak = pVehicle.pPowertrain.motorIPeak,
    motorICont = pVehicle.pPowertrain.motorICont,
    motorKtNmPerA = pVehicle.pPowertrain.motorKtNmPerA,
    motorPeakTime = pVehicle.pPowertrain.motorPeakTime,
    motorPMechPeak = pVehicle.pPowertrain.motorPMechPeak,
    motorPContLow = pVehicle.pPowertrain.motorPContLow,
    motorPContHigh = pVehicle.pPowertrain.motorPContHigh,
    motorEtaMot = pVehicle.pPowertrain.motorEtaMot,
    motorEtaReg = pVehicle.pPowertrain.motorEtaReg) annotation(
    Placement(transformation(origin = {0, -70}, extent = {{-20, -20}, {20, 20}})));

  replaceable Electronics.ElectronicsAssembly elc(
    tau_max = pVehicle.pPowertrain.tau_max,
    w_eps = pVehicle.pPowertrain.launch_w_eps,
    motorSpeedSign = pVehicle.pPowertrain.vcuMotorSpeedSign,
    inverterPMaxMot = pVehicle.pPowertrain.inverterPMaxMot,
    inverterPMaxReg = pVehicle.pPowertrain.inverterPMaxReg,
    inverterVdcMax = pVehicle.pPowertrain.inverterVdcMax)
    constrainedby Electronics.ElectronicsBase annotation(
    Placement(transformation(origin = {-58, -82}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Sources.BooleanConstant inverterEnabled(k = true) annotation(
    Placement(transformation(origin = {-96, -86}, extent = {{-8, -8}, {8, 8}})));

  Modelica.Mechanics.Rotational.Interfaces.Flange_b flangeFL annotation(
    Placement(transformation(origin = {-100, 60}, extent = {{-10, -10}, {10, 10}}),
    iconTransformation(origin = {-180, 120}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.Rotational.Interfaces.Flange_b flangeFR annotation(
    Placement(transformation(origin = {100, 60}, extent = {{-10, -10}, {10, 10}}),
    iconTransformation(origin = {180, 120}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Interfaces.RealInput uPTNTorque
    "Requested total rear-axle drive torque [Nm]" annotation(
    Placement(transformation(origin = {0, -120}, extent = {{-20, -20}, {20, 20}}, rotation = 90),
    iconTransformation(origin = {-30, -220}, extent = {{-20, -20}, {20, 20}}, rotation = 90)));

  Modelica.Blocks.Interfaces.RealInput uPTNRegenLimit annotation(
    Placement(transformation(origin = {50, -120}, extent = {{-20, -20}, {20, 20}}, rotation = 90),
    iconTransformation(origin = {30, -220}, extent = {{-20, -20}, {20, 20}}, rotation = 90)));

equation
  connect(chassis.rrAxleFrame, ptn.mountFrame) annotation(
    Line(points = {{0, -40}, {0, -50}}, color = {95, 95, 95}));

  connect(ptn.leftFlange, chassis.flangeRL) annotation(
    Line(points = {{-20, -78}, {-70, -78}, {-70, -40}, {-58, -40}}));

  connect(ptn.rightFlange, chassis.flangeRR) annotation(
    Line(points = {{20, -78}, {70, -78}, {70, -40}, {58, -40}}));

  connect(flangeFL, chassis.flangeFL) annotation(
    Line(points = {{-100, 60}, {-80, 60}, {-80, 38}, {-58, 38}}));

  connect(flangeFR, chassis.flangeFR) annotation(
    Line(points = {{100, 60}, {80, 60}, {80, 38}, {58, 38}}));

  elc.cmd_torque_motor = uPTNTorque / pVehicle.pPowertrain.finalDriveRatio;
  elc.sens_motor_speed = ptn.motorSpeed;

  connect(uPTNRegenLimit, elc.cmd_regen_limit) annotation(
    Line(points = {{50, -120}, {50, -86}, {-70, -86}}, color = {0, 0, 127}));

  connect(inverterEnabled.y, elc.cmd_inverter_enable) annotation(
    Line(points = {{-87, -86}, {-79.5, -86}, {-79.5, -82}, {-70, -82}}, color = {255, 0, 255}));

  connect(elc.P_motor, ptn.P_elec) annotation(
    Line(points = {{-46, -82}, {-28, -82}, {-28, -94}, {-12, -94}}, color = {0, 0, 127}));

  connect(elc.p, ptn.hv_p) annotation(
    Line(points = {{-68, -78}, {-78, -78}, {-78, -62}, {-20, -62}}, color = {0, 0, 255}));

  connect(elc.n, ptn.hv_n) annotation(
    Line(points = {{-68, -86}, {-84, -86}, {-84, -68}, {-20, -68}}, color = {0, 0, 255}));

annotation(
  Diagram(graphics),
  Icon(graphics = {
    Line(origin = {-130, 120}, points = {{-10, 0}, {-50, 0}}, pattern = LinePattern.Dash, thickness = 1),
    Line(origin = {190, 120}, points = {{-10, 0}, {-50, 0}}, pattern = LinePattern.Dash, thickness = 1),
    Line(origin = {-130, -120}, points = {{-10, 0}, {-30, 0}}, pattern = LinePattern.Dash, thickness = 1),
    Line(origin = {190, -120}, points = {{-30, 0}, {-50, 0}}, pattern = LinePattern.Dash, thickness = 1),
    Line(origin = {-80, -159}, points = {{80, -41}, {80, -31}, {-80, -31}, {-80, 39}}, color = {0, 0, 255}),
    Line(origin = {71.18, -171.82}, points = {{-71.1799, -18.1799}, {88.8201, -18.1799}, {88.8201, 51.8201}}, color = {0, 0, 255})
  }));
end Vehicle_DWBC_DWBC;
