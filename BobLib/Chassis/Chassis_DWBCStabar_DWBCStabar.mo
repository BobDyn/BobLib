within BobLib.Chassis;

model Chassis_DWBCStabar_DWBCStabar

  "Detailed BobLib chassis configured with DWBC stabar front and rear axles"
  import Modelica.Constants.pi;
  import Modelica.Mechanics.MultiBody.Frames;
  import SI = Modelica.Units.SI;
  import BobLib.Utilities.Math.Vector;
  import Tire = BobLib.Chassis.Suspension.Tires;
  import BobLib.Records.VehicleDefn.EVBatInvMotDiff_DWBCStabar_DWBCStabarRecord;

  parameter EVBatInvMotDiff_DWBCStabar_DWBCStabarRecord pVehicle = EVBatInvMotDiff_DWBCStabar_DWBCStabarRecord()
    "Vehicle parameter record";

  final parameter SI.Mass pTotalMass =
    pVehicle.pSprungMass.m +
    pVehicle.pFrAxleMass.unsprungMass.m +
    pVehicle.pFrAxleMass.ucaMass.m +
    pVehicle.pFrAxleMass.lcaMass.m +
    pVehicle.pFrAxleMass.tieMass.m +
    pVehicle.pRrAxleMass.unsprungMass.m +
    pVehicle.pRrAxleMass.ucaMass.m +
    pVehicle.pRrAxleMass.lcaMass.m +
    pVehicle.pRrAxleMass.tieMass.m
    "Mass used for the initial chassis reference position";

  final parameter SI.Position pVehicleCG[3] = {
    (
      pVehicle.pSprungMass.m * pVehicle.pSprungMass.rCM[1] +
      pVehicle.pFrAxleMass.unsprungMass.m * pVehicle.pFrAxleMass.unsprungMass.rCM[1] +
      pVehicle.pFrAxleMass.ucaMass.m * pVehicle.pFrAxleMass.ucaMass.rCM[1] +
      pVehicle.pFrAxleMass.lcaMass.m * pVehicle.pFrAxleMass.lcaMass.rCM[1] +
      pVehicle.pFrAxleMass.tieMass.m * pVehicle.pFrAxleMass.tieMass.rCM[1] +
      pVehicle.pRrAxleMass.unsprungMass.m * pVehicle.pRrAxleMass.unsprungMass.rCM[1] +
      pVehicle.pRrAxleMass.ucaMass.m * pVehicle.pRrAxleMass.ucaMass.rCM[1] +
      pVehicle.pRrAxleMass.lcaMass.m * pVehicle.pRrAxleMass.lcaMass.rCM[1] +
      pVehicle.pRrAxleMass.tieMass.m * pVehicle.pRrAxleMass.tieMass.rCM[1]
    ) / pTotalMass,
    (
      pVehicle.pSprungMass.m * pVehicle.pSprungMass.rCM[2] +
      pVehicle.pFrAxleMass.unsprungMass.m * pVehicle.pFrAxleMass.unsprungMass.rCM[2] +
      pVehicle.pFrAxleMass.ucaMass.m * pVehicle.pFrAxleMass.ucaMass.rCM[2] +
      pVehicle.pFrAxleMass.lcaMass.m * pVehicle.pFrAxleMass.lcaMass.rCM[2] +
      pVehicle.pFrAxleMass.tieMass.m * pVehicle.pFrAxleMass.tieMass.rCM[2] +
      pVehicle.pRrAxleMass.unsprungMass.m * pVehicle.pRrAxleMass.unsprungMass.rCM[2] +
      pVehicle.pRrAxleMass.ucaMass.m * pVehicle.pRrAxleMass.ucaMass.rCM[2] +
      pVehicle.pRrAxleMass.lcaMass.m * pVehicle.pRrAxleMass.lcaMass.rCM[2] +
      pVehicle.pRrAxleMass.tieMass.m * pVehicle.pRrAxleMass.tieMass.rCM[2]
    ) / pTotalMass,
    (
      pVehicle.pSprungMass.m * pVehicle.pSprungMass.rCM[3] +
      pVehicle.pFrAxleMass.unsprungMass.m * pVehicle.pFrAxleMass.unsprungMass.rCM[3] +
      pVehicle.pFrAxleMass.ucaMass.m * pVehicle.pFrAxleMass.ucaMass.rCM[3] +
      pVehicle.pFrAxleMass.lcaMass.m * pVehicle.pFrAxleMass.lcaMass.rCM[3] +
      pVehicle.pFrAxleMass.tieMass.m * pVehicle.pFrAxleMass.tieMass.rCM[3] +
      pVehicle.pRrAxleMass.unsprungMass.m * pVehicle.pRrAxleMass.unsprungMass.rCM[3] +
      pVehicle.pRrAxleMass.ucaMass.m * pVehicle.pRrAxleMass.ucaMass.rCM[3] +
      pVehicle.pRrAxleMass.lcaMass.m * pVehicle.pRrAxleMass.lcaMass.rCM[3] +
      pVehicle.pRrAxleMass.tieMass.m * pVehicle.pRrAxleMass.tieMass.rCM[3]
    ) / pTotalMass
  } "Initial chassis reference position";

  extends BobLib.Chassis.Chassis_LockRrSteer(
    final chassisReferencePosition = pVehicleCG,
    final frontWheelRadius = pVehicle.pFrPartialWheel.R0,
    final rearWheelRadius = pVehicle.pRrPartialWheel.R0,
    final contactPatchPosition_1 =
      pVehicle.pFrDW.wheelCenter +
      Frames.resolve1(
        Frames.axesRotations(
          {1, 2, 3},
          {
            pVehicle.pFrPartialWheel.staticGamma*pi/180,
            0,
            pVehicle.pFrPartialWheel.staticAlpha*pi/180
          },
          {0, 0, 0}),
        {0, 0, -pVehicle.pFrPartialWheel.R0}),
    final contactPatchPosition_2 =
      Vector.mirrorXZ(
        pVehicle.pFrDW.wheelCenter +
        Frames.resolve1(
          Frames.axesRotations(
            {1, 2, 3},
            {
              pVehicle.pFrPartialWheel.staticGamma*pi/180,
              0,
              pVehicle.pFrPartialWheel.staticAlpha*pi/180
            },
            {0, 0, 0}),
          {0, 0, -pVehicle.pFrPartialWheel.R0})),
    final contactPatchPosition_3 =
      pVehicle.pRrDW.wheelCenter +
      Frames.resolve1(
        Frames.axesRotations(
          {1, 2, 3},
          {
            pVehicle.pRrPartialWheel.staticGamma*pi/180,
            0,
            pVehicle.pRrPartialWheel.staticAlpha*pi/180
          },
          {0, 0, 0}),
        {0, 0, -pVehicle.pRrPartialWheel.R0}),
    final contactPatchPosition_4 =
      Vector.mirrorXZ(
        pVehicle.pRrDW.wheelCenter +
        Frames.resolve1(
          Frames.axesRotations(
            {1, 2, 3},
            {
              pVehicle.pRrPartialWheel.staticGamma*pi/180,
              0,
              pVehicle.pRrPartialWheel.staticAlpha*pi/180
            },
            {0, 0, 0}),
          {0, 0, -pVehicle.pRrPartialWheel.R0})),
    final frontLeftRideHeightOffset =
      pVehicle.pAero.FL_RideHeightRef -
      {pVehicle.pFrDW.wheelCenter[1], 0, pVehicle.pFrDW.wheelCenter[3]},
    final frontRightRideHeightOffset =
      Vector.mirrorXZ(
        pVehicle.pAero.FL_RideHeightRef -
        {pVehicle.pFrDW.wheelCenter[1], 0, pVehicle.pFrDW.wheelCenter[3]}),
    final rearLeftRideHeightOffset =
      pVehicle.pAero.RL_RideHeightRef -
      {pVehicle.pRrDW.wheelCenter[1], 0, pVehicle.pRrDW.wheelCenter[3]},
    final rearRightRideHeightOffset =
      Vector.mirrorXZ(
        pVehicle.pAero.RL_RideHeightRef -
        {pVehicle.pRrDW.wheelCenter[1], 0, pVehicle.pRrDW.wheelCenter[3]}),
    detailedChassis(
      redeclare BobLib.Chassis.Suspension.FrAxleDW_BC_Stabar frAxleDW(
        pAxle = pVehicle.pFrAxleDW,
        pRack = pVehicle.pFrRack,
        pStabar = pVehicle.pFrStabar,
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
      redeclare BobLib.Chassis.Suspension.RrAxleDW_BC_Stabar rrAxleDW(
        pAxle = pVehicle.pRrAxleDW,
        pRack = pVehicle.pRrRack,
        pStabar = pVehicle.pRrStabar,
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
      redeclare BobLib.Chassis.Body.FrameCompX spaceFrame(
        frRef = {pVehicle.pFrDW.wheelCenter[1], 0, pVehicle.pFrDW.wheelCenter[3]},
        rrRef = {pVehicle.pRrDW.wheelCenter[1], 0, pVehicle.pRrDW.wheelCenter[3]},
        pSprung = pVehicle.pSprungMass,
        pSprungMass = pVehicle.pSprungMass,
        torsionalStiff = pVehicle.pTorsionalStiff)));

  annotation(Documentation(info = "<html>
<p>
Concrete BobLib chassis configuration exposed as a VehicleInterfaces two-axle
chassis. This keeps the detailed BobLib suspension, tire, and frame physics
inside the VI chassis boundary.
</p>
</html>"));
end Chassis_DWBCStabar_DWBCStabar;
