within BobLibVehicleInterfaces.Chassis;

model Chassis_DW
  "Detailed BobLib chassis configured by replaceable double-wishbone axle variants"
  import Modelica.Constants.pi;
  import Modelica.Mechanics.MultiBody.Frames;
  import BobLibVehicleInterfaces.Utilities.Math.Vector;
  import Tire = BobLibVehicleInterfaces.Chassis.Suspension.Tires;
  import BobLibVehicleInterfaces.Records.VehicleRecord.Chassis.Suspension.Templates.Stabar.StabarRecord;

  replaceable record VehicleRecord =
    BobLibVehicleInterfaces.Records.VehicleDefn.EVBatInvMotDiff_DWBCStabar_DWBCStabarRecord;
  replaceable model FrAxleModel =
    BobLibVehicleInterfaces.Chassis.Suspension.FrAxleDW_BC_Stabar
    constrainedby BobLibVehicleInterfaces.Chassis.Suspension.AxleDWBase;
  replaceable model RrAxleModel =
    BobLibVehicleInterfaces.Chassis.Suspension.RrAxleDW_BC_Stabar
    constrainedby BobLibVehicleInterfaces.Chassis.Suspension.AxleDWBase;

  parameter VehicleRecord pVehicle = VehicleRecord()
    "Vehicle parameter record";
  parameter StabarRecord pFrStabar(
    leftArmEnd = {0, 0, 0},
    leftBarEnd = {0, 0, 0},
    barRate = 0);
  parameter StabarRecord pRrStabar(
    leftArmEnd = {0, 0, 0},
    leftBarEnd = {0, 0, 0},
    barRate = 0);

  extends BobLibVehicleInterfaces.Chassis.Chassis_LockRrSteer(
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
      redeclare FrAxleModel frAxleDW(
        pAxle = pVehicle.pFrAxleDW,
        pRack = pVehicle.pFrRack,
        pStabar = pFrStabar,
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
      redeclare RrAxleModel rrAxleDW(
        pAxle = pVehicle.pRrAxleDW,
        pRack = pVehicle.pRrRack,
        pStabar = pRrStabar,
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
      redeclare BobLibVehicleInterfaces.Chassis.Body.FrameCompX spaceFrame(
        frRef = {pVehicle.pFrDW.wheelCenter[1], 0, pVehicle.pFrDW.wheelCenter[3]},
        rrRef = {pVehicle.pRrDW.wheelCenter[1], 0, pVehicle.pRrDW.wheelCenter[3]},
        pSprung = pVehicle.pSprungMass,
        pSprungMass = pVehicle.pSprungMass,
        torsionalStiff = pVehicle.pTorsionalStiff)));

  annotation(Documentation(info = "<html>
<p>
Configurable BobLib double-wishbone chassis exposed as a VehicleInterfaces
two-axle chassis. This keeps the detailed BobLib suspension, tire, and frame
physics inside the VI chassis boundary while allowing experiment templates to
select front and rear axle variants.
</p>
</html>"));
end Chassis_DW;
