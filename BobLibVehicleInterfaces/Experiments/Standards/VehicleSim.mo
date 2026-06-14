within BobLibVehicleInterfaces.Experiments.Standards;

model VehicleSim
  "VehicleInterfaces-aligned BobLib vehicle simulation entrypoint"
  extends Templates.BaseVehicleSim(
    redeclare record VehicleRecord =
      BobLibVehicleInterfaces.Records.VehicleDefn.DWBCStabar_DWBCStabarRecord,
    redeclare BobLibVehicleInterfaces.Chassis.Chassis_DWBCStabar_DWBCStabar chassis(
      headless = headless,
      pVehicle = pVehicle),
    redeclare BobLibVehicleInterfaces.EnergyStorage.BatteryPack battery(
      includeGround = true,
      Ns = pVehicle.pBattery.Ns,
      Np = pVehicle.pBattery.Np,
      SOC_start = pVehicle.pBattery.SOC_start),
    redeclare BobLibVehicleInterfaces.Controllers.VCU vcu(
      tau_max = pVehicle.pVCU.tau_max,
      w_eps = pVehicle.pVCU.w_eps,
      motorSpeedSign = pVehicle.pVCU.motorSpeedSign),
    redeclare BobLibVehicleInterfaces.PowerElectronics.InverterDC inverter(
      P_max_mot = pVehicle.pInverter.P_max_mot,
      P_max_reg = pVehicle.pInverter.P_max_reg,
      V_dc_max = pVehicle.pInverter.V_dc_max),
    redeclare BobLibVehicleInterfaces.ElectricDrives.Motor motor(
      Vdc_max = pVehicle.pMotor.Vdc_max,
      rpm_max_peak = pVehicle.pMotor.rpm_max_peak,
      T_peak = pVehicle.pMotor.T_peak,
      T_cont = pVehicle.pMotor.T_cont,
      I_peak_2min = pVehicle.pMotor.I_peak_2min,
      I_cont = pVehicle.pMotor.I_cont,
      Kt_Nm_per_A = pVehicle.pMotor.Kt_Nm_per_A,
      peakTime = pVehicle.pMotor.peakTime,
      P_mech_peak = pVehicle.pMotor.P_mech_peak,
      P_cont_low = pVehicle.pMotor.P_cont_low,
      P_cont_high = pVehicle.pMotor.P_cont_high,
      eta_mot = pVehicle.pMotor.eta_mot,
      eta_reg = pVehicle.pMotor.eta_reg,
      w_eps = pVehicle.pMotor.w_eps,
      rotorJ = pVehicle.pMotor.rotorJ),
    redeclare BobLibVehicleInterfaces.Drivelines.RearFinalDriveDifferential driveline(
      finalDriveRatio = pVehicle.pDriveline.finalDriveRatio,
      diffInputRotorJ = pVehicle.pDriveline.diffInputRotorJ,
      diff_use_lsd = pVehicle.pDriveline.diff_use_lsd,
      diff_driveSideTorqueSign = pVehicle.pDriveline.diff_driveSideTorqueSign,
      diff_T_preload = pVehicle.pDriveline.diff_T_preload,
      diff_lockFractionAccel = pVehicle.pDriveline.diff_lockFractionAccel,
      diff_lockFractionDecel = pVehicle.pDriveline.diff_lockFractionDecel,
      diff_T_capacity_max = pVehicle.pDriveline.diff_T_capacity_max,
      diff_clutchEffectiveRadius = pVehicle.pDriveline.diff_clutchEffectiveRadius,
      diff_kineticFrictionRatio = pVehicle.pDriveline.diff_kineticFrictionRatio,
      diff_w_transition = pVehicle.pDriveline.diff_w_transition,
      diff_c_viscous = pVehicle.pDriveline.diff_c_viscous,
      halfshaftLeftC = pVehicle.pDriveline.halfshaftLeftC,
      halfshaftLeftD = pVehicle.pDriveline.halfshaftLeftD,
      halfshaftRightC = pVehicle.pDriveline.halfshaftRightC,
      halfshaftRightD = pVehicle.pDriveline.halfshaftRightD),
    redeclare VehicleInterfaces.Brakes.MinimalBrakes brakes(
      maxTorque = 1500),
    redeclare VehicleInterfaces.DriverEnvironments.DriveByWireAutomatic driverEnvironment(
      initialAccelRequest = 0,
      finalAccelRequest = 0,
      accelTime = 0.1,
      initialBrakeRequest = 0,
      finalBrakeRequest = 0,
      brakeTime = 100),
    world(
      enableAnimation = not headless,
      n = {0, 0, -1},
      driveTrainMechanics3D = false));

  annotation(
    experiment(StartTime = 0.0, StopTime = 10, Tolerance = 1e-06, Interval = 0.002),
    __OpenModelica_commandLineOptions = "--matchingAlgorithm=PFPlusExt --indexReductionMethod=dynamicStateSelection -d=initialization,NLSanalyticJacobian --maxSizeLinearTearing=5000 --generateDynamicJacobian=none",
    __OpenModelica_simulationFlags(
      jacobian = "internalNumerical",
      lv = "LOG_STDOUT,LOG_ASSERT,LOG_STATS",
      noEquidistantTimeGrid = "()",
      noEventEmit = "()",
      s = "dassl",
      variableFilter = ".*"),
    Documentation(info = "<html>
<p>
Primary entrypoint for the BobLib-native detailed chassis, suspension, tire,
contact-patch, and powertrain simulation. The assembly follows the
VehicleInterfaces demo stack while exposing the BobLib powertrain as explicit
battery, VCU, power-electronics, motor, and driveline subsystem models at the
vehicle-simulation level. The aero subsystem is also fed by the
VehicleInterfaces atmosphere model through explicit density and relative
airspeed signals.
</p>
</html>"));
end VehicleSim;
