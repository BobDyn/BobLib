within BobLibVehicleInterfaces.Experiments.Standards.Architectures;
model BatteryElectricRearDrive
  "Battery-electric rear-drive architecture using explicit BobLib subsystem adapters"
  extends BobLibVehicleInterfaces.Experiments.Standards.Templates.BaseVehicleSim(
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
    redeclare BobLibVehicleInterfaces.Transmissions.FixedRatioTransmission transmission(
      gearRatio = pVehicle.pDriveline.finalDriveRatio),
    redeclare BobLibVehicleInterfaces.Drivelines.RearFinalDriveDifferential driveline(
      finalDriveRatio = 1,
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
    redeclare BobLibVehicleInterfaces.Atmospheres.ConstantAtmosphere atmosphere,
    world(
      enableAnimation = not headless,
      n = {0, 0, -1},
      driveTrainMechanics3D = false));

  annotation(Documentation(info = "<html>
<p>
Architecture model for the current BobLib battery-electric rear-drive vehicle.
The powertrain is exposed at the vehicle-assembly level as
<code>battery</code>, <code>inverter</code>, <code>motor</code>,
<code>transmission</code>, and <code>driveline</code>. The inherited vehicle
template owns the autonomous driver commands, sends them directly to the VCU,
and also publishes the VI driver-bus signals used by braking and other
subsystems. The atmosphere is a BobLib adapter that exposes explicit signal
outputs for aero wiring while preserving the VehicleInterfaces atmosphere
functions.
</p>
<p>
Use this class as the reference architecture for EV experiments. Runnable
benchmarks should extend it and add experiment settings instead of duplicating
the subsystem redeclare list.
</p>
</html>"));
end BatteryElectricRearDrive;
