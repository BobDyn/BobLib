within BobLibVehicleInterfaces.Experiments.Standards;

model VehicleFMI

  "Driver-input-only BobLib vehicle entrypoint for FMI, DIL, and early SIL coupling"

  extends Templates.FMI.BaseVehicleFMI(
    redeclare record VehicleRecord = BobLibVehicleInterfaces.Records.VehicleDefn.EVBatInvMotDiff_DWBCStabar_DWBCStabarRecord,
    redeclare BobLibVehicleInterfaces.Chassis.Chassis_DW chassis(
      redeclare record VehicleRecord = BobLibVehicleInterfaces.Records.VehicleDefn.EVBatInvMotDiff_DWBCStabar_DWBCStabarRecord,
      redeclare model FrAxleModel = BobLibVehicleInterfaces.Chassis.Suspension.FrAxleDW_BC_Stabar,
      redeclare model RrAxleModel = BobLibVehicleInterfaces.Chassis.Suspension.RrAxleDW_BC_Stabar,
      pFrStabar(
        leftArmEnd = pVehicle.pFrStabar.leftArmEnd,
        leftBarEnd = pVehicle.pFrStabar.leftBarEnd,
        barRate = pVehicle.pFrStabar.barRate),
      pRrStabar(
        leftArmEnd = pVehicle.pRrStabar.leftArmEnd,
        leftBarEnd = pVehicle.pRrStabar.leftBarEnd,
        barRate = pVehicle.pRrStabar.barRate),
      headless = headless,
      initialLongitudinalVelocity = initialVel,
      pVehicle = pVehicle));
  extends BobLibVehicleInterfaces.Icons.SimulationIcon;

  annotation(
    experiment(StartTime = 0.0, StopTime = 10, Tolerance = 1e-06, Interval = 0.002),
    __OpenModelica_commandLineOptions = "--matchingAlgorithm=PFPlusExt --indexReductionMethod=dynamicStateSelection -d=initialization,NLSanalyticJacobian,disableStartCalc --maxSizeLinearTearing=5000 --generateDynamicJacobian=none",
    __OpenModelica_simulationFlags(
      jacobian = "internalNumerical",
      lv = "LOG_STDOUT,LOG_ASSERT,LOG_STATS",
      noEquidistantTimeGrid = "()",
      noEventEmit = "()",
      s = "dassl",
      variableFilter = "time|.*Command|vehicleSpeed|accX|accY|handwheelAngle|steerExcess|handwheelTorque|Fz_.*|leftSteerAngle|rightSteerAngle|roll|sideslip|velX|velY|yawVel|vcu\\..*|brakes\\..*"),
    Documentation(info = "<html>
<p>
Model <code>VehicleFMI</code> is the Standards entry point for a
driver-input-only full vehicle. It is intended for FMI export,
driver-in-the-loop, hardware-driver coupling, and earlier software-in-the-loop
workflows where an external source owns the driver commands.
</p>
<p>
The public signal boundary is limited to driver-environment commands:
steering-wheel angle, accelerator pedal, and brake pedal. The configured BobLib
VCU and EV plant remain inside the model, EV ready-to-drive/direct-torque
defaults are internal, and subsystems exchange information through the internal
VehicleInterfaces control bus.
</p>
</html>"));
end VehicleFMI;
