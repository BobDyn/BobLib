within BobLibVehicleInterfaces.Experiments.Standards;

model VehicleSim

  "VehicleInterfaces-aligned BobLib vehicle simulation entrypoint"
  extends Templates.Vehicle.VehicleSim_EVBatInvMotDiff_DWBCStabar_DWBCStabar;
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
      variableFilter = "time|frSteerCmd|accX|accY|handwheelAngle|steerExcess|handwheelTorque|Fz_.*|leftSteerAngle|rightSteerAngle|roll|sideslip|velX|velY|yawVel|steadyState.*|linearity.*|minTireNormalLoad"),
    Documentation(info = "<html>
<p>
Primary entrypoint for the BobLib-native EV battery-inverter-motor-differential
vehicle simulation with front and rear bellcrank-actuated double wishbone
suspension with stabilizer bars. The assembly follows the VehicleInterfaces
demo stack while exposing the BobLib powertrain as explicit battery, VCU,
power-electronics, motor, transmission, and driveline subsystem models at the
vehicle-simulation level.
</p>
<p>
The subsystem redeclare stack lives in
<code>Templates.Vehicle.VehicleSim_EVBatInvMotDiff_DWBCStabar_DWBCStabar</code>.
The shared vehicle template owns maneuver excitation and plant wiring, while
the VCU owns the PTN speed-control toggles, controller gains, target velocity,
generated torque, regenerative limit, and pedal commands.
</p>
</html>"));
end VehicleSim;
