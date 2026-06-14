within BobLibVehicleInterfaces.Experiments.Standards;

model VehicleSim
  "VehicleInterfaces-aligned BobLib vehicle simulation entrypoint"
  extends Architectures.BatteryElectricRearDrive;
  extends BobLibVehicleInterfaces.Icons.SimulationIcon;

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
battery, VCU, power-electronics, motor, transmission, and driveline subsystem
models at the vehicle-simulation level. The simulation is autonomous by
default: it defines the maneuver steering, accelerator, brake, gear,
gearbox-mode, ignition, motor-torque, regen-limit, and inverter-enable
commands, sends the driver-level commands directly to the VCU, and publishes
the appropriate VehicleInterfaces driver-bus signals for subsystems such as
mechanical brakes. The aero subsystem is fed by the BobLib atmosphere adapter
through explicit density and relative airspeed signals.
</p>
<p>
The subsystem redeclare stack lives in
<code>Architectures.BatteryElectricRearDrive</code>. This model preserves the
stable benchmark name and owns the experiment settings used for the primary
full-vehicle simulation.
</p>
</html>"));
end VehicleSim;
