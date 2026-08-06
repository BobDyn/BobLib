within BobLib.Experiments.Standards;

model VehicleSim

  "VehicleInterfaces-aligned BobLib vehicle simulation entrypoint"

  extends Templates.Vehicle.VehicleSim_EVBatInvMotDiff_DWBCStabar_DWBCStabar(
    chassis(
      chassisReferencePosition = pVehicle.pQSSInitialization.chassisReferencePosition,
      chassisReferenceAngles = pVehicle.pQSSInitialization.chassisReferenceAngles,
      fixInitialSuspensionAngles = true,
      initialFrontLeftLowerArmAngle = pVehicle.pQSSInitialization.frontLeftLowerArmAngle,
      initialFrontRightLowerArmAngle = pVehicle.pQSSInitialization.frontRightLowerArmAngle,
      initialRearLeftLowerArmAngle = pVehicle.pQSSInitialization.rearLeftLowerArmAngle,
      initialRearRightLowerArmAngle = pVehicle.pQSSInitialization.rearRightLowerArmAngle));
  extends BobLib.Icons.SimulationIcon;

initial equation
  assert(
    abs(initialVel - pVehicle.pQSSInitialization.referenceVelocity) < 1e-6,
    "VehicleSim QSS initialization was solved at a different initial velocity",
    AssertionLevel.warning);

  annotation(
    experiment(StartTime = 0.0, StopTime = 10, Tolerance = 1e-06, Interval = 0.002),
    __OpenModelica_commandLineOptions = "--matchingAlgorithm=PFPlusExt --indexReductionMethod=dynamicStateSelection -d=initialization,NLSanalyticJacobian,disableStartCalc --maxSizeLinearTearing=5000 --generateDynamicJacobian=none",
    __OpenModelica_simulationFlags(
      jacobian = "internalNumerical",
      lv = "LOG_STDOUT,LOG_ASSERT,LOG_STATS",
      noEquidistantTimeGrid = "()",
      noEventEmit = "()",
      s = "dassl",
      variableFilter = "time|frSteerCmd|accX|accY|handwheelAngle|steerExcess|handwheelTorque|Fz_.*|leftSteerAngle|rightSteerAngle|roll|sideslip|velX|velY|yawVel|steadyState.*|linearity.*|minTireNormalLoad|chassis.(front|rear)(Left|Right)SpringLength"),
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
The chassis starts from the coupled pose stored in
<code>pVehicle.pQSSInitialization</code>. Its lower-arm coordinates were solved
with the four-post rig at the settled front and rear corner loads, so the tire
contact, spring lengths, chassis heave, and chassis pitch are mutually
consistent at initialization.
</p>
<p>
The subsystem redeclare stack lives in
<code>Templates.Vehicle.VehicleSim_EVBatInvMotDiff_DWBCStabar_DWBCStabar</code>.
The shared vehicle template owns plant wiring and maneuver termination monitors,
while <code>Controllers.StandardVCU</code> owns the maneuver excitation, PTN
speed-control toggles, controller gains, target velocity, generated torque,
regenerative limit, and pedal commands.
</p>
</html>"));
end VehicleSim;
