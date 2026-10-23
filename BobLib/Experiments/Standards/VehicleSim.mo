within BobLib.Experiments.Standards;

model VehicleSim

  "VehicleInterfaces-aligned BobLib vehicle simulation entrypoint"

  extends BaseClasses.PartialVehicleSim(
    rearBrakePlacement = BobLib.Brakes.Types.Placement.Inboard,
    chassis(
      chassisReferencePosition = pInitialization.chassisReferencePosition,
      chassisReferenceAngles = pInitialization.chassisReferenceAngles,
      fixInitialSuspensionAngles = true,
      initialFrontLeftLowerArmAngle = pInitialization.frontLeftLowerArmAngle,
      initialFrontRightLowerArmAngle = pInitialization.frontRightLowerArmAngle,
      initialRearLeftLowerArmAngle = pInitialization.rearLeftLowerArmAngle,
      initialRearRightLowerArmAngle = pInitialization.rearRightLowerArmAngle));
  extends BobLib.Icons.SimulationIcon;

  parameter BobLib.Records.VehicleRecord.Chassis.QSSInitializationRecord pInitialization(
      referenceVelocity = 15,
      chassisReferencePosition = {-0.810146083618726, 1.742472521734604e-6, 0.281770875605249},
      chassisReferenceAngles = {0, 0.00104599750922714, 0},
      frontLeftLowerArmAngle = 0.0125840337509824,
      frontRightLowerArmAngle = -0.0125840337509825,
      rearLeftLowerArmAngle = 0.00868319343639186,
      rearRightLowerArmAngle = -0.00868319343639172,
      frontSpringLength = 0.175156854188570,
      rearSpringLength = 0.244911388707023,
      frontNormalLoad = 710.509308664321,
      rearNormalLoad = 721.422043064354)
    "Saved starting pose for the default vehicle; recompute after changing calibration" annotation(Dialog(group = "Initialization"));

initial equation
  assert(
    abs(initialVel - pInitialization.referenceVelocity) < 1e-6,
    "VehicleSim QSS initialization was solved at a different initial velocity",
    AssertionLevel.warning);

  annotation(
    experiment(StartTime = 0.0, StopTime = 10, Tolerance = 1e-06, Interval = 0.002),
    __OpenModelica_commandLineOptions = "--matchingAlgorithm=PFPlusExt --indexReductionMethod=dynamicStateSelection -d=initialization,NLSanalyticJacobian,disableStartCalc --maxSizeLinearTearing=5000 --generateDynamicJacobian=none --homotopyApproach=equidistantLocal",
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
suspension with stabilizer bars. Front brakes are outboard; rear brakes mount at the differential outputs, upstream of halfshaft compliance, with caliper reactions carried by the sprung differential housing. The assembly follows the VehicleInterfaces
demo stack while exposing the BobLib powertrain as explicit battery, VCU,
power-electronics, motor, transmission, and driveline subsystem models at the
vehicle-simulation level.
</p>
<p>The OpenModelica settings use local homotopy to avoid generating a second
complete initialization system. The standard QSS start initializes without
homotopy. For configurations that require coupled global homotopy, select
<code>--homotopyApproach=equidistantGlobal</code> in the translation settings
and override this model's command-line annotation.</p>
<p>
The chassis starts from the coupled pose stored in
<code>pInitialization</code>. Its lower-arm coordinates were solved
with the four-post rig at the settled front and rear corner loads, so the tire
contact, spring lengths, chassis heave, and chassis pitch are mutually
consistent at initialization.
</p>
<p>
The subsystem redeclare stack lives in
<code>BaseClasses.PartialElectricVehicle</code>.
The shared vehicle template owns plant wiring and maneuver termination monitors,
while <code>Drivers.StandardDriver</code> owns maneuver excitation, longitudinal speed control, and driver intent. <code>Controllers.VCU</code> owns torque limits, regenerative blending, and actuator requests.
</p>
</html>"));
end VehicleSim;
