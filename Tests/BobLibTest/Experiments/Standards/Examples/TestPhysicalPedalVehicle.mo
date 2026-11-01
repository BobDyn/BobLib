within BobLibTest.Experiments.Standards.Examples;
model TestPhysicalPedalVehicle

  extends BobLib.Experiments.Standards.Examples.PhysicalPedalVehicle(
    headless = true,
    driver(targetVel = 10, enableBrakeSpeedControl = true, enableDriveSpeedControl = false));

equation
  assert(abs(vcu.mechanicalBrakeTorqueCmd) < 1e-9 and abs(vcu.regenBrakeTorqueCmd) < 1e-9,
    "Physical pedal braking must not have a second VCU brake path");
  assert(time <= 0.02 or (brakes.brakePedal.s > 0 and brakes.mechanicalBrakeTorqueRequest > 0),
    "The speed-reduction request must travel through the physical pedal into friction braking");
  assert(brakes.wheelHub_1.flange.tau*der(brakes.wheelHub_1.flange.phi) +
    brakes.wheelHub_2.flange.tau*der(brakes.wheelHub_2.flange.phi) +
    brakes.wheelHub_3.flange.tau*der(brakes.wheelHub_3.flange.phi) +
    brakes.wheelHub_4.flange.tau*der(brakes.wheelHub_4.flange.phi) >= -1e-6,
    "Friction brakes must absorb rather than deliver shaft power");
  annotation(experiment(StopTime = 0.1, Tolerance = 1e-6));
end TestPhysicalPedalVehicle;
