within BobLibTest.Experiments.Standards.Examples;

model TestMixedSuspensionVehicle

  extends BobLib.Experiments.Standards.Examples.MixedSuspensionVehicle;

initial equation
  assert(abs(der(chassis.wheelHub_1.flange.phi)*pVehicle.chassis.frontAxle.tire.wheel.geometry.R0-initialVel) < 1e-6 and
    abs(der(chassis.wheelHub_2.flange.phi)*pVehicle.chassis.frontAxle.tire.wheel.geometry.R0-initialVel) < 1e-6,
    "The bellcrank front axle must initialize both wheels at the prescribed rolling speed");
  assert(abs(der(chassis.wheelHub_3.flange.phi)*pVehicle.chassis.rearAxle.tire.wheel.geometry.R0-initialVel) < 1e-6 and
    abs(der(chassis.wheelHub_4.flange.phi)*pVehicle.chassis.rearAxle.tire.wheel.geometry.R0-initialVel) < 1e-6,
    "The direct rear axle must initialize both wheels at the prescribed rolling speed");
  annotation(Documentation(info = "<html><p>Coverage fixture for <code>BobLib.Experiments.Standards.Examples.MixedSuspensionVehicle</code>.</p></html>"));
end TestMixedSuspensionVehicle;
