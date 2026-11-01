within BobLib.Experiments.Standards.Examples;
model PhysicalPedalVehicle "Native VI driver with physical pedal compliance and friction-only braking"

  parameter Modelica.Units.SI.Length pedalFullTravel = 0.05 "Pedal displacement corresponding to a normalized brake demand of one" annotation(Dialog(group = "Operating settings"));

  extends BobLib.Experiments.Standards.VehicleSim(
    driverEnvironment(usePhysicalBrakePedal = true, pedalFullTravel = pedalFullTravel),
    redeclare BobLib.Brakes.PedalBrakes brakes(pBrakes(maxTorque = pVehicle.controller.mechanicalBrakeTorqueLimit, pedalFullTravel = pedalFullTravel)),
    vcu(final enableRegenerativeBraking = false, final enableMechanicalBraking = false));

equation
  connect(driverEnvironment.brakePedal, brakes.brakePedal) annotation(
    Line(points = {{48, 40}, {48, 36}, {175, 36}, {175, -24}, {155, -24}, {155, -34}}, color = {0, 127, 0}));
  annotation(Documentation(info = "<html><p>The standard driver requests normalized brake intent. ExternalDriver converts it to physical pedal travel, PedalBrakes supplies force feedback and friction torque, and the shared VI wheel-bearing paths carry caliper reactions. The VCU's regen and friction requests are disabled so no second brake path is active. Supports are always active. Set includeRotorEffects=true to include rotor gyroscopic and carrier-acceleration effects.</p></html>"));
end PhysicalPedalVehicle;
