within BobLib.Brakes;
model PedalBrakes "Physical travel/compliance input with shared friction-brake actuation"

  extends BobLib.Brakes.Internal.BrakeActuation(final usePedal = true);
  annotation(Documentation(info = "<html><p>The VI brakePedal flange accepts physical travel in metres. Effective pedal/hydraulic compliance supplies force feedback; pedalFullTravel gives maxTorque, and brake demand is clamped to zero/maxTorque. Calibrate stiffness, damping and full travel at the pedal. There is no detailed hydraulic fluid, hard stop, or ABS model.</p><p>This variant consumes no VCU mechanical-torque request. Use it as a friction-only path with VCU regeneration disabled unless an explicit coordinated pedal/regen model is supplied. The scalar pedal housing is a chassis-relative reference; caliper supports retain the same spatial reaction paths as BasicVCUBrakes.</p></html>"));
end PedalBrakes;
