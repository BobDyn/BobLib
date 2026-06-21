within BobLibVehicleInterfaces.Chassis.Suspension.Tires.MF52.SlipModel;

model NoSlip

  extends BobLibVehicleInterfaces.Chassis.Suspension.Tires.MF52.SlipModel.BaseSlipModel;

equation
  alpha = 0;
  kappa = 0;

  annotation(
    Documentation(info = "<html>
<p>
Model <code>NoSlip</code> supplies a zero-slip tire state.
</p>
<p>
It is useful for kinematic checks, four-post tests, and model variants where tire force generation should not depend on slip dynamics.
</p>
</html>"));
end NoSlip;
