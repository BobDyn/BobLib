within BobLib.Atmospheres;
model ConstantAtmosphere "Validated VI constant atmosphere"

  extends VehicleInterfaces.Atmospheres.ConstantAtmosphere;

equation
  assert(T > 0 and ambientPressure > 0 and h >= 0 and h <= 1,
    "ConstantAtmosphere: positive absolute temperature/pressure and humidity in 0..1 required");
  annotation(Documentation(info = "<html><p>Uses the VI atmosphere function contract directly. Aerodynamic components query density and wind at their load point. No atmosphere signal bus is required.</p></html>"));
end ConstantAtmosphere;
