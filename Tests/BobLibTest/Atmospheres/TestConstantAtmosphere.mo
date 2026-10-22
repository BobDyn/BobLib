within BobLibTest.Atmospheres;

model TestConstantAtmosphere

  BobLib.Atmospheres.ConstantAtmosphere dut(v = {1, 2, 3}) annotation(Placement(transformation(origin = {0, 0}, extent = {{-10, -5}, {10, 5}})));
  output Real wind[3] = dut.windVelocity({0, 0, 0});
  output Real rho = dut.density({0, 0, 0});

equation
  assert(abs(rho - 101300/(287.0512249529787*293.15)) < 1e-10, "Ideal-gas density");
  assert(abs(wind[2] - 2) < 1e-10, "VI wind function");
  annotation(experiment(StopTime = 0.1, Tolerance = 1e-6),
    Diagram(coordinateSystem(extent = {{-60, -40}, {60, 40}}, preserveAspectRatio = true)));
end TestConstantAtmosphere;
