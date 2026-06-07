within BobLib.Tests.TestVehicle.TestAero;

model TestBilinear2D
  discrete Real zInterior;
  discrete Real zClamped;

algorithm
  when initial() then
    zInterior := BobLib.Vehicle.Aero.Bilinear2D(
      0.5,
      2.0,
      {0.0, 1.0},
      {1.0, 3.0},
      [10.0, 30.0; 20.0, 40.0]);
    zClamped := BobLib.Vehicle.Aero.Bilinear2D(
      -1.0,
      10.0,
      {0.0, 1.0},
      {1.0, 3.0},
      [10.0, 30.0; 20.0, 40.0]);

    assert(abs(zInterior - 25.0) < 1e-9, "Bilinear interpolation changed");
    assert(abs(zClamped - 30.0) < 1e-9, "Bilinear edge clamping changed");
  end when;

  annotation(
    experiment(StartTime = 0, StopTime = 0.01, Tolerance = 1e-06, Interval = 0.01));
end TestBilinear2D;
