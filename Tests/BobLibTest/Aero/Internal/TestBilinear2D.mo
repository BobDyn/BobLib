within BobLibTest.Aero.Internal;

model TestBilinear2D

  discrete Real zInterior;
  discrete Real zClamped;
  discrete Real zPadded;

algorithm
  when initial() then
    zInterior := BobLib.Aero.Internal.Bilinear2D(
      0.5,
      2.0,
      {0.0, 1.0},
      {1.0, 3.0},
      [10.0, 30.0; 20.0, 40.0]);
    zClamped := BobLib.Aero.Internal.Bilinear2D(
      -1.0,
      10.0,
      {0.0, 1.0},
      {1.0, 3.0},
      [10.0, 30.0; 20.0, 40.0]);

    assert(abs(zInterior - 25.0) < 1e-9, "Bilinear interpolation changed");
    assert(abs(zClamped - 30.0) < 1e-9, "Bilinear edge clamping changed");
    zPadded := BobLib.Aero.Internal.Bilinear2D(
      0.5, 2.0, {0, 1, -999}, {1, 3, -999},
      [10, 30, 999; 20, 40, 999; 999, 999, 999], 2, 2);
    assert(abs(zPadded - zInterior) < 1e-9, "Unused aero cells affected interpolation");
  end when;

  annotation(
    experiment(StartTime = 0, StopTime = 0.01, Tolerance = 1e-06, Interval = 0.01));
end TestBilinear2D;
