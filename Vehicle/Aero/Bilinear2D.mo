within BobLib.Vehicle.Aero;

pure function Bilinear2D "Clamped bilinear interpolation on a regular grid"
  input Real x "First axis coordinate";
  input Real y "Second axis coordinate";
  input Real xGrid[:] "Monotonic first-axis breakpoints";
  input Real yGrid[:] "Monotonic second-axis breakpoints";
  input Real table[size(xGrid, 1), size(yGrid, 1)] "Data table";
  output Real z "Interpolated value";
protected
  Integer ix;
  Integer iy;
  Real xLo;
  Real xHi;
  Real yLo;
  Real yHi;
  Real xFrac;
  Real yFrac;
algorithm
  ix := 1;
  while ix < size(xGrid, 1) - 1 and x > xGrid[ix + 1] loop
    ix := ix + 1;
  end while;

  iy := 1;
  while iy < size(yGrid, 1) - 1 and y > yGrid[iy + 1] loop
    iy := iy + 1;
  end while;

  xLo := xGrid[ix];
  xHi := xGrid[ix + 1];
  yLo := yGrid[iy];
  yHi := yGrid[iy + 1];

  xFrac := if xHi > xLo then min(max(x, xLo), xHi) - xLo else 0;
  yFrac := if yHi > yLo then min(max(y, yLo), yHi) - yLo else 0;

  if xHi > xLo then
    xFrac := xFrac / (xHi - xLo);
  else
    xFrac := 0;
  end if;

  if yHi > yLo then
    yFrac := yFrac / (yHi - yLo);
  else
    yFrac := 0;
  end if;

  z :=
    (1 - xFrac) * (1 - yFrac) * table[ix, iy] +
    (1 - xFrac) * yFrac * table[ix, iy + 1] +
    xFrac * (1 - yFrac) * table[ix + 1, iy] +
    xFrac * yFrac * table[ix + 1, iy + 1];

  annotation(Inline = true);
end Bilinear2D;
