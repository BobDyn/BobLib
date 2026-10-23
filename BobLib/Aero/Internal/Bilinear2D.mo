within BobLib.Aero.Internal;

pure function Bilinear2D "Clamped bilinear interpolation on a regular grid"
  input Real x "First axis coordinate";
  input Real y "Second axis coordinate";
  input Real xGrid[:] "Monotonic first-axis breakpoints";
  input Real yGrid[:] "Monotonic second-axis breakpoints";
  input Real table[size(xGrid, 1), size(yGrid, 1)] "Data table";
  input Integer nX = size(xGrid, 1) "Active first-axis points";
  input Integer nY = size(yGrid, 1) "Active second-axis points";
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
  assert(
    nX >= 2 and nX <= size(xGrid, 1) and nY >= 2 and nY <= size(yGrid, 1),
    "Bilinear2D active counts must be between two and the axis capacity");
  for i in 2:nX loop
    assert(xGrid[i] > xGrid[i - 1], "Bilinear2D xGrid must be strictly increasing");
  end for;
  for i in 2:nY loop
    assert(yGrid[i] > yGrid[i - 1], "Bilinear2D yGrid must be strictly increasing");
  end for;
  ix := 1;
  while ix < nX - 1 and x > xGrid[ix + 1] loop
    ix := ix + 1;
  end while;

  iy := 1;
  while iy < nY - 1 and y > yGrid[iy + 1] loop
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

  annotation(
    Inline = true,
    Documentation(info = "<html>
<p>
Function <code>Bilinear2D</code>: clamped bilinear interpolation on a regular
grid.
</p>
<p>
This helper evaluates the aerodynamic ride-height maps. Inputs outside the
provided grid are clamped to the nearest cell edge so the aero subsystem has a
defined output throughout the simulation envelope.
</p>
</html>"));
end Bilinear2D;
