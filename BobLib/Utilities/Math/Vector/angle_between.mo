within BobLib.Utilities.Math.Vector;

function angle_between

  import BobLib.Utilities.Math.Vector.cross;
  import BobLib.Utilities.Math.Vector.dot;

  input Real a[3] "First vector";
  input Real b[3] "Second vector";
  input Real n[3] "Reference axis selecting the angle orientation";
  output Real theta "Signed principal angle from a to b";

protected
  Real num;
  Real den;
  Real aScale;
  Real bScale;
  Real nScale;
  Real aUnit[3];
  Real bUnit[3];
  Real nUnit[3];
  Real crossUnit[3];
  Real orientation;

algorithm

  // Signed principal angle using atan2. The cross-product magnitude supplies
  // sin(theta); the reference axis selects its sign without changing the
  // angle magnitude when the axis is not parallel to the cross product.

  // Scale before normalization so small but nonzero vectors do not underflow.
  // Normalizing the rotation axis also makes its magnitude irrelevant.
  aScale := max(abs(a));
  bScale := max(abs(b));
  nScale := max(abs(n));

  if aScale > 0 and bScale > 0 and nScale > 0 then
    aUnit := (a/aScale)/sqrt(dot(a/aScale, a/aScale));
    bUnit := (b/bScale)/sqrt(dot(b/bScale, b/bScale));
    nUnit := (n/nScale)/sqrt(dot(n/nScale, n/nScale));
    crossUnit := cross(aUnit, bUnit);

    num := sqrt(max(0, dot(crossUnit, crossUnit)));
    den := max(-1, min(1, dot(aUnit, bUnit)));
    orientation := dot(nUnit, crossUnit);

    if orientation < 0 then
      num := -num;
    end if;
    theta := atan2(num, den);
  else

    // The angle of a zero-length vector or about a zero-length axis is
    // undefined. Return the neutral angle explicitly instead of perturbing
    // every valid result with a numerical floor.
    num := 0;
    den := 0;
    aUnit := zeros(3);
    bUnit := zeros(3);
    nUnit := zeros(3);
    crossUnit := zeros(3);
    orientation := 0;
    theta := 0;
  end if;

  annotation(
    Documentation(info = "<html>
<p>
Function <code>angle_between</code> returns the angle between two vectors.
</p>
<p>
It normalizes the geometric calculation for linkage and suspension setup code.
The result is invariant to vector and reference-axis magnitude. The angle
magnitude is the principal three-dimensional angle between the vectors; the
reference axis selects its sign through the orientation of the cross product.
If either vector or the reference axis has zero length, the function returns
zero. If the cross product is orthogonal to the reference axis, the principal
angle is returned with a nonnegative sign.
</p>
</html>"));
end angle_between;
