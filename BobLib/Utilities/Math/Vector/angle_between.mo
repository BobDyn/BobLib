within BobLib.Utilities.Math.Vector;

function angle_between

  import BobLib.Utilities.Math.Vector.cross;
  import BobLib.Utilities.Math.Vector.dot;

  input Real a[3];
  input Real b[3];
  input Real n[3]; // axis of rotation
  output Real theta;

protected
  Real num;
  Real den;
  Real aScale;
  Real bScale;
  Real nScale;
  Real aUnit[3];
  Real bUnit[3];
  Real nUnit[3];

algorithm

  // Signed angle using atan2 formulation
  // theta = atan2( n · (a × b), a · b )
  //
  // This is:
  //  - continuous
  //  - signed
  //  - well-defined for all magnitudes
  //  - numerically stable near alignment
  //  - event-free

  // Scale before normalization so small but nonzero vectors do not underflow.
  // Normalizing the rotation axis also makes its magnitude irrelevant.
  aScale := max(abs(a));
  bScale := max(abs(b));
  nScale := max(abs(n));

  if aScale > 0 and bScale > 0 and nScale > 0 then
    aUnit := (a/aScale)/sqrt(dot(a/aScale, a/aScale));
    bUnit := (b/bScale)/sqrt(dot(b/bScale, b/bScale));
    nUnit := (n/nScale)/sqrt(dot(n/nScale, n/nScale));

    num := dot(nUnit, cross(aUnit, bUnit));
    den := dot(aUnit, bUnit);
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
    theta := 0;
  end if;

  annotation(
    Documentation(info = "<html>
<p>
Function <code>angle_between</code> returns the angle between two vectors.
</p>
<p>
It normalizes the geometric calculation for linkage and suspension setup code.
The result is invariant to vector magnitude. If either vector or the rotation
axis has zero length, the function returns zero.
</p>
</html>"));
end angle_between;
