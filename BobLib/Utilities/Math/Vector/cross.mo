within BobLib.Utilities.Math.Vector;

function cross

  input Real a[3];
  input Real b[3];
  output Real result[3];

algorithm
  result := {
    a[2]*b[3] - a[3]*b[2],
    a[3]*b[1] - a[1]*b[3],
    a[1]*b[2] - a[2]*b[1]
  };
  annotation(
    Documentation(info = "<html>
<p>
Function <code>cross</code> computes the three-dimensional vector cross product.
</p>
<p>
The helper supports frame, moment, and geometry calculations without tying those calculations to a subsystem package.
</p>
</html>"));
end cross;
