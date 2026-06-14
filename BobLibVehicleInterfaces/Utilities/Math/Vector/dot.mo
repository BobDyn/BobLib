within BobLibVehicleInterfaces.Utilities.Math.Vector;

function dot
  input Real[:] a;
  input Real[:] b;
  output Real result;
algorithm
  result := sum(a[i] * b[i] for i in 1:size(a,1));
  annotation(
    Documentation(info = "<html>
<p>
Function <code>dot</code> computes the vector dot product used by geometry and mechanics helpers.
</p>
<p>
It lives in utilities so suspension, tire, and chassis models can share the same small vector operation.
</p>
</html>"));
end dot;
