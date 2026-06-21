within BobLib.Utilities.Mechanics.Functions;

function translateInertia

  import SI = Modelica.Units.SI;

  input SI.Inertia inertia[3, 3] "Inertia about the body's local CG";
  input SI.Mass mass "Body mass";
  input SI.Position delta[3] "Vector from target reference point to the body's CG";
  output SI.Inertia translated[3, 3] "Inertia translated to the target reference point";

algorithm
  translated := inertia + mass * {
    {
      delta[2] * delta[2] + delta[3] * delta[3],
      -delta[1] * delta[2],
      -delta[1] * delta[3]
    },
    {
      -delta[2] * delta[1],
      delta[1] * delta[1] + delta[3] * delta[3],
      -delta[2] * delta[3]
    },
    {
      -delta[3] * delta[1],
      -delta[3] * delta[2],
      delta[1] * delta[1] + delta[2] * delta[2]
    }
  };
  annotation(
    Documentation(info = "<html>
<p>
Function <code>translateInertia</code> translates a 3-by-3 inertia tensor from
one reference point to another.
</p>
<p>
It applies the parallel-axis theorem and is shared by vehicle mass-property
calculations.
</p>
</html>"));
end translateInertia;
