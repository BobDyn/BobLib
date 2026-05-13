within BobLib.Utilities.Mechanics;

function translateInertia
  import Modelica.SIunits;

  input SIunits.Inertia inertia[3, 3] "Inertia about the body's local CG";
  input SIunits.Mass mass "Body mass";
  input SIunits.Position delta[3] "Vector from target reference point to the body's CG";
  output SIunits.Inertia translated[3, 3] "Inertia translated to the target reference point";
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
end translateInertia;
