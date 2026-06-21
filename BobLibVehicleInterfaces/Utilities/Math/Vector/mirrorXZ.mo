within BobLibVehicleInterfaces.Utilities.Math.Vector;

function mirrorXZ

  input Real r[3] "Position vector {x, y, z}";
  output Real r_m[3] "Mirrored vector about X-Z plane (flip Y)";

algorithm
  r_m[1] := r[1];
  r_m[2] := -r[2];
  r_m[3] := r[3];

  annotation(
    Documentation(info = "<html>
<p>
Function <code>mirrorXZ</code> mirrors a vector across the XZ plane.
</p>
<p>
It supports left/right suspension symmetry by reflecting lateral coordinates and directions.
</p>
</html>"));
end mirrorXZ;