within BobLibVehicleInterfaces.Utilities.Math.Tensor;

function mirrorXZ

  input Real T[3, 3];
  output Real T_m[3, 3];

protected
  constant Real R[3, 3] = [1, 0, 0;
                          0, -1, 0;
                          0, 0, 1];

algorithm
  T_m := R * T * transpose(R);

  annotation(
    Documentation(info = "<html>
<p>
Function <code>mirrorXZ</code> mirrors a tensor across the XZ plane.
</p>
<p>
It is used when right-side suspension or mass-property data is derived from left-side definitions.
</p>
</html>"));
end mirrorXZ;
