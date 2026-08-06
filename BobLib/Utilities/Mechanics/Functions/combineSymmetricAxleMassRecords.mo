within BobLib.Utilities.Mechanics.Functions;

function combineSymmetricAxleMassRecords

  import BobLib.Records.VehicleRecord.Chassis.Suspension.Templates.AxleMassRecord;
  import BobLib.Records.VehicleRecord.Chassis.Suspension.Templates.MassRecord;
  import BobLib.Utilities.Math.Tensor;
  import BobLib.Utilities.Math.Vector;

  input MassRecord sprungMass "Mass properties not represented by the axle sides";
  input AxleMassRecord axleMasses[:] "Left-side mass properties for each axle";
  output MassRecord combined "Combined vehicle mass properties";

protected
  MassRecord masses[1 + 8*size(axleMasses, 1)];
  Integer offset;

algorithm
  masses[1] := sprungMass;

  for axle in 1:size(axleMasses, 1) loop
    offset := 2 + 8*(axle - 1);

    masses[offset] := axleMasses[axle].unsprungMass;
    masses[offset + 1] := axleMasses[axle].ucaMass;
    masses[offset + 2] := axleMasses[axle].lcaMass;
    masses[offset + 3] := axleMasses[axle].tieMass;

    masses[offset + 4] := MassRecord(
      m = axleMasses[axle].unsprungMass.m,
      rCM = Vector.mirrorXZ(axleMasses[axle].unsprungMass.rCM),
      inertia = Tensor.mirrorXZ(axleMasses[axle].unsprungMass.inertia));
    masses[offset + 5] := MassRecord(
      m = axleMasses[axle].ucaMass.m,
      rCM = Vector.mirrorXZ(axleMasses[axle].ucaMass.rCM),
      inertia = Tensor.mirrorXZ(axleMasses[axle].ucaMass.inertia));
    masses[offset + 6] := MassRecord(
      m = axleMasses[axle].lcaMass.m,
      rCM = Vector.mirrorXZ(axleMasses[axle].lcaMass.rCM),
      inertia = Tensor.mirrorXZ(axleMasses[axle].lcaMass.inertia));
    masses[offset + 7] := MassRecord(
      m = axleMasses[axle].tieMass.m,
      rCM = Vector.mirrorXZ(axleMasses[axle].tieMass.rCM),
      inertia = Tensor.mirrorXZ(axleMasses[axle].tieMass.inertia));
  end for;

  combined := combineMassRecords(masses);

  annotation(
    Documentation(info = "<html>
<p>
Combines a sprung-mass record with each left-side axle mass record and its
reflection across the vehicle XZ plane.
</p>
</html>"));
end combineSymmetricAxleMassRecords;
