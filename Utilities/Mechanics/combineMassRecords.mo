within BobLib.Utilities.Mechanics;

function combineMassRecords
  import Modelica.SIunits;
  import BobLib.Resources.VehicleRecord.Chassis.Suspension.Templates.MassRecord;

  input MassRecord masses[:] "Mass records to combine";
  output MassRecord combined;

protected
  SIunits.Mass totalMass;
  SIunits.Position weightedCM[3];
  SIunits.Position delta[3];
  SIunits.Inertia translatedInertia[3, 3];
  SIunits.Inertia totalInertia[3, 3];
  Integer count = size(masses, 1);
algorithm
  totalMass := 0;
  weightedCM := {0, 0, 0};
  totalInertia := {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

  for i in 1:count loop
    totalMass := totalMass + masses[i].m;
    weightedCM := weightedCM + masses[i].m * masses[i].rCM;
  end for;

  if totalMass <= 0 then
    combined := MassRecord(
      m = 0,
      rCM = {0, 0, 0},
      inertia = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}});
  else
    weightedCM := weightedCM / totalMass;

    for i in 1:count loop
      delta := masses[i].rCM - weightedCM;
      translatedInertia := {
        {delta[2] * delta[2] + delta[3] * delta[3], -delta[1] * delta[2], -delta[1] * delta[3]},
        {-delta[2] * delta[1], delta[1] * delta[1] + delta[3] * delta[3], -delta[2] * delta[3]},
        {-delta[3] * delta[1], -delta[3] * delta[2], delta[1] * delta[1] + delta[2] * delta[2]}
      };
      totalInertia := totalInertia + masses[i].inertia + masses[i].m * translatedInertia;
    end for;

    combined := MassRecord(
      m = totalMass,
      rCM = weightedCM,
      inertia = totalInertia);
  end if;
end combineMassRecords;
