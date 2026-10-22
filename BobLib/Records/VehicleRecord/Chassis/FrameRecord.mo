within BobLib.Records.VehicleRecord.Chassis;

record FrameRecord

  import Modelica.Units.SI;
  import BobLib.Utilities.Mechanics.Functions.translateInertia;

  parameter SI.Position frontReference[3] "Front axle mounting position in vehicle design coordinates";
  parameter SI.Position rearReference[3] "Rear axle mounting position in vehicle design coordinates";
  parameter Suspension.Templates.MassRecord baseMass "Sprung vehicle mass properties excluding the separately specified driver";

  parameter Suspension.Templates.MassRecord driverMass(
    m = 0,
    rCM = zeros(3),
    inertia = zeros(
      3,
      3)) "Driver mass properties in the same design coordinates as baseMass";

  // Bind fields separately so OMC preserves initialization-time dependencies.
  final parameter Suspension.Templates.MassRecord mass(
    m = max(0, baseMass.m + driverMass.m),
    rCM = if mass.m > 0 then (baseMass.m*baseMass.rCM + driverMass.m*driverMass.rCM)/mass.m else zeros(3),
    inertia = if mass.m > 0 then
      translateInertia(baseMass.inertia, baseMass.m, baseMass.rCM - mass.rCM) +
      translateInertia(driverMass.inertia, driverMass.m, driverMass.rCM - mass.rCM)
        else zeros(3, 3)) "Combined sprung mass, center of mass and inertia derived from baseMass and driverMass";

  parameter SI.RotationalSpringConstant torsionalStiffness = 300000 "Sprung-frame torsional stiffness about the longitudinal axis";
  parameter SI.RotationalDampingConstant torsionalDamping = 2*sqrt(torsionalStiffness*mass.inertia[1, 1]) "Sprung-frame torsional damping about the longitudinal axis";

end FrameRecord;
