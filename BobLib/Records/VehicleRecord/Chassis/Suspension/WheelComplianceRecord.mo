within BobLib.Records.VehicleRecord.Chassis.Suspension;
record WheelComplianceRecord "Wheel-stack elastic and damping calibration"

  import Modelica.Units.SI;
  parameter SI.TranslationalSpringConstant translationStiffness[3] = fill(
    1e7,
    3) "Local aligned-wheel x, y, z translational stiffness; illustrative defaults, replace with measured rates";
  parameter SI.RotationalSpringConstant rotationStiffness[3] = fill(
    1e5,
    3) "Successive local x, y, z rotational stiffness; intended for small wheel-stack deflections";
  parameter Real dampingRatio(min = 0) = 0.7 "Modal damping ratio used to derive parallel dampers";
  parameter SI.Mass referenceMass(min = Modelica.Constants.small) = 10 "Effective mass used only to derive damping; does not add physical wheel mass";
  parameter SI.Inertia referenceInertia[3] = fill(
    0.1,
    3) "Effective inertias used only to derive rotational damping; do not add physical inertia";
  final parameter SI.TranslationalDampingConstant translationDamping[3] = {2*dampingRatio*sqrt(translationStiffness[i]*referenceMass) for i in 1:3} "Derived translational damping; rigs disable its application without changing calibration";
  final parameter SI.RotationalDampingConstant rotationDamping[3] = {2*dampingRatio*sqrt(rotationStiffness[i]*referenceInertia[i]) for i in 1:3} "Derived rotational damping; rigs disable its application without changing calibration";
end WheelComplianceRecord;
