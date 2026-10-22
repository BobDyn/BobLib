within BobLib.Records.VehicleRecord.Drivelines;
record HalfshaftRecord

  import Modelica.Units.SI;

  parameter SI.RotationalSpringConstant stiffness(min = 0) = 15000 "Torsional stiffness of one halfshaft";
  parameter SI.Inertia inertia(min = Modelica.Constants.small) = 0.02 "Rotational inertia of one halfshaft";
  parameter SI.RotationalDampingConstant damping(min = 0) = 2*sqrt(stiffness*inertia) "Torsional damping of one halfshaft; default is critical damping for the lumped inertia";
end HalfshaftRecord;
