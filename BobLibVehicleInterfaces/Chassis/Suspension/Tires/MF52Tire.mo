within BobLibVehicleInterfaces.Chassis.Suspension.Tires;

model MF52Tire
  import SI = Modelica.Units.SI;

  import BobLibVehicleInterfaces.Chassis.Suspension.Tires.MF52;

  import BobLibVehicleInterfaces.Records.VehicleRecord.Chassis.Suspension.Templates.Tire.MF52.MF52Record;

  extends BobLibVehicleInterfaces.Chassis.Suspension.Tires.BaseTire(realExpressionFx(y = Fx),
                                                                    realExpressionFy(y = Fy),
                                                                    realExpressionMx(y = Mx),
                                                                    realExpressionMy(y = My),
                                                                    realExpressionMz(y = Mz));

  // Record parameters
  parameter MF52Record pTireModel;

  SI.Force Fx;
  SI.Force Fy;
  SI.Torque Mx;
  SI.Torque My;
  SI.Torque Mz;

  SI.Length t;
  SI.Length s;

equation
  // MF52 force and moment evaluation
  (Fx, Fy, Mx, My, Mz, t, s) =
    MF52.Eval(Fz, alpha, kappa, gamma, Vx, pTireModel);

  annotation(
    Documentation(info = "<html>
<p>
Model <code>MF52Tire</code> is the concrete Magic Formula 5.2 tire implementation.
</p>
<p>
It evaluates slip state, normal load, contact kinematics, and MF5.2 force and moment equations for use inside axle assemblies.
</p>
</html>"));
end MF52Tire;
