within BobLib.Vehicle.Chassis.Suspension.Templates.Tire;

model MF52Tire
  import SI = Modelica.Units.SI;

  import BobLib.Vehicle.Chassis.Suspension.Templates.Tire.MF52;

  import BobLib.Resources.VehicleRecord.Chassis.Suspension.Templates.Tire.MF52.MF52Record;

  extends BobLib.Vehicle.Chassis.Suspension.Templates.Tire.BaseTire(realExpressionFx(y = Fx),
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

end MF52Tire;
