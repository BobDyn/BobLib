within BobLib.Chassis.Suspension.Tires;

model MF52Tire

  import Modelica.Units.SI;
  import BobLib.Chassis.Suspension.Tires.MF52;

  extends BobLib.Chassis.Suspension.Tires.BaseTire(
    redeclare replaceable MF52.SlipModel.TransientSlip slipModel(pSlip = pTire.force.relaxation), final referenceRoadFriction = pTire.force.referenceRoadFriction,
                                                                    realExpressionFx(y = Fx),
                                                                    realExpressionFy(y = Fy),
                                                                    realExpressionMx(y = Mx),
                                                                    realExpressionMy(y = My),
                                                                    realExpressionMz(y = Mz));

  // Record parameters

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
    MF52.Eval(Fz, alpha, kappa, gamma, Vx, pTire.force);

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
