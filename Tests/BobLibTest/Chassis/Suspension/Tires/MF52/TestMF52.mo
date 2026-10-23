within BobLibTest.Chassis.Suspension.Tires.MF52;

model TestMF52

  import Modelica.Units.SI;
  import BobLib.Chassis.Suspension.Tires.MF52;
  import BobLib.Records.VehicleDefn;

  // Vehicle Definition
  parameter VehicleDefn.EVBatInvMotDiff_DWBCStabar_DWBCStabarRecord car "Vehicle calibration supplying the tire parameters for this test" annotation(Dialog(group = "Calibration"));

  // Operating Conditions
  parameter SI.Force Fz = 654 "Compressive tire normal load for the force-law evaluation" annotation(Dialog(group = "Operating settings"));
  parameter SI.Angle gamma = 0 "Tire camber angle used by the force-law evaluation, in radians" annotation(Dialog(group = "Operating settings"));
  parameter SI.Velocity Vx = 10 "Longitudinal tire speed used by the force-law evaluation" annotation(Dialog(group = "Operating settings"));

  // Sweep Definition
  parameter Integer nAlpha = 41 "Number of slip-angle samples in the tire-force sweep" annotation(Dialog(group = "Operating settings"));
  parameter Integer nKappa = 41 "Number of longitudinal slip samples in the tire-force sweep" annotation(Dialog(group = "Operating settings"));

  parameter SI.Angle alphaMin = -0.25 "Lower slip-angle limit of the tire-force sweep, in radians" annotation(Dialog(group = "Operating settings"));
  parameter SI.Angle alphaMax = 0.25 "Upper slip-angle limit of the tire-force sweep, in radians" annotation(Dialog(group = "Operating settings"));

  parameter Real kappaMin = -0.25 "Lower longitudinal slip-ratio limit of the tire-force sweep" annotation(Dialog(group = "Operating settings"));
  parameter Real kappaMax = 0.25 "Upper longitudinal slip-ratio limit of the tire-force sweep" annotation(Dialog(group = "Operating settings"));

  // Grids
  discrete SI.Angle alphaGrid[nAlpha](each fixed = true);
  discrete Real kappaGrid[nKappa](each fixed = true);

  // Outputs (full grid)
  discrete SI.Force Fx[nAlpha, nKappa](each fixed = true);
  discrete SI.Force Fy[nAlpha, nKappa](each fixed = true);

  discrete SI.Torque Mx[nAlpha, nKappa](each fixed = true);
  discrete SI.Torque My[nAlpha, nKappa](each fixed = true);
  discrete SI.Torque Mz[nAlpha, nKappa](each fixed = true);

  discrete SI.Length t[nAlpha, nKappa](each fixed = true);
  discrete SI.Length s[nAlpha, nKappa](each fixed = true);

  // Useful slices (pure slip)
  SI.Force Fy_alpha[nAlpha];
  SI.Torque Mz_alpha[nAlpha];
  SI.Length t_alpha[nAlpha];
  SI.Length s_alpha[nAlpha];

  SI.Force Fx_kappa[nKappa];
  SI.Length t_kappa[nKappa];

  // Validation
  discrete Real Mz_reconstructed[nAlpha, nKappa](each fixed = true);

protected
  Integer midAlpha(fixed = true);
  Integer midKappa(fixed = true);

equation
  when initial() then
    midAlpha = integer((nAlpha + 1) / 2);
    midKappa = integer((nKappa + 1) / 2);
  end when;
  for i in 1:nAlpha loop

    when initial() then
      alphaGrid[i] = if i == midAlpha then 0 else alphaMin + (alphaMax-alphaMin)*(i-1)/(nAlpha-1);
    end when;
    Fy_alpha[i] = Fy[i, midKappa];
    Mz_alpha[i] = Mz[i, midKappa];
    t_alpha[i] = t[i, midKappa];
    s_alpha[i] = s[i, midKappa];
  end for;
  for j in 1:nKappa loop

    when initial() then
      kappaGrid[j] = if j == midKappa then 0 else kappaMin + (kappaMax-kappaMin)*(j-1)/(nKappa-1);
    end when;
    Fx_kappa[j] = Fx[midAlpha, j];
    t_kappa[j] = t[midAlpha, j];
  end for;

  // Independent point equations avoid one dense dependency graph for the entire grid.
  for i in 1:nAlpha loop
    for j in 1:nKappa loop

      when initial() then
        (Fx[i, j], Fy[i, j], Mx[i, j], My[i, j], Mz[i, j], t[i, j], s[i, j]) =
          MF52.Eval(Fz, alphaGrid[i], kappaGrid[j], gamma, Vx, car.chassis.frontAxle.tire.force);
        Mz_reconstructed[i, j] = -t[i, j]*Fy[i, j] + s[i, j]*Fx[i, j];
      end when;
    end for;
  end for;

annotation(
    experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-06, Interval = 1));
end TestMF52;
