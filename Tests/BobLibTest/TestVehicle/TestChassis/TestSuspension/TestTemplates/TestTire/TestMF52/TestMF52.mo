within BobLibTest.TestVehicle.TestChassis.TestSuspension.TestTemplates.TestTire.TestMF52;

model TestMF52

  import SI = Modelica.Units.SI;

  import Tire = BobLib.Chassis.Suspension.Tires.MF52;
  import Vehicle = BobLib.Records.VehicleDefn;

  // Vehicle Definition
  parameter Vehicle.EVBatInvMotDiff_DWBCStabar_DWBCStabarRecord car;

  // Operating Conditions
  parameter SI.Force Fz = 654;
  parameter SI.Angle gamma = 0;
  parameter SI.Velocity Vx = 10;

  // Sweep Definition
  parameter Integer nAlpha = 41;
  parameter Integer nKappa = 41;

  parameter SI.Angle alphaMin = -0.25;
  parameter SI.Angle alphaMax = 0.25;

  parameter Real kappaMin = -0.25;
  parameter Real kappaMax = 0.25;

  // Grids
  SI.Angle alphaGrid[nAlpha];
  Real kappaGrid[nKappa];

  // Outputs (full grid)
  discrete SI.Force Fx[nAlpha, nKappa];
  discrete SI.Force Fy[nAlpha, nKappa];

  discrete SI.Torque Mx[nAlpha, nKappa];
  discrete SI.Torque My[nAlpha, nKappa];
  discrete SI.Torque Mz[nAlpha, nKappa];

  discrete SI.Length t[nAlpha, nKappa];
  discrete SI.Length s[nAlpha, nKappa];

  // Useful slices (pure slip)
  discrete SI.Force Fy_alpha[nAlpha];
  discrete SI.Torque Mz_alpha[nAlpha];
  discrete SI.Length t_alpha[nAlpha];
  discrete SI.Length s_alpha[nAlpha];

  discrete SI.Force Fx_kappa[nKappa];
  discrete SI.Length t_kappa[nKappa];

  // Validation
  discrete Real Mz_reconstructed[nAlpha, nKappa];

protected
  Integer midAlpha;
  Integer midKappa;

algorithm
  when initial() then

    // Mid indices
    midAlpha := integer((nAlpha + 1) / 2);
    midKappa := integer((nKappa + 1) / 2);

    // Build grids
    for i in 1:nAlpha loop
      alphaGrid[i] :=
        alphaMin + (alphaMax - alphaMin) * (i - 1) / (nAlpha - 1);
    end for;

    for j in 1:nKappa loop
      kappaGrid[j] :=
        kappaMin + (kappaMax - kappaMin) * (j - 1) / (nKappa - 1);
    end for;

    alphaGrid[midAlpha] := 0;
    kappaGrid[midKappa] := 0;

    // Sweep evaluation
    for i in 1:nAlpha loop
      for j in 1:nKappa loop

        (Fx[i, j], Fy[i, j], Mx[i, j], My[i, j], Mz[i, j], t[i, j], s[i, j]) :=
          Tire.Eval(
            Fz,
            alphaGrid[i],
            kappaGrid[j],
            gamma,
            Vx,
            car.pFrTireModel
          );

        // Reconstruction check
        Mz_reconstructed[i, j] := -t[i, j] * Fy[i, j] + s[i, j] * Fx[i, j];

      end for;
    end for;

    // Pure slip slices (kappa = 0)
    for i in 1:nAlpha loop
      Fy_alpha[i] := Fy[i, midKappa];
      Mz_alpha[i] := Mz[i, midKappa];
      t_alpha[i]  := t[i, midKappa];
      s_alpha[i]  := s[i, midKappa];
    end for;

    // Pure longitudinal (alpha = 0)
    for j in 1:nKappa loop
      Fx_kappa[j] := Fx[midAlpha, j];
      t_kappa[j]  := t[midAlpha, j]; // should collapse
    end for;
  end when;

annotation(
    experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-06, Interval = 1));
end TestMF52;
