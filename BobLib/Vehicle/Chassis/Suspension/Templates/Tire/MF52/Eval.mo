within BobLib.Vehicle.Chassis.Suspension.Templates.Tire.MF52;

function Eval
  import Modelica.SIunits;

  import BobLib.Resources.VehicleRecord.Chassis.Suspension.Templates.Tire.MF52.*;
  import BobLib.Resources.VehicleRecord.Chassis.Suspension.Templates.Tire.MF52.PureSlip.*;
  import BobLib.Resources.VehicleRecord.Chassis.Suspension.Templates.Tire.MF52.CombinedSlip.*;

  import BobLib.Vehicle.Chassis.Suspension.Templates.Tire.MF52.PureSlip.*;
  import BobLib.Vehicle.Chassis.Suspension.Templates.Tire.MF52.CombinedSlip.*;

  // Inputs
  input SIunits.Force Fz;
  input SIunits.Angle alpha;
  input SIunits.DimensionlessRatio kappa;
  input SIunits.Angle gamma;
  input SIunits.Velocity Vx;

  input MF52Record tire;

  // Outputs
  output SIunits.Force Fx;
  output SIunits.Force Fy;
  output SIunits.Torque Mx;
  output SIunits.Torque My;
  output SIunits.Torque Mz;
  
  output SIunits.Length t;
  output SIunits.Length s;

protected
  // Unpacked records
  FxPureRecord pFxPure;
  FxCombinedRecord pFxComb;

  FyPureRecord pFyPure;
  FyCombinedRecord pFyComb;

  MxPureRecord pMxPure;
  MxCombinedRecord pMxComb;

  MyPureRecord pMyPure;
  MyCombinedRecord pMyComb;

  MzPureRecord pMzPure;
  MzCombinedRecord pMzComb;

  SetupRecord setup;

  SIunits.Force FzEval;
  Real loadScale;

algorithm
  // Unpack once (clean + fast)
  pFxPure := tire.fxPure;
  pFxComb := tire.fxCombined;

  pFyPure := tire.fyPure;
  pFyComb := tire.fyCombined;

  pMxPure := tire.mxPure;
  pMxComb := tire.mxCombined;

  pMyPure := tire.myPure;
  pMyComb := tire.myCombined;

  pMzPure := tire.mzPure;
  pMzComb := tire.mzCombined;

  setup := tire.setup;

  if Fz > 1e-3 then
    if Fz > setup.FZMIN then
      FzEval := Fz;
    else
      FzEval := setup.FZMIN;
    end if;
    loadScale := Fz / FzEval;
  else
    FzEval := setup.FZMIN;
    loadScale := 0;
  end if;

  // Forces
  Fx := FxCombinedEval(
    FzEval, kappa, alpha, gamma,
    pFxPure, pFxComb, setup
  );

  Fy := FyCombinedEval(
    FzEval, alpha, kappa, gamma,
    pFyPure, pFyComb, setup
  );

  // Moments
  Mx := MxCombinedEval(
    FzEval, Fy, gamma,
    pMxPure, pMxComb, setup
  );

  My := MyCombinedEval(
    FzEval, Fx, Vx,
    pMyPure, pMyComb, setup
  );

  (Mz, t, s) := MzCombinedEval(
    FzEval,
    Fx,
    Fy,
    alpha,
    kappa,
    gamma,
  
    pFyPure,
    pFxPure,
  
    pMzPure,
    pMzComb,
    setup
  );

  Fx := loadScale * Fx;
  Fy := loadScale * Fy;
  Mx := loadScale * Mx;
  My := loadScale * My;
  Mz := loadScale * Mz;
  
  // Z-up transform (apply at boundary)
  Fy := -Fy;
  My := -My;
  Mz := -Mz;

end Eval;
