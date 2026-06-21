within BobLib.Records.VehicleRecord.Chassis.Suspension.Templates.Tire.MF52;

record RelaxationRecord

  import SI = Modelica.Units.SI;

  SI.Force FNOMIN = 650
    "Nominal vertical load Fz0 for PAC2002 relaxation";

  SI.Length UNLOADED_RADIUS = 0.2
    "Unloaded tire radius R0 for PAC2002 relaxation";

  Real LFZO = 1
    "Scale factor of nominal load lambda_Fz0 [-]";

  Real PTX1 = 0
    "Longitudinal relaxation coefficient p_Tx1 [-]";

  Real PTX2 = 0
    "Longitudinal relaxation load dependency p_Tx2 [-]";

  Real PTX3 = 0
    "Longitudinal relaxation exponential load dependency p_Tx3 [-]";

  Real PTY1 = 0
    "Peak value of lateral relaxation length p_Ty1 [-]";

  Real PTY2 = 0
    "Load at which lateral relaxation is extreme p_Ty2 [-]";

  Real PKY3 = 0
    "Lateral relaxation camber sensitivity p_Ky3 [-]";

  Real LSGKP = 1
    "Scale factor of longitudinal relaxation length [-]";

  Real LSGAL = 1
    "Scale factor of lateral relaxation length [-]";

  annotation(
    Documentation(info = "<html>
<p>
Record <code>RelaxationRecord</code> stores MF5.2 tire relaxation-length coefficients.
</p>
<p>
The tire slip and force models use this data to clamp load ranges and configure transient slip behavior.
</p>
</html>"));
end RelaxationRecord;
