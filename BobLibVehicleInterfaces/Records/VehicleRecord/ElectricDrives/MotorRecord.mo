within BobLibVehicleInterfaces.Records.VehicleRecord.ElectricDrives;

record MotorRecord

  import SI = Modelica.Units.SI;

  parameter SI.Voltage Vdc_max = 630
    "Motor maximum DC voltage reference";
  parameter Real rpm_max_peak = 6500
    "Motor peak speed reference";
  parameter SI.Torque T_peak = 220
    "Motor peak torque";
  parameter SI.Torque T_cont = 130
    "Motor continuous torque";
  parameter SI.Current I_peak_2min = 360
    "Motor peak current";
  parameter SI.Current I_cont = 180
    "Motor continuous current";
  parameter Real Kt_Nm_per_A = 0.61
    "Motor torque constant";
  parameter SI.Time peakTime = 120
    "Peak torque/current allowance duration";
  parameter SI.Power P_mech_peak = 124e3
    "Motor peak mechanical power";
  parameter SI.Power P_cont_low = 75e3
    "Motor continuous power envelope low-speed anchor";
  parameter SI.Power P_cont_high = 75e3
    "Motor continuous power envelope high-speed anchor";
  parameter Real eta_mot(unit = "1") = 0.96
    "Motor motoring efficiency reference";
  parameter Real eta_reg(unit = "1") = 0.95
    "Motor regen efficiency reference";
  parameter SI.AngularVelocity w_eps = 1.0
    "Low-speed regularization for power-to-torque conversion";
  parameter SI.Inertia rotorJ = 0.02521
    "Motor rotor inertia";

  annotation(
    Documentation(info = "<html>
<p>
Record <code>MotorRecord</code> contains the vehicle-level parameters passed to
<code>ElectricDrives.Motor</code>.
</p>
</html>"));
end MotorRecord;
