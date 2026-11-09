within BobLib.Experiments.Standards;

model VehicleSIL "Car-only FMI plant for an external VCU with signed torque and independent brakes"

  extends BaseClasses.PartialVehicleSIL;
  extends BobLib.Icons.SimulationIcon;

  annotation(experiment(StartTime = 0, StopTime = 10, Tolerance = 1e-6, Interval = 0.001),
    Documentation(info = "<html><p>Export with <code>python Tools/export_fmu.py --sil --output build/sil</code> on Linux. Use FMI 2 Co-Simulation; the external master supplies actuator commands and owns firmware scheduling. See <code>Tools/sil-interface.json</code> for the versioned signal boundary and <code>Tools/SIL.md</code> for validation and limitations.</p></html>"));
end VehicleSIL;
