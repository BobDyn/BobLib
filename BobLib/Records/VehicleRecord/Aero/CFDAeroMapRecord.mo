within BobLib.Records.VehicleRecord.Aero;

record CFDAeroMapRecord

  import SI = Modelica.Units.SI;

  parameter SI.Velocity referenceSpeed
    "Reference speed used by the CFD data";

  parameter SI.Density referenceDensity = 1.225
    "Reference air density used by the CFD data";

  parameter SI.Position aeroRef[3]
    "World-frame vector from origin to the aero force application point";

  parameter SI.Position FL_RideHeightRef[3]
    "World-frame vector from origin to front-left ride-height reference point";

  parameter SI.Position RL_RideHeightRef[3]
    "World-frame vector from origin to rear-left ride-height reference point";

  parameter SI.Length frontRideHeightGrid[:]
    "Front ride-height breakpoints for the CFD sweep";

  parameter SI.Length rearRideHeightGrid[:]
    "Rear ride-height breakpoints for the CFD sweep";

  parameter SI.Force dragTable[size(frontRideHeightGrid, 1), size(rearRideHeightGrid, 1)]
    "Positive drag magnitude from CFD";

  parameter SI.Force downforceTable[size(frontRideHeightGrid, 1), size(rearRideHeightGrid, 1)]
    "Positive downforce magnitude from CFD";

  parameter SI.Torque mxTable[size(frontRideHeightGrid, 1), size(rearRideHeightGrid, 1)]
    "Roll moment from CFD";

  parameter SI.Torque myTable[size(frontRideHeightGrid, 1), size(rearRideHeightGrid, 1)]
    "Pitch moment from CFD";

  parameter SI.Torque mzTable[size(frontRideHeightGrid, 1), size(rearRideHeightGrid, 1)]
    "Yaw moment from CFD";

  annotation(
    Documentation(info = "<html>
<p>
Record <code>CFDAeroMapRecord</code> stores the aerodynamic map data for the CFD aero model.
</p>
<p>
It contains front/rear ride-height grids, force and moment tables, reference speed, and reference density used by <code>Aero.CFDAeroMap</code>.
</p>
</html>"));
end CFDAeroMapRecord;
