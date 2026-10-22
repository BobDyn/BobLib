within BobLib.Records.VehicleRecord.Aero;

record CFDAeroMapRecord

  import Modelica.Units.SI;

  parameter SI.Velocity referenceSpeed
    "Reference speed used by the CFD data";

  parameter SI.Density referenceDensity = 1.225
    "Reference air density used by the CFD data";

  parameter SI.Position aeroRef[3]
    "Aero load reference position in vehicle design coordinates";

  parameter SI.Position FL_RideHeightRef[3]
    "World-frame vector from origin to front-left ride-height reference point";

  parameter SI.Position RL_RideHeightRef[3]
    "World-frame vector from origin to rear-left ride-height reference point";

  parameter Integer frontPoints(min = 2, max = size(frontRideHeightGrid, 1)) = size(frontRideHeightGrid, 1)
    "Number of active ride-height breakpoints" annotation(Evaluate = false);

  parameter SI.Length frontRideHeightGrid[:]
    "Front ride-height breakpoints for the CFD sweep";

  parameter Integer rearPoints(min = 2, max = size(rearRideHeightGrid, 1)) = size(rearRideHeightGrid, 1)
    "Number of active ride-height breakpoints" annotation(Evaluate = false);

  parameter SI.Length rearRideHeightGrid[:]
    "Rear ride-height breakpoints for the CFD sweep";

  parameter SI.Force dragTable[size(frontRideHeightGrid, 1), size(rearRideHeightGrid, 1)]
    "Positive drag magnitude from CFD";

  parameter SI.Force downforceTable[size(frontRideHeightGrid, 1), size(rearRideHeightGrid, 1)]
    "Positive downforce magnitude from CFD";

  parameter SI.Torque mxTable[size(frontRideHeightGrid, 1), size(rearRideHeightGrid, 1)]
    "Roll moment about aeroRef at reference conditions; rows are front height, columns rear height";

  parameter SI.Torque myTable[size(frontRideHeightGrid, 1), size(rearRideHeightGrid, 1)]
    "Pitch moment about aeroRef at reference conditions; rows are front height, columns rear height";

  parameter SI.Torque mzTable[size(frontRideHeightGrid, 1), size(rearRideHeightGrid, 1)]
    "Yaw moment about aeroRef at reference conditions; rows are front height, columns rear height";

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
