within BobLib.Resources.VehicleRecord.Aero;

record CFDAeroMapRecord
  import Modelica.SIunits;

  parameter SIunits.Velocity referenceSpeed
    "Reference speed used by the CFD data";

  parameter SIunits.Length frontRideHeightGrid[:]
    "Front ride-height breakpoints for the CFD sweep";

  parameter SIunits.Length rearRideHeightGrid[:]
    "Rear ride-height breakpoints for the CFD sweep";

  parameter SIunits.Force dragTable[size(frontRideHeightGrid, 1), size(rearRideHeightGrid, 1)]
    "Positive drag magnitude from CFD";

  parameter SIunits.Force downforceTable[size(frontRideHeightGrid, 1), size(rearRideHeightGrid, 1)]
    "Positive downforce magnitude from CFD";

  parameter SIunits.Torque mxTable[size(frontRideHeightGrid, 1), size(rearRideHeightGrid, 1)]
    "Roll moment from CFD";

  parameter SIunits.Torque myTable[size(frontRideHeightGrid, 1), size(rearRideHeightGrid, 1)]
    "Pitch moment from CFD";

  parameter SIunits.Torque mzTable[size(frontRideHeightGrid, 1), size(rearRideHeightGrid, 1)]
    "Yaw moment from CFD";

end CFDAeroMapRecord;
