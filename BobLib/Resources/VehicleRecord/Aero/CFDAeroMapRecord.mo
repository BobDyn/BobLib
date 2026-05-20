within BobLib.Resources.VehicleRecord.Aero;

record CFDAeroMapRecord
  import Modelica.SIunits;

  parameter SIunits.Velocity referenceSpeed
    "Reference speed used by the CFD data";

  parameter SIunits.Position aeroRef[3]
    "World-frame vector from origin to the aero force application point";

  parameter SIunits.Position FL_RideHeightRef[3]
    "World-frame vector from origin to front-left ride-height reference point";

  parameter SIunits.Position RL_RideHeightRef[3]
    "World-frame vector from origin to rear-left ride-height reference point";

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
