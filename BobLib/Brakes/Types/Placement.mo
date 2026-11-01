within BobLib.Brakes.Types;
type Placement = enumeration(

  Outboard "One brake at each upright",
  Inboard "One brake on each inboard halfshaft",
  DifferentialInput "Single brake upstream of the differential")
  "Location of the rotating brake and its reaction path";
