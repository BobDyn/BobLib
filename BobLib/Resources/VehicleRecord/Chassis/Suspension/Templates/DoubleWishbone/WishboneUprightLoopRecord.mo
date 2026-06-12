within BobLib.Resources.VehicleRecord.Chassis.Suspension.Templates.DoubleWishbone;

record WishboneUprightLoopRecord
  import SI = Modelica.Units.SI;

  // Geometry
  parameter SI.Position upperFore_i[3] "Upper control arm fore inboard joint, expressed in chassis frame" annotation(
    Evaluate = false,
    Dialog(group = "Geometry"));
  parameter SI.Position upperAft_i[3] "Upper control arm aft inboard joint, expressed in chassis frame" annotation(
    Evaluate = false,
    Dialog(group = "Geometry"));
  parameter SI.Position lowerFore_i[3] "Lower control arm fore inboard joint, expressed in chassis frame" annotation(
    Evaluate = false,
    Dialog(group = "Geometry"));
  parameter SI.Position lowerAft_i[3] "Lower control arm aft inboard joint, expressed in chassis frame" annotation(
    Evaluate = false,
    Dialog(group = "Geometry"));
  parameter SI.Position upper_o[3] "Upper control arm outboard joint, expressed in chassis frame" annotation(
    Evaluate = false,
    Dialog(group = "Geometry"));
  parameter SI.Position lower_o[3] "Lower control arm outboard joint, expressed in chassis frame" annotation(
    Evaluate = false,
    Dialog(group = "Geometry"));
  parameter SI.Position tie_o[3] "Tie rod outboard joint, expressed in chassis frame" annotation(
    Evaluate = false,
    Dialog(group = "Geometry"));
  parameter SI.Position wheelCenter[3] "Centroid of volume enclosing wheel, expressed in chassis frame" annotation(
    Evaluate = false,
    Dialog(group = "Geometry"));

end WishboneUprightLoopRecord;
