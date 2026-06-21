within BobLibVehicleInterfaces.Chassis.Internal;

partial model DetailedChassisBase

  extends BobLibVehicleInterfaces.Icons.TwoAxleVehicleCoreIcon;

  import SI = Modelica.Units.SI;

  inner parameter SI.Length linkDiameter = 0.020;
  inner parameter SI.Length jointDiameter = 0.030;

  // Front axle
  replaceable BobLibVehicleInterfaces.Chassis.Suspension.FrAxleDW_BC_Stabar frAxleDW annotation(
    Placement(transformation(origin = {0.464283, 63}, extent = {{-64.25, -28.5556}, {64.25, 28.5556}})));

  // Rear axle
  replaceable BobLibVehicleInterfaces.Chassis.Suspension.RrAxleDW_BC_Stabar rrAxleDW annotation(
    Placement(transformation(origin = {0.285708, -60.2776}, extent = {{-60.1429, -23.3889}, {60.1429, 23.3889}})));

  // Frame
  replaceable BobLibVehicleInterfaces.Chassis.Body.FrameBase spaceFrame annotation(
    Placement(transformation( extent = {{30, -30}, {-30, 30}}, rotation = 90)));

  // Interfaces
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frameFL annotation(
    Placement(
      transformation(origin = {-100, 42}, extent = {{-16, -16}, {16, 16}}),
      iconTransformation(origin = {-180, 70}, extent = {{-16, -16}, {16, 16}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frameFR annotation(
    Placement(
      transformation(origin = {100, 42}, extent = {{-16, -16}, {16, 16}}),
      iconTransformation(origin = {180, 70}, extent = {{-16, -16}, {16, 16}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frameRL annotation(
    Placement(
      transformation(origin = {-100, -90}, extent = {{-16, -16}, {16, 16}}),
      iconTransformation(origin = {-180, -170}, extent = {{-16, -16}, {16, 16}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frameRR annotation(
    Placement(
      transformation(origin = {100, -90}, extent = {{-16, -16}, {16, 16}}),
      iconTransformation(origin = {180, -170}, extent = {{-16, -16}, {16, 16}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b cgFrame annotation(
    Placement(
      transformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}}),
      iconTransformation(origin = {180, 0}, extent = {{-16, -16}, {16, 16}})));

  Modelica.Mechanics.Rotational.Interfaces.Flange_b flangeFL annotation(
    Placement(
      transformation(origin = {-100, 70}, extent = {{-10, -10}, {10, 10}}),
      iconTransformation(origin = {-180, 120}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.Rotational.Interfaces.Flange_b flangeFR annotation(
    Placement(
      transformation(origin = {100, 70}, extent = {{-10, -10}, {10, 10}}),
      iconTransformation(origin = {180, 120}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.Rotational.Interfaces.Flange_b flangeRL annotation(
    Placement(
      transformation(origin = {-100, -60}, extent = {{-10, -10}, {10, 10}}),
      iconTransformation(origin = {-180, -120}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.Rotational.Interfaces.Flange_b flangeRR annotation(
    Placement(
      transformation(origin = {100, -60}, extent = {{-10, -10}, {10, 10}}),
      iconTransformation(origin = {180, -120}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.Rotational.Interfaces.Flange_a frSteerFlange annotation(
    Placement(
      transformation(origin = {0, 100}, extent = {{-10, -10}, {10, 10}}),
      iconTransformation(origin = {0, 166}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frAxleFrame annotation(
    Placement(
      transformation(origin = {-40, 100}, extent = {{-16, -16}, {16, 16}}, rotation = -90),
      iconTransformation(origin = {0, 118}, extent = {{-16, -16}, {16, 16}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b rrAxleFrame annotation(
    Placement(
      transformation(origin = {-40, -100}, extent = {{-16, -16}, {16, 16}}, rotation = -90),
      iconTransformation(origin = {0, -120}, extent = {{-16, -16}, {16, 16}}, rotation = -90)));

equation
  connect(spaceFrame.rearFrame, rrAxleDW.axleFrame) annotation(
    Line(points = {{0, -30}, {0, -60}}, color = {95, 95, 95}));
  connect(flangeFL, frAxleDW.leftTorque) annotation(
    Line(points = {{-100, 70}, {-90, 70}, {-90, 59}, {-64, 59}}));
  connect(frameFL, frAxleDW.leftCP) annotation(
    Line(points = {{-100, 42}, {-64, 42}}));
  connect(flangeFR, frAxleDW.rightTorque) annotation(
    Line(points = {{100, 70}, {90, 70}, {90, 59}, {65, 59}}));
  connect(frameFR, frAxleDW.rightCP) annotation(
    Line(points = {{100, 42}, {65, 42}}));
  connect(flangeRL, rrAxleDW.leftTorque) annotation(
    Line(points = {{-100, -60}, {-60, -60}}));
  connect(frameRL, rrAxleDW.leftCP) annotation(
    Line(points = {{-100, -90}, {-90, -90}, {-90, -77}, {-60, -77}}));
  connect(flangeRR, rrAxleDW.rightTorque) annotation(
    Line(points = {{100, -60}, {100, -60.5}, {60, -60.5}, {60, -60}}));
  connect(spaceFrame.frontFrame, frAxleDW.axleFrame) annotation(
    Line(points = {{0, 30}, {0, 59}}, color = {95, 95, 95}));
  connect(frAxleDW.steerFlange, frSteerFlange) annotation(
    Line(points = {{0, 76}, {0, 100}}));
  connect(frAxleDW.axleFrame, frAxleFrame) annotation(
    Line(points = {{0, 59}, {0, 60}, {-40, 60}, {-40, 100}}, color = {95, 95, 95}));
  connect(rrAxleFrame, rrAxleDW.axleFrame) annotation(
    Line(points = {{-40, -100}, {-40, -60}, {0, -60}}));
  connect(rrAxleDW.rightCP, frameRR) annotation(
    Line(points = {{60, -76}, {90, -76}, {90, -90}, {100, -90}}, color = {95, 95, 95}));
  connect(spaceFrame.cgFrame, cgFrame) annotation(
    Line(points = {{18, 0}, {100, 0}}, color = {95, 95, 95}));
  annotation(Diagram(coordinateSystem(extent = {{-120, -120}, {120, 120}})),
    experiment(StartTime = 0, StopTime = 5, Tolerance = 1e-06, Interval = 0.002),
    __OpenModelica_commandLineOptions = "--matchingAlgorithm=PFPlusExt --indexReductionMethod=dynamicStateSelection -d=initialization,NLSanalyticJacobian --maxSizeLinearTearing=5000",
    Icon(graphics = {
      Line(origin = {-160, 74}, points = {{-20, -4}, {40, -4}, {40, 6}}),
      Line(origin = {160, 74}, points = {{20, -4}, {-40, -4}, {-40, 6}}),
      Line(origin = {0, 116}, points = {{0, 4}, {0, -12}}),
      Line(origin = {-160, -166}, points = {{-20, -4}, {40, -4}, {40, 6}}),
      Line(origin = {160, -166}, points = {{20, -4}, {-40, -4}, {-40, 6}}),
      Line(origin = {0, -124}, points = {{0, 4}, {0, -12}}),
      Line(origin = {0, 133}, points = {{0, -33}, {0, 33}}, thickness = 5)
    }),
    Icon(graphics = {
      Line(origin = {-130, 120}, points = {{-10, 0}, {-50, 0}}, pattern = LinePattern.Dash, thickness = 1),
      Line(origin = {190, 120}, points = {{-10, 0}, {-50, 0}}, pattern = LinePattern.Dash, thickness = 1),
      Line(origin = {-130, -120}, points = {{-10, 0}, {-50, 0}}, pattern = LinePattern.Dash, thickness = 1),
      Line(origin = {190, -120}, points = {{-10, 0}, {-50, 0}}, pattern = LinePattern.Dash, thickness = 1)
    }),
    Documentation(info = "<html>
<p>
Partial model <code>DetailedChassisBase</code> defines the detailed BobLib chassis implementation boundary.
</p>
<p>
It gathers sprung chassis frame, front and rear axle assemblies, aero-load mounting, ride-height outputs, wheel hubs, and control bus wiring below the public chassis adapter.
</p>
</html>"));
end DetailedChassisBase;
