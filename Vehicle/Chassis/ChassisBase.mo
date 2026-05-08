within BobLib.Vehicle.Chassis;

partial model ChassisBase
  import Modelica.SIunits;
  
  inner parameter SIunits.Length linkDiameter = 0.020;
  inner parameter SIunits.Length jointDiameter = 0.030;
  
  // Front axle
  replaceable BobLib.Vehicle.Chassis.Suspension.FrAxleDW_BC_Stabar frAxleDW annotation(
    Placement(transformation(origin = {0.464283, 63}, extent = {{-64.25, -28.5556}, {64.25, 28.5556}})));
  
  // Rear axle
  replaceable BobLib.Vehicle.Chassis.Suspension.RrAxleDW_BC_ARB rrAxleDW annotation(
    Placement(transformation(origin = {0.285708, -60.2776}, extent = {{-60.1429, -23.3889}, {60.1429, 23.3889}})));
  
  // Frame
  replaceable BobLib.Vehicle.Chassis.Body.FrameBase spaceFrame annotation(
    Placement(transformation( extent = {{30, -30}, {-30, 30}}, rotation = 90)));
  
  // Interfaces
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frameFL annotation(
    Placement(transformation(origin = {-100, 42}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {-180, 70}, extent = {{-16, -16}, {16, 16}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frameFR annotation(
    Placement(transformation(origin = {100, 42}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {180, 70}, extent = {{-16, -16}, {16, 16}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frameRL annotation(
    Placement(transformation(origin = {-100, -90}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {-180, -170}, extent = {{-16, -16}, {16, 16}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frameRR annotation(
    Placement(transformation(origin = {100, -90}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {180, -170}, extent = {{-16, -16}, {16, 16}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b cgFrame annotation(
    Placement(transformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {180, 0}, extent = {{-16, -16}, {16, 16}})));

  Modelica.Mechanics.Rotational.Interfaces.Flange_b flangeFL annotation(
    Placement(transformation(origin = {-100, 70}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {-180, 120}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.Rotational.Interfaces.Flange_b flangeFR annotation(
    Placement(transformation(origin = {100, 70}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {180, 120}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.Rotational.Interfaces.Flange_b flangeRL annotation(
    Placement(transformation(origin = {-100, -60}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {-180, -120}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Mechanics.Rotational.Interfaces.Flange_b flangeRR annotation(
    Placement(transformation(origin = {100, -60}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {180, -120}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.Rotational.Interfaces.Flange_a frSteerFlange annotation(
    Placement(transformation(origin = {0, 100}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {0, 166}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frAxleFrame annotation(
    Placement(transformation(origin = {-40, 100}, extent = {{-16, -16}, {16, 16}}, rotation = -90), iconTransformation(origin = {0, 118}, extent = {{-16, -16}, {16, 16}}, rotation = -90)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b rrAxleFrame annotation(
    Placement(transformation(origin = {-40, -100}, extent = {{-16, -16}, {16, 16}}, rotation = -90), iconTransformation(origin = {0, -120}, extent = {{-16, -16}, {16, 16}}, rotation = -90)));

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
  connect(spaceFrame.cgFrame, cgFrame) annotation(
    Line(points = {{18, 0}, {100, 0}}, color = {95, 95, 95}));
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
  annotation(
    Diagram,
    Icon(coordinateSystem(extent = {{-180, -200}, {180, 200}}), graphics = {Line(origin = {-25, 105}, points = {{-35, -15}, {25, 15}}), Line(origin = {-30, 135}, points = {{-30, -9}, {30, -15}}), Line(origin = {-82, 94}, points = {{22, -4}, {-24, 4}}, thickness = 5), Line(origin = {-82, 130}, points = {{22, -4}, {-22, 6}}, thickness = 5), Ellipse(origin = {-60, 90}, lineColor = {255, 0, 0}, fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid, extent = {{-4, 4}, {4, -4}}), Ellipse(origin = {-60, 126}, lineColor = {255, 0, 0}, fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid, extent = {{-4, 4}, {4, -4}}), Line(origin = {-30, 135}, points = {{90, -9}, {30, -15}}), Line(origin = {35, 65}, points = {{-35, 55}, {25, 25}}), Line(origin = {84, 86}, points = {{22, 12}, {-24, 4}}, thickness = 5, arrowSize = 2), Line(origin = {81, 131}, points = {{-21, -5}, {23, 5}}, thickness = 5), Ellipse(origin = {60, 126}, lineColor = {255, 0, 0}, fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid, extent = {{-4, 4}, {4, -4}}), Ellipse(origin = {60, 90}, lineColor = {255, 0, 0}, fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid, extent = {{-4, 4}, {4, -4}}), Line(origin = {-160, 74}, points = {{-20, -4}, {40, -4}, {40, 6}}), Line(origin = {160, 74}, points = {{20, -4}, {-40, -4}, {-40, 6}}), Line(origin = {-130, 120}, points = {{-10, 0}, {-50, 0}}, pattern = LinePattern.Dash, thickness = 1), Line(origin = {190, 120}, points = {{-10, 0}, {-50, 0}}, pattern = LinePattern.Dash, thickness = 1), Line(origin = {-80, 106}, points = {{20, -6}, {-22, 6}}, thickness = 5), Ellipse(origin = {-60, 100}, lineColor = {255, 0, 0}, fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid, extent = {{-4, 4}, {4, -4}}), Rectangle(origin = {-120, 120}, fillColor = {71, 71, 71}, fillPattern = FillPattern.Solid, extent = {{-20, 40}, {20, -40}}, radius = 5), Line(origin = {40, 106}, points = {{20, -6}, {64, 6}}, thickness = 5), Ellipse(origin = {60, 100}, lineColor = {255, 0, 0}, fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid, extent = {{-4, 4}, {4, -4}}), Rectangle(origin = {120, 120}, fillColor = {71, 71, 71}, fillPattern = FillPattern.Solid, extent = {{-20, 40}, {20, -40}}, radius = 5), Line(origin = {-25, 105}, points = {{-35, -15}, {25, 15}}), Line(origin = {35, 65}, points = {{-35, 55}, {25, 25}}), Line(origin = {-10, 100}, points = {{-36, 0}, {56, 0}}, thickness = 8), Line(origin = {-50, 100}, points = {{4, 0}, {-4, 0}}, color = {255, 0, 0}, thickness = 5), Line(origin = {50, 100}, points = {{4, 0}, {-4, 0}}, color = {255, 0, 0}, thickness = 5), Line(origin = {0, 116}, points = {{0, 4}, {0, -12}}), Line(origin = {-25, -135}, points = {{-35, -15}, {25, 15}}), Line(origin = {-30, -105}, points = {{-30, -9}, {30, -15}}), Line(origin = {-82, -146}, points = {{22, -4}, {-24, 4}}, thickness = 5), Line(origin = {-82, -110}, points = {{22, -4}, {-22, 6}}, thickness = 5), Ellipse(origin = {-60, -150}, lineColor = {255, 0, 0}, fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid, extent = {{-4, 4}, {4, -4}}), Ellipse(origin = {-60, -114}, lineColor = {255, 0, 0}, fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid, extent = {{-4, 4}, {4, -4}}), Line(origin = {-30, -105}, points = {{90, -9}, {30, -15}}), Line(origin = {35, -175}, points = {{-35, 55}, {25, 25}}), Line(origin = {84, -154}, points = {{22, 12}, {-24, 4}}, thickness = 5, arrowSize = 2), Line(origin = {81, -109}, points = {{-21, -5}, {23, 5}}, thickness = 5), Ellipse(origin = {60, -114}, lineColor = {255, 0, 0}, fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid, extent = {{-4, 4}, {4, -4}}), Ellipse(origin = {60, -150}, lineColor = {255, 0, 0}, fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid, extent = {{-4, 4}, {4, -4}}), Line(origin = {-160, -166}, points = {{-20, -4}, {40, -4}, {40, 6}}), Line(origin = {160, -166}, points = {{20, -4}, {-40, -4}, {-40, 6}}), Line(origin = {-130, -120}, points = {{-10, 0}, {-50, 0}}, pattern = LinePattern.Dash, thickness = 1), Line(origin = {190, -120}, points = {{-10, 0}, {-50, 0}}, pattern = LinePattern.Dash, thickness = 1), Line(origin = {-80, -134}, points = {{20, -6}, {-22, 6}}, thickness = 5), Ellipse(origin = {-60, -140}, lineColor = {255, 0, 0}, fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid, extent = {{-4, 4}, {4, -4}}), Rectangle(origin = {-120, -120}, fillColor = {71, 71, 71}, fillPattern = FillPattern.Solid, extent = {{-20, 40}, {20, -40}}, radius = 5), Line(origin = {40, -134}, points = {{20, -6}, {64, 6}}, thickness = 5), Ellipse(origin = {60, -140}, lineColor = {255, 0, 0}, fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid, extent = {{-4, 4}, {4, -4}}), Rectangle(origin = {120, -120}, fillColor = {71, 71, 71}, fillPattern = FillPattern.Solid, extent = {{-20, 40}, {20, -40}}, radius = 5), Line(origin = {-25, -135}, points = {{-35, -15}, {25, 15}}), Line(origin = {35, -175}, points = {{-35, 55}, {25, 25}}), Line(origin = {-10, -140}, points = {{-44, 0}, {64, 0}}, thickness = 8), Line(origin = {0, -124}, points = {{0, 4}, {0, -12}}), Line(origin = {0, 133}, points = {{0, -33}, {0, 33}}, thickness = 5), Ellipse(origin = {0, 166}, lineThickness = 5, extent = {{-26, 26}, {26, -26}}), Line(origin = {-10, 176}, points = {{10, -10}, {-14, -2}}, thickness = 5), Line(origin = {10, 176}, points = {{-10, -10}, {14, -2}}, thickness = 5), Ellipse(origin = {0, 166}, lineColor = {255, 255, 255}, lineThickness = 1, extent = {{-28, 28}, {28, -28}}), Line(origin = {0, -80}, rotation = 90, points = {{-40, 0}, {60, 0}}, thickness = 5), Line(origin = {0, 80}, rotation = -90, points = {{60, 0}, {-40, 0}}, thickness = 5), Line(origin = {28, -24}, rotation = -90, points = {{-3, -28}, {-15, 36}}, thickness = 1), Line(origin = {75, 36}, points = {{-15, -26}, {-75, -16}}, thickness = 1), Ellipse(origin = {60, 0}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, lineThickness = 1, extent = {{-10, 10}, {10, -10}}), Line(origin = {60, 0}, points = {{0, -10}, {0, 10}}), Line(origin = {60, 0}, points = {{-10, 0}, {10, 0}}), Polygon(origin = {65, 5}, fillPattern = FillPattern.Solid, points = {{-5, -5}, {-5, 5}, {-3, 5}, {1, 3}, {3, 1}, {5, -3}, {5, -5}, {-5, -5}}), Polygon(origin = {55, -5}, rotation = 180, fillPattern = FillPattern.Solid, points = {{-5, -5}, {-5, 5}, {-3, 5}, {1, 3}, {3, 1}, {5, -3}, {5, -5}, {-5, -5}}), Line(origin = {1.03, 0}, points = {{-1.02758, -20}, {-1.02758, -6}, {-5.02758, -4}, {2.97242, 0}, {-5.02758, 4}, {2.97242, 8}, {-1.02758, 10}, {-1.02758, 20}}, color = {0, 170, 0}, thickness = 2), Text( rotation = 90,textColor = {255, 0, 0}, extent = {{-14, 14}, {14, -14}}, textString = "rxDOF"), Line(origin = {120, 0}, points = {{-60, 0}, {60, 0}})}),
    experiment(StartTime = 0, StopTime = 5, Tolerance = 1e-06, Interval = 0.002),
    __OpenModelica_commandLineOptions = "--matchingAlgorithm=PFPlusExt --indexReductionMethod=dynamicStateSelection -d=initialization,NLSanalyticJacobian --maxSizeLinearTearing=5000");
end ChassisBase;
