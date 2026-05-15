within BobLib.Vehicle;

partial model VehicleBase
  import Modelica.SIunits;
  
  import BobLib.Utilities.Math.Vector.mirrorXZ;  
  import Modelica.Math.Vectors.norm;
    
  import BobLib.Resources.VehicleRecord.Chassis.Suspension.Templates.AxleMassRecord;
  import BobLib.Resources.VehicleRecord.Chassis.Suspension.Templates.MassRecord;
  import BobLib.Resources.VehicleRecord.Aero.CFDAeroMapRecord;
  
  inner parameter SIunits.Length linkDiameter = 0.020;
  inner parameter SIunits.Length jointDiameter = 0.030;
  
  parameter CFDAeroMapRecord pAero;
  parameter MassRecord pSprungMass;
  parameter AxleMassRecord pFrAxleMass;
  parameter AxleMassRecord pRrAxleMass;
  final parameter SIunits.Mass pTotalMass =
    pSprungMass.m +
    pFrAxleMass.unsprungMass.m + pFrAxleMass.ucaMass.m + pFrAxleMass.lcaMass.m + pFrAxleMass.tieMass.m +
    pRrAxleMass.unsprungMass.m + pRrAxleMass.ucaMass.m + pRrAxleMass.lcaMass.m + pRrAxleMass.tieMass.m;
  final parameter SIunits.Position pVehicleCG[3] = {
    (
      pSprungMass.m * pSprungMass.rCM[1] +
      pFrAxleMass.unsprungMass.m * pFrAxleMass.unsprungMass.rCM[1] +
      pFrAxleMass.ucaMass.m * pFrAxleMass.ucaMass.rCM[1] +
      pFrAxleMass.lcaMass.m * pFrAxleMass.lcaMass.rCM[1] +
      pFrAxleMass.tieMass.m * pFrAxleMass.tieMass.rCM[1] +
      pRrAxleMass.unsprungMass.m * pRrAxleMass.unsprungMass.rCM[1] +
      pRrAxleMass.ucaMass.m * pRrAxleMass.ucaMass.rCM[1] +
      pRrAxleMass.lcaMass.m * pRrAxleMass.lcaMass.rCM[1] +
      pRrAxleMass.tieMass.m * pRrAxleMass.tieMass.rCM[1]
    ) / pTotalMass,
    (
      pSprungMass.m * pSprungMass.rCM[2] +
      pFrAxleMass.unsprungMass.m * pFrAxleMass.unsprungMass.rCM[2] +
      pFrAxleMass.ucaMass.m * pFrAxleMass.ucaMass.rCM[2] +
      pFrAxleMass.lcaMass.m * pFrAxleMass.lcaMass.rCM[2] +
      pFrAxleMass.tieMass.m * pFrAxleMass.tieMass.rCM[2] +
      pRrAxleMass.unsprungMass.m * pRrAxleMass.unsprungMass.rCM[2] +
      pRrAxleMass.ucaMass.m * pRrAxleMass.ucaMass.rCM[2] +
      pRrAxleMass.lcaMass.m * pRrAxleMass.lcaMass.rCM[2] +
      pRrAxleMass.tieMass.m * pRrAxleMass.tieMass.rCM[2]
    ) / pTotalMass,
    (
      pSprungMass.m * pSprungMass.rCM[3] +
      pFrAxleMass.unsprungMass.m * pFrAxleMass.unsprungMass.rCM[3] +
      pFrAxleMass.ucaMass.m * pFrAxleMass.ucaMass.rCM[3] +
      pFrAxleMass.lcaMass.m * pFrAxleMass.lcaMass.rCM[3] +
      pFrAxleMass.tieMass.m * pFrAxleMass.tieMass.rCM[3] +
      pRrAxleMass.unsprungMass.m * pRrAxleMass.unsprungMass.rCM[3] +
      pRrAxleMass.ucaMass.m * pRrAxleMass.ucaMass.rCM[3] +
      pRrAxleMass.lcaMass.m * pRrAxleMass.lcaMass.rCM[3] +
      pRrAxleMass.tieMass.m * pRrAxleMass.tieMass.rCM[3]
    ) / pTotalMass
  };
  final parameter SIunits.Inertia pVehicleInertia[3, 3] =
    BobLib.Utilities.Mechanics.translateInertia(
      pSprungMass.inertia,
      pSprungMass.m,
      pSprungMass.rCM - pVehicleCG)
    + BobLib.Utilities.Mechanics.translateInertia(
      pFrAxleMass.unsprungMass.inertia,
      pFrAxleMass.unsprungMass.m,
      pFrAxleMass.unsprungMass.rCM - pVehicleCG)
    + BobLib.Utilities.Mechanics.translateInertia(
      pFrAxleMass.ucaMass.inertia,
      pFrAxleMass.ucaMass.m,
      pFrAxleMass.ucaMass.rCM - pVehicleCG)
    + BobLib.Utilities.Mechanics.translateInertia(
      pFrAxleMass.lcaMass.inertia,
      pFrAxleMass.lcaMass.m,
      pFrAxleMass.lcaMass.rCM - pVehicleCG)
    + BobLib.Utilities.Mechanics.translateInertia(
      pFrAxleMass.tieMass.inertia,
      pFrAxleMass.tieMass.m,
      pFrAxleMass.tieMass.rCM - pVehicleCG)
    + BobLib.Utilities.Mechanics.translateInertia(
      pRrAxleMass.unsprungMass.inertia,
      pRrAxleMass.unsprungMass.m,
      pRrAxleMass.unsprungMass.rCM - pVehicleCG)
    + BobLib.Utilities.Mechanics.translateInertia(
      pRrAxleMass.ucaMass.inertia,
      pRrAxleMass.ucaMass.m,
      pRrAxleMass.ucaMass.rCM - pVehicleCG)
    + BobLib.Utilities.Mechanics.translateInertia(
      pRrAxleMass.lcaMass.inertia,
      pRrAxleMass.lcaMass.m,
      pRrAxleMass.lcaMass.rCM - pVehicleCG)
    + BobLib.Utilities.Mechanics.translateInertia(
      pRrAxleMass.tieMass.inertia,
      pRrAxleMass.tieMass.m,
      pRrAxleMass.tieMass.rCM - pVehicleCG);
  
  replaceable BobLib.Vehicle.Chassis.ChassisBase chassis annotation(
    Placement(transformation(origin = {-0.2, -0.333347}, extent = {{-58.8, -65.3333}, {58.8, 65.3333}})));
  
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frameFL annotation(
    Placement(transformation(origin = {-100, 40}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {-180, 70}, extent = {{-16, -16}, {16, 16}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frameFR annotation(
    Placement(transformation(origin = {100, 40}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {180, 70}, extent = {{-16, -16}, {16, 16}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frameRL annotation(
    Placement(transformation(origin = {-100, -70}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {-180, -170}, extent = {{-16, -16}, {16, 16}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frameRR annotation(
    Placement(transformation(origin = {100, -70}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {180, -170}, extent = {{-16, -16}, {16, 16}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b cgFrame annotation(
    Placement(transformation(origin = {100, 0}, extent = {{-16, -16}, {16, 16}}), iconTransformation(origin = {180, 0}, extent = {{-16, -16}, {16, 16}})));
  Modelica.Mechanics.Rotational.Interfaces.Flange_a steerFlange annotation(
    Placement(transformation(origin = {0, 100}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {0, 166}, extent = {{-10, -10}, {10, 10}})));
  
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation toAeroFrame(r = pAero.aeroRef - pVehicleCG, animation = false) annotation(
    Placement(transformation(origin = {-30, 0}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Forces.WorldForceAndTorque aeroLoads(resolveInFrame = Modelica.Mechanics.MultiBody.Types.ResolveInFrameB.frame_b)  annotation(
    Placement(transformation(origin = {-70, 0}, extent = {{-10, -10}, {10, 10}})));
  BobLib.Vehicle.Aero.CFDAeroMap aeroModel(pAero = pAero) annotation(
    Placement(transformation(origin = {-90, 90}, extent = {{-10, -10}, {10, 10}}, rotation = -0)));
  
  // Ride height references
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation toFL_RH(r = pAero.FL_RideHeightRef - chassis.frAxleDW.effectiveCenter) annotation(
    Placement(transformation(origin = {-40, 70}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation toFR_RH(r = mirrorXZ(pAero.FL_RideHeightRef - chassis.frAxleDW.effectiveCenter)) annotation(
    Placement(transformation(origin = {40, 70}, extent = {{-10, -10}, {10, 10}}, rotation = -0)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation toRL_RH(r = pAero.RL_RideHeightRef - chassis.rrAxleDW.effectiveCenter) annotation(
    Placement(transformation(origin = {-40, -70}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation toRR_RH(r = mirrorXZ(pAero.RL_RideHeightRef - chassis.rrAxleDW.effectiveCenter)) annotation(
    Placement(transformation(origin = {40, -70}, extent = {{-10, -10}, {10, 10}}, rotation = -0)));

equation
  aeroModel.frontRideHeight = (toFL_RH.frame_b.r_0[3] + toFR_RH.frame_b.r_0[3]) / 2;
  aeroModel.rearRideHeight = (toRL_RH.frame_b.r_0[3] + toRR_RH.frame_b.r_0[3]) / 2;
  aeroModel.speed = norm(chassis.sprungBody.v_0);
  aeroLoads.force = aeroModel.force;
  aeroLoads.torque = aeroModel.torque;
  
  connect(frameFL, chassis.frameFL) annotation(
    Line(points = {{-100, 40}, {-90, 40}, {-90, 22}, {-58, 22}}));
  connect(frameFR, chassis.frameFR) annotation(
    Line(points = {{100, 40}, {90, 40}, {90, 22}, {58, 22}}));
  connect(frameRL, chassis.frameRL) annotation(
    Line(points = {{-100, -70}, {-90, -70}, {-90, -56}, {-58, -56}}));
  connect(frameRR, chassis.frameRR) annotation(
    Line(points = {{100, -70}, {90, -70}, {90, -56}, {58, -56}}));
  connect(chassis.cgFrame, cgFrame) annotation(
    Line(points = {{58, 0}, {100, 0}}, color = {95, 95, 95}));
  connect(steerFlange, chassis.frSteerFlange) annotation(
    Line(points = {{0, 100}, {0, 54}}));
  connect(toAeroFrame.frame_b, aeroLoads.frame_b) annotation(
    Line(points = {{-40, 0}, {-60, 0}}, color = {95, 95, 95}));
  connect(toFL_RH.frame_a, chassis.frAxleFrame) annotation(
    Line(points = {{-30, 70}, {-20, 70}, {-20, 38}, {0, 38}}, color = {95, 95, 95}));
  connect(toFR_RH.frame_a, chassis.frAxleFrame) annotation(
    Line(points = {{30, 70}, {20, 70}, {20, 38}, {0, 38}}, color = {95, 95, 95}));
  connect(toRL_RH.frame_a, chassis.rrAxleFrame) annotation(
    Line(points = {{-30, -70}, {-20, -70}, {-20, -40}, {0, -40}}, color = {95, 95, 95}));
  connect(toRR_RH.frame_a, chassis.rrAxleFrame) annotation(
    Line(points = {{30, -70}, {20, -70}, {20, -40}, {0, -40}}, color = {95, 95, 95}));
  connect(toAeroFrame.frame_a, chassis.cgFrame) annotation(
    Line(points = {{-20, 0}, {-10, 0}, {-10, -20}, {58, -20}, {58, 0}}, color = {95, 95, 95}));  
annotation(
    Diagram(graphics),
    Icon(graphics = {Line(origin = {-25, 105}, points = {{-35, -15}, {25, 15}}), Line(origin = {-30, 135}, points = {{-30, -9}, {30, -15}}), Line(origin = {-82, 94}, points = {{22, -4}, {-24, 4}}, thickness = 5), Line(origin = {-82, 130}, points = {{22, -4}, {-22, 6}}, thickness = 5), Ellipse(origin = {-60, 90}, lineColor = {255, 0, 0}, fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid, extent = {{-4, 4}, {4, -4}}), Ellipse(origin = {-60, 126}, lineColor = {255, 0, 0}, fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid, extent = {{-4, 4}, {4, -4}}), Line(origin = {-30, 135}, points = {{90, -9}, {30, -15}}), Line(origin = {35, 65}, points = {{-35, 55}, {25, 25}}), Line(origin = {84, 86}, points = {{22, 12}, {-24, 4}}, thickness = 5, arrowSize = 2), Line(origin = {81, 131}, points = {{-21, -5}, {23, 5}}, thickness = 5), Ellipse(origin = {60, 126}, lineColor = {255, 0, 0}, fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid, extent = {{-4, 4}, {4, -4}}), Ellipse(origin = {60, 90}, lineColor = {255, 0, 0}, fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid, extent = {{-4, 4}, {4, -4}}), Line(origin = {-160, 74}, points = {{-20, -4}, {40, -4}, {40, 6}}), Line(origin = {160, 74}, points = {{20, -4}, {-40, -4}, {-40, 6}}), Line(origin = {-80, 106}, points = {{20, -6}, {-22, 6}}, thickness = 5), Ellipse(origin = {-60, 100}, lineColor = {255, 0, 0}, fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid, extent = {{-4, 4}, {4, -4}}), Rectangle(origin = {-120, 120}, fillColor = {71, 71, 71}, fillPattern = FillPattern.Solid, extent = {{-20, 40}, {20, -40}}, radius = 5), Line(origin = {40, 106}, points = {{20, -6}, {64, 6}}, thickness = 5), Ellipse(origin = {60, 100}, lineColor = {255, 0, 0}, fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid, extent = {{-4, 4}, {4, -4}}), Rectangle(origin = {120, 120}, fillColor = {71, 71, 71}, fillPattern = FillPattern.Solid, extent = {{-20, 40}, {20, -40}}, radius = 5), Line(origin = {-25, 105}, points = {{-35, -15}, {25, 15}}), Line(origin = {35, 65}, points = {{-35, 55}, {25, 25}}), Line(origin = {-10, 100}, points = {{-36, 0}, {56, 0}}, thickness = 8), Line(origin = {-50, 100}, points = {{4, 0}, {-4, 0}}, color = {255, 0, 0}, thickness = 5), Line(origin = {50, 100}, points = {{4, 0}, {-4, 0}}, color = {255, 0, 0}, thickness = 5), Line(origin = {0, 116}, points = {{0, 4}, {0, -12}}), Line(origin = {-25, -135}, points = {{-35, -15}, {25, 15}}), Line(origin = {-30, -105}, points = {{-30, -9}, {30, -15}}), Line(origin = {-82, -146}, points = {{22, -4}, {-24, 4}}, thickness = 5), Line(origin = {-82, -110}, points = {{22, -4}, {-22, 6}}, thickness = 5), Ellipse(origin = {-60, -150}, lineColor = {255, 0, 0}, fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid, extent = {{-4, 4}, {4, -4}}), Ellipse(origin = {-60, -114}, lineColor = {255, 0, 0}, fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid, extent = {{-4, 4}, {4, -4}}), Line(origin = {-30, -105}, points = {{90, -9}, {30, -15}}), Line(origin = {35, -175}, points = {{-35, 55}, {25, 25}}), Line(origin = {84, -154}, points = {{22, 12}, {-24, 4}}, thickness = 5, arrowSize = 2), Line(origin = {81, -109}, points = {{-21, -5}, {23, 5}}, thickness = 5), Ellipse(origin = {60, -114}, lineColor = {255, 0, 0}, fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid, extent = {{-4, 4}, {4, -4}}), Ellipse(origin = {60, -150}, lineColor = {255, 0, 0}, fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid, extent = {{-4, 4}, {4, -4}}), Line(origin = {-160, -166}, points = {{-20, -4}, {40, -4}, {40, 6}}), Line(origin = {160, -166}, points = {{20, -4}, {-40, -4}, {-40, 6}}), Line(origin = {-80, -134}, points = {{20, -6}, {-22, 6}}, thickness = 5), Ellipse(origin = {-60, -140}, lineColor = {255, 0, 0}, fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid, extent = {{-4, 4}, {4, -4}}), Rectangle(origin = {-120, -120}, fillColor = {71, 71, 71}, fillPattern = FillPattern.Solid, extent = {{-20, 40}, {20, -40}}, radius = 5), Line(origin = {40, -134}, points = {{20, -6}, {64, 6}}, thickness = 5), Ellipse(origin = {60, -140}, lineColor = {255, 0, 0}, fillColor = {255, 0, 0}, fillPattern = FillPattern.Solid, extent = {{-4, 4}, {4, -4}}), Rectangle(origin = {120, -120}, fillColor = {71, 71, 71}, fillPattern = FillPattern.Solid, extent = {{-20, 40}, {20, -40}}, radius = 5), Line(origin = {-25, -135}, points = {{-35, -15}, {25, 15}}), Line(origin = {35, -175}, points = {{-35, 55}, {25, 25}}), Line(origin = {-10, -140}, points = {{-44, 0}, {64, 0}}, thickness = 8), Line(origin = {0, -124}, points = {{0, 4}, {0, -12}}), Line(origin = {0, 133}, points = {{0, -33}, {0, 33}}, thickness = 5), Ellipse(origin = {0, 166}, lineThickness = 5, extent = {{-26, 26}, {26, -26}}), Line(origin = {-10, 176}, points = {{10, -10}, {-14, -2}}, thickness = 5), Line(origin = {10, 176}, points = {{-10, -10}, {14, -2}}, thickness = 5), Ellipse(origin = {0, 166}, lineColor = {255, 255, 255}, lineThickness = 1, extent = {{-28, 28}, {28, -28}}), Line(origin = {0, -80}, rotation = 90, points = {{-40, 0}, {60, 0}}, thickness = 5), Line(origin = {0, 80}, rotation = -90, points = {{60, 0}, {-40, 0}}, thickness = 5), Line(origin = {28, -24}, rotation = -90, points = {{-3, -28}, {-15, 36}}, thickness = 1), Line(origin = {75, 36}, points = {{-15, -26}, {-75, -16}}, thickness = 1), Ellipse(origin = {60, 0}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, lineThickness = 1, extent = {{-10, 10}, {10, -10}}), Line(origin = {60, 0}, points = {{0, -10}, {0, 10}}), Line(origin = {60, 0}, points = {{-10, 0}, {10, 0}}), Polygon(origin = {65, 5}, fillPattern = FillPattern.Solid, points = {{-5, -5}, {-5, 5}, {-3, 5}, {1, 3}, {3, 1}, {5, -3}, {5, -5}, {-5, -5}}), Polygon(origin = {55, -5}, rotation = 180, fillPattern = FillPattern.Solid, points = {{-5, -5}, {-5, 5}, {-3, 5}, {1, 3}, {3, 1}, {5, -3}, {5, -5}, {-5, -5}}), Line(origin = {1.03, 0}, points = {{-1.02758, -20}, {-1.02758, -6}, {-5.02758, -4}, {2.97242, 0}, {-5.02758, 4}, {2.97242, 8}, {-1.02758, 10}, {-1.02758, 20}}, color = {0, 170, 0}, thickness = 2), Text(rotation = 90, textColor = {255, 0, 0}, extent = {{-14, 14}, {14, -14}}, textString = "rxDOF"), Line(origin = {120, 0}, points = {{-60, 0}, {60, 0}})}, coordinateSystem(extent = {{-180, -200}, {180, 200}})));
end VehicleBase;