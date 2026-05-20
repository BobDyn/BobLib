#!/usr/bin/env python3
# ruff: noqa: E501
from __future__ import annotations

from pathlib import Path
from textwrap import dedent
from typing import cast

from build_common import (
    axle_model_name,
    boblib_axle_model_path,
    load_yaml,
    norm_arch,
    prune_axle_models,
    require_key,
    require_section,
    require_side,
    side_parameters,
    replace_tokens,
    vehicle_yaml_path,
    write_text_file,
)


_DIRECT_TEMPLATE = dedent(
    """\
    within BobLib.Vehicle.Chassis.Suspension;

    model __MODEL_NAME__ "Double wishbone axle with direct-acting suspension"
      import Modelica.SIunits;
      import Modelica.Math.Vectors;
      import BobLib.Utilities.Math.Vector.mirrorXZ;
      import BobLib.Resources.VehicleRecord.Chassis.Suspension.AxleDW_DirectRecord;

      // Record parameters
      parameter AxleDW_DirectRecord pAxle;

      extends BobLib.Vehicle.Chassis.Suspension.AxleDWBase;

      // Left shock
      BobLib.Vehicle.Chassis.Suspension.Linkages.ShockLinkage leftShockLinkage(
        r_a = pAxle.rodMount,
        r_b = pAxle.shockMount,
        s_0 = pAxle.springFreeLength,
        springTable = pAxle.springTable,
        damperTable = pAxle.damperTable,
        n_a = {1, 0, 0},
        n_b = {0, 1, 0},
        linkDiameter = linkDiameter,
        jointDiameter = jointDiameter) annotation(
        Placement(transformation(origin = {-50, -45}, extent = {{-15, -15}, {15, 15}}, rotation = -90)));

      // Right shock
      BobLib.Vehicle.Chassis.Suspension.Linkages.ShockLinkage rightShockLinkage(
        r_a = mirrorXZ(pAxle.rodMount),
        r_b = mirrorXZ(pAxle.shockMount),
        s_0 = pAxle.springFreeLength,
        springTable = pAxle.springTable,
        damperTable = pAxle.damperTable,
        n_a = {1, 0, 0},
        n_b = mirrorXZ({0, 1, 0}),
        linkDiameter = linkDiameter,
        jointDiameter = jointDiameter) annotation(
        Placement(transformation(origin = {50, -45}, extent = {{-15, -15}, {15, 15}}, rotation = -90)));

      Modelica.Mechanics.Rotational.Interfaces.Flange_a steerFlange annotation(
        Placement(transformation(origin = {0, 140}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {0, 100}, extent = {{-10, -10}, {10, 10}})));

    protected
      // Kinematics
      Modelica.Mechanics.MultiBody.Parts.FixedTranslation toLeftShock(
        r = pAxle.shockMount - effectiveCenter,
        animation = false) annotation(
        Placement(transformation(origin = {-20, -70}, extent = {{10, -10}, {-10, 10}})));
      Modelica.Mechanics.MultiBody.Parts.FixedTranslation toRightShock(
        r = mirrorXZ(pAxle.shockMount) - effectiveCenter,
        animation = false) annotation(
        Placement(transformation(origin = {20, -70}, extent = {{-10, -10}, {10, 10}})));
      __LEFT_APEX_DECL__
      __RIGHT_APEX_DECL__

    equation
      __ROD_CONNECTS__
      connect(leftShockLinkage.frame_b, toLeftShock.frame_b) annotation(
        Line(points = {{-50, -60}, {-50, -70}, {-30, -70}}, color = {95, 95, 95}));
      connect(rightShockLinkage.frame_b, toRightShock.frame_b) annotation(
        Line(points = {{50, -60}, {50, -70}, {30, -70}}, color = {95, 95, 95}));
      connect(axleFrame, toLeftShock.frame_a) annotation(
        Line(points = {{0, 0}, {0, -70}, {-10, -70}}));
      connect(rackAndPinion.pinionFlange, steerFlange) annotation(
        Line(points = {{0, 114}, {0, 140}}));
      connect(toRightShock.frame_a, axleFrame) annotation(
        Line(points = {{10, -70}, {0, -70}, {0, 0}}, color = {95, 95, 95}));
      __LEFT_APEX_CONNECT__
      __RIGHT_APEX_CONNECT__
      annotation(
        experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-06, Interval = 0.002),
        Diagram(coordinateSystem(extent = {{-180, -140}, {180, 140}}, preserveAspectRatio = true), graphics),
        Icon(coordinateSystem(extent = {{-180, -20}, {180, 120}}, preserveAspectRatio = true)));

    end __MODEL_NAME__;
    """
)


_BELLCRANK_TEMPLATE = dedent(
    """\
    within BobLib.Vehicle.Chassis.Suspension;

    model __MODEL_NAME__ "Double wishbone axle with bellcranks mounting to shock and push/pullrod"
      import Modelica.SIunits;
      import Modelica.Math.Vectors;
      import BobLib.Utilities.Math.Vector.mirrorXZ;
      import BobLib.Resources.VehicleRecord.Chassis.Suspension.AxleDW_BCRecord;

      // Record parameters
      parameter AxleDW_BCRecord pAxle;

      extends BobLib.Vehicle.Chassis.Suspension.AxleDWBase;

      // Left bellcrank
      BobLib.Vehicle.Chassis.Suspension.Linkages.Bellcrank2 leftBellcrank(
        pivot = pAxle.bellcrankPivot,
        pivotAxis = pAxle.bellcrankPivotAxis,
        pickup_1 = __LEFT_PICKUP_1__,
        pickup_2 = __LEFT_PICKUP_2__,
        linkDiameter = linkDiameter,
        jointDiameter = jointDiameter) annotation(
        Placement(transformation(origin = {-50, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));

      // Left shock
      BobLib.Vehicle.Chassis.Suspension.Linkages.ShockLinkage leftShockLinkage(
        r_a = pAxle.bellcrankShockPickup,
        r_b = pAxle.shockMount,
        s_0 = pAxle.springFreeLength,
        springTable = pAxle.springTable,
        damperTable = pAxle.damperTable,
        n_a = pAxle.bellcrankPivotAxis,
        n_b = Vectors.normalize(pAxle.bellcrankPivot - pAxle.bellcrankShockPickup),
        linkDiameter = linkDiameter,
        jointDiameter = jointDiameter) annotation(
        Placement(transformation(origin = {-50, -55}, extent = {{-15, -15}, {15, 15}}, rotation = -90)));

      // Right bellcrank
      BobLib.Vehicle.Chassis.Suspension.Linkages.Bellcrank2 rightBellcrank(
        pivot = mirrorXZ(pAxle.bellcrankPivot),
        pivotAxis = mirrorXZ(pAxle.bellcrankPivotAxis),
        pickup_1 = __RIGHT_PICKUP_1__,
        pickup_2 = __RIGHT_PICKUP_2__,
        linkDiameter = linkDiameter,
        jointDiameter = jointDiameter) annotation(
        Placement(transformation(origin = {50, -20}, extent = {{10, -10}, {-10, 10}}, rotation = -180)));

      // Right shock
      BobLib.Vehicle.Chassis.Suspension.Linkages.ShockLinkage rightShockLinkage(
        r_a = mirrorXZ(pAxle.bellcrankShockPickup),
        r_b = mirrorXZ(pAxle.shockMount),
        s_0 = pAxle.springFreeLength,
        springTable = pAxle.springTable,
        damperTable = pAxle.damperTable,
        n_a = mirrorXZ(pAxle.bellcrankPivotAxis),
        n_b = Vectors.normalize(mirrorXZ(pAxle.bellcrankPivot - pAxle.bellcrankShockPickup)),
        linkDiameter = linkDiameter,
        jointDiameter = jointDiameter) annotation(
        Placement(transformation(origin = {50, -55}, extent = {{-15, -15}, {15, 15}}, rotation = -90)));

      Modelica.Mechanics.Rotational.Interfaces.Flange_a steerFlange annotation(
        Placement(transformation(origin = {0, 140}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {0, 100}, extent = {{-10, -10}, {10, 10}})));

    protected
      // Kinematics
      Modelica.Mechanics.MultiBody.Parts.FixedTranslation toLeftBellcrank(
        r = pAxle.bellcrankPivot - effectiveCenter,
        animation = false) annotation(
        Placement(transformation(origin = {-20, -20}, extent = {{10, -10}, {-10, 10}})));
      Modelica.Mechanics.MultiBody.Parts.FixedTranslation toLeftShock(
        r = pAxle.shockMount - effectiveCenter,
        animation = false) annotation(
        Placement(transformation(origin = {-20, -70}, extent = {{10, -10}, {-10, 10}})));
      Modelica.Mechanics.MultiBody.Parts.FixedTranslation toRightBellcrank(
        r = mirrorXZ(pAxle.bellcrankPivot) - effectiveCenter,
        animation = false) annotation(
        Placement(transformation(origin = {20, -20}, extent = {{-10, -10}, {10, 10}})));
      Modelica.Mechanics.MultiBody.Parts.FixedTranslation toRightShock(
        r = mirrorXZ(pAxle.shockMount) - effectiveCenter,
        animation = false) annotation(
        Placement(transformation(origin = {20, -70}, extent = {{-10, -10}, {10, 10}})));
      Modelica.Mechanics.MultiBody.Parts.FixedTranslation toLeftApex(
        r = __ROD_ATTACH_LEFT__,
        animation = false) annotation(
        Placement(transformation(origin = {-80, -20}, extent = {{10, -10}, {-10, 10}})));
      Modelica.Mechanics.MultiBody.Parts.FixedTranslation toRightApex(
        r = __ROD_ATTACH_RIGHT__,
        animation = false) annotation(
        Placement(transformation(origin = {90, -10}, extent = {{-10, -10}, {10, 10}})));

    public
      BobLib.Vehicle.Chassis.Suspension.Linkages.Rod leftPushrod(
        jointDiameter = jointDiameter,
        kinematicConstraint = true,
        linkDiameter = linkDiameter,
        n1_a = Vectors.normalize(pAxle.bellcrankPivotAxis),
        r_a = pAxle.bellcrankRodPickup,
        r_b = pAxle.rodMount) annotation(
        Placement(transformation(origin = {-120, -30}, extent = {{20, -20}, {-20, 20}})));
      BobLib.Vehicle.Chassis.Suspension.Linkages.Rod rightPushrod(
        jointDiameter = jointDiameter,
        kinematicConstraint = true,
        linkDiameter = linkDiameter,
        n1_a = Vectors.normalize(mirrorXZ(pAxle.bellcrankPivotAxis)),
        r_a = mirrorXZ(pAxle.bellcrankRodPickup),
        r_b = mirrorXZ(pAxle.rodMount)) annotation(
        Placement(transformation(origin = {120, -30}, extent = {{-20, -20}, {20, 20}})));

    equation
      __ROD_CONNECTS__
      __PUSHPROD_CONNECTS__
      __SHOCK_CONNECTS__

      connect(leftBellcrank.mountFrame, toLeftBellcrank.frame_b) annotation(
        Line(points = {{-40, -20}, {-30, -20}}, color = {95, 95, 95}));
      connect(leftShockLinkage.frame_b, toLeftShock.frame_b) annotation(
        Line(points = {{-50, -70}, {-30, -70}}, color = {95, 95, 95}));
      connect(toRightBellcrank.frame_b, rightBellcrank.mountFrame) annotation(
        Line(points = {{30, -20}, {40, -20}}, color = {95, 95, 95}));
      connect(rightShockLinkage.frame_b, toRightShock.frame_b) annotation(
        Line(points = {{50, -70}, {30, -70}}, color = {95, 95, 95}));
      connect(axleFrame, toLeftBellcrank.frame_a) annotation(
        Line(points = {{0, 0}, {0, -20}, {-10, -20}}));
      connect(axleFrame, toRightBellcrank.frame_a) annotation(
        Line(points = {{0, 0}, {0, -20}, {10, -20}}));
      connect(axleFrame, toLeftShock.frame_a) annotation(
        Line(points = {{0, 0}, {0, -70}, {-10, -70}}));
      connect(toRightBellcrank.frame_a, toRightShock.frame_a) annotation(
        Line(points = {{10, -20}, {0, -20}, {0, -70}, {10, -70}}, color = {95, 95, 95}));
      connect(rackAndPinion.pinionFlange, steerFlange) annotation(
        Line(points = {{0, 114}, {0, 140}}));
      connect(toRightShock.frame_a, axleFrame) annotation(
        Line(points = {{10, -70}, {0, -70}, {0, 0}}, color = {95, 95, 95}));
      connect(toLeftApex.frame_b, leftPushrod.frame_b) annotation(
        Line(points = {{-100, -10}, {-150, -10}, {-150, -30}, {-140, -30}}, color = {95, 95, 95}));
      connect(toRightApex.frame_b, rightPushrod.frame_b) annotation(
        Line(points = {{100, -10}, {150, -10}, {150, -30}, {140, -30}}, color = {95, 95, 95}));
      annotation(
        experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-06, Interval = 0.002),
        Diagram(coordinateSystem(extent = {{-180, -140}, {180, 140}}, preserveAspectRatio = true), graphics),
        Icon(coordinateSystem(extent = {{-180, -20}, {180, 140}}, preserveAspectRatio = true), graphics = {Line(origin = {0, 67}, points = {{0, -33}, {0, 33}}, thickness = 5), Ellipse(origin = {0, 100}, lineThickness = 5, extent = {{-26, 26}, {26, -26}}), Line(origin = {-10, 110}, points = {{10, -10}, {-14, -2}}, thickness = 5), Line(origin = {10, 110}, points = {{-10, -10}, {14, -2}}, thickness = 5), Ellipse(origin = {0, 100}, lineColor = {255, 255, 255}, lineThickness = 1, extent = {{-28, 28}, {28, -28}})}));

    end __MODEL_NAME__;
    """
)


_STABAR_TEMPLATE = dedent(
    """\
    within BobLib.Vehicle.Chassis.Suspension;

    model __MODEL_NAME__ "Double wishbone axle with bellcranks mounting to shock, push/pullrod, and stabar"
      import Modelica.SIunits;
      import Modelica.Math.Vectors;
      import BobLib.Utilities.Math.Vector.mirrorXZ;
      import BobLib.Resources.VehicleRecord.Chassis.Suspension.AxleDW_BC_StabarRecord;
      import BobLib.Resources.VehicleRecord.Chassis.Suspension.Templates.Stabar.StabarRecord;

      // Record parameters
      parameter AxleDW_BC_StabarRecord pAxle;
      parameter StabarRecord pStabar;

      extends BobLib.Vehicle.Chassis.Suspension.AxleDWBase;

      // Left bellcrank
      BobLib.Vehicle.Chassis.Suspension.Linkages.Bellcrank3 leftBellcrank(
        pivot = pAxle.bellcrankPivot,
        pivotAxis = pAxle.bellcrankPivotAxis,
        pickup_1 = __LEFT_PICKUP_1__,
        pickup_2 = __LEFT_PICKUP_2__,
        pickup_3 = __LEFT_PICKUP_3__,
        linkDiameter = linkDiameter,
        jointDiameter = jointDiameter) annotation(
        Placement(transformation(origin = {-50, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));

      // Left shock
      BobLib.Vehicle.Chassis.Suspension.Linkages.ShockLinkage leftShockLinkage(
        r_a = pAxle.bellcrankShockPickup,
        r_b = pAxle.shockMount,
        s_0 = pAxle.springFreeLength,
        springTable = pAxle.springTable,
        damperTable = pAxle.damperTable,
        n_a = pAxle.bellcrankPivotAxis,
        n_b = Vectors.normalize(pAxle.bellcrankPivot - pAxle.bellcrankShockPickup),
        linkDiameter = linkDiameter,
        jointDiameter = jointDiameter) annotation(
        Placement(transformation(origin = {-50, -55}, extent = {{-15, -15}, {15, 15}}, rotation = -90)));

      // Right bellcrank
      BobLib.Vehicle.Chassis.Suspension.Linkages.Bellcrank3 rightBellcrank(
        pivot = mirrorXZ(pAxle.bellcrankPivot),
        pivotAxis = mirrorXZ(pAxle.bellcrankPivotAxis),
        pickup_1 = __RIGHT_PICKUP_1__,
        pickup_2 = __RIGHT_PICKUP_2__,
        pickup_3 = __RIGHT_PICKUP_3__,
        linkDiameter = linkDiameter,
        jointDiameter = jointDiameter) annotation(
        Placement(transformation(origin = {50, -20}, extent = {{10, -10}, {-10, 10}}, rotation = -180)));

      // Right shock
      BobLib.Vehicle.Chassis.Suspension.Linkages.ShockLinkage rightShockLinkage(
        r_a = mirrorXZ(pAxle.bellcrankShockPickup),
        r_b = mirrorXZ(pAxle.shockMount),
        s_0 = pAxle.springFreeLength,
        springTable = pAxle.springTable,
        damperTable = pAxle.damperTable,
        n_a = mirrorXZ(pAxle.bellcrankPivotAxis),
        n_b = Vectors.normalize(mirrorXZ(pAxle.bellcrankPivot - pAxle.bellcrankShockPickup)),
        linkDiameter = linkDiameter,
        jointDiameter = jointDiameter) annotation(
        Placement(transformation(origin = {50, -55}, extent = {{-15, -15}, {15, 15}}, rotation = -90)));

      // Stabar
      BobLib.Vehicle.Chassis.Suspension.Templates.Stabar.Stabar stabar(
        pStabar = pStabar,
        jointDiameter = jointDiameter,
        linkDiameter = linkDiameter) annotation(
        Placement(transformation(origin = {0, -116}, extent = {{20, -20}, {-20, 20}}, rotation = -180)));
      Modelica.Mechanics.MultiBody.Joints.SphericalSpherical rightDroplink(
        rodLength = Vectors.norm(mirrorXZ(pAxle.bellcrankStabarPickup - pStabar.leftArmEnd)),
        sphereDiameter = jointDiameter,
        rodDiameter = linkDiameter) annotation(
        Placement(transformation(origin = {70, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
      Modelica.Mechanics.MultiBody.Joints.SphericalSpherical leftDroplink(
        rodLength = Vectors.norm(pAxle.bellcrankStabarPickup - pStabar.leftArmEnd),
        sphereDiameter = jointDiameter,
        rodDiameter = linkDiameter) annotation(
        Placement(transformation(origin = {-70, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));

      Modelica.Mechanics.Rotational.Interfaces.Flange_a steerFlange annotation(
        Placement(transformation(origin = {0, 140}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {0, 100}, extent = {{-10, -10}, {10, 10}})));

    protected
      // Kinematics
      Modelica.Mechanics.MultiBody.Parts.FixedTranslation toLeftBellcrank(
        r = pAxle.bellcrankPivot - effectiveCenter,
        animation = false) annotation(
        Placement(transformation(origin = {-20, -20}, extent = {{10, -10}, {-10, 10}})));
      Modelica.Mechanics.MultiBody.Parts.FixedTranslation toLeftShock(
        r = pAxle.shockMount - effectiveCenter,
        animation = false) annotation(
        Placement(transformation(origin = {-20, -70}, extent = {{10, -10}, {-10, 10}})));
      Modelica.Mechanics.MultiBody.Parts.FixedTranslation toRightBellcrank(
        r = mirrorXZ(pAxle.bellcrankPivot) - effectiveCenter,
        animation = false) annotation(
        Placement(transformation(origin = {20, -20}, extent = {{-10, -10}, {10, 10}})));
      Modelica.Mechanics.MultiBody.Parts.FixedTranslation toRightShock(
        r = mirrorXZ(pAxle.shockMount) - effectiveCenter,
        animation = false) annotation(
        Placement(transformation(origin = {20, -70}, extent = {{-10, -10}, {10, 10}})));
      Modelica.Mechanics.MultiBody.Parts.FixedTranslation toLeftApex(
        r = __ROD_ATTACH_LEFT__,
        animation = false) annotation(
        Placement(transformation(origin = {-80, -20}, extent = {{10, -10}, {-10, 10}})));
      Modelica.Mechanics.MultiBody.Parts.FixedTranslation toRightApex(
        r = __ROD_ATTACH_RIGHT__,
        animation = false) annotation(
        Placement(transformation(origin = {90, -10}, extent = {{-10, -10}, {10, 10}})));
      Modelica.Mechanics.MultiBody.Parts.FixedTranslation toStabar(
        r = {pStabar.leftBarEnd[1], 0, pStabar.leftBarEnd[3]} - effectiveCenter,
        animation = false) annotation(
        Placement(transformation(origin = {0, -90}, extent = {{-10, -10}, {10, 10}}, rotation = -90)));

    public
      BobLib.Vehicle.Chassis.Suspension.Linkages.Rod leftPushrod(
        jointDiameter = jointDiameter,
        kinematicConstraint = true,
        linkDiameter = linkDiameter,
        n1_a = Vectors.normalize(pAxle.bellcrankPivotAxis),
        r_a = pAxle.bellcrankRodPickup,
        r_b = pAxle.rodMount) annotation(
        Placement(transformation(origin = {-120, -30}, extent = {{20, -20}, {-20, 20}})));
      BobLib.Vehicle.Chassis.Suspension.Linkages.Rod rightPushrod(
        jointDiameter = jointDiameter,
        kinematicConstraint = true,
        linkDiameter = linkDiameter,
        n1_a = Vectors.normalize(mirrorXZ(pAxle.bellcrankPivotAxis)),
        r_a = mirrorXZ(pAxle.bellcrankRodPickup),
        r_b = mirrorXZ(pAxle.rodMount)) annotation(
        Placement(transformation(origin = {120, -30}, extent = {{-20, -20}, {20, 20}})));

    equation
      __ROD_CONNECTS__
      __PUSHPROD_CONNECTS__
      __SHOCK_CONNECTS__
      __STABAR_CONNECTS__

      connect(leftBellcrank.mountFrame, toLeftBellcrank.frame_b) annotation(
        Line(points = {{-40, -20}, {-30, -20}}, color = {95, 95, 95}));
      connect(leftShockLinkage.frame_b, toLeftShock.frame_b) annotation(
        Line(points = {{-50, -70}, {-30, -70}}, color = {95, 95, 95}));
      connect(toRightBellcrank.frame_b, rightBellcrank.mountFrame) annotation(
        Line(points = {{30, -20}, {40, -20}}, color = {95, 95, 95}));
      connect(rightShockLinkage.frame_b, toRightShock.frame_b) annotation(
        Line(points = {{50, -70}, {30, -70}}, color = {95, 95, 95}));
      connect(axleFrame, toLeftBellcrank.frame_a) annotation(
        Line(points = {{0, 0}, {0, -20}, {-10, -20}}));
      connect(axleFrame, toRightBellcrank.frame_a) annotation(
        Line(points = {{0, 0}, {0, -20}, {10, -20}}));
      connect(axleFrame, toLeftShock.frame_a) annotation(
        Line(points = {{0, 0}, {0, -70}, {-10, -70}}));
      connect(toRightBellcrank.frame_a, toRightShock.frame_a) annotation(
        Line(points = {{10, -20}, {0, -20}, {0, -70}, {10, -70}}, color = {95, 95, 95}));
      connect(axleFrame, toStabar.frame_a) annotation(
        Line(points = {{0, 0}, {0, -80}}));
      connect(toStabar.frame_b, stabar.supportFrame) annotation(
        Line(points = {{0, -100}, {0, -110}}, color = {95, 95, 95}));
      connect(stabar.rightArmFrame, rightDroplink.frame_a) annotation(
        Line(points = {{20, -120}, {70, -120}, {70, -100}}, color = {95, 95, 95}));
      connect(stabar.leftArmFrame, leftDroplink.frame_a) annotation(
        Line(points = {{-20, -120}, {-70, -120}, {-70, -100}}, color = {95, 95, 95}));
      connect(rackAndPinion.pinionFlange, steerFlange) annotation(
        Line(points = {{0, 114}, {0, 140}}));
      connect(toRightShock.frame_a, axleFrame) annotation(
        Line(points = {{10, -70}, {0, -70}, {0, 0}}, color = {95, 95, 95}));
      connect(toLeftApex.frame_b, leftPushrod.frame_b) annotation(
        Line(points = {{-100, -10}, {-150, -10}, {-150, -30}, {-140, -30}}, color = {95, 95, 95}));
      connect(toRightApex.frame_b, rightPushrod.frame_b) annotation(
        Line(points = {{100, -10}, {150, -10}, {150, -30}, {140, -30}}, color = {95, 95, 95}));
      annotation(
        experiment(StartTime = 0, StopTime = 1, Tolerance = 1e-06, Interval = 0.002),
        Diagram(coordinateSystem(extent = {{-180, -140}, {180, 140}}, preserveAspectRatio = true), graphics),
        Icon(coordinateSystem(extent = {{-180, -20}, {180, 140}}, preserveAspectRatio = true), graphics = {Line(origin = {0, 67}, points = {{0, -33}, {0, 33}}, thickness = 5), Ellipse(origin = {0, 100}, lineThickness = 5, extent = {{-26, 26}, {26, -26}}), Line(origin = {-10, 110}, points = {{10, -10}, {-14, -2}}, thickness = 5), Line(origin = {10, 110}, points = {{-10, -10}, {14, -2}}, thickness = 5), Ellipse(origin = {0, 100}, lineColor = {255, 255, 255}, lineThickness = 1, extent = {{-28, 28}, {28, -28}})}));
    end __MODEL_NAME__;
    """
)


def _side_axle_fields(
    data: dict[str, object],
    side_name: str,
    prefix: str,
    topology: str,
) -> dict[str, object]:
    axle_fields = side_parameters(data, side_name, prefix, topology)[0][2]
    suspension = require_section(require_side(data, side_name), side_name, "suspension")
    axle_fields["upper_o"] = require_key(suspension, f"{side_name}.suspension", "upper_o_m")
    axle_fields["lower_o"] = require_key(suspension, f"{side_name}.suspension", "lower_o_m")
    return axle_fields


def _rod_attach_expr(axle_fields: dict[str, object], *, mirrored: bool) -> str:
    attach = "pLeftDW.lower_o" if bool(axle_fields["rodToLower"]) else "pLeftDW.upper_o"
    expr = f"pAxle.rodMount - {attach}"
    return f"mirrorXZ({expr})" if mirrored else expr


def _rod_attach_is_zero(axle_fields: dict[str, object]) -> bool:
    attach_key = "lower_o" if bool(axle_fields["rodToLower"]) else "upper_o"
    rod_mount = cast(tuple[float, float, float], axle_fields["rodMount"])
    attach = cast(tuple[float, float, float], axle_fields[attach_key])
    return max(abs(float(a) - float(b)) for a, b in zip(rod_mount, attach)) < 1e-12


def _rod_connects(axle_fields: dict[str, object]) -> str:
    if bool(axle_fields["rodToLower"]):
        return (
            "connect(toLeftApex.frame_a, leftWishboneUprightLoop.lowerFrame_o);\n"
            "      connect(toRightApex.frame_a, rightWishboneUprightLoop.lowerFrame_o);"
        )
    return (
        "connect(toLeftApex.frame_a, leftWishboneUprightLoop.upperFrame_o);\n"
        "      connect(toRightApex.frame_a, rightWishboneUprightLoop.upperFrame_o);"
    )


def _role_expr(role: str, *, mirrored: bool) -> str:
    base = {
        "rod": "pAxle.bellcrankRodPickup",
        "shock": "pAxle.bellcrankShockPickup",
        "stabar": "pAxle.bellcrankStabarPickup",
    }
    if role not in base:
        raise KeyError(f"Unknown bellcrank pickup role: {role!r}")
    expr = base[role]
    return f"mirrorXZ({expr})" if mirrored else expr


def _pickup_exprs(axle_fields: dict[str, object], *, mirrored: bool) -> dict[int, str]:
    exprs = {
        int(cast(int, axle_fields["rodPickup"])): _role_expr("rod", mirrored=mirrored),
        int(cast(int, axle_fields["shockPickup"])): _role_expr("shock", mirrored=mirrored),
        int(cast(int, axle_fields["stabarPickup"])): _role_expr("stabar", mirrored=mirrored),
    }
    return exprs


def _pickup_exprs_two(axle_fields: dict[str, object], *, mirrored: bool) -> dict[int, str]:
    exprs = {
        int(cast(int, axle_fields["rodPickup"])): _role_expr("rod", mirrored=mirrored),
        int(cast(int, axle_fields["shockPickup"])): _role_expr("shock", mirrored=mirrored),
    }
    return exprs


def _render_direct_axle_model(
    *,
    prefix: str,
    axle_fields: dict[str, object],
) -> str:
    zero_apex = _rod_attach_is_zero(axle_fields)
    if zero_apex:
        rod_connects = (
            "connect(leftShockLinkage.frame_a, leftWishboneUprightLoop.lowerFrame_o);\n"
            "      connect(rightShockLinkage.frame_a, rightWishboneUprightLoop.lowerFrame_o);"
        ) if bool(axle_fields["rodToLower"]) else (
            "connect(leftShockLinkage.frame_a, leftWishboneUprightLoop.upperFrame_o);\n"
            "      connect(rightShockLinkage.frame_a, rightWishboneUprightLoop.upperFrame_o);"
        )
        left_apex_decl = ""
        right_apex_decl = ""
        left_apex_connect = ""
        right_apex_connect = ""
    else:
        rod_connects = _rod_connects(axle_fields)
        left_apex_decl = (
            "Modelica.Mechanics.MultiBody.Parts.FixedTranslation toLeftApex(\n"
            f"        r = {_rod_attach_expr(axle_fields, mirrored=False)},\n"
            "        animation = false) annotation(\n"
            "        Placement(transformation(origin = {-80, -20}, extent = {{10, -10}, {-10, 10}})));"
        )
        right_apex_decl = (
            "Modelica.Mechanics.MultiBody.Parts.FixedTranslation toRightApex(\n"
            f"        r = {_rod_attach_expr(axle_fields, mirrored=True)},\n"
            "        animation = false) annotation(\n"
            "        Placement(transformation(origin = {90, -10}, extent = {{-10, -10}, {10, 10}})));"
        )
        left_apex_connect = (
            "connect(toLeftApex.frame_b, leftShockLinkage.frame_a) annotation(\n"
            "        Line(points = {{-100, -10}, {-110, -10}, {-110, -30}, {-50, -30}}, color = {95, 95, 95}));"
        )
        right_apex_connect = (
            "connect(toRightApex.frame_b, rightShockLinkage.frame_a) annotation(\n"
            "        Line(points = {{100, -10}, {110, -10}, {110, -30}, {50, -30}}, color = {95, 95, 95}));"
        )
    return replace_tokens(
        _DIRECT_TEMPLATE,
        {
            "__MODEL_NAME__": axle_model_name(prefix, "direct"),
            "__LEFT_APEX_DECL__": left_apex_decl,
            "__RIGHT_APEX_DECL__": right_apex_decl,
            "__LEFT_APEX_CONNECT__": left_apex_connect,
            "__RIGHT_APEX_CONNECT__": right_apex_connect,
            "__ROD_CONNECTS__": rod_connects,
        },
    )


def _render_bellcrank_axle_model(
    *,
    prefix: str,
    axle_fields: dict[str, object],
) -> str:
    rod_idx = int(cast(int, axle_fields["rodPickup"]))
    shock_idx = int(cast(int, axle_fields["shockPickup"]))
    if rod_idx == shock_idx:
        raise ValueError("Bellcrank rod and shock pickups must be distinct.")
    if {rod_idx, shock_idx} != {1, 2}:
        raise ValueError("Bellcrank2 requires rod and shock pickups to use indices 1 and 2.")

    text = replace_tokens(
        _BELLCRANK_TEMPLATE,
        {
            "__MODEL_NAME__": axle_model_name(prefix, "bellcrank"),
            "__ROD_ATTACH_LEFT__": _rod_attach_expr(axle_fields, mirrored=False),
            "__ROD_ATTACH_RIGHT__": _rod_attach_expr(axle_fields, mirrored=True),
        },
    )
    pickup_exprs = _pickup_exprs_two(axle_fields, mirrored=False)
    right_exprs = _pickup_exprs_two(axle_fields, mirrored=True)
    return replace_tokens(
        text,
        {
            "__LEFT_PICKUP_1__": pickup_exprs[1],
            "__LEFT_PICKUP_2__": pickup_exprs[2],
            "__RIGHT_PICKUP_1__": right_exprs[1],
            "__RIGHT_PICKUP_2__": right_exprs[2],
            "__ROD_CONNECTS__": _rod_connects(axle_fields),
            "__PUSHPROD_CONNECTS__": (
                "connect(leftPushrod.frame_a, leftBellcrank.pickupFrame1);\n"
                "      connect(rightPushrod.frame_a, rightBellcrank.pickupFrame1);"
            ),
            "__SHOCK_CONNECTS__": f"connect(leftShockLinkage.frame_a, leftBellcrank.pickupFrame{shock_idx});\n      connect(rightShockLinkage.frame_a, rightBellcrank.pickupFrame{shock_idx});",
        },
    )


def _render_stabar_axle_model(
    *,
    prefix: str,
    axle_fields: dict[str, object],
) -> str:
    text = replace_tokens(
        _STABAR_TEMPLATE,
        {
            "__MODEL_NAME__": axle_model_name(prefix, "bellcrank_stabar"),
            "__ROD_ATTACH_LEFT__": _rod_attach_expr(axle_fields, mirrored=False),
            "__ROD_ATTACH_RIGHT__": _rod_attach_expr(axle_fields, mirrored=True),
            "__LEFT_PICKUP_1__": "pAxle.bellcrankRodPickup",
            "__LEFT_PICKUP_2__": "pAxle.bellcrankShockPickup",
            "__LEFT_PICKUP_3__": "pAxle.bellcrankStabarPickup",
        },
    )
    return replace_tokens(
        text,
        {
            "__RIGHT_PICKUP_1__": "mirrorXZ(pAxle.bellcrankRodPickup)",
            "__RIGHT_PICKUP_2__": "mirrorXZ(pAxle.bellcrankShockPickup)",
            "__RIGHT_PICKUP_3__": "mirrorXZ(pAxle.bellcrankStabarPickup)",
            "__ROD_CONNECTS__": _rod_connects(axle_fields),
            "__PUSHPROD_CONNECTS__": (
                "connect(leftPushrod.frame_a, leftBellcrank.pickupFrame1);\n"
                "      connect(rightPushrod.frame_a, rightBellcrank.pickupFrame1);"
            ),
            "__SHOCK_CONNECTS__": (
                "connect(leftShockLinkage.frame_a, leftBellcrank.pickupFrame2);\n"
                "      connect(rightShockLinkage.frame_a, rightBellcrank.pickupFrame2);"
            ),
            "__STABAR_CONNECTS__": (
                "connect(leftDroplink.frame_b, leftBellcrank.pickupFrame3);\n"
                "      connect(rightDroplink.frame_b, rightBellcrank.pickupFrame3);"
            ),
        },
    )


def build_axle_models(
    *,
    source_yaml: Path | None = None,
    overwrite: bool = True,
) -> list[Path]:
    source_yaml = source_yaml or vehicle_yaml_path()
    data = load_yaml(source_yaml)
    arch = data.get("architecture")
    if not isinstance(arch, dict):
        raise ValueError("vehicle.yml is missing architecture mapping.")

    outputs: list[Path] = []
    renderers = {
        "direct": _render_direct_axle_model,
        "bellcrank": _render_bellcrank_axle_model,
        "bellcrank_stabar": _render_stabar_axle_model,
    }
    for side_name, prefix in (("front", "Fr"), ("rear", "Rr")):
        topology = norm_arch(str(arch.get(side_name)))
        if topology not in renderers:
            raise NotImplementedError(
                "build_axle_models only supports direct, bellcrank, and bellcrank_stabar "
                f"topologies. Got {topology!r} for {side_name!r}."
            )
        axle_fields = _side_axle_fields(data, side_name, prefix, topology)
        path = boblib_axle_model_path(data, prefix, topology)
        path.parent.mkdir(parents=True, exist_ok=True)
        text = renderers[topology](prefix=prefix, axle_fields=axle_fields)
        write_text_file(path, text, overwrite)
        outputs.append(path)
    prune_axle_models(data, source_yaml)
    return outputs


def main() -> None:
    source_yaml = vehicle_yaml_path()
    paths = build_axle_models(source_yaml=source_yaml, overwrite=True)
    print(f"Generated concrete BobLib axle models from {source_yaml}")
    for path in paths:
        print(f"  {path}")


if __name__ == "__main__":
    main()
