#!/usr/bin/env python3
from __future__ import annotations

import re
from pathlib import Path

from build_axle_models import build_axle_models
from build_records import build_four_post_eval_record, build_vehicle_record
from build_common import (
    boblib_root,
    load_yaml,
    norm_arch,
    modelica_table,
    prune_vehicle_models,
    prune_standards_models,
    render_parameter,
    render_active_vehicle_model,
    require_mapping,
    record_name_from_yaml,
    side_parameters,
    vehicle_model_name,
    vehicle_yaml_path,
    write_text_file,
)


FOUR_POST_AXLE_MODEL_TYPES = {
    "direct": {"Fr": "FrAxleDW_Direct", "Rr": "RrAxleDW_Direct"},
    "bellcrank": {"Fr": "FrAxleDW_BC", "Rr": "RrAxleDW_BC"},
    "bellcrank_stabar": {"Fr": "FrAxleDW_BC_Stabar", "Rr": "RrAxleDW_BC_Stabar"},
}

FOUR_POST_AXLE_RECORD_TYPES = {
    "direct": "AxleDW_DirectRecord",
    "bellcrank": "AxleDW_BCRecord",
    "bellcrank_stabar": "AxleDW_BC_StabarRecord",
}

FOUR_POST_AXLE_PARAMETER_RE = re.compile(
    r"  parameter [A-Za-z0-9_.]+ fr_pAxle\(.*?"
    r"(?=  parameter [A-Za-z0-9_.]+ rr_pAxle\()",
    re.S,
)

FOUR_POST_FRONT_AXLE_RE = re.compile(
    r"  // Front axle\n.*?(?=  // Rear axle\n)",
    re.S,
)

FOUR_POST_REAR_AXLE_RE = re.compile(
    r"  // Rear axle\n.*?(?=  // Front chassis actuator\n)",
    re.S,
)

FOUR_POST_IMPORT_RE = re.compile(
    r"  // FourPostEval record\n.*?  // Vehicle record\n"
    r"  import BobLib\.Resources\.VehicleDefn\.[A-Za-z0-9_]+Record;\n",
    re.S,
)

FOUR_POST_TIMING_RE = re.compile(
    r"  // Time-value tables \[time, normalized value\]\n.*?"
    r"  inner Modelica\.Mechanics\.MultiBody\.World world",
    re.S,
)

# FourPost uses a 5 s pose cadence; the load pulse is short and the tail is left to settle.
FOUR_POST_POSE_STEP_S = 5
FOUR_POST_HEAVE_START_S = 2
FOUR_POST_HEAVE_MEASURED_COUNT = 11
FOUR_POST_HEAVE_BUFFER_START_S = FOUR_POST_HEAVE_START_S + FOUR_POST_POSE_STEP_S * FOUR_POST_HEAVE_MEASURED_COUNT
FOUR_POST_HEAVE_END_S = FOUR_POST_HEAVE_BUFFER_START_S + FOUR_POST_POSE_STEP_S
FOUR_POST_ROLL_START_S = FOUR_POST_HEAVE_END_S + 1
FOUR_POST_ROLL_COUNT = 11
FOUR_POST_ROLL_END_S = FOUR_POST_ROLL_START_S + FOUR_POST_POSE_STEP_S * FOUR_POST_ROLL_COUNT
FOUR_POST_STOP_TIME_S = FOUR_POST_ROLL_END_S


def _indent_block(text: str, indent: int = 2) -> str:
    prefix = " " * indent
    return "\n".join(prefix + line if line else line for line in text.splitlines())


def _topology_model_name(prefix: str, topology: str) -> str:
    return FOUR_POST_AXLE_MODEL_TYPES[topology][prefix]


def _topology_record_type(topology: str) -> str:
    return FOUR_POST_AXLE_RECORD_TYPES[topology]


def _side_axle_field_names(
    data: dict[str, object],
    side_name: str,
    prefix: str,
    topology: str,
) -> list[str]:
    axle_fields = side_parameters(data, side_name, prefix, topology)[0][2]
    return list(axle_fields.keys())


def _render_import_block(
    *,
    front_topology: str,
    rear_topology: str,
    vehicle_record_name: str,
) -> str:
    lines = [
        "  // FourPostEval record",
        "  import BobLib.Resources.StandardRecord.FourPostEvalRecord;",
        "",
        "  // Suspension axle models",
        f"  import BobLib.Vehicle.Chassis.Suspension.{_topology_model_name('Fr', front_topology)};",
        f"  import BobLib.Vehicle.Chassis.Suspension.{_topology_model_name('Rr', rear_topology)};",
        "",
        "  // Axle record types",
        "  import BobLib.Resources.VehicleRecord.Chassis.Suspension.AxleDW_DirectRecord;",
        "  import BobLib.Resources.VehicleRecord.Chassis.Suspension.AxleDW_BCRecord;",
        "  import BobLib.Resources.VehicleRecord.Chassis.Suspension.AxleDW_BC_StabarRecord;",
        "",
        "  // Bar record to override rate",
        "  import BobLib.Resources.VehicleRecord.Chassis.Suspension.Templates.Stabar.StabarRecord;",
        "",
        "  // Tire record",
        "  import BobLib.Vehicle.Chassis.Suspension.Templates.Tire;",
        "",
        "  // Vehicle record",
        f"  import BobLib.Resources.VehicleDefn.{vehicle_record_name};",
    ]
    return "\n".join(lines)


def _render_timing_block() -> str:
    heave_values = [round(-1.0 + 0.2 * i, 1) for i in range(FOUR_POST_HEAVE_MEASURED_COUNT)]
    roll_values = [round(-1.0 + 0.2 * i, 1) for i in range(FOUR_POST_ROLL_COUNT)]

    def heave_value_at(t: int) -> float:
        if t < FOUR_POST_HEAVE_START_S:
            return 0.0
        if t < FOUR_POST_HEAVE_BUFFER_START_S:
            return heave_values[(t - FOUR_POST_HEAVE_START_S) // FOUR_POST_POSE_STEP_S]
        if t < FOUR_POST_HEAVE_END_S:
            return 0.0
        return 0.0

    def roll_value_at(t: int) -> float:
        if t < FOUR_POST_ROLL_START_S:
            return 0.0
        if t < FOUR_POST_ROLL_END_S:
            return roll_values[(t - FOUR_POST_ROLL_START_S) // FOUR_POST_POSE_STEP_S]
        return 0.0

    def pulse_value_at(t: int, start_s: int, count: int) -> float:
        if t < start_s or t >= start_s + FOUR_POST_POSE_STEP_S * count:
            return 0.0
        local = t - (start_s + ((t - start_s) // FOUR_POST_POSE_STEP_S) * FOUR_POST_POSE_STEP_S)
        return 1.0 if local == 1 else 0.0

    heave_rows = [[t, heave_value_at(t)] for t in range(0, FOUR_POST_HEAVE_END_S + 1)]
    roll_rows = [[t, roll_value_at(t)] for t in range(0, FOUR_POST_ROLL_END_S + 1)]
    fx_rows = [
        [t, pulse_value_at(t, FOUR_POST_HEAVE_START_S, FOUR_POST_HEAVE_MEASURED_COUNT)]
        for t in range(0, FOUR_POST_ROLL_END_S + 1)
    ]
    fy_rows = [
        [t, pulse_value_at(t, FOUR_POST_ROLL_START_S, FOUR_POST_ROLL_COUNT)]
        for t in range(0, FOUR_POST_ROLL_END_S + 1)
    ]

    lines = [
        "  // Time-value tables [time, normalized value]",
        f"  final parameter Real rollTable[:, 2] = {modelica_table(roll_rows)};",
        f"  final parameter Real heaveTable[:, 2] = {modelica_table(heave_rows)};",
        f"  final parameter Real fxTable[:, 2] = {modelica_table(fx_rows)};",
        f"  final parameter Real fyTable[:, 2] = {modelica_table(fy_rows)};",
        "",
        "  inner Modelica.Mechanics.MultiBody.World world",
    ]
    return "\n".join(lines)


def _render_axle_parameter_block(
    data: dict[str, object],
    side_name: str,
    prefix: str,
    topology: str,
) -> str:
    field_names = _side_axle_field_names(data, side_name, prefix, topology)
    if "rodToLower" not in field_names:
        field_names.insert(0, "rodToLower")
    if "rodMount" not in field_names:
        field_names.insert(1, "rodMount")
    fields = {
        field_name: {"expr": f"pVehicle.p{prefix}AxleDW.{field_name}"}
        for field_name in field_names
    }
    return _indent_block(
        render_parameter(
            f"{prefix.lower()}_pAxle",
            _topology_record_type(topology),
            fields,
        )
    )


def _render_stabar_parameter_block(prefix: str) -> str:
    return _indent_block(
        render_parameter(
            f"p{prefix}Stabar",
            "StabarRecord",
            {
                "leftArmEnd": {"expr": f"pVehicle.p{prefix}Stabar.leftArmEnd"},
                "leftBarEnd": {"expr": f"pVehicle.p{prefix}Stabar.leftBarEnd"},
                "barRate": 0,
            },
        )
    )


def _render_tire_redeclares(prefix: str) -> str:
    wheel_ref = f"pVehicle.p{prefix}PartialWheel"
    return (
        "                              redeclare Tire.BaseTire leftTire("
        f"pPartialWheel = {wheel_ref},\n"
        "                                                               redeclare Tire.TirePhysics.Wheel0DOF "
        f"wheelModel(partialWheelParams = {wheel_ref}),\n"
        "                                                               redeclare Tire.MF52.SlipModel.NoSlip "
        "slipModel),\n"
        "                              redeclare Tire.BaseTire rightTire("
        f"pPartialWheel = {wheel_ref},\n"
        "                                                               redeclare Tire.TirePhysics.Wheel0DOF "
        f"wheelModel(partialWheelParams = {wheel_ref}),\n"
        "                                                               redeclare Tire.MF52.SlipModel.NoSlip "
        "slipModel))"
    )


def _render_axle_instance_block(
    side_name: str,
    prefix: str,
    topology: str,
) -> str:
    axle_model = _topology_model_name(prefix, topology)
    axle_name = f"{prefix.lower()}AxleDW"
    axle_ref = f"{prefix.lower()}_pAxle"
    p_stabar = ""
    if topology == "bellcrank_stabar":
        p_stabar = (
            f"                              pStabar = StabarRecord(leftBarEnd = pVehicle.p{prefix}Stabar.leftBarEnd,\n"
            f"                                                     leftArmEnd = pVehicle.p{prefix}Stabar.leftArmEnd,\n"
            f"                                                     barRate = 0),\n"
        )

    placement = (
        "    Placement(transformation(origin = {0.25, 52.4444}, extent = "
        "{{-37.25, -16.5556}, {37.25, 16.5556}})));"
        if side_name == "front"
        else (
            "    Placement(transformation(origin = {-0.285728, -49.8887}, extent = "
            "{{-36.5715, -14.2222}, {36.5715, 14.2222}})));"
        )
    )

    return (
        f"  // {side_name.title()} axle\n"
        f"  {axle_model} {axle_name}(pAxle = {axle_ref},\n"
        f"                              pRack = pVehicle.p{prefix}Rack,\n"
        f"                              pLeftPartialWheel = pVehicle.p{prefix}PartialWheel,\n"
        f"                              pLeftDW = pVehicle.p{prefix}DW,\n"
        f"                              pLeftAxleMass = pVehicle.p{prefix}AxleMass,\n"
        f"{p_stabar}"
        f"{_render_tire_redeclares(prefix)}"
        f" annotation(\n"
        f"{placement}\n"
    )


def render_four_post_sim(data: dict[str, object], yaml_path: Path) -> str:
    source_path = boblib_root(data) / "Standards" / "FourPostSim.mo"
    text = source_path.read_text(encoding="utf-8")
    text = FOUR_POST_TIMING_RE.sub(_render_timing_block(), text, count=1)
    text = re.sub(
        r"experiment\(StartTime = 0, StopTime = [0-9.]+,",
        f"experiment(StartTime = 0, StopTime = {FOUR_POST_STOP_TIME_S},",
        text,
        count=1,
    )
    arch = require_mapping(data, "architecture", yaml_path)  # type: ignore[arg-type]
    front_topology = norm_arch(str(arch.get("front")))
    rear_topology = norm_arch(str(arch.get("rear")))

    active_vehicle_record = record_name_from_yaml(data, yaml_path)
    text = FOUR_POST_IMPORT_RE.sub(
        _render_import_block(
            front_topology=front_topology,
            rear_topology=rear_topology,
            vehicle_record_name=active_vehicle_record,
        )
        + "\n",
        text,
        count=1,
    )
    text = re.sub(
        r"  parameter [A-Za-z0-9_]+ pVehicle;",
        f"  parameter {active_vehicle_record} pVehicle;",
        text,
        count=1,
    )

    front_param = _render_axle_parameter_block(data, "front", "Fr", front_topology)
    if front_topology == "bellcrank_stabar":
        front_param += "\n\n" + _render_stabar_parameter_block("Fr")
    else:
        front_param += "\n"
    front_param += "\n\n"

    rear_param = _render_axle_parameter_block(data, "rear", "Rr", rear_topology)
    if rear_topology == "bellcrank_stabar":
        rear_param += "\n\n" + _render_stabar_parameter_block("Rr")
    else:
        rear_param += "\n"
    rear_param += "\n\n"

    front_marker = re.compile(r"  parameter [A-Za-z0-9_.]+ fr_pAxle\(")
    rear_marker = re.compile(r"  parameter [A-Za-z0-9_.]+ rr_pAxle\(")
    front_axle_marker = "  // Front axle\n"
    front_match = front_marker.search(text)
    rear_match = rear_marker.search(text)
    front_start = -1 if front_match is None else front_match.start()
    rear_start = -1 if rear_match is None else rear_match.start()
    front_axle_start = text.find(front_axle_marker)
    if (
        front_start == -1
        or rear_start == -1
        or front_axle_start == -1
        or not front_start < rear_start < front_axle_start
    ):
        raise ValueError("Failed to locate FourPostSim axle parameter blocks.")
    text = text[:front_start] + front_param + rear_param + text[front_axle_start:]

    text = FOUR_POST_FRONT_AXLE_RE.sub(
        _render_axle_instance_block("front", "Fr", front_topology),
        text,
        count=1,
    )
    text = FOUR_POST_REAR_AXLE_RE.sub(
        _render_axle_instance_block("rear", "Rr", rear_topology),
        text,
        count=1,
    )

    text = re.sub(
        r"  frKnC\.stabarAngle = .*?;",
        "  frKnC.stabarAngle = "
        + ("frAxleDW.stabar.spring.phi_rel;" if front_topology == "bellcrank_stabar" else "0;"),
        text,
        count=1,
    )
    text = re.sub(
        r"  rrKnC\.stabarAngle = .*?;",
        "  rrKnC.stabarAngle = "
        + ("rrAxleDW.stabar.spring.phi_rel;" if rear_topology == "bellcrank_stabar" else "0;"),
        text,
        count=1,
    )
    text = text.replace("  KnCRecord frKnC;", "  FourPostEvalRecord frKnC;")
    text = text.replace("  KnCRecord rrKnC;", "  FourPostEvalRecord rrKnC;")
    text = text.replace("  // KnC records", "  // FourPostEval records")

    return text


def build_four_post_sim(
    *,
    source_yaml: Path | None = None,
    overwrite: bool = True,
) -> tuple[Path, Path]:
    source_yaml = source_yaml or vehicle_yaml_path()
    build_vehicle_record(source_yaml=source_yaml, overwrite=overwrite)
    record_path = build_four_post_eval_record(overwrite=overwrite)
    build_axle_models(source_yaml=source_yaml, overwrite=overwrite)
    data = load_yaml(source_yaml)
    record_name = record_name_from_yaml(data, source_yaml)
    vehicle_model = vehicle_model_name(data, record_name)
    vehicle_model_path = boblib_root(data) / "Vehicle" / f"{vehicle_model}.mo"
    vehicle_model_path.parent.mkdir(parents=True, exist_ok=True)
    write_text_file(vehicle_model_path, render_active_vehicle_model(data, source_yaml), overwrite)  # type: ignore[arg-type]
    prune_vehicle_models(data, source_yaml)
    prune_standards_models(data, source_yaml)

    model_path = boblib_root(data) / "Standards" / "FourPostSim.mo"
    model_path.parent.mkdir(parents=True, exist_ok=True)
    write_text_file(model_path, render_four_post_sim(data, source_yaml), overwrite)
    return record_path, model_path


def main() -> None:
    source_yaml = vehicle_yaml_path()
    record_path, model_path = build_four_post_sim(source_yaml=source_yaml, overwrite=True)
    print(f"Generated BobLib FourPostSim from {source_yaml}")
    print(f"  FourPostEval record: {record_path}")
    print(f"  FourPostSim model:   {model_path}")


if __name__ == "__main__":
    main()
