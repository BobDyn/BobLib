#!/usr/bin/env python3
"""
Shared helpers for BobSim code generation.

This module intentionally contains the low-level YAML / Modelica rendering
primitives used by the build entrypoints. The actual build scripts live in the
same ``Generation/scripts`` folder.
"""

from __future__ import annotations

import re
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None  # type: ignore[assignment]


GENERATION_DIR = Path(__file__).resolve().parents[1]
REPO_ROOT = GENERATION_DIR.parents[3]

if str(GENERATION_DIR) not in sys.path:
    sys.path.insert(0, str(GENERATION_DIR))

from scripts.modelica_generator import (  # noqa: E402, F401
    ModelicaFileBuilder,
    constructor,
    indent,
    modelica_number,
    modelica_table,
    modelica_value,
    render_constructor,
    render_field_value,
    render_modelica_file,
    render_parameter,
    replace_tokens,
)

from generate_vehicle_model import (  # noqa: E402, F401
    parse_variant as parse_vehicle_variant,
    render_vehicle_model as render_vehicle_model_source,
)


ARCH_TOKENS = {
    "direct": "DWDirect",
    "bellcrank": "DWBC",
    "bellcrank_stabar": "DWBCStabar",
}

AXLE_TYPES = {
    "direct": "Axle.AxleDW_DirectRecord",
    "bellcrank": "Axle.AxleDW_BCRecord",
    "bellcrank_stabar": "Axle.AxleDW_BC_StabarRecord",
}

AXLE_MODEL_TYPES = {
    "direct": {"Fr": "FrAxleDW_Direct", "Rr": "RrAxleDW_Direct"},
    "bellcrank": {"Fr": "FrAxleDW_BC", "Rr": "RrAxleDW_BC"},
    "bellcrank_stabar": {"Fr": "FrAxleDW_BC_Stabar", "Rr": "RrAxleDW_BC_Stabar"},
}

DEFAULT_IMPORTS = [
    "BobLib.Resources.VehicleRecord.Chassis.Suspension.Templates.MassRecord",
]

DEFAULT_ALIASES = {
    "Aero": "BobLib.Resources.VehicleRecord.Aero",
    "TireModel": "BobLib.Resources.VehicleRecord.Chassis.Suspension.Templates.Tire.MF52",
    "Wheel": "BobLib.Resources.VehicleRecord.Chassis.Suspension.Templates.Tire",
    "Rack": "BobLib.Resources.VehicleRecord.Chassis.Suspension.Templates.SteeringRack",
    "Stabar": "BobLib.Resources.VehicleRecord.Chassis.Suspension.Templates.Stabar",
    "DW": "BobLib.Resources.VehicleRecord.Chassis.Suspension.Templates.DoubleWishbone",
    "Axle": "BobLib.Resources.VehicleRecord.Chassis.Suspension",
}

MF52_SECTION_FIELDS = {'fxCombined': ['RBX1', 'RBX2', 'RCX1', 'REX1', 'REX2', 'RHX1'],
 'fxPure': ['LFZO',
            'LGAX',
            'PCX1',
            'PDX1',
            'PDX2',
            'PDX3',
            'PKX1',
            'PKX2',
            'PKX3',
            'PHX1',
            'PHX2',
            'PVX1',
            'PVX2',
            'PEX1',
            'PEX2',
            'PEX3',
            'PEX4',
            'LCX',
            'LMUX',
            'LKX',
            'LHX',
            'LVX',
            'LEX',
            'LXAL'],
 'fyCombined': ['RBY1',
                'RBY2',
                'RBY3',
                'RCY1',
                'REY1',
                'REY2',
                'RHY1',
                'RHY2',
                'RVY1',
                'RVY2',
                'RVY3',
                'RVY4',
                'RVY5',
                'RVY6'],
 'fyPure': ['LFZO',
            'LGAY',
            'PCY1',
            'PDY1',
            'PDY2',
            'PDY3',
            'PKY1',
            'PKY2',
            'PKY3',
            'PHY1',
            'PHY2',
            'PHY3',
            'PVY1',
            'PVY2',
            'PVY3',
            'PVY4',
            'PEY1',
            'PEY2',
            'PEY3',
            'PEY4',
            'LCY',
            'LMUY',
            'LEY',
            'LKY',
            'LHY',
            'LVY',
            'LYKA',
            'LVYKA'],
 'mxCombined': [],
 'mxPure': ['QSX1', 'QSX2', 'QSX3', 'LMX', 'LVMX'],
 'myCombined': [],
 'myPure': ['QSY1', 'QSY2', 'QSY3', 'QSY4', 'Vref', 'LMY'],
 'mzCombined': ['SSZ1',
                'SSZ2',
                'SSZ3',
                'SSZ4',
                'RVY1',
                'RVY2',
                'RVY3',
                'RVY4',
                'RVY5',
                'RVY6',
                'LS',
                'LVYKA'],
 'mzPure': ['QBZ1',
            'QBZ2',
            'QBZ3',
            'QBZ4',
            'QBZ5',
            'QCZ1',
            'QDZ1',
            'QDZ2',
            'QDZ3',
            'QDZ4',
            'QEZ1',
            'QEZ2',
            'QEZ3',
            'QEZ4',
            'QEZ5',
            'QHZ1',
            'QHZ2',
            'QHZ3',
            'QHZ4',
            'QBZ9',
            'QBZ10',
            'QDZ6',
            'QDZ7',
            'QDZ8',
            'QDZ9',
            'LTR',
            'LRES',
            'LKY',
            'LMUY',
            'LGAZ'],
 'setup': ['FNOMIN', 'FZMIN', 'FZMAX', 'UNLOADED_RADIUS']}
TIRE_SECTION_TYPES = {
    "setup": "TireModel.SetupRecord",
    "fxPure": "TireModel.PureSlip.FxPureRecord",
    "fxCombined": "TireModel.CombinedSlip.FxCombinedRecord",
    "fyPure": "TireModel.PureSlip.FyPureRecord",
    "fyCombined": "TireModel.CombinedSlip.FyCombinedRecord",
    "mxPure": "TireModel.PureSlip.MxPureRecord",
    "mxCombined": "TireModel.CombinedSlip.MxCombinedRecord",
    "myPure": "TireModel.PureSlip.MyPureRecord",
    "myCombined": "TireModel.CombinedSlip.MyCombinedRecord",
    "mzPure": "TireModel.PureSlip.MzPureRecord",
    "mzCombined": "TireModel.CombinedSlip.MzCombinedRecord",
}

# Mapping from BobLib MF52 record field names to equivalent .tir keys.
# Example: BobLib MyPureRecord uses Vref, while PAC2002 .tir files
# typically store the reference/measurement speed as LONGVL.
TIR_KEY_ALIASES = {
    "VREF": "LONGVL",
}


@dataclass(frozen=True)
class BuildOutput:
    record_name: str
    variant_name: str
    vehicle_model_name: str
    source_yaml: Path
    record_path: Path
    vehicle_sim_path: Path


def repo_root() -> Path:
    return REPO_ROOT


def vehicle_yaml_path() -> Path:
    return GENERATION_DIR / "vehicle.yml"


def node_value(node: Any, default: Any = None) -> Any:
    if isinstance(node, dict):
        if "Value" in node:
            return node["Value"]
        if "value" in node:
            return node["value"]
        if "Expression" in node:
            return {"expr": str(node["Expression"])}
        if "expr" in node:
            return {"expr": str(node["expr"])}
    return default if node is None else node


def load_yaml(path: Path) -> dict[str, Any]:
    if not path.exists():
        raise FileNotFoundError(
            f"Missing {path}. Run `make sync-vehicle-yaml` from BobSim "
            "or copy the repo-root vehicle.yml into Generation first."
        )
    if yaml is None:  # pragma: no cover
        raise RuntimeError(
            "Missing dependency: PyYAML. Install it with `python -m pip install pyyaml`."
        )
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"{path} must contain a YAML mapping.")
    if data.get("_TemplateEntrypointPlaceholder") is True:
        raise SystemExit(
            "Generation/vehicle.yml is still the placeholder. Run "
            "`make sync-vehicle-yaml` from BobSim or copy a real "
            "vehicle.yml into Generation, then rerun the scripts in "
            "Generation/scripts/."
        )
    return data


def require_mapping(data: dict[str, Any], key: str, yaml_path: Path) -> dict[str, Any]:
    value = data.get(key)
    if not isinstance(value, dict):
        raise ValueError(f"{yaml_path}: expected mapping at {key!r}.")
    return value


def get_path(data: dict[str, Any], keys: list[str], default: Any = None) -> Any:
    cur: Any = data
    for key in keys:
        if not isinstance(cur, dict) or key not in cur:
            return default
        cur = cur[key]
    return node_value(cur, default)


def norm_arch(value: str) -> str:
    v = str(value).strip()
    aliases = {
        "DWDirect": "direct",
        "direct": "direct",
        "DWBC": "bellcrank",
        "bellcrank": "bellcrank",
        "DWBCStabar": "bellcrank_stabar",
        "bellcrank_stabar": "bellcrank_stabar",
        "bellcrank+stabar": "bellcrank_stabar",
    }
    if v not in aliases:
        raise ValueError(f"Unknown architecture {value!r}. Expected one of {sorted(aliases)}.")
    return aliases[v]


def variant_name(data: dict[str, Any]) -> str:
    arch = require_mapping(data, "architecture", vehicle_yaml_path())
    front = norm_arch(str(arch.get("front")))
    rear = norm_arch(str(arch.get("rear")))
    return f"{ARCH_TOKENS[front]}_{ARCH_TOKENS[rear]}"


def record_name_from_yaml(data: dict[str, Any], yaml_path: Path) -> str:
    explicit = get_path(data, ["architecture", "record"])
    if isinstance(explicit, str) and explicit.strip():
        return explicit.strip()
    return f"{variant_name(data)}Record"


def record_to_variant(record_name: str) -> str:
    if not record_name.endswith("Record"):
        raise ValueError(f"Record name must end with `Record`. Got {record_name!r}.")
    return record_name[:-len("Record")]


def vehicle_model_name(data: dict[str, Any], record_name: str) -> str:
    explicit = get_path(data, ["architecture", "vehicle_model"])
    if isinstance(explicit, str) and explicit.strip():
        return explicit.strip()
    return f"Vehicle_{record_to_variant(record_name)}"


def output_record_package(data: dict[str, Any]) -> str:
    return str(get_path(data, ["output", "record_package"], "BobLib.Resources.VehicleDefn"))


def output_sim_package(data: dict[str, Any]) -> str:
    return str(get_path(data, ["output", "sim_package"], "BobLib.Standards"))


def boblib_root(data: dict[str, Any]) -> Path:
    raw = get_path(data, ["paths", "boblib"], "_0_Utils/external/BobLib/BobLib")
    path = Path(str(raw))
    return path if path.is_absolute() else repo_root() / path


def tire_templates_root(data: dict[str, Any]) -> Path:
    raw = get_path(
        data,
        ["paths", "tire_templates"],
        "_0_Utils/external/BobLib/Generation/tire_templates",
    )
    path = Path(str(raw))
    return path if path.is_absolute() else repo_root() / path


def vehicle_templates_root(data: dict[str, Any]) -> Path:
    raw = get_path(
        data,
        ["paths", "vehicle_templates"],
        "_0_Utils/external/BobLib/Generation/vehicle_templates",
    )
    path = Path(str(raw))
    return path if path.is_absolute() else repo_root() / path


def boblib_vehicledefn_dir(data: dict[str, Any]) -> Path:
    return boblib_root(data) / "Resources" / "VehicleDefn"


def boblib_vehicle_suspension_dir(data: dict[str, Any]) -> Path:
    return boblib_root(data) / "Vehicle" / "Chassis" / "Suspension"


def boblib_vehiclesim_path(data: dict[str, Any]) -> Path:
    return boblib_root(data) / "Standards" / "VehicleSim.mo"


def axle_model_name(prefix: str, topology: str) -> str:
    try:
        return AXLE_MODEL_TYPES[topology][prefix]
    except KeyError as exc:
        raise ValueError(f"Unknown axle model mapping for prefix={prefix!r}, topology={topology!r}") from exc


def boblib_axle_model_path(data: dict[str, Any], prefix: str, topology: str) -> Path:
    return boblib_vehicle_suspension_dir(data) / f"{axle_model_name(prefix, topology)}.mo"


def parse_tir(path: Path) -> dict[str, float | str]:
    params: dict[str, float | str] = {}
    if not path.exists():
        return params
    line_re = re.compile(r"^\s*([A-Za-z_][A-Za-z0-9_]*)\s*=\s*(.*?)\s*(?:\$.*)?$")
    for line in path.read_text(encoding="utf-8", errors="ignore").splitlines():
        match = line_re.match(line)
        if not match:
            continue
        key = match.group(1).upper()
        raw = match.group(2).strip().strip("'").strip('"')
        try:
            params[key] = float(raw)
        except ValueError:
            params[key] = raw
    return params


def tire_template_name(data: dict[str, Any], side: dict[str, Any]) -> str:
    explicit = get_path(side, ["tire", "template"])
    if isinstance(explicit, str) and explicit.strip():
        return explicit.strip()
    default = get_path(data, ["defaults", "tire_template"])
    if isinstance(default, str) and default.strip():
        return default.strip()
    raise ValueError("Missing tire template. Set defaults.tire_template or <side>.tire.template.")


def build_mf52_fields(data: dict[str, Any], side: dict[str, Any], unloaded_radius_expr: str) -> dict[str, Any]:
    """Build BobLib MF52 fields from a local .tir file.

    Vehicle YAML intentionally does not carry tire-model coefficients. The .tir
    is the source of truth for tire coefficients; missing tire files or missing
    required coefficients fail fast so the generated record is not silently
    populated with hidden defaults.
    """
    template = tire_template_name(data, side)
    tir_path = tire_templates_root(data) / f"{template}.tir"

    if not tir_path.exists():
        raise FileNotFoundError(
            f"Missing tire template: {tir_path}\n"
            "Place the .tir file under paths.tire_templates, or update "
            "<side>.tire.template / defaults.tire_template in vehicle.yml."
        )

    tir = parse_tir(tir_path)
    if not tir:
        raise ValueError(f"Tire template {tir_path} did not contain parseable key=value coefficients.")

    out: dict[str, Any] = {}
    missing: list[str] = []

    for section, keys in MF52_SECTION_FIELDS.items():
        fields: dict[str, Any] = {}
        for key in keys:
            if key == "UNLOADED_RADIUS":
                fields[key] = {"expr": unloaded_radius_expr}
                continue
            tir_key = key.upper()
            lookup_key = TIR_KEY_ALIASES.get(tir_key, tir_key)
            if lookup_key not in tir:
                if lookup_key != tir_key:
                    missing.append(f"{tir_key}({lookup_key})")
                else:
                    missing.append(tir_key)
                continue
            fields[key] = tir[lookup_key]
        out[section] = constructor(TIRE_SECTION_TYPES[section], fields)

    if missing:
        preview = ", ".join(missing[:25])
        extra = "" if len(missing) <= 25 else f", ... ({len(missing)} total)"
        raise ValueError(
            f"Tire template {tir_path} is missing required MF52 coefficients: "
            f"{preview}{extra}\n"
            "Known aliases are supported for naming differences, e.g. "
            "BobLib Vref <- .tir LONGVL."
        )

    return out


def require_side(data: dict[str, Any], side_name: str) -> dict[str, Any]:
    side = data.get(side_name)
    if not isinstance(side, dict):
        raise ValueError(f"vehicle.yml: expected mapping for {side_name!r}.")
    return side


def require_section(parent: dict[str, Any], path: str, section: str) -> dict[str, Any]:
    value = parent.get(section)
    if not isinstance(value, dict):
        raise ValueError(f"vehicle.yml: expected {path}.{section} mapping.")
    return value


def require_key(section: dict[str, Any], path: str, key: str) -> Any:
    if key not in section:
        raise ValueError(f"vehicle.yml: missing {path}.{key}")
    return node_value(section[key])


def pickup_indices(order: list[str], required: list[str]) -> dict[str, int]:
    normalized = [str(item).strip().lower() for item in order]
    if sorted(normalized) != sorted(required):
        raise ValueError(f"pickup order must contain exactly {required}, got {order}")
    return {name: normalized.index(name) + 1 for name in required}


def spring_table(rate: Any) -> dict[str, Any]:
    value = node_value(rate)
    if isinstance(value, dict) and "table" in value:
        return {"table": value["table"]}
    return {"table": [[0, 0], [1, value]]}


def damper_table(rate: Any) -> dict[str, Any]:
    value = node_value(rate)
    if isinstance(value, dict) and "table" in value:
        return {"table": value["table"]}
    return {"table": [[-1, -float(value)], [0, 0], [1, value]]}


def mass_record(value: dict[str, Any]) -> dict[str, Any]:
    return constructor("MassRecord", {
        "m": require_key(value, "mass", "mass_kg"),
        "rCM": require_key(value, "mass", "cg_m"),
        "inertia": require_key(value, "mass", "inertia_kg_m2"),
    })


def zero_mass_record() -> dict[str, Any]:
    return constructor("MassRecord", {
        "m": 0,
        "rCM": [0, 0, 0],
        "inertia": [[0, 0, 0], [0, 0, 0], [0, 0, 0]],
    })


def side_parameters(
    data: dict[str, Any],
    side_name: str,
    prefix: str,
    topology: str,
) -> list[tuple[str, str, dict[str, Any]]]:
    side = require_side(data, side_name)
    wheel = require_section(side, side_name, "wheel")
    tire = require_section(side, side_name, "tire")
    suspension = require_section(side, side_name, "suspension")
    steering = require_section(side, side_name, "steering")
    actuation = require_section(side, side_name, "actuation")
    shock = require_section(actuation, f"{side_name}.actuation", "shock")
    masses = require_section(side, side_name, "masses")

    params: list[tuple[str, str, dict[str, Any]]] = []

    axle_fields: dict[str, Any] = {
        "rodToLower": str(require_key(actuation, f"{side_name}.actuation", "rod_to")).lower() == "lower",
        "rodMount": require_key(actuation, f"{side_name}.actuation", "rod_mount_m"),
        "shockMount": require_key(shock, f"{side_name}.actuation.shock", "mount_m"),
        "springTable": spring_table(
            shock.get("spring_table")
            if "spring_table" in shock
            else require_key(shock, f"{side_name}.actuation.shock", "spring_rate_n_per_m")
        ),
        "springFreeLength": require_key(shock, f"{side_name}.actuation.shock", "free_length_m"),
        "damperTable": damper_table(
            shock.get("damper_table")
            if "damper_table" in shock
            else require_key(shock, f"{side_name}.actuation.shock", "damper_rate_n_s_per_m")
        ),
    }

    if topology in {"bellcrank", "bellcrank_stabar"}:
        bellcrank = require_section(actuation, f"{side_name}.actuation", "bellcrank")
        pickups = require_section(bellcrank, f"{side_name}.actuation.bellcrank", "pickups_m")
        required = ["rod", "shock"] + (["stabar"] if topology == "bellcrank_stabar" else [])
        indices = pickup_indices(require_key(bellcrank, f"{side_name}.actuation.bellcrank", "order"), required)
        axle_fields = {
            "bellcrankPivot": require_key(bellcrank, f"{side_name}.actuation.bellcrank", "pivot_m"),
            "bellcrankPivotAxis": require_key(bellcrank, f"{side_name}.actuation.bellcrank", "axis"),
            "bellcrankRodPickup": require_key(pickups, f"{side_name}.actuation.bellcrank.pickups_m", "rod"),
            "bellcrankShockPickup": require_key(pickups, f"{side_name}.actuation.bellcrank.pickups_m", "shock"),
            **(
                {"bellcrankStabarPickup": require_key(pickups, f"{side_name}.actuation.bellcrank.pickups_m", "stabar")}
                if topology == "bellcrank_stabar"
                else {}
            ),
            "rodPickup": indices["rod"],
            "shockPickup": indices["shock"],
            **({"stabarPickup": indices["stabar"]} if topology == "bellcrank_stabar" else {}),
            **axle_fields,
        }

    params.append((f"p{prefix}AxleDW", AXLE_TYPES[topology], axle_fields))

    if topology == "bellcrank_stabar":
        stabar = require_section(actuation, f"{side_name}.actuation", "stabar")
        params.append((f"p{prefix}Stabar", "Stabar.StabarRecord", {
            "leftArmEnd": require_key(stabar, f"{side_name}.actuation.stabar", "arm_end_m"),
            "leftBarEnd": require_key(stabar, f"{side_name}.actuation.stabar", "bar_end_m"),
            "barRate": require_key(stabar, f"{side_name}.actuation.stabar", "rate_n_m_per_rad"),
        }))

    radius = require_key(wheel, f"{side_name}.wheel", "radius_m")
    rim_radius_ratio = require_key(wheel, f"{side_name}.wheel", "rim_radius_ratio")
    rim_width_ratio = require_key(wheel, f"{side_name}.wheel", "rim_width_ratio")
    rim_width_expr = (
        f"{modelica_value(radius)}*{modelica_value(rim_radius_ratio)}*"
        f"{modelica_value(rim_width_ratio)}"
    )
    params.append((f"p{prefix}PartialWheel", "Wheel.Templates.PartialWheelRecord", {
        "R0": radius,
        "rimR0": {"expr": f"{modelica_value(radius)}*{modelica_value(rim_radius_ratio)}"},
        "rimWidth": {"expr": rim_width_expr},
        "staticAlpha": require_key(wheel, f"{side_name}.wheel", "toe_deg"),
        "staticGamma": require_key(wheel, f"{side_name}.wheel", "camber_deg"),
    }))

    params.append((f"p{prefix}Rack", "Rack.RackAndPinionRecord", {
        "leftPickup": require_key(steering, f"{side_name}.steering", "rack_pickup_m"),
        "cFactor": require_key(steering, f"{side_name}.steering", "rack_travel_per_rev_m"),
    }))

    params.append((f"p{prefix}DW", "DW.WishboneUprightLoopRecord", {
        "upperFore_i": require_key(suspension, f"{side_name}.suspension", "upper_fore_i_m"),
        "upperAft_i": require_key(suspension, f"{side_name}.suspension", "upper_aft_i_m"),
        "lowerFore_i": require_key(suspension, f"{side_name}.suspension", "lower_fore_i_m"),
        "lowerAft_i": require_key(suspension, f"{side_name}.suspension", "lower_aft_i_m"),
        "upper_o": require_key(suspension, f"{side_name}.suspension", "upper_o_m"),
        "lower_o": require_key(suspension, f"{side_name}.suspension", "lower_o_m"),
        "tie_o": require_key(suspension, f"{side_name}.suspension", "tie_o_m"),
        "wheelCenter": require_key(suspension, f"{side_name}.suspension", "wheel_center_m"),
    }))

    params.append((f"p{prefix}AxleMass", "Axle.Templates.AxleMassRecord", {
        "unsprungMass": mass_record(require_section(masses, f"{side_name}.masses", "unsprung")),
        "ucaMass": mass_record(require_section(masses, f"{side_name}.masses", "upper_control_arm")),
        "lcaMass": mass_record(require_section(masses, f"{side_name}.masses", "lower_control_arm")),
        "tieMass": mass_record(require_section(masses, f"{side_name}.masses", "tie_rod")),
    }))

    params.append((f"p{prefix}Tire1DOF_YParams", "Wheel.Wheel1DOF_YRecord", {
        "wheelJ": require_key(tire, f"{side_name}.tire", "wheel_inertia_kg_m2"),
    }))
    params.append((f"p{prefix}Tire1DOF_ZParams", "Wheel.Wheel1DOF_ZRecord", {
        "wheelC": require_key(tire, f"{side_name}.tire", "vertical_stiffness_n_per_m"),
        "wheelD": require_key(tire, f"{side_name}.tire", "vertical_damping_n_s_per_m"),
    }))
    params.append((
        f"p{prefix}TireModel",
        "TireModel.MF52Record",
        build_mf52_fields(data, side, f"p{prefix}PartialWheel.R0"),
    ))
    return params


def parameter_sections(data: dict[str, Any]) -> list[tuple[str, str, dict[str, Any]]]:
    arch = require_mapping(data, "architecture", vehicle_yaml_path())
    front_topology = norm_arch(str(arch.get("front")))
    rear_topology = norm_arch(str(arch.get("rear")))
    params = []
    params.extend(side_parameters(data, "front", "Fr", front_topology))
    params.extend(side_parameters(data, "rear", "Rr", rear_topology))

    sprung = require_mapping(data, "sprung_mass", vehicle_yaml_path())
    params.append(("pBaseSprungMass", "MassRecord", {
        "m": require_key(sprung, "sprung_mass", "mass_kg"),
        "rCM": require_key(sprung, "sprung_mass", "cg_m"),
        "inertia": require_key(sprung, "sprung_mass", "inertia_kg_m2"),
    }))

    driver = data.get("driver_mass")
    if isinstance(driver, dict):
        driver_fields = {
            "m": require_key(driver, "driver_mass", "mass_kg"),
            "rCM": require_key(driver, "driver_mass", "cg_m"),
            "inertia": require_key(driver, "driver_mass", "inertia_kg_m2"),
        }
    else:
        driver_fields = zero_mass_record()
    params.append(("pDriverMass", "MassRecord", driver_fields))
    body = require_mapping(data, "body", vehicle_yaml_path())
    params.append(("pTorsionalStiff", "Modelica.SIunits.RotationalSpringConstant", {
        "_binding": require_key(body, "body", "torsional_stiff_n_m_per_rad"),
    }))
    params.append(("pSprungMass", "MassRecord", {
        "_binding": "BobLib.Utilities.Mechanics.combineMassRecords({pBaseSprungMass, pDriverMass})",
    }))

    aero = data.get("aero")
    if isinstance(aero, dict):
        params.append(("pAero", "Aero.CFDAeroMapRecord", {
            "referenceSpeed": require_key(aero, "aero", "reference_speed_m_per_s"),
            "aeroRef": require_key(aero, "aero", "aero_ref_m"),
            "FL_RideHeightRef": require_key(aero, "aero", "front_left_ride_height_ref_m"),
            "RL_RideHeightRef": require_key(aero, "aero", "rear_left_ride_height_ref_m"),
            "frontRideHeightGrid": require_key(aero, "aero", "front_ride_height_grid_m"),
            "rearRideHeightGrid": require_key(aero, "aero", "rear_ride_height_grid_m"),
            "dragTable": require_key(aero, "aero", "drag_table_n"),
            "downforceTable": require_key(aero, "aero", "downforce_table_n"),
            "mxTable": require_key(aero, "aero", "mx_table_nm"),
            "myTable": require_key(aero, "aero", "my_table_nm"),
            "mzTable": require_key(aero, "aero", "mz_table_nm"),
        }))
    return params


def write_text_file(path: Path, text: str, overwrite: bool) -> None:
    if path.exists() and not overwrite:
        raise FileExistsError(f"{path} already exists. Set overwrite=True to replace it.")
    path.write_text(text, encoding="utf-8")


def remove_matching_files(directory: Path, patterns: list[str], keep_names: set[str]) -> list[Path]:
    removed: list[Path] = []
    if not directory.exists():
        return removed
    for pattern in patterns:
        for path in directory.glob(pattern):
            if path.name in keep_names:
                continue
            if path.is_file():
                path.unlink()
                removed.append(path)
    return removed


def write_package_order(path: Path, entries: list[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    write_text_file(path, "\n".join(entries) + "\n", overwrite=True)


def active_vehicle_variant(data: dict[str, Any], yaml_path: Path | None = None):
    yaml_path = yaml_path or vehicle_yaml_path()
    arch = require_mapping(data, "architecture", yaml_path)
    front = norm_arch(str(arch.get("front")))
    rear = norm_arch(str(arch.get("rear")))
    return parse_vehicle_variant(front, rear)


def render_active_vehicle_model(data: dict[str, Any], yaml_path: Path | None = None) -> str:
    return render_vehicle_model_source(active_vehicle_variant(data, yaml_path))


def prune_vehicle_models(data: dict[str, Any], yaml_path: Path | None = None) -> list[Path]:
    yaml_path = yaml_path or vehicle_yaml_path()
    active_record = record_name_from_yaml(data, yaml_path)
    active_vehicle_model = vehicle_model_name(data, active_record)
    vehicle_dir = boblib_root(data) / "Vehicle"
    removed = remove_matching_files(
        vehicle_dir,
        ["Vehicle_*.mo"],
        {f"{active_vehicle_model}.mo"},
    )
    write_package_order(
        vehicle_dir / "package.order",
        [
            "Aero",
            "Chassis",
            "Powertrain",
            "Electronics",
            "VehicleBase",
            active_vehicle_model,
        ],
    )
    return removed


def prune_standards_models(data: dict[str, Any], yaml_path: Path | None = None) -> list[Path]:
    yaml_path = yaml_path or vehicle_yaml_path()
    standards_dir = boblib_root(data) / "Standards"
    removed = remove_matching_files(
        standards_dir,
        ["VehicleFMI.mo", "FourPostEval.mo"],
        {"VehicleSim.mo", "FourPostSim.mo"},
    )
    removed.extend(
        remove_matching_files(
            standards_dir / "StandardSim",
            ["*.mo", "package.mo", "package.order"],
            set(),
        )
    )
    write_package_order(
        standards_dir / "package.order",
        [
            "VehicleSim",
            "FourPostSim",
        ],
    )
    return removed


def prune_axle_models(data: dict[str, Any], yaml_path: Path | None = None) -> list[Path]:
    yaml_path = yaml_path or vehicle_yaml_path()
    arch = require_mapping(data, "architecture", yaml_path)
    front_topology = norm_arch(str(arch.get("front")))
    rear_topology = norm_arch(str(arch.get("rear")))
    active_front = axle_model_name("Fr", front_topology)
    active_rear = axle_model_name("Rr", rear_topology)
    suspension_dir = boblib_vehicle_suspension_dir(data)
    removed = []
    removed.extend(
        remove_matching_files(
            suspension_dir,
            ["FrAxleDW_*.mo"],
            {f"{active_front}.mo"},
        )
    )
    removed.extend(
        remove_matching_files(
            suspension_dir,
            ["RrAxleDW_*.mo"],
            {f"{active_rear}.mo"},
        )
    )
    write_package_order(
        suspension_dir / "package.order",
        [
            "Templates",
            "Linkages",
            "AxleDWBase",
            active_front,
            active_rear,
        ],
    )
    return removed


def prune_vehicle_records(data: dict[str, Any], yaml_path: Path | None = None) -> list[Path]:
    yaml_path = yaml_path or vehicle_yaml_path()
    active_record = record_name_from_yaml(data, yaml_path)
    records_dir = boblib_vehicledefn_dir(data)
    removed = remove_matching_files(
        records_dir,
        ["*Record.mo"],
        {f"{active_record}.mo"},
    )
    write_package_order(
        records_dir / "package.order",
        [
            active_record,
        ],
    )
    return removed
