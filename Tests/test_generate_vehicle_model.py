from __future__ import annotations

from pathlib import Path
import re
import sys

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
GENERATION_DIR = REPO_ROOT / "Generation"
GENERATION_SCRIPTS_DIR = GENERATION_DIR / "scripts"
for path in (GENERATION_SCRIPTS_DIR, GENERATION_DIR):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from build_four_post_sim import (  # noqa: E402
    FOUR_POST_HEAVE_MEASURED_COUNT,
    FOUR_POST_HEAVE_START_S,
    FOUR_POST_POSE_STEP_S,
    FOUR_POST_STOP_TIME_S,
    _render_timing_block,
)
from generate_vehicle_model import (  # noqa: E402
    TOPOLOGIES,
    generate_one,
    parse_variant,
    render_vehicle_model,
    render_vehicle_sim,
)


def _table_rows(block: str, table_name: str) -> list[tuple[int, float]]:
    match = re.search(rf"{table_name}\[:, 2\] = \[(.*?)\];", block, re.S)
    assert match is not None

    rows: list[tuple[int, float]] = []
    for raw_row in match.group(1).split(";"):
        time_text, value_text = raw_row.split(",")
        rows.append((int(float(time_text.strip())), float(value_text.strip())))
    return rows


def test_variant_names_match_existing_convention() -> None:
    variant = parse_variant("bellcrank_stabar", "bellcrank_stabar")

    assert variant.variant_name == "DWBCStabar_DWBCStabar"
    assert variant.record_name == "DWBCStabar_DWBCStabarRecord"
    assert variant.vehicle_model_name == "Vehicle_DWBCStabar_DWBCStabar"


def test_direct_direct_names_match_existing_convention() -> None:
    variant = parse_variant("direct", "direct")

    assert variant.variant_name == "DWDirect_DWDirect"
    assert variant.record_name == "DWDirect_DWDirectRecord"
    assert variant.vehicle_model_name == "Vehicle_DWDirect_DWDirect"


def test_mixed_names_match_existing_convention() -> None:
    variant = parse_variant("bellcrank", "bellcrank_stabar")

    assert variant.variant_name == "DWBC_DWBCStabar"
    assert variant.record_name == "DWBC_DWBCStabarRecord"
    assert variant.vehicle_model_name == "Vehicle_DWBC_DWBCStabar"


def test_generated_vehicle_uses_matching_record_import() -> None:
    variant = parse_variant("bellcrank_stabar", "bellcrank_stabar")
    text = render_vehicle_model(variant)

    assert "within BobLib.Vehicle;" in text
    assert "model Vehicle_DWBCStabar_DWBCStabar" in text
    assert "import BobLib.Resources.VehicleDefn.DWBCStabar_DWBCStabarRecord;" in text
    assert "parameter DWBCStabar_DWBCStabarRecord pVehicle;" in text


def test_generated_sim_is_always_vehicle_sim() -> None:
    variant = parse_variant("bellcrank_stabar", "direct")
    text = render_vehicle_sim(variant)

    assert "within BobLib.Standards;" in text
    assert "model VehicleSim" in text
    assert "end VehicleSim;" in text
    assert "model DWBCStabar_DWDirectSim" not in text


def test_generated_sim_uses_matching_record_import() -> None:
    variant = parse_variant("bellcrank_stabar", "direct")
    text = render_vehicle_sim(variant)

    assert "import BobLib.Resources.VehicleDefn.DWBCStabar_DWDirectRecord;" in text
    assert "parameter DWBCStabar_DWDirectRecord pVehicle;" in text


def test_generated_sim_uses_matching_vehicle_model() -> None:
    variant = parse_variant("bellcrank", "bellcrank_stabar")
    text = render_vehicle_sim(variant)

    assert "BobLib.Vehicle.Vehicle_DWBC_DWBCStabar vehicle" in text
    assert "pVehicle = pVehicle" in text


def test_generated_sim_uses_named_integer_modes_without_chirp() -> None:
    variant = parse_variant("bellcrank_stabar", "bellcrank_stabar")
    text = render_vehicle_sim(variant)

    assert "constant Integer MODE_OPEN_LOOP_RAMP = 0;" in text
    assert "constant Integer MODE_OPEN_LOOP_SINE = 1;" in text
    assert "constant Integer MODE_STEP_STEER = 2;" in text
    assert "useMode == MODE_OPEN_LOOP_SINE" in text
    assert "VehicleSim.useMode must be 0 (open-loop ramp), 1 (open-loop sine), or 2 (step steer)." in text
    assert "chirp" not in text.lower()
    assert "useMode == 3" not in text


def test_generated_sim_has_clean_vehicle_diagram_wiring() -> None:
    variant = parse_variant("bellcrank_stabar", "bellcrank_stabar")
    text = render_vehicle_sim(variant)

    assert "Placement(transformation(origin = {100, 10}, extent = {{10, -10}, {-10, 10}}))" in text
    assert "Placement(transformation(origin = {100, -50}, extent = {{10, -10}, {-10, 10}}))" in text
    assert "Line(points = {{-59, -50}, {-42, -50}}, color = {0, 0, 127})" in text
    assert "Line(points = {{-59, -80}, {-30, -80}, {-30, -62}}, color = {0, 0, 127})" in text
    assert "Line(points = {{-20, 110}, {0, 110}, {0, 62}, {0, 62}})" not in text


def test_generated_sim_exposes_animation_switch() -> None:
    variant = parse_variant("bellcrank_stabar", "bellcrank_stabar")
    text = render_vehicle_sim(variant)

    assert "inner parameter Boolean enableAnimation = false" in text
    assert "World world(n = {0, 0, -1}, enableAnimation = enableAnimation)" in text


def test_generated_sim_keeps_structural_anchors_hidden() -> None:
    variant = parse_variant("bellcrank_stabar", "bellcrank_stabar")
    text = render_vehicle_sim(variant)

    assert "FreeMotion cgFreeMotion(\n    animation = false," in text
    assert "Fixed fixedFL(\n    r = cpInitFL,\n    animation = false)" in text
    assert "Fixed cgFixed(\n    r = vehicle.pVehicleCG,\n    animation = false)" in text


def test_generated_sim_uses_fast_dassl_jacobian_mode() -> None:
    variant = parse_variant("bellcrank_stabar", "bellcrank_stabar")
    text = render_vehicle_sim(variant)

    assert 'jacobian = "internalNumerical"' in text
    assert 's = "dassl"' in text


def test_generated_sim_omits_vestigial_open_loop_diagnostics() -> None:
    variant = parse_variant("bellcrank_stabar", "bellcrank_stabar")
    text = render_vehicle_sim(variant)

    vestigial_names = [
        "enableCurvatureSteerLimiter",
        "steerCurvatureLimitGain",
        "steerCurvatureLimitDeadbandFraction",
        "steerNormalLoadLimitGain",
        "steerRatioEstimate",
        "targetAyCmd",
        "targetCurvatureCmd",
        "targetRoadwheel",
        "steerLimiter",
        "linearityGainLossPct",
        "steeringNonlinearityPct",
        "radError",
        "velError",
    ]

    for name in vestigial_names:
        assert name not in text


def test_generated_vehicle_uses_front_bellcrank_stabar_model() -> None:
    variant = parse_variant("bellcrank_stabar", "direct")
    text = render_vehicle_model(variant)

    assert "BobLib.Vehicle.Chassis.Suspension.FrAxleDW_BC_Stabar frAxleDW" in text
    assert "pStabar = pVehicle.pFrStabar" in text


def test_generated_vehicle_wires_transient_relaxation_from_tire_record() -> None:
    variant = parse_variant("bellcrank_stabar", "bellcrank_stabar")
    text = render_vehicle_model(variant)

    assert "redeclare Tire.MF52.SlipModel.TransientSlip slipModel" in text
    assert "FNOMIN = pVehicle.pFrTireModel.relaxation.FNOMIN" in text
    assert "UNLOADED_RADIUS = pVehicle.pFrTireModel.relaxation.UNLOADED_RADIUS" in text
    assert "LFZO = pVehicle.pFrTireModel.relaxation.LFZO" in text
    assert "PTX1 = pVehicle.pFrTireModel.relaxation.PTX1" in text
    assert "PTY1 = pVehicle.pFrTireModel.relaxation.PTY1" in text
    assert "PKY3 = pVehicle.pFrTireModel.relaxation.PKY3" in text
    assert "LSGKP = pVehicle.pFrTireModel.relaxation.LSGKP" in text
    assert "LSGAL = pVehicle.pFrTireModel.relaxation.LSGAL" in text
    assert "FNOMIN = pVehicle.pRrTireModel.relaxation.FNOMIN" in text
    assert "UNLOADED_RADIUS = pVehicle.pRrTireModel.relaxation.UNLOADED_RADIUS" in text
    assert "LFZO = pVehicle.pRrTireModel.relaxation.LFZO" in text
    assert "PTX1 = pVehicle.pRrTireModel.relaxation.PTX1" in text
    assert "PTY1 = pVehicle.pRrTireModel.relaxation.PTY1" in text
    assert "PKY3 = pVehicle.pRrTireModel.relaxation.PKY3" in text
    assert "LSGKP = pVehicle.pRrTireModel.relaxation.LSGKP" in text
    assert "LSGAL = pVehicle.pRrTireModel.relaxation.LSGAL" in text


def test_four_post_timing_tables_cover_symmetric_heave_poses() -> None:
    block = _render_timing_block()
    heave_rows = dict(_table_rows(block, "heaveTable"))

    assert FOUR_POST_HEAVE_MEASURED_COUNT == 11
    assert FOUR_POST_STOP_TIME_S == 118
    assert heave_rows[0] == 0.0
    assert heave_rows[62] == 0.0

    sample_times = [
        FOUR_POST_HEAVE_START_S + FOUR_POST_POSE_STEP_S * i + 4
        for i in range(FOUR_POST_HEAVE_MEASURED_COUNT)
    ]
    sample_values = [heave_rows[time] for time in sample_times]
    assert sample_values == pytest.approx([-1.0 + 0.2 * i for i in range(11)])


def test_generated_vehicle_uses_rear_bellcrank_stabar_model() -> None:
    variant = parse_variant("direct", "bellcrank_stabar")
    text = render_vehicle_model(variant)

    assert "BobLib.Vehicle.Chassis.Suspension.RrAxleDW_BC_Stabar rrAxleDW" in text
    assert "pStabar = pVehicle.pRrStabar" in text


def test_generated_direct_axles_do_not_reference_stabar() -> None:
    variant = parse_variant("direct", "direct")
    text = render_vehicle_model(variant)

    assert "FrAxleDW_Direct frAxleDW" in text
    assert "RrAxleDW_Direct rrAxleDW" in text
    assert "pFrStabar" not in text
    assert "pRrStabar" not in text


def test_all_topology_combinations_render() -> None:
    for front in TOPOLOGIES:
        for rear in TOPOLOGIES:
            variant = parse_variant(front, rear)

            vehicle_text = render_vehicle_model(variant)
            sim_text = render_vehicle_sim(variant)

            assert f"model {variant.vehicle_model_name}" in vehicle_text
            assert f"end {variant.vehicle_model_name};" in vehicle_text

            assert "model VehicleSim" in sim_text
            assert "end VehicleSim;" in sim_text

            assert "extends BobLib.Vehicle.VehicleBase" in vehicle_text
            assert "Powertrain.PTNPlaceholder ptnPlaceholder" in vehicle_text


def test_generate_one_creates_expected_files(tmp_path: Path) -> None:
    generation_dir = tmp_path / "Generation"
    boblib_root = tmp_path / "BobLib"

    boblib_root.mkdir()
    (boblib_root / "package.mo").write_text("package BobLib\nend BobLib;\n")

    output = generate_one(
        front="bellcrank_stabar",
        rear="direct",
        generation_dir=generation_dir,
        boblib_root=boblib_root,
        overwrite=False,
    )

    assert output.build_dir.exists()
    assert output.results_dir.exists()
    assert output.source_vehicle_path.exists()
    assert output.vehicle_model_path.exists()
    assert output.sim_model_path.exists()
    assert (output.results_dir / ".gitkeep").exists()

    assert output.build_dir.name == "DWBCStabar_DWDirect"
    assert output.results_dir.name == "DWBCStabar_DWDirect"
    assert output.source_vehicle_path.name == "Vehicle_DWBCStabar_DWDirect.mo"

    assert output.vehicle_model_path.name == "Vehicle_DWBCStabar_DWDirect.mo"
    assert output.sim_model_path.name == "VehicleSim.mo"


def test_generate_one_refuses_to_overwrite_without_flag(tmp_path: Path) -> None:
    generation_dir = tmp_path / "Generation"
    boblib_root = tmp_path / "BobLib"

    boblib_root.mkdir()
    (boblib_root / "package.mo").write_text("package BobLib\nend BobLib;\n")

    generate_one(
        front="direct",
        rear="direct",
        generation_dir=generation_dir,
        boblib_root=boblib_root,
        overwrite=False,
    )

    with pytest.raises(FileExistsError):
        generate_one(
            front="direct",
            rear="direct",
            generation_dir=generation_dir,
            boblib_root=boblib_root,
            overwrite=False,
        )
