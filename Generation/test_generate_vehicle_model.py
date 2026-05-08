from __future__ import annotations

from pathlib import Path

import pytest

from generate_vehicle_model import (
    TOPOLOGIES,
    generate_one,
    parse_variant,
    render_vehicle_model,
    render_vehicle_sim,
)


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


def test_generated_vehicle_uses_front_bellcrank_stabar_model() -> None:
    variant = parse_variant("bellcrank_stabar", "direct")
    text = render_vehicle_model(variant)

    assert "BobLib.Vehicle.Chassis.Suspension.FrAxleDW_BC_Stabar frAxleDW" in text
    assert "pStabar = pVehicle.pFrStabar" in text


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
    assert output.vehicle_model_path.exists()
    assert output.sim_model_path.exists()
    assert output.build_script_path.exists()
    assert output.readme_path.exists()
    assert (output.results_dir / ".gitkeep").exists()

    assert output.build_dir.name == "DWBCStabar_DWDirect"
    assert output.results_dir.name == "DWBCStabar_DWDirect"

    assert output.vehicle_model_path.name == "Vehicle_DWBCStabar_DWDirect.mo"
    assert output.sim_model_path.name == "VehicleSim.mo"
    assert output.build_script_path.name == "Build.mos"


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