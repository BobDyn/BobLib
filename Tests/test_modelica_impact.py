from pathlib import Path

import pytest

from modelica_impact import UncertainGraph, affected_models, dependency_graph, revision_sources


def sources(**models):
    return {f"BobLib/{name}.mo": f"within BobLib; model {name} {body} end {name};"
            for name, body in models.items()}


def test_transitive_users_and_tests_are_selected_but_unrelated_models_are_not():
    tree = sources(Damper="", Axle="Damper damper;", Vehicle="Axle axle;", Battery="")
    tree["Tests/BobLibTest/TestVehicle.mo"] = "within BobLibTest; model TestVehicle BobLib.Vehicle v; end TestVehicle;"
    affected, reasons = affected_models(["BobLib/Damper.mo"], tree, tree)
    assert set(affected) == {"BobLib.Damper", "BobLib.Axle", "BobLib.Vehicle", "BobLibTest.TestVehicle"}
    assert reasons["BobLibTest.TestVehicle"] == [
        "BobLib/Damper.mo", "BobLib/Axle.mo", "BobLib/Vehicle.mo", "Tests/BobLibTest/TestVehicle.mo",
    ]


def test_removed_reference_still_selects_previous_consumers():
    before = sources(Old="", User="Old old;", Top="User u;")
    after = sources(User="Real x;", Top="User u;")
    affected, _ = affected_models(["BobLib/Old.mo", "BobLib/User.mo"], after, before)
    assert set(affected) == {"BobLib.User", "BobLib.Top"}


def test_renamed_class_selects_old_and_new_users():
    before = sources(Old="", User="Old old;")
    after = sources(New="", User="New new;")
    affected, _ = affected_models(["BobLib/Old.mo", "BobLib/New.mo"], after, before)
    assert set(affected) == {"BobLib.New", "BobLib.User"}


@pytest.mark.parametrize("use", [
    "extends Base;", "replaceable model Choice = Base constrainedby Contract;",
    "redeclare model Choice = Base;", "Base component if false;",
    "parameter Real x = Base();", "import Alias = BobLib.Base; Alias component;",
    "import BobLib.Base; Base component;", "parameter Base calibration;",
])
def test_all_reference_roles_retain_dependencies(use):
    tree = sources(Base="", Contract="", User=use)
    assert "BobLib.User" in affected_models(["BobLib/Base.mo"], tree, tree)[0]


def test_ambiguous_short_names_keep_every_candidate_and_cycles_terminate():
    tree = sources(User="Thing x;", Other="User y;")
    tree.update({"BobLib/A/Thing.mo": "model Thing Other o; end Thing;",
                 "BobLib/B/Thing.mo": "model Thing end Thing;"})
    assert {"BobLib/A/Thing.mo", "BobLib/B/Thing.mo"} <= dependency_graph(tree)["BobLib/User.mo"]
    assert "BobLib.Other" in affected_models(["BobLib/B/Thing.mo"], tree, tree)[0]


def test_package_alias_and_package_order_cover_descendants():
    tree = sources(User="import P = BobLib.P; P.Child c;", Unrelated="")
    tree.update({"BobLib/P/package.mo": "within BobLib; package P end P;",
                 "BobLib/P/Child.mo": "within BobLib.P; model Child end Child;"})
    for path in ("BobLib/P/package.mo", "BobLib/P/package.order", "BobLib/P/Child.mo"):
        affected, _ = affected_models([path], tree, tree)
        assert "BobLib.User" in affected
        assert "BobLib.Unrelated" not in affected


def test_nested_declarations_resolve_to_owning_source():
    tree = sources(Owner="record Nested Real x; end Nested;", User="BobLib.Owner.Nested x;")
    assert "BobLib.User" in affected_models(["BobLib/Owner.mo"], tree, tree)[0]


def test_comments_and_strings_do_not_create_dependencies():
    tree = sources(Base="", User='Real x "Base \\\"quoted\\\""; // Base\n /* Base */')
    assert "BobLib.User" not in affected_models(["BobLib/Base.mo"], tree, tree)[0]


@pytest.mark.parametrize("body", ["'quoted name' x;", 'Real x "unfinished;',
                                   "/* unfinished", "BobLib.Missing x;"])
def test_unsupported_or_unresolved_source_cannot_claim_a_safe_graph(body):
    with pytest.raises(UncertainGraph):
        dependency_graph(sources(User=body))


def test_revision_reads_deleted_files_without_checkout(tmp_path):
    import subprocess

    def git(*args):
        return subprocess.run(["git", *args], cwd=tmp_path, check=True, capture_output=True, text=True)
    git("init")
    (tmp_path / "BobLib").mkdir()
    path = tmp_path / "BobLib/A.mo"
    path.write_text("model A end A;\n")
    git("add", ".")
    git("-c", "user.name=Test", "-c", "user.email=test@example.invalid", "commit", "-m", "base")
    path.unlink()
    assert revision_sources(tmp_path, "HEAD") == {"BobLib/A.mo": "model A end A;\n"}
    assert not path.exists()


def test_real_library_damper_reaches_vehicle_but_not_battery():
    from modelica_impact import working_sources

    root = Path(__file__).resolve().parents[1]
    tree = working_sources(root)
    affected, _ = affected_models(["BobLib/Chassis/Suspension/Components/TabularDamper.mo"], tree, tree)
    assert "BobLib.Experiments.Standards.VehicleSim" in affected
    assert "BobLibTest.Chassis.Suspension.Components.TestTabularDamper" in affected
    assert "BobLib/EnergyStorage/BatteryPack.mo" in tree
    assert "BobLib.EnergyStorage.BatteryPack" not in affected
    assert len(affected) < len(tree) // 2
