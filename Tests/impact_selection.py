"""Apply a CI impact plan before sharding, without executing test fixtures.

Only explicit dependency contracts can exclude a test. Unmapped tests stay in
selection. Never infer completeness from a partial string scan of Python code.
"""
from __future__ import annotations

import json
from pathlib import Path

import pytest

from ci_changes import source_digest
from modelica_impact import working_sources

# These test functions pass exactly the named parameter to their model runner.
# New families must add an explicit contract or remain conservatively selected.
PARAMETER_MODELS = {
    ("test_modelica_components.py", "test_modelica_component_simulates"): ("case", "test"),
    ("test_modelica_translation.py", "test_modelica_model_translates"): ("model_check", "name"),
    ("test_modelica_initialization.py", "test_modelica_model_initializes_to_baseline"): ("model", None),
    ("test_modelica_physics_validation.py", "test_modelica_physical_baseline_matches"): ("baseline", "model"),
    ("test_modelica_regression.py", "test_low_level_signal_model_returns_expected_values"): ("model", None),
    ("test_boblib_modelica.py", "test_boblib_model_translates"): ("model", None),
}


def dependencies(item) -> set[str] | None:
    """Return complete model dependencies, or None if they are not declared."""
    markers = list(item.iter_markers("modelica_models"))
    if markers:
        return {name for marker in markers for name in marker.args}
    key = (item.path.name, getattr(item, "originalname", item.name))
    contract = PARAMETER_MODELS.get(key)
    if contract and hasattr(item, "callspec"):
        parameter, field = contract
        value = item.callspec.params.get(parameter)
        if field:
            value = value.get(field) if isinstance(value, dict) else getattr(value, field, None)
        if isinstance(value, str):
            return {value}
    # This module has three independent fixture builds. Only the vehicle build
    # depends on BobLib; the other two test the compiler's generic FMU runtime.
    if item.path.name == "test_modelica_fmu_export.py":
        if "vehicle_fmu" in item.fixturenames:
            return {"BobLib.Experiments.Standards.VehicleFMI"}
        if {"reset_probe_fmu", "decay_probe_fmu"} & set(item.fixturenames):
            return set()
    return None


def select(items, plan: dict, root: Path) -> tuple[list, list[dict]]:
    """Select conservatively and explain every keep/drop decision."""
    selected, decisions = [], []
    changed = set(plan["changed_tests"])
    affected = set(plan["affected_models"])
    for item in items:
        path = item.path.resolve().relative_to(root.resolve()).as_posix()
        models = dependencies(item)
        if plan["mode"] == "full":
            keep, reason = True, plan["reason"]
        elif path in changed:
            keep, reason = True, "Test module changed"
        elif plan["mode"] in {"none", "tests"}:
            keep, reason = False, "No changed dependency"
        elif models is None:
            keep, reason = True, "Dependency contract unknown; retained conservatively"
        else:
            # An unrecognized model could be newly generated or misspelled.
            known = set(plan.get("known_models", ()))
            unknown = models - known
            keep = bool(models & affected or unknown)
            reason = ("Unknown model dependency; retained conservatively" if unknown else
                      "Affected model dependency" if keep else "No affected model dependency")
        if keep:
            selected.append(item)
        decisions.append({"nodeid": item.nodeid, "selected": keep, "reason": reason,
                          "models": None if models is None else sorted(models)})
    return selected, decisions


def configure(config, root: Path) -> None:
    """Validate once on the controller and on every xdist worker."""
    path = config.getoption("--boblib-impact-plan")
    if not path:
        return
    try:
        plan = json.loads(Path(path).read_text(encoding="utf-8"))
        if plan["schema"] != 1 or plan["mode"] not in {"full", "impact", "tests", "none"}:
            raise ValueError("Unsupported impact plan")
        if not all(isinstance(plan[key], list) for key in ("changed_tests", "affected_models")):
            raise ValueError("Malformed test/model lists")
        if plan["mode"] == "impact" and plan["source_digest"] != source_digest(working_sources(root)):
            raise ValueError("Modelica sources changed after impact planning")
    except (OSError, ValueError, KeyError, TypeError) as exc:
        raise pytest.UsageError(f"Invalid impact plan {path}: {exc}. Regenerate it or run the full gate.") from exc
    config._boblib_impact_plan = plan


def apply(config, items, root: Path) -> None:
    plan = getattr(config, "_boblib_impact_plan", None)
    if plan is None:
        return
    selected, decisions = select(items, plan, root)
    selected_ids = {id(item) for item in selected}
    config.hook.pytest_deselected(items=[item for item in items if id(item) not in selected_ids])
    items[:] = selected
    config._boblib_impact_selected = len(selected)
    report = config.getoption("--boblib-impact-report")
    if report:
        destination = Path(report)
        worker = getattr(config, "workerinput", {}).get("workerid")
        if worker:
            destination = destination.with_name(f"{destination.stem}-{worker}{destination.suffix}")
        destination.write_text(json.dumps(decisions, indent=2) + "\n", encoding="utf-8")
