"""Guard the OMC 1.27.1 nonlinear initial-guess correction."""
from pathlib import Path

import pytest

import modelica_fmu_export as fmu_export

HOMOTOPY = "simulation/solver/nonlinearSolverHomotopy.c"
EARLY_SUCCESS = (
    "        success = NLS_SOLVED;\n"
    "        /* take the solution */\n"
    "        vecCopy(homotopyData->n, homotopyData->x, nlsData->nlsx);\n"
    "        /* reset continous flag */"
)


def original_sources(root: Path) -> None:
    for edit in fmu_export.source_edits("Probe"):
        path = root / edit.path
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("a") as stream:
            stream.write(edit.old)


def test_accepted_initial_guess_is_returned(tmp_path: Path) -> None:
    original_sources(tmp_path)
    path = tmp_path / HOMOTOPY
    path.write_text(EARLY_SUCCESS)
    fmu_export.apply_source_edits(tmp_path, "Probe")
    assert path.read_text() == EARLY_SUCCESS.replace("homotopyData->x,", "homotopyData->x0,")


def test_models_without_nonlinear_solver_still_export(tmp_path: Path) -> None:
    original_sources(tmp_path)
    fmu_export.apply_source_edits(tmp_path, "Probe")
    assert not (tmp_path / HOMOTOPY).exists()


def test_unknown_nonlinear_runtime_is_rejected(tmp_path: Path) -> None:
    original_sources(tmp_path)
    (tmp_path / HOMOTOPY).write_text("changed upstream runtime")
    with pytest.raises(RuntimeError, match="no longer matches"):
        fmu_export.apply_source_edits(tmp_path, "Probe")
