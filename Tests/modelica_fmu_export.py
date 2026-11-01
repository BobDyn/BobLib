#!/usr/bin/env python3
from __future__ import annotations

import argparse
import contextlib
import json
import os
import shutil
import signal
import subprocess
import tempfile
import zipfile
from dataclasses import dataclass
from pathlib import Path

MODELICA_VERSION = "4.1.0"
VEHICLE_INTERFACES_VERSION = "2.0.2"
OMC_COMMAND_LINE_OPTIONS = (
    "--matchingAlgorithm=PFPlusExt "
    "--indexReductionMethod=dynamicStateSelection "
    "-d=initialization,NLSanalyticJacobian,disableStartCalc "
    "--maxSizeLinearTearing=5000 "
    "--generateDynamicJacobian=none"
)
# Without a solver flag, the co-simulation FMU integrates with explicit Euler
# at the communication step.
FMI_COMMAND_LINE_OPTIONS = "--fmiFlags=s:cvode"
FMU_NAME = "VehicleFMI"
# VehicleFMI with headless = true, as for batch runs: no MultiBody animation geometry.
EXPORT_MODEL = (
    f"model {FMU_NAME} "
    "extends BobLib.Experiments.Standards.VehicleFMI(headless = true); "
    f"end {FMU_NAME};"
)
DEFAULT_TIMEOUT_S = 3600
# Most state events one communication step may handle; one more fails the step.
# An ordinary step raises a few at most (VehicleFMI: up to 4 in stress runs);
# OpenModelica's simulation runtime warns about chattering from 100 in a row.
CHATTERING_STATE_EVENTS = 100


@dataclass(frozen=True)
class SourceEdit:
    path: str
    old: str
    new: str


def source_edits(name: str) -> tuple[SourceEdit, ...]:
    """Edits to the sources OpenModelica 1.27.1 writes for FMU ``name``.

    Each must match exactly once, so a changed OpenModelica stops the export
    instead of building an FMU without the edit.
    """
    cvode = "simulation/solver/cvode_solver.c"
    interface = "fmi-export/fmu2_model_interface.c"
    return (
        # Roots below the integration accuracy chatter at a held brake or motor
        # direction threshold. Keep hysteresis at the requested tolerance,
        # scaled by each relation's nominal values in Modelica's runtime.
        SourceEdit(
            "simulation/solver/model_help.c",
            "tolZC = TOL_HYSTERESIS_ZEROCROSSINGS * fmax(relativeTol, MINIMAL_STEP_SIZE);",
            "tolZC = fmax(relativeTol, MINIMAL_STEP_SIZE);",
        ),
        # OpenModelica puts OMC_DISABLE_OPT (no optimization) above the first
        # function in this file, read_simulation_info, but not above
        # read_input_fmu, which sets every variable's name, unit and start
        # value. Optimized, read_input_fmu takes clang more than an hour.
        SourceEdit(
            f"{name}_init_fmu.c",
            f"\n\nvoid {name}_read_input_fmu(MODEL_DATA* modelData)\n",
            f"\n\nOMC_DISABLE_OPT\nvoid {name}_read_input_fmu(MODEL_DATA* modelData)\n",
        ),
        # A failed model evaluation inside a CVODE step returns -1, which CVODE
        # treats as unrecoverable; 1 makes it retry with a smaller step, as
        # OpenModelica master does since
        # https://github.com/OpenModelica/OpenModelica/pull/16988 (not in
        # 1.27.1). Without this, driving off after holding the brake at
        # standstill ends the co-simulation.
        SourceEdit(
            cvode,
            "  if (!success)\n  {\n    retVal = -1;\n  }\n",
            "  if (!success)\n  {\n    retVal = 1;\n  }\n",
        ),
        # fmi2Instantiate and fmi2Reset set CVODE up with the model's default
        # tolerance before fmi2SetupExperiment is called, so the requested
        # tolerance never reached it. Apply it, scaled by each state's nominal
        # value as cvode_solver_initial does, whenever cvode_solver_fmi_step
        # starts or restarts CVODE.
        SourceEdit(
            cvode,
            "  if (solverInfo->didEventStep || !cvodeData->isInitialized)"
            "    // TODO Save if we have had an event\n"
            "  {\n"
            "    cvode_solver_reinit(data, threadData, solverInfo, cvodeData);\n"
            "    cvodeData->isInitialized = TRUE;\n"
            "  }\n",
            "  if (solverInfo->didEventStep || !cvodeData->isInitialized)"
            "    // TODO Save if we have had an event\n"
            "  {\n"
            "    cvode_solver_reinit(data, threadData, solverInfo, cvodeData);\n"
            "    cvodeData->isInitialized = TRUE;\n"
            "    if (comp->toleranceDefined && comp->tolerance > 0)\n"
            "    {\n"
            "      long int i;\n"
            "      for (i = 0; i < cvodeData->N; i++)\n"
            "      {\n"
            "        NV_Ith_S(cvodeData->absoluteTolerance, i) = comp->tolerance * fmax(fabs(\n"
            "          getNominalFromScalarIdx(data->simulationInfo, data->modelData,\n"
            "                                  VAR_KIND_STATE, i)), 1e-32);\n"
            "      }\n"
            "      flag = CVodeSVtolerances(cvodeData->cvode_mem, comp->tolerance,\n"
            "                               cvodeData->absoluteTolerance);\n"
            "      if (flag < 0)\n"
            "      {\n"
            "        FILTERED_LOG(comp, fmi2Fatal, LOG_STATUSFATAL,\n"
            "          \"fmi2DoStep: ##CVODE## CVodeSVtolerances failed with flag %i.\", flag)\n"
            "        return -1;\n"
            "      }\n"
            "    }\n"
            "  }\n",
        ),
        # The zero-crossing hysteresis came from the tolerance in
        # fmi2SetupExperiment even when none was requested (0), which leaves
        # relations with no hysteresis. Use the smaller of the model's step size
        # and tolerance then, as OpenModelica's simulation runtime does. Without
        # hysteresis, the event search below lets solver noise flip the
        # relations of VehicleFMI's motor direction latch back and forth, and a
        # stop-and-go drive fails.
        SourceEdit(
            interface,
            "  setZCtol(comp->tolerance); /* set zero-crossing tolerance */\n",
            "  /* set zero-crossing tolerance */\n"
            "  setZCtol(comp->toleranceDefined && comp->tolerance > 0 ? comp->tolerance\n"
            "           : fmin(comp->fmuData->simulationInfo->stepSize,\n"
            "                  comp->fmuData->simulationInfo->tolerance));\n",
        ),
        # State events were only detected at the end of a communication step,
        # so events such as a reset happened late. Let CVODE find them inside
        # the step (it already does outside FMUs). Its root function must read
        # the states CVODE passes, as the right-hand side function does.
        SourceEdit(
            cvode,
            "  if(omc_flag[FLAG_NO_ROOTFINDING] || isFMI)\n",
            "  if(omc_flag[FLAG_NO_ROOTFINDING])\n",
        ),
        SourceEdit(
            cvode,
            "#endif\n"
            "  /* eval needed equations (exclude from timer) */\n",
            "#endif\n"
            "  {\n"
            "    long int i;\n"
            "    for (i = 0; i < cvodeData->N; i++)\n"
            "    {\n"
            "      data->localData[0]->realVars[i] = NV_Ith_S(y, i);\n"
            "    }\n"
            "  }\n"
            "  /* eval needed equations (exclude from timer) */\n",
        ),
        # When CVODE stops at a root, cvode_solver_fmi_step reports it ...
        SourceEdit(
            cvode,
            "step done to time = %.15g.\", comp->solverInfo->currentTime)\n"
            "  }\n"
            "  else\n",
            "step done to time = %.15g.\", comp->solverInfo->currentTime)\n"
            "  }\n"
            "  else if (flag == CV_ROOT_RETURN)\n"
            "  {\n"
            "    FILTERED_LOG(comp, fmi2OK, LOG_ALL,\n"
            "      \"fmi2DoStep:##CVODE## state event at time = %.15g.\",\n"
            "      solverInfo->currentTime)\n"
            "    return 1;\n"
            "  }\n"
            "  else\n",
        ),
        # ... and fmi2DoStep handles the event there, then integrates on to the
        # end of the communication step. The event flags are reset for each
        # pass through the loop, or one event would repeat in every later pass.
        # A relation can cross its threshold again and again within one step.
        # Without a limit, VehicleFMI once handled 2,725 state events in one
        # 20 ms step, the last ones less than 1e-12 s apart, and then the
        # process aborted inside the OpenModelica runtime. A step with more than
        # CHATTERING_STATE_EVENTS events fails with fmi2Error instead; it does
        # not skip events. VehicleFMI's motor direction latch can chatter like
        # this when the car pulls away again after holding the brake at
        # standstill.
        SourceEdit(
            interface,
            "  int i, zc_event = 0, time_event = 0;\n"
            "  int flag;\n",
            "  int i, zc_event = 0, time_event = 0, state_events = 0;\n"
            "  int flag;\n",
        ),
        SourceEdit(
            interface,
            "  while (status == fmi2OK && comp->fmuData->localData[0]->timeValue < tEnd)\n"
            "  {\n"
            "    /* fprintf(stderr, \"DoStep",
            "  while (status == fmi2OK && comp->fmuData->localData[0]->timeValue < tEnd)\n"
            "  {\n"
            "    zc_event = 0;\n"
            "    time_event = 0;\n"
            "    /* fprintf(stderr, \"DoStep",
        ),
        SourceEdit(
            interface,
            "          status = fmi2Fatal;\n"
            "          goto doStep_cleanup;\n"
            "        }\n"
            "#else\n",
            "          status = fmi2Fatal;\n"
            "          goto doStep_cleanup;\n"
            "        }\n"
            "        if (flag == 1)\n"
            "        {\n"
            f"          if (++state_events > {CHATTERING_STATE_EVENTS})\n"
            "          {\n"
            "            FILTERED_LOG(comp, fmi2Error, LOG_STATUSERROR,\n"
            f"              \"fmi2DoStep: more than {CHATTERING_STATE_EVENTS} state events \"\n"
            "              \"in one communication step, the last at time = %.15g.\",\n"
            "              comp->solverInfo->currentTime)\n"
            "            status = fmi2Error;\n"
            "            goto doStep_cleanup;\n"
            "          }\n"
            "          tNext = comp->solverInfo->currentTime;\n"
            "          time_event = 0;\n"
            "          zc_event = 1;\n"
            "        }\n"
            "#else\n",
        ),
    )


def default_package_root() -> Path:
    cwd = Path.cwd()
    candidates = [
        cwd / "BobLib",
        cwd / "_0_Utils/external/BobLib/BobLib",
    ]
    for candidate in candidates:
        if (candidate / "package.mo").is_file():
            return candidate
    raise FileNotFoundError(
        "Could not locate BobLib/package.mo. Pass --package-root explicitly."
    )


def render_mos(package_root: Path, work_dir: Path, name: str, model: str) -> str:
    model_string = model.replace("\\", "\\\\").replace('"', '\\"')
    return f"""
clear();
setCommandLineOptions("{OMC_COMMAND_LINE_OPTIONS}");
setCommandLineOptions("{FMI_COMMAND_LINE_OPTIONS}");
loadModel(Modelica, {{"{MODELICA_VERSION}"}});
loadModel(VehicleInterfaces, {{"{VEHICLE_INTERFACES_VERSION}"}});
loadFile("{package_root.as_posix()}/package.mo");
loadString("{model_string}");
cd("{work_dir.as_posix()}");
writeFile("compiler.txt", getCompiler());
buildModelFMU(
  {name},
  version="2.0",
  fmuType="me_cs",
  fileNamePrefix="{name}",
  platforms=fill("", 0));
getErrorString();
""".strip() + "\n"


def _kill_process_group(process: subprocess.Popen[str]) -> None:
    # macOS reports EPERM when every process left in the group is a zombie.
    with contextlib.suppress(ProcessLookupError, PermissionError):
        os.killpg(process.pid, signal.SIGKILL)


def run_command(command: list[str], *, cwd: Path, timeout_s: int) -> str:
    # In its own session, the command and every job it starts (make, the
    # compilers) share one process group, so a timeout can stop all of them.
    with subprocess.Popen(
        command,
        cwd=cwd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    ) as process:
        try:
            output, _ = process.communicate(timeout=timeout_s)
        except subprocess.TimeoutExpired as exc:
            _kill_process_group(process)
            try:
                output, _ = process.communicate(timeout=10)
            except subprocess.TimeoutExpired as drain:
                # A process outside the group still holds the pipe.
                process.wait()
                output = drain.output or ""
                if isinstance(output, bytes):
                    output = output.decode("utf-8", errors="replace")
            raise RuntimeError(
                f"{' '.join(command)} timed out after {timeout_s} seconds:\n{output}"
            ) from exc
        except BaseException:
            # For example Ctrl+C, which the new session no longer receives.
            _kill_process_group(process)
            process.wait()
            raise
    if process.returncode != 0:
        raise RuntimeError(f"{' '.join(command)} failed:\n{output}")
    return output


def apply_source_edits(sources: Path, name: str) -> None:
    edits = source_edits(name)
    # OMC 1.27.1 accepts x0 here, but returns the stale working vector x.
    # Models without nonlinear systems omit this runtime source entirely.
    homotopy = "simulation/solver/nonlinearSolverHomotopy.c"
    if (sources / homotopy).is_file():
        old = (
            "        success = NLS_SOLVED;\n"
            "        /* take the solution */\n"
            "        vecCopy(homotopyData->n, homotopyData->x, nlsData->nlsx);\n"
            "        /* reset continous flag */"
        )
        edits += (SourceEdit(homotopy, old, old.replace("homotopyData->x,", "homotopyData->x0,")),)
    for edit in edits:
        path = sources / edit.path
        # Bytes keep the line endings as OpenModelica wrote them.
        text = path.read_bytes().decode("utf-8")
        if text.count(edit.old) != 1:
            raise RuntimeError(
                f"{path} no longer matches the code this export edits. "
                "Check whether this OpenModelica version still needs the edit."
            )
        path.write_bytes(text.replace(edit.old, edit.new).encode("utf-8"))


def zip_fmu(unpacked: Path, fmu_path: Path) -> None:
    # zipfile stores the files that symlinks such as libsundials_cvode.so.5
    # point to. CMake's create_fmu target stores the links themselves, which
    # importers that unzip with Python cannot load.
    fmu_path.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(
        fmu_path, "w", zipfile.ZIP_DEFLATED, strict_timestamps=False
    ) as archive:
        for path in sorted(unpacked.rglob("*")):
            if path.is_file():
                archive.write(path, path.relative_to(unpacked).as_posix())


def export_fmu(
    package_root: Path,
    output_dir: Path,
    *,
    timeout_s: int,
    name: str = FMU_NAME,
    model: str = EXPORT_MODEL,
) -> Path:
    """Export the top-level Modelica class ``name``, defined by ``model``, to output_dir."""
    if os.name == "nt":
        # On Windows, buildModelFMU compiles with OpenModelica's own MSYS
        # toolchain, which this export does not set up.
        raise RuntimeError("The FMU export does not support Windows yet")
    omc = shutil.which("omc")
    if omc is None:
        raise RuntimeError("OpenModelica omc is not installed")
    cmake = shutil.which("cmake")
    if cmake is None:
        raise RuntimeError("CMake is not installed")

    with tempfile.TemporaryDirectory(prefix="boblib_fmu_export_") as raw_dir:
        work_dir = Path(raw_dir)
        mos_path = work_dir / "export.mos"
        mos_path.write_text(render_mos(package_root, work_dir, name, model), encoding="utf-8")
        output = run_command([omc, str(mos_path)], cwd=work_dir, timeout_s=timeout_s)
        # omc exits 0 when a script call fails, so check what it wrote instead.
        source_fmu = work_dir / f"{name}.fmu"
        compiler_file = work_dir / "compiler.txt"
        for path in (source_fmu, compiler_file):
            if not path.is_file():
                raise RuntimeError(f"omc did not write {path}:\n{output}")
        compiler = compiler_file.read_text(encoding="utf-8").strip()

        # OpenModelica writes a source FMU with a CMake project. Configuring it and
        # building its install target puts the binaries into the unpacked FMU;
        # buildModelFMU runs the same two steps on Linux.
        unpacked = work_dir / "unpacked"
        with zipfile.ZipFile(source_fmu) as archive:
            archive.extractall(unpacked)
        flags_file = unpacked / "resources" / f"{name}_flags.json"
        flags = json.loads(flags_file.read_text(encoding="utf-8")) if flags_file.is_file() else {}
        if flags.get("s") != "cvode":
            raise RuntimeError(f"omc did not apply {FMI_COMMAND_LINE_OPTIONS}:\n{output}")
        apply_source_edits(unpacked / "sources", name)
        build_dir = work_dir / "build"
        run_command(
            [
                cmake,
                "-S",
                str(unpacked / "sources"),
                "-B",
                str(build_dir),
                f"-DCMAKE_C_COMPILER={compiler}",
            ],
            cwd=work_dir,
            timeout_s=timeout_s,
        )
        run_command(
            [
                cmake,
                "--build",
                str(build_dir),
                "--parallel",
                str(os.cpu_count() or 1),
                "--target",
                "install",
            ],
            cwd=work_dir,
            timeout_s=timeout_s,
        )
        built_fmu = work_dir / "compiled" / f"{name}.fmu"
        zip_fmu(unpacked, built_fmu)

        output_dir.mkdir(parents=True, exist_ok=True)
        fmu_path = output_dir / f"{name}.fmu"
        shutil.copyfile(built_fmu, fmu_path)
    return fmu_path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Export BobLib.Experiments.Standards.VehicleFMI as an FMI 2.0 FMU."
    )
    parser.add_argument(
        "--package-root",
        type=Path,
        default=None,
        help="Path to the BobLib package directory containing package.mo.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("build"),
        help="Directory that receives VehicleFMI.fmu.",
    )
    parser.add_argument(
        "--timeout-s",
        type=int,
        default=DEFAULT_TIMEOUT_S,
        help="Timeout in seconds for each OpenModelica and CMake step.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    package_root = args.package_root or default_package_root()
    package_root = package_root.resolve()

    if not (package_root / "package.mo").is_file():
        raise FileNotFoundError(f"package.mo not found under {package_root}")

    fmu_path = export_fmu(package_root, args.output_dir.resolve(), timeout_s=args.timeout_s)
    print(f"Wrote {fmu_path}")


if __name__ == "__main__":
    main()
