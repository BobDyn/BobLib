#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import shutil
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


@dataclass(frozen=True)
class SourceEdit:
    path: str
    old: str
    new: str


# Edits to the sources OpenModelica 1.27.1 writes, applied before compiling.
# Each must match exactly once, so a changed OpenModelica stops the export
# instead of building an FMU without the edit.
SOURCE_EDITS = (
    # OpenModelica puts OMC_DISABLE_OPT (no optimization) above the first
    # function in this file, read_simulation_info, but not above
    # read_input_fmu, which sets every variable's name, unit and start value.
    # Optimized, read_input_fmu takes clang more than an hour to compile.
    SourceEdit(
        f"{FMU_NAME}_init_fmu.c",
        f"\n\nvoid {FMU_NAME}_read_input_fmu(MODEL_DATA* modelData)\n",
        f"\n\nOMC_DISABLE_OPT\nvoid {FMU_NAME}_read_input_fmu(MODEL_DATA* modelData)\n",
    ),
    # A failed model evaluation inside a CVODE step returns -1, which CVODE
    # treats as unrecoverable; 1 makes it retry with a smaller step, as
    # OpenModelica master does since
    # https://github.com/OpenModelica/OpenModelica/pull/16988 (not in 1.27.1).
    # Without this, driving off after holding the brake at standstill ends the
    # co-simulation.
    SourceEdit(
        "simulation/solver/cvode_solver.c",
        "  if (!success)\n  {\n    retVal = -1;\n  }\n",
        "  if (!success)\n  {\n    retVal = 1;\n  }\n",
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


def render_mos(package_root: Path, work_dir: Path) -> str:
    return f"""
clear();
setCommandLineOptions("{OMC_COMMAND_LINE_OPTIONS}");
setCommandLineOptions("{FMI_COMMAND_LINE_OPTIONS}");
loadModel(Modelica, {{"{MODELICA_VERSION}"}});
loadModel(VehicleInterfaces, {{"{VEHICLE_INTERFACES_VERSION}"}});
loadFile("{package_root.as_posix()}/package.mo");
loadString("{EXPORT_MODEL}");
cd("{work_dir.as_posix()}");
writeFile("compiler.txt", getCompiler());
buildModelFMU(
  {FMU_NAME},
  version="2.0",
  fmuType="me_cs",
  fileNamePrefix="{FMU_NAME}",
  platforms=fill("", 0));
getErrorString();
""".strip() + "\n"


def run_command(command: list[str], *, cwd: Path, timeout_s: int) -> str:
    try:
        completed = subprocess.run(
            command,
            cwd=cwd,
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=timeout_s,
        )
    except subprocess.TimeoutExpired as exc:
        output = exc.stdout or ""
        if isinstance(output, bytes):
            output = output.decode("utf-8", errors="replace")
        raise RuntimeError(
            f"{' '.join(command)} timed out after {timeout_s} seconds:\n{output}"
        ) from exc
    if completed.returncode != 0:
        raise RuntimeError(f"{' '.join(command)} failed:\n{completed.stdout}")
    return completed.stdout


def apply_source_edits(sources: Path) -> None:
    for edit in SOURCE_EDITS:
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


def export_fmu(package_root: Path, output_dir: Path, *, timeout_s: int) -> Path:
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
        mos_path.write_text(render_mos(package_root, work_dir), encoding="utf-8")
        output = run_command([omc, str(mos_path)], cwd=work_dir, timeout_s=timeout_s)
        # omc exits 0 when a script call fails, so check what it wrote instead.
        source_fmu = work_dir / f"{FMU_NAME}.fmu"
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
        flags_file = unpacked / "resources" / f"{FMU_NAME}_flags.json"
        flags = json.loads(flags_file.read_text(encoding="utf-8")) if flags_file.is_file() else {}
        if flags.get("s") != "cvode":
            raise RuntimeError(f"omc did not apply {FMI_COMMAND_LINE_OPTIONS}:\n{output}")
        apply_source_edits(unpacked / "sources")
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
        built_fmu = work_dir / "compiled" / f"{FMU_NAME}.fmu"
        zip_fmu(unpacked, built_fmu)

        output_dir.mkdir(parents=True, exist_ok=True)
        fmu_path = output_dir / f"{FMU_NAME}.fmu"
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
