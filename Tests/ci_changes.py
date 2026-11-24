"""Decide which OpenModelica CI jobs a pull request needs.

Most of BobLib CI's wall clock is the OpenModelica compile, and about a third
of pull requests change only test modules. Those still need the tests they
touch, but not a recompile of every smoke model, FMU and component shard. The
`changes` job in .github/workflows/ci.yml pipes the PR's changed paths in here
and gates the Modelica jobs on the outputs.

Anything this script does not recognise forces the full run, so a new file
type or directory can only make CI slower, never skip a check it needed.
"""

from __future__ import annotations

import argparse
import json
import sys
from fnmatch import fnmatch
from pathlib import Path

# Paths no CI job reads.
IGNORED = (
    "*.md",
    "LICENSE",
    ".gitignore",
    ".github/ISSUE_TEMPLATE/*",
    ".github/dependabot.yml",
    "development/visual-record-drafts/*",
)

# Test modules the makefile runs outside the component shards. Keep in sync
# with MODELICA_*_TESTS in the makefile; a test here is checked there.
NOT_IN_COMPONENT_SHARDS = frozenset(
    {
        "Tests/test_modelica_translation.py",
        "Tests/test_modelica_initialization.py",
        "Tests/test_modelica_physics_validation.py",
        "Tests/test_modelica_regression.py",
        "Tests/test_boblib_modelica.py",
        "Tests/test_modelica_fmu_export.py",
    }
)

FULL_SHARDS = 4


def _is_test_module(path: str) -> bool:
    return fnmatch(path, "Tests/test_*.py") and "/" not in path[len("Tests/") :]


def classify(paths: list[str], root: Path | None = None) -> dict[str, str]:
    """Return GitHub Actions outputs for the changed ``paths``.

    ``root`` is the checkout; when given, deleted test modules are dropped from
    the selection because pytest cannot collect them.
    """
    paths = [p.strip() for p in paths if p.strip()]
    # An empty list means the diff failed or saw nothing; never skip on that.
    if not paths:
        paths = ["<unknown>"]
    relevant = [p for p in paths if not any(fnmatch(p, g) for g in IGNORED)]
    full = any(not _is_test_module(p) for p in relevant)
    tests = sorted({p for p in relevant if _is_test_module(p)})
    if root is not None:
        tests = [t for t in tests if (root / t).is_file()]

    if full:
        return {
            "full": "true",
            "smoke": "true",
            "fmu": "true",
            "components": "true",
            "shards": json.dumps(list(range(1, FULL_SHARDS + 1))),
            "shard_count": str(FULL_SHARDS),
            "component_tests": "",
        }

    component_tests = [t for t in tests if t not in NOT_IN_COMPONENT_SHARDS]
    return {
        "full": "false",
        "smoke": str("Tests/test_modelica_initialization.py" in tests).lower(),
        "fmu": str(any("fmu" in Path(t).name for t in tests)).lower(),
        "components": str(bool(component_tests)).lower(),
        "shards": json.dumps([1] if component_tests else []),
        "shard_count": "1",
        "component_tests": " ".join(component_tests),
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--full",
        action="store_true",
        help="Run everything regardless of input (pushes, tags, manual runs).",
    )
    parser.add_argument("--root", type=Path, default=Path.cwd())
    args = parser.parse_args(argv)
    if args.full:
        outputs = classify(["makefile"])
    else:
        outputs = classify(sys.stdin.read().splitlines(), args.root)
    for key, value in outputs.items():
        print(f"{key}={value}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
