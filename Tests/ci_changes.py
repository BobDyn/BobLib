"""Select CI jobs and write a conservative Modelica impact plan.

PRs use the union of the merge-base and checkout dependency graphs. Pushes,
tags, manual runs, shared tooling changes and uncertain graphs run the full
gate. No Modelica translation or compilation is needed to build the graph.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
from fnmatch import fnmatch
from pathlib import Path

from modelica_impact import (
    ROOTS,
    UncertainGraph,
    affected_models,
    class_name,
    is_source,
    revision_sources,
    working_sources,
)

IGNORED = (
    "*.md", "LICENSE", ".gitignore", ".github/ISSUE_TEMPLATE/*",
    ".github/dependabot.yml", "development/visual-record-drafts/*",
)
FULL_SHARDS = 4


def _is_test_module(path: str) -> bool:
    return fnmatch(path, "Tests/test_*.py") and "/" not in path[len("Tests/"):]


def source_digest(sources: dict[str, str]) -> str:
    digest = hashlib.sha256()
    for path, text in sorted(sources.items()):
        digest.update(path.encode() + b"\0" + text.encode() + b"\0")
    return digest.hexdigest()


def make_plan(paths: list[str], root: Path | None = None, base: str | None = None) -> dict:
    """Describe selection and its reasons; missing history always widens it."""
    paths = sorted({p.strip() for p in paths if p.strip()})
    relevant = [p for p in paths if not any(fnmatch(p, g) for g in IGNORED)]
    plan = {
        "schema": 1, "mode": "full", "reason": "Shared or unknown inputs changed",
        "changed_paths": paths, "changed_tests": [p for p in relevant if _is_test_module(p)],
        "affected_models": [], "reasons": {},
    }
    if not paths:
        plan["reason"] = "Empty change list"
        return plan
    model_paths = [p for p in relevant if is_source(p) or (
        p.startswith(ROOTS) and p.endswith("/package.order")
    )]
    if set(relevant) - set(model_paths) - set(plan["changed_tests"]):
        return plan
    if not model_paths:
        plan.update(mode="tests" if plan["changed_tests"] else "none", reason="No Modelica sources changed")
        return plan
    if root is None or base is None:
        plan["reason"] = "Modelica selection requires the base revision and checkout"
        return plan
    try:
        current = working_sources(root)
        previous = revision_sources(root, base)
        models, reasons = affected_models(model_paths, current, previous)
        plan.update(mode="impact", reason="Reverse dependencies in base and checkout",
                    affected_models=models, reasons=reasons, base=base,
                    source_digest=source_digest(current),
                    known_models=sorted(class_name(path) for path in current))
    except (UncertainGraph, OSError, UnicodeError, ValueError, subprocess.CalledProcessError) as exc:
        plan["reason"] = f"Full fallback: {exc}"
    return plan


def job_outputs(plan: dict, root: Path | None = None) -> dict[str, str]:
    tests = plan["changed_tests"]
    if root is not None:
        tests = [t for t in tests if (root / t).is_file()]
    mode = plan["mode"]
    full = mode == "full"
    impact = mode == "impact"
    components = full or impact or bool(tests)
    shards = FULL_SHARDS if full or impact else 1
    return {
        "full": str(full).lower(),
        "smoke": str(full or impact).lower(),
        "fmu": str(full or impact or any("fmu" in Path(t).name for t in tests)).lower(),
        "components": str(components).lower(),
        "shards": json.dumps(list(range(1, shards + 1)) if components else []),
        "shard_count": str(shards),
        # Every changed test module runs, including suites excluded from the
        # normal component job. This is essential for test-only PRs.
        "component_tests": " ".join(tests) if mode == "tests" else "",
        "extra_tests": " ".join(tests) if impact or full else "",
    }


def classify(paths: list[str], root: Path | None = None) -> dict[str, str]:
    """Compatibility entry point for path-only callers (no selective models)."""
    return job_outputs(make_plan(paths, root), root)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--full", action="store_true")
    parser.add_argument("--root", type=Path, default=Path.cwd())
    parser.add_argument("--base", help="PR merge-base commit, required for Modelica selection")
    parser.add_argument("--plan", type=Path, help="Write the plan for pytest and CI artifacts")
    args = parser.parse_args(argv)
    plan = make_plan(["makefile"] if args.full else sys.stdin.read().splitlines(), args.root, args.base)
    if args.plan:
        args.plan.write_text(json.dumps(plan, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(f"Selection: {plan['mode']}: {plan['reason']}; {len(plan['affected_models'])} affected classes",
          file=sys.stderr)
    for key, value in job_outputs(plan, args.root).items():
        print(f"{key}={value}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
