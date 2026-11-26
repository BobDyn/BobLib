"""Split the test run into CI shards of about equal wall clock.

The component job runs four shards side by side, so a PR waits for the slowest
one. Dealing files round-robin by name put the four heaviest OpenModelica files
together and left the shards at 7, 28, 40 and 7 minutes. This deals the
heaviest units first, each to the shard with the least work so far, using the
durations recorded in `shard_durations.json`.

A unit is a whole test file, so its module fixtures compile once, or a single
test for files that set `BOBLIB_SHARD_BY_TEST = True`. A unit with no recorded
duration counts as the median of its kind, so a new test lands somewhere
sensible and the next refresh records it.

Every shard collects the same items and computes the same plan, so the shards
never overlap or miss a test. Refresh the durations from the component jobs'
JUnit artifacts (`boblib-component-contracts-*`) after a big change:

    python3 Tests/shard_plan.py path/to/boblib-components-*.xml
"""

from __future__ import annotations

import argparse
import json
import statistics
import sys
import xml.etree.ElementTree as ET
from collections.abc import Iterable, Sequence
from pathlib import Path

TESTS_DIR = Path(__file__).resolve().parent
ROOT = TESTS_DIR.parent
DURATIONS = TESTS_DIR / "shard_durations.json"
PER_TEST_FLAG = "BOBLIB_SHARD_BY_TEST = True"


def load_durations(path: Path = DURATIONS) -> dict[str, float]:
    try:
        data = json.loads(path.read_text())
    except FileNotFoundError:
        return {}
    return {str(unit): float(seconds) for unit, seconds in data.items()}


def is_test_unit(unit: str) -> bool:
    return "::" in unit


def weights(units: Iterable[str], durations: dict[str, float]) -> dict[str, float]:
    """Recorded seconds per unit; unrecorded units get the median of their kind."""
    defaults = {}
    for kind in (True, False):
        known = [s for u, s in durations.items() if is_test_unit(u) is kind]
        defaults[kind] = statistics.median(known) if known else 1.0
    return {u: durations.get(u, defaults[is_test_unit(u)]) for u in units}


def plan(units: Iterable[str], count: int, durations: dict[str, float]) -> list[set[str]]:
    """Longest-first greedy split of ``units`` into ``count`` shards.

    Ties break on name and then on shard index, so every shard gets the same
    answer from the same collection. Equal loads go to the shard with fewer
    units, so units recorded at 0.0 s spread out instead of piling on one shard.
    """
    load = [0.0] * count
    shards: list[set[str]] = [set() for _ in range(count)]
    weighted = weights(set(units), durations)
    for unit, seconds in sorted(weighted.items(), key=lambda kv: (-kv[1], kv[0])):
        target = min(range(count), key=lambda i: (load[i], len(shards[i]), i))
        shards[target].add(unit)
        load[target] += seconds
    return shards


def _per_test_files(root: Path) -> set[str]:
    return {
        path.relative_to(root).as_posix()
        for path in (root / "Tests").glob("test_*.py")
        if PER_TEST_FLAG in path.read_text(encoding="utf-8")
    }


def _test_file(classname: str, root: Path) -> tuple[str, list[str]] | None:
    """Split a JUnit classname like ``Tests.test_x.TestY`` into file and class parts."""
    parts = classname.split(".")
    for end in range(len(parts), 0, -1):
        candidate = "/".join(parts[:end]) + ".py"
        if (root / candidate).is_file():
            return candidate, parts[end:]
    return None


def measured(reports: Sequence[Path], root: Path = ROOT) -> dict[str, float]:
    """Seconds per unit from pytest JUnit XML reports."""
    per_test = _per_test_files(root)
    seconds: dict[str, float] = {}
    for report in reports:
        for case in ET.parse(report).iter("testcase"):
            found = _test_file(case.get("classname", ""), root)
            if found is None:
                continue
            path, classes = found
            unit = "::".join([path, *classes, case.get("name", "")]) if path in per_test else path
            seconds[unit] = seconds.get(unit, 0.0) + float(case.get("time") or 0.0)
    return seconds


def refresh(reports: Sequence[Path], path: Path = DURATIONS, root: Path = ROOT) -> dict[str, float]:
    """Merge new measurements over the recorded ones and drop units whose file is gone."""
    merged = {**load_durations(path), **{u: round(s, 1) for u, s in measured(reports, root).items()}}
    merged = {u: s for u, s in merged.items() if (root / u.split("::")[0]).is_file()}
    path.write_text(json.dumps(dict(sorted(merged.items())), indent=2) + "\n")
    return merged


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("reports", nargs="+", type=Path, help="JUnit XML from the component shards.")
    parser.add_argument("--shards", type=int, default=4, help="Print the resulting split for this many shards.")
    args = parser.parse_args(argv)
    durations = refresh(args.reports)
    for index, shard in enumerate(plan(durations, args.shards, durations), start=1):
        minutes = sum(durations[u] for u in shard) / 60
        print(f"shard {index}/{args.shards}: {minutes:5.1f} min, {len(shard)} units")
    return 0


if __name__ == "__main__":
    sys.exit(main())
