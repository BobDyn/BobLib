"""Conservative source dependency graph for Modelica test selection.

This is an over-approximation, not a Modelica compiler. Qualified references
resolve to their containing source file; short names include *all* matching
local classes. Imports, extends, constrainedby, redeclarations, functions and
conditional branches therefore retain every candidate. Package references
include their descendants. Unknown syntax or missing source history requests
the full gate instead of claiming that a test is unaffected.
"""
from __future__ import annotations

import re
import subprocess
from collections import defaultdict, deque
from pathlib import Path

ROOTS = ("BobLib/", "Tests/BobLibTest/")
TOKEN = re.compile(r"[A-Za-z_]\w*(?:\s*\.\s*[A-Za-z_]\w*)*")
MASK = re.compile(r'//[^\n]*|/\*.*?\*/|"(?:\\.|[^"\\])*"', re.S)


class UncertainGraph(ValueError):
    """A source cannot be represented safely by this graph."""


def is_source(path: str) -> bool:
    return path.startswith(ROOTS) and path.endswith(".mo")


def class_name(path: str) -> str:
    name = path.removeprefix("Tests/").removesuffix(".mo").replace("/", ".")
    return name.removesuffix(".package")


def working_sources(root: Path) -> dict[str, str]:
    return {
        path.relative_to(root).as_posix(): path.read_text(encoding="utf-8")
        for directory in ROOTS
        for path in sorted((root / directory).rglob("*.mo"))
    }


def revision_sources(root: Path, revision: str) -> dict[str, str]:
    """Read a revision without checking it out or touching the user's files."""
    result = subprocess.run(
        ["git", "ls-tree", "-r", "--name-only", revision, "--", *ROOTS],
        cwd=root, check=True, capture_output=True, text=True,
    )
    paths = [path for path in result.stdout.splitlines() if is_source(path)]
    if not paths:
        raise UncertainGraph(f"No Modelica sources at {revision}")
    # One git process, including an exact byte count for each source blob.
    result = subprocess.run(
        ["git", "cat-file", "--batch"], cwd=root,
        input="".join(f"{revision}:{path}\n" for path in paths).encode(),
        check=True, capture_output=True,
    )
    data, position, sources = result.stdout, 0, {}
    for path in paths:
        end = data.index(b"\n", position)
        header = data[position:end].split()
        if len(header) != 3 or header[1] != b"blob":
            raise UncertainGraph(f"Cannot read {revision}:{path}")
        size = int(header[2])
        position = end + 1
        sources[path] = data[position:position + size].decode("utf-8")
        position += size + 1
    return sources


def dependency_graph(sources: dict[str, str]) -> dict[str, set[str]]:
    """Map source paths to possible local dependencies, without translation.

    Keep source-file granularity for nested declarations. Any use of a nested
    class reaches its containing file. References to external libraries need no
    edge: library/toolchain inputs trigger the full gate in ci_changes.py.
    """
    if not sources:
        raise UncertainGraph("Empty Modelica source graph")
    names = {class_name(path): path for path in sources}
    short: dict[str, set[str]] = defaultdict(set)
    for name, path in names.items():
        short[name.rsplit(".", 1)[-1]].add(path)

    def expand(path: str) -> set[str]:
        if path.endswith("/package.mo"):
            prefix = path.removesuffix("package.mo")
            return {p for p in sources if p.startswith(prefix)}
        return {path}

    graph: dict[str, set[str]] = {}
    for path, text in sources.items():
        code = MASK.sub(" ", text)
        # Quoted identifiers and unclosed comments/strings are deliberately not
        # parsed. A parser extension must come with dependency-selection tests.
        if "'" in code or '"' in code or "/*" in code:
            raise UncertainGraph(f"Unsupported or incomplete Modelica syntax in {path}")
        code = re.sub(r"\bwithin\s*[^;]*;", "", code)
        dependencies = set()
        for match in TOKEN.finditer(code):
            token = re.sub(r"\s+", "", match[0])
            parts = token.split(".")
            if parts[0] in {"Modelica", "VehicleInterfaces", "OpenModelica"}:
                continue
            if parts[0] in {"BobLib", "BobLibTest"}:
                owner = token
                while owner not in names and "." in owner:
                    owner = owner.rsplit(".", 1)[0]
                # Resolving only the library root would hide a misspelled or
                # deleted reference. Let the full checks diagnose it.
                if owner not in names or (owner in {"BobLib", "BobLibTest"} and len(parts) > 1):
                    raise UncertainGraph(f"Unresolved local reference {token} in {path}")
                dependencies.update(expand(names[owner]))
            else:
                # Match every segment as well as the leading name: a relative
                # nested reference may begin with a component or inherited alias.
                for part in parts:
                    for candidate in short.get(part, ()):
                        dependencies.update(expand(candidate))
        dependencies.discard(path)
        graph[path] = dependencies
    return graph


def affected_models(
    paths: list[str], current: dict[str, str], previous: dict[str, str],
) -> tuple[list[str], dict[str, list[str]]]:
    """Walk the union of base/head reverse edges, retaining an explanation path."""
    reverse: dict[str, set[str]] = defaultdict(set)
    for sources in (previous, current):
        for consumer, dependencies in dependency_graph(sources).items():
            for dependency in dependencies:
                reverse[dependency].add(consumer)
    seeds = set()
    for path in paths:
        if path.endswith(("/package.mo", "/package.order")):
            prefix = path.rsplit("/", 1)[0] + "/"
            seeds.update(p for p in current.keys() | previous.keys() if p.startswith(prefix))
        elif is_source(path):
            if path not in current and path not in previous:
                raise UncertainGraph(f"Changed source missing from both revisions: {path}")
            seeds.add(path)
    reasons = {path: [path] for path in sorted(seeds)}
    queue = deque(reasons)
    while queue:
        dependency = queue.popleft()
        for consumer in sorted(reverse[dependency]):
            if consumer not in reasons:
                reasons[consumer] = [*reasons[dependency], consumer]
                queue.append(consumer)
    affected = sorted(class_name(path) for path in reasons if path in current)
    return affected, {class_name(p): chain for p, chain in reasons.items() if p in current}
